/*
 * Copyright 2024 Rob Meades
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** @file
 * @brief Implementation of the aMotor API for a Toshiba TB6612FNG
 * dual motor driver chip.
 */

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>

#include <esp_timer.h>

#include <driver/gpio.h>
#include <driver/ledc.h>

#include <a_util.h>
#include <a_pwm.h>
#include <a_motor.h>

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS: MISC
 * -------------------------------------------------------------- */

// Prefix for all logging prints from this file.
#define A_LOG_TAG "A_MOTOR: "

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

// Function entry macro; used where thread-safety is required.
#define A_TB6612FNG_LOCK(espErr)    espErr = (int32_t) ESP_ERR_INVALID_STATE;               \
                                    if (gMutex != NULL) {                                   \
                                        espErr = (int32_t) ESP_OK;                      \
                                        xSemaphoreTake(gMutex, (TickType_t) portMAX_DELAY);

// Function exit macro; must be used after A_TB6612FNG_LOCK().
#define A_TB6612FNG_UNLOCK()            xSemaphoreGive(gMutex);   \
                                    };

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

// Mutex to arbitrate activity.
static SemaphoreHandle_t gMutex = NULL;

// Root of the linked-list of motors.
static aUtilLinkedList_t *gpMotorList = NULL; // A list of aMotor_t entries.

// Enable pin for the motors.
static gpio_num_t gPinEnable = -1;

// A place to store the last error code from a call to aMotorOpen().
static esp_err_t gMotorOpenLastErrorCode = ESP_OK;

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS
 * -------------------------------------------------------------- */

// Set the direction pins.
static esp_err_t setDirectionPinsClockwise(bool clockwiseNotAnticlockwise,
                                           gpio_num_t pinMotorControl1,
                                           gpio_num_t pinMotorControl2)
{
    esp_err_t espErr;

    if (clockwiseNotAnticlockwise) {
        // From the TB6612FNG data-sheet, control input 1 high,
        // control input 2 low, means clockwise
        espErr = gpio_set_level(pinMotorControl1, 1);
        if (espErr == ESP_OK) {
            espErr = gpio_set_level(pinMotorControl2, 0);
        }
    } else {
        // Conversely, control input 1 low, control input 2 high
        // means anti-clockwise
        espErr = gpio_set_level(pinMotorControl1, 0);
        if (espErr == ESP_OK) {
            espErr = gpio_set_level(pinMotorControl2, 1);
        }
    }

    return espErr;
}

// Set the direction of motor's rotation to clockwise or anticlockwise.
static esp_err_t setDirectionClockwise(aMotor_t *pMotor,
                                       bool clockwiseNotAnticlockwise)
{
    esp_err_t espErr;

    A_TB6612FNG_LOCK(espErr);

    espErr = ESP_ERR_INVALID_ARG;
    if (pMotor != NULL) {
        espErr = setDirectionPinsClockwise(clockwiseNotAnticlockwise,
                                           pMotor->pinMotorControl1,
                                           pMotor->pinMotorControl2);
        if (espErr == ESP_OK) {
            pMotor->directionIsAnticlockwise = !clockwiseNotAnticlockwise;
            printf(A_LOG_TAG "motor \"%s\" set to %sclockwise.\n",
                   pMotor->pNameStr,
                   pMotor->directionIsAnticlockwise ? "anti-": "");
        }
    }

    A_TB6612FNG_UNLOCK();

    return espErr;
}

// Close a motor, freeing memory, removing it from the linked list.
// Note: A_TB6612FNG_LOCK()/A_TB6612FNG_UNLOCK() should be called
// before this is called.
static void motorClose(aMotor_t *pMotor)
{
    const char *pNameStr;

    if (pMotor != NULL) {
        pNameStr = pMotor->pNameStr;
        if (pMotor->pPwm != NULL) {
            // Close the PWM
            aPwmClose(pMotor->pPwm);
        }
        // Set the pins low (for off)
        gpio_set_level(pMotor->pinMotorControl1, 0);
        gpio_set_level(pMotor->pinMotorControl2, 0);
        // Remove from the linked list and free memory
        aUtilLinkedListRemove(&gpMotorList, pMotor);
        free(pMotor);
        if ((gpMotorList == NULL) && (gPinEnable >= 0)) {
            // If there are no more motors in the list,
            // set the enable pin low as well
            aUtilPinOutputSet(gPinEnable, 0);
        }
        printf(A_LOG_TAG "motor \"%s\" closed.\n", pNameStr);
    }
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * -------------------------------------------------------------- */

// Initialise this API.
esp_err_t aMotorInit(gpio_num_t pinEnable)
{
    esp_err_t espErr = ESP_OK;

    if (gMutex == NULL) {
        // Set the enable pin to low
        espErr = aUtilPinOutputSet(pinEnable, 0);
        if (espErr == ESP_OK) {
            // Create a mutex to arbitrate activity
            espErr = ESP_ERR_NO_MEM;
            gMutex = xSemaphoreCreateMutex();
            if (gMutex != NULL) {
                // Store the enable pin for use later
                gPinEnable = pinEnable;
                espErr = ESP_OK;
            }
        }
    }

    if (espErr == ESP_OK) {
        printf(A_LOG_TAG "motor driver initialised.\n");
    } else {
        // Clean up on error
        if (gMutex != NULL) {
            vSemaphoreDelete(gMutex);
            gMutex = NULL;
        }
        gPinEnable = -1;
        printf(A_LOG_TAG "unable to initialise motor driver"
               " (0x%02x)!\n", espErr);
    }

    return espErr;
}

// Open a motor.
aMotor_t *pAMotorOpen(gpio_num_t pinPwm,
                      gpio_num_t pinMotorControl1,
                      gpio_num_t pinMotorControl2,
                      const char *pNameStr)
{
    esp_err_t espErr;
    aPwm_t *pPwm = NULL;
    aMotor_t *pMotor = NULL;

    A_TB6612FNG_LOCK(espErr);

    // Set the pins as outputs and low (for off)
    espErr = aUtilPinOutputSet(pinMotorControl1, 0);
    if (espErr == ESP_OK) {
        espErr = aUtilPinOutputSet(pinMotorControl2, 0);
    }
    if (espErr == ESP_OK) {
        espErr = ESP_ERR_NO_MEM;
        // Allocate memory for the motor
        pMotor = (aMotor_t *) malloc(sizeof(*pMotor));
        if (pMotor != NULL) {
            memset(pMotor, 0, sizeof(*pMotor));
            // Open a PWM
            pPwm = pAPwmOpen(pinPwm, pNameStr);
            if (pPwm != NULL) {
                // Set the direction or it won't go
                espErr = setDirectionPinsClockwise(true,
                                                   pinMotorControl1,
                                                   pinMotorControl2);
            } else {
                espErr = aPwmOpenLastErrorGetReset();
            }
            if (espErr == ESP_OK) {
                // Populate the rest and add the motor
                // to the linked list
                espErr = ESP_ERR_NO_MEM;
                pMotor->pPwm = pPwm;
                pMotor->pinMotorControl1 = pinMotorControl1;
                pMotor->pinMotorControl2 = pinMotorControl2;
                pMotor->pNameStr = pNameStr;
                if (pMotor->pNameStr == NULL) {
                    pMotor->pNameStr = "no name";
                }
                // Add the motor to the linked list
                if (aUtilLinkedListAdd(&gpMotorList, pMotor)) {
                    espErr = ESP_OK;
                }
            }
        }
    }

    A_TB6612FNG_UNLOCK();

    if (espErr < 0) {
        // Clean up on error
        aPwmClose(pPwm);
        free(pMotor);
        gMotorOpenLastErrorCode = espErr;
        printf(A_LOG_TAG "unable to open motor \"%s\","
               " PWM pin %d, control pins %d and %d (0x%02x)!\n",
               pNameStr, pinPwm, pinMotorControl1,
               pinMotorControl2, espErr);
    } else {
        if (gPinEnable >= 0) {
            // Make sure the enable pin is set high; don't check
            // for errors here as this would have barfed during
            // aMotorInit() if it was going to barf
            aUtilPinOutputSet(gPinEnable, 1);
        }
        printf(A_LOG_TAG "motor \"%s\" opened,"
               " PWM pin %d, control pins %d and %d.\n",
               pMotor->pNameStr, pinPwm,
               pMotor->pinMotorControl1,
               pMotor->pinMotorControl2);
    }

    return pMotor;
}

// Get the last error code from a failed call to pAMotorOpen().
esp_err_t aMotorOpenLastErrorGetReset()
{
    esp_err_t espErr = gMotorOpenLastErrorCode;

    gMotorOpenLastErrorCode = ESP_OK;

    return espErr;
}

// Close a motor.
void aMotorClose(aMotor_t *pMotor)
{
    esp_err_t espErr;

    A_TB6612FNG_LOCK(espErr);

    motorClose(pMotor);

    A_TB6612FNG_UNLOCK();

    // To prevent compiler warnings as we aren't returning espErr
    (void) espErr;
}

// Set the direction of motor's rotation to clockwise.
esp_err_t aMotorDirectionClockwiseSet(aMotor_t *pMotor)
{
    return setDirectionClockwise(pMotor, true);
}

// Set the direction of motor's rotation to anti-clockwise.
esp_err_t aMotorDirectionAnticlockwiseSet(aMotor_t *pMotor)
{
    return setDirectionClockwise(pMotor, false);
}

// Get the direction of a motor's rotation.
int32_t aMotorDirectionIsClockwise(aMotor_t *pMotor)
{
    int32_t directionOrEspErr;

    A_TB6612FNG_LOCK(directionOrEspErr);

    directionOrEspErr = (int32_t) ESP_ERR_INVALID_ARG;
    if (pMotor != NULL) {
        directionOrEspErr = (int32_t) !pMotor->directionIsAnticlockwise;
    }

    A_TB6612FNG_UNLOCK();

    return directionOrEspErr;
}

// Set the speed of a motor relative to its current speed.
int32_t aMotorSpeedRelativeSet(aMotor_t *pMotor, int32_t percent)
{
    int32_t speedPercentOrEspErr;

    A_TB6612FNG_LOCK(speedPercentOrEspErr);

    speedPercentOrEspErr = ESP_ERR_INVALID_ARG;
    if (pMotor != NULL) {
        speedPercentOrEspErr = aPwmRateRelativeSet(pMotor->pPwm, percent);
    }

    A_TB6612FNG_UNLOCK();

    return speedPercentOrEspErr;
}

// Set the speed of a motor to an absolute value.
int32_t aMotorSpeedAbsoluteSet(aMotor_t *pMotor, size_t percent)
{
    int32_t speedPercentOrEspErr;

    A_TB6612FNG_LOCK(speedPercentOrEspErr);

    speedPercentOrEspErr = (int32_t) ESP_ERR_INVALID_ARG;
    if (pMotor != NULL) {
        speedPercentOrEspErr = aPwmRateAbsoluteSet(pMotor->pPwm, percent);
    }

    A_TB6612FNG_UNLOCK();

    return speedPercentOrEspErr;
}

// Get the current speed of a motor.
int32_t aMotorSpeedGet(aMotor_t *pMotor)
{
    int32_t speedPercentOrEspErr;

    A_TB6612FNG_LOCK(speedPercentOrEspErr);

    speedPercentOrEspErr = (int32_t) ESP_ERR_INVALID_ARG;
    if (pMotor != NULL) {
        speedPercentOrEspErr = (int32_t) aPwmRateGet(pMotor->pPwm);
    }

    A_TB6612FNG_UNLOCK();

    return speedPercentOrEspErr;
}

// Set the transition time for a speed change.
int32_t aMotorSpeedTransitionTimeSet(aMotor_t *pMotor, size_t timeMs)
{
    int32_t timeMsOrEspErr;

    A_TB6612FNG_LOCK(timeMsOrEspErr);

    timeMsOrEspErr = (int32_t) ESP_ERR_INVALID_ARG;
    if (pMotor != NULL) {
        timeMsOrEspErr = aPwmRateTransitionTimeSet(pMotor->pPwm, timeMs);
    }

    A_TB6612FNG_UNLOCK();

    return timeMsOrEspErr;
}

// Get the transition time for a speed change.
int32_t aMotorSpeedTransitionTimeGet(aMotor_t *pMotor)
{
    int32_t timeMsOrEspErr;

    A_TB6612FNG_LOCK(timeMsOrEspErr);

    timeMsOrEspErr = (int32_t) ESP_ERR_INVALID_ARG;
    if (pMotor != NULL) {
        timeMsOrEspErr = aPwmRateTransitionTimeGet(pMotor->pPwm);
    }

    A_TB6612FNG_UNLOCK();

    return timeMsOrEspErr;
}

// Deinitialise motors and free resources.
void aMotorDeinit()
{
    esp_err_t espErr;
    aMotor_t *pMotor;

    A_TB6612FNG_LOCK(espErr);

    // Free each motor
    do {
        pMotor = pAUtilLinkedListGetFirst(&gpMotorList, NULL);
        // The following function will remove the motor,
        // which was at the front of the list, so we always
        // only need to do a "Get First" in this loop.
        motorClose(pMotor);
    } while (pMotor != NULL);

    if (gPinEnable >= 0) {
        // Set the enable pin low, if there was one
        gpio_set_level(gPinEnable, 0);
        gPinEnable = -1;
    }

    A_TB6612FNG_UNLOCK();

    if (gMutex != NULL) {
        // Destroy the mutex
        vSemaphoreDelete(gMutex);
        gMutex = NULL;
    }

    // To prevent compiler warnings as we aren't returning espErr
    (void) espErr;
}

// End of file
