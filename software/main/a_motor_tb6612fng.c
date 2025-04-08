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
#include <a_gpio.h>
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
#define A_TB6612FNG_LOCK(negEspErr)    negEspErr = -ESP_ERR_INVALID_STATE;       \
                                       if (gMutex != NULL) {                     \
                                           negEspErr = ESP_OK;                   \
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
static int32_t gMotorOpenLastErrorCode = ESP_OK;

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS
 * -------------------------------------------------------------- */

// Set the direction pins.
static int32_t setDirectionPinsClockwise(bool clockwiseNotAnticlockwise,
                                         gpio_num_t pinMotorControl1,
                                         gpio_num_t pinMotorControl2)
{
    int32_t negEspErr;

    if (clockwiseNotAnticlockwise) {
        // From the TB6612FNG data-sheet, control input 1 high,
        // control input 2 low, means clockwise
        negEspErr = -gpio_set_level(pinMotorControl1, 1);
        if (negEspErr == ESP_OK) {
            negEspErr = -gpio_set_level(pinMotorControl2, 0);
        }
    } else {
        // Conversely, control input 1 low, control input 2 high
        // means anti-clockwise
        negEspErr = -gpio_set_level(pinMotorControl1, 0);
        if (negEspErr == ESP_OK) {
            negEspErr = -gpio_set_level(pinMotorControl2, 1);
        }
    }

    return negEspErr;
}

// Set the direction of motor's rotation to clockwise or anticlockwise.
static int32_t setDirectionClockwise(aMotor_t *pMotor,
                                     bool clockwiseNotAnticlockwise)
{
    int32_t negEspErr;

    A_TB6612FNG_LOCK(negEspErr);

    negEspErr = -ESP_ERR_INVALID_ARG;
    if (pMotor != NULL) {
        negEspErr = setDirectionPinsClockwise(clockwiseNotAnticlockwise,
                                              pMotor->pinMotorControl1,
                                              pMotor->pinMotorControl2);
        if (negEspErr == ESP_OK) {
            pMotor->directionIsAnticlockwise = !clockwiseNotAnticlockwise;
            printf(A_LOG_TAG "motor \"%s\" set to %sclockwise.\n",
                   pMotor->pNameStr,
                   pMotor->directionIsAnticlockwise ? "anti-": "");
        }
    }

    A_TB6612FNG_UNLOCK();

    return negEspErr;
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
            aGpioOutputSet(gPinEnable, 0);
        }
        printf(A_LOG_TAG "motor \"%s\" closed.\n", pNameStr);
    }
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * -------------------------------------------------------------- */

// Initialise this API.
int32_t aMotorInit(gpio_num_t pinEnable)
{
    int32_t negEspErr = ESP_OK;

    if (gMutex == NULL) {
        // Set the enable pin to low
        negEspErr = aGpioOutputSet(pinEnable, 0);
        if (negEspErr == ESP_OK) {
            // Create a mutex to arbitrate activity
            negEspErr = -ESP_ERR_NO_MEM;
            gMutex = xSemaphoreCreateMutex();
            if (gMutex != NULL) {
                // Store the enable pin for use later
                gPinEnable = pinEnable;
                negEspErr = ESP_OK;
            }
        }
    }

    if (negEspErr == ESP_OK) {
        printf(A_LOG_TAG "motor driver initialised.\n");
    } else {
        // Clean up on error
        if (gMutex != NULL) {
            vSemaphoreDelete(gMutex);
            gMutex = NULL;
        }
        gPinEnable = -1;
        printf(A_LOG_TAG "unable to initialise motor driver"
               " (0x%02x)!\n", (int) negEspErr);
    }

    return negEspErr;
}

// Open a motor.
aMotor_t *pAMotorOpen(gpio_num_t pinPwm,
                      gpio_num_t pinMotorControl1,
                      gpio_num_t pinMotorControl2,
                      const char *pNameStr)
{
    int32_t negEspErr;
    aPwm_t *pPwm = NULL;
    aMotor_t *pMotor = NULL;

    A_TB6612FNG_LOCK(negEspErr);

    // Set the pins as outputs and low (for off)
    negEspErr = aGpioOutputSet(pinMotorControl1, 0);
    if (negEspErr == ESP_OK) {
        negEspErr = aGpioOutputSet(pinMotorControl2, 0);
    }
    if (negEspErr == ESP_OK) {
        negEspErr = -ESP_ERR_NO_MEM;
        // Allocate memory for the motor
        pMotor = (aMotor_t *) malloc(sizeof(*pMotor));
        if (pMotor != NULL) {
            memset(pMotor, 0, sizeof(*pMotor));
            // Open a PWM
            pPwm = pAPwmOpen(pinPwm, pNameStr);
            if (pPwm != NULL) {
                // Set the direction or it won't go
                negEspErr = setDirectionPinsClockwise(true,
                                                      pinMotorControl1,
                                                      pinMotorControl2);
            } else {
                negEspErr = aPwmOpenLastErrorGetReset();
            }
            if (negEspErr == ESP_OK) {
                // Populate the rest and add the motor
                // to the linked list
                negEspErr = -ESP_ERR_NO_MEM;
                pMotor->pPwm = pPwm;
                pMotor->pinMotorControl1 = pinMotorControl1;
                pMotor->pinMotorControl2 = pinMotorControl2;
                pMotor->pNameStr = pNameStr;
                if (pMotor->pNameStr == NULL) {
                    pMotor->pNameStr = "no name";
                }
                // Add the motor to the linked list
                if (aUtilLinkedListAdd(&gpMotorList, pMotor)) {
                    negEspErr = ESP_OK;
                }
            }
        }
    }

    A_TB6612FNG_UNLOCK();

    if (negEspErr < 0) {
        // Clean up on error
        aPwmClose(pPwm);
        free(pMotor);
        gMotorOpenLastErrorCode = negEspErr;
        printf(A_LOG_TAG "unable to open motor \"%s\","
               " PWM pin %d, control pins %d and %d (0x%02x)!\n",
               pNameStr, pinPwm, pinMotorControl1,
               pinMotorControl2, (int) negEspErr);
    } else {
        if (gPinEnable >= 0) {
            // Make sure the enable pin is set high; don't check
            // for errors here as this would have barfed during
            // aMotorInit() if it was going to barf
            aGpioOutputSet(gPinEnable, 1);
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
int32_t aMotorOpenLastErrorGetReset()
{
    int32_t negEspErr = gMotorOpenLastErrorCode;

    gMotorOpenLastErrorCode = ESP_OK;

    return negEspErr;
}

// Close a motor.
void aMotorClose(aMotor_t *pMotor)
{
    int32_t negEspErr;

    A_TB6612FNG_LOCK(negEspErr);

    motorClose(pMotor);

    A_TB6612FNG_UNLOCK();

    // To prevent compiler warnings as we aren't returning negEspErr
    (void) negEspErr;
}

// Set the direction of motor's rotation to clockwise.
int32_t aMotorDirectionClockwiseSet(aMotor_t *pMotor)
{
    return setDirectionClockwise(pMotor, true);
}

// Set the direction of motor's rotation to anti-clockwise.
int32_t aMotorDirectionAnticlockwiseSet(aMotor_t *pMotor)
{
    return setDirectionClockwise(pMotor, false);
}

// Get the direction of a motor's rotation.
int32_t aMotorDirectionIsClockwise(aMotor_t *pMotor)
{
    int32_t directionOrNegEspErr;

    A_TB6612FNG_LOCK(directionOrNegEspErr);

    directionOrNegEspErr = -ESP_ERR_INVALID_ARG;
    if (pMotor != NULL) {
        directionOrNegEspErr = (int32_t) !pMotor->directionIsAnticlockwise;
    }

    A_TB6612FNG_UNLOCK();

    return directionOrNegEspErr;
}

// Set the speed of a motor relative to its current speed.
int32_t aMotorSpeedRelativeSet(aMotor_t *pMotor, int32_t percent)
{
    int32_t speedPercentOrNegEspErr;

    A_TB6612FNG_LOCK(speedPercentOrNegEspErr);

    speedPercentOrNegEspErr = -ESP_ERR_INVALID_ARG;
    if (pMotor != NULL) {
        speedPercentOrNegEspErr = aPwmRateRelativeSet(pMotor->pPwm, percent);
    }

    A_TB6612FNG_UNLOCK();

    return speedPercentOrNegEspErr;
}

// Set the speed of a motor to an absolute value.
int32_t aMotorSpeedAbsoluteSet(aMotor_t *pMotor, size_t percent)
{
    int32_t speedPercentOrNegEspErr;

    A_TB6612FNG_LOCK(speedPercentOrNegEspErr);

    speedPercentOrNegEspErr = -ESP_ERR_INVALID_ARG;
    if (pMotor != NULL) {
        speedPercentOrNegEspErr = aPwmRateAbsoluteSet(pMotor->pPwm, percent);
    }

    A_TB6612FNG_UNLOCK();

    return speedPercentOrNegEspErr;
}

// Get the current speed of a motor.
int32_t aMotorSpeedGet(aMotor_t *pMotor)
{
    int32_t speedPercentOrNegEspErr;

    A_TB6612FNG_LOCK(speedPercentOrNegEspErr);

    speedPercentOrNegEspErr = -ESP_ERR_INVALID_ARG;
    if (pMotor != NULL) {
        speedPercentOrNegEspErr = (int32_t) aPwmRateGet(pMotor->pPwm);
    }

    A_TB6612FNG_UNLOCK();

    return speedPercentOrNegEspErr;
}

// Set the transition time for a speed change.
int32_t aMotorSpeedTransitionTimeSet(aMotor_t *pMotor, size_t timeMs)
{
    int32_t timeMsOrNegEspErr;

    A_TB6612FNG_LOCK(timeMsOrNegEspErr);

    timeMsOrNegEspErr = -ESP_ERR_INVALID_ARG;
    if (pMotor != NULL) {
        timeMsOrNegEspErr = aPwmRateTransitionTimeSet(pMotor->pPwm, timeMs);
    }

    A_TB6612FNG_UNLOCK();

    return timeMsOrNegEspErr;
}

// Get the transition time for a speed change.
int32_t aMotorSpeedTransitionTimeGet(aMotor_t *pMotor)
{
    int32_t timeMsOrNegEspErr;

    A_TB6612FNG_LOCK(timeMsOrNegEspErr);

    timeMsOrNegEspErr = -ESP_ERR_INVALID_ARG;
    if (pMotor != NULL) {
        timeMsOrNegEspErr = aPwmRateTransitionTimeGet(pMotor->pPwm);
    }

    A_TB6612FNG_UNLOCK();

    return timeMsOrNegEspErr;
}

// Deinitialise motors and free resources.
void aMotorDeinit()
{
    esp_err_t negEspErr;
    aMotor_t *pMotor;

    A_TB6612FNG_LOCK(negEspErr);

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

    // To prevent compiler warnings as we aren't returning negEspErr
    (void) negEspErr;
}

// End of file
