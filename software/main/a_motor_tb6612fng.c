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

// The frequency of PWM output to use, in Hertz.
#ifndef A_MOTOR_PWM_FREQUENCY_HERTZ
# define A_MOTOR_PWM_FREQUENCY_HERTZ 100
#endif

// The resolution of PWM to use, in bits.
#ifndef A_MOTOR_PWM_RESOLUTION_BITS
# define A_MOTOR_PWM_RESOLUTION_BITS (LEDC_TIMER_8_BIT)
#endif


// The speed mode of the PWM.
#ifndef A_MOTOR_PWM_SPEED_MODE
# define A_MOTOR_PWM_SPEED_MODE (LEDC_LOW_SPEED_MODE)
#endif

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

// Array to track PWM channel occupancy.
static aMotor_t *gpPwmChannelList[LEDC_CHANNEL_MAX] = {0};

// Array of semaphores to keep track of whether a speed transition
// is in progress.  MUST have the same number of members as
// gpPwmChannelList as the same index is used for both.
static SemaphoreHandle_t gSemaphoreSpeedTransition[LEDC_CHANNEL_MAX] = {0};

// Array to keep track of PWM timers.
static aMotor_t *gpPwmTimerList[LEDC_TIMER_MAX] = {0};

// NOTE: there are more variables after the static functions.

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS
 * -------------------------------------------------------------- */

// "end of fade operation" callback function.
// pUserArg should point to the relevant entry in gSemaphoreSpeedTransition,
static IRAM_ATTR bool pwmTransitionEnd(const ledc_cb_param_t *pParam,
                                       void *pUserArg)
{
    BaseType_t mustYield = false;

    if ((pParam->event == LEDC_FADE_END_EVT) && (pUserArg != NULL)) {
        // Give the semaphore
        xSemaphoreGiveFromISR(*(SemaphoreHandle_t *) pUserArg, &mustYield);
    }

    return mustYield;
}

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

// Convert a percentage speed to a duty cycle.
static size_t percentToDuty(int32_t percent)
{
    return ((1 << A_MOTOR_PWM_RESOLUTION_BITS) - 1) * percent / 100;
}

// Find the given motor in an array of aMotor_t pointers.
static aMotor_t **ppFindEntry(aMotor_t *pMotor, aMotor_t **ppList,
                              size_t listSize)
{
    aMotor_t **ppEntry = NULL;

    if (ppList != NULL) {
        for (size_t x = 0; x < listSize; x++) {
            if (*ppList == pMotor) {
                ppEntry = ppList;
                break;
            }
            ppList++;
        }
    }

    return ppEntry;
}

// Set the speed of a motor via PWM, limiting it to a valid range.
// Note: A_TB6612FNG_LOCK()/A_TB6612FNG_LOCK() should be called
// before this is called.
static int32_t setAndLimitSpeed(aMotor_t *pMotor, int32_t percent)
{
    esp_err_t espErr = ESP_ERR_INVALID_ARG;
    int32_t speedPercentOrEspErr = percent;
    aMotor_t **ppChannel = NULL;
        size_t targetDuty = 0;
    ledc_channel_t pwmChannel;

    if (speedPercentOrEspErr < 0) {
        speedPercentOrEspErr = 0;
    } else if (speedPercentOrEspErr > 100) {
        speedPercentOrEspErr = 100;
    }

    // Find the channel used by this motor
    ppChannel = ppFindEntry(pMotor, gpPwmChannelList,
                            A_UTIL_ARRAY_COUNT(gpPwmChannelList));
    if (ppChannel != NULL) {
        pwmChannel = ppChannel - gpPwmChannelList;
        // Need to wait on the transition semaphore as we can't
        // change speed if a transition is already in progress;
        // the semaphore is given by the callback at the end
        // of a fade
        xSemaphoreTake(gSemaphoreSpeedTransition[pwmChannel],
                       (TickType_t) portMAX_DELAY);
        // Fade to the target duty cycle
        targetDuty = percentToDuty(speedPercentOrEspErr);
        espErr = ledc_set_fade_time_and_start(A_MOTOR_PWM_SPEED_MODE,
                                              pwmChannel,
                                              targetDuty,
                                              pMotor->speedTransitionTimeMs,
                                              LEDC_FADE_NO_WAIT);
    }

    if (espErr != ESP_OK) {
        printf(A_LOG_TAG "unable to set motor \"%s\" speed to %d%%"
              " (duty cycle %d), transition time %d ms (0x%02x)!.\n",
              pMotor->pNameStr, (int) speedPercentOrEspErr, targetDuty,
              pMotor->speedTransitionTimeMs,
              (int) espErr);
        speedPercentOrEspErr = (int32_t) espErr;
    } else {
        pMotor->speedPercent = (size_t) speedPercentOrEspErr;
        printf(A_LOG_TAG "motor \"%s\" speed set to %d%%.\n", pMotor->pNameStr,
               pMotor->speedPercent);
    }

    return  speedPercentOrEspErr;
}

// Close a motor, freeing memory, removing it from the linked list.
// Note: A_TB6612FNG_LOCK()/A_TB6612FNG_LOCK() should be called
// before this is called.
static void motorClose(aMotor_t *pMotor)
{
    const char *pNameStr;
    aMotor_t **ppTimerOrChannel;
    size_t pwmChannel;
    ledc_timer_config_t pwmTimerConfig = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                          .timer_num = -1, // timer_num is set below
                                          .clk_cfg = LEDC_AUTO_CLK,
                                          .deconfigure = true};

    if (pMotor != NULL) {
        pNameStr = pMotor->pNameStr;
        // Stop the motor
        setAndLimitSpeed(pMotor, 0);
        // Set the pins low (for off)
        gpio_set_level(pMotor->pinMotorControl1, 0);
        gpio_set_level(pMotor->pinMotorControl2, 0);
        // Free PWM channel and transition semaphore
        ppTimerOrChannel = ppFindEntry(pMotor, gpPwmChannelList,
                                       A_UTIL_ARRAY_COUNT(gpPwmChannelList));
        if (ppTimerOrChannel != NULL) {
            pwmChannel = ppTimerOrChannel - gpPwmChannelList;
            if (gSemaphoreSpeedTransition[pwmChannel] != NULL) {
                // Wait for any speed transition to complete
                xSemaphoreTake(gSemaphoreSpeedTransition[pwmChannel],
                               (TickType_t) portMAX_DELAY);
                // Give the semaphore again so that we can delete it
                xSemaphoreGive(gSemaphoreSpeedTransition[pwmChannel]);
                vSemaphoreDelete(gSemaphoreSpeedTransition[pwmChannel]);
                gSemaphoreSpeedTransition[pwmChannel] = NULL;
            }
            *ppTimerOrChannel = NULL;
        }
        // Deconfigure and free PWM timer
        ppTimerOrChannel = ppFindEntry(pMotor, gpPwmTimerList,
                                       A_UTIL_ARRAY_COUNT(gpPwmTimerList));
        if (ppTimerOrChannel != NULL) {
            pwmTimerConfig.timer_num = ppTimerOrChannel - gpPwmTimerList;
            ledc_timer_pause(pwmTimerConfig.speed_mode,
                             pwmTimerConfig.timer_num);
            ledc_timer_config(&pwmTimerConfig);
            *ppTimerOrChannel = NULL;
        }
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
 * MORE VARIABLES
 * -------------------------------------------------------------- */

// Callback to track the end of PWM transitions.
static ledc_cbs_t gPwmCallbacks = {
        .fade_cb = pwmTransitionEnd
    };

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
                // Install PWM
                espErr = ledc_fade_func_install(0);
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
                      gpio_num_t pinMotorControl1, gpio_num_t pinMotorControl2,
                      const char *pNameStr)
{
    esp_err_t espErr;
    aMotor_t *pMotor = NULL;
    ledc_timer_config_t pwmTimerConfig = {.duty_resolution = A_MOTOR_PWM_RESOLUTION_BITS,
                                          .freq_hz = A_MOTOR_PWM_FREQUENCY_HERTZ,
                                          .speed_mode = A_MOTOR_PWM_SPEED_MODE,
                                          .timer_num = -1, // timer_num is set below
                                          .clk_cfg = LEDC_AUTO_CLK,
                                          .deconfigure = false};
    ledc_channel_config_t pwmChannelConfig = {.channel = -1, // Channel is set below
                                              .duty = 0,
                                              .gpio_num = pinPwm,
                                              .speed_mode = A_MOTOR_PWM_SPEED_MODE,
                                              .hpoint = 0,
                                              .timer_sel = -1, // timer_sel is set below
                                              .flags.output_invert = 0};
    aMotor_t **ppTimer = NULL;
    aMotor_t **ppChannel = NULL;

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
            // Set up PWM: find a spare timer
            ppTimer = ppFindEntry(NULL, gpPwmTimerList,
                                  A_UTIL_ARRAY_COUNT(gpPwmTimerList));
            if (ppTimer != NULL) {
                // Found one: allocate it to this motor and configure the PWM timer
                *ppTimer = pMotor;
                pwmTimerConfig.timer_num = ppTimer - gpPwmTimerList;
                espErr = ledc_timer_config(&pwmTimerConfig);
            }
            if (espErr == ESP_OK) {
                // Set up PWM: find a spare channel
                espErr = ESP_ERR_NO_MEM;
                ppChannel = ppFindEntry(NULL, gpPwmChannelList,
                                        A_UTIL_ARRAY_COUNT(gpPwmChannelList));
                if (ppChannel != NULL) {
                    // Found one: allocate it to this motor and configure the PWM
                    *ppChannel = pMotor;
                    pwmChannelConfig.channel = ppChannel - gpPwmChannelList;
                    pwmChannelConfig.timer_sel = pwmTimerConfig.timer_num;
                    espErr = ledc_channel_config(&pwmChannelConfig);
                    if (espErr == ESP_OK) {
                        espErr = ESP_ERR_NO_MEM;
                        // Create a semaphore which we will use to track whether
                        // a speed transition is active or not (otherwise we
                        // can't change speed reliably as a fade command is
                        // ignored if one is already in progress)
                        // Just the one value available in a counting semaphore,
                        // with that value available initially.
                        gSemaphoreSpeedTransition[pwmChannelConfig.channel] = xSemaphoreCreateCounting(1, 1);
                        if (gSemaphoreSpeedTransition[pwmChannelConfig.channel] != NULL) {
                            // Register a callback and pass it a pointer to the
                            // mutex so that it can be given by the callback
                            espErr = ledc_cb_register(pwmChannelConfig.speed_mode,
                                                      pwmChannelConfig.channel,
                                                      &gPwmCallbacks,
                                                      (void *) &(gSemaphoreSpeedTransition[pwmChannelConfig.channel]));
                        }
                        if (espErr == ESP_OK) {
                            // And finally, the pins to set direction or it won't go
                            espErr = setDirectionPinsClockwise(true,
                                                               pinMotorControl1,
                                                               pinMotorControl2);
                        }
                    }
                }
            }
            if (espErr == ESP_OK) {
                // Populate the rest and add the motor
                // to the linked list
                espErr = ESP_ERR_NO_MEM;
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
        if (ppTimer != NULL) {
            pwmTimerConfig.deconfigure = true;
            ledc_timer_config(&pwmTimerConfig);
            *ppTimer = NULL;
        }
        if (ppChannel != NULL) {
            *ppChannel = NULL;
        }
        if ((pwmChannelConfig.channel >= 0) &&
            (gSemaphoreSpeedTransition[pwmChannelConfig.channel] != NULL)) {
            vSemaphoreDelete(gSemaphoreSpeedTransition[pwmChannelConfig.channel]);
            gSemaphoreSpeedTransition[pwmChannelConfig.channel] = NULL;
        }
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
int32_t aMotorDirectionGet(aMotor_t *pMotor)
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
        percent += (int32_t) pMotor->speedPercent;
        speedPercentOrEspErr = setAndLimitSpeed(pMotor, percent);
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
        speedPercentOrEspErr = setAndLimitSpeed(pMotor, percent);
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
        speedPercentOrEspErr = (int32_t) pMotor->speedPercent;
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
        pMotor->speedTransitionTimeMs = timeMs;
        timeMsOrEspErr = (int32_t) pMotor->speedTransitionTimeMs;
        printf(A_LOG_TAG "motor \"%s\" transition time set to %d ms.\n",
               pMotor->pNameStr, (int) timeMsOrEspErr);
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
        timeMsOrEspErr = (int32_t) pMotor->speedTransitionTimeMs;
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
        // Uninstall PWM
        ledc_fade_func_uninstall();
        // Destroy the mutex
        vSemaphoreDelete(gMutex);
        gMutex = NULL;
    }

    // To prevent compiler warnings as we aren't returning espErr
    (void) espErr;
}

// End of file
