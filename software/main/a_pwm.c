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
 * @brief Implementation of the aPwm API for an ESP32 MCU.
 */

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>

#include <esp_timer.h>

#include <driver/gpio.h>
#include <driver/ledc.h>

#include <a_util.h>
#include <a_pwm.h>

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS: MISC
 * -------------------------------------------------------------- */

// Prefix for all logging prints from this file.
#define A_LOG_TAG "A_PWM: "

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

// Function entry macro; used where thread-safety is required.
#define A_PWM_LOCK(negEspErr) negEspErr = -ESP_ERR_INVALID_STATE;          \
                              if (gMutex != NULL) {                        \
                                  negEspErr = ESP_OK;                      \
                                  xSemaphoreTake(gMutex, (TickType_t) portMAX_DELAY);

// Function exit macro; must be used after A_PWM_LOCK().
#define A_PWM_UNLOCK()            xSemaphoreGive(gMutex);   \
                              };

// The frequency of PWM output to use, in Hertz.
#ifndef A_PWM_FREQUENCY_HERTZ
# define A_PWM_FREQUENCY_HERTZ 100
#endif

// The resolution of PWM to use, in bits.
#ifndef A_PWM_RESOLUTION_BITS
# define A_PWM_RESOLUTION_BITS (LEDC_TIMER_8_BIT)
#endif

// The speed mode of the PWM.
#ifndef A_PWM_SPEED_MODE
# define A_PWM_SPEED_MODE (LEDC_LOW_SPEED_MODE)
#endif

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

// Mutex to arbitrate activity.
static SemaphoreHandle_t gMutex = NULL;

// Root of the linked-list of PWMs.
static aUtilLinkedList_t *gpPwmList = NULL; // A list of aPwm_t entries.

// Array to track PWM channel occupancy.
static aPwm_t *gpPwmChannelList[LEDC_CHANNEL_MAX] = {0};

// Array of semaphores to keep track of whether a PWM transition
// is in progress.  MUST have the same number of members as
// gpPwmChannelList as the same index is used for both.
static SemaphoreHandle_t gSemaphoreRateTransition[LEDC_CHANNEL_MAX] = {0};

// Array to keep track of PWM timers.
static aPwm_t *gpPwmTimerList[LEDC_TIMER_MAX] = {0};

// A place to store the last error code from a call to aPwmOpen().
static int32_t gPwmOpenLastErrorCode = ESP_OK;

// NOTE: there are more variables after the static functions.

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS
 * -------------------------------------------------------------- */

// "end of fade operation" callback function.
// pUserArg should point to the relevant entry in gSemaphoreRateTransition.
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

// Convert a percentage to a duty cycle.
static size_t percentToDuty(int32_t percent)
{
    return ((1 << A_PWM_RESOLUTION_BITS) - 1) * percent / 100;
}

// Find the given PWM pointer in an array of aPwm_t pointers.
static aPwm_t **ppFindEntry(aPwm_t *pPwm, aPwm_t **ppList,
                            size_t listSize)
{
    aPwm_t **ppEntry = NULL;

    if (ppList != NULL) {
        for (size_t x = 0; x < listSize; x++) {
            if (*ppList == pPwm) {
                ppEntry = ppList;
                break;
            }
            ppList++;
        }
    }

    return ppEntry;
}

// Set the rate of a PWM in percent, limiting it to a valid range.
// Note: A_PWM_LOCK()/A_PWM_UNLOCK() should be called
// before this is called.
static int32_t setAndLimitRate(aPwm_t *pPwm, int32_t percent)
{
    int32_t negEspErr = -ESP_ERR_INVALID_ARG;
    int32_t ratePercentOrNegEspErr = percent;
    size_t rateTransitionTimeMs;
    aPwm_t **ppChannel = NULL;
    size_t targetDuty = 0;
    ledc_channel_t pwmChannel;

    if (ratePercentOrNegEspErr < 0) {
        ratePercentOrNegEspErr = 0;
    } else if (ratePercentOrNegEspErr > 100) {
        ratePercentOrNegEspErr = 100;
    }

    rateTransitionTimeMs = pPwm->rateTransitionTimeMs;
    if (rateTransitionTimeMs < A_PWM_RATE_TRANSITION_TIME_MIN_MS) {
        rateTransitionTimeMs = A_PWM_RATE_TRANSITION_TIME_MIN_MS;
    }

    // Find the channel used by this PWM
    ppChannel = ppFindEntry(pPwm, gpPwmChannelList,
                            A_UTIL_ARRAY_COUNT(gpPwmChannelList));
    if (ppChannel != NULL) {
        pwmChannel = ppChannel - gpPwmChannelList;
        // Need to wait on the transition semaphore as we can't
        // change rate if a transition is already in progress;
        // the semaphore is given by the callback at the end
        // of a fade
        xSemaphoreTake(gSemaphoreRateTransition[pwmChannel],
                       (TickType_t) portMAX_DELAY);
        // Fade to the target duty cycle
        targetDuty = percentToDuty(ratePercentOrNegEspErr);
        negEspErr = -ledc_set_fade_time_and_start(A_PWM_SPEED_MODE,
                                                  pwmChannel,
                                                  targetDuty,
                                                  rateTransitionTimeMs,
                                                  LEDC_FADE_NO_WAIT);
    }

    if (negEspErr != ESP_OK) {
        printf(A_LOG_TAG "unable to set PWM \"%s\" rate to %d%%"
              " (duty cycle %d), transition time %d ms (0x%02x)!.\n",
              pPwm->pNameStr, (int) ratePercentOrNegEspErr, targetDuty,
              rateTransitionTimeMs,
              (int) negEspErr);
        ratePercentOrNegEspErr = negEspErr;
    } else {
        pPwm->ratePercent = (size_t) ratePercentOrNegEspErr;
        printf(A_LOG_TAG "PWM \"%s\" rate set to %d%%.\n", pPwm->pNameStr,
               pPwm->ratePercent);
    }

    return  ratePercentOrNegEspErr;
}

// Close a PWM, freeing memory, removing it from the linked list.
// Note: A_PWM_LOCK()/A_PWM_UNLOCK() should be called
// before this is called.
static void pwmClose(aPwm_t *pPwm)
{
    const char *pNameStr;
    aPwm_t **ppTimerOrChannel;
    size_t pwmChannel;
    ledc_timer_config_t pwmTimerConfig = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                          .timer_num = -1, // timer_num is set below
                                          .clk_cfg = LEDC_AUTO_CLK,
                                          .deconfigure = true};

    if (pPwm != NULL) {
        pNameStr = pPwm->pNameStr;
        // Stop the PWM
        setAndLimitRate(pPwm, 0);
        // Free PWM channel and transition semaphore
        ppTimerOrChannel = ppFindEntry(pPwm, gpPwmChannelList,
                                       A_UTIL_ARRAY_COUNT(gpPwmChannelList));
        if (ppTimerOrChannel != NULL) {
            pwmChannel = ppTimerOrChannel - gpPwmChannelList;
            if (gSemaphoreRateTransition[pwmChannel] != NULL) {
                // Wait for any speed transition to complete
                xSemaphoreTake(gSemaphoreRateTransition[pwmChannel],
                               (TickType_t) portMAX_DELAY);
                // Give the semaphore again so that we can delete it
                xSemaphoreGive(gSemaphoreRateTransition[pwmChannel]);
                vSemaphoreDelete(gSemaphoreRateTransition[pwmChannel]);
                gSemaphoreRateTransition[pwmChannel] = NULL;
            }
            *ppTimerOrChannel = NULL;
        }
        // Deconfigure and free PWM timer
        ppTimerOrChannel = ppFindEntry(pPwm, gpPwmTimerList,
                                       A_UTIL_ARRAY_COUNT(gpPwmTimerList));
        if (ppTimerOrChannel != NULL) {
            pwmTimerConfig.timer_num = ppTimerOrChannel - gpPwmTimerList;
            ledc_timer_pause(pwmTimerConfig.speed_mode,
                             pwmTimerConfig.timer_num);
            ledc_timer_config(&pwmTimerConfig);
            *ppTimerOrChannel = NULL;
        }
        // Remove from the linked list and free memory
        aUtilLinkedListRemove(&gpPwmList, pPwm);
        free(pPwm);
        printf(A_LOG_TAG "PWM \"%s\" closed.\n", pNameStr);
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
int32_t aPwmInit()
{
    int32_t negEspErr = ESP_OK;

    if (gMutex == NULL) {
        // Create a mutex to arbitrate activity
        negEspErr = -ESP_ERR_NO_MEM;
        gMutex = xSemaphoreCreateMutex();
        if (gMutex != NULL) {
            // Install PWM
            negEspErr = -ledc_fade_func_install(0);
        }
    }

    if (negEspErr == ESP_OK) {
        printf(A_LOG_TAG "PWM driver initialised.\n");
    } else {
        // Clean up on error
        if (gMutex != NULL) {
            vSemaphoreDelete(gMutex);
            gMutex = NULL;
        }
        printf(A_LOG_TAG "unable to initialise PWM driver"
               " (0x%02x)!\n", (int) negEspErr);
    }

    return negEspErr;
}

// Open a PWM.
aPwm_t *pAPwmOpen(gpio_num_t pin, const char *pNameStr)
{
    int32_t negEspErr;
    aPwm_t *pPwm = NULL;
    ledc_timer_config_t pwmTimerConfig = {.duty_resolution = A_PWM_RESOLUTION_BITS,
                                          .freq_hz = A_PWM_FREQUENCY_HERTZ,
                                          .speed_mode = A_PWM_SPEED_MODE,
                                          .timer_num = -1, // timer_num is set below
                                          .clk_cfg = LEDC_AUTO_CLK,
                                          .deconfigure = false};
    ledc_channel_config_t pwmChannelConfig = {.channel = -1, // Channel is set below
                                              .duty = 0,
                                              .gpio_num = pin,
                                              .speed_mode = A_PWM_SPEED_MODE,
                                              .hpoint = 0,
                                              .timer_sel = -1, // timer_sel is set below
                                              .flags.output_invert = 0};
    aPwm_t **ppTimer = NULL;
    aPwm_t **ppChannel = NULL;

    A_PWM_LOCK(negEspErr);

    negEspErr = -ESP_ERR_NO_MEM;
    // Allocate memory for the PWM
    pPwm = (aPwm_t *) malloc(sizeof(*pPwm));
    if (pPwm != NULL) {
        memset(pPwm, 0, sizeof(*pPwm));
        // Find a spare timer
        ppTimer = ppFindEntry(NULL, gpPwmTimerList,
                              A_UTIL_ARRAY_COUNT(gpPwmTimerList));
        if (ppTimer != NULL) {
            // Found one: allocate it to this PWM and configure the PWM timer
            *ppTimer = pPwm;
            pwmTimerConfig.timer_num = ppTimer - gpPwmTimerList;
            negEspErr = -ledc_timer_config(&pwmTimerConfig);
        }
        if (negEspErr == ESP_OK) {
            // Find a spare PWM channel
            negEspErr = -ESP_ERR_NO_MEM;
            ppChannel = ppFindEntry(NULL, gpPwmChannelList,
                                    A_UTIL_ARRAY_COUNT(gpPwmChannelList));
            if (ppChannel != NULL) {
                // Found one: allocate it to this PWM and configure the PWM
                *ppChannel = pPwm;
                pwmChannelConfig.channel = ppChannel - gpPwmChannelList;
                pwmChannelConfig.timer_sel = pwmTimerConfig.timer_num;
                negEspErr = -ledc_channel_config(&pwmChannelConfig);
                if (negEspErr == ESP_OK) {
                    negEspErr = -ESP_ERR_NO_MEM;
                    // Create a semaphore which we will use to track whether
                    // a rate transition is active or not (otherwise we
                    // can't change rate reliably as a fade command is
                    // ignored if one is already in progress)
                    // Just the one value available in a counting semaphore,
                    // with that value available initially.
                    gSemaphoreRateTransition[pwmChannelConfig.channel] = xSemaphoreCreateCounting(1, 1);
                    if (gSemaphoreRateTransition[pwmChannelConfig.channel] != NULL) {
                        // Register a callback and pass it a pointer to the
                        // mutex so that it can be given by the callback
                        negEspErr = -ledc_cb_register(pwmChannelConfig.speed_mode,
                                                      pwmChannelConfig.channel,
                                                      &gPwmCallbacks,
                                                      (void *) &(gSemaphoreRateTransition[pwmChannelConfig.channel]));
                    }
                }
            }
        }
        if (negEspErr == ESP_OK) {
            // Populate the name and add the PWM
            // to the linked list
            negEspErr = -ESP_ERR_NO_MEM;
            pPwm->pNameStr = pNameStr;
            if (pPwm->pNameStr == NULL) {
                pPwm->pNameStr = "no name";
            }
            // Add the PWM to the linked list
            if (aUtilLinkedListAdd(&gpPwmList, pPwm)) {
                negEspErr = ESP_OK;
            }
        }
    }

    A_PWM_UNLOCK();

    if (negEspErr < 0) {
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
            (gSemaphoreRateTransition[pwmChannelConfig.channel] != NULL)) {
            vSemaphoreDelete(gSemaphoreRateTransition[pwmChannelConfig.channel]);
            gSemaphoreRateTransition[pwmChannelConfig.channel] = NULL;
        }
        free(pPwm);
        gPwmOpenLastErrorCode = negEspErr;
        printf(A_LOG_TAG "unable to open PWM \"%s\", pin %d (0x%02x)!\n",
               pNameStr, pin, (int) negEspErr);
    } else {
        printf(A_LOG_TAG "PWM \"%s\" opened, pin %d.\n", pPwm->pNameStr, pin);
    }

    return pPwm;
}

// Get the last error code from a failed call to pAPwmOpen().
int32_t aPwmOpenLastErrorGetReset()
{
    int32_t negEspErr = gPwmOpenLastErrorCode;

    gPwmOpenLastErrorCode = ESP_OK;

    return negEspErr;
}

// Close a PWM.
void aPwmClose(aPwm_t *pPwm)
{
    int32_t negEspErr;

    A_PWM_LOCK(negEspErr);

    pwmClose(pPwm);

    A_PWM_UNLOCK();

    // To prevent compiler warnings as we aren't returning espErr
    (void) negEspErr;
}

// Set the rate of a PWM relative to its current rate.
int32_t aPwmRateRelativeSet(aPwm_t *pPwm, int32_t percent)
{
    int32_t ratePercentOrNegEspErr;

    A_PWM_LOCK(ratePercentOrNegEspErr);

    ratePercentOrNegEspErr = -ESP_ERR_INVALID_ARG;
    if (pPwm != NULL) {
        percent += (int32_t) pPwm->ratePercent;
        ratePercentOrNegEspErr = setAndLimitRate(pPwm, percent);
    }

    A_PWM_UNLOCK();

    return ratePercentOrNegEspErr;
}

// Set the rate of a PWM to an absolute value.
int32_t aPwmRateAbsoluteSet(aPwm_t *pPwm, size_t percent)
{
    int32_t ratePercentOrNegEspErr;

    A_PWM_LOCK(ratePercentOrNegEspErr);

    ratePercentOrNegEspErr = -ESP_ERR_INVALID_ARG;
    if (pPwm != NULL) {
        ratePercentOrNegEspErr = setAndLimitRate(pPwm, percent);
    }

    A_PWM_UNLOCK();

    return ratePercentOrNegEspErr;
}

// Get the current rate of a PWM.
int32_t aPwmRateGet(aPwm_t *pPwm)
{
    int32_t ratePercentOrNegEspErr;

    A_PWM_LOCK(ratePercentOrNegEspErr);

    ratePercentOrNegEspErr = -ESP_ERR_INVALID_ARG;
    if (pPwm != NULL) {
        ratePercentOrNegEspErr = (int32_t) pPwm->ratePercent;
    }

    A_PWM_UNLOCK();

    return ratePercentOrNegEspErr;
}

// Set the transition time for a rate change.
int32_t aPwmRateTransitionTimeSet(aPwm_t *pPwm, size_t timeMs)
{
    int32_t timeMsOrNegEspErr;

    A_PWM_LOCK(timeMsOrNegEspErr);

    timeMsOrNegEspErr = -ESP_ERR_INVALID_ARG;
    if (pPwm != NULL) {
        pPwm->rateTransitionTimeMs = timeMs;
        timeMsOrNegEspErr = (int32_t) pPwm->rateTransitionTimeMs;
        printf(A_LOG_TAG "PWM \"%s\" transition time set to %d ms.\n",
               pPwm->pNameStr, (int) timeMsOrNegEspErr);
    }

    A_PWM_UNLOCK();

    return timeMsOrNegEspErr;
}

// Get the transition time for a rate change.
int32_t aPwmRateTransitionTimeGet(aPwm_t *pPwm)
{
    int32_t timeMsOrNegEspErr;

    A_PWM_LOCK(timeMsOrNegEspErr);

    timeMsOrNegEspErr = -ESP_ERR_INVALID_ARG;
    if (pPwm != NULL) {
        timeMsOrNegEspErr = (int32_t) pPwm->rateTransitionTimeMs;
    }

    A_PWM_UNLOCK();

    return timeMsOrNegEspErr;
}

// Deinitialise PWM and free resources.
void aPwmDeinit()
{
    int32_t negEspErr;
    aPwm_t *pPwm;

    A_PWM_LOCK(negEspErr);

    // Free each PWM
    do {
        pPwm = pAUtilLinkedListGetFirst(&gpPwmList, NULL);
        // The following function will remove the PWM,
        // which was at the front of the list, so we always
        // only need to do a "Get First" in this loop.
        pwmClose(pPwm);
    } while (pPwm != NULL);

    A_PWM_UNLOCK();

    if (gMutex != NULL) {
        // Uninstall PWM
        ledc_fade_func_uninstall();
        // Destroy the mutex
        vSemaphoreDelete(gMutex);
        gMutex = NULL;
    }

    // To prevent compiler warnings as we aren't returning negEspErr
    (void) negEspErr;
}

// End of file
