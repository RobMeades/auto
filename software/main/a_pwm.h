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

#ifndef _A_PWM_H_
#define _A_PWM_H_

/** @file
 * @brief Definition of the PWM API, used by the motor API
 * and also for driving LEDs. This API is thread-safe with the
 * exception of aMotorInit() and aMotorDeinit(), which must not be
 * called at the same time as any other of these API calls.
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

/** A structure defining a PWM entry; this should NOT be referred to
 * by the application, should not be read/written, do not rely
 * on its contents, they are subject to change.  It is only exposed
 * here in order that it can be passed around by the application;
 * consider it to be anonymous.
 */
typedef struct {
    const char *pNameStr;
    size_t ratePercent;
    size_t rateTransitionTimeMs;
} aPwm_t;

/* ----------------------------------------------------------------
 * FUNCTIONS
 * -------------------------------------------------------------- */

/** Initialise the PWM API.  Must be called before any other PWM
 * function can be used.  If the API is already initialised this
 * function will do nothing and return ESP_OK.  Call aPwmDeinit()
 * to deinitialise the API and free resources.
 *
 * @return  ESP_OK on success, else negative error code.
 */
esp_err_t aPwmInit();

/** Open a PWM; aPwmInit() must have returned successfully
 * before this function will return successfully.  Use aPwmClose()
 * to close a PWM once more.  A newly opened PWN will have a rate
 * and a rate transition time of zero.
 *
 * @param pin       the MCU pin that is to output PWM.
 * @param pNameStr  optional name for the PWM, must be
 *                  a true constant (i.e. the string is not
 *                  copied, only the pointer is copied), may
 *                  be NULL.
 * @return          on success a non-NULL pointer which can
 *                  be passed to the PWM functions of this
 *                  API, else NULL.  If the return value is
 *                  NULL aPwmOpenLastErrorGetReset()
 *                  should be called to get and reset the
 *                  error value.
 */
aPwm_t *pAPwmOpen(gpio_num_t pin, const char *pNameStr);

/** Get the error code from a failed call to pAPwmOpen(), resetting
 * the error code to zero afterwards.  Only the last result of a
 * _failed_ call is stored, i.e. if pAPwmOpen() fails then
 * pAPwmOpen() succeeds, a call to this function will return
 * the error code from the first call, the failed one, hence it is
 * good practice to always call this function when pAPwmOpen() has
 * returned NULL, to avoid confusion.
 *
 * @return the last code from a failed call to pAPwmOpen().
 */
esp_err_t aPwmOpenLastErrorGetReset();

/** Close a PWM that was opened with a call to pAPwmOpen(),
 * freeing resources.  If the PWM was running it will be
 * stopped in an organised way, respecting transition times,
 * and set to 0.
 *
 * @param[in] pPwm the PWM to close, as was returned by
 *                 pAPwmOpen(); may be NULL.
 */
void aPwmClose(aPwm_t *pPwm);

/** Set the rate of a PWM relative to its current rate.
 *
 * @param[in] pPwm  the PWM to set the rate of; cannot be NULL.
 * @param percent   the percentage to change by, positive to
 *                  increase, negative to decrease.  The rate
 *                  will be capped at zero and 100%.
 * @return          on success the new rate, else a negative
 *                  value from esp_err_t.
 */
int32_t aPwmRateRelativeSet(aPwm_t *pPwm, int32_t percent);

/** Set the rate of a PWM to an absolute value.
 *
 * @param[in] pPwm  the PWM to set the rate of; cannot be NULL.
 * @param percent   the rate as a percentage; if more than 100%
 *                  is specified then 100% will be applied.
 * @return          on success the new rate as a percentage,
 *                  else a negative value from esp_err_t.
 */
int32_t aPwmRateAbsoluteSet(aPwm_t *pPwm, size_t percent);

/** Get the current rate of a PWM.
 *
 * @param[in] pPwm  the PWM to get the rate of; cannot be NULL.
 * @return          on success the current rate as a
 *                  percentage, else a negatve value from
 *                  esp_err_t.
 */
int32_t aPwmRateGet(aPwm_t *pPwm);

/** Set the transition time for a rate change.
 *
 * @param[in] pPwm  the PWM to set the transition time of;
 *                  cannot be NULL.
 * @param timeMs    the time that a rate change should
 *                  take in milliseconds.
 * @return          on success the new transition time in
 *                  milliseconds, else a negative value
 *                  from esp_err_t.
 */
int32_t aPwmRateTransitionTimeSet(aPwm_t *pPwm, size_t timeMs);

/** Get the transition time for a rate change.
 *
 * @param[in] pPwm  the PWM to get the transition time of;
 *                  cannot be NULL.
 * @return          on success the transition time for a
 *                  rate change in milliseconds, else a
 *                  negatve value from esp_err_t.
 */
int32_t aPwmRateTransitionTimeGet(aPwm_t *pPwm);

/** Deinitialise the PWM API and free resources.  If PWM(s)
 * have been opened they will be stopped and closed in an
 * orderly manner, respecting transition times.  aPwmInit()
 * must be called before this API can be used once more.
 */
void aPwmDeinit();

#ifdef __cplusplus
}
#endif

/** @}*/

#endif // _A_PWM_H_

// End of file
