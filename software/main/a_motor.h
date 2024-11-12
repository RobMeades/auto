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

#ifndef _A_MOTOR_H_
#define _A_MOTOR_H_

/** @file
 * @brief Definition of the motor API, used to drive a motor. This
 * API is thread-safe with the exception of aMotorInit() and
 * aMotorDeinit(), which must not be called at the same time as
 * any other of these API calls.
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

/** A structure defining a motor; this should NOT be referred to
 * by the application, should not be read/written, do not rely
 * on its contents, they are subject to change.  It is only exposed
 * here in order that it can be passed around by the application;
 * consider it to be anonymous.
 */
typedef struct {
    const char *pNameStr;
    gpio_num_t pinMotorControl1;
    gpio_num_t pinMotorControl2;
    bool directionIsAnticlockwise;
    size_t speedPercent;
    size_t speedTransitionTimeMs;
} aMotor_t;

/* ----------------------------------------------------------------
 * FUNCTIONS
 * -------------------------------------------------------------- */

/** Initialise the motor API.  Must be called before any other motor
 * function can be used.  If the API is already initialised this
 * function will do nothing and return ESP_OK.  Call aMotorDeinit()
 * to deinitialise the API and free resources.
 *
 * @param pinEnable  an MCU pin which, when pulled high, enables
 *                   drive to the motors; use -1 if there is no
 *                   such pin.
 * @return           ESP_OK on success, else error code.
 */
esp_err_t aMotorInit(gpio_num_t pinEnable);

/** Open a motor; aMotorInit() must have returned successfully
 * before this function will return successfully.  Use aMotorClose()
 * to close a motor once more.  A motor will be opened with zero
 * speed, clockwise direction and zero speed transition time.
 *
 * @param pinPwm            the MCU pin connected to the PWM input
 *                          of the motor driver,
 * @param pinMotorControl1  the MCU pin connected to control pin 1
 *                          of the motor driver.
 * @param pinMotorControl2  the MCU pin connected to control pin 2
 *                          of the motor driver.
 * @param pNameStr          optional name for the motor, must be
 *                          a true constant (i.e. the string is not
 *                          copied, only the pointer is copied), may
 *                          be NULL.
 * @return                  on success a non-NULL pointer which can
 *                          be passed to the motor control functions
 *                          of this API, else NULL.  If the return
 *                          value is NULL aMotorOpenLastErrorGetReset()
 *                          should be called to get and reset the
 *                          error value.
 */
aMotor_t *pAMotorOpen(gpio_num_t pinPwm,
                      gpio_num_t pinMotorControl1,
                      gpio_num_t pinMotorControl2,
                      const char *pNameStr);

/** Get the error code from a failed call to pAMotorOpen(), resetting
 * the error code to zero afterwards.  Only the last result of a
 * _failed_ call is stored, i.e. if pAMotorOpen() fails then
 * pAMotorOpen() succeeds, a call to this function will return
 * the error code from the first call, the failed one, hence it is
 * good practice to always call this function when pAMotorOpen() has
 * returned NULL, to avoid confusion.
 *
 * @return the last code from a failed call to pAMotorOpen().
 */
esp_err_t aMotorOpenLastErrorGetReset();

/** Close a motor that was opened with a call to aMotorOpen(),
 * freeing resources.  If the motor was running it will be
 * stopped and, if possible, disabled.
 *
 * @param[in] pMotor the motor to close, as was returned by
 *                   pAMotorOpen(); may be NULL.
 */
void aMotorClose(aMotor_t *pMotor);

/** Set the direction of motor's rotation to clockwise.
 *
 * @param[in] pMotor the motor to set the direction of; cannot
 *                   be NULL.
 * @return           ESP_OK on success, else error code.
 */
esp_err_t aMotorDirectionClockwiseSet(aMotor_t *pMotor);

/** Set the direction of motor's rotation to anti-clockwise.
 *
 * @param[in] pMotor the motor to set the direction of; cannot
 *                   be NULL.
 * @return           ESP_OK on success, else error code.
 */
esp_err_t aMotorDirectionAnticlockwiseSet(aMotor_t *pMotor);

/** Get the direction of a motor's rotation.
 *
 * @param[in] pMotor the motor to get the direction of;
 *                   cannot be NULL.
 * @return           on success a positive value, 0 for
 *                   clockwise, 1 for anti-clockwise,
 *                   else a negatve value from esp_err_t.
 */
int32_t aMotorDirectionGet(aMotor_t *pMotor);

/** Set the speed of a motor relative to its current
 * speed by the given percentage of the maximum speed.
 *
 * @param[in] pMotor the motor to set the speed of; cannot
 *                   be NULL.
 * @param percent    the percentage of the maximum speed
 *                   to change by, positive to increase,
 *                   negative to decrease.  The speed
 *                   will be capped at zero and 100%.
 * @return           on success the new speed as a percentage
 *                   of the maximum speed once the relative
 *                   change has been applied, else a
 *                   negative value from esp_err_t.
 */
int32_t aMotorSpeedRelativeSet(aMotor_t *pMotor, int32_t percent);

/** Set the speed of a motor to an absolute value.
 *
 * @param[in] pMotor the motor to set the speed of; cannot
 *                   be NULL.
 * @param percent    the speed as a percentage of the maximum;
 *                   if more than 100% is specified then 100%
 *                   will be applied.
 * @return           on success the new speed as a percentage
 *                   of the maximum speed, else a negative
 *                   value from esp_err_t.
 */
int32_t aMotorSpeedAbsoluteSet(aMotor_t *pMotor, size_t percent);

/** Get the current speed of a motor as a percentage of
 * the maximum speed.
 *
 * @param[in] pMotor the motor to get the speed of; cannot
 *                   be NULL.
 * @return           on success the current speed, as a
 *                   percentage of the maximum speed, else
 *                   a negatve value from esp_err_t.
 */
int32_t aMotorSpeedGet(aMotor_t *pMotor);

/** Set the transition time for a speed change.
 *
 * @param[in] pMotor the motor to set the transition time of;
 *                   cannot be NULL.
 * @param timeMs     the time that a speed change should
 *                   take in milliseconds.
 * @return           on success the new transition time in
 *                   milliseconds, else a negative value
 *                   from esp_err_t.
 */
int32_t aMotorSpeedTransitionTimeSet(aMotor_t *pMotor, size_t timeMs);

/** Get the transition time for a speed change.
 *
 * @param[in] pMotor    the motor to get the transition time of;
 *                      cannot be NULL.
 * @return              on success the transition time for a
 *                      speed change in milliseconds, else a
 *                      negatve value from esp_err_t.
 */
int32_t aMotorSpeedTransitionTimeGet(aMotor_t *pMotor);

/** Deinitialise the motor API and free resources.  If motor(s)
 * have been opened they will be stopped, disabled if possible,
 * and closed in an orderly manner.  aMotorInit() must be called
 * before this API can be used once more.
 */
void aMotorDeinit();

#ifdef __cplusplus
}
#endif

/** @}*/

#endif // _A_MOTOR_H_

// End of file
