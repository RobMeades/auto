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

#ifndef _A_SENSOR_HALL_EFFECT_H_
#define _A_SENSOR_HALL_EFFECT_H_

/** @file
 * @brief Definition of the hall effect sensor API, used to control a
 * pair of hall effect sensors, one facing left, one facing right.
 * This API is not thread-safe, though readings from the hall effect
 * sensors, of course, are.
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

/** The direction that a hall effect sensor is facing.
 */
typedef enum {
    A_SENSOR_HALL_EFFECT_DIRECTION_LEFT,
    A_SENSOR_HALL_EFFECT_DIRECTION_RIGHT,
    A_SENSOR_HALL_EFFECT_DIRECTION_NUM
} aSensorHallEffectDirection_t;

/** Definition of a function that will receive readings from a
 * hall effect sensor, called once aSensorHallEffectReadStart()
 * has returned successfully, not called again after
 * aSensorHallEffectReadStop(), aSensorHallEffectClose() or
 * aSensorHallEffectDeinit() is called.
 *
 * @param direction     the direction that the hall effect sensor
 *                      which produced this reading is facing.
 * @param fluxTeslaX1e6 the magnetic flux read by the sensor in
 *                      micro-Teslas.
 * @param pParameter    the parameter that was passed to 
 *                      aSensorHallEffectReadStart().
 */
typedef void (*aSensorHallEffectCallbackRead_t) (aSensorHallEffectDirection_t direction,
                                                 int32_t fluxTeslaX1e6,
                                                 void *pParameter);

/* ----------------------------------------------------------------
 * FUNCTIONS
 * -------------------------------------------------------------- */

/** Initialise the hall effect sensor API.  Must be called before
 * any other hall effect sensor function can be used; if the API
 * has already been initialised, this will have the effect of
 * resetting the hall effect sensors.  Call aSensorHallEffectDeinit()
 * to free resources and deinitialise the API.
 *
 * @param busHandle       the handle of an opened I2C bus API on which
 *                        the hall effect sensors can be found.
 * @param pinDisableLeft  an MCU pin which, when set high, will
 *                        disable/reset the left-hand hall effect
 *                        sensor.
 * @param pinDisableRight an MCU pin which, when set high, will
 *                        disable/reset the right-hand hall effect
 *                        sensor.
 * @return                ESP_OK on success, else negative value from
 *                        esp_err_t.
 */
int32_t aSensorHallEffectInit(i2c_master_bus_handle_t busHandle,
                              gpio_num_t pinDisableLeft,
                              gpio_num_t pinDisableRight);

/** Open the hall effect sensors so that they can be read;
 * aSensorHallEffectInit() must have returned successfully before
 * this function will return successfully.  Use aSensorHallEffectClose()
 * to close the sensors once more.
 *
 * @param busHandle      the handle of an opened I2C bus API on which
 *                       the hall effect sensors can be found.
 * @return               ESP_OK on success, else negative value
 *                       from esp_err_t.
 */
int32_t aSensorHallEffectOpen(i2c_master_bus_handle_t busHandle);

/** Close the hall effect sensors that were opened with a call to
 * aSensorHallEffectOpen().  This does not power the sensors down;
 * for that use aSensorHallEffectDeinit().
 */
void aSensorHallEffectClose();

/** Start reading the hall effect sensors.  aSensorHallEffectOpen()
 * must have been called for this function to succeed.  Call
 * aSensorHallEffectReadStop() to stop readings once more.
 *
 * @param pCallbackRead          the callback to be called when a
 *                               hall effect sensor has returned a
 *                               reading.
 * @param pCallbackReadParameter parameter passed to pCallbackRead
 *                               as its last parameter.
 * @param pinIntLeft             the MCU pin which is connected to
 *                               the interrupt output of the left-hand
 *                               hall effect sensor.
 * @param pinIntLeft             the MCU pin which is connected to
 *                               the interrupt output of the right-hand
 *                               hall effect sensor.
 * @return                       ESP_OK on success, else negative value
 *                               from esp_err_t.
 */
int32_t aSensorHallEffectReadStart(aSensorHallEffectCallbackRead_t pCallbackRead,
                                   void *pCallbackReadParameter,
                                   gpio_num_t pinIntLeft,
                                   gpio_num_t pinIntRight);

/** Stop the reading of the hall effect sensors that was started with
 * a call to aSensorHallEffectReadStart().  Once this has returned
 * pCallbackRead will no longer be called.
 */
void aSensorHallEffectReadStop();

/** Deinitialise the hall effect sensor API and free resources.
 * If the sensors were opened, or if a read was in progress,
 * those processes will be stopped in an orderly manner before
 * deinitialisation.  The hall effect sensors will have been
 * disabled when this function returns.
 */
void aSensorHallEffectDeinit();

#ifdef __cplusplus
}
#endif

/** @}*/

#endif // _A_SENSOR_HALL_EFFECT_H_

// End of file
