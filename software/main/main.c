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
 * @brief The application, main().
 */

#include <stdio.h>

#include <driver/gpio.h>
#include <driver/i2c_master.h>

#include <a_util.h>
#include <a_sensor_hall_effect.h>
#include <a_pwm.h>
#include <a_motor.h>

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

// MCU pins.
#define A_PIN_MOTOR_ENABLE                     13
#define A_PIN_MOTOR_DRIVING_PWM                 1
#define A_PIN_MOTOR_DRIVING_CONTROL_1           2
#define A_PIN_MOTOR_DRIVING_CONTROL_2           3
#define A_PIN_MOTOR_STEERING_PWM                4
#define A_PIN_MOTOR_STEERING_CONTROL_1          5
#define A_PIN_MOTOR_STEERING_CONTROL_2          6

#define A_PIN_I2C_SCL                           7
#define A_PIN_I2C_SDA                           8

#define A_PIN_SENSOR_HALL_EFFECT_DISABLE_LEFT   9
#define A_PIN_SENSOR_HALL_EFFECT_DISABLE_RIGHT 10
#define A_PIN_SENSOR_HALL_EFFECT_INT_LEFT      11
#define A_PIN_SENSOR_HALL_EFFECT_INT_RIGHT     12

#define A_PIN_LED_HEADLIGHTS                   17
#define A_PIN_LED_BREAKLIGHTS                  18
#define A_PIN_LED_INDICATOR_LEFT               38
#define A_PIN_LED_INDICATOR_RIGHT              39

// The size of averaging buffer to use.
#define A_AVERAGING_BUFFER_LENGTH 32

// The amount to change the driving speed by at each loop.
#define A_SPEED_CHANGE_PERCENT 11

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

// An averaging buffer.
typedef struct {
    int32_t reading[A_AVERAGING_BUFFER_LENGTH];
    size_t numReadings;
    int32_t *pOldestReading; // non-NULL only when numReadings = A_AVERAGING_BUFFER_LENGTH
    int32_t total;
    int32_t average;
} aAveragingBuffer_t;

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

// I2C bus configuration.
static const i2c_master_bus_config_t gI2cMasterBusConfig = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = 0,
    .scl_io_num = A_PIN_I2C_SCL,
    .sda_io_num = A_PIN_I2C_SDA,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = false
};

// How many times callbackRead() has been called.
static size_t gCallbackReadCount = 0;

// Storage for the readings from both TMAG5273s, left and right.
static aAveragingBuffer_t gBuffers[A_SENSOR_HALL_EFFECT_DIRECTION_NUM] = {0};

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS
 * -------------------------------------------------------------- */

// Add a new reading to the averaging buffer for a given direction.
static void addToAverage(int32_t fluxTeslaX1e6,
                         aSensorHallEffectDirection_t direction)
{
    aAveragingBuffer_t *pAveragingBuffer;

    if (direction < A_UTIL_ARRAY_COUNT(gBuffers)) {
        pAveragingBuffer = &(gBuffers[direction]);
        if (pAveragingBuffer->pOldestReading == NULL) {
            // Haven't yet filled the buffer up, just add the new reading
            // and update the total
            pAveragingBuffer->reading[pAveragingBuffer->numReadings] = fluxTeslaX1e6;
            pAveragingBuffer->numReadings++;
            pAveragingBuffer->total += fluxTeslaX1e6;
            if (pAveragingBuffer->numReadings >= A_UTIL_ARRAY_COUNT(pAveragingBuffer->reading)) {
                pAveragingBuffer->pOldestReading = &(pAveragingBuffer->reading[0]);
            }
        } else {
            // The buffer is full, need to rotate it
            pAveragingBuffer->total -= *pAveragingBuffer->pOldestReading;
            *pAveragingBuffer->pOldestReading = fluxTeslaX1e6;
            pAveragingBuffer->total += fluxTeslaX1e6;
            pAveragingBuffer->pOldestReading++;
            if (pAveragingBuffer->pOldestReading >= pAveragingBuffer->reading + A_UTIL_ARRAY_COUNT(pAveragingBuffer->reading)) {
                pAveragingBuffer->pOldestReading = &(pAveragingBuffer->reading[0]);
            }
        }

        if (pAveragingBuffer->numReadings > 0) {
            // Note: the average becomes an unsigned value unless the
            // denominator is cast to an integer
            pAveragingBuffer->average = pAveragingBuffer->total / (int) pAveragingBuffer->numReadings;
        }
    }
}

// Callback for magnetic flux readings from the hall effect sensors.
static void callbackRead(aSensorHallEffectDirection_t direction,
                         int32_t fluxTeslaX1e6,
                         void *pParameter)
{
    (void) pParameter;

    gCallbackReadCount++;
    addToAverage(fluxTeslaX1e6, direction);
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * -------------------------------------------------------------- */

void app_main(void)
{
    esp_err_t espErr;
    i2c_master_bus_handle_t busHandle;
    aMotor_t *pMotorDriving = NULL;
    aMotor_t *pMotorSteering = NULL;

    // Open the I2C bus
    espErr = i2c_new_master_bus(&gI2cMasterBusConfig, &busHandle);
    if (espErr == ESP_OK) {
        // Initialise the hall effect stuff
        espErr = aSensorHallEffectInit(busHandle,
                                       A_PIN_SENSOR_HALL_EFFECT_DISABLE_LEFT,
                                       A_PIN_SENSOR_HALL_EFFECT_DISABLE_RIGHT);
        if (espErr == ESP_OK) {
            // Initialse PWM
            espErr = aPwmInit();
        }
        if (espErr == ESP_OK) {
            // Initialise the motor driver
            espErr = aMotorInit(A_PIN_MOTOR_ENABLE);
        }
        if (espErr == ESP_OK) {
            // Open the hall effect stuff
            espErr = aSensorHallEffectOpen(busHandle);
        }
        if (espErr == ESP_OK) {
            // Open the driving motor
            pMotorDriving = pAMotorOpen(A_PIN_MOTOR_DRIVING_PWM,
                                        A_PIN_MOTOR_DRIVING_CONTROL_1,
                                        A_PIN_MOTOR_DRIVING_CONTROL_2,
                                        "driving");
        }
        if (pMotorDriving != NULL) {
            // Open the steering motor
            pMotorSteering = pAMotorOpen(A_PIN_MOTOR_STEERING_PWM,
                                         A_PIN_MOTOR_STEERING_CONTROL_1,
                                         A_PIN_MOTOR_STEERING_CONTROL_2,
                                         "steering");
        } else {
            espErr = aMotorOpenLastErrorGetReset();
        }
        if (pMotorSteering != NULL) {
            // Configure the driving motor to transit between speeds gently
            aMotorSpeedTransitionTimeSet(pMotorDriving, 10);
            // Full speed ahead!
            aMotorSpeedAbsoluteSet(pMotorDriving, 100);
            // Start the hall effect stuff reading
            espErr = aSensorHallEffectReadStart(callbackRead, NULL,
                                                A_PIN_SENSOR_HALL_EFFECT_INT_LEFT,
                                                A_PIN_SENSOR_HALL_EFFECT_INT_RIGHT);
        } else {
            espErr = aMotorOpenLastErrorGetReset();
        }
        if (espErr == ESP_OK) {
            printf("Reading (in micro-Teslas); if this is the ESP-IDF monitor"
                   " program, press CTRL ] to terminate:\n");
            int64_t startTimeMs = aUtilTimeSinceBootMs();
#if OUTPUT_CSV
            printf("\"Time (ms)\", \"Left (uTesla)\", \"Right (uTesla)\"\n");
#else
            size_t readingsPerSecond = 0;
#endif
            while (1) {
                // Print the readings out; gCallbackReadCount / 2 as two callbacks
                // are required, one from the left-hand hall effect sensor and
                // one from the right-hand hall effect sensor, for a single reading
#if OUTPUT_CSV
                // Print the output in comma-separated variable form so that
                // it can be imported into Excel
                printf("%lld, %d, %d\n", aUtilTimeSinceBootMs() - startTimeMs,
                       (int) gBuffers[A_SENSOR_HALL_EFFECT_DIRECTION_LEFT].average,
                       (int) gBuffers[A_SENSOR_HALL_EFFECT_DIRECTION_RIGHT].average);
#else
                // The extra characters on the end below make the line long enough that
                // it gets flushed immediately; without that the output is jerky
                printf("%10d (%4d/second) reading:       %6d     <--> %6d                \n",
                      gCallbackReadCount >> 1, readingsPerSecond,
                      (int) gBuffers[A_SENSOR_HALL_EFFECT_DIRECTION_LEFT].average,
                      (int) gBuffers[A_SENSOR_HALL_EFFECT_DIRECTION_RIGHT].average);
                int64_t durationSeconds = (aUtilTimeSinceBootMs() - startTimeMs) / 1000;
                if (durationSeconds > 0) {
                    // Divide by two for the same reason as above
                    readingsPerSecond = (gCallbackReadCount / durationSeconds) >> 1;
                }
#endif
                // Don't crowd the output, let the idle task in
                aUtilDelayMs(100);
            }
        }
        aSensorHallEffectClose();
        aMotorClose(pMotorDriving);
        aMotorClose(pMotorSteering);
        aSensorHallEffectDeinit();
        aMotorDeinit();
        i2c_del_master_bus(busHandle);
    }

    if (espErr == ESP_OK) {
        printf("Finished.\n");
    } else {
        printf("Unable to start, ESP error 0x%02x.\n", espErr);
    }
    printf("If this is the ESP-IDF monitor program, press CTRL ] to terminate it.\n");
}

// End of file
