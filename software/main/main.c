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

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

// MCU pins.
#define A_PIN_I2C_SCL                           7
#define A_PIN_I2C_SDA                           8
#define A_PIN_SENSOR_HALL_EFFECT_DISABLE_LEFT   9
#define A_PIN_SENSOR_HALL_EFFECT_DISABLE_RIGHT 10
#define A_PIN_SENSOR_HALL_EFFECT_INT_LEFT      11
#define A_PIN_SENSOR_HALL_EFFECT_INT_RIGHT     12

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

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

// Storage for the readings from left-hand TMAG5273.
static int32_t gReadingLeft[1000];

// Storage for the readings from right-hand TMAG5273.
static int32_t gReadingRight[1000];

// How many entries have been added to gReadingLeft.
static size_t gReadingCountLeft = 0;

// How many entries have been added to gReadingRight.
static size_t gReadingCountRight = 0;

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS
 * -------------------------------------------------------------- */

// Callback for magnetic flux readings from the hall effect sensors.
static void callbackRead(aSensorHallEffectDirection_t direction,
                         int32_t fluxTeslaX1e6,
                         void *pParameter)
{
    (void) pParameter;

    gCallbackReadCount++;
    if (direction == 0) {
        if (gReadingCountLeft < sizeof(gReadingLeft) / sizeof(gReadingLeft[0])) {
            gReadingLeft[gReadingCountLeft] = fluxTeslaX1e6;
            gReadingCountLeft++;
        }
    }
    if (direction == 1) {
        if (gReadingCountRight < sizeof(gReadingRight) / sizeof(gReadingRight[0])) {
            gReadingRight[gReadingCountRight] = fluxTeslaX1e6;
            gReadingCountRight++;
        }
    }
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * -------------------------------------------------------------- */

void app_main(void)
{
    esp_err_t espErr;
    i2c_master_bus_handle_t busHandle;
    int64_t sum;
    int32_t rangeUpper;
    int32_t rangeLower;

    // Open the I2C bus
    espErr = i2c_new_master_bus(&gI2cMasterBusConfig, &busHandle);
    if (espErr == ESP_OK) {
        for (size_t x = 0; (espErr == ESP_OK) && (x < 2); x++) {
            // Initialise the hall effect stuff
            espErr = aSensorHallEffectInit(busHandle, A_PIN_SENSOR_HALL_EFFECT_DISABLE_LEFT,
                                           A_PIN_SENSOR_HALL_EFFECT_DISABLE_RIGHT);
            if (espErr == ESP_OK) {
                // Open the hall effect stuff
                espErr = aSensorHallEffectOpen(busHandle);
                if (espErr == ESP_OK) {
                    // Start the hall effect stuff reading
                    espErr = aSensorHallEffectReadStart(callbackRead, NULL,
                                                        A_PIN_SENSOR_HALL_EFFECT_INT_LEFT,
                                                        A_PIN_SENSOR_HALL_EFFECT_INT_RIGHT);
                    if (espErr == ESP_OK) {
                        // Wait a little while before stopping
                        printf("Waiting for readings.\n");
                        aUtilDelayMs(5000);
                        aSensorHallEffectReadStop();
                        printf("Callback called %d time(s), stored %d left reading(s),"
                               " %d right reading(s), here are the values in micro-Tesla:\n",
                               gCallbackReadCount, gReadingCountLeft,
                               gReadingCountRight);
                        sum = 0;
                        rangeUpper = gReadingLeft[0];
                        rangeLower = rangeUpper;
                        for (size_t y = 0; y < gReadingCountLeft; y++) {
                            printf("%d ", (int) gReadingLeft[y]);
                            sum += gReadingLeft[y];
                            if (gReadingLeft[y] > rangeUpper) {
                                rangeUpper = gReadingLeft[y];
                            }
                            if (gReadingLeft[y] < rangeLower) {
                                rangeLower = gReadingLeft[y];
                            }
                        }
                        printf("\nAverage %lld, upper %d, lower %d, range %d.\n",
                               sum / gReadingCountLeft, (int) rangeUpper, (int) rangeLower,
                               (int) (rangeUpper - rangeLower));
                        sum = 0;
                        rangeUpper = gReadingRight[0];
                        rangeLower = rangeUpper;
                        for (size_t y = 0; y < gReadingCountRight; y++) {
                            printf("%d ", (int) gReadingRight[y]);
                            sum += gReadingRight[y];
                            if (gReadingRight[y] > rangeUpper) {
                                rangeUpper = gReadingRight[y];
                            }
                            if (gReadingRight[y] < rangeLower) {
                                rangeLower = gReadingRight[y];
                            }
                        }
                        printf("\nAverage %lld, upper %d, lower %d, range %d.\n",
                               sum / gReadingCountRight, (int) rangeUpper, (int) rangeLower,
                               (int) (rangeUpper - rangeLower));
                        gCallbackReadCount = 0;
                        gReadingCountLeft = 0;
                        gReadingCountRight = 0;
                    } else {
                        printf("Unable to start read of hall effect sensors (0x%02x)!\n", espErr);
                    }
                    aSensorHallEffectClose();
                    printf("Finished.\n");
                } else {
                    printf("Unable to open hall effect sensors (0x%02x)!\n", espErr);
                }
                aSensorHallEffectDeinit();
            } else {
                printf("Unable to initialise hall effect sensors (0x%02x)!\n", espErr);
            }
        }
        i2c_del_master_bus(busHandle);
    } else {
        printf("Unable to open I2C bus (0x%02x)!\n", espErr);
    }

    printf("If this is the ESP-IDF monitor program, press CTRL ] to terminate it.\n");
}

// End of file
