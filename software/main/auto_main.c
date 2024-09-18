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

// TODO:
// - Convert readings into something meaningful
// - Make a SensorHallEffect API for the main application, with a TMAG5273 implementation of it

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_timer.h>

#include <driver/gpio.h>
#include <driver/i2c_master.h>

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

#ifndef A_I2C_SPEED_TMAG5273_HZ
// I2C speed for the TMAG5273.
# define A_I2C_SPEED_TMAG5273_HZ 400000
#endif

// I2C addresses.
#define A_I2C_ADDRESS_TMAG5273_SPARKFUN_DEFAULT 0x22
#define A_I2C_ADDRESS_TMAG5273_SPARKFUN_LEFT A_I2C_ADDRESS_TMAG5273_SPARKFUN_DEFAULT
#define A_I2C_ADDRESS_TMAG5273_SPARKFUN_RIGHT (A_I2C_ADDRESS_TMAG5273_SPARKFUN_DEFAULT + 1)

// Register addresses in the TMAG5273 device, see
// https://www.ti.com/lit/ds/symlink/tmag5273.pdf
#define A_TMAG5273_REG_ADDRESS_DEVICE_CONFIG_1      0x00
#define A_TMAG5273_REG_ADDRESS_DEVICE_CONFIG_2      0x01
#define A_TMAG5273_REG_ADDRESS_SENSOR_CONFIG_1      0x02
#define A_TMAG5273_REG_ADDRESS_T_CONFIG             0x07
#define A_TMAG5273_REG_ADDRESS_INT_CONFIG_1         0x08
#define A_TMAG5273_REG_ADDRESS_I2C_ADDRESS          0x0c
#define A_TMAG5273_REG_ADDRESS_DEVICE_ID            0x0d
#define A_TMAG5273_REG_ADDRESS_MANUFACTURER_ID_LSB  0x0e
#define A_TMAG5273_REG_ADDRESS_MANUFACTURER_ID_MSB  0x0f
#define A_TMAG5273_REG_ADDRESS_CONV_STATUS          0x18

// Static register contents for a TMAG5273 device, DEVICE_CONFIG_1.
#define A_TMAG5273_REG_CONTENTS_DEVICE_CONFIG_1_CRC_EN      0x00  // Bit 7
#define A_TMAG5273_REG_CONTENTS_DEVICE_CONFIG_1_MAG_TEMPCO  0x00  // Bits 6 and 5
#define A_TMAG5273_REG_CONTENTS_DEVICE_CONFIG_1_CONV_AVG    0x05  // Bits 4, 3 and 2

// Static register contents for a TMAG5273 device, DEVICE_CONFIG_2.
#define A_TMAG5273_REG_CONTENTS_DEVICE_CONFIG_2_THR_HYST          0x00  // Bits 7, 6 and 5
#define A_TMAG5273_REG_CONTENTS_DEVICE_CONFIG_2_LP_LN             0x00  // Bit 4
#define A_TMAG5273_REG_CONTENTS_DEVICE_CONFIG_2_I2C_GLITCH_FILTER 0x00  // Bit 3
#define A_TMAG5273_REG_CONTENTS_DEVICE_CONFIG_2_TRIGGER_MODE      0x00  // Bit 2

// Static register contents for a TMAG5273 device, SENSOR_CONFIG_1.
#define A_TMAG5273_REG_CONTENTS_SENSOR_CONFIG_1_MAG_CH_EN   A_TMAG5273_MAGNETIC_CHANNEL_Y // Bits 7 to 4
#define A_TMAG5273_REG_CONTENTS_SENSOR_CONFIG_1_SLEEPTIME   A_TMAG5273_SLEEP_TIME_1_MS    // Bits 3 to 0

// Static register contents for a TMAG5273 device, INT_CONFIG_1.
#define A_TMAG5273_REG_CONTENTS_INT_CONFIG_1_THRSLD_INT 0x00  // Bit 6
#define A_TMAG5273_REG_CONTENTS_INT_CONFIG_1_INT_STATE  0x00  // Bit 5
#define A_TMAG5273_REG_CONTENTS_INT_CONFIG_1_RESERVED   0x00  // Bit 1
#define A_TMAG5273_REG_CONTENTS_INT_CONFIG_1_MASK_INTB  0x00  // Bit 0

// MCU pins.
#define A_PIN_I2C_SCL                           7
#define A_PIN_I2C_SDA                           8
#define A_PIN_SENSOR_HALL_EFFECT_DISABLE_LEFT   9
#define A_PIN_SENSOR_HALL_EFFECT_DISABLE_RIGHT 10
#define A_PIN_SENSOR_HALL_EFFECT_INT_LEFT      11
#define A_PIN_SENSOR_HALL_EFFECT_INT_RIGHT     12

#ifndef A_READ_TASK_STACK_SIZE_BYTES
// Stack size for the hall effect sensor read task (in bytes).
# define A_READ_TASK_STACK_SIZE_BYTES (1024 * 2)
#endif

#ifndef A_READ_TASK_PRIORITY
// Priority of the hall effect sensor read task.
# define A_READ_TASK_PRIORITY 10
#endif

#ifndef A_READ_TASK_EXIT_WAIT_MS
// How long to wait for the hall effect sensor read task to exit.
# define A_READ_TASK_EXIT_WAIT_MS 1000
#endif

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

// The operating mode of a TMAG5273 device, see Table 8-4 of the data sheet.
typedef enum {
    A_TMAG5273_OPERATING_MODE_STANDBY_TRIGGERED = 0,
    A_TMAG5273_OPERATING_MODE_SLEEP = 1,
    A_TMAG5273_OPERATING_MODE_CONTINUOUS = 2,
    A_TMAG5273_OPERATING_MODE_WAKE_UP_AND_SLEEP = 3
} aTmag5273OperatingMode_t;

// The read mode of a TMAG5273 device, see Table 8-3 of the data sheet.
typedef enum {
    A_TMAG5273_READ_MODE_STANDARD_3_BYTE = 0,
    A_TMAG5273_READ_MODE_1_BYTE_16_BIT = 1,
    A_TMAG5273_READ_MODE_1_BYTE_8_BIT = 2
} aTmag5273ReadMode_t;

// The magnetic channel combinations for a 1-byte read, see Table 8-5
// of the data sheet.
typedef enum {
    A_TMAG5273_MAGNETIC_CHANNEL_NONE  = 0,  // the default, just 16-bit temperature if enabled
    A_TMAG5273_MAGNETIC_CHANNEL_X     = 1,  // 1 16-bit word, plus 16-bit temperature (first) if enabled
    A_TMAG5273_MAGNETIC_CHANNEL_Y     = 2,  // 1 16-bit word, plus 16-bit temperature (first) if enabled
    A_TMAG5273_MAGNETIC_CHANNEL_X_Y   = 3,  // 2 16-bit word, plus 16-bit temperature (first) if enabled
    A_TMAG5273_MAGNETIC_CHANNEL_Z     = 4,  // 1 16-bit word, plus 16-bit temperature (first) if enabled
    A_TMAG5273_MAGNETIC_CHANNEL_Z_X   = 5,  // 2 16-bit words, plus 16-bit temperature (first) if enabled
    A_TMAG5273_MAGNETIC_CHANNEL_Y_Z   = 6,  // 2 16-bit words, plus 16-bit temperature (first) if enabled
    A_TMAG5273_MAGNETIC_CHANNEL_X_Y_Z = 7,  // 3 16-bit words, plus 16-bit temperature (first) if enabled
    A_TMAG5273_MAGNETIC_CHANNEL_X_Y_X = 8,  // 3 16-bit words, plus 16-bit temperature (first) if enabled
    A_TMAG5273_MAGNETIC_CHANNEL_Y_X_Y = 9,  // 3 16-bit words, plus 16-bit temperature (first) if enabled
    A_TMAG5273_MAGNETIC_CHANNEL_Y_Z_Y = 10, // 3 16-bit words, plus 16-bit temperature (first) if enabled
    A_TMAG5273_MAGNETIC_CHANNEL_X_Z_X = 11  // 3 16-bit words, plus 16-bit temperature (first) if enabled
} aTmag5273MagneticChannels_t;

// The gap between measurements in A_TMAG5273_OPERATING_MODE_WAKE_UP_AND_SLEEP,
// see Table 8-5.
typedef enum {
    A_TMAG5273_SLEEP_TIME_1_MS = 0,
    A_TMAG5273_SLEEP_TIME_5_MS = 1,
    A_TMAG5273_SLEEP_TIME_10_MS = 2,
    A_TMAG5273_SLEEP_TIME_15_MS = 3,
    A_TMAG5273_SLEEP_TIME_20_MS = 4,
    A_TMAG5273_SLEEP_TIME_30_MS = 5,
    A_TMAG5273_SLEEP_TIME_50_MS = 6,
    A_TMAG5273_SLEEP_TIME_100_MS = 7,
    A_TMAG5273_SLEEP_TIME_500_MS = 8,
    A_TMAG5273_SLEEP_TIME_1000_MS = 9,
    A_TMAG5273_SLEEP_TIME_2000_MS = 10,
    A_TMAG5273_SLEEP_TIME_5000_MS = 11,
    A_TMAG5273_SLEEP_TIME_20000_MS = 12
} aTmag5283SleepTime_t;

// The interrupt mode of a TMAG5273 device, see Table 8-11 of the data sheet.
typedef enum {
    A_TMAG5273_INT_MODE_NONE = 0,
    A_TMAG5273_INT_MODE_INT = 1,
    A_TMAG5273_INT_MODE_INT_EXCEPT_I2C_BUSY = 2,
    A_TMAG5273_INT_MODE_SCL = 3,
    A_TMAG5273_INT_MODE_SCL_EXCEPT_I2C_BUSY = 4
} aTmag5273IntMode_t;

// The hall effect sensor directions; these are hard
// code as they also reflect the index into the
// gTmag5273 array.
typedef enum {
    A_SENSOR_HALL_EFFECT_DIRECTION_LEFT = 0,
    A_SENSOR_HALL_EFFECT_DIRECTION_RIGHT = 1
} aSensorHallEffectDirection_t;

// Function signature of a hall effect sensor read callback.
typedef void (*aSensorHallEffectCallbackRead_t) (aSensorHallEffectDirection_t direction,
                                                 int16_t reading,
                                                 uint8_t conversionStatus,
                                                 void *pParameter);

// Structure defining a TMAG5273.
typedef struct {
    uint8_t i2cAddress;
    char *pNameStr;
    aSensorHallEffectDirection_t direction;
    i2c_master_dev_handle_t devHandle;
    gpio_num_t pinInt;
    SemaphoreHandle_t readTaskMutex;
    TaskHandle_t readTask;
    SemaphoreHandle_t readSemaphore;
    volatile bool readTaskAbort;
    aTmag5273ReadMode_t readMode;
    aSensorHallEffectCallbackRead_t pCallbackRead;
    void *pCallbackReadParameter;
} aTmag5273Device_t;

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

// Mutex to arbitrate I2C activity.
static SemaphoreHandle_t gI2cMutex = NULL;

// Storage for the TMAG5273 hall effect sensors.
static aTmag5273Device_t gTmag5273[] = {{.i2cAddress = A_I2C_ADDRESS_TMAG5273_SPARKFUN_LEFT,
                                         .pNameStr = "LEFT",
                                         .direction = A_SENSOR_HALL_EFFECT_DIRECTION_LEFT},
                                        {.i2cAddress = A_I2C_ADDRESS_TMAG5273_SPARKFUN_RIGHT,
                                         .pNameStr = "RIGHT",
                                         .direction = A_SENSOR_HALL_EFFECT_DIRECTION_RIGHT}};

// Keep track of whether the ISR service is already installed.
static bool gIsrServiceInstalled = false;

// How many times callbackRead() has been called.
static size_t gCallbackReadCount = 0;

// Storage for the conversion status of each reading, left-hand TMAG5273.
static uint8_t gConversionStatusLeft[1000];

// Storage for the conversion status of each reading, right-hand TMAG5273.
static uint8_t gConversionStatusRight[1000];

// How many entries have been added to gConversionStatusLeft.
static size_t gConversationStatusCountLeft = 0;

// How many entries have been added to gConversionStatusRight.
static size_t gConversationStatusCountRight = 0;

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: MISC
 * -------------------------------------------------------------- */

// Task block for the given number of milliseconds.
static void delayMs(size_t timeMs)
{
    vTaskDelay(timeMs / portTICK_PERIOD_MS);
}

// Wot it says.
static int64_t timeSinceBootMs()
{
    return esp_timer_get_time() / 1000;
}

// Set an MCU pin to be an output at the given level.
static esp_err_t pinOutputSet(gpio_num_t pin, int32_t level)
{
    esp_err_t espErr;
    gpio_config_t gpioConfig = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pin_bit_mask =  1ULL << pin
    };

    espErr = gpio_set_level(pin, level);
    if (espErr == ESP_OK) {
        espErr = gpio_config(&gpioConfig);
        if (espErr != ESP_OK) {
            printf("A_SENSOR_HALL_EFFECT: unable to configure pin %d as an output"
                   " (error 0x%02x)!\n", (int) pin, espErr);
        }
    } else {
        printf("A_SENSOR_HALL_EFFECT: unable to set pin %d to level %d"
               " (error 0x%02x)!\n", (int) pin, (int) level, espErr);
    }

    return espErr;
}

// Set an MCU pin as an interrupt source.
static esp_err_t pinIntSet(gpio_num_t pin)
{
    esp_err_t espErr;
    gpio_config_t gpioConfig = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pin_bit_mask =  1ULL << pin
    };

    espErr = gpio_config(&gpioConfig);
    if (espErr != ESP_OK) {
        printf("A_SENSOR_HALL_EFFECT: unable to configure pin %d as an"
               " interrupt source (error 0x%02x)!\n", (int) pin, espErr);
    }

    return espErr;
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: I2C OPERATIONS FOR TMAG5273
 * -------------------------------------------------------------- */

// Add a TMAG5273 to the I2C bus.
static esp_err_t i2cAddDevice(i2c_master_bus_handle_t busHandle,
                              uint8_t i2cAddress,
                              i2c_master_dev_handle_t *pDevHandle)
{
    esp_err_t espErr = ESP_ERR_INVALID_STATE;
    i2c_device_config_t i2cDeviceConfig = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2cAddress,
        .scl_speed_hz = A_I2C_SPEED_TMAG5273_HZ
    };

    if ((gI2cMutex != NULL) &&
        (xSemaphoreTake(gI2cMutex,
                        (TickType_t) portMAX_DELAY) == pdPASS)) {
        espErr = i2c_master_probe(busHandle, i2cAddress, -1);
        if (espErr == ESP_OK) {
            if (pDevHandle != NULL) {
                espErr = i2c_master_bus_add_device(busHandle, &i2cDeviceConfig, pDevHandle);
                if (espErr != ESP_OK) {
                    printf("A_SENSOR_HALL_EFFECT: unable to add TMAG5273 at address %d"
                           " as an I2C device (0x%02x)!\n", i2cAddress, espErr);
                }
            }
        } else {
            printf("A_SENSOR_HALL_EFFECT: unable to find a TMAG5273 at"
                   " I2C address 0x%02x (0x%02x)!\n", i2cAddress, espErr);
        }

        xSemaphoreGive(gI2cMutex);
    }

    return espErr;
}

// Write to a register of a TMAG5273 (with auto-increment).
static esp_err_t i2cWriteTmag5273(i2c_master_dev_handle_t devHandle, uint8_t reg,
                                  const uint8_t *pBuffer, size_t bufferLength)
{
    esp_err_t espErr = ESP_ERR_INVALID_STATE;
    uint8_t writeBuffer[16];
    size_t writeBufferLength = 0;

    if ((gI2cMutex != NULL) &&
        (xSemaphoreTake(gI2cMutex,
                        (TickType_t) portMAX_DELAY) == pdPASS)) {
        espErr = ESP_OK;

        writeBuffer[0] = reg;
        writeBufferLength++;
        if (pBuffer != NULL) {
            if (bufferLength > sizeof(writeBuffer) - writeBufferLength) {
                espErr = ESP_ERR_INVALID_SIZE;
            } else {
                memcpy(&(writeBuffer[writeBufferLength]), pBuffer, bufferLength);
                writeBufferLength += bufferLength;
            }
        }

        if (espErr == ESP_OK) {
            espErr = i2c_master_transmit(devHandle, writeBuffer, writeBufferLength, -1);
            if (espErr != ESP_OK) {
                printf("A_SENSOR_HALL_EFFECT: i2c_master_transmit() of %d byte(s) to"
                       " TMAG5273 register 0x%02x returned error 0x%02x!\n",
                       bufferLength, reg, espErr);
            }
        }

        xSemaphoreGive(gI2cMutex);
    }

    return espErr;
}

// Read 16-bit words from a TMAG5273; bufferLength is in bytes, not words.
// Before this is called the read mode (see tmag5273ReadModeSet()) must be set
// to A_TMAG5273_READ_MODE_1_BYTE_16_BIT and the channels that will be read must
// have been selected by setting MAG_CH_EN (in A_TMAG5273_REG_ADDRESS_SENSOR_CONFIG_1,
// see tmag5273MagneticChannelSet()) and, optionally, enabling T_CH_EN
// (in A_TMAG5273_REG_ADDRESS_T_CONFIG); this will dictate how many 16-bit words are
// returned (up to four), with a conversion status byte tacked on the end.
static esp_err_t i2cReadTmag5273Int16(i2c_master_dev_handle_t devHandle,
                                      int16_t *pBufferInt16, size_t bufferLength,
                                      uint8_t *pConversionStatus)
{
    esp_err_t espErr = ESP_ERR_INVALID_STATE;
    uint8_t buffer[4 * 2 + 1];  // Up to four 16-bit words plus a conversion status byte
    size_t readLength;

    if ((gI2cMutex != NULL) &&
        (xSemaphoreTake(gI2cMutex,
                        (TickType_t) portMAX_DELAY) == pdPASS)) {
        if (bufferLength > sizeof(buffer) - 1) {
           bufferLength = sizeof(buffer) - 1;
        }
        readLength = bufferLength + 1;

        espErr = i2c_master_receive(devHandle, buffer, readLength, -1);
        if (espErr == ESP_OK) {
            if (pBufferInt16 != NULL) {
                memcpy(pBufferInt16, buffer, bufferLength);
            }
            if (pConversionStatus != NULL) {
                *pConversionStatus = buffer[bufferLength];
            }
        } else {
            printf("A_SENSOR_HALL_EFFECT: i2c_master_receive() of %d byte(s) "
                   " returned error 0x%02x!\n", readLength, espErr);
        }

        xSemaphoreGive(gI2cMutex);
    }

    return espErr;
}

// Read the registers (with auto-increment) from a TMAG5273.  Before
// this is called the read mode (see tmag5273ReadModeSet()) must be set
// to A_TMAG5273_READ_MODE_STANDARD_3_BYTE.
static esp_err_t i2cReadTmag5273Reg(i2c_master_dev_handle_t devHandle,
                                    uint8_t regAddress, uint8_t *pBuffer,
                                    size_t bufferLength)
{
    esp_err_t espErr = ESP_ERR_INVALID_STATE;

    if ((gI2cMutex != NULL) &&
        (xSemaphoreTake(gI2cMutex,
                        (TickType_t) portMAX_DELAY) == pdPASS)) {

        espErr = i2c_master_transmit_receive(devHandle, &regAddress, 1,
                                             pBuffer, bufferLength, -1);
        if (espErr != ESP_OK) {
            printf("A_SENSOR_HALL_EFFECT: i2c_master_transmit_receive() of"
                   " %d byte(s) returned error 0x%02x!\n", bufferLength, espErr);
        }

        xSemaphoreGive(gI2cMutex);
    }

    return espErr;
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: TMAG5273 SETTINGS
 * -------------------------------------------------------------- */

// Clear the power on reset bit of a TMAG5273.
static esp_err_t tmag5273PowerOnResetClear(aTmag5273Device_t *pTmag5273Device)
{
    uint8_t writeBuffer[] = {0x10};  // Bit 4 is POR

    return i2cWriteTmag5273(pTmag5273Device->devHandle,
                            A_TMAG5273_REG_ADDRESS_CONV_STATUS,
                            writeBuffer, sizeof(writeBuffer));
}

// Set the read mode of a TMAG5273.
static esp_err_t tmag5273ReadModeSet(aTmag5273Device_t *pTmag5273Device,
                                     aTmag5273ReadMode_t readMode)
{
    esp_err_t espErr;
    uint8_t writeBuffer[] = {(A_TMAG5273_REG_CONTENTS_DEVICE_CONFIG_1_CRC_EN << 7)     |
                             (A_TMAG5273_REG_CONTENTS_DEVICE_CONFIG_1_MAG_TEMPCO << 5) |
                             (A_TMAG5273_REG_CONTENTS_DEVICE_CONFIG_1_CONV_AVG << 2)   |
                             readMode};

    espErr = i2cWriteTmag5273(pTmag5273Device->devHandle,
                              A_TMAG5273_REG_ADDRESS_DEVICE_CONFIG_1,
                              writeBuffer, sizeof(writeBuffer));
    if (espErr == ESP_OK) {
        pTmag5273Device->readMode = readMode;
    }

    return espErr;
}

// Set the magnetic channels enabled for the 1-byte read mode of a TMAG5273.
static esp_err_t tmag5273MagneticChannelSet(aTmag5273Device_t *pTmag5273Device,
                                            aTmag5273MagneticChannels_t magChEn)
{
    uint8_t writeBuffer[] = {(magChEn << 4) | A_TMAG5273_REG_CONTENTS_SENSOR_CONFIG_1_SLEEPTIME};

    return i2cWriteTmag5273(pTmag5273Device->devHandle,
                            A_TMAG5273_REG_ADDRESS_SENSOR_CONFIG_1,
                            writeBuffer, sizeof(writeBuffer));
}

// Set the operating mode of a TMAG5273.
static esp_err_t tmag5273OperatingModeSet(aTmag5273Device_t *pTmag5273Device,
                                          aTmag5273OperatingMode_t operatingMode)
{
    uint8_t writeBuffer[] = {(A_TMAG5273_REG_CONTENTS_DEVICE_CONFIG_2_THR_HYST << 5)          |
                             (A_TMAG5273_REG_CONTENTS_DEVICE_CONFIG_2_LP_LN << 4)             |
                             (A_TMAG5273_REG_CONTENTS_DEVICE_CONFIG_2_I2C_GLITCH_FILTER << 3) |
                             (A_TMAG5273_REG_CONTENTS_DEVICE_CONFIG_2_TRIGGER_MODE << 2)      |
                             operatingMode};

    return i2cWriteTmag5273(pTmag5273Device->devHandle,
                            A_TMAG5273_REG_ADDRESS_DEVICE_CONFIG_2,
                            writeBuffer, sizeof(writeBuffer));
}

// Set the interrupt mode of a TMAG5273.
static esp_err_t tmag5273IntModeSet(aTmag5273Device_t *pTmag5273Device,
                                    aTmag5273IntMode_t intMode)
{
    // The 1 in bit 7 means assert an interrupt when results are available
    uint8_t writeBuffer[] = {(0x01 << 7)                                            |
                             (A_TMAG5273_REG_CONTENTS_INT_CONFIG_1_THRSLD_INT << 6) |
                             (A_TMAG5273_REG_CONTENTS_INT_CONFIG_1_INT_STATE << 5)  |
                             (intMode << 2)                                         |
                             (A_TMAG5273_REG_CONTENTS_INT_CONFIG_1_RESERVED << 1)   |
                             A_TMAG5273_REG_CONTENTS_INT_CONFIG_1_MASK_INTB};

    return i2cWriteTmag5273(pTmag5273Device->devHandle,
                             A_TMAG5273_REG_ADDRESS_INT_CONFIG_1,
                            writeBuffer, sizeof(writeBuffer));
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: SENSOR, HALL EFFECT
 * -------------------------------------------------------------- */

// ISR handler function for the TMAG5273 hall effect sensor.
// pParameter must be a pointer to an aTmag5273Device_t structure.
static void isrHandler(void *pParameter)
{
    const aTmag5273Device_t *pTmag5273Device = (const aTmag5273Device_t *) pParameter;
    BaseType_t mustYield = false;

    // Give the semaphore to indicate that there is something
    // for readTask() to read
    xSemaphoreGiveFromISR(pTmag5273Device->readSemaphore, &mustYield);
    if (mustYield) {
        // FreeRTOS requires this
        portYIELD_FROM_ISR();
    }
}

// The read task for a TMAG5273 hall effect sensor.
// pParameter must be a pointer to an aTmag5273Device_t structure.
static void readTask(void *pParameter)
{
    const aTmag5273Device_t *pTmag5273Device = (const aTmag5273Device_t *) pParameter;
    int16_t bufferInt16;
    uint8_t conversionStatus;

    // Take the task mutex to signal that we are running
    if ((pTmag5273Device->readTaskMutex != NULL) &&
        (xSemaphoreTake(pTmag5273Device->readTaskMutex,
                        (TickType_t) portMAX_DELAY) == pdPASS)) {

        printf("A_SENSOR_HALL_EFFECT: started reading %s hall effect sensor.\n",
               pTmag5273Device->pNameStr);
        // While we haven't been asked to abort...
        while (!pTmag5273Device->readTaskAbort) {
            // See if the interrupt has signalled anything for us
            if (xSemaphoreTake(pTmag5273Device->readSemaphore,
                               100 / portTICK_PERIOD_MS) == pdTRUE) {
                if ((i2cReadTmag5273Int16(pTmag5273Device->devHandle,
                                          &bufferInt16, sizeof(bufferInt16),
                                          &conversionStatus) == ESP_OK) &&
                    (pTmag5273Device->pCallbackRead != NULL)) {
                    // Pass the data to the read callback
                    pTmag5273Device->pCallbackRead(pTmag5273Device->direction,
                                                   bufferInt16, conversionStatus,
                                                   pTmag5273Device->pCallbackReadParameter);
                }
            }
        }

        printf("A_SENSOR_HALL_EFFECT: stopped reading %s hall effect sensor.\n",
               pTmag5273Device->pNameStr);
        // Give the task mutex to indicate that we are no longer running
        xSemaphoreGive(pTmag5273Device->readTaskMutex);
    }

    // Delete ourself
    vTaskDelete(NULL);
}

// Stop a TMAG5273 read (interrupt, semaphore, task and associated mutex).
static void readStop(aTmag5273Device_t *pTmag5273Device)
{
    int64_t startTimeMs;

    if (pTmag5273Device->pinInt >= 0) {
        // Stop the interrupt
        gpio_isr_handler_remove(pTmag5273Device->pinInt);
        pTmag5273Device->pinInt = -1;
    }

    if (pTmag5273Device->readTask != NULL) {
        // Tell the read task to abort and wait for it to do so
        pTmag5273Device->readTaskAbort = true;
        if (pTmag5273Device->readTaskMutex != NULL) {
            // Now wait for the read task to exit in its own time
            startTimeMs = timeSinceBootMs();
            while ((timeSinceBootMs() < startTimeMs + A_READ_TASK_EXIT_WAIT_MS) &&
                   (xSemaphoreTake(pTmag5273Device->readTaskMutex,
                                   100 / portTICK_PERIOD_MS) != pdTRUE)) {
                // Waiting...
            }
            if (xSemaphoreGetMutexHolder(pTmag5273Device->readTaskMutex) == xTaskGetCurrentTaskHandle()) {
                // If we have the task mutex then the read task exited
                // by itself in an organised way: NULL the stored read
                // task handle so that we don't attempt to destroy it
                // again below
                pTmag5273Device->readTask = NULL;
            } else {
                printf("A_SENSOR_HALL_EFFECT: %s hall effect sensor read task"
                       " did not exit (waited about %lld ms)!\n",
                       pTmag5273Device->pNameStr, timeSinceBootMs() - startTimeMs);
            }
        }
    }

    // Whether the task exited or not, clean it up, along with the
    // mutex and semaphore
    if (pTmag5273Device->readTask != NULL) {
        vTaskDelete(pTmag5273Device->readTask);
        pTmag5273Device->readTask = NULL;
    }
    if (pTmag5273Device->readTaskMutex != NULL) {
        vSemaphoreDelete(pTmag5273Device->readTaskMutex);
        pTmag5273Device->readTaskMutex = NULL;
    }
    if (pTmag5273Device->readSemaphore != NULL) {
        vSemaphoreDelete(pTmag5273Device->readSemaphore);
        pTmag5273Device->readSemaphore = NULL;
    }

    // Let the idle task run so that the read task is tidied-up
    // by the idle task
    delayMs(100);

    // Reset the abort flag so that we can use the structure again
    pTmag5273Device->readTaskAbort = false;
    // Device back to standby
    tmag5273OperatingModeSet(pTmag5273Device,
                             A_TMAG5273_OPERATING_MODE_STANDBY_TRIGGERED);

    // Forget the read callback and its parameter
    pTmag5273Device->pCallbackRead = NULL;
    pTmag5273Device->pCallbackReadParameter = NULL;
}

// Initialise the I2C addresses of the hall effect sensors.
static esp_err_t aSensorHallEffectInit(i2c_master_bus_handle_t busHandle,
                                       gpio_num_t pinDisableLeft,
                                       gpio_num_t pinDisableRight)
{
    esp_err_t espErr = ESP_ERR_NO_MEM;
    uint8_t buffer[1];
    i2c_master_dev_handle_t devHandle;
    i2c_device_config_t i2cDeviceConfig = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = A_I2C_ADDRESS_TMAG5273_SPARKFUN_DEFAULT,
        .scl_speed_hz = A_I2C_SPEED_TMAG5273_HZ
    };

    // Create a mutex to arbitrate I2C activity
    if (gI2cMutex == NULL) {
        gI2cMutex = xSemaphoreCreateMutex();
        if (gI2cMutex != NULL) {
            espErr = ESP_OK;
        }
    }

    if (espErr == ESP_OK) {
        // Set both disables in order to reset both sensors
        pinOutputSet(pinDisableLeft, 1);
        pinOutputSet(pinDisableRight, 1);
        delayMs(10);
        // Enable just the right-hand hall effect sensor to begin with
        espErr = pinOutputSet(pinDisableRight, 0);
        if (espErr == ESP_OK) {
            printf("A_SENSOR_HALL_EFFECT: right-hand TMAG5273 hall effect sensor enabled.\n");
            // Disable the left-hand hall effect sensor while we check, and if necessary set,
            // the I2C address of the right-hand one
            espErr = pinOutputSet(pinDisableLeft, 1);
            if (espErr == ESP_OK) {
                delayMs(10);
                espErr = i2c_master_probe(busHandle, A_I2C_ADDRESS_TMAG5273_SPARKFUN_RIGHT, -1);
                if (espErr == ESP_OK) {
                    printf("A_SENSOR_HALL_EFFECT: found TMAG5273 already at I2C address 0x%02x.\n",
                            A_I2C_ADDRESS_TMAG5273_SPARKFUN_RIGHT);
                } else {
                    espErr = i2c_master_probe(busHandle, i2cDeviceConfig.device_address, -1);
                    if (espErr == ESP_OK) {
                        printf("A_SENSOR_HALL_EFFECT: found a TMAG5273 at I2C address 0x%02x,"
                               " assumed to be the right-hand hall effect sensor.\n",
                               i2cDeviceConfig.device_address);
                        espErr = i2c_master_bus_add_device(busHandle, &i2cDeviceConfig, &devHandle);
                        if (espErr == ESP_OK) {
                            printf("A_SENSOR_HALL_EFFECT: changing I2C address of this TMAG5273"
                                   " to 0x%02x...\n", A_I2C_ADDRESS_TMAG5273_SPARKFUN_RIGHT);
                            buffer[0] = ((A_I2C_ADDRESS_TMAG5273_SPARKFUN_RIGHT) << 1) | 0x01;
                            espErr = i2cWriteTmag5273(devHandle, A_TMAG5273_REG_ADDRESS_I2C_ADDRESS, buffer, 1);
                            if (espErr == ESP_OK) {
                                espErr = i2c_master_probe(busHandle, A_I2C_ADDRESS_TMAG5273_SPARKFUN_RIGHT, -1);
                                if (espErr != ESP_OK) {
                                    printf("A_SENSOR_HALL_EFFECT: unable to find a TMAG5273 at"
                                           " I2C address 0x%02x after I2C address change (0x%02x)!\n",
                                           A_I2C_ADDRESS_TMAG5273_SPARKFUN_RIGHT, espErr);
                                }
                            } else {
                                printf("A_SENSOR_HALL_EFFECT: unable to change I2C address of"
                                       " TMAG5273 (0x%02x)!\n", espErr);
                            }
                            i2c_master_bus_rm_device(devHandle);
                        }
                    } else {
                        printf("A_SENSOR_HALL_EFFECT: unable to find any TMAG5273 devices"
                               " (I2C address 0x%02x) (0x%02x)!\n", i2cDeviceConfig.device_address,
                               espErr);
                    }
                }

                // Should now have both TMAG5273 devices on different I2C addresses
                if (espErr == ESP_OK) {
                    espErr = pinOutputSet(pinDisableLeft, 0);
                    if (espErr == ESP_OK) {
                        printf("A_SENSOR_HALL_EFFECT: left-hand TMAG5273 hall effect sensor enabled.\n");
                        delayMs(10);
                    }
                }
            } else {
                printf("A_SENSOR_HALL_EFFECT: unable to disable the left-hand hall effect"
                       " sensor (disable pin %d) in order to configure the I2C address of"
                       " the right-hand one (0x%02x)!\n", pinDisableLeft, espErr);
            }
        } else {
            printf("A_SENSOR_HALL_EFFECT: unable to enable the right-hand hall effect"
                   " sensor (pin %d) (0x%02x)!\n", pinDisableRight, espErr);
        }
    }

    return espErr;
}

// Open the hall effect sensors.
static esp_err_t aSensorHallEffectOpen(i2c_master_bus_handle_t busHandle)
{
    esp_err_t espErr = ESP_OK;
    aTmag5273Device_t *pTmag5273Device;
    uint8_t buffer[3] = {0};
    uint8_t version;

    // Add and configure the devices
    for (size_t x = 0; (espErr == ESP_OK) && (x < sizeof(gTmag5273) / sizeof(gTmag5273[0])); x++) {
        pTmag5273Device = &(gTmag5273[x]);
        espErr = i2cAddDevice(busHandle, pTmag5273Device->i2cAddress,
                              &(pTmag5273Device->devHandle));
        if (espErr == ESP_OK) {
            // Read the device ID and manufacturer ID registers
            // into buffer[]
            tmag5273ReadModeSet(pTmag5273Device, A_TMAG5273_READ_MODE_STANDARD_3_BYTE);
            i2cReadTmag5273Reg(pTmag5273Device->devHandle,
                               A_TMAG5273_REG_ADDRESS_DEVICE_ID,
                               buffer, sizeof(buffer));
            // The version of the device is in bits 0 and 1 of the device ID
            // where 1 = ±40-mT and ±80-mT range and 2 = ±133-mT and ±266-mT range
            version = buffer[0] & 0x03;
            // Clear the power-on reset flag
            tmag5273PowerOnResetClear(pTmag5273Device);
            printf("A_SENSOR_HALL_EFFECT: %s hall effect sensor opened,"
                   " range %s, manufacturer ID 0x%04x.\n",
                   pTmag5273Device->pNameStr,
                   version == 1 ? "40 to 80 mT" : version == 2 ? "133 to 266 mT" : "unknown",
                   (((uint16_t) buffer[1]) << 8) +  buffer[2]);
        } else {
            printf("A_SENSOR_HALL_EFFECT: unable to add TMAG5273 %s at"
                   " I2C address 0x%02x (0x%02x)!", pTmag5273Device->pNameStr,
                   pTmag5273Device->i2cAddress, espErr);
        }
    }

    if (espErr != ESP_OK) {
        // Tidy-up on error
        for (size_t x = 0; x < sizeof(gTmag5273) / sizeof(gTmag5273[0]); x++) {
            pTmag5273Device = &(gTmag5273[x]);
            if (pTmag5273Device->devHandle != NULL) {
                i2c_master_bus_rm_device(pTmag5273Device->devHandle);
                pTmag5273Device->devHandle = NULL;
            }
        }
    }

    return espErr;
}

// Close the hall effect sensors.
static void aSensorHallEffectClose(i2c_master_bus_handle_t busHandle)
{
    aTmag5273Device_t *pTmag5273Device;

    for (size_t x = 0; x < sizeof(gTmag5273) / sizeof(gTmag5273[0]); x++) {
        pTmag5273Device = &(gTmag5273[x]);
        if (pTmag5273Device->devHandle != NULL) {
            i2c_master_bus_rm_device(pTmag5273Device->devHandle);
            pTmag5273Device->devHandle = NULL;
        }
    }
}

// Start reading the hall effect sensors.
static esp_err_t aSensorHallEffectReadStart(aSensorHallEffectCallbackRead_t pCallbackRead,
                                            void *pCallbackReadParameter,
                                            gpio_num_t pinIntLeft,
                                            gpio_num_t pinIntRight)
{
    esp_err_t espErr = ESP_OK;
    aTmag5273Device_t *pTmag5273Device;
    char buffer[16];

    for (size_t x = 0; (espErr == ESP_OK) && 
                       x < sizeof(gTmag5273) / sizeof(gTmag5273[0]); x++) {
        pTmag5273Device = &(gTmag5273[x]);
        // For each opened device, if there's no read task then start one
        if ((pTmag5273Device->devHandle != NULL) &&
            (pTmag5273Device->readTask == NULL)) {
            // Hook up the read callback and its parameter
            pTmag5273Device->pCallbackRead = pCallbackRead;
            pTmag5273Device->pCallbackReadParameter = pCallbackReadParameter;
            // In case things are already buzzing, switch off interrupt
            // mode in the TMAG5273 to begin with
            tmag5273IntModeSet(pTmag5273Device, A_TMAG5273_INT_MODE_NONE);
            if (pTmag5273Device->readSemaphore == NULL) {
                espErr = ESP_ERR_NO_MEM;
                // We need a semaphore so that the interrupt
                // can signal the task, and have it initially "taken"
                // (i.e. 0 available) in order that the read task won't
                // be triggered until the interrupt gives the semaphore
                pTmag5273Device->readSemaphore = xSemaphoreCreateCounting(1, 0);
                if (pTmag5273Device->readSemaphore != NULL) {
                    espErr = ESP_OK;
                }
            }
            if (espErr == ESP_OK) {
                // Store the interrupt pin
                pTmag5273Device->pinInt = -1;
                if (pTmag5273Device->direction == A_SENSOR_HALL_EFFECT_DIRECTION_LEFT) {
                    pTmag5273Device->pinInt = pinIntLeft;
                } else if (pTmag5273Device->direction == A_SENSOR_HALL_EFFECT_DIRECTION_RIGHT) {
                    pTmag5273Device->pinInt = pinIntRight;
                }
                if (pTmag5273Device->pinInt >= 0) {
                    if (!gIsrServiceInstalled) {
                        // Install the interrupt service
                        espErr = gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
                    }
                    if (espErr == ESP_OK) {
                        // Create the interrupt
                        gIsrServiceInstalled = true;
                        espErr = pinIntSet(pTmag5273Device->pinInt);
                        if (espErr == ESP_OK) {
                            espErr = gpio_isr_handler_add(pTmag5273Device->pinInt,
                                                          isrHandler,
                                                          pTmag5273Device);
                        }
                    }
                }
            }
            if ((espErr == ESP_OK) && (pTmag5273Device->readTaskMutex == NULL)) {
                espErr = ESP_ERR_NO_MEM;
                // A mutex for the read task, so that we can tell
                // whether it is running or not
                pTmag5273Device->readTaskMutex = xSemaphoreCreateMutex();
                if (pTmag5273Device->readTaskMutex != NULL) {
                    espErr = ESP_OK;
                }
            }
            if (espErr == ESP_OK) {
                espErr = ESP_ERR_NO_MEM;
                // Now create the task with a nice descriptive name
                // and a pointer to the TMAG5273 storage as parameter
                snprintf(buffer, sizeof(buffer), "hall read %d", x);
                if (xTaskCreate(readTask, buffer, A_READ_TASK_STACK_SIZE_BYTES,
                                pTmag5273Device, A_READ_TASK_PRIORITY,
                                &(pTmag5273Device->readTask)) == pdPASS) {
                    espErr = ESP_OK;
                }
            }
            if (espErr == ESP_OK) {
                // Configure the TMAG5273 to stream the relevant reading(s) to us
                if (espErr == ESP_OK) {
                    espErr = tmag5273MagneticChannelSet(pTmag5273Device,
                                                        A_TMAG5273_REG_CONTENTS_SENSOR_CONFIG_1_MAG_CH_EN);
                    if (espErr == ESP_OK) {
                        espErr = tmag5273ReadModeSet(pTmag5273Device,
                                                     A_TMAG5273_READ_MODE_1_BYTE_16_BIT);
                        if (espErr == ESP_OK) {
                            espErr = tmag5273OperatingModeSet(pTmag5273Device,
                                                              A_TMAG5273_OPERATING_MODE_CONTINUOUS);
                        }
                    }
                }
            }
            if ((espErr == ESP_OK) && (pTmag5273Device->pinInt >= 0)) {
                // Set the interrupt mode in the TMAG5273
                espErr = tmag5273IntModeSet(pTmag5273Device,
                                            A_TMAG5273_INT_MODE_INT_EXCEPT_I2C_BUSY);
            }
            if (espErr != ESP_OK) {
                // Clean up on error
                readStop(pTmag5273Device);
            }
        }
    }

    return espErr;
}

// Stop reading sensors.
static void aSensorHallEffectReadStop()
{
    aTmag5273Device_t *pTmag5273Device;

    for (size_t x = 0; x < sizeof(gTmag5273) / sizeof(gTmag5273[0]); x++) {
        pTmag5273Device = &(gTmag5273[x]);
        if (pTmag5273Device->devHandle != NULL) {
            readStop(pTmag5273Device);
        }
    }
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: APPLICATION
 * -------------------------------------------------------------- */

// Callback for readings
static void callbackRead(aSensorHallEffectDirection_t direction,
                         int16_t reading, uint8_t conversionStatus,
                         void *pParameter)
{
    (void) reading;
    (void) pParameter;

    gCallbackReadCount++;
    if (direction == 0) {
        if (gConversationStatusCountLeft < sizeof(gConversionStatusLeft) / sizeof(gConversionStatusLeft[0])) {
            gConversionStatusLeft[gConversationStatusCountLeft] = conversionStatus;
            gConversationStatusCountLeft++;
        }
    }
    if (direction == 1) {
        if (gConversationStatusCountRight < sizeof(gConversionStatusRight) / sizeof(gConversionStatusRight[0])) {
            gConversionStatusRight[gConversationStatusCountRight] = conversionStatus;
            gConversationStatusCountRight++;
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

    // Open the I2C bus
    espErr = i2c_new_master_bus(&gI2cMasterBusConfig, &busHandle);
    if (espErr == ESP_OK) {
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
                    delayMs(5000);
                    aSensorHallEffectReadStop();
                    printf("Callback called %d time(s), stored %d left readings,"
                           " %d right readings, here are the count values:\n",
                           gCallbackReadCount, gConversationStatusCountLeft,
                           gConversationStatusCountRight);
                    for (size_t x = 0; x < gConversationStatusCountLeft; x++) {
                        printf("%d: 0x%02x ", gConversionStatusLeft[x] >> 5, gConversionStatusLeft[x] & 0x03);
                    }
                    printf("\n");
                    for (size_t x = 0; x < gConversationStatusCountRight; x++) {
                        printf("%d: 0x%02x ", gConversionStatusRight[x] >> 5, gConversionStatusRight[x] & 0x03);
                    }
                    printf("\n");
                } else {
                    printf("Unable to start read of hall effect sensors (0x%02x)!\n", espErr);
                }
                aSensorHallEffectClose(busHandle);
                printf("Finished.\n");
            } else {
                printf("Unable to open hall effect sensors (0x%02x)!\n", espErr);
            }
        } else {
            printf("Unable to initialise hall effect sensors (0x%02x)!\n", espErr);
        }
        i2c_del_master_bus(busHandle);
    } else {
        printf("Unable to open I2C bus (0x%02x)!\n", espErr);
    }

    printf("If this is the ESP-IDF monitor program, press CTRL ] to terminate it.\n");
}

// End of file
