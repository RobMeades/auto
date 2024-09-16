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
// - Set up INT and read only when a conversion is ready
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

// Indexes of each of the TMAG5273 sensors in the array gTmag5273.
#define TMAG5273_LEFT  0
#define TMAG5273_RIGHT 1

#ifndef I2C_SPEED_TMAG5273_HZ
// I2C speed for the TMAG5273.
# define I2C_SPEED_TMAG5273_HZ 400000
#endif

// I2C addresses.
#define I2C_ADDRESS_TMAG5273_SPARKFUN_DEFAULT 0x22
#define I2C_ADDRESS_TMAG5273_SPARKFUN_LEFT I2C_ADDRESS_TMAG5273_SPARKFUN_DEFAULT
#define I2C_ADDRESS_TMAG5273_SPARKFUN_RIGHT (I2C_ADDRESS_TMAG5273_SPARKFUN_DEFAULT + 1)

// Register addresses in the TMAG5273 device, see
// https://www.ti.com/lit/ds/symlink/tmag5273.pdf
#define TMAG5273_REG_ADDRESS_DEVICE_CONFIG_1  0x00
#define TMAG5273_REG_ADDRESS_DEVICE_CONFIG_2  0x01
#define TMAG5273_REG_ADDRESS_SENSOR_CONFIG_1  0x02
#define TMAG5273_REG_ADDRESS_T_CONFIG         0x07
#define TMAG5273_REG_ADDRESS_INT_CONFIG_1     0x08
#define TMAG5273_REG_ADDRESS_I2C_ADDRESS      0x0c
#define TMAG5273_REG_ADDRESS_CONV_STATUS      0x18

// Static register contents for a TMAG5273 device, DEVICE_CONFIG_1.
#define TMAG5273_REG_CONTENTS_DEVICE_CONFIG_1_CRC_EN      0x00  // Bit 7
#define TMAG5273_REG_CONTENTS_DEVICE_CONFIG_1_MAG_TEMPCO  0x00  // Bits 6 and 5
#define TMAG5273_REG_CONTENTS_DEVICE_CONFIG_1_CONV_AVG    0x00  // Bits 4, 3 and 2

// Static register contents for a TMAG5273 device, DEVICE_CONFIG_2.
#define TMAG5273_REG_CONTENTS_DEVICE_CONFIG_2_THR_HYST          0x00  // Bits 7, 6 and 5
#define TMAG5273_REG_CONTENTS_DEVICE_CONFIG_2_LP_LN             0x00  // Bit 4
#define TMAG5273_REG_CONTENTS_DEVICE_CONFIG_2_I2C_GLITCH_FILTER 0x00  // Bit 3
#define TMAG5273_REG_CONTENTS_DEVICE_CONFIG_2_TRIGGER_MODE      0x00  // Bit 2

// Static register contents for a TMAG5273 device, SENSOR_CONFIG_1.
#define TMAG5273_REG_CONTENTS_SENSOR_CONFIG_1_SLEEPTIME   0x00  // Bits 3 to 0

// Static register contents for a TMAG5273 device, INT_CONFIG_1.
#define TMAG5273_REG_CONTENTS_INT_CONFIG_1_THRSLD_INT 0x00  // Bit 6
#define TMAG5273_REG_CONTENTS_INT_CONFIG_1_INT_STATE  0x00  // Bit 5
#define TMAG5273_REG_CONTENTS_INT_CONFIG_1_RESERVED   0x00  // Bit 1
#define TMAG5273_REG_CONTENTS_INT_CONFIG_1_MASK_INTB  0x00  // Bit 0

// MCU pins.
#define PIN_I2C_SCL                           7
#define PIN_I2C_SDA                           8
#define PIN_SENSOR_HALL_EFFECT_DISABLE_LEFT   9
#define PIN_SENSOR_HALL_EFFECT_DISABLE_RIGHT 10
#define PIN_SENSOR_HALL_EFFECT_INT_LEFT      11
#define PIN_SENSOR_HALL_EFFECT_INT_RIGHT     12

#ifndef READ_TASK_STACK_SIZE_BYTES
// Stack size for the hall effect sensor read task (in bytes).
# define READ_TASK_STACK_SIZE_BYTES (1024 * 2)
#endif

#ifndef READ_TASK_PRIORITY
// Priority of the hall effect sensor read task.
# define READ_TASK_PRIORITY 10
#endif

#ifndef READ_TASK_EXIT_WAIT_MS
// How long to wait for the hall effect sensor read task to exit.
# define READ_TASK_EXIT_WAIT_MS 1000
#endif

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

// The operating mode of a TMAG5273 device, see Table 8-4 of the data sheet.
typedef enum {
    TMAG5273_OPERATING_MODE_STANDBY_TRIGGERED = 0,
    TMAG5273_OPERATING_MODE_SLEEP = 1,
    TMAG5273_OPERATING_MODE_CONTINUOUS = 2,
    TMAG5273_OPERATING_MODE_WAKE_UP_AND_SLEEP = 3
} tmag5273OperatingMode_t;

// The read mode of a TMAG5273 device, see Table 8-3 of the data sheet.
typedef enum {
    TMAG5273_READ_MODE_STANDARD_3_BYTE = 0,
    TMAG5273_READ_MODE_1_BYTE_16_BIT = 1,
    TMAG5273_READ_MODE_1_BYTE_8_BIT = 2
} tmag5273ReadMode_t;

// The magnetic channel combinations for a 1-byte read, see Table 8-5
// of the data sheet.
typedef enum {
    TMAG5273_MAGNETIC_CHANNEL_NONE  = 0,  // the default, just 16-bit temperature if enabled
    TMAG5273_MAGNETIC_CHANNEL_X     = 1,  // 1 16-bit word, plus 16-bit temperature (first) if enabled
    TMAG5273_MAGNETIC_CHANNEL_Y     = 2,  // 1 16-bit word, plus 16-bit temperature (first) if enabled
    TMAG5273_MAGNETIC_CHANNEL_X_Y   = 3,  // 2 16-bit word, plus 16-bit temperature (first) if enabled
    TMAG5273_MAGNETIC_CHANNEL_Z     = 4,  // 1 16-bit word, plus 16-bit temperature (first) if enabled
    TMAG5273_MAGNETIC_CHANNEL_Z_X   = 5,  // 2 16-bit words, plus 16-bit temperature (first) if enabled
    TMAG5273_MAGNETIC_CHANNEL_Y_Z   = 6,  // 2 16-bit words, plus 16-bit temperature (first) if enabled
    TMAG5273_MAGNETIC_CHANNEL_X_Y_Z = 7,  // 3 16-bit words, plus 16-bit temperature (first) if enabled
    TMAG5273_MAGNETIC_CHANNEL_X_Y_X = 8,  // 3 16-bit words, plus 16-bit temperature (first) if enabled
    TMAG5273_MAGNETIC_CHANNEL_Y_X_Y = 9,  // 3 16-bit words, plus 16-bit temperature (first) if enabled
    TMAG5273_MAGNETIC_CHANNEL_Y_Z_Y = 10, // 3 16-bit words, plus 16-bit temperature (first) if enabled
    TMAG5273_MAGNETIC_CHANNEL_X_Z_X = 11  // 3 16-bit words, plus 16-bit temperature (first) if enabled
} tmag5273MagneticChannels_t;

// The interrupt mode of a TMAG5273 device, see Table 8-22 of the data sheet.
typedef enum {
    TMAG5273_INT_MODE_NONE = 0,
    TMAG5273_INT_MODE_INT = 1,
    TMAG5273_INT_MODE_INT_EXCEPT_I2C_BUSY = 2,
    TMAG5273_INT_MODE_SCL = 3,
    TMAG5273_INT_MODE_SCL_EXCEPT_I2C_BUSY = 4
} tmag5273IntMode_t;

// Structure defining a TMAG5273.
typedef struct {
    uint8_t i2cAddress;
    char *pNameStr;
    i2c_master_dev_handle_t devHandle;
    gpio_num_t pinInt;
    SemaphoreHandle_t readTaskMutex;
    TaskHandle_t readTask;
    SemaphoreHandle_t readSemaphore;
    volatile bool readTaskAbort;
    tmag5273ReadMode_t readMode;
} tmag5273Device_t;

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

// I2C bus configuration.
static const i2c_master_bus_config_t gI2cMasterBusConfig = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = 0,
    .scl_io_num = PIN_I2C_SCL,
    .sda_io_num = PIN_I2C_SDA,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = false
};

// Storage for the TMAG5273 hall effect sensors.
static tmag5273Device_t gTmag5273[] = {{.i2cAddress = I2C_ADDRESS_TMAG5273_SPARKFUN_LEFT,
                                        .pNameStr = "LEFT"},
                                       {.i2cAddress = I2C_ADDRESS_TMAG5273_SPARKFUN_RIGHT,
                                        .pNameStr = "RIGHT"}};

// Keep track of whether the ISR service is already installed.
static bool gIsrServiceInstalled = false;

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
            printf("Unable to configure pin %d as an output (error 0x%02x)!\n", (int) pin, espErr);
        }
    } else {
        printf("Unable to set pin %d to level %d (error 0x%02x)!\n", (int) pin, (int) level, espErr);
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
    esp_err_t espErr;
    i2c_device_config_t i2cDeviceConfig = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2cAddress,
        .scl_speed_hz = I2C_SPEED_TMAG5273_HZ
    };

    espErr = i2c_master_probe(busHandle, i2cAddress, -1);
    if (espErr == ESP_OK) {
        if (pDevHandle != NULL) {
            espErr = i2c_master_bus_add_device(busHandle, &i2cDeviceConfig, pDevHandle);
            if (espErr != ESP_OK) {
                printf("Unable to add TMAG5273 at address %d as an I2C device (0x%02x)!\n",
                       i2cAddress, espErr);
            }
        }
    } else {
        printf("Unable to find a TMAG5273 at I2C address 0x%02x (0x%02x)!\n", i2cAddress, espErr);
    }

    return espErr;
}

// Write to a register of a TMAG5273 (with auto-increment).
static esp_err_t i2cWriteTmag5273(i2c_master_dev_handle_t devHandle, uint8_t reg,
                                  const uint8_t *pBuffer, size_t bufferLength)
{
    esp_err_t espErr = ESP_OK;
    uint8_t writeBuffer[16];
    size_t writeBufferLength = 0;

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
            printf("i2c_master_transmit() of %d byte(s) to TMAG5273 register 0x%02x"
                   " returned error 0x%02x!\n", bufferLength, reg, espErr);
        }
    }

    return espErr;
}

// Read 16-bit words from a TMAG5273; bufferLength is in bytes, not words.
// Before this is called the read mode (see tmag5273ReadModeSet()) must be set
// to TMAG5273_READ_MODE_1_BYTE_16_BIT and the channels that will be read must
// have been selected by setting MAG_CH_EN (in TMAG5273_REG_ADDRESS_SENSOR_CONFIG_1,
// see tmag5273MagneticChannelSet()) and, optionally, enabling T_CH_EN
// (in TMAG5273_REG_ADDRESS_T_CONFIG); this will dictate how many 16-bit words are
// returned (up to four), with a conversion status byte tacked on the end.
static esp_err_t i2cReadTmag5273Int16(i2c_master_dev_handle_t devHandle,
                                      int16_t *pBufferInt16, size_t bufferLength,
                                      uint8_t *pConversionStatus)
{
    esp_err_t espErr = ESP_OK;
    uint8_t buffer[4 * 2 + 1];  // Up to four 16-bit words plus a conversion status byte
    size_t readLength;

   if (bufferLength > sizeof(buffer) - 1) {
       bufferLength = sizeof(buffer) - 1;
   }
    readLength = bufferLength + 1;

    espErr = i2c_master_receive(devHandle, buffer, readLength, -1);
    if (espErr == ESP_OK) {
        if (pBufferInt16 != NULL) {
            memcpy(pBufferInt16, buffer, bufferLength );
        }
        if (pConversionStatus != NULL) {
            *pConversionStatus = buffer[bufferLength];
        }
    } else {
        printf("i2c_master_receive() of %d byte(s)  returned error 0x%02x!\n",
               readLength, espErr);
    }

    return espErr;
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: TMAG5273 SETTINGS
 * -------------------------------------------------------------- */

// Clear the power on reset bit of a TMAG5273.
static esp_err_t tmag5273PowerOnResetClear(i2c_master_dev_handle_t devHandle)
{
    uint8_t writeBuffer[] = {0x10};  // Bit 4 is POR

    return i2cWriteTmag5273(devHandle, TMAG5273_REG_ADDRESS_CONV_STATUS,
                            writeBuffer, sizeof(writeBuffer));
}

// Set the read mode of a TMAG5273.
static esp_err_t tmag5273ReadModeSet(tmag5273Device_t *pTmag5273,
                                     tmag5273ReadMode_t mode)
{
    esp_err_t espErr;
    uint8_t writeBuffer[] = {(TMAG5273_REG_CONTENTS_DEVICE_CONFIG_1_CRC_EN << 7)     |
                             (TMAG5273_REG_CONTENTS_DEVICE_CONFIG_1_MAG_TEMPCO << 5) |
                             (TMAG5273_REG_CONTENTS_DEVICE_CONFIG_1_CONV_AVG << 2)   |
                             (mode & 0x03)};

    espErr = i2cWriteTmag5273(pTmag5273->devHandle, TMAG5273_REG_ADDRESS_DEVICE_CONFIG_1,
                              writeBuffer, sizeof(writeBuffer));
    if (espErr == ESP_OK) {
        pTmag5273->readMode = mode;
    }
    return espErr;
}

// Set the magnetic channels enabled for the 1-byte read mode of a TMAG5273.
static esp_err_t tmag5273MagneticChannelSet(i2c_master_dev_handle_t devHandle,
                                            tmag5273MagneticChannels_t channels)
{
    uint8_t writeBuffer[] = {((channels & 0x0F) << 4) |
                             (TMAG5273_REG_CONTENTS_SENSOR_CONFIG_1_SLEEPTIME)};

    return i2cWriteTmag5273(devHandle, TMAG5273_REG_ADDRESS_SENSOR_CONFIG_1,
                            writeBuffer, sizeof(writeBuffer));
}

// Set the operating mode of a TMAG5273.
static esp_err_t tmag5273OperatingModeSet(i2c_master_dev_handle_t devHandle,
                                          tmag5273OperatingMode_t mode)
{
    uint8_t writeBuffer[] = {(TMAG5273_REG_CONTENTS_DEVICE_CONFIG_2_THR_HYST << 5)          |
                             (TMAG5273_REG_CONTENTS_DEVICE_CONFIG_2_LP_LN << 4)             |
                             (TMAG5273_REG_CONTENTS_DEVICE_CONFIG_2_I2C_GLITCH_FILTER << 3) |
                             (TMAG5273_REG_CONTENTS_DEVICE_CONFIG_2_TRIGGER_MODE << 2)      |
                             (mode & 0x02)};

    return i2cWriteTmag5273(devHandle, TMAG5273_REG_ADDRESS_DEVICE_CONFIG_2,
                            writeBuffer, sizeof(writeBuffer));
}

// Set the interrupt mode of a TMAG5273.
static esp_err_t tmag5273IntModeSet(i2c_master_dev_handle_t devHandle,
                                    tmag5273IntMode_t mode)
{
    // The 1 in bit 7 means assert an interrupt when results are available
    uint8_t writeBuffer[] = {(0x01 << 7)                                           |
                             (TMAG5273_REG_CONTENTS_INT_CONFIG_1_THRSLD_INT << 6)  |
                             (TMAG5273_REG_CONTENTS_INT_CONFIG_1_INT_STATE << 5)   |
                             ((mode & 0x07) << 2)                                  |
                             (TMAG5273_REG_CONTENTS_INT_CONFIG_1_RESERVED << 1)    |
                             TMAG5273_REG_CONTENTS_INT_CONFIG_1_MASK_INTB};

    return i2cWriteTmag5273(devHandle, TMAG5273_REG_ADDRESS_INT_CONFIG_1,
                            writeBuffer, sizeof(writeBuffer));
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: SENSOR, HALL EFFECT
 * -------------------------------------------------------------- */

// ISR handler function for the TMAG5273 hall effect sensor.
// pParameter must be a pointer to a tmag5273Device_t structure.
static void isrHandler(void *pParameter)
{
    const tmag5273Device_t *pTmag5273Device = (const tmag5273Device_t *) pParameter;
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
// pParameter must be a pointer to a tmag5273Device_t structure.
static void readTask(void *pParameter)
{
    const tmag5273Device_t *pTmag5273Device = (const tmag5273Device_t *) pParameter;

    // Take the task mutex to signal that we are running
    if ((pTmag5273Device->readTaskMutex != NULL) &&
        (xSemaphoreTake(pTmag5273Device->readTaskMutex,
                        (TickType_t) portMAX_DELAY) == pdPASS)) {

        printf("Started reading %s hall effect sensor.\n", pTmag5273Device->pNameStr);
        // While we haven't been asked to abort...
        while (!pTmag5273Device->readTaskAbort) {
            // See if the interrupt has signalled anything for us
            if (xSemaphoreTake(pTmag5273Device->readSemaphore,
                               100 / portTICK_PERIOD_MS) == pdTRUE) {
                // There is data: read it
                // TODO: put it somewhere
                i2cReadTmag5273Int16(pTmag5273Device->devHandle, NULL, 0, NULL);
            }
        }

        printf("Stopped reading %s hall effect sensor.\n", pTmag5273Device->pNameStr);
        // Give the task mutex to indicate that we are no longer running
        xSemaphoreGive(pTmag5273Device->readTaskMutex);
    }

    // Delete ourself
    vTaskDelete(NULL);
}

// Stop a TMAG5273 read (interrupt, semaphore, task and associated mutex).
static void readStop(tmag5273Device_t *pTmag5273Device)
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
            while ((timeSinceBootMs() < startTimeMs + READ_TASK_EXIT_WAIT_MS) &&
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
                printf("%s hall effect sensor read task did not exit "
                       " (waited about %lld ms)!\n", pTmag5273Device->pNameStr,
                       timeSinceBootMs() - startTimeMs);
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
    // Device goes sleepy, hopefully
    tmag5273OperatingModeSet(pTmag5273Device->devHandle, TMAG5273_OPERATING_MODE_SLEEP);
}

// Initialise the I2C addresses of the hall effect sensors.
static esp_err_t sensorHallEffectInit(i2c_master_bus_handle_t busHandle,
                                      gpio_num_t pinDisableLeft,
                                      gpio_num_t pinDisableRight)
{
    esp_err_t espErr;
    uint8_t buffer[1];
    i2c_master_dev_handle_t devHandle;
    i2c_device_config_t i2cDeviceConfig = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_ADDRESS_TMAG5273_SPARKFUN_DEFAULT,
        .scl_speed_hz = I2C_SPEED_TMAG5273_HZ
    };

    // Enable just the right-hand hall effect sensor to begin with
    espErr = pinOutputSet(pinDisableRight, 0);
    if (espErr == ESP_OK) {
        printf("Right-hand TMAG5273 hall effect sensor enabled.\n");
        // Disable the left-hand hall effect sensor while we check, and if necessary set,
        // the I2C address of the right-hand one
        espErr = pinOutputSet(pinDisableLeft, 1);
        if (espErr == ESP_OK) {
            delayMs(10);
            espErr = i2c_master_probe(busHandle, I2C_ADDRESS_TMAG5273_SPARKFUN_RIGHT, -1);
            if (espErr == ESP_OK) {
                printf("Found TMAG5273 already at I2C address 0x%02x.\n",
                        I2C_ADDRESS_TMAG5273_SPARKFUN_RIGHT);
            } else {
                espErr = i2c_master_probe(busHandle, i2cDeviceConfig.device_address, -1);
                if (espErr == ESP_OK) {
                    printf("Found a TMAG5273 at I2C address 0x%02x, assumed to be the"
                           " right-hand hall effect sensor.\n", i2cDeviceConfig.device_address);
                    espErr = i2c_master_bus_add_device(busHandle, &i2cDeviceConfig, &devHandle);
                    if (espErr == ESP_OK) {
                        printf("Changing I2C address of this TMAG5273 to 0x%02x...\n",
                               I2C_ADDRESS_TMAG5273_SPARKFUN_RIGHT);
                        buffer[0] = ((I2C_ADDRESS_TMAG5273_SPARKFUN_RIGHT) << 1) | 0x01;
                        espErr = i2cWriteTmag5273(devHandle, TMAG5273_REG_ADDRESS_I2C_ADDRESS, buffer, 1);
                        if (espErr == ESP_OK) {
                            espErr = i2c_master_probe(busHandle, I2C_ADDRESS_TMAG5273_SPARKFUN_RIGHT, -1);
                            if (espErr != ESP_OK) {
                                printf("Unable to find a TMAG5273 at I2C address 0x%02x after"
                                       " I2C address change (0x%02x)!\n",
                                       I2C_ADDRESS_TMAG5273_SPARKFUN_RIGHT, espErr);
                            }
                        } else {
                            printf("Unable to change I2C address of TMAG5273 (0x%02x)!\n", espErr);
                        }
                        i2c_master_bus_rm_device(devHandle);
                    }
                } else {
                    printf("Unable to find any TMAG5273 devices (I2C address 0x%02x) (0x%02x)!\n",
                           i2cDeviceConfig.device_address, espErr);
                }
            }

            // Should now have both TMAG5273 devices on different I2C addresses
            if (espErr == ESP_OK) {
                espErr = pinOutputSet(pinDisableLeft, 0);
                if (espErr == ESP_OK) {
                    printf("Left-hand TMAG5273 hall effect sensor enabled.\n");
                    delayMs(10);
                }
            }
        } else {
            printf("Unable to disable the left-hand hall effect sensor (disable pin %d)"
                   " in order to configure the I2C address of the right-hand one (0x%02x)!\n",
                   pinDisableLeft, espErr);
        }
    } else {
        printf("Unable to enable the right-hand hall effect sensor (pin %d) (0x%02x)!\n",
               pinDisableRight, espErr);
    }

    return espErr;
}

// Open the hall effect sensors.
static esp_err_t sensorHallEffectOpen(i2c_master_bus_handle_t busHandle)
{
    esp_err_t espErr = ESP_OK;
    tmag5273Device_t *pTmag5273Device;

    // Add and configure the devices
    for (size_t x = 0; (espErr == ESP_OK) && (x < sizeof(gTmag5273) / sizeof(gTmag5273[0])); x++) {
        pTmag5273Device = &(gTmag5273[x]);
        espErr = i2cAddDevice(busHandle, pTmag5273Device->i2cAddress, &(pTmag5273Device->devHandle));
        if (espErr == ESP_OK) {
            tmag5273PowerOnResetClear(pTmag5273Device->devHandle);
            printf("%s hall effect sensor opened.\n", pTmag5273Device->pNameStr);
        } else {
            printf("Unable to add TMAG5273 %s at I2C address 0x%02x (0x%02x)!",
                   pTmag5273Device->pNameStr, pTmag5273Device->i2cAddress, espErr);
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
static void sensorHallEffectClose(i2c_master_bus_handle_t busHandle)
{
    tmag5273Device_t *pTmag5273Device;

    for (size_t x = 0; x < sizeof(gTmag5273) / sizeof(gTmag5273[0]); x++) {
        pTmag5273Device = &(gTmag5273[x]);
        if (pTmag5273Device->devHandle != NULL) {
            i2c_master_bus_rm_device(pTmag5273Device->devHandle);
            pTmag5273Device->devHandle = NULL;
        }
    }
}

// Start reading the hall effect sensors.
static esp_err_t sensorHallEffectReadStart(gpio_num_t pinIntLeft,
                                           gpio_num_t pinIntRight)
{
    esp_err_t espErr = ESP_OK;
    tmag5273Device_t *pTmag5273Device;
    char buffer[16];

    for (size_t x = 0; (espErr == ESP_OK) && 
                       x < sizeof(gTmag5273) / sizeof(gTmag5273[0]); x++) {
        pTmag5273Device = &(gTmag5273[x]);
        // For each opened device, if there's no read task then start one
        if ((pTmag5273Device->devHandle != NULL) &&
            (pTmag5273Device->readTask == NULL)) {
            if (pTmag5273Device->readSemaphore == NULL) {
                espErr = ESP_ERR_NO_MEM;
                // We need a semaphore so that the interrupt
                // can signal the task
                pTmag5273Device->readSemaphore = xSemaphoreCreateCounting(1, 1);
                if (pTmag5273Device->readSemaphore != NULL) {
                    espErr = ESP_OK;
                }
            }
            if (espErr == ESP_OK) {
                // Store the interrupt pin
                pTmag5273Device->pinInt = -1;
                if (x == TMAG5273_LEFT) {
                    pTmag5273Device->pinInt = pinIntLeft;
                } else if (x == TMAG5273_RIGHT) {
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
                        espErr = gpio_isr_handler_add(pTmag5273Device->pinInt,
                                                      isrHandler,
                                                      pTmag5273Device);
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
                if (xTaskCreate(readTask, buffer, READ_TASK_STACK_SIZE_BYTES,
                                pTmag5273Device, READ_TASK_PRIORITY,
                                &(pTmag5273Device->readTask)) == pdPASS) {
                    espErr = ESP_OK;
                }
            }
            if ((espErr == ESP_OK) && (pTmag5273Device->pinInt >= 0)) {
                // Set the interrupt mode in the TMAG5273
                espErr = tmag5273IntModeSet(pTmag5273Device->devHandle,
                                            TMAG5273_INT_MODE_INT_EXCEPT_I2C_BUSY);
            }
            if (espErr == ESP_OK) {
                // Configure the TMAG5273 to stream readings at us
                espErr = tmag5273MagneticChannelSet(pTmag5273Device->devHandle,
                                                    TMAG5273_MAGNETIC_CHANNEL_X_Y_Z);
                if (espErr == ESP_OK) {
                    espErr = tmag5273ReadModeSet(pTmag5273Device,
                                                 TMAG5273_READ_MODE_1_BYTE_16_BIT);
                    if (espErr == ESP_OK) {
                        espErr = tmag5273OperatingModeSet(pTmag5273Device->devHandle,
                                                          TMAG5273_OPERATING_MODE_CONTINUOUS);
                    }
                }
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
static void sensorHallEffectReadStop()
{
    tmag5273Device_t *pTmag5273Device;

    for (size_t x = 0; x < sizeof(gTmag5273) / sizeof(gTmag5273[0]); x++) {
        pTmag5273Device = &(gTmag5273[x]);
        if (pTmag5273Device->devHandle != NULL) {
            readStop(pTmag5273Device);
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
    int16_t bufferInt16[4];
    uint8_t conversionStatus;

    // Open the I2C bus
    espErr = i2c_new_master_bus(&gI2cMasterBusConfig, &busHandle);
    if (espErr == ESP_OK) {
        // Initialise the hall effect stuff
        espErr = sensorHallEffectInit(busHandle, PIN_SENSOR_HALL_EFFECT_DISABLE_LEFT,
                                      PIN_SENSOR_HALL_EFFECT_DISABLE_RIGHT);
        if (espErr == ESP_OK) {
            // Open the hall effect stuff
            espErr = sensorHallEffectOpen(busHandle);
            if (espErr == ESP_OK) {
                // Start the hall effect stuff reading
                espErr = sensorHallEffectReadStart(PIN_SENSOR_HALL_EFFECT_INT_LEFT,
                                                   PIN_SENSOR_HALL_EFFECT_INT_RIGHT);
                if (espErr == ESP_OK) {
                    // Read the devices in a loop
                    printf("Reading sensors.\n");
                    for (size_t x = 0; (espErr == ESP_OK) && (x < 100); x++) {
                        for (size_t y = 0; (espErr == ESP_OK) && (y < sizeof(gTmag5273) / sizeof(gTmag5273[0])); y++) {
                            espErr = i2cReadTmag5273Int16(gTmag5273[y].devHandle, bufferInt16, 6,
                                                          &conversionStatus);
                            if (espErr == ESP_OK) {
                                printf("%s: X %d, Y %d, Z %d, status 0x%02x.\n",
                                       gTmag5273[y].pNameStr,
                                       bufferInt16[0], bufferInt16[1], bufferInt16[2], conversionStatus);
                            } else {
                                printf("Unable to perform a 1 byte, 16 bit read from TMAG5273 %s (0x%02x)!\n",
                                       gTmag5273[y].pNameStr, espErr);
                            }
                        }
                    }
                    sensorHallEffectReadStop();
                    sensorHallEffectClose(busHandle);
                    printf("Finished.\n");
                } else {
                    printf("Unable to start read of hall effect sensors (0x%02x)!\n", espErr);
                }
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
