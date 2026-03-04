// SPDX-License-Identifier: GPL-3.0
#ifndef __MAX30102RPI_H
#define __MAX30102RPI_H

/**
 * Copyright (C) 2024 Bernd Porr <bernd@glasgowneuro.com>
 * Copyright (C) 2024 Mark Craig <MarkCraig@4cdesign.co.uk>
 * Copyright (C) 2017 Matt Ranostay <matt.ranostay@konsulko.com>
 * Copyright (C) 2017 Peter Meerwald-Stadler <pmeerw@pmeerw.net>
 **/

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <assert.h>
#include <thread>
#include <vector>

#include <gpiod.h>

// enable debug messages and error messages to stderr
#ifndef NDEBUG
#define DEBUG
#endif

static const char could_not_open_i2c[] = "Could not open I2C.\n";

#define ISR_TIMEOUT 1 // sec

// default address if ADDR is pulled to GND
#define DEFAULT_MAX30102_ADDRESS 0x57

// default GPIO pin for the ALRT/DRY signal
#define DEFAULT_ALERT_RDY_TO_GPIO 27

constexpr uint8_t REG_INTR_STATUS_1 = 0x00;
constexpr uint8_t REG_INTR_STATUS_2 = 0x01;
constexpr uint8_t REG_INTR_ENABLE_1 = 0x02;
constexpr uint8_t REG_INTR_ENABLE_2 = 0x03;
constexpr uint8_t REG_FIFO_WR_PTR = 0x04;
constexpr uint8_t REG_OVF_COUNTER = 0x05;
constexpr uint8_t REG_FIFO_RD_PTR = 0x06;
constexpr uint8_t REG_FIFO_DATA = 0x07;
constexpr uint8_t REG_FIFO_CONFIG = 0x08;
constexpr uint8_t REG_MODE_CONFIG = 0x09;
constexpr uint8_t REG_SPO2_CONFIG = 0x0A;
constexpr uint8_t REG_LED1_PA = 0x0C;
constexpr uint8_t REG_LED2_PA = 0x0D;
constexpr uint8_t REG_PILOT_PA = 0x10;
constexpr uint8_t REG_MULTI_LED_CTRL1 = 0x11;
constexpr uint8_t REG_MULTI_LED_CTRL2 = 0x12;
constexpr uint8_t REG_TEMP_INTR = 0x1F;
constexpr uint8_t REG_TEMP_FRAC = 0x20;
constexpr uint8_t REG_TEMP_CONFIG = 0x21;
constexpr uint8_t REG_PROX_INT_THRESH = 0x30;
constexpr uint8_t REG_REV_ID = 0xFE;
constexpr uint8_t REG_PART_ID = 0xFF;

constexpr int FIFOSIZE = 32;

struct MAX30102settings
{

    /**
     * I2C bus used (99% always set to one)
     **/
    int i2c_bus = 1;

    /**
     * I2C address of the max30102
     **/
    uint8_t address = DEFAULT_MAX30102_ADDRESS;

    /**
     * If set to true the pigpio will be initialised
     **/
    bool initPIGPIO = true;

    /**
     * GPIO pin connected to ALERT/RDY
     **/
    int drdy_gpio = DEFAULT_ALERT_RDY_TO_GPIO;

    /**
     * Chip which has the GPIO pin for ALERT/RDY
     **/
    int drdy_chip = 0;

    /**
     * Possible sampling rates
     **/
    enum SamplingRates
    {
        FS50HZ = 0,
        FS100HZ = 1,
        FS200HZ = 2,
        FS400HZ = 3,
        FS800HZ = 4,
    };

    /**
     * Sampling rate. Default is 100Hz.
     **/
    SamplingRates samplingRate = FS100HZ;

    /**
     * Get sampling rate in Hz
     **/
    inline unsigned getSamplingRateHz() const
    {
        const unsigned SamplingRateEnum2Value[5] =
            {50, 100, 200, 400, 800};
        return SamplingRateEnum2Value[samplingRate];
    }

    /**
     * LED currents
     **/
    enum LEDCurrents
    {
        LED_OFF = 0x00,
        LED_3MA = 0x0F,
        LED_6MA = 0x1F,
        LED_13MA = 0x3F,
        LED_25MA = 0x7F,
        LED_51MA = 0xFF
    };

    /**
     * Vis LED current
     **/
    LEDCurrents currentRed = LED_13MA;

    /**
     * IR LED current
     **/
    LEDCurrents currentIR = LED_13MA;
};

/**
 * This class reads data from the MAX30102 in the background (separate
 * thread) and calls a callback function whenever data is available.
 **/
class MAX30102rpi
{

public:
    /**
     * Defines a sample delivered by the callback below.
     * It's the value of the red channel and the infrared one.
     **/
    struct Sample
    {
        float red = 0;
        float ir = 0;
    };

public:
    /**
     * Destructor which makes sure the data acquisition
     * stops on exit.
     **/
    ~MAX30102rpi()
    {
        stop();
    }

    /**
     * Callback interface class which contains the
     * method which is called whenever a new sample
     * has arrived. hasSample is abstract and needs
     * to be implemented by the receiver. It's usually
     * inherited into the receiving class.
     **/
    struct Callback
    {
        /**
         * Called when a new sample is available.
         * This needs to be implemented in a derived
         * class by the client. Defined as abstract.
         * \param sample Voltage from the selected channel.
         **/
        virtual void hasSample(Sample sample) = 0;
    };

    /**
     * Starts the data acquisition in the background and the
     * callback is called with new samples.
     * \param settings A struct with the settings.
     **/
    void start(MAX30102settings settings = MAX30102settings());

    /**
     * Returns the current settings
     **/
    MAX30102settings getMAX30102settings() const
    {
        return max30102settings;
    }

    /**
     * Stops the data acquistion
     **/
    void stop();

    /**
     * Registers the callbacks which want to receive
     * the IR/VIS data samples at the requested sampling rate.
     * These can be more than one.
     **/
    void registerCallback(Callback *cb)
    {
        callbacks.push_back(cb);
    }

private:
    // Worker of the data acquisition thead which wakes up
    // after an interrupt has been triggered.
    void dataWorker();

    // Called after an interrupt has been triggered from
    // dataWorker().
    void dataReady();

    // All settings
    MAX30102settings max30102settings;

    // Writes a byte to the register of the max30102
    void i2c_writeReg(uint8_t reg, uint8_t data);

    // Reads a byte from a register of the max30102
    uint8_t i2c_readReg(uint8_t reg);

    // Resets the max30102 chip
    void reset();

    // Flag in the dataWorker() thread if we continue
    // receiving data in dataWorker().
    bool running = false;

    // Name of the gpiochip which has the interrupt GPIO
    // pin. This is true for Raspberry Pi 1-4. Not 5.
    const char *chipname = "gpiochip0";

    // Pointer to the chip (via libgpiod)
    struct gpiod_chip *chip;

    // Pointer to the IO pin on that chip
    struct gpiod_line *lineIRQ;

    // Timeout if no interrupt has happened.
    const struct timespec ts = {ISR_TIMEOUT, 0};

    // Thread which hosts the worker.
    std::thread thr;

    // File descriptor of the I2C device.
    int fd = -1;

    // List of callbacks which have been registered.
    std::vector<Callback *> callbacks;
};

#endif
