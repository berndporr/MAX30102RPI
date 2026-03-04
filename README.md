# PulseOxy driver for the MAX30102 on the Raspberry PI

## Hardware

Connect a MAX30102 to the Raspberry PI:

 - 3.3V to 3.3V (RPI pin 1)
 - GND to GND (RPI pin 9)
 - SCL to SCL (RPI pin 5)
 - SDA to SDA (RPI pin 3)
 - /INT to GPIO 27 (RPI pin 13)

## Prerequisites

This code has been written for and tested under debian `bookworm`.

Install the packages:

```
apt-get install cmake
apt-get install libgpiod-dev
```

## Building:

To build:

```
cmake .
make
```

## Install

```
sudo make install
```

## C++ library

This directory contains the C++ class which provides callbacks for the
raw pulseoxy samples, finger on/off and heartrate.

### Low level driver class
The header `max30102rpi.h` and its class `MAX30102rpi` manages the
communication between the MAX30102 and the client who just needs to
register the callback `hasSample(Sample sample)` which is then called
at the requested sampling rate.

The class uses the `/INT` pin of the MAX30102 connected to GPIO 27
to establish the sampling rate. This pin fires whenever a new
pulseoxy reading is available.

## Further reading

https://www.analog.com/media/en/technical-documentation/user-guides/max3010x-ev-kits-recommended-configurations-and-operating-profiles.pdf

## Authors: 

 - (c) 2024, Bernd Porr, bernd@glasgowneuro.com
 - (c) 2024, Mark Craig, MarkCraig@4cdesign.co.uk

Based on the MAX30102 C driver by Analog Devices: https://github.com/analogdevicesinc/linux/blob/main/drivers/iio/health/max30102.c (GPLv2 or later).
