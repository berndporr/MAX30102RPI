#include "max30102rpi.h"

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdexcept>

void MAX30102rpi::start(MAX30102settings settings)
{
	max30102settings = settings;

	char filename[20];
	snprintf(filename, 19, "/dev/i2c-%d", settings.i2c_bus);
	fd = open(filename, O_RDWR);
	if (fd < 0)
	{
		throw std::invalid_argument("Could not open I2C.");
	}

	if (ioctl(fd, I2C_SLAVE, settings.address) < 0)
	{
		throw std::invalid_argument("Could not access I2C slave address: " + settings.address);
	}

#ifdef DEBUG
	fprintf(stderr, "Init .\n");
#endif

	chip = gpiod_chip_open_by_number(settings.drdy_chip);
	lineIRQ = gpiod_chip_get_line(chip, settings.drdy_gpio);

	int ret = gpiod_line_request_input(lineIRQ, "DummyRq");
	if (ret < 0)
	{
		perror("Request event notification failed\n");
		throw std::invalid_argument("Could not request IRQ");
	}
	gpiod_line_release(lineIRQ);

	ret = gpiod_line_request_falling_edge_events(lineIRQ, "Consumer");
	if (ret < 0)
	{
		perror("Request event notification failed\n");
		throw std::invalid_argument("Could not request IRQ");
	}

	reset();

	i2c_writeReg(REG_INTR_ENABLE_1, 0xc0);
	i2c_writeReg(REG_INTR_ENABLE_2, 0x00);
	i2c_writeReg(REG_FIFO_WR_PTR, 0x00);
	i2c_writeReg(REG_OVF_COUNTER, 0x00);
	i2c_writeReg(REG_FIFO_RD_PTR, 0x00);
	i2c_writeReg(REG_FIFO_CONFIG, 0x0f); // no averaging
	i2c_writeReg(REG_MODE_CONFIG, 0x03); // SpO2 mode
	uint8_t spo2 = 0x63 | (uint8_t)((settings.samplingRate) << 2);
	i2c_writeReg(REG_SPO2_CONFIG, spo2);
	i2c_writeReg(REG_LED1_PA, settings.currentRed);
	i2c_writeReg(REG_LED2_PA, settings.currentIR);
	i2c_writeReg(REG_PILOT_PA, 0x7f);

#ifdef DEBUG
	uint8_t a = i2c_readReg(REG_REV_ID);
	uint8_t b = i2c_readReg(REG_PART_ID);
	fprintf(stderr, "ID = %02x %02x\n", a, b);
#endif

	i2c_readReg(REG_INTR_STATUS_1);
	i2c_readReg(REG_INTR_STATUS_2);

#ifdef DEBUG
	fprintf(stderr, "Receiving data.\n");
#endif

	i2c_writeReg(REG_FIFO_WR_PTR, 0x00);
	i2c_writeReg(REG_OVF_COUNTER, 0x00);
	i2c_writeReg(REG_FIFO_RD_PTR, 0x00);

	running = true;

	thr = std::thread(&MAX30102rpi::dataWorker, this);
}

void MAX30102rpi::stop()
{
	if (!running)
		return;
	running = false;
	thr.join();
	reset();

	gpiod_line_release(lineIRQ);
	gpiod_chip_close(chip);
}

// i2c read and write protocols
void MAX30102rpi::i2c_writeReg(uint8_t reg, uint8_t data)
{
	uint8_t buf[2];
	buf[0] = reg;
	buf[1] = data;
	long int r = write(fd, buf, 2);
	if (r < 0)
	{
#ifdef DEBUG
		fprintf(stderr, "i2c_writeReg(%d,%d): Could not write word to %02x. ret=%ld.\n", reg, data, max30102settings.address, r);
#endif
		throw std::invalid_argument("Could not write to a register in the MAX30102.");
	}
}

uint8_t MAX30102rpi::i2c_readReg(uint8_t reg)
{
	const char eexp[] = "readReg: Could not read.\n";
	uint8_t buf = reg;
	long int r = write(fd, &buf, 1);
	if (r < 0)
	{
#ifdef DEBUG
		fprintf(stderr, "ReadReg: Could not request register %d value. Error code = %ld\n", reg, r);
#endif
		throw std::invalid_argument(eexp);
	}
	r = read(fd, &buf, 1);
	if (r < 0)
	{
#ifdef DEBUG
		fprintf(stderr, "ReadReg: Could not read register %d. Error code = %ld\n", reg, r);
#endif
		throw std::invalid_argument(eexp);
	}
	return (uint8_t)buf;
}

void MAX30102rpi::reset()
{
	// reset
	i2c_writeReg(REG_MODE_CONFIG, 0x40);
#ifdef DEBUG
	fprintf(stderr, "Mode reg after reset: %02x\n", i2c_readReg(REG_MODE_CONFIG));
#else
	i2c_readReg(REG_MODE_CONFIG);
#endif
}

void MAX30102rpi::dataReady()
{
	char tmp[6];
	long int r;
	tmp[0] = REG_FIFO_DATA;
	r = write(fd, tmp, 1);
	if (r < 0)
	{
#ifdef DEBUG
		fprintf(stderr, "Could not send reg to read ADC value. ret=%ld.\n", r);
#endif
		throw std::invalid_argument("dataReady: Could not send register to MAX30102.");
	}
	r = read(fd, tmp, 6);
	if (r < 0)
	{
#ifdef DEBUG
		fprintf(stderr, "Could not read ADC value. ret=%ld.\n", r);
#endif
		throw std::invalid_argument("dataReady: Could not read data from MAX30102.");
	}
	Sample s;
	const float maxValue = (float)(1 << 24);
	const unsigned redRaw = ((unsigned)(tmp[0])) << 16 | (((unsigned)(tmp[1])) << 8) | ((unsigned)(tmp[2]));
	const unsigned irRaw = ((unsigned)(tmp[3])) << 16 | (((unsigned)(tmp[4])) << 8) | ((unsigned)(tmp[5]));
	s.red = (float)redRaw / maxValue;
	s.ir = (float)irRaw / maxValue;
	for (auto &callback : callbacks)
	{
		callback->hasSample(s);
	}
	i2c_readReg(REG_INTR_STATUS_1);
	i2c_readReg(REG_INTR_STATUS_2);
}

void MAX30102rpi::dataWorker()
{
	while (running)
	{
		gpiod_line_event_wait(lineIRQ, &ts);
		struct gpiod_line_event event;
		gpiod_line_event_read(lineIRQ, &event);
		dataReady();
	}
}
