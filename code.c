#include "code.h"

uint32_t* getValue(int file_desc, uint8_t* command, int clock_length)
{
	uint8_t* rx = calloc(sizeof(command), 1);
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)command,
		.rx_buf = (unsigned long)rx,
		.len = clock_length,
		.delay_usecs = 0,
		.speed_hz = 20000000,
		.bits_per_word = 8,
	};

	ioctl(file_desc, SPI_IOC_MESSAGE(1), &tr);
	uint32_t* return_array = convertToLongArray(rx);
	free(rx);
	return return_array;
}

uint32_t* convertToLongArray(uint8_t* arr)
{
	uint32_t* return_value = calloc(sizeof(arr)/4, sizeof(uint32_t));
	uint32_t tmp = 0x0;
	int i = 0;
	for (; i < sizeof(arr); i++)
	{
		tmp = (tmp << 8) + *(arr+i);
		if (i%4 == 0 && i != 0)
			*(return_value+(i/4)-1) = tmp;
	}
	*(return_value+sizeof(arr)/4-1) = tmp;
	return return_value;
}

int initTDC(int file_desc)
{
	uint8_t tx[4] = {0x41, 0x0, 0x0, 0x0};
	uint32_t* return_buffer = getValue(file_desc, tx, 2);
	if (return_buffer)
	{
		free(return_buffer);
		return 0;
	}
	else
	{
		return -1;
	}
}


int startMeas(int file_desc)
{
	uint8_t tx[4] = {0x40, 0x43, 0x0, 0x0};
	uint32_t* return_buffer = getValue(file_desc, tx, 2);
	if (return_buffer)
	{
		free(return_buffer);
		return 0;
	}
	else
	{
		return -1;
	}
}

double getToF(int file_desc)
{
	uint8_t command[5];
	command[0] = 0x10;
	command[1] = 0x11;
	command[2] = 0x12;
	command[3] = 0x1B;
	command[4] = 0x1C;
	uint8_t padding[4] = {0x0, };
	uint32_t results[5];
	uint32_t* tmp;
	int i = 0;
	for (; i < 5; i++)
	{
		padding[0] = command[i];
		tmp = getValue(file_desc, padding, 4);
		results[i] = *tmp & 0x7FFFFF;
		free(tmp);
	}
	results[1] &= 0xFFFF;
	int32_t time1 = results[0];
	int32_t clock_count1 = results[1];
	int32_t time2 = results[2];
	int32_t calibration1 = results[3];
	int32_t calibration2 = results[4];
	double cal_count = calibration2 - calibration1;
	if (cal_count <= 0.0)
		return 0;
	return ((time1-time2)/cal_count + clock_count1)/CLOCK_SPEED;
}

uint32_t* configurePins(int mem_file)
{
	wiringPiSetup();
	uint32_t* clk_reg = (uint32_t *) mmap(0, 0xA8, PROT_READ|PROT_WRITE|PROT_EXEC, MAP_SHARED|MAP_LOCKED, mem_file, 0x3f101000);
	if (!clk_reg)
		return 0;
	clk_reg[28] = 0x5a000020;
	while (clk_reg[28] & 0x00000080);
	clk_reg[29] = 0x5a002000;
	delayMicroseconds(10);
	clk_reg[28] = 0x5a000001;
	delayMicroseconds(10);
	clk_reg[28] |= 0x5a000010;
	pinMode(PIN_CLK, GPIO_CLOCK);
	pinMode(PIN_EN, OUTPUT);
	pinMode(PIN_START, OUTPUT);
	pinMode(PIN_STOP, OUTPUT);
	digitalWrite(PIN_EN, 0);
	digitalWrite(PIN_START, 0);
	digitalWrite(PIN_STOP, 0);
	digitalWrite(PIN_EN, 1);
	delay(1);
	return clk_reg;
}

void deconfigurePins(uint32_t* clk_reg)
{
	pinMode(PIN_CLK, INPUT);
	clk_reg[28] = 0x5a000020;
	while (clk_reg[28] & 0x00000080);
}

int main()
{
	int fd = open(device, O_RDWR);
	int mem_file = open("/dev/mem", O_RDWR | O_SYNC);
	FILE* csv = fopen("values.csv", "a");
	if (!fd || !mem_file || !csv)
	{
		printf("Initialization Error\n");
		return 1;
	}
	uint32_t* clk_reg = configurePins(mem_file);
	if  (!clk_reg)
	{
		printf("Pin Configuration Error\n");
		return 2;
	}
	initTDC(fd);
	time_t date;
	double tof;
	char buffer[100];
	for (int i = 0; i < 1000; i++)
	{
		while (micros() % 1000);
		startMeas(fd);
		while(!digitalRead(PIN_TRIG));
		digitalWrite(PIN_START, 1);
		digitalWrite(PIN_START, 0);
		delayMicroseconds(5);
		digitalWrite(PIN_STOP, 1);
		digitalWrite(PIN_STOP, 0);
		while(digitalRead(PIN_INTB));
		tof = getToF(fd)*1000000;
		time(&date);
		strftime(buffer, 100, "%F,%T", gmtime(&date));
		fprintf(csv, "%s,%f us\n", buffer, tof);


	}
	deconfigurePins(clk_reg);
	close(mem_file);
	fclose(csv);
	close(fd);
	return 0;
}
