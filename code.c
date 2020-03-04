#include "code.h"

//Sends commands and returns results via SPI from TDC
uint32_t* getValue(int file_desc, uint8_t* command, int clock_length)
{
	//Initializes return buffer
	uint8_t* rx = calloc(sizeof(command), 1);

	//Initializes command
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)command,
		.rx_buf = (unsigned long)rx,
		.len = clock_length,
		.delay_usecs = 0,
		.speed_hz = 20000000,
		.bits_per_word = 8,
	};

	//Sends command over spidev driver
	ioctl(file_desc, SPI_IOC_MESSAGE(1), &tr);

	//Converts results to unit32_t array
	uint32_t* return_array = convertToLongArray(rx);

	//Frees results buffer
	free(rx);

	//Returns the unit32_t array
	return return_array;
}

//Converts uint8_t array to uint32_t* array
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

//Initializes CONFIG1 Register
int initTDC(int file_desc)
{
	//Writes to CONFIG1 to not do multi cycle averaging
	uint8_t tx[4] = {0x41, 0x0, 0x0, 0x0};

	//Sends command
	uint32_t* return_buffer = getValue(file_desc, tx, 2);
	
	//If successful command
	if (return_buffer)
	{
		//Frees the buffer and returns success
		free(return_buffer);
		return 0;
	}
	//Otherwise
	else
	{
		return -1;
	}
}

//Starts a measurement
int startMeas(int file_desc)
{
	//Writes to CONFIG0 to start measurements in
	//configuration mode 2 with a parity bit
	uint8_t tx[4] = {0x40, 0x43, 0x0, 0x0};

	//Sends command
	uint32_t* return_buffer = getValue(file_desc, tx, 2);
	
	//If successful command
	if (return_buffer)
	{
		//Frees the buffer and returns success
		free(return_buffer);
		return 0;
	}
	//Otherwise
	else
	{
		return -1;
	}
}

//Gets time of flight from TDC
double getToF(int file_desc)
{
	//Initializes commands to be sent to TDC
	uint8_t command[5];
	command[0] = 0x10;	//Read TIME1
	command[1] = 0x11;	//Read CLOCK_COUNT1
	command[2] = 0x12;	//Read TIME2
	command[3] = 0x1B;	//Read CALIBRATION1
	command[4] = 0x1C;	//Read CALIBRATION2

	//Formats commands and results
	uint8_t padding[4] = {0x0, };
	uint32_t results[5];
	uint32_t* tmp;
	int i = 0;

	//Sends out the commands
	for (; i < 5; i++)
	{
		//Sets command
		padding[0] = command[i];

		//Sends command
		tmp = getValue(file_desc, padding, 4);

		//Applies correct bitmask and stores number
		results[i] = *tmp & 0x7FFFFF;

		//Frees dynamic array
		free(tmp);
	}

	//Applies different bitmask for CLOCK_COUNT1
	results[1] &= 0xFFFF;

	//Stores dynamic array information in variables for readability
	//Casts them to signed integers for correct math
	int32_t time1 = results[0];
	int32_t clock_count1 = results[1];
	int32_t time2 = results[2];
	int32_t calibration1 = results[3];
	int32_t calibration2 = results[4];
	
	//Computes time of flight
	double cal_count = calibration2 - calibration1;
	if (cal_count <= 0.0)
		return 0;
	return ((time1-time2)/cal_count + clock_count1)/CLOCK_SPEED;
}

//Pin Configuration function
uint32_t* configurePins(int mem_file)
{
	//Sets up pin addresses and other under the hood stuff
	wiringPiSetup();

	//Memory maps clock register to userspace
	uint32_t* clk_reg = (uint32_t *) mmap(0, 0xA8, 
			PROT_READ|PROT_WRITE|PROT_EXEC, 
			MAP_SHARED|MAP_LOCKED, mem_file,
		       	0x3f101000);

	//If unsuccessful mapping
	if (!clk_reg)
		return 0;

	//Kills clock and waits for it to die
	clk_reg[28] = 0x5a000020;
	while (clk_reg[28] & 0x00000080);
	
	//Configures clock to divide base by two
	clk_reg[29] = 0x5a002000;
	delayMicroseconds(10);

	//Configures clock to use oscillator as base clock
	clk_reg[28] = 0x5a000001;
	delayMicroseconds(10);

	//Start the clock
	clk_reg[28] |= 0x5a000010;

	//Configures the following pins in these manners
	pinMode(PIN_CLK, GPIO_CLOCK);
	pinMode(PIN_EN, OUTPUT);
	pinMode(PIN_START, OUTPUT);
	pinMode(PIN_STOP, OUTPUT);

	//Enables the TDC and ensures pins are in correct state
	digitalWrite(PIN_EN, 0);
	digitalWrite(PIN_START, 0);
	digitalWrite(PIN_STOP, 0);
	digitalWrite(PIN_EN, 1);
	delay(1);

	//Returns address to clock register
	return clk_reg;
}

//Pin Deconfiguration Function
void deconfigurePins(uint32_t* clk_reg)
{
	//Sets the clock pin to input
	pinMode(PIN_CLK, INPUT);

	//Kills the clock
	clk_reg[28] = 0x5a000020;
	while (clk_reg[28] & 0x00000080);
}

//Main Function
int main()
{
	//Initialize files to be read/write to
	//SPI Driver
	int spi_driver = open("/dev/spidev0.0", O_RDWR);
	
	//Memory File to access clock pins
	int mem_file = open("/dev/mem", O_RDWR | O_SYNC);

	//File where values are to be written to
	FILE* csv = fopen("values.csv", "a");

	//If it was unsuccessful at initializing those files
	if (!spi_driver || !mem_file || !csv)
	{
		printf("Initialization Error\n");
		return 1;
	}

	//Configures pins and gets address of clock register
	uint32_t* clk_reg = configurePins(mem_file);

	//If unsuccessful pin registration
	if  (!clk_reg)
	{
		printf("Pin Configuration Error\n");
		return 2;
	}

	//Initalizes the TDC's CONFIG1 register
	initTDC(spi_driver);

	//Instantiates important file writing variables
	time_t date;
	double tof;
	char buffer[100];
	struct timeval currentTime;
	gettimeofday(&currentTime, NULL);
	uint32_t timestamp = currentTime.tv_usec;
	uint32_t prev_timestamp = currentTime.tv_usec;
	//Repeats 5000 times
	for (int i = 0; i < 5000; i++)
	{
		//While the current time is not an exact millisecond
		//or the same time as previously recorded
		while (timestamp % 1000 || timestamp == prev_timestamp)
		{
			gettimeofday(&currentTime, NULL);
			timestamp = currentTime.tv_usec;
		}

		prev_timestamp = timestamp;

		//Begins the TDC's timing process
		startMeas(spi_driver);

		//Waits for a trigger from the TDC
		while(!digitalRead(PIN_TRIG));

		//Sends a start pulse to the TDC (testing)
		digitalWrite(PIN_START, 1);
		digitalWrite(PIN_START, 0);

		//Waits for 10 microseconds (testing)
		delayMicroseconds(10);

		//Sends stop pulse (testing)
		digitalWrite(PIN_STOP, 1);
		digitalWrite(PIN_STOP, 0);

		//Waits for interrupt to go low from the TDC
		while(digitalRead(PIN_INTB));

		//Gets the time of flight from the TDC in microseconds
		tof = getToF(spi_driver)*1000000;

		//Gets current UTC Time
		time(&date);
		strftime(buffer, 100, "%F,%T", gmtime(&date));

		//Writes date, time, and time of flight to file
		fprintf(csv, "%s,%f us\n", buffer, tof);
		fflush(csv);


	}

	//Deconfigures the pins
	deconfigurePins(clk_reg);

	//Closes files to be read/written to
	close(mem_file);
	fclose(csv);
	close(spi_driver);

	//Returns 0
	return 0;
}
