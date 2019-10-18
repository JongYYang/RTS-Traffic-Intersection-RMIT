#include <sys/procmgr.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <hw/inout.h>      // for in32() and out32();
#include <sys/mman.h>      // for mmap_device_io();
#include <stdint.h>        // for unit32 types
#include <sys/netmgr.h>
#include <sys/neutrino.h>
#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <sys/iofunc.h>
#include "states.h"
#include "modes.h"
#include <sys/dispatch.h>
#include <fcntl.h>
#include <share.h>
#include <string.h>
#include <devctl.h>
#include <hw/i2c.h>
#include <unistd.h>
#include <sys/neutrino.h>
#include "lcd.h"

#define TRAFFICLIGHT_ATTACH_POINT  "/net/cycloneV_user/dev/name/local/TrafficLightAttach"  // hostname using full path, change myname to the name used for server

#define BUF_SIZE 100

char lcd[21];
char lcd2[21];

/*
 * I2C LCD test program using BBB I2C1
 *
 * LCD data:  MIDAS  MCCOG22005A6W-FPTLWI  Alphanumeric LCD, 20 x 2, Black on White, 3V to 5V, I2C, English, Japanese, Transflective
 * LCD data sheet, see:
 *                      http://au.element14.com/midas/mccog22005a6w-fptlwi/lcd-alpha-num-20-x-2-white/dp/2425758?ost=2425758&selectedCategoryId=&categoryNameResp=All&searchView=table&iscrfnonsku=false
 *
 *  BBB P9 connections:
 *    - P9_Pin17 - SCL - I2C1    GPIO3_2
 *    - P9_Pin18 - SDA - I2C1    GPIO3_1
 *
 *  LCD connections:
 *    - pin 1 – VOUT to the 5V     (external supply should be used)
 *    - pin 4 – VDD  to the 5V
 *    - pin 8 – RST  to the 5V
 *
 *    - pin 2 - Not connected  (If 3.3V is used then add two caps)
 *    - pin 3 - Not connected  (If 3.3V is used then add two caps)
 *
 *    - pin 5 - VSS  to Ground
 *    - pin 6 – SDA  to the I2C SDA Pin  pulled high using a 4.7kohm resitor connected to 5V
 *    - pin 7 - SCL  to the I2C SCL Pin  pulled high using a 4.7kohm resitor connected to 5V
 *
 * Author:  Samuel Ippolito
 * Date:    10/04/2017
 */

 //#include "shared_data_structures.h"
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

char* progname = "majorproject_kevin.c";
#define MY_PULSE_CODE   _PULSE_CODE_MINAVAIL

#define AM335X_CONTROL_MODULE_BASE   (uint64_t) 0x44E10000
#define AM335X_CONTROL_MODULE_SIZE   (size_t)   0x00001448
#define AM335X_GPIO_SIZE             (uint64_t) 0x00001000
#define AM335X_GPIO1_BASE            (size_t)   0x4804C000

#define LED0          (1<<21)   // GPIO1_21
#define LED1          (1<<22)   // GPIO1_22
#define LED2          (1<<23)   // GPIO1_23
#define LED3          (1<<24)   // GPIO1_24

#define SD0 (1<<28)  // SD0 is connected to GPIO1_28
#define SCL (1<<16)  // SCL is connected to GPIO1_16

#define GPIO_OE        0x134
#define GPIO_DATAIN    0x138
#define GPIO_DATAOUT   0x13C

#define GPIO_IRQSTATUS_SET_1 0x38   // enable interrupt generation
#define GPIO_IRQWAKEN_1      0x48   // Wakeup Enable for Interrupt Line
#define GPIO_FALLINGDETECT   0x14C  // set falling edge trigger
#define GPIO_CLEARDATAOUT    0x190  // clear data out Register
#define GPIO_IRQSTATUS_1     0x30   // clear any prior IRQs

#define GPIO1_IRQ 99  // TRG page 465 list the IRQs for the am335x

#define P9_12_pinConfig 0x878 //  conf_gpmc_ben1 (TRM pp 1364) for GPIO1_28,  P9_12

// GPMC_A1_Configuration
#define PIN_MODE_0   0x00
#define PIN_MODE_1   0x01
#define PIN_MODE_2   0x02
#define PIN_MODE_3   0x03
#define PIN_MODE_4   0x04
#define PIN_MODE_5   0x05
#define PIN_MODE_6   0x06
#define PIN_MODE_7   0x07

// PIN MUX Configuration strut values  (page 1420 from TRM)
#define PU_ENABLE    0x00
#define PU_DISABLE   0x01
#define PU_PULL_UP   0x01
#define PU_PULL_DOWN 0x00
#define RECV_ENABLE  0x01
#define RECV_DISABLE 0x00
#define SLEW_FAST    0x00
#define SLEW_SLOW    0x01

#define BUF_SIZE 100

#define Q_FLAGS O_RDWR | O_CREAT | O_EXCL
#define Q_Mode S_IRUSR | S_IWUSR

int inputCheck = 0;
enum modes CurrentMode = sensor;

//********************************************MESSAGE STUFF***************************************************
typedef struct
{
	struct _pulse hdr; // Our real data comes after this header
	int ClientID; // our data (unique id from client)
	int data;     // our data
} my_data;

typedef struct
{
	struct _pulse hdr; // Our real data comes after this header
	char buf[BUF_SIZE];// Message we send back to clients to tell them the messages was processed correctly.
} my_reply;



// prototypes
//int client(char *sname);
int* getTrafficLightmode();
void printTrafficlightStatus(char* data);
int* KEYPAD_THREAD(void* data);

void selectMode();

//*********************************************************************************
//******************************************KEYPAD***********************************

typedef union _CONF_MODULE_PIN_STRUCT // See TRMPage 1420
{
	unsigned int d32;
	struct
	{ // name: field size
		unsigned int conf_mmode : 3; // LSB
		unsigned int conf_puden : 1;
		unsigned int conf_putypesel : 1;
		unsigned int conf_rxactive : 1;
		unsigned int conf_slewctrl : 1;
		unsigned int conf_res_1 : 13; // reserved
		unsigned int conf_res_2 : 12; // reserved MSB
	} b;
} _CONF_MODULE_PIN;

void strobe_SCL(uintptr_t gpio_port_add)
{
	uint32_t PortData;
	PortData = in32(gpio_port_add + GPIO_DATAOUT); // value that is currently on the GPIOport
	PortData &= ~(SCL);
	out32(gpio_port_add + GPIO_DATAOUT, PortData); // Clock low
	delaySCL();
	PortData = in32(gpio_port_add + GPIO_DATAOUT); // get port value
	PortData |= SCL; // Clock high
	out32(gpio_port_add + GPIO_DATAOUT, PortData);
	delaySCL();
}

void delaySCL()
{ // Small delay used to get timing correct for BBB
	volatile int i, a;
	for (i = 0; i < 0x1F; i++) // 0x1F results in a delay that sets F_SCLto ~480 kHz
	{ // i*1 is faster than i+1 (i+1 results in F_SCL~454 kHz, whereas i*1 is the same as a=i)

		a = i;

	}
	// usleep(1); //why doesn't this work? Ans: Results in a period of 4ms as
	// fastest time, which is 250Hz (This is to slow for the TTP229 chip as it
	// requires F_SCLto be between 1 kHz and 512 kHz)
}

uint32_t KeypadReadIObit(uintptr_t gpio_base, uint32_t BitsToRead)
{
	volatile uint32_t val = 0;
	val = in32(gpio_base + GPIO_DATAIN);    // value that is currently on the GPIOport
	val &= BitsToRead; // mask bit

	if (val == BitsToRead)
		return 1;
	else
		return 0;
}

//Decoding
void DecodeKeyValue(uint32_t word)
{
	switch (word)
	{
	case 0x01:
		printf("EAST-WEST TURN\n");
		pthread_mutex_lock(&mutex);
		inputCheck = 1;
		pthread_mutex_unlock(&mutex);
		//pthread_create(NULL, NULL, Flash_LED0_ex, NULL); // flash LED
		break;
	case 0x02:
		printf("TRAINS INCOMING\n");
		pthread_mutex_lock(&mutex);
		inputCheck = 2;
		pthread_mutex_unlock(&mutex);
		//pthread_create(NULL, NULL, Flash_LED0_ex, NULL); // flash LED
		break;
	case 0x04:
		printf("NORTH-WEST TURN\n");
		pthread_mutex_lock(&mutex);
		inputCheck = 3;
		pthread_mutex_unlock(&mutex);
		//pthread_create(NULL, NULL, Flash_LED0_ex, NULL); // flash LED
		break;
	case 0x08:
		//printf("Key  4 pressed\n");
		//pthread_create(NULL, NULL, Flash_LED0_ex, NULL); // flash LED
		break;
	case 0x10:
		//printf("Key  5 pressed\n");
		//pthread_create(NULL, NULL, Flash_LED0_ex, NULL); // flash LED
		break;
	case 0x20:
		//printf("Key  6 pressed\n");
		//pthread_create(NULL, NULL, Flash_LED0_ex, NULL); // flash LED
		break;
	case 0x40:
		//printf("Key  7 pressed\n");
		//pthread_create(NULL, NULL, Flash_LED0_ex, NULL); // flash LED
		break;
	case 0x80:
		//printf("Key  8 pressed\n");
		//pthread_create(NULL, NULL, Flash_LED0_ex, NULL); // flash LED
		break;
	case 0x100:
		//printf("Key  9 pressed\n");
		//pthread_create(NULL, NULL, Flash_LED0_ex, NULL); // flash LED
		break;
	case 0x200:
		//printf("Key 10 pressed\n");
		//pthread_create(NULL, NULL, Flash_LED0_ex, NULL); // flash LED
		break;
	case 0x400:
		//printf("Key 11 pressed\n");
		//pthread_create(NULL, NULL, Flash_LED0_ex, NULL); // flash LED
		break;
	case 0x800:
		//printf("Key 12 pressed\n");
		//pthread_create(NULL, NULL, Flash_LED0_ex, NULL); // flash LED
		break;
	case 0x1000:
		//printf("Key 13 pressed\n");
		//pthread_create(NULL, NULL, Flash_LED0_ex, NULL); // flash LED
		break;
	case 0x2000:
		//printf("Key 14 pressed\n");
		//pthread_create(NULL, NULL, Flash_LED0_ex, NULL); // flash LED
		break;
	case 0x4000:
		//printf("Key 15 pressed\n");
		//pthread_create(NULL, NULL, Flash_LED0_ex, NULL); // flash LED
		break;
	case 0x8000:
		//          printf("Key 16 pressed\n");
				//pthread_create(NULL, NULL, Flash_LED0_ex, NULL); // flash LED
		break;
	case 0x00:  // key release event (do nothing)
		break;
	default:
		printf("Key pressed could not be determined - %lu\n", word);
	}
}

//*****************************************KEYPAD THREAD************************************
int* KEYPAD_THREAD(void* data)
{
	uintptr_t control_module = mmap_device_io(AM335X_CONTROL_MODULE_SIZE, AM335X_CONTROL_MODULE_BASE);
	uintptr_t gpio1_base = mmap_device_io(AM335X_GPIO_SIZE, AM335X_GPIO1_BASE);

	if ((control_module) && (gpio1_base))  // if mapped then
	{
		ThreadCtl(_NTO_TCTL_IO_PRIV, 1);  // Request I/O privileges;
		volatile uint32_t val = 0;

		// set DDR for LEDsto output and GPIO_28 to input
		val = in32(gpio1_base + GPIO_OE); // read in current setup for GPIO1 port
		val |= 1 << 28; // set IO_BIT_28 high (1=input, 0=output)
		out32(gpio1_base + GPIO_OE, val); // write value to input enable for data pins
		val &= ~(LED0 | LED1 | LED2 | LED3); // write value to output enable
		out32(gpio1_base + GPIO_OE, val); // write value to output enable for LED pins

		in32s(&val, 1, control_module + P9_12_pinConfig);
		//printf("Original pinmux configuration for GPIO1_28 = %#010x\n", val);

		// set up pin mux for the pins we are going to use (see page 1354 of TRM)
		volatile _CONF_MODULE_PIN pinConfigGPMC; // Pin configuration strut
		pinConfigGPMC.d32 = 0;

		// Pin MUX register default setup for input (GPIOinput, disable pull up/down -Mode 7)
		pinConfigGPMC.b.conf_slewctrl = SLEW_SLOW; // Select between faster or slower slew rate
		pinConfigGPMC.b.conf_rxactive = RECV_ENABLE; // Input enable value for the PAD
		pinConfigGPMC.b.conf_putypesel = PU_PULL_UP; // Pad pullup/pulldowntype selection
		pinConfigGPMC.b.conf_puden = PU_ENABLE; // Pad pullup/pulldownenable
		pinConfigGPMC.b.conf_mmode = PIN_MODE_7; // Pad functional signal mux select 0 -7

		// Write to PinMuxregisters for the GPIO1_28
		out32(control_module + P9_12_pinConfig, pinConfigGPMC.d32);
		in32s(&val, 1, control_module + P9_12_pinConfig); // Read it back
		//printf("New configuration register for GPIO1_28 = %#010x\n", val);

		val = in32(gpio1_base + GPIO_OE);
		val &= ~SCL; // 0 for output
		out32(gpio1_base + GPIO_OE, val); // write value to output enable for data pins

		val = in32(gpio1_base + GPIO_DATAOUT);
		val |= SCL; // Set Clock Line High as per TTP229-BSF datasheet
		out32(gpio1_base + GPIO_DATAOUT, val); // for 16-Key active-Low timing diagram

		int i = 0;
		for (;;) // for loop that correctly decodes key press
		{
			volatile uint32_t word = 0;
			// confirm that SD0 is still low (that is a valid Key press event has occurred)
			val = KeypadReadIObit(gpio1_base, SD0); // read SD0 (means data is ready)
			if (val == 0) // start reading key value form the keypad
			{
				pthread_mutex_lock(&mutex);
				word = 0; // clear word variable
				pthread_mutex_unlock(&mutex);

				delaySCL(); // wait a short period of time before reading the data Tw (10 us)

				for (i = 0; i < 16; i++) // get data from SD0 (16 bits)
				{
					strobe_SCL(gpio1_base); // strobe the SCLline so we can read in data bit
					val = KeypadReadIObit(gpio1_base, SD0); // read in data bit
					val = ~val & 0x01; // invert bit and mask out everything but the LSB
					//printf("val[%u]=%u, ",i, val); // debug code
					pthread_mutex_lock(&mutex);
					word = word | (val << i); // add data bit to word in unique position (build word up bit by bit)
					pthread_mutex_unlock(&mutex);
				}

				//printf("word=%u\n",word); // debug code
				DecodeKeyValue(word);
			}
		}
		// will never get here
		munmap_device_io(control_module, AM335X_CONTROL_MODULE_SIZE);
	}

}

//*****************************************************************************************************

//******************************SENSOR MODE STATE MACHINE************************************************
typedef union
{
	struct _pulse pulse;
	// your other message structures would go here too
} my_message_t;

void LCD_display(char* inputState, char* inputState2)
{
	int file;
	int error;
	volatile uint8_t LCDi2cAdd = 0x3C;
	_Uint32t speed = 10000; // nice and slow (will work with 200000)

	uint8_t LCDdata[21] = { };

	// Open I2C resource and set it up
	if ((file = open("/dev/i2c1", O_RDWR)) < 0)    // OPEN I2C1
		printf("Error while opening Device File.!!\n");
	else
		printf("I2C1 Opened Successfully\n");

	error = devctl(file, DCMD_I2C_SET_BUS_SPEED, &(speed), sizeof(speed), NULL);  // Set Bus speed
	if (error)
	{
		fprintf(stderr, "Error setting the bus speed: %d\n", strerror(error));
		return;
		exit(EXIT_FAILURE);
	}
	else
		printf("Bus speed set = %d\n", speed);

	Initialise_LCD(file, LCDi2cAdd);

	usleep(1);

	// write some Text to the LCD screen
		 /*  SetCursor(file, LCDi2cAdd, 0, 0); // set cursor on LCD to first position first line
		   strcpy(LCDdata, "Line 1 (0,0)");
		   I2cWrite_(file, LCDi2cAdd, DATA_SEND, &LCDdata[0], sizeof(LCDdata));        // write new data to I2C

		   SetCursor(file, LCDi2cAdd, 1, 0); // set cursor on LCD to first position second line
		   strcpy(LCDdata, "Line 2 (1,0)");
		   I2cWrite_(file, LCDi2cAdd, DATA_SEND, &LCDdata[0], sizeof(LCDdata));        // write new data to I2C

		  */
		  // replace the letter 'k' with '_'

	SetCursor(file, LCDi2cAdd, 0, 0); // set cursor on LCD to first position first line
	strcpy(LCDdata, inputState);
	I2cWrite_(file, LCDi2cAdd, DATA_SEND, &LCDdata[0], sizeof(LCDdata));

	//  sleep(2);

	if (inputState2 != NULL)
	{
		SetCursor(file, LCDi2cAdd, 1, 0); // set cursor on LCD to first position second line
		strcpy(LCDdata, inputState2);
		I2cWrite_(file, LCDi2cAdd, DATA_SEND, &LCDdata[0], sizeof(LCDdata));        // write new data to I2C

		sleep(2);
	}
}


int SensorModeStateMachine(CurrentState)
{

	char lcd[21];
	char lcd2[21];



	switch (CurrentState)
	{
	case State0:


		printf("S0: ALL RED\n"); //Initial State
		strcpy(lcd, "S0: ALL RED\n");
		strcpy(lcd2, "S0");
		LCD_display(lcd, lcd2);

		time_t endwait;
		time_t start = time(NULL);
		time_t seconds = 3; // end loop after this time has elapsed

		endwait = start + seconds;

		//  printf("start time is : %s", ctime(&start)); Debug line to see start time of loop.

		while (start < endwait)
		{
			/* Do stuff while waiting */
			//sleep(1);   // sleep 1s.
			start = time(NULL);
			//  printf("loop time is : %s", ctime(&start)); Debug line to see the times within the loop.

			if (inputCheck == 2)
			{

				inputCheck = 0;
				pthread_mutex_lock(&mutex);
				CurrentState = State5;
				pthread_mutex_unlock(&mutex);
				goto StateNUMBER5;
			}


			if (inputCheck == 1)
			{
				pthread_mutex_lock(&mutex);
				CurrentState = State1;
				pthread_mutex_unlock(&mutex);
				goto StateNUMBER1;
			}
		}

		goto StateNUMBER3;

		//CurrentState = State1;
		//printf("end time is %s", ctime(&endwait)); Debug line to see the end times within the loop.

		sleep(1);
		break;
	case State1:

	StateNUMBER1: printf("S1: EW TURN GREEN\n");
		strcpy(lcd, "S1: EW TURN GREEN\n");
		LCD_display(lcd, NULL);

		if (inputCheck == 2) //Checking for Train.
		{
			inputCheck = 0;
			pthread_mutex_lock(&mutex);
			CurrentState = State5;
			pthread_mutex_unlock(&mutex);
			goto StateNUMBER5;
		}


		pthread_mutex_lock(&mutex);
		CurrentState = State2;
		pthread_mutex_unlock(&mutex);
		sleep(1);
		break;
	case State2:

		printf("S2: EW TURN YELLOW\n");
		strcpy(lcd, "S2: EW TURN YELLOW\n");
		LCD_display(lcd, NULL);

		if (inputCheck == 2) //Checking for Train.
		{
			inputCheck = 0;
			pthread_mutex_lock(&mutex);
			CurrentState = State5;
			pthread_mutex_unlock(&mutex);
			goto StateNUMBER5;
		}

		pthread_mutex_lock(&mutex);
		CurrentState = State3;
		pthread_mutex_unlock(&mutex);
		sleep(2);
		break;

	case State3:

	StateNUMBER3: printf("S3: EW TURN RED\n");
		strcpy(lcd, "S3: EW TURN RED\n");
		LCD_display(lcd, NULL);

		if (inputCheck == 2) //Checking for Train.
		{
			inputCheck = 0;
			pthread_mutex_lock(&mutex);
			CurrentState = State5;
			pthread_mutex_unlock(&mutex);
			goto StateNUMBER5;
		}

		pthread_mutex_lock(&mutex);
		CurrentState = State4;
		pthread_mutex_unlock(&mutex);
		sleep(1);
		break;

	case State4:
		if (CurrentMode != sensor)
		{
			printf("yes\n");
		}
		printf("S4: EW STRAIGHT GREEN, & EW PEDESTRIAN\n");
		strcpy(lcd, "S4: EW GREEN");
		strcpy(lcd2, "S4:EW PED GREEN");
		LCD_display(lcd, lcd2);

		if (inputCheck == 2) //Checking for Train.
		{
			inputCheck = 0;
			pthread_mutex_lock(&mutex);
			CurrentState = State5;
			pthread_mutex_unlock(&mutex);
			goto StateNUMBER5;
		}

		pthread_mutex_lock(&mutex);
		CurrentState = State5;
		pthread_mutex_unlock(&mutex);
		sleep(1);
		break;

	case State5:    //long delay

	StateNUMBER5: printf("S5: EW STRAIGHT/TURN YELLOW, & PEDESTRIAN BLINKING\n");

		strcpy(lcd, "S5: EW YELLOW\n");
		strcpy(lcd2, "S5: PED BLINKING ");
		LCD_display(lcd, lcd2);
		time_t endwait5;
		time_t start5 = time(NULL);
		time_t seconds5 = 3; // end loop after this time has elapsed

		endwait5 = start5 + seconds5;

		//  printf("start time is : %s", ctime(&start)); Debug line to see start time of loop.

		while (start5 < endwait5)
		{
			// Do stuff while waiting
			//sleep(1);   // sleep 1s.
			start5 = time(NULL);
			//  printf("loop time is : %s", ctime(&start)); Debug line to see the times within the loop.

			if (inputCheck == 2)
			{
				inputCheck = 0;
				pthread_mutex_lock(&mutex);
				CurrentState = State12;
				pthread_mutex_unlock(&mutex);
				goto StateNUMBER12;
			}
		}

		goto StateNUMBER6;
		sleep(1);
		break;

	case State6:

	StateNUMBER6: printf("S6: ALL RED\n");
		strcpy(lcd, "S6: ALL RED\n");
		LCD_display(lcd, NULL);

		if (inputCheck == 2) //Checking for Train.
		{
			inputCheck = 0;
			pthread_mutex_lock(&mutex);
			CurrentState = State11;
			pthread_mutex_unlock(&mutex);
			goto StateNUMBER11;
		}

		time_t endwait2;
		time_t start2 = time(NULL);
		time_t seconds2 = 10; // end loop after this time has elapsed

		endwait2 = start2 + seconds2;

		//  printf("start time is : %s", ctime(&start)); Debug line to see start time of loop.

		while (start2 < endwait2)
		{
			/* Do stuff while waiting */
			//sleep(1);   // sleep 1s.
			start2 = time(NULL);
			//  printf("loop time is : %s", ctime(&start)); Debug line to see the times within the loop.

			if (inputCheck == 3)
			{
				pthread_mutex_lock(&mutex);
				CurrentState = State7;
				pthread_mutex_unlock(&mutex);
				goto StateNUMBER7;
			}
		}

		goto StateNUMBER9;

	case State7:

	StateNUMBER7: printf("S7: NS TURN GREEN\n");
		strcpy(lcd, "S7: NST GREEN\n");
		LCD_display(lcd, NULL);

		if (inputCheck == 2) //Checking for Train.
		{
			inputCheck = 0;
			pthread_mutex_lock(&mutex);
			CurrentState = State11;
			pthread_mutex_unlock(&mutex);
			goto StateNUMBER11;
		}


		pthread_mutex_lock(&mutex);
		CurrentState = State8;
		pthread_mutex_unlock(&mutex);
		sleep(1);
		break;

	case State8:

		printf("S8: NS TURN YELLOW\n");
		strcpy(lcd, "S8: NST YELLOW\n");
		LCD_display(lcd, NULL);

		if (inputCheck == 2) //Checking for Train.
		{
			inputCheck = 0;
			pthread_mutex_lock(&mutex);
			CurrentState = State11;
			pthread_mutex_unlock(&mutex);
			goto StateNUMBER11;
		}

		pthread_mutex_lock(&mutex);
		CurrentState = State9;
		pthread_mutex_unlock(&mutex);
		sleep(1);
		break;

	case State9:

	StateNUMBER9: printf("S9: NS TURN RED\n");
		strcpy(lcd, "S9: NST RED\n");
		LCD_display(lcd, NULL);

		if (inputCheck == 2) //Checking for Train.
		{
			inputCheck = 0;
			pthread_mutex_lock(&mutex);
			CurrentState = State11;
			pthread_mutex_unlock(&mutex);
			goto StateNUMBER11;
		}

		pthread_mutex_lock(&mutex);
		CurrentState = State10;
		pthread_mutex_unlock(&mutex);
		sleep(1);
		break;
	case State10:

		printf("S10: NS Straight Green & NS Pedestrian\n");
		strcpy(lcd, "S10: NS Green");
		strcpy(lcd2, "NS Pedestrian\n");
		LCD_display(lcd, lcd2);

		if (inputCheck == 2) //Checking for Train.
		{
			inputCheck = 0;
			pthread_mutex_lock(&mutex);
			CurrentState = State11;
			pthread_mutex_unlock(&mutex);
			goto StateNUMBER11;
		}

		pthread_mutex_lock(&mutex);
		CurrentState = State11;
		pthread_mutex_unlock(&mutex);
		sleep(1);
		break;

	case State11:

	StateNUMBER11: printf("S11: NS Straight/Turn Yellow & Pedestrian Blinking\n");
		strcpy(lcd, "S11: NS Yellow \n");
		strcpy(lcd2, "PED Blinking\n");
		LCD_display(lcd, lcd2);

		time_t endwait3;
		time_t start3 = time(NULL);
		time_t seconds3 = 3; // end loop after this time has elapsed

		endwait3 = start3 + seconds3;

		//  printf("start time is : %s", ctime(&start)); Debug line to see start time of loop.

		while (start3 < endwait3)
		{
			/* Do stuff while waiting */
			//sleep(1);   // sleep 1s.
			start3 = time(NULL);
			//  printf("loop time is : %s", ctime(&start)); Debug line to see the times within the loop.

			if (inputCheck == 2)
			{
				inputCheck = 0;
				pthread_mutex_lock(&mutex);
				CurrentState = State12;
				pthread_mutex_unlock(&mutex);
				goto StateNUMBER12;
			}
		}

		goto StateNUMBER15;

		pthread_mutex_lock(&mutex);
		CurrentState = State12;
		pthread_mutex_unlock(&mutex);
		sleep(1);
		break;

	case State12:

	StateNUMBER12: printf("S12: ALL RED\n");
		strcpy(lcd, "S12: ALL RED\n");
		LCD_display(lcd, NULL);
		pthread_mutex_lock(&mutex);
		CurrentState = State13;
		pthread_mutex_unlock(&mutex);
		sleep(1);
		break;

	case State13:

		printf("S13: NS STRAIGHT GREEN & NS  PEDESTRIAN\n");
		strcpy(lcd, "S13: NS GREEN");
		strcpy(lcd2, "NS  PEDESTRIAN\n");
		LCD_display(lcd, lcd2);
		pthread_mutex_lock(&mutex);
		CurrentState = State14;
		pthread_mutex_unlock(&mutex);
		sleep(1);
		break;

	case State14:

		printf("S14: NS STRAIGHT/TURN YELLOW, & PEDESTRIAN BLINKING\n");
		strcpy(lcd, "S14: NS YELLOW\n");
		strcpy(lcd2, "PED BLINKING\n");
		LCD_display(lcd, lcd2);
		pthread_mutex_lock(&mutex);
		CurrentState = State15;
		pthread_mutex_unlock(&mutex);
		sleep(1);
		break;

	case State15:

	StateNUMBER15: pthread_mutex_lock(&mutex);
		CurrentState = State15;
		pthread_mutex_unlock(&mutex);
		printf("S15: ALL RED ALL CYCLES COMPLETE\n");
		printf("STATE MACHINE CYCLE COMPLETE STARTING NEW CYCLE\n");

		strcpy(lcd, "S15: ALL RED\n");
		strcpy(lcd2, "ALL CYCLES COMPLETE\n");
		LCD_display(lcd, lcd2);
		inputCheck = 0;


		sleep(3);
		pthread_mutex_lock(&mutex);
		CurrentState = State0;
		pthread_mutex_unlock(&mutex);
		break;
	}
	return CurrentState;
}
//********************************************************************************************************

//******************************TIMER MODE STATE MACHINE************************************************

int TimerModeStateMachine(enum timermode_states TimerState)
{
	switch (TimerState)
	{

	}
	return TimerState;
}

//*****************************************************************************************************

//******************************FAULT MODE STATE MACHINE************************************************

int FaultModeStateMachine(enum faultmode_states FaultState)
{
	switch (FaultState)
	{

	}
	return FaultState;
}

//*****************************************************************************************************


//*******************************************SENSOR MODE***************************************************

void sensormode()
{
	if (CurrentMode != sensor)
	{
		return;
	}

	enum sensormode_states CurrentState = State0; // Declaring the enum within the main
//    struct sigevent event;
//    struct itimerspec itime;
//    struct itimerspec itime_B;
//    timer_t timer_id;
//    int chid;
//    int rcvid;
//    my_message_t msg;
//
//    chid = ChannelCreate(0); // Create a communications channel
//
//    event.sigev_notify = SIGEV_PULSE;
//
//    // create a connection back to ourselves for the timer to send the pulse on
//    event.sigev_coid = ConnectAttach(ND_LOCAL_NODE, 0, chid, _NTO_SIDE_CHANNEL, 0);
//    if (event.sigev_coid == -1)
//    {
//        printf(stderr, "%s:  couldn't ConnectAttach to self!\n", progname);
//        perror(NULL);
//        exit(EXIT_FAILURE);
//    }
//    //event.sigev_priority = getprio(0);  // this function is depreciated in QNX 700
//    struct sched_param th_param;
//    pthread_getschedparam(pthread_self(), NULL, &th_param);
//    event.sigev_priority = th_param.sched_curpriority;    // old QNX660 version getprio(0);
//
//    event.sigev_code = MY_PULSE_CODE;
//
//    // create the timer, binding it to the event
//    if (timer_create(CLOCK_REALTIME, &event, &timer_id) == -1)
//    {
//        printf(stderr, "%s:  couldn't create a timer, errno %d\n", progname, errno);
//        perror(NULL);
//        exit(EXIT_FAILURE);
//    }
//
//    // setup the timer (1s initial delay value, 0s reload interval)
//    itime.it_value.tv_sec = 1;            // 1 second
//    itime.it_value.tv_nsec = 0;    //
//    itime.it_interval.tv_sec = 1;          // 1 second
//    itime.it_interval.tv_nsec = 0; //
//
//    // and start the timer!
//    timer_settime(timer_id, 0, &itime, NULL);
//
//    // setup the timer (2s initial delay value, 0s reload interval)
//    itime_B.it_value.tv_sec = 2;              // 2 second
//    itime_B.it_value.tv_nsec = 0;    //
//    itime_B.it_interval.tv_sec = 2;          // 2 second
//    itime_B.it_interval.tv_nsec = 0; //
//
//    int timer_Runtimes = 30, timer_counter = 0;
//
//    for (timer_counter = 0; timer_counter < timer_Runtimes; timer_counter++)
//    {
//        // wait for message/pulse
//        rcvid = MsgReceive(chid, &msg, sizeof(msg), NULL);
//
//        // determine who the message came from
//        if (rcvid == 0) // this process
//        {
//            // received a pulse, now check "code" field...
//            if (msg.pulse.code == MY_PULSE_CODE) // we got a pulse
//            {
//                // Here we implement the state machine
//
//                if (CurrentState == State2 || CurrentState == State5)
//                {
//                    timer_settime(timer_id, 0, &itime_B, NULL);
//                }
//                else
//                {
//                    timer_settime(timer_id, 0, &itime, NULL);
//                }
//
//                CurrentState = (SensorModeStateMachine(CurrentState)); // pass address
//                timer_counter++;
//                fflush(stdout); // make sure we print to the screen
//            }
//        }
//    }
//
//    printf("\nSwitch statement got called %d times\n", timer_counter);

	// Create and start the thread using the explicit attributes

	int Runtimes = 15, counter = 0;

	while (CurrentMode == sensor)
	{

		pthread_mutex_lock(&mutex);
		CurrentState = (SensorModeStateMachine(CurrentState)); // pass address
		//counter++;
		pthread_mutex_unlock(&mutex);

	}

	if (CurrentMode != sensor)
	{
		printf("Hi\n");
		selectMode();
		return;

	}

	// return;
}

//*****************************************************************************************************

//******************************************OVERRIDE MODE********************************************************

void overrideI1N()
{
	while (CurrentMode == override_I1N)
	{

		printf("hi\n");

		strcpy(lcd, "O0: I1 N_ALLGREEN\n");
		strcpy(lcd2, "");
		LCD_display(lcd, lcd2);

	}
	if (CurrentMode != override_I1N)
	{
		selectMode();
		return;
	}

	//    enum overridemode_states OverrideState = I1NSG; // Declaring the enum within the main
	//    printf("O0: I1 N_ALLGREEN\n");
	//    printf("O0: I2 W_ALLGREEN\n");

		//OverrideModeStateMachine(OverrideState);

}

//*****************************************************************************************************

//******************************************OVERRIDE MODE********************************************************

void overrideI1S()
{
	while (CurrentMode == override_I1S)
	{



		strcpy(lcd, "O1: I1 S_ALLGREEN\n");
		strcpy(lcd2, "");
		LCD_display(lcd, lcd2);

	}
	if (CurrentMode != override_I1S)
	{
		selectMode();
		return;
	}

}

//*****************************************************************************************************

//******************************************OVERRIDE MODE********************************************************

void overrideI2N()
{
	while (CurrentMode == override_I2N)
	{


		strcpy(lcd, "O2: I1 E_ALLGREEN\n");
		strcpy(lcd2, "");
		LCD_display(lcd, lcd2);

	}
	if (CurrentMode != override_I2N)
	{
		selectMode();
		return;
	}

}

//*****************************************************************************************************

//******************************************OVERRIDE MODE********************************************************

void overrideI2S()
{
	while (CurrentMode == override_I2S)
	{



		strcpy(lcd, "O3: I1 E_ALLGREEN\n");
		strcpy(lcd2, "");
		LCD_display(lcd, lcd2);

	}
	if (CurrentMode != override_I2S)
	{
		selectMode();
		return;
	}

}

//*****************************************************************************************************

//******************************************OVERRIDE MODE********************************************************

void overrideW()
{
	while (CurrentMode == override_W)
	{



		strcpy(lcd, "O4: I1 W_ALLGREEN\n");
		strcpy(lcd2, "");
		LCD_display(lcd, lcd2);

	}
	if (CurrentMode != override_W)
	{
		selectMode();
		return;
	}

}

//*****************************************************************************************************

//******************************************OVERRIDE MODE********************************************************

void overrideE()
{
	while (CurrentMode == override_E)
	{


		strcpy(lcd, "O5: I1 E_ALLGREEN\n");
		strcpy(lcd2, "");
		LCD_display(lcd, lcd2);

	}
	if (CurrentMode != override_E)
	{
		selectMode();
		return;
	}


}

//*****************************************************************************************************

//******************************************OVERRIDE MODE********************************************************

void overrideRail()
{
	while (CurrentMode == override_Rail)
	{


		strcpy(lcd, "O6: ALL_RED_RAIL\n");
		strcpy(lcd2, "");
		LCD_display(lcd, lcd2);

	}
	if (CurrentMode != override_Rail)
	{
		selectMode();
		return;
	}

}

//*****************************************************************************************************

//******************************************TIMER MODE***********************************************

void timermode()
{
	while (CurrentMode == timer)
	{



		strcpy(lcd, "timer\n");
		strcpy(lcd2, "");
		LCD_display(lcd, lcd2);

	}
	if (CurrentMode != timer)
	{
		selectMode();
		return;
	}
}
//*****************************************************************************************************

//******************************************FAULT MODE***********************************************

void faultmode()
{
	while (CurrentMode == fault)
	{



		strcpy(lcd, "O7: ALL_BLINKING_YELLOW\n");
		strcpy(lcd2, "");
		LCD_display(lcd, lcd2);

	}
	if (CurrentMode != fault)
	{
		selectMode();
		return;
	}

}
//*****************************************************************************************************

//******************************************PEAK MODE***********************************************

void peakmode()
{


}
//*****************************************************************************************************

/*** Client code ***/
int* getTrafficLightmode(char* sname)
{

	//struct s_trafficlight_message_data traffic_message;
	my_data msg;
	my_reply reply;

	msg.ClientID = 600; // unique number for this client (optional)

	int server_coid;
	int index = 0;

	printf("  ---> Trying to connect to server named: %s\n", TRAFFICLIGHT_ATTACH_POINT);
	if ((server_coid = name_open(TRAFFICLIGHT_ATTACH_POINT, 0)) == -1)
	{
		printf("\n    ERROR, could not connect to server!\n\n");
		return EXIT_FAILURE;
	}

	printf("Connection established to: %s\n", TRAFFICLIGHT_ATTACH_POINT);

	// We would have pre-defined data to stuff here
	msg.hdr.type = 0x00;
	msg.hdr.subtype = 0x00;

	// Do whatever work you wanted with server connection
	for (index = 0; index < 100000; index++) // send data packets
	{
		// set up data packet
		msg.data = 10; //+index;

		// the data we are sending is in msg.data
		printf("Client (ID:%d), sending data packet with the integer value: %d \n", msg.ClientID, msg.data);
		fflush(stdout);

		if (MsgSend(server_coid, &msg, sizeof(msg), &reply, sizeof(reply)) == -1)
		{
			printf(" Error data '%d' NOT sent to server\n", msg.data);
			// maybe we did not get a reply from the server
			break;
		}
		else
		{ // now process the reply
			printf("   -->Reply is: '%s'\n", reply.buf);
		}

		sleep(5);	// wait a few seconds before sending the next data packet

		printTrafficlightStatus(reply.buf);
	}

	// Close the connection

	printf("\n Sending message to server to tell it to close the connection\n");
	name_close(server_coid);

	return EXIT_SUCCESS;
}

void printTrafficlightStatus(char* data)
{

	if (strcmp(data, "SENSOR_MODE") == 0)
	{
		CurrentMode = sensor;
		puts("SENSOR_MODE");
	}
	else if (strcmp(data, "TIMER_MODE") == 0)
	{
		CurrentMode = timer;
		puts("TIMER_MODE");
	}
	else if (strcmp(data, "I1N_MODE") == 0)
	{
		CurrentMode = override_I1N;
		puts("I1N_MODE");
	}
	else if (strcmp(data, "I1S_MODE") == 0)
	{
		CurrentMode = override_I1S;
		puts("I1S_MODE");
	}
	else if (strcmp(data, "I2N_MODE") == 0)
	{
		CurrentMode = override_I2N;
		puts("I2N_MODE");
	}
	else if (strcmp(data, "I2S_MODE") == 0)
	{
		CurrentMode = override_I2S;
		puts("I2S_MODE");
	}
	else if (strcmp(data, "W_MODE") == 0)
	{
		CurrentMode = override_W;
		puts("W_MODE");
	}
	else if (strcmp(data, "E_MODE") == 0)
	{
		CurrentMode = override_E;
		puts("E_MODE");
	}
	else if (strcmp(data, "RAIL_MODE") == 0)
	{
		CurrentMode = override_Rail;
		puts("RAIL_MODE");
	}
	else if (strcmp(data, "fault") == 0)
	{
		CurrentMode = fault;
		puts("fault");
	}
	else
	{
		puts("Something is not right");
	}

}

void selectMode()
{

	switch (CurrentMode)
	{
	case sensor:
		sensormode();
		break;

	case timer:
		timermode();
		break;

	case override_I1N:
		overrideI1N();
		break;

	case override_I1S:
		overrideI1S();
		break;

	case override_I2N:
		overrideI2N();
		break;

	case override_I2S:
		overrideI2S();
		break;

	case override_W:
		overrideW();
		break;

	case override_E:
		overrideE();
		break;

	case override_Rail:
		overrideRail();
		break;

	case peak:
		peakmode();
		break;
	case fault:
		faultmode();
		break;
	default:
		sensormode();
		break;

	}
}



//********************************************MAIN************************************************
//void printState(enum e_STATE state);
//void* get(void* data);


//int main( int argc, char *argv[] )
//{
//    pthread_t th1;
//    void *retval;
//    pthread_attr_t th1_attr;
//    //enum modes CurrentMode = sensor;  // starting the sensor at sensor mode
//
//    pthread_t th2;
//    void *retval2;
//    pthread_attr_t th2_attr;
//
//	// Creates thread and thread attributes for server
//	pthread_t client_thread;
//	pthread_attr_t client_thread_attr;
//	struct sched_param client_thread_param;
//	pthread_attr_init(&client_thread_attr);
//	client_thread_param.sched_priority = 20;
//	pthread_attr_setschedparam(&client_thread_attr, &client_thread_param);
//
//    pthread_create(&th1, NULL, KEYPAD_THREAD, NULL);
//
//
//
//    printf("##########################################################\n");
//    printf("#                                                        #\n");
//    printf("#                Major Project Group 4                   #\n");
//    printf("#                                                        #\n");
//    printf("#             Traffic Lights State Machine               #\n");
//    printf("#                                                        #\n");
//    printf("##########################################################\n");
//
//    printf("Traffic Lights State Machine\n");
//
//
//	printf("This is A Client running\n");
//
//	//pthread_create(&client_thread, &client_thread_attr, getTrafficLightmode, NULL);
//	getTrafficLightmode(TRAFFICLIGHT_ATTACH_POINT);
//	//getTrafficLightmode(TRAFFICLIGHT_ATTACH_POINT);
//	selectMode();
//}

int main(int argc, char* argv[]) {

	printf("##########################################################\n");
	printf("#                                                        #\n");
	printf("#                Major Project Group 4                   #\n");
	printf("#                                                        #\n");
	printf("#             Traffic Lights State Machine               #\n");
	printf("#                                                        #\n");
	printf("##########################################################\n");




	// Creates thread and thread attributes for getmode
	pthread_t keypad_thread;
	pthread_attr_t keypad_thread_attr;
	struct sched_param keypad_thread_param;
	pthread_attr_init(&keypad_thread_attr);
	keypad_thread_param.sched_priority = 10;
	pthread_attr_setschedparam(&keypad_thread_attr, &keypad_thread_param);

	// Creates thread and thread attributes for print mode
	pthread_t modeselect_thread;
	pthread_attr_t modeselect_thread_attr;
	struct sched_param modeselect_thread_param;
	pthread_attr_init(&modeselect_thread_attr);
	modeselect_thread_param.sched_priority = 8;
	pthread_attr_setschedparam(&modeselect_thread_attr, &modeselect_thread_param);

	// Creates thread and thread attributes for server
	pthread_t client_thread;
	pthread_attr_t client_thread_attr;
	struct sched_param client_thread_param;
	pthread_attr_init(&client_thread_attr);
	client_thread_param.sched_priority = 5;
	pthread_attr_setschedparam(&client_thread_attr, &client_thread_param);





	// Starts traffic light control thread.
	pthread_create(&keypad_thread, &keypad_thread_attr, KEYPAD_THREAD, NULL);
	// Starts traffic light control thread.
	//pthread_create(&modeselect_thread, &modeselect_thread_attr, selectMode, NULL);
	//data share message thread
	pthread_create(&client_thread, &client_thread_attr, getTrafficLightmode, NULL);
	//	printf("This is A Client running\n");

		//printmode(CurrentMode);
	printf("Client running\n");
	selectMode();


	void* ret;
	pthread_join(keypad_thread, &ret);
	//pthread_join(modeselect_thread, &ret);
	pthread_join(client_thread, &ret);

	printf("Main (Server) Terminated....\n");
	return ret;
}
//*****************************************************************************************************



