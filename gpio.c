// Access from ARM Running Linux

#define BCM2708_PERI_BASE        0x3F000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */


#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <wiringPi.h>

#define BLOCK_SIZE (4*1024)

int  mem_fd;
void *gpio_map;

// I/O access
volatile unsigned *gpio;

/*
 * micros:
 *      Return a number of microseconds as an unsigned int.
 *      Wraps after 71 minutes.
 *********************************************************************************
 */
unsigned int micros (void)
{
  struct timespec ts ;
  clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ;
  uint64_t now  = (uint64_t)ts.tv_sec * (uint64_t)1000000 + (uint64_t)(ts.tv_nsec / 1000) ;
  return (uint32_t)now;
}

//
// Set up a memory regions to access GPIO
//
void setup_io()
{
	/* open /dev/mem */
	if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
		printf("can't open /dev/mem \n");
		exit(-1);
	}
	
	/* mmap GPIO */
	gpio_map = mmap(
		NULL,             //Any adddress in our space will do
		BLOCK_SIZE,       //Map length
		PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
		MAP_SHARED,       //Shared with other processes
		mem_fd,           //File to map
		GPIO_BASE         //Offset to GPIO peripheral
	);
	
	close(mem_fd); //No need to keep mem_fd open after mmap
	
	if (gpio_map == MAP_FAILED) {
		printf("mmap error %d\n", (int)gpio_map);//errno also set!
		exit(-1);
	}
	
	gpio = (volatile unsigned *)gpio_map;
}

// gpioToGPFSEL:
//      Map a BCM_GPIO pin to it's Function Selection
//      control port. (GPFSEL 0-5)
//      Groups of 10 - 3 bits per Function - 30 bits per port

static uint8_t gpioToGPFSEL [] =
{
	0,0,0,0,0,0,0,0,0,0,
	1,1,1,1,1,1,1,1,1,1,
	2,2,2,2,2,2,2,2,2,2,
	3,3,3,3,3,3,3,3,3,3,
	4,4,4,4,4,4,4,4,4,4,
	5,5,5,5,5,5,5,5,5,5,
};

// gpioToShift
//      Define the shift up for the 3 bits per pin in each GPFSEL port

static uint8_t gpioToShift [] =
{
	0,3,6,9,12,15,18,21,24,27,
	0,3,6,9,12,15,18,21,24,27,
	0,3,6,9,12,15,18,21,24,27,
	0,3,6,9,12,15,18,21,24,27,
	0,3,6,9,12,15,18,21,24,27,
	0,3,6,9,12,15,18,21,24,27,
};

// gpioToGPSET:
//      (Word) offset to the GPIO Set registers for each GPIO pin

static uint8_t gpioToGPSET [] =
{
	7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
	8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
};

// gpioToGPCLR:
//      (Word) offset to the GPIO Clear registers for each GPIO pin

static uint8_t gpioToGPCLR [] =
{
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
	11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,
};

// gpioToGPLEV:
//      (Word) offset to the GPIO Input level registers for each GPIO pin

static uint8_t gpioToGPLEV [] =
{
	13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
	14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
};

#define	INPUT			 0
#define	OUTPUT			 1

void pinMode (int pin, int mode) {
	int fSel    = gpioToGPFSEL [pin] ;
	int shift   = gpioToShift  [pin] ;
	
	if (mode == INPUT) {
		*(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)); // Sets bits to zero = input
	} else if (mode == OUTPUT) {
		*(gpio + fSel) = (*(gpio + fSel) & ~(7 << shift)) | (1 << shift);
	}
}

int digitalRead(int pin) {
	return ((*(gpio + gpioToGPLEV [pin]) & (1 << (pin & 31))) != 0);
}

#define	LOW			 0
#define	HIGH			 1

void digitalWrite(int pin, int value) {
	if (value == LOW) {
		*(gpio + gpioToGPCLR [pin]) = 1 << (pin & 31) ;
	} else {
		*(gpio + gpioToGPSET [pin]) = 1 << (pin & 31) ;
	}
}

// GPPUD:
//      GPIO Pin pull up/down register

#define GPPUD   37

// gpioToPUDCLK
//      (Word) offset to the Pull Up Down Clock regsiter

static uint8_t gpioToPUDCLK [] =
{
  38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,
  39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,
} ;

#define PUD_OFF                  0
#define PUD_DOWN                 1
#define PUD_UP                   2

void pullUpDnControl (int pin, int pud) {
	*(gpio + GPPUD)              = pud & 3 ;            delayMicroseconds (5) ;
	*(gpio + gpioToPUDCLK [pin]) = 1 << (pin & 31) ;    delayMicroseconds (5) ;
	*(gpio + GPPUD)              = 0 ;                  delayMicroseconds (5) ;
	*(gpio + gpioToPUDCLK [pin]) = 0 ;                  delayMicroseconds (5) ;
}

// gpio 35 / bcm 19 / pin 7 / strand 12 = miso | data0 
// gpio 38 / bcm 20 / pin 3 / strand 6  = mosi | cmd/res 
// gpio 40 / bcm 21 / pin 5 / strand 10 = sclk
// gpio 37 / bcm 26 / pin 2 / strand 4  = card select/detection

int main(int argc, char **argv)
{
	printf ("Raspberry Pi SD Card\n") ;
	piHiPri(99);

	setup_io();

	pinMode(19, OUTPUT);
	pinMode(20, INPUT);
	pinMode(21, INPUT);
	pinMode(26, INPUT);

	digitalWrite(19, LOW);
	pullUpDnControl(20, PUD_DOWN);
	pullUpDnControl(21, PUD_DOWN);
	pullUpDnControl(26, PUD_UP);

	// clock
	int high = 0;
	uint64_t hz = 0;
	int begin = -1;

	// cmd
	uint64_t word = 0;
	int bits = 0;
	int start = 0;


	while (1) {
		int gpioreg = *(gpio + 13);
		int cmd = (gpioreg & (1 << 20)) != 0;
		int clk = (gpioreg & (1 << 21)) != 0;
		int cds = (gpioreg & (1 << 26)) != 0;

		if (!high && clk) {
			if (begin == -1) { begin = micros(); }
			++hz;
			high = 1;

			if (start) {
				word <<= 1;
				word |= cmd;
				++bits;

				if(bits == 48) {
					printf("%#llx\n", word);
					word = 0;
					bits = 0;
					break;
				}
			} else {
				if (cmd == 0) {
					start = 1;
					word = 0;
					bits = 1;
				}
			}

		} else if (high && !clk) {
			high = 0;
		}
	}

	int time = micros() - begin;
	if (begin == -1) {
		printf("No high clocks\n");
	} else {
		printf("%dus\n", time);
		printf("%dms\n", time / 1000);
	}

	hz = (hz * 1000000) / time;

	printf("%lluhz\n", hz);
	printf("%llukhz\n", hz / 1000);

	return 0;

} // main


