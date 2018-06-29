// Access from ARM Running Linux

#define BCM2708_PERI_BASE        0x3F000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */


#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
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

// gpio 36 / bcm 16 / pin 4 / strand 10 = power
// gpio 35 / bcm 19 / pin 7 / strand 12 = miso | data0 
// gpio 38 / bcm 20 / pin 3 / strand 6  = mosi | cmd/res 
// gpio 40 / bcm 21 / pin 5 / strand 10 = sclk
// gpio 37 / bcm 26 / pin 2 / strand 4  = card select/detection

// ============================
// END OF GPIO, START OF SDCARD
// ============================

// Card status register
struct {
	uint32_t OUT_OF_RANGE : 1;
	uint32_t ADDRESS_ERROR : 1;
	uint32_t BLOCK_LEN_ERROR : 1;
	uint32_t ERASE_SEQ_ERROR : 1;
	uint32_t ERASE_PARAM : 1;
	uint32_t WP_VIOLATION : 1;
	uint32_t CARD_IS_LOCKED : 1;
	uint32_t LOCK_UNLOCK_FAILED : 1;
       	uint32_t COM_CRC_ERROR : 1;
	uint32_t ILLEGAL_COMMAND : 1;
	uint32_t CARD_ECC_FAILED : 1;
	uint32_t CC_ERROR : 1;
	uint32_t reserved1 : 1;
	uint32_t reserved2 : 1;
	uint32_t CSD_OVERWRITE : 1;
	uint32_t WP_ERASE_SKIP : 1;
	uint32_t CARD_ECC_DISABLED : 1;
	uint32_t ERASE_RESET : 1;
	// 0 == idle
	// 1 == ready
	// 2 == ident
	// 3 == stby
	// 4 == tran
	// 5 == data
	// 6 == rcv
	// 7 == prg
	// 8 == dis
	// 9-15 == reserved
	uint32_t CURRENT_STATE : 4;
	uint32_t READY_FOR_DATA : 1;
	uint32_t reserved3 : 1;
	uint32_t FX_EVENT : 1;
	uint32_t APP_CMD : 1;
	uint32_t reserved4 : 1;
	uint32_t AKE_SEQ_ERROR : 1;
	uint32_t reserved5 : 1;
	uint32_t reserved6 : 2;
} CSR;

uint8_t CRCTable[256];
 
void generateCRCTable()
{
	int i, j;
	uint8_t CRCPoly = 0x89;  // the value of our CRC-7 polynomial

	// generate a table value for all 256 possible byte values
	for (i = 0; i < 256; ++i) {
		CRCTable[i] = (i & 0x80) ? i ^ CRCPoly : i;
		for (j = 1; j < 8; ++j) {
			CRCTable[i] <<= 1;
			if (CRCTable[i] & 0x80) {
			    CRCTable[i] ^= CRCPoly;
			}
		}
	}
}

uint8_t crc7(uint64_t word) {
	uint8_t crc = 0;
	crc = CRCTable[(crc << 1) ^ ((word >> 8 * 5) & 0xFF)];
	crc = CRCTable[(crc << 1) ^ ((word >> 8 * 4) & 0xFF)];
	crc = CRCTable[(crc << 1) ^ ((word >> 8 * 3) & 0xFF)];
	crc = CRCTable[(crc << 1) ^ ((word >> 8 * 2) & 0xFF)];
	crc = CRCTable[(crc << 1) ^ ((word >> 8 * 1) & 0xFF)];
	crc = CRCTable[(crc << 1) ^ ((word >> 8 * 0) & 0xFF)];
	return crc;
}

int checkcrc(uint64_t word) {
	uint8_t crc = (word & 0xFF) >> 1;
	uint64_t message = word >> 8;
	uint8_t real_crc = crc7(message);
	if(real_crc!= crc) {
		printf("Expected crc %x, got crc %x!\n", crc, real_crc);
		printf("%#llx\n", word);
		CSR.COM_CRC_ERROR = 1;
	} else {
		CSR.COM_CRC_ERROR = 0;
	}
}

uint64_t send_r(uint64_t response) {
	pinMode(20, OUTPUT);

	uint64_t hz = 0;
	int high = 0;
	int bits = 0;
	while(bits != 48) { 
		int gpioreg = *(gpio + 13);
		int clk = (gpioreg & (1 << 21)) != 0;

		if(!high && clk) { 
			++hz;
			high = 1;
		} else if (high && !clk) {
			high = 0;
			int bit = (response & 0x800000000000) != 0;
			response <<= 1;
			++bits;
			digitalWrite(20, bit);
		}
	}
	pinMode(20, INPUT);
	return hz;
}


uint64_t send_r3() {
	uint64_t r3 = 0x3F40300000FF;
	uint64_t hz = send_r(r3);
	printf("Sent R3\n");
	return hz;
}


uint64_t send_r1(int cmdindex) {
	uint64_t response = (uint64_t)cmdindex << 32;
	response |= *((uint32_t*)&CSR);
	uint8_t crc = crc7(response);
	response <<= 8;
	response |= (crc << 1);
	response |= 1;
	uint64_t hz = send_r(response);
	printf("Sent R1 (%#llx)\n", response);
	return hz;
}

int main(int argc, char **argv)
{
	generateCRCTable();

	printf("Raspberry Pi SD Card\n");
	piHiPri(99);

	printf("Tests\n");
	// CMD0
	printf("CRC7(0x4000000000) == 0x4a\n");
	printf("%x\n", crc7(0x4000000000));

	printf("CRC7(0x5100000000) == 0x2a\n");
	printf("%x\n", crc7(0x5100000000));

	setup_io();

	pinMode(16, INPUT);
	pinMode(19, OUTPUT);
	pinMode(20, INPUT);
	pinMode(21, INPUT);
	pinMode(26, INPUT);

	digitalWrite(19, LOW);
	pullUpDnControl(16, PUD_DOWN);
	pullUpDnControl(19, PUD_OFF);
	pullUpDnControl(20, PUD_OFF);
	pullUpDnControl(12, PUD_OFF);
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
		int power = (gpioreg & (1 << 16)) != 0;
		int cmd = (gpioreg & (1 << 20)) != 0;
		int clk = (gpioreg & (1 << 21)) != 0;
		int cds = (gpioreg & (1 << 26)) != 0;

		if (!power) {
			high = 0;
			word = 0;
			bits = 0;
			start = 0;
			memset(&CSR, sizeof(CSR), 0);
			continue;
		}

		if (!high && clk) {
			if (begin == -1) { begin = micros(); }
			++hz;
			high = 1;

			if (start) {
				word <<= 1;
				word |= cmd;
				++bits;

				if(bits == 48) {
					checkcrc(word);

					int cmdindex = (word >> 40) & 0x3F;

					if (cmdindex == 0) {
						printf("Got CMD0 (GO_IDLE_STATE)\ncds=%d\n", cds);
						CSR.CURRENT_STATE = 0;
					} else if(cmdindex == 1) {
						printf("Got CMD1 (SEND_OP_COND)\n");
						CSR.CURRENT_STATE = 1;
						hz += send_r3();
					} else if (cmdindex == 8) {
						printf("Got CMD8 (SEND_IF_COND)\n");
						uint64_t args = (word >> 8) & 0xFFFFFFFF;
						printf("args = %#llx\n", args);
						printf("Ignore, don't reply.\n");
					} else if (cmdindex == 55) {
						printf("Got CMD55 (APP_CMD)\n");
						CSR.APP_CMD = 1;
						hz += send_r1(55);
					} else {
						printf("Unknown command!\n");
						printf("%#llx\n", word);
						break;
					}

					start = 0;
					word = 0;
					bits = 0;
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

	hz = (uint64_t)((hz * 1000000.0) / time);

	printf("%lluhz\n", hz);
	printf("%llukhz\n", hz / 1000);

	return 0;

} // main


