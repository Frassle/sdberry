// Access from ARM Running Linux

#define BCM2708_PERI_BASE        0x3F000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */


#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sched.h>
#include <time.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#define BLOCK_SIZE (4*1024)

int  mem_fd;
void *gpio_map;

// I/O access
volatile unsigned *gpio;

int hiPri (const int pri)
{
	struct sched_param sched ;
	memset (&sched, 0, sizeof(sched)) ;

	if (pri > sched_get_priority_max (SCHED_RR)) {
		sched.sched_priority = sched_get_priority_max (SCHED_RR);
	} else {
		sched.sched_priority = pri;
	}

	return sched_setscheduler (0, SCHED_RR, &sched);
}

unsigned int micros (void)
{
	struct timespec ts ;
	clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ;
	uint64_t now  = (uint64_t)ts.tv_sec * (uint64_t)1000000 + (uint64_t)(ts.tv_nsec / 1000) ;
	return (uint32_t)now;
}

void delayMicroseconds (unsigned int howLong)
{
	unsigned int deadline = micros() + howLong;
	while(micros() <= deadline) {

	}
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

// Cross method clock high flag
int clock_high = 0;

// Relative Card Address register
uint16_t RCA = 0;

// Card status register
struct {
	uint32_t reserved6 : 2;
	uint32_t reserved5 : 1;
	uint32_t AKE_SEQ_ERROR : 1;
	uint32_t reserved4 : 1;
	uint32_t APP_CMD : 1;
	uint32_t FX_EVENT : 1;
	uint32_t reserved3 : 1;
	uint32_t READY_FOR_DATA : 1;
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
	uint32_t ERASE_RESET : 1;
	uint32_t CARD_ECC_DISABLED : 1;
	uint32_t WP_ERASE_SKIP : 1;
	uint32_t CSD_OVERWRITE : 1;
	uint32_t reserved2 : 1;
	uint32_t reserved1 : 1;
	uint32_t ERROR : 1;
	uint32_t CC_ERROR : 1;
	uint32_t CARD_ECC_FAILED : 1;
	uint32_t ILLEGAL_COMMAND : 1;
	uint32_t COM_CRC_ERROR : 1;
	uint32_t LOCK_UNLOCK_FAILED : 1;
	uint32_t CARD_IS_LOCKED : 1;
	uint32_t WP_VIOLATION : 1;
	uint32_t ERASE_PARAM : 1;
	uint32_t ERASE_SEQ_ERROR : 1;
	uint32_t BLOCK_LEN_ERROR : 1;
	uint32_t ADDRESS_ERROR : 1;
	uint32_t OUT_OF_RANGE : 1;
} CSR;

// Card IDentification register
struct {
	// Low word
	uint64_t not_used : 1;
	uint64_t CRC : 7;
	uint64_t MDT : 12;
	uint64_t reserved : 4;
	uint64_t PSN : 32;
	uint64_t PRV : 8;
	// High word
	uint64_t PNM : 40;
	uint64_t OID : 16;
	uint64_t MID : 8;
} CID;

// Card-Specific Data register
struct {
	uint64_t not_used : 1;
	uint64_t CRC : 7;
	uint64_t reserved1 : 2;
	uint64_t FILE_FORMAT : 2;
	uint64_t TMP_WRITE_PROTECT : 1;
	uint64_t PERM_WRITE_PROTECT : 1;
	uint64_t COPY : 1;
	uint64_t FILE_FORMAT_GRP : 1;
	uint64_t reserved2 : 5;
	uint64_t WRITE_BL_PARTIAL : 1;
	uint64_t WRITE_BL_LEN : 4;
	uint64_t R2W_FACTOR : 3;
	uint64_t reserved3 : 2;
	uint64_t WP_GRP_ENABLE : 1;
	uint64_t WP_GRP_SIZE : 7;
	uint64_t SECTOR_SIZE : 7;
	uint64_t ERASE_BLK_EN : 1;
	uint64_t C_SIZE_MULT : 3;
	uint64_t VDD_W_CURR_MAX : 3;
	uint64_t VDD_W_CURR_MIN : 3;
	uint64_t VDD_R_CURR_MAX : 3;
	uint64_t VDD_R_CURR_MIN : 3;
	uint64_t C_SIZE : 12;
	uint64_t reserved4 : 2;
	uint64_t DSR_IMP : 1;
	uint64_t READ_BLK_MISALIGN : 1;
	uint64_t WRITE_BLK_MISALIGN : 1;
	uint64_t READ_BL_PARTIAL : 1;
	uint64_t READ_BL_LEN : 4;
	uint64_t CCC : 12;
	uint64_t TRAN_SPEED : 8;
	uint64_t NSAC : 8;
	uint64_t TAAC : 8;
	uint64_t reserved5 : 6;
	uint64_t CSD_STRUCTURE : 2;
} CSDV1;

struct {
	uint64_t not_used : 1;
	uint64_t CRC : 7;
	uint64_t reserved1 : 2;
	uint64_t FILE_FORMAT : 2;
	uint64_t TMP_WRITE_PROTECT : 1;
	uint64_t PERM_WRITE_PROTECT : 1;
	uint64_t COPY : 1;
	uint64_t FILE_FORMAT_GRP : 1;
	uint64_t reserved2 : 5;
	uint64_t WRITE_BL_PARTIAL : 1;
	uint64_t WRITE_BL_LEN : 4;
	uint64_t R2W_FACTOR : 3;
	uint64_t reserved3 : 2;
	uint64_t WP_GRP_ENABLE : 1;
	uint64_t WP_GRP_SIZE : 7;
	uint64_t SECTOR_SIZE : 7;
	uint64_t ERASE_BLK_EN : 1;
	uint64_t reserved4 : 1;
	uint64_t C_SIZE : 22;
	uint64_t reserved5 : 6;
	uint64_t DSR_IMP : 1;
	uint64_t READ_BLK_MISALIGN : 1;
	uint64_t WRITE_BLK_MISALIGN : 1;
	uint64_t READ_BL_PARTIAL : 1;
	uint64_t READ_BL_LEN : 4;
	uint64_t CCC : 12;
	uint64_t TRAN_SPEED : 8;
	uint64_t NSAC : 8;
	uint64_t TAAC : 8;
	uint64_t reserved6 : 6;
	uint64_t CSD_STRUCTURE : 2;
} CSDV2;

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

uint8_t crc7_bytes(uint8_t* bytes, uint8_t count) {
	uint8_t crc = 0;
	do {
		crc = CRCTable[(crc << 1) ^ bytes[--count]];
	} while (count > 0);
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
		return 0;
	} else {
		return 1;
	}
}

void setupCID() {
	memset(&CID, 0, sizeof(CID));

	// July 2018
	CID.MDT = (18 << 4) | 6;
	CID.PSN = 0xDEADBEEF;
	// 1.0
	CID.PRV = (1 << 4);
	CID.PNM = 
		((uint64_t)'W' << (8 * 4)) |
		((uint64_t)'y' << (8 * 3)) |
		((uint64_t)'a' << (8 * 2)) |
		((uint64_t)'t' << (8 * 1)) |
		((uint64_t)'t' << (8 * 0));
	CID.OID = 0x0;
	// Pretend we're sandisk
	CID.MID = 0x2;
	CID.not_used = 1;

	uint8_t crc = crc7_bytes(((uint8_t*)&CID) + 1, 15);
	CID.CRC = crc;
}

void setupCSR() {
	memset(&CSR, 0, sizeof(CSR));
}

void setupCSD() {
	memset(&CSDV2, 0, sizeof(CSDV2));

	CSDV2.CSD_STRUCTURE = 1;
	CSDV2.TAAC = 0x0E;
	CSDV2.NSAC = 0;
	CSDV2.TRAN_SPEED = 0x32;
	CSDV2.CCC = 0x1;
	CSDV2.READ_BL_LEN = 0x9;
	CSDV2.READ_BL_PARTIAL = 0;
	CSDV2.WRITE_BLK_MISALIGN = 0;
	CSDV2.READ_BLK_MISALIGN = 0;
	CSDV2.DSR_IMP = 0;
	// About 1GB
	CSDV2.C_SIZE = 1999;
	CSDV2.ERASE_BLK_EN = 1;
	CSDV2.SECTOR_SIZE = 0x7F;
	CSDV2.WP_GRP_SIZE = 0;
	CSDV2.WP_GRP_ENABLE = 0;
	CSDV2.R2W_FACTOR = 2;
	CSDV2.WRITE_BL_LEN = 0x9;
	CSDV2.WRITE_BL_PARTIAL = 0;
	CSDV2.FILE_FORMAT_GRP = 0;
	CSDV2.COPY = 0;
	CSDV2.PERM_WRITE_PROTECT = 0;
	CSDV2.TMP_WRITE_PROTECT = 0;
	CSDV2.FILE_FORMAT = 0;

	uint8_t crc = crc7_bytes(((uint8_t*)&CSDV2) + 1, 15);
	CSDV2.CRC = crc;
}


uint64_t wait_clocks(int wait) {
	pinMode(20, OUTPUT);
	digitalWrite(20, 1);

	uint64_t hz = 0;
	while(wait > 0) { 
		int gpioreg = *(gpio + 13);
		int clk = (gpioreg & (1 << 21)) != 0;

		if(!clock_high && clk) { 
			++hz;
			clock_high = 1;
			--wait;
		} else if (clock_high && !clk) {
			clock_high = 0;
		}
	}
	return hz;
}


uint64_t send_r(uint64_t response, uint64_t bits) {
	uint64_t hz = 0;
	uint64_t mask = (1LL << (bits - 1));
	while(bits > 0) { 
		int gpioreg = *(gpio + 13);
		int clk = (gpioreg & (1 << 21)) != 0;

		if(!clock_high && clk) { 
			++hz;
			clock_high = 1;
		} else if (clock_high && !clk) {
			clock_high = 0;
			int bit = (response & mask) != 0;
			digitalWrite(20, bit);
			response <<= 1;
			--bits;
		}
	}
	return hz;
}

uint64_t send_r1(int cmdindex) {
	uint64_t hz = 0; //wait_clocks(2);

	uint64_t response = (uint64_t)cmdindex << (40 - 8);
	response |= *((uint32_t*)&CSR);
	uint8_t crc = crc7(response);
	response <<= 8;
	response |= (crc << 1);
	response |= 1;
	hz += send_r(response, 48);
	printf("Sent R1 (%#llx)\n", response);
	return hz;
}

uint64_t send_r2(uint64_t *reg) {
	uint64_t hz = 0; //wait_clocks(5);

	// 00111111 = 0x3F
	hz += send_r(0x3F, 8);
	hz += send_r(*(reg + 1), 64);
	hz += send_r(*(reg), 64);
	printf("Sent R2 (%#.16llx%.16llx)\n", *(reg + 1), *reg);
	return hz;
}

uint64_t send_r3(int ccs, int busy) {
	uint64_t hz = 0; //wait_clocks(2);

	uint64_t ocr = (1 << 20) | (1 << 21); // set 3.2-3.4V
	if (ccs) {
		ocr |= (1 << 30);
	}
	if (busy) {
		ocr |= (1 << 31);
	}
	// 001111110000000000000000000000000000000011111111 == 0x3F00000000FF
	uint64_t response = 0x3F00000000FF | (ocr << 8);
	hz += send_r(response, 48);
	printf("Sent R3 (%#llx)\n", response);
	return hz;
}

uint64_t send_r6() {
	RCA = (uint16_t)(rand() + 1);
	printf("Set RCA to %hu\n", RCA);

	// 00000011 = 0x03
	uint64_t response = (uint64_t)0x03 << 32;
	response |= ((uint64_t)RCA << 16);
	response |= (CSR.COM_CRC_ERROR << 15);
	response |= (CSR.ILLEGAL_COMMAND << 14);
	response |= (CSR.ERROR << 13);
	response |= (CSR.CURRENT_STATE << 9);
	response |= (CSR.READY_FOR_DATA << 8);
	response |= (CSR.FX_EVENT << 6);
	response |= (CSR.APP_CMD << 5);
	response |= (CSR.AKE_SEQ_ERROR << 3);

	uint8_t crc = crc7(response);
	response <<= 8;
	response |= (crc << 1);
	response |= 1;
	uint64_t hz = send_r(response, 48);
	printf("Sent R6 (%#llx)\n", response);
	return hz;
}

uint64_t send_r7(uint32_t voltage, uint32_t pattern) {
	uint64_t hz = 0; //wait_clocks(2);

	// 1001000000000000000000000000000 == 0x48000000
	uint64_t response = 0x48000000;
	response |= (voltage << 8);
	response |= pattern;
	uint8_t crc = crc7(response);
	response <<= 8;
	response |= (crc << 1);
	response |= 1;
	hz += send_r(response, 48);
	printf("Sent R7 (%#llx)\n", response);
	return hz;
}

int main(int argc, char **argv)
{
	generateCRCTable();
	printf("Tests\n");

	// 0100000000000000000000000000000000000000 = 1001010
	printf("crc7(%#llx) = %#hhx = %#x\n", 0x4000000000, crc7(0x4000000000), 0x4A);
	uint64_t test = 0x4000000000;
	printf("crc7_bytes(%#llx) = %#hhx = %#x\n", 0x4000000000, crc7_bytes((uint8_t*)&test, 5), 0x4A);
	// 0101000100000000000000000000000000000000 = 0101010
	printf("crc7(%#llx) = %#hhx = %#x\n", 0x5100000000, crc7(0x5100000000), 0x2A);
	// 0001000100000000000000000000100100000000 = 0110011
	printf("crc7(%#llx) = %#hhx = %#x\n", 0x1100000900, crc7(0x1100000900), 0x33);

	printf("Raspberry Pi SD Card\n");
	hiPri(99);
	setup_io();

	// Clock tracker
	int begin = -1;
	uint64_t hz = 0;

reset:
	RCA = 0;
	setupCID();
	setupCSR();
	setupCSD();

	pinMode(16, INPUT);
	pinMode(19, OUTPUT);
	pinMode(20, INPUT);
	pinMode(21, INPUT);
	pinMode(26, INPUT);

	digitalWrite(19, LOW);
	pullUpDnControl(16, PUD_DOWN);
	pullUpDnControl(19, PUD_OFF);
	pullUpDnControl(20, PUD_OFF);
	pullUpDnControl(21, PUD_OFF);
	pullUpDnControl(26, PUD_UP);

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
			goto reset;
		}

		if (!clock_high && clk) {
			if (begin == -1) { begin = micros(); }
			++hz;
			clock_high = 1;

			if (start) {
				word <<= 1;
				word |= cmd;
				++bits;

				if(bits == 48 && checkcrc(word)) {
					pinMode(20, OUTPUT);
					int cmdindex = (word >> 40) & 0x3F;

					// Application commands
					if (CSR.APP_CMD && cmdindex == 41) {
						printf("Got ACMD41 (SD_SEND_OP_COND)\n");
						if (CSR.CURRENT_STATE != 0) {
							printf("Illegal command, in state %llu\n", CSR.CURRENT_STATE);
							CSR.ILLEGAL_COMMAND = 1;
						} else {
							printf("%#llx\n", (word >> 8) & 0xFFFFFFFF);
							// Just gonna assume requested features/voltage are ok.
							CSR.CURRENT_STATE = 1;
							hz += send_r3(1, 1);

							CSR.APP_CMD = 0;
							CSR.ILLEGAL_COMMAND = 0;
						}
					} 
					
					// Normal commands	
					else if (cmdindex == 0) {
						printf("Got CMD0 (GO_IDLE_STATE)\ncds=%d\n", cds);
						goto reset;
					} else if (cmdindex == 1) {
						CSR.ILLEGAL_COMMAND = 1;
						printf("Got CMD1 (SEND_OP_COND)\n");
						printf("Don't reply to CMD1, not an MMC card\n");
					} else if (cmdindex == 2) {
						printf("Got CMD2 (ALL_SEND_CID)\n");

						if (CSR.CURRENT_STATE != 1) { 
							CSR.ILLEGAL_COMMAND = 1;
						} else {
							CSR.APP_CMD = 0;
							CSR.CURRENT_STATE = 2;

							hz += send_r2((uint64_t*)&CID);

							CSR.ILLEGAL_COMMAND = 0;
						}
					} else if (cmdindex == 3) {
						printf("Got CMD3 (SEND_RELATIVE_ADDR)\n");

						if (CSR.CURRENT_STATE != 2) { 
							CSR.ILLEGAL_COMMAND = 1;
						} else {
							CSR.APP_CMD = 0;
							CSR.CURRENT_STATE = 3;
							hz += send_r6();
							CSR.ILLEGAL_COMMAND = 0;
						}
					} else if (cmdindex == 8) {
						CSR.APP_CMD = 0;

						printf("Got CMD8 (SEND_IF_COND)\n");
						uint64_t voltage = (word >> 16) & 0xF;
						uint64_t pattern = (word >> 8) & 0xFF;
						printf("voltage = %#llx\n", voltage);
						printf("pattern = %#llx\n", pattern);
						if (voltage == 1) {
							hz += send_r7(voltage, pattern);
						} else {
							printf("voltage invalid, not responding\n");
						}
						CSR.ILLEGAL_COMMAND = 0;
					} else if (cmdindex == 9) {
						printf("Got CMD9  (SEND_CSD)\n");
						uint16_t rca = (word >> (8 + 16));
						printf("RCA = %"PRIu16"\n", rca);

						if(rca == RCA) {
							CSR.APP_CMD = 0;
							hz += send_r2((uint64_t*)&CSDV2);
							CSR.ILLEGAL_COMMAND = 0;
						}

					} else if (cmdindex == 55) {
						printf("Got CMD55 (APP_CMD)\n");
						uint16_t rca = (word >> (8 + 16));
						printf("RCA = %"PRIu16"\n", rca);

						if(rca == RCA) {
							CSR.APP_CMD = 1;
							hz += send_r1(55);
							CSR.ILLEGAL_COMMAND = 0;
						}
					} else {
						printf("Unknown command!\n");
						printf("%#llx\n", word);
						break;
					}
					
					// Clear crc error
					CSR.COM_CRC_ERROR = 0;

					pinMode(20, INPUT);
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

		} else if (clock_high && !clk) {
			clock_high = 0;
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


