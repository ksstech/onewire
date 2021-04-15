/*
 * Copyright 2017-21 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma		once

#include	"hal_i2c.h"
#include	"ds248x.h"

#include	<stddef.h>
#include	<stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* https://www.maximintegrated.com/en/products/ibutton/software/1wire/wirekit.cfm
 * https://www.maximintegrated.com/en/app-notes/index.mvp/id/74
 *
 * ONEWIRE commands take a couple of formats resulting in a range of durations
 * 1. Instantaneous (0uS), no 1W bus activity, only affect DS2484, optional status
 * 2. Fast (< 1uS), no 1W bus activity, only affect DS2484,
 * 3. Medium (1uS < ?? < 1mS)
 * 4. Slow (> 1mS)
 * In order to optimise system performance minimal time should be spent in a tight
 * loop waiting for status, task should yield (delay) whenever possible.
 *				[DRST]	[SRP]	[WCFG]	[CHSL]	1WRST	1WWB	1WRB	1WSB	1WT
 *	Duration	525nS	0nS		0nS		0nS		1244uS	8x73uS	8x73uS	1x73uS	3x73uS
 */

// ############################################# Macros ############################################

#define	owPLATFORM_MAXCHAN			9

// ################################## Generic 1-Wire Commands ######################################

#define OW_CMD_SEARCHROM     		0xF0
#define OW_CMD_SEARCHALARM   		0xEC
#define OW_CMD_SKIPROM       		0xCC
#define OW_CMD_MATCHROM      		0x55
#define OW_CMD_READROM       		0x33

// ##################################### iButton Family Codes #####################################

#define	OWFAMILY_01		0x01			// (DS1990A), (DS1990R), DS2401, DS2411	1-Wire net address (registration number) only
#define	OWFAMILY_02		0x02			// (DS1991)�	Multikey iButton, 1152-bit secure memory
#define	OWFAMILY_04		0x04			// (DS1994), DS2404	4Kb NV RAM memory and clock, timer, alarms
#define	OWFAMILY_05		0x05			// DS2405�	Single addressable switch
#define	OWFAMILY_06		0x06			// (DS1993)	4Kb NV RAM memory
#define	OWFAMILY_08		0x08			// (DS1992)	1Kb NV RAM memory
#define	OWFAMILY_09		0x09			// (DS1982), DS2502	1Kb EPROM memory
#define	OWFAMILY_0A		0x0A			// (DS1995)	16Kb NV RAM memory
#define	OWFAMILY_0B		0x0B			// (DS1985), DS2505	16Kb EPROM memory
#define	OWFAMILY_0C		0x0C			// (DS1996)	64Kb NV RAM memory
#define	OWFAMILY_OF		0x0F			// (DS1986), DS2506	64Kb EPROM memory
#define	OWFAMILY_10		0x10			// (DS1820), DS18S20 Temperature with alarm trips
#define	OWFAMILY_12		0x12			// DS2406, DS2407�	1Kb EPROM memory, 2-channel addressable switch
#define	OWFAMILY_14		0x14			// (DS1971), DS2430A�	256-bit EEPROM memory and 64-bit OTP register
#define	OWFAMILY_1A		0x1A			// (DS1963L)�	4Kb NV RAM memory with write cycle counters
#define	OWFAMILY_1C		0x1C			// DS28E04-100	4096-bit EEPROM memory, 2-channel addressable switch
#define	OWFAMILY_1D		0x1D			// DS2423�	4Kb NV RAM memory with external counters
#define	OWFAMILY_1F		0x1F			// DS2409�	2-channel addressable coupler for sub-netting
#define	OWFAMILY_20		0x20			// DS2450	4-channel A/D converter (ADC)
#define	OWFAMILY_21		0x21			// (DS1921G), (DS1921H), (DS1921Z)	Thermochron� temperature logger
#define	OWFAMILY_23		0x23			// (DS1973), DS2433	4Kb EEPROM memory
#define	OWFAMILY_24		0x24			// (DS1904), DS2415	Real-time clock (RTC)
#define	OWFAMILY_27		0x27			// DS2417	RTC with interrupt
#define	OWFAMILY_28		0x28			// DS18B20 (9-12 bit programmable) Thermometer
#define	OWFAMILY_29		0x29			// DS2408	8-channel addressable switch
#define	OWFAMILY_2C		0x2C			// DS2890�	1-channel digital potentiometer
#define	OWFAMILY_2D		0x2D			// (DS1972), DS2431	1024-bit, 1-Wire EEPROM
#define	OWFAMILY_37		0x37			// (DS1977)	Password-protected 32KB (bytes) EEPROM
#define	OWFAMILY_3A		0x3A			// (DS2413)	2-channel addressable switch
#define	OWFAMILY_41		0x41			// (DS1922L/T), (DS1923), DS2422 High-capacity Thermochron (temperature) and Hygrochron� (humidity) loggers
#define	OWFAMILY_42		0x42			// DS28EA00	Programmable resolution digital thermometer with sequenced detection and PIO
#define	OWFAMILY_43		0x43			// DS28EC20	20Kb 1-Wire EEPROM

// ######################################## Enumerations ###########################################


// Intention is to remove this and use the i2cDEV_TYPE definitions to determine the device detected.
//		DS248x			ESP32 RMT    GPIO Software
enum { owTYPE_DS248X, owTYPE_RMTXXX, owTYPE_GPIOSW,	owTYPE_MAXNUM } ;
enum { owSPEED_STANDARD, owSPEED_ODRIVE	} ;
enum { owPOWER_STANDARD, owPOWER_STRONG	} ;
enum { owFAM28_RES9B, owFAM28_RES10B, owFAM28_RES11B, owFAM28_RES12B } ;

// ######################################### Structures ############################################

typedef	struct __attribute__((packed)) onewire_s {
	ow_rom_t		ROM ;								// size = 1+6+1
	uint8_t 		LastDiscrepancy ;
	uint8_t 		LastFamilyDiscrepancy ;
	uint8_t 		crc8 ;
	uint8_t 		LastDeviceFlag	: 1 ;
	uint8_t			BusType			: 2 ;				// owTYPE_XXXXXX
	uint8_t			DevNum			: 2 ;				// index into 1W DevInfo table
	uint8_t			PhyChan			: 3 ;
} onewire_t ;
DUMB_STATIC_ASSERT(sizeof(onewire_t) == 12) ;

/* PER CHANNEL info keeping track of last device read (ROM & timestamp) on each channel
 * Used to avoid re-reading a device (primarily DS1990X type) too regularly.
 */
typedef struct __attribute__((packed)) ow_chan_info_s {
	ow_rom_t	LastROM ;
	seconds_t	LastRead ;
	union {
		struct {
			uint8_t		ds18b20	: 4 ;
			uint8_t		ds18s20	: 4 ;
			uint8_t		ds18xxx	: 4 ;
			uint8_t		spare	: 4 ;
		} ;
		uint16_t	ds18any ;
	} ;
} ow_chan_info_t ;
DUMB_STATIC_ASSERT(sizeof(ow_chan_info_t) == 14) ;

typedef struct ow_flags_s {
	uint8_t	Level 	: 2 ;
	uint8_t	Spare	: 6 ;
} ow_flags_t ;

// ################################ Generic 1-Wire LINK API's ######################################

int		OWSetSPU(onewire_t * psOW) ;
int		OWReset(onewire_t * psOW) ;
int		OWSpeed(onewire_t * psOW, bool speed) ;
int		OWLevel(onewire_t * psOW, bool level) ;
uint8_t	OWCheckCRC(uint8_t * buf, uint8_t buflen) ;
uint8_t	OWCalcCRC8(onewire_t * psOW, uint8_t data) ;
uint8_t	OWSearchTriplet(onewire_t * psOW, uint8_t search_direction) ;
int		OWChannelSelect(onewire_t * psOW) ;

// ################################## Bit/Byte Read/Write ##########################################

uint8_t OWTouchBit(onewire_t * psOW, uint8_t sendbit) ;
void	OWWriteBit(onewire_t * psOW, uint8_t sendbit) ;
uint8_t OWReadBit(onewire_t * psOW) ;
void	OWWriteByte(onewire_t * psOW, uint8_t sendbyte) ;
int		OWWriteBytePower(onewire_t * psOW, int sendbyte) ;
int		OWReadBitPower(onewire_t * psOW, uint8_t applyPowerResponse) ;
uint8_t	OWReadByte(onewire_t * psOW) ;
uint8_t OWTouchByte(onewire_t * psOW, uint8_t sendbyte) ;
void	OWBlock(onewire_t * psOW, uint8_t * tran_buf, int32_t tran_len) ;
int		OWReadROM(onewire_t * psOW) ;
void	OWAddress(onewire_t * psOW, uint8_t nAddrMethod) ;

// ############################## Search and Variations thereof ####################################

void	OWTargetSetup(onewire_t * psOW, uint8_t family_code) ;
void	OWFamilySkipSetup(onewire_t * psOW) ;
int		OWSearch(onewire_t * psOW, bool alarm_only) ;
int 	OWFirst(onewire_t * psOW, bool alarm_only) ;
int 	OWNext(onewire_t * psOW, bool alarm_only) ;
int		OWVerify(onewire_t * psOW) ;

#ifdef __cplusplus
}
#endif
