/*
 * Copyright 2014-18 AM Maree/KSS Technologies (Pty) Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/*
 * ds2482.h
 */

#pragma		once

#include	"hal_i2c.h"
#include	"onewire.h"

#include	<stdint.h>

// ############################################# Macros ############################################

#define	configHAL_I2C_DS2482_ADDR			0x18		// Device base address

#define	owDELAY_WB							1			// 0=Yield >0=mS Delay
// 0=NoWait 1=Yield >1=Delay(n-1)
#define	owDELAY_ST							2			// Search Triplet
#define	owDELAY_RST							2			// Bus Reset
#define	owDELAY_TB							1			// Touch Bit

// DS2482 commands
#define CMD_DRST   							0xF0		// Device Reset
#define CMD_SRP								0xE1		// Set Read Pointer
#define CMD_WCFG   							0xD2		// Write Config
#define CMD_CHSL   							0xC3		// Channel Select (-800)
#define CMD_1WRS   							0xB4		// 1-Wire Reset
#define CMD_1WWB   							0xA5		// 1-Wire Write Byte
#define CMD_1WRB   							0x96		// 1-Wire Read Byte
#define CMD_1WSB   							0x87		// 1-Wire Single Bit
#define CMD_1WT								0x78		// 1-Wire Triplet

// DS2482 config bits
#define CONFIG_APU							0x01		// Active Pull Up
#define CONFIG_PPM							0x02		// Presence Pulse Mask
#define CONFIG_SPU							0x04		// Strong Pull Up
#define CONFIG_1WS							0x08		// 1-Wire Speed (0=Standard, 1= Overdrive)

// DS2482 status register bits
#define STATUS_1WB							0x01		// 1-Wire Busy
#define STATUS_PPD							0x02		// Presence Pulse Detected
#define STATUS_SD							0x04		// Short Detected
#define STATUS_LL							0x08		// Logic Level
#define STATUS_RST							0x10		// ReSeT
#define STATUS_SBR							0x20		// Single Bit Result
#define STATUS_TSB							0x40		// Triplet Second Bit
#define STATUS_DIR							0x80		// branch DIRection taken

// ######################################## Enumerations ###########################################

enum {													// Supported 1W families & devices
	idxOWFAMILY_01,
#if		(configBUILD_WITH_DS18X20 == 1)
	idxOWFAM10_28,
#endif
	idxOWFAMILY_NUM,
} ;

enum { ds2482REG_STAT, ds2482REG_DATA, ds2482REG_CHAN, ds2482REG_CONF, ds2482REG_NUM } ;
enum { sd2482CHAN_0, sd2482CHAN_1, sd2482CHAN_2, sd2482CHAN_3, sd2482CHAN_4, sd2482CHAN_5, sd2482CHAN_6, sd2482CHAN_7, sd2482CHAN_NUM } ;

// ######################################### Structures ############################################

// See http://www.catb.org/esr/structure-packing/
// Also http://c0x.coding-guidelines.com/6.7.2.1.html

typedef union {
	struct {
	// STATus register
/*LSB*/	uint8_t		OWB		: 1 ;			// 1-Wire Busy
		uint8_t		PPD		: 1 ;			// Presence Pulse Detected
		uint8_t		SD		: 1 ;
		uint8_t		LL		: 1 ;			// Link Level
		uint8_t		RST		: 1 ;			// ReSeT
		uint8_t		SBR		: 1 ;			//
		uint8_t		TSB		: 1 ;			//
/*MSB*/	uint8_t		DIR		: 1 ;			// DIRection

		uint8_t		CHAN	: 8 ;			// CHANnel Selected register
		uint8_t		DATA	: 8 ;			// DATA Register

	// CONFiguration register
/*LSB*/	uint8_t		APU		: 1 ;			// Active Pull Up
		uint8_t		RES2	: 1 ;			// Reserved (PPM)
		uint8_t		SPU		: 1 ;			// Strong Pull Up
		uint8_t		OWS		: 1 ;			// 1-Wire Speed
/*MSB*/	uint8_t		RES1	: 4 ;
	} ;
	struct {
		uint8_t		Rstat ;
		uint8_t		Rdata ;
		uint8_t		Rchan ;
		uint8_t		Rconf ;
	} ;
	uint8_t			RegX[4] ;
} ds2482_regs_t ;

typedef struct {										// DS2482 I2C <> 1Wire bridge
	halI2Cdev_t		sI2Cdev ;
	ds2482_regs_t	Regs ;
	ow_rom_t		ROM ;
	SemaphoreHandle_t	Mux ;
	int32_t 		LastDiscrepancy ;
	int32_t 		LastFamilyDiscrepancy ;
	uint8_t 		crc8 ;
	uint8_t			CurChan			: 3 ;
	uint8_t			RegPntr			: 2 ;
	uint8_t 		LastDeviceFlag	: 1 ;
} ds2482_t ;

// #################################### Public Data structures #####################################

extern uint8_t	FamilyCount[], ChannelCount[] ;
extern ds2482_t	sDS2482 ;

// ###################################### Private functions ########################################

int32_t OWReset(void) ;

uint8_t	OWCheckCRC(uint8_t * buf, uint8_t buflen) ;
void	OWAddress(uint8_t nAddrMethod) ;
int32_t	OWWriteByte(uint8_t sendbyte) ;
int32_t OWWriteBytePower(int32_t sendbyte) ;
int32_t	OWWriteByteWait(uint8_t sendbyte) ;
void	OWBlock(uint8_t * tran_buf, int32_t tran_len) ;
int32_t OWLevel(int32_t new_level) ;

void	ds2482PrintROM(ow_rom_t * psOW_ROM) ;
uint8_t	ds2482Report(void) ;
int32_t ds2482ChannelSelect(uint8_t Chan) ;

int32_t	ds2482ScanChannelAll(void) ;
int32_t	ds2482ScanFam01(void) ;

float	DS2482_ReadFam10(int32_t Idx) ;
int32_t	DS2482_ScanFam10All(ep_work_t * psEpWork) ;

int32_t	ds2482ScanAndCall(uint8_t Family, int32_t (* Handler)(int32_t)) ;
int32_t	ds2482Diagnostics(void) ;
int32_t	ds2482Identify(uint8_t chanI2C, uint8_t addrI2C) ;
int32_t	ds2482Config(void) ;
void	ds2482Tests(void) ;
