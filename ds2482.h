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

// Device base address
#define	configHAL_I2C_DS2482_ADDR			0x18
#define configHAL_I2C_1WIRE_IN				8

// misc constants
#define POLL_LIMIT  						200

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

// API mode bit flags
#define MODE_STANDARD						0x00
#define MODE_OVERDRIVE						0x01
#define MODE_STRONG							0x02

/*
 * DS2482 Register numbers
 */
#define	DS2482_REGNUM_STAT					0
#define	DS2482_REGNUM_DATA					1
#define	DS2482_REGNUM_CHAN					2
#define	DS2482_REGNUM_CONF					3
#define	DS2482_REGNUM_MAX					4


// ######################################## Enumerations ###########################################

enum {
	idxOWFAMILY_01,
	idxOWFAMILY_10,
	idxOWFAMILY_28,
	idxOWFAMILY_NUM,
} ;

enum {
	sd2482CHAN_0,
	sd2482CHAN_1,
	sd2482CHAN_2,
	sd2482CHAN_3,
	sd2482CHAN_4,
	sd2482CHAN_5,
	sd2482CHAN_6,
	sd2482CHAN_7,
	sd2482CHAN_NUM,
	sd2482CHAN_ANY	= 0xFF,
} ;

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
		uint8_t		PPM		: 1 ;			// PPM
		uint8_t		SPU		: 1 ;			// Strong Pull Up
		uint8_t		OWS		: 1 ;			// 1-Wire Speed
/*MSB*/	uint8_t		RES1	: 4 ;
	} ;
	uint8_t			RegX[4] ;
} ds2482_regs_t ;

typedef struct {
	halI2Cdev_t		sI2Cdev ;
	ds2482_regs_t	Regs ;
	ow_rom_t		ROM ;
	int32_t 		LastDiscrepancy ;
	int32_t 		LastFamilyDiscrepancy ;
	uint8_t 		crc8 ;
	uint8_t			CurChan			: 3 ;
	uint8_t			RegPntr			: 2 ;
	uint8_t 		LastDeviceFlag	: 1 ;
} DS2482_t ;

extern DS2482_t	sDS2482 ;

// ###################################### Private functions ########################################

void	halDS2482_PrintRegister(DS2482_t * psDS2482, uint8_t Reg) ;
uint8_t	halDS2482_ReadRegister(DS2482_t * psDS2482, uint8_t Reg) ;
uint8_t	halDS2482_DecodeRegisters(DS2482_t * psDS2482) ;
int32_t	halDS2482_reset(DS2482_t * psDS2482) ;
int32_t halDS2482_detect(DS2482_t * psDS2482) ;
uint8_t halDS2482_search_triplet(DS2482_t * psDS2482, uint8_t search_direction) ;
int32_t halDS2482_write_config(DS2482_t * psDS2482) ;
int32_t halDS2482_channel_select(DS2482_t * psDS2482, uint8_t Chan) ;
int32_t	halDS2482_SetReadPointer(DS2482_t * psDS2482, uint8_t Reg) ;
int32_t	halDS2482_ScanChannel(DS2482_t * psDS2482, uint8_t Chan) ;

int32_t	xDS2482_ScanCB(ep_work_t * pEpWork) ;

// ####################################### Global functions ########################################

// 1-Wire API for DS2482 function prototypes
void	OWWriteByte(DS2482_t * psDS2482, uint8_t sendbyte) ;
uint8_t	OWReadByte(DS2482_t * psDS2482) ;
uint8_t	OWTouchByte(DS2482_t * psDS2482, uint8_t sendbyte) ;
uint8_t	OWTouchBit(DS2482_t * psDS2482, uint8_t sendbit) ;
void	OWWriteBit(DS2482_t * psDS2482, uint8_t sendbit) ;
uint8_t	OWReadBit(DS2482_t * psDS2482) ;
void	OWBlock(DS2482_t * psDS2482, uint8_t *tran_buf, int tran_len) ;
int32_t	OWFirst(DS2482_t * psDS2482) ;
int32_t	OWNext(DS2482_t * psDS2482) ;
int32_t	OWVerify(DS2482_t * psDS2482) ;
void	OWTargetSetup(DS2482_t * psDS2482, uint8_t family_code) ;
void	OWFamilySkipSetup(DS2482_t * psDS2482) ;
int32_t	OWSearch(DS2482_t * psDS2482) ;

// Extended 1-Wire functions
int32_t	OWSpeed(DS2482_t * psDS2482, int32_t new_speed) ;
int32_t	OWLevel(DS2482_t * psDS2482, int32_t new_level) ;
int32_t OWWriteBytePower(DS2482_t * psDS2482, int32_t sendbyte) ;
int32_t OWReadBitPower(DS2482_t * psDS2482, int32_t applyPowerResponse) ;
void 	OWAddress(DS2482_t * psDS2482, uint8_t nAddrMethod) ;
uint8_t	OWCheckCRC(DS2482_t * psDS2482, uint8_t * buf, uint8_t buflen) ;

// Helper functions
uint8_t OWCalcCRC8(DS2482_t * psDS2482, uint8_t data) ;

// DS2482 Diagnostics function(s)
int32_t	halDS2482_CountDevices(DS2482_t * psDS2482) ;
void	halDS2482_PrintROM(ow_rom_t * psOW_ROM) ;
int32_t	halDS2482_Diagnostics(void) ;
int32_t	halDS2482_Identify(uint8_t chanI2C, uint8_t addrI2C) ;
struct rule_s ;
int32_t halDS2482_ConfigMode(struct rule_s * psRule) ;
