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
 * ds2482.c
 */

#include	"FreeRTOS_Support.h"

#if		(halHAS_DS2482 > 0)

#include	"ds2482.h"
#include	"rules_engine.h"

#include	"hal_i2c.h"

#include	"x_debug.h"
#include	"x_buffers.h"
#include	"x_errors_events.h"
#include	"x_systiming.h"					// timing debugging
#include	"x_syslog.h"

#include	<stdint.h>
#include	<string.h>

#define	debugFLAG					0x400C

#define	debugTIMING					(debugFLAG & 0x0001)
#define	debugBUS_CFG				(debugFLAG & 0x0004)
#define	debugCONFIG					(debugFLAG & 0x0008)

#define	debugTRACK					(debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG & 0x8000)

// DS2484 channel Number to Selection (1's complement) translation
static	const	uint8_t		ds2482_N2S[sd2482CHAN_NUM] = { 0xF0, 0xE1, 0xD2, 0xC3, 0xB4, 0xA5, 0x96, 0x87 } ;
// DS2482 channel Value (read/check) to Number translation
static	const	uint8_t		ds2482_V2N[sd2482CHAN_NUM] = { 0xB8, 0xB1, 0xAA, 0xA3, 0x9C, 0x95, 0x8E, 0x87 } ;

DS2482_t	sDS2482 = { 0 } ;

// ###################################### Private functions ########################################

/**
 * Display register contents
 */
void	halDS2482_PrintRegister(DS2482_t * psDS2482, uint8_t Reg) {
	IF_myASSERT(debugPARAM, Reg < DS2482_REGNUM_MAX) ;
	switch (Reg) {
	case DS2482_REGNUM_STAT:
		PRINT("STAT(%d)= 0x%02X DIR=%c TSB=%c SBR=%c RST=%c LL=%c SD=%c PPD=%c 1WB=%c\n",
				Reg, psDS2482->Regs.RegX[DS2482_REGNUM_STAT],
				psDS2482->Regs.DIR ? '1' : '0',
				psDS2482->Regs.TSB ? '1' : '0',
				psDS2482->Regs.SBR ? '1' : '0',
				psDS2482->Regs.RST ? '1' : '0',
				psDS2482->Regs.LL  ? '1' : '0',
				psDS2482->Regs.SD  ? '1' : '0',
				psDS2482->Regs.PPD ? '1' : '0',
				psDS2482->Regs.OWB ? '1' : '0') ;
		break ;

	case DS2482_REGNUM_DATA:
		PRINT("DATA(%d)= 0x%02X\n", Reg, psDS2482->Regs.RegX[DS2482_REGNUM_DATA]) ;
		break ;

	case DS2482_REGNUM_CHAN: {
		// start by finding the matching Channel #
		int32_t channel ;
		for (channel = 0; channel < configHAL_I2C_1WIRE_IN; ++channel) {
			if (psDS2482->Regs.RegX[DS2482_REGNUM_CHAN] == ds2482_V2N[channel]) {
				break ;
			}
		}
		IF_myASSERT(debugRESULT, channel < configHAL_I2C_1WIRE_IN)
		PRINT("CHAN(%d)= 0x%02x ==> %d\n", Reg, psDS2482->Regs.RegX[DS2482_REGNUM_CHAN], channel) ;
		break ;
	}

	case DS2482_REGNUM_CONF:
		PRINT("CONF(%d)= 0x%02X RES1=0x%02X 1WS=%c SPU=%c RES2=%c APU=%c\n",
				Reg, psDS2482->Regs.RegX[DS2482_REGNUM_CONF],
				psDS2482->Regs.RES1,
				psDS2482->Regs.OWS	? '1' : '0',
				psDS2482->Regs.SPU	? '1' : '0',
				psDS2482->Regs.PPM	? '1' : '0',
				psDS2482->Regs.APU	? '1' : '0') ;
		break ;
	}
}

/**
 * Read a register based on last/current Read Pointer status
 */
uint8_t	halDS2482_ReadRegister(DS2482_t * psDS2482, uint8_t Reg) {
	IF_myASSERT(debugPARAM, Reg < DS2482_REGNUM_MAX)
	int32_t	iRV = halI2C_Read(&psDS2482->sI2Cdev, (uint8_t *) &psDS2482->Regs.RegX[Reg], sizeof(uint8_t)) ;
	if (iRV != erSUCCESS) {
		return 0 ;
	}
	return 1 ;
}

/**
 * Read ALL the registers
 */
uint8_t	halDS2482_DecodeRegisters(DS2482_t * psDS2482) {
	for (uint8_t Reg = 0; Reg < 4; Reg++) {
		halDS2482_SetReadPointer(psDS2482, Reg) ;
		if (halDS2482_ReadRegister(psDS2482, Reg) == 0) {
			return 0 ;
		}
		halDS2482_PrintRegister(psDS2482, Reg) ;
	}
	return 1 ;
}

/**
 * halDS2482_WriteAndWait()
 * \brief	Primarily to handle the Maxim/Dallas 1-Wire protocol and the waiting for status
 * 			Perform repeated reads waiting for a matching bit(map) to be cleared.
 * 			Returns when the correct status is received or the max retries are exceeded.
 */
int32_t	halDS2482_WriteAndWait(DS2482_t * psDS2482, uint8_t* pTxBuf, size_t TxSize) {
	uint32_t	Tries = 0 ;
	uint8_t		Status ;
	halI2C_Write(&psDS2482->sI2Cdev, pTxBuf, TxSize) ;
	do {
		vTaskDelay(pdMS_TO_TICKS(1)) ;			// repeated read phase
		halI2C_Read(&psDS2482->sI2Cdev, &Status, 1) ;
	} while ((Status & STATUS_1WB) && (++Tries < POLL_LIMIT)) ;
	if (Tries == POLL_LIMIT) {
		return erFAILURE ;
	}
	psDS2482->RegPntr	= DS2482_REGNUM_STAT ;
	psDS2482->Regs.RegX[DS2482_REGNUM_STAT] = Status ;
	return erSUCCESS;
}

/**
 * Perform a device reset on the DS2482
 * Returns: true if device was reset
 *			 false device not detected or failure to perform reset
 */
int32_t halDS2482_reset(DS2482_t * psDS2482) {
// Device Reset
//	S AD,0 [A] DRST [A] Sr AD,1 [A] [SS] A\ P
//  [] indicates from slave
//  SS status byte to read to verify state
	uint8_t	cChr = CMD_DRST ;
	uint8_t status ;
	int32_t iRV = halI2C_WriteRead(&psDS2482->sI2Cdev, &cChr, sizeof(cChr), &status, 1) ;
	if (iRV != erSUCCESS) {
		return 0 ;
	}
	psDS2482->Regs.RegX[DS2482_REGNUM_STAT] = status ;
	psDS2482->RegPntr = DS2482_REGNUM_STAT ;
	return ((status & ~STATUS_LL) == STATUS_RST) ;		// RESET true or false...
}

/**
 * DS2482_SetReadPointer()
 * Once set the pointer remains static to allow reread of same (normally status) register
 * Read Pointer will be changed by a new SRP command or by a devie reset
 */
int32_t	halDS2482_SetReadPointer(DS2482_t * psDS2482, uint8_t Reg) {
	IF_myASSERT(debugPARAM, Reg < DS2482_REGNUM_MAX)
	uint8_t		cBuf[2] ;
	cBuf[0] = CMD_SRP ;
	cBuf[1] = (~Reg << 4) | Reg ;
	int32_t	iRetVal = halI2C_Write(&psDS2482->sI2Cdev, cBuf, sizeof(cBuf)) ;
	if (iRetVal != erSUCCESS) {
		return 0 ;
	}
// update the read pointer
	psDS2482->RegPntr = Reg ;
	return iRetVal ;
}

/**
 * Write the configuration register in the DS2482. The configuration
 * options are provided in the lower nibble of the provided config byte.
 * The uppper nibble in bitwise inverted when written to the DS2482.
 *
 * Returns:  true: config written and response correct
 *			  false: response incorrect
 */
int32_t halDS2482_write_config(DS2482_t * psDS2482) {
// Write configuration (Case A)
//	S AD,0 [A] WCFG [A] CF [A] Sr AD,1 [A] [CF] A\ P
//  [] indicates from slave
//  CF configuration byte to write
	uint8_t	config = psDS2482->Regs.RegX[DS2482_REGNUM_CONF] & 0x0F ;
	IF_myASSERT(debugPARAM, config < 0x10)
	IF_myASSERT(debugBUS_CFG, psDS2482->Regs.OWB == 0) ;				// check that bus not busy
	uint8_t	cBuf[2] ;
	cBuf[0]	= CMD_WCFG ;
	cBuf[1] = (~config << 4) | config ;
	uint8_t new_conf ;
	int32_t iRetVal = halI2C_WriteRead(&psDS2482->sI2Cdev, cBuf, sizeof(cBuf), &new_conf, sizeof(new_conf)) ;
	if (iRetVal != erSUCCESS) {
		return 0 ;
	}
// update the saved configuration
	psDS2482->Regs.RegX[DS2482_REGNUM_CONF] = new_conf ;
// update the read pointer
	psDS2482->RegPntr	= DS2482_REGNUM_CONF ;
	return 1;
}

/**
 * DS2428 Detect routine that sets the I2C address and then performs a
 * device reset followed by writing the configuration byte to default values:
 *	1-Wire speed (c1WS) = standard (0)
 *	Strong pull-up (cSPU) = off (0)
 *	Presence pulse masking (cPPM) = off (0)
 *	Active pull-up (cAPU) = on (CONFIG_APU = 0x01)
 *
 * Returns: true if device was detected and written
 *			 false device not detected or failure to write configuration byte
 */
int32_t halDS2482_detect(DS2482_t * psDS2482) {
	if (halDS2482_reset(psDS2482) == 0) {				// reset the DS2482 ON selected address
		return 0;
	}
	// default configuration 0xE1 (0xE? is the 1s complement of 0x?1)
	psDS2482->Regs.APU	= 1 ;						// LSBit
	psDS2482->Regs.PPM	= 0 ;
	psDS2482->Regs.SPU	= 0 ;
	psDS2482->Regs.OWS	= 0 ;
	psDS2482->Regs.RES1	= 0 ;						// MSBit
	// confirm bit packing order is correct
	IF_myASSERT(debugCONFIG, psDS2482->Regs.RegX[DS2482_REGNUM_CONF] == CONFIG_APU) ;
	return halDS2482_write_config(psDS2482) ;
}

/**
 * Select the 1-Wire channel on a DS2482-800.
 *
 * Returns: erSUCCESS if channel selected
 *			erFAILURE if device not detected or failure to perform select; else
 *			whatever status returned by halI2C_WriteRead()
 */
int32_t halDS2482_channel_select(DS2482_t * psDS2482, uint8_t Chan) {
// Channel Select (Case A)
//	S AD,0 [A] CHSL [A] CC [A] Sr AD,1 [A] [RR] A\ P
//  [] indicates from slave
//  CC channel value
//  RR channel read back
	IF_myASSERT(debugPARAM, Chan < sd2482CHAN_NUM) ;
	IF_myASSERT(debugBUS_CFG, psDS2482->Regs.OWB == 0) ;				// check that bus not busy
	uint8_t	cBuf[2] ;
	cBuf[0]	= CMD_CHSL ;
	cBuf[1] = ds2482_N2S[Chan] ;
	uint8_t result ;
	int32_t iRV = halI2C_WriteRead(&psDS2482->sI2Cdev, cBuf, sizeof(cBuf), &result, sizeof(result)) ;
	if (iRV != erSUCCESS) {
		return erFAILURE ;
	}

	psDS2482->RegPntr	= DS2482_REGNUM_CHAN ;			// update the read pointer
	/* value read back not same as the channel number sent
	 * so verify the return against the code expected, but
	 * store the actual channel number if successful */
	if (result != ds2482_V2N[Chan]) {
		IF_myASSERT(debugRESULT, 0) ;
		return erFAILURE ;
	}
	psDS2482->Regs.RegX[DS2482_REGNUM_CHAN] = result ;	// update saved channel selected value (translated value)
	psDS2482->CurChan	= Chan ;						// and the actual (normalized) channel number
	return erSUCCESS ;
}

/**
 * Use the DS2482 help command '1-Wire triplet' to perform one bit of a 1-Wire
 * search. This command does two read bits and one write bit. The write bit
 * is either the default direction (all device have same bit) or in case of
 * a discrepancy, the 'search_direction' parameter is used.
 *
 * Returns � The DS2482 status byte result from the triplet command
 */
uint8_t halDS2482_search_triplet(DS2482_t * psDS2482, uint8_t search_direction) {
// 1-Wire Triplet (Case B)
//	S AD,0 [A] 1WT [A] SS [A] Sr AD,1 [A] [Status] A [Status] A\ P
//							  \--------/
//				Repeat until 1WB bit has changed to 0
//  [] indicates from slave
//  SS indicates byte containing search direction bit value in msbit
	IF_myASSERT(debugPARAM, search_direction < 2) ;
	uint8_t	cBuf[2] ;
	cBuf[0]	= CMD_1WT ;
	cBuf[1]	= search_direction ? 0x80 : 0x00 ;
	if (halDS2482_WriteAndWait(psDS2482, cBuf, sizeof(cBuf)) == erFAILURE) {
		halDS2482_reset(psDS2482);
		return 0;
	}
	return psDS2482->Regs.RegX[DS2482_REGNUM_STAT] ;
}

/**
 * halDS2482_ScanChannel() - Scan a specific device & channel for a reader
 * @param pDS24282	pointer to device structure
 * @param Chan		channel number (0->7) to scan
 * @return			true if a device found, false if not
 * 					1-Wire device ROM SN# will be in the structure
 */
int32_t	halDS2482_ScanChannel(DS2482_t * psDS2482, uint8_t Chan) {
	IF_myASSERT(debugPARAM, INRANGE_SRAM(psDS2482) && Chan < configHAL_I2C_1WIRE_IN) ;
	int32_t	iRV ;
	if (psDS2482->CurChan != Chan) {
		iRV = halDS2482_channel_select(psDS2482, Chan) ;
		if (iRV != erSUCCESS) {
			return erFAILURE ;
		}
		iRV = OWFirst(psDS2482) ;
	} else {
		iRV = OWNext(psDS2482) ;
	}
	return iRV ;
}

// ####################################### Global functions ########################################

/**
 * Reset all of the devices on the 1-Wire Net and return the result.
 *
 * Returns: true(1):  presence pulse(s) detected, device(s) reset
 *			 false(0): no presence pulses detected
 */
int32_t OWReset(DS2482_t * psDS2482) {
// 1-Wire reset (Case B)
//	S AD,0 [A] 1WRS [A] Sr AD,1 [A] [Status] A [Status] A\ P
//									\--------/
//						Repeat until 1WB bit has changed to 0
//  [] indicates from slave
	IF_myASSERT(debugBUS_CFG, psDS2482->Regs.OWB == 0 && psDS2482->Regs.SPU == 0) ;
	uint8_t	cChr = CMD_1WRS ;
	if (halDS2482_WriteAndWait(psDS2482, &cChr, sizeof(cChr)) == erFAILURE) {
		halDS2482_reset(psDS2482);
		return 0;
	}
	return psDS2482->Regs.PPD ;
}

/**
 * Send 1 bit of communication to the 1-Wire Net.
 * The parameter 'sendbit' least significant bit is used.
 *
 * 'sendbit' - 1 bit to send (least significant byte)
 */
void	OWWriteBit(DS2482_t * psDS2482, uint8_t sendbit) { OWTouchBit(psDS2482, sendbit); }

/**
 * Reads 1 bit of communication from the 1-Wire Net and returns the
 * result
 *
 * Returns:  1 bit read from 1-Wire Net
 */
uint8_t OWReadBit(DS2482_t * psDS2482) { return OWTouchBit(psDS2482, 0x01) ; }

/**
 * Send 1 bit of communication to the 1-Wire Net and return the
 * result 1 bit read from the 1-Wire Net.  The parameter 'sendbit'
 * least significant bit is used and the least significant bit
 * of the result is the return bit.
 *
 * 'sendbit' - the least significant bit is the bit to send
 *
 * Returns: 0:	0 bit read from sendbit
 *			 1:	1 bit read from sendbit
 */
uint8_t OWTouchBit(DS2482_t * psDS2482, uint8_t sendbit) {
// 1-Wire bit (Case B)
//	S AD,0 [A] 1WSB [A] BB [A] Sr AD,1 [A] [Status] A [Status] A\ P
//										   \--------/
//								Repeat until 1WB bit has changed to 0
//  [] indicates from slave
//  BB indicates byte containing bit value in msbit
	IF_myASSERT(debugPARAM, sendbit < 2) ;
	IF_myASSERT(debugBUS_CFG, psDS2482->Regs.OWB == 0)	;
	uint8_t	cBuf[2] ;
	cBuf[0]	= CMD_1WSB ;
	cBuf[1] = sendbit ? 0x80 : 0x00 ;
	if (halDS2482_WriteAndWait(psDS2482, cBuf, sizeof(cBuf)) == erFAILURE) {
		return 0;
	}
// return bit state
	return psDS2482->Regs.SBR ;
}

/**
 * Send 8 bits of communication to the 1-Wire Net and verify that the
 * 8 bits read from the 1-Wire Net is the same (write operation).
 * The parameter 'sendbyte' least significant 8 bits are used.
 *
 * 'sendbyte' - 8 bits to send (least significant byte)
 */
void	OWWriteByte(DS2482_t * psDS2482, uint8_t sendbyte) {
// 1-Wire Write Byte (Case B)
//	S AD,0 [A] 1WWB [A] DD [A] Sr AD,1 [A] [Status] A [Status] A\ P
//										   \--------/
//							Repeat until 1WB bit has changed to 0
//  [] indicates from slave
//  DD data to write
	IF_myASSERT(debugBUS_CFG, psDS2482->Regs.OWB == 0)	;
	uint8_t	cBuf[2] ;
	cBuf[0]	= CMD_1WWB ;
	cBuf[1] = sendbyte ;
	if (halDS2482_WriteAndWait(psDS2482, cBuf, sizeof(cBuf)) == erFAILURE) {
		halDS2482_reset(psDS2482) ;
	}
}

/**
 * Send 8 bits of read communication to the 1-Wire Net and return the
 * result 8 bits read from the 1-Wire Net.
 *
 * Returns:  8 bits read from 1-Wire Net
 */
uint8_t OWReadByte(DS2482_t * psDS2482) {
/* 1-Wire Read Bytes (Case C)
 *	S AD,0 [A] 1WRB [A] Sr AD,1 [A] [Status] A [Status] A\
 *										\--------/
 *							Repeat until 1WB bit has changed to 0
 *	Sr AD,0 [A] SRP [A] E1 [A] Sr AD,1 [A] DD A\ P
 *
 *  [] indicates from slave
 *  DD data read
 */
	uint8_t	cChr= CMD_1WRB ;
	halDS2482_WriteAndWait(psDS2482, &cChr, sizeof(cChr)) ;
	halDS2482_SetReadPointer(psDS2482, DS2482_REGNUM_DATA) ;	// set pointer to address the data register
	uint8_t		data ;
	halI2C_Read(&psDS2482->sI2Cdev, &data, sizeof(data)) ;
	return data ;
}

/**
 * Send 8 bits of communication to the 1-Wire Net and return the
 * result 8 bits read from the 1-Wire Net.  The parameter 'sendbyte'
 * least significant 8 bits are used and the least significant 8 bits
 * of the result is the return byte.
 *
 * 'sendbyte' - 8 bits to send (least significant byte)
 *
 * Returns:  8 bits read from sendbyte
 */
uint8_t OWTouchByte(DS2482_t * psDS2482, uint8_t sendbyte) {
	if (sendbyte == 0xFF) {
		return OWReadByte(psDS2482);
	} else {
		OWWriteByte(psDS2482, sendbyte);
		return sendbyte;
	}
}

/**
 * The 'OWBlock' transfers a block of data to and from the
 * 1-Wire Net. The result is returned in the same buffer.
 *
 * 'tran_buf' - pointer to a block of unsigned
 *				  chars of length 'tran_len' that will be sent
 *				  to the 1-Wire Net
 * 'tran_len' - length in bytes to transfer
 */
void	OWBlock(DS2482_t * psDS2482, uint8_t *tran_buf, int32_t tran_len){
	for (int32_t i = 0; i < tran_len; i++) {
		tran_buf[i] = OWTouchByte(psDS2482, tran_buf[i]) ;
	}
}

/**
 * Find the 'first' devices on the 1-Wire network
 * Return true  : device found, ROM number in ROM.Number buffer
 *		  false : no device present
 */
int32_t OWFirst(DS2482_t * psDS2482) {
	psDS2482->LastDiscrepancy	= 0 ;					// reset the search state
	psDS2482->LastDeviceFlag	= 0 ;
	psDS2482->LastFamilyDiscrepancy = 0 ;
	return OWSearch(psDS2482) ;
}

/**
 * Find the 'next' devices on the 1-Wire network
 * Return true  : device found, ROM number in ROM.Number buffer
 *		  false : device not found, end of search
 */
int32_t OWNext(DS2482_t * psDS2482) { return OWSearch(psDS2482) ; }

/**
 * Verify the device with the ROM number in ROM.Number buffer is present.
 * Return true  : device verified present
 *		  false : device not present
 */
int32_t OWVerify(DS2482_t * psDS2482) {
	uint8_t rom_backup[ONEWIRE_ROM_LENGTH];
	for (int32_t i = 0; i < ONEWIRE_ROM_LENGTH; i++) {
		rom_backup[i] = psDS2482->ROM.HexChars[i];		// keep a backup copy of the current state
	}
	int32_t	ld_backup	= psDS2482->LastDiscrepancy;
	int32_t	ldf_backup	= psDS2482->LastDeviceFlag;
	int32_t	lfd_backup	= psDS2482->LastFamilyDiscrepancy;
// set search to find the same device
	psDS2482->LastDiscrepancy	= 64 ;
	psDS2482->LastDeviceFlag	= 0 ;

	int32_t iRV ;
	if (OWSearch(psDS2482)) {
	// check if same device found
		iRV = 1 ;
		for (int32_t i = 0; i < ONEWIRE_ROM_LENGTH; i++) {
			if (rom_backup[i] != psDS2482->ROM.HexChars[i]) {
				iRV = 0 ;
				break ;
			}
		}
	} else {
	  iRV = 0 ;
	}
// restore the search state
	for (int32_t i = 0; i < ONEWIRE_ROM_LENGTH; i++) {
		psDS2482->ROM.HexChars[i] = rom_backup[i];
	}
	psDS2482->LastDiscrepancy = ld_backup;
	psDS2482->LastDeviceFlag	= ldf_backup;
	psDS2482->LastFamilyDiscrepancy = lfd_backup;
// return the result of the verify
	return iRV;
}

/**
 * Setup the search to find the device type 'family_code' on the next call
 * to OWNext() if it is present.
 */
void	OWTargetSetup(DS2482_t * psDS2482, uint8_t family_code) {
	psDS2482->ROM.Value				= 0ULL ;			// reset all ROM fields
	psDS2482->ROM.Family 			= family_code ;
	psDS2482->LastDiscrepancy		= 64 ;
	psDS2482->LastFamilyDiscrepancy = 0 ;
	psDS2482->LastDeviceFlag		= 0 ;
}

/**
 * Setup the search to skip the current device type on the next call
 * to OWNext().
 */
void	OWFamilySkipSetup(DS2482_t * psDS2482) {
	psDS2482->LastDiscrepancy = psDS2482->LastFamilyDiscrepancy ;	// set the Last discrepancy to last family discrepancy
	psDS2482->LastFamilyDiscrepancy = 0 ;				// clear the last family discrepancy
	if (psDS2482->LastDiscrepancy == 0) {				// check for end of list
		psDS2482->LastDeviceFlag	= 1 ;
	}
}

/**
 * The 'OWSearch' function does a general search.  This function
 * continues from the previous search state. The search state
 * can be reset by using the 'OWFirst' function.
 * This function contains one parameter 'alarm_only'.
 * When 'alarm_only' is true (1) the find alarm command
 * 0xEC(ONEWIRE_CMD_ALARMSEARCH) is sent instead of
 * the normal search command 0xF0(ONEWIRE_CMD_SEARCHROM).
 * Using the find alarm command 0xEC will limit the search to only
 * 1-Wire devices that are in an 'alarm' state.
 *
 * Returns:	true (1) : when a 1-Wire device was found and its
 *						  Serial Number placed in the global ROM
 *			false (0): when no new device was found.  Either the
 *						  last search was the last device or there
 *						  are no devices on the 1-Wire Net.
 */
int32_t OWSearch(DS2482_t * psDS2482) {
// initialize for search
	int32_t	id_bit_number = 1;
	int32_t	last_zero = 0;
	int32_t	rom_byte_number = 0;
	uint8_t	rom_byte_mask = 1;
	int32_t	search_result = 0;
	psDS2482->crc8 = 0;
// if the last call was not the last device
	if (psDS2482->LastDeviceFlag == 0) {
		if (OWReset(psDS2482) == 0) {
		// reset the search
			psDS2482->LastDiscrepancy = 0 ;
			psDS2482->LastDeviceFlag	= 0 ;
			psDS2482->LastFamilyDiscrepancy = 0 ;
			return 0;
		}
	// issue the search command
		OWWriteByte(psDS2482, ONEWIRE_CMD_SEARCHROM);
	// loop to do the search
		uint8_t search_direction, status;
		do {
		// if this discrepancy if before the Last Discrepancy
		// on a previous next then pick the same as last time
			if (id_bit_number < psDS2482->LastDiscrepancy) {
				if ((psDS2482->ROM.HexChars[rom_byte_number] & rom_byte_mask) > 0) {
					search_direction = 1 ;
				} else {
					search_direction = 0 ;
				}
			} else {
			// if equal to last pick 1, if not then pick 0
				if (id_bit_number == psDS2482->LastDiscrepancy) {
					search_direction = 1 ;
				} else {
					search_direction = 0 ;
				}
			}
		// Perform a triple operation on the DS2482 which will perform 2 read bits and 1 write bit
			status = halDS2482_search_triplet(psDS2482, search_direction);
		// check bit results in status byte
			int32_t	id_bit = ((status & STATUS_SBR) == STATUS_SBR);
			int32_t	cmp_id_bit = ((status & STATUS_TSB) == STATUS_TSB);
			search_direction = ((status & STATUS_DIR) == STATUS_DIR) ? 1 : 0;
		// check for no devices on 1-Wire
			if ((id_bit) && (cmp_id_bit)) {
				break;
			} else {
				if ((!id_bit) && (!cmp_id_bit) && (search_direction == 0)) {
					last_zero = id_bit_number;
				// check for Last discrepancy in family
					if (last_zero < 9) {
						psDS2482->LastFamilyDiscrepancy = last_zero;
					}
				}
			// set or clear the bit in the ROM byte rom_byte_number
			// with mask rom_byte_mask
				if (search_direction == 1) {
					psDS2482->ROM.HexChars[rom_byte_number] |= rom_byte_mask;
				} else {
					psDS2482->ROM.HexChars[rom_byte_number] &= ~rom_byte_mask;
				}
			// increment the byte counter id_bit_number
			// and shift the mask rom_byte_mask
				id_bit_number++;
				rom_byte_mask <<= 1;
			// if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
				if (rom_byte_mask == 0) {
					OWCalcCRC8(psDS2482, psDS2482->ROM.HexChars[rom_byte_number]);  // accumulate the CRC
					rom_byte_number++;
					rom_byte_mask = 1;
				}
			}
		} while(rom_byte_number < ONEWIRE_ROM_LENGTH) ;  // loop until all

	// if the search was successful then
		if (!((id_bit_number < 65) || (psDS2482->crc8 != 0))) {
		// search successful so set LastDiscrepancy,LastDeviceFlag,search_result
			psDS2482->LastDiscrepancy = last_zero;
		// check for last device
			if (psDS2482->LastDiscrepancy == 0) {
				psDS2482->LastDeviceFlag	= 1 ;
			}
			search_result = 1 ;
		}
	}

	// if no device found then reset counters so next 'search' will be like a first
	if (!search_result || (psDS2482->ROM.Family == 0)) {
		psDS2482->LastDiscrepancy	= 0 ;
		psDS2482->LastDeviceFlag	= 0 ;
		psDS2482->LastFamilyDiscrepancy = 0 ;
		search_result = 0 ;
	}
	return search_result;
}

/**
 * Set the 1-Wire Net communication speed.
 *
 * 'new_speed' - new speed defined as
 *					 MODE_STANDARD	0x00
 *					 MODE_OVERDRIVE  0x01
 *
 * Returns:  current 1-Wire Net speed
 */
int32_t OWSpeed(DS2482_t * psDS2482, int32_t new_speed) {
	if (new_speed == MODE_OVERDRIVE) {
		psDS2482->Regs.OWS = 1 ;
	} else {
		psDS2482->Regs.OWS = 0 ;
	}
	halDS2482_write_config(psDS2482);
	return new_speed;
}

/**
 * Set the 1-Wire Net line level pull-up to normal. The DS2482 only allows
 * enabling strong pull-up on a bit or byte event. Consequently this
 * function only allows the MODE_STANDARD argument. To enable strong pull-up
 * use OWWriteBytePower or OWReadBitPower.
 *
 * 'new_level' - new level defined as
 *					 MODE_STANDARD	  0x00
 *
 * Returns:  current 1-Wire Net level
 */
int32_t OWLevel(DS2482_t * psDS2482, int32_t new_level) {
	if (new_level != MODE_STANDARD) {
		return MODE_STRONG ;
	}
	psDS2482->Regs.SPU = 0 ;
	halDS2482_write_config(psDS2482) ;
	return MODE_STANDARD ;
}

/**
 * Send 8 bits of communication to the 1-Wire Net and verify that the
 * 8 bits read from the 1-Wire Net is the same (write operation).
 * The parameter 'sendbyte' least significant 8 bits are used.  After the
 * 8 bits are sent change the level of the 1-Wire net.
 *
 * 'sendbyte' - 8 bits to send (least significant bit)
 *
 * Returns:  true: bytes written and echo was the same, strong pullup now on
 *			  false: echo was not the same
 */
int32_t OWWriteBytePower(DS2482_t * psDS2482, int32_t sendbyte) {
	psDS2482->Regs.SPU = 1 ;
	if (halDS2482_write_config(psDS2482) == 0) {
		return 0 ;
	}
	OWWriteByte(psDS2482, sendbyte);
	return 1 ;
}

/**
 * Send 1 bit of communication to the 1-Wire Net and verify that the
 * response matches the 'applyPowerResponse' bit and apply power delivery
 * to the 1-Wire net.  Note that some implementations may apply the power
 * first and then turn it off if the response is incorrect.
 *
 * 'applyPowerResponse' - 1 bit response to check, if correct then start
 *								power delivery
 *
 * Returns:  true: bit written and response correct, strong pullup now on
 *			  false: response incorrect
 */
int32_t OWReadBitPower(DS2482_t * psDS2482, int32_t applyPowerResponse) {
	psDS2482->Regs.SPU = 1 ;
	if (halDS2482_write_config(psDS2482) == 0) {
		return 0 ;
	}
	uint8_t rdbit = OWReadBit(psDS2482);
	if (rdbit != applyPowerResponse) {					// check if response was correct
		OWLevel(psDS2482, MODE_STANDARD);				// if not, turn off strong pull-up
		return 0 ;
	}
	return 1 ;
}

/**
 * OWAddress() - Addresses a single or all devices on the 1-wire bus
 * @param psDS2482		pointer to I2C device instance
 * @param nAddrMethod	use ONEWIRE_CMD_MATCHROM to select a single
 *						device or ONEWIRE_CMD_SKIPROM to select all
 */
void	OWAddress(DS2482_t * psDS2482, uint8_t nAddrMethod) {
	if (nAddrMethod == ONEWIRE_CMD_MATCHROM) {
		OWWriteByte(psDS2482, ONEWIRE_CMD_MATCHROM);	/* address single devices on bus */
		for (uint8_t i = 0; i < ONEWIRE_ROM_LENGTH; i ++) {
			OWWriteByte(psDS2482, psDS2482->ROM.HexChars[i]);
		}
	} else {
		OWWriteByte(psDS2482, ONEWIRE_CMD_SKIPROM);	 /* address all devices on bus */
	}
}

/**
 * OWCheckCRC() - Checks if CRC is ok (ROM Code or Scratch PAD RAM)
 * @param psDS2482		pointer to I2C device instance
 * @param buf			buffer to be checked for proper CRC
 * @param buflen		buffer length
 * @return				TRUE if there are more devices on the 1-wire bus,
 * 						FALSE otherwise
 */
uint8_t	OWCheckCRC(DS2482_t * psDS2482, uint8_t * buf, uint8_t buflen) {
	uint8_t shift_reg = 0 ;
	for (int8_t i = 0; i < buflen; i++) {
		for (int8_t j = 0; j < 8; j++) {
			uint8_t	data_bit = (buf[i] >> j) & 0x01 ;
			uint8_t	sr_lsb = shift_reg & 0x01 ;
			uint8_t	fb_bit = (data_bit ^ sr_lsb) & 0x01 ;
			shift_reg = shift_reg >> 1 ;
			if (fb_bit) {
				shift_reg = shift_reg ^ 0x8c ;
			}
		}
	}
	return (shift_reg == 0) ? 1 : 0 ;
}

/**
 * OWCalcCRC8() - Calculate the CRC8 of the byte value provided with the current 'crc8' value
 * @brief	See Application Note 27
 * @param	psDS2482	pointer to I2C device instance
 * @param	data
 * @return				Returns current crc8 value
 */
uint8_t	OWCalcCRC8(DS2482_t * psDS2482, uint8_t data) {
	psDS2482->crc8 = psDS2482->crc8 ^ data;
	for (int32_t i = 0; i < BITS_IN_BYTE; ++i) {
		if (psDS2482->crc8 & 1) {
			psDS2482->crc8 = (psDS2482->crc8 >> 1) ^ 0x8c;
		} else {
			psDS2482->crc8 = (psDS2482->crc8 >> 1);
		}
	}
	return psDS2482->crc8;
}

// ################################# Application support functions #################################

#include	"task_events.h"

ow_rom_t	LastROM[configHAL_I2C_1WIRE_IN]		= { 0 } ;
seconds_t	LastRead[configHAL_I2C_1WIRE_IN]	= { 0 } ;
uint8_t		OWdelay		= ONEWIRE_DEFAULT_DELAY ;

#if		(ESP32_VARIANT == 2)
uint8_t	OWremapTable[configHAL_I2C_1WIRE_IN] = { 3,	2,	1,	0,	4,	5,	6,	7 } ;
#endif

int32_t	xDS2482_ScanCB(ep_work_t * pEpWork) {
	IF_SYSTIMER_START(debugTIMING, systimerDS2482) ;
	for (uint8_t Chan = 0; Chan < sd2482CHAN_NUM; ++Chan) {
		if (halDS2482_ScanChannel(&sDS2482, Chan) == 1) {		// found a device
			switch (sDS2482.ROM.Family) {
			case OWFAMILY_01: {							// DS1990A/R, 2401/11 devices
				seconds_t NowRead = xTimeStampAsSeconds(sTSZ.usecs) ;
				/* To avoid registering multiple reads if the iButton is held in place for too long
				 * we enforce a period of 'x' seconds within which successive reads of the same tag
				 *  will be ignored. */
				if ((LastROM[Chan].Value == sDS2482.ROM.Value) && (NowRead - LastRead[Chan]) <= OWdelay) {
					IF_PRINT(debugRESULT, "SAME iButton in 5sec, Skipped...\n") ;
					break ;
				}
				LastROM[Chan].Value = sDS2482.ROM.Value ;
				LastRead[Chan] = NowRead ;
#if		(ESP32_VARIANT == 2)
				xTaskNotify(EventsHandle, 1UL << (OWremapTable[Chan] + se1W_FIRST), eSetBits) ;
#elif	(ESP32_VARIANT == 3)
				xTaskNotify(EventsHandle, 1UL << (Chan + se1W_FIRST), eSetBits) ;
#else
	#error	"Incorrect/missing hardware variant !!!"
#endif
				portYIELD() ;
				IF_PRINT(debugRESULT, "NEW iButton Read, or >5sec passed\n") ;
				IF_EXEC_1(debugRESULT, halDS2482_PrintROM, &sDS2482.ROM) ;
				break ;
			}
			default:
				SL_ERR("Invalid OW device FAM=%02x", sDS2482.ROM.Family) ;
			}
		}
	}
	IF_SYSTIMER_STOP(debugTIMING, systimerDS2482) ;
	return erSUCCESS ;
}

// ################################## Diagnostics functions ########################################

/**
 * halDS2482_CountDevices() - Scan all buses and count/list all devices on all channels
 * @param pDS24282	pointer to device structure
 * @return			number of devices found
 */
int32_t	halDS2482_CountDevices(DS2482_t * psDS2482) {
	int32_t	iCount = 0, iRV ;
	IF_myASSERT(debugPARAM, INRANGE_SRAM(psDS2482)) ;
	for (int32_t Chan = 0; Chan < configHAL_I2C_1WIRE_IN; Chan++) {
		do {
			iRV = halDS2482_ScanChannel(psDS2482, Chan) ;
			if (iRV == erFAILURE) {
				return iRV ;
			}
			if (iRV == 1) {
				iCount++ ;
				IF_EXEC_1(debugRESULT, halDS2482_PrintROM, &psDS2482->ROM) ;
			}
		} while (iRV == 1) ;
	}
	IF_PRINT(debugTRACK, "DS2482: Found %d devices\n", iCount) ;
	return iCount ;
}

/**
 * halDS2482_PrintROM() - print the 1-Wire ROM information
 * @param psOW_ROM
 */
void	halDS2482_PrintROM(ow_rom_t * psOW_ROM) {
	PRINT("DS2482: ROM Family=0x%02x TagNum=%#m CRC=0x%02x\n", psOW_ROM->Family, psOW_ROM->TagNum, psOW_ROM->CRC) ;
}

/**
 * halDS2482_Diagnostics() - perform diagnostics on DS2482 device & channels
 * @brief			Scan bus for iButtons, all 8 channels:
 * @brief			If no buttons found,	16	mS
 * @brief			If 1 button found,		81	mSec	+65 mSec
 * @brief			If 2 buttons found,		146	mSec	+65	mSec
 * @brief			Estimated 8 buttons		536 mSec	+520 mSec
 * @brief			Pre-selection of a specific family makes no difference
 * @return			erSUCCESS or erFAILURE
 */
int32_t	halDS2482_Diagnostics(void) {
	if (halDS2482_DecodeRegisters(&sDS2482) == 0) {
		return erFAILURE ;
	}
	if (halDS2482_CountDevices(&sDS2482) == erFAILURE) {
		return erFAILURE ;
	}
	return erSUCCESS ;
}

/**
 * halDS2482_Identify() - Try to identify I2C device
 * @param eChan			halI2C channel to use
 * @param Addr			I2C address where device is located
 * @return				erSUCCESS if device identifies as DS2482 else erFAILURE
 */
int32_t	halDS2482_Identify(uint8_t chanI2C, uint8_t addrI2C) {
	IF_myASSERT(debugPARAM, chanI2C < halI2C_NUM) ;
	sDS2482.sI2Cdev.chanI2C	= chanI2C ;
	sDS2482.sI2Cdev.addrI2C	= addrI2C ;
	sDS2482.sI2Cdev.dlayI2C	= pdMS_TO_TICKS(750) ;
	if (halDS2482_detect(&sDS2482) == 0) {				// if no device found
		sDS2482.sI2Cdev.chanI2C			= 0 ;			// reset all & return
		sDS2482.sI2Cdev.addrI2C			= 0 ;
		sDS2482.sI2Cdev.dlayI2C			= 0 ;
		return erFAILURE ;
	}
	sDS2482.sI2Cdev.epidI2C.devclass	= devDS2482 ;
	sDS2482.sI2Cdev.epidI2C.subclass	= subIBUTTON ;
	sDS2482.sI2Cdev.epidI2C.epuri		= URI_DS2482 ;
	sDS2482.sI2Cdev.epidI2C.epunit		= UNIT_IDENT ;
	IF_SYSTIMER_RESET_NUM(debugTIMING, systimerDS2482, systimerTICKS, "DS2482", myMS_TO_CLOCKS(5), myMS_TO_CLOCKS(25)) ;
	return erSUCCESS ;
}

int32_t halDS2482_ConfigMode(rule_t * psRule) {
	if (psRule->para.u32[psRule->ActIdx][0] == 1) {		// Mode p0=1 specifies scan interval
		table_work[URI_DS2482].tSenseIntvl	= psRule->para.u32[psRule->ActIdx][1] ; // Set new interval
	} else {
		SL_DBG("Invalid mode specified") ;
		return erFAILURE ;
	}
	IF_SL_DBG(debugTRACK, "Mode %d set to %d", psRule->para.u32[psRule->ActIdx][0], psRule->para.u32[psRule->ActIdx][1]) ;
	return erSUCCESS ;
}

#endif
