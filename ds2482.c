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


#include	"task_events.h"

#include	"rules_engine.h"
#include	"actuators.h"

#include	"x_printf.h"
#include	"x_buffers.h"
#include	"x_errors_events.h"
#include	"x_systiming.h"								// timing debugging
#include	"x_syslog.h"
#include	"x_formprint.h"

#include	"hal_debug.h"
#include	"hal_i2c.h"
#include	"onewire/ds2482.h"

#if		(configBUILD_WITH_DS1990X == 1)
	#include	"onewire/ds1990x.h"
#endif

#if		(configBUILD_WITH_DS18X20 == 1)
	#include	"onewire/ds18x20.h"
#endif

#if		(halHAS_PCA9555 == 1)
	#include	"pca9555/pca9555.h"
#endif

#if		(ESP32_PLATFORM == 1)
	#include	"esp32/rom/crc.h"						// ESP32 ROM routine
#else
	#include	"crc-barr.h"							// Barr group CRC
#endif

#include	<stdint.h>
#include	<string.h>

#define	debugFLAG					0xC00F

#define	debugTIMING					(debugFLAG & 0x0001)
#define	debugBUS_CFG				(debugFLAG & 0x0002)
#define	debugCONFIG					(debugFLAG & 0x0004)
#define	debugCRC					(debugFLAG & 0x0008)

#define	debugTRACK					(debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG & 0x8000)

// ###################################### General macros ###########################################

// DS2484 channel Number to Selection (1's complement) translation
const	uint8_t		ds2482_N2S[sd2482CHAN_NUM] = { 0xF0, 0xE1, 0xD2, 0xC3, 0xB4, 0xA5, 0x96, 0x87 } ;

// DS2482 channel Value(read/check)->Number xlat	0	  1		2	  3		4	  5		6	  7
const	uint8_t		ds2482_V2N[sd2482CHAN_NUM] = { 0xB8, 0xB1, 0xAA, 0xA3, 0x9C, 0x95, 0x8E, 0x87 } ;

// Used to fix the incorrect logical to physical 1-Wire mapping
const	uint8_t	OWremapTable[sd2482CHAN_NUM] = { 3,	2,	1,	0,	4,	5,	6,	7 } ;

/* https://www.maximintegrated.com/en/products/ibutton/software/1wire/wirekit.cfm
 * https://www.maximintegrated.com/en/app-notes/index.mvp/id/74
 *
 * ONEWIRE commands take a couple of formats resulting in a range of durations
 * 1. Instantaneous (0uS), no 1W bus activity, only affect DS2482, optional status
 * 2. Fast (< 1uS), no 1W bus activity, only affect DS2482,
 * 3. Medium (1uS < ?? < 1mS)
 * 4. Slow (> 1mS)
 * In order to optimise system performance minimal time should be spent in a tight
 * loop waiting for status, task should yield (delay) whenever possible.
 *				[DRST]	[SRP]	[WCFG]	[CHSL]	1WRST	1WWB	1WRB	1WSB	1WT
 *	Duration	525nS	0nS		0nS		0nS		1244uS	8x73uS	8x73uS	1x73uS	3x73uS
 */

uint8_t		ChannelCount[sd2482CHAN_NUM] = { 0 } ;
ds2482_t	sDS2482		= { 0 } ;

// ############################## DS2482-800 CORE support functions ################################

/**
 * Perform a device reset on the DS2482
 * Returns: true if device was reset
 *			 false device not detected or failure to perform reset
 */
int32_t ds2482Reset(void) {
// Device Reset
//	S AD,0 [A] DRST [A] Sr AD,1 [A] [SS] A\ P
//  [] indicates from slave
//  SS status byte to read to verify state
	uint8_t	cChr = CMD_DRST ;
	uint8_t status ;
	int32_t iRV = halI2C_WriteRead(&sDS2482.sI2Cdev, &cChr, sizeof(cChr), &status, sizeof(status)) ;
	IF_myASSERT(debugRESULT, iRV == erSUCCESS) ;
	NE_RETURN(iRV, erSUCCESS) ;
	sDS2482.Regs.Rstat	= status ;
	sDS2482.RegPntr		= ds2482REG_STAT ;
	sDS2482.CurChan		= 0 ;
	return ((status & ~STATUS_LL) == STATUS_RST) ;		// RESET true or false...
}

/**
 * DS2482_SetReadPointer()
 * Once set the pointer remains static to allow reread of same (normally status) register
 * Read Pointer will be changed by a new SRP command or by a device reset
 */
int32_t	ds2482SetReadPointer(uint8_t Reg) {
	IF_myASSERT(debugPARAM, Reg < ds2482REG_NUM) ;
	if (sDS2482.RegPntr == Reg) {
		return erSUCCESS ;
	}
	uint8_t	cBuf[2] = { CMD_SRP, (~Reg << 4) | Reg } ;
	int32_t iRV = halI2C_Write(&sDS2482.sI2Cdev, cBuf, sizeof(cBuf)) ;
	IF_myASSERT(debugRESULT, iRV == erSUCCESS) ;
	NE_RETURN(iRV, erSUCCESS) ;
// update the read pointer
	sDS2482.RegPntr = Reg ;
	return erSUCCESS ;
}

/**
 * Write the configuration register in the DS2482. The configuration
 * options are provided in the lower nibble of the provided config byte.
 * The uppper nibble in bitwise inverted when written to the DS2482.
 *
 * Returns:  true: config written and response correct
 *			  false: response incorrect
 */
int32_t ds2482WriteConfig(void) {
// Write configuration (Case A)
//	S AD,0 [A] WCFG [A] CF [A] Sr AD,1 [A] [CF] A\ P
//  [] indicates from slave
//  CF configuration byte to write
	IF_myASSERT(debugBUS_CFG, sDS2482.Regs.OWB == 0) ;				// check that bus not busy
	uint8_t	config = sDS2482.Regs.Rconf & 0x0F ;
	uint8_t	cBuf[2] = { CMD_WCFG , (~config << 4) | config } ;
	uint8_t new_conf ;
	int32_t iRV = halI2C_WriteRead(&sDS2482.sI2Cdev, cBuf, sizeof(cBuf), &new_conf, sizeof(new_conf)) ;
	IF_myASSERT(debugRESULT, iRV == erSUCCESS) ;
	NE_RETURN(iRV, erSUCCESS) ;
	// update the saved configuration
	sDS2482.Regs.Rconf	= new_conf ;
	sDS2482.RegPntr		= ds2482REG_CONF ;
	return 1 ;
}

/**
 * Select the 1-Wire channel on a DS2482-800.
 *
 * Returns: erSUCCESS if channel selected
 *			erFAILURE if device not detected or failure to perform select; else
 *			whatever status returned by halI2C_WriteRead()
 */
int32_t ds2482ChannelSelect(uint8_t Chan) {
// Channel Select (Case A)
//	S AD,0 [A] CHSL [A] CC [A] Sr AD,1 [A] [RR] A\ P
//  [] indicates from slave
//  CC channel value
//  RR channel read back
	IF_myASSERT(debugPARAM, Chan < sd2482CHAN_NUM) ;
	IF_myASSERT(debugBUS_CFG, sDS2482.Regs.OWB == 0) ;// check that bus not busy
	uint8_t	cBuf[2] = { CMD_CHSL, ds2482_N2S[Chan] } ;
	uint8_t ChanRet ;
	int32_t iRV = halI2C_WriteRead(&sDS2482.sI2Cdev, cBuf, sizeof(cBuf), &ChanRet, sizeof(ChanRet)) ;
	IF_myASSERT(debugRESULT, iRV == erSUCCESS) ;
	NE_RETURN(iRV, erSUCCESS) ;

	sDS2482.RegPntr		= ds2482REG_CHAN ;			// update the read pointer
	sDS2482.Regs.Rchan	= ChanRet ;					// update channel code (read back)
	/* value read back not same as the channel number sent so verify the return
	 * against the code expected, but store the actual channel number if successful */
	if (ChanRet != ds2482_V2N[Chan]) {
		SL_ERR("Read %d != %d Expected", ChanRet, ds2482_V2N[Chan]) ;
		IF_myASSERT(debugRESULT, 0) ;
		return erFAILURE ;
	}
	sDS2482.CurChan		= Chan ;					// and the actual (normalized) channel number
	return erSUCCESS ;
}

int32_t	ds2482Write(uint8_t * pTxBuf, size_t TxSize) {
	IF_myASSERT(debugBUS_CFG, sDS2482.Regs.OWB == 0)	;
	int32_t iRV = halI2C_Write(&sDS2482.sI2Cdev, pTxBuf, TxSize) ;
	IF_myASSERT(debugRESULT, iRV == erSUCCESS) ;
	return iRV ;
}

int32_t	ds2482WaitNotBusy(int32_t Delay) {
	int32_t	iRV, Retry = 20 ;
	uint8_t	Status ;
	do {
		vTaskDelay(Delay) ;
		iRV = halI2C_Read(&sDS2482.sI2Cdev, &Status, sizeof(Status)) ;
		IF_myASSERT(debugRESULT, iRV == erSUCCESS) ;
	} while ((Status & STATUS_1WB) && --Retry) ;
	if (Retry == 0 || (Status & STATUS_1WB)) {
		IF_myASSERT(debugRESULT, 0) ;
		return erFAILURE ;
	}
	sDS2482.Regs.Rstat	= Status ;
	sDS2482.RegPntr		= ds2482REG_STAT ;
	return iRV ;
}

/**
 * ds2482WriteAndWait()
 * \brief	Primarily to handle the Maxim/Dallas 1-Wire protocol and the waiting for status
 * 			Perform repeated reads waiting for 1WB status bit to be cleared.
 * @return	erSUCCESS or erFAILURE
 */
int32_t	ds2482WriteAndWait(uint8_t * pTxBuf, size_t TxSize, size_t Delay) {
	int32_t iRV = ds2482Write(pTxBuf, TxSize) ;
	NE_RETURN(iRV, erSUCCESS) ;
	if (Delay) {
		iRV = ds2482WaitNotBusy(Delay - 1) ;
	}
	return iRV ;
}

/**
 * Use the DS2482 help command '1-Wire triplet' to perform one bit of a 1-Wire
 * search. This command does two read bits and one write bit. The write bit
 * is either the default direction (all device have same bit) or in case of
 * a discrepancy, the 'search_direction' parameter is used.
 *
 * Returns – The DS2482 status byte result from the triplet command
 */
uint8_t ds2482SearchTriplet(uint8_t search_direction) {
// 1-Wire Triplet (Case B)
//	S AD,0 [A] 1WT [A] SS [A] Sr AD,1 [A] [Status] A [Status] A\ P
//							  \--------/
//				Repeat until 1WB bit has changed to 0
//  [] indicates from slave
//  SS indicates byte containing search direction bit value in msbit
	IF_myASSERT(debugPARAM, search_direction < 2) ;
	uint8_t	cBuf[2] = { CMD_1WT, search_direction ? 0x80 : 0x00 } ;
	if (ds2482WriteAndWait(cBuf, sizeof(cBuf), owDELAY_ST) == erFAILURE) {
		ds2482Reset();
		return 0;
	}
	return sDS2482.Regs.Rstat ;
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
int32_t ds2482Detect(void) {
	if (ds2482Reset() == 0) {
		return 0;
	}
	// default configuration 0xE1 (0xE? is the 1s complement of 0x?1)
	sDS2482.Regs.APU	= 1 ;							// LSBit
	sDS2482.Regs.RES2	= 0 ;
	sDS2482.Regs.SPU	= 0 ;
	sDS2482.Regs.OWS	= 0 ;
	sDS2482.Regs.RES1	= 0 ;							// MSBit
	// confirm bit packing order is correct
	IF_myASSERT(debugCONFIG, sDS2482.Regs.Rconf == CONFIG_APU) ;
	return ds2482WriteConfig() ;
}

// ############################### DS2482-800 DEBUG support functions ##############################

/**
 * Read a register based on last/current Read Pointer status
 */
uint8_t	ds2482ReadRegister(uint8_t Reg) {
	IF_myASSERT(debugPARAM, Reg < ds2482REG_NUM)
	int32_t	iRV = halI2C_Read(&sDS2482.sI2Cdev, (uint8_t *) &sDS2482.Regs.RegX[Reg], sizeof(uint8_t)) ;
	if (iRV != erSUCCESS) {
		return 0 ;
	}
	return 1 ;
}

/**
 * ds2482PrintROM() - print the 1-Wire ROM information
 * @param psOW_ROM
 */
void	ds2482PrintROM(ow_rom_t * psOW_ROM) { PRINT("%02X/%#M/%02X\n", psOW_ROM->Family, psOW_ROM->TagNum, psOW_ROM->CRC) ; }

/**
 * Display register contents
 */
void	ds2482PrintRegisters(void) {
	// Status
	PRINT("STAT(0)=0x%02X  DIR=%c  TSB=%c  SBR=%c  RST=%c  LL=%c  SD=%c  PPD=%c  1WB=%c\n",
				sDS2482.Regs.Rstat,
				sDS2482.Regs.DIR ? '1' : '0',
				sDS2482.Regs.TSB ? '1' : '0',
				sDS2482.Regs.SBR ? '1' : '0',
				sDS2482.Regs.RST ? '1' : '0',
				sDS2482.Regs.LL  ? '1' : '0',
				sDS2482.Regs.SD  ? '1' : '0',
				sDS2482.Regs.PPD ? '1' : '0',
				sDS2482.Regs.OWB ? '1' : '0') ;
	PRINT("DATA(1)=0x%02X\n", sDS2482.Regs.Rdata) ;	// Data

	int32_t Chan ;										// Channel, start by finding the matching Channel #
	for (Chan = sd2482CHAN_0; Chan < sd2482CHAN_NUM && sDS2482.Regs.Rchan != ds2482_V2N[Chan]; ++Chan) ;
	IF_myASSERT(debugRESULT, Chan < sd2482CHAN_NUM) ;
	PRINT("CHAN(2)=0x%02X ==> %d\n", sDS2482.Regs.Rchan, Chan) ;

	PRINT("CONF(3)=0x%02X  1WS=%c  SPU=%c  APU=%c\n",	// Configuration
			sDS2482.Regs.Rconf,
			sDS2482.Regs.OWS	? '1' : '0',
			sDS2482.Regs.SPU	? '1' : '0',
			sDS2482.Regs.APU	? '1' : '0') ;
}

/**
 * Read ALL the registers
 */
uint8_t	ds2482ReadRegisters(void) {
	for (uint8_t Reg = ds2482REG_STAT; Reg < ds2482REG_NUM; ++Reg) {
		ds2482SetReadPointer(Reg) ;
		if (ds2482ReadRegister(Reg) == 0) {
			return 0 ;
		}
	}
	return 1 ;
}

uint8_t	ds2482Report(void) {
	if (ds2482ReadRegisters() == 1) {
		ds2482PrintRegisters() ;
	}
	return 1 ;
}

// ################################## Bit/Byte Read/Write ##########################################

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
uint8_t OWTouchBit(uint8_t sendbit) {
// 1-Wire bit (Case B)
//	S AD,0 [A] 1WSB [A] BB [A] Sr AD,1 [A] [Status] A [Status] A\ P
//										   \--------/
//								Repeat until 1WB bit has changed to 0
//  [] indicates from slave
//  BB indicates byte containing bit value in msbit
	IF_myASSERT(debugPARAM, sendbit < 2) ;
	IF_myASSERT(debugBUS_CFG, sDS2482.Regs.OWB == 0)	;
	uint8_t	cBuf[2] ;
	cBuf[0]	= CMD_1WSB ;
	cBuf[1] = sendbit ? 0x80 : 0x00 ;
	if (ds2482WriteAndWait(cBuf, sizeof(cBuf), owDELAY_TB) == erFAILURE) {
		return 0;
	}
// return bit state
	return sDS2482.Regs.SBR ;
}

/**
 * Send 1 bit of communication to the 1-Wire Net.
 * The parameter 'sendbit' least significant bit is used.
 *
 * 'sendbit' - 1 bit to send (least significant byte)
 */
void	OWWriteBit(uint8_t sendbit) { OWTouchBit(sendbit); }

/**
 * Read 1 bit of communication from the 1-Wire Net and return the result
 *
 * Returns:  1 bit read from 1-Wire Net
 */
uint8_t OWReadBit(void) { return OWTouchBit(0x01) ; }

/**
 * Send 8 bits of communication to the 1-Wire Net and verify that the
 * 8 bits read from the 1-Wire Net is the same (write operation).
 * The parameter 'sendbyte' least significant 8 bits are used.
 *
 * 'sendbyte' - 8 bits to send (least significant byte)
 * @return	erSUCCESS or erFAILURE
 */
int32_t	OWWriteByte(uint8_t sendbyte) {
// 1-Wire Write Byte (Case B)
//	S AD,0 [A] 1WWB [A] DD [A] Sr AD,1 [A] [Status] A [Status] A\ P
//										   \--------/
//							Repeat until 1WB bit has changed to 0
//  [] indicates from slave
//  DD data to write
	uint8_t	cBuf[2] = { CMD_1WWB, sendbyte } ;
	int32_t iRV = ds2482Write(cBuf, sizeof(cBuf)) ;
	IF_myASSERT(debugRESULT, iRV > erFAILURE) ;
	return iRV ;
}

int32_t	OWWriteByteWait(uint8_t sendbyte) {
	int32_t iRV = OWWriteByte(sendbyte) ;
	IF_myASSERT(debugRESULT, iRV == erSUCCESS) ;
	iRV = ds2482WaitNotBusy(owDELAY_WB) ;
	IF_myASSERT(debugRESULT, iRV == erSUCCESS) ;
	return iRV ;
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
int32_t OWWriteBytePower(int32_t sendbyte) {
	sDS2482.Regs.SPU = 1 ;
	if (ds2482WriteConfig() == 0) {
		IF_myASSERT(debugRESULT, 0) ;
		return 0 ;
	}
	int32_t iRV = OWWriteByte(sendbyte) ;
	IF_myASSERT(debugRESULT, iRV == erSUCCESS && sDS2482.Regs.SPU == 1) ;
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
int32_t OWReadBitPower(int32_t applyPowerResponse) {
	sDS2482.Regs.SPU = 1 ;
	if (ds2482WriteConfig() == 0) {
		return 0 ;
	}
	uint8_t rdbit = OWReadBit();
	if (rdbit != applyPowerResponse) {					// check if response was correct
		OWLevel(owMODE_STANDARD);						// if not, turn off strong pull-up
		return 0 ;
	}
	return 1 ;
}

/**
 * Send 8 bits of read communication to the 1-Wire Net and return the
 * result 8 bits read from the 1-Wire Net.
 *
 * Returns:  8 bits read from 1-Wire Net
 */
int32_t	OWReadByte(void) {
/* 1-Wire Read Bytes (Case C)
 *	S AD,0 [A] 1WRB [A] Sr AD,1 [A] [Status] A [Status] A\
 *										\--------/
 *							Repeat until 1WB bit has changed to 0
 *	Sr AD,0 [A] SRP [A] E1 [A] Sr AD,1 [A] DD A\ P
 *
 *  [] indicates from slave
 *  DD data read
 */
	int32_t iRV = OWWriteByteWait(CMD_1WRB) ;			// send the READ command
	IF_myASSERT(debugRESULT, iRV > erFAILURE) ;

	iRV = ds2482SetReadPointer(ds2482REG_DATA) ;		// set pointer to data register
	IF_myASSERT(debugRESULT, iRV > erFAILURE) ;

	uint8_t	cRead ;
	iRV = halI2C_Read(&sDS2482.sI2Cdev, &cRead, sizeof(cRead)) ;			// read the register
	IF_myASSERT(debugRESULT, iRV > erFAILURE) ;
	return cRead ;
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
uint8_t OWTouchByte(uint8_t sendbyte) {
	if (sendbyte == 0xFF) {
		return OWReadByte();
	} else {
		OWWriteByteWait(sendbyte);
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
void	OWBlock(uint8_t * tran_buf, int32_t tran_len) {
	for (int32_t i = 0; i < tran_len; ++i) {
		tran_buf[i] = OWTouchByte(tran_buf[i]) ;
	}
}

/**
 * OWReadROM() - Check PPD, send command and loop for 8byte read
 * @brief	To be used if only a single device on a bus and the ROM ID must be read
 * 			Probably will fail if more than 1 device on the bus
 * @return	erFAILURE or CRC byte
 */
int32_t	OWReadROM(void) {
	int32_t iRV = OWWriteByteWait(OW_CMD_READROM) ;
	LT_GOTO(iRV, erSUCCESS, exit) ;

	sDS2482.ROM.Value = 0ULL ;
	do {
		for (uint8_t i = 0; i < ONEWIRE_ROM_LENGTH; ++i) {
			iRV = OWReadByte() ;							// read 8x bytes making up the ROM FAM+ID+CRC
			LT_GOTO(iRV, erSUCCESS, exit) ;
			sDS2482.ROM.HexChars[i] = iRV ;
		}
	} while (OWCheckCRC(sDS2482.ROM.HexChars, ONEWIRE_ROM_LENGTH) == 0) ;
exit:
	return iRV ;
}

// ################################ Generic 1-Wire LINK API's ######################################

/**
 * OWCheckCRC() - Checks if CRC is ok (ROM Code or Scratch PAD RAM)
 * @param	buf			buffer to be checked for proper CRC
 * @param	buflen		buffer length
 * @return	TRUE if the CRC is correct, FALSE otherwise
 */
uint8_t	OWCheckCRC(uint8_t * buf, uint8_t buflen) {
	uint8_t shift_reg = 0 ;
	for (int8_t i = 0; i < buflen; i++) {
		for (int8_t j = 0; j < CHAR_BIT; j++) {
			uint8_t	data_bit = (buf[i] >> j) & 0x01 ;
			uint8_t	sr_lsb = shift_reg & 0x01 ;
			uint8_t	fb_bit = (data_bit ^ sr_lsb) & 0x01 ;
			shift_reg = shift_reg >> 1 ;
			if (fb_bit) {
				shift_reg = shift_reg ^ 0x8c ;
			}
		}
	}
	IF_PRINT(debugCRC && shift_reg, "CRC=%x FAIL %'-+b\n", shift_reg, buflen, buf) ;
	return (shift_reg == 0) ? 1 : 0 ;
}

/**
 * OWCalcCRC8() - Calculate the CRC8 of the byte value provided with the current 'crc8' value
 * @brief	See Application Note 27
 * @param	data
 * @return				Returns current crc8 value
 */
uint8_t	OWCalcCRC8(uint8_t data) {
	sDS2482.crc8 = sDS2482.crc8 ^ data;
	for (int32_t i = 0; i < BITS_IN_BYTE; ++i) {
		if (sDS2482.crc8 & 1) {
			sDS2482.crc8 = (sDS2482.crc8 >> 1) ^ 0x8c;
		} else {
			sDS2482.crc8 = (sDS2482.crc8 >> 1);
		}
	}
	return sDS2482.crc8;
}

/**
 * Reset all of the devices on the 1-Wire Net and return the result.
 *
 * Returns: true(1):  presence pulse(s) detected, device(s) reset
 *			 false(0): no presence pulses detected
 */
int32_t OWReset(void) {
// 1-Wire reset (Case B)
//	S AD,0 [A] 1WRS [A] Sr AD,1 [A] [Status] A [Status] A\ P
//									\--------/
//						Repeat until 1WB bit has changed to 0
//  [] indicates from slave
	IF_myASSERT(debugBUS_CFG, sDS2482.Regs.OWB == 0 && sDS2482.Regs.SPU == 0) ;
	uint8_t	cChr = CMD_1WRS ;
	if (ds2482WriteAndWait(&cChr, sizeof(cChr), owDELAY_RST) == erFAILURE) {
		ds2482Reset();
		return 0;
	}
	return sDS2482.Regs.PPD ;
}

/**
 * Set the 1-Wire Net communication speed.
 * 'new_speed' - new speed defined as
 *					 owMODE_STANDARD	0x00
 *					 owMODE_OVERDRIVE	0x01
 *
 * Returns:  new current 1-Wire Net speed
 */
int32_t OWSpeed(int32_t new_speed) {
	if (new_speed == owMODE_STANDARD) {
		sDS2482.Regs.OWS = 0 ;						// AMM swapped these around from default!!!
	} else {
		sDS2482.Regs.OWS = 1 ;
	}
	ds2482WriteConfig();
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
int32_t OWLevel(int32_t new_level) {
	if (new_level != owMODE_STANDARD) {
		return owMODE_STRONG ;
	}
	sDS2482.Regs.SPU = 0 ;
	ds2482WriteConfig() ;
	return sDS2482.Regs.SPU ;
}

/**
 * OWAddress() - Addresses a single or all devices on the 1-wire bus
 * @param nAddrMethod	use OW_CMD_MATCHROM to select a single
 *						device or OW_CMD_SKIPROM to select all
 */
void	OWAddress(uint8_t nAddrMethod) {
	int32_t iRV ;
	if (nAddrMethod == OW_CMD_MATCHROM) {
		iRV = OWWriteByteWait(OW_CMD_MATCHROM) ;		// address single/individual device
		IF_myASSERT(debugRESULT, iRV > erFAILURE) ;
		for (uint8_t i = 0; i < ONEWIRE_ROM_LENGTH; ++i) {
			iRV = OWWriteByteWait(sDS2482.ROM.HexChars[i]);
			IF_myASSERT(debugRESULT, iRV > erFAILURE) ;
		}
	} else {
		iRV = OWWriteByteWait(OW_CMD_SKIPROM) ;			// address all devices
		IF_myASSERT(debugRESULT, iRV > erFAILURE) ;
	}
}

// ############################## Search and Variations thereof ####################################

/**
 * The 'OWSearch' function does a general search.  This function
 * continues from the previous search state. The search state
 * can be reset by using the 'OWFirst' function.
 * This function contains one parameter 'alarm_only'.
 * When 'alarm_only' is true (1) the find alarm command
 * 0xEC(OW_CMD_ALARMSEARCH) is sent instead of
 * the normal search command 0xF0(OW_CMD_SEARCHROM).
 * Using the find alarm command 0xEC will limit the search to only
 * 1-Wire devices that are in an 'alarm' state.
 *
 * Returns:	true (1) : when a 1-Wire device was found and its
 *						  Serial Number placed in the global ROM
 *			false (0): when no new device was found.  Either the
 *						  last search was the last device or there
 *						  are no devices on the 1-Wire Net.
 */
int32_t OWSearch(void) {
	int32_t	id_bit_number = 1, last_zero = 0, rom_byte_number = 0, search_result = 0;
	uint8_t	rom_byte_mask = 1;
	sDS2482.crc8 = 0;
	if (sDS2482.LastDeviceFlag == 0) {					// if the last call was not the last device
		if (OWReset() == 0) {							// reset the search
			sDS2482.LastDiscrepancy			= 0 ;
			sDS2482.LastDeviceFlag			= 0 ;
			sDS2482.LastFamilyDiscrepancy	= 0 ;
			return 0;
		}
		OWWriteByteWait(OW_CMD_SEARCHROM) ;				// search for device
		uint8_t search_direction ;
		do {											// loop to do the search
		// if this discrepancy is before the Last Discrepancy
		// on a previous next then pick the same as last time
			if (id_bit_number < sDS2482.LastDiscrepancy) {
				if ((sDS2482.ROM.HexChars[rom_byte_number] & rom_byte_mask) > 0) {
					search_direction = 1 ;
				} else {
					search_direction = 0 ;
				}
			} else {									// if equal to last pick 1, if not then pick 0
				if (id_bit_number == sDS2482.LastDiscrepancy) {
					search_direction = 1 ;
				} else {
					search_direction = 0 ;
				}
			}
		// Perform a triple operation on the DS2482 which will perform 2 read bits and 1 write bit
			uint8_t status = ds2482SearchTriplet(search_direction) ;
		// check bit results in status byte
			int32_t	id_bit		= ((status & STATUS_SBR) == STATUS_SBR) ;
			int32_t	cmp_id_bit	= ((status & STATUS_TSB) == STATUS_TSB) ;
			search_direction	= ((status & STATUS_DIR) == STATUS_DIR) ? 1 : 0 ;
			if ((id_bit) && (cmp_id_bit)) {				// check for no devices on 1-Wire
				break ;
			} else {
				if ((!id_bit) && (!cmp_id_bit) && (search_direction == 0)) {
					last_zero = id_bit_number ;
					if (last_zero < 9) {				// check for Last discrepancy in family
						sDS2482.LastFamilyDiscrepancy = last_zero ;
					}
				}
				if (search_direction == 1) {			// set or clear the bit in the ROM byte rom_byte_number with mask rom_byte_mask
					sDS2482.ROM.HexChars[rom_byte_number] |= rom_byte_mask ;
				} else {
					sDS2482.ROM.HexChars[rom_byte_number] &= ~rom_byte_mask ;
				}
				++id_bit_number ;						// increment the byte counter id_bit_number & shift the mask rom_byte_mask
				rom_byte_mask <<= 1 ;
				if (rom_byte_mask == 0) {				// if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
					OWCalcCRC8(sDS2482.ROM.HexChars[rom_byte_number]);  // accumulate the CRC
					++rom_byte_number ;
					rom_byte_mask = 1 ;
				}
			}
		} while(rom_byte_number < ONEWIRE_ROM_LENGTH) ;  // loop until all

	// if the search was successful then
		if (!((id_bit_number < 65) || (sDS2482.crc8 != 0))) {
			sDS2482.LastDiscrepancy = last_zero;		// search successful, set LastDiscrepancy,LastDeviceFlag,search_result
			if (sDS2482.LastDiscrepancy == 0) {			// check for last device
				sDS2482.LastDeviceFlag	= 1 ;
			}
			search_result = 1 ;
		}
	}

	// if no device found then reset counters so next 'search' will be like a first
	if (!search_result || (sDS2482.ROM.Family == 0)) {
		sDS2482.LastDiscrepancy	= 0 ;
		sDS2482.LastDeviceFlag	= 0 ;
		sDS2482.LastFamilyDiscrepancy = 0 ;
		search_result = 0 ;
	}
	return search_result;
}

/**
 * Find the 'first' device on the 1W network
 * Return true  : device found, ROM number in ROM.Number buffer
 *		  false : no device present
 */
int32_t OWFirst(void) {
	sDS2482.LastDiscrepancy			= 0 ;				// reset the search state
	sDS2482.LastFamilyDiscrepancy	= 0 ;
	sDS2482.LastDeviceFlag			= 0 ;
	return OWSearch() ;
}

/**
 * Find the 'next' device on the 1W network
 * Return true  : device found, ROM number in ROM.Number buffer
 *		  false : device not found, end of search
 */
int32_t OWNext(void) { return OWSearch() ; }

/**
 * Verify the device with the ROM number in ROM buffer is present.
 * Return true  : device verified present
 *		  false : device not present
 */
int32_t OWVerify(void) {
	uint8_t rom_backup[ONEWIRE_ROM_LENGTH] ;
	// make a backup copy of the current state
	memcpy(rom_backup, sDS2482.ROM.HexChars, sizeof(ow_rom_t)) ;
	int32_t	ld_backup				= sDS2482.LastDiscrepancy ;
	int32_t	ldf_backup				= sDS2482.LastDeviceFlag ;
	int32_t	lfd_backup				= sDS2482.LastFamilyDiscrepancy;
	sDS2482.LastDiscrepancy			= 64 ;				// set search to find the same device
	sDS2482.LastDeviceFlag			= 0 ;

	int32_t iRV ;
	if ((iRV = OWSearch()) == 1) {
		for (int32_t i = 0; i < ONEWIRE_ROM_LENGTH; ++i) {		// check if same device found
			if (rom_backup[i] != sDS2482.ROM.HexChars[i]) {
				iRV = 0 ;
				break ;
			}
		}
	}
// restore the search state
	memcpy(sDS2482.ROM.HexChars, rom_backup, sizeof(ow_rom_t)) ;
	sDS2482.LastDiscrepancy			= ld_backup;
	sDS2482.LastDeviceFlag			= ldf_backup;
	sDS2482.LastFamilyDiscrepancy	= lfd_backup;
	return iRV ;								// return the result of the verify
}

/**
 * Setup search to find the first 'family_code' device on the next call to OWNext().
 * If no (more) devices of 'family_code' can be found return first device of next family
 */
void	OWTargetSetup(uint8_t family_code) {
	sDS2482.ROM.Value				= 0ULL ;			// reset all ROM fields
	sDS2482.ROM.Family 				= family_code ;
	sDS2482.LastDiscrepancy			= 64 ;
	sDS2482.LastFamilyDiscrepancy	= 0 ;
	sDS2482.LastDeviceFlag			= 0 ;
}

/**
 * Setup the search to skip the current device family on the next call to OWNext().
 * Can ONLY be done after a search had been performed.
 * Will find the first device of the next family.
 */
void	OWFamilySkipSetup(void) {
	sDS2482.LastDiscrepancy = sDS2482.LastFamilyDiscrepancy ;	// set the Last discrepancy to last family discrepancy
	sDS2482.LastFamilyDiscrepancy = 0 ;				// clear the last family discrepancy
	if (sDS2482.LastDiscrepancy == 0) {				// check for end of list
		sDS2482.LastDeviceFlag	= 1 ;
	}
}

// ################################# Application support functions #################################

/**
 * ds2482HandleFamilies() - Call handler based on device family
 * @return	return value from handler or
 */
int32_t	ds2482HandleFamilies(int32_t iCount, void * pVoid) {
	int32_t	iRV = erFAILURE ;
	switch (sDS2482.ROM.Family) {
#if		(configBUILD_WITH_DS1990X == 1)
	case OWFAMILY_01:							// DS1990A/R, 2401/11 devices
		iRV = ds1990xHandleRead(iCount, pVoid) ;
		break ;
#endif

#if		(configBUILD_WITH_DS18X20 == 1)
	case OWFAMILY_10:							// DS18S20 Thermometer
	case OWFAMILY_28:							// DS18B20 Thermometer
		iRV = ds18x20Handler(iCount, pVoid) ;
		break ;
#endif

	default:
		SL_ERR("Invalid OW device FAM=%02x", sDS2482.ROM.Family) ;
	}
	return iRV ;
}

/**
 * ds2482ScanChannel() - Scan preselected channel for all devices of [specified] family
 * @brief	Channel must be preselected,
 * @param	Family
 * @param	Handler
 * @return	erFAILURE if an error occurred
 * 			erSUCCESS if no [matching] device found or no error returned
 */
int32_t	ds2482ScanChannel(uint8_t Family, int32_t (* Handler)(int32_t, void *), int32_t xCount, void * pVoid) {
#if 0
	int32_t	iCount = 0 ;
	int32_t	iRV = OWFirst() ;
	while (iRV == 1) {
		iRV = OWCheckCRC(sDS2482.ROM.HexChars, sizeof(ow_rom_t)) ;
		myASSERT(iRV == 1) ;
		if (Family == 0 || Family == sDS2482.ROM.Family) {
			if (Handler) {
				iRV = Handler(xCount + iCount, pVoid) ;
				LT_BREAK(iRV, erSUCCESS) ;
			}
			++iCount ;
		} else {
			OWFamilySkipSetup() ;
		}
		iRV = OWNext() ;								// try to find next device (if any)
	}
	return iRV < erSUCCESS ? iRV : iCount ;
#else
	int32_t	iCount = 0 ;
	OWTargetSetup(Family) ;
	int32_t	iRV = OWSearch() ;
	while (iRV == 1) {
		iRV = OWCheckCRC(sDS2482.ROM.HexChars, sizeof(ow_rom_t)) ;
		myASSERT(iRV == 1) ;
		if (Family == 0 || Family == sDS2482.ROM.Family) {
			if (Handler) {
				iRV = Handler(xCount + iCount, pVoid) ;
				LT_BREAK(iRV, erSUCCESS) ;
			}
			++iCount ;
		}
		iRV = OWNext() ;								// try to find next device (if any)
	}
	return iRV < erSUCCESS ? iRV : iCount ;
#endif
}

/**
 * ds2482ScanAllChanAllFam() - scan ALL channels sequentially for [specified] family
 * @return
 */
int32_t	ds2482ScanAllChannels(uint8_t Family, int (* Handler)(int32_t, void *), void * pVoid) {
	int32_t	iRV = erSUCCESS, xCount = 0 ;
	xRtosSemaphoreTake(&sDS2482.Mux, portMAX_DELAY) ;
	for (uint8_t Chan = sd2482CHAN_0; Chan < sd2482CHAN_NUM; ++Chan) {
		iRV = ds2482ChannelSelect(Chan) ;
		LT_BREAK(iRV, erSUCCESS) ;
		iRV = ds2482ScanChannel(Family, Handler, xCount, pVoid) ;
		LT_BREAK(iRV, erSUCCESS) ;						// if callback failed, return
		xCount += iRV ;									// update running count
	}
	xRtosSemaphoreGive(&sDS2482.Mux) ;
	IF_SL_ERR(iRV < erSUCCESS, "iRV=%d", iRV) ;
	return iRV < erSUCCESS ? iRV : xCount ;
}

// ################### Identification, Diagnostics & Configuration functions #######################

/**
 * DS2482Diagnostics() - perform diagnostics on DS2482 device & channels
 * @brief			Scan bus for iButtons, all 8 channels:
 * @brief			If no buttons found,	16	mS
 * @brief			If 1 button found,		81	mSec	+65 mSec
 * @brief			If 2 buttons found,		146	mSec	+65	mSec
 * @brief			Estimated 8 buttons		536 mSec	+520 mSec
 * @brief			Pre-selection of a specific family makes no difference
 * @return			erSUCCESS or erFAILURE
 */
int32_t	ds2482Diagnostics(void) {
	if (ds2482Report() == 0) {
		return erFAILURE ;
	}
	return erSUCCESS ;
}

/**
 * DS2482CountDevices() - Scan all buses and count/list all devices on all channels
 * @param pDS24282	pointer to device structure
 * @return			number of devices found
 */
int32_t	ds2482CountDevices(void) {
	int32_t	iCount = 0 ;
	for (int32_t Chan = sd2482CHAN_0; Chan < sd2482CHAN_NUM; ++Chan) {
		int32_t iRV = ds2482ChannelSelect(Chan) ;
		EQ_RETURN(iRV, erFAILURE) ;

#if		(ds18x20PWR_SOURCE == 1)
		xActuatorBlock(Chan) ;
		vActuateSetLevelDIG(Chan, 1) ;
		pca9555DIG_OUT_WriteAll() ;
		int32_t	PwrFlag = 0 ;
#endif

		iRV = OWFirst() ;
		while (iRV == 1) {
			switch (sDS2482.ROM.Family) {
#if		(configBUILD_WITH_DS1990X == 1)
			case OWFAMILY_01:							// DS1990A/R, 2401/11 devices
				++Family01Count ;						// count ONLY for sake of reporting
				break ;
#endif

#if		(configBUILD_WITH_DS18X20 == 1)
			case OWFAMILY_10:							// DS1820 & DS18S20 Thermometer
			case OWFAMILY_28:							// DS18B20 Thermometer
				++Fam10_28Count ;
#if		(ds18x20PWR_SOURCE == 1)
				++PwrFlag ;								// Set flag to leave power on
#endif
				break ;
#endif

			default:
				SL_ERR("Invalid/unsupported 1W family '0x%02X' found", sDS2482.ROM.Family) ;
			}
			++ChannelCount[Chan] ;
			++iCount ;
			IF_EXEC_1(debugTRACK, ds2482PrintROM, &sDS2482.ROM) ;
			iRV = OWNext() ;
		}
#if		(ds18x20PWR_SOURCE == 1)
		if (PwrFlag == 0) {
			vActuateSetLevelDIG(Chan, 0) ;
			pca9555DIG_OUT_WriteAll() ;
			xActuatorUnBlock(Chan) ;
		}
#endif
		EQ_RETURN(iRV, erFAILURE) ;
	}
	IF_PRINT(debugTRACK, "DS2482: Found %d device(s)\n", iCount) ;
	return iCount ;
}

/**
 * halDS2482_Identify() - Try to identify I2C device
 * @param eChan			halI2C channel to use
 * @param Addr			I2C address where device is located
 * @return				erSUCCESS if device identifies as DS2482 else erFAILURE
 */
int32_t	ds2482Identify(uint8_t chanI2C, uint8_t addrI2C) {
	IF_myASSERT(debugPARAM, chanI2C < halI2C_NUM) ;
	sDS2482.sI2Cdev.chanI2C	= chanI2C ;
	sDS2482.sI2Cdev.addrI2C	= addrI2C ;
//	sDS2482.sI2Cdev.dlayI2C	= pdMS_TO_TICKS(750) ;
	sDS2482.sI2Cdev.dlayI2C	= pdMS_TO_TICKS(10) ;
	if (ds2482Detect() == 0) {							// if no device found
		sDS2482.sI2Cdev.chanI2C			= 0 ;			// reset all & return
		sDS2482.sI2Cdev.addrI2C			= 0 ;
		sDS2482.sI2Cdev.dlayI2C			= 0 ;
		return erFAILURE ;
	}
	sDS2482.Mux	= xSemaphoreCreateMutex() ;
	return erSUCCESS ;
}

int32_t	ds2482Config(void) {
	IF_SYSTIMER_INIT(debugTIMING, systimerDS2482A, systimerTICKS, "DS2482A", myMS_TO_TICKS(30), myMS_TO_TICKS(150)) ;
	IF_SYSTIMER_INIT(debugTIMING, systimerDS2482B, systimerTICKS, "DS2482B", myMS_TO_TICKS(1), myMS_TO_TICKS(20)) ;
	IF_SYSTIMER_INIT(debugTIMING, systimerDS2482WW, systimerTICKS, "DS2482WW", myMS_TO_TICKS(1), myMS_TO_TICKS(10)) ;

	int32_t iRV = ds2482CountDevices() ;
	LT_RETURN(iRV, 1) ;

#if		(configBUILD_WITH_DS1990X == 1)
	ds1990xDiscover() ;
#endif

#if		(configBUILD_WITH_DS18X20 == 1)
	ds18x20Discover(URI_DS18X20) ;
#endif
	return erSUCCESS ;
}

int32_t	ds2482TestsHandler(int32_t iCount, void * pVoid) {
	return PRINT("#%d Ch%d %02X/%#M/%02X  ", iCount, sDS2482.CurChan, sDS2482.ROM.Family, sDS2482.ROM.TagNum, sDS2482.ROM.CRC) ;
}

void	ds2482Tests(void) {
	PRINT("\nChecking") ;
#if		(configBUILD_WITH_DS1990X == 1)
	PRINT("\nF01 ") ;
	ds2482ScanAllChannels(OWFAMILY_01, ds2482TestsHandler, NULL) ;
#endif

#if		(configBUILD_WITH_DS18X20 == 1)
	PRINT("\nF28 ") ;
	ds2482ScanAllChannels(OWFAMILY_28, ds2482TestsHandler, NULL) ;
#endif

#if		(configBUILD_WITH_DS1990X == 1)
	PRINT("\nF01 ") ;
	ds2482ScanAllChannels(OWFAMILY_01, ds2482TestsHandler, NULL) ;
#endif
	PRINT("\n") ;
}
