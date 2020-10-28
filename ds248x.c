/*
 * Copyright 2020 AM Maree/KSS Technologies (Pty) Ltd.
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
 * ds248x.c
 */

#include	"onewire_platform.h"
#include	"printfx.h"
#include	"syslog.h"
#include	"systiming.h"								// timing debugging
#include	"x_errors_events.h"
#include	"x_string_general.h"

#include	"hal_debug.h"

#include	<string.h>

#define	debugFLAG					0xF007

#define	debugBUS_CFG				(debugFLAG & 0x0001)
#define	debugCONFIG					(debugFLAG & 0x0002)
#define	debugCRC					(debugFLAG & 0x0004)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ##################################### Developer notes ###########################################
/*
	Test at 400KHx I2C speed, maybe add auto detect and step up mode in SCAN routine?
	Add support to configure the PADJ register timing?
 */

// ###################################### General macros ###########################################


// ###################################### Local variables ##########################################

const char * const RegNames[ds248xREG_NUM] = {"Stat", "Data", "Chan", "Conf", "Port" } ;
const char * const StatNames[8] = { "OWB", "PPD", "SD", "LL", "RST", "SBR", "TSB", "DIR" } ;
#if		(halHAS_DS2482_800 > 0)
	// DS2482-800 channel# check xlat	0	  1		2	  3		4	  5		6	  7
	static const uint8_t	ds248x_V2N[8] = { 0xB8, 0xB1, 0xAA, 0xA3, 0x9C, 0x95, 0x8E, 0x87 } ;
#endif
#if		(halHAS_DS2484 > 0)
	static const uint8_t	Trstl[16]	= { 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74 } ;
	static const uint8_t	Tmsp0[16]	= { 58, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 76, 76, 76, 76, 76 } ;
	static const uint8_t	Tmsp1[16]	= { 55, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 110, 110, 110 } ;
	static const uint8_t	Twol0[16]	= { 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 70, 70, 70, 70, 70, 70 } ;
	static const uint8_t	Twol1[16]	= { 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 100, 100, 100, 100, 100 } ;
	static const uint16_t	Trec0[16]	= { 275, 275, 275, 275, 275, 275, 525, 775, 1025, 1275, 1525, 1775, 2025, 2275, 2525, 2525 } ;
	static const uint16_t	Rwpu[16]	= { 500, 500, 500, 500, 500, 500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000 } ;
#endif

// ###################################### Public variables #########################################

uint8_t		ds248xCount	= 0 ;
ds248x_t *	psaDS248X = NULL ;

// ############################### Forward function declarations ###################################

void	ds248xCheckStatus(ds248x_t * psDS248X) ;
int32_t ds248xReset(ds248x_t * psDS248X) ;

// ################################ Local ONLY utility functions ###################################

int32_t	ds248xI2C_Read(ds248x_t * psDS248X) {
	IF_myASSERT(debugBUS_CFG, psDS248X->OWB == 0) ;
	int32_t iRV = halI2C_Read(psDS248X->psI2C, &psDS248X->RegX[psDS248X->Rptr], 1) ;
	// During the device discovery/config phases errors are valid and indicate pre/absence
	// of a specific device, type or submodel (DS2482-800 vs -10x). Hence selective error check
	IF_myASSERT(debugRESULT && psDS248X->Test == 0, iRV == erSUCCESS) ;
	if (iRV == erSUCCESS) {
		if (psDS248X->Rptr == ds248xREG_STAT) {
			ds248xCheckStatus(psDS248X) ;
		}
		return true ;
	}
	return false ;
}

int32_t	ds248xI2C_WriteDelayRead(ds248x_t * psDS248X, uint8_t * pTxBuf, size_t TxSize, uint32_t Delay) {
	IF_myASSERT(debugBUS_CFG, psDS248X->OWB == 0) ;
	int32_t iRV ;
	if (Delay > 0) {
		iRV = halI2C_Write(psDS248X->psI2C, pTxBuf, TxSize) ;
		// During the device discovery/config phases errors are valid and indicate pre/absence
		// of a specific device, type or submodel (DS2482-800 vs -10x). Hence selective error check
		IF_myASSERT(debugRESULT && psDS248X->Test == 0, iRV == erSUCCESS) ;
		if (iRV == erSUCCESS) {
			i64TaskDelayUsec(Delay) ;
			iRV = halI2C_Read(psDS248X->psI2C, &psDS248X->RegX[psDS248X->Rptr], 1) ;
		}
	} else {
		iRV = halI2C_WriteRead(psDS248X->psI2C, pTxBuf, TxSize, &psDS248X->RegX[psDS248X->Rptr], 1) ;
	}
	// During the device discovery/config phases errors are valid and indicate pre/absence
	// of a specific device, type or submodel (DS2482-800 vs -10x). Hence selective error check
	IF_myASSERT(debugRESULT && psDS248X->Test == 0, iRV == erSUCCESS) ;
	if (iRV == erSUCCESS) {
		if (psDS248X->Rptr == ds248xREG_STAT) {
			ds248xCheckStatus(psDS248X) ;
			if (psDS248X->OWB) {
				IF_myASSERT(debugRESULT, 0) ;
				ds248xReset(psDS248X) ;
				return false ;
			}
		}
		return true ;
	}
	return false ;
}

void	ds248xCheckStatus(ds248x_t * psDS248X) {
	const uint8_t DS248Xmask[4] = { 0b00000100, 0b00010110, 0b00011111, 0b11111111 } ;
	uint8_t Mask = DS248Xmask[OWflags.Level] ;
	if ((psDS248X->Rstat & Mask) != (psDS248X->StatX & Mask)) {
		char cBuffer[8 * 12] ;
		xBitMapDecodeChanges(psDS248X->StatX, psDS248X->Rstat, 0x000000FF, StatNames, cBuffer, sizeof(cBuffer)) ;
		PRINT("D=%d (0x%02X) : %s\n", psDS248X->psI2C->DevIdx, psDS248X->Rstat, cBuffer) ;
	}
	psDS248X->StatX = psDS248X->Rstat ;
}

int32_t	ds248xPrintConfig(ds248x_t * psDS248X, uint8_t Reg) {
	int32_t iRV= halI2C_DeviceReport((void *) ((uint32_t) psDS248X->I2Cnum)) ;
	iRV += PRINT("1-W:  NumCh=%d  Cur#=%d  Rptr=%d (%s)  Reg=0x%02X\n",
		psDS248X->NumChan, psDS248X->CurChan, psDS248X->Rptr, RegNames[psDS248X->Rptr], Reg) ;
	return iRV ;
}

/**
 * ds248xReset() - does device HW reset
 * @brief	if reset successful reads & (indirectly) stores status, also reset CONF & CurChan values
 * @return	status of RST bit ie true or false
 */
int32_t ds248xReset(ds248x_t * psDS248X) {
	// Device Reset
	//	S AD,0 [A] DRST [A] Sr AD,1 [A] [SS] A\ P
	//  [] indicates from slave
	//  SS status byte to read to verify state
	uint8_t	cChr 	= ds248xCMD_DRST ;
	psDS248X->Rptr	= ds248xREG_STAT ;				// After ReSeT pointer set to STATus register
	IF_SYSTIMER_START(debugTIMING, systimerDS248xA) ;
	ds248xI2C_WriteDelayRead(psDS248X, &cChr, sizeof(cChr), 0) ;
	IF_SYSTIMER_STOP(debugTIMING, systimerDS248xA) ;
	psDS248X->Rdata		= 0 ;
	psDS248X->Rconf		= 0 ;							// all bits cleared (default) config
	psDS248X->CurChan	= 0 ;
#if		(halHAS_DS2482_800 > 0)
	psDS248X->Rchan		= ds248x_V2N[0] ;				// ONLY for DS2482-800
#endif
#if		(halHAS_DS2484 > 0)
	psDS248X->Rpadj		= 0 ;							// ONLY for DS2484
#endif
	return psDS248X->RST ;
}

/**
 * Write the configuration register in the DS248x. The configuration
 * options are provided in the lower nibble of the provided config byte.
 * The upper nibble in bitwise inverted when written to the DS248x.
 *
 * Returns:  true: config written and response correct
 *			  false: response incorrect
 */
int32_t ds248xWriteConfig(ds248x_t * psDS248X) {
// Write configuration (Case A)
//	S AD,0 [A] WCFG [A] CF [A] Sr AD,1 [A] [CF] A\ P
//  [] indicates from slave
//  CF configuration byte to write
	uint8_t	config	= psDS248X->Rconf & 0x0F ;
	uint8_t	cBuf[2] = { ds248xCMD_WCFG , (~config << 4) | config } ;
	psDS248X->Rptr = ds248xREG_CONF ;
	IF_SYSTIMER_START(debugTIMING, systimerDS248xA) ;
	ds248xI2C_WriteDelayRead(psDS248X, cBuf, sizeof(cBuf), 0) ;
	IF_SYSTIMER_STOP(debugTIMING, systimerDS248xA) ;
	if (psDS248X->Rconf != config) {
		IF_myASSERT(debugRESULT, 0) ;
		ds248xReset(psDS248X) ;
		return false ;
	}
	return true ;
}

/**
 * ds248xReadRegister() -  set the Read Pointer and reads the register
 * @brief	Once set the pointer remains static to allow reread of same register
 * @return
 */
int32_t	ds248xReadRegister(ds248x_t * psDS248X, uint8_t Reg) {
	// check for validity of CHAN (only DS2482-800) and PADJ (only DS2484)
	if ((Reg == ds248xREG_CHAN && psDS248X->psI2C->Type != i2cDEV_DS2482_800) ||
		(Reg == ds248xREG_PADJ && psDS248X->psI2C->Type != i2cDEV_DS2484)) {
		ds248xPrintConfig(psDS248X, Reg) ;
		PRINT("Invalid register combination!!!\n") ;
		return false ;
	}
	psDS248X->Rptr	= Reg ;
	uint8_t	cBuf[2] = { ds248xCMD_SRP, (~Reg << 4) | Reg } ;
	IF_SYSTIMER_START(debugTIMING, systimerDS248xA) ;
	ds248xI2C_WriteDelayRead(psDS248X, cBuf, sizeof(cBuf), 0) ;
	IF_SYSTIMER_STOP(debugTIMING, systimerDS248xA) ;
	return true ;
}

// ############################### DS248x(-x00) DEBUG support functions ############################

/**
 * Display register contents, decode status & configuration
 */
int32_t	ds248xReportRegister(ds248x_t * psDS248X, uint32_t Reg) {
	int32_t iRV = 0 ;
	switch (Reg) {
	case ds248xREG_STAT:
		if (ds248xReadRegister(psDS248X, Reg) == false)
			return false ;
		iRV += printfx_nolock("STAT(0)=0x%02X  DIR=%c  TSB=%c  SBR=%c  RST=%c  LL=%c  SD=%c  PPD=%c  1WB=%c\n",
				psDS248X->Rstat,
				psDS248X->DIR ? '1' : '0',
				psDS248X->TSB ? '1' : '0',
				psDS248X->SBR ? '1' : '0',
				psDS248X->RST ? '1' : '0',
				psDS248X->LL  ? '1' : '0',
				psDS248X->SD  ? '1' : '0',
				psDS248X->PPD ? '1' : '0',
				psDS248X->OWB ? '1' : '0') ;
		break ;
	case ds248xREG_DATA:
		iRV += printfx_nolock("DATA(1)=0x%02X (Last read)\n", psDS248X->Rdata) ;
		break ;
	case ds248xREG_CHAN:
#if		(halHAS_DS2482_800 > 0)
		if (psDS248X->psI2C->Type != i2cDEV_DS2482_800)
			return false ;
		if (ds248xReadRegister(psDS248X, Reg) == false)
			return false ;
		{	int32_t Chan ;										// Channel, start by finding the matching Channel #
			for (Chan = 0; (Chan < psDS248X->NumChan) && (psDS248X->Rchan != ds248x_V2N[Chan]); ++Chan) ;
			IF_myASSERT(debugRESULT, Chan < psDS248X->NumChan && psDS248X->Rchan == ds248x_V2N[Chan]) ;
			iRV = printfx_nolock("CHAN(2)=0x%02X  Rchan=0x%02X  Chan=%d  Xlat=0x%02X\n", psDS248X->CHAN, psDS248X->Rchan, Chan, ds248x_V2N[Chan]) ;
		}
#endif
		break ;
	case ds248xREG_CONF:
		if (ds248xReadRegister(psDS248X, Reg) == false)
			return false ;
		iRV += printfx_nolock("CONF(3)=0x%02X  1WS=%c  SPU=%c  PDN=%c  APU=%c\n",
				psDS248X->Rconf,
				psDS248X->OWS	? '1' : '0',
				psDS248X->SPU	? '1' : '0',
				psDS248X->PDN	? '1' : '0',
				psDS248X->APU	? '1' : '0') ;
		break ;
	case ds248xREG_PADJ:
#if		(halHAS_DS2484 > 0)
		if (psDS248X->psI2C->Type != i2cDEV_DS2484)
			return false ;
		// PAR = 0b000 ~ tRSTL
		if (ds248xReadRegister(psDS248X, Reg) == false)
			return false ;
		iRV += printfx_nolock("PADJ(4a)=0x%02X  PAR=0  OD=%c  tRSTL=%d uS\n",
				psDS248X->Rpadj, psDS248X->OD ? '1' : '0',
				Trstl[psDS248X->VAL] * (psDS248X->OD ? 1 : 10)) ;
		// PAR = 0b001 ~ tMSP
		ds248xI2C_Read(psDS248X) ;
		iRV += printfx_nolock("PADJ(4b)=0x%02X  PAR=1  OD=%c  tMSP=",
				psDS248X->Rpadj, psDS248X->OD	? '1' : '0') ;
		iRV += printfx_nolock(psDS248X->OD ? "%d uS\n" : "%.1f uS\n",
				psDS248X->OD ? (float) Tmsp1[psDS248X->VAL] / 10.0 : Tmsp0[psDS248X->VAL]) ;
		// PAR = 0b010 ~ tWOL
		ds248xI2C_Read(psDS248X) ;
		iRV += printfx_nolock("PADJ(4c)=0x%02X  PAR=2  OD=%c  tWOL=",
				psDS248X->Rpadj, psDS248X->OD	? '1' : '0') ;
		iRV += printfx_nolock(psDS248X->OD ? "%d uS\n" : "%.1f uS\n",
				psDS248X->OD ? (float) Twol1[psDS248X->VAL] / 10.0 : Twol0[psDS248X->VAL]) ;
		// PAR = 0b011 ~ tREC0
		ds248xI2C_Read(psDS248X) ;
		iRV += printfx_nolock("PADJ(4d)=0x%02X  PAR=3  OD=%c  tREC0=%.2f uS\n",
				psDS248X->Rpadj, psDS248X->OD	? '1' : '0', (float) Trec0[psDS248X->VAL] / 100.0) ;
		// PAR = 0b100 ~ rWPU
		ds248xI2C_Read(psDS248X) ;
		iRV += printfx_nolock("PADJ(4e)=0x%02X  PAR=4  OD=%c  rWPU=%f ohm\n",
				psDS248X->Rpadj, psDS248X->OD	? '1' : '0', (float) Rwpu[psDS248X->VAL]) ;
#endif
		break ;
	}
	return iRV ;
}

/**
 * ds248xReport() - report decoded status of a specific device
 */
int32_t	ds248xReport(ds248x_t * psDS248X) {
	int32_t iRV = halI2C_DeviceReport((void *) psDS248X->psI2C) ;
	for (uint32_t Reg = 0; Reg < ds248xREG_NUM; ++Reg) {
		iRV += ds248xReportRegister(psDS248X, Reg) ;
	}
	iRV += printfx_nolock("\n") ;
	return iRV ;
}

/**
 * ds248xReportAll() - report decoded status of all devices and all registers
 */
int32_t ds248xReportAll(void) {
	int32_t iRV = 0 ;
	printfx_lock() ;
	for (int32_t i = 0; i < ds248xCount; ++i)
		iRV += ds248xReport(&psaDS248X[i]) ;
	printfx_unlock() ;
	return iRV ;
}

// ################### Identification, Diagnostics & Configuration functions #######################

/**
 * ds248xDeviceIdentify() - device reset+register reads to ascertain exact device type
 * Returns: erSUCCESS if identified  device was detected and written
 *			 false device not detected
 */
int32_t	ds248xDeviceIdentify(i2c_dev_info_t * psI2C_DI) {
#if		(halHAS_DS248X > 0)
	ds248x_t	sDS248X = { 0 } ;						// temporary device structure
	psI2C_DI->Delay		= pdMS_TO_TICKS(10) ;			// default device timeout
	sDS248X.psI2C		= psI2C_DI ;					// link to I2C device discovered
	sDS248X.Test		= 1 ;							// disable I2C error messages in both
	psI2C_DI->Test		= 1 ;							// this and halI2C modules
	#if	(halHAS_DS2484 > 0)
	sDS248X.psI2C->Type = i2cDEV_DS2484 ;
	if (ds248xReset(&sDS248X) &&						// generic DS248X, check DS2484
		ds248xReadRegister(&sDS248X, ds248xREG_PADJ) &&	// PADJ read OK
		sDS248X.VAL == 0b00000110) {					// VAL is default for PAR=000 & OD=0
		psI2C_DI->DevIdx 	= ds248xCount++ ;
		return erSUCCESS ;
	}
	#endif
	#if	(halHAS_DS2482_800 > 0)
	sDS248X.psI2C->Type = i2cDEV_DS2482_800 ;
	if (ds248xReset(&sDS248X) &&						// generic DS248X, check DS2482-800
		ds248xReadRegister(&sDS248X, ds248xREG_CHAN) &&	// CSR read OK
		sDS248X.CHAN == ds248x_V2N[0]) {				// CHAN=0 default
		psI2C_DI->DevIdx 	= ds248xCount++ ;
		return erSUCCESS ;
	}
	#endif
	#if	(halHAS_DS2482_10X > 0)							// NOT YET TESTED !!!!
	sDS248X.psI2C->Type = i2cDEV_DS2482_10X ;
	if (ds248xReset(&sDS248X) &&						// generic DS248X, check DS2482-800
		!ds248xReadRegister(&sDS248X, ds248xREG_CHAN)) {// CSR read FAIL !!!
		psI2C_DI->DevIdx 	= ds248xCount++ ;
		return erSUCCESS ;
	}
	#endif
#endif
	return erFAILURE ;
}

int32_t	ds248xDriverConfig(i2c_dev_info_t * psI2C_DI) {
	if (psaDS248X == NULL) {
		IF_SYSTIMER_INIT(debugTIMING, systimerDS248xA, systimerCLOCKS, "DS248xA", myUS_TO_CLOCKS(100), myUS_TO_CLOCKS(1000)) ;
		IF_SYSTIMER_INIT(debugTIMING, systimerDS248xB, systimerCLOCKS, "DS248xB", myUS_TO_CLOCKS(200), myUS_TO_CLOCKS(2000)) ;
		IF_SYSTIMER_INIT(debugTIMING, systimerDS248xC, systimerCLOCKS, "DS248xC", myUS_TO_CLOCKS(10), myUS_TO_CLOCKS(100)) ;
		IF_SYSTIMER_INIT(debugTIMING, systimerDS248xD, systimerCLOCKS, "DS248xD", myUS_TO_CLOCKS(300), myUS_TO_CLOCKS(3000)) ;
		IF_SYSTIMER_INIT(debugTIMING, systimerDS248xE, systimerCLOCKS, "DS248xE", myUS_TO_CLOCKS(300), myUS_TO_CLOCKS(3000)) ;
		IF_SYSTIMER_INIT(debugTIMING, systimerDS248xF, systimerCLOCKS, "DS248xF", myUS_TO_CLOCKS(300), myUS_TO_CLOCKS(3000)) ;
		psaDS248X = malloc(ds248xCount * sizeof(ds248x_t)) ;
		memset(psaDS248X, 0, ds248xCount * sizeof(ds248x_t)) ;
		IF_myASSERT(debugPARAM, psI2C_DI->DevIdx == 0) ;
	}
	psI2C_DI->Delay		= pdMS_TO_TICKS(10) ;			// default device timeout
	ds248x_t * psDS248X = &psaDS248X[psI2C_DI->DevIdx] ;
	psDS248X->psI2C		= psI2C_DI ;
	switch(psI2C_DI->Type) {
#if		(halHAS_DS2482_10X > 0)
		case i2cDEV_DS2482_10X:		psDS248X->NumChan = 1 ;			break ;
#endif
#if		(halHAS_DS2482_800 > 0)
		case i2cDEV_DS2482_800:		psDS248X->NumChan = 8 ;			break ;
#endif
#if		(halHAS_DS2484 > 0)
		case i2cDEV_DS2484:			psDS248X->NumChan = 1 ;			break ;
#endif
		default: myASSERT(0) ;
	}
	ds248xReset(psDS248X) ;
	IF_myASSERT(debugRESULT, psDS248X->RST == 1) ;

	psDS248X->Rconf	= 0 ;
	psDS248X->APU	= 1 ;								// LSBit
	ds248xWriteConfig(psDS248X) ;
	IF_myASSERT(debugRESULT, psDS248X->APU == 1) ;
	return erSUCCESS ;
}

// ############################## DS248x-x00 1-Wire support functions ##############################

/**
 * ds248xOWChannelSelect() - Select the 1-Wire channel on a DS2482-800.
 * Returns: erSUCCESS if channel selected
 *			erFAILURE if device not detected or failure to perform select; else
 *			whatever status returned by halI2C_WriteRead()
 */
int32_t ds248xOWChannelSelect(ds248x_t * psDS248X, uint8_t Chan) {
#if		(halHAS_DS2482_800 > 0)
	/* Channel Select (Case A)
	 *	S AD,0 [A] CHSL [A] CC [A] Sr AD,1 [A] [RR] A\ P
	 *  [] indicates from slave
	 *  CC channel value
	 *  RR channel read back
	 */
	if (psDS248X->psI2C->Type == i2cDEV_DS2482_800)	{
		uint8_t	cBuf[2] = { ds2482CMD_CHSL, ~Chan<<4 | Chan } ;	// calculate Channel value
		psDS248X->Rptr	= ds248xREG_CHAN ;
		IF_SYSTIMER_START(debugTIMING, systimerDS248xA) ;
		ds248xI2C_WriteDelayRead(psDS248X, cBuf, sizeof(cBuf), 0) ;
		IF_SYSTIMER_STOP(debugTIMING, systimerDS248xA) ;
		// value read back not same as the channel number sent so verify the return
		// against the code expected, but store the actual channel number if successful
		if (psDS248X->Rchan != ds248x_V2N[Chan]) {
			SL_ERR("Read %d != %d Expected", psDS248X->RegX[psDS248X->Rptr], ds248x_V2N[Chan]) ;
			IF_myASSERT(debugRESULT, 0) ;
			return false ;
		}
		psDS248X->CurChan	= Chan ;
		return true ;
	}
#endif
	return true ;
}

int32_t	ds248xOWSetSPU(ds248x_t * psDS248X) {
	psDS248X->SPU = 1 ;
	ds248xWriteConfig(psDS248X) ;
	IF_myASSERT(debugRESULT, psDS248X->SPU == 1) ;
	return psDS248X->SPU ;
}

int32_t ds248xOWReset(ds248x_t * psDS248X) {
// 1-Wire reset (Case B)
//	S AD,0 [A] 1WRS [A] Sr AD,1 [A] [Status] A [Status] A\ P
//									\--------/
//						Repeat until 1WB bit has changed to 0
//  [] indicates from slave
	IF_myASSERT(debugBUS_CFG, psDS248X->SPU == 0) ;
	uint8_t	cChr = ds248xCMD_1WRS ;
	psDS248X->Rptr	= ds248xREG_STAT ;
	IF_SYSTIMER_START(debugTIMING, systimerDS248xB) ;
	ds248xI2C_WriteDelayRead(psDS248X, &cChr, sizeof(cChr), psDS248X->OWS ? owDELAY_RST_OD : owDELAY_RST) ;
	IF_SYSTIMER_STOP(debugTIMING, systimerDS248xB) ;
	return psDS248X->PPD ;
}

int32_t	ds248xOWSpeed(ds248x_t * psDS248X, bool speed) {
	psDS248X->OWS = speed ;
	ds248xWriteConfig(psDS248X) ;
	IF_myASSERT(debugRESULT, psDS248X->OWS == speed) ;
	return psDS248X->OWS ;
}

int32_t ds248xOWLevel(ds248x_t * psDS248X, bool level) {
	if (level)											// DS248X only allow disabling STRONG
		return psDS248X->SPU ;
	psDS248X->SPU = level ;
	ds248xWriteConfig(psDS248X) ;
	IF_myASSERT(debugRESULT, psDS248X->SPU == level) ;
	return psDS248X->SPU ;
}

uint8_t ds248xOWTouchBit(ds248x_t * psDS248X, uint8_t sendbit) {
// 1-Wire bit (Case B)
//	S AD,0 [A] 1WSB [A] BB [A] Sr AD,1 [A] [Status] A [Status] A\ P
//										   \--------/
//								Repeat until 1WB bit has changed to 0
//  [] indicates from slave
//  BB indicates byte containing bit value in msbit
	IF_myASSERT(debugPARAM, sendbit < 2) ;
	uint8_t	cBuf[2] = {	ds248xCMD_1WSB, sendbit ? 0x80 : 0x00 } ;
	psDS248X->Rptr	= ds248xREG_STAT ;
	IF_SYSTIMER_START(debugTIMING, systimerDS248xC) ;
	ds248xI2C_WriteDelayRead(psDS248X, cBuf, sizeof(cBuf), psDS248X->OWS ? owDELAY_SB_OD : owDELAY_SB) ;
	IF_SYSTIMER_STOP(debugTIMING, systimerDS248xC) ;
	return psDS248X->SBR ;
}

void	ds248xOWWriteByte(ds248x_t * psDS248X, uint8_t sendbyte) {
// 1-Wire Write Byte (Case B)
//	S AD,0 [A] 1WWB [A] DD [A] Sr AD,1 [A] [Status] A [Status] A\ P
//										   \--------/
//							Repeat until 1WB bit has changed to 0
//  [] indicates from slave
//  DD data to write
	uint8_t	cBuf[2] = { ds248xCMD_1WWB, sendbyte } ;
	psDS248X->Rptr	= ds248xREG_STAT ;
	IF_SYSTIMER_START(debugTIMING, systimerDS248xD) ;
	ds248xI2C_WriteDelayRead(psDS248X, cBuf, sizeof(cBuf), psDS248X->OWS ? owDELAY_WB_OD : owDELAY_WB) ;
	IF_SYSTIMER_STOP(debugTIMING, systimerDS248xD) ;
}

int32_t ds248xOWWriteBytePower(ds248x_t * psDS248X, uint8_t sendbyte) {
	if (ds248xOWSetSPU(psDS248X) == false)
		return false ;
	ds248xOWWriteByte(psDS248X, sendbyte) ;
	return psDS248X->SPU ;
}

int32_t	ds248xOWReadByte(ds248x_t * psDS248X) {
/* 1-Wire Read Bytes (Case C)
 *	S AD,0 [A] 1WRB [A] Sr AD,1 [A] [Status] A [Status] A\
 *										\--------/
 *							Repeat until 1WB bit has changed to 0
 *	Sr AD,0 [A] SRP [A] E1 [A] Sr AD,1 [A] DD A\ P
 *  [] indicates from slave
 *  DD data read	*/
	uint8_t	cBuf	= ds248xCMD_1WRB ;
	psDS248X->Rptr	= ds248xREG_STAT ;
	IF_SYSTIMER_START(debugTIMING, systimerDS248xE) ;
	ds248xI2C_WriteDelayRead(psDS248X, &cBuf, sizeof(cBuf), psDS248X->OWS ? owDELAY_RB_OD : owDELAY_RB) ;
	IF_SYSTIMER_STOP(debugTIMING, systimerDS248xE) ;
	ds248xReadRegister(psDS248X, ds248xREG_DATA) ;
	return psDS248X->Rdata ;
}

uint8_t ds248xOWSearchTriplet(ds248x_t * psDS248X, uint8_t search_direction) {
// 1-Wire Triplet (Case B)
//	S AD,0 [A] 1WT [A] SS [A] Sr AD,1 [A] [Status] A [Status] A\ P
//							  \--------/
//				Repeat until 1WB bit has changed to 0
//  [] indicates from slave
//  SS indicates byte containing search direction bit value in msbit
	IF_myASSERT(debugPARAM, search_direction < 2) ;
	uint8_t	cBuf[2] = { ds248xCMD_1WT, search_direction ? 0x80 : 0x00 } ;
	psDS248X->Rptr	= ds248xREG_STAT ;
	IF_SYSTIMER_START(debugTIMING, systimerDS248xF) ;
	ds248xI2C_WriteDelayRead(psDS248X, cBuf, sizeof(cBuf), psDS248X->OWS ? owDELAY_ST_OD : owDELAY_ST) ;
	IF_SYSTIMER_STOP(debugTIMING, systimerDS248xF) ;
	return psDS248X->Rstat ;
}
