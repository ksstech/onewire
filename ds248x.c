/*
 * Copyright 2020-21 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#include	"hal_variables.h"
#include	"onewire_platform.h"
#include	"FreeRTOS_Support.h"
#include	"printfx.h"
#include	"syslog.h"
#include	"systiming.h"								// timing debugging
#include	"x_errors_events.h"
#include	"x_string_general.h"

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

// DS2482-800 only CHAN register xlat	0	  1		2	  3		4	  5		6	  7
static const uint8_t ds248x_V2N[8] = { 0xB8, 0xB1, 0xAA, 0xA3, 0x9C, 0x95, 0x8E, 0x87 } ;
// DS2484 only reporting/debugging
static const uint8_t Trstl[16]	= { 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74 } ;
static const uint8_t Tmsp0[16]	= { 58, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 76, 76, 76, 76, 76 } ;
static const uint8_t Tmsp1[16]	= { 55, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 110, 110, 110 } ;
static const uint8_t Twol0[16]	= { 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 70, 70, 70, 70, 70, 70 } ;
static const uint8_t Twol1[16]	= { 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 100, 100, 100, 100, 100 } ;
static const uint16_t Trec0[16]	= { 275, 275, 275, 275, 275, 275, 525, 775, 1025, 1275, 1525, 1775, 2025, 2275, 2525, 2525 } ;
static const uint16_t Rwpu[16]	= { 500, 500, 500, 500, 500, 500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000 } ;

// ############################### Forward function declarations ###################################

int32_t ds248xReset(ds248x_t * psDS248X) ;

// ################################ Local ONLY utility functions ###################################

void	ds248xCheckStatus(ds248x_t * psDS248X) {
	const uint8_t DS248Xmask[4] = { 0b00000111, 0b00011111, 0b00111111, 0b11111111 } ;
	uint8_t Mask = DS248Xmask[OWflags.Level] ;
	uint8_t StatX = psDS248X->PrvStat[psDS248X->CurChan] ;
	if ((psDS248X->Rstat & Mask) != (StatX & Mask)) {
		char * pcBuf = pcBitMapDecodeChanges(StatX, psDS248X->Rstat, 0x000000FF, StatNames) ;
		printfx("I2C=%d  OW=%u  Stat=0x%02X->0x%02X : %s\n", psDS248X->psI2C->DevIdx,
				psDS248X->CurChan, StatX, psDS248X->Rstat, pcBuf) ;
		free(pcBuf) ;
	}
	psDS248X->PrvStat[psDS248X->CurChan] = psDS248X->Rstat ;
}

int32_t	ds248xI2C_Read(ds248x_t * psDS248X) {
	xRtosSemaphoreTake(&psDS248X->psI2C->mux, portMAX_DELAY) ;
	IF_myASSERT(debugBUS_CFG, psDS248X->OWB == 0) ;
	int32_t iRV = halI2C_Read(psDS248X->psI2C, &psDS248X->RegX[psDS248X->Rptr], 1) ;
	xRtosSemaphoreGive(&psDS248X->psI2C->mux) ;
	if (iRV == erSUCCESS) {
		if (psDS248X->Rptr == ds248xREG_STAT) {
			ds248xCheckStatus(psDS248X) ;
		}
		return 1 ;
	} else {
		// During the device discovery/config phases errors are valid and indicate pre/absence
		// of a specific device, type or submodel (DS2482-800 vs -10x). Hence selective error check
		IF_myASSERT(debugRESULT && psDS248X->Test == 0, 0) ;
	}
	return 0 ;
}

int32_t	ds248xI2C_WriteDelayRead(ds248x_t * psDS248X, uint8_t * pTxBuf, size_t TxSize, uint32_t Delay) {
	xRtosSemaphoreTake(&psDS248X->psI2C->mux, portMAX_DELAY) ;
	IF_myASSERT(debugBUS_CFG, psDS248X->OWB == 0) ;
	int32_t	iRV = halI2C_Write(psDS248X->psI2C, pTxBuf, TxSize) ;
	if (iRV == erSUCCESS) {
		if (Delay) {
			i64TaskDelayUsec(Delay) ;
		}
		iRV = halI2C_Read(psDS248X->psI2C, &psDS248X->RegX[psDS248X->Rptr], 1) ;
	}
	if (iRV == erSUCCESS) {
		iRV = 1 ;
		if (psDS248X->Rptr == ds248xREG_STAT) {
			ds248xCheckStatus(psDS248X) ;
			if (psDS248X->OWB) {						// 1W still busy?
				ds248xReport(psDS248X, 0) ;				// avoid, tries to read registers = lockup
				ds248xReset(psDS248X) ;
				iRV = 0 ;
			}
		} else {
			// normally nothing to do, maybe some tests..
			if (psDS248X->Rptr == ds248xREG_CONF) {
				myASSERT(psDS248X->APU == 1) ;
			}
		}
	} else {
		// During device discovery/config errors are valid, indicate presence or absence of specific
		// device, type or submodel (DS2482-800 vs -10x). Hence selective error check
		IF_myASSERT(debugRESULT && psDS248X->Test == 0, 0) ;
	}
	xRtosSemaphoreGive(&psDS248X->psI2C->mux) ;
	return iRV ;
}

void	ds248xPrintConfig(ds248x_t * psDS248X, uint8_t Reg) {
	halI2C_DeviceReport((void *) ((uint32_t) psDS248X->I2Cnum)) ;
	printfx("1-W:  NumCh=%d  Cur#=%d  Rptr=%d (%s)  Reg=0x%02X\n",
		psDS248X->NumChan, psDS248X->CurChan, psDS248X->Rptr, RegNames[psDS248X->Rptr], Reg) ;
}

/**
 * ds248xReset() - does device HW reset
 * @brief	if reset successful reads & (indirectly) stores status, also reset CONF & CurChan values
 * @return	status of RST bit ie 1 or 0
 */
int32_t ds248xReset(ds248x_t * psDS248X) {
	// Device Reset
	//	S AD,0 [A] DRST [A] Sr AD,1 [A] [SS] A\ P
	//  [] indicates from slave
	//  SS status byte to read to verify state
	uint8_t	cChr 	= ds248xCMD_DRST ;
	psDS248X->Rptr	= ds248xREG_STAT ;					// After ReSeT pointer set to STATus register
	IF_SYSTIMER_START(debugTIMING, systimerDS248xA) ;
	ds248xI2C_WriteDelayRead(psDS248X, &cChr, sizeof(cChr), 0) ;
	IF_SYSTIMER_STOP(debugTIMING, systimerDS248xA) ;
	psDS248X->Rdata		= 0 ;
	psDS248X->Rconf		= 0 ;							// all bits cleared (default) config
	psDS248X->CurChan	= 0 ;
	psDS248X->Rchan		= ds248x_V2N[0] ;				// DS2482-800 specific
	psDS248X->Rpadj		= 0 ;							// DS2484 specific
	return psDS248X->RST ;
}

/**
 * Write the configuration register in the DS248x. The configuration
 * options are provided in the lower nibble of the provided config byte.
 * The upper nibble in bitwise inverted when written to the DS248x.
 *
 * Returns:  1: config written and response correct
 *			  0: response incorrect
 */
int		ds248xWriteConfig(ds248x_t * psDS248X) {
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
		return 0 ;
	}
	return 1 ;
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
		printfx("Invalid register combination!!!\n") ;
		return 0 ;
	}
	psDS248X->Rptr	= Reg ;
	uint8_t	cBuf[2] = { ds248xCMD_SRP, (~Reg << 4) | Reg } ;
	IF_SYSTIMER_START(debugTIMING, systimerDS248xA) ;
	ds248xI2C_WriteDelayRead(psDS248X, cBuf, sizeof(cBuf), 0) ;
	IF_SYSTIMER_STOP(debugTIMING, systimerDS248xA) ;
	return 1 ;
}

// ############################### DS248x(-x00) DEBUG support functions ############################

int32_t	ds248xReportStatus(uint8_t Num, ds248x_stat_t Stat) {
	return printfx("STAT(4) #%u=0x%02X  DIR=%c  TSB=%c  SBR=%c  RST=%c  LL=%c  SD=%c  PPD=%c  1WB=%c\n",
			Num,
			Stat.STAT,
			Stat.DIR ? '1' : '0',
			Stat.TSB ? '1' : '0',
			Stat.SBR ? '1' : '0',
			Stat.RST ? '1' : '0',
			Stat.LL  ? '1' : '0',
			Stat.SD  ? '1' : '0',
			Stat.PPD ? '1' : '0',
			Stat.OWB ? '1' : '0') ;
}

/**
 * Display register contents, decode status & configuration
 */
int32_t	ds248xReportRegister(ds248x_t * psDS248X, int Reg, bool Refresh) {
	int32_t iRV = 0 ;
	int	Chan ;
	switch (Reg) {
	case ds248xREG_STAT:
		if (Refresh && ds248xReadRegister(psDS248X, Reg) == 0) {
			return 0 ;
		}
		for (int i = 0; i < psDS248X->NumChan; iRV += ds248xReportStatus(i, (ds248x_stat_t) psDS248X->PrvStat[i++]));
		break ;
	case ds248xREG_DATA:
		iRV += printfx("DATA(1)=0x%02X (Last read)\n", psDS248X->Rdata) ;
		break ;
	case ds248xREG_CHAN:
		if ((psDS248X->psI2C->Type != i2cDEV_DS2482_800) ||
			(Refresh && ds248xReadRegister(psDS248X, Reg) == 0)) {
			return 0 ;
		}
		// Channel, start by finding the matching Channel #
		for (Chan = 0; Chan < psDS248X->NumChan && psDS248X->Rchan != ds248x_V2N[Chan]; ++Chan) ;
		IF_myASSERT(debugRESULT, Chan < psDS248X->NumChan && psDS248X->Rchan == ds248x_V2N[Chan]) ;
		iRV = printfx("CHAN(2)=0x%02X  Rchan=0x%02X  Chan=%d  Xlat=0x%02X\n",
				psDS248X->CHAN, psDS248X->Rchan, Chan, ds248x_V2N[Chan]) ;
		break ;
	case ds248xREG_CONF:
		if (Refresh && ds248xReadRegister(psDS248X, Reg) == 0) {
			return 0 ;
		}
		iRV += printfx("CONF(3)=0x%02X  1WS=%c  SPU=%c  PDN=%c  APU=%c\n",
				psDS248X->Rconf,
				psDS248X->OWS	? '1' : '0',
				psDS248X->SPU	? '1' : '0',
				psDS248X->PDN	? '1' : '0',
				psDS248X->APU	? '1' : '0') ;
		break ;
	case ds248xREG_PADJ:
		if (Refresh == 0) {
			return 0 ;
		}
		if ((psDS248X->psI2C->Type != i2cDEV_DS2484) ||
			(ds248xReadRegister(psDS248X, Reg) == 0)) {
			return 0 ;
		}
		iRV += printfx("PADJ=0x%02X  OD=%c | tRSTL=%duS | tMSP=", psDS248X->Rpadj,
				psDS248X->OD ? '1' : '0', Trstl[psDS248X->VAL] * (psDS248X->OD ? 1 : 10)) ;
		ds248xI2C_Read(psDS248X) ;
		iRV += printfx(psDS248X->OD ? "%duS" : "%.1fuS",
				psDS248X->OD ? (float) Tmsp1[psDS248X->VAL] / 10.0 : Tmsp0[psDS248X->VAL]) ;
		ds248xI2C_Read(psDS248X) ;
		iRV += printfx(psDS248X->OD ? " | tWOL=%duS" : " | tWOL=%.1fuS",
				psDS248X->OD ? (float) Twol1[psDS248X->VAL] / 10.0 : Twol0[psDS248X->VAL]) ;
		ds248xI2C_Read(psDS248X) ;
		iRV += printfx(" | tREC0=%.2fuS", (float) Trec0[psDS248X->VAL] / 100.0) ;
		ds248xI2C_Read(psDS248X) ;
		iRV += printfx(" | rWPU=%f ohm\n", (float) Rwpu[psDS248X->VAL]) ;
		break ;
	}
	return iRV ;
}

/**
 * ds248xReport() - report decoded status of a specific device
 */
void	ds248xReport(ds248x_t * psDS248X, bool Refresh) {
	halI2C_DeviceReport((void *) psDS248X->psI2C) ;
	for (int Reg = 0; Reg < ds248xREG_NUM; ds248xReportRegister(psDS248X, Reg++, Refresh)) ;
	printfx("\n") ;
}

/**
 * ds248xReportAll() - report decoded status of all devices and all registers
 */
void	ds248xReportAll(bool Refresh) {
	for (int i = 0; i < ds248xCount; ds248xReport(&psaDS248X[i++], Refresh)) ;
}

// ################### Identification, Diagnostics & Configuration functions #######################

/**
 * ds248xDeviceIdentify() - device reset+register reads to ascertain exact device type
 * @return	erSUCCESS if supported device was detected, if not erFAILURE
 */
int32_t	ds248xDeviceIdentify(i2c_dev_info_t * psI2C_DI) {
	ds248x_t sDS248X = { 0 } ;							// temporary device structure
	psI2C_DI->Delay	= pdMS_TO_TICKS(10) ;				// default device timeout
	sDS248X.psI2C	= psI2C_DI ;						// link to I2C device discovered
	sDS248X.Test	= 1 ;								// disable I2C error messages in DS248X
	psI2C_DI->Test	= 1 ;								// and halI2C modules
	if (ds248xReset(&sDS248X) == 1) {
		psI2C_DI->Type = i2cDEV_DS2484 ;
		int32_t iRV = ds248xReadRegister(&sDS248X, ds248xREG_PADJ) ;
		if (iRV == 1 &&	sDS248X.VAL == 0b00000110) {	// PADJ=OK & PAR=000 & OD=0
			psI2C_DI->DevIdx 	= ds248xCount++ ;		// valid DS2484
		} else {
			psI2C_DI->Type = i2cDEV_DS2482_800 ;		// assume -800 there
			iRV = ds248xReadRegister(&sDS248X, ds248xREG_CHAN) ;
			if (iRV == 0) {								// CSR read FAIL
				psI2C_DI->Type = i2cDEV_DS2482_10X ;	// NOT YET TESTED !!!!
				psI2C_DI->DevIdx 	= ds248xCount++ ;	// valid 2482-10x
			} else if (sDS248X.CHAN == ds248x_V2N[0]) {	// CHAN=0 default
				psI2C_DI->DevIdx 	= ds248xCount++ ;	// valid 2482-800
			} else {
				psI2C_DI->Type 		= i2cDEV_UNDEF ;	// not successful, undefined
			}
		}
	}
	psI2C_DI->Test = 0 ;
	return psI2C_DI->Type == i2cDEV_UNDEF ? erFAILURE : erSUCCESS ;
}

int32_t	ds248xDriverConfig(i2c_dev_info_t * psI2C_DI) {
	if (psaDS248X == NULL) {							// 1st time here...
		IF_myASSERT(debugPARAM, psI2C_DI->DevIdx == 0) ;
		psaDS248X = malloc(ds248xCount * sizeof(ds248x_t)) ;
		memset(psaDS248X, 0, ds248xCount * sizeof(ds248x_t)) ;
		IF_SYSTIMER_INIT(debugTIMING, systimerDS248xA, systimerCLOCKS, "DS248xA", myUS_TO_CLOCKS(100), myUS_TO_CLOCKS(1000)) ;
		IF_SYSTIMER_INIT(debugTIMING, systimerDS248xB, systimerCLOCKS, "DS248xB", myUS_TO_CLOCKS(200), myUS_TO_CLOCKS(2000)) ;
		IF_SYSTIMER_INIT(debugTIMING, systimerDS248xC, systimerCLOCKS, "DS248xC", myUS_TO_CLOCKS(10), myUS_TO_CLOCKS(100)) ;
		IF_SYSTIMER_INIT(debugTIMING, systimerDS248xD, systimerCLOCKS, "DS248xD", myUS_TO_CLOCKS(300), myUS_TO_CLOCKS(3000)) ;
		IF_SYSTIMER_INIT(debugTIMING, systimerDS248xE, systimerCLOCKS, "DS248xE", myUS_TO_CLOCKS(300), myUS_TO_CLOCKS(3000)) ;
		IF_SYSTIMER_INIT(debugTIMING, systimerDS248xF, systimerCLOCKS, "DS248xF", myUS_TO_CLOCKS(300), myUS_TO_CLOCKS(3000)) ;
	}
	psI2C_DI->Delay		= pdMS_TO_TICKS(10) ;			// default device timeout
	ds248x_t * psDS248X = &psaDS248X[psI2C_DI->DevIdx] ;
	psDS248X->psI2C		= psI2C_DI ;
	switch(psI2C_DI->Type) {
		case i2cDEV_DS2482_800:
			psDS248X->NumChan = 8 ;
			break ;
		case i2cDEV_DS2482_10X:
		case i2cDEV_DS2484:
			psDS248X->NumChan = 1 ;
			break ;
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
int		ds248xOWChannelSelect(ds248x_t * psDS248X, uint8_t Chan) {
	if (psDS248X->psI2C->Type == i2cDEV_DS2482_800)	{
		/* Channel Select (Case A)
		 *	S AD,0 [A] CHSL [A] CC [A] Sr AD,1 [A] [RR] A\ P
		 *  [] indicates from slave
		 *  CC channel value
		 *  RR channel read back
		 */
		uint8_t	cBuf[2] = { ds2482CMD_CHSL, ~Chan<<4 | Chan } ;	// calculate Channel value
		psDS248X->Rptr	= ds248xREG_CHAN ;
		IF_SYSTIMER_START(debugTIMING, systimerDS248xA) ;
		ds248xI2C_WriteDelayRead(psDS248X, cBuf, sizeof(cBuf), 0) ;
		IF_SYSTIMER_STOP(debugTIMING, systimerDS248xA) ;
		// value read back not same as the channel number sent so verify the return
		// against the code expected, but store the actual channel number if successful
		if (psDS248X->Rchan != ds248x_V2N[Chan]) {
			SL_ERR("Read %d != %d Expected", psDS248X->RegX[psDS248X->Rptr], ds248x_V2N[Chan]) ;
			return 0 ;
		}
		psDS248X->CurChan	= Chan ;
		return 1 ;
	}
#endif
	return 1 ;
}

int		ds248xOWSetSPU(ds248x_t * psDS248X) {
	psDS248X->SPU = 1 ;
	int iRV = ds248xWriteConfig(psDS248X) ;
	if (iRV == 0 || psDS248X->SPU == 0) {
		SL_ERR("I2C=%u  Ch=%u  iRV=%d Fail", psDS248X->I2Cnum, psDS248X->CurChan, iRV) ;
	}
	return psDS248X->SPU ;
}

int		ds248xOWReset(ds248x_t * psDS248X) {
// 1-Wire reset (Case B)
//	S AD,0 [A] 1WRS [A] Sr AD,1 [A] [Status] A [Status] A\ P
//									\--------/
//						Repeat until 1WB bit has changed to 0
//  [] indicates from slave
	// No SPU == 0 checking, will be reset by itself...
	uint8_t	cChr = ds248xCMD_1WRS ;
	psDS248X->Rptr	= ds248xREG_STAT ;
	IF_SYSTIMER_START(debugTIMING, systimerDS248xB) ;
	ds248xI2C_WriteDelayRead(psDS248X, &cChr, sizeof(cChr), psDS248X->OWS ? owDELAY_RST_OD : owDELAY_RST) ;
	IF_SYSTIMER_STOP(debugTIMING, systimerDS248xB) ;
	return psDS248X->PPD ;
}

int		ds248xOWSpeed(ds248x_t * psDS248X, bool speed) {
	psDS248X->OWS = speed ;
	ds248xWriteConfig(psDS248X) ;
	IF_myASSERT(debugRESULT, psDS248X->OWS == speed) ;
	return psDS248X->OWS ;
}

int		ds248xOWLevel(ds248x_t * psDS248X, bool level) {
	if (level == owPOWER_STRONG) {						// DS248X only allow STANDARD
		return psDS248X->SPU ;
	}
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

int		ds248xOWWriteBytePower(ds248x_t * psDS248X, uint8_t sendbyte) {
	if (ds248xOWSetSPU(psDS248X) == 0) {
		return 0 ;
	}
	ds248xOWWriteByte(psDS248X, sendbyte) ;
	return psDS248X->SPU ;
}

uint8_t	ds248xOWReadByte(ds248x_t * psDS248X) {
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
