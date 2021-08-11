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

// ##################################### Developer notes ###########################################
/*
	Test at 400KHx I2C speed, maybe add auto detect and step up mode in SCAN routine?
	Add support to configure the PADJ register timing
 */

// ###################################### General macros ###########################################

#define	debugFLAG					0xF007

#define	debugBUS_CFG				(debugFLAG & 0x0001)
#define	debugCONFIG					(debugFLAG & 0x0002)
#define	debugCRC					(debugFLAG & 0x0004)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ###################################### Local variables ##########################################

const char * const RegNames[ds248xREG_NUM] = {"Stat", "Data", "Chan", "Conf", "Port" } ;

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

// ################################ Local ONLY utility functions ###################################

int ds248xLogError(ds248x_t * psDS248X, char const * pcMess) {
	SL_ERR("Dev=%d Ch=%d %s error", psDS248X->psI2C->DevIdx, psDS248X->CurChan, pcMess) ;
	ds248xReset(psDS248X) ;
	return 0 ;
}

int	ds248xCheckRead(ds248x_t * psDS248X, uint8_t Value) {
	int iRV = 1 ;
	if (psDS248X->Rptr == ds248xREG_STAT) {		// STATus register
		if (psDS248X->OWB) {					// Check for error if not blocking in I2C task
			iRV = ds248xLogError(psDS248X, "OWB") ;
		} else {
			#if	(!defined(NDEBUG) || defined(DEBUG))
			const char * const StatNames[8] = { "OWB", "PPD", "SD", "LL", "RST", "SBR", "TSB", "DIR" } ;
			const uint8_t DS248Xmask[4] = { 0b00000111, 0b00011111, 0b00111111, 0b11111111 } ;
			uint8_t Mask = DS248Xmask[OWflags.Level] ;
			uint8_t StatX = psDS248X->PrvStat[psDS248X->CurChan] ;
			if (ioB1GET(ioB1_0) && ((psDS248X->Rstat & Mask) != (StatX & Mask))) {
				char * pcBuf = pcBitMapDecodeChanges(StatX, psDS248X->Rstat, 0x000000FF, StatNames) ;
				printfx("D=%d C=%u x%02X->x%02X %s\n", psDS248X->psI2C->DevIdx,
					psDS248X->CurChan, StatX, psDS248X->Rstat, pcBuf) ;
				free(pcBuf) ;
			}
			psDS248X->PrvStat[psDS248X->CurChan] = psDS248X->Rstat ;
			#endif
		}
	} else if (psDS248X->Rptr == ds248xREG_CONF) {		// CONFiguration register
		if (Value == 0xC3) return iRV ;					// Just read CONF, no change
		Value &= 0x0F ;
		if (Value != psDS248X->Rconf) {
			ds248x_conf_t sConf = { .Rconf = Value } ;
			char caBuf[18] ;
			char * pcMess	= (psDS248X->OWS != sConf.OWS) ? "OWS"
							: (psDS248X->SPU != sConf.SPU) ? "SPU"
							: ((psDS248X->psI2C->Type == i2cDEV_DS2484) && (psDS248X->PDN != sConf.PDN)) ? "PDN"
							: (psDS248X->APU != sConf.APU) ? "APU" : "???" ;
			snprintfx(caBuf, sizeof(caBuf), "W=x%02X R=x%02X (%s)", Value, psDS248X->Rconf, pcMess) ;
			iRV = ds248xLogError(psDS248X, caBuf) ;
		} else {
			#if	(!defined(NDEBUG) || defined(DEBUG))
			if (ioB1GET(ioB1_0) && (psDS248X->Rconf != psDS248X->PrvConf[psDS248X->CurChan])) {
				const char * const ConfNames[4] = { "APU", "PDN", "SPU", "OWS" } ;
				uint8_t ConfX = psDS248X->PrvConf[psDS248X->CurChan] ;
				char * pcBuf = pcBitMapDecodeChanges(ConfX, psDS248X->Rconf, 0x0000000F, ConfNames) ;
				printfx("D=%d C=%u x%02X->x%02X %s\n", psDS248X->psI2C->DevIdx,
					psDS248X->CurChan, ConfX, psDS248X->Rconf, pcBuf) ;
				free(pcBuf) ;
				psDS248X->PrvConf[psDS248X->CurChan] = psDS248X->Rconf ;
			}
			#endif
		}
	} else if (psDS248X->Rptr == ds248xREG_CHAN && psDS248X->Rchan != ds248x_V2N[psDS248X->CurChan]) {
		iRV = ds248xLogError(psDS248X, "CHAN") ;
	}
	return iRV ;
}

int	ds248xI2C_Read(ds248x_t * psDS248X) {
	#if (d248xAUTO_LOCK == 1)
	xRtosSemaphoreTake(&psDS248X->mux, portMAX_DELAY) ;
	#endif
	IF_myASSERT(debugBUS_CFG, psDS248X->OWB == 0) ;
	int iRV = halI2C_Queue(psDS248X->psI2C, i2cR_B,
			NULL, 0,
			&psDS248X->RegX[psDS248X->Rptr], SO_MEM(ds248x_t, Rconf),
			(i2cq_p1_t) NULL, (i2cq_p2_t) NULL) ;
	#if (d248xAUTO_LOCK == 1)
	xRtosSemaphoreGive(&psDS248X->mux) ;
	#endif
	if (iRV == erSUCCESS) return ds248xCheckRead(psDS248X, 0xFF) ;
	return 0 ;
}

int	ds248xI2C_WriteDelayRead(ds248x_t * psDS248X, uint8_t * pTxBuf, size_t TxSize, uint32_t uSdly) {
	#if (d248xAUTO_LOCK == 1)
	xRtosSemaphoreTake(&psDS248X->mux, portMAX_DELAY) ;
	#endif
	IF_myASSERT(debugBUS_CFG, psDS248X->OWB == 0) ;
	int iRV = halI2C_Queue(psDS248X->psI2C, i2cWDR_B,
			pTxBuf, TxSize,
			&psDS248X->RegX[psDS248X->Rptr], 1,
			(i2cq_p1_t) uSdly, (i2cq_p2_t) NULL) ;
	#if (d248xAUTO_LOCK == 1)
	xRtosSemaphoreGive(&psDS248X->mux) ;
	#endif
	if (iRV == erSUCCESS) return ds248xCheckRead(psDS248X, (TxSize > 1) ? pTxBuf[1] : 0xFF) ;
	return 0 ;
}

/**
 * @brief	reset device, read & store status
 * @return	status of RST bit ie 1 or 0
 *
 *	WDR			100KHz	400KHz
 *				200uS	50uS
 *		uS-----+------+-------+
 *	NS	0		200		50
 *	OD	0		200		50
 */
int ds248xReset(ds248x_t * psDS248X) {
	// Device Reset
	//	S AD,0 [A] DRST [A] Sr AD,1 [A] [SS] A\ P
	//  [] indicates from slave
	//  SS status byte to read to verify state
	uint8_t	cChr 	= ds248xCMD_DRST ;
	psDS248X->Rptr	= ds248xREG_STAT ;					// After ReSeT pointer set to STATus register
	IF_SYSTIMER_START(debugTIMING, stDS248xA) ;
	ds248xI2C_WriteDelayRead(psDS248X, &cChr, sizeof(cChr), 0) ;
	IF_SYSTIMER_STOP(debugTIMING, stDS248xA) ;
	psDS248X->Rdata		= 0 ;
	psDS248X->Rconf		= 0 ;							// all bits cleared (default) config
	psDS248X->CurChan	= 0 ;
	psDS248X->Rchan		= ds248x_V2N[0] ;				// DS2482-800 specific
	psDS248X->Rpadj		= 0 ;							// DS2484 specific
	return psDS248X->RST ;
}

/**
 * @brief	Write config value lower nibble, upper nibble bitwise inverted.
 * @param	psDS248X
 * @return	1 if config written & response correct else 0
 *
 *	WWDR		100KHz	400KHz
 *				300uS	75uS
 *		uS-----+------+-------+
 *	NS	0		300		75
 *	OD	0		300		75
 */
int	ds248xWriteConfig(ds248x_t * psDS248X) {
// Write configuration (Case A)
//	S AD,0 [A] WCFG [A] CF [A] Sr AD,1 [A] [CF] A\ P
//  [] indicates from slave
//  CF configuration byte to write
	uint8_t	config	= psDS248X->Rconf & 0x0F ;
	uint8_t	cBuf[2] = { ds248xCMD_WCFG , (~config << 4) | config } ;
	psDS248X->Rptr = ds248xREG_CONF ;
	IF_SYSTIMER_START(debugTIMING, stDS248xA) ;
	int iRV = ds248xI2C_WriteDelayRead(psDS248X, cBuf, sizeof(cBuf), 0) ;
	IF_SYSTIMER_STOP(debugTIMING, stDS248xA) ;
	return iRV ;
}

/**
 * @brief	Set the Read Pointer and reads the register
 *			Once set the pointer remains static to allow reread of same register
 * @return	1 if successfully read else 0
 *
 *	WWDR		100KHz	400KHz
 *				300uS	75uS
 *		uS-----+------+-------+
 *	NS	0		300		75
 *	OD	0		300		75
 */
int	ds248xReadRegister(ds248x_t * psDS248X, uint8_t Reg) {
	// check for validity of CHAN (only DS2482-800) and PADJ (only DS2484)
	int iRV ;
	if ((Reg == ds248xREG_CHAN && psDS248X->psI2C->Type != i2cDEV_DS2482_800) ||
		(Reg == ds248xREG_PADJ && psDS248X->psI2C->Type != i2cDEV_DS2484)) {
		halI2C_DeviceReport((void *) ((uint32_t) psDS248X->I2Cnum)) ;
		printfx("Invalid device/register combo Reg=%d (%s)",Reg, RegNames[Reg]) ;
		iRV = 0 ;
	} else {
		psDS248X->Rptr	= Reg ;
		uint8_t	cBuf[2] = { ds248xCMD_SRP, (~Reg << 4) | Reg } ;
		IF_SYSTIMER_START(debugTIMING, stDS248xA) ;
		iRV = ds248xI2C_WriteDelayRead(psDS248X, cBuf, sizeof(cBuf), 0) ;
		IF_SYSTIMER_STOP(debugTIMING, stDS248xA) ;
	}
	return iRV ;
}

/**
 * @brief	Select the 1-Wire bus on a DS2482-800.
 * @param	psDS248X
 * @param 	Chan
 * @return	1 if bus selected
 *			0 if device not detected or failure to perform select
 *
 *	WWR			100KHz	400KHz
 *				300uS	75uS
 *		uS-----+------+-------+
 *	NS	0		300		75
 *	OD	0		300		75
 */
int	ds248xBusSelect(ds248x_t * psDS248X, uint8_t Bus) {
	int iRV = 1 ;
	if ((psDS248X->psI2C->Type == i2cDEV_DS2482_800)
	&& (psDS248X->CurChan != Bus))	{					// optimise to avoid unnecessary IO
		/* Channel Select (Case A)
		 *	S AD,0 [A] CHSL [A] CC [A] Sr AD,1 [A] [RR] A\ P
		 *  [] indicates from slave
		 *  CC channel value
		 *  RR channel read back
		 */
		uint8_t	cBuf[2] = { ds2482CMD_CHSL, (~Bus << 4) | Bus } ;	// calculate Channel value
		psDS248X->Rptr	= ds248xREG_CHAN ;
		psDS248X->CurChan = Bus ;			// save in advance will auto reset if error
		IF_SYSTIMER_START(debugTIMING, stDS248xA) ;
		iRV = ds248xI2C_WriteDelayRead(psDS248X, cBuf, sizeof(cBuf), 0) ;
		IF_SYSTIMER_STOP(debugTIMING, stDS248xA) ;
	}
#if (d248xAUTO_LOCK == 2)
	xRtosSemaphoreTake(&psDS248X->mux, portMAX_DELAY) ;
#endif
	return iRV ;
}

void ds248xBusRelease(ds248x_t * psDS248X) {
#if (d248xAUTO_LOCK == 2)
	xRtosSemaphoreGive(&psDS248X->mux) ;
#endif
}

// #################################### DS248x debug/reporting #####################################

int	ds248xReportStatus(uint8_t Num, ds248x_stat_t Stat) {
	return printfx("STAT(0) #%u=0x%02X  DIR=%c  TSB=%c  SBR=%c  RST=%c  LL=%c  SD=%c  PPD=%c  1WB=%c\n",
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
int	ds248xReportRegister(ds248x_t * psDS248X, int Reg, bool Refresh) {
	int iRV = 0 ;
	int	Chan ;
	if (Refresh) ds248xBusSelect(psDS248X, psDS248X->CurChan) ;
	switch (Reg) {
	case ds248xREG_STAT:
		if (Refresh && ds248xReadRegister(psDS248X, Reg) == 0) break;
	#if	(!defined(NDEBUG)) || defined(DEBUG)
		for (int i = 0; i < (psDS248X->NumChan ? 8 : 1); ++i)
			iRV += ds248xReportStatus(i, (ds248x_stat_t) psDS248X->PrvStat[i]);
	#else
		iRV += ds248xReportStatus(0, (ds248x_stat_t) psDS248X->Rstat) ;
	#endif
		break ;

	case ds248xREG_DATA:
		iRV += printfx("DATA(1)=0x%02X (Last read)\n", psDS248X->Rdata) ;
		break ;

	case ds248xREG_CHAN:
		if ((psDS248X->psI2C->Type != i2cDEV_DS2482_800)
		|| (Refresh && ds248xReadRegister(psDS248X, Reg) == 0)) break;
		// Channel, start by finding the matching Channel #
		for (Chan = 0; Chan < (psDS248X->NumChan ? 8 : 1) && psDS248X->Rchan != ds248x_V2N[Chan]; ++Chan) ;
		IF_myASSERT(debugRESULT, Chan < (psDS248X->NumChan ? 8 : 1) && psDS248X->Rchan == ds248x_V2N[Chan]) ;
		iRV = printfx("CHAN(2)=0x%02X Chan=%d Xlat=0x%02X\n", psDS248X->Rchan, Chan, ds248x_V2N[Chan]) ;
		break ;

	case ds248xREG_CONF:
		if (Refresh && ds248xReadRegister(psDS248X, Reg) == 0) break;
		iRV += printfx("CONF(3)=0x%02X  1WS=%c  SPU=%c  PDN=%c  APU=%c\n",
				psDS248X->Rconf,
				psDS248X->OWS	? '1' : '0',
				psDS248X->SPU	? '1' : '0',
				psDS248X->PDN	? '1' : '0',
				psDS248X->APU	? '1' : '0') ;
		break ;

	case ds248xREG_PADJ:
		if ((Refresh == 0)
		|| (psDS248X->psI2C->Type != i2cDEV_DS2484)
		|| (ds248xReadRegister(psDS248X, Reg) == 0)) break;
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
	if (Refresh) ds248xBusRelease(psDS248X) ;
	return iRV ;
}

/**
 * ds248xReport() - report decoded status of a specific device
 */
void ds248xReport(ds248x_t * psDS248X, bool Refresh) {
	halI2C_DeviceReport((void *) psDS248X->psI2C) ;
	for (int Reg = 0; Reg < ds248xREG_NUM; ds248xReportRegister(psDS248X, Reg++, Refresh)) ;
	printfx("\n") ;
}

/**
 * ds248xReportAll() - report decoded status of all devices and all registers
 */
void ds248xReportAll(bool Refresh) {
	for (int i = 0; i < ds248xCount; ds248xReport(&psaDS248X[i++], Refresh)) ;
}

// ################### Identification, Diagnostics & Configuration functions #######################

/**
 * ds248xIdentify() - device reset+register reads to ascertain exact device type
 * @return	erSUCCESS if supported device was detected, if not erFAILURE
 */
int	ds248xIdentify(i2c_di_t * psI2C_DI) {
	ds248x_t sDS248X = { 0 } ;							// temporary device structure
	psI2C_DI->Delay	= pdMS_TO_TICKS(10) ;				// default device timeout
	psI2C_DI->Test	= 1 ;								// and halI2C modules
	sDS248X.psI2C	= psI2C_DI ;						// link to I2C device discovered
	if (ds248xReset(&sDS248X) == 1) {
		psI2C_DI->Type = i2cDEV_DS2484 ;
		int iRV = ds248xReadRegister(&sDS248X, ds248xREG_PADJ) ;
		if (iRV == 1 &&	sDS248X.VAL == 0b00000110) {	// PADJ=OK & PAR=000 & OD=0
			psI2C_DI->DevIdx = ds248xCount++ ;		// valid DS2484
		} else {
			psI2C_DI->Type = i2cDEV_DS2482_800 ;		// assume -800 there
			iRV = ds248xReadRegister(&sDS248X, ds248xREG_CHAN) ;
			if (iRV == 0) {								// CSR read FAIL
				psI2C_DI->Type = i2cDEV_DS2482_10X ;	// NOT YET TESTED !!!!
				psI2C_DI->DevIdx = ds248xCount++ ;		// valid 2482-10x
			} else if (sDS248X.Rchan == ds248x_V2N[0]) {// CHAN=0 default
				psI2C_DI->DevIdx = ds248xCount++ ;		// valid 2482-800
			} else psI2C_DI->Type = i2cDEV_UNDEF ;		// not successful, undefined
		}
	}
	psI2C_DI->Test	= 0 ;
	if (psI2C_DI->Type != i2cDEV_UNDEF) psI2C_DI->Speed = i2cSPEED_400 ;
#if (d248xAUTO_LOCK == 1)
	if (sDS248X.mux) vSemaphoreDelete(sDS248X.mux) ;
#endif
	return (psI2C_DI->Type == i2cDEV_UNDEF) ? erFAILURE : erSUCCESS ;
}

int	ds248xConfig(i2c_di_t * psI2C_DI) {
	if (psaDS248X == NULL) {							// 1st time here...
		IF_myASSERT(debugPARAM, psI2C_DI->DevIdx == 0) ;
		psaDS248X = malloc(ds248xCount * sizeof(ds248x_t)) ;
		memset(psaDS248X, 0, ds248xCount * sizeof(ds248x_t)) ;
		IF_SYSTIMER_INIT(debugTIMING, stDS248xA, stMICROS, "DS248xA", 100, 1000) ;
		IF_SYSTIMER_INIT(debugTIMING, stDS248xB, stMICROS, "DS248xB", 200, 2000) ;
		IF_SYSTIMER_INIT(debugTIMING, stDS248xC, stMICROS, "DS248xC", 10, 100) ;
		IF_SYSTIMER_INIT(debugTIMING, stDS248xD, stMICROS, "DS248xD", 300, 3000) ;
		IF_SYSTIMER_INIT(debugTIMING, stDS248xE, stMICROS, "DS248xE", 300, 3000) ;
		IF_SYSTIMER_INIT(debugTIMING, stDS248xF, stMICROS, "DS248xF", 300, 3000) ;
	}
	ds248x_t * psDS248X = &psaDS248X[psI2C_DI->DevIdx] ;
	psDS248X->psI2C		= psI2C_DI ;
	if (psI2C_DI->Type == i2cDEV_DS2482_800) psDS248X->NumChan = 1;	// 0=1Ch, 1=8Ch
	ds248xReConfig(psI2C_DI);

	void OWP_DS18X20ReadSample(TimerHandle_t) ;
	psDS248X->tmr = xTimerCreate("ds248x", pdMS_TO_TICKS(5), pdFALSE, NULL, OWP_DS18X20ReadSample) ;
	return erSUCCESS ;
}

void ds248xReConfig(i2c_di_t * psI2C_DI) {
	ds248x_t * psDS248X = &psaDS248X[psI2C_DI->DevIdx] ;
	ds248xReset(psDS248X) ;
	psDS248X->Rconf	= 0;
	psDS248X->APU	= 1;								// LSBit
	ds248xWriteConfig(psDS248X) ;
}

// ################################## DS248x-x00 1-Wire functions ##################################

/**
 * @brief	Reset 1W bus
 * @param	psDS248X
 * @param 	Chan
 * @return	1 if device detected, 0 if not
 *
 *	WDR			100KHz	400KHz
 *				200uS	50uS
 *		uS-----+------+-------+
 *	NS	1148	1348	1198
 *	OD	146		346		196
 */
int	ds248xOWReset(ds248x_t * psDS248X) {
	// DS2482-800 datasheet page 7 para 2
	if (psDS248X->SPU == owPOWER_STRONG) ds248xOWLevel(psDS248X, owPOWER_STANDARD);
// 1-Wire reset (Case B)
//	S AD,0 [A] 1WRS [A] Sr AD,1 [A] [Status] A [Status] A\ P
//									\--------/
//						Repeat until 1WB bit has changed to 0
//  [] indicates from slave
	uint8_t	cChr = ds248xCMD_1WRS ;
	psDS248X->Rptr	= ds248xREG_STAT ;
	IF_SYSTIMER_START(debugTIMING, stDS248xB) ;
	ds248xI2C_WriteDelayRead(psDS248X, &cChr, sizeof(cChr), psDS248X->OWS ? owDELAY_RST_OD : owDELAY_RST) ;
	IF_SYSTIMER_STOP(debugTIMING, stDS248xB) ;
	return psDS248X->PPD ;
}

int	ds248xOWSpeed(ds248x_t * psDS248X, bool speed) {
	psDS248X->OWS = speed;
	ds248xWriteConfig(psDS248X);
	return psDS248X->OWS;
}

/**
 *	WWR			100KHz	400KHz
 *				300uS	75uS
 *		uS-----+------+-------+
 *	NS	0		300		75
 *	OD	0		300		75
 */
int	ds248xOWLevel(ds248x_t * psDS248X, bool level) {
	psDS248X->SPU = level;
	ds248xWriteConfig(psDS248X);
	return psDS248X->SPU;
}

bool ds248xOWTouchBit(ds248x_t * psDS248X, bool Bit) {
// 1-Wire bit (Case B)
//	S AD,0 [A] 1WSB [A] BB [A] Sr AD,1 [A] [Status] A [Status] A\ P
//										   \--------/
//								Repeat until 1WB bit has changed to 0
//  [] indicates from slave
//  BB indicates byte containing bit value in msbit
	uint8_t	cBuf[2] = {	ds248xCMD_1WSB, Bit << 7 } ;
	psDS248X->Rptr	= ds248xREG_STAT ;
	ds248xI2C_WriteDelayRead(psDS248X, cBuf, sizeof(cBuf), psDS248X->OWS ? owDELAY_SB_OD : owDELAY_SB) ;
	return psDS248X->SBR ;
}

/**
 *	WWDR		100KHz	400KHz
 *				300uS	75uS
 *		uS-----+------+-------+
 *	NS	560		860		635
 *	OD	88		388		163
 */
void ds248xOWWriteByte(ds248x_t * psDS248X, uint8_t Byte) {
// 1-Wire Write Byte (Case B)
//	S AD,0 [A] 1WWB [A] DD [A] Sr AD,1 [A] [Status] A [Status] A\ P
//										   \--------/
//							Repeat until 1WB bit has changed to 0
//  [] indicates from slave
//  DD data to write
	uint8_t	cBuf[2] = { ds248xCMD_1WWB, Byte } ;
	psDS248X->Rptr	= ds248xREG_STAT ;
	IF_SYSTIMER_START(debugTIMING, stDS248xD) ;
	ds248xI2C_WriteDelayRead(psDS248X, cBuf, sizeof(cBuf), psDS248X->OWS ? owDELAY_WB_OD : owDELAY_WB) ;
	IF_SYSTIMER_STOP(debugTIMING, stDS248xD) ;
}

/**
 *	WRDWWR		100KHz	400KHz
 *				500uS	125uS
 *		uS-----+------+-------+
 *	NS	583		1083	708
 *	OD	88		588		213
 */
uint8_t	ds248xOWReadByte(ds248x_t * psDS248X) {
/* 1-Wire Read Bytes (Case C)
 *	S AD,0 [A] 1WRB [A] Sr AD,1 [A] [Status] A [Status] A\
 *										\--------/
 *							Repeat until 1WB bit has changed to 0
 *	Sr AD,0 [A] SRP [A] E1 [A] Sr AD,1 [A] DD A\ P
 *  [] indicates from slave
 *  DD data read	*/
	uint8_t	cBuf = ds248xCMD_1WRB;
	psDS248X->Rptr = ds248xREG_STAT;
	IF_SYSTIMER_START(debugTIMING, stDS248xE);
	ds248xI2C_WriteDelayRead(psDS248X, &cBuf, sizeof(cBuf), psDS248X->OWS ? owDELAY_RB_OD : owDELAY_RB);
	IF_SYSTIMER_STOP(debugTIMING, stDS248xE);
	ds248xReadRegister(psDS248X, ds248xREG_DATA);
	return psDS248X->Rdata;
}

bool ds248xOWSearchTriplet(ds248x_t * psDS248X, bool bDir) {
// 1-Wire Triplet (Case B)
//	S AD,0 [A] 1WT [A] SS [A] Sr AD,1 [A] [Status] A [Status] A\ P
//							  \--------/
//				Repeat until 1WB bit has changed to 0
//  [] indicates from slave
//  SS indicates byte containing search direction bit value in msbit
	uint8_t	cBuf[2] = { ds248xCMD_1WT, bDir ? 0x80 : 0x00 } ;
	psDS248X->Rptr	= ds248xREG_STAT ;
	IF_SYSTIMER_START(debugTIMING, stDS248xF) ;
	ds248xI2C_WriteDelayRead(psDS248X, cBuf, sizeof(cBuf), psDS248X->OWS ? owDELAY_ST_OD : owDELAY_ST) ;
	IF_SYSTIMER_STOP(debugTIMING, stDS248xF) ;
	return psDS248X->Rstat ;
}
