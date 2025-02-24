// ds248x.c - Copyright (c) 2020-25 Andre M. Maree / KSS Technologies (Pty) Ltd.

#include "hal_platform.h"

#if (HAL_DS248X > 0)
#include "hal_i2c_common.h"
#include "hal_options.h"
#include "FreeRTOS_Support.h"
#include "onewire_platform.h"
#include "printfx.h"
#include "syslog.h"
#include "systiming.h"								// timing debugging
#include "errors_events.h"
#include "string_general.h"

#include <string.h>

// ##################################### Developer notes ###########################################
/*
	Test at 400KHx I2C speed, maybe add auto detect and step up mode in SCAN routine?
	Add support to configure the PADJ register timing
 */

// ###################################### General macros ###########################################

#define	debugFLAG					0xF000
#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ######################################## Build macros ###########################################

#define	ds248xLOCK_DIS				0					// no locking
#define	ds248xLOCK_IO				1					// un/locked on I2C access level
#define	ds248xLOCK_BUS				2					// un/locked on Bus select level
#define	ds248xLOCK					ds248xLOCK_DIS

// ##################################### Local structures ##########################################

typedef union __attribute__((packed)) {
	struct {
/*LSB*/	u8_t	OWB	: 1;			// 1-Wire Busy
		u8_t	PPD	: 1;			// Presence Pulse Detected
		u8_t	SD	: 1;
		u8_t	LL	: 1;			// Link Level
		u8_t	RST	: 1;			// ReSeT
		u8_t	SBR	: 1;			// Single Bit Read
		u8_t	TSB	: 1;			//
/*MSB*/	u8_t	DIR	: 1;			// DIRection
	};
	u8_t STAT;
} ds248x_stat_t;

typedef union __attribute__((packed)) {
	struct __attribute__((packed)) {
/*LSB*/	u8_t	APU	: 1;			// Active Pull Up
		u8_t	PDN	: 1;			// Pull Down (DS2484 only)
		u8_t	SPU	: 1;			// Strong Pull Up
		u8_t	OWS	: 1;			// 1-Wire Speed
/*MSB*/	u8_t	RES	: 4;
	};
	u8_t	Rconf;
} ds248x_conf_t;

typedef union __attribute__((packed)) {
	struct __attribute__((packed)) {
/*LSB*/	u8_t	VAL	: 4;			// PARameter VALue
		u8_t	OD	: 1;			// OverDrive control
/*MSB*/	u8_t	PAR	: 3;			// PARameter selector
	};
	u8_t RadjX;
} ds248x_padj_t;

// ###################################### Local variables ##########################################

const char * const RegNames[ds248xREG_NUM] = {"Stat", "Data", "Chan", "Conf", "Port" };

// DS2482-800 only CHAN register xlat	0	  1		2	  3		4	  5		6	  7
static const u8_t ds248x_V2N[8] = { 0xB8, 0xB1, 0xAA, 0xA3, 0x9C, 0x95, 0x8E, 0x87 };
// DS2484 only reporting/debugging
static const u8_t Trstl[16]	= { 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74 };
static const u8_t Tmsp0[16]	= { 58, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 76, 76, 76, 76, 76 };
static const u8_t Tmsp1[16]	= { 55, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 110, 110, 110 };
static const u8_t Twol0[16]	= { 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 70, 70, 70, 70, 70, 70 };
static const u8_t Twol1[16]	= { 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 100, 100, 100, 100, 100 };
static const uint16_t Trec0[16]	= { 275, 275, 275, 275, 275, 275, 525, 775, 1025, 1275, 1525, 1775, 2025, 2275, 2525, 2525 };
static const uint16_t Rwpu[16]	= { 500, 500, 500, 500, 500, 500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000 };

// ##################################### Global variables ##########################################

u8_t ds248xCount = 0;
ds248x_t * psaDS248X = NULL;
static int ResetOK = 0, ResetErr = 0;


/**
 * Report decoded status of all devices & registers
 */
static int ds248xLogError(ds248x_t * psDS248X, char const * pcMess) {
	SL_ERR("Dev=%d  Ch=%d  %s error", psDS248X->psI2C->DevIdx, psDS248X->CurChan, pcMess);
	return ds248xReset(psDS248X);
}

/**
 * @brief
 * @param
 * @return
 */
int	ds248xCheckRead(ds248x_t * psDS248X, u8_t Value) {
	int iRV = 1;
	if (psDS248X->Rptr == ds248xREG_STAT) {				// STATus register
		if (psDS248X->OWB) {							// Check for error in case not blocking in I2C task
			iRV = ds248xLogError(psDS248X, "OWB");
		} else {
			#if	(appPRODUCTION == 0)
			if (ioB2GET(dbgDS248X) > 1) {
				const u8_t DS248Xmask[3] = { 0b00001111, 0b00111111, 0b11111111 };
				u8_t Mask = DS248Xmask[ioB2GET(dbgDS248X) - 1];
				u8_t StatX = psDS248X->PrvStat[psDS248X->CurChan];
				if ((psDS248X->Rstat & Mask) != (StatX & Mask)) {
					wprintfx(NULL, "D=%d  C=%u  x%02X->x%02X  ", psDS248X->psI2C->DevIdx, psDS248X->CurChan, StatX, psDS248X->Rstat);
					ds248xReportStatus(NULL, StatX, psDS248X->Rstat);
				}
			}
			psDS248X->PrvStat[psDS248X->CurChan] = psDS248X->Rstat;
			#endif
		}
	} else if (psDS248X->Rptr == ds248xREG_CONF) {		// CONFiguration register
		if (Value == 0xC3)
			return iRV;									// Just read CONF, no change
		Value &= 0x0F;
		if (Value != psDS248X->Rconf) {
			ds248x_conf_t sConf = { .Rconf = Value };
			char caBuf[36];
			char * pcMess	= (psDS248X->OWS != sConf.OWS) ? "OWS"
							: (psDS248X->SPU != sConf.SPU) ? "SPU"
							: ((psDS248X->psI2C->Type == i2cDEV_DS2484) && (psDS248X->PDN != sConf.PDN)) ? "PDN"
							: (psDS248X->APU != sConf.APU) ? "APU" : "???";
			snprintfx(caBuf, sizeof(caBuf), "W=x%.2x  R=x%.2x (%s)", Value, psDS248X->Rconf, pcMess);
			iRV = ds248xLogError(psDS248X, caBuf);
		} else {
			#if	(configPRODUCTION == 0)
			if (ioB2GET(dbgDS248X)) {
				u8_t ConfX = psDS248X->PrvConf[psDS248X->CurChan];
				if (psDS248X->Rconf != ConfX) {
					wprintfx(NULL, "D=%d C=%u x%02X->x%02X ", psDS248X->psI2C->DevIdx, psDS248X->CurChan, ConfX, psDS248X->Rconf);
					ds248xReportConfig(NULL, ConfX, psDS248X->Rconf);
				}
			}
			psDS248X->PrvConf[psDS248X->CurChan] = psDS248X->Rconf;
			#endif
		}
		IF_myASSERT(debugRESULT, psDS248X->APU == 1);
	} else if (psDS248X->Rptr == ds248xREG_CHAN && (psDS248X->Rchan != ds248x_V2N[psDS248X->CurChan])) {
		char caBuf[36];
		snprintfx(caBuf, sizeof(caBuf)," CHAN (x%02X vs x%02X)", psDS248X->Rchan, ds248x_V2N[psDS248X->CurChan]);
		iRV = ds248xLogError(psDS248X, caBuf);
	}
	return iRV;
}

/**
 * @brief
 * @param
 * @return
 */
__attribute__((unused)) static int ds248xRead(ds248x_t * psDS248X) {
	#if (ds248xLOCK == ds248xLOCK_IO)
		xRtosSemaphoreTake(&psDS248X->mux, portMAX_DELAY);
	#endif
	IF_myASSERT(debugTRACK, psDS248X->OWB == 0);
	int iRV = halI2C_Queue(psDS248X->psI2C, i2cR_B, NULL, 0, &psDS248X->RegX[psDS248X->Rptr],
		SO_MEM(ds248x_t, Rconf), (i2cq_p1_t) NULL, (i2cq_p2_t) NULL);
	#if (ds248xLOCK == ds248xLOCK_IO)
		xRtosSemaphoreGive(&psDS248X->mux);
	#endif
	return iRV == erSUCCESS ? ds248xCheckRead(psDS248X, 0xFF) : 0;
}

/**
 * @brief
 * @param
 * @return
 */
static int ds248xWriteDelayRead(ds248x_t * psDS248X, u8_t * pTxBuf, size_t TxSize, u32_t uSdly) {
	#if (ds248xLOCK == ds248xLOCK_IO)
		xRtosSemaphoreTake(&psDS248X->mux, portMAX_DELAY);
	#endif
	IF_myASSERT(debugTRACK, psDS248X->OWB == 0);
	int iRV = halI2C_Queue(psDS248X->psI2C, i2cWDR_B, pTxBuf, TxSize, &psDS248X->RegX[psDS248X->Rptr],
		psDS248X->Rptr == ds248xREG_PADJ ? SO_MEM(ds248x_t, Rpadj) : 1, (i2cq_p1_t) uSdly, (i2cq_p2_t) NULL);
	#if (ds248xLOCK == ds248xLOCK_IO)
		xRtosSemaphoreGive(&psDS248X->mux);
	#endif
	return iRV;
}

/**
 * @brief
 * @param[in]
 * @return	1 if OK, 0 if error
 */
static int ds248xWriteDelayReadCheck(ds248x_t * psDS248X, u8_t * pTxBuf, size_t TxSize, u32_t uSdly) {
	int iRV = ds248xWriteDelayRead(psDS248X, pTxBuf, TxSize, uSdly);
	return (iRV == erSUCCESS) ? ds248xCheckRead(psDS248X, (TxSize > 1) ? pTxBuf[1] : 0xFF) : 0;
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
}

/**
 * @brief	Write config value lower nibble, upper nibble bitwise inverted.
 * @param	psDS248X
 * @return	result from ds248xWriteDelayReadCheck(), 1 if config written & response correct else 0
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
	u8_t config	= psDS248X->Rconf & 0x0F;
	u8_t cBuf[2] = { ds248xCMD_WCFG , (~config << 4) | config };
	psDS248X->Rptr = ds248xREG_CONF;
	IF_SYSTIMER_START(debugTIMING, stDS248xIO);
	int iRV = ds248xWriteDelayReadCheck(psDS248X, cBuf, sizeof(cBuf), 0);
	IF_SYSTIMER_STOP(debugTIMING, stDS248xIO);
	return iRV;
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
int	ds248xBusSelect(ds248x_t * psDS248X, u8_t Bus) {
	int iRV = 1;
	#if (ds248xLOCK == ds248xLOCK_BUS)
	xRtosSemaphoreTake(&psDS248X->mux, portMAX_DELAY);
	#endif
	if ((psDS248X->psI2C->Type == i2cDEV_DS2482_800) && (psDS248X->CurChan != Bus))	{					// optimise to avoid unnecessary IO
		/* Channel Select (Case A)
		 *	S AD,0 [A] CHSL [A] CC [A] Sr AD,1 [A] [RR] A\ P
		 *  [] indicates from slave
		 *  CC channel value
		 *  RR channel read back
		 */
		u8_t cBuf[2] = { ds2482CMD_CHSL, (~Bus << 4) | Bus };	// calculate Channel value
		psDS248X->Rptr = ds248xREG_CHAN;
		psDS248X->CurChan = Bus;			// save in advance will auto reset if error
// ################### Identification, Diagnostics & Configuration functions #######################

int ds248xReset(ds248x_t * psDS248X) {
	int Retries = 0;
	do {
		u8_t cChr = ds248xCMD_DRST;
		psDS248X->Rptr = ds248xREG_STAT;				// After ReSeT pointer set to STATus register
		IF_SYSTIMER_START(debugTIMING, stDS248xIO);
		ds248xWriteDelayRead(psDS248X, &cChr, sizeof(cChr), 0);
		IF_SYSTIMER_STOP(debugTIMING, stDS248xIO);
	}
	#if (ds248xLOCK == ds248xLOCK_BUS)
	if (iRV == 0) xRtosSemaphoreGive(&psDS248X->mux);	// error, release...
	#endif
	return iRV;
}

void ds248xBusRelease(ds248x_t * psDS248X) {
	#if (ds248xLOCK == ds248xLOCK_BUS)
	xRtosSemaphoreGive(&psDS248X->mux);
	#endif
		if (psDS248X->RST == 1) {						// ReSeT successful?
			++ResetOK;									// yes, update counter
			break;										// break to return
		}
		++ResetErr;										// update FAIL counter
		vTaskDelay(pdMS_TO_TICKS(10));
	} while (++Retries < 20);
	// set register mirrors & variables to defaults
	psDS248X->CurChan = 0;
	psDS248X->Rdata = 0;
	psDS248X->Rchan = ds248x_V2N[0];					// DS2482-800 specific
	psDS248X->Rconf = 0;								// all bits cleared (default) config
	memset(psDS248X->Rpadj, 0, SO_MEM(ds248x_t, Rpadj));// DS2484 specific
	if (Retries)
		SL_LOG(psDS248X->RST ? SL_SEV_WARNING : SL_SEV_ALERT, "(%#I) %s after %d retries  OK=%d  Err=%d", nvsWifi.ipSTA, psDS248X->RST ? "Success" : "FAILED", Retries, ResetOK, ResetErr);
	return psDS248X->RST;
}

// ################### Identification, Diagnostics & Configuration functions #######################

/**
 * @brief	device reset+register reads to ascertain exact device type
 * @return	erSUCCESS if supported device was detected, if not erFAILURE
 */
int	ds248xIdentify(i2c_di_t * psI2C) {
	ds248x_t sDS248X = { 0 };							// temporary device structure
	sDS248X.psI2C = psI2C;
	psI2C->Speed = i2cSPEED_400;
	psI2C->TObus = 25;
	psI2C->Test	= 1;
	int iRV;
	if (ds248xReset(&sDS248X) == 1) {
		psI2C->Type = i2cDEV_DS2484;					// assume
		iRV = ds248xReadRegister(&sDS248X, ds248xREG_PADJ);
		// PADJ=OK & PAR=000 & OD=0, valid DS2484
		if (iRV == 1 &&	sDS248X.Rpadj[0] == 0b00000110) goto done;
		psI2C->Type = i2cDEV_DS2482_10X;
		iRV = ds248xReadRegister(&sDS248X, ds248xREG_CHAN);
		// DS2482-10X CSR read FAIL, 2482-10x, NOT YET TESTED !!!
		if (iRV == erSUCCESS) goto done;
		if (sDS248X.Rchan == ds248x_V2N[0]) {			// CHAN=0 default, valid 2482-800
			psI2C->Type = i2cDEV_DS2482_800;
			iRV = erSUCCESS;
		} else {
			psI2C->Type = i2cDEV_UNDEF;
			iRV = erINV_WHOAMI;
		}
	} else {
		iRV = erINV_DEVICE;
	}
done:
	if (psI2C->Type != i2cDEV_UNDEF) {
		psI2C->DevIdx = ds248xCount++;
		psI2C->IDok = 1;
		psI2C->Test	= 0;
	}
	return iRV;
	#if (ds248xLOCK == ds248xLOCK_IO)					/* if locking enabled.... */
		if (sDS248X.mux)								/* mux will be initialised in ds248xReset() */
			vSemaphoreDelete(sDS248X.mux);				/* thus delete and free up allocation */
	#endif
}

/**
 * Sets default device config
 *	1-Wire speed (c1WS) = standard (0)
 *	Strong pull-up (cSPU) = off (0)
 *	Presence pulse masking (cPPM) = off (0)		[Discontinued, support removed]
 *	Active pull-up (cAPU) = on (ds2484DCNF_APU = 0x01)
 */
int	ds248xConfig(i2c_di_t * psI2C) {
	if (psI2C->IDok == 0)
		return erINV_STATE;
	if (psaDS248X == NULL) {
		IF_myASSERT(debugPARAM, psI2C->DevIdx == 0);
		psaDS248X = malloc(ds248xCount * sizeof(ds248x_t));
		if (psaDS248X == NULL)
			return erNO_MEM;
		memset(psaDS248X, 0, ds248xCount * sizeof(ds248x_t));
		IF_SYSTIMER_INIT(debugTIMING, stDS248xIO, stMICROS, "DS248xIO", 300, 2700);
		IF_SYSTIMER_INIT(debugTIMING, stDS248x1R, stMICROS, "DS248x1R", 1400, 18000);
		IF_SYSTIMER_INIT(debugTIMING, stDS248xWR, stMICROS, "DS248xWR", 900, 4000);
		IF_SYSTIMER_INIT(debugTIMING, stDS248xRD, stMICROS, "DS248xRD", 300, 3000);
		IF_SYSTIMER_INIT(debugTIMING, stDS248xST, stMICROS, "DS248xST", 500, 4400);
	}
	ds248x_t * psDS248X = &psaDS248X[psI2C->DevIdx];
	if (psI2C->CFGok == 0) {							// definite 1st time for specific device...
		psDS248X->psI2C = psI2C;
		if (psI2C->Type == i2cDEV_DS2482_800)
			psDS248X->NumChan = 1;						// 0=1Ch, 1=8Ch
		#if (HAL_DS18X20 > 0)
			void ds18x20StepThreeRead(TimerHandle_t);
			psDS248X->th = xTimerCreateStatic("tmrDS248x", pdMS_TO_TICKS(5), pdFALSE, NULL, ds18x20StepThreeRead, &psDS248X->ts);
		#endif
	}

	psI2C->CFGok = 0;
	halEventUpdateDevice(devMASK_DS248X, 0);
	int iRV = ds248xReset(psDS248X);
	if (iRV != 1)									return erINV_DEVICE;
	psDS248X->Rconf = 0;
	psDS248X->APU = 1;									// LSBit
	iRV = ds248xWriteConfig(psDS248X);
	IF_myASSERT(debugRESULT, psDS248X->APU == 1);
	if (iRV < erSUCCESS)
		goto exit;
	psI2C->CFGok = 1;
	halEventUpdateDevice(devMASK_DS248X, 1);
exit:
	return iRV;
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
	if (psDS248X->SPU == owPOWER_STRONG)
		ds248xOWLevel(psDS248X, owPOWER_STANDARD);
	// 1-Wire reset (Case B)
	//	S AD,0 [A] 1WRS [A] Sr AD,1 [A] [Status] A [Status] A\ P
	//									\--------/
	//						Repeat until 1WB bit has changed to 0
	//  [] indicates from slave
	u8_t cChr = ds248xCMD_1WRS;
	psDS248X->Rptr = ds248xREG_STAT;
	IF_SYSTIMER_START(debugTIMING, stDS248x1R);
	ds248xWriteDelayReadCheck(psDS248X, &cChr, sizeof(cChr), psDS248X->OWS ? owDELAY_RST_OD : owDELAY_RST);
	IF_SYSTIMER_STOP(debugTIMING, stDS248x1R);
	return psDS248X->PPD;
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
	u8_t cBuf[2] = { ds248xCMD_1WSB, Bit << 7 };
	psDS248X->Rptr = ds248xREG_STAT;
	ds248xWriteDelayReadCheck(psDS248X, cBuf, sizeof(cBuf), psDS248X->OWS ? owDELAY_SB_OD : owDELAY_SB);
	return psDS248X->SBR;
}

/**
 *	WWDR		100KHz	400KHz
 *				300uS	75uS
 *		uS-----+------+-------+
 *	NS	560		860		635
 *	OD	88		388		163
 */
u8_t ds248xOWWriteByte(ds248x_t * psDS248X, u8_t Byte) {
	// 1-Wire Write Byte (Case B)
	//	S AD,0 [A] 1WWB [A] DD [A] Sr AD,1 [A] [Status] A [Status] A\ P
	//										   \--------/
	//							Repeat until 1WB bit has changed to 0
	//  [] indicates from slave
	//  DD data to write
	u8_t cBuf[2] = { ds248xCMD_1WWB, Byte };
	psDS248X->Rptr = ds248xREG_STAT;
	IF_SYSTIMER_START(debugTIMING, stDS248xWR);
	ds248xWriteDelayReadCheck(psDS248X, cBuf, sizeof(cBuf), psDS248X->OWS ? owDELAY_WB_OD : owDELAY_WB);
	IF_SYSTIMER_STOP(debugTIMING, stDS248xWR);
	return psDS248X->Rstat;
}

/**
 *	WRDWWR		100KHz	400KHz
 *				500uS	125uS
 *		uS-----+------+-------+
 *	NS	583		1083	708
 *	OD	88		588		213
 */
u8_t ds248xOWReadByte(ds248x_t * psDS248X) {
	// 1-Wire Read Bytes (Case C)
	//	S AD,0 [A] 1WRB [A] Sr AD,1 [A] [Status] A [Status] A\ '
	//										\--------/
	//							Repeat until 1WB bit has changed to 0
	//	Sr AD,0 [A] SRP [A] E1 [A] Sr AD,1 [A] DD A\ P
	//  [] indicates from slave
	//  DD data read
	u8_t cBuf = ds248xCMD_1WRB;
	psDS248X->Rptr = ds248xREG_STAT;
	IF_SYSTIMER_START(debugTIMING, stDS248xRD);
	ds248xWriteDelayReadCheck(psDS248X, &cBuf, sizeof(cBuf), psDS248X->OWS ? owDELAY_RB_OD : owDELAY_RB);
	IF_SYSTIMER_STOP(debugTIMING, stDS248xRD);
	ds248xReadRegister(psDS248X, ds248xREG_DATA);
	return psDS248X->Rdata;
}

u8_t ds248xOWSearchTriplet(ds248x_t * psDS248X, u8_t u8Dir) {
	// 1-Wire Triplet (Case B)
	//	S AD,0 [A] 1WT [A] SS [A] Sr AD,1 [A] [Status] A [Status] A\ P
	//							  \--------/
	//				Repeat until 1WB bit has changed to 0
	//  [] indicates from slave
	//  SS indicates byte containing search direction bit value in msbit
	u8_t cBuf[2] = { ds248xCMD_1WT, u8Dir ? 0x80 : 0x00 };
	psDS248X->Rptr = ds248xREG_STAT;
	IF_SYSTIMER_START(debugTIMING, stDS248xST);
	ds248xWriteDelayReadCheck(psDS248X, cBuf, sizeof(cBuf), psDS248X->OWS ? owDELAY_ST_OD : owDELAY_ST);
	IF_SYSTIMER_STOP(debugTIMING, stDS248xST);
	return psDS248X->Rstat;
}

// #################################### DS248x debug/reporting #####################################

int ds248xReportStatus(report_t * psR, u8_t Val1, u8_t Val2) {
	const char * const StatNames[8] = { "OWB", "PPD", "SD", "LL", "RST", "SBR", "TSB", "DIR" };
	return xBitMapDecodeChanges(psR, Val1, Val2, 0x000000FF, StatNames);
}

int ds248xReportConfig(report_t * psR, u8_t Val1, u8_t Val2) {
	const char * const ConfNames[4] = { "APU", "PDN", "SPU", "OWS" };
	return xBitMapDecodeChanges(psR, Val1, Val2, 0x0000000F, ConfNames);
}

int	ds248xReportRegister(report_t * psR, ds248x_t * psDS248X, int Reg) {
	int iRV = 0, Chan;
	switch (Reg) {
	case ds248xREG_STAT:
	#if	(appPRODUCTION == 0)
		iRV += wprintfx(psR, "STAT(0)");
		for (int i = 0; i < (psDS248X->NumChan ? 8 : 1); ++i) {
			iRV += wprintfx(psR, "\t#%u:", i, psDS248X->PrvStat[i]);
			iRV += ds248xReportStatus(psR, 0, psDS248X->PrvStat[i]);
		}
	#endif
		break;

	case ds248xREG_DATA:
		iRV += wprintfx(psR, "DATA(1)=0x%02X (Last read)\r\n", psDS248X->Rdata);
		break;

	case ds248xREG_CHAN:
		if (psDS248X->psI2C->Type != i2cDEV_DS2482_800)
			break;
		// Channel, start by finding the matching Channel #
		for (Chan = 0; Chan < (psDS248X->NumChan ? 8 : 1) && psDS248X->Rchan != ds248x_V2N[Chan]; ++Chan);
		IF_myASSERT(debugRESULT, Chan < (psDS248X->NumChan ? 8 : 1) && psDS248X->Rchan == ds248x_V2N[Chan]);
		iRV += wprintfx(psR, "CHAN(2)=0x%02X Chan=%d Xlat=0x%02X\r\n", psDS248X->Rchan, Chan, ds248x_V2N[Chan]);
		break;

	case ds248xREG_CONF:
		iRV += wprintfx(psR, "CONF(3)=0x%02X  ", psDS248X->Rconf);
		iRV += ds248xReportConfig(psR, 0, psDS248X->Rconf);
		break;

	case ds248xREG_PADJ:
		if (psDS248X->psI2C->Type != i2cDEV_DS2484)
			break;
		ds248xReadRegister(psDS248X, Reg);
		ds248x_padj_t sPadj;
		sPadj.RadjX = psDS248X->Rpadj[0];
		iRV += wprintfx(psR, "PADJ(4)=0x%02X  OD=%c | tRSTL=%duS", sPadj.RadjX,
				sPadj.OD ? CHR_1 : CHR_0, Trstl[sPadj.VAL] * (sPadj.OD ? 1 : 10));
		sPadj.RadjX = psDS248X->Rpadj[1];
		iRV += wprintfx(psR, " | tMSP=%.1fuS", sPadj.OD ? (double) Tmsp1[sPadj.VAL] / 10.0 : (double) Tmsp0[sPadj.VAL]);
		sPadj.RadjX = psDS248X->Rpadj[2];
		iRV += wprintfx(psR, " | tWOL=%.1fuS", sPadj.OD ? (double) Twol1[sPadj.VAL] / 10.0 : (double) Twol0[sPadj.VAL]);
		sPadj.RadjX = psDS248X->Rpadj[3];
		iRV += wprintfx(psR, " | tREC0=%.2fuS", (double) Trec0[sPadj.VAL] / 100.0);
		sPadj.RadjX = psDS248X->Rpadj[4];
		iRV += wprintfx(psR, " | rWPU=%f ohm\r\n", (double) Rwpu[sPadj.VAL]);
		break;
	}
	return iRV;
}

int ds248xReport(report_t * psR, ds248x_t * psDS248X) {
	int iRV = halI2C_DeviceReport(psR, (void *) psDS248X->psI2C);
	for (int Reg = 0; Reg < ds248xREG_NUM; iRV += ds248xReportRegister(psR, psDS248X, Reg++));
#if (HAL_DS18X20 > 0)
	iRV += xRtosReportTimer(psR, psDS248X->th);
#endif
	iRV += wprintfx(psR, strNL);
	return iRV;
}

int ds248xReportAll(report_t * psR) {
	int iRV = 0;
	for (int i = 0; i < ds248xCount; iRV += ds248xReport(psR, &psaDS248X[i++]));
	return iRV;
}

#endif
