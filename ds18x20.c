/*
 * Copyright (c) 2018-22 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#include	<string.h>

#include	"hal_variables.h"

#include	"onewire_platform.h"
#include	"onewire.h"
#include	"ds248x.h"
#include	"ds18x20.h"
#include	"endpoints.h"

#include	"options.h"
#include	"printfx.h"
#include	"syslog.h"
#include	"systiming.h"					// timing debugging

#include	"x_errors_events.h"

#define	debugFLAG					0xF000

#define	debugREAD					(debugFLAG & 0x0001)
#define	debugCONVERT				(debugFLAG & 0x0002)
#define	debugPOWER					(debugFLAG & 0x0004)
#define	debugSPAD					(debugFLAG & 0x0008)

#define	debugOWP					(debugFLAG & 0x0008)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ##################################### Developer notes ###########################################

/* DS18x20 is a 1-wire type device and thus BUS oriented:
 * 	multiple devices sharing a single bus.
 * 	each device can be individually R/W addressed
 * 	some operations eg temp sample/convert
 * 		happens reasonably slowly (up to 750mS)
 * 		can be triggered to execute in parallel for all "equivalent" devices on a bus
 *	To optimise operation, this driver is based on the following decisions/constraints:
 *		Tsns is specified at device type (psEWP level) for ALL /ow/ds18x20 devices and will
 *		always trigger a sample+convert operation for ALL devices on a bus at same time.
 *		EWP Tsns kept at a value equal to lowest of all EWS Tsns values
 *		Maintain a minimum Tsns of 1000mSec to be bigger than the ~750mS standard.
 * 	Test parasitic power
 * 	Test & benchmark overdrive speed
 * 	Implement and test ALARM scan and over/under alarm status scan
 *
 * 	Optimisation:
 * 	If more than 1 DS248XZ present Tsns will trigger convert on 1st bus of each DS248x device (parallelism)
 * 	Each device will start a timer to call handler to read and convert all DS18X20's on the bus
 * 	Handler will loop and read each sensor on the current bus.
 * 	If more than 1 bus on the device (DS2482-800) handler will release current bus.
 * 	The next bus will be selected and convert trigger triggered.
 * 	Logic will ONLY trigger convert on bus if 1 or more ds18x20 were discovered at boot.
 *
 */

// ############################################# Macros ############################################

#define	ds18x20DELAY_CONVERT		750		// mSec
#define	ds18x20DELAY_SP_COPY		11		// mSec
#define	ds18x20T_SNS_MIN			1000
#define	ds18x20T_SNS_NORM			60000

// ################################## DS18X20 1-Wire Commands ######################################

#define	DS18X20_CONVERT				0x44
#define	DS18X20_COPY_SP				0x48
#define	DS18X20_WRITE_SP			0x4E
#define	DS18X20_READ_PSU			0xB4
#define	DS18X20_RECALL_EE			0xB8
#define	DS18X20_READ_SP				0xBE

// ################################ Forward function declaration ###################################


// ######################################### Constants #############################################


// ###################################### Local variables ##########################################

ds18x20_t *	psaDS18X20 = NULL;
u8_t	Fam10Count = 0, Fam28Count = 0, Fam10_28Count = 0;

// #################################### Local ONLY functions #######################################

/**
 * @brief	Read power status bit, AI1 operation, select & release bus
 * @param	psDS18X20
 * @return	Power status
 */
bool ds18x20CheckPower(ds18x20_t * psDS18X20) {
	if (OWResetCommand(&psDS18X20->sOW, DS18X20_READ_PSU, owADDR_SKIP, 0) == 0) return 0 ;
	psDS18X20->sOW.PSU = OWReadBit(&psDS18X20->sOW) ;	// 0=parasitic 1=external
	return psDS18X20->sOW.PSU ;
}

// ###################################### scratchpad support #######################################

/**
 * @brief
 * @param	psDS18X20
 * @param	Len
 * @return
 * @note	Timing is as follows
 *	OWReset		196/1348uS
 *	OWCommand	1447/7740uS
 *	OWReadBlock		163/860 per byte, 326/1720 for temperature, 815/4300 for all.
 *	Total Time	1969/10808 for temperature
 */
int	ds18x20ReadSP(ds18x20_t * psDS18X20, int Len) {
	if (OWResetCommand(&psDS18X20->sOW, DS18X20_READ_SP, owADDR_MATCH, 0) == 0) return 0 ;
	OWReadBlock(&psDS18X20->sOW, psDS18X20->RegX, Len);
	IF_PL(debugSPAD, "%`-B ", Len, psDS18X20->RegX);
	// If full SP read, verify CRC else terminate read
	return (Len == SO_MEM(ds18x20_t, RegX))
			? OWCheckCRC(psDS18X20->RegX, SO_MEM(ds18x20_t, RegX))
			: OWReset(&psDS18X20->sOW) ;
}

int	ds18x20WriteSP(ds18x20_t * psDS18X20) {
	if (OWResetCommand(&psDS18X20->sOW, DS18X20_WRITE_SP, owADDR_MATCH, 0) == 0) return 0 ;
	int Len = (psDS18X20->sOW.ROM.HexChars[owFAMILY] == OWFAMILY_28) ? 3 : 2 ;	// Thi, Tlo [+Conf]
	OWWriteBlock(&psDS18X20->sOW, (u8_t *) &psDS18X20->Thi, Len);
	IF_PL(debugSPAD, "%`-B ", Len, psDS18X20->RegX);
	return 1 ;
}

int	ds18x20WriteEE(ds18x20_t * psDS18X20) {
	if (OWResetCommand(&psDS18X20->sOW, DS18X20_COPY_SP, owADDR_MATCH, 1) == 0) return 0 ;
	vTaskDelay(pdMS_TO_TICKS(ds18x20DELAY_SP_COPY));
	OWLevel(&psDS18X20->sOW, owPOWER_STANDARD);
	return 1 ;
}

// ################################ Basic temperature support ######################################

int	ds18x20TempRead(ds18x20_t * psDS18X20) { return ds18x20ReadSP(psDS18X20, 2) ; }

// ###################################### IRMACOS support ##########################################

int	ds18x20Initialize(ds18x20_t * psDS18X20) {
	if (ds18x20ReadSP(psDS18X20, SO_MEM(ds18x20_t, RegX)) == 0) return 0;
	ds18x20CheckPower(psDS18X20);
	psDS18X20->Res	= (psDS18X20->sOW.ROM.HexChars[owFAMILY] == OWFAMILY_28)
					? psDS18X20->fam28.Conf >> 5
					: owFAM28_RES9B;
	return ds18x20ConvertTemperature(psDS18X20);
}

/**
 * @brief	reset device to default via SP, not written to EE
 * @param	psDS18X20
 * @return
 */
int	ds18x20ResetConfig(ds18x20_t * psDS18X20) {
	psDS18X20->Thi	= 75 ;
	psDS18X20->Tlo	= 70 ;
	if (psDS18X20->sOW.ROM.HexChars[owFAMILY] == OWFAMILY_28)
		psDS18X20->fam28.Conf = 0x7F ;	// 12 bit resolution
	ds18x20WriteSP(psDS18X20) ;
	return ds18x20Initialize(psDS18X20) ;
}

int	ds18x20ConvertTemperature(ds18x20_t * psDS18X20) {
	const u8_t u8Mask[4] = { 0xF8, 0xFC, 0xFE, 0xFF } ;
	uint16_t u16Adj = (psDS18X20->Tmsb << 8) | (psDS18X20->Tlsb & u8Mask[psDS18X20->Res]) ;
	psDS18X20->sEWx.var.val.x32.f32 = (float) u16Adj / 16.0 ;
	if (debugTRACK && ioB1GET(ioDS18x20)) {
		flagmask_t sFM = { .u32Val = makeMASK09x23(0,1,0,0,0,0,0,0,0,psDS18X20->Idx) };
		ds18x20Print_CB(sFM, psDS18X20);
	}
	return 1;
}

// ################################ Rules configuration support ####################################

int	ds18x20SetResolution(ds18x20_t * psDS18X20, int Res) {
	if (psDS18X20->sOW.ROM.HexChars[owFAMILY] == OWFAMILY_28 && INRANGE(9, Res, 12, int)) {
		Res -= 9 ;
		u8_t u8Res = (Res << 5) | 0x1F ;
		IF_P(debugTRACK && ioB1GET(ioMode), "SP Res x%02X->x%02X (%d->%d)\r\n",
				psDS18X20->fam28.Conf, u8Res, psDS18X20->Res, Res) ;
		IF_RETURN_X(psDS18X20->fam28.Conf == u8Res, 0);	// nothing changed
		psDS18X20->fam28.Conf = u8Res;
		psDS18X20->Res = Res ;
		return 1 ;										// changed, must write
	}
	RETURN_MX("Invalid Family/Resolution", erINVALID_VALUE);
}

int	ds18x20SetAlarms(ds18x20_t * psDS18X20, int Lo, int Hi) {
	if (INRANGE(-128, Lo, 127, int) && INRANGE(-128, Hi, 127, int)) {
		IF_P(debugTRACK && ioB1GET(ioMode), "SP Tlo:%d -> %d  Thi:%d -> %d\r\n", psDS18X20->Tlo, Lo, psDS18X20->Thi, Hi) ;
		IF_RETURN_X(psDS18X20->Tlo == Lo && psDS18X20->Thi == Hi, 0);
		psDS18X20->Tlo = Lo ;
		psDS18X20->Thi = Hi ;
		return 1 ;										// changed, must write
	}
	RETURN_MX("Invalid Lo/Hi alarm limits", erINVALID_VALUE);
}

int	ds18x20ConfigMode (struct rule_t * psR, int Xcur, int Xmax) {
	if (psaDS18X20 == NULL)
		RETURN_MX("No DS18x20 enumerated", erINVALID_OPERATION);
	// support syntax mode /ow/ds18x20 idx lo hi res [1=persist]
	int iRV = erFAILURE, iRVx = erFAILURE;
	u8_t	AI = psR->ActIdx ;
	u32_t lo	= psR->para.x32[AI][0].u32;
	u32_t hi	= psR->para.x32[AI][1].u32;
	u32_t res = psR->para.x32[AI][2].u32;
	u32_t wr	= psR->para.x32[AI][3].u32;
	IF_P(debugTRACK && ioB1GET(ioMode), "MODE 'DS18X20' Xcur=%d Xmax=%d lo=%d hi=%d res=%d wr=%d\r\n", Xcur, Xmax, lo, hi, res, wr);

	IF_RETURN_MX(wr != 0 && wr != 1, "Invalid persist flag, not 0/1", erINVALID_MODE);
	do {
		ds18x20_t * psDS18X20 = &psaDS18X20[Xcur] ;
		if (OWP_BusSelect(&psDS18X20->sOW) == 1) {
			// Do resolution 1st since small range (9-12) a good test for valid parameter
			iRV = ds18x20SetResolution(psDS18X20, res) ;
			if (iRV > erFAILURE) {
				iRVx = ds18x20SetAlarms(psDS18X20, lo, hi);
				if (iRVx > erFAILURE) {
					if (iRV == 1 || iRVx == 1) {	// 1 or both changed in scratchpad
						iRV = ds18x20WriteSP(psDS18X20);
						if (wr == 1)
							ds18x20WriteEE(psDS18X20);
					}
				}
			}
			OWP_BusRelease(&psDS18X20->sOW) ;
		}
		if (iRVx < erSUCCESS)
			break ;
	} while (++Xcur < Xmax) ;
	return iRV < erSUCCESS? iRV : iRVx;
}

// #################################### 1W Platform support ########################################

epw_t * ds18x20GetWork(int x) ;
void ds18x20SetDefault(epw_t * psEWP, epw_t *psEWS) ;
void ds18x20SetSense(epw_t * psEWP, epw_t * psEWS) ;
float ds18x20GetTemperature(epw_t * psEWx) ;

const vt_enum_t	sDS18X20Func = {
	.work	= ds18x20GetWork,
	.reset	= ds18x20SetDefault,
	.sense	= ds18x20SetSense,
	.get	= ds18x20GetTemperature,
};

epw_t * ds18x20GetWork(int x) {
	IF_myASSERT(debugPARAM, halCONFIG_inSRAM(psaDS18X20) && (x < Fam10_28Count));
	return &psaDS18X20[x].sEWx ;
}

void ds18x20SetDefault(epw_t * psEWP, epw_t * psEWS) {
	IF_myASSERT(debugPARAM, psEWP->fSECsns == 0) ;
	psEWP->Rsns = 0 ;	// Stop EWP sensing ,vEpConfigReset() will handle EWx
}

void ds18x20SetSense(epw_t * psEWP, epw_t * psEWS) {
	/* Optimal 1-Wire bus operation require that all devices (of a type) are detected
	 * (and read) in a single bus scan. BUT, for the DS18x20 the temperature conversion
	 * time is 750mSec (per bus or device) at normal (not overdrive) bus speed.
	 * When we get here the psEWS structure will already having been configured with the
	 * parameters as supplied, just check & adjust for validity & new min Tsns */
	if (psEWS->Tsns < ds18x20T_SNS_MIN)
		psEWS->Tsns = ds18x20T_SNS_MIN ;				// default to minimum
	if (psEWS->Tsns < psEWP->Tsns)
		psEWP->Tsns = psEWS->Tsns ;						// set lowest of EWP/EWS
	psEWS->Tsns = 0 ;									// discard EWS value
	psEWP->Rsns = psEWP->Tsns ;							// restart SNS timer
}

float ds18x20GetTemperature(epw_t * psEWx) { return psEWx->var.val.x32.f32; }

int	ds18x20EnumerateCB(flagmask_t sFM, owdi_t * psOW) {
	ds18x20_t * psDS18X20 = &psaDS18X20[sFM.uCount];
	memcpy(&psDS18X20->sOW, psOW, sizeof(owdi_t));
	psDS18X20->Idx = sFM.uCount;

	epw_t * psEWS = &psDS18X20->sEWx;
	memset(psEWS, 0, sizeof(epw_t));
	psEWS->var.def.cv.vf = vfFXX;
	psEWS->var.def.cv.vt = vtVALUE;
	psEWS->var.def.cv.vs = vs32B;
	psEWS->var.def.cv.vc = 1;
	psEWS->idx = sFM.uCount;
	psEWS->uri = URI_DS18X20;
	ds18x20Initialize(psDS18X20);

	owbi_t * psOW_CI = psOWP_BusGetPointer(OWP_BusP2L(psOW));
	switch(psOW->ROM.HexChars[owFAMILY]) {
	case OWFAMILY_10: psOW_CI->ds18s20++;	break;
	case OWFAMILY_28: psOW_CI->ds18b20++;	break;
	default: IF_myASSERT(debugRESULT, 0);
	}
	return 1 ;											// number of devices enumerated
}

int	ds18x20Enumerate(void) {
	u8_t	ds18x20NumDev = 0;
	Fam10_28Count = Fam10Count + Fam28Count;
	IF_SL_INFO(debugOWP, "DS18x20 found %d devices", Fam10_28Count) ;
	IF_SYSTIMER_INIT(debugTIMING, stDS1820A, stMILLIS, "DS1820A", 10, 1000) ;
	IF_SYSTIMER_INIT(debugTIMING, stDS1820B, stMILLIS, "DS1820B", 1, 10) ;

	// Init primary EWP endpoint (leave fSecSNS = 0 to force parallel sensing
	epw_t * psEWP = &table_work[URI_DS18X20];
	psEWP->var.def.cv.pntr = 1;
	psEWP->var.def.cv.vf = vfFXX ;
	psEWP->var.def.cv.vs = vs32B ;
	psEWP->var.def.cv.ve = 1;							// Enumerated type
	psEWP->var.def.cv.vc = Fam10_28Count ;
	psEWP->var.val.px.pv = (void *) &sDS18X20Func ;
	psEWP->Tsns	= psEWP->Rsns = ds18x20T_SNS_NORM;
	psEWP->uri = URI_DS18X20 ;							// Used in OWPlatformEndpoints()

	psaDS18X20 = pvRtosMalloc(Fam10_28Count * sizeof(ds18x20_t)) ;
	memset(psaDS18X20, 0, Fam10_28Count * sizeof(ds18x20_t)) ;
	int	iRV = 0 ;
	if (Fam10Count) {
		iRV = OWP_Scan(OWFAMILY_10, ds18x20EnumerateCB) ;
		if (iRV > 0) ds18x20NumDev += iRV ;
	}
	if (Fam28Count) {
		iRV = OWP_Scan(OWFAMILY_28, ds18x20EnumerateCB) ;
		if (iRV > 0) ds18x20NumDev += iRV ;
	}
	if (ds18x20NumDev == Fam10_28Count)
		iRV = ds18x20NumDev ;
	else {
		SL_ERR("Only %d of %d enumerated!!!", ds18x20NumDev, Fam10_28Count) ;
		iRV = erFAILURE ;
	}
	return iRV ;										// number of devices enumerated
}

int	ds18x20Print_CB(flagmask_t FlagMask, ds18x20_t * psDS18X20) {
	int iRV = OWP_Print1W_CB((flagmask_t) (FlagMask.u32Val & ~mfbNL), &psDS18X20->sOW) ;
	iRV += printfx(" Traw=0x%04X/%.4fC Tlo=%d Thi=%d Res=%d",
		psDS18X20->Tmsb << 8 | psDS18X20->Tlsb,
		psDS18X20->sEWx.var.val.x32.f32, psDS18X20->Tlo, psDS18X20->Thi, psDS18X20->Res+9) ;
	if (psDS18X20->sOW.ROM.HexChars[owFAMILY] == OWFAMILY_28)
		iRV += printfx(" Conf=0x%02X %s", psDS18X20->fam28.Conf, ((psDS18X20->fam28.Conf >> 5) != psDS18X20->Res) ? "ERROR" : "OK") ;
	if (FlagMask.bNL)
		iRV += printfx("\r\n") ;
	return iRV ;
}

TickType_t ds18x20CalcDelay(ds18x20_t * psDS18X20, bool All) {
	TickType_t tConvert = pdMS_TO_TICKS(ds18x20DELAY_CONVERT) ;
	/* ONLY decrease delay if:
	 * 	specific ROM is addressed AND and it is DS18B20 ; OR
	 * 	ROM match skipped AND only DS18B20 devices on the bus */
	owbi_t * psOWBI = psOWP_BusGetPointer(OWP_BusP2L(&psDS18X20->sOW)) ;
	if (((All == 1) && (psOWBI->ds18s20 == 0)) ||
		((All == 0) && (psDS18X20->sOW.ROM.HexChars[owFAMILY] == OWFAMILY_28))) {
		tConvert /= (4 - psDS18X20->Res) ;
	}
	return tConvert ;
}

/**
 * @brief	Trigger convert (bus at a time) then read SP, normalise RAW value & persist in EPW
 * @param 	psEPW
 * @return
 */
int	ds18x20StartAllInOne(epw_t * psEWP) {
	u8_t	PrevBus = 0xFF ;
	for (int i = 0; i < Fam10_28Count; ++i) {
		ds18x20_t * psDS18X20 = &psaDS18X20[i];
		if (psDS18X20->sOW.PhyBus != PrevBus) {
			if (OWP_BusSelect(&psDS18X20->sOW) == 0) continue ;
			if (OWResetCommand(&psDS18X20->sOW, DS18X20_CONVERT, owADDR_SKIP, 1) == 1) {
				PrevBus = psDS18X20->sOW.PhyBus ;
				vTaskDelay(ds18x20CalcDelay(psDS18X20, 1)) ;
				OWLevel(&psDS18X20->sOW, owPOWER_STANDARD) ;
				OWP_BusRelease(&psDS18X20->sOW) ;		// keep locked for period of delay
			}
		}
		if (OWP_BusSelect(&psDS18X20->sOW) && ds18x20ReadSP(psDS18X20, 2)) {
			ds18x20ConvertTemperature(psDS18X20) ;
			OWP_BusRelease(&psDS18X20->sOW) ;			// TODO maybe simplify Release ?
		} else
			SL_ERR("Read/Convert failed") ;
	}
	return erSUCCESS ;
}

int	ds18x20StepTwoBusConvert(ds18x20_t * psDS18X20, int i) {
	if (OWP_BusSelect(&psDS18X20->sOW) == 1) {
		OWResetCommand(&psDS18X20->sOW, DS18X20_CONVERT, owADDR_SKIP, 1);
		vTimerSetTimerID(psaDS248X[psDS18X20->sOW.DevNum].tmr, (void *) i);
		xTimerStart(psaDS248X[psDS18X20->sOW.DevNum].tmr, ds18x20CalcDelay(psDS18X20, 1));
		SL_DBG("Start Dev=%d Ch=%d", psDS18X20->sOW.DevNum, psDS18X20->sOW.PhyBus);
		return 1 ;
	}
	SL_ERR("Failed to start convert Dev=%d Ch=%d", psDS18X20->sOW.DevNum, psDS18X20->sOW.PhyBus) ;
	return 0 ;
}

int ds18x20StepOneStart(epw_t * psEWx) {			// Step 1: Start CONVERT on each physical bus
	u8_t	PrevDev = 0xFF ;						// where 1+ DS18x20 has been enumerated on.
	for (int i = 0; i < Fam10_28Count; ++i) {		// Although sense is configured on primary level,
		ds18x20_t * psDS18X20 = &psaDS18X20[i];		// log can be different for each instance
		if (psDS18X20->sOW.DevNum != PrevDev) {
			if (ds18x20StepTwoBusConvert(psDS18X20, i) == 1) {
//				TRACK("DevNum=%d\r\n", psDS18X20->sOW.DevNum);
				PrevDev = psDS18X20->sOW.DevNum;
			}
		}
	}
	return erSUCCESS;
}

void ds18x20StepThreeRead(TimerHandle_t pxHandle) {
	int	i = (int) pvTimerGetTimerID(pxHandle);
	do {												// Handle all sensors on this BUS
		ds18x20_t * psDS18X20 = &psaDS18X20[i];
		if (ds18x20ReadSP(psDS18X20, 2) == 1) ds18x20ConvertTemperature(psDS18X20);
		else SL_ERR("Read/Convert failed");
		++i ;
		// no more sensors or different device - release bus, exit loop
		if ((i == Fam10_28Count) || (psDS18X20->sOW.DevNum != psaDS18X20[i].sOW.DevNum)) {
//			TRACK("i=%d  DevNum=%d\r\n", i, psDS18X20->sOW.DevNum);
			OWP_BusRelease(&psDS18X20->sOW);
			break;
		}
		// more sensors, same device but new bus - release bus, start convert on new bus.
		if (psDS18X20->sOW.PhyBus != psaDS18X20[i].sOW.PhyBus) {
//			TRACK("i=%d  DevNum=%d\r\n", i, psDS18X20->sOW.DevNum);
			OWP_BusRelease(&psDS18X20->sOW);
			ds18x20StepTwoBusConvert(&psaDS18X20[i], i);
			break;
		}
		// more sensors, same device and same bus
	} while  (i < Fam10_28Count) ;
}

// ######################################### Reporting #############################################

void ds18x20ReportAll(void) {
	for (int i = 0; i < Fam10_28Count; ++i) {
		if (i == 0)
			printfx("\r# DS18x20 #\r\n");
		flagmask_t sFM = { .u32Val = makeMASK09x23(0,1,1,1,1,1,1,1,1,i) };
		ds18x20Print_CB(sFM, &psaDS18X20[i]) ;
	}
	if (Fam10_28Count)
		printfx("\r\n");
}
