/*
 * Copyright 2020-2021 Andre M. Maree/KSS Technologies (Pty) Ltd.
 * onewire_platform.c
 */

#include	"hal_variables.h"

#include	"onewire_platform.h"
#include	"task_events.h"
#include	"endpoints.h"

#include	"printfx.h"
#include	"syslog.h"
#include	"systiming.h"								// timing debugging

#include	"x_errors_events.h"
#include	"x_utilities.h"								// vShowActivity

#include	<string.h>

// ################################ Global/Local Debug macros ######################################

#define	debugFLAG					0xF003

#define	debugCONFIG					(debugFLAG & 0x0001)
#define	debugSCANNER				(debugFLAG & 0x0002)
#define	debugMAPPING				(debugFLAG & 0x0004)
#define	debugDS18X20				(debugFLAG & 0x0008)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ###################################### General macros ###########################################


// ################################# Platform related variables ####################################

owbi_t * psaOWBI = NULL ;
ow_flags_t	OWflags ;

static uint8_t	OWP_NumBus = 0 ;
static uint8_t	OWP_NumDev = 0 ;

// ################################# Application support functions #################################

owbi_t * psOWP_BusGetPointer(uint8_t LogBus) {
	IF_myASSERT(debugPARAM, halCONFIG_inSRAM(psaOWBI) && (LogBus < OWP_NumBus)) ;
	return &psaOWBI[LogBus] ;
}

/**
 * @brief	Map LOGICAL (platform) bus to PHYSICAL (device) bus
 * @param	psOW - 1W device structure to be updated
 * @param	LogBus
 * @note	Physical device & bus info returned in the psOW structure
 */
void OWP_BusL2P(owdi_t * psOW, uint8_t LogBus) {
	IF_myASSERT(debugPARAM, halCONFIG_inSRAM(psOW) && (LogBus < OWP_NumBus)) ;
	memset(psOW, 0, sizeof(owdi_t)) ;
#if		(halHAS_DS248X > 0)
	for (int i = 0; i < ds248xCount; ++i) {
		ds248x_t * psDS248X = &psaDS248X[i] ;
		IF_TRACK(debugMAPPING, "Read: Ch=%d  Idx=%d  N=%d  L=%d  H=%d\n", LogBus, i, psDS248X->NumChan, psDS248X->Lo, psDS248X->Hi) ;
		if (psDS248X->NumChan && INRANGE(psDS248X->Lo, LogBus, psDS248X->Hi, uint8_t)) {
			psOW->DevNum	= i ;
			psOW->PhyBus	= LogBus - psDS248X->Lo ;
			IF_TRACK(debugMAPPING, "Done: Ch=%d  DN=%d  N=%d  L=%d  H=%d  P=%d\n", LogBus, psOW->DevNum, psDS248X->NumChan, psDS248X->Lo, psDS248X->Hi, psOW->PhyBus) ;
			return ;
		}
	}
#endif
	SL_ERR("Invalid Logical Ch=%d", LogBus) ;
	IF_myASSERT(debugRESULT, 0) ;
}

int	OWP_BusP2L(owdi_t * psOW) {
	IF_myASSERT(debugPARAM, halCONFIG_inSRAM(psaDS248X) && halCONFIG_inSRAM(psOW)) ;
	ds248x_t * psDS248X = &psaDS248X[psOW->DevNum] ;
	return (psDS248X->Lo + psOW->PhyBus) ;
}

/**
 * @brief	Select the physical bus based on the 1W device info
 * @note	NOT an All-In-One function, bus MUST be released after completion
 * @param	psOW
 * @return	1 if selected, 0 if error
 */
int	OWP_BusSelect(owdi_t * psOW) { return ds248xBusSelect(&psaDS248X[psOW->DevNum], psOW->PhyBus); }

void OWP_BusRelease(owdi_t * psOW) { ds248xBusRelease(&psaDS248X[psOW->DevNum]) ; }

// #################################### Handler functions ##########################################

/**
 * @brief	Print the 1-Wire ROM information
 * @param	FlagCount -
 * @param	psOW_ROM - pointer to 1-Wire ROM structure
 * @return	number of characters printed
 */
int	OWP_PrintROM_CB(flagmask_t FlagMask, ow_rom_t * psOW_ROM) {
	int iRV = 0 ;
	if (FlagMask.bRT) iRV += printfx("%!.R: ", RunTime) ;
	if (FlagMask.bCount) iRV += printfx("#%u ", FlagMask.uCount) ;
	iRV += printfx("%02X/%#M/%02X", psOW_ROM->Family, psOW_ROM->TagNum, psOW_ROM->CRC) ;
	if (FlagMask.bNL) iRV += printfx("\n") ;
	return iRV ;
}

int	OWP_Print1W_CB(flagmask_t FlagMask, owdi_t * psOW) {
	int iRV = OWP_PrintROM_CB((flagmask_t) (FlagMask.u32Val & ~mfbNL), &psOW->ROM) ;
	iRV += printfx("  Dev=%d  Log=%d  Phy=%d", psOW->DevNum, OWP_BusP2L(psOW), psOW->PhyBus) ;
	if (FlagMask.bNL) iRV += printfx("\n") ;
	return iRV ;
}

int	OWP_PrintDS18_CB(flagmask_t FlagMask, ds18x20_t * psDS18X20) {
	int iRV = OWP_Print1W_CB((flagmask_t) (FlagMask.u32Val & ~mfbNL), &psDS18X20->sOW) ;
	iRV += printfx(" Traw=0x%04X/%.4fC Tlo=%d Thi=%d", psDS18X20->Tmsb << 8 | psDS18X20->Tlsb,
		psDS18X20->sEWx.var.val.x32.f32, psDS18X20->Tlo, psDS18X20->Thi) ;
	iRV += printfx(" Res=%d PSU=%s", psDS18X20->Res + 9, psDS18X20->Pwr ? "Ext" : "Para") ;
	if (psDS18X20->sOW.ROM.Family == OWFAMILY_28) iRV += printfx(" Conf=0x%02X %s",
		psDS18X20->fam28.Conf, ((psDS18X20->fam28.Conf >> 5) != psDS18X20->Res) ? "ERROR" : "") ;
	if (FlagMask.bNL) iRV += printfx("\n") ;
	return iRV ;
}

int	 OWP_PrintChan_CB(flagmask_t FlagMask, owbi_t * psCI) {
	int iRV = printfx("OW ch=%d  ", FlagMask.uCount) ;
	if (psCI->LastRead) iRV += printfx("%r  ", psCI->LastRead) ;
	if (psCI->LastROM.Family) iRV += OWP_PrintROM_CB((flagmask_t) (FlagMask.u32Val & ~(mfbRT|mfbNL|mfbCOUNT)), &psCI->LastROM) ;
	if (psCI->ds18any) iRV += printfx("  DS18B=%d  DS18S=%d", psCI->ds18b20, psCI->ds18s20) ;
	if (FlagMask.bNL) iRV += printfx("\n") ;
	return iRV ;
}

/**
 * @brief
 * @param	FlagCount
 * @param	psOW
 * @return
 */
int	OWP_Count_CB(flagmask_t FlagCount, owdi_t * psOW) {
	switch (psOW->ROM.Family) {
#if		(halHAS_DS1990X > 0)							// DS1990A/R, 2401/11 devices
	case OWFAMILY_01:	++Family01Count ;	return 1 ;
#endif

#if		(halHAS_DS18X20 > 0)							// DS18x20 Thermometers
	case OWFAMILY_10:	++Fam10Count ;		return 1 ;
	case OWFAMILY_28:	++Fam28Count ;		return 1 ;
#endif

	default: SL_ERR("Invalid/unsupported OW device FAM=%02x", psOW->ROM.Family) ;
	}
	return 0 ;
}

int	OWP_ScanAlarms_CB(flagmask_t sFM, owdi_t * psOW) {
	sFM.bNL	= 1 ;
	sFM.bRT	= 1 ;
	OWP_Print1W_CB(sFM, psOW) ;
	return 1 ;
}

// ################################### Common Scanner functions ####################################

/**
 * @brief	Scan ALL channels sequentially for [specified] family
 * @param	Family
 * @param	Handler
 * @param	psOW
 * @return	number of matching ROM's found (>= 0) or an error code (< 0)
 */
int	OWP_Scan(uint8_t Family, int (* Handler)(flagmask_t, owdi_t *)) {
	IF_myASSERT(debugPARAM, halCONFIG_inFLASH(Handler)) ;
	int	iRV = erSUCCESS ;
	uint32_t uCount = 0 ;
	owdi_t sOW ;
	for (int LogBus = 0; LogBus < OWP_NumBus; ++LogBus) {
		vShowActivity(1) ;
		OWP_BusL2P(&sOW, LogBus);
		if (OWP_BusSelect(&sOW)) {
			if (Family != 0) {
				OWTargetSetup(&sOW, Family) ;
				iRV = OWSearch(&sOW, 0) ;
				if (iRV > 0 && sOW.ROM.Family != Family) {
					// Strictly speaking should never get here, iRV must be 0 if same family not found
					IF_TRACK(debugSCANNER, "Family 0x%02X wanted, 0x%02X found\n", Family, sOW.ROM.Family) ;
					OWP_BusRelease(&sOW) ;
					continue ;
				}
			} else {
				iRV = OWFirst(&sOW, 0) ;
			}
			while (iRV) {
				IF_EXEC_2(debugSCANNER, OWP_Print1W_CB, makeMASKFLAG(0,1,0,0,0,0,0,0,0,0,0,0,LogBus), &sOW) ;
				iRV = OWCheckCRC(sOW.ROM.HexChars, sizeof(ow_rom_t)) ;
				IF_myASSERT(debugRESULT, iRV == 1) ;
				iRV = Handler((flagmask_t) uCount, &sOW) ;
				if (iRV < erSUCCESS) break ;
				if (iRV > 0) ++uCount ;
				iRV = OWNext(&sOW, 0) ;						// try to find next device (if any)
			}
			OWP_BusRelease(&sOW) ;
			if (iRV < erSUCCESS) break ;
		}
	}
	IF_SL_ERR(iRV < erSUCCESS, "Handler error=%d", iRV) ;
	return iRV < erSUCCESS ? iRV : uCount ;
}

int	OWP_Scan2(uint8_t Family, int (* Handler)(flagmask_t, void *, owdi_t *), void * pVoid, owdi_t * psOW) {
	IF_myASSERT(debugPARAM, halCONFIG_inFLASH(Handler)) ;
	int	iRV = erSUCCESS ;
	uint32_t uCount = 0 ;
	for (uint8_t LogBus = 0; LogBus < OWP_NumBus; ++LogBus) {
		OWP_BusL2P(psOW, LogBus) ;
		if (OWP_BusSelect(psOW) == 0) continue ;
		if (Family) {
			OWTargetSetup(psOW, Family) ;
			iRV = OWSearch(psOW, 0) ;
			if (iRV > 0 && psOW->ROM.Family != Family) {
				IF_TRACK(debugSCANNER, "Family 0x%02X wanted, 0x%02X found\n", Family, psOW->ROM.Family) ;
				OWP_BusRelease(psOW) ;
				continue ;
			}
		} else iRV = OWFirst(psOW, 0) ;
		while (iRV) {
			iRV = OWCheckCRC(psOW->ROM.HexChars, sizeof(ow_rom_t)) ;
			IF_myASSERT(debugRESULT, iRV == 1) ;
			iRV = Handler((flagmask_t) uCount, pVoid, psOW) ;
			if (iRV < erSUCCESS)  break ;
			if (iRV > 0) ++uCount ;
			iRV = OWNext(psOW, 0) ;						// try to find next device (if any)
		}
		OWP_BusRelease(psOW) ;
		if (iRV < erSUCCESS) break ;
	}
	IF_SL_ERR(iRV < erSUCCESS, "Handler error=%d", iRV) ;
	return iRV < erSUCCESS ? iRV : uCount ;
}

int	OWP_ScanAlarmsFamily(uint8_t Family) { return OWP_Scan(Family, OWP_ScanAlarms_CB); }

// ################### Identification, Diagnostics & Configuration functions #######################

/**
 * @brief
 * @return
 */
int	OWP_Config(void) {
	IF_SYSTIMER_INIT(debugTIMING, stOW1, stMICROS, "OW1", 100, 1000) ;
	IF_SYSTIMER_INIT(debugTIMING, stOW2, stMICROS, "OW2", 100, 1000) ;
	/* Start by iterating over each instance of each type of 1-Wire technology (DS248x/RTM/GPIO) supported.
	 * For each technology enumerate each physical device and the logical channels on each device before
	 * moving on to the next device (same type) or next technology */
#if		(halHAS_DS248X > 0)
	for (int i = 0; i < ds248xCount; ++i) {
		ds248x_t * psDS248X = &psaDS248X[i] ;
		psDS248X->Lo = OWP_NumBus ;
		psDS248X->Hi = OWP_NumBus + (psDS248X->NumChan ? 7 : 0);
		OWP_NumBus	+= (psDS248X->NumChan ? 8 : 1) ;
	}
#endif

	// When all technologies & devices individually enumerated
	if (OWP_NumBus) {
		psaOWBI = malloc(OWP_NumBus * sizeof(owbi_t)) ;	// initialize the logical channel structures
		memset(psaOWBI, 0, OWP_NumBus * sizeof(owbi_t)) ;
		// enumerate any/all physical devices (possibly) (permanently) attached to individual channel(s)
		int	iRV = OWP_Scan(0, OWP_Count_CB) ;
		if (iRV > 0) OWP_NumDev += iRV ;

#if		(halHAS_DS18X20 > 0)
		if (Fam10Count || Fam28Count) ds18x20Enumerate(); // enumerate & config individually
#endif

#if		(halHAS_DS1990X > 0)
		ds1990xConfig() ;								// cannot enumerate, simple config
#endif
	}
	return OWP_NumDev ;
}

void OWP_Report(void) {
#if 	(halHAS_DS248X > 0)
	ds248xReportAll(1) ;
#endif
#if 	(halHAS_DS18X20 > 0)
	ds18x20ReportAll() ;
#endif
	for (int LogBus = 0; LogBus < OWP_NumBus; ++LogBus)
		OWP_PrintChan_CB(makeMASKFLAG(0,1,0,0,0,0,0,0,0,0,0,0,LogBus), &psaOWBI[LogBus]) ;
}

// ###################################### DS18X20 support ##########################################

epw_t * ds18x20GetWork(int32_t x) ;
void ds18x20SetDefault(epw_t * psEWP, epw_t *psEWS) ;
void ds18x20SetSense(epw_t * psEWP, epw_t * psEWS) ;
float ds18x20GetTemperature(epw_t * psEWx) ;

const vt_enum_t	sDS18X20Func = {
	.work	= ds18x20GetWork,
	.reset	= ds18x20SetDefault,
	.sense	= ds18x20SetSense,
	.get	= ds18x20GetTemperature,
};

epw_t * ds18x20GetWork(int32_t x) {
	IF_myASSERT(debugPARAM, halCONFIG_inSRAM(psaDS18X20) && x < Fam10_28Count) ;
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
	if (psEWS->Tsns < ds18x20T_SNS_MIN)	psEWS->Tsns = ds18x20T_SNS_MIN ;	// no, default to minimum
	if (psEWS->Tsns < psEWP->Tsns) psEWP->Tsns = psEWS->Tsns ;	// set lowest of EWP/EWS
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
	psEWS->var.def.cv.vf	= vfFXX;
	psEWS->var.def.cv.vt	= vtVALUE;
	psEWS->var.def.cv.vs	= vs32B;
	psEWS->var.def.cv.vc	= 1;
	psEWS->idx	= sFM.uCount;
	psEWS->uri	= URI_DS18X20;
	ds18x20Initialize(psDS18X20);

	owbi_t * psOW_CI = psOWP_BusGetPointer(OWP_BusP2L(psOW));
	switch(psOW->ROM.Family) {
	case OWFAMILY_10: psOW_CI->ds18s20++;	break;
	case OWFAMILY_28: psOW_CI->ds18b20++;	break;
	default: myASSERT(0);
	}
	return 1 ;											// number of devices enumerated
}

int	ds18x20Enumerate(void) {
	uint8_t	ds18x20NumDev = 0;
	Fam10_28Count = Fam10Count + Fam28Count;
	IF_SL_INFO(debugDS18X20, "DS18x20 found %d devices", Fam10_28Count) ;
	IF_SYSTIMER_INIT(debugTIMING, stDS1820A, stMILLIS, "DS1820A", 10, 1000) ;
	IF_SYSTIMER_INIT(debugTIMING, stDS1820B, stMILLIS, "DS1820B", 1, 10) ;

	// Once-off EWP initialization
	epw_t * psEWP = &table_work[URI_DS18X20];
	IF_myASSERT(debugRESULT, halCONFIG_inSRAM(psEWP));
	psEWP->var.def.cv.pntr	= 1;
	psEWP->var.def.cv.vf	= vfFXX ;
	psEWP->var.def.cv.vs	= vs32B ;
	psEWP->var.def.cv.vt	= vtENUM ;
	psEWP->var.def.cv.vc	= Fam10_28Count ;
	psEWP->var.val.px.pv	= (void *) &sDS18X20Func ;
	psEWP->Tsns				= ds18x20T_SNS_NORM ;	// All channels read in succession
	psEWP->Rsns				= ds18x20T_SNS_NORM ;	// with blocking I2C driver
	psEWP->uri				= URI_DS18X20 ;			// Used in OWPlatformEndpoints()

	psaDS18X20 = malloc(Fam10_28Count * sizeof(ds18x20_t)) ;
	memset(psaDS18X20, 0, Fam10_28Count * sizeof(ds18x20_t)) ;
	IF_myASSERT(debugRESULT, halCONFIG_inSRAM(psaDS18X20)) ;
	int	iRV = 0 ;
	if (Fam10Count) {
		iRV = OWP_Scan(OWFAMILY_10, ds18x20EnumerateCB) ;
		if (iRV > 0) ds18x20NumDev += iRV ;
	}
	if (Fam28Count) {
		iRV = OWP_Scan(OWFAMILY_28, ds18x20EnumerateCB) ;
		if (iRV > 0) ds18x20NumDev += iRV ;
	}
	if (ds18x20NumDev == Fam10_28Count) iRV = ds18x20NumDev ;
	else {
		SL_ERR("Only %d of %d enumerated!!!", ds18x20NumDev, Fam10_28Count) ;
		iRV = erFAILURE ;
	}
	return iRV ;										// number of devices enumerated
}

TickType_t OWP_DS18X20CalcDelay(ds18x20_t * psDS18X20, bool All) {
	TickType_t tConvert = pdMS_TO_TICKS(ds18x20DELAY_CONVERT) ;
	/* ONLY decrease delay if:
	 * 	specific ROM is addressed AND and it is DS18B20 ; OR
	 * 	ROM match skipped AND only DS18B20 devices on the bus */
	owbi_t * psOWBI = psOWP_BusGetPointer(OWP_BusP2L(&psDS18X20->sOW)) ;
	if ((All && (psOWBI->ds18s20 == 0))
	|| ((All == 0) && (psDS18X20->sOW.ROM.Family == OWFAMILY_28))) {
		tConvert /= (4 - psDS18X20->Res) ;
	}
	return tConvert ;
}

/**
 * @brief	Trigger convert (bus at a time) then read SP, normalise RAW value & persist in EPW
 * @param 	psEPW
 * @return
 */
int	OWP_DS18X20Ai1(epw_t * psEWP) {
	uint8_t	PrevBus = 0xFF ;
	for (int i = 0; i < Fam10_28Count; ++i) {
		ds18x20_t * psDS18X20 = &psaDS18X20[i] ;
		if (psDS18X20->sOW.PhyBus != PrevBus) {
			if (OWP_BusSelect(&psDS18X20->sOW) == 0) continue ;
			if (OWResetCommand(&psDS18X20->sOW, DS18X20_CONVERT, owADDR_SKIP, 1) == 1) {
				PrevBus = psDS18X20->sOW.PhyBus ;
				vTaskDelay(OWP_DS18X20CalcDelay(psDS18X20, 1)) ;
				OWLevel(&psDS18X20->sOW, owPOWER_STANDARD) ;
				OWP_BusRelease(&psDS18X20->sOW) ;		// keep locked for period of delay
			}
		}
		if ((OWP_BusSelect(&psDS18X20->sOW) == 1)
		&& (ds18x20ReadSP(psDS18X20, 2) == 1)) {
			ds18x20ConvertTemperature(psDS18X20) ;
			OWP_BusRelease(&psDS18X20->sOW) ;
		} else SL_ERR("Read/Convert failed") ;
	}
	return erSUCCESS ;
}

int	OWP_DS18X20StartBus(ds18x20_t * psDS18X20, int i) {
	if (OWP_BusSelect(&psDS18X20->sOW) == 1) {
		OWResetCommand(&psDS18X20->sOW, DS18X20_CONVERT, owADDR_SKIP, 1);
		vTimerSetTimerID(psaDS248X[psDS18X20->sOW.DevNum].tmr, (void *) i);
		xTimerStart(psaDS248X[psDS18X20->sOW.DevNum].tmr, OWP_DS18X20CalcDelay(psDS18X20, 1));
		IF_TRACK(debugDS18X20, "Start Dev=%d Bus=%d", psDS18X20->sOW.DevNum, psDS18X20->sOW.PhyBus);
		return 1 ;
	}
	SL_ERR("Failed to start convert Dev=%d Bus=%d", psDS18X20->sOW.DevNum, psDS18X20->sOW.PhyBus) ;
	return 0 ;
}

int OWP_DS18X20StartSample(epw_t * psEWx) {				// Stage 1 -
	uint8_t	PrevDev = 0xFF ;
	for (int i = 0; i < Fam10_28Count; ++i) {
		ds18x20_t * psDS18X20 = &psaDS18X20[i] ;
		if (psDS18X20->sOW.DevNum != PrevDev) {
			if (OWP_DS18X20StartBus(psDS18X20, i) == 1) PrevDev = psDS18X20->sOW.DevNum ;
		}
	}
	return erSUCCESS;
}

void OWP_DS18X20ReadSample(TimerHandle_t pxHandle) {
	int	ThisDev = (int) pvTimerGetTimerID(pxHandle) ;
	ds18x20_t * psDS18X20 = &psaDS18X20[ThisDev] ;
	OWLevel(&psDS18X20->sOW, owPOWER_STANDARD) ;		// Set OWLevel to standard
	int i = ThisDev ;
	do {												// Handle all sensors on this BUS
		psDS18X20 = &psaDS18X20[i] ;
		OWAddress(&psDS18X20->sOW, OW_CMD_MATCHROM) ;
		if (ds18x20ReadSP(psDS18X20, 2) == 1) ds18x20ConvertTemperature(psDS18X20) ;
		else SL_ERR("Read/Convert failed") ;
		++i ;
		// no more sensors or different device - release bus, exit loop
		if ((i == Fam10_28Count)
		|| (psDS18X20->sOW.DevNum != psaDS18X20[i].sOW.DevNum)) {
			OWP_BusRelease(&psDS18X20->sOW) ;
			break ;
		}
		// more sensors, same device, new bus - release bus, start convert on new bus.
		if (psDS18X20->sOW.PhyBus != psaDS18X20[i].sOW.PhyBus) {
			OWP_BusRelease(&psDS18X20->sOW) ;
			OWP_DS18X20StartBus(&psaDS18X20[i], i) ;
			break ;
		}
		// more sensors, same device and bus
	} while  (i < Fam10_28Count) ;
}

// ###################################### DS1990X support ##########################################

/* To avoid registering multiple reads if iButton is held in place too long we enforce a
 * period of 'x' seconds within which successive reads of the same tag will be ignored */
int	OWP_DS1990ScanCB(flagmask_t sFM, owdi_t * psOW) {
	seconds_t	NowRead = xTimeStampAsSeconds(sTSZ.usecs) ;
	uint8_t		LogChan = OWP_BusP2L(psOW) ;
	owbi_t * psOW_CI = psOWP_BusGetPointer(LogChan) ;
	++Family01Count ;
	if ((psOW_CI->LastROM.Value == psOW->ROM.Value)
	&& ((NowRead - psOW_CI->LastRead) <= ds1990ReadIntvl)) {
		IF_PRINT(debugTRACK, "SAME iButton in %d sec, Skipped...\n", ds1990ReadIntvl) ;
		return erSUCCESS ;
	}
	psOW_CI->LastROM.Value	= psOW->ROM.Value ;
	psOW_CI->LastRead		= NowRead ;
	xTaskNotify(EventsHandle, 1UL << (LogChan + evtFIRST_OW), eSetBits) ;
	portYIELD() ;
#if		(debugEVENTS)
	sFM.bRT	= 1 ;
	sFM.bNL	= 1 ;
	OWP_Print1W_CB(sFM, psOW) ;
#endif
	return erSUCCESS ;
}

int	OWP_DS1990ScanAll(epw_t * psEWP) {
	vShowActivity(0) ;
	Family01Count = 0;
	return OWP_Scan(OWFAMILY_01, OWP_DS1990ScanCB) ;
}

