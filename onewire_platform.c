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

static uint8_t	OWP_NumBus = 0, OWP_NumDev = 0 ;

/* In order to avoid multiple successive reads of the same iButton on the same OW channel
 * we filter reads based on the value of the iButton read and time expired since the last
 * successful read. If the same ID is read on the same channel within 'x' seconds, skip it */

uint8_t	ds1990ReadIntvl	= ds1990READ_INTVL ;
uint8_t	Family01Count = 0 ;

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
#if		(halHAS_DS248X > 0)
	for (int i = 0; i < ds248xCount; ++i) {
		ds248x_t * psDS248X = &psaDS248X[i] ;
		IF_TRACK(debugMAPPING, "Log=%d Dev=%d Lo=%d Hi=%d", LogBus, i, psDS248X->Lo, psDS248X->Hi) ;
		if (INRANGE(psDS248X->Lo, LogBus, psDS248X->Hi, uint8_t)) {
			psOW->DevNum = i;
			psOW->PhyBus = LogBus - psDS248X->Lo;
			IF_TRACK(debugMAPPING, " -> P=%d\n", psOW->PhyBus) ;
			return ;
		}
		IF_TRACK(debugMAPPING, "\n") ;
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
	iRV += printfx(" Log=%d Dev=%d Phy=%d PSU=%d", OWP_BusP2L(psOW), psOW->DevNum, psOW->PhyBus, psOW->PSU);
	if (FlagMask.bNL) iRV += printfx("\n") ;
	return iRV ;
}

int	OWP_PrintDS18_CB(flagmask_t FlagMask, ds18x20_t * psDS18X20) {
	int iRV = OWP_Print1W_CB((flagmask_t) (FlagMask.u32Val & ~mfbNL), &psDS18X20->sOW) ;
	iRV += printfx(" Traw=0x%04X/%.4fC Tlo=%d Thi=%d Res=%d", psDS18X20->Tmsb << 8 | psDS18X20->Tlsb,
		psDS18X20->sEWx.var.val.x32.f32, psDS18X20->Tlo, psDS18X20->Thi, psDS18X20->Res+9) ;
	if (psDS18X20->sOW.ROM.Family == OWFAMILY_28) iRV += printfx(" Conf=x%02X %s",
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
		memset(&sOW, 0, sizeof(owdi_t));
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
				IF_EXEC_2(debugSCANNER, OWP_Print1W_CB, makeMASKFLAG(0,1,0,0,0,0,0,0,0,LogBus), &sOW) ;
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
	if (iRV < erSUCCESS) SL_ERR("Handler error=%d", iRV) ;
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
		psaOWBI = pvRtosMalloc(OWP_NumBus * sizeof(owbi_t)) ;	// initialize the logical channel structures
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
		OWP_PrintChan_CB(makeMASKFLAG(0,1,0,0,0,0,0,0,0,LogBus), &psaOWBI[LogBus]) ;
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

