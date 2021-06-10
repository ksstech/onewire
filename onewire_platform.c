/*
 * Copyright 2020-2021 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

/*
 * onewire_platform.c
 */

#include	"endpoint_struct.h"
#include	"endpoint_id.h"
#include	"printfx.h"
#include	"syslog.h"
#include	"systiming.h"								// timing debugging
#include	"x_errors_events.h"

#include	"onewire_platform.h"
#include	"task_events.h"

#include	<string.h>

// ################################ Global/Local Debug macros ######################################

#define	debugFLAG					0xF000

#define	debugCONFIG					(debugFLAG & 0x0001)
#define	debugSCANNER				(debugFLAG & 0x0002)
#define	debugMAPPING				(debugFLAG & 0x0004)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ###################################### General macros ###########################################


// ################################# Platform related variables ####################################

ow_chan_info_t * psaOW_CI = NULL ;					// Array of last read ROM & timestamp info
ow_flags_t	OWflags ;

static const char * const OWBusType[] = { "DS248x", "RTM" "GPIO" } ;
static uint8_t	OWNumChan = 0 ;
static uint8_t	OWNumDev = 0 ;						// total ALL types, counted but not yet used

// ################################# Application support functions #################################

/* Support to map LOGICAL (platform) channel to PHYSICAL (device) channel
 * Define a sequence in which LOGICAL channels will be allocated:
 * 	1.	248x	1/8 channels based on device(s) detected
 * 	2.	RCT 	depending on config
 * 	3.	GPIO	depending on config
 */
void	OWPlatformChanLog2Phy(onewire_t * psOW, uint8_t Chan) {
	IF_myASSERT(debugPARAM, halCONFIG_inSRAM(psOW) && (Chan < OWNumChan)) ;
	memset(psOW, 0, sizeof(onewire_t)) ;
#if		(halHAS_DS248X > 0)
	for (int i = 0; i < ds248xCount; ++i) {
		ds248x_t * psDS248X = &psaDS248X[i] ;
		IF_TRACK(debugMAPPING, "Read: Ch=%d  Idx=%d  N=%d  L=%d  H=%d\n", Chan, i, psDS248X->NumChan, psDS248X->Lo, psDS248X->Hi) ;
		if (psDS248X->NumChan && INRANGE(psDS248X->Lo, Chan, psDS248X->Hi, uint8_t)) {
			psOW->BusType	= owTYPE_DS248X ;
			psOW->DevNum	= i ;
			psOW->PhyChan	= Chan - psDS248X->Lo ;
			IF_TRACK(debugMAPPING, "Done: Ch=%d  DN=%d  N=%d  L=%d  H=%d  P=%d\n", Chan, psOW->DevNum, psDS248X->NumChan, psDS248X->Lo, psDS248X->Hi, psOW->PhyChan) ;
			return ;
		}
	}
#endif
	SL_ERR("Invalid Logical Ch=%d", Chan) ;
	IF_myASSERT(debugRESULT, 0) ;
}

int32_t	OWPlatformChanPhy2Log(onewire_t * psOW) {
	IF_myASSERT(debugPARAM, halCONFIG_inSRAM(psaDS248X) && halCONFIG_inSRAM(psOW)) ;
	ds248x_t * psDS248X = &psaDS248X[psOW->DevNum] ;
	return (psDS248X->Lo + psOW->PhyChan) ;
}

ow_chan_info_t * psOWPlatformGetInfoPointer(uint8_t LogChan) {
	IF_myASSERT(debugPARAM, halCONFIG_inSRAM(psaOW_CI) && (LogChan < OWNumChan)) ;
	return &psaOW_CI[LogChan] ;
}

// #################################### Handler functions ##########################################

/**
 * OWPlatformCB_PrintROM() - print the 1-Wire ROM information
 * @param	FlagCount -
 * @param	psOW_ROM - pointer to 1-Wire ROM structure
 * @return	number of characters printed
 */
int32_t	OWPlatformCB_PrintROM(flagmask_t FlagMask, ow_rom_t * psOW_ROM) {
	int32_t iRV = 0 ;
	if (FlagMask.bRT) {
		iRV += printfx("%!.R: ", RunTime) ;
	}
	if (FlagMask.bCount) {
		iRV += printfx("#%u ", FlagMask.uCount) ;
	}
	iRV += printfx("%02X/%#M/%02X", psOW_ROM->Family, psOW_ROM->TagNum, psOW_ROM->CRC) ;
	if (FlagMask.bNL) {
		iRV += printfx("\n") ;
	}
	return iRV ;
}

int32_t	OWPlatformCB_Print1W(flagmask_t FlagMask, onewire_t * psOW) {
	int32_t iRV = OWPlatformCB_PrintROM((flagmask_t) (FlagMask.u32Val & ~mfbNL), &psOW->ROM) ;
	iRV += printfx("  Log=%d  Type=%s[%d]  Phy=%d", OWPlatformChanPhy2Log(psOW), OWBusType[psOW->BusType], psOW->DevNum, psOW->PhyChan) ;
	if (FlagMask.bNL) {
		iRV += printfx("\n") ;
	}
	return iRV ;
}

int32_t	OWPlatformCB_PrintDS18(flagmask_t FlagMask, ds18x20_t * psDS18X20) {
	int32_t iRV = OWPlatformCB_Print1W((flagmask_t) (FlagMask.u32Val & ~mfbNL), &psDS18X20->sOW) ;
	iRV += printfx("  Traw=0x%04X/%.4fC  Tlo=%d  Thi=%d", psDS18X20->Tmsb << 8 | psDS18X20->Tlsb,
		psDS18X20->sEWx.var.val.x32.f32, psDS18X20->Tlo, psDS18X20->Thi) ;
	iRV += printfx("  Res=%d  PSU=%s", psDS18X20->Res + 9, psDS18X20->Pwr ? "Ext" : "Para") ;
	if (psDS18X20->sOW.ROM.Family == OWFAMILY_28) {
		iRV += printfx("  Conf=0x%02X %s", psDS18X20->fam28.Conf,
				((psDS18X20->fam28.Conf >> 5) != psDS18X20->Res) ? "ERROR" : "") ; ;
	}
	if (FlagMask.bNL) {
		iRV += printfx("\n") ;
	}
	return iRV ;
}

int32_t OWPlatformCB_PrintChan(flagmask_t FlagMask, ow_chan_info_t * psCI) {
	int32_t iRV = printfx("OW ch=%d  ", FlagMask.uCount) ;
	if (psCI->LastRead) {
		iRV += printfx("%r  ", psCI->LastRead) ;
	}
	if (psCI->LastROM.Family) {
		iRV += OWPlatformCB_PrintROM((flagmask_t) (FlagMask.u32Val & ~(mfbRT|mfbNL|mfbCOUNT)), &psCI->LastROM) ;
	}
	if (psCI->ds18any) {
		iRV += printfx("  DS18B=%d  DS18S=%d  DS18X=%d", psCI->ds18b20, psCI->ds18s20, psCI->ds18xxx) ;
	}
	if (FlagMask.bNL) {
		iRV += printfx("\n") ;
	}
	return iRV ;
}

/**
 * OWPlatformCB_Count() - Call handler based on device family
 * @return	return value from handler or
 */
int32_t	OWPlatformCB_Count(flagmask_t FlagCount, onewire_t * psOW) {
	switch (psOW->ROM.Family) {
#if		(halHAS_DS1990X == 1)							// DS1990A/R, 2401/11 devices
	case OWFAMILY_01:
		++Family01Count ;
		return 1 ;
#endif

#if		(halHAS_DS18X20 == 1)							// DS18x20 Thermometers
	case OWFAMILY_10:
	case OWFAMILY_28:
		++Fam10_28Count ;
		return 1 ;
#endif

	default:
		SL_ERR("Invalid/unsupported OW device FAM=%02x", psOW->ROM.Family) ;
	}
	return 0 ;
}

// ################################### Common Scanner functions ####################################

/**
 * OWPlatformScanner() - scan ALL channels sequentially for [specified] family
 * @return	number of matching ROM's found (>= 0) or an error code (< 0)
 */
int32_t	OWPlatformScanner(uint8_t Family, int (* Handler)(flagmask_t, onewire_t *), onewire_t * psOW) {
	IF_myASSERT(debugPARAM, halCONFIG_inFLASH(Handler)) ;
	int32_t	iRV = erSUCCESS ;
	uint32_t uCount = 0 ;
	for (uint8_t OWChan = 0; OWChan < OWNumChan; ++OWChan) {
		OWPlatformChanLog2Phy(psOW, OWChan) ;
		if (OWChannelSelect(psOW) == 0) {
			continue ;
		}
		if (Family != 0) {
			OWTargetSetup(psOW, Family) ;
			iRV = OWSearch(psOW, 0) ;
			if (psOW->ROM.Family != Family) {
				IF_TRACK(debugSCANNER, "Family 0x%02X wanted, 0x%02X found\n", Family, psOW->ROM.Family) ;
				continue ;
			}
		} else {
			iRV = OWFirst(psOW, 0) ;
		}
		while (iRV) {
			IF_EXEC_2(debugSCANNER, OWPlatformCB_Print1W, makeMASKFLAG(0,0,0,0,0,0,0,0,0,0,0,0,OWChan), psOW) ;
			iRV = OWCheckCRC(psOW->ROM.HexChars, sizeof(ow_rom_t)) ;
			IF_myASSERT(debugRESULT, iRV == 1) ;
			iRV = Handler((flagmask_t) uCount, psOW) ;
			if (iRV < erSUCCESS) {
				break ;
			}
			if (iRV > 0) {
				++uCount ;
			}
			iRV = OWNext(psOW, 0) ;						// try to find next device (if any)
		}
		if (iRV < erSUCCESS) {
			break ;
		}
	}
	IF_SL_ERR(iRV < erSUCCESS, "Handler error=%d", iRV) ;
	return iRV < erSUCCESS ? iRV : uCount ;
}

#if 0
int32_t	OWPlatformScan(uint8_t Family, int (* Handler)(flagmask_t, void *, onewire_t *), void * pVoid, onewire_t * psOW) {
	IF_myASSERT(debugPARAM, halCONFIG_inFLASH(Handler)) ;
	int32_t	iRV = erSUCCESS ;
	uint32_t uCount = 0 ;
	for (uint8_t OWChan = 0; OWChan < OWNumChan; ++OWChan) {
		OWPlatformChanLog2Phy(psOW, OWChan) ;
		if (OWChannelSelect(psOW) == 0)	{
			IF_SL_INFO(debugSCANNER, "Channel selection error") ;
			break ;
		}
		if (Family) {
			OWTargetSetup(psOW, Family) ;
			iRV = OWSearch(psOW, 0) ;
			if (psOW->ROM.Family != Family) {
				IF_SL_INFO(debugSCANNER, "Family 0x%02X wanted, 0x%02X found", Family, psOW->ROM.Family) ;
				continue ;
			}
		} else {
			iRV = OWFirst(psOW, 0) ;
		}
		while (iRV) {
			iRV = OWCheckCRC(psOW->ROM.HexChars, sizeof(ow_rom_t)) ;
			IF_myASSERT(debugRESULT, iRV == 1) ;
			iRV = Handler((flagmask_t) uCount, pVoid, psOW) ;
			if (iRV < erSUCCESS) {
				break ;
			}
			if (iRV > 0) {
				++uCount ;
			}
			iRV = OWNext(psOW, 0) ;						// try to find next device (if any)
		}
		if (iRV < erSUCCESS) {
			break ;
		}
	}
	IF_SL_ERR(iRV < erSUCCESS, "Handler error=%d", iRV) ;
	return iRV < erSUCCESS ? iRV : uCount ;
}
#endif

// ################### Identification, Diagnostics & Configuration functions #######################

/**
 * OWPlatformConfig() -
 */
int32_t	OWPlatformConfig(void) {
	IF_SYSTIMER_INIT(debugTIMING, systimerOW1, systimerTICKS, "OW1", myMS_TO_TICKS(10), myMS_TO_TICKS(1000)) ;
	IF_SYSTIMER_INIT(debugTIMING, systimerOW2, systimerTICKS, "OW2", myMS_TO_TICKS(10), myMS_TO_TICKS(1000)) ;
	/* Start by iterating over each instance of each type of 1-Wire technology (DS248x/RTM/GPIO) supported.
	 * For each technology enumerate each physical device and the logical channels on each device before
	 * moving on to the next device (same type) or next technology */
#if		(halHAS_DS248X > 0)
	for (int i = 0; i < ds248xCount; ++i) {
		ds248x_t * psDS248X = &psaDS248X[i] ;
		psDS248X->Lo	= OWNumChan ;
		psDS248X->Hi	= OWNumChan + psDS248X->NumChan - 1 ;
		OWNumChan		+= psDS248X->NumChan ;
	}
#endif

	// When all technologies & devices individually enumerated
	if (OWNumChan) {
		// initialize the logical channel structures
		psaOW_CI = malloc(OWNumChan * sizeof(ow_chan_info_t)) ;
		memset(psaOW_CI, 0, OWNumChan * sizeof(ow_chan_info_t)) ;

		// enumerate any/all physical devices (possibly) (permanently) attached to individual channel(s)
		onewire_t	sOW ;
		int32_t	iRV = 0 ;
		if ((iRV = OWPlatformScanner(0, OWPlatformCB_Count, &sOW)) > 0) {
			OWNumDev += iRV ;
		}
#if		(halHAS_DS1990X > 0)
		IF_SL_INFO(debugCONFIG && Family01Count, "DS1990x found %d devices", Family01Count) ;
		iRV = ds1990xConfig() ;				// cannot enumerate, simple config
		IF_SYSTIMER_INIT(debugTIMING, systimerDS1990, systimerTICKS, "DS1990", myMS_TO_TICKS(10), myMS_TO_TICKS(1000)) ;
#endif

#if		(halHAS_DS18X20 > 0)
		if (Fam10_28Count) {
			IF_SL_INFO(debugCONFIG, "DS18x20 found %d devices", Fam10_28Count) ;
			iRV = ds18x20Enumerate() ;		// enumerate & config individually
		}
		IF_SYSTIMER_INIT(debugTIMING, systimerDS1820A, systimerTICKS, "DS1820A", myMS_TO_TICKS(10), myMS_TO_TICKS(1000)) ;
		IF_SYSTIMER_INIT(debugTIMING, systimerDS1820B, systimerTICKS, "DS1820B", myMS_TO_TICKS(1), myMS_TO_TICKS(10)) ;
#endif
	}
	return OWNumDev ;
}

void	OWPlatformReportAll(void) {
	for (int OWChan = 0; OWChan < OWNumChan; ++OWChan) {
		OWPlatformCB_PrintChan(makeMASKFLAG(0,1,0,0,0,0,0,0,0,0,0,0,OWChan), &psaOW_CI[OWChan]) ;
	}
#if 	(halHAS_DS248X > 0)
	ds248xReportAll(1) ;
#endif
#if 	(halHAS_DS18X20 > 0)
	ds18x20ReportAll() ;
#endif
}
