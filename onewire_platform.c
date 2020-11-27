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
 * onewire_platform.c
 */

#include	"endpoint_id.h"
#include	"endpoint_struct.h"
#include	"printfx.h"
#include	"syslog.h"
#include	"systiming.h"								// timing debugging
#include	"x_errors_events.h"

#include	"onewire_platform.h"
#include	"task_events.h"

#include	"hal_debug.h"

#include	<string.h>

#define	debugFLAG					0xD000

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ###################################### General macros ###########################################


// ################################# Platform related variables ####################################

const char * const OWBusType[] = { "DS248x", "RTM" "GPIO" } ;
ow_chan_info_t * psaOW_CI = NULL ;					// Array of last read ROM & timestamp info
uint8_t		OWNumChan = 0, OWNumDev = 0 ;
ow_flags_t	OWflags ;

// ################################# Application support functions #################################

/* Support to map LOGICAL (platform) channel to PHYSICAL (device) channel
 * Define a sequence in which LOGICAL channels will be allocated:
 * 	1.	248x	1/8 channels based on device(s) detected
 * 	2.	RCT 	depending on config
 * 	3.	GPIO	depending on config
 */
int32_t	OWPlatformChanLog2Phy(onewire_t * psOW, uint8_t Chan) {
	IF_myASSERT(debugPARAM, INRANGE_SRAM(psOW) && (Chan < OWNumChan)) ;
	memset(psOW, 0, sizeof(onewire_t)) ;
#if		(halHAS_DS248X > 0)
	for (int32_t i = 0; i < ds248xCount; ++i) {
		ds248x_t * psDS248X = &psaDS248X[i] ;
//		TRACK("Read: C=%d  Did=%d  N=%d  L=%d  H=%d", Chan, i, psDS248X->NumChan, psDS248X->Lo, psDS248X->Hi) ;
		if (psDS248X->NumChan && INRANGE(psDS248X->Lo, Chan, psDS248X->Hi, uint8_t)) {
			psOW->BusType	= owTYPE_DS248X ;
			psOW->DevNum	= i ;
			psOW->PhyChan	= Chan - psDS248X->Lo ;
//			TRACK("Done: Ch=%d  DN=%d  N=%d  L=%d  H=%d  P=%d", Chan, psOW->DevNum, psDS248X->NumChan, psDS248X->Lo, psDS248X->Hi, psOW->PhyChan) ;
			return true ;
		}
	}
#endif
	IF_myASSERT(debugRESULT, 0) ;
	return false ;
}

int32_t	OWPlatformChanPhy2Log(onewire_t * psOW) {
	IF_myASSERT(debugPARAM, INRANGE_SRAM(psaDS248X) && INRANGE_SRAM(psOW)) ;
	ds248x_t * psDS248X = &psaDS248X[psOW->DevNum] ;
	return (psDS248X->Lo + psOW->PhyChan) ;
}

ow_chan_info_t * psOWPlatformGetInfoPointer(uint8_t LogChan) {
	IF_myASSERT(debugPARAM, INRANGE_SRAM(psaOW_CI) && LogChan < OWNumChan) ;
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
	if (FlagMask.bRT)
		iRV += printfx_nolock("%!.R: ", RunTime) ;
	if (FlagMask.bCount)
		iRV += printfx_nolock("#%u ", FlagMask.uCount) ;
	iRV += printfx_nolock("%02X/%#M/%02X", psOW_ROM->Family, psOW_ROM->TagNum, psOW_ROM->CRC) ;
	if (FlagMask.bNL)
		iRV += printfx_nolock("\n") ;
	return iRV ;
}

int32_t	OWPlatformCB_Print1W(flagmask_t FlagMask, onewire_t * psOW) {
	int32_t iRV = OWPlatformCB_PrintROM((flagmask_t) (FlagMask.u32Val & ~mfbNL), &psOW->ROM) ;
	iRV += printfx_nolock("  Log=%d  Type=%s[%d]  Phy=%d", OWPlatformChanPhy2Log(psOW), OWBusType[psOW->BusType], psOW->DevNum, psOW->PhyChan) ;
	if (FlagMask.bNL)
		iRV += printfx_nolock("\n") ;
	return iRV ;
}

int32_t	OWPlatformCB_PrintDS18(flagmask_t FlagMask, ds18x20_t * psDS18X20) {
	int32_t iRV = OWPlatformCB_Print1W((flagmask_t) (FlagMask.u32Val & ~mfbNL), &psDS18X20->sOW) ;
	iRV += printfx_nolock("  Traw=0x%04X (Tc=%.4f) Thi=%d  Tlo=%d",
		psDS18X20->Tmsb << 8 | psDS18X20->Tlsb, psDS18X20->sWork.Var.varVal.x32.f32, psDS18X20->Thi, psDS18X20->Tlo) ;
	iRV += printfx_nolock("  Res=%d", psDS18X20->Res + 9) ;
	if (psDS18X20->sOW.ROM.Family == OWFAMILY_28)
		iRV += printfx_nolock("  Conf=0x%02X %s", psDS18X20->fam28.Conf, psDS18X20->fam28.Conf >> 5 != psDS18X20->Res ? "ERROR" : "") ; ;
	if (FlagMask.bNL)
		iRV += printfx_nolock("\n") ;
	return iRV ;
}

int32_t OWPlatformCB_PrintChan(flagmask_t FlagMask, ow_chan_info_t * psCI) {
	int32_t iRV = printfx_nolock("OW ch=%d  ", FlagMask.uCount) ;
	if (psCI->LastRead)
		iRV += printfx_nolock("%r  ", FlagMask.uCount, psCI->LastRead) ;
	if (psCI->LastROM.Family)
		iRV += OWPlatformCB_PrintROM((flagmask_t) (FlagMask.u32Val & ~(mfbRT|mfbNL|mfbCOUNT)), &psCI->LastROM) ;
	iRV += printfx_nolock("  DS18B=%d  DS18S=%d  DS18X=%d", psCI->ds18b20, psCI->ds18s20, psCI->ds18xxx) ;
	if (FlagMask.bNL)
		iRV += printfx_nolock("\n") ;
	return iRV ;
}

/**
 * OWPlatformCB_Count() - Call handler based on device family
 * @return	return value from handler or
 */
int32_t	OWPlatformCB_Count(flagmask_t FlagCount, onewire_t * psOW) {
	switch (psOW->ROM.Family) {
#if		(halHAS_DS1990X == 1)							// DS1990A/R, 2401/11 devices
	case OWFAMILY_01:	++Family01Count ;	return 1 ;
#endif

#if		(halHAS_DS18X20 == 1)							// DS18x20 Thermometers
	case OWFAMILY_10:
	case OWFAMILY_28:	++Fam10_28Count ;	return 1 ;
#endif

	default:	SL_ERR("Invalid/unsupported OW device FAM=%02x", psOW->ROM.Family) ;
	}
	return 0 ;
}

// ################################### Common Scanner functions ####################################

/**
 * OWPlatformScanner() - scan ALL channels sequentially for [specified] family
 * @return	number of matching ROM's found (>= 0) or an error code (< 0)
 */
int32_t	OWPlatformScanner(uint8_t Family, int (* Handler)(flagmask_t, onewire_t *), onewire_t * psOW) {
	IF_myASSERT(debugPARAM, INRANGE_FLASH(Handler)) ;
	int32_t	iRV = erSUCCESS ;
	uint32_t uCount = 0 ;
	for (uint8_t OWBus = 0; OWBus < OWNumChan; ++OWBus) {
		OWPlatformChanLog2Phy(psOW, OWBus) ;
		if (OWChannelSelect(psOW) == false)	{
			IF_SL_DBG(debugTRACK, "Channel selection error") ;
			break ;
		}
		if (Family) {
			OWTargetSetup(psOW, Family) ;
			iRV = OWSearch(psOW, false) ;
			if (psOW->ROM.Family != Family) {
				IF_SL_DBG(debugTRACK, "Family 0x%02X wanted, 0x%02X found", Family, psOW->ROM.Family) ;
				continue ;
			}
		} else {
			iRV = OWFirst(psOW, false) ;
		}
		while (iRV) {
			iRV = OWCheckCRC(psOW->ROM.HexChars, sizeof(ow_rom_t)) ;
			IF_myASSERT(debugRESULT, iRV == 1) ;
			iRV = Handler((flagmask_t) uCount, psOW) ;
			if (iRV < erSUCCESS) {
				break ;
			}
			if (iRV > 0) {
				++uCount ;
			}
			iRV = OWNext(psOW, false) ;						// try to find next device (if any)
		}
		if (iRV < erSUCCESS) {
			break ;
		}
	}
	IF_SL_ERR(iRV < erSUCCESS, "Handler error=%d", iRV) ;
	return iRV < erSUCCESS ? iRV : uCount ;
}

int32_t OWPlatformEndpoints(struct ep_work_s * psEpWork) {
	int32_t iRV = erFAILURE;
	switch(psEpWork->uri) {
#if		(halHAS_DS18X20 > 0)
	case URI_DS18X20: iRV = ds18x20ReadConvertAll(NULL) ;	break ;
#endif

#if		(halHAS_DS1990X > 0)
	case URI_DS1990X:
	{	onewire_t sOW ;
		Family01Count = 0 ;
		iRV = OWPlatformScanner(OWFAMILY_01, OWPlatformCB_ReadDS1990X, &sOW) ;
		break ;
	}
#endif
	default: SL_ERR("Invalid/Unsupported 1-Wire family (URI=%d)", psEpWork->uri) ; break ;
	}
	return iRV ;
}

// ################### Identification, Diagnostics & Configuration functions #######################

/**
 * OWPlatformConfig() -
 */
int32_t	OWPlatformConfig(void) {
	int32_t	iRV = 0 ;
	IF_SYSTIMER_INIT(debugTIMING, systimerOW1, systimerTICKS, "OW1", myMS_TO_TICKS(10), myMS_TO_TICKS(1000)) ;
	IF_SYSTIMER_INIT(debugTIMING, systimerOW2, systimerTICKS, "OW2", myMS_TO_TICKS(10), myMS_TO_TICKS(1000)) ;
	/* Start by iterating over each instance of each type of 1-Wire technology (DS248x/RTM/GPIO) supported.
	 * For each technology enumerate each physical device and the logical channels on each device before
	 * moving on to the next device (same type) or next technology */
#if		(halHAS_DS248X > 0)
	for (int32_t i = 0; i < ds248xCount; ++i) {
		ds248x_t * psDS248X = &psaDS248X[i] ;
		psDS248X->Lo	= OWNumChan ;
		psDS248X->Hi	= OWNumChan + psDS248X->NumChan -1 ;
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
		if ((iRV = OWPlatformScanner(0, OWPlatformCB_Count, &sOW)) > 0) {
			OWNumDev += iRV ;
		}
#if		(halHAS_DS1990X > 0)
		if (Family01Count) {
			IF_SL_DBG(debugTRACK, "DS1990x found %d devices", Family01Count) ;
		}
		iRV = ds1990xConfig(URI_DS1990X) ;
		IF_SYSTIMER_INIT(debugTIMING, systimerDS1990, systimerTICKS, "DS1990", myMS_TO_TICKS(10), myMS_TO_TICKS(1000)) ;
#endif

#if		(halHAS_DS18X20 > 0)
		if (Fam10_28Count) {
			IF_SL_DBG(debugTRACK, "DS18x20 found %d devices", Fam10_28Count) ;
			iRV = ds18x20Enumerate(URI_DS18X20) ;
		}
		IF_SYSTIMER_INIT(debugTIMING, systimerDS1820A, systimerTICKS, "DS1820A", myMS_TO_TICKS(10), myMS_TO_TICKS(1000)) ;
		IF_SYSTIMER_INIT(debugTIMING, systimerDS1820B, systimerTICKS, "DS1820B", myMS_TO_TICKS(1), myMS_TO_TICKS(10)) ;
#endif
	}
	return OWNumDev ;
}

void	OWPlatformReportAll(void) {
	for (int x = 0; x < OWNumChan; ++x)
		OWPlatformCB_PrintChan(makeMASKFLAG(0,1,0,0,0,0,0,0,0,0,0,0,x), &psaOW_CI[x]) ;
#if 	(halHAS_DS248X > 0)
	ds248xReportAll() ;
#endif
#if 	(halHAS_DS18X20 > 0)
	ds18x20ReportAll() ;
#endif
}
