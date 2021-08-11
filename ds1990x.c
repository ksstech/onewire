/*
 * Copyright 2020-21 Andre M. Maree/KSS Technologies (Pty) Ltd.
 * ds1990x.c
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

#define	debugFLAG					0xE001

#define	debugEVENTS					(debugFLAG & 0x0001)
#define	debugCONFIG					(debugFLAG & 0x0002)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ###################################### General macros ###########################################


// ################################# Platform related variables ####################################

/* In order to avoid multiple successive reads of the same iButton on the same OW channel
 * we filter reads based on the value of the iButton read and time expired since the last
 * successful read. If the same ID is read on the same channel within 'x' seconds, skip it */

uint8_t	Family01Count 	= 0 ;
uint8_t	ds1990ReadIntvl	= ds1990READ_INTVL ;

// ################################# Application support functions #################################

/* To avoid registering multiple reads if iButton is held in place too long we enforce a
 * period of 'x' seconds within which successive reads of the same tag will be ignored */
int32_t	ds1990xDetectCB(flagmask_t sFM, owdi_t * psOW) {
	seconds_t	NowRead = xTimeStampAsSeconds(sTSZ.usecs) ;
	uint8_t		LogChan = OWP_BusP2L(psOW) ;
	owbi_t * psOW_CI = psOWP_BusGetPointer(LogChan) ;
	++Family01Count ;
	if ((psOW_CI->LastROM.Value == psOW->ROM.Value) && (NowRead - psOW_CI->LastRead) <= ds1990ReadIntvl) {
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

int32_t	ds1990xScanAll(epw_t * psEWP) {
	vShowActivity(0) ;
	owdi_t sOW ;
	Family01Count = 0 ;
	return OWP_Scan(OWFAMILY_01, ds1990xDetectCB, &sOW) ;
}

int	ds1990xConfig(void) {
	epw_t * psEWP = &table_work[URI_DS1990X] ;
	IF_myASSERT(debugRESULT, halCONFIG_inSRAM(psEWP)) ;
	psEWP->var.def.cv.vc	= 1 ;
	psEWP->var.def.cv.vf	= vfUXX ;
	psEWP->var.def.cv.vt	= vtVALUE ;
	psEWP->var.def.cv.vs	= vs32B ;
	psEWP->Tsns				= ds1990xT_SNS_NORM ;
	psEWP->Rsns				= ds1990xT_SNS_NORM ;
	psEWP->uri				= URI_DS1990X ;				// Used in OWPlatformEndpoints()
	IF_SYSTIMER_INIT(debugTIMING, stDS1990, stMILLIS, "DS1990", 10, 1000) ;
	return erSUCCESS ;
}
