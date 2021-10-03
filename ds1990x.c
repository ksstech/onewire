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

#define	debugFLAG					0xF001

#define	debugEVENTS					(debugFLAG & 0x0001)
#define	debugCONFIG					(debugFLAG & 0x0002)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ###################################### General macros ###########################################

#define	DS1990X_T_SNS			1000

// ################################# Platform related variables ####################################

uint8_t	Family01Count = 0 ;

// ################################# Application support functions #################################

int	ds1990xConfig(void) {
	epw_t * psEWP = &table_work[URI_DS1990X] ;
	IF_myASSERT(debugRESULT, halCONFIG_inSRAM(psEWP)) ;
	psEWP->var.def.cv.vc	= 1 ;
	psEWP->var.def.cv.vf	= vfUXX ;
	psEWP->var.def.cv.vt	= vtVALUE ;
	psEWP->var.def.cv.vs	= vs32B ;
	psEWP->Tsns = psEWP->Rsns = DS1990X_T_SNS ;
	psEWP->uri				= URI_DS1990X ;				// Used in OWPlatformEndpoints()
	IF_SYSTIMER_INIT(debugTIMING, stDS1990, stMILLIS, "DS1990x", 1, 100) ;
	return erSUCCESS;
}

// #################################### 1W Platform support ########################################

/* To avoid registering multiple reads if iButton is held in place too long we enforce a
 * period of 'x' seconds within which successive reads of the same tag will be ignored */
int	OWP_DS1990ScanCB(flagmask_t sFM, owdi_t * psOW) {
	seconds_t NowRead = xTimeStampAsSeconds(sTSZ.usecs);
	uint8_t LogChan = OWP_BusP2L(psOW);
	owbi_t * psOW_CI = psOWP_BusGetPointer(LogChan);
	if ((psOW_CI->LastROM.Value == psOW->ROM.Value) &&
		(NowRead-psOW_CI->LastRead) <= ioB4GET(ioDS1990RdDly)) {
		IF_PRINT(debugTRACK && ioB1GET(ioDS1990x), "Tag repeat %ds\n", ioB4GET(ioDS1990RdDly)) ;
		return erSUCCESS ;
	}
	psOW_CI->LastROM.Value	= psOW->ROM.Value ;
	psOW_CI->LastRead		= NowRead ;
	xTaskNotify(EventsHandle, 1UL << (LogChan + evtFIRST_OW), eSetBits) ;
	portYIELD() ;
	if (debugTRACK && ioB1GET(ioDS1990x)) { sFM.bRT = 1; sFM.bNL = 1; OWP_Print1W_CB(sFM, psOW); }
	return erSUCCESS ;
}

int	OWP_DS1990ScanAll(epw_t * psEWP) {
	IF_SYSTIMER_START(debugTIMING, stDS1990) ;
	int iRV = OWP_Scan(OWFAMILY_01, OWP_DS1990ScanCB) ;
	IF_SYSTIMER_STOP(debugTIMING, stDS1990) ;
	return iRV;
}

