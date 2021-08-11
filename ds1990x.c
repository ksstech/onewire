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

// ################################# Application support functions #################################

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
