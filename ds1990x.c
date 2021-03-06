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

#include	"onewire_platform.h"
#include	"task_events.h"
#include	"endpoints.h"
#include	"printfx.h"
#include	"syslog.h"
#include	"systiming.h"								// timing debugging
#include	"x_errors_events.h"
#include	"hal_variables.h"

#include	<string.h>

#define	debugFLAG					0xC001

#define	debugEVENTS					(debugFLAG & 0x0001)

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
int32_t	OWPlatformCB_ReadDS1990X(flagmask_t sFM, onewire_t * psOW) {
	seconds_t	NowRead = xTimeStampAsSeconds(sTSZ.usecs) ;
	uint8_t		LogChan = OWPlatformChanPhy2Log(psOW) ;
	ow_chan_info_t * psOW_CI = psOWPlatformGetInfoPointer(LogChan) ;
	++Family01Count ;
	if ((psOW_CI->LastROM.Value == psOW->ROM.Value) && (NowRead - psOW_CI->LastRead) <= ds1990ReadIntvl) {
		IF_PRINT(debugTRACK, "SAME iButton in %d sec, Skipped...\n", ds1990ReadIntvl) ;
		return erSUCCESS ;
	}
	psOW_CI->LastROM.Value	= psOW->ROM.Value ;
	psOW_CI->LastRead		= NowRead ;
	xTaskNotify(EventsHandle, 1UL << (LogChan + se1W_FIRST), eSetBits) ;
	portYIELD() ;
#if		(debugEVENTS)
	printfx_lock() ;
	sFM.bRT	= 1 ;
	sFM.bNL	= 1 ;
	OWPlatformCB_Print1W(sFM, psOW) ;
	printfx_unlock() ;
#endif
	return erSUCCESS ;
}

int32_t	ds1990xConfig(int32_t xUri) {
	ep_info_t	sEpInfo ;
	vEpGetInfoWithIndex(&sEpInfo, xUri) ;			// setup pointers to static and work tables
	IF_myASSERT(debugRESULT, sEpInfo.psES && sEpInfo.psEW) ;
	sEpInfo.psEW->uri	= xUri ;
	return erSUCCESS ;
}
