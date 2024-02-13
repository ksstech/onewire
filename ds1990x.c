/*
 * ds1990x.c - Copyright (c) 2020-24 Andre M. Maree / KSS Technologies (Pty) Ltd.
 */

#include "hal_config.h"

#if (HAL_ONEWIRE > 0) && (HAL_DS1990X > 0)
#include "endpoints.h"
#include "hal_options.h"
#include "onewire_platform.h"
#include "printfx.h"
#include "syslog.h"
#include "systiming.h"								// timing debugging
#include "x_errors_events.h"
#include "x_utilities.h"								// vShowActivity

#define	debugFLAG					0xF000

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ###################################### General macros ###########################################

#define	DS1990X_T_SNS			1000

// ################################# Platform related variables ####################################

u8_t Fam01Count = 0;

// ################################# Application support functions #################################

void ds1990xConfig(void) {
	epw_t * psEWP = &table_work[URI_DS1990X];
	psEWP->var.def = SETDEF_CVAR(0, 0, vtVALUE, cvU32, 1, 0);
	psEWP->Tsns = psEWP->Rsns = DS1990X_T_SNS;
	psEWP->uri = URI_DS1990X;		// Used in OWPlatformEndpoints()
	IF_SYSTIMER_INIT(debugTIMING, stDS1990, stMILLIS, "DS1990x", 1, 100);
	xRtosSetDevice(devMASK_DS1990X);
}

// #################################### 1W Platform support ########################################

/* To avoid registering multiple reads if iButton is held in place too long we enforce a
 * period of 'x' seconds within which successive reads of the same tag will be ignored */
int	ds1990SenseCB(report_t * psR, owdi_t * psOW) {
	seconds_t NowRead = xTimeStampAsSeconds(sTSZ.usecs);
	u8_t LogChan = OWP_BusP2L(psOW);
	owbi_t * psOW_CI = psOWP_BusGetPointer(LogChan);
	u8_t Dly = ioB4GET(dlyDS1990);
	if ((psOW_CI->LastROM.Value == psOW->ROM.Value) &&
		(NowRead - psOW_CI->LastRead) <= Dly) {
		IF_P(debugTRACK && ioB1GET(dbgDS1990x), "Tag repeat %ds\r\n", Dly);
	} else {
		psOW_CI->LastROM.Value = psOW->ROM.Value;
		psOW_CI->LastRead = NowRead;
		xTaskNotify(EventsHandle, 1UL << (LogChan + evtFIRST_OW), eSetBits);
		portYIELD();
		if (debugTRACK && ioB1GET(dbgDS1990x)) {
			psR->sFM.bRT = 1;
			psR->sFM.bNL = 1;
			OWP_Print1W_CB(psR, psOW);
		}
	}
	return erSUCCESS;
}

int	ds1990Sense(epw_t * psEWP) {
	IF_SYSTIMER_START(debugTIMING, stDS1990);
	int iRV = OWP_Scan(OWFAMILY_01, ds1990SenseCB);
	IF_SYSTIMER_STOP(debugTIMING, stDS1990);
	return iRV;
}
#endif
