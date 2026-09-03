// ds1990x.c - Copyright (c) 2020-26 Andre M. Maree / KSS Technologies (Pty) Ltd.

#include "hal_platform.h"

#if (HAL_ONEWIRE > 0) && (HAL_DS1990X > 0) && (appUSE_ENDPOINTS > 0)
#include "endpoints.h"
#include "errors_events.h"
#include "onewire_platform.h"
#include "options.h"
#include "rules.h"
#include "syslog.h"
#include "systiming.h"								// timing debugging
#include "task_events.h"
#include "utilitiesX.h"								// vShowActivity
#include "string_general.h"
#include "string_parse.h"
#include "string_to_values.h"
#include "FreeRTOS_Support.h"

#include <ctype.h>

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
	psEWP->var.def = SETDEF_CVAR(0,0,vtVALUE,cvU32,1,0,0);
	psEWP->Tsns = psEWP->Rsns = DS1990X_T_SNS;
	psEWP->uri = URI_DS1990X;		// Used in OWPlatformEndpoints()
	IF_SYSTIMER_INIT(debugTIMING, stDS1990, stTICKS, "DS1990x", 1, 100);
	halEventUpdateDevice(devMASK_DS1990X, 1);
}

// #################################### 1W Platform support ########################################

/* To avoid registering multiple reads if iButton is held in place too long we enforce a
 * period of 'x' seconds within which successive reads of the same tag will be ignored */
int	ds1990SenseCB(report_t * psR, owdi_t * psOW) {
	seconds_t NowRead = xTimeStampSeconds(sTSZ.usecs);
	u8_t LogChan = OWP_BusP2L(psOW);
	owbi_t * psOW_CI = psOWP_BusGetPointer(LogChan);
	u8_t Dly = xOptionGet(dlyDS1990);
	if ((psOW_CI->LastROM.Value == psOW->ROM.Value) && (NowRead - psOW_CI->LastRead) <= Dly) {
		IF_PX(debugTRACK && xOptionGet(dbgDS1990x), "Tag repeat %ds" strNL, Dly);
	} else {
		IF_PX(debugTRACK && xOptionGet(dbgDS1990x), "Tag %-.8hhY L=%d P=%d" strNL, &psOW->ROM, LogChan, psOW->PhyBus);
		#if (ds248xSTAT_DEBUG > 0)					// accepted reads only: repeats above are NOT counted
		++psaDS248X[psOW->DevNum].TagCnt[psOW->PhyBus];
		#endif
		psOW_CI->LastROM.Value = psOW->ROM.Value;
		psOW_CI->LastRead = NowRead;
		if (EventsHandle)								// NULLed on Events task exit - Sense and Events
			xTaskNotify(EventsHandle, 1UL << (LogChan + evtFIRST_OW), eSetBits);	// die in the same phase, unordered
		portYIELD();
	}
	return erSUCCESS;
}

int	ds1990Sense(epw_t * psEWP) {
	IF_SYSTIMER_START(debugTIMING, stDS1990);
	#if (HAL_DS248X > 0) && (ds248xCHAN_ATTRIB > 0)
	ds248xAuditRun();								// I-3: deferred audits (boot baseline/degraded health)
	#endif
	int iRV = OWP_Scan(OWFAMILY_01, ds1990SenseCB);
	IF_SYSTIMER_STOP(debugTIMING, stDS1990);
	return iRV;
}

#if (cmakeAEP == 2)		// ThingsBoard test tool: drives the REAL actuation pipeline from rule
						// text, which arrives unprivileged - excluded from every SiteWhere image
/* Simulated tag presentation: CMD /ow/ds1990x 0 <chan> <rom>
 * <rom> = 12 hex chars (tag serial MSB first, FAM 0x01 + CRC computed) or
 *         16 hex chars (full ROM as engraved ie CRC,serial,FAM - CRC verified)
 * Drives the REAL pipeline: dedup, psaOWBI, Events notify, rules, identity, actuation */
char * pcEpDS1990_CMD(rule_t * psR, char * pSrc) {
	if (psR->actPar1[psR->ActIdx] != 0)				// only code 0 (simulate) defined
		return pcFAILURE;
	u8_t Chan = 0;
	char * pcRV = cvParseRangeX32(pSrc, (px_t) &Chan, cvU08, (x32_t) 0, (x32_t) 7);
	if (pcRV == pcFAILURE || Chan >= OWP_GetNumBus())	// bad channel or OW absent/unconfigured
		return pcFAILURE;
	pcRV += xStringCountSpaces(pcRV);
	char caTok[20];
	int nChr = 0;
	while (nChr < 17 && isxdigit((int) pcRV[nChr])) {	// bounded copy, 17+ digits fails below
		caTok[nChr] = pcRV[nChr];
		++nChr;
	}
	if (nChr != 12 && nChr != 16)
		return pcFAILURE;
	caTok[nChr] = 0;
	u8_t caBin[8];
	if (xParseHexString(caTok, caBin, sizeof(caBin)) != nChr)
		return pcFAILURE;
	owdi_t sOW = { 0 };
	if (nChr == 16) {								// full ROM, engraved order -> FAM first
		for (int i = 0; i < sizeof(ow_rom_t); ++i)
			sOW.ROM.HexChars[i] = caBin[(sizeof(ow_rom_t) - 1) - i];
		if (sOW.ROM.FAM != OWFAMILY_01 || OWCheckCRC(sOW.ROM.HexChars, sizeof(ow_rom_t)) == 0)
			return pcFAILURE;
	} else {										// serial only, MSB first
		sOW.ROM.FAM = OWFAMILY_01;
		for (int i = 0; i < SO_MEM(ow_rom_t, TAG); ++i)
			sOW.ROM.TAG[(SO_MEM(ow_rom_t, TAG) - 1) - i] = caBin[i];
		sOW.ROM.CRC = OWCalcCRC8(sOW.ROM.HexChars, sizeof(ow_rom_t) - 1);
	}
	OWP_BusL2P(&sOW, Chan);							// fills DevNum/PhyBus incl AC00Xlat
	/* psaOWBI[] only writer (Sense task) holds shEPtiming across the scan - take the same
	 * mutex so injection never races it; bounded, a wedged scan must not hang the caller */
	if (xRtosSemaphoreTake(&shEPtiming, pdMS_TO_TICKS(5000)) != pdTRUE)
		return pcFAILURE;
	ds1990SenseCB(NULL, &sOW);						// real path: dedup, psaOWBI, notify
	xRtosSemaphoreGive(&shEPtiming);
	SL_NOT("SIM Tag %-.8hhY Ch=%d", &sOW.ROM, Chan);	// host-visible audit line
	return pcRV + nChr;
}
#endif		// (cmakeAEP == 2)
#endif
