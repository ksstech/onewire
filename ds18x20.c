/*
 * Copyright 2018-20 AM Maree/KSS Technologies (Pty) Ltd.
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
 * ds18x20.c
 */

#include	"onewire_platform.h"

#if		(halHAS_DS18X20 > 0)
#include	"endpoints.h"
#include	"printfx.h"
#include	"syslog.h"
#include	"systiming.h"					// timing debugging
#include	"x_errors_events.h"
#include	"x_values_convert.h"
#include	"x_string_to_values.h"
#include	"x_string_general.h"

#include	"commands.h"
#include	"ds18x20_cmds.h"
#include	"rules_engine.h"

#include	"hal_debug.h"

#include	<string.h>
#include	<stdint.h>

#define	debugFLAG					0xD000

#define	debugCONFIG					(debugFLAG & 0x0001)
#define	debugCONVERT				(debugFLAG & 0x0002)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ##################################### Developer notes ###########################################
/*
 *
 * DS18x20 is a 1-wire type device and thus BUS oriented:
 * 	multiple devices sharing a single bus.
 * 	each device can be individually R/W addressed
 * 	some operations eg temp sample/convert
 * 		happens reasonably slowly (up to 750mS)
 * 		can be triggered to execute in parallel for all devices on a bus
 *	To optimise operation, this driver is based on the following decisions/constraints:
 *		Tsns is specified at device type (psEWP level) for ALL /ow/ds18x20 devices
 *		always trigger a sample+convert operation for ALL devices on a bus at same time.
 *		maintains Tsns at a value equal to lowest Tsns specified for any one ds18x20 device
 *		maintain a minimum Tsns of 1000mSec to be bigger than the ~750mS standard.
 *
 *
 * 	Test parasitic power
 * 	Test & benchmark overdrive speed
 * 	Implement and test ALARM scan and over/under event generation
 */

// ################################ Forward function declaration ###################################

ep_work_t * ds18x20GetWork(int32_t x) ;
void	ds18x20SetDefault(ep_work_t * psEWP, ep_work_t *psEWS) ;
void	ds18x20SetSense(ep_work_t * psEWP, ep_work_t * psEWS) ;
float	ds18x20GetTemperature(ep_work_t * psEWS) ;

// ###################################### Local variables ##########################################

const complex_t	sDS18X20Func = {
	.work	= ds18x20GetWork,
	.reset	= ds18x20SetDefault,
	.sense	= ds18x20SetSense,
	.get	= ds18x20GetTemperature,
} ;

cmnd_t saDS18Cmnd[] = {
	{ "RDT",	CmndDS18RDT },
	{ "RDSP",	CmndDS18RDSP },
	{ "WRSP",	CmndDS18WRSP },
	{ "WREE",	CmndDS18WREE },
} ;

// #################################### Local ONLY functions #######################################

int32_t	ds18x20CheckPower(ds18x20_t * psDS18X20) {
	int32_t iRV = OWChannelSelect(&psDS18X20->sOW) ;
	IF_myASSERT(debugRESULT, iRV != false) ;

	OWAddress(&psDS18X20->sOW, OW_CMD_SKIPROM) ;
	OWWriteByte(&psDS18X20->sOW, DS18X20_READ_PSU) ;
	iRV = OWReadBit(&psDS18X20->sOW) ;					// return status 0=parasitic 1=external
	IF_PRINT(debugTRACK, iRV ? "External Power" : "Parasitic Power") ;
	return iRV ;
}

int32_t	ds18x20SelectAndAddress(ds18x20_t * psDS18X20, uint8_t u8AddrMethod) {
	IF_myASSERT(debugPARAM, INRANGE_SRAM(psDS18X20)) ;
	int32_t iRV = OWChannelSelect(&psDS18X20->sOW) ;
	IF_myASSERT(debugRESULT, iRV != false) ;

	iRV = OWReset(&psDS18X20->sOW) ;
	IF_myASSERT(debugRESULT, iRV != false) ;

//	iRV = DS2482SetOverDrive() ;
//	IF_myASSERT(debugRESULT, iRV != false) ;

	OWAddress(&psDS18X20->sOW, u8AddrMethod) ;
	return iRV ;
}

// ###################################### scratchpad support #######################################

int32_t	ds18x20ReadSP(ds18x20_t * psDS18X20, int32_t Len) {
	IF_myASSERT(debugPARAM, INRANGE(0, Len, SIZEOF_MEMBER(ds18x20_t, RegX), int32_t)) ;
	ds18x20SelectAndAddress(psDS18X20, OW_CMD_MATCHROM);
	OWWriteByte(&psDS18X20->sOW, DS18X20_READ_SP) ;
	memset(psDS18X20->RegX, 0xFF, Len) ;				// 0xFF to read
	OWBlock(&psDS18X20->sOW, psDS18X20->RegX, Len) ;

	int32_t iRV ;
	if (Len == SIZEOF_MEMBER(ds18x20_t, RegX))			// if full scratchpad read, check CRC
		iRV = OWCheckCRC(psDS18X20->RegX, SIZEOF_MEMBER(ds18x20_t, RegX)) ;
	else
		iRV = OWReset(&psDS18X20->sOW) ;				// terminate read process
	IF_myASSERT(debugRESULT, iRV != false) ;
	IF_PRINT(debugTRACK, "SP Read: %-'+b\n", Len, psDS18X20->RegX) ;
	return iRV ;
}

int32_t	ds18x20WriteSP(ds18x20_t * psDS18X20) {
	ds18x20SelectAndAddress(psDS18X20, OW_CMD_MATCHROM) ;
	OWWriteByte(&psDS18X20->sOW, DS18X20_WRITE_SP) ;
	OWBlock(&psDS18X20->sOW, (uint8_t *) &psDS18X20->Thi, psDS18X20->sOW.ROM.Family == OWFAMILY_28 ? 3 : 2) ;	// Thi, Tlo [+Conf]
	return true ;
}

int32_t	ds18x20WriteEE(ds18x20_t * psDS18X20) {
	ds18x20SelectAndAddress(psDS18X20, OW_CMD_MATCHROM) ;
	int32_t iRV = OWWriteBytePower(&psDS18X20->sOW, DS18X20_COPY_SP) ;
	IF_myASSERT(debugRESULT, iRV != false) ;

	IF_SYSTIMER_START(debugTIMING, systimerDS1820B) ;
	vTaskDelay(pdMS_TO_TICKS(ds18x20DELAY_SP_COPY)) ;
	IF_SYSTIMER_STOP(debugTIMING, systimerDS1820B) ;

	OWLevel(&psDS18X20->sOW, owPOWER_STANDARD) ;
	return 1 ;
}

// ############################### ds18x20 (Family 10 & 28) support ################################

int32_t	ds18x20Initialize(ds18x20_t * psDS18X20) {
	ds18x20ReadSP(psDS18X20, SIZEOF_MEMBER(ds18x20_t, RegX)) ;
	psDS18X20->Res = (psDS18X20->sOW.ROM.Family == OWFAMILY_28) ? psDS18X20->fam28.Conf >> 5 : owFAM28_RES9B ;
	ds18x20ConvertTemperature(psDS18X20) ;
	return true ;
}

int32_t	ds18x20EnumerateCB(flagmask_t sFM, onewire_t * psOW) {
	ds18x20_t * psDS18X20 = &psaDS18X20[sFM.uCount] ;
	memcpy(&psDS18X20->sOW, psOW, sizeof(onewire_t)) ;
	psDS18X20->Idx	= sFM.uCount ;

	ep_work_t * psEWS =&psDS18X20->sWork ;
	memset(psEWS, 0, sizeof(ep_work_t)) ;
	psEWS->uri	= URI_DS18X20 ;
	psEWS->idx	= sFM.uCount ;
	psEWS->Var.varDef.cv.vartype	= vtVALUE ;
	psEWS->Var.varDef.cv.varsize	= vs32B ;
	psEWS->Var.varDef.cv.varform	= vfFXX ;
	psEWS->Var.varDef.cv.varcount	= 1 ;
	ds18x20Initialize(psDS18X20) ;

	ow_chan_info_t * psOW_CI = psOWPlatformGetInfoPointer(OWPlatformChanPhy2Log(psOW)) ;
	switch(psOW->ROM.Family) {
	case OWFAMILY_10:	psOW_CI->ds18s20++ ;	break ;
	case OWFAMILY_28:	psOW_CI->ds18b20++ ;	break ;
	default:			psOW_CI->ds18xxx++ ;	break ;
	}
	return 1 ;											// number of devices enumerated
}

int32_t	ds18x20Enumerate(int32_t xUri) {
	int32_t	iRV = 0 ;
	uint8_t	DevCount = 0 ;
	psaDS18X20 = malloc(Fam10_28Count * sizeof(ds18x20_t)) ;
	memset(psaDS18X20, 0, Fam10_28Count * sizeof(ds18x20_t)) ;
	IF_myASSERT(debugRESULT, INRANGE_SRAM(psaDS18X20)) ;

	IF_PRINT(debugTRACK, "AutoEnum DS18X20: Found=%d", Fam10_28Count) ;
	onewire_t	sOW ;
	iRV = OWPlatformScanner(OWFAMILY_10, ds18x20EnumerateCB, &sOW) ;
	if (iRV > 0)		DevCount += iRV ;

	iRV = OWPlatformScanner(OWFAMILY_28, ds18x20EnumerateCB, &sOW) ;
	if (iRV > 0)		DevCount += iRV ;

	IF_PRINT(debugTRACK, "  Enum=%d\n", DevCount) ;

	// Do once-off initialization for work structure entries
	ep_info_t	sEI ;
	vEpGetInfoWithIndex(&sEI, xUri) ;			// setup pointers to static and work tables
	IF_myASSERT(debugRESULT, sEI.psES && sEI.psEW) ;

	sEI.psEW->uri	= xUri ;
	sEI.psEW->Tsns	= ds18x20T_SNS_NORM ;
	sEI.psEW->Var.varDef.cv.pntr	= 1 ;
	sEI.psEW->Var.varDef.cv.varcount = DevCount ;	// number enumerated
	sEI.psEW->Var.varVal.pvoid		= (void *) &sDS18X20Func ;

	if (DevCount == Fam10_28Count) {
		iRV = DevCount ;
	} else {
		SL_ERR("Only %d of %d enumerated!!!", DevCount, Fam10_28Count) ;
		iRV = erFAILURE ;
	}
	return iRV ;										// number of devices enumerated
}

int32_t	ds18x20ResetConfig(ds18x20_t * psDS18X20) {
	psDS18X20->Thi	= 75 ;
	psDS18X20->Tlo	= 70 ;
	if (psDS18X20->sOW.ROM.Family == OWFAMILY_28)
		psDS18X20->fam28.Conf = 0x7F ;	// 12 bit resolution
	ds18x20WriteSP(psDS18X20) ;
	return ds18x20Initialize(psDS18X20) ;
}

int32_t	ds18x20SampleTemperature(ds18x20_t * psDS18X20, uint8_t u8AddrMethod) {
	ds18x20SelectAndAddress(psDS18X20, u8AddrMethod) ;
	int32_t iRV = OWWriteBytePower(&psDS18X20->sOW, DS18X20_CONVERT) ;
	IF_myASSERT(debugRESULT, iRV != false) ;

	TickType_t Tconv = pdMS_TO_TICKS(ds18x20DELAY_CONVERT) ;
	/* ONLY decrease delay if:
	 * 	specific ROM is addressed AND and it is DS18B20 ; OR
	 * 	ROM match skipped AND only DS18B20 devices on the bus */
	ow_chan_info_t * psOW_CI = psOWPlatformGetInfoPointer(OWPlatformChanPhy2Log(&psDS18X20->sOW)) ;
	if ((u8AddrMethod == OW_CMD_SKIPROM && psOW_CI->ds18s20 == 0 && psOW_CI->ds18xxx == 0) ||
		(u8AddrMethod == OW_CMD_MATCHROM && psDS18X20->sOW.ROM.Family == OWFAMILY_28))
		Tconv /= (4 - psDS18X20->Res) ;
	}

	IF_SYSTIMER_START(debugTIMING, systimerDS1820A) ;
	vTaskDelay(Tconv) ;
	IF_SYSTIMER_STOP(debugTIMING, systimerDS1820A) ;

	OWLevel(&psDS18X20->sOW, owPOWER_STANDARD) ;
	return true ;
}

int32_t	ds18x20ReadTemperature(ds18x20_t * psDS18X20) { return ds18x20ReadSP(psDS18X20, 2) ; }

int32_t	ds18x20ConvertTemperature(ds18x20_t * psDS18X20) {
	const uint8_t	u8Mask[4] = { 0xF8, 0xFC, 0xFE, 0xFF } ;
	uint16_t u16Adj = (psDS18X20->Tmsb << 8) | (psDS18X20->Tlsb & u8Mask[psDS18X20->Res]) ;
	psDS18X20->sWork.Var.varVal.x32.f32 = (float) u16Adj / 16.0 ;

#if		(debugCONVERT)
 	printfx_lock() ;
	OWPlatformCB_PrintDS18(makeMASKFLAG(1,1,0,0,0,0,0,0,0,0,0,0,psDS18X20->Idx), psDS18X20) ;
	printfx_nolock("  u16A=0x%04X\n", u16Adj) ;
	printfx_unlock() ;
#endif
	return true ;
}

float	ds18x20GetTemperature(int32_t Idx) {
	IF_myASSERT(debugPARAM, INRANGE_SRAM(psaDS18X20)) ;
	IF_myASSERT(debugPARAM, INRANGE(0, Idx, Fam10_28Count, uint8_t)) ;
	return psaDS18X20[Idx].xVal.f32 ;
}

void	ds18x20SetDefault(ep_work_t * psEWP, ep_work_t * psEWS) {
	IF_myASSERT(debugPARAM, psEWP->fSECsns == 0)
	// Stop sensing on EWP level since vEpConfigReset() will handle EWx
	psEWP->Rsns = 0 ;
}

void	ds18x20SetSense(ep_work_t * psEWP, ep_work_t * psEWS) {
	/* Optimal 1-Wire bus operation require that all devices (of a type) are detected
	 * (and read) in a single bus scan. BUT, for the DS18x20 the temperature conversion
	 * time if 750mSec (per bus or device) at normal (not overdrive) bus speed.
	 * When we get here the psEWS structure will already having been configured with the
	 * parameters as supplied, just check & adjust for validity & new min Tsns */
	if (psEWS->Tsns < ds18x20T_SNS_MIN)					// requested value in range?
		psEWS->Tsns = ds18x20T_SNS_MIN ;				// no, default to minimum

	if (psEWP->Tsns > psEWS->Tsns)
		psEWP->Tsns = psEWS->Tsns ;						// set lowest of EWP/EWS
	psEWS->Tsns = 0 ;									// discard EWS value
	psEWP->Rsns = psEWP->Tsns ;							// restart SNS timer
}

int32_t	ds18x20ReadConvertAll(struct ep_work_s * psEpWork) {
#if 0				// read & convert each enumerated, 1 by 1
	for (int32_t Idx = 0; Idx < Fam10_28Count; ++Idx) {
		ds18x20_t * psDS18X20 = &psaDS18X20[Idx] ;
		if (ds18x20SampleTemperature(psDS18X20, OW_CMD_MATCHROM) == false) {
			SL_ERR("Sampling failed") ;
			continue ;
		}
		if (ds18x20ReadTemperature(psDS18X20))
			ds18x20ConvertTemperature(psDS18X20) ;
		else
			SL_ERR("Read/Convert failed") ;
	}
	return erSUCCESS ;
#else				// read per bus, all on bus, then convert 1 by 1
	static uint8_t	PrevBus = 0xFF ;
	for (int32_t Idx = 0; Idx < Fam10_28Count; ++Idx) {
		ds18x20_t * psDS18X20 = &psaDS18X20[Idx] ;
		if (psDS18X20->sOW.PhyChan != PrevBus) {
			if (ds18x20SampleTemperature(psDS18X20, OW_CMD_SKIPROM) == false) {
				SL_ERR("Sampling failed") ;
				continue ;
			}
			PrevBus = psDS18X20->sOW.PhyChan ;
		}
		if (ds18x20ReadTemperature(psDS18X20))
			ds18x20ConvertTemperature(psDS18X20) ;
		else
			SL_ERR("Read/Convert failed") ;
	}
	return erSUCCESS ;
#endif
}

int32_t	ds18x20Alarm(flagmask_t sFM, onewire_t * psOW) {
	sFM.bNL	= 1 ;
	sFM.bRT	= 1 ;
	OWPlatformCB_Print1W(sFM, psOW) ;
	return 1 ;
}

int32_t	ds18x20ScanAlarmsAll(void) {
	onewire_t	sOW ;
	OWPlatformScanner(OWFAMILY_28, ds18x20Alarm, &sOW) ;
	return erSUCCESS ;
}

/*
 SKIP +SPU +750mS		Work
 MATCH +SPU +750mS		Work
 SKIP +SPU +XXXms		Work
 MATCH +SPU +XXXmS		Work
 SKIP -SPU +XXXmS		Work

 */
int32_t	ds18x20SetResolution(ds18x20_t * psDS18X20, int8_t i8Res) {
	if (psDS18X20->sOW.ROM.Family == OWFAMILY_28 && INRANGE(9, i8Res, 12, uint8_t)) {
		uint8_t u8Res = ((i8Res - 9) << 5) | 0x1F ;
		if (psDS18X20->fam28.Conf != u8Res) {
			IF_PRINT(debugCONFIG, "Config Res:0x%02X -> 0x%02X (%d -> %d)\n", psDS18X20->fam28.Conf, u8Res, psDS18X20->Res + 9, i8Res) ;
			psDS18X20->fam28.Conf = u8Res ;
			psDS18X20->Res = i8Res - 9 ;
			ds18x20WriteSP(psDS18X20) ;
		}
		return true ;
	}
	return false ;
}

int32_t	ds18x20SetAlarms(ds18x20_t * psDS18X20, int8_t i8Lo, int8_t i8Hi) {
	if (psDS18X20->Tlo != i8Lo || psDS18X20->Thi != i8Hi) {
		IF_PRINT(debugCONFIG, "Config Tlo:%d -> %d  Thi:%d -> %d\n", psDS18X20->Tlo, i8Lo, psDS18X20->Thi, i8Hi) ;
		psDS18X20->Tlo = i8Lo ;
		psDS18X20->Thi = i8Hi ;
		ds18x20WriteSP(psDS18X20) ;
		return true ;
	}
	return false ;
}

enum { optINVALID = 0, optRESOLUTION, optTHRESHOLDS, optWRITE } ;

int32_t	ds18x20ConfigMode (struct rule_s * psRule) {
	ep_work_t * psEW = &table_work[psRule->actPar0[psRule->ActIdx]] ;
	p32_t	paX32 ;
	paX32.pi32 = (int32_t *) &psRule->para.i32[0][0] ;
	IF_PRINT(debugCONFIG, "DS18X20 Mode p0=%d p1=%d p2=%d p3=%d\n", *paX32.pi32, *(paX32.pi32+1), *(paX32.pi32+2), *(paX32.pi32+3)) ;

	int	Xcur = 0, p = 0 ;
	int Xmax = psEW->Var.varDef.cv.varcount ;
	if (Xmax > 1)										// multiple possible end-points?
		Xcur = *(paX32.pi32 + p++) ;					// get # of selected end-point(s)
	IF_PRINT(debugPARAM, "  XCur=%d/%d", Xcur, Xmax) ;
	if (Xcur == 255)									// non-specific total count ?
		Xcur = Xmax ;									// yes, set to actual count.
	else if (Xcur > Xmax)
		return erSCRIPT_INV_INDEX ;
	if (Xcur == Xmax)
		Xcur = 0 ; 										// range 0 -> Xmax
	else
		Xmax = Xcur ;									// single Xcur

	int p0 = *(paX32.pi32 + p++);
	int p1 = *(paX32.pi32 + p++);
	int p2 = *(paX32.pi32 + p++);
	int32_t iRV ;

	do {
		ds18x20_t * psDS18X20 = &psaDS18X20[Xcur] ;
		switch (p0) {
		case optRESOLUTION:
			iRV = ds18x20SetResolution(psDS18X20, p1) ;
			break ;
		case optTHRESHOLDS:
			iRV = ds18x20SetAlarms(psDS18X20, p1, p2) ;
			break ;
		case optWRITE:
			iRV = ds18x20WriteSP(psDS18X20) ;
			break ;
		default:
			iRV = erSCRIPT_INV_MODE ;
		}
		if (iRV < erSUCCESS)
			break ;
	} while (++Xcur < Xmax) ;
	return iRV ;
}

// ##################################### CLI functionality #########################################

int32_t	CmndDS18RDT(cli_t * psCLI) {
	do {
		ds18x20_t * psDS18X20 = &psaDS18X20[psCLI->z64Var.x64.u8[0]++] ;
		ds18x20SampleTemperature(psDS18X20, OW_CMD_MATCHROM) ;
		ds18x20ReadTemperature(psDS18X20) ;
		ds18x20ConvertTemperature(psDS18X20) ;
	} while (psCLI->z64Var.x64.u8[0] < psCLI->z64Var.x64.u8[1]) ;
	return erSUCCESS ;
}

int32_t	CmndDS18RDSP(cli_t * psCLI) {
	do ds18x20ReadSP(&psaDS18X20[psCLI->z64Var.x64.u8[0]++], 9) ; while (psCLI->z64Var.x64.u8[0] < psCLI->z64Var.x64.u8[1]) ;
	return erSUCCESS ;
}

int32_t	CmndDS18WRSP(cli_t * psCLI) {
	do ds18x20WriteSP(&psaDS18X20[psCLI->z64Var.x64.u8[0]++]) ; while (psCLI->z64Var.x64.u8[0] < psCLI->z64Var.x64.u8[1]) ;
	return erSUCCESS ;
}

int32_t	CmndDS18WREE(cli_t * psCLI) {
	do ds18x20WriteEE(&psaDS18X20[psCLI->z64Var.x64.u8[0]++]) ; while(psCLI->z64Var.x64.u8[0] < psCLI->z64Var.x64.u8[1]) ;
	return erSUCCESS ;
}

int32_t	CmndDS18(cli_t * psCLI) {
	int32_t iRV = erFAILURE ;
	psCLI->pasList	= saDS18Cmnd ;
	psCLI->u8LSize	= NUM_OF_MEMBERS(saDS18Cmnd) ;
	psCLI->pcParse	+= xStringSkipDelim(psCLI->pcParse, sepSPACE_COMMA, psCLI->pcStore - psCLI->pcParse ) ;
	int32_t	i32SC = xCLImatch(psCLI) ;
	if (i32SC >= 0) {
		// parse the logical channel number
		char * pTmp = pcStringParseValueRange(psCLI->pcParse, (p32_t) &psCLI->z64Var.x32[0].u32, vfUXX, vs32B, sepSPACE_LF, (x32_t) 0, (x32_t) ((uint32_t) Fam10_28Count)) ;
		TRACK("Cmnd=%d  Chan=%d", i32SC, psCLI->z64Var.x32[0].u32) ;
		if (pTmp != pcFAILURE) {
			psCLI->pcParse = pTmp ;
			psCLI->z64Var.x64.u8[0] = (psCLI->z64Var.x32[0].u32 == Fam10_28Count) ? 0 : psCLI->z64Var.x32[0].u32 ;
			psCLI->z64Var.x64.u8[1] = (psCLI->z64Var.x32[0].u32 == Fam10_28Count) ? psCLI->z64Var.x32[0].u32 : psCLI->z64Var.x64.u8[0] ;
			iRV = psCLI->pasList[i32SC].hdlr(psCLI) ;
		}
	}
	return iRV ;
}

#endif
