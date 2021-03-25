/*
 * Copyright 2018-21 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

/*
 * ds18x20.c
 */

#include	"onewire_platform.h"

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

#include	<string.h>
#include	<stdint.h>

#define	debugFLAG					0xD000

#define	debugCONFIG					(debugFLAG & 0x0001)
#define	debugREAD					(debugFLAG & 0x0002)
#define	debugCONVERT				(debugFLAG & 0x0004)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ##################################### Developer notes ###########################################

/* DS18x20 is a 1-wire type device and thus BUS oriented:
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
 * 	Test parasitic power
 * 	Test & benchmark overdrive speed
 * 	Implement and test ALARM scan and over/under event generation
 */


#ifndef URI_DS18X20
	#define	URI_DS18X20 URI_UNKNOWN	// dummy value to facilitate compile if EndPoint not used.
#endif

// ################################ Forward function declaration ###################################

epw_t * ds18x20GetWork(int32_t x) ;
void	ds18x20SetDefault(epw_t * psEWP, epw_t *psEWS) ;
void	ds18x20SetSense(epw_t * psEWP, epw_t * psEWS) ;
float	ds18x20GetTemperature(epw_t * psEWS) ;

// ######################################### Constants #############################################

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

// ###################################### Local variables ##########################################

ds18x20_t *	psaDS18X20	= NULL ;
uint8_t		Fam10_28Count	= 0 ;
static uint8_t	PrevBus ;

// #################################### Local ONLY functions #######################################

int32_t	ds18x20CheckPower(ds18x20_t * psDS18X20) {
	int32_t iRV = OWChannelSelect(&psDS18X20->sOW) ;
	IF_myASSERT(debugRESULT, iRV != 0) ;

	OWAddress(&psDS18X20->sOW, OW_CMD_SKIPROM) ;
	OWWriteByte(&psDS18X20->sOW, DS18X20_READ_PSU) ;
	iRV = OWReadBit(&psDS18X20->sOW) ;					// return status 0=parasitic 1=external
	IF_PRINT(debugTRACK, iRV ? "External Power" : "Parasitic Power") ;
	return iRV ;
}

int32_t	ds18x20SelectAndAddress(ds18x20_t * psDS18X20, uint8_t u8AddrMethod) {
	IF_myASSERT(debugPARAM, halCONFIG_inSRAM(psDS18X20)) ;
	int32_t iRV = OWChannelSelect(&psDS18X20->sOW) ;
	IF_myASSERT(debugRESULT, iRV != 0) ;

	iRV = OWReset(&psDS18X20->sOW) ;
	IF_myASSERT(debugRESULT, iRV != 0) ;

//	iRV = DS2482SetOverDrive() ;
//	IF_myASSERT(debugRESULT, iRV != 0) ;

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

	int32_t iRV = 1 ;									// default return status
	if (Len == SIZEOF_MEMBER(ds18x20_t, RegX)) {		// if full scratchpad read, check CRC
		iRV = OWCheckCRC(psDS18X20->RegX, SIZEOF_MEMBER(ds18x20_t, RegX)) ;
		IF_myASSERT(debugRESULT, iRV != 0) ;			// ensure CRC is correct
	} else {
		OWReset(&psDS18X20->sOW) ;						// terminate read
	}
	IF_PRINT(debugREAD, "SP Read: %-'+b\n", Len, psDS18X20->RegX) ;
	return iRV ;
}

int32_t	ds18x20WriteSP(ds18x20_t * psDS18X20) {
	ds18x20SelectAndAddress(psDS18X20, OW_CMD_MATCHROM) ;
	OWWriteByte(&psDS18X20->sOW, DS18X20_WRITE_SP) ;
	OWBlock(&psDS18X20->sOW, (uint8_t *) &psDS18X20->Thi, psDS18X20->sOW.ROM.Family == OWFAMILY_28 ? 3 : 2) ;	// Thi, Tlo [+Conf]
	return 1 ;
}

int32_t	ds18x20WriteEE(ds18x20_t * psDS18X20) {
	ds18x20SelectAndAddress(psDS18X20, OW_CMD_MATCHROM) ;
	int32_t iRV = OWWriteBytePower(&psDS18X20->sOW, DS18X20_COPY_SP) ;
	IF_myASSERT(debugRESULT, iRV != 0) ;

	IF_SYSTIMER_START(debugTIMING, systimerDS1820B) ;
	vTaskDelay(pdMS_TO_TICKS(ds18x20DELAY_SP_COPY)) ;
	IF_SYSTIMER_STOP(debugTIMING, systimerDS1820B) ;

	OWLevel(&psDS18X20->sOW, owPOWER_STANDARD) ;
	return 1 ;
}

// ############################### ds18x20 (Family 10 & 28) support ################################

int32_t	ds18x20Initialize(ds18x20_t * psDS18X20) {
	ds18x20ReadSP(psDS18X20, SIZEOF_MEMBER(ds18x20_t, RegX)) ;
	psDS18X20->Res = (psDS18X20->sOW.ROM.Family == OWFAMILY_28)
					? psDS18X20->fam28.Conf >> 5
					: owFAM28_RES9B ;
	ds18x20ConvertTemperature(psDS18X20) ;
	return 1 ;
}

int32_t	ds18x20EnumerateCB(flagmask_t sFM, onewire_t * psOW) {
	ds18x20_t * psDS18X20 = &psaDS18X20[sFM.uCount] ;
	memcpy(&psDS18X20->sOW, psOW, sizeof(onewire_t)) ;
	psDS18X20->Idx	= sFM.uCount ;

	epw_t * psEWx = &psDS18X20->sEWx ;
	memset(psEWx, 0, sizeof(epw_t)) ;
	psEWx->uri						= URI_DS18X20 ;
	psEWx->idx						= sFM.uCount ;
	psEWx->Var.varDef.cv.vartype	= vtVALUE ;
	psEWx->Var.varDef.cv.varsize	= vs32B ;
	psEWx->Var.varDef.cv.varform	= vfFXX ;
	psEWx->Var.varDef.cv.varcount	= 1 ;
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
	IF_myASSERT(debugRESULT, halCONFIG_inSRAM(psaDS18X20)) ;

	IF_PRINT(debugTRACK, "AutoEnum DS18X20: Found=%d", Fam10_28Count) ;
	onewire_t	sOW ;
	iRV = OWPlatformScanner(OWFAMILY_10, ds18x20EnumerateCB, &sOW) ;
	if (iRV > 0) {
		DevCount += iRV ;
	}
	iRV = OWPlatformScanner(OWFAMILY_28, ds18x20EnumerateCB, &sOW) ;
	if (iRV > 0) {
		DevCount += iRV ;
	}
	IF_PRINT(debugREAD, "  Enum=%d\n", DevCount) ;

	// Do once-off initialization for work structure entries
	epi_t	sEI ;
	vEpGetInfoWithIndex(&sEI, xUri) ;			// setup pointers to static and work tables
	IF_myASSERT(debugRESULT, halCONFIG_inFLASH(sEI.psES) && halCONFIG_inSRAM(sEI.psEW)) ;

	sEI.psEW->uri					= xUri ;
	sEI.psEW->Var.varDef.cv.pntr	= 1 ;
	sEI.psEW->Var.varDef.cv.varcount= DevCount ;		// number enumerated
	sEI.psEW->Var.varVal.px.pv	= (void *) &sDS18X20Func ;

	if (DevCount == Fam10_28Count) {
		iRV = DevCount ;
	} else {
		SL_ERR("Only %d of %d enumerated!!!", DevCount, Fam10_28Count) ;
		iRV = erFAILURE ;
	}
	return iRV ;										// number of devices enumerated
}

/**
 * ds18x20ResetConfig - reset device to default via SP, not written to EE
 */
int32_t	ds18x20ResetConfig(ds18x20_t * psDS18X20) {
	psDS18X20->Thi	= 75 ;
	psDS18X20->Tlo	= 70 ;
	if (psDS18X20->sOW.ROM.Family == OWFAMILY_28) {
		psDS18X20->fam28.Conf = 0x7F ;	// 12 bit resolution
	}
	ds18x20WriteSP(psDS18X20) ;
	return ds18x20Initialize(psDS18X20) ;
}

int32_t	ds18x20SampleTemperature(ds18x20_t * psDS18X20, uint8_t u8AddrMethod) {
	ds18x20SelectAndAddress(psDS18X20, u8AddrMethod) ;
	int32_t iRV = OWWriteBytePower(&psDS18X20->sOW, DS18X20_CONVERT) ;
	if (iRV == 0) {
		return iRV ;
	}
	TickType_t Tconv = pdMS_TO_TICKS(ds18x20DELAY_CONVERT) ;
	/* ONLY decrease delay if:
	 * 	specific ROM is addressed AND and it is DS18B20 ; OR
	 * 	ROM match skipped AND only DS18B20 devices on the bus */
	ow_chan_info_t * psOW_CI = psOWPlatformGetInfoPointer(OWPlatformChanPhy2Log(&psDS18X20->sOW)) ;
	if ((u8AddrMethod == OW_CMD_SKIPROM && psOW_CI->ds18s20 == 0 && psOW_CI->ds18xxx == 0) ||
		(u8AddrMethod == OW_CMD_MATCHROM && psDS18X20->sOW.ROM.Family == OWFAMILY_28)) {
		Tconv /= (4 - psDS18X20->Res) ;
	}
	IF_SYSTIMER_START(debugTIMING, systimerDS1820A) ;
	vTaskDelay(Tconv) ;
	IF_SYSTIMER_STOP(debugTIMING, systimerDS1820A) ;

	OWLevel(&psDS18X20->sOW, owPOWER_STANDARD) ;
	return 1 ;
}

void	ds18x20ReportAll(void) {
	for (int i = 0; i < Fam10_28Count; ++i) {
		ds18x20_t * psDS18X20 = &psaDS18X20[i] ;
		OWPlatformCB_PrintDS18(makeMASKFLAG(0,1,0,0,0,1,1,1,1,1,1,1,i), psDS18X20) ;
	}
}

// #################################### IRMACOS support ############################################

int32_t	ds18x20ReadTemperature(ds18x20_t * psDS18X20) { return ds18x20ReadSP(psDS18X20, 2) ; }

int32_t	ds18x20ConvertTemperature(ds18x20_t * psDS18X20) {
	const uint8_t	u8Mask[4] = { 0xF8, 0xFC, 0xFE, 0xFF } ;
	uint16_t u16Adj = (psDS18X20->Tmsb << 8) | (psDS18X20->Tlsb & u8Mask[psDS18X20->Res]) ;
	psDS18X20->sEWx.Var.varVal.x32.f32 = (float) u16Adj / 16.0 ;

#if		(debugCONVERT)
	OWPlatformCB_PrintDS18(makeMASKFLAG(1,0,0,0,0,0,0,0,0,0,0,0,psDS18X20->Idx), psDS18X20) ;
	printfx("  u16A=0x%04X\n", u16Adj) ;
#endif
	return 1 ;
}

epw_t * ds18x20GetWork(int32_t x) {
	IF_myASSERT(debugPARAM, halCONFIG_inSRAM(psaDS18X20) && x < Fam10_28Count) ;
	return &psaDS18X20[x].sEWx ;
}

void	ds18x20SetDefault(epw_t * psEWP, epw_t * psEWS) {
	IF_myASSERT(debugPARAM, psEWP->fSECsns == 0) ;
	// Stop sensing on EWP level since vEpConfigReset() will handle EWx
	psEWP->Rsns = 0 ;
}

void	ds18x20SetSense(epw_t * psEWP, epw_t * psEWS) {
	/* Optimal 1-Wire bus operation require that all devices (of a type) are detected
	 * (and read) in a single bus scan. BUT, for the DS18x20 the temperature conversion
	 * time if 750mSec (per bus or device) at normal (not overdrive) bus speed.
	 * When we get here the psEWS structure will already having been configured with the
	 * parameters as supplied, just check & adjust for validity & new min Tsns */
	if (psEWS->Tsns < ds18x20T_SNS_MIN)	{				// requested value in range?
		psEWS->Tsns = ds18x20T_SNS_MIN ;				// no, default to minimum
	}
	if (psEWP->Tsns > psEWS->Tsns) {
		psEWP->Tsns = psEWS->Tsns ;						// set lowest of EWP/EWS
	}
	psEWS->Tsns = 0 ;									// discard EWS value
	psEWP->Rsns = psEWP->Tsns ;							// restart SNS timer
}

float	ds18x20GetTemperature(epw_t * psEWS) { return psEWS->Var.varVal.x32.f32 ; }

int32_t	ds18x20ReadConvertAll(epw_t * psEWP) {
	PrevBus = 0xFF ;
	for (int i = 0; i < Fam10_28Count; ++i) {
		ds18x20_t * psDS18X20 = &psaDS18X20[i] ;
		if (psDS18X20->sOW.PhyChan != PrevBus) {
			if (ds18x20SampleTemperature(psDS18X20, OW_CMD_SKIPROM) == 0) {
				SL_ERR("Sampling failed") ;
				continue ;
			}
			PrevBus = psDS18X20->sOW.PhyChan ;
		}
		if (ds18x20ReadTemperature(psDS18X20)) {
			ds18x20ConvertTemperature(psDS18X20) ;
		} else {
			SL_ERR("Read/Convert failed") ;
		}
	}
	return erSUCCESS ;
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
			return 1 ;
		}
	}
	return 0 ;
}

int32_t	ds18x20SetAlarms(ds18x20_t * psDS18X20, int8_t i8Lo, int8_t i8Hi) {
	if (psDS18X20->Tlo != i8Lo || psDS18X20->Thi != i8Hi) {
		IF_PRINT(debugCONFIG, "Config Tlo:%d -> %d  Thi:%d -> %d\n", psDS18X20->Tlo, i8Lo, psDS18X20->Thi, i8Hi) ;
		psDS18X20->Tlo = i8Lo ;
		psDS18X20->Thi = i8Hi ;
		ds18x20WriteSP(psDS18X20) ;
		return 1 ;
	}
	return 0 ;
}

enum { optINVALID = 0, optRESOLUTION, optTHRESHOLDS, optWRITE } ;

int32_t	ds18x20ConfigMode (struct rule_t * psRule) {
	epw_t * psEW = &table_work[psRule->actPar0[psRule->ActIdx]] ;
	px_t	paX32 ;
	paX32.pi32 = (int32_t *) &psRule->para.i32[0][0] ;
	IF_PRINT(debugCONFIG, "DS18X20 Mode p0=%d p1=%d p2=%d p3=%d\n", *paX32.pi32, *(paX32.pi32+1), *(paX32.pi32+2), *(paX32.pi32+3)) ;

	int	Xcur = 0, p = 0 ;
	int Xmax = psEW->Var.varDef.cv.varcount ;
	if (Xmax > 1) {										// multiple possible end-points?
		Xcur = *(paX32.pi32 + p++) ;					// get # of selected end-point(s)
	}
	IF_PRINT(debugCONFIG, "  XCur=%d/%d\n", Xcur, Xmax) ;
	if (Xcur == 255) {									// non-specific total count ?
		Xcur = Xmax ;									// yes, set to actual count.
	} else if (Xcur > Xmax) {
		return erSCRIPT_INV_INDEX ;
	}
	if (Xcur == Xmax) {
		Xcur = 0 ; 										// range 0 -> Xmax
	} else {
		Xmax = Xcur ;									// single Xcur
	}
	int p0 = *(paX32.pi32 + p++);
	int p1 = *(paX32.pi32 + p++);
	int p2 = *(paX32.pi32 + p++);
	int32_t iRV = erSUCCESS ;

	do {
		ds18x20_t * psDS18X20 = &psaDS18X20[Xcur] ;
		switch (p0) {
		case optRESOLUTION:
			ds18x20SetResolution(psDS18X20, p1) ;
			break ;
		case optTHRESHOLDS:
			ds18x20SetAlarms(psDS18X20, p1, p2) ;
			break ;
		case optWRITE:
			ds18x20WriteSP(psDS18X20) ;
			break ;
		default:
			iRV = erSCRIPT_INV_MODE ;
		}
		if (iRV < erSUCCESS) {
			break ;
		}
	} while (++Xcur < Xmax) ;
	return iRV ;
}

// ##################################### CLI functionality #########################################

int32_t	CmndDS18RDT(cli_t * psCLI) {
	do {
		ds18x20_t * psDS18X20 = &psaDS18X20[psCLI->z64Var.x64.x8[0].u8++] ;
		ds18x20SampleTemperature(psDS18X20, OW_CMD_MATCHROM) ;
		ds18x20ReadTemperature(psDS18X20) ;
		ds18x20ConvertTemperature(psDS18X20) ;
	} while (psCLI->z64Var.x64.x8[0].u8 < psCLI->z64Var.x64.x8[1].u8) ;
	return erSUCCESS ;
}

int32_t	CmndDS18RDSP(cli_t * psCLI) {
	do {
		ds18x20ReadSP(&psaDS18X20[psCLI->z64Var.x64.x8[0].u8++], 9) ;
	} while (psCLI->z64Var.x64.x8[0].u8 < psCLI->z64Var.x64.x8[1].u8) ;
	return erSUCCESS ;
}

int32_t	CmndDS18WRSP(cli_t * psCLI) {
	do {
		ds18x20WriteSP(&psaDS18X20[psCLI->z64Var.x64.x8[0].u8++]) ;
	} while (psCLI->z64Var.x64.x8[0].u8 < psCLI->z64Var.x64.x8[1].u8) ;
	return erSUCCESS ;
}

int32_t	CmndDS18WREE(cli_t * psCLI) {
	do {
		ds18x20WriteEE(&psaDS18X20[psCLI->z64Var.x64.x8[0].u8++]) ;
	} while(psCLI->z64Var.x64.x8[0].u8 < psCLI->z64Var.x64.x8[1].u8) ;
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
		char * pTmp = pcStringParseValueRange(psCLI->pcParse, (px_t) &psCLI->z64Var.x64.x32[0].u32, vfUXX, vs32B, sepSPACE_LF, (x32_t) 0, (x32_t) ((uint32_t) Fam10_28Count)) ;
		TRACK("Cmnd=%d  Chan=%d", i32SC, psCLI->z64Var.x64.x32[0].u32) ;
		if (pTmp != pcFAILURE) {
			psCLI->pcParse = pTmp ;
			psCLI->z64Var.x64.x8[0].u8 = (psCLI->z64Var.x64.x32[0].u32 == Fam10_28Count) ? 0 : psCLI->z64Var.x64.x32[0].u32 ;
			psCLI->z64Var.x64.x8[1].u8 = (psCLI->z64Var.x64.x32[0].u32 == Fam10_28Count) ? psCLI->z64Var.x64.x32[0].u32 : psCLI->z64Var.x64.x8[0].u8 ;
			iRV = psCLI->pasList[i32SC].hdlr(psCLI) ;
		}
	}
	return iRV ;
}
