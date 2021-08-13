/*
 * Copyright 2018-21 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#include	"hal_variables.h"
#include	"onewire_platform.h"
#include	"endpoints.h"

#include	"printfx.h"
#include	"syslog.h"
#include	"systiming.h"					// timing debugging

#include	"x_errors_events.h"

#include	<string.h>

#define	debugFLAG					0xF000

#define	debugCONFIG					(debugFLAG & 0x0001)
#define	debugREAD					(debugFLAG & 0x0002)
#define	debugCONVERT				(debugFLAG & 0x0004)
#define	debugPOWER					(debugFLAG & 0x0008)

#define	debugSPAD					(debugFLAG & 0x0010)

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
 * 		can be triggered to execute in parallel for all "equivalent" devices on a bus
 *	To optimise operation, this driver is based on the following decisions/constraints:
 *		Tsns is specified at device type (psEWP level) for ALL /ow/ds18x20 devices and will
 *		always trigger a sample+convert operation for ALL devices on a bus at same time.
 *		EWP Tsns kept at a value equal to lowest of all EWS Tsns values
 *		Maintain a minimum Tsns of 1000mSec to be bigger than the ~750mS standard.
 * 	Test parasitic power
 * 	Test & benchmark overdrive speed
 * 	Implement and test ALARM scan and over/under alarm status scan
 *
 * 	Optimisation:
 * 	If more than 1 DS248XZ present Tsns will trigger convert on 1st bus of each DS248x device (parallelism)
 * 	Each device will start a timer to call handler to read and convert all DS18X20's on the bus
 * 	Handler will loop and read each sensor on the current bus.
 * 	If more than 1 bus on the device (DS2482-800) handler will release current bus.
 * 	The next bus will be selected and convert trigger triggered.
 * 	Logic will ONLY trigger convert on bus if 1 or more ds18x20 were discovered at boot.
 *
 */

// ################################ Forward function declaration ###################################


// ######################################### Constants #############################################


// ###################################### Local variables ##########################################


// #################################### Local ONLY functions #######################################

/**
 * @brief	Read power status bit, AI1 operation, select & release bus
 * @param	psDS18X20
 * @return	Power status
 */
bool ds18x20CheckPower(ds18x20_t * psDS18X20) {
	if (OWResetCommand(&psDS18X20->sOW, DS18X20_READ_PSU, owADDR_SKIP, 0) == 0) return 0 ;
	psDS18X20->sOW.PSU = OWReadBit(&psDS18X20->sOW) ;	// 0=parasitic 1=external
	return psDS18X20->sOW.PSU ;
}

// ###################################### scratchpad support #######################################

/**
 * @brief
 * @param	psDS18X20
 * @param	Len
 * @return
 * @note	Timing is as follows
 *	OWReset		196/1348uS
 *	OWCommand	1447/7740uS
 *	OWReadBlock		163/860 per byte, 326/1720 for temperature, 815/4300 for all.
 *	Total Time	1969/10808 for temperature
 */
int	ds18x20ReadSP(ds18x20_t * psDS18X20, int32_t Len) {
	if (OWResetCommand(&psDS18X20->sOW, DS18X20_READ_SP, owADDR_MATCH, 0) == 0) return 0 ;
	OWReadBlock(&psDS18X20->sOW, psDS18X20->RegX, Len);
	IF_TRACK(debugSPAD, "%'-B ", Len, psDS18X20->RegX);
	// If full SP read, verify CRC else terminate read
	return (Len == SO_MEM(ds18x20_t, RegX))
			? OWCheckCRC(psDS18X20->RegX, SO_MEM(ds18x20_t, RegX))
			: OWReset(&psDS18X20->sOW) ;
}

int	ds18x20WriteSP(ds18x20_t * psDS18X20) {
	if (OWResetCommand(&psDS18X20->sOW, DS18X20_WRITE_SP, owADDR_MATCH, 0) == 0) return 0 ;
	int Len = (psDS18X20->sOW.ROM.Family == OWFAMILY_28) ? 3 : 2 ;	// Thi, Tlo [+Conf]
	OWWriteBlock(&psDS18X20->sOW, (uint8_t *) &psDS18X20->Thi, Len);
	IF_TRACK(debugSPAD, "%'-B ", Len, psDS18X20->RegX);
	return 1 ;
}

int	ds18x20WriteEE(ds18x20_t * psDS18X20) {
	if (OWResetCommand(&psDS18X20->sOW, DS18X20_COPY_SP, owADDR_MATCH, 1) == 0) return 0 ;
	vTaskDelay(pdMS_TO_TICKS(ds18x20DELAY_SP_COPY));
	OWLevel(&psDS18X20->sOW, owPOWER_STANDARD);
	return 1 ;
}

// ################################ Basic temperature support ######################################

int	ds18x20TempRead(ds18x20_t * psDS18X20) { return ds18x20ReadSP(psDS18X20, 2) ; }

// ###################################### IRMACOS support ##########################################

int	ds18x20Initialize(ds18x20_t * psDS18X20) {
	if (ds18x20ReadSP(psDS18X20, SO_MEM(ds18x20_t, RegX)) == 0) return 0;
	ds18x20CheckPower(psDS18X20);
	psDS18X20->Res	= (psDS18X20->sOW.ROM.Family == OWFAMILY_28)
					? psDS18X20->fam28.Conf >> 5
					: owFAM28_RES9B;
	ds18x20ConvertTemperature(psDS18X20);
#if		(debugCONFIG && !debugCONVERT)
	OWP_PrintDS18_CB(makeMASKFLAG(0,1,0,0,0,0,0,0,0,0,0,0,psDS18X20->Idx), psDS18X20) ;
#endif
	return 1 ;
}

/**
 * @brief	reset device to default via SP, not written to EE
 * @param	psDS18X20
 * @return
 */
int	ds18x20ResetConfig(ds18x20_t * psDS18X20) {
	psDS18X20->Thi	= 75 ;
	psDS18X20->Tlo	= 70 ;
	if (psDS18X20->sOW.ROM.Family == OWFAMILY_28) psDS18X20->fam28.Conf = 0x7F ;	// 12 bit resolution
	ds18x20WriteSP(psDS18X20) ;
	return ds18x20Initialize(psDS18X20) ;
}

int	ds18x20ConvertTemperature(ds18x20_t * psDS18X20) {
	const uint8_t u8Mask[4] = { 0xF8, 0xFC, 0xFE, 0xFF } ;
	uint16_t u16Adj = (psDS18X20->Tmsb << 8) | (psDS18X20->Tlsb & u8Mask[psDS18X20->Res]) ;
	psDS18X20->sEWx.var.val.x32.f32 = (float) u16Adj / 16.0 ;
#if		(debugCONVERT && !debugCONFIG)
	OWP_PrintDS18_CB(makeMASKFLAG(0,1,0,0,0,0,0,0,0,0,0,0,psDS18X20->Idx), psDS18X20) ;
#endif
	return 1 ;
}

// ################################ Rules configuration support ####################################

int	ds18x20SetResolution(ds18x20_t * psDS18X20, int Res) {
	if (psDS18X20->sOW.ROM.Family == OWFAMILY_28 && INRANGE(9, Res, 12, int)) {
		Res -= 9 ;
		uint8_t u8Res = (Res << 5) | 0x1F ;
		IF_PRINT(debugCONFIG, "SP Res x%02X->x%02X (%d->%d)\n",
				psDS18X20->fam28.Conf, u8Res, psDS18X20->Res, Res) ;
		if (psDS18X20->fam28.Conf == u8Res) return 0;	// nothing changed
		psDS18X20->fam28.Conf = u8Res;
		psDS18X20->Res = Res ;
		return 1 ;										// changed, must write
	}
	SET_ERRINFO("Invalid Family/Resolution") ;
	return erSCRIPT_INV_VALUE ;
}

int	ds18x20SetAlarms(ds18x20_t * psDS18X20, int Lo, int Hi) {
	if (INRANGE(-128, Lo, 127, int) && INRANGE(-128, Hi, 127, int)) {
		IF_PRINT(debugCONFIG, "SP Tlo:%d -> %d  Thi:%d -> %d\n", psDS18X20->Tlo, Lo, psDS18X20->Thi, Hi) ;
		if (psDS18X20->Tlo == Lo && psDS18X20->Thi == Hi) return 0 ;
		psDS18X20->Tlo = Lo ;
		psDS18X20->Thi = Hi ;
		return 1 ;										// changed, must write
	}
	SET_ERRINFO("Invalid Lo/Hi alarm limits") ;
	return erSCRIPT_INV_VALUE ;
}

int	ds18x20ConfigMode (struct rule_t * psRule) {
	if (psaDS18X20 == NULL) {
		SET_ERRINFO("No DS18x20 enumerated");
		return erSCRIPT_INV_OPERATION;
	}
	// support syntax mode /ow/ds18x20 idx lo hi res [1=persist]
	uint8_t	AI = psRule->ActIdx ;
	epw_t * psEW = &table_work[psRule->actPar0[AI]] ;
	px_t	px ;
	px.pu32 = (uint32_t *) &psRule->para.u32[AI][0] ;
	int	Xcur = *px.pu32++ ;
	int Xmax = psEW->var.def.cv.vc ;

	if (Xcur == 255) Xcur = Xmax ;						// set to actual count.
	else if (Xcur > Xmax) {
		SET_ERRINFO("Invalid EP Index");
		return erSCRIPT_INV_INDEX;
	}

	if (Xcur == Xmax) Xcur = 0 ; 						// range 0 -> Xmax
	else Xmax = Xcur ;									// single Xcur

	uint32_t lo	= *px.pu32++ ;
	uint32_t hi	= *px.pu32++ ;
	uint32_t res = *px.pu32++ ;
	uint32_t wr	= *px.pu32 ;
	IF_PRINT(debugCONFIG, "DS18X20 Mode Xcur=%d lo=%d hi=%d res=%d wr=%d\n", Xcur, lo, hi, res, wr) ;
	int iRV1, iRV2 ;
	if (wr == 0 || wr == 1) {							// if parameter omitted, do not persist
		do {
			iRV1 = erFAILURE ;
			iRV2 = erFAILURE ;
			ds18x20_t * psDS18X20 = &psaDS18X20[Xcur] ;
			if (OWP_BusSelect(&psDS18X20->sOW) == 1) {
				// Do resolution 1st since small range (9-12) a good test for valid parameter
				iRV1 = ds18x20SetResolution(psDS18X20, res) ;
				if (iRV1 > erFAILURE) {
					iRV2 = ds18x20SetAlarms(psDS18X20, lo, hi)  ;
					if (iRV2 > erFAILURE) {
						if (iRV1 == 1 || iRV2 == 1) {
							iRV1 = ds18x20WriteSP(psDS18X20) ;
							if (wr == 1) ds18x20WriteEE(psDS18X20);
						}
					}
				}
				OWP_BusRelease(&psDS18X20->sOW) ;
			}
			if (iRV2 < erSUCCESS) break ;
		} while (++Xcur < Xmax) ;
	} else {
		SET_ERRINFO("Invalid persist flag, not 0/1") ;
		iRV2 = erSCRIPT_INV_MODE ;
	}
	return iRV2 ;
}

// ######################################### Reporting #############################################

void ds18x20ReportAll(void) {
	for (int i = 0; i < Fam10_28Count; ++i)
		OWP_PrintDS18_CB(makeMASKFLAG(0,1,1,1,1,1,1,1,1,i), &psaDS18X20[i]) ;
}
