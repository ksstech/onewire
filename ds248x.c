// ds248x.c - Copyright (c) 2020-26 Andre M. Maree / KSS Technologies (Pty) Ltd.

#include "hal_platform.h"

#if (HAL_DS248X > 0)
#include "hal_i2c_common.h"
#include "hal_network.h"							// temporary access to IP address
#include "FreeRTOS_Support.h"
#include "onewire_platform.h"
#include "options.h"
#include "syslog.h"
#include "systiming.h"								// timing debugging
#include "errors_events.h"
#include "string_general.h"

#include <string.h>

/* ##################################### Developer notes ###########################################
	Test at 400KHx I2C speed, maybe add auto detect and step up mode in SCAN routine?
	Add support to configure the PADJ register timing
*/

// ###################################### General macros ###########################################

#define	debugFLAG					0xF000
#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ######################################## Build macros ###########################################

#define	ds248xLOCK_DIS				0					// no locking
#define	ds248xLOCK_IO				1					// un/locked on I2C access level
#define	ds248xLOCK_BUS				2					// un/locked on Bus select level
#define	ds248xLOCK					ds248xLOCK_BUS

#define	dsERR_LOG_INTERVAL			pdMS_TO_TICKS(60000)	// rate limit: <=1 health report / device / minute
#define	dsBACKOFF_MIN				pdMS_TO_TICKS(5000)		// I5: first WEDGED recovery retry after 5s
#define	dsBACKOFF_MAX				pdMS_TO_TICKS(60000)	// I5: cap, one recovery attempt per minute

// ##################################### Local structures ##########################################

// ###################################### Local constants ##########################################

// ###################################### Local variables ##########################################

const char * const RegNames[ds248xREG_NUM] = {"Stat", "Data", "Chan", "Conf", "Port" };

// DS2482-800 only CHAN register xlat	0	  1		2	  3		4	  5		6	  7
static const u8_t ds248x_V2N[9] = { 0xB8, 0xB1, 0xAA, 0xA3, 0x9C, 0x95, 0x8E, 0x87, 0x00 };	// [8] = invalid/error sentinel
// DS2484 only reporting/debugging
static const u8_t Trstl[16]	= { 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74 };
static const u8_t Tmsp0[16]	= { 58, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 76, 76, 76, 76, 76 };
static const u8_t Tmsp1[16]	= { 55, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 110, 110, 110 };
static const u8_t Twol0[16]	= { 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 70, 70, 70, 70, 70, 70 };
static const u8_t Twol1[16]	= { 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 100, 100, 100, 100, 100 };
static const uint16_t Trec0[16]	= { 275, 275, 275, 275, 275, 275, 525, 775, 1025, 1275, 1525, 1775, 2025, 2275, 2525, 2525 };
static const uint16_t Rwpu[16]	= { 500, 500, 500, 500, 500, 500, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000 };

// ##################################### Global variables ##########################################

u8_t ds248xCount = 0;
ds248x_t * psaDS248X = NULL;
static int ResetOK = 0, ResetErr = 0, ResetBusy = 0;	// ResetBusy: DRST OK but 1W engine left wedged

enum { ds248xSTATE_OK, ds248xSTATE_ERR, ds248xSTATE_WEDGED };	// ds248x_t.State, owned by ds248xReportHealth()

// ################################ Local ONLY utility functions ###################################

static int ds248xWriteConfigRaw(ds248x_t * psDS248X, ds248x_conf_t sConf);	// fwd: used by ds248xLogError APU restore

/**
 * @brief	SINGLE device-level syslog originator for runtime DS248x errors.
 * @note	All error sites RECORD (counters + LastMsg) and call here; this owns the one report
 *			window, the OK/ERRORS/WEDGED state machine and the emission. One line carries what the
 *			previous three originators (per-channel LogError, the ds248xReset block, LogBusy) spread
 *			over ~10/min on a dead bus: per-channel counts, transport failures, recovery skips,
 *			DRST totals and the freshest error text.
 *			Hysteresis: escalations (state worsens) emit IMMEDIATELY; periodic repeats, recovery
 *			and de-escalations wait for the window - a flapping device costs at most one line per
 *			interval each way. Throttling applies in BOTH builds (the c764 lesson: losing the
 *			limiter in production, where every SL_* can block on a TCP send, is the worst place).
 *			WEDGED = two consecutive windows of either: (a) NO DRST succeeding AND failure wide
 *			(>=6 channels, or the transport failing repeatedly) - the c98c/c764 class; or (b) EVERY
 *			DRST succeeding while >=6 channels keep failing at volume - the c9a4 class, where the
 *			1W engine re-wedges after each successful reset. Both need a power cycle, hence the
 *			ALERT + action text.
 */
static void ds248xReportHealth(ds248x_t * psDS248X) {
	if (psDS248X->psI2C->Test)							// identify-time probing errors are expected;
		return;											//  ds248xIdentify reports its own outcome
	u32_t Sum = psDS248X->XErrCnt + psDS248X->SkipCnt;
	int Bad = 0;
	for (int i = 0; i < 8; ++i) {
		Sum += psDS248X->ErrSupp[i];
		if (psDS248X->ErrSupp[i])
			++Bad;
	}
	u32_t DRSTerr = (u32_t)ResetErr - psDS248X->PrvResetErr;
	u32_t DRSTok = (u32_t)ResetOK - psDS248X->PrvResetOK;
	u32_t now = xTaskGetTickCount();
	bool bWindow = (u32_t)(now - psDS248X->ErrLogTick) >= dsERR_LOG_INTERVAL;
	u8_t NewState = (Sum || DRSTerr) ? ds248xSTATE_ERR : ds248xSTATE_OK;
	if (bWindow) {										// wedge assessment advances once per WINDOW
		/* "dead in practice": >=10 DRST failures AND <=~10% success in the window. Zero-success
		 * was the original test, but c98c proved a wedged DS2482 still lands the odd lucky DRST
		 * (4 OK vs 237 failed per window) - and the intermittent wedge IS the power-cycle class. */
		if (psDS248X->State == ds248xSTATE_WEDGED) {
			/* I5: STICKY while erroring. The backoff caps DRST attempts far below the >=10
			 * entry criteria above, so re-testing them here would read the throttle itself as
			 * recovery and flap WEDGED->ERR->storm->WEDGED forever. And a lone lucky DRST is
			 * not recovery either (c98c: 4 OK vs 237 in-wedge). Only an ERROR-FREE window
			 * closes the wedge - the device answering everything again IS the recovery test. */
			if (Sum == 0 && DRSTerr == 0)
				psDS248X->WedgeCnt = 0;
		} else if (NewState == ds248xSTATE_ERR &&
			(  /* (a) DRST itself failing - the c98c/c764 class */
			   (  (Bad >= 6 || DRSTerr >= 10 || psDS248X->XErrCnt >= 10)
			   && DRSTerr >= 10 && DRSTerr >= 10 * (DRSTok + 1))
			   /* (b) DRST SUCCEEDS every time yet every channel keeps failing - c9a4 2026-08-31.
			    * The 1W engine re-wedges on the next xB4, so the DRST triple reads 147169/0/0 and
			    * (a) is structurally blind to it: 8 channels x ~60 errors and ~480 SUCCESSFUL
			    * DRST per window, for hours, until it RTCWDT-crashed. */
			|| (Bad >= 6 && Sum >= 100 && DRSTok >= 100))) {
			if (psDS248X->WedgeCnt < 255)
				++psDS248X->WedgeCnt;
		} else {
			psDS248X->WedgeCnt = 0;
		}
	}
	if (psDS248X->WedgeCnt >= 2)
		NewState = ds248xSTATE_WEDGED;
	// Escalations bypass the window; everything else (periodic, recovery, de-escalation) waits for it
	if (NewState <= psDS248X->State && !(bWindow && (Sum || DRSTerr || NewState != psDS248X->State)))
		return;
	static const char * const StateName[3] = { "OK", "ERRORS", "WEDGED" };
	char caFirst[64] = "";
	#if (ds248xCHAN_ATTRIB > 0)
	if (psDS248X->FirstMsg[0])							// I-1: the TRIGGER, not the freshest error
		snprintfx(caFirst, sizeof(caFirst), "  first=Ch%u '%s' Cmd=x%02X Stat=x%02X +%lus",
			psDS248X->FirstChan, psDS248X->FirstMsg, psDS248X->FirstCmd, psDS248X->FirstStat,
			(now - psDS248X->FirstTick) / configTICK_RATE_HZ);
	if (psDS248X->State == ds248xSTATE_OK && NewState != ds248xSTATE_OK)
		psDS248X->AuditPend = 1;						// I-3: audit ONCE at episode onset, not every degraded report
	#endif
	SL_LOG((NewState == ds248xSTATE_WEDGED && psDS248X->State != ds248xSTATE_WEDGED) ? SL_SEV_ALERT :
			NewState ? SL_SEV_ERROR : SL_SEV_NOTICE,
		"Dev=%d  1W %s->%s  Ch=%u/%u/%u/%u/%u/%u/%u/%u  XErr=%u  Skip=%u  Bkof=%u  DRST=%d/%d/%d%s%s%s%s",
		psDS248X->psI2C->DevIdx, StateName[psDS248X->State], StateName[NewState],
		psDS248X->ErrSupp[0], psDS248X->ErrSupp[1], psDS248X->ErrSupp[2], psDS248X->ErrSupp[3],
		psDS248X->ErrSupp[4], psDS248X->ErrSupp[5], psDS248X->ErrSupp[6], psDS248X->ErrSupp[7],
		psDS248X->XErrCnt, psDS248X->SkipCnt, psDS248X->BkofCnt, ResetOK, ResetErr, ResetBusy,
		psDS248X->LastMsg[0] ? "  last=" : "", psDS248X->LastMsg, caFirst,
		(NewState == ds248xSTATE_WEDGED) ? "  POWER CYCLE required, reboot cannot clear a DS2482" : "");
	psDS248X->ErrLogTick = now;
	psDS248X->PrvResetOK = (u32_t)ResetOK;
	psDS248X->PrvResetErr = (u32_t)ResetErr;
	memset(psDS248X->ErrSupp, 0, sizeof(psDS248X->ErrSupp));
	psDS248X->XErrCnt = psDS248X->SkipCnt = psDS248X->BkofCnt = 0;
	/* I5: the backoff arms/disarms HERE, the single place State commits. Entering WEDGED starts
	 * the ladder at MIN (the storm that got us here already proved full rate is useless);
	 * leaving it - which the sticky window above only allows after an error-free window -
	 * restores full-rate recovery for the next episode. */
	if (NewState == ds248xSTATE_WEDGED && psDS248X->State != ds248xSTATE_WEDGED) {
		psDS248X->BkofTick = now;
		psDS248X->BkofTicks = dsBACKOFF_MIN;
	} else if (NewState != ds248xSTATE_WEDGED) {
		psDS248X->BkofTicks = 0;
	}
	psDS248X->State = NewState;
	if (NewState == ds248xSTATE_OK) {
		psDS248X->LastMsg[0] = 0;
		#if (ds248xCHAN_ATTRIB > 0)
		psDS248X->FirstMsg[0] = 0;						// episode closed: re-arm the trigger latch
		#endif
	}
}

#if (ds248xCHAN_ATTRIB > 0)
/**
 * @brief	I-1: latch the FIRST fault of an error episode - the attribution datum. In-wedge data
 *			is uniform across channels (the die is latched); only the trigger names the channel.
 *			Cleared by ds248xReportHealth when the episode closes (State returns OK).
 */
static void ds248xFirstFault(ds248x_t * psDS248X, const char * pcMess) {
	if (psDS248X->FirstMsg[0])
		return;											// episode open, trigger already latched
	strncpy(psDS248X->FirstMsg, pcMess, sizeof(psDS248X->FirstMsg) - 1);
	psDS248X->FirstMsg[sizeof(psDS248X->FirstMsg) - 1] = 0;
	psDS248X->FirstChan = psDS248X->CurChan;
	psDS248X->FirstStat = psDS248X->Rstat;
	#if (ds248xSTAT_DEBUG > 0)
	psDS248X->FirstCmd = psDS248X->OpCmd;
	#endif
	psDS248X->FirstTick = xTaskGetTickCount();
}
#endif

/**
 * @brief	Record an error detected by ds248xCheckRead, then attempt device-level recovery
 * @param[in]	psDS248X required device control/config/status structure
 * @param[in]	specific error message to log
 * @return		result from ds248xReset, status of RST bit
 */
static int ds248xLogEvent(ds248x_t * psDS248X, char const * pcMess) {	// record + report, NO device reset
	u8_t ch = psDS248X->CurChan;
	++psDS248X->ErrSupp[ch];							// counted per channel, reported consolidated
	#if (ds248xCHAN_ATTRIB > 0)
	ds248xFirstFault(psDS248X, pcMess);
	#endif
	strncpy(psDS248X->LastMsg, pcMess, sizeof(psDS248X->LastMsg) - 1);
	psDS248X->LastMsg[sizeof(psDS248X->LastMsg) - 1] = 0;
	/* Per-EVENT detail at INFO: invisible at default thresholds and near free (xvSyslog drops it
	 * BEFORE formatting). Diagnosis mode = raise ioSLOGhi (and ioSLhost for remote) to 6: the raw
	 * per-channel event stream then flows 1:1, deliberately UNTHROTTLED - the severity gate is the
	 * rate control. The always-on device-level picture lives in ds248xReportHealth(). */
	SL_INFO("Dev=%d  Ch=%d  %s", psDS248X->psI2C->DevIdx, ch, pcMess);
	ds248xReportHealth(psDS248X);
	return 0;
}

static int ds248xLogError(ds248x_t * psDS248X, char const * pcMess) {
	ds248xLogEvent(psDS248X, pcMess);
	int iRV = ds248xReset(psDS248X);					// DRST reverts the device to POR-default (APU=0)
	// A DRST on an SD/OWB/CONF error clears the device config. If the device was already configured,
	// re-apply the configured state (APU=1) so it is not left silently at POR-default. NON-checking
	// write (ds248xWriteConfigRaw) => no ds248xCheckRead => no ds248xLogError re-entry => no
	// recursion. Skipped when CFGok==0 (identify, and ds248xConfig's own reset which re-configs
	// itself).
	if (iRV == 1 && psDS248X->psI2C->CFGok) {
		psDS248X->CfgSet.APU = 1;						// restore Active Pull-Up in the INTENDED config
		ds248xWriteConfigRaw(psDS248X, psDS248X->CfgSet);
	}
	return iRV;
}

#if (ds248xCHAN_ATTRIB > 0)
void ds248xLogCRC(u8_t DevNum, u8_t PhyBus) {
	ds248x_t * psDS248X = &psaDS248X[DevNum];
	++psDS248X->CRCerr[PhyBus];
	char caBuf[24];
	snprintfx(caBuf, sizeof(caBuf), "CRC Ch=%u #%u", PhyBus, psDS248X->CRCerr[PhyBus]);
	++psDS248X->ErrSupp[PhyBus];						// participates in the per-channel window counts
	strncpy(psDS248X->LastMsg, caBuf, sizeof(psDS248X->LastMsg) - 1);
	psDS248X->LastMsg[sizeof(psDS248X->LastMsg) - 1] = 0;
	ds248xFirstFault(psDS248X, caBuf);					// a CRC fail can BE the episode trigger
	SL_INFO("Dev=%d  Ch=%d  %s", psDS248X->psI2C->DevIdx, PhyBus, caBuf);	// diagnosis-mode stream
	ds248xReportHealth(psDS248X);						// immediate on escalation, windowed otherwise
}
#endif

/**
 * @brief	Monitor resuts from register changes to check for consistency
 * @param[in]	psDS248X pointer to device structure
 * @param[in]	Value (previously written) now to be verified
 * @return	0 if an error, 1 if all OK
 * @note	All logic relies on the fact that only certain bits can/should change in certain registers
 * 			Also, the register pointer value is critical to determine the logic of what needs to be checked
 * 
 */
static int ds248xCheckRead(ds248x_t * psDS248X, u8_t Value) {
	char caBuf[48];
	char * pcTmp = caBuf;
	int xLen;
	if (psDS248X->Rptr == ds248xREG_STAT) {				// STATus register
		#if (ds248xCHAN_ATTRIB > 0)
		if (psDS248X->Rstat == 0xFF) {					// all 8 bits set = floating dead-I2C read,
			++psDS248X->FFCnt;							//  NOT device state: keep it OUT of the
			snprintfx(caBuf, sizeof(caBuf), "STAT=xFF artifact #%u", psDS248X->FFCnt);
			goto err_exit;								//  SD/OWB statistics (c998 polluted them)
		}
		#endif
		// Short Detected (SD) check
		if (psDS248X->SD)			pcTmp = stpcpy(pcTmp, "SD ");
		// 1W Bus Busy (OWB) check
		if (psDS248X->OWB)			pcTmp = stpcpy(pcTmp, "OWB ");
		// if anything in buffer, append the status value
		if (pcTmp != caBuf) {
			xLen = pcTmp - caBuf;						// determine size used
			#if (ds248xSTAT_DEBUG > 0)					// add command + episode size
			++psDS248X->SDseq[psDS248X->CurChan];		// consecutive error on this channel
			if (psDS248X->SD)  ++psDS248X->SDtotal;		// lifetime SD count
			snprintfx(pcTmp, sizeof(caBuf)-xLen, "Cmd=x%02X Stat=x%02X seq=%u",
				psDS248X->OpCmd, psDS248X->Rstat, psDS248X->SDseq[psDS248X->CurChan]);
			#else
			snprintfx(pcTmp, sizeof(caBuf)-xLen, "Stat=x%02X", psDS248X->Rstat);
			#endif
			if (psDS248X->SD && psDS248X->OWB == 0)		// SD alone = external line condition (wet or
				return ds248xLogEvent(psDS248X, caBuf);	//  shorted reader): count+report, do NOT reset
			goto err_exit;
		}
		#if (ds248xSTAT_DEBUG > 0)
		psDS248X->SDseq[psDS248X->CurChan] = 0;			// clean read → close any error episode
		#endif
		// No error in STATus register
		#if	(appPRODUCTION == 0)
		if (xOptionGet(dbgDS248X) > 1) {
			const u8_t DS248Xmask[3] = { 0b00001111, 0b00111111, 0b11111111 };
			u8_t Mask = DS248Xmask[xOptionGet(dbgDS248X) - 1];
			u8_t StatX = psDS248X->PrvStat[psDS248X->CurChan];
			if ((psDS248X->Rstat & Mask) != (StatX & Mask)) {
				PX("D=%d  C=%u  x%02X->x%02X  ", psDS248X->psI2C->DevIdx, psDS248X->CurChan, StatX, psDS248X->Rstat);
				ds248xReportStatus(NULL, StatX, psDS248X->Rstat);
			}
		}
		psDS248X->PrvStat[psDS248X->CurChan] = psDS248X->Rstat;
		#endif
	} else if (psDS248X->Rptr == ds248xREG_CONF) {		// CONFiguration register
		if (Value == 0xC3) {							// ds2482CMD_CHSL / ds2484CMD_PADJ
			goto done;									// ignore
		}
		// try find out IF anything went wrong
		ds248x_conf_t sConf = { .Rconf = Value & 0x0F};	// discard the top nibble		
		if (psDS248X->OWS != sConf.OWS)					// All DS248x devices 1W Speed
			pcTmp = stpcpy(pcTmp, "OWS ");
		if (psDS248X->SPU != sConf.SPU)					// All DS248x devices Strong Pull Up
			pcTmp = stpcpy(pcTmp, "SPU ");
		if (psDS248X->psI2C->Type == i2cDEV_DS2484) {	// DS2484 only
			if (psDS248X->PDN != sConf.PDN)				// PullDown bit different?
				pcTmp = stpcpy(pcTmp, "PDN ");
		} else {										// DS2482-xxx devices
			if (psDS248X->PDN || sConf.PDN)				// PPM discontinued, should not be set in either value
				pcTmp = stpcpy(pcTmp, "PPM? ");
		}
		if (psDS248X->APU != sConf.APU)					// All DS248x devices Active Pull Up
			pcTmp = stpcpy(pcTmp, "APU ");
		xLen = pcTmp - caBuf;							// determine size used
		if (xLen) {
			IF_myASSERT(debugTRACK, xLen < sizeof(caBuf));
			snprintfx(pcTmp, sizeof(caBuf)-xLen, "W=x%X R=x%X", sConf.Rconf, psDS248X->Rconf);
			goto err_exit;
		}
		// No error in CONF....
		#if	(appPRODUCTION == 0)						// for DEVelopment builds
		if (xOptionGet(dbgDS248X)) {				// if debug option enabled
			u8_t ConfX = psDS248X->PrvConf[psDS248X->CurChan];	// if configuration different from previous
			if (psDS248X->Rconf != ConfX) {				// report old vs new config
				PX("Dev=%d  Ch=%u  x%02X->x%02X ", psDS248X->psI2C->DevIdx, psDS248X->CurChan, ConfX, psDS248X->Rconf);
				ds248xReportConfig(NULL, ConfX, psDS248X->Rconf);	// decode the changes
			}
		}
		psDS248X->PrvConf[psDS248X->CurChan] = psDS248X->Rconf;
		#endif
	} else if (psDS248X->Rptr == ds248xREG_CHAN) {		// CHANnel register...
		if (psDS248X->Rchan != ds248x_V2N[psDS248X->CurChan]) {	// and values don't match?
			#if (ds248xCHAN_ATTRIB > 0)
			if (psDS248X->Rchan == 0xFF)
				++psDS248X->FFCnt;						// same floating-read artifact class
			#endif
			snprintfx(caBuf, sizeof(caBuf)," CHAN (x%02X vs x%02X)", psDS248X->Rchan, ds248x_V2N[psDS248X->CurChan]);
			goto err_exit;
		}
	}
done:
	return 1;
err_exit:
	ds248xLogError(psDS248X, caBuf);
	return 0;
}

#if (benchTEST_DS248X_INJECT > 0)			// ####### bench DS248x fault injection (console c-L) #######
static u32_t ds248xInjectCnt = 0;			// upcoming SUCCESSFUL reads to poison with 0xFF
void ds248xFaultInject(u32_t Count) {		// fakes the c70c class: die ACKs everything, registers float
	ds248xInjectCnt = Count;
	SL_WARN("DS248x fault-inject %s (%lu reads)", Count ? "ARMED" : "disarmed", Count);
}
#endif

/**
 * @brief
 * @param
 * @return
 */
static int ds248xWriteDelayRead(ds248x_t * psDS248X, u8_t * pTxBuf, size_t TxSize, u32_t uSdly) {
	#if (ds248xSTAT_DEBUG > 0)
		psDS248X->OpCmd = pTxBuf[0];					// record command in flight for CheckRead diagnostics
	#endif
	#if (ds248xLOCK == ds248xLOCK_IO)
		xRtosSemaphoreTake(&psDS248X->mux, portMAX_DELAY);
	#endif
//	IF_SYSTIMER_START(debugTIMING, stDS248x);
	/* Snapshot Rptr ONCE. It was read twice here - for the destination address and again for the
	 * length - so a concurrent task changing it in between could hand the driver RegX[CONF] with
	 * the 5-byte PADJ length, writing 4 bytes past that member and over the neighbouring register
	 * mirrors. Rptr is a shared 3-bit field with no lock; the two reads must at least agree. */
	u8_t Rptr = psDS248X->Rptr;
	int iRV = halI2C_Queue(psDS248X->psI2C, i2cWDR_B, pTxBuf, TxSize, &psDS248X->RegX[Rptr],
		Rptr == ds248xREG_PADJ ? SO_MEM(ds248x_t, Rpadj) : 1, (i2cq_p1_t) uSdly, (i2cq_p2_t) NULL);
//	IF_SYSTIMER_STOP(debugTIMING, stDS248x);
	#if (benchTEST_DS248X_INJECT > 0)
	if (ds248xInjectCnt && iRV == erSUCCESS) {			// transport OK, data poisoned = the c70c wedge class
		--ds248xInjectCnt;
		psDS248X->RegX[Rptr] = 0xFF;
	}
	#endif
	if (iRV != erSUCCESS && psDS248X->psI2C->Test == 0) {
		/* Transport failure: previously INVISIBLE at this layer (CheckRead never runs when the
		 * transfer fails), so a fully dead bus produced no ds248x-side line at all. Counted here,
		 * surfaced as XErr= in the consolidated health line.
		 * "!=" not "<": esp_err_t codes are POSITIVE (c98c's ESP_ERR_INVALID_STATE = +259), only
		 * the internal er### codes are negative - "<" left XErr=0 while the bus failed 810x/min. */
		++psDS248X->XErrCnt;
		#if (ds248xCHAN_ATTRIB > 0)
		ds248xFirstFault(psDS248X, "XErr(transport)");
		#endif
		ds248xReportHealth(psDS248X);
	}
	#if (ds248xLOCK == ds248xLOCK_IO)
		xRtosSemaphoreGive(&psDS248X->mux);
	#endif
	return iRV;
}

/**
 * @brief		
 * @param[in]
 * @return	1 if OK, 0 if error
 */
static int ds248xWriteDelayReadCheck(ds248x_t * psDS248X, u8_t * pTxBuf, size_t TxSize, u32_t uSdly) {
	int iRV = ds248xWriteDelayRead(psDS248X, pTxBuf, TxSize, uSdly);
	return (iRV == erSUCCESS) ? ds248xCheckRead(psDS248X, (TxSize > 1) ? pTxBuf[1] : 0x0F) : 0;
}

/**
 * @brief	Set the Read Pointer and reads the register
 *			Once set the pointer remains static to allow reread of same register
 * @return	erFAILURE if invalid type/register combination; else
 * 			result from ds248xWriteDelayReadCheck()
 * @note	WWDR	100KHz	400KHz
 *			uS-----+------+-------+
 *			NS	0	300		75
 *			OD	0	300		75
 */
static int ds248xReadRegister(ds248x_t * psDS248X, u8_t Reg) {
	if (psDS248X->psI2C->Test)
		goto skip;
	// check for validity of CHAN (only DS2482-800) and PADJ (only DS2484)
	if ((Reg == ds248xREG_CHAN && psDS248X->psI2C->Type != i2cDEV_DS2482_800) ||
		(Reg == ds248xREG_PADJ && psDS248X->psI2C->Type != i2cDEV_DS2484)) {
		SL_ALRT("Invalid device/register combo Reg=%d (%s)", Reg, RegNames[Reg]);
		return erFAILURE;
	}
skip:
	u8_t cBuf[2] = { ds248xCMD_SRP, (~Reg << 4) | Reg };
	psDS248X->Rptr = Reg;
	return ds248xWriteDelayReadCheck(psDS248X, cBuf, sizeof(cBuf), 0);
}

/**
 * @brief	Write the configuration register WITHOUT read-back checking
 * @param	sConf - the config to send, BY VALUE (write lower nibble, upper nibble bitwise inverted)
 * @return	result from ds248xWriteDelayRead()
 * @note	Takes the config as a parameter rather than reading psDS248X->Rconf back out. Rconf
 *			lives in the RegX[] union - every CONF-register reply overwrites it and ds248xReset()
 *			zeroes it - so building the command from it could send something other than what the
 *			caller just decided. Callers pass psDS248X->CfgSet, which only they mutate. This does
 *			not make the driver lock free: Rptr sequencing is a separate shared-state problem,
 *			still covered by the bus lock.
 *			Non-checking => no ds248xCheckRead => safe from ds248xLogError (no recursion).
 *
 *	WWDR		100KHz	400KHz
 *				300uS	75uS
 *		uS-----+------+-------+
 *	NS	0		300		75
 *	OD	0		300		75
 */
static int ds248xWriteConfigRaw(ds248x_t * psDS248X, ds248x_conf_t sConf) {
	// Write configuration (Case A)
	//	S AD,0 [A] WCFG [A] CF [A] Sr AD,1 [A] [CF] A\ P
	//  [] indicates from slave
	//  CF configuration byte to write
	sConf.RES = 0;										// only APU/PDN/SPU/OWS are writable
	u8_t cBuf[2] = { ds248xCMD_WCFG , (~sConf.Rconf << 4) | sConf.Rconf };
	psDS248X->Rptr = ds248xREG_CONF;
	return ds248xWriteDelayRead(psDS248X, cBuf, sizeof(cBuf), 0);
}

/**
 * @brief	Write the configuration register and verify the device echoed it back
 * @return	1 if config written & response correct else 0 (see ds248xCheckRead)
 */
static int ds248xWriteConfig(ds248x_t * psDS248X, ds248x_conf_t sConf) {
	sConf.RES = 0;
	int iRV = ds248xWriteConfigRaw(psDS248X, sConf);
	return (iRV == erSUCCESS) ? ds248xCheckRead(psDS248X, sConf.Rconf) : 0;
}

// ################### Identification, Diagnostics & Configuration functions #######################

int ds248xReset(ds248x_t * psDS248X) {
	/* I5: WEDGED backoff - the single choke point every recovery path funnels through
	 * (ds248xLogError, ds248xConfig via halI2C_ResetSubSystem, CfgPend in BusSelect). c998 ran
	 * ~5 DRST cycles/sec for 56 minutes against a die only a power cycle can clear, and that
	 * storm is what fed the fragile panic path. While WEDGED, allow one attempt per interval
	 * (MIN doubling to MAX), fail everything else fast with NO I2C traffic. Suppressed calls
	 * are counted (Bkof= in the health line), not silent. Identify-time probing (Test) and the
	 * disarmed state (BkofTicks==0, degenerate: always allowed) bypass cleanly. */
	if (psDS248X->State == ds248xSTATE_WEDGED && psDS248X->psI2C->Test == 0 && psDS248X->BkofTicks) {
		u32_t tNow = xTaskGetTickCount();
		if ((u32_t)(tNow - psDS248X->BkofTick) < psDS248X->BkofTicks) {
			if (psDS248X->BkofCnt < 65535)
				++psDS248X->BkofCnt;
			return 0;									// fail fast: no DRST, no delays, no config
		}
		psDS248X->BkofTick = tNow;
		psDS248X->BkofTicks = (psDS248X->BkofTicks >= dsBACKOFF_MAX / 2) ? dsBACKOFF_MAX
															: psDS248X->BkofTicks * 2;
	}
	const u8_t cmdDRST = ds248xCMD_DRST;
	int Retries = 0, iRV;
	psDS248X->Rptr = ds248xREG_STAT;					// After ReSeT pointer set to STATus register
	do {
		iRV = ds248xWriteDelayRead(psDS248X, (u8_t *) &cmdDRST, sizeof(u8_t), 0);
		/* Transport FAILED => the read never wrote RegX[ds248xREG_STAT], so Rstat still holds the
		 * PREVIOUS reading. Interpreting it reports stale bits as current device state: a stale
		 * RST=1 was counted as ResetOK and logged "Success" while the bus was dead, and a stale
		 * OWB=1 could never clear (a fresh value is never read), so the wedge became self-
		 * sustaining in software regardless of what the hardware was doing. Discard instead. */
		if (iRV != erSUCCESS) {
			psDS248X->Rstat = 0;						// RST=0 => reported+counted as a real failure
			break;										// retrying a dead transport just burns time
		}
		if (psDS248X->Rstat == 0xFF) {					// floating dead-I2C read, NOT device state
			#if (ds248xCHAN_ATTRIB > 0)
			++psDS248X->FFCnt;
			#endif
			psDS248X->Rstat = 0;						// RST=0 => reported+counted as a real failure
			break;										// more DRSTs at a dead bus just burn time
		}
		if (psDS248X->RST && psDS248X->OWB == 0)		// ReSeT successful AND 1W engine left idle?
			break;										// exit to complete
		vTaskDelay(pdMS_TO_TICKS(10));					// else spend the remaining attempt(s) trying
	} while(++Retries < 2);								//  to shake 1WB loose with another DRST
	/* A DRST that reports success but leaves 1WB set means the device is wedged. That is a DEVICE
	 * fault to report, NOT a firmware invariant: this was an IF_myASSERT(debugTRACK, OWB == 0),
	 * which abort()ed and reboot-looped field units once their DS2482 wedged (v0.6.1.4 c764, 11
	 * coredumps in 12 minutes). Deliberately non-fatal now - the device is left configured and
	 * usable, and recovery is re-attempted on every subsequent I2C error via
	 * halI2C_ResetSubSystem, so a stray failure self-heals without taking the unit offline. */
	bool bBusy = (psDS248X->OWB != 0);
	if (psDS248X->RST) {
		++ResetOK;										// yes, update counter
		if (bBusy)
			++ResetBusy;								// present but wedged: telemetry, not fatal
		// set register mirrors & variables to defaults
		psDS248X->CurChan = 0;							// all device, common requirement
		psDS248X->Rdata = 0;
		psDS248X->Rconf = 0;							// all bits cleared (default) config
		if (psDS248X->psI2C->Type == i2cDEV_DS2482_800)	// DS2482-800 specific
			psDS248X->Rchan = ds248x_V2N[0];
		if (psDS248X->psI2C->Type == i2cDEV_DS2484)		// DS2484 specific
			memset(psDS248X->Rpadj, 0, SO_MEM(ds248x_t, Rpadj));
	} else {
		++ResetErr;										// update FAIL counter
		// possibly do hardware reset/reboot?
	}
	/* No logging HERE, by design: DRST outcomes are folded into the consolidated device health
	 * line (ds248xReportHealth: DRST=OK/Err/Busy plus window deltas). This replaces the previous
	 * ~30-line fault/change/recovered machinery - the reporter's state hysteresis provides the
	 * same change-only + rate-limited + recovery-one-shot behaviour generically (see the c764
	 * flood history in git for why per-event logging here is forbidden). Failures escalate and
	 * report immediately; a success only matters when an error episode is open (State != OK),
	 * letting the reporter close it. 'Retries' stays out of any test for the same c764 reason. */
	(void) Retries;
	if (psDS248X->RST == 0 || bBusy || psDS248X->State)
		ds248xReportHealth(psDS248X);
	return psDS248X->RST;
}

int	ds248xIdentify(i2c_di_t * psI2C) {
	ds248x_t sDS248X = { 0 };							// temporary device structure
	sDS248X.psI2C = psI2C;
	psI2C->Speed = i2cSPEED_100;						// v2 driver 400KHz marginality on classic ESP32 (esp-idf #14401); v1-era margins
	psI2C->TObus = 25;
	psI2C->Test	= 1;
	psI2C->Type = i2cDEV_UNDEF;							// unidentified at this stage
	int iRV;
	if (ds248xReset(&sDS248X) == 1) {
		iRV = ds248xReadRegister(&sDS248X, ds248xREG_PADJ);
		// Read PADJ=OK with PAR=000 & OD=0, valid DS2484
		if (iRV == 1 &&	sDS248X.Rpadj[0] == 0b00000110) {
			psI2C->Type = i2cDEV_DS2484;				// definite DS2484
			goto done;
		}		
		iRV = ds248xReadRegister(&sDS248X, ds248xREG_CHAN);
		// -10x CSR should fail, -800 should succeed...
		if (iRV != 1) {									// CSR read FAIL
			psI2C->Type = i2cDEV_DS2482_10X;			// Must be DS2482-10X
		} else if (sDS248X.Rchan == ds248x_V2N[0]) {	// CSR read OK, CHAN=0 default
			psI2C->Type = i2cDEV_DS2482_800;			// Must be DS2482-800
		} else {
			// remain an unidentified device.....
		}
	} else {
		SL_ALRT("Dev=%d  Ch=%d  Missing/faulty DS248x !!!", sDS248X.psI2C->DevIdx, sDS248X.CurChan);
	}
done:
	#if (ds248xLOCK == ds248xLOCK_IO)					/* if locking enabled.... */
		if (sDS248X.mux)								/* mux will be initialised in ds248xReset() */
			vSemaphoreDelete(sDS248X.mux);				/* thus delete and free up allocation */
	#endif
	psI2C->IgnoreACK = 0;
	if (psI2C->Type == i2cDEV_UNDEF)
		return erINV_DEVICE;
	psI2C->DevIdx = ds248xCount++;
	psI2C->IDok = 1;
	psI2C->Test	= 0;
	return erSUCCESS;
}

int	ds248xConfig(i2c_di_t * psI2C) {
	if (psI2C->IDok == 0)
		return erINV_STATE;
	if (psaDS248X == NULL) {
		IF_myASSERT(debugPARAM, psI2C->DevIdx == 0);
		psaDS248X = malloc(ds248xCount * sizeof(ds248x_t));
		if (psaDS248X == NULL)
			return erNO_MEM;
		memset(psaDS248X, 0, ds248xCount * sizeof(ds248x_t));
//		IF_SYSTIMER_INIT(debugTIMING, stDS248x, stMICROS, "DS248x", 300, 18000)
	}
	ds248x_t * psDS248X = &psaDS248X[psI2C->DevIdx];
	if (psI2C->CFGok == 0) {							// definite 1st time for specific device...
		psDS248X->psI2C = psI2C;
		if (psI2C->Type == i2cDEV_DS2482_800) {
			psDS248X->NumChan = 1;						// 0=1Ch, 1=8Ch
		}
		#if (HAL_DS18X20 > 0)
			void ds18x20StepThreeRead(TimerHandle_t);
			psDS248X->th = xTimerCreateStatic("tmrDS248x", pdMS_TO_TICKS(5), pdFALSE, NULL, ds18x20StepThreeRead, &psDS248X->ts);
		#endif
	}

	/* Recovery MUST participate in the same lock the 1-Wire transaction path uses, or it corrupts
	 * Rptr/Rconf mid transaction - that IS the race. Owner aware, because recovery runs INLINE on
	 * whichever task hit the error (hal_i2c_common.c, i2cInlineDepth): if that task already holds
	 * the mutex, taking a PLAIN mutex (xSemaphoreCreateMutex) a second time self deadlocks - not a
	 * rare interleaving, but EVERY time an error occurs inside a locked transaction.
	 * Timeout is the device's OWN bus timeout: between transfers we get it immediately; mid
	 * transfer we wait no longer than that transfer is already permitted to take. Recovery can
	 * therefore never stall I2C longer than one ordinary transfer already can. */
	#if (ds248xLOCK == ds248xLOCK_BUS)
		BaseType_t bHeld = xRtosSemaphoreCheckCurrent(&psDS248X->mux);
		if (bHeld == pdFALSE &&
			xRtosSemaphoreTake(&psDS248X->mux, pdMS_TO_TICKS(psI2C->TObus)) != pdTRUE) {
			/* Held elsewhere: state untouched, but COUNTED - a silently skipped recovery is an
			 * invisible no-op (the c98c lesson: diagnosable only from Papertrail history). The
			 * skip surfaces as Skip= in the consolidated health line. */
			++psDS248X->SkipCnt;
			#if (ds248xCHAN_ATTRIB > 0)
			ds248xFirstFault(psDS248X, "Skip(recovery)");
			#endif
			ds248xReportHealth(psDS248X);
			psDS248X->CfgPend = 1;			// defer: next BusSelect re-runs config under the lock it holds
			return erBUSY;
		}
	#endif

	psI2C->CFGok = 0;
	int iRV = ds248xReset(psDS248X);
	if (iRV != 1) {
		halEventUpdateDevice(devMASK_DS248X, 0);
		iRV = erINV_DEVICE;
		goto exit;										// was a bare return - would leak the mutex
	}
	psDS248X->CfgSet.Rconf = 0;							// DRST above left the device at POR default
	psDS248X->CfgSet.APU = 1;							// Even though only single slave ALWAYS enabled
	iRV = ds248xWriteConfig(psDS248X, psDS248X->CfgSet);
	if (iRV != 1) {										// returns 1/0, never negative: the < erSUCCESS test always passed
		iRV = erINV_CONFIG;								// propagate a real failure, CFGok stays 0
		goto exit;
	}
	psI2C->CFGok = 1;
	halEventUpdateDevice(devMASK_DS248X, 1);
	#if (ds248xCHAN_ATTRIB > 0)
	psDS248X->AuditPend = 1;							// I-3: baseline audit after boot/recovery config
	#endif
exit:
	#if (ds248xLOCK == ds248xLOCK_BUS)
		if (bHeld == pdFALSE)
			xRtosSemaphoreGive(&psDS248X->mux);
	#endif
	return iRV;
}

// ################################## DS248x-x00 1-Wire functions ##################################

int	ds248xBusSelect(ds248x_t * psDS248X, u8_t Bus) {
	/* I5b: fast-fail the whole scan while WEDGED+backoff - only DRSTs were throttled, the Sense
	 * sweep kept 8 CHSL/sec hitting the dead device. Lock-free; expiry falls through so the one
	 * probe select fails into ds248xReset, which owns advancing the backoff. */
	if (psDS248X->State == ds248xSTATE_WEDGED && psDS248X->psI2C->Test == 0 && psDS248X->BkofTicks) {
		if ((u32_t)(xTaskGetTickCount() - psDS248X->BkofTick) < psDS248X->BkofTicks) {
			if (psDS248X->BkofCnt < 65535)
				++psDS248X->BkofCnt;
			return 0;
		}
	}
	int iRV = 1;
	#if (ds248xLOCK == ds248xLOCK_BUS)
		xRtosSemaphoreTake(&psDS248X->mux, portMAX_DELAY);
		/* Recovery that was skipped because WE (or a predecessor) held the lock mid-transaction.
		 * Run it now, BEFORE the channel select: ds248xConfig's DRST resets CurChan to 0, so the
		 * select below then proceeds from known-good state. The owner-aware take inside
		 * ds248xConfig sees the mutex we hold (CheckCurrent) and runs inline - no deadlock, no
		 * double-take, and its result is deliberately ignored: a failing device produces its own
		 * (throttled) log lines, and the select below fails loudly on its own. */
		if (psDS248X->CfgPend) {
			psDS248X->CfgPend = 0;
			ds248xConfig(psDS248X->psI2C);
		}
	#endif
	if ((psDS248X->psI2C->Type == i2cDEV_DS2482_800) &&
		((psDS248X->CurChan != Bus) || (psDS248X->State != ds248xSTATE_OK))) {	// skip-if-same only while OK: post-DRST CurChan=0 fabricated Ch0 SEL on a dead device
		/* Channel Select (Case A)
		 *	S AD,0 [A] CHSL [A] CC [A] Sr AD,1 [A] [RR] A\ P
		 *  [] indicates from slave
		 *  CC channel value
		 *  RR channel read back
		 */
		u8_t cBuf[2] = { ds2482CMD_CHSL, (~Bus << 4) | Bus };	// calculate Channel value
		psDS248X->Rptr = ds248xREG_CHAN;
		psDS248X->CurChan = Bus;						// save in advance will auto reset if error
		iRV = ds248xWriteDelayReadCheck(psDS248X, cBuf, sizeof(cBuf), 0);
	}
	#if (ds248xLOCK == ds248xLOCK_BUS)
		/* Release on ANY non success, not just 0. Success is 1 - either no IO was needed or the
		 * channel select succeeded - and only then does the caller own the lock through to
		 * ds248xBusRelease(). Testing "== 0" leaked the mutex for any other value, and a leak
		 * here is unrecoverable: every later BusSelect on that device blocks forever. */
		if (iRV != 1)
			xRtosSemaphoreGive(&psDS248X->mux);			// release the lock...
	#endif
	/* Recovery assessment: ReportHealth was only ever called from ERROR paths, so a device whose
	 * errors STOP could never close its episode - ERRORS stuck open forever, WedgeCnt stale across
	 * quiet gaps, and a recovered WEDGED device stayed scan-throttled for life. A successful select
	 * while degraded feeds the reporter; its own window gating keeps this near-free (~8/s compares,
	 * one emission per clean window -> WEDGED/ERRORS -> OK, backoff disarms at the commit). */
	if (iRV == 1 && psDS248X->State != ds248xSTATE_OK)
		ds248xReportHealth(psDS248X);
	return iRV;
}

void ds248xBusRelease(ds248x_t * psDS248X) {
	#if (ds248xLOCK == ds248xLOCK_BUS)
		xRtosSemaphoreGive(&psDS248X->mux);
	#endif
}

int	ds248xOWReset(ds248x_t * psDS248X) {
	// DS2482-800 datasheet page 7 para 2
	if (psDS248X->CfgSet.SPU == owPOWER_STRONG)			// INTENT, not the mirror: a clobbered mirror
		ds248xOWLevel(psDS248X, owPOWER_STANDARD);		// would skip dropping a pull-up that IS still on
	// 1-Wire reset (Case B)
	//	S AD,0 [A] 1WRS [A] Sr AD,1 [A] [Status] A [Status] A\ P
	//									\--------/
	//						Repeat until 1WB bit has changed to 0
	//  [] indicates from slave
	const u8_t cmd1WRS = ds248xCMD_1WRS;
	psDS248X->Rptr = ds248xREG_STAT;
	ds248xWriteDelayReadCheck(psDS248X, (u8_t *) &cmd1WRS, sizeof(u8_t), psDS248X->CfgSet.OWS ? owDELAY_RST_OD : owDELAY_RST);
	#if (ds248xSTAT_DEBUG > 0)						// poll rate, and how often anything answered
	++psDS248X->RstCnt[psDS248X->CurChan];
	if (psDS248X->PPD)
		++psDS248X->PPDcnt[psDS248X->CurChan];
	#endif
	return psDS248X->PPD;
}

/* Speed/Level record the INTENT in CfgSet then command it. The return value stays the read-back
 * bitfield: the caller asked what the device ended up at, and CheckRead has already flagged any
 * mismatch. Only the decision of WHAT to send comes from CfgSet. */
int	ds248xOWSpeed(ds248x_t * psDS248X, bool speed) {
	psDS248X->CfgSet.OWS = speed;
	ds248xWriteConfig(psDS248X, psDS248X->CfgSet);
	return psDS248X->OWS;
}

int	ds248xOWLevel(ds248x_t * psDS248X, bool level) {
	psDS248X->CfgSet.SPU = level;
	ds248xWriteConfig(psDS248X, psDS248X->CfgSet);
	return psDS248X->SPU;
}

bool ds248xOWTouchBit(ds248x_t * psDS248X, bool Bit) {
	// 1-Wire bit (Case B)
	//	S AD,0 [A] 1WSB [A] BB [A] Sr AD,1 [A] [Status] A [Status] A\ P
	//										   \--------/
	//								Repeat until 1WB bit has changed to 0
	//  [] indicates from slave
	//  BB indicates byte containing bit value in msbit
	u8_t cBuf[2] = { ds248xCMD_1WSB, Bit << 7 };
	psDS248X->Rptr = ds248xREG_STAT;
	ds248xWriteDelayReadCheck(psDS248X, cBuf, sizeof(cBuf), psDS248X->CfgSet.OWS ? owDELAY_SB_OD : owDELAY_SB);
	return psDS248X->SBR;
}

u8_t ds248xOWWriteByte(ds248x_t * psDS248X, u8_t Byte) {
	// 1-Wire Write Byte (Case B)
	//	S AD,0 [A] 1WWB [A] DD [A] Sr AD,1 [A] [Status] A [Status] A\ P
	//										   \--------/
	//							Repeat until 1WB bit has changed to 0
	//  [] indicates from slave
	//  DD data to write
	u8_t cBuf[2] = { ds248xCMD_1WWB, Byte };
	psDS248X->Rptr = ds248xREG_STAT;
	ds248xWriteDelayReadCheck(psDS248X, cBuf, sizeof(cBuf), psDS248X->CfgSet.OWS ? owDELAY_WB_OD : owDELAY_WB);
	return psDS248X->Rstat;
}

u8_t ds248xOWReadByte(ds248x_t * psDS248X) {
	// 1-Wire Read Bytes (Case C)
	//	S AD,0 [A] 1WRB [A] Sr AD,1 [A] [Status] A [Status] A\ '
	//										\--------/
	//							Repeat until 1WB bit has changed to 0
	//	Sr AD,0 [A] SRP [A] E1 [A] Sr AD,1 [A] DD A\ P
	//  [] indicates from slave
	//  DD data read
	const u8_t cmd1WRB = ds248xCMD_1WRB;
	psDS248X->Rptr = ds248xREG_STAT;
	ds248xWriteDelayReadCheck(psDS248X, (u8_t *) &cmd1WRB, sizeof(u8_t), psDS248X->CfgSet.OWS ? owDELAY_RB_OD : owDELAY_RB);
	ds248xReadRegister(psDS248X, ds248xREG_DATA);
	return psDS248X->Rdata;
}

u8_t ds248xOWSearchTriplet(ds248x_t * psDS248X, u8_t u8Dir) {
	// 1-Wire Triplet (Case B)
	//	S AD,0 [A] 1WT [A] SS [A] Sr AD,1 [A] [Status] A [Status] A\ P
	//							  \--------/
	//				Repeat until 1WB bit has changed to 0
	//  [] indicates from slave
	//  SS indicates byte containing search direction bit value in msbit
	u8_t cBuf[2] = { ds248xCMD_1WT, u8Dir ? 0x80 : 0x00 };
	psDS248X->Rptr = ds248xREG_STAT;
	ds248xWriteDelayReadCheck(psDS248X, cBuf, sizeof(cBuf), psDS248X->CfgSet.OWS ? owDELAY_ST_OD : owDELAY_ST);
	#if (ds248xSTAT_DEBUG > 0)						// enumeration workload (64 triplets per ROM found)
	++psDS248X->TripCnt[psDS248X->CurChan];
	#endif
	return psDS248X->Rstat;
}

// #################################### DS248x debug/reporting #####################################

#if (ds248xCHAN_ATTRIB > 0)
void ds248xAuditRun(void) {
	for (int i = 0; i < ds248xCount; ++i) {
		ds248x_t * psDS248X = &psaDS248X[i];
		if (psDS248X->AuditPend == 0 || psDS248X->psI2C->Type != i2cDEV_DS2482_800)
			continue;
		psDS248X->AuditPend = 0;
		u8_t SEL = 0, LL = 0, SD = 0, PPD = 0;			// bit N = channel N
		for (u8_t Ch = 0; Ch < 8; ++Ch) {
			if (ds248xBusSelect(psDS248X, Ch) != 1)
				continue;								// select failed: absent from SEL map
			SEL |= (1 << Ch);
			if (ds248xReadRegister(psDS248X, ds248xREG_STAT) == 1 && psDS248X->LL)
				LL |= (1 << Ch);						// idle line level; 0 = held LOW = short/leakage
			ds248xOWReset(psDS248X);					// 1WRS: SD sampled during reset, PPD = presence
			if (psDS248X->SD)  SD  |= (1 << Ch);
			if (psDS248X->PPD) PPD |= (1 << Ch);
			ds248xBusRelease(psDS248X);
		}
		/* Severity escalates with content: a clean audit is a NOTICE, one with a suspect channel
		 * (selected but line idles LOW, or short detected during reset) must reach the host. */
		u8_t Suspect = (SEL & ~LL) | SD;
		char caSus[16] = "";
		if (Suspect)
			snprintfx(caSus, sizeof(caSus), "  SUSPECT=x%02X", Suspect);
		SL_LOG(Suspect ? SL_SEV_WARNING : SL_SEV_NOTICE,
			"Audit Dev=%d  SEL=x%02X  LL=x%02X  SD=x%02X  PPD=x%02X%s",
			psDS248X->psI2C->DevIdx, SEL, LL, SD, PPD, caSus);
	}
}
#endif

int ds248xReportStatus(report_t * psR, u8_t Val1, u8_t Val2) {
	const char * const StatNames[8] = { "OWB", "PPD", "SD", "LL", "RST", "SBR", "TSB", "DIR" };
	return xReportBitMap(psR, Val1, Val2, 0x000000FF, StatNames);
}

int ds248xReportConfig(report_t * psR, u8_t Val1, u8_t Val2) {
	const char * const ConfNames[4] = { "APU", "PDN", "SPU", "OWS" };
	return xReportBitMap(psR, Val1, Val2, 0x0000000F, ConfNames);
}

int	ds248xReportRegister(report_t * psR, ds248x_t * psDS248X, int Reg) {
	int iRV = 0, Chan;
	switch (Reg) {
	case ds248xREG_STAT: {
		#if	(appPRODUCTION == 0)
			iRV += xReport(psR, "STAT(0)");
			for (int i = 0; i < (psDS248X->NumChan ? 8 : 1); ++i) {
				iRV += xReport(psR, "\t#%u:", i, psDS248X->PrvStat[i]);
				iRV += ds248xReportStatus(psR, 0, psDS248X->PrvStat[i]);
			}
		#endif
		break;
	}
	case ds248xREG_DATA: {
		iRV += xReport(psR, "DATA(1)=0x%02X (Last read)\r\n", psDS248X->Rdata);
		break;
	}
	case ds248xREG_CHAN: {
		if (psDS248X->psI2C->Type != i2cDEV_DS2482_800)
			break;
		// Channel, start by finding the matching Channel #
		for (Chan = 0; Chan < (psDS248X->NumChan ? 8 : 1) && psDS248X->Rchan != ds248x_V2N[Chan]; ++Chan);
		if (Chan >= (psDS248X->NumChan ? 8 : 1)) {		// no match: corrupted/unrecognized channel read-back
			SL_LOG(SL_SEV_CRITICAL, "DS2482 CHAN read-back x%02X unrecognized", psDS248X->Rchan);
			Chan = 8;									// clamp to error-sentinel slot (no reboot, no OOB)
		}
		iRV += xReport(psR, "CHAN(2)=0x%02X Chan=%d Xlat=0x%02X\r\n", psDS248X->Rchan, Chan, ds248x_V2N[Chan]);
		break;
	}
	case ds248xREG_CONF: {
		iRV += xReport(psR, "CONF(3)=0x%02X  ", psDS248X->Rconf);
		iRV += ds248xReportConfig(psR, 0, psDS248X->Rconf);
		break;
	}
	case ds248xREG_PADJ: {
		if (psDS248X->psI2C->Type != i2cDEV_DS2484)
			break;
		ds248xReadRegister(psDS248X, Reg);
		ds248x_padj_t sPadj;
		sPadj.RadjX = psDS248X->Rpadj[0];
		iRV += xReport(psR, "PADJ(4)=0x%02X  OD=%c | tRSTL=%duS", sPadj.RadjX,
				sPadj.OD ? CHR_1 : CHR_0, Trstl[sPadj.VAL] * (sPadj.OD ? 1 : 10));
		sPadj.RadjX = psDS248X->Rpadj[1];
		iRV += xReport(psR, " | tMSP=%.1fuS", sPadj.OD ? (double) Tmsp1[sPadj.VAL] / 10.0 : (double) Tmsp0[sPadj.VAL]);
		sPadj.RadjX = psDS248X->Rpadj[2];
		iRV += xReport(psR, " | tWOL=%.1fuS", sPadj.OD ? (double) Twol1[sPadj.VAL] / 10.0 : (double) Twol0[sPadj.VAL]);
		sPadj.RadjX = psDS248X->Rpadj[3];
		iRV += xReport(psR, " | tREC0=%.2fuS", (double) Trec0[sPadj.VAL] / 100.0);
		sPadj.RadjX = psDS248X->Rpadj[4];
		iRV += xReport(psR, " | rWPU=%f ohm\r\n", (double) Rwpu[sPadj.VAL]);
		break;
	}
	}
	return iRV;
}

int ds248xReport(report_t * psR, ds248x_t * psDS248X) {
	int iRV = halI2C_DeviceReport(psR, (void *) psDS248X->psI2C);
	for (int Reg = 0; Reg < ds248xREG_NUM; ++Reg) iRV += ds248xReportRegister(psR, psDS248X, Reg);
	#if (ds248xCHAN_ATTRIB > 0)
		iRV += xReport(psR, "FF=%u  CRCerr=%u/%u/%u/%u/%u/%u/%u/%u\r\n", psDS248X->FFCnt,
			psDS248X->CRCerr[0], psDS248X->CRCerr[1], psDS248X->CRCerr[2], psDS248X->CRCerr[3],
			psDS248X->CRCerr[4], psDS248X->CRCerr[5], psDS248X->CRCerr[6], psDS248X->CRCerr[7]);
	#endif
	#if (ds248xSTAT_DEBUG > 0)
		iRV += xReport(psR, "SDtotal=%u  supp=%u/%u/%u/%u/%u/%u/%u/%u\r\n", psDS248X->SDtotal,
			psDS248X->ErrSupp[0], psDS248X->ErrSupp[1], psDS248X->ErrSupp[2], psDS248X->ErrSupp[3],
			psDS248X->ErrSupp[4], psDS248X->ErrSupp[5], psDS248X->ErrSupp[6], psDS248X->ErrSupp[7]);
		const char * const ActNames[4] = { "Rst", "PPD", "Trip", "Tag" };
		const u32_t * const ActCnts[4] = { psDS248X->RstCnt, psDS248X->PPDcnt, psDS248X->TripCnt, psDS248X->TagCnt };
		for (int a = 0; a < 4; ++a)					// per-channel activity: Ch0 -> Ch7
			iRV += xReport(psR, "%s=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu\r\n", ActNames[a],
				ActCnts[a][0], ActCnts[a][1], ActCnts[a][2], ActCnts[a][3],
				ActCnts[a][4], ActCnts[a][5], ActCnts[a][6], ActCnts[a][7]);
	#endif
	#if (HAL_DS18X20 > 0)
		iRV += xRtosReportTimer(psR, psDS248X->th);
	#endif
	return iRV;
}

int ds248xReportAll(report_t * psR) {
	int iRV = 0;
	for (int i = 0; i < ds248xCount; iRV += ds248xReport(psR, &psaDS248X[i++]));
	return iRV;
}

#endif
