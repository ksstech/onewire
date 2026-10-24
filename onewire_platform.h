// onewire_platform.h

#pragma once

#if (appUSE_ENDPOINTS > 0)
	#include "endpoints.h"					// epw_t/URI world - only for endpoint-integrated apps
#endif

#include "priv/onewire.h"
#include "priv/ds248x.h"
#include "priv/ds1990x.h"
#if (HAL_DS18X20 > 0)
	#include "priv/ds18x20.h"				// header uses epw_t (endpoints) unconditionally
#endif
#include "priv/esp_rmt.h"

#ifdef __cplusplus
extern "C" {
#endif

// ############################################# Macros ############################################

// ######################################## Enumerations ###########################################

enum { owHELD_UNKNOWN = 0, owHELD_GRANTED, owHELD_DENIED };	// owbi_t.LastValid, set by the rules engine

// ######################################### Structures ############################################

/* Bus related info, ie last device read (ROM & timestamp)
 * Used to avoid re-reading a device (primarily DS1990X type) too regularly.
 */
typedef struct __attribute__((packed)) owbi_t {
	seconds_t	LastRead;			// size=4
	ow_rom_t	LastROM;			// size=8
	union __attribute__((packed)) {
		struct { u8_t ds18b20:4, ds18s20:4; };
		u8_t ds18any;
	};
	// Held-tag detector (ds1990x.c): same ROM present every sweep for longer than dlyDS1990
	seconds_t	HeldLog;			// last HELD syslog line, paces the ioHeldRem reminder
	union __attribute__((packed)) {
		struct __attribute__((packed)) {
			u32_t	Held:1;			// 1 = HELD: repeat reads suppressed while ioHeldTag
			u32_t	LastValid:2;	// owHELD_UNKNOWN/GRANTED/DENIED for the last accepted read
			u32_t	HeldCnt:29;		// consecutive sweeps the same ROM was present
		};
		u32_t	HeldAll;
	};
} owbi_t;
DUMB_STATIC_ASSERT(sizeof(owbi_t) == 21);

// #################################### Public Data structures #####################################

// ###################################### Public functions #########################################

owbi_t * psOWP_BusGetPointer(u8_t);
int OWP_GetNumBus(void);
u32_t OWP_GetScanned(void);			// bit per logical bus whose select succeeded in the last OWP_Scan
void OWP_BusL2P(owdi_t *, u8_t);
int	OWP_BusP2L(owdi_t *);
int	OWP_BusSelect(owdi_t *);
void OWP_BusRelease(owdi_t *);

// Common callback handlers
int	OWP_PrintROM_CB(struct report_t * psR, ow_rom_t * psROM);
int	OWP_Print1W_CB(struct report_t * psR, owdi_t * psOW);
int	OWP_PrintChan_CB(struct report_t * psR, owbi_t * psCI);
int	OWP_Count_CB(struct report_t * psR, owdi_t *);

int	OWP_Scan(u8_t, int (*)(struct report_t *, owdi_t *));
int	OWP_Scan2(u8_t, int (*)(struct report_t *, void *, owdi_t *), void *);
int	OWP_ScanAlarmsFamily(u8_t Family);

int	OWP_Config(void);
int OWP_Report(struct report_t * psR);

#ifdef __cplusplus
}
#endif
