/*
 * Copyright 2018-21 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma		once

#include	"onewire.h"

#ifdef __cplusplus
extern "C" {
#endif

// ############################################# Macros ############################################


// ######################################## Enumerations ###########################################


// ######################################### Structures ############################################

/* Bus related info, ie last device read (ROM & timestamp)
 * Used to avoid re-reading a device (primarily DS1990X type) too regularly.
 */
typedef struct owbi_t {
	seconds_t	LastRead;			// size=4
	ow_rom_t	LastROM;			// size=8
	union {
		struct { u8_t ds18b20:4, ds18s20:4; };
		u8_t ds18any;
	};
} owbi_t;
//DUMB_STATIC_ASSERT(sizeof(owbi_t) == 13);

// #################################### Public Data structures #####################################


// ###################################### Public functions #########################################

owbi_t * psOWP_BusGetPointer(u8_t) ;
void OWP_BusL2P(owdi_t *, u8_t) ;
int	OWP_BusP2L(owdi_t *) ;
int	OWP_BusSelect(owdi_t *) ;
void OWP_BusRelease(owdi_t *) ;

// Common callback handlers
int	OWP_PrintROM_CB(fm_t FlagMask, ow_rom_t * psROM) ;
int	OWP_Print1W_CB(fm_t FlagMask, owdi_t * psOW) ;
int	OWP_PrintChan_CB(fm_t FlagMask, owbi_t * psCI) ;
int	OWP_Count_CB(fm_t FlagMask, owdi_t *) ;

int	OWP_Scan(u8_t, int (*)(fm_t, owdi_t *)) ;
int	OWP_Scan2(u8_t, int (*)(fm_t, void *, owdi_t *), void *) ;
int	OWP_ScanAlarmsFamily(u8_t Family) ;

int	OWP_Config(void) ;
void OWP_Report(void) ;

#ifdef __cplusplus
}
#endif
