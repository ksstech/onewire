/*
 * Copyright 2018-21 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma		once

#include	"onewire.h"
#include	"ds1990x.h"
#include	"ds18x20.h"

#ifdef __cplusplus
extern "C" {
#endif

// ############################################# Macros ############################################

#define	ds1990READ_INTVL			5					// successive read interval, avoid duplicates

// ######################################## Enumerations ###########################################


// ######################################### Structures ############################################

/* Bus related info, ie last device read (ROM & timestamp)
 * Used to avoid re-reading a device (primarily DS1990X type) too regularly.
 */
typedef struct __attribute__((packed)) owbi_t {
	ow_rom_t	LastROM ;
	seconds_t	LastRead ;
	union {
		struct __attribute__((packed)) {
			uint8_t		ds18b20	: 4 ;
			uint8_t		ds18s20	: 4 ;
			uint8_t		spare ;
		} ;
		uint16_t	ds18any ;
	} ;
} owbi_t ;
DUMB_STATIC_ASSERT(sizeof(owbi_t) == 14) ;

// #################################### Public Data structures #####################################

extern	owbi_t * psaOWBI ;
extern ow_flags_t OWflags;

// ###################################### Public functions #########################################

owbi_t * psOWP_BusGetPointer(uint8_t) ;
void OWP_BusL2P(owdi_t *, uint8_t) ;
int	OWP_BusP2L(owdi_t *) ;
int	OWP_BusSelect(owdi_t *) ;
int	OWP_BusSelectAndAddress(owdi_t *, uint8_t) ;
void OWP_BusRelease(owdi_t *) ;

// Common callback handlers
int	OWP_PrintROM_CB(flagmask_t FlagMask, ow_rom_t * psROM) ;
int	OWP_Print1W_CB(flagmask_t FlagMask, owdi_t * psOW) ;
int	OWP_PrintDS18_CB(flagmask_t FlagMask, ds18x20_t * psDS18X20) ;
int	OWP_PrintChan_CB(flagmask_t FlagMask, owbi_t * psCI) ;
int	OWP_Count_CB(flagmask_t FlagMask, owdi_t *) ;

int	OWP_Scan(uint8_t, int (*)(flagmask_t, owdi_t *), owdi_t *) ;
int	OWP_Scan2(uint8_t, int (*)(flagmask_t, void *, owdi_t *), void *, owdi_t *) ;
int	OWP_ScanAlarmsFamily(uint8_t Family) ;

struct epw_t ;
int	OWP_TempStartSample(epw_t * psEWP) ;
int	OWP_TempAllInOne(struct epw_t * psEPW) ;

int	OWP_Config(void) ;
void OWP_Report(void) ;

int	OWP_DS1990ScanCB(flagmask_t, owdi_t *) ;
int	OWP_DS1990ScanAll(epw_t * psEWP) ;

#ifdef __cplusplus
}
#endif
