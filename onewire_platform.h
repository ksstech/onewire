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


// ######################################## Enumerations ###########################################


// ######################################### Structures ############################################


// #################################### Public Data structures #####################################

extern	ow_chan_info_t * psaOW_CI ;
extern	ow_flags_t	OWflags ;

// ###################################### Public functions #########################################

ow_chan_info_t * psOWPlatformGetInfoPointer(uint8_t) ;
int32_t	OWPlatformChanLog2Phy(onewire_t *, uint8_t) ;
int32_t	OWPlatformChanPhy2Log(onewire_t *) ;
// Common callback handlers
int32_t	OWPlatformCB_PrintROM(flagmask_t FlagMask, ow_rom_t * psROM) ;
int32_t	OWPlatformCB_Print1W(flagmask_t FlagMask, onewire_t * psOW) ;
int32_t	OWPlatformCB_PrintDS18(flagmask_t FlagMask, ds18x20_t * psDS18X20) ;
int32_t	OWPlatformCB_PrintChan(flagmask_t FlagMask, ow_chan_info_t * psCI) ;
int32_t	OWPlatformCB_Count(flagmask_t FlagMask, onewire_t *) ;

int32_t	OWPlatformEndpoints(struct epw_t *) ;
int32_t	OWPlatformScan(uint8_t, int32_t (*)(flagmask_t, void *, onewire_t *), void *, onewire_t *) ;
int32_t	OWPlatformScanner(uint8_t, int32_t (*)(flagmask_t, onewire_t *), onewire_t *) ;

int32_t	OWPlatformConfig(void) ;
void	OWPlatformReportAll(void) ;

#ifdef __cplusplus
}
#endif
