/*
 * Copyright 2018-19 AM Maree/KSS Technologies (Pty) Ltd.
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
 * onewire_platform.h
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

/* PER CHANNEL info keeping track of last device read (ROM & timestamp) on each channel
 * Used to avoid re-reading a device (primarily DS1990X type) too regularly.
 */
typedef struct __attribute__((packed)) ow_chan_info_s {
	ow_rom_t	LastROM ;
	seconds_t	LastRead ;
} ow_chan_info_t ;
DUMB_STATIC_ASSERT(sizeof(ow_chan_info_t) == 12) ;

typedef struct ow_flags_s {
	uint8_t	Level 	: 2 ;
	uint8_t	Spare	: 6 ;
} ow_flags_t ;

// #################################### Public Data structures #####################################

extern	ow_chan_info_t * psaOW_CI ;
extern	ow_flags_t	OWflags ;

// ###################################### Public functions #########################################

ow_chan_info_t * psOWPlatformGetInfoPointer(uint8_t) ;
int32_t	OWPlatformChanLog2Phy(onewire_t *, uint8_t) ;
int32_t	OWPlatformChanPhy2Log(onewire_t *) ;
// Common callback handlers
int32_t	OWPlatformCB_PrintROM(uint32_t uCount, ow_rom_t * psROM) ;
int32_t	OWPlatformCB_Print1W(uint32_t uCount, onewire_t * psOW) ;
int32_t	OWPlatformCB_PrintDS18(uint32_t uCount, ds18x20_t * psDS18X20) ;
int32_t	OWPlatformCB_Count(uint32_t uCount, onewire_t * psOW) ;

int32_t OWPlatformEndpoints(struct ep_work_s * psEpWork) ;
int32_t	OWPlatformScanner(uint8_t Family, int32_t (*Handler)(uint32_t, onewire_t *), onewire_t * psOW) ;
int32_t	OWPlatformConfig(void) ;

#ifdef __cplusplus
}
#endif
