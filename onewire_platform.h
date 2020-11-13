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
int32_t	OWPlatformCB_Count(flagmask_t FlagMask, onewire_t *) ;

int32_t OWPlatformEndpoints(struct ep_work_s *) ;
int32_t	OWPlatformScanner(uint8_t Family, int32_t (*Handler)(flagmask_t, onewire_t *), onewire_t *) ;
int32_t	OWPlatformConfig(void) ;

#ifdef __cplusplus
}
#endif
