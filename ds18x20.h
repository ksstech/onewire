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
 * ds18x20.h
 */

#pragma		once

#include	"x_definitions.h"

#include	"onewire.h"
#include	"endpoints.h"

#include	<stdint.h>

// ############################################# Macros ############################################

#define	DS18X20_EXT_POWER					0			// use enable external power not parasitic
#define	ds18x20TRIGGER_GLOBAL				0

// ######################################## Enumerations ###########################################

enum { ds18x20LSB, ds18x20MSB, ds18x20HI, ds18x20LO, ds18x20CONF, ds18x20RSVD, ds18x20REM, ds18x20CNT, ds18x20CRC, ds18x20NUM } ;

// ######################################### Structures ############################################

// See http://www.catb.org/esr/structure-packing/
// Also http://c0x.coding-guidelines.com/6.7.2.1.html

typedef struct __attribute__((packed)) {				// DS1820, DS18S20 & DS18B20 9[12] bit Temperature sensors
	ow_rom_t	ROM ;
	union {												// Specific 1-Wire device type scratch pad info
		struct fam10 { uint8_t	Tlsb, Tmsb, Thi, Tlo, Rsvd[2], Remain, Count, CRC ; } fam10 ;
		struct fam28 { uint8_t	Tlsb, Tmsb, Thi, Tlo, Conf, Rsvd[3], CRC ; } fam28 ;
		uint8_t	RegX[ds18x20NUM] ;
	} ;
	struct {											// Common 1-Wire endpoint enumeration info
		uint8_t		Ch	: 3 ;							// Channel the device was discovered on
		uint8_t		Idx	: 3 ;							// Endpoint index (0->7) of this specific device
		uint8_t		Res	: 2 ;							// Resolution 0=9b 1=10b 2=11b 3=12b
		uint8_t		spare[2] ;
	} ;
	x32_t		xVal ;
} ds18x20_t ;

DUMB_STATIC_ASSERT(sizeof(struct fam10) == sizeof(struct fam28)) ;
DUMB_STATIC_ASSERT(sizeof(ds18x20_t) == 24) ;

// #################################### Public Data structures #####################################

extern uint8_t Fam10_28Count ;

// ###################################### Private functions ########################################

void	ds18x20EnableExtPSU(ds18x20_t * psDS18X20) ;
void	ds18x20DisableExtPSU(ds18x20_t * psDS18X20) ;
int32_t	ds18x20Discover(void)  ;

float	ds18x20GetTemperature(int32_t Idx) ;
struct ep_work_s ;
int32_t	ds18x20ConvertAndReadAll(struct ep_work_s * psEpWork) ;
int32_t	ds18x20Handler(int32_t, int32_t) ;
