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
 * ds18x20.h
 */

#pragma		once

#ifdef __cplusplus
	extern "C" {
#endif

// ############################################# Macros ############################################

#define	ds18x20SINGLE_DEVICE				0
#define	ds18x20TRIGGER_GLOBAL				0
#define	ds18x20DELAY_CONVERT				752
#define	ds18x20DELAY_SP_COPY				11

// ################################## DS18X20 1-Wire Commands ######################################

#define	DS18X20_CONVERT						0x44
#define	DS18X20_COPY_SP						0x48
#define	DS18X20_WRITE_SP					0x4E
#define	DS18X20_READ_PSU					0xB4
#define	DS18X20_RECALL_EE					0xB8
#define	DS18X20_READ_SP						0xBE

// ######################################## Enumerations ###########################################


// ######################################### Structures ############################################

// See http://www.catb.org/esr/structure-packing/
// Also http://c0x.coding-guidelines.com/6.7.2.1.html

struct fam10 { uint8_t Rsvd[2], Remain, Count ; } __attribute__((packed)) ;
struct fam28 { uint8_t Conf, Rsvd[3] ; } __attribute__((packed)) ;
DUMB_STATIC_ASSERT(sizeof(struct fam10) == sizeof(struct fam28)) ;

typedef struct __attribute__((packed)) ds18x20_s {		// DS1820, DS18S20 & DS18B20 9[12] bit Temperature sensors
	onewire_t	sOW ;									// size = 12
	x32_t		xVal ;
	union {												// Scratchpad
		struct {
			uint8_t		Tlsb, Tmsb ;					// last RAM sample
			uint8_t		Thi, Tlo ;						// Integer portion of Lo & Hi alarm thresholds
			union {
				struct fam10 fam10 ;
				struct fam28 fam28 ;
			} ;
			uint8_t	CRC ;								// calculated CRC of previous 8 bytes
		} __attribute__((packed)) ;
		uint8_t	RegX[9] ;
	} ;
	uint8_t		Idx		: 3 ;							// Endpoint index (0->7) of this specific device
	uint8_t		Res		: 2 ;							// Resolution 0=9b 1=10b 2=11b 3=12b
	uint8_t		Pwr		: 1 ;
	uint8_t		SBits	: 2 ;
} ds18x20_t ;
DUMB_STATIC_ASSERT(sizeof(ds18x20_t) == (12+4+9+1)) ;

// ###################################### Public variables #########################################

extern	uint8_t	Fam10_28Count ;

// ###################################### Public functions #########################################

/**
 * ds18x20CheckPower() - Read the power supply type (parasitic or external)
 */
int32_t	ds18x20CheckPower(ds18x20_t * psDS18X20) ;

/**
 * ds18x20SelectAndAddress() - Select logical bus (device, physical bus) and address/skip ROM
 */
int32_t	ds18x20SelectAndAddress(ds18x20_t * psDS18X20, uint8_t nAddrMethod) ;

float	ds18x20GetTemperature(int32_t Idx) ;
int32_t	ds18x20ConvertTemperature(ds18x20_t * psDS18X20) ;
int32_t	ds18x20ReadTemperature(ds18x20_t * psDS18X20) ;

int32_t	ds18x20ReadSP(ds18x20_t * psDS18X20, int32_t Len) ;
int32_t	ds18x20WriteSP(ds18x20_t * psDS18X20) ;
int32_t	ds18x20WriteEE(ds18x20_t * psDS18X20) ;

int32_t	ds18x20Initialize(ds18x20_t * psDS18X20) ;
int32_t	ds18x20ResetConfig(ds18x20_t * psDS18X20) ;

/*
 * ds18x20ReadConvertAll() - 1 bus at a time, all devices address & convert, then read & convert 1 at a time.
 */
struct ep_work_s ;
int32_t	ds18x20ReadConvertAll(struct ep_work_s * psEpWork) ;
int32_t	ds18x20TestCase2(void) ;

int32_t	ds18x20SetResolution(ds18x20_t * psDS18X20, int8_t i8Res) ;
int32_t	ds18x20SetAlarms(ds18x20_t * psDS18X20, int8_t i8Lo, int8_t i8Hi) ;
struct rule_t ;
int32_t	ds18x20SetMode (void *, struct rule_t * psRule) ;
int32_t	ds18x20EnumerateCB(flagmask_t sFM, onewire_t * psOW) ;
int32_t	ds18x20Enumerate(int32_t xUri)  ;

#ifdef __cplusplus
	}
#endif
