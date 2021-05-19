/*
 * Copyright 2018-21 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

/*
 * ds18x20.h
 */

#pragma		once

#include	"endpoint_struct.h"

#ifdef __cplusplus
	extern "C" {
#endif

// ############################################# Macros ############################################

#define	ds18x20SINGLE_DEVICE				0
#define	ds18x20TRIGGER_GLOBAL				0
#define	ds18x20DELAY_CONVERT				752
#define	ds18x20DELAY_SP_COPY				11

#define	ds18x20T_SNS_MIN					1000
#define	ds18x20T_SNS_NORM					60000

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

typedef	struct __attribute__((packed)) fam10 { uint8_t Res0, Res1, Remain, Count ; } fam10 ;
typedef	struct __attribute__((packed)) fam28 { uint8_t Conf, Res1, Res2, Res3 ; } fam28 ;
DUMB_STATIC_ASSERT(sizeof(fam10) == sizeof(fam28)) ;

typedef struct __attribute__((packed)) ds18x20_s {		// DS1820, DS18S20 & DS18B20 9[12] bit Temperature sensors
	onewire_t	sOW ;									// size = 12
	union {												// Scratchpad
		struct __attribute__((packed)) {
			uint8_t	Tlsb, Tmsb ;						// last RAM sample
			uint8_t	Thi, Tlo ;							// Integer portion of Lo & Hi alarm thresholds
			union {
				fam10 fam10 ;
				fam28 fam28 ;
			} ;
			uint8_t	CRC ;								// calculated CRC of previous 8 bytes
		} ;
		uint8_t	RegX[9] ;
	} ;
	uint8_t	Idx		: 3 ;								// Endpoint index (0->7) of this specific device
	uint8_t	Res		: 2 ;								// Resolution 0=9b 1=10b 2=11b 3=12b
	uint8_t	Pwr		: 1 ;								// Power  0=Parasitic  1=External
	uint8_t	OD		: 1 ;								// OverDrive 0=Disabled 1=Enabled
	uint8_t	SBits	: 1 ;
	epw_t	sEWx ;
} ds18x20_t ;

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

int32_t	ds18x20ConvertTemperature(ds18x20_t * psDS18X20) ;
int32_t	ds18x20ReadTemperature(ds18x20_t * psDS18X20) ;

int32_t	ds18x20ReadSP(ds18x20_t * psDS18X20, int32_t Len) ;
int32_t	ds18x20WriteSP(ds18x20_t * psDS18X20) ;
int32_t	ds18x20WriteEE(ds18x20_t * psDS18X20) ;

int32_t	ds18x20ResetConfig(ds18x20_t * psDS18X20) ;
void	ds18x20ReportAll(void) ;

/*
 * ds18x20ReadConvertAll() - 1 bus at a time, all devices address & convert, then read & convert 1 at a time.
 */
struct epw_t ;
int32_t	ds18x20ReadConvertAll(struct epw_t * psEpWork) ;
int32_t	ds18x20ScanAlarmsAll(void) ;

int32_t	ds18x20SetResolution(ds18x20_t * psDS18X20, int Res) ;
int32_t	ds18x20SetAlarms(ds18x20_t * psDS18X20, int Lo, int Hi) ;
struct rule_t ;
int32_t	ds18x20ConfigMode (struct rule_t * psRule) ;
int32_t	ds18x20EnumerateCB(flagmask_t sFM, onewire_t * psOW) ;
int32_t	ds18x20Enumerate(int32_t xUri)  ;

#ifdef __cplusplus
	}
#endif
