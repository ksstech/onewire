/*
 * ds18x20.h - Copyright (c) 2018-24 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// ######################################## Enumerations ###########################################


// ######################################### Structures ############################################

// See http://www.catb.org/esr/structure-packing/
// Also http://c0x.coding-guidelines.com/6.7.2.1.html

typedef	struct fam10 { u8_t Res0, Res1, Remain, Count; } fam10;
DUMB_STATIC_ASSERT(sizeof(fam10) == 4);

typedef	struct fam28 { u8_t Conf, Res1, Res2, Res3; } fam28;
DUMB_STATIC_ASSERT(sizeof(fam28) == 4);

typedef struct __attribute__((packed)) {				// DS1820/S20/B20 9/12 bit Temp sensors
	owdi_t sOW;						// address of enumerated sensor (size = 12)
	epw_t sEWx;						// size = 32
	union {							// Scratchpad
		struct {
			u8_t Tlsb, Tmsb;		// last RAM sample
			u8_t Thi, Tlo;			// Integer portion of Lo & Hi alarm thresholds
			union { fam10 fam10; fam28 fam28; };
			u8_t CRC;				// calculated CRC of previous 8 bytes
		};
		u8_t RegX[9];
	};
	struct {
		u8_t Idx : 3;				// Endpoint index (0->7) of this specific device
		u8_t Res : 2;				// Resolution 0=9b 1=10b 2=11b 3=12b
		u8_t SBits : 3;
	};
} ds18x20_t;
//DUMB_STATIC_ASSERT(sizeof(ds18x20_t) == 58);

// ###################################### Public variables #########################################

#if (HAL_DS18X20 > 0)
	extern u8_t Fam10Count, Fam28Count;
#endif

// ###################################### Public functions #########################################

/**
 * ds18x20CheckPower() - Read the power supply type (parasitic or external)
 */
bool ds18x20CheckPower(ds18x20_t * psDS18X20);;
int	ds18x20ConvertTemperature(ds18x20_t * psDS18X20);;

int	ds18x20ReadSP(ds18x20_t * psDS18X20, int Len);;
int	ds18x20WriteSP(ds18x20_t * psDS18X20);;
int	ds18x20WriteEE(ds18x20_t * psDS18X20);;

int	ds18x20Initialize(ds18x20_t * psDS18X20);;
int	ds18x20ResetConfig(ds18x20_t * psDS18X20);;
int ds18x20ReportAll(report_t * psR);

// ##################################### I2C Task support ##########################################

struct rule_t;
int	ds18x20ConfigMode (struct rule_t *, int Xcur, int Xmax);
int	ds18x20EnumerateCB(report_t * psR, owdi_t * psOW);
int	ds18x20Enumerate(void);
int	ds18x20Print_CB(report_t * psR, ds18x20_t * psDS18X20);

struct epw_t;;
int	ds18x20Sense(epw_t * psEWP);;
int	ds18x20StartAllInOne(struct epw_t * psEPW);;

#ifdef __cplusplus
}
#endif
