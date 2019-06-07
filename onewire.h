/*
 * Copyright 2014-18 AM Maree/KSS Technologies (Pty) Ltd.
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
 * onewire.h
 */

#pragma		once

#include	"x_definitions.h"

#include	<stdint.h>

// ################################## Generic 1-Wire Commands ######################################

#define OW_CMD_SEARCHROM     				0xF0
#define OW_CMD_READROM       				0x33
#define OW_CMD_MATCHROM      				0x55
#define OW_CMD_SKIPROM       				0xCC
#define OW_CMD_ALARMSEARCH   				0xEC

// ################################## DS18X20 Function Commands ####################################

#define	DS18X20_CONVERT						0x44
#define	DS18X20_COPY_SP						0x48
#define	DS18X20_WRITE_SP					0x4E
#define	DS18X20_READ_PSU					0xB4
#define	DS18X20_RECALL_EE					0xB8
#define	DS18X20_READ_SP						0xBE

#define ONEWIRE_ROM_LENGTH        			8

// ##################################### iButton Family Codes #####################################

#define	OWFAMILY_01		0x01			// (DS1990A), (DS1990R), DS2401, DS2411	1-Wire net address (registration number) only
#define	OWFAMILY_02		0x02			// (DS1991)¹	Multikey iButton, 1152-bit secure memory
#define	OWFAMILY_04		0x04			// (DS1994), DS2404	4Kb NV RAM memory and clock, timer, alarms
#define	OWFAMILY_05		0x05			// DS2405¹	Single addressable switch
#define	OWFAMILY_06		0x06			// (DS1993)	4Kb NV RAM memory
#define	OWFAMILY_08		0x08			// (DS1992)	1Kb NV RAM memory
#define	OWFAMILY_09		0x09			// (DS1982), DS2502	1Kb EPROM memory
#define	OWFAMILY_0A		0x0A			// (DS1995)	16Kb NV RAM memory
#define	OWFAMILY_0B		0x0B			// (DS1985), DS2505	16Kb EPROM memory
#define	OWFAMILY_0C		0x0C			// (DS1996)	64Kb NV RAM memory
#define	OWFAMILY_OF		0x0F			// (DS1986), DS2506	64Kb EPROM memory
#define	OWFAMILY_10		0x10			// (DS1820), DS18S20 Temperature with alarm trips
#define	OWFAMILY_12		0x12			// DS2406, DS2407¹	1Kb EPROM memory, 2-channel addressable switch
#define	OWFAMILY_14		0x14			// (DS1971), DS2430A¹	256-bit EEPROM memory and 64-bit OTP register
#define	OWFAMILY_1A		0x1A			// (DS1963L)¹	4Kb NV RAM memory with write cycle counters
#define	OWFAMILY_1C		0x1C			// DS28E04-100	4096-bit EEPROM memory, 2-channel addressable switch
#define	OWFAMILY_1D		0x1D			// DS2423¹	4Kb NV RAM memory with external counters
#define	OWFAMILY_1F		0x1F			// DS2409¹	2-channel addressable coupler for sub-netting
#define	OWFAMILY_20		0x20			// DS2450	4-channel A/D converter (ADC)
#define	OWFAMILY_21		0x21			// (DS1921G), (DS1921H), (DS1921Z)	Thermochron® temperature logger
#define	OWFAMILY_23		0x23			// (DS1973), DS2433	4Kb EEPROM memory
#define	OWFAMILY_24		0x24			// (DS1904), DS2415	Real-time clock (RTC)
#define	OWFAMILY_27		0x27			// DS2417	RTC with interrupt
#define	OWFAMILY_28		0x28			// DS18B20 (9-12 bit programmable) Thermometer
#define	OWFAMILY_29		0x29			// DS2408	8-channel addressable switch
#define	OWFAMILY_2C		0x2C			// DS2890¹	1-channel digital potentiometer
#define	OWFAMILY_2D		0x2D			// (DS1972), DS2431	1024-bit, 1-Wire EEPROM
#define	OWFAMILY_37		0x37			// (DS1977)	Password-protected 32KB (bytes) EEPROM
#define	OWFAMILY_3A		0x3A			// (DS2413)	2-channel addressable switch
#define	OWFAMILY_41		0x41			// (DS1922L/T), (DS1923), DS2422 High-capacity Thermochron (temperature) and Hygrochron™ (humidity) loggers
#define	OWFAMILY_42		0x42			// DS28EA00	Programmable resolution digital thermometer with sequenced detection and PIO
#define	OWFAMILY_43		0x43			// DS28EC20	20Kb 1-Wire EEPROM

// ######################################## Enumerations ###########################################

enum {													// API mode bit flags
	owMODE_STANDARD,									// for Speed & PullUp
	owMODE_OVERDRIVE,									// Speed only
	owMODE_STRONG,										// PullUp only
} ;

enum {
	owFAM28_RES9B,
	owFAM28_RES10B,
	owFAM28_RES11B,
	owFAM28_RES12B,
} ;

// ######################################### Structures ############################################

typedef union ow_rom_u {
	uint64_t	Value ;
	uint8_t		HexChars[ONEWIRE_ROM_LENGTH] ;
	struct {
		uint8_t		Family ;
		uint8_t		TagNum[6] ;
		uint8_t		CRC ;
	} ;
} ow_rom_t ;

DUMB_STATIC_ASSERT( sizeof(ow_rom_t) == ONEWIRE_ROM_LENGTH) ;

// ###################################### Private functions ########################################


// ####################################### Global functions ########################################
