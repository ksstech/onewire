/*
 * Copyright 2014-21 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#include	"hal_config.h"
#include	"onewire_platform.h"

#include	"endpoint_id.h"
#include	"printfx.h"
#include	"syslog.h"
#include	"x_errors_events.h"

#ifdef	ESP_PLATFORM
	#include	"esp32/rom/crc.h"					// ESP32 ROM routine
#else
	#include	"crc-barr.h"						// Barr group CRC
#endif

#include	<string.h>
#include	<limits.h>

#define	debugFLAG					0xD007

#define	debugBUS_CFG				(debugFLAG & 0x0001)
#define	debugCONFIG					(debugFLAG & 0x0002)
#define	debugCRC					(debugFLAG & 0x0004)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

/* https://www.maximintegrated.com/en/products/ibutton/software/1wire/wirekit.cfm
 * https://www.maximintegrated.com/en/app-notes/index.mvp/id/74
 *
 * ONEWIRE commands take a couple of formats resulting in a range of durations
 * 1. Instantaneous (0uS), no 1W bus activity, only affect DS2482, optional status
 * 2. Fast (< 1uS), no 1W bus activity, only affect DS2482,
 * 3. Medium (1uS < ?? < 1mS)
 * 4. Slow (> 1mS)
 * In order to optimise system performance minimal time should be spent in a tight
 * loop waiting for status, task should yield (delay) whenever possible.
 *				[DRST]	[SRP]	[WCFG]	[CHSL]	1WRST	1WWB	1WRB	1WSB	1WT
 *	Duration	525nS	0nS		0nS		0nS		1244uS	8x73uS	8x73uS	1x73uS	3x73uS
 */

// ################################ Generic 1-Wire LINK API's ######################################

int32_t	OWSetSPU(onewire_t * psOW) {
	IF_myASSERT(debugPARAM, halCONFIG_inSRAM(psOW) && (psOW->BusType < owTYPE_MAXNUM)) ;
	switch (psOW->BusType) {
#if		(halHAS_DS248X > 0)
	case owTYPE_DS248X:			return ds248xOWSetSPU(&psaDS248X[psOW->DevNum]) ;
#endif
	default:	myASSERT(0) ;	return 0 ;
	}
}

/**
 * Reset all of the devices on the 1-Wire Net and return the result.
 *
 * Returns	1 :  presence pulse(s) detected, device(s) reset
 *			0 : no presence pulses detected
 */
int32_t OWReset(onewire_t * psOW) {
	IF_myASSERT(debugPARAM, halCONFIG_inSRAM(psOW) && (psOW->BusType < owTYPE_MAXNUM)) ;
	switch (psOW->BusType) {
#if		(halHAS_DS248X > 0)
	case owTYPE_DS248X:			return ds248xOWReset(&psaDS248X[psOW->DevNum]) ;
#endif
	default:	myASSERT(0) ;	return 0 ;
	}
}

/**
 * Set the 1-Wire Net communication speed.
 * 'new_speed' - new speed defined as
 *					 owSPEED_STANDARD	0
 *					 owSPEED_OVERDRIVE	1
 *
 * Returns:  new current 1-Wire Net speed
 */
int32_t OWSpeed(onewire_t * psOW, bool speed) {
	IF_myASSERT(debugPARAM, halCONFIG_inSRAM(psOW) && (psOW->BusType < owTYPE_MAXNUM)) ;
	switch (psOW->BusType) {
#if		(halHAS_DS248X > 0)
	case owTYPE_DS248X:			return ds248xOWSpeed(&psaDS248X[psOW->DevNum], speed) ;
#endif
	default:	myASSERT(0) ;	return 0 ;
	}
}

/**
 * Set the 1-Wire Net line level pull-up to normal.
 * 'new_level' - new level defined as
 *		MODE_STANDARD	0x00
 *		MODE_STRONG		0x02
 * Returns:  current 1-Wire Net level
 */
int32_t OWLevel(onewire_t * psOW, bool level) {
	IF_myASSERT(debugPARAM, halCONFIG_inSRAM(psOW) && (psOW->BusType < owTYPE_MAXNUM)) ;
	switch (psOW->BusType) {
#if		(halHAS_DS248X > 0)
	case owTYPE_DS248X:			return ds248xOWLevel(&psaDS248X[psOW->DevNum], level) ;
#endif
	default:	myASSERT(0) ;	return 0 ;
	}
}

/**
 * OWCheckCRC() - Checks if CRC is ok (ROM Code or Scratch PAD RAM)
 * @param	buf			buffer to be checked for proper CRC
 * @param	buflen		buffer length
 * @return	1 if the CRC is correct, 0 otherwise
 */
uint8_t	OWCheckCRC(uint8_t * buf, uint8_t buflen) {
	uint8_t data_bit, sr_lsb, fb_bit, shift_reg = 0 ;
	for (int8_t iByte = 0; iByte < buflen; iByte++) {
		for (int8_t iBit = 0; iBit < CHAR_BIT; iBit++) {
			data_bit	= (buf[iByte] >> iBit) & 0x01 ;
			sr_lsb		= shift_reg & 0x01 ;
			fb_bit		= (data_bit ^ sr_lsb) & 0x01 ;
			shift_reg	= shift_reg >> 1 ;
			if (fb_bit) {
				shift_reg = shift_reg ^ 0x8c ;
			}
		}
	}
	IF_PRINT(debugCRC && shift_reg, "CRC=%x FAIL %'-+b\n", shift_reg, buflen, buf) ;
	return shift_reg ? 0 : 1 ;
}

/**
 * OWCalcCRC8() - Calculate the CRC8 of the byte value provided with the current 'crc8' value
 * @brief	See Application Note 27
 * @param	data
 * @return				Returns current crc8 value
 */
uint8_t	OWCalcCRC8(onewire_t * psOW, uint8_t data) {
	psOW->crc8 = psOW->crc8 ^ data;
	for (int32_t i = 0; i < CHAR_BIT; ++i) {
		if (psOW->crc8 & 1) {
			psOW->crc8 = (psOW->crc8 >> 1) ^ 0x8c;
		} else {
			psOW->crc8 = (psOW->crc8 >> 1);
		}
	}
	return psOW->crc8;
}

/**
 * OWSearchTriplet() -
 */
int32_t	OWSearchTriplet(onewire_t * psOW, uint8_t search_direction) {
	IF_myASSERT(debugPARAM, halCONFIG_inSRAM(psOW) && (psOW->BusType < owTYPE_MAXNUM)) ;
	switch (psOW->BusType) {
#if		(halHAS_DS248X > 0)
	case owTYPE_DS248X:			return ds248xOWSearchTriplet(&psaDS248X[psOW->DevNum], search_direction) ;
#endif
	default:	myASSERT(0) ;	return 0 ;
	}
}

/**
 * OWChannelSelect() -
 */
int32_t OWChannelSelect(onewire_t * psOW) {
	IF_myASSERT(debugPARAM, halCONFIG_inSRAM(psOW) && (psOW->BusType < owTYPE_MAXNUM)) ;
	switch (psOW->BusType) {
#if		(halHAS_DS248X > 0)
	case owTYPE_DS248X:	return ds248xOWChannelSelect(&psaDS248X[psOW->DevNum], psOW->PhyChan) ;
#endif
	default:	myASSERT(0) ;	return 0 ;
	}
}

// ################################## Bit/Byte Read/Write ##########################################

/**
 * Send 1 bit of communication to the 1-Wire Net and return the
 * result 1 bit read from the 1-Wire Net.  The parameter 'sendbit'
 * least significant bit is used and the least significant bit
 * of the result is the return bit.
 *
 * 'sendbit' - the least significant bit is the bit to send
 *
 * Returns: 0:	0 bit read from sendbit
 *			 1:	1 bit read from sendbit
 */
uint8_t OWTouchBit(onewire_t * psOW, uint8_t sendbit) {
	IF_myASSERT(debugPARAM, halCONFIG_inSRAM(psOW) && (psOW->BusType < owTYPE_MAXNUM)) ;
	switch (psOW->BusType) {
#if		(halHAS_DS248X > 0)
	case owTYPE_DS248X:			return ds248xOWTouchBit(&psaDS248X[psOW->DevNum], sendbit) ;
#endif
	default:	myASSERT(0) ;	return 0 ;
	}
}

/**
 * Send 1 bit of communication to the 1-Wire Net.
 * The parameter 'sendbit' least significant bit is used.
 *
 * 'sendbit' - 1 bit to send (least significant byte)
 */
void	OWWriteBit(onewire_t * psOW, uint8_t sendbit) { OWTouchBit(psOW, sendbit); }

/**
 * Read 1 bit of communication from the 1-Wire Net and return the result
 *
 * Returns:  1 bit read from 1-Wire Net
 */
uint8_t OWReadBit(onewire_t * psOW) { return OWTouchBit(psOW, 1) ; }

/**
 * Send 8 bits of communication to the 1-Wire Net and verify that the
 * 8 bits read from the 1-Wire Net is the same (write operation).
 * The parameter 'sendbyte' least significant 8 bits are used.
 *
 * 'sendbyte' - 8 bits to send (least significant byte)
 * @return	erSUCCESS or erFAILURE
 */
void	OWWriteByte(onewire_t * psOW, uint8_t sendbyte) {
	IF_myASSERT(debugPARAM, halCONFIG_inSRAM(psOW) && (psOW->BusType < owTYPE_MAXNUM)) ;
	switch (psOW->BusType) {
#if		(halHAS_DS248X > 0)
	case owTYPE_DS248X:
		ds248xOWWriteByte(&psaDS248X[psOW->DevNum], sendbyte) ;
		break ;
#endif
	default:
		myASSERT(0) ;
	}
}

/**
 * Send 8 bits of communication to the 1-Wire Net and verify that the
 * 8 bits read from the 1-Wire Net is the same (write operation).
 * The parameter 'sendbyte' least significant 8 bits are used.  After the
 * 8 bits are sent change the level of the 1-Wire net.
 *
 * 'sendbyte' - 8 bits to send (least significant bit)
 *
 * Returns:  1: bytes written and echo was the same, strong pullup now on
 *			  0: echo was not the same
 */
int32_t OWWriteBytePower(onewire_t * psOW, int32_t sendbyte) {
	if (OWSetSPU(psOW) != 1) {
		return 0 ;
	}
	OWWriteByte(psOW, sendbyte) ;
	return 1 ;
}

/**
 * Send 1 bit of communication to the 1-Wire Net and verify that the
 * response matches the 'applyPowerResponse' bit and apply power delivery
 * to the 1-Wire net.  Note that some implementations may apply the power
 * first and then turn it off if the response is incorrect.
 *
 * 'applyPowerResponse' - 1 bit response to check, if correct then start
 *								power delivery
 *
 * Returns:  1: bit written and response correct, strong pullup now on
 *			  0: response incorrect
 */
int32_t OWReadBitPower(onewire_t * psOW, int32_t applyPowerResponse) {
	if (OWSetSPU(psOW) != 1) {
		return 0 ;
	}
	uint8_t rdbit = OWReadBit(psOW);
	if (rdbit != applyPowerResponse) {					// check if response was correct
		OWLevel(psOW, owSPEED_STANDARD);					// if not, turn off strong pull-up
		return 0 ;
	}
	return 1 ;
}

/**
 * Send 8 bits of read communication to the 1-Wire Net and return the
 * result 8 bits read from the 1-Wire Net.
 *
 * Returns:  8 bits read from 1-Wire Net
 */
int32_t	OWReadByte(onewire_t * psOW) {
	IF_myASSERT(debugPARAM, halCONFIG_inSRAM(psOW) && (psOW->BusType < owTYPE_MAXNUM)) ;
	switch (psOW->BusType) {
#if		(halHAS_DS248X > 0)
	case owTYPE_DS248X:			return ds248xOWReadByte(&psaDS248X[psOW->DevNum]) ;
#endif
	default:	myASSERT(0) ;	return 0 ;
	}
}

/**
 * Send 8 bits of communication to the 1-Wire Net and return the
 * result 8 bits read from the 1-Wire Net.  The parameter 'sendbyte'
 * least significant 8 bits are used and the least significant 8 bits
 * of the result is the return byte.
 *
 * 'sendbyte' - 8 bits to send (least significant byte)
 *
 * Returns:  8 bits read from sendbyte
 */
uint8_t OWTouchByte(onewire_t * psOW, uint8_t sendbyte) {
	if (sendbyte == 0xFF) {
		return OWReadByte(psOW);
	} else {
		OWWriteByte(psOW, sendbyte);
		return sendbyte;
	}
}

/**
 * The 'OWBlock' transfers a block of data to and from the
 * 1-Wire Net. The result is returned in the same buffer.
 *
 * 'tran_buf' - pointer to a block of unsigned
 *				  chars of length 'tran_len' that will be sent
 *				  to the 1-Wire Net
 * 'tran_len' - length in bytes to transfer
 */
void	OWBlock(onewire_t * psOW, uint8_t * tran_buf, int32_t tran_len) {
	for (int32_t i = 0; i < tran_len; ++i) {
		tran_buf[i] = OWTouchByte(psOW, tran_buf[i]) ;
	}
}

/**
 * OWReadROM() - Check PPD, send command and loop for 8byte read
 * @brief	To be used if only a single device on a bus and the ROM ID must be read
 * 			Probably will fail if more than 1 device on the bus
 * @return	erFAILURE or CRC byte
 */
int32_t	OWReadROM(onewire_t * psOW) {
	OWWriteByte(psOW, OW_CMD_READROM) ;

	psOW->ROM.Value = 0ULL ;
	int32_t iRV ;
	do {
		for (uint8_t i = 0; i < sizeof(ow_rom_t); ++i) {
			iRV = OWReadByte(psOW) ;					// read 8x bytes ie ROM FAM+ID+CRC
			LT_GOTO(iRV, erSUCCESS, exit) ;
			psOW->ROM.HexChars[i] = iRV ;
		}
	} while (OWCheckCRC(psOW->ROM.HexChars, sizeof(ow_rom_t)) == 0) ;
exit:
	return iRV ;
}

/**
 * OWAddress() - Addresses a single or all devices on the 1-wire bus
 * @param nAddrMethod	use OW_CMD_MATCHROM to select a single
 *						device or OW_CMD_SKIPROM to select all
 */
void	OWAddress(onewire_t * psOW, uint8_t nAddrMethod) {
	IF_myASSERT(debugPARAM, nAddrMethod == OW_CMD_MATCHROM || nAddrMethod == OW_CMD_SKIPROM) ;
	OWWriteByte(psOW, nAddrMethod) ;
	if (nAddrMethod == OW_CMD_MATCHROM) {
		for (uint8_t i = 0; i < sizeof(ow_rom_t); ++i) {
			OWWriteByte(psOW, psOW->ROM.HexChars[i]) ;
		}
	}
}

// ############################## Search and Variations thereof ####################################

/**
 * Setup search to find the first 'family_code' device on the next call to OWNext().
 * If no (more) devices of 'family_code' can be found return first device of next family
 */
void	OWTargetSetup(onewire_t * psOW, uint8_t family_code) {
	psOW->ROM.Value				= 0ULL ;				// reset all ROM fields
	psOW->ROM.Family 			= family_code ;
	psOW->LastDiscrepancy		= 64 ;
	psOW->LastFamilyDiscrepancy	= 0 ;
	psOW->LastDeviceFlag		= 0 ;
}

/**
 * Setup the search to skip the current device family on the next call to OWNext().
 * Can ONLY be done after a search had been performed.
 * Will find the first device of the next family.
 */
void	OWFamilySkipSetup(onewire_t * psOW) {
	psOW->LastDiscrepancy 		= psOW->LastFamilyDiscrepancy ;	// set the Last discrepancy to last family discrepancy
	psOW->LastFamilyDiscrepancy = 0 ;					// clear the last family discrepancy
	if (psOW->LastDiscrepancy == 0) {					// check for end of list
		psOW->LastDeviceFlag	= 1 ;
	}
}

/**
 * The 'OWSearch' function does a general search.  This function
 * continues from the previous search state. The search state
 * can be reset by using the 'OWFirst' function.
 * This function contains one parameter 'alarm_only'.
 * When 'alarm_only' is 1 the find alarm command
 * 0xEC(OW_CMD_SEARCHALARM) is sent instead of
 * the normal search command 0xF0(OW_CMD_SEARCHROM).
 * Using the find alarm command 0xEC will limit the search to only
 * 1-Wire devices that are in an 'alarm' state.
 *
 * Returns:	1: when a 1-Wire device was found and its
 *						  Serial Number placed in the global ROM
 *			0: when no new device was found.  Either the
 *						  last search was the last device or there
 *						  are no devices on the 1-Wire Net.
 */
int32_t OWSearch(onewire_t * psOW, bool alarm_only) {
	bool	search_result = 0, search_direction ;
	uint8_t	rom_byte_number = 0, last_zero = 0, id_bit_number = 1, rom_byte_mask = 1, status ;
	psOW->crc8 = 0;
	if (psOW->LastDeviceFlag == 0) {					// if the last call was not the last device
		if (OWReset(psOW) == 0) {						// reset the search
			psOW->LastDiscrepancy		= 0 ;
			psOW->LastDeviceFlag		= 0 ;
			psOW->LastFamilyDiscrepancy	= 0 ;
			return 0 ;
		}
		OWWriteByte(psOW, alarm_only ? OW_CMD_SEARCHALARM : OW_CMD_SEARCHROM) ;
		do {											// loop to do the search
		// if this discrepancy is before the Last Discrepancy
		// on a previous next then pick the same as last time
			if (id_bit_number < psOW->LastDiscrepancy) {
				search_direction = ((psOW->ROM.HexChars[rom_byte_number] & rom_byte_mask) > 0) ? 1 : 0 ;
			} else {									// if equal to last pick 1, if not then pick 0
				search_direction = (id_bit_number == psOW->LastDiscrepancy) ? 1 : 0 ;
			}
		// Perform a triple operation on the DS2482 which will perform 2 read bits and 1 write bit
			status = OWSearchTriplet(psOW, search_direction) ;
		// check bit results in status byte
			int32_t	id_bit		= ((status & ds248xSTAT_SBR) == ds248xSTAT_SBR) ;
			int32_t	cmp_id_bit	= ((status & ds248xSTAT_TSB) == ds248xSTAT_TSB) ;
			search_direction	= ((status & ds248xSTAT_DIR) == ds248xSTAT_DIR) ? 1 : 0 ;
			if ((id_bit) && (cmp_id_bit)) {				// check for no devices on 1-Wire
				break ;
			} else {
				if ((!id_bit) && (!cmp_id_bit) && (search_direction == 0)) {
					last_zero = id_bit_number ;
					if (last_zero < 9) {				// check for Last discrepancy in family
						psOW->LastFamilyDiscrepancy = last_zero ;
					}
				}
				if (search_direction == 1) {			// set or clear the bit in the ROM byte rom_byte_number with mask rom_byte_mask
					psOW->ROM.HexChars[rom_byte_number] |= rom_byte_mask ;
				} else {
					psOW->ROM.HexChars[rom_byte_number] &= ~rom_byte_mask ;
				}
				++id_bit_number ;						// increment the byte counter id_bit_number & shift the mask rom_byte_mask
				rom_byte_mask <<= 1 ;
				if (rom_byte_mask == 0) {				// if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
					OWCalcCRC8(psOW, psOW->ROM.HexChars[rom_byte_number]);  // accumulate the CRC
					++rom_byte_number ;
					rom_byte_mask = 1 ;
				}
			}
		} while(rom_byte_number < sizeof(ow_rom_t)) ;	// loop until all
	// if the search was successful then
		if (!((id_bit_number < 65) || (psOW->crc8 != 0))) {
			psOW->LastDiscrepancy = last_zero;			// search successful
			if (psOW->LastDiscrepancy == 0) {			// last device ?
				psOW->LastDeviceFlag	= 1 ;			// yes, set flag
			}
			search_result = 1 ;
		}
	}

	// if no device found then reset counters so next 'search' will be like a first
	if (!search_result || (psOW->ROM.Family == 0)) {
		psOW->LastDiscrepancy	= 0 ;
		psOW->LastDeviceFlag	= 0 ;
		psOW->LastFamilyDiscrepancy = 0 ;
		search_result = 0 ;
	}
	return search_result ;
}

/**
 * Find the 'first' device on the 1W network
 * Return 1  : device found, ROM number in ROM.Number buffer
 *		  0 : no device present
 */
int32_t OWFirst(onewire_t * psOW, bool alarm_only) {
	psOW->LastDiscrepancy		= 0 ;					// reset the search state
	psOW->LastFamilyDiscrepancy	= 0 ;
	psOW->LastDeviceFlag		= 0 ;
	return OWSearch(psOW, alarm_only) ;
}

/**
 * Find the 'next' device on the 1W network
 * Return 1  : device found, ROM number in ROM.Number buffer
 *		  0 : device not found, end of search
 */
int32_t OWNext(onewire_t * psOW, bool alarm_only) { return OWSearch(psOW, alarm_only) ; }

/**
 * Verify the device with the ROM number in ROM buffer is present.
 * Return 1  : device verified present
 *		  0 : device not present
 */
int32_t OWVerify(onewire_t * psOW) {
	onewire_t	backup ;
	memcpy((void *) &backup, (const void *) psOW, sizeof(onewire_t)) ;
	psOW->LastDiscrepancy	= 64 ;				// set search to find the same device
	psOW->LastDeviceFlag	= 0 ;

	int32_t iRV = OWSearch(psOW, 0) ;
	if (iRV == 1) {
		for (int32_t i = 0; i < sizeof(ow_rom_t); ++i) {
			if (backup.ROM.HexChars[i] != psOW->ROM.HexChars[i]) {
				iRV = 0 ;
				break ;
			}
		}
	}
	memcpy((void *) psOW, (const void *) &backup, sizeof(onewire_t)) ;			// restore the search state
	return iRV ;										// return the result of the verify
}
