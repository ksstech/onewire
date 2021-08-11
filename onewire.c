/*
 * Copyright 2014-21 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#include	"hal_variables.h"
#include	"onewire_platform.h"

#include	"endpoint_id.h"

#include	"printfx.h"
#include	"syslog.h"

#include	"x_errors_events.h"

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

// ################################# Basic 1-Wire operations #######################################

/**
 * Reset all of the devices on the 1-Wire Net and return the result.
 * @return	1 if presence pulse(s) detected, device(s) reset
 *			0 if no presence pulse(s) detected
 */
int	 OWReset(owdi_t * psOW) { return ds248xOWReset(&psaDS248X[psOW->DevNum]) ; }

// ############################### Bit/Byte/Block Read/Write #######################################

/**
 * Send 1 bit of communication to the 1-Wire Net.
 * The parameter 'sendbit' least significant bit is used.
 *
 * 'sendbit' - 1 bit to send (least significant byte)
 */
void OWWriteBit(owdi_t * psOW, bool Bit) { ds248xOWTouchBit(&psaDS248X[psOW->DevNum], Bit); }

/**
 * Read 1 bit of communication from the 1-Wire Net and return the result
 *
 * Returns:  1 bit read from 1-Wire Net
 */
bool OWReadBit(owdi_t * psOW) { return ds248xOWTouchBit(&psaDS248X[psOW->DevNum], 1) ; }

/**
 * Send 8 bits of communication to the 1-Wire Net and verify that the
 * 8 bits read from the 1-Wire Net is the same (write operation).
 * The parameter 'sendbyte' least significant 8 bits are used.
 *
 * 'sendbyte' - 8 bits to send (least significant byte)
 */
/**
 * @brief
 * @param	psOW
 * @param	Byte
 */
void OWWriteByte(owdi_t * psOW, uint8_t Byte) { ds248xOWWriteByte(&psaDS248X[psOW->DevNum], Byte) ; }

/**
 * Reads 8 bits of communication from the 1-Wire Net
 * @return	8 bits read from 1-Wire Net
 */
uint8_t	OWReadByte(owdi_t * psOW) { return ds248xOWReadByte(&psaDS248X[psOW->DevNum]) ; }

void OWWriteBlock(owdi_t * psOW, uint8_t * pBuf, int Len) {
	for (int i = 0; i < Len; OWWriteByte(psOW, pBuf[i++])) ;
}

void OWReadBlock(owdi_t * psOW, uint8_t * pBuf, int Len) {
	for (int i = 0; i < Len; pBuf[i++] = OWReadByte(psOW)) ;
}

// ############################## Search and Variations thereof ####################################

/**
 * Setup search to find the first 'family_code' device on the next call to OWNext().
 * If no (more) devices of 'family_code' can be found return first device of next family
 */
void OWTargetSetup(owdi_t * psOW, uint8_t family_code) {
	psOW->ROM.Value				= 0ULL ;				// reset all ROM fields
	psOW->ROM.Family 			= family_code ;
	psOW->LD		= 64 ;
	psOW->LFD	= 0 ;
	psOW->LDF		= 0 ;
}

/**
 * Setup the search to skip the current device family on the next call to OWNext().
 * Can ONLY be done after a search had been performed.
 * Will find the first device of the next family.
 */
void OWFamilySkipSetup(owdi_t * psOW) {
	psOW->LD = psOW->LFD ;			// set the Last discrepancy to last family discrepancy
	psOW->LFD = 0 ;					// clear the last family discrepancy
	if (psOW->LD == 0) psOW->LDF = 1 ;	// check for end of list
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
 * Returns:	1 if 1W device was found (Serial Number placed in the global ROM)
 *			0 if NO 1W device found. Either last search was last device or
 *			there are no devices on the 1-Wire Net.
 */
	bool	search_result = 0, search_direction ;
int OWSearch(owdi_t * psOW, bool alarm_only) {
	uint8_t	rom_byte_number = 0, last_zero = 0, id_bit_number = 1, rom_byte_mask = 1, status ;
	psOW->crc8 = 0;
	if (psOW->LDF == 0) {					// if the last call was not the last device
		if (OWReset(psOW) == 0) {						// reset the search
			psOW->LD	= 0 ;
			psOW->LDF	= 0 ;
			psOW->LFD	= 0 ;
			return 0 ;
		}
		OWWriteByte(psOW, alarm_only ? OW_CMD_SEARCHALARM : OW_CMD_SEARCHROM) ;
		do {											// loop to do the search
		// if this discrepancy is before the Last Discrepancy
		// on a previous next then pick the same as last time
			if (id_bit_number < psOW->LD) {
				search_direction = ((psOW->ROM.HexChars[rom_byte_number] & rom_byte_mask) > 0) ? 1 : 0 ;
			} else {									// if equal to last pick 1, if not then pick 0
				search_direction = (id_bit_number == psOW->LD) ? 1 : 0 ;
			}
		// Perform a triple operation on the DS2482 which will perform 2 read bits and 1 write bit
			status = ds248xOWSearchTriplet(&psaDS248X[psOW->DevNum], search_direction) ;
		// check bit results in status byte
			int	id_bit		= ((status & ds248xSTAT_SBR) == ds248xSTAT_SBR) ;
			int	cmp_id_bit	= ((status & ds248xSTAT_TSB) == ds248xSTAT_TSB) ;
			search_direction	= ((status & ds248xSTAT_DIR) == ds248xSTAT_DIR) ? 1 : 0 ;
			if ((id_bit) && (cmp_id_bit)) {				// check for no devices on 1-Wire
				break ;
			} else {
				if ((!id_bit) && (!cmp_id_bit) && (search_direction == 0)) {
					last_zero = id_bit_number ;
					if (last_zero < 9) {				// check for Last discrepancy in family
						psOW->LFD = last_zero ;
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
			psOW->LD = last_zero;						// search successful
			if (psOW->LD == 0) psOW->LDF = 1;			// last device? set flag
			bRV = 1 ;
		}
	}

	// if no device found then reset counters so next 'search' will be like a first
		search_result = 0 ;
	if ((bRV == 0) || (psOW->ROM.Family == 0)) {
		psOW->LD = 0;
		psOW->LDF = 0;
		psOW->LFD = 0;
	}
	return search_result ;
}

/**
 * Find the 'first' device on the 1W network
 * Return 1  : device found, ROM number in ROM.Number buffer
 *		  0 : no device present
 */
int	OWFirst(owdi_t * psOW, bool alarm_only) {
	psOW->LD	= 0 ;					// reset the search state
	psOW->LFD	= 0 ;
	psOW->LDF	= 0 ;
	return OWSearch(psOW, alarm_only) ;
}

/**
 * Find the 'next' device on the 1W network
 * Return 1  : device found, ROM number in ROM.Number buffer
 *		  0 : device not found, end of search
 */
int OWNext(owdi_t * psOW, bool alarm_only) { return OWSearch(psOW, alarm_only) ; }

// ################################## Utility 1-Wire operations ####################################

/**
 * @brief	Set the 1-Wire Net communication speed.
 * @param	psOW
 * @param	speed - owSPEED_STANDARD or owSPEED_OVERDRIVE
 * @return	new current 1W speed (0 = Standard, 1= Overdrive)
 */
int	OWSpeed(owdi_t * psOW, bool Spd) { return ds248xOWSpeed(&psaDS248X[psOW->DevNum], Spd) ; }

/**
 * Set the 1-Wire Net line level pull-up to normal.
 * 'new_level' - new level defined as
 *		STANDARD	0
 *		STRONG		1
 * Returns:  current 1-Wire Net level
 */

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
int		OWReadBitPower(owdi_t * psOW, uint8_t applyPowerResponse) {
	if (OWSetSPU(psOW) == 0) return 0 ;
	uint8_t rdbit = OWReadBit(psOW);
	if (rdbit != applyPowerResponse) {					// check if response was correct
		OWLevel(psOW, owPOWER_STANDARD);				// if not, turn off strong pull-up
		return 0 ;
	}
	return 1 ;
}

/**
 * Send 8 bits of communication to the 1-Wire Net and verify that the
 * 8 bits read from the 1-Wire Net is the same (write operation).
 * The parameter 'sendbyte' least significant 8 bits are used.
 * After the 8 bits are sent change the level of the 1-Wire net.
 *
 * 'sendbyte' - 8 bits to send (least significant bit)
 *
 * Returns:  1: bytes written and echo was the same, strong pullup now on
 *			  0: echo was not the same
 */
/**
 * @brief
 * @param	psOW
 * @param	Byte - value to be sent on the bus
 * @return
 */
int		OWWriteBytePower(owdi_t * psOW, int Byte) {
	if (OWSetSPU(psOW) == 0) return 0 ;
	OWWriteByte(psOW, Byte) ;
	return 1 ;
}

// ################################## Utility 1-Wire operations ####################################

int	OWSetSPU(owdi_t * psOW) { return ds248xOWSetSPU(&psaDS248X[psOW->DevNum]) ; }
int	OWLevel(owdi_t * psOW, bool Pwr) { return ds248xOWLevel(&psaDS248X[psOW->DevNum], Pwr) ; }

/**
 * OWCheckCRC() - Checks if CRC is ok (ROM Code or Scratch PAD RAM)
 * @param	buf			buffer to be checked for proper CRC
 * @param	buflen		buffer length
 * @return	1 if the CRC is correct, 0 otherwise
 */
uint8_t	OWCheckCRC(uint8_t * buf, uint8_t buflen) {
	uint8_t data_bit, sr_lsb, fb_bit, shift_reg = 0 ;
	for (int iByte = 0; iByte < buflen; iByte++) {
		for (int iBit = 0; iBit < CHAR_BIT; iBit++) {
			data_bit	= (buf[iByte] >> iBit) & 0x01 ;
			sr_lsb		= shift_reg & 0x01 ;
			fb_bit		= (data_bit ^ sr_lsb) & 0x01 ;
			shift_reg	= shift_reg >> 1 ;
			if (fb_bit) shift_reg = shift_reg ^ 0x8c ;
		}
	}
	if (shift_reg) SL_ERR("CRC=%x FAIL %'-+B\n", shift_reg, buflen, buf) ;
	return shift_reg ? 0 : 1 ;
}

/**
 * OWCalcCRC8() - Calculate the CRC8 of the byte value provided with the current 'crc8' value
 * @brief	See Application Note 27
 * @param	data
 * @return				Returns current crc8 value
 */
uint8_t	OWCalcCRC8(owdi_t * psOW, uint8_t data) {
	psOW->crc8 = psOW->crc8 ^ data;
	for (int i = 0; i < CHAR_BIT; ++i) {
		if (psOW->crc8 & 1)	psOW->crc8 = (psOW->crc8 >> 1) ^ 0x8c;
		else 				psOW->crc8 = (psOW->crc8 >> 1);
	}
	return psOW->crc8;
}

/**
 * OWReadROM() - Check PPD, send command and loop for 8byte read
 * @brief	To be used if only a single device on a bus and the ROM ID must be read
 * 			Probably will fail if more than 1 device on the bus
 * @return	erFAILURE or CRC byte
 */
int	OWReadROM(owdi_t * psOW) {
	OWWriteByte(psOW, OW_CMD_READROM) ;
	psOW->ROM.Value = 0ULL ;
	int	iRV = 0 ;
	do {
		for (int i = 0; i < sizeof(ow_rom_t); ++i) {
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
 * @note	Timing is 163/860 (SKIPROM) or 1447/7740 (MATCHROM)
 */
void OWAddress(owdi_t * psOW, uint8_t nAddrMethod) {
	OWWriteByte(psOW, nAddrMethod) ;
	if (nAddrMethod == OW_CMD_MATCHROM) {
		for (int i = 0; i < sizeof(ow_rom_t); OWWriteByte(psOW, psOW->ROM.HexChars[i++])) ;
	}
}

int OWCommand(owdi_t * psOW, uint8_t Command, bool All) {
	OWAddress(psOW, All ? OW_CMD_SKIPROM : OW_CMD_MATCHROM) ;
	OWWriteByte(psOW, Command) ;
	return 1 ;
}

int OWResetCommand(owdi_t * psOW, uint8_t Command, bool All) {
	if (OWReset(psOW) == 0) return 0 ;
	return OWCommand(psOW, Command, All) ;
}

/**
 * Verify the device with the ROM number in ROM buffer is present.
 * Return 1  : device verified present
 *		  0 : device not present
 */
int	OWVerify(owdi_t * psOW) {
	owdi_t	backup ;
	memcpy((void *) &backup, (const void *) psOW, sizeof(owdi_t)) ;
	psOW->LD	= 64 ;				// set search to find the same device
	psOW->LDF	= 0 ;

	int	iRV = OWSearch(psOW, 0) ;
	if (iRV == 1) {
		for (int i = 0; i < sizeof(ow_rom_t); ++i) {
			if (backup.ROM.HexChars[i] != psOW->ROM.HexChars[i]) {
				iRV = 0 ;
				break ;
			}
		}
	}
	memcpy((void *) psOW, (const void *) &backup, sizeof(owdi_t)) ;			// restore the search state
	return iRV ;										// return the result of the verify
}
