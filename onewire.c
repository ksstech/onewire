// onewire.c - Copyright (c) 2014-24 Andre M. Maree / KSS Technologies (Pty) Ltd.

#include "hal_platform.h"

#if (HAL_ONEWIRE > 0)
#include "onewire_platform.h"

#include "printfx.h"
#include "syslog.h"
#include "x_errors_events.h"

#include <string.h>

#define	debugFLAG					0xF000

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
 * @brief	Reset all of the devices on the 1-Wire Net and return the result.
 * @return	1 if presence pulse(s) detected, device(s) reset
 *			0 if no presence pulse(s) detected
 */
int OWReset(owdi_t * psOW) {
	return ds248xOWReset(&psaDS248X[psOW->DevNum]);
//	return psOW->Type ? rmtOWReset(&psaRMT[psOW->DevNum]) : ds248xOWReset(&psaDS248X[psOW->DevNum]);
}

// ############################### Bit/Byte/Block Read/Write #######################################

/**
 * Send 1 bit of communication to the 1-Wire Net.
 * The parameter 'sendbit' least significant bit is used.
 *
 * 'sendbit' - 1 bit to send (least significant byte)
 */
void OWWriteBit(owdi_t * psOW, bool Bit) { ds248xOWTouchBit(&psaDS248X[psOW->DevNum], Bit); }

/**
 * @brief	Read 1 bit of communication from the 1-Wire Net and return the result
 * @return	1 bit read from 1-Wire Net
 */
bool OWReadBit(owdi_t * psOW) { return ds248xOWTouchBit(&psaDS248X[psOW->DevNum], 1) ; }

/**
 * @brief	Send 8 bits of communication to the 1-Wire Net and verify that the
 * 			8 bits read from the 1-Wire Net is the same (write operation).
 * @param	psOW
 * @param	Byte
 * @return	status register value after write
 */
u8_t OWWriteByte(owdi_t * psOW, u8_t Byte) {
	return ds248xOWWriteByte(&psaDS248X[psOW->DevNum], Byte);
}

/**
 * @brief	Reads 8 bits of communication from the 1-Wire Net
 * @return	8 bits read from 1-Wire Net
 */
u8_t OWReadByte(owdi_t * psOW) { return ds248xOWReadByte(&psaDS248X[psOW->DevNum]) ; }

void OWWriteBlock(owdi_t * psOW, u8_t * pBuf, int Len) {
	for (int i = 0; i < Len; OWWriteByte(psOW, pBuf[i++])) ;
}

void OWReadBlock(owdi_t * psOW, u8_t * pBuf, int Len) {
	for (int i = 0; i < Len; pBuf[i++] = OWReadByte(psOW)) ;
}

// ############################## Search and Variations thereof ####################################

/**
 * Setup search to find the first 'family_code' device on the next call to OWNext().
 * If no (more) devices of 'family_code' can be found return first device of next family
 */
void OWTargetSetup(owdi_t * psOW, u8_t family_code) {
	psOW->ROM.Value	= 0ULL;								// reset all ROM fields
	psOW->ROM.HexChars[owFAMILY] = family_code;
	psOW->LD = 64;
	psOW->LFD = 0;
	psOW->LDF = 0;
}

/**
 * Setup the search to skip the current device family on the next call to OWNext().
 * Can ONLY be done after a search had been performed.
 * Will find the first device of the next family.
 */
void OWFamilySkipSetup(owdi_t * psOW) {
	psOW->LD = psOW->LFD;			// set Last discrepancy to last family discrepancy
	psOW->LFD = 0;					// clear the last family discrepancy
	if (psOW->LD == 0)
		psOW->LDF = 1;				// check for end of list
}

/**
 * @brief	Update CRC8 with byte value provided (See Application Note 27)
 * @param	data
 * @return	Returns current crc8 value
 */
static u8_t OWUpdateCRC8(u8_t crc8, u8_t data) {
	crc8 ^= data;
	for (int i = 0; i < BITS_IN_BYTE; ++i)
		crc8 = (crc8 & 1) ? (crc8 >> 1) ^ 0x8c : (crc8 >> 1);
	return crc8;
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
int OWSearch(owdi_t * psOW, bool alarm_only) {
	u8_t BitNum = 1, LastZero = 0, u8SrcDir, u8Status, u8ByteMask = 1;
	s8_t i8SrcRes = 0, i8ByteNum = 0;
	u8_t crc8 = 0;
	if (psOW->LDF == 0) {								// if the last call was not the last device
		if (OWReset(psOW) == 0) {						// any device there?
			psOW->LD = 0;								// no, reset the search
			psOW->LDF = 0;
			psOW->LFD = 0;
			return 0;
		}
		OWWriteByte(psOW, alarm_only ? OW_CMD_SEARCHALARM : OW_CMD_SEARCHROM);
		do {
		// if this discrepancy is before the Last Discrepancy
		// on a previous next then pick the same as last time
			if (BitNum < psOW->LD) {
				u8SrcDir = ((psOW->ROM.HexChars[i8ByteNum] & u8ByteMask) > 0) ? 1 : 0 ;
			} else {									// if equal to last pick 1, if not then pick 0
				u8SrcDir = (BitNum == psOW->LD) ? 1 : 0;
			}
			u8Status = ds248xOWSearchTriplet(&psaDS248X[psOW->DevNum], u8SrcDir) ;
			s8_t i8IdBit = ((u8Status & ds248xSTAT_SBR) == ds248xSTAT_SBR);
			s8_t i8IdBitCmp	= ((u8Status & ds248xSTAT_TSB) == ds248xSTAT_TSB);
			u8SrcDir = ((u8Status & ds248xSTAT_DIR) == ds248xSTAT_DIR) ? 1 : 0 ;
			if (i8IdBit && i8IdBitCmp) {				// check for no devices on 1-Wire
				break ;
			} else {
				if ((!i8IdBit) && (!i8IdBitCmp) && (u8SrcDir == 0)) {
					LastZero = BitNum ;
					if (LastZero < 9)
						psOW->LFD = LastZero ;
				}
				if (u8SrcDir == 1)
					psOW->ROM.HexChars[i8ByteNum] |= u8ByteMask ;
				else
					psOW->ROM.HexChars[i8ByteNum] &= ~u8ByteMask;
				++BitNum ;						// increment the byte counter id_bit_number
				u8ByteMask <<= 1 ;					// adjust mask for next bit
				if (u8ByteMask == 0) {				// if mask is 0 byte is done
					crc8 = OWUpdateCRC8(crc8, psOW->ROM.HexChars[i8ByteNum]);  // Accumulate CRC
					++i8ByteNum ;					// Next ROM byte
					u8ByteMask = 1 ;					// Reset the mask
				}
			}
		} while(i8ByteNum < sizeof(ow_rom_t));			// loop till 8 bytes done

		if (!((BitNum < 65) || (crc8 != 0))) {			// search successful ?
			psOW->LD = LastZero;						// yes
			if (psOW->LD == 0) 							// last discrepancy?
				psOW->LDF = 1;							// set flag
			i8SrcRes = 1;								// status = FOUND !!!
		}
	}

	// if no device found then reset counters so next 'search' will be like a first
	if ((i8SrcRes == 0) || (psOW->ROM.HexChars[owFAMILY] == 0)) {
		psOW->LD = 0;
		psOW->LDF = 0;
		psOW->LFD = 0;
		i8SrcRes = 0;
	}
	return i8SrcRes ;
}

/**
 * Find the 'first' device on the 1W network
 * Return 1  : device found, ROM number in ROM.Number buffer
 *		  0 : no device present
 */
int	OWFirst(owdi_t * psOW, bool alarm_only) {
	psOW->LD = 0;					// reset the search state
	psOW->LFD = 0;
	psOW->LDF = 0;
	return OWSearch(psOW, alarm_only);
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
int	OWLevel(owdi_t * psOW, bool Pwr) { return ds248xOWLevel(&psaDS248X[psOW->DevNum], Pwr) ; }

/**
 * OWCheckCRC() - Checks if CRC is ok (ROM Code or Scratch PAD RAM)
 * @param	buf			buffer to be checked for proper CRC
 * @param	buflen		buffer length
 * @return	1 if the CRC is correct, 0 otherwise
 */
u8_t OWCheckCRC(u8_t * buf, u8_t buflen) {
	u8_t data_bit, sr_lsb, fb_bit, shift_reg = 0 ;
	for (int iByte = 0; iByte < buflen; iByte++) {
		for (int iBit = 0; iBit < BITS_IN_BYTE; iBit++) {
			data_bit	= (buf[iByte] >> iBit) & 0x01 ;
			sr_lsb		= shift_reg & 0x01 ;
			fb_bit		= (data_bit ^ sr_lsb) & 0x01 ;
			shift_reg	= shift_reg >> 1 ;
			if (fb_bit)
				shift_reg = shift_reg ^ 0x8c ;
		}
	}
	if (shift_reg)
		SL_ERR("CRC=%x (%d) FAIL %'-+hhY", shift_reg, buflen, buflen, buf) ;
	return shift_reg ? 0 : 1 ;
}

/**
 * OWReadROM() - Check PPD, send command and loop for 8byte read
 * @brief	To be used if only a single device on a bus and the ROM ID must be read
 * 			Probably will fail if more than 1 device on the bus
 * @return	erFAILURE or CRC byte
 */
int	OWReadROM(owdi_t * psOW) {
	OWWriteByte(psOW, OW_CMD_READROM);
	OWReadBlock(psOW, psOW->ROM.HexChars, sizeof(ow_rom_t));
	return OWCheckCRC(psOW->ROM.HexChars, sizeof(ow_rom_t));
}

/**
 * OWAddress() - Addresses a single or all devices on the 1-wire bus
 * @param nAddrMethod	use OW_CMD_MATCHROM to select a single
 *						device or OW_CMD_SKIPROM to select all
 * @note	Timing is 163/860 (SKIPROM) or 1447/7740 (MATCHROM)
 */
void OWAddress(owdi_t * psOW, bool Skip) {
	OWWriteByte(psOW, Skip ? OW_CMD_SKIPROM : OW_CMD_MATCHROM);
	if (Skip == owADDR_MATCH)
		OWWriteBlock(psOW, psOW->ROM.HexChars, sizeof(ow_rom_t));
}

int OWResetCommand(owdi_t * psOW, u8_t Command, bool Skip, bool Pwr) {
	if (OWReset(psOW) == 0) 		// check if any device there
		return 0;
	OWAddress(psOW, Skip);			// address bus or device
	if (Pwr && (psOW->PSU == 0)) 	// If PWR requested but not enabled...
		OWLevel(psOW, owPOWER_STRONG);
	OWWriteByte(psOW, Command);		// send the command
	return 1;
}

/**
 * Verify the device with the ROM number in ROM buffer is present.
 * Return 1  : device verified present
 *		  0 : device not present
 */
int	OWVerify(owdi_t * psOW) {
	owdi_t	backup;
	memcpy((void *) &backup, (const void *) psOW, sizeof(owdi_t));
	psOW->LD = 64;					// set search to find the same device
	psOW->LDF = 0;

	int	iRV = OWSearch(psOW, 0);
	if (iRV == 1) {
		for (int i = 0; i < sizeof(ow_rom_t); ++i) {
			if (backup.ROM.HexChars[i] != psOW->ROM.HexChars[i]) {
				iRV = 0;
				break;
			}
		}
	}
	memcpy((void *) psOW, (const void *) &backup, sizeof(owdi_t));			// restore the search state
	return iRV;											// return the result of the verify
}

u64_t OWAddr2Value(ow_rom_t * psROM) {
	u64_t U64val = 0;
	for (int Idx = 0; Idx < SO_MEM(ow_rom_t, TAG); ++Idx) {
		U64val <<= 8;
		U64val += psROM->TAG[(SO_MEM(ow_rom_t, TAG) - 1) - Idx];
	}
	return U64val;
}
#endif
