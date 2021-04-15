/*
 * Copyright 2020-21 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma		once

#include	<stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ################################### DS248X 1-Wire Commands ######################################

#define ds248xCMD_DRST   			0xF0				// Device Reset (525nS)
#define ds248xCMD_SRP				0xE1				// Set Read Pointer (0nS)
#define ds248xCMD_WCFG   			0xD2				// Write Config	(0nS)
#define ds2482CMD_CHSL   			0xC3				// Channel Select DS2482-800 (0nS)
#define ds2484CMD_PADJ				0xC3				// Adjust 1W Port DS2484
#define ds248xCMD_1WRS   			0xB4				// 1-Wire Reset (see below)
#define ds248xCMD_1WWB   			0xA5				// 1-Wire Write Byte
#define ds248xCMD_1WRB   			0x96				// 1-Wire Read Byte
#define ds248xCMD_1WSB   			0x87				// 1-Wire Single Bit
#define ds248xCMD_1WT				0x78				// 1-Wire Triplet

// ############################################# Macros ############################################

#define	owDELAY_RST					1243U				// 630 + 613.2 + 0.2625
#define	owDELAY_RST_OD				153U				// 75.6 + 77.7 + 0.2625
#define	owDELAY_SB					73U					// 72.8 + 0.2625
#define	owDELAY_SB_OD				11U					// 11 + 0.2625
#define	owDELAY_RB					583U				// (8 * 72.8) + 0.2625
#define	owDELAY_RB_OD				98U					// (8 * 11) + 0.2625
#define	owDELAY_WB					583U				// (8 * 72.8) + 0.2625
#define	owDELAY_WB_OD				98U					// (8 * 11) + 0.2625
#define	owDELAY_ST					219					// (3 * 72.8) + 0.2625
#define	owDELAY_ST_OD				33					// (3 * 11) + 0.2625

// ######################################## Enumerations ###########################################

enum {													// DS248X register numbers
	ds248xREG_STAT,										// STATus (all)
	ds248xREG_DATA,										// DATA (all)
	ds248xREG_CHAN,										// CHANnel (DS2482-800 only, used to detect -10x vs -800)
	ds248xREG_CONF,										// CONFiguration (all)
	ds248xREG_PADJ,										// Port Adjust (DS2484 only)
	ds248xREG_NUM,
} ;

enum {													// STATus register bitmap
	ds248xSTAT_1WB		= (1 << 0),						// 1W Busy
	ds248xSTAT_PPD		= (1 << 1),						// Presence Pulse Detected
	ds248xSTAT_SD		= (1 << 2),						// Short Detected
	ds248xSTAT_LL		= (1 << 3),						// Logic Level
	ds248xSTAT_RST		= (1 << 4),						// ReSeT
	ds248xSTAT_SBR		= (1 << 5),						// Single Bit Received
	ds248xSTAT_TSB		= (1 << 6),						// Triple Search Bit
	ds248xSTAT_DIR		= (1 << 7),						// DIRection
} ;

// ######################################### Structures ############################################

// See http://www.catb.org/esr/structure-packing/
// Also http://c0x.coding-guidelines.com/6.7.2.1.html

/**
 * PER DEVICE info to track DS248X devices detected and map assignment of LOG to PHY channels
 */
typedef struct __attribute__((packed)) ds248x_s {		// DS248X I2C <> 1Wire bridge
	i2c_dev_info_t *	psI2C ;							// size = 4
	union {												// size = 5
		struct {
	/*LSB*/	uint8_t		OWB		: 1 ;					// 1-Wire Busy
			uint8_t		PPD		: 1 ;					// Presence Pulse Detected
			uint8_t		SD		: 1 ;
			uint8_t		LL		: 1 ;					// Link Level
			uint8_t		RST		: 1 ;					// ReSeT
			uint8_t		SBR		: 1 ;					//
			uint8_t		TSB		: 1 ;					//
	/*MSB*/	uint8_t		DIR		: 1 ;					// DIRection

			uint8_t		DATA 	: 8 ;					// DATA Register
			uint8_t		CHAN	: 8 ;					// CHANnel Selected Register

	/*LSB*/	uint8_t		APU		: 1 ;					// Active Pull Up
			uint8_t		PDN		: 1 ;					// Pull Down (DS2484 only)
			uint8_t		SPU		: 1 ;					// Strong Pull Up
			uint8_t		OWS		: 1 ;					// 1-Wire Speed
	/*MSB*/	uint8_t		RES1	: 4 ;

	/*LSB*/	uint8_t		VAL		: 4 ;					// PARameter VALue
			uint8_t		OD		: 1 ;					// OverDrive control
	/*MSB*/	uint8_t		PAR		: 3 ;					// PARameter selector
		} ;
		struct {
			uint8_t		Rstat ;
			uint8_t		Rdata ;
			uint8_t		Rchan ;							// Code read back after ChanSel
			uint8_t		Rconf ;
			uint8_t		Rpadj ;
		} ;
		uint8_t			RegX[5] ;
	} ;
	uint8_t				CurChan	: 3 ;					// 0 -> 7
	uint8_t				Rptr	: 3 ;					// 0 -> 4
	uint8_t				Test	: 1 ;					// indicate test/identify stage
	uint8_t				Spare	: 1 ;
	// Static info
	uint8_t				I2Cnum	: 4 ;					// index into I2C Device Info table
	uint8_t				NumChan	: 4 ;					// 0 / 1 / 8
	uint8_t				Lo		: 4 ;
	uint8_t				Hi		: 4 ;
	// Status bits for 8 channels
	uint8_t				PrvStat[8] ;					// previous STAT reg
} ds248x_t ;
DUMB_STATIC_ASSERT(sizeof(ds248x_t) == 20) ;

// #################################### Public Data structures #####################################

extern	uint8_t		ds248xCount ;
extern	ds248x_t *	psaDS248X	;

// ################################ DS248X I2C Read/Write support ##################################


// ############################## DS248X-x00 CORE support functions ################################


// ###################################### Device debug support #####################################

int32_t	ds248xReportRegister(ds248x_t * psDS248X, int Reg, bool Refresh) ;
void	ds248xReport(ds248x_t * psDS248X, bool Refresh) ;
void	ds248xReportAll(bool Refresh) ;

// ############################### Identify, test and configure ####################################

int32_t ds248xDetect(ds248x_t * psDS248X) ;
int32_t	ds248xDeviceIdentify(i2c_dev_info_t * psI2C_DI) ;
/**
 * ds248xDriverConfig() - sets default device config
 *	1-Wire speed (c1WS) = standard (0)
 *	Strong pull-up (cSPU) = off (0)
 *	Presence pulse masking (cPPM) = off (0)		[Discontinued, support removed]
 *	Active pull-up (cAPU) = on (ds2484DCNF_APU = 0x01)
 */
int32_t	ds248xDriverConfig(i2c_dev_info_t * psI2C_DI) ;

// ############################## DS248X-x00 1-Wire support functions ##############################

int		ds248xOWChannelSelect(ds248x_t * psDS248X, uint8_t Chan) ;
int		ds248xOWSetSPU(ds248x_t * psDS248X) ;
int		ds248xOWReset(ds248x_t * psDS248X) ;
int		ds248xOWSpeed(ds248x_t * psDS248X, bool speed) ;
/**
 * Set the 1-Wire Net line level pull-up to normal. The DS248x only allows
 * enabling strong pull-up on a bit or byte event. Consequently this
 * function only allows the MODE_STANDARD argument. To enable strong pull-up
 * use OWWriteBytePower or OWReadBitPower.
 *
 * 'new_level' - new level defined as
 *					 MODE_STANDARD	  0x00
 *
 * Returns:  current 1-Wire Net level
 */
int		ds248xOWLevel(ds248x_t * psDS248X, bool level) ;
uint8_t ds248xOWTouchBit(ds248x_t * psDS248X, uint8_t sendbit) ;
void	ds248xOWWriteByte(ds248x_t * psDS248X, uint8_t sendbyte) ;
int		ds248xOWWriteBytePower(ds248x_t * psDS248X, uint8_t sendbyte) ;
uint8_t	ds248xOWReadByte(ds248x_t * psDS248X) ;
/**
 * Use the DS248x help command '1-Wire triplet' to perform one bit of a 1-Wire
 * search. This command does two read bits and one write bit. The write bit
 * is either the default direction (all device have same bit) or in case of
 * a discrepancy, the 'search_direction' parameter is used.
 *
 * Returns � The DS248x status byte result from the triplet command
 */
uint8_t ds248xOWSearchTriplet(ds248x_t * psDS248X, uint8_t search_direction) ;

#ifdef __cplusplus
}
#endif
