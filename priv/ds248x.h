// ds248x.h

#pragma once

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

// ############################### Normal & Overdrive (uS) delays ##################################

// tRSTL=72/600uS  tRSTH=74/584  tSLOT=11/70
#define	owDELAY_RST					1148U				// 600 + 584 + 0.2625
#define	owDELAY_RB					560U				// (8 * 70) + 0.2625
#define	owDELAY_WB					560U				// (8 * 70) + 0.2625
#define	owDELAY_ST					210U				// (3 * 70) + 0.2625
#define	owDELAY_SB					70U					// (1 * 70) + 0.2625

#define	owDELAY_RST_OD				146U				// 72 + 74 + 0.2625
#define	owDELAY_RB_OD				88U					// (8 * 11) + 0.2625
#define	owDELAY_WB_OD				88U					// (8 * 11) + 0.2625
#define	owDELAY_ST_OD				33U					// (3 * 11) + 0.2625
#define	owDELAY_SB_OD				11U					// (1 * 11) + 0.2625

// ######################################## Enumerations ###########################################

enum {													// DS248X register numbers
	ds248xREG_STAT,										// STATus (all)
	ds248xREG_DATA,										// DATA (all)
	ds248xREG_CHAN,										// CHANnel (DS2482-800 only, used to detect -10x vs -800)
	ds248xREG_CONF,										// CONFiguration (all)
	ds248xREG_PADJ,										// Port Adjust (DS2484 only)
	ds248xREG_NUM,
};

enum {													// STATus register bitmap
	ds248xSTAT_1WB		= (1 << 0),						// 1W Busy
	ds248xSTAT_PPD		= (1 << 1),						// Presence Pulse Detected
	ds248xSTAT_SD		= (1 << 2),						// Short Detected
	ds248xSTAT_LL		= (1 << 3),						// Logic Level
	ds248xSTAT_RST		= (1 << 4),						// ReSeT
	ds248xSTAT_SBR		= (1 << 5),						// Single Bit Received
	ds248xSTAT_TSB		= (1 << 6),						// Triple Search Bit
	ds248xSTAT_DIR		= (1 << 7),						// DIRection
};

// ######################################### Structures ############################################

// See http://www.catb.org/esr/structure-packing/
// Also http://c0x.coding-guidelines.com/6.7.2.1.html

struct i2c_di_t;

typedef struct __attribute__((packed)) ds248x_t {		// DS248X I2C <> 1Wire bridge
	struct i2c_di_t * psI2C;		// size = 4
	SemaphoreHandle_t mux;			// size = 4
#if (HAL_DS18X20 > 0)		        // size = 4
	TimerHandle_t th;
	#define DS18X20x1	sizeof(TimerHandle_t)
#else
	#define DS18X20x1	0
#endif
	StaticTimer_t ts;
	union {							// size = 9
		struct {
			union {					// STATus register
				struct {
			/*LSB*/	u8_t OWB : 1;	// 1-Wire Busy
					u8_t PPD : 1;	// Presence Pulse Detected
					u8_t SD : 1;
					u8_t LL : 1;	// Link Level
					u8_t RST : 1;	// ReSeT
					u8_t SBR : 1;	// Single Bit Read
					u8_t TSB : 1;
			/*MSB*/	u8_t DIR : 1;	// DIRection
				};
				u8_t Rstat;
			};
			u8_t Rdata;				// DATA register
			u8_t Rchan;				// DS2482-800 CHANnel SELect
			union {					// CONFiguration
				struct {
			/*LSB*/	u8_t APU : 1;	// Active Pull Up
					u8_t PDN : 1;	// Pull Down (DS2484 only)
					u8_t SPU : 1;	// Strong Pull Up
					u8_t OWS : 1;	// 1-Wire Speed
			/*MSB*/	u8_t RES1 : 4;
				};
				u8_t Rconf;
			};
			u8_t Rpadj[5];			// DS2484 ADJustments register
		};
		u8_t RegX[9];				// 4 + Rpadj[5]
	};
	struct {						// 3 bytes
		u8_t CurChan : 3;			// 0 -> 7
		u8_t Rptr : 3;				// 0 -> 4
		u8_t NumChan : 1;			// 0 / 1 / 8
		u8_t Sp1 : 1;
		u8_t I2Cnum	: 4;			// index into I2C Device Info table
		u8_t Lo : 4;
		u8_t Hi : 4;
		u8_t Sp2 : 4;
	};
#if	(configPRODUCTION == 0)		    // 16 bytes
	u8_t PrvStat[8];				// previous STAT reg
	u8_t PrvConf[8];				// previous CONF reg
	#define DS18X20x2	(16)
#else
	#define DS18X20x2	0
#endif
} ds248x_t;
DUMB_STATIC_ASSERT(sizeof(ds248x_t) == (4+4+ DS18X20x1 + sizeof(StaticTimer_t) + 9+3+ DS18X20x2));

// #################################### Public Data structures #####################################

extern ds248x_t * psaDS248X;

// ################################ DS248X I2C Read/Write support ##################################

// ############################## DS248X-x00 CORE support functions ################################

// ############################### Identify, test and configure ####################################

// ###################################### Device debug support #####################################

int	ds248xReset(ds248x_t * psDS248X);
int	ds248xReportRegister(struct report_t * psR, ds248x_t * psDS248X, int Reg);
int ds248xReport(struct report_t * psR, ds248x_t * psDS248X);
int ds248xReportAll(struct report_t * psR);

// ############################### Identify, test and configure ####################################

int	ds248xIdentify(struct i2c_di_t * psI2C);
int	ds248xConfig(struct i2c_di_t * psI2C);

// ############################## DS248X-x00 1-Wire support functions ##############################

int	ds248xBusSelect(ds248x_t * psDS248X, u8_t Chan);
void ds248xBusRelease(ds248x_t * psDS248X);
int	ds248xOWReset(ds248x_t * psDS248X);
int	ds248xOWSpeed(ds248x_t * psDS248X, bool speed);
int	ds248xOWLevel(ds248x_t * psDS248X, bool level);
bool ds248xOWTouchBit(ds248x_t * psDS248X, bool bit);
u8_t ds248xOWWriteByte(ds248x_t * psDS248X, u8_t sendbyte);
u8_t ds248xOWReadByte(ds248x_t * psDS248X);
/**
 * Use the DS248x help command '1-Wire triplet' to perform one bit of a 1-Wire
 * search. This command does two read bits and one write bit. The write bit
 * is either the default direction (all device have same bit) or in case of
 * a discrepancy, the 'search_direction' parameter is used.
 *
 * Returns � The DS248x status byte result from the triplet command
 */
u8_t ds248xOWSearchTriplet(ds248x_t * psDS248X, u8_t u8Dir);

#ifdef __cplusplus
}
#endif
