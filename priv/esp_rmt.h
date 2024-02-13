// esp_rmt.h -  Copyright (c) 2023-24 Andre M. Maree/KSS Technologies (Pty) Ltd.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// ######################################## Enumerations ###########################################


// ######################################### Structures ############################################

typedef struct owb_rmt_t {
	SemaphoreHandle_t mux;
	StaticTimer_t ts;
	#if (HAL_DS18X20 > 0)
	TimerHandle_t th;
	#endif
} owb_rmt_t;

// #################################### Public Data structures #####################################

extern owb_rmt_t * psaRMT;

// ###################################### Device debug support #####################################

int	owb_rmtReportRegister(owb_rmt_t * psDS248X, int Reg) ;
void owb_rmtReport(owb_rmt_t * psDS248X) ;
void owb_rmtReportAll(void) ;
int	owb_rmtBusSelect(owb_rmt_t * psDS248X, u8_t Chan) ;
void owb_rmtBusRelease(owb_rmt_t * psDS248X) ;

// #################################### 1-Wire support functions ###################################

int	rmtOWReset(owb_rmt_t * psDS248X) ;
int	rmtOWSpeed(owb_rmt_t * psDS248X, bool speed) ;
int	rmtOWLevel(owb_rmt_t * psDS248X, bool level) ;
bool rmtOWTouchBit(owb_rmt_t * psDS248X, bool bit) ;
u8_t rmtOWWriteByte(owb_rmt_t * psDS248X, u8_t sendbyte) ;
u8_t rmtOWReadByte(owb_rmt_t * psDS248X);
u8_t owb_rmtOWSearchTriplet(owb_rmt_t * psDS248X, u8_t u8Dir) ;

#ifdef __cplusplus
}
#endif
