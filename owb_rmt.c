/*
 * owb_rmt.c - Copyright (c) 2020-23 Andre M. Maree / KSS Technologies (Pty) Ltd.
 */

#include "hal_config.h"

#if (halRMT_1W > 0)
#include "FreeRTOS_Support.h"
#include "onewire_platform.h"
#include "hal_options.h"
#include "printfx.h"
#include "syslog.h"
#include "systiming.h"								// timing debugging
#include "x_errors_events.h"
#include "x_string_general.h"

// ###################################### General macros ###########################################

#define	debugFLAG					0xF000

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ######################################## Build macros ###########################################


// ##################################### Local structures ##########################################


// ###################################### Local variables ##########################################


// ##################################### Global variables ##########################################

u8_t rmtCount	= 0;
owb_rmt_t * psaRMT = NULL;

// ##################################### Forward declarations ######################################


// #################################### DS248x debug/reporting #####################################

void ow_rmtReport(report_t * psR, owb_rmt_t * psRMT) {}

void ow_rmtReportAll(report_t * psR) {}

// ################################## DS248x-x00 1-Wire functions ##################################

int	rmtOWReset(owb_rmt_t * psRMT) { esp_err_t onewire_bus_reset(onewire_bus_handle_t handle); }

int	rmtOWSpeed(owb_rmt_t * psRMT, bool speed) {}

int	rmtOWLevel(owb_rmt_t * psRMT, bool level) {}

bool rmtOWWriteBit(owb_rmt_t * psRMT, bool Bit) { esp_err_t onewire_bus_write_bit(onewire_bus_handle_t handle, uint8_t tx_bit); }

bool rmtOWReadBit(owb_rmt_t * psRMT, bool Bit) { esp_err_t onewire_bus_read_bit(onewire_bus_handle_t handle, uint8_t *rx_bit); }

u8_t rmtOWWriteByte(owb_rmt_t * psRMT, u8_t Byte) {}

u8_t rmtOWWriteBytes(owb_rmt_t * psRMT, u8_t * Bytes) {
	esp_err_t onewire_bus_read_bytes(onewire_bus_handle_t handle, uint8_t *rx_data, size_t rx_data_size)
}

u8_t rmtOWReadByte(owb_rmt_t * psRMT) {}

u8_t rmtOWSearchTriplet(owb_rmt_t * psRMT, u8_t u8Dir) {}
#endif
