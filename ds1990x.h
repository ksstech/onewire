/*
 * Copyright 2018-21 Andre M. Maree/KSS Technologies (Pty) Ltd.
 * ds1990x.h
 */

#pragma		once

#ifdef __cplusplus
extern "C" {
#endif

// ############################################# Macros ############################################

#define	ds1990READ_INTVL			5					// successive read interval, avoid duplicates
#define	ds1990xT_SNS_NORM			1000

// ######################################## Enumerations ###########################################


// ######################################### Structures ############################################


// ###################################### Public variables #########################################

extern	uint8_t	Family01Count, ds1990ReadIntvl ;

// ###################################### Public functions #########################################

int32_t	ds1990xDetectCB(flagmask_t, owdi_t *) ;
int32_t	ds1990xScanAll(epw_t * psEWP) ;
int	ds1990xConfig(void) ;

#ifdef __cplusplus
}
#endif
