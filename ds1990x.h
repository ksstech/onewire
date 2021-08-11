/*
 * Copyright 2018-21 Andre M. Maree/KSS Technologies (Pty) Ltd.
 * ds1990x.h
 */

#pragma		once

#ifdef __cplusplus
extern "C" {
#endif

// ############################################# Macros ############################################

#define	ds1990xT_SNS_NORM			1000

// ######################################## Enumerations ###########################################


// ######################################### Structures ############################################


// ###################################### Public variables #########################################

extern	uint8_t	Family01Count, ds1990ReadIntvl ;

// ###################################### Public functions #########################################

int	ds1990xConfig(void) ;

#ifdef __cplusplus
}
#endif
