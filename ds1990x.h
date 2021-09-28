/*
 * Copyright 2018-21 Andre M. Maree/KSS Technologies (Pty) Ltd.
 * ds1990x.h
 */

#pragma		once

#ifdef __cplusplus
extern "C" {
#endif

// ############################################# Macros ############################################


// ######################################## Enumerations ###########################################


// ######################################### Structures ############################################


// ###################################### Public variables #########################################

extern uint8_t	Family01Count;

// ###################################### Public functions #########################################

int	ds1990xConfig(void) ;
int	OWP_DS1990ScanCB(flagmask_t, owdi_t *) ;
int	OWP_DS1990ScanAll(epw_t * psEWP) ;

#ifdef __cplusplus
}
#endif
