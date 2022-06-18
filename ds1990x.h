/*
 * ds1990x.h
 * Copyright (c) 2018-22 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma		once

#ifdef __cplusplus
extern "C" {
#endif

// ############################################# Macros ############################################


// ######################################## Enumerations ###########################################


// ######################################### Structures ############################################


// ###################################### Public variables #########################################

extern u8_t	Family01Count;

// ###################################### Public functions #########################################

void ds1990xConfig(void) ;
int	OWP_DS1990ScanCB(flagmask_t, owdi_t *) ;
int	OWP_DS1990ScanAll(epw_t * psEWP) ;

#ifdef __cplusplus
}
#endif
