/*
 * ds1990x.h - Copyright (c) 2018-23 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// ############################################# Macros ############################################


// ######################################## Enumerations ###########################################


// ######################################### Structures ############################################


// ###################################### Public variables #########################################


// ###################################### Public functions #########################################

void ds1990xConfig(void);
struct epw_t;
int	ds1990Sense(struct epw_t * psEWP);

#ifdef __cplusplus
}
#endif

