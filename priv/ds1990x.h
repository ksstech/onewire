// ds1990x.h

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
#if (cmakeAEP == 2)
struct rule_t;
char * pcEpDS1990_CMD(struct rule_t * psR, char * pSrc);	// CMD /ow/ds1990x 0 <chan> <rom>
#endif

#ifdef __cplusplus
}
#endif
