/*
 * Copyright 2020-21 Andre M. Maree/KSS Technologies (Pty) Ltd.
 * ds18x20_cmds.h
 */

#pragma		once

#include	"commands.h"

#ifdef __cplusplus
extern "C" {
#endif

// ####################################### Global functions ########################################

int	CmndDS18RDT(cli_t * psCLI) ;
int	CmndDS18RDSP(cli_t * psCLI) ;
int	CmndDS18WRSP(cli_t * psCLI) ;
int	CmndDS18WREE(cli_t * psCLI) ;
int CmndDS18(cli_t * psCLI) ;

#ifdef __cplusplus
}
#endif
