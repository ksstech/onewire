/*
 * Copyright 2018-21 Andre M. Maree/KSS Technologies (Pty) Ltd.
 */

#include	"hal_config.h"
#include	"ds18x20_cmds.h"
#include	"onewire_platform.h"
#include	"commands.h"

#include	"x_errors_events.h"
#include	"x_string_to_values.h"
#include	"x_string_general.h"

#define	debugFLAG					0xF000

#define	debugCONFIG					(debugFLAG & 0x0001)
#define	debugREAD					(debugFLAG & 0x0002)
#define	debugCONVERT				(debugFLAG & 0x0004)
#define	debugPOWER					(debugFLAG & 0x0008)

#define	debugTIMING					(debugFLAG_GLOBAL & debugFLAG & 0x1000)
#define	debugTRACK					(debugFLAG_GLOBAL & debugFLAG & 0x2000)
#define	debugPARAM					(debugFLAG_GLOBAL & debugFLAG & 0x4000)
#define	debugRESULT					(debugFLAG_GLOBAL & debugFLAG & 0x8000)

// ######################################### Constants #############################################


// ##################################### CLI functionality #########################################

int	CmndDS18RDSP(cli_t * psCLI) {
	do ds18x20ReadSP(&psaDS18X20[psCLI->z64Var.x64.x8[0].u8++], 9) ; while (psCLI->z64Var.x64.x8[0].u8 < psCLI->z64Var.x64.x8[1].u8) ;
	return erSUCCESS ;
}

int	CmndDS18WRSP(cli_t * psCLI) {
	do ds18x20WriteSP(&psaDS18X20[psCLI->z64Var.x64.x8[0].u8++]) ; while (psCLI->z64Var.x64.x8[0].u8 < psCLI->z64Var.x64.x8[1].u8) ;
	return erSUCCESS ;
}

int	CmndDS18WREE(cli_t * psCLI) {
	do ds18x20WriteEE(&psaDS18X20[psCLI->z64Var.x64.x8[0].u8++]) ; while(psCLI->z64Var.x64.x8[0].u8 < psCLI->z64Var.x64.x8[1].u8) ;
	return erSUCCESS ;
}

cmnd_t saDS18Cmnd[] = {
	{ "RDSP",	CmndDS18RDSP },
	{ "WRSP",	CmndDS18WRSP },
	{ "WREE",	CmndDS18WREE },
} ;

int	CmndDS18(cli_t * psCLI) {
	int iRV = erFAILURE ;
	psCLI->pasList	= saDS18Cmnd ;
	psCLI->u8LSize	= NO_MEM(saDS18Cmnd) ;
	psCLI->pcParse	+= xStringSkipDelim(psCLI->pcParse, sepSPACE_COMMA, psCLI->pcStore - psCLI->pcParse ) ;
	int	i32SC = xCLImatch(psCLI) ;
	if (i32SC >= 0) {
		char * pTmp = pcStringParseValueRange(psCLI->pcParse, (px_t) &psCLI->z64Var.x64.x32[0].u32, vfUXX, vs32B, sepSPACE_LF, (x32_t) 0, (x32_t) ((uint32_t) Fam10_28Count)) ;
		if (pTmp != pcFAILURE) {
			psCLI->pcParse = pTmp ;
			psCLI->z64Var.x64.x8[0].u8 = (psCLI->z64Var.x64.x32[0].u32 == Fam10_28Count) ? 0 : psCLI->z64Var.x64.x32[0].u32 ;
			psCLI->z64Var.x64.x8[1].u8 = (psCLI->z64Var.x64.x32[0].u32 == Fam10_28Count) ? psCLI->z64Var.x64.x32[0].u32 : psCLI->z64Var.x64.x8[0].u8 ;
			iRV = psCLI->pasList[i32SC].hdlr(psCLI) ;
		}
	}
	return iRV ;
}
