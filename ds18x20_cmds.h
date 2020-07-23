/*
 * Copyright 2020 AM Maree/KSS Technologies (Pty) Ltd.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/*
 * ds18x20_cmds.h
 */

#pragma		once

#include	"x_struct_union.h"

#ifdef __cplusplus
extern "C" {
#endif

// ####################################### Global functions ########################################

int32_t	CmndDS18RDT(cli_t * psCLI) ;
int32_t	CmndDS18RDSP(cli_t * psCLI) ;
int32_t	CmndDS18WRSP(cli_t * psCLI) ;
int32_t	CmndDS18WREE(cli_t * psCLI) ;
int32_t	CmndDS18MODE(cli_t * psCLI) ;
int32_t CmndDS18(cli_t * psCLI) ;

#ifdef __cplusplus
}
#endif
