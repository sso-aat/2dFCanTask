#ifndef _2DFCANTASK_INC
#define _2DFCANTASK_INC

/*
 *+			{ n a m e }

 * Include File name:
      {name}

 * Function:
      {one line description of include file}

 * Description:
      {detailed description of include file}

 * Language:
      C/C++   (See blow for a changes required between one and another)


 * Support: {function author}, AAO

 *-

 * "@(#) $Id: ACMM:2dFCanTask/2dfcantask.h,v 1.2 28-Mar-2018 08:58:40+10 ks $"

 * History:
     02-Mar-2018 - KS - Original version

 *  Copyright (c) Australian Astronomical Observatory, 2018.
    Not to be used for commercial purposes without permission.
    No warranty of any form unless subject to a specific agreement.

 */

/*
 *  Include files
 */
#include "DitsTypes.h"                  /* Basic dits types             */
#include "Dits_Err.h"                   /* Dits error codes             */
#include "sds.h"                        /* SDS stuff                    */
/* 
 * Constant definitions
 */

/*
 * Macro function definitions
 */

/*
 * type definitions
 */

/*
 * Have this only if this is a C header, not if it is a C++ header, it allows
 * a C header to be used by a C++ module.
 */
#ifdef __cplusplus
extern "C" {
#endif

/*
 * exported global variable declarations. (Should not really have any, use
 * function interfaces instead)
 */

/*
 * exported function prototypes
 */
extern void 2dFCanTaskActivate (SdsIdType parsysid, StatusType *status);
extern void 2dFCanTaskDeActivate (StatusType *status);


/*
 * Have this only if this is a C header, not if it is a C++ header, it allows
 * a C header to be used by a C++ module.
 */
#ifdef __cplusplus
}
#endif
#endif
