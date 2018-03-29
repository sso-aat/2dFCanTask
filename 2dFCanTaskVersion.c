/*+		2 D F C A N T A S K V E R S I O N

 * Module name:
     2dFCanTaskVersion

 * Function:
     Provides the build time date and version for the 2DFCANTASK module.

 * Description:
     Just sets the values of the variables TdFCanTaskVersion and TdFCanTaskVerDate

 * Language:
      C

 * Support: {author}, AAO

 *-

 * "@(#) $Id: ACMM:2dFCanTask/2dFCanTaskVersion.c,v 1.4 29-Mar-2018 20:38:37+10 ks $"

 * History:
     02-Mar-2018 - KS - Original version

 *  Copyright (c) Australian Astronomical Observatory, 2018.
    Not to be used for commercial purposes without permission.
    No warranty of any form unless subject to a specific agreement.

 */
/*
 * RCS id
 */
static const char *rcsId="@(#) $Id: ACMM:2dFCanTask/2dFCanTaskVersion.c,v 1.4 29-Mar-2018 20:38:37+10 ks $";
static void *use_rcsId = (0 ? (void *)(&use_rcsId) : (void *) &rcsId);
/*
 * Variables defining the version and date.
 */
const char * const TdFCanTaskVersion= TDFCANTASK_VER;
const char * const TdFCanTaskDate   = TDFCANTASK_DATE;



