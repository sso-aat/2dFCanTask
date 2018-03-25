/*
 *+			2 D F C A N T A S K

 * Program name:
      {Template DRAMA Task}

 * Function:
      {one line description of program}

 * Description:
      This module implements a template DRAMA task.  It can be
      used to start writing a DRAMA task.  Please edit this
      comment appropiately.

 * Language:
      C

 * Command:
      {program invokation}

 * Parameters:   (">" input, "!" modified, "W" workspace, "<" output)
      {parameters}

 * Returned value:
      {description of return value, if any}

 * See Also: {references}

 * External functions used:
      {external functions}

 * External values used:
      {external variables referenced}

 * Prior requirements:
      {List any prior requirements}

 * Support: {function author}, AAO

 *-

 * "@(#) $Id: ACMM:2dFCanTask/2dfcantaskmain.c,v 1.1 26-Mar-2018 08:48:13+10 ks $"

 *  History:
     02-Mar-2018 - KS - Original version

 *  Copyright (c) Australian Astronomical Observatory, 2018.
    Not to be used for commercial purposes without permission.
    No warranty of any form unless subject to a specific agreement.

 */
/*
 * RCS id
 */
static const char *rcsId="@(#) $Id: ACMM:2dFCanTask/2dfcantaskmain.c,v 1.1 26-Mar-2018 08:48:13+10 ks $";
static void *use_rcsId = (0 ? (void *)(&use_rcsId) : (void *) &rcsId);
/*
 *  Include files
 */
#include "2dfcantask.h"                        /* Our stuff     */

#include "DitsSys.h"                    /* Dits System function         */
#include "DitsParam.h"                  /* Dits Parameter system stuff  */  
#include "Sdp.h"                        /* Simple dits parameter system */
#include "Git.h"                        /* Generic Dits instrument task */
#include "mess.h"                       /* For MessGetMsg               */


/* 
 * Constant definitions
 * 
 * GLOBAL_BUF_SIZE is the size of the DRAMA global buffer allocated
 * by this task when it starts up.  The default value of 15000 is sufficent 
 * only for small tasks which don't expect to be receiving big messages
 * and which don't expect multiple tasks to be talking to them at the
 * same time.  Increasing this increases the shared memory used by the
 * task - so don't worry too much for Unix/Linux tasks - just increase it as
 * needed.  For VxWorks and other small memory systems, be more carefull.
 * The DRAMA talk (ACMM Sub-system DramaTalk) power point slides discuss
 * this in more detail.
 */
#define GLOBAL_BUF_SIZE 15000
/*
 * Macro function definitions
 */

/*
 * type definitions
 */

/*
 * Global variables and functions imported (but not declared in include files)
 */

/*
 * Global variables exported (definitions and initialization of globals
 * declared in the associated include file)
 */

/*
 * Variables private to this module (static variables)
 */


/*
 *  The entry function for the task. Under system such as VxWorks, this will
 *  be 2dFCanTaskMain.   Under other systems it will be main.
 */


#ifdef DITS_MAIN_NEEDED
    int main()
#else
    int 2dFCanTaskMain()
#endif
{
    StatusType status = STATUS__OK;         /* Status variable          */

    SdsIdType parsysid = 0;             /* For parameter system id  */
    

/*
 *  First, initialse the parameter system and dits. 
 */
    DitsAppInit("2DFCANTASK", GLOBAL_BUF_SIZE ,0, 0,&status);
    SdpInit(&parsysid,&status);
/*
 *  Initialise TPI module used by Git
 */
    GitTpiInit(&status);
/*
 *  Activate the modules we use.  This enables action handlers and creates
 *  parameters
 */
    GitActivate(parsysid,&status);  /* Generic instrument task */
    2dFCanTaskActivate(parsysid,&status);  /* This module */
/*
 *  Loop receiving messages.
 */
    DitsMainLoop(&status);
/*
 *  Tidy up, ensure we do so even if status is bad.
 */
    {
        StatusType status2 = STATUS__OK;
        2dFCanTaskDeActivate(&status2);        /* Tidy up */
        if (status == STATUS__OK) status = status2;
    }
/*
 *  Shutdown DRAMA and exit. 
 */
    return(DitsStop("2DFCANTASK",&status));
}
