/*+		2 D F C A N T A S K

 * Module name:
     2dfcantask

 * Function:
     {short description of module function}

 * Description:
     {more details description}

 * Language:
      C

 * Support: {author}, AAO

 *-

 * "@(#) $Id: ACMM:2dFCanTask/2dfcantask.c,v 1.4 29-Mar-2018 20:38:38+10 ks $"

 * History:
     02-Mar-2018 - KS - Original version

 *  Copyright (c) Australian Astronomical Observatory, 2018.
    Not to be used for commercial purposes without permission.
    No warranty of any form unless subject to a specific agreement.

 */
/*
 * RCS id
 */
static const char *rcsId="@(#) $Id: ACMM:2dFCanTask/2dfcantask.c,v 1.4 29-Mar-2018 20:38:38+10 ks $";
static void *use_rcsId = (0 ? (void *)(&use_rcsId) : (void *) &rcsId);
/*
 *  Include files
 */
#include "2dfcantask.h"                        /* Out include file */
#include "DitsSys.h"                    /* For PutActionHandlers        */
#include "DitsMsgOut.h"                 /* For MsgOut                   */
#include "arg.h"                        /* For ARG_ macros              */
#include "Sdp.h"                        /* Sdp routines                 */
#include "Ers.h"                        /* Ers routines                 */
#include "Git.h"                        /* Git routines                 */
#include "Git_Err.h"                    /* GIT__ codes                  */

#include "2dfcantask_err.h"
#include "mess.h"
#include "2dfcantask_err_msgt.h"
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
 *  This typedef defines a structure used to store any information specific
 *  to this library on a per-program basis (would traditionally be
 *  static/global objects under Unix)
 */
typedef struct {
    int initialised;
} 2dFCanTaskDetailsType;


/*
 * Global variables and functions imported (but not declared in include files)
 */

/*
 *  2dFCanTaskVerson and 2dFCanTaskDate are defined in the module 2dfcantaskversion.c
 */
extern const char * const 2dFCanTaskVersion;
extern const char * const 2dFCanTaskDate;


/*
 * Global variables exported (definitions and initialization of globals
 * declared in the associated include file)
 */

/*
 * Variables private to this module (static variables)
 *
 *
 *  Action defintions, first the prototypes and then the action map array
 */
static void 2dFCanTaskInitialise(StatusType *status);
static void 2dFCanTaskFilter1   (StatusType *status);
static void 2dFCanTaskFilter2   (StatusType *status);

static DitsActionMapType 2dFCanTaskMap[] = {
    /*
     * Override the GIT INITIALISE action.
     */
    {2dFCanTaskInitialise, 0, 0, "INITIALISE"},
    /*
     * Add our own task specific actions.
     */
    {2dFCanTaskFilter1,    0, 0, "FILTER1" },
    {2dFCanTaskFilter2,    0, 0, "FILTER2" }
         };
/*
 *  Now the parameter definitions.  First define some values we need addresses
 *  of and the and paramater definition array.
 */
static int one = 1;
static float two = 2.2;
static unsigned int three = 3;

static SdpParDefType 2dFCanTaskParams[] = {
/*
 * Override these Git parameters
 */
    { "ENQ_DEV_DESCR",   "{Template Drama Task}", ARG_STRING },
    { "ENQ_VER_NUM",     "",                    ARG_STRING },
    { "ENQ_VER_DATE",    "",                    ARG_STRING },
/*
 * Task specific parameters
 */
    { "PARAM1", &one,       SDS_INT    },
    { "PARAM2", "hi there", ARG_STRING },
    { "PARAM3", &two,       SDS_FLOAT  },
    { "PARAM4", &three,     SDS_UINT   }};

/*
 *  Internal Function name:
      2dFCanTaskMyDetails

 *  Description:
       Convenience function which combines a look up using GitTpiGet with
       a check that the library's initialise call has been invoked.
 
       {Example note - this does not have to be static if there i
        is more then on C module in the task which needs this }

 *  History:
     02-Mar-2018 - KS - Original version
     {@change entry@}
 */
static 2dFCanTaskDetailsType *2dFCanTaskDetails(StatusType *status)
{
    2dFCanTaskDetailsType *details = 0;

    if (*status != STATUS__OK) return 0;


    GitTpiGet(2DFCANTASK__FACNUM,
              (void *)&details,
              status);

    if (*status == GIT__TPI_NOTFOUND)
        ErsRep(0,status,"2DFCANTASK module not activated");
    else if ((*status == STATUS__OK)&&(!details->initialised))
        *status = 2DFCANTASK__NOTINIT;
    return(details);
}



/*
 *+			2 D F C A N T A S K A C T I V A T E

 * Function name:
      2dFCanTaskActivate

 * Function:
       Activate {this module's} task action handlers.

 * Description:
      Actions handlers are put using DitsPutActionHandlers and the 
      parameters  are created using SdpCreate.   The structure used
      to store internal task details is created and stored using GitTpi.

 * Language:
      C

 * Call:
      (Void) = 2dFCanTaskActivate (parsysid, status)

 *  Parameters:   (">" input, "!" modified, "W" workspace, "<" output)
      (>) parsysid      (SdsIdType) The parameter system id returned by a 
                            call to SdpInit
      (!) status        (StatusType *) Modified status. 


 * Include files: 2dFCanTask.h

 * See Also: {references}

 * External functions used:
      {external functions}

 * External values used:
      {external variables referenced}

 * Prior requirements:
      DitsAppInit and SdpInit should have been called.  Should not be called
      from a Dits action handler routine.
      {List any other prior requirements}

 * Support: {function author}, AAO

 *-

 * History:
     02-Mar-2018 - KS - Original version
 */
extern void 2dFCanTaskActivate (SdsIdType parsysid, StatusType *status)
{
    2dFCanTaskDetailsType *details;
/*
 *  Add the message facility codes which may be generated by this program.
 */
    MessPutFacility(&MessFac_2DFCANTASK);
/*
 *  Add the actions and parameters
 */
    DitsPutActionHandlers(DitsNumber(2dFCanTaskMap),2dFCanTaskMap,status);
    SdpCreate(parsysid, DitsNumber(2dFCanTaskParams),2dFCanTaskParams,status);

/*
 *  Put the version number and date parameter values
 */
    SdpPutString ("ENQ_VER_NUM",2dFCanTaskVersion,status);
    SdpPutString ("ENQ_VER_DATE",2dFCanTaskDate,status);

/*
 *  Create the details structure.
 */
    details = (2dFCanTaskDetailsType *)malloc(sizeof(2dFCanTaskDetailsType));
    if (details == 0)
    {
        *status = 2DFCANTASK__MALLOCERR;
        return;
    }
/*
 *  Store the details using GitTpi
 */
    GitTpiPut(2DFCANTASK__FACNUM,details,status);
/*
 *  Note we have not been initliased.
 */
    details->initialised = 0;
}


/*
 *+			2 D F C A N T A S K D E A C T I V A T E

 * Function name:
      2dFCanTaskDeActivate

 * Function:
       DeActivate {this module's} task action handlers.

 * Description:

 * Language:
      C

 * Call:
      (Void) = 2dFCanTaskDeActivate (parsysid, status)

 *  Parameters:   (">" input, "!" modified, "W" workspace, "<" output)
      (!) status        (StatusType *) Modified status. 


 * Include files: 2dFCanTask.h

 * See Also: {references}

 * External functions used:
      {external functions}

 * External values used:
      {external variables referenced}

 * Prior requirements:

 * Support: {function author}, AAO

 *-

 * History:
     02-Mar-2018 - KS - Original version
 */
extern void 2dFCanTaskDeActivate (StatusType *status)
{
    if (*status != STATUS__OK) return; 
}

/*
 *  Internal Function name:
      2dFCanTaskInitialise

 *  Description:
      Initialise the 2DFCANTASK task.

 *  History:
     02-Mar-2018 - KS - Original version
     {@change entry@}
 */

static void 2dFCanTaskInitialise(StatusType *status)
{
    2dFCanTaskDetailsType *details = 0;

    ErsPush();
    details = 2dFCanTaskDetails(status);

    if (*status == 2DFCANTASK__NOTINIT)
    {
        ErsAnnul(status);
        details->initialised = 1;
    }
    else if (*status == STATUS__OK)
    {
        /* Already intialised, may or may not be an error */
    }
    ErsPop();


}




/*
 *  Internal Function name:
      2dFCanTaskFilter1

 *  Description:
      2dFCanTask filter number 1 command

 *  History:
     02-Mar-2018 - KS - Original version
     {@change entry@}
 */

static void 2dFCanTaskFilter1(StatusType *status)
{
    /*
     * At the moment, the main effect of this is to check that
     * the task has been initialised - hence the resultant
     * variable is unused and is marked as such.   The DUNUSED
     * flag should be removed when the details structure is accessed.
     */
    2dFCanTaskDetailsType *details DUNUSED = 2dFCanTaskDetails(status);

    if (*status != STATUS__OK) return;

    MsgOut(status,"Moving filter %d",1);
}

/*
 *  Internal Function, name:
      2dFCanTaskFilter2

 *  Description:
      2dFCanTask filter number 1 command

 *  History:
     02-Mar-2018 - KS - Original version
      {@change entry@}
 */

static void 2dFCanTaskFilter2(StatusType *status)
{
    2dFCanTaskDetailsType *details DUNUSED = 2dFCanTaskDetails(status);

    if (*status != STATUS__OK) return;

    MsgOut(status,"Moving filter %d",2);
}
