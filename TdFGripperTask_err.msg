!       2DFCANTASK Error code facility definition file.
!
! Description:
!       Error code defintions for the 2DFCANTASK sub-system.
!
! Support: {function author}, AAO
!
! History:
!     02-Mar-2018 - KS - Original version
!    
!
! "@(#) $Id: ACMM:TdFGripperTask/TdFGripperTask_err.msg,v 1.6 23-Aug-2021 10:48:18+10 tjf $"
!
.TITLE   TDFGRIPPERTASK_ERR  template DRAMA Task 
.FACILITY TDFGRIPPERTASK,1092/PREFIX=TDFGRIPPERTASK__
.SEVERITY ERROR
.BASE 2
AMP_ERR "Raised an error when setting up the amps"
CAMERA "CAMERA raised an error"
BUTT_IN_JAWS "Button is in the jaw"
MALLOCERR "Malloc error"
NO_GANTRY_MOVEMENT "The gantry failed to move to the target position"
NOTINIT "The task has not been initialised"
NO_ARGUMENTS "No arguments are provided"
NO_ENV_VAR "The environment variable used to find the parameter files is not defined"
NO_FIBRE "Fibre-end not in field of view"
NO_FID "Fiducial not in field of view"
NO_IMAGE "No image is taken by the camera"
NOT_SAFEMOVE "The move is not safe"
NOT_PARKED "Gantry is not parked"
NO_BUTTON "No button is being carried"
IN_USE "The task is taking other actions"
INSUF_FIDS "Insufficent fids found to do a survey calculation"
INVWIN "Invalid centroiding window definition"
INV_PICK_POS "The pick up position is invalid"
INV_INPUT_ARGUMENT "Invalid input arguments"
INV_CENTROID "Invalid centroid value"
INV_POS "Invalid target gantry position"
PARKED "Gantry is parked"
PROG_ERROR "raised an error when running a subtask"
READ_ERROR "Check the SDS file, some fields are missing"
SIMULATION "Action not possible as we are in simulation mode"
SLALIB "Error returned by slalib function"
SEARCH "Error occured during search action"
WRONG_HEIGHT "The height of Z axis is not correct"
.END
