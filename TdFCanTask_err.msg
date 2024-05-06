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
! "@(#) $Id: ACMM:2dFCanTask/2dFCanTask_err.msg,v 1.6 23-Aug-2021 10:48:18+10 tjf $"
!
.TITLE   TDFCANTASK_ERR  template DRAMA Task 
.FACILITY TDFCANTASK,1092/PREFIX=TDFCANTASK__
.SEVERITY ERROR
.BASE 2
MALLOCERR "Malloc error"
NOTINIT "The task has not been initialised"
NO_ENV_VAR "The environment variable used to find the parameter files is not defined"
NO_FIBRE "Fibre-end not in field of view"
NO_FID "Fiducial not in field of view"
NOT_PARKED "Gantry is not parked"
INSUF_FIDS "Insufficent fids found to do a survey calculation"
INVWIN "Invalid centroiding window definition"
INV_CENTROID "Invalid centroid value"
INV_POS "Invalid target gantry position"
PARKED "Gantry is parked"
READ_ERROR "Check the SDS file, some fields are missing"
SIMULATION "Action not possible as we are in simulation mode"
SLALIB "Error returned by slalib function"
.END
