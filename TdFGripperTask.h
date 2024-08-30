#ifndef TDFGRIPPERTASK_H
#define TDFGRIPPERTASK_H

#include "sds.h"
#include "Git.h"
#include "gcam.h"
#include "slalib.h"
#include <math.h>
#include "fitsio.h"
#include "dul.h"
#include "drama.hh"
#include <exception>

// to check if a directory exists
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

/*
 *  2dF Gripper Gantry Task definitions.
 */
#define NO                          0
#define YES                         1

#define NUM_FIDUCIALS              21  /* Number of fiducial marks on field plate    */


#define PI               3.1415926535897932384626433832795028841971693993751
#define PION2            1.5707963267948966192313216916397514420985846996876

#define HI                          2  /* Used for accuracy levels                   */
#define LOW                         1

#define MED			    4

#define _XI                          0  /* Used for array indexes                     */
#define _YI                          1

#define X1                     (1<<0)
#define X2                     (1<<1)
#define X                   (X1 | X2)
#define Y                      (1<<2)
#define Z                      (1<<3)
#define THETA                  (1<<4)
#define JAW                    (1<<5)

#define NO_FILES                    0  /* Used to identify parameter files           */
#define DEFS_FILE              (1<<0)
#define FLEX_FILE              (1<<1)

#define JAW_SYN                     0  /* Open jaws in synchronised fashion          */
#define JAW_SEP                     1  /* Open moving and fixed fingers seperately   */

#define MOVE                        1  /* Used by DitsGetCode to differentiate ...   */
#define OSET                        2  /* ... between abs and relative coordinates   */
#define NO_POLL                     0
#define POLL                        1

#define GRIP_CAM_NUM     "/grabber/0"  /* Gripper camera device name                 */
#define CAM_X_SPAN                1456  /* Pixels along the gripper camera x axis     */
#define CAM_Y_SPAN                1088  /* Pixels along the gripper camera y axis     */
#define PIXEL_SIZE                  1  /* Pixel size of Pulnix CCD gripper camera    */
#define FIBRE_IN_IMAGE_LIMIT      1000  /* If the sum of the pixel values in an image */
                                       /* is less than this we consider there to be  */
                                       /* no fibre in the image.                     */

#define ZMAP_MAX                 5000  /* Maximum allowable Zmap parameter value     */
#define ZMAP_MIN                -5000  /* Minimum allowable Zmap parameter value     */

#define CHECK_ALL                   0  /* Check mask - initialiser                   */
#define DISPLAY                (1<<0)  /*            - display message text          */
#define _DEBUG                 (1<<1)  /*            - display debug information     */
#define SHOW       (DISPLAY | _DEBUG)  /*            - display message & debug info. */
#define NO_INT_CHECK           (1<<2)  /*            - no interlock checking         */
#define HARD                   (1<<3)  /*            - hardware reset required       */
#define NO_SAVE                (1<<4)  /*            - do not update defaults file   */
#define SET_COEFFS             (1<<5)  /*            - used during SURVEY routine    */
#define BLIND                  (1<<6)  /*            - pickup button blind           */
#define TO_PARK                (1<<7)  /*            - putting button in park pos    */
#define FROM_PARK              (1<<8)  /*            - getting button from park pos  */
#define JAW_FULL               (1<<9)  /*            - open jaw to OPEN_JAW_FULL     */
#define STATS                 (1<<10)  /*            - update stats structure        */
#define TIME1		      (1<<11)  /*	     - Timing information mode 1     */
#define TIME2		      (1<<12)  /*	     - Timing information mode 2     */

#define DONT_ITERATE          (1<<13)  /*            - Don't iterate during putdown */
#define NO_ZMAP               (1<<14)  /*            - Don't use zmap (MOVE_AXIS)   */

#define ILOCK__TASK_INIT       (1<<0)  /* Used to specify interlock checks           */
#define ILOCK__GAN_PARKED      (1<<1)
#define ILOCK__GAN_NOT_PARKED  (1<<2)

#define FREE_LOCKS                  1  /* Used by tdFgripStructFree to release locks */
#define LEAVE_LOCKS                 2  /* ..                           ignore     .. */

#define FILENAME_LENGTH            30  /* Max "Command File" name length             */
#define PATH_LENGTH               100  /* Max length of path name                    */
#define PMAC_RESPONSE_LENGTH      512  /* Max PMAC response length                   */

#define IMAGE                       1  /* Used when returning image                  */
#define CENTROID                    2  /* Only centroid                              */

#define _ALL                        1  /* Identify the area for the SURVEY action    */
#define _COEFFS                     2  /*  These are passed to conversion ops        */
#define _TEMP                       3
#define _FLEX                       4
#define _FULL                       5
#define _FULL_NO_ZMAP               6  /* As per FULL, but don't adjust Z for Z map */

#define GRASPED                     0  /* Button focus distances/windows             */
#define FREE                        1
#define _CENTRE                     0
#define _OFFSET                     1

#define _INIT                       1  /* Used to identify an INITIALISE action      */
#define _RESET                      2  /* ..                  RESET          ..      */

#define GR_X1                       1  /* Used to determine the PMAC status word ... */
#define GR_X2                       2  /* ... we are interested in                   */
#define GR_Y                        3
#define GR_Z                        4
#define GR_JAW                      5
#define GR_THETA                    6
#define CS_GRIP                     7
#define PMAC_0                      8

#define TDFGRIP_MSG_BUFFER      200000  /* Size of message buffer for TDFGRIP         */

#ifndef TDFPT_PARAM_DIR
#define TDFPT_PARAM_DIR "/home/lliu/Project_Codes/buildanagate/Parameters/"
#endif
/*
 * Indicates a standard deviation of a offset is invalid. (MOVE_FIBRE and
 * PUTDOWN actions).
 */
#define STDDEV_INVALID 256000
/*
 *  If not on VxWorks, must define the following.
 */
#ifndef VxWorks
#   define   ERROR -1
#endif

/*
 *  Macro definitions.
 */
#define SQRD(x)   ((x)*(x))            /* Return the square of a number              */
#define ABS(x)    ((x)<0? -(x): (x))   /* Return the absolute value of a number      */

#define FPI_XY_FR_MIN 1       /* FPI XY axes minimum feedrate value (?)        */
#define FPI_XY_FR_MAX 1000000 /* FPI XY axes maximum feedrate value (?)        */
#define PLATE_FR_MIN 1        /* Plate rotators minimum feedrate value (?)     */
#define PLATE_FR_MAX 3000000  /* Plate rotators maximum feedrate value (?)     */
#define XMIN -108396            /* X-axis minimum value (microns)                */
#define XMAX 110000           /* X-axis maximum value (microns)                */
#define YMIN -110963            /* Y-axis minimum value (microns)                */
#define YMAX 123059           /* Y-axis maximum value (microns)                */
#define ZMIN 0                /* Z-axis minimum value (microns)                */
#define ZMAX 60000            /* Z-axis maximum value (microns)                */

#define QUADRANT_RADIUS 277000
#define INSIDE_RADIUS 270430
#define OUTSIDE_RADIUS 347300
#define HALF_GUIDE_EXPAN 2250

#define JAW_HWP 12220   /* Jaw half with +ve theta */
#define JAW_HWM 8300    /* Jaw half with -ve theta */
#define JAW_LENGTH 5250 /* Jaw length		*/

#define TDFFPI_MSG_BUFFER 20000 /* Size of message buffer for TDFFPI          */

#define FPI_CLEAR_X 253513
#define FPI_CLEAR_Y -240488

#define TAN4P5 0.07870171
#define SIN9 0.156434465
#define COS9 0.987688341
#define SIN4P5 0.0784590957
#define COS4P5 0.9969173337
#define CLEARANCE 500.0
#define PI 3.1415926535
#define TWOPI 6.283185308
#define PION2 1.570796327
#define R4P5 0.07853981634
#define R9 0.1570796327
#define SF_IMAGES 10

#define MAX_POINTS 25
#define GRID_SIZE 10

#define GLOBAL_BUFFER_SPACE 21000000

#ifndef SAFE
#define SAFE 2
#endif
/*
 *  Structure definitions required for the main tdFgrip task structure.
 */

/*
 *  Z-Map details.  These are in microns so could be as
 *  int, but since they are used in calcuations, keep as double.
 */
typedef struct tdFzmap {
    double  _cen;
    double  _xm;
    double  _xp;
    double  _ym;
    double  _yp;
    }  tdFzmap;

/*
 *  Gripper encoder position.
 */
typedef struct tdFencPos {
    int  theta;
    int  x;
    int  y;
    int  z;
    int  jaw;
    }  tdFencPos;

/*
 *  Ideal gripper gantry position.
 */
typedef struct tdFganPos {
    double    theta;
    long int  x;
    long int  y;
    long int  z;
    long int  jaw;
    }  tdFganPos;
    
 
/*
 *  This type is used by our routines which work out the centroid position
 *  at the time of the centroid (tdFgripPreExp and tdFgripPostExp).  We
 *  actually keep this in the tdFgripTaskType structure.  
 */    
typedef struct tdFimagePos {
    int       enable;		/* Enable the operation */
    short     useDpr;		/* Do we use the DPR	*/
    short     displayText;	/* Display text data	*/
    tdFganPos p;		/* Gantry position during centroid */
    tdFencPos  enc;		/* Encoder value        */ 
    } tdFimagePos;

/*
 *  Camera parameters.
 */
typedef struct tdFcamImage {
    GCamShutterType  shutter;
    double           camCoeffs[6];
    double           invCoeffs[6];
    float            updateTime;
    float            exposureTime;
    int              camNo;
    int              bias;
    int              xMax;
    int              yMax;
    int              PixelSize;
    }  tdFcamImage;

typedef struct tdFcamWindow {
    double           xCen;
    double           yCen;
    int              xSpan;
    int              ySpan;
    }  tdFcamWindow;

/*
 *  Transformation parameters.  Used to convert between field-plate and encoder units.
 */
typedef struct tdFconvert {
    double       coeffs[6];     /* Convert field->plate coords     */
    double       invCoeffs[6];  /* Convert plate->field coords     */
    int          thetaShift;    /* For theta axis conversion       */
    int          jawShift;      /* For jaw axis conversion         */
    int          zShift;        /* For z axis conversion           */
    TdfFlexType  flex;          /* System flexure model details    */
    tdFzmap      zmap;          /* Z-map details                   */
    }  tdFconvert;
/*
 *  Parameter lookup.  We store here the values returned by SdsPointer for
 *  parameters we used often
 */
typedef struct {
    short  *backIllAlways;     /* Is backillumination always on */
    double *backIllWarmUp;     /* backillumination warm up time */
    double *zeroCamCenWait;	/* zero cam gantry settle time before centroid*/
    double *distPar1;          /* Gripper/FPI distortion removal parameters */
    double *distPar2;
    double *distPar3;
    double *distPar4;
    double *distPar5;
    double *distPar6;
    double *distPar7;
    double *distPar8;
    double *distPar9;
    double *distPar10;
    double *distPar11;
    double *distPar12;
    double *distPar13;
    double *distPar14;
    double *distPar15;
    double *distPar16;
    double *distPar17;
    double *distPar18;
    short  *distRemEnable;    /* Is distortion removal enabled/disabled 
                                 see  calculation in tdFgripCvt.c */
    
    short  *plt1CenterFidOffsetX; 
    short  *plt1CenterFidOffsetY;
    } tdFgripParLookup;


/*
 * Distortion map - hold the Cannon distortion map.
 */
#define GRID_WIDTH 23
#define GRID_STEP (318.0*66.66667)  /* Step between grid elements - from Russels program. */
#define GRID_CENTRE_OFFSET  GRID_STEP*((double)(GRID_WIDTH+1)/2.0) /* Offset to centre of grid */

typedef struct {
    int x[GRID_WIDTH*GRID_WIDTH];
    int y[GRID_WIDTH*GRID_WIDTH];
    int loaded;
} tdFgripDistMap;

/*
 *  The tdFgrip main task structure.
 */
typedef struct {    
    GitSimulationType  Simulation;    /* Simulation level                */
    tdFcamImage        freeImg;       /* Image parameters for free focus */
    tdFcamImage        graspImg;      /*   ''     ''          grasp  ''  */
    tdFcamWindow       searchWin;     /* Def'n of search image window    */
    tdFcamWindow       normWin;       /*   ''     norm     ''   ''       */
    tdFconvert         convert;       /* FP<-->enc conversion parameters */
    tdFencPos          toEnc;         /* Target encoder position         */
    tdFencPos          atEnc;         /* Current encoder position        */
    tdFganPos          ideal;         /* Ideal gantry position           */
    tdFimagePos	       imagePos;     /* Use to get position during image */
    float              TimeBase;      /* Simulation time base            */
    int                dprAddress;    /* Address of dual ported ram      */
    short              Initialised;   /* Task initialised flag           */
    short              cameraInit;    /* Camera initialised flag         */
    short              dprFeedback;   /* Get axis/status info from dpr?  */
    short              inUse;         /* Motion semaphore                */
    short	       backIllumOn;   /* Is back illumination on	 */
    short	       illumPlate;    /* If on, which plate		 */
    short	       configPlate;   /* Config plate recorded here	 */
    short              plateOneDontRemove; /* Set true to turn off the 
                                              removal of the plate 1 offset */
    short               ipsMode;       /* Is IPS camera mode enabled       */
    tdFgripParLookup   pars;	      /* SdsPointers to parameters for quick */
				      /* access				 */   
    tdFgripDistMap     distMap;       /* Cannon distortion map           */
    }  tdFgripTaskType;


/*
 *  Structure definitions for action parameters (handled by DitsPutActData() and
 *  DitsGetActData()).
 */

/*
 *  This is the structure that is stored for all parameters.
 */
typedef struct  tdFgripDataType {
      void                    *userData;
      short                   usingLock;
      short                   check;
      short                   type;
      struct tdFgripDataType  *more;
      }  tdFgripDataType;

/*
 *  The following structures are stored in the "userData" section of the
 *  tdFgripDataType structure.
 */
#define  IN_TYPE     1     /* Used by tdFgripSysInit */
typedef struct tdFgrip_INtype {
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      short                  process;
      short                  axisWord;
      short                  reset;
      }  tdFgrip_INtype;

#define  SE_TYPE     2     /* Used by tdFgripSysExit */
typedef struct tdFgrip_SEtype {
      short                  reset;
      }  tdFgrip_SEtype;

#define  MB_TYPE      3    /* Used by tdFgripMoveButton */
typedef struct tdFgrip_MBtype {
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      double                 atTheta;
      double                 toTheta;
      double		     theta;	/* Resultant button theta position */
      double                 stdDevX;   /* Iteration offset FIFO  standard deviation */
      double                 stdDevY; 
      long int               atX;
      long int               atY;
      long int               toX;
      long int               toY;
      long int               xErr;
      long int               yErr;
      long int               xBut;
      long int               yBut;
      long int               xOff;     /* Positioning offset to presume */
      long int               yOff;
      long int               xOffIt;   /* Positioning offset for iterations */
      long int               yOffIt;
      long int               graspX;   /* Grasp offsets */
      long int               graspY;
      long int               actXoff;   
      long int               actYoff;
      long int               actGraspX; /* Actual grasp offset in x   */
      long int               actGraspY; /* Actual grasp offset in y   */
      long int		     xf;	/* Resultant x fibre position */
      long int		     yf;	/* Resultant y fibre position */
      long int               buttonZ;   /* Calculated by PutDown      */
      short                  inTolerance;
      short                  iterations;
      short                  reset;
      unsigned short         fibreNum; 
      unsigned short         fifoNum;  /* Number of values in iteration offset
                                          fifo */
      unsigned short         guideBut;  /* Is a guide button */ 

      }  tdFgrip_MBtype;

#define  MBC_TYPE     4    /* Stored by tdFgripMoveButton after completion */
typedef struct tdFgrip_MBCtype {
      double		     theta;	/* Resultant button theta position */
      long int               xErr;
      long int               yErr;
      long int               xBut;
      long int               yBut;
      long int               actXoff;
      long int               actYoff;
      long int		     xf;	/* Resultant x fibre position */
      long int		     yf;	/* Resultant y fibre position */
      long int               actGraspX;
      long int               actGraspY;
      short                  inTolerance;
      short                  iterations;
      unsigned short         fibreNum; 
      }  tdFgrip_MBCtype;

#define  PUB_TYPE     5    /* Used by tdFgripPickUpButton */
typedef struct tdFgrip_PUBtype {
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      double                 atTheta;
      long int               atX;
      long int               atY;
      long int               graspX;
      long int               graspY;
      long int               xErr;
      long int               yErr;
      short                  centroidOK;
      short                  reset;
      unsigned short         fibreNum; 
      unsigned short         guideBut;  /* Is a guide button */ 
      }  tdFgrip_PUBtype;

#define  PDB_TYPE     6    /* Used by tdFgripPutDownButton */
typedef struct tdFgrip_PDBtype {
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      double                 toTheta;
      double                 stdDevX;   /* Iteration offset FIFO  standard deviation */
      double                 stdDevY; 
      long int               toX;
      long int               toY;
      long int               graspX;
      long int               graspY;
      long int               xOff;     /* Positioning offset to presume */
      long int               yOff;
      long int               xOffIt;   /* Positioning offset for iterations */
      long int               yOffIt;
      long int               xErr;
      long int               yErr;
      short                  centroidOK;
      short                  reset;
      unsigned short         fibreNum; 
      unsigned short         fifoNum;  /* Number of values in iteration offset
                                          fifo */
      unsigned short         guideBut;  /* Is a guide button */ 
      }  tdFgrip_PDBtype;

#define  PDC_TYPE     7    /* Returned by tdFgripPutDownButton */
typedef struct tdFgrip_PDCtype {
      double		     theta;	/* Resultant button theta position */
      long int               xErr;      /* Putdown error                   */
      long int               yErr;
      long int               xBut;      /* Actual button position          */
      long int               yBut;
      long int               actXoff;   /* Actual positioning offset in x  */
      long int               actYoff;   /* Actual positioning offset in y  */
      long int		     xf;	/* Resultant x fibre position */
      long int		     yf;	/* Resultant y fibre position */
      long int               actGraspX; /* Actual grasp offset in x   */
      long int               actGraspY; /* Actual grasp offset in y   */
      long int               buttonZ;   /* Calculated button Z */
      short                  inTolerance;
      short                  iterations;
      unsigned short         fibreNum; 
      }  tdFgrip_PDCtype;

#define  XYZT_TYPE    8    /* Used by tdFgripMoveXYZT */
typedef struct tdFgrip_XYZTtype {
      const char *           operation;
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      double                 toTheta;
      long int               toX;
      long int               toY;
      long int               toZ;
      }  tdFgrip_XYZTtype;

#define  MA_TYPE      9    /* Used by tdFgripMoveAxis */
typedef struct tdFgrip_MAtype {
      const char *           operation;
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      double                 tarAxis;
      short                  axis;
      }  tdFgrip_MAtype;

#define  JO_TYPE      10   /* Used by tdFgripJawOpen */
typedef struct tdFgrip_JOtype {
      const char *           operation;
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      long int               jawOpenDist;
      long int               handleWidth;
      long int               toZ;   /* Used for method = JAW_SEP only */
      short                  method;
      short                  reset;
      }  tdFgrip_JOtype;

#define  JS_TYPE      11   /* Used by tdFgripJawShut */
typedef struct tdFgrip_JStype {
      const char *           operation;
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      long int               atX;
      long int               atY;
      short                  method;
      short                  reset;
      unsigned short         guideBut;  /* Is a guide button */ 
      }  tdFgrip_JStype;

#define  HA_TYPE      12   /* Used by tdFgripHomeAxis */
typedef struct tdFgrip_HAtype {
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      int                    axis;
      }  tdFgrip_HAtype;

#define  PG_TYPE      13   /* Used by tdFgripParkGan */
typedef struct tdFgrip_PGtype {
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      short                  axisWord;
      short                  ganParked;
      }  tdFgrip_PGtype;

#define  UPG_TYPE     14   /* Used by tdFgripUnParkGan */
typedef struct tdFgrip_UPGtype {
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      }  tdFgrip_UPGtype;

#define  SF_TYPE      15   /* Used by tdFgripSearch */
#define  SF_IMAGES    10
typedef struct tdFgrip_SFtype {
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      long int               searchStartX;
      long int               searchStartY;
      long int               searchStartZ;
      long int               stepSize;
      long int               maxError;
      long int               tolerance;
      long int               xErr;
      long int               yErr;
      short                  centroidOK;
      short                  attempts;
      short                  reset;
      short                  resultsValid;
      short                  statCheck;
      short                  peakVal;
      short                  saturated;
      long int               resultsX[SF_IMAGES];
      long int               resultsY[SF_IMAGES];

      }  tdFgrip_SFtype;

#define  SC_TYPE      16   /* Stored by tdFgripSearch after completion */
typedef struct tdFgrip_SCtype {
      long int  xf;
      long int  yf;
      long int  dx;
      long int  dy;
      long int  xErr;
      long int  yErr;
      int       xEnc;
      int       yEnc;
      short     found;
      }  tdFgrip_SCtype;

#define  S_TYPE       17   /* Used by tdFgripSurvey */
typedef struct tdFgrip_Stype {
      double    temp;
      long int  x[NUM_FIDUCIALS];
      long int  y[NUM_FIDUCIALS];
      long int  z[NUM_FIDUCIALS];
      long int  fidNum[NUM_FIDUCIALS];  /* fiducial number */
      long int  dx;
      long int  dy;
      int       xEnc;
      int       yEnc;
      short     found;
      short     numMarks;
      short     area;
      short     reset;
      }  tdFgrip_Stype;

#define  BR_TYPE      18   /* Used by tdFgripBrake */
typedef struct tdFgrip_BRtype {
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      short                  brakeWord;
      short                  brakesOn;
      }  tdFgrip_BRtype;

#define  SV_TYPE      19   /* Used by tdFgripServo */
typedef struct tdFgrip_SVtype {
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      short                  servoWord;
      short                  servo;
      }  tdFgrip_SVtype;

#define  ZC_TYPE      20   /* Used by tdFgripZerocam */
typedef struct tdFgrip_ZCtype {
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      double                 xFull;
      double                 yFull;
      short                  reset;
      short                  centroidOK;
      }  tdFgrip_ZCtype;

#define  SH_TYPE      21   /* Used by tdFgripShiftCoeffs */
typedef struct tdFgrip_SHtype {
      long int               expX;
      long int               expY;
      long int               focusZ;
      long int               measuredX;
      long int               measuredY;
      short                  reset;
      short                  found;
      }  tdFgrip_SHtype;

#define  CEN_TYPE     22   /* Used by tdFgripCentroid */
typedef struct tdFgrip_CENtype {
      const char *           operation;
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      GCamWindowType         window;
      tdFcamImage            *img;
      double                 settleTime;
      short                  wantImage;
      short		     tryagain;
      IMP_AbsTime	     startTime;
      char		     saveName[40];
      }  tdFgrip_CENtype;

#define  GCC_TYPE     23   /* Stored by tdFgripCentroid after completion */
typedef struct tdFgrip_GCCtype {
      long int               xErr;
      long int               yErr;
      double                 xFull;
      double                 yFull;
      short                  centroidOK;
      short                  peakVal;
      short                  saturated;
      }  tdFgrip_GCCtype;

#define  PSC_TYPE     24   /* Stored after completion of tdFgripPMACrunCmd */
typedef struct tdFgrip_PSCtype {
      char                   response[PMAC_RESPONSE_LENGTH];
      StatusType             cmdSendStatus;
      int                    progNum;
      double                 elapsed;
      }  tdFgrip_PSCtype;

#define  A_TYPE       25  /* Used by tdFgripAbort */
typedef struct tdFgrip_Atype {
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      StatusType             errorStatus;
      }  tdFgrip_Atype;

#define  AM_TYPE      26  /* Used by tdFgripAmps */
typedef struct tdFgrip_AMtype {
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      short                  motorWord;
      }  tdFgrip_AMtype;

#define  LM_TYPE      27  /* Used by tdFgripLimits */
typedef struct tdFgrip_LMtype {
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      short                  motorWord;
      short                  enable;
      }  tdFgrip_LMtype;

#define  RL_TYPE      28  /* Used by tdFgripRelays */
typedef struct tdFgrip_RLtype {
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      short                  mask;
      short                  relay;
      short                  set;
      }  tdFgrip_RLtype;

#define  RD_TYPE      29  /* Returned by tdFgripRelays */
typedef struct tdFgrip_RDtype {
      short  relayState;
      }  tdFgrip_RDtype;

#define  PH_TYPE      30  /* Used by tdFgripPhase */
typedef struct tdFgrip_PHtype {
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      short                  motorWord;
      }  tdFgrip_PHtype;

#define  UD_TYPE      31  /* Used by tdFgripUpdate */
typedef struct tdFgrip_UDtype {
      double  updateInterval;
      }  tdFgrip_UDtype;

#define  MEAS_Z_TYPE    32    /* Used by tdFgripMeasureZHeight */
typedef struct tdFgrip_MEAS_Ztype {
    /* DitsActionRoutineType  handler; */
    /* DitsActionRoutineType  kickHandler; */
      double                 theta;
      long int               x;
      long int               y;
      long int               height;
      short                  reset;
      }  tdFgrip_MEAS_Ztype;


#define  CHKCEN_TYPE     33    /* Used by tdFgripCheckCen */
typedef struct tdFgrip_CHKCENtype {
      DitsActionRoutineType  handler;
      DitsActionRoutineType  kickHandler;
      double                 atTheta;
      long int               atX;
      long int               atY;
      long int               graspX;
      long int               graspY;
      long int               xSearchErr;
      long int               ySearchErr;
      long int               xPickupErr;
      long int               yPickupErr;
      long int               xErr;
      long int               yErr;
      short                  centroidOK;
      short                  reset;
      unsigned short         fibreNum; 
      }  tdFgrip_CHKCENtype;


#endif