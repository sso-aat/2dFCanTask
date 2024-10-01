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
#include "CanAccess.h"
#include "CML.h"
#include "SimCanInterface.h"
#include "TcsUtil.h"
#include <vector>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>

// to check if a directory exists
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include "TdFGripperTask_err.h"
#include "TdFGripperTask_err_msgt.h"

#ifdef USE_CAN_ANAGATE
#include <can_anagate.h>
#else
#define AnaGateCan SimCanInterface
#endif

#define MAX_TDF_AMPS 6
#define NUM_PIVOTS_2DF 400 /* Number of pivots in 2dF */
#define NUM_FIDS_2DF 21    /* Number of fiducials on each 2dF field */
#define NUM_FIELDS_2DF 2   /* Number of 2dF fields */

enum AmpId
{
    X1_AMP = 0,
    X2_AMP = 1,
    Y_AMP = 2,
    Z_AMP = 3,
    JAW_AMP = 4,
    THETA_AMP = 5
};

typedef struct
{
    AmpId AxisId;    //  The axis in question.
    double Position; //  The demanded position.
    double Velocity; //  The demanded velocity. 0 => not specified.
} AxisDemand;
/*
 *  2dF Gripper Gantry Task definitions.
 */
#define NO 0
#define YES 1

#define NUM_FIDUCIALS 21 /* Number of fiducial marks on field plate    */

#define PI 3.1415926535897932384626433832795028841971693993751
#define PION2 1.5707963267948966192313216916397514420985846996876

#define HI 2 /* Used for accuracy levels                   */
#define LOW 1

#define MED 4

#define _XI 0 /* Used for array indexes                     */
#define _YI 1

#define X1 (1 << 0)
#define X2 (1 << 1)
#define X (X1 | X2)
#define Y (1 << 2)
#define Z (1 << 3)
#define THETA (1 << 4)
#define JAW (1 << 5)

#define NO_FILES 0 /* Used to identify parameter files           */
#define DEFS_FILE (1 << 0)
#define FLEX_FILE (1 << 1)

#define JAW_SYN 0 /* Open jaws in synchronised fashion          */
#define JAW_SEP 1 /* Open moving and fixed fingers seperately   */

#define MOVE 1 /* Used by DitsGetCode to differentiate ...   */
#define OSET 2 /* ... between abs and relative coordinates   */
#define NO_POLL 0
#define POLL 1
#define WIN_UPSIZE 2.0

#define GRIP_CAM_NUM "/grabber/0" /* Gripper camera device name                 */
#define CAM_X_SPAN 1456           /* Pixels along the gripper camera x axis     */
#define CAM_Y_SPAN 1088           /* Pixels along the gripper camera y axis     */
#define PIXEL_SIZE 1              /* Pixel size of Pulnix CCD gripper camera    */
#define FIBRE_IN_IMAGE_LIMIT 1000 /* If the sum of the pixel values in an image */
                                  /* is less than this we consider there to be  */
                                  /* no fibre in the image.                     */

#define ZMAP_MAX 5000  /* Maximum allowable Zmap parameter value     */
#define ZMAP_MIN -5000 /* Minimum allowable Zmap parameter value     */

#define CHECK_ALL 0             /* Check mask - initialiser                   */
#define DISPLAY (1 << 0)        /*            - display message text          */
#define _DEBUG (1 << 1)         /*            - display debug information     */
#define SHOW (DISPLAY | _DEBUG) /*            - display message & debug info. */
#define NO_INT_CHECK (1 << 2)   /*            - no interlock checking         */
#define HARD (1 << 3)           /*            - hardware reset required       */
#define NO_SAVE (1 << 4)        /*            - do not update defaults file   */
#define SET_COEFFS (1 << 5)     /*            - used during SURVEY routine    */
#define BLIND (1 << 6)          /*            - pickup button blind           */
#define TO_PARK (1 << 7)        /*            - putting button in park pos    */
#define FROM_PARK (1 << 8)      /*            - getting button from park pos  */
#define JAW_FULL (1 << 9)       /*            - open jaw to OPEN_JAW_FULL     */
#define STATS (1 << 10)         /*            - update stats structure        */
#define TIME1 (1 << 11)         /*	     - Timing information mode 1     */
#define TIME2 (1 << 12)         /*	     - Timing information mode 2     */

#define DONT_ITERATE (1 << 13) /*            - Don't iterate during putdown */
#define NO_ZMAP (1 << 14)      /*            - Don't use zmap (MOVE_AXIS)   */

#define ILOCK__TASK_INIT (1 << 0) /* Used to specify interlock checks           */
#define ILOCK__GAN_PARKED (1 << 1)
#define ILOCK__GAN_NOT_PARKED (1 << 2)

#define FREE_LOCKS 1  /* Used by tdFgripStructFree to release locks */
#define LEAVE_LOCKS 2 /* ..                           ignore     .. */

#define FILENAME_LENGTH 30       /* Max "Command File" name length             */
#define PATH_LENGTH 100          /* Max length of path name                    */
#define PMAC_RESPONSE_LENGTH 512 /* Max PMAC response length                   */

#define IMAGE 1    /* Used when returning image                  */
#define CENTROID 2 /* Only centroid                              */

#define _ALL 1    /* Identify the area for the SURVEY action    */
#define _COEFFS 2 /*  These are passed to conversion ops        */
#define _TEMP 3
#define _FLEX 4
#define _FULL 5
#define _FULL_NO_ZMAP 6 /* As per FULL, but don't adjust Z for Z map */

#define GRASPED 0 /* Button focus distances/windows             */
#define FREE 1
#define _CENTRE 0
#define _OFFSET 1

#define _INIT 1  /* Used to identify an INITIALISE action      */
#define _RESET 2 /* ..                  RESET          ..      */

#define GR_X1 1 /* Used to determine the PMAC status word ... */
#define GR_X2 2 /* ... we are interested in                   */
#define GR_Y 3
#define GR_Z 4
#define GR_JAW 5
#define GR_THETA 6
#define CS_GRIP 7
#define PMAC_0 8

#define TDFGRIP_MSG_BUFFER 200000 /* Size of message buffer for TDFGRIP         */

#ifndef TDFPT_PARAM_DIR
#define TDFPT_PARAM_DIR "/home/lliu/Project_Codes/buildanagate/Parameters/"
#endif

/*
 *  Note: all dimensions in microns unless stated.
 */
#define BUTTON_FVP 9100 /* Fibre virtual pivot                   */
#define BUTTON_PHW 600  /* Prism half width                      */
#define BUTTON_PLE -600 /* Prism leading edge                    */
#define BUTTON_BHW 1000 /* Button half width                     */
#define BUTTON_BLE 1100 /* Button leading edge                   */
#define BUTTON_BTE 5100 /* Button trailing edge                  */
#define BUTTON_THW 300  /* Tail half width                       */

#define FIBRE_DIA 225       /* Diameter of fibre optics              */
#define EXTENSION 283000    /* Maximum allowed extension             */
#define EXT_SQRD 8.0089e10  /* Maximum extension, squared            */
#define FIBRE_CLEAR 400     /* Minimum allowed button/fibre clear.   */
#define BUTTON_CLEAR 400    /* Minimum allowed button/button clear.  */
#define MAX_CLEARANCE 5000  /* Maximum allowed but or fib clearances */
#define MAX_PIV_ANGLE 0.250 /* +/- max allowed piv/fib angle (rads)  */
#define MAX_BUT_ANGLE 0.189 /* +/- max allowed but/fib angle (rads)  */

#define HANDLE_WIDTH 500 /* Width of the button handle (microns)  */

#define PIVOT_PROBE_DIST 10000 /* The distance between the pivot pos &..*/
                               /* the prism when the button is parked   */

double distP17 = 100.0;    /* Maximum absolute conversion expected */
double distP18 = 250000.0; /* Plate scale used in Will's model */
double distP1 = -72.02;    /* dX = X^2 */
double distP2 = 71.71;     /* dX = Y^2 */
double distP3 = -12.85;    /* dX = X^3 */
double distP4 = 15.98;     /* dX = Y^3 */
double distP5 = 45.11;     /* dX = X^4 */
double distP6 = -26.50;    /* dX = Y^4 */
double distP7 = 0.34;      /* dX = 1  */
double distP8 = 0.00;      /* dX = 0  */
double distP9 = -19.33;    /* dY = X^2 */
double distP10 = -8.72;    /* dY = Y^2 */
double distP11 = -9.81;    /* dY = X^3 */
double distP12 = -4.73;    /* dY = Y^3 */
double distP13 = 27.93;    /* dY = X^4 */
double distP14 = -35.04;   /* dY = Y^4 */
double distP15 = 2.43;     /* dY = sin(angle)    */
double distP16 = 0.00;     /* dY = cosine(angle) */

/*
 * Indicates a standard deviation of a offset is invalid. (MOVE_FIBRE and
 * PUTDOWN actions).
 */
#define STDDEV_INVALID 256000
#define FIELD_RAD 253000          /* Usable field plate radius                */
#define FIELD_RAD_INC_PARK 285000 /* Field radius include allowance for the park positions */

#define NSMASK_INNER_RAD FIELD_RAD /* Nod and shuffle mask inner radius */
#define QUADRANT_RADIUS 277000     /* Radius of a quadrant of the field plate */
#define INSIDE_RADIUS 270430       /* Radius of largest circle which is inside */
/* area which can be accessed by the gripper */
#define OUTSIDE_RADIUS 347300 /* Radius of smallest circule which is 	*/
/* outside the retractor ring		*/
#define HALF_GUIDE_EXPAN 2250 /* The amount added between the four quadrants*/
#define BACKILLAL
/* to account for the guide fibres	*/

/*
 *  If not on VxWorks, must define the following.
 */
#ifndef VxWorks
#define ERROR -1
#endif

/*
 *  Macro definitions.
 */
#define SQRD(x) ((x) * (x))           /* Return the square of a number              */
#define ABS(x) ((x) < 0 ? -(x) : (x)) /* Return the absolute value of a number      */

#define PLATE_FR_MIN 1       /* Plate rotators minimum feedrate value (?)     */
#define PLATE_FR_MAX 3000000 /* Plate rotators maximum feedrate value (?)     */
#define XMIN -108396         /* X-axis minimum value (microns)                */
#define XMAX 110000          /* X-axis maximum value (microns)                */
#define YMIN -110963         /* Y-axis minimum value (microns)                */
#define YMAX 123059          /* Y-axis maximum value (microns)                */
#define ZMIN 0               /* Z-axis minimum value (microns)                */
#define ZMAX 60000           /* Z-axis maximum value (microns)                */

#define QUADRANT_RADIUS 277000
#define INSIDE_RADIUS 270430
#define OUTSIDE_RADIUS 347300
#define HALF_GUIDE_EXPAN 2250

#define JAW_HWP 12220   /* Jaw half with +ve theta */
#define JAW_HWM 8300    /* Jaw half with -ve theta */
#define JAW_LENGTH 5250 /* Jaw length		*/

#define TDFGRIPPER_MSG_BUFFER 20000 /* Size of message buffer for TDFFPI          */

#define GRIPPER_CLEAR_X 253513
#define GRIPPER_CLEAR_Y -240488

#define TAN4P5 0.07870171
#define SIN9 0.156434465
#define COS9 0.987688341
#define SIN4P5 0.0784590957
#define COS4P5 0.9969173337
#define CLEARANCE 500.0
#define PI 3.1415926535
#define D2PI (2 * PI)
#define MILLION 1000000.0
#define D2PI_MRADS (D2PI * MILLION)
#define DPI_MRADS (PI * MILLION)
#define TWOPI 6.283185308
#define PION2 1.570796327
#define R4P5 0.07853981634
#define R9 0.1570796327
#define SF_IMAGES 10

#define MAX_POINTS 25
#define GRID_SIZE 10
#define FREE_GRID_SIZE 20

#define GLOBAL_BUFFER_SPACE 21000000

#ifndef SAFE
#define SAFE 2
#endif

#define Z_UP -1      /* Height safe for crossing retractors */
#define Z_DOWN -2    /* Height for putting fibre on plate	*/
#define Z_CARRY -3   /* Height for carrying fibres		*/
#define Z_CAM -4     /* Height for viewing fibres		*/
#define Z_CAM_FID -5 /* Height for fibre fiducials		*/
#define Z_IT -6      /* Iteration Z				*/
#define Z_PARK -7    /* Z park position			*/
#define Z_CLEAN -8   /* Z height for cleaning                */
#define Z_SP_MIN -8  /* Most negative of these values	*/

#define JAWMIN 0           /* JAW-axis minimum value (microns)              */
#define JAWMAX 8000        /* JAW-axis maximum value (microns)              */
#define THETAMIN (-2 * PI) /* THETA-axis minimum value (radians)            */
#define THETAMAX (2 * PI)  /* THETA-axis minimum value (radians)            */
#define PHIMIN (-PI / 360) /* PHI-axis (plate rotation) minimum (radians)   */
#define PHIMAX (PI / 360)  /* PHI-axis (plate rotation) maximum (radians)   */
#define TUMMIN -45         /* Tumbler-axis minimum (degrees)                */
#define TUMMAX 225         /* Tumbler-axis maximum (degrees)                */

#define GRIPPER_PARKED (GRIP_A_X | GRIP_A_Y | GRIP_A_Z) /* Axes required to ... */
#define GRIPPER_X_PARK -260000                          /* Gripper X axis park position (microns)        */
#define GRIPPER_Y_PARK -260000                          /* Gripper Y axis park position (microns)        */
#define GRIPPER_Z_PARK 50000                            /* Gripper Z axis park position (microns)        */
#define GRIPPER_JAW_PARK 0                              /* Gripper JAW axis park position (microns)      */
#define GRIPPER_THETA_PARK 0                            /* Gripper THETA axis park position (microns)    */

#define GRIPPER_XY_FR_MIN 1       /* GRIP XY axes minimum feedrate value (?)       */
#define GRIPPER_XY_FR_MAX 1000000 /* GRIP XY axes maximum feedrate value (?)       */
#define GRIPPER_Z_FR_MIN 1        /* GRIP Z axis minimum feedrate value (?)        */
#define GRIPPER_Z_FR_MAX 200000   /* GRIP Z axis maximum feedrate value (?)        */
#define THETA_FR_MIN 1            /* GRIP theta axis minimum feedrate value (?)    */
#define THETA_FR_MAX 9000000      /* GRIP theta axis maximum feedrate value (?)    */
#define JAW_FR_MIN 1              /* GRIP jaw axis minimum feedrate value (?)      */
#define JAW_FR_MAX 200000         /* GRIP jaw axis maximum feedrate value (?)      */

#define THETA_TOL 0.05

#define NSMASK_INTERLOCK_PLATE_0 1
#define NSMASK_INTERLOCK_PLATE_1 2
#define NSMASK_INTERLOCK_BOTH 3
/*
 *  Structure definitions required for the main tdFgrip task structure.
 */

/*
 *  Z-Map details.  These are in microns so could be as
 *  int, but since they are used in calcuations, keep as double.
 */
typedef struct tdFzmap
{
public:
    double _cen;
    double _xm;
    double _xp;
    double _ym;
    double _yp;

public:
    tdFzmap()
    {
        _cen = _xm = _xp = _ym = _yp = 0;
    }
    ~tdFzmap()
    {
    }
} tdFzmap;

/*
 *  Gripper encoder position.
 */
typedef struct tdFencPos
{
public:
    int theta;
    int x;
    int y;
    int z;
    int jaw;

public:
    tdFencPos()
    {
        theta = x = y = z = jaw = 0;
    }
    ~tdFencPos() {}
} tdFencPos;

/*
 *  Ideal gripper gantry position.
 */
typedef struct tdFganPos
{
public:
    double theta;
    long int x;
    long int y;
    long int z;
    long int jaw;

public:
    tdFganPos()
    {
        theta = x = y = z = jaw = 0;
    }
    ~tdFganPos() {}
} tdFganPos;

/*
 *  This type is used by our routines which work out the centroid position
 *  at the time of the centroid (tdFgripPreExp and tdFgripPostExp).  We
 *  actually keep this in the tdFgripTaskType structure.
 */
typedef struct tdFimagePos
{
public:
    int enable;        /* Enable the operation */
    short useDpr;      /* Do we use the DPR	*/
    short displayText; /* Display text data	*/
    tdFganPos p;       /* Gantry position during centroid */
    tdFencPos enc;     /* Encoder value        */
public:
    tdFimagePos()
    {
        enable = 1;
        useDpr = 1;
        displayText = 1;
    }
    ~tdFimagePos() {}
} tdFimagePos;

/*
 *  Camera parameters.
 */
typedef struct tdFcamImage
{
public:
    GCamShutterType shutter;
    double camCoeffs[6];
    double invCoeffs[6];
    float updateTime;
    float exposureTime;
    int camNo;
    int bias;
    int xMax;
    int yMax;
    int PixelSize;

public:
    tdFcamImage()
    {
        for (int i = 0; i < 6; i++)
        {
            camCoeffs[i] = invCoeffs[i] = 0.0;
        }
        exposureTime = 0.001667;
        updateTime = 0.005;
        bias = 0;
        PixelSize = 1;
        xMax = CAM_X_SPAN;
        yMax = CAM_Y_SPAN;
    }
    ~tdFcamImage() {}
} tdFcamImage;

typedef struct tdFcamWindow
{
public:
    double xCen;
    double yCen;
    int xSpan;
    int ySpan;

public:
    tdFcamWindow()
    {
        xCen = yCen = xSpan = ySpan = 0;
    }
    ~tdFcamWindow() {}
} tdFcamWindow;

typedef struct TdfFlexType
{
public:
    double offset;
    double length;
    double k;

public:
    TdfFlexType()
    {
        offset = length = k = 0.0;
    }
    ~TdfFlexType() {}
} TdfFlexType;
/*
 *  Transformation parameters.  Used to convert between field-plate and encoder units.
 */
typedef struct tdFconvert
{
public:
    double coeffs[6];    /* Convert field->plate coords     */
    double invCoeffs[6]; /* Convert plate->field coords     */
    int thetaShift;      /* For theta axis conversion       */
    int jawShift;        /* For jaw axis conversion         */
    int zShift;          /* For z axis conversion           */
    TdfFlexType flex;    /* System flexure model details    */
    tdFzmap zmap;        /* Z-map details                   */
public:
    tdFconvert()
    {
        for (int i = 0; i < 6; i++)
        {
            coeffs[i] = invCoeffs[i] = 0.0;
        }
        thetaShift = jawShift = zShift = 0;
    }
    ~tdFconvert() {}
} tdFconvert;
/*
 *  Parameter lookup.  We store here the values returned by SdsPointer for
 *  parameters we used often
 */
typedef struct tdFgripParLookup
{
public:
    short *backIllAlways;   /* Is backillumination always on */
    double *backIllWarmUp;  /* backillumination warm up time */
    double *zeroCamCenWait; /* zero cam gantry settle time before centroid*/
    double *distPar1;       /* Gripper/FPI distortion removal parameters */
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
    short *distRemEnable; /* Is distortion removal enabled/disabled
                             see  calculation in tdFgripCvt.c */

    short *plt1CenterFidOffsetX;
    short *plt1CenterFidOffsetY;

public:
    tdFgripParLookup()
    {
        backIllAlways = distRemEnable = plt1CenterFidOffsetX = plt1CenterFidOffsetY = nullptr;
        distPar1 = distPar2 = distPar3 = distPar4 = distPar5 = distPar6 = nullptr;
        distPar7 = distPar8 = distPar9 = distPar10 = distPar11 = distPar12 = nullptr;
        distPar13 = distPar14 = distPar15 = distPar16 = distPar17 = distPar18 = nullptr;
    }
    ~tdFgripParLookup()
    {
        if (backIllAlways)
        {
            delete backIllAlways;
            backIllAlways = nullptr;
        }
        if (distRemEnable)
        {
            delete distRemEnable;
            distRemEnable = nullptr;
        }
        if (plt1CenterFidOffsetX)
        {
            delete plt1CenterFidOffsetX;
            plt1CenterFidOffsetX = nullptr;
        }
        if (plt1CenterFidOffsetY)
        {
            delete plt1CenterFidOffsetY;
            plt1CenterFidOffsetY = nullptr;
        }
        if (distPar1)
        {
            delete distPar1;
            distPar1 = nullptr;
        }
        if (distPar2)
        {
            delete distPar2;
            distPar2 = nullptr;
        }
        if (distPar3)
        {
            delete distPar3;
            distPar3 = nullptr;
        }
        if (distPar4)
        {
            delete distPar4;
            distPar4 = nullptr;
        }
        if (distPar5)
        {
            delete distPar5;
            distPar5 = nullptr;
        }
        if (distPar6)
        {
            delete distPar6;
            distPar6 = nullptr;
        }
        if (distPar7)
        {
            delete distPar7;
            distPar7 = nullptr;
        }
        if (distPar8)
        {
            delete distPar8;
            distPar8 = nullptr;
        }
        if (distPar9)
        {
            delete distPar9;
            distPar9 = nullptr;
        }
        if (distPar10)
        {
            delete distPar10;
            distPar10 = nullptr;
        }
        if (distPar11)
        {
            delete distPar11;
            distPar11 = nullptr;
        }
        if (distPar12)
        {
            delete distPar12;
            distPar12 = nullptr;
        }
        if (distPar13)
        {
            delete distPar13;
            distPar13 = nullptr;
        }
        if (distPar14)
        {
            delete distPar14;
            distPar14 = nullptr;
        }
        if (distPar15)
        {
            delete distPar15;
            distPar15 = nullptr;
        }
        if (distPar16)
        {
            delete distPar16;
            distPar16 = nullptr;
        }
        if (distPar17)
        {
            delete distPar17;
            distPar17 = nullptr;
        }
        if (distPar18)
        {
            delete distPar18;
            distPar18 = nullptr;
        }
    }
} tdFgripParLookup;

/*
 * Distortion map - hold the Cannon distortion map.
 */
#define GRID_WIDTH 23
#define GRID_STEP (318.0 * 66.66667)                                   /* Step between grid elements - from Russels program. */
#define GRID_CENTRE_OFFSET GRID_STEP *((double)(GRID_WIDTH + 1) / 2.0) /* Offset to centre of grid */

typedef struct tdFgripDistMap
{
public:
    int x[GRID_WIDTH * GRID_WIDTH];
    int y[GRID_WIDTH * GRID_WIDTH];
    int loaded;

public:
    tdFgripDistMap()
    {
        loaded = 0;
        for (int i = 0; i < GRID_WIDTH * GRID_WIDTH; i++)
        {
            x[i] = y[i] = 0;
        }
    }
    ~tdFgripDistMap() {}
} tdFgripDistMap;

/*
 *  The tdFgrip main task structure.
 */
typedef struct tdFgripTaskType
{
public:
    GitSimulationType Simulation; /* Simulation level                */
    tdFcamImage freeImg;          /* Image parameters for free focus */
    tdFcamImage graspImg;         /*   ''     ''          grasp  ''  */
    tdFcamWindow searchWin;       /* Def'n of search image window    */
    tdFcamWindow normWin;         /*   ''     norm     ''   ''       */
    tdFconvert convert;           /* FP<-->enc conversion parameters */
    tdFencPos toEnc;              /* Target encoder position         */
    tdFencPos atEnc;              /* Current encoder position        */
    tdFganPos ideal;              /* Ideal gantry position           */
    tdFimagePos imagePos;         /* Use to get position during image */
    float TimeBase;               /* Simulation time base            */
    int dprAddress;               /* Address of dual ported ram      */
    short Initialised;            /* Task initialised flag           */
    short cameraInit;             /* Camera initialised flag         */
    short dprFeedback;            /* Get axis/status info from dpr?  */
    short inUse;                  /* Motion semaphore                */
    short backIllumOn;            /* Is back illumination on	 */
    short illumPlate;             /* If on, which plate		 */
    short configPlate;            /* Config plate recorded here	 */
    short plateOneDontRemove;     /* Set true to turn off the
                                     removal of the plate 1 offset */
    short ipsMode;                /* Is IPS camera mode enabled       */
    tdFgripParLookup pars;        /* SdsPointers to parameters for quick */
                                  /* access				 */
    tdFgripDistMap distMap;       /* Cannon distortion map           */
public:
    tdFgripTaskType()
    {
        Initialised = NO;
        cameraInit = NO;
        dprFeedback = YES;
        inUse = YES;
        backIllumOn = NO;
        illumPlate = -1;  /* negative means not set */
        configPlate = -1; /* negative means not set */
        plateOneDontRemove = 0;
        distMap.loaded = NO;
    }
    ~tdFgripTaskType() {}
} tdFgripTaskType;

/*
 *  Structure definitions for action parameters (handled by DitsPutActData() and
 *  DitsGetActData()).
 */

/*
 *  This is the structure that is stored for all parameters.
 */
typedef struct tdFgripDataType
{
public:
    void *userData;
    short usingLock;
    short check;
    short type;
    struct tdFgripDataType *more;

public:
    tdFgripDataType()
    {
        check = CHECK_ALL;
        usingLock = NO;
        userData = nullptr;
        type = 0;
        more = nullptr;
    }
    ~tdFgripDataType()
    {
        if (userData)
        {
            delete userData;
            userData = nullptr;
        }
        if (more)
        {
            delete more;
            more = nullptr;
        }
    }
} tdFgripDataType;

/*
 *  The following structures are stored in the "userData" section of the
 *  tdFgripDataType structure.
 */
#define IN_TYPE 1 /* Used by tdFgripSysInit */
typedef struct tdFgrip_INtype
{
public:
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;
    short process;
    short axisWord;
    short reset;

public:
    tdFgrip_INtype()
    {
        handler = 0;
        kickHandler = 0;
        axisWord = 0;
        process = _INIT;
        reset = YES;
    }
    ~tdFgrip_INtype() {}
} tdFgrip_INtype;

#define SE_TYPE 2 /* Used by tdFgripSysExit */
typedef struct tdFgrip_SEtype
{
public:
    short reset;

public:
    tdFgrip_SEtype()
    {
        reset = YES;
    }
    ~tdFgrip_SEtype() {}
} tdFgrip_SEtype;

#define MB_TYPE 3 /* Used by tdFgripMoveButton */
typedef struct tdFgrip_MBtype
{
public:
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;
    double atTheta;
    double toTheta;
    double theta;   /* Resultant button theta position */
    double stdDevX; /* Iteration offset FIFO  standard deviation */
    double stdDevY;
    long int atX;
    long int atY;
    long int toX;
    long int toY;
    long int xErr;
    long int yErr;
    long int xBut;
    long int yBut;
    long int xOff; /* Positioning offset to presume */
    long int yOff;
    long int xOffIt; /* Positioning offset for iterations */
    long int yOffIt;
    long int graspX; /* Grasp offsets */
    long int graspY;
    long int actXoff;
    long int actYoff;
    long int actGraspX; /* Actual grasp offset in x   */
    long int actGraspY; /* Actual grasp offset in y   */
    long int xf;        /* Resultant x fibre position */
    long int yf;        /* Resultant y fibre position */
    long int buttonZ;   /* Calculated by PutDown      */
    short inTolerance;
    short iterations;
    short reset;
    unsigned short fibreNum;
    unsigned short fifoNum;  /* Number of values in iteration offset
                                fifo */
    unsigned short guideBut; /* Is a guide button */
public:
    tdFgrip_MBtype()
    {
        handler = 0;
        kickHandler = 0;
        atTheta = toTheta = 0;
        atX = atY = toX = toY = 0;
        xOff = yOff = 0;
        xErr = yErr = 0;
        xBut = yBut = 0;
        graspX = graspY = 0;
        actXoff = actYoff = 0;
        actGraspX = actGraspY = 0;
        iterations = 0;
        inTolerance = NO;
        reset = YES;
        buttonZ = 0;
        fibreNum = 0;
        guideBut = NO;
    }
    ~tdFgrip_MBtype() {}
} tdFgrip_MBtype;

#define MBC_TYPE 4 /* Stored by tdFgripMoveButton after completion */
typedef struct tdFgrip_MBCtype
{
public:
    double theta; /* Resultant button theta position */
    long int xErr;
    long int yErr;
    long int xBut;
    long int yBut;
    long int actXoff;
    long int actYoff;
    long int xf; /* Resultant x fibre position */
    long int yf; /* Resultant y fibre position */
    long int actGraspX;
    long int actGraspY;
    short inTolerance;
    short iterations;
    unsigned short fibreNum;

public:
    tdFgrip_MBCtype()
    {
        xErr = yErr = 0;
        xBut = yBut = 0;
        actXoff = actYoff = 0;
        actGraspX = actGraspY = 0;
        iterations = 0;
        inTolerance = NO;
        fibreNum = 0;
    }
    ~tdFgrip_MBCtype() {}
} tdFgrip_MBCtype;

#define PUB_TYPE 5 /* Used by tdFgripPickUpButton */
typedef struct tdFgrip_PUBtype
{
public:
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;
    double atTheta;
    long int atX;
    long int atY;
    long int graspX;
    long int graspY;
    long int xErr;
    long int yErr;
    short centroidOK;
    short reset;
    unsigned short fibreNum;
    unsigned short guideBut; /* Is a guide button */
public:
    tdFgrip_PUBtype()
    {
        handler = 0;
        kickHandler = 0;
        atTheta = 0;
        atX = atY = 0;
        graspX = graspY = 0;
        xErr = yErr = 0;
        centroidOK = reset = YES;
        fibreNum = 0;
        guideBut = NO;
    }
    ~tdFgrip_PUBtype() {}
} tdFgrip_PUBtype;

#define PDB_TYPE 6 /* Used by tdFgripPutDownButton */
typedef struct tdFgrip_PDBtype
{
public:
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;
    double toTheta;
    double stdDevX; /* Iteration offset FIFO  standard deviation */
    double stdDevY;
    long int toX;
    long int toY;
    long int graspX;
    long int graspY;
    long int xOff; /* Positioning offset to presume */
    long int yOff;
    long int xOffIt; /* Positioning offset for iterations */
    long int yOffIt;
    long int xErr;
    long int yErr;
    short centroidOK;
    short reset;
    unsigned short fibreNum;
    unsigned short fifoNum;  /* Number of values in iteration offset
                                fifo */
    unsigned short guideBut; /* Is a guide button */
public:
    tdFgrip_PDBtype()
    {
        handler = 0;
        kickHandler = 0;
        toTheta = 0;
        toX = toY = 0;
        graspX = graspY = 0;
        xOff = yOff = 0;
        xErr = yErr = 0;
        centroidOK = reset = YES;
        fibreNum = 0;
        guideBut = NO;
    }
    ~tdFgrip_PDBtype() {}
} tdFgrip_PDBtype;

#define PDC_TYPE 7 /* Returned by tdFgripPutDownButton */
typedef struct tdFgrip_PDCtype
{
public:
    double theta;  /* Resultant button theta position */
    long int xErr; /* Putdown error                   */
    long int yErr;
    long int xBut; /* Actual button position          */
    long int yBut;
    long int actXoff;   /* Actual positioning offset in x  */
    long int actYoff;   /* Actual positioning offset in y  */
    long int xf;        /* Resultant x fibre position */
    long int yf;        /* Resultant y fibre position */
    long int actGraspX; /* Actual grasp offset in x   */
    long int actGraspY; /* Actual grasp offset in y   */
    long int buttonZ;   /* Calculated button Z */
    short inTolerance;
    short iterations;
    unsigned short fibreNum;

public:
    tdFgrip_PDCtype()
    {
        xErr = yErr = 0;
        xBut = yBut = 0;
        actXoff = actYoff = 0;
        actGraspX = actGraspY = 0;
        iterations = 0;
        inTolerance = NO;
        buttonZ = 0;
        fibreNum = 0;
    }
    ~tdFgrip_PDCtype()
    {
    }
} tdFgrip_PDCtype;

#define XYZT_TYPE 8 /* Used by tdFgripMoveXYZT */
typedef struct tdFgrip_XYZTtype
{
public:
    const char *operation;
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;
    double toTheta;
    long int toX;
    long int toY;
    long int toZ;

public:
    tdFgrip_XYZTtype()
    {
        handler = 0;
        kickHandler = 0;
        toTheta = 0;
        toX = toY = toZ = 0;
        operation = 0;
    }
    ~tdFgrip_XYZTtype() {}
} tdFgrip_XYZTtype;

#define MA_TYPE 9 /* Used by tdFgripMoveAxis */
typedef struct tdFgrip_MAtype
{
public:
    const char *operation;
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;
    double tarAxis;
    short axis;

public:
    tdFgrip_MAtype()
    {
        handler = 0;
        kickHandler = 0;
        tarAxis = 0;
        axis = JAW;
        operation = nullptr;
    }
    ~tdFgrip_MAtype()
    {
        if (operation)
        {
            delete operation;
            operation = nullptr;
        }
    }
} tdFgrip_MAtype;

#define JO_TYPE 10 /* Used by tdFgripJawOpen */
typedef struct tdFgrip_JOtype
{
public:
    const char *operation;
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;
    long int jawOpenDist;
    long int handleWidth;
    long int toZ; /* Used for method = JAW_SEP only */
    short method;
    short reset;

public:
    tdFgrip_JOtype()
    {
        handler = 0;
        kickHandler = 0;
        jawOpenDist = handleWidth = 0;
        method = JAW_SEP;
        reset = YES;
        operation = nullptr;
    }
    ~tdFgrip_JOtype()
    {
        if (operation)
        {
            delete operation;
            operation = nullptr;
        }
    }
} tdFgrip_JOtype;

#define JS_TYPE 11 /* Used by tdFgripJawShut */
typedef struct tdFgrip_JStype
{
public:
    const char *operation;
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;
    long int atX;
    long int atY;
    short method;
    short reset;
    unsigned short guideBut; /* Is a guide button */
public:
    tdFgrip_JStype()
    {
        handler = 0;
        kickHandler = 0;
        atX = atY = 0;
        method = JAW_SEP;
        reset = YES;
        operation = nullptr;
        guideBut = NO;
    }
    ~tdFgrip_JStype()
    {
        if (operation)
        {
            delete operation;
            operation = nullptr;
        }
    }
} tdFgrip_JStype;

#define HA_TYPE 12 /* Used by tdFgripHomeAxis */
typedef struct tdFgrip_HAtype
{
public:
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;
    int axis;

public:
    tdFgrip_HAtype()
    {
        handler = 0;
        kickHandler = 0;
        axis = JAW;
    }
    ~tdFgrip_HAtype() {}
} tdFgrip_HAtype;

#define PG_TYPE 13 /* Used by tdFgripParkGan */
typedef struct tdFgrip_PGtype
{
public:
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;
    short axisWord;
    short ganParked;

public:
    tdFgrip_PGtype()
    {
        handler = 0;
        kickHandler = 0;
        axisWord = 0;
        ganParked = NO;
    }
    ~tdFgrip_PGtype() {}
} tdFgrip_PGtype;

#define UPG_TYPE 14 /* Used by tdFgripUnParkGan */
typedef struct tdFgrip_UPGtype
{
public:
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;

public:
    tdFgrip_UPGtype()
    {
        handler = 0;
        kickHandler = 0;
    }
    ~tdFgrip_UPGtype() {}
} tdFgrip_UPGtype;

#define SF_TYPE 15 /* Used by tdFgripSearch */
#define SF_IMAGES 10
typedef struct tdFgrip_SFtype
{
public:
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;
    long int searchStartX;
    long int searchStartY;
    long int searchStartZ;
    long int stepSize;
    long int maxError;
    long int tolerance;
    long int xErr;
    long int yErr;
    short centroidOK;
    short attempts;
    short reset;
    short resultsValid;
    short statCheck;
    short peakVal;
    short saturated;
    long int resultsX[SF_IMAGES];
    long int resultsY[SF_IMAGES];

public:
    tdFgrip_SFtype()
    {
        for (int i = 0; i < SF_IMAGES; i++)
        {
            resultsX[i] = resultsY[i] = 0;
        }
        handler = 0;
        kickHandler = 0;
        searchStartX = searchStartY = 0;
        searchStartZ = 0;
        stepSize = maxError = tolerance = 0;
        xErr = yErr = 0;
        centroidOK = YES;
        attempts = 0;
        statCheck = 0;
        reset = YES;
    }
    ~tdFgrip_SFtype() {}
} tdFgrip_SFtype;

#define SC_TYPE 16 /* Stored by tdFgripSearch after completion */
typedef struct tdFgrip_SCtype
{
public:
    long int xf;
    long int yf;
    long int dx;
    long int dy;
    long int xErr;
    long int yErr;
    int xEnc;
    int yEnc;
    short found;

public:
    tdFgrip_SCtype()
    {
        xf = yf = 0;
        dx = dy = 0;
        xErr = yErr = 0;
        xEnc = yEnc = 0;
        found = YES;
    }
    ~tdFgrip_SCtype() {}
} tdFgrip_SCtype;

#define S_TYPE 17 /* Used by tdFgripSurvey */
typedef struct tdFgrip_Stype
{
public:
    double temp;
    long int x[NUM_FIDUCIALS];
    long int y[NUM_FIDUCIALS];
    long int z[NUM_FIDUCIALS];
    long int fidNum[NUM_FIDUCIALS]; /* fiducial number */
    long int dx;
    long int dy;
    int xEnc;
    int yEnc;
    short found;
    short numMarks;
    short area;
    short reset;

public:
    tdFgrip_Stype()
    {
        temp = 0;
        for (i = 0; i < NUM_FIDUCIALS; i++)
            x[i] = y[i] = z[i] = fidNum[i] = 0;
        xEnc = yEnc = 0;
        dx = dy = 0;
        found = YES;
        numMarks = 0;
        area = _ALL;
        reset = YES;
    }
    ~tdFgrip_Stype() {}
} tdFgrip_Stype;

#define BR_TYPE 18 /* Used by tdFgripBrake */
typedef struct tdFgrip_BRtype
{
public:
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;
    short brakeWord;
    short brakesOn;

public:
    tdFgrip_BRtype()
    {
        handler = 0;
        kickHandler = 0;
        brakeWord = 0;
        brakesOn = YES;
    }
    ~tdFgrip_BRtype() {}
} tdFgrip_BRtype;

#define SV_TYPE 19 /* Used by tdFgripServo */
typedef struct tdFgrip_SVtype
{
public:
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;
    short servoWord;
    short servo;

public:
    tdFgrip_SVtype()
    {
        handler = 0;
        kickHandler = 0;
        servoWord = 0;
        servo = YES;
    }
    ~tdFgrip_SVtype() {}
} tdFgrip_SVtype;

#define ZC_TYPE 20 /* Used by tdFgripZerocam */
typedef struct tdFgrip_ZCtype
{
public:
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;
    double xFull;
    double yFull;
    short reset;
    short centroidOK;

public:
    tdFgrip_ZCtype()
    {
        handler = 0;
        kickHandler = 0;
        xFull = yFull = 0;
        reset = centroidOK = YES;
    }
    ~tdFgrip_ZCtype() {}
} tdFgrip_ZCtype;

#define SH_TYPE 21 /* Used by tdFgripShiftCoeffs */
typedef struct tdFgrip_SHtype
{
public:
    long int expX;
    long int expY;
    long int focusZ;
    long int measuredX;
    long int measuredY;
    short reset;
    short found;

public:
    tdFgrip_SHtype()
    {
        expX = expY = 0;
        measuredX = measuredY = 0;
        reset = found = YES;
    }
    ~tdFgrip_SHtype() {}
} tdFgrip_SHtype;

#define CEN_TYPE 22 /* Used by tdFgripCentroid */
typedef struct tdFgrip_CENtype
{
public:
    const char *operation;
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;
    GCamWindowType window;
    tdFcamImage *img;
    double settleTime;
    short wantImage;
    short tryagain;
    IMP_AbsTime startTime;
    char saveName[40];

public:
    tdFgrip_CENtype()
    {
        handler = 0;
        kickHandler = 0;
        img = NULL;
        settleTime = 0;
        wantImage = NO;
        saveName[0] = '\0';
        operation = nullptr;
    }
    ~tdFgrip_CENtype()
    {
        if (operation)
        {
            delete operation;
            operation = nullptr;
        }
    }
} tdFgrip_CENtype;

#define GCC_TYPE 23 /* Stored by tdFgripCentroid after completion */
typedef struct tdFgrip_GCCtype
{
public:
    long int xErr;
    long int yErr;
    double xFull;
    double yFull;
    short centroidOK;
    short peakVal;
    short saturated;

public:
    tdFgrip_GCCtype()
    {
        xErr = yErr = 0;
        xFull = yFull = 0;
        centroidOK = YES;
        saturated = NO;
        peakVal = 0;
    }
    ~tdFgrip_GCCtype() {}
} tdFgrip_GCCtype;

#define PSC_TYPE 24 /* Stored after completion of tdFgripPMACrunCmd */
typedef struct tdFgrip_PSCtype
{
public:
    char response[PMAC_RESPONSE_LENGTH];
    StatusType cmdSendStatus;
    int progNum;
    double elapsed;

public:
    tdFgrip_PSCtype()
    {
        response[0] = '\0';
        cmdSendStatus = STATUS__OK;
        progNum = -1;
        elapsed = -1;
    }
    ~tdFgrip_PSCtype() {}
} tdFgrip_PSCtype;

#define A_TYPE 25 /* Used by tdFgripAbort */
typedef struct tdFgrip_Atype
{
public:
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;
    StatusType errorStatus;

public:
    tdFgrip_Atype()
    {
        handler = 0;
        kickHandler = 0;
        errorStatus = STATUS__OK;
    }
    ~tdFgrip_Atype() {}
} tdFgrip_Atype;

#define AM_TYPE 26 /* Used by tdFgripAmps */
typedef struct tdFgrip_AMtype
{
public:
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;
    short motorWord;

public:
    tdFgrip_AMtype()
    {
        handler = 0;
        kickHandler = 0;
        motorWord = 0;
    }
    ~tdFgrip_AMtype() {}
} tdFgrip_AMtype;

#define LM_TYPE 27 /* Used by tdFgripLimits */
typedef struct tdFgrip_LMtype
{
public:
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;
    short motorWord;
    short enable;

public:
    tdFgrip_LMtype()
    {
        handler = 0;
        kickHandler = 0;
        motorWord = 0;
        enable = YES;
    }
    ~tdFgrip_LMtype() {}
} tdFgrip_LMtype;

#define RL_TYPE 28 /* Used by tdFgripRelays */
typedef struct tdFgrip_RLtype
{
public:
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;
    short mask;
    short relay;
    short set;

public:
    tdFgrip_RLtype()
    {
        handler = 0;
        kickHandler = 0;
        mask = relay = 0;
        set = NO;
    }
    ~tdFgrip_RLtype() {}
} tdFgrip_RLtype;

#define RD_TYPE 29 /* Returned by tdFgripRelays */
typedef struct tdFgrip_RDtype
{
public:
    short relayState;

public:
    tdFgrip_RDtype() { relayState = 0; }
    ~tdFgrip_RDtype() {}
} tdFgrip_RDtype;

#define PH_TYPE 30 /* Used by tdFgripPhase */
typedef struct tdFgrip_PHtype
{
public:
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;
    short motorWord;

public:
    tdFgrip_PHtype()
    {
        handler = 0;
        kickHandler = 0;
        motorWord = 0;
    }
    ~tdFgrip_PHtype() {}
} tdFgrip_PHtype;

#define UD_TYPE 31 /* Used by tdFgripUpdate */
typedef struct tdFgrip_UDtype
{
public:
    double updateInterval;

public:
    tdFgrip_UDtype() { updateInterval = 5.0; }
    ~tdFgrip_UDtype() {}
} tdFgrip_UDtype;

#define MEAS_Z_TYPE 32 /* Used by tdFgripMeasureZHeight */
typedef struct tdFgrip_MEAS_Ztype
{
public:
    /* DitsActionRoutineType  handler; */
    /* DitsActionRoutineType  kickHandler; */
    double theta;
    long int x;
    long int y;
    long int height;
    short reset;

public:
    tdFgrip_MEAS_Ztype()
    {
        reset = YES;
        x = y = 0;
        theta = 0;
        height = 0;
    }
    ~tdFgrip_MEAS_Ztype() {}
} tdFgrip_MEAS_Ztype;

#define CHKCEN_TYPE 33 /* Used by tdFgripCheckCen */
typedef struct tdFgrip_CHKCENtype
{
public:
    DitsActionRoutineType handler;
    DitsActionRoutineType kickHandler;
    double atTheta;
    long int atX;
    long int atY;
    long int graspX;
    long int graspY;
    long int xSearchErr;
    long int ySearchErr;
    long int xPickupErr;
    long int yPickupErr;
    long int xErr;
    long int yErr;
    short centroidOK;
    short reset;
    unsigned short fibreNum;

public:
    tdFgrip_CHKCENtype()
    {
        handler = 0;
        kickHandler = 0;
        atTheta = 0;
        atX = atY = 0;
        graspX = graspY = 0;
        xErr = yErr = 0;
        centroidOK = reset = YES;
        fibreNum = 0;
    }
    ~tdFgrip_CHKCENtype() {}
} tdFgrip_CHKCENtype;

#endif