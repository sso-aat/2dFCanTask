/**
 * author: lliu@anu.edu.au
 * date: 01/05/2024
 * description: this file defines the structures of 2dFfpi task, some structures are not used
 * in current testbed, they are reserved in order to be consistant with the original 2dFfpi task.
 * 
 **/


#ifndef TDFCANTASK_H
#define TDFCANTASK_H

#include "sds.h"
#include "Git.h"
#include "gcam.h"
#include "slalib.h"
#include <math.h>
#include "fitsio.h"
#include "dul.h"

//to check if a directory exists
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
/*
 *  Constant definitions.
 */
#define NO                          0
#define YES                         1

#define NUM_FIDUCIALS              21  /* Number of fiducial marks/plate             */  

#define _XI                         0  /* Used as array indexes                      */
#define _YI                         1

#define X1                     (1<<0)
#define X2                     (1<<1)
#define FPIX                   (X1 | X2)
#define FPIY                   (1<<2)

#define NO_FILES                    0  /* Used to identify parameter files           */
#define DEFS_FILE              (1<<0)
#define FLEX_FILE              (1<<1)

#define MOVE                        1  /* Used by DitsGetCode to differentiate ...   */
#define OSET                        2  /* ... between abs and relative coordinates   */
#define NO_POLL                     0
#define POLL                        1

#define FPI_CAM_NUM      "/grabber/1"  /* Focal Plane Imager camera device name      */
#define CAM_X_SPAN                768  /* Pixels along the fpi camera x axis         */
#define CAM_Y_SPAN                576  /* Pixels along the fpi camera y axis         */
#define PIXEL_SIZE                  1  /* Pixel size of Pulnix CCD fpi camera        */
#define FIBRE_IN_IMAGE_LIMIT     1000  /* If the sum of the pixel values in an image */
                                       /* is less than this we consider there to be  */
                                       /* no fibre in the image.                     */

#define CHECK_ALL                   0  /* Check mask - initialiser                   */
#define DISPLAY                (1<<0)  /*            - display message text          */
#define _DEBUG                 (1<<1)  /*            - display debug information     */
#define SHOW       (DISPLAY | _DEBUG)  /*            - display message & debug info. */
#define NO_INT_CHECK           (1<<2)  /*            - no interlock checking         */
#define HARD                   (1<<3)  /*            - hardware reset required       */
#define NO_SAVE                (1<<4)  /*            - do not update defaults file   */
#define SET_COEFFS             (1<<5)  /*            - used during SURVEY action     */

#define ILOCK__TASK_INIT       (1<<0)  /* Used to specify interlock checks           */
#define ILOCK__GAN_PARKED      (1<<1)
#define ILOCK__GAN_NOT_PARKED  (1<<2)

#define FREE_LOCKS                  1  /* Used by tdFfpiStructFree to release locks  */
#define LEAVE_LOCKS                 2  /* ..                          ignore     ..  */

#define FILENAME_LENGTH            30  /* Max "Command File" name length             */
#define PATH_LENGTH               100  /* Max length of path name                    */

#define IMAGE                       1  /* Used when returning image                  */
#define CENTROID                    2  /* Only centroid                              */

#define _ALL                        1  /* Identify the area to for SURVEY action     */
#define _COEFFS                     2
#define _TEMP                       3
#define _FLEX                       4
#define _FULL                       5

#define _INIT                       1  /* Used to identify an INITIALISE action      */
#define _RESET                      2  /*                     RESET                  */

#define FPI_X1                      1  /* Used to identify the status words to return*/
#define FPI_X2                      2
#define FPI_Y                       3
#define CS_FPI                      4
 
#ifndef TDFPT_PARAM_DIR
#define TDFPT_PARAM_DIR  "/home/lliu/Project_Codes/buildanagate/Parameters/"
#endif 

#define FPI_XY_FR_MIN         1  /* FPI XY axes minimum feedrate value (?)        */
#define FPI_XY_FR_MAX   1000000  /* FPI XY axes maximum feedrate value (?)        */
#define PLATE_FR_MIN          1  /* Plate rotators minimum feedrate value (?)     */
#define PLATE_FR_MAX    3000000  /* Plate rotators maximum feedrate value (?)     */
#define XMIN              -1000  /* X-axis minimum value (microns)                */
#define XMAX             145000  /* X-axis maximum value (microns)                */
#define YMIN              -1000  /* Y-axis minimum value (microns)                */
#define YMAX             145000  /* Y-axis maximum value (microns)                */
#define ZMIN                  0  /* Z-axis minimum value (microns)                */
#define ZMAX              60000  /* Z-axis maximum value (microns)                */

#define QUADRANT_RADIUS	277000
#define INSIDE_RADIUS	270430
#define OUTSIDE_RADIUS  347300
#define HALF_GUIDE_EXPAN 2250

#define JAW_HWP		12220	/* Jaw half with +ve theta */
#define JAW_HWM		8300	/* Jaw half with -ve theta */	
#define JAW_LENGTH 	5250	/* Jaw length		*/

#define TDFFPI_MSG_BUFFER       20000  /* Size of message buffer for TDFFPI          */

#define FPI_CLEAR_X 253513
#define FPI_CLEAR_Y -240488

#define TAN4P5 0.07870171
#define SIN9   0.156434465
#define COS9   0.987688341
#define SIN4P5 0.0784590957
#define COS4P5  0.9969173337
#define CLEARANCE 500.0
#define PI     3.1415926535
#define TWOPI  6.283185308
#define PION2  1.570796327
#define R4P5   0.07853981634
#define R9     0.1570796327
#define  SF_IMAGES    10


#ifndef SAFE
#define SAFE 2
#endif 
/*
 *  If not on VxWorks, must define the following.
 */
#ifndef VxWorks
#   define   ERROR -1
#endif

/*
 *  Macro definitions.
 */
#define SQRD(x)   ((x)*(x))            /* Return the square of a number         */
#define ABS(x)    ((x)<0? -(x): (x))   /* Return the absolute value of a number */


/*
 *  Focal Plane Imager encoder position.
 */
typedef struct tdFencPos {
    int  x;
    int  y;
    }  tdFencPos;

/*
 *  Focal Plane Imager gantry position.
 */
typedef struct tdFganPos {
    long int  x;
    long int  y;
    }  tdFganPos;

/*
 *  This type is used by our routines which work out the centroid position
 *  at the time of the centroid (tdFfpiPreExp and tdFfpiPostExp).  We
 *  actually keep this in the tdFfpiTaskType structure.
 */
typedef struct tdFimagePos {
    int       enable;           /* Enable the operation */
    short     useDpr;           /* Do we use the DPR    */
    short     displayText;      /* Display text data    */
    tdFganPos p;                /* Gantry position during centroid */
    tdFencPos enc;              /* Gantry encoder position during centroid */
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


typedef struct TdfFlexType
   {
      double offset;
      double length;
      double k;
   } TdfFlexType;

/*
 *  Transformation parameters.  Used to convert between field-plate and encoder units.
 */
typedef struct tdFconvert {
    double              coeffs[6];     /* Convert field->plate coords     */
    double              invCoeffs[6];  /* Convert plate->field coords     */
    TdfFlexType         flex;          /* System flexure model details    */
    }  tdFconvert;


/*
 *  Parameter lookup.  We store here the values returned by SdsPointer for
 *  parameters we used often
 */
typedef struct {
    double *zeroCamCenWait;   /* zero cam gantry settle time before centroid*/
    short  *plt1CenterFidOffsetX;
    short  *plt1CenterFidOffsetY;
    } tdFfpiParLookup;

/*
 *  The tdFfpi main task structure.
 */
typedef struct {    
    GitSimulationType   Simulation;    /* Simulation level                 */
    tdFcamImage         freeImg;       /* Image parameters for free focus  */
    tdFcamWindow        normWin;       /* Def'n of search image window     */
    tdFcamWindow        searchWin;     /*  ''      norm     ''    ''       */
    tdFconvert          convert;       /* FP<-->enc conversion parameters  */
    tdFencPos           toEnc;         /* Target encoder position          */
    tdFencPos           atEnc;         /* Current encoder position         */
    tdFganPos           ideal;         /* Ideal gantry position            */
    tdFimagePos         imagePos;      /* Use to get position during image */
    float               TimeBase;      /* Simulation time base             */
    int                 dprAddress;    /* Dual-ported Ram start address    */
    short               Initialised;   /* Task initialised flag            */
    short               cameraInit;    /* Camera initialised flag          */
    short               dprFeedback;   /* Get axis/status info from dpr?   */
    short               inUse;         /* Motion semaphore                 */
    short               currentPlate;  /* Current FPI plate                */
    short               plateOneDontRemove; /* Set true to turn off the 
                                              removal of the plate 1 offset */
    short               ipsMode;       /* Is IPS camera mode enabled       */
    tdFfpiParLookup     pars;          /* SdsPointers to parameters for    */
                                       /*quick access                      */
    
    }  tdFfpiTaskType;



/*
 *  This is the structure that is stored for all parameters.
 */
typedef struct  tdFfpiDataType {
      void                   *userData;
      short                  usingLock;
      short                  check;
      struct tdFfpiDataType  *more;
}  tdFfpiDataType;

#endif
