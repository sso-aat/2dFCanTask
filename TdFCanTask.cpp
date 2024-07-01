//
//                        T d F  C a n  T a s k . c p p
//
//  Function:
//     Controls the CANBus-based version of the 2dF positioner.
//
//  Author(s): Keith Shortridge, Keith@KnaveAndVarlet.com.au.
//
//  History:
//      6th Mar 2018. Original version. KS.
//     14th Mar 2018. Now has a version of INITIALISE that runs, after some problems connected
//                    with DRAMA threads and signals and interraction with CML. KS.
//     22nd Mar 2018. Tidied up the task a bit, including renaming a number of the actions.
//                    Gantry actions are now called G_MOVE_AXIS etc, and there are some
//                    experimental non-threaded versions such as G_MOVE_AXIS_NT. INITIALISE
//                    now initialises the task instead of the 2dF gantry. Experimented
//                    (unsuccessfully) with cancellation of a move. KS.
//     23rd Aug 2021  ThisAction argument to MoveAxes() is unused, comment it out.
//
//  Copyright (c) Australian Astronomical Observatory (AAO), (2018).
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without
//  restriction, including without limitation the rights to use, copy, modify, merge, publish,
//  distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or
//  substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
//  DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//  RCS id:
//     "@(#) $Id: ACMM:2dFCanTask/2dFCanTask.cpp,v 1.6 23-Aug-2021 10:48:18+10 tjf $"

//  ------------------------------------------------------------------------------------------------

//                                  I n c l u d e s

//  DRAMA definitions
#include "drama.hh"

// #include "drama/parsys.hh"
// #include "drama/sds.hh"

#include "TdFCanTask.h"
#include "TdFCanTask_err.h"
#include "TdFCanTask_err_msgt.h"
//  Access to CML amplifiers and I/O modules that handles all simulation aspects.
#include "CanAccess.h"

//  CML (Copley motion library) definitions.
#include "CML.h"

//  The simuated CAN interface definitions.
#include "SimCanInterface.h"

//  Utility routines like ExpandFileName() were written ages ago for the TCS system, and are
//  still picked up from there.
#include "TcsUtil.h"

//  Standard C/C++ includes that we need.
#include <math.h>
#include <vector>

// the header file to include the sleep function.
#include <unistd.h>
#include <exception>

//  The CAN Anagate interface is an interface to CANBus hardware that uses an Internet interface
//  and the Anagate library. This is not available on all systems - in general, only a Linux box
//  set up as an instrument controller will have this - so for systems that don't have that,
//  we cheat a bit and set up to use a second SimCanInterface instead. In practice, on such
//  systems we probably won't even try to use real hardware (we'll just run everything in
//  simulation) but in principle we could try to use to simulated CANBuses to see if things like
//  CML linkages work with different amplifiers on different CANBuses.

#ifdef USE_CAN_ANAGATE
#include <can_anagate.h>
#else
#define AnaGateCan SimCanInterface
#endif

//  Using DEBUG makes temporary diagnostic messages stand out in the code. It needs to be
//  undefined first because it turns out that the IMP sub-system in DRAMA also defines this.
//  We could use the DRAMA definition, but this puts this code in control of what it does.

#undef DEBUG
#define DEBUG printf

//  The maximum number of amplifiers we handle - two X amplifiers, one each for Y,Z,Theta,Jaw.

#define MAX_TDF_AMPS 3

//  A convenient enum that we can use for the different amps. These can be used as indexes into
//  arrays.

enum AmpId
{
   X1_AMP = 0,
   X2_AMP,
   Y_AMP
};

//  The names of the various amps, as used in the .ini file(s). The order has to match that
//  of the above enum.

static const std::string G_AmpNames[MAX_TDF_AMPS] = {
    "2dFsimGantryX1", "2dFsimGantryX2", "2dFsimGantryY"};

//  The configuration file that defines the set of CANBuses and simulation settings to be used.

static const std::string CONFIGURATION_FILE = "$TDFCTASK_DIR/TdFCTask.conf";

extern const char *const TdFCanTaskVersion;
extern const char *const TdFCanTaskDate;

//  ------------------------------------------------------------------------------------------------

//                                  S t r u c t u r e s
//
//  An AxisDemand structure contains the demand position and velocity for a single axis.

typedef struct
{
   AmpId AxisId;    //  The axis in question.
   double Position; //  The demanded position.
   double Velocity; //  The demanded velocity. 0 => not specified.
} AxisDemand;

//  ------------------------------------------------------------------------------------------------

//                             A c t i o n  D e f i n i t i o n s

//  INITIALISE - initialise the task as a whole.

class InitialiseAction : public drama::thread::TAction
{
public:
   InitialiseAction(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~InitialiseAction() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

//  G_INIT - initialise and home the 2dF positioner gantry.

class GInitAction : public drama::thread::TAction
{
public:
   GInitAction(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~GInitAction() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

//  G_HOME - homes one or more of the 2dF positioner gantry axes.

class GHomeAction : public drama::thread::TAction
{
public:
   GHomeAction(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~GHomeAction() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// G_HOME_NT is a threaded version of G_HOME
class GHomeActionNT : public drama::thread::TAction
{
public:
   GHomeActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~GHomeActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

//  G_MOVE_AXIS - move one (or more) 2dF positioner gantry axes to specified position(s).

class GMoveAxisAction : public drama::thread::TAction
{
public:
   GMoveAxisAction(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~GMoveAxisAction() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

//  G_MOVE_AXIS_NT is a threaded version of G_MOVE_AXIS, purely for testing purposes.

class GMoveAxisActionNT : public drama::thread::TAction
{
public:
   GMoveAxisActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~GMoveAxisActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

//  G_INIT_NT is a threaded version of G_INIT, purely for testing purposes.

class GInitActionNT : public drama::thread::TAction
{
public:
   GInitActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~GInitActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// G_Park_Gantry_NT is a newly-added action to park the gantry
class GParkGantryActionNT : public drama::thread::TAction
{
public:
   GParkGantryActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~GParkGantryActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// G_UNPARK_NT is a thread version of UnParkGantry action
class GUnParkActionNT : public drama::thread::TAction
{
public:
   GUnParkActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~GUnParkActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// G_MOVEOFFSET_NT is a thread version of MoveOffset action
class GMoveOffsetActionNT : public drama::thread::TAction
{
public:
   GMoveOffsetActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~GMoveOffsetActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// G_EXIT is a thread version of Exit action
class GEXITActionNT : public drama::thread::TAction
{
public:
   GEXITActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~GEXITActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// G_MOVEActionNT is a thread version of Move action, it moves a particular axis
class GMOVEActionNT : public drama::thread::TAction
{
public:
   GMOVEActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~GMOVEActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// G_RESETActionNT is a thread version of Move action, it moves a particular axis
class GRESETActionNT : public drama::thread::TAction
{
public:
   GRESETActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~GRESETActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// P_TELPOSActionNT is a thread version of TELPOS action, it sets the current position of telescope
class PTELPOSActionNT : public drama::thread::TAction
{
public:
   PTELPOSActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~PTELPOSActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// P_SetActionNT is a thread version of SET action, it sets a signal parameter of tdffpi task
class PSetActionNT : public drama::thread::TAction
{
public:
   PSetActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~PSetActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// P_ResetLockActionNT is a thread version of RESET_LOCK action, it resets the internal lock
class PResetLockActionNT : public drama::thread::TAction
{
public:
   PResetLockActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~PResetLockActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// P_SetCoeffsActionNT is a thread version of SET_COFFS action, it resets the image cofficients
class PSetCoeffsActionNT : public drama::thread::TAction
{
public:
   PSetCoeffsActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~PSetCoeffsActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// P_SetImageActionNT is a thread version of SET_IMAGE action, it sets the image
class PSetImageActionNT : public drama::thread::TAction
{
public:
   PSetImageActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~PSetImageActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// P_SetWindowActionNT is a thread version of SET_Window action, it sets the window size
class PSetWindowActionNT : public drama::thread::TAction
{
public:
   PSetWindowActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~PSetWindowActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// P_SetVelActionNT is a thread version of SET_VEL action, it sets the velocity of the gantry
class PSetVelActionNT : public drama::thread::TAction
{
public:
   PSetVelActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~PSetVelActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// P_SetPlateActionNT is a thread version of SET_PLATE action, it sets the status of the plate
class PSetPlateActionNT : public drama::thread::TAction
{
public:
   PSetPlateActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~PSetPlateActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// P_UpdateFlexActionNT is a thread version of UPDATE_FLEX action, it rereads the flex file
class PUpdateFlexActionNT : public drama::thread::TAction
{
public:
   PUpdateFlexActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~PUpdateFlexActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// P_UpdateActionNT is a thread version of UPDATE action, it updates the pose
class PUpdateActionNT : public drama::thread::TAction
{
public:
   PUpdateActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~PUpdateActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// P_ReportActionNT is a thread version of Report action, it reports the value of a fpi task parameter
class PReportActionNT : public drama::thread::TAction
{
public:
   PReportActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~PReportActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// P_ReportLocksActionNT is a thread version of ReportLocks action, it reports the status of internal lock
class PReportLocksActionNT : public drama::thread::TAction
{
public:
   PReportLocksActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~PReportLocksActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// P_ReportCoeffsActionNT is a thread version of ReportCoeffs action, it reports the coeffs
class PReportCoeffsActionNT : public drama::thread::TAction
{
public:
   PReportCoeffsActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~PReportCoeffsActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// P_ReportImageActionNT is a thread version of ReportImage action, it reports the details of image
class PReportImageActionNT : public drama::thread::TAction
{
public:
   PReportImageActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~PReportImageActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// P_ReportWindowActionNT is a thread version of ReportWindow action, it reports the details of window
class PReportWindowActionNT : public drama::thread::TAction
{
public:
   PReportWindowActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~PReportWindowActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// P_SaveDefsActionNT is a thread version of SaveDef action, it saves Def into the tdfFpiDef.sds
class PSaveDefsActionNT : public drama::thread::TAction
{
public:
   PSaveDefsActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~PSaveDefsActionNT() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

// C_Search is a thread version of Search action, it searches the fibre-end or fiducial
class CSearchAction : public drama::thread::TAction
{
public:
   CSearchAction(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask), _theTask(theTask) {}
   ~CSearchAction()
   {
      if (m_tdFfpiSFStruct)
      {
         delete (m_tdFfpiSFStruct);
         m_tdFfpiSFStruct = nullptr;
      }
   }

private:
   tdFfpiSFtype *m_tdFfpiSFStruct;
   std::weak_ptr<drama::Task> _theTask;

private:
   void ActionThread(const drama::sds::Id &) override;

   bool MoveToSearchPosition(const long searchX, const long searchY, short *const atSearchXY,
                             short *const searchStarted);
   bool PerformCentroid(drama::Path &cameraPath, const tdFfpiCENtype *cenWin, const double settletime, short *const centroided);

   bool CheckCentroid(tdFfpiTaskType *details, short *const attempts, long *const searchX, long *const searchY, short *const atSearchXY,
                      short *const centroided, short *const foundIt, int *const i, int *const j, int *const k, short *const checkedCentroid,
                      short *const centroidRepeated, short *const repeatChecked);

   void CheckCent_ObjectHasNotYetBeenSeen(tdFfpiTaskType *details, long *const searchX, long *const searchY, short *const atSearchXY, short *const centroided,
                                          short *const attempts, short *const foundIt, int *const i, int *const j, int *const k,
                                          short *const checkedCentroid, short *const centroidRepeated, short *const repeatChecked);

   void CheckCent_ObjectNotFound(long *const searchX, long *const searchY, short *const atSearchXY, short *const centroided,
                                 int *const i, int *const j, int *const k, short *const checkedCentroid,
                                 short *const centroidRepeated, short *const repeatChecked);

   void CheckCent_ObjectFound(tdFfpiTaskType *details, long *const searchX, long *const searchY, short *const atSearchXY,
                              short *const centroided, short *const attempts, short *const foundIt, short *const checkedCentroid);

   bool CheckRepeatCentroid(tdFfpiTaskType *details, short *const centroidRepeated, short *const repeatChecked);

   void ActionComplete_CalculateMean(long *const searchX, long *const searchY);
   void ActionComplete_FoundItCheck(long *const searchX, long *const searchY, short *const foundIt);
   void ActionComplete(tdFfpiTaskType *details, long searchX, long searchY, short foundIt);
};

class CCentroidAction : public drama::thread::TAction
{
public:
   CCentroidAction(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~CCentroidAction() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

class CImageAction : public drama::thread::TAction
{
public:
   CImageAction(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
   ~CImageAction() {}

private:
   void ActionThread(const drama::sds::Id &) override;
};

class CZeroCamAction : public drama::thread::TAction
{
public:
   CZeroCamAction(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask), _theTask(theTask) {}
   ~CZeroCamAction()
   {
      if (m_tdFfpiZCStruct)
      {
         delete (m_tdFfpiZCStruct);
         m_tdFfpiZCStruct = nullptr;
      }
   }

private:
   tdFfpiZCtype *m_tdFfpiZCStruct;
   std::weak_ptr<drama::Task> _theTask;

private:
   void ActionThread(const drama::sds::Id &) override;
};

class CShiftCoAction : public drama::thread::TAction
{
public:
   CShiftCoAction(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask), _theTask(theTask) {}
   ~CShiftCoAction()
   {
      if (m_tdFfpiSHStruct)
      {
         delete (m_tdFfpiSHStruct);
         m_tdFfpiSHStruct = nullptr;
      }
   }

private:
   tdFfpiSHtype *m_tdFfpiSHStruct;
   std::weak_ptr<drama::Task> _theTask;

private:
   void ActionThread(const drama::sds::Id &) override;
};

class CSurveyAction : public drama::thread::TAction
{
public:
   CSurveyAction(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask), _theTask(theTask) {}
   ~CSurveyAction()
   {
      if (m_tdFfpiSStruct)
      {
         delete (m_tdFfpiSStruct);
         m_tdFfpiSStruct = nullptr;
      }
   }

private:
   tdFfpiStype *m_tdFfpiSStruct;
   std::weak_ptr<drama::Task> _theTask;
   const int MAX_FAILURES = 4;
   const int MIN_MARKS = 4;
   const double RMS_WARNING=20.0;

private:
   void ActionThread(const drama::sds::Id &) override;
   bool FidNotFound(double *const expectedX, double *const expectedY,
                    const short curFid, short *const atFid, short *const recordedFid);
   void SearchForFid(const long offsetX, const long offsetY, const short curFid, short *const atFid);
   void RecordFid(double *const expectedX, double *const expectedY, double *const measuredX,
                  double *const measuredY, long *const offsetX, long *const offsetY,
                  short *const curFid, short *const atFid, short *const recordedFid);
   void ActionComplete(tdFfpiTaskType *details, const double *const expectedX, const double *const expectedY,
                       const double *const measuredX, const double *const measuredY,
                       const double ha, const double dec, const short fitType, const double plateTheta,
                       drama::sds::Id *paramId);
   void SetCoeffs(tdFfpiTaskType *details, const double *const expectedX, const double *const expectedY,
                  const double *const measuredX, const double *const measuredY,
                  const short fitType, double *const coeffs, const double plateTheta, drama::sds::Id *paramId);
   bool OffsetAndDisplayResults(tdFfpiTaskType *details, const double platetheta, const short fitType, const short centerFid, double measuredArray[][2], double fiducialArray[][2],
                                double *const coeffs, drama::sds::Id *paramId);
   void tdFfpiSurveyCheckCoeffs(const double *current, const double *newcoeffs,
                                int plate, const double plateTheta, drama::sds::Id *paramId);
   void DisplayResultsBasic(const int newmodel, double fiducialArray[][2],
                            double cal[][2], const double xrms, const double yrms, const double rrms, drama::sds::Id *paramId);
   void DisplayResultsInEncoderUnits(tdFfpiTaskType *details, const double plateTheta,
                                     const int newmodel, const double *const coeffs, double measuredArray[][2]);
   void ConstructReturnValue(tdFfpiTaskType *details, const double *const expectedX, const double *const expectedY, const double *const measuredX,
                                         const double *const measuredY, const double *const coeffs, const double ha,
                                         const double dec, drama::sds::Id *paramId);
};

//  ------------------------------------------------------------------------------------------------

//                                  T a s k  D e f i n i t i o n

class TdFCanTask : public drama::Task
{
public:
   //  Constructor
   TdFCanTask(const std::string &taskName);
   //  Destructor
   ~TdFCanTask();
   //  Initialise the various mechanisms needed to control the amplifiers.
   bool SetupAmps(void);
   //  Home a specified combination of axes.
   bool HomeAxes(bool HomeX, bool HomeY, bool HomeZ, bool HomeTheta, bool HomeJaw);
   //  Move one or more axes to specified target positions.
   bool MoveAxes(const std::vector<AxisDemand> &AxisDemands, bool MoveOffset = false,
                 drama::thread::TAction *ThisAction = NULL);
   //  Return a string describing the most recent error.
   const std::string &GetError(void) { return I_ErrorString; }
   //  Clear the error string.
   void ClearError(void) { I_ErrorString = ""; }

   // Disable the Amplifiers
   bool DisableAmps(void);

   // Initialise the fpi struct
   bool InitialisefpiMainStruct(void);
   // returns the pointer of the fpi task
   tdFfpiTaskType *tdFfpiGetMainStruct() { return I_tdFfpiMainStruct; }
   // lliu added on 02-05-2024 to read default parameter sds file
   bool tdFfpiDefRead(short loadingFiles, short check);
   // lliu added on 06-05-2024 to set parameter
   bool SetParameter(string &ParameterName, string &ParameterValue);
   // lliu added on 07-05-2024 to write parameter
   bool tdFfpiDefWrite(short savingFiles, short check);
   // lliu added on 09-05-2024 to check the locks
   bool tdFfpiIlocks(long int ilocks);
   // lliu added on 09-05-2024 to update the pos;
   bool tdFfpiUpdatePos(short updateIdeal, short useDpr, short displayText);
   // lliu added on 06-06-2024 to compute the theata based on the gantry position;
   double tdFautoThetaPos(long int x, long int y);
   // lliu added on 06-06-2024 to check if a position is forbidden
   int tdFforbidden(double x, double y, double theta, double rq, double ri,
                    double ro, double hw, double jhwp, double jhwm, double jl, double clearance,
                    int *inside, int *outside);
   // lliu added on 26-06-2024 to check if the position of an image before and after centorid
   void tdFfpiPreExp();
   void tdFfpiPostExp();

   void tdFfpiConvertFromFP(long int xFp, long int yFp, double plateTheta, short level,
                            double *xCon, double *yCon);
   void tdFfpiConvertFromEnc(int xEnc, int yEnc, double plateTheta,
                             short level, double *xCon, double *yCon);

   drama::Path &tdFGetCameraPath() { return I_pCameraPath; };

private:
   //  Set up the homing configuration for a specified amplifier.
   bool SetupHomeConfig(CML::HomeConfig HomeConfigs[], int Index, AmpId Amp);
   //  Return a pointer to the amplifier corresponding to a specified axis/amp.
   CML::Amp *GetAmp(AmpId AxisId);
   //  Perform standard setup for a linkage, given a list of amplifiers to link.
   bool SetupLinkage(CML::Linkage &TheLinkage, unsigned int NumberAmps,
                     CML::Amp *LinkedAmps[], double Limits[] = NULL);

   // Read the default parameter file
   bool tdFfpiReadFile(drama::sds::Id &defId);
   // Write the parameters into a file
   bool tdFfpiWriteFile(drama::sds::Id &defId);
   // Read the flex file
   bool tdFfpiReadFlexFile(drama::sds::Id &defId);

   void tdFfpiFlexure(long int x, long int y,
                      double ha, double dec, TdfFlexType *pars, long int *dx, long int *dy);
   void tdFstateBitSet(unsigned char bit);
   void tdFstateBitClear(unsigned char bit);

   // lliu added on 17-05-2024 to check the target poisiton
   void tdFfpiPositionCheck(int Index, CML::uunit &Position);
   void tdFfpiEncPos(short useDpr, short displayText, tdFencPos *position);
   // --------------------------------------------------------------------------
   // Gantry movement related actions
   // --------------------------------------------------------------------------
   //  The action handler for the INITIALISE action.
   InitialiseAction I_InitialiseActionObj;
   //  The action handler for the G_INIT action.
   GInitAction I_GInitActionObj;
   //  A non-threaded version of G_INIT, called G_INIT_NT
   GInitActionNT I_GInitActionNTObj;
   // The new action handler for G_EXIT action.
   GEXITActionNT I_GExitActionNTObj;
   // The new action handler for G_RESET action.
   GRESETActionNT I_GResetActionNTObj;

   // --------------------------------------------------------------------------
   // Gantry movement related actions
   // --------------------------------------------------------------------------
   //  The action handler for the G_MOVE_AXIS action.
   GMoveAxisAction I_GMoveAxisActionObj;
   //  The action handler for the G_MOVE_AXIS_NT (non-threaded) action.
   GMoveAxisActionNT I_GMoveAxisActionNTObj;
   //  The action handler for the HOME action.
   GHomeAction I_GHomeActionObj;
   // newly-added action handler for the G_HOME_NT (non-threaded) action
   GHomeActionNT I_GHomeActionNTObj;
   // The new action handler for the ParkGantry action
   GParkGantryActionNT I_GParkGantryActionNTObj;
   // The new action handler for G_UNPARK_NT action.
   GUnParkActionNT I_GUnParkActionNTObj;
   // The new action handler for G_MOVEOFFSET_NT action.
   GMoveOffsetActionNT I_GMoveOffsetActionNTObj;
   // The new action handler for G_MOVE_NT action.
   GMOVEActionNT I_GMoveActionNTObj;

   // --------------------------------------------------------------------------
   // Parameter setting related actions
   // --------------------------------------------------------------------------
   // The new action handler for TELPOS action;
   PTELPOSActionNT I_PTelposActionNTObj;
   // The new action handler for Set action;
   PSetActionNT I_PSetActionNTObj;
   // The new action handler for ResetLock action
   PResetLockActionNT I_PResetLockNTObj;
   // The new action handler for SetCoeffs action
   PSetCoeffsActionNT I_PSetCoeffsActionNTObj;
   // The new action handler for SetImage action
   PSetImageActionNT I_PSetImageActionNTObj;
   // The new action handler for SetImage action
   PSetWindowActionNT I_PSetWindowActionNTObj;
   // The new action handler for SetVel action
   PSetVelActionNT I_PSetVelActionNTObj;
   // The new action handler for SetPlate action
   PSetPlateActionNT I_PSetPlateActionNTObj;
   // The new action handler for UpdateFlex action
   PUpdateFlexActionNT I_PUpdateFlexActionNTObj;
   // The new action handler for Update action
   PUpdateActionNT I_PUpdateActionNTObj;

   // --------------------------------------------------------------------------
   // Parameter reporting related actions
   // --------------------------------------------------------------------------
   // The new action handler for Report action
   PReportActionNT I_PReportActionNTObj;
   // The new action handler for Report Lock action
   PReportLocksActionNT I_PReportLocksActionNTObj;
   // The new action handler for Report Coeffs action
   PReportCoeffsActionNT I_PReportCoeffsActionNTObj;
   // The new action handler for Report Image action
   PReportImageActionNT I_PReportImageActionNTObj;
   // The new action handler for Report Window action
   PReportWindowActionNT I_PReportWindowActionNTObj;
   // The new action handler for Parameter Saving action
   PSaveDefsActionNT I_PSaveDefsActionNTObj;

   // --------------------------------------------------------------------------
   // Camera related actions
   // --------------------------------------------------------------------------

   CCentroidAction I_CCentroidActionObj;
   CImageAction I_CImageActionObj;
   CZeroCamAction I_CZeroCamActionObj;

   // --------------------------------------------------------------------------
   // Calibration related actions
   // --------------------------------------------------------------------------
   CSearchAction I_CSearchActionObj;
   CShiftCoAction I_CShiftCoActionObj;
   CSurveyAction I_CSurveyActionObj;

   //  Interface to the CanAccess layer.
   CanAccess I_CanAccess;
   //  True once the CanAccess layer has been initialised.
   bool I_CanAccessInitialised;
   //  The amplifier object used to interact with the X1 gantry servo amp.
   CML::Amp *I_X1Amp;
   //  The amplifier object used to interact with the X2 gantry servo amp.
   CML::Amp *I_X2Amp;
   //  The amplifier object used to interact with the Y gantry servo amp.
   CML::Amp *I_YAmp;
   //  The amplifier object used to interact with the Z gantry servo amp.
   CML::Amp *I_ZAmp;
   //  The amplifier object used to interact with the theta servo amp.
   CML::Amp *I_ThetaAmp;
   //  The amplifier object used to interact with the jaw gantry servo amp.
   CML::Amp *I_JawAmp;
   //  Description of the latest error, for use by action handlers.
   std::string I_ErrorString;

   // the pointer structure to store the parameters of fpi task added by lliu 01/05/2024
   tdFfpiTaskType *I_tdFfpiMainStruct;
   drama::ParSys I_TdFCanTaskParSys;
   drama::Path I_pCameraPath;

public:
   drama::Parameter<double> dzero;
   drama::Parameter<double> dzeroHA;
   drama::Parameter<double> dzeroDEC;
   drama::Parameter<double> plateTheta;
   drama::Parameter<double> settleTime;
   drama::Parameter<double> zerocamCenWait; /* Wait for gantry to settle before oing centroid in zerocam operation. */

   drama::Parameter<INT32> xPark;
   drama::Parameter<INT32> yPark;
   drama::Parameter<INT32> xCopleyPark;
   drama::Parameter<INT32> yCopleyPark;
   drama::Parameter<INT32> xyVel;
   drama::Parameter<INT32> xaccuracy;
   drama::Parameter<INT32> yaccuracy;
   drama::Parameter<INT32> stepSize;
   drama::Parameter<INT32> maxError;
   drama::Parameter<INT32> posTol;
   drama::Parameter<INT32> timeoutFac;
   drama::Parameter<INT32> fibInImgThres;
   drama::Parameter<INT32> copleyCodeVer; /* <---- PMAC code version number */
   drama::Parameter<INT32> overlayPlaneEnabled;
   // drama::Parameter<INT32> copleyXPosLim;
   //  drama::Parameter<INT32> copleyXNegLim;
   //  drama::Parameter<INT32> copleyYPosLim;
   //  drama::Parameter<INT32> copleyYNegLim;

   drama::Parameter<short> attempts;
   drama::Parameter<short> shortZeroX;
   drama::Parameter<short> shortZeroY;

   drama::Parameter<unsigned short> ushortZero;
   drama::Parameter<unsigned short> ushortZeroCentroid;

   drama::Parameter<std::string> tdfTaskStr;
   drama::Parameter<std::string> tdfSimSearchRunStr;
   drama::Parameter<std::string> tdfVersionStr;
   drama::Parameter<std::string> tdfVersionDate;
   drama::Parameter<std::string> tdfCanMode;
   drama::Parameter<std::string> tdfCanParked;
   drama::Parameter<std::string> tdfGantryLamp;
};

//  ------------------------------------------------------------------------------------------------

//                              U t i l i t y  R o u t i n e s
//
//  This is a set of Utilty rotuines that don't need to access internal variables of any class,
//  and so are much simpler as stand-alone routines. It might make more sense to collect them
//  in some separate file. Although some, like WhichAxes() are clearly quite specific to this
//  task, others, like SplitString() are much more general.

//  ------------------------------------------------------------------------------------------------

//                          U n b l o c k  S I G U S R 2
//
//  This is an important little bit of housekeeping to do with signals and DRAMA threads.
//  The CML implementation used on OS X makes use of the SIGUSR2 signal. (This is because CML
//  makes a lot of use of sem_timedwait(), a routine that OS X inexplicably doesn't implement.
//  The AAO CML implementation uses a version of sig_timedwait() that interrupts a sem_wait()
//  call with a SIGUSR2 signal when the timeout period expires.) Now, normally, DRAMA2 sets up
//  all threads with a signal mask that blocks all signals. (See the DRAMA documentation for
//  more details.) This means that in order for CML code to be used in a DRAMA thread, this
//  routine has to be called before any CML code gets executed. It's probably good form to
//  reblock SIGUSR2 at the end of the CML code, using a call to BlockSIGUSR2(), but it's very
//  important that this not be given until all CML code has finished executing - and this
//  includes destructors! The best thing is to have any CML code in a subroutine called from
//  the main action handler, and have the action handler call this routine right at the start
//  and then call BlockSIGUSR2 at the end. Note that the task itself has instance variables
//  that are CML objects, and so their destructors will be called as the task closes down,
//  and SIGUSR2 will need to be unblocked at that point in the main thread.
//
//  This code could be null on non-OSX systems, but that probably isn't important.

void UnblockSIGUSR2(void)
{
   /* Moving to a version of CML that doesn't use signals seemed to fix a lot of problems. To
    * make sure that signals aren't an issue with this CML version, I've commented out both
    * UnblockSIGUSR2() and BlockSIGUSR2(). If we ever move back to a signals-based CML, at least
    * on OSX, these will need to be re-enabled.
      sigset_t Set;
      sigemptyset(&Set);
      sigaddset(&Set, SIGUSR2);
      pthread_sigmask(SIG_UNBLOCK, &Set, NULL);
   */
}

//  ------------------------------------------------------------------------------------------------

//                           B l o c k  S I G U S R 2
//
//  This is an important little bit of housekeeping to do with signals and DRAMA threads, which
//  blocks the SIGUSR2 signal. See the comments for UnblockSIGUSR2() for details.

void BlockSIGUSR2(void)
{
   /* For explanation about why this is commented out, see UnblockSIGUSR2() comments.
      sigset_t Set;
      sigemptyset(&Set);
      sigaddset(&Set, SIGUSR2);
      pthread_sigmask(SIG_BLOCK, &Set, NULL);
   */
}

//  ------------------------------------------------------------------------------------------------
//
//                                  S p l i t  S t r i n g
//
//  SplitString() is a simple utility that takes a string that contains sub-strings delimited
//  by spaces and/or commas and returns a vector of the individual sub-strings. Some examples:
//
//  "ab,cde,f"                     => 'ab' 'cd' 'f'
//   "  ab     c , k , ,abc ,,"    => 'ab' 'c' 'k' '' 'abc' '' ''
//
//  Note that successive commas are treated as delimiting an empty string, and a final comma
//  is treated as indicating the presence of a final empty string. There is no way to include
//  a sub-string that has a non-zero number of spaces - a string of spaces is simply taken as
//  a delimiter.

std::vector<std::string> SplitString(const std::string &ArgString)
{
   std::vector<std::string> ReturnedList;

   unsigned int Len = ArgString.length();
   std::string Item = "";
   unsigned int Index = 0;
   int Commas = 0;
   for (;;)
   {
      //  Skip leading spaces
      while (Index < Len && ArgString[Index] == ' ')
      {
         Index++;
      }
      if (Index == Len)
         break;
      //  Add characters up to next delimiter into current Item.
      while (Index < Len && ArgString[Index] != ' ' && ArgString[Index] != ',')
      {
         Item = Item + ArgString[Index++];
      }
      if (Index == Len)
         break;
      //  Add the delimited string to the returned list, and start again with an empty string.
      ReturnedList.push_back(Item);
      Item = "";
      //  Go through until we come to something that isn't commas or blanks.
      //  Count the commas as we go.
      Commas = 0;
      while (Index < Len && (ArgString[Index] == ' ' || ArgString[Index] == ','))
      {
         if (ArgString[Index++] == ',')
            Commas++;
      }
      if (Index == Len)
         break;
      //  One less than the number of commas, is the number of blank items in that
      //  set of commas and delimiters.
      for (int I = 1; I < Commas; I++)
         ReturnedList.push_back("");
   }
   //  And once we're through the string, we have to see what we had found between
   //  the last item and the end of the string. Here the number of commas is the
   //  number of blank items, since we count a final comma as indicating a final
   //  blank item. Normally, though, we just have the non-blank item we were
   //  building up when we hit the end of the string.
   if (Item != "")
   {
      ReturnedList.push_back(Item);
   }
   else
   {
      for (int I = 0; I < Commas; I++)
         ReturnedList.push_back("");
   }
   return ReturnedList;
}

//  ------------------------------------------------------------------------------------------------

//                                   G e t  D o u b l e
//
//  A small utility that packages up use of std::stod() - which means this needs C++11 - to
//  parse a string that is supposed to contain a floating point value. If the string is valid,
//  this routine returns true and the value in the Value parameter. If invalid, it returns
//  false and sets the Value parameter to zero.

bool GetDouble(const std::string &ValueString, double &Value)
{
   bool OK = true;
   std::size_t Pos;
   try
   {
      Value = std::stod(ValueString, &Pos);
      if (Pos != ValueString.length())
         OK = false;
   }
   catch (...)
   {
      Value = 0.0;
      OK = false;
   }
   return OK;
}

long int doubleToLong(double doubleNum)
{
   double doubleTemp;
   long int longTemp, addAmount, one = 1;
   short negativeNumber;

   negativeNumber = (doubleNum < 0) ? YES : NO;
   if (negativeNumber == YES)
      doubleNum = -doubleNum;

   longTemp = doubleNum / one;
   doubleTemp = doubleNum - (double)longTemp;
   addAmount = (doubleTemp < 0.5) ? 0 : 1;
   longTemp += addAmount;
   if (negativeNumber)
      longTemp = -longTemp;

   return (longTemp);
}

// Setup the Positions and Velocities
void SetupPosAndVel(std::string Axes, std::string &Positions, std::string &Velocities, bool isHomed = false)
{
   std::vector<std::string> AxesList = SplitString(Axes);
   unsigned int NumberAxes = AxesList.size();
   if (NumberAxes == 0)
   {
      return;
   }
   if (isHomed)
   {
      for (unsigned int Index = 0; Index < NumberAxes; Index++)
      {
         Positions += "0";
         Positions += ",";
         Velocities += "1";
         Velocities += ",";
      }
   }
   else
   {
      for (unsigned int Index = 0; Index < NumberAxes; Index++)
      {
         Positions += "5000";
         Positions += ",";
         Velocities += "1";
         Velocities += ",";
      }
   }
   if (Positions.back() == ',')
      Positions.pop_back();
   if (Velocities.back() == ',')
      Velocities.pop_back();
}
//  ------------------------------------------------------------------------------------------------

//                                   G e t  D e m a n d s
//
//  GetDemands() is a utility that takes a set of strings giving a set of axes, a set of
//  corresponding positions, and a set of corresponding velocities (optional - this string
//  can be null) - and validates them, at least in terms of syntax, and returns a vector of
//  AxisDemand structures that match the contents of the strings. If there is an error,
//  this routine returns an empty list and a desctiption of the problem in the passed Error
//  string. The specification strings should be lists separated by commas and/or blanks, as
//  accepted by SplitString().

std::vector<AxisDemand> GetDemands(
    const std::string &Axes, const std::string &Positions, const std::string &Velocities,
    std::string &Error)
{
   std::vector<AxisDemand> AxisDemands;

   bool OKSoFar = true;
   bool VelocitiesSpecified = false;

   std::vector<std::string> AxesList = SplitString(Axes);
   std::vector<std::string> PositionsList = SplitString(Positions);
   std::vector<std::string> VelocitiesList = SplitString(Velocities);

   unsigned int NumberAxes = AxesList.size();
   if (NumberAxes == 0)
   {
      Error = "No axes specified.";
      OKSoFar = false;
   }
   else
   {
      if (PositionsList.size() != NumberAxes)
      {
         Error = "Number of positions does not match number of axes.";
         OKSoFar = false;
      }
      else
      {
         if (VelocitiesList.size() > 0)
         {
            VelocitiesSpecified = true;
            if (VelocitiesList.size() != NumberAxes)
            {
               Error = "Number of velocities does not match number of axes.";
               OKSoFar = false;
            }
         }
      }
   }
   if (OKSoFar)
   {
      for (unsigned int Index = 0; Index < NumberAxes; Index++)
      {
         std::string AxisName = AxesList[Index];
         double Position = 0.0;
         double Velocity = 0.0;
         if (VelocitiesSpecified)
         {
            if (!GetDouble(VelocitiesList[Index], Velocity))
            {
               Error = "Invalid velocity specification: " + VelocitiesList[Index];
               OKSoFar = false;
               break;
            }
         }
         if (!GetDouble(PositionsList[Index], Position))
         {
            Error = "Invalid position specification: " + PositionsList[Index];
            OKSoFar = false;
            break;
         }
         AxisDemand Demand;
         Demand.Position = Position;
         Demand.Velocity = Velocity;

         //  'X' ia a special case, because it refers to both X1 and X2 axes.

         if (TcsUtil::MatchCaseBlind(AxisName, "X"))
         {
            Demand.AxisId = X1_AMP;
            AxisDemands.push_back(Demand);
            Demand.AxisId = X2_AMP;
            AxisDemands.push_back(Demand);
         }
         else
         {
            if (TcsUtil::MatchCaseBlind(AxisName, "Y"))
               Demand.AxisId = Y_AMP;
            else
            {
               Error = "Invalid axis specification: " + AxesList[Index];
               OKSoFar = false;
               break;
            }
            AxisDemands.push_back(Demand);
         }
      }
   }
   if (!OKSoFar)
      AxisDemands.clear();

   return AxisDemands;
}

//  ------------------------------------------------------------------------------------------------

//                                    W h i c h  A x e s
//
//  WhichAxes() is a simple utility that takes a string that specifies a set of axes, and returns
//  a set of bool values to indicate which of the axes were specified in the string. The axes are
//  X,Y,Z,Theta and Jaw. These can be specified with their full names, separated by commas or
//  spaces. Alternatively, a short form can be used where only the first letter of each axis is
//  used, in which case space or comma delimiters are not needed. So 'XYZT' is the same as
//  "X,Y,Z,Theta". This routine returns true if it parsed the string without error, false if
//  the string was invalid.

static bool WhichAxes(const std::string &Axes, bool &X, bool &Y, bool &Z, bool &Theta, bool &Jaw)
{
   X = Y = Z = Theta = Jaw = false;

   std::string AxisNames[] = {"X", "Y", "Z", "THETA", "JAW"};
   bool AxesEnabled[5];

   bool Valid = true;

   for (int I = 0; I < 5; I++)
   {
      AxesEnabled[I] = false;
   }

   //  Are all the characters one of X,Y,Z,T,J? That is, is this a string like "XYZT" or "J"?

   bool Simple = true;
   int Len = Axes.length();
   for (int I = 0; I < Len; I++)
   {
      char Chr = toupper(Axes[I]);
      bool Known = false;
      for (int Axis = 0; Axis < 5; Axis++)
      {
         if (Chr == AxisNames[Axis][0])
         {
            AxesEnabled[Axis] = true;
            Known = true;
            break;
         }
      }
      if (!Known)
      {
         Simple = false;
         break;
      }
   }

   //  If so, we're there. If not, we need to split the string up, either by commas or spaces.
   //  We build up Name as the name of the current axis, and once we hit a delimiter, we check it
   //  against the various full axis names. (I suppose this could be a compare with a subset of
   //  the name, which would allow abbreviations.) The loop goes one past the end of the actual
   //  characters in the string, to catch the case where there are no delimiters in the string -
   //  this treats the string as effectively ending with a single blank character.

   if (!Simple)
   {
      std::string Name = "";
      for (int I = 0; I <= Len; I++)
      {
         char Chr = ' ';
         if (I < Len)
            Chr = Axes[I];
         if (Chr == ' ' || Chr == ',')
         {

            //  Found a delimiter. See if we recognise what we have in Name..

            if (Name != "")
            {
               bool Known = false;
               for (int Axis = 0; Axis < 5; Axis++)
               {
                  if (Name == AxisNames[Axis])
                  {
                     AxesEnabled[Axis] = true;
                     Name = "";
                     Known = true;
                     break;
                  }
               }
               if (!Known)
               {
                  Valid = false;
                  break;
               }
            }
         }
         else
         {

            //  Not a delimiter. Add the character to Name, so Name is the upper case version
            //  of what was specified in the string.

            Name += toupper(Chr);
         }
      }
   }

   //  I think this was more complicated than it needed to be. I suspect I could have combined
   //  the simple and the more complex loops through the string.

   if (Valid)
   {

      //  This is one bit of the code where the order of the axes in AxisNames[] matters.

      X = AxesEnabled[0];
      Y = AxesEnabled[1];
      Z = AxesEnabled[2];
      Theta = AxesEnabled[3];
      Jaw = AxesEnabled[4];
   }
   return Valid;
}

void TdFCanTask::tdFfpiEncPos(short useDpr, short displayText, tdFencPos *position)
{
   if (useDpr)
   {
      if (displayText)
         DEBUG("Assuming current encoder position is previous target position.\n");
      position->x = I_tdFfpiMainStruct->atEnc.x;
      position->y = I_tdFfpiMainStruct->atEnc.y;
   }
}

void TdFCanTask::tdFfpiPreExp()
{
   tdFimagePos *details = &I_tdFfpiMainStruct->imagePos;
   if (details->enable)
      tdFfpiEncPos(YES, YES, &details->enc);
   else
      DEBUG("tdFfpiPreExp cannot be implemented because details->enable is set to NO.\n");
}

void TdFCanTask::tdFfpiPostExp()
{
   tdFimagePos *details = &I_tdFfpiMainStruct->imagePos;
   if (details->enable)
   {
      tdFencPos now;
      double atFpX, atFpY, plateTheta;
      tdFfpiEncPos(YES, YES, &now);
      details->enc.x = (now.x + details->enc.x) / 2;
      details->enc.y = (now.y + details->enc.y) / 2;
      I_TdFCanTaskParSys.Get("PLATE_THETA", &plateTheta);
      tdFfpiConvertFromEnc(details->enc.x, details->enc.y, plateTheta, _FULL,
                           &atFpX, &atFpY);
      details->p.x = doubleToLong(atFpX);
      details->p.y = doubleToLong(atFpY);
      DEBUG("Average gantry position during image is x:%ld, y:%ld\n",
            details->p.x, details->p.y);
   }
}

void TdFCanTask::tdFfpiConvertFromFP(long int xFp, long int yFp,
                                     double plateTheta, short level, double *xCon, double *yCon)
{
   double x_fp, y_fp, x_enc, y_enc, ha, dec;
   long int dx = 0, dy = 0;

   I_TdFCanTaskParSys.Get("HA", &ha);
   I_TdFCanTaskParSys.Get("DEC", &dec);

   x_fp = (double)(xFp * cos(plateTheta) - yFp * sin(plateTheta));
   y_fp = (double)(yFp * cos(plateTheta) + xFp * sin(plateTheta));

   if ((level == _ALL) || (level == _COEFFS))
   {
      *xCon = x_fp;
      *yCon = y_fp;
      return;
   }
   /*
    * Adjust for offset of plate one center fiducial from plate 2 center fiducial.
    */
   if (I_tdFfpiMainStruct->currentPlate == 1)
   {
      if (I_tdFfpiMainStruct->plateOneDontRemove)
      {
         /*
        fprintf(stderr,
                "FPI:FromFP:Converting for plate 1, but offset removal disabled\n");
         */
         ;
      }
      else
      {
         double t_x = x_fp;
         double t_y = y_fp;
         x_fp = t_x - (double)(*(I_tdFfpiMainStruct->pars.plt1CenterFidOffsetX));
         y_fp = t_y - (double)(*(I_tdFfpiMainStruct->pars.plt1CenterFidOffsetY));
         /*
         fprintf(stderr,
                 "FPI:FromFP:Plate 1 position changed by %d, %d from %.1f, %.1f to %.1f, %.1f\n",
                 -1*(*(I_tdFfpiMainStruct->pars.plt1CenterFidOffsetX)),
                 -1*(*(details->pars.plt1CenterFidOffsetY)),
                 t_x, t_y, x_fp, y_fp);
         */
      }
   }

   slaXy2xy(x_fp, y_fp,                         /* Field plate coordinates */
            I_tdFfpiMainStruct->convert.coeffs, /* Transformation matrix   */
            &x_enc, &y_enc);                    /* Encoder coordinates     */

   if (level == _TEMP)
   {
      *xCon = x_enc;
      *yCon = y_enc;
      return;
   }

   if (level == _FLEX)
   {
      *xCon = x_enc;
      *yCon = y_enc;
      return;
   }

   /*
    *  Compensate for system deflections.
    */
   tdFfpiFlexure((long int)x_enc, (long int)y_enc, ha, dec,
                 &I_tdFfpiMainStruct->convert.flex, &dx, &dy);
   *xCon = x_enc - (double)dx;
   *yCon = y_enc - (double)dy;
}

int TdFCanTask::tdFforbidden(double x, double y, double theta, double rq, double ri,
                             double ro, double hw, double jhwp, double jhwm, double jl, double clearance,
                             int *inside, int *outside)
{
   double t;      /* temporary					*/
   double rr;     /* positioner radius squared			*/
   double xb, yb; /* coordinates after rotation			*/
   double jhw;    /* greater jaw halfwidth                        */
   double jo;     /* jaw offset from reference position		*/
   double jjo;    /* jaw offset squared				*/
   int sector;    /* 0 = guide sector, otherwise > 0              */

   if (clearance < CLEARANCE)
      clearance = CLEARANCE;

   jhw = jhwp;
   if (jhwm > jhw)
      jhw = jhwm;
   rr = x * x + y * y;
   jjo = jhw * jhw + jl * jl;
   jo = sqrt(jjo) + clearance;

   if (rr < (ri - jo) * (ri - jo))
   {
      *inside = 1;
      *outside = 0;
      return (0);
   }

   /* quick check for outside						*/

   if (rr > (ro + jo) * (ro + jo))
   {
      *inside = 0;
      *outside = 1;
      return (0);
   }

   *outside = 0;

   theta += PION2;

   /* Flip to get mirror image.  *** MAY NEED TO REMOVE THIS. ***		*/

   x = -x;
   theta = TWOPI - theta;

   /* Collapse to single quadrant.						*/

   if (x < hw && y >= hw)
   {
      t = -x;
      x = y;
      y = t;
      theta -= PION2;
   }
   else if (x < -hw && y < hw)
   {
      x = -x;
      y = -y;
      theta += PI;
   }
   else if (x >= -hw && y < -hw)
   {
      t = -y;
      y = x;
      x = t;
      theta += PION2;
   }

   while (theta > PI)
      theta -= TWOPI;
   while (theta < -PI)
      theta += TWOPI;

   x -= hw;
   y -= hw;

   xb = x * COS4P5 + y * SIN4P5;
   yb = y * COS4P5 - x * SIN4P5;
   x = xb;
   y = yb;
   theta -= R4P5;
   sector = 0;

   while (y / x > TAN4P5)
   {
      xb = x * COS9 + y * SIN9;
      yb = y * COS9 - x * SIN9;
      x = xb;
      y = yb;
      theta -= R9;
      sector++;
   }

   jo = jl * cos(theta) + clearance;
   if (theta > 0.0)
      jo += jhwp * sin(theta); /* REMINDER: I have fliped   */
   else
      jo -= jhwm * sin(theta); /* the coordinate system !   */

   if (x < rq - jo)
   {

      /*  So far, so good.  What about the adjacent sector ?                  */

      if (y > 0.0)
      {
         /* Next sector                                                          */
         if (sector == 9)
            x -= 2. * hw;          /* Next sector may be the  */
         xb = x * COS9 + y * SIN9; /* next quadrant !         */
         yb = y * COS9 - x * SIN9;
         theta -= R9;
         jo = jl * cos(theta) + clearance - jhwm * sin(theta);

         if (xb < rq - jo)
         {
            *inside = 1;
            return (0);
         }
      }
      else
      {
         /* Previous sector                                                      */
         if (sector == 0)
            y += 2. * hw;          /* Previous sector can be   */
         xb = x * COS9 - y * SIN9; /* the previous quadrant !  */
         yb = y * COS9 + x * SIN9;
         theta += R9;
         jo = jl * cos(theta) + clearance + jhwp * sin(theta);

         if (xb < rq - jo)
         {
            *inside = 1;
            return (0);
         }
      }
   }

   *inside = 0;
   return (1);
}

//  ------------------------------------------------------------------------------------------------

//                               W a i t  L i n k e d  H o m e
//
//  WaitLinkedHome() is a version of the standard CML::Linkage routine WaitMoveDone(), except that
//  WaitMoveDone() is not able to detect following errors that might be caused by an axis sticking
//  or otherwise failing during the homing, for the simple reason that doing so requires it to
//  know the actual position of each mechanism, and that isn't available to it during a homing
//  operation, because it's the home operation that locates the zero points of the encoders and
//  allows the amplifiers to know their absolute positions. This routine adds a polling loop to
//  the normal WaitMoveDone() code, that checks that any mechanism that has not yet reached its
//  home position is continuing to move. Note that for 2dF, this can be very important - if X1
//  were to stick while X2 continued to move, the gantry alignment would go beyond tolerances
//  and something nasty is almost certain to result.

const CML::Error *WaitLinkedHome(
    CML::Linkage *TheLinkage, CML::int32 timeout, CML::uunit Targets[] = NULL)
{
   DEBUG("In WaitLinkedHome\n");
   const int MAX_AMPS = 10;
   static const CML::uunit MOVE_TOLERANCE = 5.0;

   CML::cml.Debug("Link %d waiting on move\n", TheLinkage->GetAmp(0).GetNodeID());

   CML::uint32 value = CML::LINKEVENT_MOVEDONE | CML::LINKEVENT_NODEGUARD | CML::LINKEVENT_FAULT |
                       CML::LINKEVENT_ERROR | CML::LINKEVENT_DISABLED | CML::LINKEVENT_QUICKSTOP |
                       CML::LINKEVENT_ABORT;

   CML::EventAny e(value);

   CML::LINK_EVENT match;

   //  At this point, the proper linkage code does a WaitEvent() with the specified
   //  timeout. Because we want to monitor the home, we replace this with a series
   //  of calls with a smaller timeout, effectively making this a poll loop, in which
   //  we can look for any sign that the home is in trouble.

   int ampct = TheLinkage->GetAmpCount();
   DEBUG("Amp count %d\n", ampct);
   CML::uunit AmpPositions[MAX_AMPS];
   bool Homed[MAX_AMPS];
   for (int i = 0; i < ampct; i++)
   {
      Homed[i] = false;
      CML::uunit value;
      TheLinkage->GetAmp(i).GetPositionActual(value);
      AmpPositions[i] = value;
      DEBUG("Amp %d, position %f\n", i, value);
   }
   const CML::Error *err;
   int pollTimeout = 500; // 0.5 sec.
   int timeWaited = 0;
   bool Halt = false;
   while (timeWaited < timeout || timeout == -1)
   {
      DEBUG("Waiting\n");
      err = TheLinkage->WaitEvent(e, pollTimeout, match);
      DEBUG("Wait event returns\n");
      if (err != &CML::ThreadError::Timeout)
         break;
      DEBUG("That was a timeout\n");
      int HomedCount = 0;
      for (int i = 0; i < ampct; i++)
      {
         if (Homed[i])
         {
            HomedCount++;
         }
         else
         {
            CML::EVENT_STATUS status;
            TheLinkage->GetAmp(i).GetEventStatus(status);
            if (!(status & CML::ESTAT_MOVING))
            {
               printf("Amp %d has finished its move\n", i);
               Homed[i] = true;
               HomedCount++;
            }
            else
            {
               CML::uunit value;
               TheLinkage->GetAmp(i).GetPositionActual(value);

               printf("Amp %d was %f now %f\n", i, AmpPositions[i], value);
               if (fabs(AmpPositions[i] - value) < MOVE_TOLERANCE)
               {
                  bool Stuck = true;
                  if (Targets != NULL)
                  {
                     if (fabs(AmpPositions[i] - Targets[i] < MOVE_TOLERANCE))
                     {
                        printf("Amp %d seems to be at target, but not yet finished its move\n", i);
                        Stuck = false;
                     }
                  }
                  if (Stuck)
                  {
                     printf("*** Amp %d seems to be stuck ***\n", i);
                     TheLinkage->HaltMove();
                     Halt = true;
                     err = &CML::AmpError::TrackErr;
                  }
                  break;
               }
               AmpPositions[i] = value;
            }
         }
      }
      DEBUG("Total number of home axis now: %d\n", HomedCount);
      if (HomedCount == ampct)
      {
         printf("All amps homed\n");
         break;
      }
      if (Halt)
         break;
      timeWaited += pollTimeout;
   }

   if (!err)
   {
      match = (CML::LINK_EVENT)(match & value);
      if (match == CML::LINKEVENT_MOVEDONE)
         return 0;

      // There should be a latched error
      int ndx;
      err = TheLinkage->GetLatchedError(ndx);

      // If not, take a best guess (the Linkage WaitMoveDone() call uses GetError()
      // but that's private so I can't access it. But a WaitMoveDone() call with a
      // zero timeout should be able to pick it up.

      if (!err)
         err = TheLinkage->WaitMoveDone(0);

      CML::cml.Debug("Link %d - Linkage::WaitMoveDone returned: %s\n",
                     TheLinkage->GetAmp(0).GetNodeID(), err->toString());
   }
   DEBUG("Timeout now: %d\n", timeWaited);
   return err;
}

//  ------------------------------------------------------------------------------------------------

//               L i n k a g e  K i c k e r  -  c l a s s   d e f i n i t i o n
//
//  A LinkageKicker is an experimental class that inherits from DRAMA's KickNotifier. The idea
//  is that this can be installed as a kick handler for any action that is moving axes using a
//  CML Linkage. When kicked, it needs to cancel the current movement. At the moment, this isn't
//  working - I suspect an interaction with KickNotifier and CML's use of SIGUSR2 signals, but
//  haven't confirmed that yet. (22/3/18).

class LinkageKicker : public drama::thread::KickNotifier
{
public:
   LinkageKicker(drama::thread::TAction *ThisThread) : drama::thread::KickNotifier(ThisThread)
   {
      I_LinkagePtr = NULL;
      // UnblockSIGUSR2();
   }
   void SetLinkage(CML::Linkage *LinkagePtr)
   {
      I_LinkagePtr = LinkagePtr;
   }
   bool Kicked(const drama::sds::Id & /*Arg*/)
   {
      DEBUG("Kicked\n");

      if (I_LinkagePtr)
      {
         /*
         int Ampct = I_LinkagePtr->GetAmpCount();
         DEBUG ("Disabling %d amps\n",Ampct);
         for (int I = 0; I < Ampct; I++) {
            I_LinkagePtr->GetAmp(I).Disable();
         }
         */
         DEBUG("Halting move\n");
         I_LinkagePtr->HaltMove();
      }

      return false;
   }

private:
   CML::Linkage *I_LinkagePtr;
};

//  ------------------------------------------------------------------------------------------------

//                    T d F  C a n  T a s k  : :  C o n s t r u c t o r
//
//  This is the only contrstuctor for the task. It sets up the various actions, and leaves the
//  rest to the DRAMA infrastructure.

TdFCanTask::TdFCanTask(const std::string &taskName) : drama::Task(taskName),
                                                      I_InitialiseActionObj(TaskPtr()),
                                                      I_GInitActionObj(TaskPtr()),
                                                      I_GMoveAxisActionObj(TaskPtr()),
                                                      I_GHomeActionObj(TaskPtr()),
                                                      I_CanAccessInitialised(false),
                                                      I_TdFCanTaskParSys(TaskPtr()),

                                                      I_GParkGantryActionNTObj(TaskPtr()),
                                                      I_GUnParkActionNTObj(TaskPtr()),
                                                      I_GHomeActionNTObj(TaskPtr()),
                                                      I_GMoveOffsetActionNTObj(TaskPtr()),
                                                      I_GMoveActionNTObj(TaskPtr()),
                                                      I_GMoveAxisActionNTObj(TaskPtr()),
                                                      I_GExitActionNTObj(TaskPtr()),
                                                      I_GResetActionNTObj(TaskPtr()),
                                                      I_GInitActionNTObj(TaskPtr()),

                                                      I_PTelposActionNTObj(TaskPtr()),
                                                      I_PSetActionNTObj(TaskPtr()),
                                                      I_PResetLockNTObj(TaskPtr()),
                                                      I_PSetCoeffsActionNTObj(TaskPtr()),
                                                      I_PSetImageActionNTObj(TaskPtr()),
                                                      I_PSetWindowActionNTObj(TaskPtr()),
                                                      I_PSetVelActionNTObj(TaskPtr()),
                                                      I_PSetPlateActionNTObj(TaskPtr()),
                                                      I_PUpdateFlexActionNTObj(TaskPtr()),
                                                      I_PUpdateActionNTObj(TaskPtr()),

                                                      I_PReportActionNTObj(TaskPtr()),
                                                      I_PReportLocksActionNTObj(TaskPtr()),
                                                      I_PReportCoeffsActionNTObj(TaskPtr()),
                                                      I_PReportImageActionNTObj(TaskPtr()),
                                                      I_PReportWindowActionNTObj(TaskPtr()),
                                                      I_PSaveDefsActionNTObj(TaskPtr()),
                                                      I_CSearchActionObj(TaskPtr()),
                                                      I_CCentroidActionObj(TaskPtr()),
                                                      I_CImageActionObj(TaskPtr()),
                                                      I_CZeroCamActionObj(TaskPtr()),
                                                      I_CShiftCoActionObj(TaskPtr()),
                                                      I_CSurveyActionObj(TaskPtr()),

                                                      tdfTaskStr(TaskPtr(), "ENQ_DEV_DESCR", "2dF Focal Plane Imager Gantry Task"),
                                                      tdfSimSearchRunStr(TaskPtr(), "SIM_SEARCH_RAN", "No"),
                                                      tdfVersionStr(TaskPtr(), "ENQ_VER_NUM", ""),
                                                      tdfVersionDate(TaskPtr(), "ENQ_VER_DATE", ""),
                                                      tdfCanMode(TaskPtr(), "MODE", "PROTECTED"),
                                                      tdfCanParked(TaskPtr(), "PARKED", "NO"),
                                                      tdfGantryLamp(TaskPtr(), "GANTRY_LAMPS", "OFF"),
                                                      copleyCodeVer(TaskPtr(), "COPLEY_CODE_VER", 1000),
                                                      dzero(TaskPtr(), "SURVEY_PROG", 0.0),
                                                      dzeroHA(TaskPtr(), "HA", 0.0),
                                                      dzeroDEC(TaskPtr(), "DEC", 0.0),
                                                      xPark(TaskPtr(), "X", 0),
                                                      yPark(TaskPtr(), "Y", 0),
                                                      xCopleyPark(TaskPtr(), "XCopley", 0),
                                                      yCopleyPark(TaskPtr(), "YCopley", 0),
                                                      plateTheta(TaskPtr(), "PLATE_THETA", 0.0),
                                                      xyVel(TaskPtr(), "XY_VEL", 150000),
                                                      xaccuracy(TaskPtr(), "X_ACCURACY", 5),
                                                      yaccuracy(TaskPtr(), "Y_ACCURACY", 5),
                                                      posTol(TaskPtr(), "POS_TOL", 5),
                                                      attempts(TaskPtr(), "POS_ATTEMPTS", 5),
                                                      fibInImgThres(TaskPtr(), "FIBRE_IN_IMAGE", FIBRE_IN_IMAGE_LIMIT),
                                                      settleTime(TaskPtr(), "SETTLE_TIME", 1.0),
                                                      maxError(TaskPtr(), "MAX_ERROR", 5000),
                                                      stepSize(TaskPtr(), "STEP_SIZE", 1000),
                                                      timeoutFac(TaskPtr(), "TIMEOUT_FAC", 10),
                                                      overlayPlaneEnabled(TaskPtr(), "VFG_OP_ENABLE", 1),
                                                      ushortZero(TaskPtr(), "TASK_STATE", 0),
                                                      ushortZeroCentroid(TaskPtr(), "DEBUG_CENTROID", 0),
                                                      zerocamCenWait(TaskPtr(), "ZEROCAM_CENWAIT", 1.0),

                                                      shortZeroX(TaskPtr(), "PLT1_CFID_OFF_X", 0),
                                                      shortZeroY(TaskPtr(), "PLT1_CFID_OFF_Y", 0),
                                                      I_X1Amp(NULL),
                                                      I_X2Amp(NULL),
                                                      I_YAmp(NULL),
                                                      I_ZAmp(NULL),
                                                      I_ThetaAmp(NULL),
                                                      I_JawAmp(NULL),
                                                      I_tdFfpiMainStruct(nullptr),
                                                      I_pCameraPath(TaskPtr(), "VimbaFPI", "", "/instsoft/vimbacam/vimbacam")
{
   //  cml is a global defined by the CML library. Logging everything is a very good idea,
   //  even for a production system, and it's essential during development. Note that the
   //  logs produced are timestamped in the same way as the CANOpen.log files produced by
   //  the simulator, and can be compared for diagnostic purposes.

   CML::cml.SetDebugLevel(CML::LOG_EVERYTHING);

   //  Set up the various actions. Note that 'INITIALISE' which initialises the task as a
   //  whole, is quite different to 'G_INIT' which initialises the 2dF gantry. (There
   //  will later be an 'F_INIT' which initialises the FPI gantry, and others.)

   Add("INITIALISE", drama::MessageHandlerPtr(&I_InitialiseActionObj, drama::nodel()));
   Add("G_INIT", drama::MessageHandlerPtr(&I_GInitActionObj, drama::nodel()));
   Add("G_MOVE_AXIS", drama::MessageHandlerPtr(&I_GMoveAxisActionObj, drama::nodel()));
   Add("G_MOVE_AXIS_NT", drama::MessageHandlerPtr(&I_GMoveAxisActionNTObj, drama::nodel()));
   Add("G_INIT_NT", drama::MessageHandlerPtr(&I_GInitActionNTObj, drama::nodel()));
   Add("G_HOME", drama::MessageHandlerPtr(&I_GHomeActionObj, drama::nodel()));
   Add("G_HOME_NT", drama::MessageHandlerPtr(&I_GHomeActionNTObj, drama::nodel()));

   // add new ParkGantry action
   Add("G_PARK_NT", drama::MessageHandlerPtr(&I_GParkGantryActionNTObj, drama::nodel()));
   // add new UnParkGantry action
   Add("G_UNPARK_NT", drama::MessageHandlerPtr(&I_GUnParkActionNTObj, drama::nodel()));
   // add new Move Gantry Offset action
   Add("G_MOVEOFFSET_NT", drama::MessageHandlerPtr(&I_GMoveOffsetActionNTObj, drama::nodel()));
   // add new Exit action, which disables all the amplifiers
   Add("G_EXIT", drama::MessageHandlerPtr(&I_GExitActionNTObj, drama::nodel()));
   // add new Move Axis action, which only moves a particular axis
   Add("G_MOVE_NT", drama::MessageHandlerPtr(&I_GMoveActionNTObj, drama::nodel()));
   // add new Reset action, which disables the amplifiers and setup them again.
   Add("G_RESET", drama::MessageHandlerPtr(&I_GResetActionNTObj, drama::nodel()));

   // add new TOLPOS action which updates the current recorded telescope orientation
   Add("P_TELPOS", drama::MessageHandlerPtr(&I_PTelposActionNTObj, drama::nodel()));
   // add new Set action which sets the FPI task parameters;
   Add("P_SET", drama::MessageHandlerPtr(&I_PSetActionNTObj, drama::nodel()));
   // add new Reset Lock action which resets the lock;
   Add("P_RESET_LOCK", drama::MessageHandlerPtr(&I_PResetLockNTObj, drama::nodel()));
   // add new Set Coeffs action which sets the coefficients;
   Add("P_SET_COEFFS", drama::MessageHandlerPtr(&I_PSetCoeffsActionNTObj, drama::nodel()));
   // add new Set Image action which sets the coefficients of camera;
   Add("P_SET_IMAGE", drama::MessageHandlerPtr(&I_PSetImageActionNTObj, drama::nodel()));
   // add new Set Window action which sets the search window during the centroid;
   Add("P_SET_WINDOW", drama::MessageHandlerPtr(&I_PSetWindowActionNTObj, drama::nodel()));
   // add new SetVel action which sets the velocity of gantry
   Add("P_SET_VEL", drama::MessageHandlerPtr(&I_PSetVelActionNTObj, drama::nodel()));
   // add new SetPlate action which sets the velocity of gantry
   Add("P_SET_PLATE", drama::MessageHandlerPtr(&I_PSetPlateActionNTObj, drama::nodel()));
   // add new UpdateFlex action which rereads flex file
   Add("P_UPDATE_FLEX", drama::MessageHandlerPtr(&I_PUpdateFlexActionNTObj, drama::nodel()));
   // add new Update action which updates the pose
   Add("P_UPDATE", drama::MessageHandlerPtr(&I_PUpdateActionNTObj, drama::nodel()));

   // add new Report action which reports the parameters of fpi task
   Add("P_REPORT", drama::MessageHandlerPtr(&I_PReportActionNTObj, drama::nodel()));
   // add new Report Lock action which reports the status of the lock
   Add("P_REPORT_LOCKS", drama::MessageHandlerPtr(&I_PReportLocksActionNTObj, drama::nodel()));
   // add new Report Coeffs action which reports the coefficents
   Add("P_REPORT_COEFFS", drama::MessageHandlerPtr(&I_PReportCoeffsActionNTObj, drama::nodel()));
   // add new Report Image action which reports the image
   Add("P_REPORT_IMAGE", drama::MessageHandlerPtr(&I_PReportImageActionNTObj, drama::nodel()));
   // add new Report Window action which reports the window
   Add("P_REPORT_WINDOW", drama::MessageHandlerPtr(&I_PReportWindowActionNTObj, drama::nodel()));
   // add new Save Defs action which saves the parameter into a file
   Add("P_SAVE_DEFS", drama::MessageHandlerPtr(&I_PSaveDefsActionNTObj, drama::nodel()));

   // add new Search action which looks for fibre-end or fiducial
   Add("C_SEARCH", drama::MessageHandlerPtr(&I_CSearchActionObj, drama::nodel()));
   Add("C_CENTROID", drama::MessageHandlerPtr(&I_CCentroidActionObj, drama::nodel()));
   Add("C_IMAGE", drama::MessageHandlerPtr(&I_CImageActionObj, drama::nodel()));
   Add("C_ZEROCAM", drama::MessageHandlerPtr(&I_CZeroCamActionObj, drama::nodel()));
   Add("C_SHIFT_COEFFS", drama::MessageHandlerPtr(&I_CShiftCoActionObj, drama::nodel()));
   Add("C_SURVEY", drama::MessageHandlerPtr(&I_CSurveyActionObj, drama::nodel()));

   Add("EXIT", &drama::SimpleExitAction);

   {
      I_TdFCanTaskParSys.Create("PMAC_LIM_X_POS", (INT32)(528000));
      I_TdFCanTaskParSys.Create("PMAC_LIM_X_NEG", (INT32)(-16300));
      I_TdFCanTaskParSys.Create("PMAC_LIM_Y_POS", (INT32)(537400));
      I_TdFCanTaskParSys.Create("PMAC_LIM_Y_NEG", (INT32)(-3400));
   }
}

//  ------------------------------------------------------------------------------------------------

//                    T d F  C a n  T a s k  : :  D e s t r u c t o r
//
//  The destructor for the task releases any resources.

TdFCanTask::~TdFCanTask()
{

   //  Most of the release of resources is taken care of by the destructor for the CanAccess
   //  object. This releases all the CML objects it has created. Note that it is important for
   //  SIGUSR2 to be unblocked at this point if running on OS X. (See UnblockSIGUSR2() comments.)

   //  The pointers to the amplifiers will point to non-existent objects once the CanAccess
   //  destructor runs, so it's good form to null them too.

   I_X1Amp = I_X2Amp = I_YAmp = I_ZAmp = I_ThetaAmp = I_JawAmp = NULL;

   if (I_tdFfpiMainStruct)
   {
      free(I_tdFfpiMainStruct);
      I_tdFfpiMainStruct = nullptr;
   }
}

bool TdFCanTask::tdFfpiDefWrite(short savingFiles, short check)
{
   string pathName = TDFPT_PARAM_DIR;
   if (!boost::filesystem::exists(pathName) || !boost::filesystem::is_directory(pathName))
   {
      I_ErrorString += "The folder directory does not exist. Creating a new directory now.\n";
      boost::filesystem::create_directory(pathName);
   }
   string fName;
   if (savingFiles & DEFS_FILE)
   {
      tdFfpiTaskType *details = tdFfpiGetMainStruct();
      drama::sds::Id defId;
      if (tdFfpiWriteFile(defId))
      {
         if (defId)
         {
            fName = pathName + "tdFfpiDefsSave.sds";
            defId.Write(fName);
            DEBUG("\nThe output parameter array is:\n");
            defId.List();
         }
         else
         {
            I_ErrorString += "The created sds structure is invalid.\n";
            return false;
         }
      }
      else
      {
         I_ErrorString += "Fail to create a sds structure.\n";
         return false;
      }
   }
   return true;
}

// TdF CanTask Read Default Parameter SDS File
bool TdFCanTask::tdFfpiDefRead(short loadingFiles, short check)
{
   string pathName = TDFPT_PARAM_DIR;
   string fName;

   if (!boost::filesystem::exists(pathName) || !boost::filesystem::is_directory(pathName))
   {
      I_ErrorString += "The folder directory is not correct.\n";
      DramaTHROW(TDFCANTASK__NO_ENV_VAR, "The folder directory is not correct.");
   }
   if (loadingFiles & DEFS_FILE)
   {
      fName = pathName + "tdFfpiDefs.sds";
      if (!boost::filesystem::exists(fName) || !boost::filesystem::is_regular_file(fName))
      {
         I_ErrorString += "tdFfpiDefs.sds doesn't exist.\n";
         DramaTHROW(TDFCANTASK__NO_ENV_VAR, "tdFfpiDefs.sds doesn't exist.");
      }
      // This is the drama version
      // SdsRead(fName.c_str(), &defId, status);
      drama::sds::Id defId(drama::sds::Id::FromFile(fName));
      bool readRes = tdFfpiReadFile(defId);
      if (!readRes)
      {
         I_ErrorString += "Failed to read tdFfpiDefs.sds\n";
         DEBUG("\nREAD tdFfpiDefs.sds: %s", I_ErrorString.c_str());
         DramaTHROW(TDFCANTASK__READ_ERROR, "Failed to read tdFfpiDefs.sds.");
      }
      if (check & SHOW)
      {
         DEBUG("\nREAD tdFfpiDefs.sds: %s", fName.c_str());
         DEBUG("\nThe parameter array is:\n");
         defId.List();
      }
   }
   if (loadingFiles & FLEX_FILE)
   {
      fName = pathName + "tdFfpiFlex.sds";
      if (!boost::filesystem::exists(fName) || !boost::filesystem::is_regular_file(fName))
      {
         I_ErrorString += "tdFfpiFlex.sds doesn't exist.\n";
         DramaTHROW(TDFCANTASK__NO_ENV_VAR, "tdFfpiFlex.sds doesn't exist.");
      }
      drama::sds::Id defId(drama::sds::Id::FromFile(fName));
      bool readRes = tdFfpiReadFlexFile(defId);
      if (!readRes)
      {
         I_ErrorString += "Failed to read tdFfpiFlex.sds\n";
         DEBUG("READ tdFfpiFlex.sds: %s", I_ErrorString.c_str());
         DramaTHROW(TDFCANTASK__READ_ERROR, "Failed to read tdFfpiFlex.sds.");
      }
      if (check & SHOW)
      {
         DEBUG("\nREAD tdFfpiFlex.sds: %s", fName.c_str());
         DEBUG("\nThe parameter array is :\n");
         defId.List();
      }
   }

   return true;
}

bool TdFCanTask::tdFfpiWriteFile(drama::sds::Id &defId)
{
   tdFfpiTaskType *details = tdFfpiGetMainStruct();
   drama::sds::Id tmpId, tmp2Id;
   double dParam;
   unsigned long int dimVal = 6;
   long int lIntParam;
   short shortParam;
   std::vector<unsigned long int> dims;
   dims.push_back(dimVal);

   if (details == nullptr)
   {
      DramaTHROW(TDFCANTASK__MALLOCERR, "Malloc error.");
   }
   defId = drama::sds::Id::CreateTopLevel(
       "tdFfpiDefs", SDS_STRUCT);
   tmpId = defId.CreateChildItem("freeImage", SDS_STRUCT);
   tmpId.Put("bias", (short)details->freeImg.bias);
   tmp2Id = tmpId.CreateChildArray("camCoeffs", SDS_DOUBLE, dims);

   drama::sds::ArrayWriteHelper<double> arrayHelper;
   tmp2Id.ArrayAccess(&arrayHelper);

   for (int index = 0; index < (int)dimVal; index++)
   {
      arrayHelper[index] = details->freeImg.camCoeffs[index];
   }

   tmpId = defId.CreateChildItem("normWindow", SDS_STRUCT);
   tmpId.Put("xCen", (double)details->normWin.xCen);
   tmpId.Put("yCen", (double)details->normWin.yCen);
   tmpId.Put("xSpan", (short)details->normWin.xSpan);
   tmpId.Put("ySpan", (short)details->normWin.ySpan);

   tmpId = defId.CreateChildItem("searchWindow", SDS_STRUCT);
   tmpId.Put("xCen", (double)details->searchWin.xCen);
   tmpId.Put("yCen", (double)details->searchWin.yCen);
   tmpId.Put("xSpan", (short)details->searchWin.xSpan);
   tmpId.Put("ySpan", (short)details->searchWin.ySpan);

   tmpId = defId.CreateChildItem("conversion", SDS_STRUCT);
   tmp2Id = tmpId.CreateChildArray("coeffs", SDS_DOUBLE, dims);
   drama::sds::ArrayWriteHelper<double> arrayHelperCoeffs;
   tmp2Id.ArrayAccess(&arrayHelperCoeffs);
   for (int index = 0; index < (int)dimVal; index++)
   {
      arrayHelperCoeffs[index] = details->convert.coeffs[index];
   }

   tmpId = defId.CreateChildItem("parameters", SDS_STRUCT);
   lIntParam = (long int)xyVel;
   lIntParam = I_TdFCanTaskParSys.GetInt("XY_VEL");
   tmpId.Put("XY_VEL", (INT32)lIntParam);

   lIntParam = (long int)stepSize;
   lIntParam = I_TdFCanTaskParSys.GetInt("STEP_SIZE");
   tmpId.Put("STEP_SIZE", (INT32)lIntParam);

   lIntParam = (long int)maxError;
   lIntParam = I_TdFCanTaskParSys.GetInt("MAX_ERROR");
   tmpId.Put("MAX_ERROR", (INT32)lIntParam);

   lIntParam = (long int)posTol;
   lIntParam = I_TdFCanTaskParSys.GetInt("POS_TOL");
   tmpId.Put("POS_TOL", (INT32)lIntParam);

   lIntParam = (long int)attempts;
   lIntParam = I_TdFCanTaskParSys.GetInt("POS_ATTEMPTS");
   tmpId.Put("POS_ATTEMPTS", (INT32)lIntParam);

   lIntParam = (long int)fibInImgThres;
   lIntParam = I_TdFCanTaskParSys.GetInt("FIBRE_IN_IMAGE");
   tmpId.Put("FIBRE_IN_IMAGE", (INT32)lIntParam);

   dParam = (double)settleTime;
   dParam = I_TdFCanTaskParSys.GetDouble("SETTLE_TIME");
   tmpId.Put("SETTLE_TIME", dParam);

   lIntParam = (long int)timeoutFac;
   lIntParam = I_TdFCanTaskParSys.GetInt("TIMEOUT_FAC");
   tmpId.Put("TIMEOUT_FAC", (INT32)lIntParam);

   lIntParam = (long int)xaccuracy;
   lIntParam = I_TdFCanTaskParSys.GetInt("X_ACCURACY");
   tmpId.Put("X_ACCURACY", (INT32)lIntParam);

   lIntParam = (long int)yaccuracy;
   lIntParam = I_TdFCanTaskParSys.GetInt("Y_ACCURACY");
   tmpId.Put("Y_ACCURACY", (INT32)lIntParam);

   if (details->dprFeedback)
   {
      tmpId.Put("DPR_FEEDBACK", "YES");
   }
   else
   {
      tmpId.Put("DPR_FEEDBACK", "NO");
   }

   lIntParam = (long int)overlayPlaneEnabled;
   lIntParam = I_TdFCanTaskParSys.GetInt("VFG_OP_ENABLE");
   tmpId.Put("VFG_OP_ENABLE", (INT32)lIntParam);

   dParam = (double)zerocamCenWait;
   dParam = I_TdFCanTaskParSys.GetDouble("ZEROCAM_CENWAIT");
   tmpId.Put("ZEROCAM_CENWAIT", dParam);

   // lIntParam = (long int)copleyXPosLim;
   lIntParam = I_TdFCanTaskParSys.GetInt("PMAC_LIM_X_POS");
   tmpId.Put("PMAC_LIM_X_POS", (INT32)lIntParam);

   // lIntParam = (long int)copleyXNegLim;
   lIntParam = I_TdFCanTaskParSys.GetInt("PMAC_LIM_X_NEG");
   tmpId.Put("PMAC_LIM_X_NEG", (INT32)lIntParam);

   // lIntParam = (long int)copleyYPosLim;
   lIntParam = I_TdFCanTaskParSys.GetInt("PMAC_LIM_Y_POS");
   tmpId.Put("PMAC_LIM_Y_POS", (INT32)lIntParam);

   // lIntParam = (long int)copleyYNegLim;
   lIntParam = I_TdFCanTaskParSys.GetInt("PMAC_LIM_Y_NEG");
   tmpId.Put("PMAC_LIM_Y_NEG", (INT32)lIntParam);

   lIntParam = (long int)shortZeroX;
   lIntParam = I_TdFCanTaskParSys.GetInt("PLT1_CFID_OFF_X");
   tmpId.Put("PLT1_CFID_OFF_X", (short)lIntParam);

   lIntParam = (long int)shortZeroY;
   lIntParam = I_TdFCanTaskParSys.GetInt("PLT1_CFID_OFF_Y");
   tmpId.Put("PLT1_CFID_OFF_Y", (short)lIntParam);
   return true;
}

bool TdFCanTask::tdFfpiReadFile(drama::sds::Id &defId)
{
   tdFfpiTaskType *details = tdFfpiGetMainStruct();
   // this is the drama version
   // SdsIdType tmpId, tmp2Id;

   drama::sds::Id tmpId, tmp2Id;
   double dParam = 0.0;
   unsigned long actlen = 0;
   long int lIntParam = 0;
   int j = 0;
   short sParam;
   string strParam;

   if (!defId)
   {
      DEBUG("The sds::Id is invalid");
      I_ErrorString += "Failed to read the tdFfpiDef.sds.\n";
      DramaTHROW(TDFCANTASK__READ_ERROR, "Failed to read the tdFfpiDef.sds.");
      // return false;
   }

   /*
    *  Set camera constant parameters.
    */
   details->freeImg.shutter = GCAM_OPEN;
   details->freeImg.updateTime = 0.8;
   details->freeImg.exposureTime = 0.457;
   details->freeImg.camNo = 1;
   details->freeImg.xMax = CAM_X_SPAN;
   details->freeImg.yMax = CAM_Y_SPAN;
   details->freeImg.PixelSize = PIXEL_SIZE;

   /*
    *  Read free-focus image details.
    */
   // this is the drama version
   // SdsFind(defId, "freeImage", &tmpId, status);
   tmpId = defId.Find("freeImage");
   if (tmpId)
   {
      // this is the drama version
      // ArgGets(tmpId, "bias", &sParam, status);
      tmpId.Get("bias", &sParam);
      DEBUG("The bias of freeImage is %d\n", (int)sParam);
      details->freeImg.bias = (int)sParam;

      // this is the drama version
      // SdsFind(tmpId, "camCoeffs", &tmp2Id, status);
      // SdsGet(tmp2Id, 6 * sizeof(double), 0, details->freeImg.camCoeffs, &actlen, status);
      // SdsFreeId(tmp2Id, status);
      // SdsFreeId(tmpId, status);

      tmp2Id = tmpId.Find("camCoeffs");
      if (tmp2Id)
      {
         tmp2Id.Get(6 * sizeof(double), details->freeImg.camCoeffs, &actlen, 0);
         if (actlen != 6 * sizeof(double)) // this is not correct actlen returns bytes values
         {
            I_ErrorString += "The field of camCoeffs is invalid.\n";
            DramaTHROW(TDFCANTASK__READ_ERROR, "The field of camCoeffs is invalid.");
            // return false;
         }
         DEBUG("The FreeImage camCoeffs array is:\n");
         for (int index = 0; index < 6; index++)
         {
            DEBUG(" %f ", details->freeImg.camCoeffs[index]);
         }
         DEBUG("\n");
      }
      else
      {
         I_ErrorString += "Cannot find the field of camCoeffs.\n";
         DramaTHROW(TDFCANTASK__READ_ERROR, "Cannot find the field of camCoeffs.");
         // return false;
      }
   }
   else
   {
      I_ErrorString += "Cannot find the field of freeImage.\n";
      DramaTHROW(TDFCANTASK__READ_ERROR, "Cannot find the field of freeImage.");
      // return false;
   }

   slaInvf(details->freeImg.camCoeffs, details->freeImg.invCoeffs, &j);
   if (j != 0)
   {
      register int i;
      for (i = 0; i < 6; i++)
         details->freeImg.invCoeffs[i] = details->freeImg.camCoeffs[i];
   }

   DEBUG("The FreeImage invCoeffs array is:\n");
   for (int index = 0; index < 6; index++)
   {
      DEBUG(" %f ", details->freeImg.invCoeffs[index]);
   }
   DEBUG("\n");
   /*
    *  Read the normal wimdow definitions.
    */
   // this is the drama version
   // SdsFind(defId, "normWindow", &tmpId, status);
   tmpId = defId.Find("normWindow");
   if (tmpId)
   {
      // ArgGetd(tmpId, "xCen", &dParam, status);
      tmpId.Get("xCen", &dParam);
      DEBUG("xCen from file is %f\n", dParam);
      details->normWin.xCen = dParam;
      // DEBUG("normWin.xCen is %f\n", details->normWin.xCen);

      // ArgGetd(tmpId, "yCen", &dParam, status);
      tmpId.Get("yCen", &dParam);
      DEBUG("yCen from file is %f\n", dParam);
      details->normWin.yCen = dParam;
      // DEBUG("normWin.yCen is %f\n", details->normWin.yCen);

      // ArgGets(tmpId, "xSpan", &sParam, status);
      tmpId.Get("xSpan", &sParam);
      DEBUG("xSpan from file is %hd\n", sParam);
      details->normWin.xSpan = (int)sParam;
      // DEBUG("normWin.xSpan is %d\n", details->normWin.xSpan);

      // ArgGets(tmpId, "ySpan", &sParam, status);
      tmpId.Get("ySpan", &sParam);
      DEBUG("ySpan from file is %hd\n", sParam);
      details->normWin.ySpan = (int)sParam;
      // DEBUG("normWin.ySpan is %d\n", details->normWin.ySpan);
      //  SdsFreeId(tmpId, status);
   }

   /*
    *  Read search window details.
    */
   // this is the drama version
   // SdsFind(defId, "searchWindow", &tmpId, status);
   tmpId = defId.Find("searchWindow");
   if (tmpId)
   {
      // ArgGetd(tmpId, "xCen", &dParam, status);
      tmpId.Get("xCen", &dParam);
      details->searchWin.xCen = dParam;

      // ArgGetd(tmpId, "yCen", &dParam, status);
      tmpId.Get("yCen", &dParam);
      details->searchWin.yCen = dParam;

      // ArgGets(tmpId, "xSpan", &sParam, status);
      tmpId.Get("xSpan", &sParam);
      details->searchWin.xSpan = (int)sParam;

      // ArgGets(tmpId, "ySpan", &sParam, status);
      tmpId.Get("ySpan", &sParam);
      details->searchWin.ySpan = (int)sParam;
      // SdsFreeId(tmpId, status);
   }

   /*
    *  Get parameters used to convert between field-plate and encoder units.
    */
   // this is the drama version
   // SdsFind(defId, "conversion", &tmpId, status);
   tmpId = defId.Find("conversion");
   if (tmpId)
   {
      // SdsFind(tmpId, "coeffs", &tmp2Id, status);
      tmp2Id = tmpId.Find("coeffs");
      if (tmp2Id)
      {
         // SdsGet(tmp2Id, 6 * sizeof(double), 0, details->convert.coeffs, &actlen, status);
         tmp2Id.Get(6 * sizeof(double), details->convert.coeffs, &actlen, 0);
         if (actlen != 6 * sizeof(double)) //
         {
            I_ErrorString += "The field of conversion coeffs is invalid.\n";
            DramaTHROW(TDFCANTASK__READ_ERROR, "The field of conversion coeffs is invalid.");
         }
         // SdsFreeId(tmpId, status);
         // SdsFreeId(tmp2Id, status);
         DEBUG("The SearchWindow conversion array is:\n");
         for (int index = 0; index < 6; index++)
         {
            DEBUG(" %f ", details->convert.coeffs[index]);
         }
         DEBUG("\n");
      }
      else
      {
         I_ErrorString += "Cannot find the field of coeffs.\n";
         DramaTHROW(TDFCANTASK__READ_ERROR, "Cannot find the field of coeffs.");
      }
   }
   else
   {
      I_ErrorString += "Cannot find the field of conversion.\n";
      DramaTHROW(TDFCANTASK__READ_ERROR, "Cannot find the field of conversion.");
   }

   // if (*status != STATUS__OK)
   //    return;

   slaInvf(details->convert.coeffs, details->convert.invCoeffs, &j);
   if (j != 0)
   {
      int i;
      for (i = 0; i < 6; i++)
         details->convert.invCoeffs[i] = details->convert.coeffs[i];
   }
   DEBUG("The SearchWindow invconversion array is:\n");
   for (int index = 0; index < 6; index++)
   {
      DEBUG(" %f ", details->convert.invCoeffs[index]);
   }
   DEBUG("\n");
   /*
    *  Set default parameter values.
    */
   // this is the drama version
   // SdsFind(defId, "parameters", &tmpId, status);
   tmpId = defId.Find("parameters");
   if (tmpId)
   {
      // this is the drama version
      // ArgGeti(tmpId, "XY_VEL", &lIntParam, status);
      // if (*status == STATUS__OK)
      //    SdpPuti("XY_VEL", lIntParam, status);
      // else
      // {
      //    *status = STATUS__OK;
      // }
      tmpId.Get("XY_VEL", &lIntParam);
      if (lIntParam)
      {
         // I_TdFCanTaskParSys.Put("XY_VEL", lIntParam);
         DEBUG("XY_VEL from file is %ld\n", lIntParam);
         // xyVel = static_cast<INT32>(lIntParam);
         // DEBUG("xyVel original is %d\n", this->xyVel);
         I_TdFCanTaskParSys.Put("XY_VEL", (INT32)lIntParam);
         // DEBUG("xyVel after altering is %d\n", this->xyVel);
         lIntParam = 0;
      }

      // ArgGeti(tmpId, "STEP_SIZE", &lIntParam, status);
      // if (*status == STATUS__OK)
      //    SdpPuti("STEP_SIZE", lIntParam, status);
      // else
      // {
      //    *status = STATUS__OK;
      // }
      tmpId.Get("STEP_SIZE", &lIntParam);
      if (lIntParam)
      {
         // I_TdFCanTaskParSys.Put("STEP_SIZE", lIntParam);
         // DEBUG("stepSize before setting is %d\n", stepSize);
         stepSize = lIntParam;
         // DEBUG("stepSize after setting directly is %d\n", stepSize);
         DEBUG("STEP_SIZE from file is %ld\n", lIntParam);
         I_TdFCanTaskParSys.Put("STEP_SIZE", (INT32)lIntParam);
         // DEBUG("stepSize after setting using Parsys is %d\n", stepSize);
         lIntParam = 0;
      }

      // ArgGeti(tmpId, "MAX_ERROR", &lIntParam, status);
      // if (*status == STATUS__OK)
      //    SdpPuti("MAX_ERROR", lIntParam, status);
      // else
      // {
      //    *status = STATUS__OK;
      // }
      tmpId.Get("MAX_ERROR", &lIntParam);
      if (lIntParam)
      {
         // I_TdFCanTaskParSys.Put("MAX_ERROR", lIntParam);
         maxError = lIntParam;
         DEBUG("MAX_ERROR from file is %ld\n", lIntParam);
         I_TdFCanTaskParSys.Put("MAX_ERROR", (INT32)lIntParam);
         // DEBUG("maxError is %d\n", maxError);
         lIntParam = 0;
      }

      // ArgGeti(tmpId, "POS_TOL", &lIntParam, status);
      // if (*status == STATUS__OK)
      //    SdpPuti("POS_TOL", lIntParam, status);
      // else
      // {
      //    *status = STATUS__OK;
      // }
      tmpId.Get("POS_TOL", &lIntParam);
      if (lIntParam)
      {
         // I_TdFCanTaskParSys.Put("POS_TOL", lIntParam);
         posTol = lIntParam;
         DEBUG("POS_TOL from file is %ld\n", lIntParam);
         I_TdFCanTaskParSys.Put("POS_TOL", (INT32)lIntParam);
         // DEBUG("posTol is %d\n", posTol);
         lIntParam = 0;
      }

      // ArgGets(tmpId, "POS_ATTEMPTS", &sParam, status);
      // if (*status == STATUS__OK)
      //    SdpPuts("POS_ATTEMPTS", sParam, status);
      // else
      // {
      //    *status = STATUS__OK;
      // }
      tmpId.Get("POS_ATTEMPTS", &sParam);
      if (sParam)
      {
         // I_TdFCanTaskParSys.Put("POS_ATTEMPTS", sParam);
         attempts = sParam;
         DEBUG("POS_ATTEMPTS from file is %hd\n", sParam);
         I_TdFCanTaskParSys.Put("POS_ATTEMPTS", (short)sParam);
         // DEBUG("attempts is %hd\n", attempts);
         sParam = 0;
      }

      // ArgGeti(tmpId, "FIBRE_IN_IMAGE", &lIntParam, status);
      // if (*status == STATUS__OK)
      //    SdpPuti("FIBRE_IN_IMAGE", lIntParam, status);
      // else
      // {
      //    *status = STATUS__OK;
      // }
      tmpId.Get("FIBRE_IN_IMAGE", &lIntParam);
      if (lIntParam)
      {

         // I_TdFCanTaskParSys.Put("FIBRE_IN_IMAGE", lIntParam);
         fibInImgThres = lIntParam;
         DEBUG("FIBRE_IN_IMAGE from file is %ld\n", lIntParam);
         I_TdFCanTaskParSys.Put("POS_ATTEMPTS", (INT32)lIntParam);
         // DEBUG("fibInImgThres is %d\n", fibInImgThres);
         lIntParam = 0;
      }

      // ArgGetd(tmpId, "SETTLE_TIME", &dParam, status);
      // if (*status == STATUS__OK)
      //    SdpPutd("SETTLE_TIME", dParam, status);
      // else
      // {
      //    *status = STATUS__OK;
      // }
      tmpId.Get("SETTLE_TIME", &dParam);
      if (dParam)
      {
         // I_TdFCanTaskParSys.Put("SETTLE_TIME", dParam);
         settleTime = dParam;
         DEBUG("SETTLE_TIME from file is %lf\n", dParam);
         I_TdFCanTaskParSys.Put("SETTLE_TIME", (double)dParam);
         // DEBUG("settleTime is %f\n", settleTime);
         dParam = 0.0;
      }

      // ArgGeti(tmpId, "X_ACCURACY", &lIntParam, status);
      // if (*status == STATUS__OK)
      //    SdpPuti("X_ACCURACY", lIntParam, status);
      // else
      // {
      //    *status = STATUS__OK;
      // }
      tmpId.Get("X_ACCURACY", &lIntParam);
      if (lIntParam)
      {
         // I_TdFCanTaskParSys.Put("X_ACCURACY", lIntParam);
         xaccuracy = lIntParam;
         DEBUG("X_ACCURACY from file is %ld\n", lIntParam);
         I_TdFCanTaskParSys.Put("X_ACCURACY", (INT32)lIntParam);
         // DEBUG("xaccuracy is %d\n", xaccuracy);
         lIntParam = 0;
      }

      // ArgGeti(tmpId, "Y_ACCURACY", &lIntParam, status);
      // if (*status == STATUS__OK)
      //    SdpPuti("Y_ACCURACY", lIntParam, status);
      // else
      // {
      //    *status = STATUS__OK;
      // }
      tmpId.Get("Y_ACCURACY", &lIntParam);
      if (lIntParam)
      {
         // I_TdFCanTaskParSys.Put("Y_ACCURACY", lIntParam);
         yaccuracy = lIntParam;
         DEBUG("Y_ACCURACY from file is %ld\n", lIntParam);
         I_TdFCanTaskParSys.Put("Y_ACCURACY", (INT32)lIntParam);
         // DEBUG("yaccuracy is %d\n", yaccuracy);
         lIntParam = 0;
      }

      // ArgGeti(tmpId, "TIMEOUT_FAC", &lIntParam, status);
      // if (*status == STATUS__OK)
      //    SdpPuti("TIMEOUT_FAC", lIntParam, status);
      // else
      // {
      //    *status = STATUS__OK;
      // }
      tmpId.Get("TIMEOUT_FAC", &lIntParam);
      if (lIntParam)
      {
         // I_TdFCanTaskParSys.Put("TIMEOUT_FAC", lIntParam);
         timeoutFac = lIntParam;
         DEBUG("TIMEOUT_FAC from file is %ld\n", lIntParam);
         I_TdFCanTaskParSys.Put("TIMEOUT_FAC", (INT32)lIntParam);
         // DEBUG("timeoutFac is %d\n", timeoutFac);
         lIntParam = 0;
      }

      // ArgGetString(tmpId, "DPR_FEEDBACK", sizeof(strParam), strParam, status);
      // if (*status == STATUS__OK)
      // {
      //    details->dprFeedback = strcmp("NO", strParam) ? YES : NO;
      // }
      // else
      // {
      //    *status = STATUS__OK;
      // }
      tmpId.Get("DPR_FEEDBACK", &strParam);
      if (strParam.empty() == false)
      {
         DEBUG("DPR_FEEDBACK from file is %s\n", strParam.c_str());
         details->dprFeedback = strcmp("NO", strParam.c_str()) ? YES : NO;
         // DEBUG("details->dprFeedback is %hd\n", details->dprFeedback);
         strParam = "";
      }

      // ArgGeti(tmpId, "VFG_OP_ENABLE", &lIntParam, status);
      // if (*status == STATUS__OK)
      //    SdpPuti("VFG_OP_ENABLE", lIntParam, status);
      // else
      // {
      //    *status = STATUS__OK;
      // }
      tmpId.Get("VFG_OP_ENABLE", &lIntParam);
      if (lIntParam)
      {
         // I_TdFCanTaskParSys.Put("VFG_OP_ENABLE", lIntParam);
         overlayPlaneEnabled = lIntParam;
         DEBUG("VFG_OP_ENABLE from file is %ld\n", lIntParam);
         I_TdFCanTaskParSys.Put("VFG_OP_ENABLE", (INT32)lIntParam);
         // DEBUG("overlayPlaneEnabled is %d\n", overlayPlaneEnabled);
         lIntParam = 0;
      }

      // ArgGetd(tmpId, "ZEROCAM_CENWAIT", &dParam, status);
      // if (*status == STATUS__OK)
      //    SdpPutd("ZEROCAM_CENWAIT", dParam, status);
      // else
      // {
      //    *status = STATUS__OK;
      // }
      tmpId.Get("ZEROCAM_CENWAIT", &dParam);
      if (dParam)
      {
         // I_TdFCanTaskParSys.Put("ZEROCAM_CENWAIT", dParam);
         zerocamCenWait = dParam;
         DEBUG("ZEROCAM_CENWAIT from file is %f\n", dParam);
         I_TdFCanTaskParSys.Put("ZEROCAM_CENWAIT", (double)dParam);
         // DEBUG("zerocamCenWait is %f\n", zerocamCenWait);
         dParam = 0.0;
      }

      // ArgGeti(tmpId, "PMAC_LIM_X_POS", &lIntParam, status);
      // if (*status == STATUS__OK)
      //    SdpPuti("PMAC_LIM_X_POS", lIntParam, status);
      // else
      // {
      //    *status = STATUS__OK;
      // }
      tmpId.Get("PMAC_LIM_X_POS", &lIntParam);
      if (lIntParam)
      {
         // I_TdFCanTaskParSys.Put("PMAC_LIM_X_POS", lIntParam);
         // copleyXPosLim = lIntParam;
         DEBUG("PMAC_LIM_X_POS from file is %ld\n", lIntParam);
         I_TdFCanTaskParSys.Put("PMAC_LIM_X_POS", (INT32)lIntParam);
         // DEBUG("copleyXPosLim is %d\n", copleyXPosLim);
         lIntParam = 0;
      }

      // ArgGeti(tmpId, "PMAC_LIM_X_NEG", &lIntParam, status);
      // if (*status == STATUS__OK)
      //    SdpPuti("PMAC_LIM_X_NEG", lIntParam, status);
      // else
      // {
      //    *status = STATUS__OK;
      // }
      tmpId.Get("PMAC_LIM_X_NEG", &lIntParam);
      if (lIntParam)
      {
         // I_TdFCanTaskParSys.Put("PMAC_LIM_X_NEG", lIntParam);
         // copleyXNegLim = lIntParam;
         DEBUG("PMAC_LIM_X_NEG from file is %ld\n", lIntParam);
         I_TdFCanTaskParSys.Put("PMAC_LIM_X_NEG", (INT32)lIntParam);
         // DEBUG("copleyXNegLim is %d\n", copleyXNegLim);
         lIntParam = 0;
      }

      // ArgGeti(tmpId, "PMAC_LIM_Y_POS", &lIntParam, status);
      // if (*status == STATUS__OK)
      //    SdpPuti("PMAC_LIM_Y_POS", lIntParam, status);
      // else
      // {
      //    *status = STATUS__OK;
      // }
      tmpId.Get("PMAC_LIM_Y_POS", &lIntParam);
      if (lIntParam)
      {
         // I_TdFCanTaskParSys.Put("PMAC_LIM_Y_POS", lIntParam);
         // copleyYPosLim = lIntParam;
         // DEBUG("copleyYPosLim is %d\n", copleyYPosLim);
         DEBUG("PMAC_LIM_Y_POS from file is %ld\n", lIntParam);
         I_TdFCanTaskParSys.Put("PMAC_LIM_Y_POS", (INT32)lIntParam);
         lIntParam = 0;
      }

      // ArgGeti(tmpId, "PMAC_LIM_Y_NEG", &lIntParam, status);
      // if (*status == STATUS__OK)
      //    SdpPuti("PMAC_LIM_Y_NEG", lIntParam, status);
      // else
      // {
      //    *status = STATUS__OK;
      // }
      tmpId.Get("PMAC_LIM_Y_NEG", &lIntParam);
      if (lIntParam)
      {
         // I_TdFCanTaskParSys.Put("PMAC_LIM_Y_NEG", lIntParam);
         // copleyYNegLim = lIntParam;
         // DEBUG("copleyYNegLim is %d\n", copleyYNegLim);
         DEBUG("PMAC_LIM_Y_NEG from file is %ld\n", lIntParam);
         I_TdFCanTaskParSys.Put("PMAC_LIM_Y_NEG", (INT32)lIntParam);
         lIntParam = 0;
      }

      // ArgGeti(tmpId, "PLT1_CFID_OFF_X", &lIntParam, status);
      // if (*status == STATUS__OK)
      //    SdpPuti("PLT1_CFID_OFF_X", lIntParam, status);
      // else
      // {
      //    *status = STATUS__OK;
      // }
      tmpId.Get("PLT1_CFID_OFF_X", &lIntParam);
      if (lIntParam)
      {
         // I_TdFCanTaskParSys.Put("PLT1_CFID_OFF_X", lIntParam);
         shortZeroX = lIntParam;
         DEBUG("PLT1_CFID_OFF_X from file is %ld\n", lIntParam);
         I_TdFCanTaskParSys.Put("PLT1_CFID_OFF_X", (short)lIntParam);
         // DEBUG("shortZeroX is %hd\n", shortZeroX);
         lIntParam = 0;
      }

      // ArgGeti(tmpId, "PLT1_CFID_OFF_Y", &lIntParam, status);
      // if (*status == STATUS__OK)
      //    SdpPuti("PLT1_CFID_OFF_Y", lIntParam, status);
      // else
      // {
      //    *status = STATUS__OK;
      // }
      tmpId.Get("PLT1_CFID_OFF_Y", &lIntParam);
      if (lIntParam)
      {
         // I_TdFCanTaskParSys.Put("PLT1_CFID_OFF_Y", lIntParam);
         shortZeroY = lIntParam;
         DEBUG("PLT1_CFID_OFF_Y from file is %ld\n", lIntParam);
         I_TdFCanTaskParSys.Put("PLT1_CFID_OFF_Y", (short)lIntParam);
         // DEBUG("shortZeroY is %hd\n", shortZeroY);
         lIntParam = 0;
      }

      // this is the drama version
      // SdsFreeId(tmpId, status);
   }
   /*
    *  Free default file sds id.
    */
   // this is the drama version
   // SdsReadFree(defId, status);
   // SdsFreeId(defId, status);
   return true;
}

bool TdFCanTask::tdFfpiReadFlexFile(drama::sds::Id &defId)
{
   if (!defId)
   {
      DEBUG("The sds::Id is invalid");
      I_ErrorString += "Failed to read the tdFfpiDef.sds.\n";
      DramaTHROW(TDFCANTASK__READ_ERROR, "Failed to read the tdFfpiFlex.sds.");
   }

   drama::sds::Id id;
   unsigned long int actlen;
   id = defId.Find("offset");
   if (id)
   {
      id.Get(sizeof(double), &I_tdFfpiMainStruct->convert.flex.offset, &actlen, 0);
      if (actlen != sizeof(double))
      {
         I_ErrorString += "The field of offset is invalid.\n";
         DramaTHROW(TDFCANTASK__READ_ERROR, "The field of offset is invalid.");
      }
      actlen = 0;
   }
   else
   {
      I_ErrorString += "Cannot find the field of offset.\n";
      DramaTHROW(TDFCANTASK__READ_ERROR, "Cannot find the field of offset.");
   }
   id = defId.Find("length");
   if (id)
   {
      id.Get(sizeof(double), &I_tdFfpiMainStruct->convert.flex.length, &actlen, 0);
      if (actlen != sizeof(double))
      {
         I_ErrorString += "The field of length is invalid.\n";
         DramaTHROW(TDFCANTASK__READ_ERROR, "The field of length is invalid.");
      }
      actlen = 0;
   }
   else
   {
      I_ErrorString += "Cannot find the field of length.\n";
      DramaTHROW(TDFCANTASK__READ_ERROR, "Cannot find the field of length.");
   }
   id = defId.Find("k");
   if (id)
   {
      id.Get(sizeof(double), &I_tdFfpiMainStruct->convert.flex.k, &actlen, 0);
      if (actlen != sizeof(double))
      {
         I_ErrorString += "The field of length is k.\n";
         DramaTHROW(TDFCANTASK__READ_ERROR, "The field of k is invalid.");
      }
      actlen = 0;
   }
   else
   {
      I_ErrorString += "Cannot find the field of k.\n";
      DramaTHROW(TDFCANTASK__READ_ERROR, "Cannot find the field of k.");
   }

   return true;
}

bool TdFCanTask::tdFfpiIlocks(long int ilocks)
{

   if (ilocks & ILOCK__TASK_INIT)
   {
      if (I_tdFfpiMainStruct->Initialised == NO)
      {
         I_ErrorString += "The task has not been initialised.\n";
         DEBUG("The task has not been initialised.\n");
         return false;
      }
   }

   if (ilocks & ILOCK__GAN_PARKED)
   {
      if (I_TdFCanTaskParSys.GetString("PARKED") == "YES")
      {
         I_ErrorString += "The gantry has parked.\n";
         DEBUG("The gantry has parked.\n");
         return false;
      }
   }

   if (ilocks & ILOCK__GAN_NOT_PARKED)
   {
      if (I_TdFCanTaskParSys.GetString("PARKED") == "NO")
      {
         I_ErrorString += "The gantry has not parked.\n";
         DEBUG("The gantry has not parked.\n");
         return false;
      }
   }
   DEBUG("Lock check has completed.\n");
   return true;
}

void TdFCanTask::tdFfpiFlexure(long int x, long int y,
                               double ha, double dec, TdfFlexType *pars, long int *dx, long int *dy)
{
   double pa, zd;
   double phi = -0.54589;
   double yd;
   double dyd;

   *dy = 0;

   /*  parallactic angle */
   pa = slaPa(ha, dec, phi);

   /*  Zenith distance */
   zd = slaZd(ha, dec, phi);

   yd = (double)y - pars->offset;

   dyd = pars->k * yd * (pars->length - yd) * cos(pa) * sin(zd);

   *dx = (long)(dyd * 4.0 / ((pars->length) * (pars->length)));
}

void TdFCanTask::tdFstateBitSet(unsigned char bit)
{
   unsigned short oldval, newval = 0;
   oldval = ushortZero;
   newval = oldval | bit;

   /* Only update the parameter (triggering monitors) if value has changed*/
   if (newval != oldval)
      ushortZero = newval;
}

void TdFCanTask::tdFstateBitClear(unsigned char bit)
{
   unsigned short oldval, newval = 0;
   oldval = ushortZero;

   newval = oldval & (~bit);

   /* Only update the parameter (triggering monitors) if value has changed*/
   if (newval != oldval)
      ushortZero = newval;
}

void TdFCanTask::tdFfpiConvertFromEnc(int xEnc, int yEnc, double plateTheta,
                                      short level, double *xCon, double *yCon)
{

   long int dx = 0, dy = 0;
   double x_enc, y_enc, x_fp, y_fp, ha, dec;

   if ((level == _ALL) || (level == _FLEX))
   {
      *xCon = (double)xEnc;
      *yCon = (double)yEnc;
      return;
   }

   ha = dzeroHA;
   ha = I_TdFCanTaskParSys.GetDouble("HA");
   dec = dzeroDEC;
   dec = I_TdFCanTaskParSys.GetDouble("DEC");
   tdFfpiFlexure((long int)xEnc, (long int)yEnc,
                 ha, dec, &I_tdFfpiMainStruct->convert.flex,
                 &dx, &dy);

   x_enc = (double)xEnc + (double)dx;
   y_enc = (double)yEnc + (double)dy;
   /*
    *  This is all we want if the level is _TEMP.
    */
   if (level == _TEMP)
   {
      *xCon = x_enc;
      *yCon = y_enc;
      return;
   }

   if (level == _COEFFS)
   {
      *xCon = x_enc;
      *yCon = y_enc;
      return;
   }

   /*
    *  Translate/rotate/scale to non-deflected coordinates for set temperature.
    */
   slaXy2xy(x_enc, y_enc,                          /* Encoder coordinates     */
            I_tdFfpiMainStruct->convert.invCoeffs, /* Transformation matrix   */
            &x_fp, &y_fp);                         /* Field-plate coordinates */
   if (I_tdFfpiMainStruct->currentPlate == 1)
   {
      if (I_tdFfpiMainStruct->plateOneDontRemove)
      {
      }
      else
      {
         double t_x = x_fp;
         double t_y = y_fp;
         x_fp = t_x + (double)(*(I_tdFfpiMainStruct->pars.plt1CenterFidOffsetX));
         y_fp = t_y + (double)(*(I_tdFfpiMainStruct->pars.plt1CenterFidOffsetY));
      }
   }
   /*
    *  Remove rotation due to field plate rotation.
    */
   *xCon = x_fp * cos(-plateTheta) - y_fp * sin(-plateTheta);
   *yCon = y_fp * cos(-plateTheta) + x_fp * sin(-plateTheta);
}

bool TdFCanTask::tdFfpiUpdatePos(short updateIdeal, short useDpr, short displayText)
{
   double atFpX, atFpY;
   long int oldFpX, oldFpY, newFpX, newFpY;
   int ixPark, iyPark;

   if (I_tdFfpiMainStruct == nullptr)
   {
      I_ErrorString += "The structure pointer is not initialised.\n";
      return false;
   }
   if (useDpr)
   {
      CML::Linkage *MoveLinkage = I_CanAccess.GetLinkage(MAX_TDF_AMPS, G_AmpNames);
      if (MoveLinkage)
      {
         for (int Index = 0; Index < MAX_TDF_AMPS; Index++)
         {
            CML::uunit CurrentPosition = 0.0;
            (*MoveLinkage)[Index].GetPositionActual(CurrentPosition);
            if (Index < 2)
            {
               I_tdFfpiMainStruct->atEnc.x = (int)CurrentPosition;
               DEBUG("\nNow the encoder position of X axis is %d", I_tdFfpiMainStruct->atEnc.x);
            }
            else if (Index == 2)
            {
               I_tdFfpiMainStruct->atEnc.y = (int)CurrentPosition;
               DEBUG("\nNow the encoder position of Y axis is %d", I_tdFfpiMainStruct->atEnc.y);
            }
         }
      }
      else
      {
         I_ErrorString += "Failed to get information from the motor.\n";
         return false;
      }
   }
   else
   {
      I_tdFfpiMainStruct->atEnc.x = I_tdFfpiMainStruct->toEnc.x;
      I_tdFfpiMainStruct->atEnc.y = I_tdFfpiMainStruct->toEnc.y;
   }

   {
      /*
       * Update the parameters which contain the encoder values - but only if they
       * have actually changed (to avoid flooding parameter monitors)
       */
      ixPark = I_TdFCanTaskParSys.GetInt("X");
      if (ixPark != I_tdFfpiMainStruct->atEnc.x)
      {
         // xPark = I_tdFfpiMainStruct->atEnc.x;
         // xCopleyPark = I_tdFfpiMainStruct->atEnc.x;
         I_TdFCanTaskParSys.Put("X", (long int)I_tdFfpiMainStruct->atEnc.x);
         I_TdFCanTaskParSys.Put("XCopley", (long int)I_tdFfpiMainStruct->atEnc.x);
      }
      iyPark = I_TdFCanTaskParSys.GetInt("Y");
      if (iyPark != I_tdFfpiMainStruct->atEnc.y)
      {
         // yPark = I_tdFfpiMainStruct->atEnc.y;
         // yCopleyPark = I_tdFfpiMainStruct->atEnc.y;
         I_TdFCanTaskParSys.Put("Y", (long int)I_tdFfpiMainStruct->atEnc.y);
         I_TdFCanTaskParSys.Put("YCopley", (long int)I_tdFfpiMainStruct->atEnc.y);
      }
   }
   double dPlateTheta = I_TdFCanTaskParSys.GetDouble("PLATE_THETA");
   tdFfpiConvertFromEnc(I_tdFfpiMainStruct->atEnc.x,
                        I_tdFfpiMainStruct->atEnc.y,
                        (double)dPlateTheta, _FULL,
                        &atFpX, &atFpY);

   if (displayText)
      DEBUG("\ntdFfpi:Converted encoder values %d, %d to %f, %f.\n", I_tdFfpiMainStruct->atEnc.x, I_tdFfpiMainStruct->atEnc.y, atFpX, atFpY);

   oldFpX = (long int)(ixPark);
   oldFpX = I_TdFCanTaskParSys.GetLong("X");
   oldFpY = (long int)(iyPark);
   oldFpY = I_TdFCanTaskParSys.GetLong("Y");
   newFpX = doubleToLong(atFpX);
   newFpY = doubleToLong(atFpY);

   if (oldFpX != newFpX)
   {
      xPark = newFpX;
      xCopleyPark = newFpX;
      I_TdFCanTaskParSys.Put("X", (long int)newFpX);
      I_TdFCanTaskParSys.Put("XCopley", (long int)newFpX);
   }
   if (oldFpY != newFpY)
   {
      yPark = newFpY;
      yCopleyPark = newFpY;
      I_TdFCanTaskParSys.Put("Y", (long int)newFpY);
      I_TdFCanTaskParSys.Put("YCopley", (long int)newFpY);
   }
   if ((newFpX >= (FPI_CLEAR_X - 1000)) && (newFpY <= (FPI_CLEAR_Y + 1000)))
      tdFstateBitSet(SAFE);
   else
      tdFstateBitClear(SAFE);

   if (updateIdeal)
   {
      I_tdFfpiMainStruct->ideal.x = newFpX;
      I_tdFfpiMainStruct->ideal.y = newFpY;
   }
   return true;
}

bool CSearchAction::MoveToSearchPosition(const long searchX, const long searchY, short *const atSearchXY,
                                         short *const searchStarted)
{
   *searchStarted = YES;
   *atSearchXY = YES;
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());

   if (!(ThisTask->SetupAmps()))
   {
      DEBUG("MoveToSearchPosition: Failed to set up the amplifiers %s.\n", ThisTask->GetError().c_str());
      return false;
   }
   else
   {
      std::string Error;
      std::string Axes = "X,Y";
      std::string Velocities;
      std::string Positions = to_string(searchX) + "," + to_string(searchY);
      printf("MoveToSearchPosition: the position is %s.\n", Positions.c_str());
      std::vector<AxisDemand> AxisDemands = GetDemands(Axes, Positions, Velocities, Error);
      if (!(ThisTask->MoveAxes(AxisDemands, false)))
      {
         DEBUG("MoveToSearchPosition: failed to move to the position %s.\n", ThisTask->GetError().c_str());
         return false;
      }
   }
   return true;
}

char *Dul___FitsImgErrStr(int status)
{
   static char errorMessage[80];
   fits_get_errstatus(status, errorMessage);

   return (errorMessage);
}

bool CSearchAction::PerformCentroid(drama::Path &cameraPath, const tdFfpiCENtype *cenWin, const double settletime, short *const centroided)
{
   *centroided = YES;
   if (settletime > 0.0)
   {
      std::this_thread::sleep_for(std::chrono::seconds(int(settletime)));
   }

   cameraPath.GetPath(this);

   MessageUser("C_SEARCH PerformCentroid: - grabbing image");
   MessageUser("C_SEARCH PerformCentroid:  Window size -> Max %ld %ld off %ld %ld, dim %ld %ld",
               cenWin->window.MaxX, cenWin->window.MaxY,
               cenWin->window.Xoffset, cenWin->window.Yoffset,
               cenWin->window.Xdim, cenWin->window.Ydim);

   drama::sds::Id messageArg(drama::sds::Id::CreateArgStruct());
   drama::sds::IdPtr returnedArg;
   std::string strWindowType;
   strWindowType = to_string(cenWin->window.Xoffset) + ":" + to_string(cenWin->window.Yoffset) + ":" + to_string(cenWin->window.Xoffset + cenWin->window.Xdim - 1) + ":" + to_string(cenWin->window.Yoffset + cenWin->window.Ydim - 1);

   messageArg.Put("CENTROID", strWindowType);
   messageArg.Put("BIAS", cenWin->img->bias);
   messageArg.Put("EXPOSURE_TIME", cenWin->img->exposureTime);
   messageArg.Put("SHUTTER_OPEN", (int)cenWin->img->shutter);
   messageArg.Put("UPDATE", cenWin->img->updateTime);

   cameraPath.Obey(this, "CENTRECT", messageArg, &returnedArg);

   if (returnedArg == nullptr)
   {
      DEBUG("C_SEARCH PerformCentroid: Failed to perform centroid and return an image.\n");
      return false;
   }
   returnedArg->Get("XVALUE", &m_tdFfpiSFStruct->xErr);
   returnedArg->Get("YVALUE", &m_tdFfpiSFStruct->yErr);

   return true;

   //    if (settletime > 0)
   //    {
   //       int dTime = static_cast<int>(settletime);
   //       sleep(dTime);
   //    }
   //    fitsfile *fptr = 0;
   //    int status = 0;
   //    std::string filename = "";
   //    fits_open_file(&fptr, filename.c_str(), READONLY, &status);

   //    if (status != 0)
   //    {
   //       DEBUG("FitsImgOpen:Failed to open fits file \"%s\", FITSIO error \"%s\"", filename.c_str(), Dul___FitsImgErrStr(status));
   //       return false;
   //    }
   // #define MAXDIM 20
   //    int simple;         /* Does file confirm to fits standard*/
   //    int bitpix;         /* Number of bits per pixel value */
   //    int naxis;          /* Number of axes in data aray	*/
   //    long naxes[MAXDIM]; /* Array of length of each axis	*/
   //    long pcount;        /* Number of group parameters 	*/
   //    long gcount;        /* Number of random groups	*/
   //    int extend;         /* May extensions be present	*/

   //    long nelem; /* Number of data elements to read  */
   //    int anynul; /* Set true if any values were null */

   //    fits_read_imghdr(fptr, MAXDIM, &simple, &bitpix, &naxis, naxes,
   //                     &pcount, &gcount, &extend, &status);
   //    if (status != 0)
   //    {
   //       DEBUG("FitsImgRead: Failed to read fits file keywords, FITSIO error \"%s\"", Dul___FitsImgErrStr(status));
   //       return false;
   //    }

   //    nelem = naxes[0] * naxes[1];
   //    if ((*data = (unsigned char *)DitsMalloc(nelem * (sizeof *data))) == 0)
   //    {
   //       DEBUG("FitsImgRead:Failed to allocate memory for %ld pixels", nelem);
   //       fits_close_file(fptr, &status);
   //       return false;
   //    }

   //    fits_read_img_byt(fptr, 0, 1, nelem, 0, *data, &anynul, &status);
   //    if (status != 0)
   //    {
   //       DitsFree(*data);
   //       DEBUG("FitsImgRead:Failed to read data from fits file, FITSIO error \"%s\"", Dul___FitsImgErrStr(status));
   //       status = 0;
   //       fits_close_file(fptr, &status);
   //    }

   //    fits_close_file(fptr, &status);
   //    if (status != 0)
   //    {
   //       DitsFree(*data);
   //       DEBUG("FitsImgRead:Failed to close fits file \"%s\", FITSIO error \"%s\"",
   //             filename.c_str(), Dul___FitsImgErrStr(status));
   //       return false;
   //    }

   //    return true;
}

//  ------------------------------------------------------------------------------------------------

//                    T d F  C a n  T a s k  : :  S e t u p  A m p s
//
//  This sets up the CML::Amp objects used to control the six servo amplifiers used for the
//  gantry. This is a routine that is part of the task code, so that it can be used by multiple
//  actions. (See programming notes.)

bool TdFCanTask::SetupAmps(void)
{

   //  See if all the amps have already been setup successfully.

   // bool AllFound = I_X1Amp && I_X2Amp && I_YAmp && I_ZAmp && I_ThetaAmp && I_JawAmp;
   bool AllFound = I_X1Amp && I_X2Amp && I_YAmp;

   if (!AllFound)
   {

      //  If not, lets start from scratch and get them all afresh. (All we're doing is asking
      //  the CanAccess layer for pointers to objects it should already have, so there's no
      //  problem doing this.)

      I_X1Amp = I_X2Amp = I_YAmp = I_ZAmp = I_ThetaAmp = I_JawAmp = NULL;

      //  Initialise the CanAccess system, on the basis of the configuration described in the
      //  overall configuration file. This should contain details of the CANBus layout, and
      //  should specify which items are being run in simulation. All aspects of simulation
      //  should be handled by the CanAccess layer.

      if (!I_CanAccessInitialised)
      {
         std::string FileName = CONFIGURATION_FILE;
         std::string ExpandedName;
         if (TcsUtil::ExpandFileName(FileName, ExpandedName))
            FileName = ExpandedName;

         if (!I_CanAccess.Initialise(FileName))
         {
            I_ErrorString = "Error initialising CANBus access. " + I_CanAccess.GetError();
         }
         else
         {
            I_CanAccessInitialised = true;
         }
      }

      DEBUG("\nSetting amp addrs\n");

      //  Access the CML::Amp objects used to control the specified axes.

      static CML::Amp **AmpAddrs[MAX_TDF_AMPS] =
          {&I_X1Amp, &I_X2Amp, &I_YAmp};

      if (I_CanAccessInitialised)
      {
         int FoundCount = 0;
         for (int Index = 0; Index < MAX_TDF_AMPS; Index++)
         {
            *(AmpAddrs[Index]) = I_CanAccess.GetAmp(G_AmpNames[Index]);
            if (*(AmpAddrs[Index]))
               FoundCount++;
         }
         if (FoundCount == MAX_TDF_AMPS)
            AllFound = true;
         if (!AllFound)
         {
            I_ErrorString = "Error setting up amplifiers: " + I_CanAccess.GetError();
         }
      }
   }
   return AllFound;
}

bool TdFCanTask::InitialisefpiMainStruct()
{
   if (I_tdFfpiMainStruct == nullptr)
   {
      // allocate memory to tdFfpiTaskType point lliu added on 02-05-2024
      I_tdFfpiMainStruct = (tdFfpiTaskType *)malloc(sizeof(tdFfpiTaskType));
      if (nullptr == I_tdFfpiMainStruct)
      {
         DEBUG("\nI_tdFfpiMainStruct fail to initialise\n");
         return false;
      }
      else
      {
         I_tdFfpiMainStruct->Initialised = NO;
         I_tdFfpiMainStruct->cameraInit = NO;
         I_tdFfpiMainStruct->currentPlate = -1; /* negative means not set */
         I_tdFfpiMainStruct->plateOneDontRemove = 0;
      }
   }
   return true;
}
double TdFCanTask::tdFautoThetaPos(long int x, long int y)
{
   double tmpdx, tmpdy;
   double theta;

   if ((x == 0) && (y == 0))
      return 0;

   tmpdx = (x < 0) ? -1 * x : x;
   tmpdy = (y < 0) ? -1 * y : y;
   if (x > 0 && y == 0)
      theta = 3 * PI / 2;
   else if (x > 0 && y > 0)
      theta = 3 * PI / 2 + atan(tmpdy / tmpdx);
   else if (x == 0 && y > 0)
      theta = 0;
   else if (x < 0 && y > 0)
      theta = PI / 2 - atan(tmpdy / tmpdx);
   else if (x < 0 && y == 0)
      theta = PI / 2;
   else if (x < 0 && y < 0)
      theta = PI / 2 + atan(tmpdy / tmpdx);
   else if (x == 0 && y < 0)
      theta = PI;
   else /* (x>0 && y<0) */
      theta = 3 * PI / 2 - atan(tmpdy / tmpdx);

   return theta;
}

bool TdFCanTask::DisableAmps()
{
   bool DisableAllAmps = false;
   const CML::Error *Err = NULL;

   if (!I_CanAccessInitialised)
   {
      DEBUG("\nCanAccess is not inlitialised!\n");
      return DisableAllAmps;
   }

   if (I_X1Amp)
   {
      Err = I_CanAccess.GetAmp(G_AmpNames[0])->Disable();
      if (Err)
      {
         I_ErrorString = "Error disabling X_1 amplifier. ";
         I_ErrorString += Err->toString();
         return DisableAllAmps;
      }
      I_X1Amp = nullptr;
      DEBUG("X_1 amplifier disabled!\n");
   }
   if (I_X2Amp)
   {
      Err = I_CanAccess.GetAmp(G_AmpNames[1])->Disable();
      if (Err)
      {
         I_ErrorString = "Error disabling X_2 amplifier. ";
         I_ErrorString += Err->toString();
         return DisableAllAmps;
      }
      I_X2Amp = nullptr;
      DEBUG("X_2 amplifier disabled!\n");
   }
   if (I_YAmp)
   {
      Err = I_CanAccess.GetAmp(G_AmpNames[2])->Disable();
      if (Err)
      {
         I_ErrorString = "Error disabling Y amplifier. ";
         I_ErrorString += Err->toString();
         return DisableAllAmps;
      }
      I_YAmp = nullptr;
      DEBUG("Y amplifier disabled!\n");
   }

   DEBUG("All amplifiers disabled!\n");
   return true;
}

//  ------------------------------------------------------------------------------------------------

//                    T d F  C a n  T a s k  : :  H o m e  A x e s
//
//  Home the specified axes. This uses a CML Linkage, so all the axes in question need to be
//  using the same CANBus.

bool TdFCanTask::HomeAxes(bool HomeX, bool HomeY, bool HomeZ, bool HomeTheta, bool HomeJaw)
{

   bool ReturnOK = false;
   //  Get the Linkage we can use.  Note that this will contain all the amplifiers for
   //  all the amplifiers for the gantry, in the order specified by G_AmpNames and the
   //  AmpId enum.

   CML::Linkage *HomeLinkage = I_CanAccess.GetLinkage(MAX_TDF_AMPS, G_AmpNames);
   if (HomeLinkage)
   {

      const CML::Error *Err = NULL;
      // lliu added on 15/05/2024 to check if a motion is in progress
      I_tdFfpiMainStruct->inUse = YES;

      CML::HomeConfig HomeConfigs[MAX_TDF_AMPS];
      for (int Index = 0; Index < MAX_TDF_AMPS; Index++)
      {
         SetupHomeConfig(HomeConfigs, Index, AmpId(Index));
      }

      //  Set flags to show which amps will be homed.

      bool HomeFlags[MAX_TDF_AMPS];
      for (int Index = 0; Index < MAX_TDF_AMPS; Index++)
      {
         HomeFlags[Index] = false;
      }
      if (HomeX)
      {
         HomeFlags[X1_AMP] = true;
         HomeFlags[X2_AMP] = true;
         I_tdFfpiMainStruct->toEnc.x = 0;
      }
      if (HomeY)
      {
         HomeFlags[Y_AMP] = true;
         I_tdFfpiMainStruct->toEnc.x = 0;
      }

      //  Set all the amps in the linkage to a pre-programmed halt mode, specifying that a
      //  halted axis will be 'floppy'.

      for (int Index = 0; Index < MAX_TDF_AMPS; Index++)
      {
         Err = (*HomeLinkage)[Index].SetHaltMode(CML::HALT_DISABLE);
         if (Err)
            DEBUG("Setting halt mode %s\n", Err->toString());
      }

      //  We set up movement limits for the home. At the moment, these are taken from
      //  Shannon's test code in circle-demo.cpp. Really, we should be getting these from
      //  the configurator(s). It does seem to suggest that the move limits for all amps
      //  in a linkage need to be the same. (See programming notes.)

      double VelLimit = 2000000;   // velocity (encoder counts / second)
      double AccelLimit = 2000000; // acceleration (cts/sec/sec)
      double DecelLimit = 2000000; // deceleration (cts/sec/sec)
      double JerkLimit = 2000000;  // jerk (cts/sec/sec/sec)
      Err = HomeLinkage->SetMoveLimits(VelLimit, AccelLimit, DecelLimit, JerkLimit);
      if (Err)
      {
         I_ErrorString = "Error setting linkage move limits. ";
         I_ErrorString += Err->toString();
      }
      else
      {

         //  Enable each axis.

         for (int Index = 0; Index < MAX_TDF_AMPS; Index++)
         {
            if (HomeFlags[Index])
            {
               Err = (*HomeLinkage)[Index].Enable();
               if (Err)
                  DEBUG("Enabling %s\n", Err->toString());
            }
         }

         //  Now we set the homing configuration for each axis and set it homing.

         for (int Index = 0; Index < MAX_TDF_AMPS; Index++)
         {
            if (HomeFlags[Index])
            {
               DEBUG("Homing amp %d\n", Index);
               Err = (*HomeLinkage)[Index].GoHome(HomeConfigs[Index]);
               // Err = HomeLinkage->GetAmp(Index).WaitMoveDone( 20000 );
               // CML::uunit Pos;
               // HomeLinkage->GetAmp(Index).GetPositionActual(Pos);
               //(*HomeLinkage)[Index].GetPositionActual(Pos);
               // DEBUG("Amp %d controls the gantry, now in Position %f\n", Index, Pos);
               if (Err)
               {
                  I_ErrorString = "Error starting homing. ";
                  I_ErrorString += Err->toString();
                  //  We ought to stop all the homing at this point.
               }
            }
         }

         //  And now we wait for the homing to complete.

         DEBUG("Waiting for completion\n");

         Err = WaitLinkedHome(HomeLinkage, 20000);
         // Err = HomeLinkage->WaitMoveDone (20000);
         if (Err)
         {
            I_ErrorString = "Homing of axes completed with error. ";
            I_ErrorString += Err->toString();
         }

         DEBUG("Move complete\n");

         /// the code is used to test if the homing poisition is set to zero after the move completes.
         /// We have tried different homing methods. CHM_EXTENDED doesn't home properly, even though
         /// the position is zero after the homing process.

         // sleep(10);
         // for (int Index = 0; Index < MAX_TDF_AMPS; Index++) {
         //    if (HomeFlags[Index]) {
         //       CML::uunit Pos;
         //       HomeLinkage->GetAmp(Index).GetPositionActual(Pos);
         //       //(*HomeLinkage)[Index].GetPositionActual(Pos);
         //       DEBUG("Move completes, now Amp %d is in Position %f\n", Index, Pos);
         //    }
         // }
         //  Disable the amplifiers.

         // Perry requested these amplifers to be active after these moves, so disable this block of codes
         // as request.

         // for (int Index = 0; Index < MAX_TDF_AMPS; Index++) {
         //    Err = (*HomeLinkage)[Index].Disable();
         //    if (Err) {
         //       I_ErrorString = "Error disabling amplifier. ";
         //       I_ErrorString += Err->toString();
         //    }
         // }
         // DEBUG ("All complete\n");
      }
      if (Err == NULL)
      {
         ReturnOK = true;
         I_TdFCanTaskParSys.Put("PARKED", "YES");
      }

      I_tdFfpiMainStruct->inUse = NO;
   }
   DEBUG("HomeAxes returns\n");
   return ReturnOK;
}

//------------------------------------------------------------------------------------------------
//
bool TdFCanTask::SetParameter(string &ParameterName, string &ParameterValue)
{
   if (strcmp(ParameterName.c_str(), "MODE") == 0)
   {
      tdfCanMode = ParameterValue;
      DEBUG("\nSetting up the mode %s\n", ParameterValue.c_str());
   }
   else if ((strcmp(ParameterName.c_str(), "X") == 0) || (strcmp(ParameterName.c_str(), "Y") == 0))
   {
      if (tdfCanMode != "NOT_PROTECTED")
      {
         I_ErrorString += "In Protected Mode, cannot set up the gantry!";
         return false;
      }
      else
      {
         int PosVal = stoi(ParameterValue);
         ;
         if (strcmp(ParameterName.c_str(), "X") == 0)
         {
            xPark = PosVal;
            I_TdFCanTaskParSys.Put("X", (INT32)PosVal);
            DEBUG("Setting up the X %d\n", (int)xPark);
         }
         else
         {
            yPark = PosVal;
            I_TdFCanTaskParSys.Put("Y", (INT32)PosVal);
            DEBUG("Setting up the Y %d\n", (int)yPark);
         }
      }
   }
   else if (strcmp(ParameterName.c_str(), "XY_VEL") == 0)
   {
      int VelVal = stoi(ParameterValue);
      xyVel = VelVal;
      I_TdFCanTaskParSys.Put("XY_VEL", (INT32)VelVal);
      DEBUG("Setting up the velocity %d\n", (int)xyVel);
   }
   else if ((strcmp(ParameterName.c_str(), "X_ACCURACY") == 0) ||
            (strcmp(ParameterName.c_str(), "Y_ACCURACY") == 0))
   {
      int accuracy = stoi(ParameterValue);
      if (strcmp(ParameterName.c_str(), "X_ACCURACY") == 0)
      {
         xaccuracy = accuracy;
         I_TdFCanTaskParSys.Put("X_ACCURACY", (INT32)accuracy);
         DEBUG("Setting up the X_ACCURACY %d\n", (int)xaccuracy);
      }
      else
      {
         yaccuracy = accuracy;
         I_TdFCanTaskParSys.Put("Y_ACCURACY", (INT32)accuracy);
         DEBUG("Setting up the Y_ACCURACY %d\n", (int)yaccuracy);
      }
   }
   else if (strcmp(ParameterName.c_str(), "PLATE_THETA") == 0)
   {
      double ThetaVal = stod(ParameterValue);
      plateTheta = ThetaVal;
      I_TdFCanTaskParSys.Put("PLATE_THETA", (double)ThetaVal);
      DEBUG("Setting up the plate theta %f\n", (double)plateTheta);
   }
   else if (strcmp(ParameterName.c_str(), "PARKED") == 0)
   {
      if (tdfCanMode != "NOT_PROTECTED")
      {
         I_ErrorString += "In Protected Mode, cannot set up the Parked Mode!";
         return false;
      }
      // validate the value
      if (ParameterValue == "YES" || ParameterValue == "NO")
      {
         tdfCanParked = ParameterValue;
         I_TdFCanTaskParSys.Put("PARKED", std::string(ParameterValue));
      }

      DEBUG("Setting up the PARKED Mode %s\n", ParameterValue.c_str());
   }
   else if ((strcmp(ParameterName.c_str(), "HA") == 0) ||
            (strcmp(ParameterName.c_str(), "DEC") == 0))
   {
      double dVal = stod(ParameterValue);
      if (strcmp(ParameterName.c_str(), "HA") == 0)
      {
         dzeroHA = dVal;
         I_TdFCanTaskParSys.Put("HA", (double)dVal);
      }
      else
      {
         I_TdFCanTaskParSys.Put("DEC", (double)dVal);
         dzeroDEC = dVal;
      }
      DEBUG("Setting up the %s to be %f\n", ParameterName.c_str(), dVal);
   }
   else if ((strcmp(ParameterName.c_str(), "STEP_SIZE") == 0) ||
            (strcmp(ParameterName.c_str(), "MAX_ERROR") == 0) ||
            (strcmp(ParameterName.c_str(), "FIBRE_IN_IMAGE") == 0) ||
            (strcmp(ParameterName.c_str(), "POS_TOL") == 0) ||
            (strcmp(ParameterName.c_str(), "VFG_OP_ENABLE") == 0) ||
            (strcmp(ParameterName.c_str(), "TIMEOUT_FAC") == 0))
   {
      int iVal = stoi(ParameterValue);
      if (strcmp(ParameterName.c_str(), "STEP_SIZE") == 0)
      {
         stepSize = iVal;
         I_TdFCanTaskParSys.Put("STEP_SIZE", (INT32)(iVal));
      }
      else if (strcmp(ParameterName.c_str(), "MAX_ERROR") == 0)
      {
         maxError = iVal;
         I_TdFCanTaskParSys.Put("MAX_ERROR", (INT32)(iVal));
      }
      else if (strcmp(ParameterName.c_str(), "FIBRE_IN_IMAGE") == 0)
      {
         fibInImgThres = iVal;
         I_TdFCanTaskParSys.Put("FIBRE_IN_IMAGE", (INT32)(iVal));
      }
      else if (strcmp(ParameterName.c_str(), "POS_TOL") == 0)
      {
         posTol = iVal;
         I_TdFCanTaskParSys.Put("POS_TOL", (INT32)(iVal));
      }
      else if (strcmp(ParameterName.c_str(), "TIMEOUT_FAC") == 0)
      {
         timeoutFac = iVal;
         I_TdFCanTaskParSys.Put("TIMEOUT_FAC", (INT32)(iVal));
      }
      else if (strcmp(ParameterName.c_str(), "VFG_OP_ENABLE") == 0)
      {
         overlayPlaneEnabled = iVal;
         I_TdFCanTaskParSys.Put("VFG_OP_ENABLE", (INT32)(iVal));
      }
      DEBUG("Setting up the %s to be %d\n", ParameterName.c_str(), iVal);
   }
   else if ((strcmp(ParameterName.c_str(), "PLT1_CFID_OFF_X") == 0) ||
            (strcmp(ParameterName.c_str(), "PLT1_CFID_OFF_Y") == 0))
   {
      int iVal = stoi(ParameterValue);
      if (strcmp(ParameterName.c_str(), "PLT1_CFID_OFF_X") == 0)
      {
         shortZeroX = iVal;
         I_TdFCanTaskParSys.Put("PLT1_CFID_OFF_X", (short)(iVal));
      }
      else
      {
         shortZeroY = iVal;
         I_TdFCanTaskParSys.Put("PLT1_CFID_OFF_Y", (short)(iVal));
      }

      DEBUG("Setting up the %s to be %d\n", ParameterName.c_str(), iVal);
   }
   else if (strcmp(ParameterName.c_str(), "POS_ATTEMPTS") == 0)
   {
      int iVal = stoi(ParameterValue);
      attempts = iVal;
      I_TdFCanTaskParSys.Put("POS_ATTEMPTS", (INT32)(iVal));
      DEBUG("Setting up the %s to be %d\n", ParameterName.c_str(), iVal);
   }
   else if (strcmp(ParameterName.c_str(), "SETTLE_TIME") == 0)
   {
      double dVal = stod(ParameterValue);
      settleTime = dVal;
      I_TdFCanTaskParSys.Put("SETTLE_TIME", (double)(dVal));
      DEBUG("Setting up the %s to be %f\n", ParameterName.c_str(), dVal);
   }
   else if (strcmp(ParameterName.c_str(), "ZEROCAM_CENWAIT") == 0)
   {
      double dVal = stod(ParameterValue);
      zerocamCenWait = dVal;
      I_TdFCanTaskParSys.Put("ZEROCAM_CENWAIT", (double)(dVal));
      DEBUG("Setting up the %s to be %f\n", ParameterName.c_str(), dVal);
   }
   else if (strcmp(ParameterName.c_str(), "GANTRY_LAMPS") == 0)
   {
      tdfGantryLamp = ParameterValue;
      I_TdFCanTaskParSys.Put("GANTRY_LAMPS", std::string(ParameterValue));
      DEBUG("Setting up the %s to be %s\n", ParameterName.c_str(), ParameterValue);
   }
   else if ((strcmp(ParameterName.c_str(), "PMAC_LIM_X_POS") == 0) ||
            (strcmp(ParameterName.c_str(), "PMAC_LIM_X_NEG") == 0) ||
            (strcmp(ParameterName.c_str(), "PMAC_LIM_Y_POS") == 0) ||
            (strcmp(ParameterName.c_str(), "PMAC_LIM_Y_NEG") == 0))
   {
      int iVal = stoi(ParameterValue);
      if (strcmp(ParameterName.c_str(), "PMAC_LIM_X_POS") == 0)
      {
         // copleyXPosLim = iVal;
         I_TdFCanTaskParSys.Put("PMAC_LIM_X_POS", (INT32)(iVal));
      }
      else if (strcmp(ParameterName.c_str(), "PMAC_LIM_X_NEG") == 0)
      {
         // copleyXNegLim = iVal;
         I_TdFCanTaskParSys.Put("PMAC_LIM_X_NEG", (INT32)(iVal));
      }
      else if (strcmp(ParameterName.c_str(), "PMAC_LIM_Y_POS") == 0)
      {
         // copleyYPosLim = iVal;
         I_TdFCanTaskParSys.Put("PMAC_LIM_Y_POS", (INT32)(iVal));
      }
      else if (strcmp(ParameterName.c_str(), "PMAC_LIM_Y_NEG") == 0)
      {
         // copleyYNegLim = iVal;
         I_TdFCanTaskParSys.Put("PMAC_LIM_Y_NEG", (INT32)(iVal));
      }
      DEBUG("Setting up the %s to be %d\n", ParameterName.c_str(), iVal);
   }
   else if (strcmp(ParameterName.c_str(), "DEBUG_CENTROID") == 0)
   {
      int iVal = stoi(ParameterValue);
      ushortZeroCentroid = iVal;
      I_TdFCanTaskParSys.Put("DEBUG_CENTROID", (unsigned short)(iVal));
      DEBUG("Setting up the %s to be %d\n", ParameterName.c_str(), iVal);
   }
   return true;
}

//  ------------------------------------------------------------------------------------------------

//                 T d F  C a n  T a s k  : :  S e t u p  H o m e  C o n f i g
//
//  This routine fills one entry (specified by Index) of the array of amplifier homing
//  configurations (HomeConfigs) for the specified amplifier.

bool TdFCanTask::SetupHomeConfig(CML::HomeConfig HomeConfigs[], int Index, AmpId Amp)
{

   bool ReturnOK = true;

   if (Amp < 0 || Amp > 5)
   {
      I_ErrorString = "Invalid amplifier number for a home configuration.";
      ReturnOK = false;
   }
   else
   {

      //  For the moment, we use the single configuration from Shannon's circle-demo code for
      //  all amplifiers. What we should be doing is using the configurators to get the individual
      //  settings that apply to each amplifier.

      HomeConfigs[Index].method = CML::CHM_NLIM_ONDX;
      HomeConfigs[Index].extended = 0b0000001000110001; // homes to neg limit, then to next index
      // HomeConfigs[Index].current = 100; // max current in units of 0.01A (hardstop only)
      HomeConfigs[Index].velFast = 10000;
      HomeConfigs[Index].velSlow = 1500;
      HomeConfigs[Index].accel = 20000;
      HomeConfigs[Index].offset = 0;
   }
   return ReturnOK;
}

//  ------------------------------------------------------------------------------------------------

//                    T d F  C a n  T a s k  : :  G e t  A m p
//
//  Given an AmpId, returns a pointer to the CML Amp object controlling the axis in question.
//  If no amp can be found, this returns NULL.

CML::Amp *TdFCanTask::GetAmp(AmpId AxisId)
{

   //  The addresses of the instance variables containing the pointers to the different
   //  amplifiers, once SetupAmps() has run successfully. Note that the order has to match
   //  that used for the definition of the AmpId enum.

   static CML::Amp **AmpPtrs[MAX_TDF_AMPS] =
       {&I_X1Amp, &I_X2Amp, &I_YAmp};

   CML::Amp *AmpPtr = NULL;
   if (SetupAmps())
   {
      if (AxisId >= 0 && AxisId < MAX_TDF_AMPS)
      {
         AmpPtr = *AmpPtrs[AxisId];
      }
   }
   return AmpPtr;
}

//  ------------------------------------------------------------------------------------------------

//                   T d F  C a n  T a s k  : :  S e t u p  L i n k a g e
//
//  Given a CML Linkage and an array of addresses of CML::Amp objects that are to be linked,
//  this performs the common basic setup for the Linkage, suitable for both HomeAxes() and
//  MoveAxes(). The Linkage is set up and each amplifier is enabled.

bool TdFCanTask::SetupLinkage(
    CML::Linkage &TheLinkage, unsigned int NumberAmps, CML::Amp *LinkedAmps[], double Limits[])
{
   const CML::Error *Err = NULL;

   DEBUG("Initialising linkage\n");
   Err = TheLinkage.Init(NumberAmps, LinkedAmps);
   if (Err)
   {
      I_ErrorString = "Error initialising linkage. ";
      I_ErrorString += Err->toString();
   }
   else
   {

      DEBUG("Configuring linkage\n");
      CML::LinkSettings LinkConfig;
      LinkConfig.haltOnPosWarn = true; // halt all amps if position following error detected
      LinkConfig.haltOnVelWin = false; // halt all amps if velocity following error detected
      Err = TheLinkage.Configure(LinkConfig);
      if (Err)
         DEBUG("Configuring %s\n", Err->toString());

      //  Set all the amps in the linkage to a pre-programmed halt mode, specifying that a
      //  halted axis will be 'floppy'.

      DEBUG("Setting halt mode\n");
      for (unsigned int Index = 0; Index < NumberAmps; Index++)
      {
         Err = TheLinkage[Index].SetHaltMode(CML::HALT_DISABLE);
         if (Err)
            DEBUG("Setting halt mode %s\n", Err->toString());
      }

      //  We set up movement limits for the home. At the moment, these are taken from
      //  Shannon's test code in circle-demo.cpp. Really, we should be getting these from
      //  the configurator(s). It does seem to suggest that the move limits for all amps
      //  in a linkage need to be the same. (See programming notes.) We have default
      //  limits set here, but these can be overriden by those supplied in the call.

      DEBUG("Setting move limits\n");
      double VelLimit = 2000000;   // velocity (encoder counts / second)
      double AccelLimit = 2000000; // acceleration (cts/sec/sec)
      double DecelLimit = 2000000; // deceleration (cts/sec/sec)
      double JerkLimit = 2000000;  // jerk (cts/sec/sec/sec)
      if (Limits != NULL)
      {
         VelLimit = Limits[0];
         AccelLimit = Limits[1];
         DecelLimit = Limits[2];
         JerkLimit = Limits[3];
      }
      DEBUG("Setting move limits %f %f %f %f\n", VelLimit, AccelLimit, DecelLimit, JerkLimit);
      Err = TheLinkage.SetMoveLimits(VelLimit, AccelLimit, DecelLimit, JerkLimit);
      if (Err)
      {
         I_ErrorString = "Error setting linkage move limits. ";
         I_ErrorString += Err->toString();
      }
      else
      {

         //  Enable each axis.

         DEBUG("Enabling axes\n");
         for (unsigned int Index = 0; Index < NumberAmps; Index++)
         {
            Err = TheLinkage[Index].ClearFaults();
            if (Err)
               DEBUG("Clearing fault %s\n", Err->toString());
            DEBUG("Enabling axis %d, amp addr %p\n", Index, (void *)LinkedAmps[Index]);
            Err = TheLinkage[Index].Enable();
            if (Err)
               DEBUG("Enabling %s\n", Err->toString());
         }
      }
   }
   return (Err == NULL);
}

//  ------------------------------------------------------------------------------------------------

//                    T d F  C a n  T a s k  : :
void TdFCanTask::tdFfpiPositionCheck(int Index, CML::uunit &Position)
{
   switch (Index)
   {
   case 0:
   case 1:
   {
      if (Position < XMIN)
      {
         Position = XMIN;
      }
      else if (Position > XMAX)
         Position = XMAX;
      break;
   }
   case 2:
   {
      if (Position < YMIN)
      {
         Position = YMIN;
      }
      else if (Position > YMAX)
         Position = YMAX;
      break;
   }
   case 3:
   {
      if (Position < ZMIN)
      {
         Position = ZMIN;
      }
      else if (Position > ZMAX)
         Position = ZMAX;
      break;
   }
   }
}

//  ------------------------------------------------------------------------------------------------

//                    T d F  C a n  T a s k  : :  M o v e  A x e s
//
//  Moves the specified axes to a set of positions specified in a vector of AxisDemands, which
//  has a target position and, optionally, a demanded velocity for each axis.
//

bool TdFCanTask::MoveAxes(const std::vector<AxisDemand> &AxisDemands, bool MoveOffset /*=false*/,
                          drama::thread::TAction * /*ThisAction*/)
{

   bool ReturnOK = false;

   //  Get the Linkage we can use.  Note that this will contain all the amplifiers for
   //  all the amplifiers for the gantry, in the order specified by G_AmpNames and the
   //  AmpId enum.

   CML::Linkage *MoveLinkage = I_CanAccess.GetLinkage(MAX_TDF_AMPS, G_AmpNames);
   if (MoveLinkage)
   {

      const CML::Error *Err = NULL;

      //  We need to set the target point for each amplifier in the Linkage, given that
      //  this covers all the amplifiers in the gantry. Ones specified in the AxisDemands
      //  will have the new demanded position as their target, ones that are not to move
      //  will simply use their current position. Also note in MoveFlags[] which amps are
      //  specified in AxisDemands.

      CML::Point<MAX_TDF_AMPS> TargetPoint; // Used to pass positions to the Linkage
      CML::uunit Targets[MAX_TDF_AMPS];     // Used to pass positions to WaitLinkedHome(), if used.
      bool MoveFlags[MAX_TDF_AMPS];

      int NumberDemands = AxisDemands.size();
      for (int Index = 0; Index < MAX_TDF_AMPS; Index++)
      {
         CML::uunit TargetPosition = 0.0;
         AmpId Id = AmpId(Index);
         bool Found = false;
         for (int DemandIndex = 0; DemandIndex < NumberDemands; DemandIndex++)
         {
            if (AxisDemands[DemandIndex].AxisId == Id)
            {
               if (MoveOffset == true)
               {
                  CML::uunit CurrentPosition = 0.0;
                  (*MoveLinkage)[DemandIndex].GetPositionActual(CurrentPosition);
                  TargetPosition = AxisDemands[DemandIndex].Position + CurrentPosition;
               }
               else
               {
                  TargetPosition = AxisDemands[DemandIndex].Position;
               }
               Found = true;
               break;
            }
         }
         if (!Found)
         {
            (*MoveLinkage)[Index].GetPositionActual(TargetPosition);
         }

         tdFfpiPositionCheck(Index, TargetPosition);
         Targets[Index] = TargetPosition;
         TargetPoint[Index] = TargetPosition;
         DEBUG("Target for axis %d is %f\n", Index, TargetPosition);
         if (Index == 0 || Index == 1)
         {
            I_tdFfpiMainStruct->toEnc.x = doubleToLong(TargetPosition);
         }
         else if (Index == 2)
         {
            I_tdFfpiMainStruct->toEnc.y = doubleToLong(TargetPosition);
         }
         MoveFlags[Index] = Found;
      }
      TargetPoint.setDim(MAX_TDF_AMPS);

      //  We set up movement limits for the move. At the moment, these are taken from
      //  Shannon's test code in circle-demo.cpp. Really, we should be getting these from
      //  the configurator(s). It does seem to suggest that the move limits for all amps
      //  in a linkage need to be the same. (See programming notes.) The units are encoder
      //  counts and seconds. We take the specified velocity as a scale from 0 to 1, but
      //  have to use that from the first axis and ignore the others.

      double Scale = AxisDemands[0].Velocity;
      if (Scale > 1.0)
         Scale = 1.0;
      if (Scale < 0.001)
         Scale = 0.001;

      double VelLimit = 2000000 * Scale; // velocity (encoder counts / second)
      double AccelLimit = 2000000;       // acceleration (cts/sec/sec)
      double DecelLimit = 2000000;       // deceleration (cts/sec/sec)
      double JerkLimit = 2000000;        // jerk (cts/sec/sec/sec)

      DEBUG("Setting move limits %f %f %f %f\n", VelLimit, AccelLimit, DecelLimit, JerkLimit);
      Err = MoveLinkage->SetMoveLimits(VelLimit, AccelLimit, DecelLimit, JerkLimit);
      if (Err)
      {
         I_ErrorString = "Error setting linkage move limits. ";
         I_ErrorString += Err->toString();
      }

      if (Err == NULL)
      {

         //  Enable each axis.

         DEBUG("Enabling axes\n");
         for (int Index = 0; Index < MAX_TDF_AMPS; Index++)
         {
            Err = (*MoveLinkage)[Index].ClearFaults();
            if (Err)
               DEBUG("Clearing fault %s\n", Err->toString());
            //  I thought you might be able to disable axes that weren't to move, but
            //  if you do that MoveTo() fails with an 'axis not enabled' error. We have
            //  to rely on moving to the current position being OK.
            DEBUG("Enabling axis %d\n", Index);
            Err = (*MoveLinkage)[Index].Enable();
            if (Err)
               DEBUG("Enabling %s\n", Err->toString());
         }
      }

      if (Err == NULL)
      {

         //  I've commented out the KickHandler, because it seems it interferes with the CML
         //  use of semaphores, even with that UnblockSIGUSR2() call in there. This still
         //  needs to be understood - and preferably fixed. Tentatively, it seems that moving
         //  to the latest version of KickNotifier (Mar 23) and using a version of CML that
         //  doesn't use signals at all, produces a version that works.

         // DEBUG ("Establishing kick handler\n");
         // LinkageKicker KickHandler(ThisAction);
         // KickHandler.SetLinkage(&MoveLinkage);

         Err = MoveLinkage->MoveTo(TargetPoint);
         if (Err)
            DEBUG("Moving to target %s\n", Err->toString());
      }

      if (Err == NULL)
      {
         DEBUG("Waiting\n");

         //  Now that I no longer restore the SIGUSR2 blocking at the end of each thread, both
         //  WaitMoveDone() and WaitLinkedHome() seem to work as a way of waiting. Previously
         //  only WaitLinkedHome would work after the first move. I still don't know just why,
         //  but it's clearly connected with the use of SIGUSR2. I have both calls here to
         //  switch beteen them for testing.

         Err = MoveLinkage->WaitMoveDone(20000);
         // Err = WaitLinkedHome (MoveLinkage,20000,Targets);
         if (Err)
         {
            MoveLinkage->HaltMove();
            I_ErrorString = "Error moving to target position. ";
            I_ErrorString += Err->toString();
         }
         for (int Index = 0; Index < MAX_TDF_AMPS; Index++)
         {
            (*MoveLinkage)[Index].ClearFaults();
         }
      }
      if (Err == NULL)
         ReturnOK = true;
   }
   return ReturnOK;
}

/*
   //  Set up an array of pointers to the amps, containing just those that are going to be
   //  moved, suitable for use with a CML Linkage.

   bool OKSoFar = true;
   CML::Amp* LinkedAmps[MAX_TDF_AMPS];
   unsigned int NumberAmps = AxisDemands.size();
   for (unsigned int Index = 0; Index < NumberAmps; Index++) {
      LinkedAmps[Index] = GetAmp(AxisDemands[Index].AxisId);
      if (LinkedAmps[Index] == NULL) {
         I_ErrorString = "Not all amplifiers initialised properly.";
         OKSoFar = false;
         break;
      }
   }

   if (OKSoFar) {

      const CML::Error* Err = NULL;
      CML::Linkage MoveLinkage;

      //  We set up movement limits for the home. At the moment, these are taken from
      //  Shannon's test code in circle-demo.cpp. Really, we should be getting these from
      //  the configurator(s). It does seem to suggest that the move limits for all amps
      //  in a linkage need to be the same. (See programming notes.) The units are encoder
      //  counts and seconds. We take the specified velocity as a scale from 0 to 1, but
      //  have to use that from the first axis and ignore the others.

      double Scale = AxisDemands[0].Velocity;
      DEBUG ("Scale = %f\n",Scale);
      if (Scale > 1.0) Scale = 1.0;
      if (Scale < 0.001) Scale = 0.001;
      DEBUG ("Scale = %f\n",Scale);

      double Limits[4];
      Limits[0] = 2000000 * Scale;    // velocity (encoder counts / second)
      Limits[1] = 2000000;    // acceleration (cts/sec/sec)
      Limits[2] = 2000000;    // deceleration (cts/sec/sec)
      Limits[3] = 2000000;    // jerk (cts/sec/sec/sec)

      if (SetupLinkage(MoveLinkage,NumberAmps,LinkedAmps,Limits)) {

         DEBUG ("Setting target point\n");
         CML::uunit Targets[MAX_TDF_AMPS];
         CML::Point<MAX_TDF_AMPS> TargetPoint;
         for (unsigned int Index = 0; Index < NumberAmps; Index++) {
            TargetPoint[Index] = AxisDemands[Index].Position;
            Targets[Index] = AxisDemands[Index].Position;
         }
         TargetPoint.setDim(NumberAmps);

         //  I've commented out the KickHandler, because it seems it interferes with the CML
         //  use of semaphores, even with that UnblockSIGUSR2() call in there. This still
         //  needs to be understood - and preferably fixed. Tentatively, it seems that moving
         //  to the latest version of KickNotifier (Mar 23) and using a version of CML that
         //  doesn't use signals at all, produces a version that works.
         DEBUG ("Establishing kick handler\n");
         LinkageKicker KickHandler(ThisAction);
         KickHandler.SetLinkage(&MoveLinkage);

         MoveLinkage.MoveTo(TargetPoint);
         DEBUG ("Waiting\n");

         //  Now that I no longer restore the SIGUSR2 blocking at the end of each thread, both
         //  WaitMoveDone() and WaitLinkedHome() seem to work as a way of waiting. Previously
         //  only WaitLinkedHome would work after the first move. I still don't know just why,
         //  but it's clearly connected with the use of SIGUSR2. I have both calls here to
         //  switch beteen them for testing.
         Err = MoveLinkage.WaitMoveDone(40000);
         //Err = WaitLinkedHome (&MoveLinkage,20000,Targets);
         if (Err) {
            I_ErrorString = "Error moving to target position. ";
            I_ErrorString += Err->toString();
            OKSoFar = false;
         }
         for (unsigned int Index = 0; Index < NumberAmps; Index++) {
            LinkedAmps[Index]->ClearFaults();
         }


      }
      UnblockSIGUSR2();
      for (unsigned int Index = 0; Index < NumberAmps; Index++) {
         LinkedAmps[Index]->Disable();
      }
   }

   return OKSoFar;
}
*/

//  ------------------------------------------------------------------------------------------------

//              I n i t i a l i s e   A c t i o n : :  M e s s a g e  R e c e i v e d
//
//  This is the handler for the INITIALISE action. This initialises the overall task. At present
//  all it has to do is initialise the CanAccess layer and set up access for the task to the
//  various mechanisms, all of which is handled by one call to SetupAxes(). Note that this
//  runs in the main thread, which is probably the best place to create all the various CML
//  objects that will be accessed by the action handler threads that actually move mechanisms.
//  SetupAmps() is the routine that reads the overall configuration file - whose value is set
//  in the const string CONFIGURATION_FILE, which defines the CANBus structure and also which
//  parts run in simulation and which run on real hardware.

// lliu added initialisation of fpiMainStruct on 02/05/2024. When the initialisation fails, drama will
// exit
void InitialiseAction::ActionThread(const drama::sds::Id &)
{

   UnblockSIGUSR2();

   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   // drama::ParSys parSys(GetTask());
   drama::sds::Id parSysId(drama::sds::Id::CreateFromSdsIdType((long)(DitsGetParId())));

   ThisTask->ClearError();

   // lliu added on 15/05/2024
   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   if (details != nullptr && details->Initialised == YES)
   {
      MessageUser("INITIALISE: TdFCanTask is already initialised.");
      return;
   }

   if (!(ThisTask->InitialisefpiMainStruct()))
   {
      DEBUG("Fail to allocate memory to fpiMainStruct\n");
      return;
   }
   else
   {
      DEBUG("Succeed to allocate memory to fpiMainStruct\n");

      parSysId.Put("ENQ_VER_NUM", TdFCanTaskVersion);
      parSysId.Put("ENQ_VER_DATE", TdFCanTaskDate);

      drama::sds::Id id;        /* Parameter id                    */
      unsigned long int length; /* Length of parameter item        */
      short check = SHOW;
      tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
      /*
       *  For parameters we want quick access to, get pointers to them
       *  (we  can't do data conversion when getting such parameters)
       */
      // SdpGetSds("ZEROCAM_CENWAIT", &id, &status);
      id = parSysId.Find("ZEROCAM_CENWAIT");
      // This is the drama version
      // SdsPointer(id, (void **)&details->pars.zeroCamCenWait, &length, &status);
      id.Pointer(&details->pars.zeroCamCenWait, &length);
      if (length != sizeof(*(details->pars.zeroCamCenWait)))
      {

         DEBUG("Parameter ZEROCAM_CENWAIT length mismatch");
         return;
      }
      length = 0;
      // SdsFreeId(id, &status);

      // SdpGetSds("PLT1_CFID_OFF_X", &id, &status);
      id = parSysId.Find("PLT1_CFID_OFF_X");
      // This is the drama version
      // SdsPointer(id, (void **)&details->pars.plt1CenterFidOffsetX, &length, &status);
      id.Pointer(&details->pars.plt1CenterFidOffsetX, &length);
      if (length != sizeof(*(details->pars.plt1CenterFidOffsetX)))
      {
         DEBUG("Parameter PLT1_CFID_OFF_X length mismatch");
         return;
      }
      length = 0;
      // SdsFreeId(id, &status);

      // This is the drama version
      // SdpGetSds("PLT1_CFID_OFF_Y", &id, &status);
      id = parSysId.Find("PLT1_CFID_OFF_Y");
      // SdsPointer(id, (void **)&details->pars.plt1CenterFidOffsetY, &length, &status);
      id.Pointer(&details->pars.plt1CenterFidOffsetY, &length);
      if (length != sizeof(*(details->pars.plt1CenterFidOffsetY)))
      {
         DEBUG("Parameter PLT1_CFID_OFF_Y length mismatch");
         return;
      }
      length = 0;
      // SdsFreeId(id, &status);

      details->inUse = YES;

      bool defRead = ThisTask->tdFfpiDefRead(DEFS_FILE | FLEX_FILE,
                                             check);
      if (defRead == false)
      {
         details->inUse = NO;
         return;
      }

      details->ipsMode = 0;
      details->dprAddress = ERROR;
      details->toEnc.x = 0;
      details->toEnc.y = 0;

      details->Initialised = YES;
      details->inUse = NO;
   }

   ThisTask->tdFGetCameraPath().GetPath(this);
   ThisTask->tdFGetCameraPath().Obey(this, "INITIALISE");
   details->cameraInit = YES;

   if (!(ThisTask->SetupAmps()))
   {
      DEBUG("SetupAmps fails\n");
      MessageUser("INITIALISE: " + ThisTask->GetError());
   }
   else
   {
      DEBUG("Setup amps OK\n");
      MessageUser("INITIALISE: Task initialised from configuration file " + CONFIGURATION_FILE);
   }
   // We never re-enable the blocking of SIGUSR2 - it causes too many problems.
   // BlockSIGUSR2();
}

//  ------------------------------------------------------------------------------------------------

//                     G  I n i t   A c t i o n : :  A c t i o n  T h r e a d
//
//  This is the handler for the G_INIT action. This runs in its own thread. It initialises
//  the 2dF gantry, including homing the axes. It takes a single string argument (AXES) which
//  specifies the set of axes (X,Y,Z,Theta,Jaw) in question. Each specified axis is initialised,
//  which includes moving it to its home position.

void GInitAction::ActionThread(const drama::sds::Id &Arg)
{

   UnblockSIGUSR2();
   // lliu added on 16/05/2024 to add the lock
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());

   std::string Axes("");
   if (Arg)
   {
      drama::gitarg::String AxesArg(this, Arg, "AXES", 1);
      Axes = AxesArg;
   }
   MessageUser("G_INIT: " + Axes);
   bool UseX, UseY, UseZ, UseTheta, UseJaw;
   if (WhichAxes(Axes, UseX, UseY, UseZ, UseTheta, UseJaw))
   {
      if (UseX)
         MessageUser("Initialising X axis");
      if (UseY)
         MessageUser("Initialising Y axis");
      if (UseZ)
         MessageUser("Initialising Z axis");
      if (UseTheta)
         MessageUser("Initialising Theta axis");
      if (UseJaw)
         MessageUser("Initialising Jaw axis");

      if (!(ThisTask->SetupAmps()))
      {
         MessageUser("G_INIT: " + ThisTask->GetError());
      }
      else
      {
         if (!(ThisTask->HomeAxes(UseX, UseY, UseZ, UseTheta, UseJaw)))
         {
            MessageUser("G_INIT: " + ThisTask->GetError());
         }
         else
         {
            MessageUser("G_INIT: Axes homed");
         }
      }
   }
   else
   {
      MessageUser("G_INIT: Invalid axis specification");
   }

   // We never re-enable the blocking of SIGUSR2 - it causes too many problems.
   // BlockSIGUSR2();
}

//  ------------------------------------------------------------------------------------------------

//                   G  I n i t   A c t i o n : :  M e s s a g e  R e c e i v e d
//
//  This is the handler for the G_INIT action. This is just a diagnostic to see if the
//  G_INIT action worked differently if not threaded. Originally, this was the case, but
//  I now believe I understand the problem, which is connected to CML and use of signals. This
//  action is probably no longer needed.

void GInitActionNT::ActionThread(const drama::sds::Id &Arg)
{

   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());

   // lliu added on 15/05/2024
   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   if (details == nullptr || details->Initialised == NO)
   {
      MessageUser("G_INIT_NT: TdFCanTask is not initialised. Please use \"ditscmd TdFCanTask INITIALISE\" to initialise the task.");
      return;
   }
   if (details->inUse)
   {
      MessageUser("MOVE_AXES: TdFCanTask is running other actions.\n");
      return;
   }

   std::string Axes("");
   if (Arg)
   {
      drama::gitarg::String AxesArg(this, Arg, "AXES", 1);
      Axes = AxesArg;
   }
   MessageUser("G_INIT_NT: " + Axes);
   bool UseX, UseY, UseZ, UseTheta, UseJaw;
   if (WhichAxes(Axes, UseX, UseY, UseZ, UseTheta, UseJaw))
   {
      if (UseX)
         MessageUser("Initialising X axis");
      if (UseY)
         MessageUser("Initialising Y axis");
      if (UseZ)
         MessageUser("Initialising Z axis");
      if (UseTheta)
         MessageUser("Initialising Theta axis");
      if (UseJaw)
         MessageUser("Initialising Jaw axis");

      if (!(ThisTask->SetupAmps()))
      {
         MessageUser("G_INIT_NT: " + ThisTask->GetError());
      }
      else
      {
         details->inUse = YES;
         if (!(ThisTask->HomeAxes(UseX, UseY, UseZ, UseTheta, UseJaw)))
         {
            MessageUser("G_INIT_NT: " + ThisTask->GetError());
         }
         else
         {
            MessageUser("G_INIT_NT: Axes homed");
            MessageUser("G_INIT_NT: Waiting to receive signals about the current position of the gantry.");
            // lliu added on 15/05/2024 to update the position of gantry after initialisation
            std::this_thread::sleep_for(2000ms);
            if (!ThisTask->tdFfpiUpdatePos(YES, details->dprFeedback, YES))
            {
               MessageUser("G_INIT_NT: failed to Update the current position of gantry.");
            }
         }
         details->inUse = NO;
      }
   }
   else
   {
      MessageUser("G_INIT_NT: Invalid axis specification");
   }
   // We never re-enable the blocking of SIGUSR2 - it causes too many problems.
   // BlockSIGUSR2();
}

//  ------------------------------------------------------------------------------------------------

//                   M o v e  A x i s   A c t i o n : :  A c t i o n  T h r e a d
//
//  This is the handler for the G_MOVE_AXIS action. This runs in its own thread. It takes a single
//  string argument (AXES) which specifies the set of axes (X,Y,Z,Theta,Jaw) to be homed, and
//  a set of target encoder positions and a set of positions and velocities.
//

void GMoveAxisAction::ActionThread(const drama::sds::Id &Arg)
{

   UnblockSIGUSR2();
   // lliu added on 16/05/2024 to add the lock
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());

   std::string Axes;
   std::string Positions;
   std::string Velocities;
   if (Arg)
   {
      //  I didn't want a try block here. I just didn't want an exception - I wanted Git
      //  to use the default values I specified if there wasn't one in the arguments.
      //  But I can't work out how to do that.
      try
      {
         drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
         drama::gitarg::String AxesArg(this, Arg, "AXES", 1, "", NoFlags);
         Axes = AxesArg;
         drama::gitarg::String PositionsArg(this, Arg, "POSITIONS", 2, "", NoFlags);
         Positions = PositionsArg;
         drama::gitarg::String VelocitiesArg(this, Arg, "VELOCITIES", 3, "", NoFlags);
         Velocities = VelocitiesArg;
      }
      catch (...)
      {
      }
   }
   std::string Error;
   std::vector<AxisDemand> AxisDemands = GetDemands(Axes, Positions, Velocities, Error);

   int NumberAxes = AxisDemands.size();
   if (NumberAxes <= 0)
   {
      MessageUser("MOVE_AXES: " + Error);
   }
   else
   {
      unsigned int NumberAxes = AxisDemands.size();
      for (unsigned int Index = 0; Index < NumberAxes; Index++)
      {
         DEBUG("AxisId: %d, position %f, velocity %f\n", AxisDemands[Index].AxisId,
               AxisDemands[Index].Position, AxisDemands[Index].Velocity);
      }

      if (!(ThisTask->SetupAmps()))
      {
         MessageUser("MOVE_AXES: " + ThisTask->GetError());
      }
      else
      {
         if (!(ThisTask->MoveAxes(AxisDemands, false, this)))
         {
            MessageUser("MOVE_AXES: " + ThisTask->GetError());
         }
         else
         {
            MessageUser("MOVE_AXES: Move complete");
         }
      }
   }

   // We never re-enable the blocking of SIGUSR2 - it causes too many problems.
   // BlockSIGUSR2();
}

//  ------------------------------------------------------------------------------------------------

//         G  M o v e  A x i s   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
//
//  This is a version of the handler for the G_MOVE_AXIS action that isn't threaded and runs in
//  the contest of the main DRAMA thread. This responds to the G_MOVE_AXIS_NT action, which
//  has exactly the same arguments as G_MOVE_AXIS. The code is almost identical, except for
//  the way it accesses the arguments, the value it returns and the fact that it doesn't pass
//  a thread pointer to MoveAxes().

void GMoveAxisActionNT::ActionThread(const drama::sds::Id &Arg)
{

   UnblockSIGUSR2();

   // lliu added on 15/05/2024
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());

   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   if (details == nullptr || details->Initialised == NO)
   {
      MessageUser("G_MOVE_AXIS_NT: TdFCanTask is not initialised. Please use \"ditscmd TdFCanTask INITIALISE\" to initialise the task.");
      return;
   }
   if (!(ThisTask->tdFfpiIlocks(ILOCK__GAN_PARKED)))
   {
      MessageUser("G_MOVE_AXIS_NT: the gantry is parked, please unpark the gantry first.");
      return;
   }
   if (details->inUse)
   {
      MessageUser("G_MOVE_AXIS_NT: TdFCanTask is running other actions.\n");
      return;
   }

   std::string Axes;
   std::string Positions;
   std::string Velocities;
   std::string OffsetFlag = "";
   if (Arg)
   {
      //  I didn't want a try block here. I just didn't want an exception - I wanted Git
      //  to use the default values I specified if there wasn't one in the arguments.
      //  But I can't work out how to do that.
      try
      {
         drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
         drama::gitarg::String AxesArg(this, Arg, "AXES", 1, "", NoFlags);
         Axes = AxesArg;
         drama::gitarg::String PositionsArg(this, Arg, "POSITIONS", 2, "", NoFlags);
         Positions = PositionsArg;
         drama::gitarg::String VelocitiesArg(this, Arg, "VELOCITIES", 3, "", NoFlags);
         Velocities = VelocitiesArg;
         drama::gitarg::String OffsetFlagArg(this, Arg, "OFFSET", 4, "", NoFlags);
         OffsetFlag = OffsetFlagArg;
      }
      catch (...)
      {
      }
   }
   std::string Error;
   std::vector<AxisDemand> AxisDemands = GetDemands(Axes, Positions, Velocities, Error);
   bool MoveOffset = false;
   bool MoveBackward = false;
   if (!OffsetFlag.compare("+") || !OffsetFlag.compare("-"))
   {
      MoveOffset = true;
      if (!OffsetFlag.compare("-"))
         MoveBackward = true;
   }
   int NumberAxes = AxisDemands.size();
   if (NumberAxes <= 0)
   {
      MessageUser("G_MOVE_AXIS_NT: " + Error);
   }
   else
   {
      details->inUse = YES;

      unsigned int NumberAxes = AxisDemands.size();
      for (unsigned int Index = 0; Index < NumberAxes; Index++)
      {
         if (MoveBackward == true)
            AxisDemands[Index].Position *= (-1.0);
         DEBUG("AxisId: %d, position %f, velocity %f\n", AxisDemands[Index].AxisId,
               AxisDemands[Index].Position, AxisDemands[Index].Velocity);
      }

      if (!(ThisTask->SetupAmps()))
      {
         MessageUser("G_MOVE_AXIS_NT: " + ThisTask->GetError());
      }
      else
      {
         if (!(ThisTask->MoveAxes(AxisDemands, MoveOffset)))
         {
            MessageUser("G_MOVE_AXIS_NT: " + ThisTask->GetError());
         }
         else
         {
            MessageUser("G_MOVE_AXIS_NT: Move complete");
            MessageUser("G_MOVE_AXIS_NT: Waiting to receive signals about the current position of the gantry.");
            std::this_thread::sleep_for(2000ms);
            if (!(ThisTask->tdFfpiUpdatePos(YES, details->dprFeedback, YES)))
            {
               MessageUser("G_MOVE_AXIS_NT: Failed to update the position of the gantry");
            }
         }
      }

      details->inUse = NO;
   }

   // We never re-enable the blocking of SIGUSR2 - it causes too many problems.
   // BlockSIGUSR2();
}

//  ------------------------------------------------------------------------------------------------

//                   H o m e   A c t i o n : :  A c t i o n  T h r e a d
//
//  This is the handler for the HOME action. This runs in its own thread. It takes a single
//  string argument (AXES) which specifies the set of axes (X,Y,Z,Theta,Jaw) to be homed.
//
//  Apart from the fact that it says 'homing' instead of 'initialising', G_HOME is exactly
//  the same as G_INIT. It may turn out that there are other things that G_INIT needs to do
//  that it isn't doing at the moment.

void GHomeAction::ActionThread(const drama::sds::Id &Arg)
{

   UnblockSIGUSR2();
   // lliu added on 16/05/2024 to add the lock
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();

   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());
   std::string Axes("");
   if (Arg)
   {
      drama::gitarg::String AxesArg(this, Arg, "AXES", 1);
      Axes = AxesArg;
   }
   MessageUser("G_HOME: " + Axes);
   bool X, Y, Z, Theta, Jaw;
   if (WhichAxes(Axes, X, Y, Z, Theta, Jaw))
   {
      if (X)
         MessageUser("Homing X axis");
      if (Y)
         MessageUser("Homing Y axis");
      if (Z)
         MessageUser("Homing Z axis");
      if (Theta)
         MessageUser("Homing Theta axis");
      if (Jaw)
         MessageUser("Homing Jaw axis");

      if (!(ThisTask->SetupAmps()))
      {
         MessageUser("G_HOME: " + ThisTask->GetError());
      }
      else
      {
         if (!(ThisTask->HomeAxes(X, Y, Z, Theta, Jaw)))
         {
            MessageUser("G_HOME: " + ThisTask->GetError());
         }
         else
         {
            MessageUser("G_HOME: Axes homed");
         }
      }
   }
   else
   {
      MessageUser("G_HOME: Invalid axis specification");
   }

   // We never re-enable the blocking of SIGUSR2 - it causes too many problems.
   // BlockSIGUSR2();
}

//  ------------------------------------------------------------------------------------------------

//         G  Park  Gantry   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
//
// Currently the ParkGantry action still uses the HomeAxes() program to park the gantry.
// At some stage, this program should be changed to MoveAxes(Linkage,Target) because this
// process is faster. The issue is CML library computes a curve based on the start position
// end position, and moves based on the curve. The end position of the curve is not necessarily
// the exact position we want.

void GParkGantryActionNT::ActionThread(const drama::sds::Id &Arg)
{

   UnblockSIGUSR2();

   // lliu added on 15/05/2024
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());

   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   drama::sds::Id parSysId(drama::sds::Id::CreateFromSdsIdType((long)(DitsGetParId())));

   if (details == nullptr || details->Initialised == NO)
   {
      MessageUser("G_PARK_NT: TdFCanTask is not initialised. Please use \"ditscmd TdFCanTask INITIALISE\" to initialise the task.");
      return;
   }
   if (!(ThisTask->tdFfpiIlocks(ILOCK__GAN_PARKED)))
   {
      MessageUser("G_PARK_NT: the gantry is already parked.");
      return;
   }
   if (details->inUse)
   {
      MessageUser("G_PARK_NT: TdFCanTask is running other actions.\n");
      return;
   }

   std::string Axes("");
   std::string Positions;
   std::string Velocities;

   if (Arg)
   {
      drama::gitarg::String AxesArg(this, Arg, "AXES", 1);
      Axes = AxesArg;
   }
   MessageUser("G_PARK_NT: " + Axes);
   SetupPosAndVel(Axes, Positions, Velocities, true);
   bool X, Y, Z, Theta, Jaw;
   if (WhichAxes(Axes, X, Y, Z, Theta, Jaw))
   {
      if (X)
         MessageUser("Park X axis");
      if (Y)
         MessageUser("Park Y axis");
      if (Z)
         MessageUser("Park Z axis");
      if (Theta)
         MessageUser("Park Theta axis");
      if (Jaw)
         MessageUser("Park Jaw axis");

      if (!(ThisTask->SetupAmps()))
      {
         MessageUser("G_PARK_NT: " + ThisTask->GetError());
      }
      else
      {
         details->inUse = YES;
         std::string Error;
         std::vector<AxisDemand> AxisDemands = GetDemands(Axes, Positions, Velocities, Error);
         if (!(ThisTask->MoveAxes(AxisDemands)))
         {
            MessageUser("G_PARK_NT: " + ThisTask->GetError());
         }
         else
         {
            MessageUser("G_PARK_NT: Axes parked");
            MessageUser("G_PARK_NT: Waiting to receive signals about the current position of the gantry.");
            parSysId.Put("PARKED", "YES");
            std::this_thread::sleep_for(2000ms);
            if (!(ThisTask->tdFfpiUpdatePos(YES, details->dprFeedback, YES)))
            {
               MessageUser("G_PARK_NT: Failed to update the position of gantry.");
            }
         }
         details->inUse = NO;
      }
   }
   else
   {
      MessageUser("G_PARK_NT: Invalid axis specification");
   }
}

//  ------------------------------------------------------------------------------------------------

//         G  Home   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
// Currently the G_HOME Action doesn't work properly. It will throw out an error when the G_HOME is invoked, indicating
// there should have been an interlock check. It is very strange that G_HOME function uses the same code as G_INIT_NT.
// G_HOME fails to process the Home action whilst G_INIT_NT functions properly. The error may result from the thread version drama?
// Need to check if the non-threaded version works.

void GHomeActionNT::ActionThread(const drama::sds::Id &Arg)
{

   UnblockSIGUSR2();

   // lliu added on 15/05/2024
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());
   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   drama::sds::Id parSysId(drama::sds::Id::CreateFromSdsIdType((long)(DitsGetParId())));

   if (details == nullptr || details->Initialised == NO)
   {
      MessageUser("G_HOME_NT: TdFCanTask is not initialised. Please use \"ditscmd TdFCanTask INITIALISE\" to initialise the task.");
      return;
   }
   if (!(ThisTask->tdFfpiIlocks(ILOCK__GAN_PARKED)))
   {
      MessageUser("G_HOME_NT: the gantry is already homed.");
      return;
   }
   if (details->inUse)
   {
      MessageUser("G_HOME_NT: TdFCanTask is running other actions.\n");
      return;
   }

   std::string Axes("");

   if (Arg)
   {
      drama::gitarg::String AxesArg(this, Arg, "AXES", 1);
      Axes = AxesArg;
   }
   MessageUser("G_HOME_NT: " + Axes);
   bool X, Y, Z, Theta, Jaw;
   if (WhichAxes(Axes, X, Y, Z, Theta, Jaw))
   {
      if (X)
         MessageUser("Home X axis");
      if (Y)
         MessageUser("Home Y axis");
      if (Z)
         MessageUser("Home Z axis");
      if (Theta)
         MessageUser("Home Theta axis");
      if (Jaw)
         MessageUser("Home Jaw axis");

      if (!(ThisTask->SetupAmps()))
      {
         MessageUser("G_HOME_NT: " + ThisTask->GetError());
      }
      else
      {
         details->inUse = YES;

         if (!(ThisTask->HomeAxes(X, Y, Z, Theta, Jaw)))
         {
            MessageUser("G_HOME_NT: " + ThisTask->GetError());
         }
         else
         {
            MessageUser("G_HOME_NT: Axes homed");
            MessageUser("G_HOME_NT: Waiting to receive signals about the current position of the gantry.");
            parSysId.Put("PARKED", "YES");
            std::this_thread::sleep_for(2000ms);
            if (!(ThisTask->tdFfpiUpdatePos(YES, details->dprFeedback, YES)))
            {
               MessageUser("G_HOME_NT: Failed to update the position of gantry.");
            }
         }
         details->inUse = NO;
      }
   }
   else
   {
      MessageUser("G_HOME_NT: Invalid axis specification");
   }
}

//  ------------------------------------------------------------------------------------------------

//         G  UnPark   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
void GUnParkActionNT::ActionThread(const drama::sds::Id &Arg)
{
   UnblockSIGUSR2();
   std::string Axes("");
   std::string Positions;
   std::string Velocities;

   // lliu added on 15/05/2024
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());

   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   drama::sds::Id parSysId(drama::sds::Id::CreateFromSdsIdType((long)(DitsGetParId())));

   if (details == nullptr || details->Initialised == NO)
   {
      MessageUser("G_UNPARK_NT: TdFCanTask is not initialised. Please use \"ditscmd TdFCanTask INITIALISE\" to initialise the task.");
      return;
   }
   if (!(ThisTask->tdFfpiIlocks(ILOCK__GAN_NOT_PARKED)))
   {
      MessageUser("G_UNPARK_NT: the gantry is already not-parked.");
      return;
   }
   if (details->inUse)
   {
      MessageUser("G_UNPARK_NT: TdFCanTask is running other actions.\n");
      return;
   }

   if (Arg)
   {
      drama::gitarg::String AxesArg(this, Arg, "AXES", 1);
      Axes = AxesArg;
   }
   MessageUser("G_UNPARK_NT: " + Axes);
   SetupPosAndVel(Axes, Positions, Velocities, false);
   bool X, Y, Z, Theta, Jaw;
   if (WhichAxes(Axes, X, Y, Z, Theta, Jaw))
   {
      if (X)
         MessageUser("UnPark X axis");
      if (Y)
         MessageUser("UnPark Y axis");
      if (Z)
         MessageUser("UnPark Z axis");
      if (Theta)
         MessageUser("UnPark Theta axis");
      if (Jaw)
         MessageUser("UnPark Jaw axis");

      if (!(ThisTask->SetupAmps()))
      {
         MessageUser("G_UNPARK_NT: " + ThisTask->GetError());
      }
      else
      {
         details->inUse = YES;

         std::string Error;
         std::vector<AxisDemand> AxisDemands = GetDemands(Axes, Positions, Velocities, Error);
         if (!(ThisTask->MoveAxes(AxisDemands)))
         {
            MessageUser("G_UNPARK_NT: " + ThisTask->GetError());
         }
         else
         {
            MessageUser("G_UNPARK_NT: Axes unparked");
            MessageUser("G_UNPARK_NT: Waiting to receive signals about the current position of the gantry.");
            parSysId.Put("PARKED", "NO");
            std::this_thread::sleep_for(2000ms);
            if (!(ThisTask->tdFfpiUpdatePos(YES, details->dprFeedback, YES)))
            {
               MessageUser("G_UNPARK_NT: Failed to update the position of gantry.");
            }
         }

         details->inUse = NO;
      }
   }
   else
   {
      MessageUser("G_HOME_NT: Invalid axis specification");
   }
}

//  ------------------------------------------------------------------------------------------------

//         G  MoveOffset   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
// The G_MOVEOFFSET Action moves an offset amount from its current position. This is a non-thread version.

void GMoveOffsetActionNT::ActionThread(const drama::sds::Id &Arg)
{
   UnblockSIGUSR2();
   // lliu added on 15/05/2024
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());
   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();

   if (details == nullptr || details->Initialised == NO)
   {
      MessageUser("G_MOVEOFFSET_NT: TdFCanTask is not initialised. Please use \"ditscmd TdFCanTask INITIALISE\" to initialise the task.");
      return;
   }
   if (!(ThisTask->tdFfpiIlocks(ILOCK__GAN_PARKED)))
   {
      MessageUser("G_MOVEOFFSET_NT: the gantry is parked.");
      return;
   }
   if (details->inUse)
   {
      MessageUser("G_MOVEOFFSET_NT: TdFCanTask is running other actions.\n");
      return;
   }

   std::string Axes;
   std::string Positions;
   std::string Velocities;
   std::string OffsetFlag = "";
   if (Arg)
   {
      //  I didn't want a try block here. I just didn't want an exception - I wanted Git
      //  to use the default values I specified if there wasn't one in the arguments.
      //  But I can't work out how to do that.
      try
      {
         drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
         drama::gitarg::String AxesArg(this, Arg, "AXES", 1, "", NoFlags);
         Axes = AxesArg;
         drama::gitarg::String PositionsArg(this, Arg, "POSITIONS", 2, "", NoFlags);
         Positions = PositionsArg;
         drama::gitarg::String VelocitiesArg(this, Arg, "VELOCITIES", 3, "", NoFlags);
         Velocities = VelocitiesArg;
         drama::gitarg::String OffsetFlagArg(this, Arg, "OFFSET", 4, "", NoFlags);
         OffsetFlag = OffsetFlagArg;
      }
      catch (...)
      {
      }
   }

   bool MoveBackward = false;
   if (!OffsetFlag.compare("-"))
   {
      MoveBackward = true;
   }

   std::string Error;
   std::vector<AxisDemand> AxisDemands = GetDemands(Axes, Positions, Velocities, Error);
   int NumberAxes = AxisDemands.size();
   if (NumberAxes <= 0)
   {
      MessageUser("G_MOVEOFFSET_NT: " + Error);
   }
   else
   {
      details->inUse = YES;
      unsigned int NumberAxes = AxisDemands.size();
      for (unsigned int Index = 0; Index < NumberAxes; Index++)
      {
         if (MoveBackward == true)
            AxisDemands[Index].Position *= (-1.0);
         DEBUG("AxisId: %d, offset position %f, velocity %f\n", AxisDemands[Index].AxisId,
               AxisDemands[Index].Position, AxisDemands[Index].Velocity);
      }

      if (!(ThisTask->SetupAmps()))
      {
         MessageUser("G_MOVEOFFSET_NT: " + ThisTask->GetError());
      }
      else
      {
         if (!(ThisTask->MoveAxes(AxisDemands, true)))
         {
            MessageUser("G_MOVEOFFSET_NT: " + ThisTask->GetError());
         }
         else
         {
            MessageUser("G_MOVEOFFSET_NT: Move complete.");
            MessageUser("G_MOVEOFFSET_NT: Waiting to receive signals about the current position of the gantry.");
            std::this_thread::sleep_for(2000ms);
            if (!(ThisTask->tdFfpiUpdatePos(YES, details->dprFeedback, YES)))
            {
               MessageUser("G_MOVEOFFSET_NT: Failed to update the position of gantry.");
            }
         }
      }
      details->inUse = NO;
   }
}

//  ------------------------------------------------------------------------------------------------

//         G  Exit   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
void GEXITActionNT::ActionThread(const drama::sds::Id &)
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());

   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();

   if (details == nullptr || details->Initialised == NO)
   {
      MessageUser("G_EXIT: TdFCanTask is not initialised. Please use \"ditscmd TdFCanTask INITIALISE\" to initialise the task.");
      return;
   }
   if (details->inUse)
   {
      MessageUser("G_EXIT: TdFCanTask is running other actions.\n");
      return;
   }

   if (ThisTask->DisableAmps() == true)
   {
      DEBUG("Disable amps OK\n");
      MessageUser("G_EXIT: All amps have been disabled ");
   }
   else
   {
      DEBUG("DisableAmps fails\n");
      MessageUser("G_EXIT: " + ThisTask->GetError());
   }

   ThisTask->tdFGetCameraPath().GetPath(this);
   ThisTask->tdFGetCameraPath().Obey(this, "EXIT");
   DEBUG("EXIT camera OK\n");
   return;
}

//  ------------------------------------------------------------------------------------------------

//         G  Move   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
void GMOVEActionNT::ActionThread(const drama::sds::Id &Arg)
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());

   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();

   if (details == nullptr || details->Initialised == NO)
   {
      MessageUser("G_MOVE_NT: TdFCanTask is not initialised. Please use \"ditscmd TdFCanTask INITIALISE\" to initialise the task.");
      return;
   }
   if (!(ThisTask->tdFfpiIlocks(ILOCK__GAN_PARKED)))
   {
      MessageUser("G_MOVE_NT: the gantry is parked, please unpark the gantry first.");
      return;
   }
   if (details->inUse)
   {
      MessageUser("G_MOVE_NT: TdFCanTask is running other actions.\n");
      return;
   }

   std::string Axes;
   std::string Positions;
   std::string Velocities;
   std::string OffsetFlag;
   if (Arg)
   {
      //  I didn't want a try block here. I just didn't want an exception - I wanted Git
      //  to use the default values I specified if there wasn't one in the arguments.
      //  But I can't work out how to do that.
      try
      {
         drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
         drama::gitarg::String AxesArg(this, Arg, "AXES", 1, "", NoFlags);
         Axes = AxesArg;
         drama::gitarg::String PositionsArg(this, Arg, "POSITIONS", 2, "", NoFlags);
         Positions = PositionsArg;
         drama::gitarg::String VelocitiesArg(this, Arg, "VELOCITIES", 3, "", NoFlags);
         Velocities = VelocitiesArg;
         drama::gitarg::String OffsetFlagArg(this, Arg, "OFFSET", 4, "", NoFlags);
         OffsetFlag = OffsetFlagArg;
      }
      catch (...)
      {
      }
   }
   bool MoveOffset = false;
   bool MoveBackward = false;
   if (!OffsetFlag.compare("+") || !OffsetFlag.compare("-"))
   {
      MoveOffset = true;
      if (!OffsetFlag.compare("-"))
         MoveBackward = true;
   }
   if (Axes.size() > 1 || Velocities.find(",") != string::npos || Positions.find(",") != string::npos)
   {
      MessageUser("G_MOVE_NT can only work on one axis. If more than one axes need to move, please choose G_MOVE_AXIS_NT.\n");
      return;
   }
   std::string Error;
   std::vector<AxisDemand> AxisDemands = GetDemands(Axes, Positions, Velocities, Error);
   int NumberAxes = AxisDemands.size();
   if ((NumberAxes != 1 && Axes != "X"))
   {
      MessageUser("G_MOVE_NT can only work on one axis. If more than one axes need to move, please choose G_MOVE_AXIS_NT.\n" + Error);
   }
   else
   {
      unsigned int NumberAxes = AxisDemands.size();
      for (unsigned int Index = 0; Index < NumberAxes; Index++)
      {
         if (MoveBackward)
            AxisDemands[Index].Position *= (-1.0);
         DEBUG("AxisId: %d, position %f, velocity %f\n", AxisDemands[Index].AxisId,
               AxisDemands[Index].Position, AxisDemands[Index].Velocity);
      }
      if (!(ThisTask->SetupAmps()))
      {
         MessageUser("G_MOVE_NT: " + ThisTask->GetError());
      }
      else
      {
         if (!(ThisTask->MoveAxes(AxisDemands, MoveOffset)))
         {
            MessageUser("G_MOVE_NT: " + ThisTask->GetError());
         }
         else
         {
            MessageUser("G_MOVE_NT: Move complete");
            MessageUser("G_MOVE_NT: Waiting to receive signals about the current position of the gantry.");
            std::this_thread::sleep_for(2000ms);
            if (!(ThisTask->tdFfpiUpdatePos(YES, details->dprFeedback, YES)))
            {
               MessageUser("G_MOVE_NT: Failed to update the position of gantry.");
            }
         }
      }
   }
}

//  ------------------------------------------------------------------------------------------------

//         G  Reset   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
void GRESETActionNT::ActionThread(const drama::sds::Id &)
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());
   drama::sds::Id parSysId(drama::sds::Id::CreateFromSdsIdType((long)(DitsGetParId())));

   parSysId.Put("ENQ_VER_NUM", TdFCanTaskVersion);
   parSysId.Put("ENQ_VER_DATE", TdFCanTaskDate);
   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   if (details == nullptr)
   {
      details = (tdFfpiTaskType *)malloc(sizeof(tdFfpiTaskType));
   }
   drama::sds::Id id;        /* Parameter id                    */
   unsigned long int length; /* Length of parameter item        */
   short check = SHOW;

   id = parSysId.Find("ZEROCAM_CENWAIT");
   id.Pointer(&details->pars.zeroCamCenWait, &length);
   if (length != sizeof(*(details->pars.zeroCamCenWait)))
   {

      DEBUG("Parameter ZEROCAM_CENWAIT length mismatch");
      return;
   }
   length = 0;

   id = parSysId.Find("PLT1_CFID_OFF_X");
   id.Pointer(&details->pars.plt1CenterFidOffsetX, &length);
   if (length != sizeof(*(details->pars.plt1CenterFidOffsetX)))
   {
      DEBUG("Parameter PLT1_CFID_OFF_X length mismatch");
      return;
   }
   length = 0;

   id = parSysId.Find("PLT1_CFID_OFF_Y");
   id.Pointer(&details->pars.plt1CenterFidOffsetY, &length);
   if (length != sizeof(*(details->pars.plt1CenterFidOffsetY)))
   {
      DEBUG("Parameter PLT1_CFID_OFF_Y length mismatch");
      return;
   }
   length = 0;

   details->inUse = YES;

   bool defRead = ThisTask->tdFfpiDefRead(DEFS_FILE | FLEX_FILE,
                                          check);
   if (defRead == false)
   {
      details->inUse = NO;
      return;
   }

   details->ipsMode = 0;
   details->dprAddress = ERROR;
   details->toEnc.x = 0;
   details->toEnc.y = 0;

   details->Initialised = YES;
   DEBUG("Reset tdFfpiTaskType OK\n");

   if (ThisTask->DisableAmps() == true)
   {
      DEBUG("Disable all the amps, now reset the amps\n");
      if (ThisTask->SetupAmps() == true)
      {
         DEBUG("Reset amps OK\n");
      }
      else
      {
         DEBUG("Reset Amps fails\n");
         MessageUser("G_RESET: " + ThisTask->GetError());
      }
   }
   else
   {
      DEBUG("Reset Amps fails\n");
      MessageUser("G_RESET: " + ThisTask->GetError());
   }
   details->inUse = NO;
}

//  ------------------------------------------------------------------------------------------------

//         P  TELPOS   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
void PTELPOSActionNT::ActionThread(const drama::sds::Id &Arg)
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());
   ThisTask->ClearError();

   drama::ParId paramHA(GetTask(), "HA");
   drama::ParId paramDEC(GetTask(), "DEC");

   string HA = "";
   string DEC = "";
   if (Arg)
   {
      try
      {
         drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
         drama::gitarg::String HAArg(this, Arg, "HA", 1, "", NoFlags);
         HA = HAArg;
         drama::gitarg::String DECArg(this, Arg, "DEC", 2, "", NoFlags);
         DEC = DECArg;
      }
      catch (...)
      {
      }
   }
   if (!HA.empty() && !DEC.empty())
   {
      double dHA = std::stod(HA);
      double dDEC = std::stod(DEC);
      DEBUG("Before the update, the current HA is:\n");
      paramHA.List();
      DEBUG("Before the update, the current DEC is:\n");
      paramDEC.List();

      paramHA.Put(dHA);
      paramDEC.Put(dDEC);

      DEBUG("After the update, the current HA is:\n");
      paramHA.List();
      DEBUG("After the update, the current DEC is:\n");
      paramDEC.List();
   }
   else
   {
      DEBUG("P_TELPOS please check your input arguments\n");
      MessageUser("P_TELPOS: " + ThisTask->GetError());
   }
}

//  ------------------------------------------------------------------------------------------------

//         P  Set   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
void PSetActionNT::ActionThread(const drama::sds::Id &Arg)
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());
   ThisTask->ClearError();

   drama::ParSys parSys(GetTask()); // this is duplicated;
   drama::sds::Id parSysId(drama::sds::Id::CreateFromSdsIdType((long)(DitsGetParId())));

   string ParameterName = "";
   string ParameterValue = "";
   if (Arg)
   {
      try
      {
         drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
         drama::gitarg::String ParameterNameArg(this, Arg, "ParameterName", 1, "", NoFlags);
         ParameterName = ParameterNameArg;
         drama::gitarg::String ParameterValueArg(this, Arg, "ParameterValue", 2, "", NoFlags);
         ParameterValue = ParameterValueArg;
      }
      catch (exception &e)
      {
         MessageUser("P_SET: " + string(e.what()));
         return;
      }
   }
   if (ParameterName.find(',') != string::npos || ParameterName.find(' ') != string::npos || ParameterValue.find(',') != string::npos || ParameterValue.find(' ') != string::npos)
   {
      MessageUser("P_SET: please check the name of parameter, this action only supports a single parameter.\n");
      return;
   }
   if (!ParameterName.empty() && !ParameterValue.empty())
   {
      if (parSys.Exists(ParameterName))
      {
         drama::ParId paramOriginal(GetTask(), ParameterName);
         DEBUG("Before the Parameter Setting, the original val is:\n");
         paramOriginal.List();
         if (ThisTask->SetParameter(ParameterName, ParameterValue))
         {
            DEBUG("After the Parameter Setting, the new val is:\n");
            paramOriginal.List();
            MessageUser("P_SET: Set Parameter Complete.\n");
         }
         else
         {
            MessageUser("P_SET: " + ThisTask->GetError());
         }
      }
      else
      {
         MessageUser("P_SET: the parameter does not exist. Please use ditscmd _ALL_ to check the current parameters\n");
      }
   }
}

//  ------------------------------------------------------------------------------------------------

//         P  ResetLock   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
void PResetLockActionNT::ActionThread(const drama::sds::Id &)
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());
   ThisTask->ClearError();
   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   if (details != nullptr)
   {
      details->inUse = NO;
      MessageUser("P_RESET_LOCK: Reset internal lock completed.\n");
   }
   else
   {
      MessageUser("P_RESET_LOCK: the structure pointer is null, please initialise the task first.\n");
   }
}

//  ------------------------------------------------------------------------------------------------

//         P  SetCoeffs   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
void PSetCoeffsActionNT::ActionThread(const drama::sds::Id &Arg)
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());
   ThisTask->ClearError();
   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();

   string CoffsVal;
   string SaveFlag;
   int j = 0;
   short check = 0;
   if (details != nullptr)
   {
      if (Arg)
      {
         try
         {
            drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
            drama::gitarg::String CoffsValArg(this, Arg, "CoffsVal", 1, "", NoFlags);
            CoffsVal = CoffsValArg;
            drama::gitarg::String SaveFlagArg(this, Arg, "SaveFlag", 2, "", NoFlags);
            SaveFlag = SaveFlagArg;
         }
         catch (exception &e)
         {
            MessageUser("P_SET_COEFFS: " + string(e.what()));
            return;
         }
         std::vector<string> ParameterVec = SplitString(CoffsVal);
         if ((int)ParameterVec.size() == 6)
         {
            DEBUG("The Coeffs array is:\n");
            for (int index = 0; index < 6; index++)
            {
               details->convert.coeffs[index] = stod(ParameterVec[index]);
               DEBUG("%f ", details->convert.coeffs[index]);
            }
            DEBUG("\n");
            slaInvf(details->convert.coeffs, details->convert.invCoeffs, &j);
            DEBUG("The Inverse Coeffs array is:\n");
            if (j != 0)
            {
               for (int index = 0; index < 6; index++)
                  details->convert.invCoeffs[index] = details->convert.coeffs[index];
            }
            for (int index = 0; index < 6; index++)
            {
               DEBUG("%f ", details->convert.invCoeffs[index]);
            }
            DEBUG("\n");
            if (SaveFlag == "SAVE" || SaveFlag == "save")
            {
               if (!ThisTask->tdFfpiDefWrite(DEFS_FILE + FLEX_FILE, check))
               {
                  MessageUser("P_SET_COEFFS: Save Coffs failed " + ThisTask->GetError());
               }
            }
            else
               MessageUser("P_SET_COEFFS: Set Coffs completed.\n");
         }
         else
         {
            MessageUser("P_SET_COEFFS: Invalid input arguments. Require 6 input arguments.\n");
         }
      }
      else
      {
         MessageUser("P_SET_COEFFS: No input arguments.\n");
      }
   }
   else
   {
      MessageUser("P_SET_COEFFS: the structure pointer is null, please initialise the task first.\n");
   }
}

//  ------------------------------------------------------------------------------------------------

//         P  SetImage   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
void PSetImageActionNT::ActionThread(const drama::sds::Id &Arg)
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());

   ThisTask->ClearError();
   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   if (details == nullptr)
   {
      MessageUser("P_SET_IMAGE: the structure pointer is null, please initialise the task first.\n");
      return;
   }

   if (Arg)
   {

      double coeffs[6] = {0.0};
      long int bias;
      int j = 0;
      short check = 0;
      drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
      drama::gitarg::String ImageArg(this, Arg, "image", 1, "FREE", NoFlags);
      drama::gitarg::Int<0, 255> BiasArg(this, Arg, "bias", 2, 0, NoFlags);
      bias = BiasArg;

      drama::gitarg::String Coeffs1Arg(this, Arg, "camCoeffs0", 3, "", NoFlags);
      coeffs[0] = (Coeffs1Arg.empty() ? 0.0 : stod(Coeffs1Arg));
      drama::gitarg::String Coeffs2Arg(this, Arg, "camCoeffs1", 4, "", NoFlags);
      coeffs[1] = (Coeffs2Arg.empty() ? 0.0 : stod(Coeffs2Arg));
      drama::gitarg::String Coeffs3Arg(this, Arg, "camCoeffs2", 5, "", NoFlags);
      coeffs[2] = (Coeffs3Arg.empty() ? 0.0 : stod(Coeffs3Arg));
      drama::gitarg::String Coeffs4Arg(this, Arg, "camCoeffs3", 6, "", NoFlags);
      coeffs[3] = (Coeffs4Arg.empty() ? 0.0 : stod(Coeffs4Arg));
      drama::gitarg::String Coeffs5Arg(this, Arg, "camCoeffs4", 7, "", NoFlags);
      coeffs[4] = (Coeffs5Arg.empty() ? 0.0 : stod(Coeffs5Arg));
      drama::gitarg::String Coeffs6Arg(this, Arg, "camCoeffs5", 8, "", NoFlags);
      coeffs[5] = (Coeffs6Arg.empty() ? 0.0 : stod(Coeffs6Arg));

      details->freeImg.bias = (int)bias;
      DEBUG("The free image bias is %d\n", details->freeImg.bias);
      DEBUG("The camCoeffs array is:\n");
      for (int counter = 0; counter < 6; counter++)
      {
         details->freeImg.camCoeffs[counter] = coeffs[counter];
         DEBUG("%f ", details->freeImg.camCoeffs[counter]);
      }
      DEBUG("\n");

      DEBUG("The Inverse camCoeffs array is:\n");
      slaInvf(details->freeImg.camCoeffs, details->freeImg.invCoeffs, &j);
      if (j != 0)
      {
         int i;
         for (i = 0; i < 6; i++)
            details->freeImg.invCoeffs[i] = details->freeImg.camCoeffs[i];
      }
      for (int i = 0; i < 6; i++)
      {
         DEBUG("%f ", details->freeImg.invCoeffs[i]);
      }
      DEBUG("\n");

      if (!ThisTask->tdFfpiDefWrite(DEFS_FILE, check))
      {
         MessageUser("P_SET_IMAGE: Save Image failed " + ThisTask->GetError());
      }
      else
         MessageUser("P_SET_IMAGE: Set Image completed.\n");
   }
   else
   {
      MessageUser("P_SET_IMAGE: No input arguments.\n");
   }
}

//  ------------------------------------------------------------------------------------------------

//         P  SetWindow   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
void PSetWindowActionNT::ActionThread(const drama::sds::Id &Arg)
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());

   ThisTask->ClearError();
   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   if (details == nullptr)
   {
      MessageUser("P_SET_WINDOW: the structure pointer is null, please initialise the task first.\n");
      return;
   }
   if (Arg)
   {
      std::string SearchStr = "SEARCH";
      double dXCen = 0.0, dYCen = 0.0;
      long int iXPan, iYPan;

      drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
      drama::gitarg::String WindowArg(this, Arg, "window", 1, "SEARCH", NoFlags);
      if (WindowArg == "SEARCH" || WindowArg == "NORM" || WindowArg == "FULL")
         SearchStr = WindowArg;
      drama::gitarg::String xCenArg(this, Arg, "xCen", 2, "0.0", NoFlags);
      double xCent = stod(xCenArg);
      if (xCent >= 0 && xCent <= CAM_X_SPAN)
         dXCen = xCent;
      drama::gitarg::String yCenArg(this, Arg, "yCen", 3, "0.0", NoFlags);
      double yCent = stod(yCenArg);
      if (yCent >= 0 && yCent <= CAM_Y_SPAN)
         dYCen = yCent;
      drama::gitarg::Int<0, 2 * CAM_X_SPAN> xSpanArg(this, Arg, "xSpan", 4, 0, NoFlags);
      iXPan = xSpanArg;
      drama::gitarg::Int<0, 2 * CAM_Y_SPAN> ySpanArg(this, Arg, "ySpan", 5, 0, NoFlags);
      iYPan = ySpanArg;

      if (SearchStr == "SEARCH")
      {
         details->searchWin.xCen = dXCen;
         details->searchWin.yCen = dYCen;
         details->searchWin.xSpan = iXPan;
         details->searchWin.ySpan = iYPan;
      }
      else
      {
         details->normWin.xCen = dXCen;
         details->normWin.yCen = dYCen;
         details->normWin.xSpan = iXPan;
         details->normWin.ySpan = iYPan;
      }
      if (!ThisTask->tdFfpiDefWrite(DEFS_FILE, 0))
      {
         MessageUser("P_SET_WINDOW: Save Window failed " + ThisTask->GetError());
      }
      else
         MessageUser("P_SET_WINDOW: Set Window completed.\n");
   }
   else
   {
      MessageUser("P_SET_WINDOW: No input arguments.\n");
   }
}

//  ------------------------------------------------------------------------------------------------

//         P  SetVel   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
void PSetVelActionNT::ActionThread(const drama::sds::Id &Arg)
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());
   ThisTask->ClearError();
   long int iFeedrate;

   if (Arg)
   {
      drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
      drama::gitarg::Int<FPI_XY_FR_MIN, FPI_XY_FR_MAX> feedRateArg(this, Arg, "feedrate", 1, 0, NoFlags);
      iFeedrate = feedRateArg;
      drama::ParId paramFeedRate(GetTask(), "XY_VEL");
      MessageUser("P_SET_VEL: before the setting, the value of XY_VEL is: \n");
      paramFeedRate.List();
      paramFeedRate.Put((int)iFeedrate);
      MessageUser("P_SET_VEL: after the setting, the value of XY_VEL is: \n");
      paramFeedRate.List();

      if (!ThisTask->tdFfpiDefWrite(DEFS_FILE, 0))
      {
         MessageUser("P_SET_VEL: Save XY_VEL failed " + ThisTask->GetError());
      }
      else
         MessageUser("P_SET_VEL: Set VEL completed.\n");
   }
   else
   {
      MessageUser("P_SET_VEL: No input arguments.\n");
   }
}

//  ------------------------------------------------------------------------------------------------

//         P  SetPlate   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
void PSetPlateActionNT::ActionThread(const drama::sds::Id &Arg)
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());
   ThisTask->ClearError();
   long int iPlate;

   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   if (details == nullptr)
   {
      MessageUser("P_SET_PLATE: the structure pointer is null, please initialise the task first.\n");
      return;
   }
   if (Arg)
   {
      drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
      drama::gitarg::Int<-1, 1> plateArg(this, Arg, "PLATE", 1, 0, NoFlags);
      iPlate = plateArg;
      DEBUG("P_SET_PLATE: before the setting, the value of plate is: %hd\n", details->currentPlate);
      details->currentPlate = (short)iPlate;
      DEBUG("P_SET_PLATE: after the setting, the value of plate is: %hd\n", details->currentPlate);
      MessageUser("P_SET_PLATE: Set Plate completed.\n");
   }
   else
   {
      MessageUser("P_SET_PLATE: No input arguments.\n");
   }
}

//  ------------------------------------------------------------------------------------------------

//         P  UpdateFlex   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
void PUpdateFlexActionNT::ActionThread(const drama::sds::Id &)
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());

   ThisTask->ClearError();
   short check = SHOW;
   if (!ThisTask->tdFfpiDefRead(FLEX_FILE, check))
   {
      DEBUG("P_UPDATE_FLEX: failed to read tdfFpiFlex.sds.\n");
      MessageUser("P_UPDATE_FLEX: failed to read tdfFpiFlex.sds " + ThisTask->GetError());
   }
   else
   {
      MessageUser("P_UPDATE_FLEX: Update Flex completed.\n");
   }
}

//  ------------------------------------------------------------------------------------------------

//         P  Update   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
void PUpdateActionNT::ActionThread(const drama::sds::Id &Arg)
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());
   ThisTask->ClearError();

   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   if (details == nullptr || (details && details->Initialised == NO))
   {
      MessageUser("P_UPDATE: the structure pointer is null, please initialise the task first.\n");
      return;
   }
   if (!ThisTask->tdFfpiIlocks(ILOCK__TASK_INIT))
   {
      MessageUser("Unable to execute this action: " + ThisTask->GetError());
      return;
   }

   if (Arg)
   {
      drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
      drama::gitarg::String dTimeoutArg(this, Arg, "timeout", 1, "0.0", NoFlags);
      // dTimeout = stod(dTimeoutArg) > 0.0 ? stod(dTimeoutArg) : 0.0;
      if (!ThisTask->tdFfpiUpdatePos(NO, YES, YES))
      {
         MessageUser("P_POLL_POSE: failed to complete the action, " + ThisTask->GetError());
      }
      else
         MessageUser("P_POLL_POSE: action completed");

      // confirmed with Tony that this action is no longer used by users and he suggested blocking this action.
      // MessageUser("P_POLL_POSE: this action is not supported any more.\n");
   }
   else
   {
      if (!ThisTask->tdFfpiUpdatePos(YES, details->dprFeedback, YES))
      {
         MessageUser("P_UPDATE_POSE: failed to complete the action, " + ThisTask->GetError());
      }
      else
         MessageUser("P_UPDATE_POSE: action completed.\n");
   }
}

//  ------------------------------------------------------------------------------------------------

//         P  Report   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
void PReportActionNT::ActionThread(const drama::sds::Id &Arg)
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());
   ThisTask->ClearError();

   drama::sds::Id parSysId(drama::sds::Id::CreateFromSdsIdType((long)(DitsGetParId())));
   if (Arg)
   {
      drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
      drama::gitarg::String parameterNameArg(this, Arg, "parameter", 1, "", NoFlags);
      string parameterName = parameterNameArg;
      if (parameterName == "_ALL_")
      {
         MessageUser("P_REPORT: report all the parameters.\n");
         parSysId.List();
      }
      else if (parSysId.Exists(parameterName))
      {
         drama::sds::Id parameterId = parSysId.Find(parameterName);
         MessageUser("P_REPORT: report the value of " + parameterName + "\n");
         parameterId.List();
      }
      else
      {
         MessageUser("P_REPORT: the input argument is not valid. The parameter does not exist.\n");
      }
   }
   else
   {
      MessageUser("P_REPORT: no input argument.\n");
   }
   MessageUser("P_REPORT: action completed.\n");
}

//  ------------------------------------------------------------------------------------------------

//         P  ReportLock   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
void PReportLocksActionNT::ActionThread(const drama::sds::Id &)
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());
   ThisTask->ClearError();
   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   if (details == nullptr)
   {
      MessageUser("P_REPORT_LOCKS: the structure pointer is null, please initialise the task!\n");
      return;
   }
   if (details->inUse)
   {
      MessageUser("P_REPORT_LOCKS: LOCK IN USE");
   }
   else
   {
      MessageUser("P_REPORT_LOCKS:LOCK NOT IN USE");
   }
   MessageUser("P_REPORT_LOCKS: action completed.\n");
}

//  ------------------------------------------------------------------------------------------------

//         P  ReportCoeffs   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
void PReportCoeffsActionNT::ActionThread(const drama::sds::Id &)
{
   // UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());
   ThisTask->ClearError();
   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   if (details == nullptr)
   {
      MessageUser("P_REPORT_COEFFS: the structure pointer is null, please initialise the task!\n");
      return;
   }
   DEBUG("The coeffs array is: \n");
   for (int index = 0; index < 6; ++index)
   {
      DEBUG("%f  ", details->convert.coeffs[index]);
   }
   DEBUG("\n");
   MessageUser("P_REPORT_COEFFS: action completed.\n");
}

//  ------------------------------------------------------------------------------------------------

//         P  ReportImage   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
void PReportImageActionNT::ActionThread(const drama::sds::Id &)
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());

   ThisTask->ClearError();
   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   if (details == nullptr)
   {
      MessageUser("P_REPORT_IMAGE: the structure pointer is null, please initialise the task!\n");
      return;
   }
   DEBUG("FREE image details: \n");
   DEBUG("The bias(centroiding) is: %hd\n", (short)details->freeImg.bias);
   DEBUG("The Camera coeffs array is: \n");
   for (int index = 0; index < 6; ++index)
   {
      DEBUG("%f  ", details->freeImg.camCoeffs[index]);
   }
   DEBUG("\n");
   DEBUG("The Inverse coeffs array is: \n");
   for (int index = 0; index < 6; ++index)
   {
      DEBUG("%f  ", details->freeImg.invCoeffs[index]);
   }
   DEBUG("\n");
   MessageUser("P_REPORT_COEFFS: action completed.\n");
}

//  ------------------------------------------------------------------------------------------------

//         P  ReportWindow   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
void PReportWindowActionNT::ActionThread(const drama::sds::Id &Arg)
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());

   ThisTask->ClearError();
   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   if (details == nullptr)
   {
      MessageUser("P_REPORT_WINDOW: the structure pointer is null, please initialise the task!\n");
      return;
   }

   string windowStr = "SEARCH";
   if (Arg)
   {
      drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
      drama::gitarg::String windowArg(this, Arg, "window", 1, "SEARCH", NoFlags);
      if (windowArg == "SEARCH" || windowArg == "NORM" || windowArg == "FULL")
      {
         windowStr = windowArg;
      }
      if (windowStr == "SEARCH")
      {
         DEBUG("SEARCH window details:\n");
         DEBUG("Window centre = %.1f,  %.1f\n", details->searchWin.xCen, details->searchWin.yCen);
         DEBUG("Window span   = %d,  %d\n", details->searchWin.xSpan, details->searchWin.ySpan);
      }
      else
      {
         DEBUG("NORM window details:\n");
         DEBUG("Window centre = %.1f,  %.1f\n", details->normWin.xCen, details->normWin.yCen);
         DEBUG("Window span   = %d,  %d\n", details->normWin.xSpan, details->normWin.ySpan);
      }
   }
   else
   {
      DEBUG("SEARCH window details:\n");
      DEBUG("Window centre = %.1f,  %.1f\n", details->searchWin.xCen, details->searchWin.yCen);
      DEBUG("Window span   = %d,  %d\n", details->searchWin.xSpan, details->searchWin.ySpan);
   }

   MessageUser("P_REPORT_WINDOW: action completed.\n");
}

//  ------------------------------------------------------------------------------------------------

//         P  SaveDefs   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
void PSaveDefsActionNT::ActionThread(const drama::sds::Id &)
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());
   ThisTask->ClearError();
   if (!ThisTask->tdFfpiDefWrite(DEFS_FILE + FLEX_FILE, 0))
   {
      MessageUser("P_SAVE_DEFS: Save DEFS_FILE failed " + ThisTask->GetError());
   }
   MessageUser("P_SAVE_DEFS: action completed.\n");
}

//  ------------------------------------------------------------------------------------------------

//         C  Search   A c t i o n  N T  : :  A c t i o n N T
void CSearchAction::ActionThread(const drama::sds::Id &Arg)
{
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();
   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   if (details == nullptr)
   {
      DramaTHROW(TDFCANTASK__NOTINIT, "C_SEARCH: the structure pointer is null, please initialise the task!");
   }
   if (details->inUse)
   {
      DramaTHROW(TDFCANTASK__IN_USE, "C_SEARCH: TdFCanTask is running other actions.");
   }

   m_tdFfpiSFStruct = new tdFfpiSFtype();
   if (m_tdFfpiSFStruct == nullptr)
   {
      DramaTHROW(TDFCANTASK__MALLOCERR, "C_SEARCH: failed to allocate memory to the SEARCH data structure.");
   }
   if (!Arg)
   {
      DramaTHROW(TDFCANTASK__NO_ARGUMENTS, "C_SEARCH: no input argument.");
   }
   details->inUse = YES;
   drama::ParSys parSysId(ThisTask->TaskPtr());
   drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
   drama::gitarg::Int<XMIN, XMAX> XFArg(this, Arg, "XF", 1, 0, NoFlags);
   long int searchStartX = XFArg;
   long int searchX = searchStartX;
   drama::gitarg::Int<YMIN, YMAX> YFArg(this, Arg, "YF", 2, 0, NoFlags);
   long int searchStartY = YFArg;
   long int searchY = searchStartY;

   int inside, outside, forbidden;
   double settletime;
   double platetheta;

   parSysId.Get("STEP_SIZE", &m_tdFfpiSFStruct->stepSize);
   parSysId.Get("MAX_ERROR", &m_tdFfpiSFStruct->maxError);
   parSysId.Get("POS_TOL", &m_tdFfpiSFStruct->tolerance);
   parSysId.Get("POS_ATTEMPTS", &m_tdFfpiSFStruct->attempts);
   parSysId.Get("SETTLE_TIME", &settletime);
   parSysId.Get("PLATE_THETA", &platetheta);

   m_tdFfpiSFStruct->searchStartX = searchStartX;
   m_tdFfpiSFStruct->searchStartY = searchStartY;

   tdFfpiCENtype *cenWin = new tdFfpiCENtype();
   int i = 1, j = 1, k = 0;                                    /* Used to determine next search point  */
   short atSearchXY = NO,                                      /* Flag - above fibre-end location      */
       centroided = NO,                                        /*      - fibre-end centroided          */
       checkedCentroid = NO,                                   /*      - analysed last centroid        */
       iAttempts = 0,                                          /* Current number of positioning trys   */
       foundIt = NO,                                           /* Have we found the fibre              */
       searchStarted = NO,                                     /* Have we completed our first move     */
       centroidRepeated = NO,                                  /* Have we repeated the centroid        */
       repeatChecked = m_tdFfpiSFStruct->statCheck ? NO : YES; /* Has te repeated centroid been checks */

   double theta = ThisTask->tdFautoThetaPos(m_tdFfpiSFStruct->searchStartX, m_tdFfpiSFStruct->searchStartY);
   forbidden = ThisTask->tdFforbidden(m_tdFfpiSFStruct->searchStartX, m_tdFfpiSFStruct->searchStartY,
                                      theta, QUADRANT_RADIUS, INSIDE_RADIUS,
                                      OUTSIDE_RADIUS, HALF_GUIDE_EXPAN,
                                      JAW_HWP, JAW_HWM, JAW_LENGTH, 0, &inside, &outside);
   if (forbidden || outside)
   {
      details->inUse = NO;
      DramaTHROW_S(TDFCANTASK__INV_POS, "C_SEARCH: Initial search position (%d,%d) invalid.\n", m_tdFfpiSFStruct->searchStartX, m_tdFfpiSFStruct->searchStartX);
   }
   details->imagePos.enable = 1;
   details->imagePos.displayText = 1;
   details->imagePos.useDpr = details->dprFeedback;

   ThisTask->tdFfpiUpdatePos(YES, details->dprFeedback, YES);

   cenWin->window.MaxX = details->freeImg.xMax;
   cenWin->window.MaxY = details->freeImg.yMax;
   cenWin->window.PixelSize = details->freeImg.PixelSize;
   cenWin->window.Xoffset = details->searchWin.xCen - details->searchWin.xSpan / 2;
   cenWin->window.Yoffset = details->searchWin.yCen - details->searchWin.ySpan / 2;
   if (cenWin->window.Xoffset < 0)
      cenWin->window.Xoffset = 0;
   if (cenWin->window.Yoffset < 0)
      cenWin->window.Yoffset = 0;
   cenWin->window.Xdim = details->searchWin.xSpan;
   cenWin->window.Ydim = details->searchWin.ySpan;
   if (cenWin->window.Xdim + cenWin->window.Xoffset > cenWin->window.MaxX)
      cenWin->window.Xdim = cenWin->window.MaxX - cenWin->window.Xoffset;
   if (cenWin->window.Ydim + cenWin->window.Yoffset > cenWin->window.MaxY)
      cenWin->window.Ydim = cenWin->window.MaxY - cenWin->window.Yoffset;
   cenWin->img = &details->freeImg;

   bool flag = true;
   drama::Path cameraPath(ThisTask->TaskPtr(), "VimbaFPI", "", "/instsoft/vimbacam/vimbacam");
   while ((!atSearchXY || !centroided || !checkedCentroid || !centroidRepeated || !repeatChecked))
   {
      if (!atSearchXY)
      {
         if (!MoveToSearchPosition(searchX, searchY, &atSearchXY, &searchStarted))
         {
            MessageUser("C_SEARCH: Fail to move to search position (%d,%d).\n", searchX, searchY);
            flag = false;
            break;
         }
      }
      else if (!centroided)
      {
         if (!PerformCentroid(cameraPath, cenWin, settletime, &centroided))
         {
            MessageUser("C_SEARCH: Fail to perform centroid at position (%d,%d).\n", searchX, searchY);
            flag = false;
            break;
         }
      }
      else if (!checkedCentroid)
      {

         if (!CheckCentroid(details, &iAttempts, &searchX, &searchY, &atSearchXY, &centroided,
                            &foundIt, &i, &j, &k, &checkedCentroid, &centroidRepeated, &repeatChecked))
         {
            MessageUser("C_SEARCH: Fail to check centroid at position (%d,%d).\n", searchX, searchY);
         }
      }
      else if (!centroidRepeated)
      {

         if (!PerformCentroid(cameraPath, cenWin, settletime, &centroidRepeated))
         {
            MessageUser("C_SEARCH: Fail to perform centroid again at position (%d,%d).\n", searchX, searchY);
            flag = false;
            break;
         }
      }
      else if (!repeatChecked)
      {
         if (!CheckRepeatCentroid(details, &centroidRepeated, &repeatChecked))
         {
            MessageUser("C_SEARCH: Fail to check centroid again at position (%d,%d).\n", searchX, searchY);
            flag = false;
            break;
         }
      }
   }

   drama::sds::Id newArg;

   if (flag == true)
   {
      ActionComplete(details, searchX, searchY, foundIt);

      newArg = drama::sds::Id::CreateArgStruct();
      double expXenc, expYenc, atXenc, atYenc;
      ThisTask->tdFfpiConvertFromFP(m_tdFfpiSFStruct->searchStartX, m_tdFfpiSFStruct->searchStartY,
                                    platetheta, _FULL, &expXenc, &expYenc);
      ThisTask->tdFfpiConvertFromFP(searchX, searchY, platetheta, _FULL,
                                    &atXenc, &atYenc);
      newArg.Put("FOUND", foundIt);
      newArg.Put("MEASUREDX", searchX);
      newArg.Put("MEASUREDY", searchY);
      newArg.Put("EncoderX", atXenc);
      newArg.Put("EncoderY", atYenc);
      newArg.Put("STARTX", m_tdFfpiSFStruct->searchStartX);
      newArg.Put("STARTY", m_tdFfpiSFStruct->searchStartY);
      newArg.Put("XErr", m_tdFfpiSFStruct->xErr);
      newArg.Put("YErr", m_tdFfpiSFStruct->yErr);
      newArg.Put("XEncoderErr", doubleToLong(atXenc - expXenc));
      newArg.Put("YEncoderErr", doubleToLong(atYenc - expYenc));
   }
   SetReturnArg(&newArg);
   details->inUse = NO;
   MessageUser("C_SEARCH: action completed.\n");
}

void CSearchAction::CheckCent_ObjectHasNotYetBeenSeen(tdFfpiTaskType *details, long *const searchX, long *const searchY, short *const atSearchXY,
                                                      short *const centroided, short *const attempts, short *const foundIt, int *const i, int *const j, int *const k,
                                                      short *const checkedCentroid, short *const centroidRepeated, short *const repeatChecked)
{
   if (m_tdFfpiSFStruct->centroidOK == YES)
   {
      CheckCent_ObjectFound(details, searchX, searchY, atSearchXY, centroided, attempts, foundIt, checkedCentroid);
   }
   else
   {
      CheckCent_ObjectNotFound(searchX, searchY, atSearchXY, centroided, i, j, k, checkedCentroid, centroidRepeated, repeatChecked);
   }
}

bool CSearchAction::CheckRepeatCentroid(tdFfpiTaskType *details, short *const centroidRepeated, short *const repeatChecked)
{
   if (m_tdFfpiSFStruct->centroidOK == YES)
   {
      if (m_tdFfpiSFStruct->resultsValid == 0)
         MessageUser("CheckRepeatCentroid: Found object within tolerance - will repeat centroid 10 times");

      m_tdFfpiSFStruct->resultsX[m_tdFfpiSFStruct->resultsValid] = details->imagePos.p.x - m_tdFfpiSFStruct->xErr;
      m_tdFfpiSFStruct->resultsY[m_tdFfpiSFStruct->resultsValid] =
          details->imagePos.p.y - m_tdFfpiSFStruct->yErr;
      ++(m_tdFfpiSFStruct->resultsValid);

      int index = m_tdFfpiSFStruct->resultsValid - 1;
      MessageUser("CheckRepeatCentroid: Search Object found at position %ld, %ld (in tolerance image %d)",
                  m_tdFfpiSFStruct->resultsX[index], m_tdFfpiSFStruct->resultsY[index], m_tdFfpiSFStruct->resultsValid);

#define SEARCH_TOL 4 /* A difference of this amount in 3 centroids is ignored */
      if (m_tdFfpiSFStruct->resultsValid == 3)
      {
         if ((labs(m_tdFfpiSFStruct->resultsX[0] - m_tdFfpiSFStruct->resultsX[1]) <= SEARCH_TOL) &&
             (labs(m_tdFfpiSFStruct->resultsX[0] - m_tdFfpiSFStruct->resultsX[2]) <= SEARCH_TOL) &&
             (labs(m_tdFfpiSFStruct->resultsY[0] - m_tdFfpiSFStruct->resultsY[1]) <= SEARCH_TOL) &&
             (labs(m_tdFfpiSFStruct->resultsY[0] - m_tdFfpiSFStruct->resultsY[2]) <= SEARCH_TOL))
         {
            *repeatChecked = YES;
            MessageUser("CheckRepeatCentroid: First three images give the same result - won't do the other seven");
         }
         else
         {
            *centroidRepeated = NO;

            MessageUser("CheckRepeatCentroid: First three images on thsi object give different results - will do 10");
            MessageUser("CheckRepeatCentroid: Deltas %ld,%ld - %ld,%ld", labs(m_tdFfpiSFStruct->resultsX[0] - m_tdFfpiSFStruct->resultsX[1]),
                        labs(m_tdFfpiSFStruct->resultsY[0] - m_tdFfpiSFStruct->resultsY[1]), labs(m_tdFfpiSFStruct->resultsX[0] - m_tdFfpiSFStruct->resultsX[2]),
                        labs(m_tdFfpiSFStruct->resultsY[0] - m_tdFfpiSFStruct->resultsY[2]));
         }
      }
      else if (m_tdFfpiSFStruct->resultsValid >= SF_IMAGES)
      {
         *repeatChecked = YES;
      }
      else
      {
         *centroidRepeated = NO;
      }
      return true;
   }
   else
   {
      MessageUser("CheckRepeatCentroid: Fibre was found, but is not in a subsequent image.");
      return false;
   }
}

void CSearchAction::CheckCent_ObjectNotFound(long *const searchX, long *const searchY, short *const atSearchXY, short *const centroided,
                                             int *const i, int *const j, int *const k, short *const checkedCentroid,
                                             short *const centroidRepeated, short *const repeatChecked)
{
   long int dist;
   int pi = *i; /* Save values on entry of i,j and k */
   int pj = *j;
   int pk = *k;

   if (*j > *i)
   { /*  A little cryptic, but this just */
      if (*k == 0)
      { /*  gives the next point on the     */
         *j = 1;
         *k = 1; /*  outwards spiral                 */
      }
      else
      {
         (*i)++;
         *j = 1;
         *k = 0;
      }
   }
   if (*k == 0)
      (*searchY += (*i % 2) ? m_tdFfpiSFStruct->stepSize : -m_tdFfpiSFStruct->stepSize);
   else
      (*searchX += (*i % 2) ? m_tdFfpiSFStruct->stepSize : -m_tdFfpiSFStruct->stepSize);
   (*j)++;

   dist = (ABS(*searchX - m_tdFfpiSFStruct->searchStartX) >
           ABS(*searchY - m_tdFfpiSFStruct->searchStartY))
              ? ABS(*searchX - m_tdFfpiSFStruct->searchStartX)
              : ABS(*searchY - m_tdFfpiSFStruct->searchStartY);
   MessageUser("CheckCent_ObjectNotFound: Current search distance is %ld (max %ld). i=%d(%d),j=%d(%d),k=%d(%d)",
               dist, m_tdFfpiSFStruct->maxError, *i, pi, *j, pj, *k, pk);

   if (dist > m_tdFfpiSFStruct->maxError)
   {
      MessageUser("Completed search to distance of %ld microns without finding object.", m_tdFfpiSFStruct->maxError);
      *checkedCentroid = YES;
      *centroidRepeated = YES;
      *repeatChecked = YES;
   }
   else
   {
      *atSearchXY = *centroided = NO;
      if ((pk == 0) && (pi == pj) && ((pi % 2) == 1))
         MessageUser("CheckCent_ObjectNotFound: Searching at distance %ld microns from original position",
                     dist);
   }
}

void CSearchAction::CheckCent_ObjectFound(tdFfpiTaskType *details, long *const searchX, long *const searchY, short *const atSearchXY,
                                          short *const centroided, short *const attempts, short *const foundIt, short *const checkedCentroid)
{
   double error;
   double xErr;
   double yErr;
   *foundIt = YES;
   error = sqrt(SQRD((double)m_tdFfpiSFStruct->xErr) + SQRD((double)m_tdFfpiSFStruct->yErr));

   MessageUser("CheckCent_ObjectFound: Search Image found, error = %.1f (%ld,%ld)", error, m_tdFfpiSFStruct->xErr, m_tdFfpiSFStruct->yErr);

   MessageUser("CheckCent_ObjectFound: Robot position error is %ld, %ld", details->imagePos.p.x - *searchX,
               details->imagePos.p.y - *searchY);

   xErr = m_tdFfpiSFStruct->xErr - (details->imagePos.p.x - *searchX);
   yErr = m_tdFfpiSFStruct->yErr - (details->imagePos.p.y - *searchY);
   error = sqrt(SQRD(xErr) + SQRD(yErr));
   MessageUser("CheckCent_ObjectFound: After subtracting robot pos error, error = %.1f (%ld,%ld)",
               error, (long)xErr, (long)yErr);
   if ((error <= (double)m_tdFfpiSFStruct->tolerance) ||
       (*attempts >= m_tdFfpiSFStruct->attempts))
   {
      *checkedCentroid = YES;
      *searchX = details->imagePos.p.x - m_tdFfpiSFStruct->xErr;
      *searchY = details->imagePos.p.y - m_tdFfpiSFStruct->yErr;
      m_tdFfpiSFStruct->resultsX[0] = *searchX;
      m_tdFfpiSFStruct->resultsY[0] = *searchY;
      m_tdFfpiSFStruct->resultsValid = 1;
      MessageUser("CheckCent_ObjectFound: Search Object found at position %ld, %ld (in tolerance image %d)",
                  m_tdFfpiSFStruct->resultsX[0], m_tdFfpiSFStruct->resultsY[0], m_tdFfpiSFStruct->resultsValid);
   }
   else
   {
      MessageUser("CheckCent_ObjectFound: Object was outside tolerance (error %g, tolerance %ld), try to get closer",
                  error, m_tdFfpiSFStruct->tolerance);
      *atSearchXY = *centroided = NO;
      *searchX = details->imagePos.p.x - m_tdFfpiSFStruct->xErr;
      *searchY = details->imagePos.p.y - m_tdFfpiSFStruct->yErr;
      (*attempts)++;
   }
}

bool CSearchAction::CheckCentroid(tdFfpiTaskType *details, short *const attempts, long *const searchX, long *const searchY, short *const atSearchXY,
                                  short *const centroided, short *const foundIt, int *const i, int *const j, int *const k, short *const checkedCentroid,
                                  short *const centroidRepeated, short *const repeatChecked)
{
   if (*attempts == 0)
   {
      CheckCent_ObjectHasNotYetBeenSeen(details, searchX, searchY,
                                        atSearchXY, centroided, attempts, foundIt, i, j, k,
                                        checkedCentroid, centroidRepeated, repeatChecked);
   }
   else if (m_tdFfpiSFStruct->centroidOK != YES)
   {
      MessageUser("CheckCentroid: Fibre was found, but is no longer in field of view");
      return false;
   }
   else
   {
      CheckCent_ObjectFound(details, searchX, searchY, atSearchXY, centroided,
                            attempts, foundIt, checkedCentroid);
   }
   return true;
}

void CSearchAction::ActionComplete_CalculateMean(long *const searchX, long *const searchY)
{
   double xSum = 0;
   double ySum = 0;
   double xMean = 0;
   double yMean = 0;
   register unsigned i;
   int usedCounter;
   double minX, maxX, minY, maxY;
   double xDev = 0, yDev = 0, xStdDev, yStdDev;

   for (i = 0; i < SF_IMAGES; i++)
   {
      xSum += (double)m_tdFfpiSFStruct->resultsX[i];
      ySum += (double)m_tdFfpiSFStruct->resultsY[i];
   }
   xMean = xSum / (double)SF_IMAGES;
   yMean = ySum / (double)SF_IMAGES;
   /*
    * Round the results into the output variables.
    */
#ifdef VxWorks
   *searchX = iround(xMean); /* iround() rounds a double */
   *searchY = iround(yMean);
#else
   *searchX = rint(xMean); /* rint() rounds a double */
   *searchY = rint(yMean);
#endif

   /*
    *  Work out the standard deviations
    */
   for (i = 0; i < SF_IMAGES; ++i)
   {
      double d;
      d = ((double)m_tdFfpiSFStruct->resultsX[i] - xMean);
      xDev += d * d;
      d = ((double)m_tdFfpiSFStruct->resultsY[i] - yMean);
      yDev += d * d;
   }
   xStdDev = sqrt(xDev / (SF_IMAGES - 1));
   yStdDev = sqrt(yDev / (SF_IMAGES - 1));

#define SF_CUT 1.5
   /*
    *  Get the X and Y ranges
    */
   minX = xMean - xStdDev * SF_CUT;
   maxX = xMean + xStdDev * SF_CUT;

   minY = yMean - yStdDev * SF_CUT;
   maxY = yMean + yStdDev * SF_CUT;
   /*
    *  Calculate a new mean, use only the offsets where both X and
    *  Y values are within 3*StdDev.
    */
   xSum = ySum = 0;
   usedCounter = 0;
   for (i = 0; i < SF_IMAGES; ++i)
   {
      if ((m_tdFfpiSFStruct->resultsX[i] >= minX) &&
          (m_tdFfpiSFStruct->resultsX[i] <= maxX) &&
          (m_tdFfpiSFStruct->resultsY[i] >= minY) &&
          (m_tdFfpiSFStruct->resultsY[i] <= maxY))
      {
         xSum += (double)m_tdFfpiSFStruct->resultsX[i];
         ySum += (double)m_tdFfpiSFStruct->resultsY[i];
         ++usedCounter;
      }
   }

   MessageUser("ActionComplete_CalculateMean: Search Standard Deviations %.1f,%.1f. Rejected %d of %d values",
               xStdDev, yStdDev, SF_IMAGES - usedCounter, SF_IMAGES);

   /*
    * We only really have a sensible value if the number of values left
    * is at least two.
    */
   if (usedCounter >= 2)
   {
      xMean = xSum / (double)usedCounter;
      yMean = ySum / (double)usedCounter;
#ifdef VxWorks
      *searchX = iround(xMean);
      *searchY = iround(yMean);
#else
      *searchX = rint(xMean);
      *searchY = rint(yMean);
#endif

      MessageUser("ActionComplete_CalculateMean: Search resultant averages are %g, %g", xMean, yMean);
   }

   else if (usedCounter == 1)
   {
      MessageUser("ActionComplete_CalculateMean: After search rejected outlier image centroids, there was only one value left.");
   }
   else
   {
      MessageUser("ActionComplete_CalculateMean: After search rejected outliers image centroids, there were no values left.");
   }
}

void CSearchAction::ActionComplete_FoundItCheck(long *const searchX, long *const searchY, short *const foundIt)
{
   long int dist;

   if (m_tdFfpiSFStruct->resultsValid != 3)
      ActionComplete_CalculateMean(searchX, searchY);

   if (ABS(*searchX - m_tdFfpiSFStruct->searchStartX) >
       ABS(*searchY - m_tdFfpiSFStruct->searchStartY))
      dist = ABS(*searchX - m_tdFfpiSFStruct->searchStartX);
   else
      dist = ABS(*searchY - m_tdFfpiSFStruct->searchStartY);

   if (dist > m_tdFfpiSFStruct->maxError)
   {
      MessageUser("ActionComplete_FoundItCheck: Search found object outside search distance - so it is considered not found");
      *foundIt = NO;
   }
}

void CSearchAction::ActionComplete(tdFfpiTaskType *details, long searchX, long searchY, short foundIt)
{
   if (foundIt == YES)
   {
      ActionComplete_FoundItCheck(&searchX, &searchY, &foundIt);
   }

   MessageUser("ActionComplete: Given fibre location  = %ld,%ld", m_tdFfpiSFStruct->searchStartX, m_tdFfpiSFStruct->searchStartY);
   MessageUser("ActionComplete: Current encoder location = %d,%d", details->imagePos.enc.x, details->imagePos.enc.y);
   if (foundIt == YES)
   {
      MessageUser("ActionComplete: Final fibre position  = %ld,%ld", searchX, searchY);
      MessageUser("ActionComplete: Offset from expected     = %ld,%ld", doubleToLong(m_tdFfpiSFStruct->searchStartX - searchX), doubleToLong(m_tdFfpiSFStruct->searchStartY - searchY));
      MessageUser("ActionComplete: Gantry/image-centroid offset = %ld,%ld", m_tdFfpiSFStruct->xErr, m_tdFfpiSFStruct->yErr);
   }
   else
      MessageUser("ActionComplete: Image not found within %ld microns from given location", m_tdFfpiSFStruct->maxError);
}

//  ------------------------------------------------------------------------------------------------

//         C  Centroid   A c t i o n  N T  : :  A c t i o n N T
void CCentroidAction::ActionThread(const drama::sds::Id &Arg)
{
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();
   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   if (details == nullptr)
   {
      DramaTHROW(TDFCANTASK__NOTINIT, "C_CENTROID: the structure pointer is null, please initialise the task!");
      // return;
   }
   if (details->inUse)
   {
      DramaTHROW(TDFCANTASK__IN_USE, "C_CENTROID: TdFCanTask is running other actions.");
      // return;
   }
   if (!Arg) // throw out if there is no argument quickly
   {
      DramaTHROW(TDFCANTASK__NO_ARGUMENTS, "C_CENTROID: No input argument is provided.");
   }
   details->inUse = YES;

   drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
   drama::gitarg::String ImageArg(this, Arg, "Image", 1, "FREE", NoFlags);
   string strImage = "FREE";
   if (ImageArg != "FREE" && ImageArg != "free")
   {
      details->inUse = NO;
      DramaTHROW(TDFCANTASK__INV_INPUT_ARGUMENT, "C_CENTROID: TdFCanTask can only take \"FREE\" Image type.");

      // return;
   }
   drama::gitarg::String WindowArg(this, Arg, "Window", 2, "FULL", NoFlags);
   if (WindowArg != "FULL" && WindowArg != "NORM" && WindowArg != "SEARCH")
   {
      details->inUse = NO;
      DramaTHROW(TDFCANTASK__INV_INPUT_ARGUMENT, "C_CENTROID: TdFCanTask can only take \"FULL\", \"NORM\", \"SEARCH\" Window type.");
      // return;
   }

   string strWindow = WindowArg;
   tdFfpiCENtype *cenData = new tdFfpiCENtype();
   sprintf(cenData->saveName, "%s_%ld", "Centroid", (long int)time(0));
   cenData->window.MaxX = details->freeImg.xMax;
   cenData->window.MaxY = details->freeImg.yMax;
   cenData->window.PixelSize = details->freeImg.PixelSize;
   cenData->img = &details->freeImg;

   details->imagePos.enable = 1;
   details->imagePos.displayText = 1;
   details->imagePos.useDpr = YES;

   if (strWindow == "FULL")
   {
      cenData->window.Xdim = cenData->window.MaxX;
      cenData->window.Ydim = cenData->window.MaxY;
      cenData->window.Xoffset = cenData->window.Yoffset = 0;
   }
   else if (strWindow == "SEARCH")
   {
      cenData->window.Xdim = details->searchWin.xSpan;
      cenData->window.Ydim = details->searchWin.ySpan;
      cenData->window.Xoffset = doubleToLong(details->searchWin.xCen -
                                             details->searchWin.xSpan / 2.0);
      cenData->window.Yoffset = doubleToLong(details->searchWin.yCen -
                                             details->searchWin.ySpan / 2.0);
   }
   else
   {
      cenData->window.Xdim = details->normWin.xSpan;
      cenData->window.Ydim = details->normWin.ySpan;
      cenData->window.Xoffset = doubleToLong(details->normWin.xCen -
                                             details->normWin.xSpan / 2.0);
      cenData->window.Yoffset = doubleToLong(details->normWin.yCen -
                                             details->normWin.ySpan / 2.0);
   }

   if (cenData->window.Xoffset < 0)
      cenData->window.Xoffset = 0;
   if (cenData->window.Yoffset < 0)
      cenData->window.Yoffset = 0;
   if (cenData->window.Xdim + cenData->window.Xoffset > cenData->window.MaxX)
      cenData->window.Xdim = cenData->window.MaxX - cenData->window.Xoffset;
   if (cenData->window.Ydim + cenData->window.Yoffset > cenData->window.MaxY)
      cenData->window.Ydim = cenData->window.MaxY - cenData->window.Yoffset;

   if (cenData->settleTime > 0.0) // this should be renamed to settletime
   {
      std::this_thread::sleep_for(std::chrono::seconds(int(cenData->settleTime)));
   }

   ThisTask->tdFfpiPreExp();
   MessageUser("C_CENTROID: - grabbing image");
   MessageUser("C_CENTROID:  Window size -> Max %ld %ld off %ld %ld, dim %ld %ld",
               cenData->window.MaxX, cenData->window.MaxY,
               cenData->window.Xoffset, cenData->window.Yoffset,
               cenData->window.Xdim, cenData->window.Ydim);

   ThisTask->tdFGetCameraPath().GetPath(this);
   drama::sds::Id messageArg(drama::sds::Id::CreateArgStruct());
   drama::sds::IdPtr returnedArg;
   std::string strWindowType;
   strWindowType = to_string(cenData->window.Xoffset) + ":" + to_string(cenData->window.Yoffset) + ":" + to_string(cenData->window.Xoffset + cenData->window.Xdim - 1) + ":" + to_string(cenData->window.Yoffset + cenData->window.Ydim - 1);

   // windowArg.Put("Xoffset", cenData.window.Xoffset);
   // windowArg.Put("Yoffset", cenData.window.Yoffset);
   // windowArg.Put("XMax", cenData.window.MaxX);
   // windowArg.Put("YMax", cenData.window.MaxY);
   // X1:Y1:X2:Y2---> xoffset, yoffset, xoffset+xdim-1, yoffset+ydim-1;

   // messageArg.Put("Argument1", cenData.img->camNo);
   messageArg.Put("CENTROID", strWindowType);
   messageArg.Put("BIAS", cenData->img->bias);
   messageArg.Put("EXPOSURE_TIME", cenData->img->exposureTime);
   messageArg.Put("SHUTTER_OPEN", (int)cenData->img->shutter);
   messageArg.Put("UPDATE", cenData->img->updateTime);

   ThisTask->tdFGetCameraPath().Obey(this, "CENTRECT", messageArg, &returnedArg);
   if (*returnedArg)
   {
      double xCent, yCent, xFull, yFull, dFWHM;
      returnedArg->Get("XVALUE", &xCent);
      returnedArg->Get("YVALUE", &yCent);
      returnedArg->Get("XFULL", &xFull);
      returnedArg->Get("YFULL", &yFull);
      returnedArg->Get("FWHM", &dFWHM);
      MessageUser("C_CENTROID:  result \nError:  %lf %lf \nWinodw Size: %lf %lf\nFull Width Half Max: %lf",
                  xCent, yCent, xFull, yFull, dFWHM);
   }
   details->inUse = NO;
   ThisTask->tdFfpiPostExp();
   MessageUser("C_CENTROID: - Action complete.");
}

//  ------------------------------------------------------------------------------------------------

//         C  Image   A c t i o n  N T  : :  A c t i o n N T
void CImageAction::ActionThread(const drama::sds::Id &Arg)
{
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();
   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   if (details == nullptr)
   {
      DramaTHROW(TDFCANTASK__NOTINIT, "C_IMAGE: the structure pointer is null, please initialise the task!");
      // return;
   }
   if (details->inUse)
   {
      DramaTHROW(TDFCANTASK__IN_USE, "C_IMAGE: TdFCanTask is running other actions.");
      // return;
   }
   string strImage, strWindow;
   if (Arg)
   {
      drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
      drama::gitarg::String ImageArg(this, Arg, "Image", 1, "FREE", NoFlags);
      strImage = ImageArg;
      drama::gitarg::String WindowArg(this, Arg, "Window", 2, "FULL", NoFlags);
      strWindow = WindowArg;
   }
   details->inUse = YES;

   tdFfpiCENtype *cenData = new tdFfpiCENtype();
   sprintf(cenData->saveName, "%s_%ld", "Image", (long int)time(0));
   cenData->window.MaxX = details->freeImg.xMax;
   cenData->window.MaxY = details->freeImg.yMax;
   cenData->window.PixelSize = details->freeImg.PixelSize;
   cenData->img = &details->freeImg;

   if (cenData->settleTime > 0.0) // this should be renamed to settletime
   {
      std::this_thread::sleep_for(std::chrono::seconds(int(cenData->settleTime)));
   }

   MessageUser("C_IMAGE: - grabbing image");

   ThisTask->tdFGetCameraPath().GetPath(this);
   drama::sds::Id messageArg(drama::sds::Id::CreateArgStruct());
   drama::sds::IdPtr returnedArg;

   messageArg.Put("EXPOSURE_TIME", cenData->img->exposureTime);
   messageArg.Put("SHUTTER_OPEN", (int)cenData->img->shutter);
   messageArg.Put("UPDATE", cenData->img->updateTime);
   ThisTask->tdFGetCameraPath().Obey(this, "IMAGE", messageArg, &returnedArg);
   if (returnedArg == nullptr)
   {
      details->inUse = NO;
      DramaTHROW(TDFCANTASK__NO_IMAGE, "C_IMAGE: No image is taken, please check the camera.");
   }
   // drama::sds::Id DataArray= returnedArg->Find("DATA_ARRAY");
   // DataArray.List();
   details->inUse = NO;
   MessageUser("C_IMAGE: - Action complete.");
}

void CZeroCamAction::ActionThread(const drama::sds::Id &)
{
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();
   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   if (details == nullptr)
   {
      DramaTHROW(TDFCANTASK__NOTINIT, "C_ZEROCAM: the structure pointer is null, please initialise the task!");
   }
   if (details->inUse)
   {
      DramaTHROW(TDFCANTASK__IN_USE, "C_ZEROCAM: TdFCanTask is running other actions.");
   }

   m_tdFfpiZCStruct = new tdFfpiZCtype();
   drama::ParSys parSysId(ThisTask->TaskPtr());

   GCamWindowType cenWin;
   double grid[MAX_POINTS][2], measured[MAX_POINTS][2], settleTime;
   long int cenX, cenY;
   bool atNextPoint, centroided, doneGrid, settled;
   short curPoint;

   std::vector<std::string> fitErrors = {"illegal itype",
                                         "insufficent data", "singular solution"};
#define FITERR_ILL_ITYPE 0
#define FITERR_INSUFF_DATA 1
#define FITERR_SINGULAR 2

   if (m_tdFfpiZCStruct->reset == YES)
   {
      long int i, j, k, nextX, nextY;
      details->imagePos.enable = 1;
      details->imagePos.displayText = YES;
      details->imagePos.useDpr = details->dprFeedback;

      ThisTask->tdFfpiUpdatePos(YES, details->dprFeedback, YES);
      cenX = details->ideal.x;
      cenY = details->ideal.y;
      parSysId.Get("SETTLE_TIME", &settleTime);
      centroided = doneGrid = NO;
      atNextPoint = NO;

      if ((*details->pars.zeroCamCenWait) > 0)
         settled = NO;
      else
         settled = YES;

      k = nextX = nextY = 0;
      i = j = 1;
      for (curPoint = 0; curPoint < MAX_POINTS; curPoint++)
      {
         if (j > i)
         {
            if (k == 0)
            {
               j = 1;
               k = 1;
            }
            else
            {
               i++;
               j = 1;
               k = 0;
            }
         }
         (k == 0) ? (nextY += (i % 2) ? GRID_SIZE : -GRID_SIZE) : (nextX += (i % 2) ? GRID_SIZE : -GRID_SIZE);
         j++;
         grid[curPoint][_XI] = (double)nextX;
         grid[curPoint][_YI] = (double)nextY;
      }
      curPoint = 0;

      cenWin.MaxX = cenWin.Xdim = details->freeImg.xMax;
      cenWin.MaxY = cenWin.Ydim = details->freeImg.yMax;
      cenWin.PixelSize = details->freeImg.PixelSize;
      cenWin.Xoffset = cenWin.Yoffset = 0;
      m_tdFfpiZCStruct->reset = NO;
   }

   bool flag = true;
   while (!settled || !centroided || !doneGrid)
   {
      if (!settled)
      {
         settled = YES;
         std::this_thread::sleep_for(std::chrono::seconds(int(*details->pars.zeroCamCenWait)));
         long int Xpos, Ypos;
         parSysId.Get("X", &Xpos);
         parSysId.Get("Y", &Ypos);
         MessageUser("C_ZEROCAM: waiting %g seconds for gantry to settle",
                     (*details->pars.zeroCamCenWait));
         MessageUser("C_ZEROCAM: Position after move was %ld, %ld", Xpos, Ypos);
      }
      else if (!centroided)
      {
         centroided = YES;
         if (cenWin.Xoffset < 0)
            cenWin.Xoffset = 0;
         if (cenWin.Yoffset < 0)
            cenWin.Yoffset = 0;
         if (cenWin.Xdim + cenWin.Xoffset > cenWin.MaxX)
            cenWin.Xdim = cenWin.MaxX - cenWin.Xoffset;
         if (cenWin.Ydim + cenWin.Yoffset > cenWin.MaxY)
            cenWin.Ydim = cenWin.MaxY - cenWin.Yoffset;

         if (settleTime > 0.0)
         {
            std::this_thread::sleep_for(std::chrono::seconds(int(settleTime)));
         }
         ThisTask->tdFfpiPreExp();
         MessageUser("C_ZEROCAM: - grabbing image");
         MessageUser("C_ZEROCAM:  Window size -> Max %ld %ld off %ld %ld, dim %ld %ld",
                     cenWin.MaxX, cenWin.MaxY, cenWin.Xoffset, cenWin.Yoffset,
                     cenWin.Xdim, cenWin.Ydim);

         ThisTask->tdFGetCameraPath().GetPath(this);
         drama::sds::Id messageArg(drama::sds::Id::CreateArgStruct());
         drama::sds::IdPtr returnedArg;
         std::string strWindowType;
         strWindowType = to_string(cenWin.Xoffset) + ":" + to_string(cenWin.Yoffset) + ":" + to_string(cenWin.Xoffset + cenWin.Xdim - 1) + ":" + to_string(cenWin.Yoffset + cenWin.Ydim - 1);
         messageArg.Put("CENTROID", strWindowType);
         messageArg.Put("BIAS", details->freeImg.bias);
         messageArg.Put("EXPOSURE_TIME", details->freeImg.exposureTime);
         messageArg.Put("SHUTTER_OPEN", (int)details->freeImg.shutter);
         messageArg.Put("UPDATE", details->freeImg.updateTime);

         ThisTask->tdFGetCameraPath().Obey(this, "CENTRECT", messageArg, &returnedArg);
         if (*returnedArg)
         {
            double xCent, yCent, xFull, yFull, dFWHM;
            returnedArg->Get("XVALUE", &xCent);
            returnedArg->Get("YVALUE", &yCent);
            returnedArg->Get("XFULL", &xFull);
            returnedArg->Get("YFULL", &yFull);
            returnedArg->Get("FWHM", &dFWHM);
            MessageUser("C_ZEROCAM: centroid result \nError:  %lf %lf \nWinodw Size: %lf %lf\nFull Width Half Max: %lf",
                        xCent, yCent, xFull, yFull, dFWHM);
            m_tdFfpiZCStruct->centroidOK = dFWHM ? YES : NO;
            m_tdFfpiZCStruct->xFull = xFull;
            m_tdFfpiZCStruct->yFull = yFull;
         }
         ThisTask->tdFfpiPostExp();
      }
      else if (!doneGrid)
      {
         if (!atNextPoint)
         {
            atNextPoint = YES;
            centroided = NO;
            long int XCoor = cenX + doubleToLong(grid[curPoint][_XI]);
            long int YCoor = cenY + doubleToLong(grid[curPoint][_YI]);
            MessageUser("C_ZEROCAM: About to move to next grid point (%ld,%ld)", XCoor, YCoor);
            drama::Path thisTaskPath(_theTask);
            thisTaskPath.GetPath(this);
            drama::sds::Id messageArg(drama::sds::Id::CreateArgStruct());
            std::string strAxis, strCoordinates;
            strAxis = "X,Y";
            strCoordinates = to_string(XCoor) + "," + to_string(YCoor);
            messageArg.Put("AXES", strAxis);
            messageArg.Put("POSITIONS", strCoordinates);
            thisTaskPath.Obey(this, "G_MOVE_NT", messageArg);

            if ((*details->pars.zeroCamCenWait) > 0)
               settled = NO;
         }
         else
         {
            if (m_tdFfpiZCStruct->centroidOK != YES)
            {
               flag = false;
               break;
            }
            else
            {
               measured[curPoint][_XI] = m_tdFfpiZCStruct->xFull;
               measured[curPoint][_YI] = m_tdFfpiZCStruct->yFull;
               MessageUser("C_ZEROCAM: Position during centroid was %ld, %ld", details->imagePos.p.x, details->imagePos.p.y);
               grid[curPoint][_XI] = details->imagePos.p.x - cenX;
               grid[curPoint][_YI] = details->imagePos.p.y - cenY;
            }
            if (curPoint < MAX_POINTS - 1)
            {
               curPoint++;
               atNextPoint = NO;
            }
            else
            {
               double coeffs[6];
               int i, j = 0;
               curPoint = 0;
               slaFitxy(6, MAX_POINTS, grid, measured, coeffs, &j);
               if (j != 0)
               {
                  char *err;
                  char scratch[40];
                  if (j == -1)
                     err = (char *)fitErrors[FITERR_ILL_ITYPE].c_str();
                  else if (j == -2)
                     err = (char *)fitErrors[FITERR_INSUFF_DATA].c_str();
                  else if (j == -3)
                     err = (char *)fitErrors[FITERR_SINGULAR].c_str();
                  else
                  {
                     sprintf(scratch, "Unexpected fit error %d", j);
                     err = scratch;
                  }

                  MessageUser("C_ZEROCAM: error message %s\n", err);
                  MessageUser("    Array contents -\n");
                  MessageUser("      (pixels)           (microns)\n");
                  for (j = 0; j < MAX_POINTS; j++)
                     MessageUser("      %7.3f,%7.3f    %7.3f,%7.3f\n",
                                 measured[j][_XI], measured[j][_YI],
                                 grid[j][_XI], grid[j][_YI]);

                  flag = false;
                  break;
               }

               MessageUser("Actual (micron) and Measured (pixel) arrays are:\n");
               for (i = 0; i < MAX_POINTS; i++)
                  MessageUser("    %4ld,%4ld    %8.4f,%8.4f\n",
                              doubleToLong(grid[i][_XI]), doubleToLong(grid[i][_YI]),
                              measured[i][_XI], measured[i][_YI]);

               /*
                *  Record new transformation matrices.
                */
               for (i = 0; i < 6; i++)
                  details->freeImg.camCoeffs[i] = coeffs[i];
               slaInvf(details->freeImg.camCoeffs,
                       details->freeImg.invCoeffs, &j);
               if (j != 0)
               {
                  for (i = 0; i < 6; i++)
                     details->freeImg.invCoeffs[i] = details->freeImg.camCoeffs[i];
               }
               if (1)
               {
                  int i;
                  MessageUser("====================================================\n");
                  MessageUser("  expected             calculated           error\n");
                  MessageUser("====================================================\n");
                  for (i = 0; i < MAX_POINTS; i++)
                  {
                     double calX, calY;
                     slaXy2xy(measured[i][_XI], measured[i][_YI], coeffs, &calX, &calY);
                     MessageUser("%8.1f,%8.1f   %8.1f,%8.1f  %6.1f,%6.1f\n",
                                 grid[i][_XI], grid[i][_YI], calX, calY,
                                 grid[i][_XI] - calX, grid[i][_YI] - calY);
                  }
                  MessageUser("====================================================\n");
                  MessageUser("New camera coeffs array is:\n");
                  MessageUser("    %8.4f  %8.4f  %8.4f\n", coeffs[0], coeffs[1], coeffs[2]);
                  MessageUser("    %8.4f  %8.4f  %8.4f\n", coeffs[3], coeffs[4], coeffs[5]);
               }
               doneGrid = YES;
            }
         }
      }
   }
   drama::sds::Id newArg;
   if (flag)
   {
      newArg = drama::sds::Id::CreateArgStruct();
      std::vector<unsigned long> dims;
      dims.push_back(6);
      drama::sds::Id coeffs(newArg.CreateChildArray("coeffs", SDS_DOUBLE, dims));

      unsigned long i;
      unsigned long count;

      drama::sds::ArrayWriteHelper<double> array;
      coeffs.ArrayAccess(&array);

      count = array.Size();
      for (i = 0; i < count; ++i)
      {
         array[i] = details->freeImg.camCoeffs[i];
      }
   }
   SetReturnArg(&newArg);
   MessageUser("C_ZEROCAM: - Action complete.");
}

void CShiftCoAction::ActionThread(const drama::sds::Id &Arg)
{
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();
   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   if (details == nullptr)
   {
      DramaTHROW(TDFCANTASK__NOTINIT, "C_SHIFT_COEFF: the structure pointer is null, please initialise the task!");
   }
   if (details->inUse)
   {
      DramaTHROW(TDFCANTASK__IN_USE, "C_SHIFT_COEFF: TdFCanTask is running other actions.");
   }

   m_tdFfpiSHStruct = new tdFfpiSHtype();
   drama::ParSys parSysId(ThisTask->TaskPtr());
   if (!Arg)
   {
      DramaTHROW(TDFCANTASK__NO_ARGUMENTS, "C_SHIFT_COEFF: No input argument is provided.");
   }
   int expX, expY;
   drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
   drama::gitarg::Int<XMIN, XMAX> XFArg(this, Arg, "expX", 1, 0, NoFlags);
   expX = XFArg;
   m_tdFfpiSHStruct->expX = expX;
   drama::gitarg::Int<YMIN, YMAX> YFArg(this, Arg, "expY", 2, 0, NoFlags);
   expY = YFArg;

   short doneSearch;
   if (m_tdFfpiSHStruct->reset == YES)
   {
      doneSearch = NO;
      m_tdFfpiSHStruct->reset = NO;
   }

   if (!doneSearch)
   {
      doneSearch = YES;
      drama::Path thisTaskPath(_theTask);
      thisTaskPath.GetPath(this);
      drama::sds::Id messageArg(drama::sds::Id::CreateArgStruct());
      messageArg.Put("XF", expX);
      messageArg.Put("YF", expY);
      drama::sds::IdPtr returnedArg;
      thisTaskPath.Obey(this, "C_SEARCH", messageArg, &returnedArg);
      if (returnedArg)
      {
         returnedArg->Get("FOUND", &m_tdFfpiSHStruct->found);
         returnedArg->Get("MEASUREDX", &m_tdFfpiSHStruct->measuredX);
         returnedArg->Get("MEASUREDY", &m_tdFfpiSHStruct->measuredY);
      }
   }
   else
   {
      double expXenc, expYenc, atXenc, atYenc,
          plateTheta, dx, dy;

      parSysId.Get("PLATE_THETA", &plateTheta);
      ThisTask->tdFfpiConvertFromFP(m_tdFfpiSHStruct->expX, m_tdFfpiSHStruct->expY,
                                    plateTheta, _FULL,
                                    &expXenc, &expYenc);
      ThisTask->tdFfpiConvertFromFP(m_tdFfpiSHStruct->measuredX, m_tdFfpiSHStruct->measuredY,
                                    plateTheta, _FULL,
                                    &atXenc, &atYenc);
      dx = atXenc - expXenc;
      dy = atYenc - expYenc;

      if (m_tdFfpiSHStruct->found == NO)
      {
         MessageUser("C_SHIFT_COEFF: Search for image at %ld,%ld failed.\n", expX, expY);
         return;
      }
      else
      {
         int j;

         details->convert.coeffs[0] += dx;
         details->convert.coeffs[3] += dy;
         slaInvf(details->convert.coeffs, details->convert.invCoeffs, &j);
         if (j != 0)
         {
            int i;
            for (i = 0; i < 6; i++)
               details->convert.invCoeffs[i] = details->convert.coeffs[i];
         }
      }
      MessageUser("C_SHIFT_COEFF: Measured shift = %.1f,%.1f", dx, dy);
      MessageUser("C_SHIFT_COEFF: New COEFFS array is %f,%f,%f,%f,%f,%f",
                  details->convert.coeffs[0], details->convert.coeffs[1],
                  details->convert.coeffs[2], details->convert.coeffs[3],
                  details->convert.coeffs[4], details->convert.coeffs[5]);

      drama::sds::Id newArg = drama::sds::Id::CreateArgStruct();
      std::vector<unsigned long> dims;
      dims.push_back(6);
      drama::sds::Id coeffs(newArg.CreateChildArray("coeffs", SDS_DOUBLE, dims));

      unsigned long i;
      unsigned long count;

      drama::sds::ArrayWriteHelper<double> array;
      coeffs.ArrayAccess(&array);

      count = array.Size();
      for (i = 0; i < count; ++i)
      {
         array[i] = details->convert.coeffs[i];
      }

      SetReturnArg(&newArg);
   }
   MessageUser("C_SHIFT_COEFF: - Action complete.");
}

bool CSurveyAction::FidNotFound(double *const expectedX, double *const expectedY,
                                const short curFid, short *const atFid, short *const recordedFid)
{
   DEBUG("C_SURVEY: Search for fiducial index %d, at %ld,%ld failed", curFid,
         m_tdFfpiSStruct->x[curFid], m_tdFfpiSStruct->y[curFid]);

   --m_tdFfpiSStruct->numMarks;
   if ((m_tdFfpiSStruct->numMarks) < MIN_MARKS)
   {
      DEBUG("C_SURVEY: Insufficent marks found to do a survey, have %d, need %d\n",
            m_tdFfpiSStruct->numMarks, MIN_MARKS);
      return false;
   }

   drama::ParSys parSysId(_theTask);
   if (curFid == m_tdFfpiSStruct->numMarks)
   {
      *recordedFid = YES;
      parSysId.Put("SURVEY_PROG", 100.0);
      return true;
   }
   for (int i = curFid; i < m_tdFfpiSStruct->numMarks; ++i) /// why is no larger than numMarks here?
   {
      m_tdFfpiSStruct->x[i] = m_tdFfpiSStruct->x[i + 1];
      m_tdFfpiSStruct->y[i] = m_tdFfpiSStruct->y[i + 1];
      expectedX[i] = expectedX[i + 1];
      expectedY[i] = expectedY[i + 1];
   }

   parSysId.Put("SURVEY_PROG",
                100.0 * (curFid + 1.0) / (double)m_tdFfpiSStruct->numMarks);
   *atFid = NO;
   return true;
}

void CSurveyAction::RecordFid(double *const expectedX, double *const expectedY, double *const measuredX,
                              double *const measuredY, long *const offsetX, long *const offsetY,
                              short *const curFid, short *const atFid, short *const recordedFid)
{
   double plateTheta;
   drama::ParSys parSysId(_theTask);
   parSysId.Get("PLATE_THETA", &plateTheta);

   auto thisTask = _theTask.lock()->TaskPtrAs<TdFCanTask>();
   thisTask->tdFfpiConvertFromFP(m_tdFfpiSStruct->x[*curFid], m_tdFfpiSStruct->y[*curFid],
                                 plateTheta, m_tdFfpiSStruct->area,
                                 &expectedX[*curFid], &expectedY[*curFid]);

   thisTask->tdFfpiConvertFromEnc(m_tdFfpiSStruct->xEnc, m_tdFfpiSStruct->yEnc,
                                  plateTheta, m_tdFfpiSStruct->area,
                                  &measuredX[*curFid], &measuredY[*curFid]);

   if (m_tdFfpiSStruct->area == _COEFFS)
   {
      double xErr, yErr, absErr;
      double expEncX, expEncY;
      thisTask->tdFfpiConvertFromFP(m_tdFfpiSStruct->x[*curFid], m_tdFfpiSStruct->y[*curFid],
                                    plateTheta, _FULL,
                                    &expEncX, &expEncY);

      xErr = expEncX - m_tdFfpiSStruct->xEnc;
      yErr = expEncY - m_tdFfpiSStruct->yEnc;
      absErr = sqrt(SQRD(xErr) + SQRD(yErr));
      DEBUG("C_SURVEY: Fiducial index %d found %.1f (x:%.1f, y:%.1f) microns from the expected position\n", *curFid, absErr, xErr, yErr);
   }

   *offsetX += m_tdFfpiSStruct->dx;
   *offsetY += m_tdFfpiSStruct->dy;

   if (*curFid == m_tdFfpiSStruct->numMarks - 1)
   {
      *recordedFid = YES;
      parSysId.Put("SURVEY_PROG", 100.0);
   }
   else
   {
      parSysId.Put("SURVEY_PROG",
                   100.0 * (*curFid + 1.0) / (double)m_tdFfpiSStruct->numMarks);
      *atFid = NO;
      (*curFid)++;
   }
}

void CSurveyAction::SearchForFid(const long offsetX, const long offsetY,
                                 const short curFid, short *const atFid)
{
   *atFid = YES;
   drama::Path thisTaskPath(_theTask);
   thisTaskPath.GetPath(this);
   drama::sds::Id messageArg(drama::sds::Id::CreateArgStruct());
   long int startX, startY;
   startX = m_tdFfpiSStruct->x[curFid] + offsetX;
   startY = m_tdFfpiSStruct->y[curFid] + offsetY;
   messageArg.Put("XF", startX);
   messageArg.Put("YF", startY);
   drama::sds::IdPtr returnedArg;

   DEBUG("C_SURVEY: Will search for fiducial index %d at %7ld, %7ld - search at %7ld, %7ld\n",
         curFid, m_tdFfpiSStruct->x[curFid], m_tdFfpiSStruct->y[curFid], startX, startY);
   thisTaskPath.Obey(this, "C_SEARCH", messageArg, &returnedArg);
   if (returnedArg)
   {
      returnedArg->Get("FOUND", &m_tdFfpiSStruct->found);
      if (m_tdFfpiSStruct->found == NO)
      {
         m_tdFfpiSStruct->xEnc = m_tdFfpiSStruct->yEnc = 0;
      }
      else
      {
         returnedArg->Get("EncoderX", &m_tdFfpiSStruct->xEnc);
         returnedArg->Get("EncoderY", &m_tdFfpiSStruct->yEnc);
         returnedArg->Get("XEncoderErr", &m_tdFfpiSStruct->dx);
         returnedArg->Get("YEncoderErr", &m_tdFfpiSStruct->dy);
      }
   }
}

void CSurveyAction::DisplayResultsBasic(const int newmodel, double fiducialArray[][2],
                                        double cal[][2], const double xrms, const double yrms, const double rrms, drama::sds::Id *paramId)
{
   int counter;

   double xErr[NUM_FIDUCIALS];
   double yErr[NUM_FIDUCIALS];
   double xExpected[NUM_FIDUCIALS];
   double yExpected[NUM_FIDUCIALS];
   double xCalculated[NUM_FIDUCIALS];
   double yCalculated[NUM_FIDUCIALS];

   for (counter = 0; counter < m_tdFfpiSStruct->numMarks; counter++)
   {
      xErr[counter] = fiducialArray[counter][_XI] - cal[counter][_XI];
      yErr[counter] = fiducialArray[counter][_YI] - cal[counter][_YI];
      xExpected[counter] = fiducialArray[counter][_XI];
      yExpected[counter] = fiducialArray[counter][_YI];
      xCalculated[counter] = cal[counter][_XI];
      yCalculated[counter] = cal[counter][_YI];
   }

   if (paramId)
   {
      drama::sds::Id id = paramId->Copy();
      const char *calNameX = 0;
      const char *calNameY = 0;
      if (newmodel == 10)
      {
         calNameX = "New1stPassErrX";
         calNameY = "New1stPassErrY";
         id.Put("New1stPassRMSX", xrms);
         id.Put("New1stPassRMSY", yrms);
         id.Put("New1stPassRMS", rrms);
      }
      else if (newmodel)
      {
         calNameX = "NewErrX";
         calNameY = "NewErrY";
         id.Put("NewRMSX", xrms);
         id.Put("NewRMSY", yrms);
         id.Put("NewRMS", rrms);
      }
      else
      {
         calNameX = "OldErrX";
         calNameY = "OldErrY";
         id.Put("OldRMSX", xrms);
         id.Put("OldRMSY", yrms);
         id.Put("OldRMS", rrms);
      }
      std::vector<unsigned long> dims;
      dims.push_back(m_tdFfpiSStruct->numMarks);
      drama::sds::Id xErrId(id.CreateChildArray(calNameX, SDS_DOUBLE, dims));
      drama::sds::ArrayWriteHelper<double> xarray;
      xErrId.ArrayAccess(&xarray);
      for (counter = 0; counter < (int)xarray.Size(); ++counter)
      {
         xarray[counter] = xErr[counter];
      }

      drama::sds::Id yErrId(id.CreateChildArray(calNameY, SDS_DOUBLE, dims));
      drama::sds::ArrayWriteHelper<double> yarray;
      yErrId.ArrayAccess(&yarray);
      for (counter = 0; counter < (int)yarray.Size(); ++counter)
      {
         yarray[counter] = yErr[counter];
      }

      paramId->ShallowCopy(&id, true);
   }

   DEBUG("========================================================\n");
   if (newmodel == 10)
   {
      DEBUG("C_SURVEY: Field Plate units - new model first pass\n");
   }
   else
   {
      DEBUG("C_SURVEY: Field Plate units - %s\n", newmodel ? "new model" : "old model");
   }
   DEBUG("#################\n");
   DEBUG("  Error between specified fiducial position and what we\n");
   DEBUG("  get by converting the encoder position we found it at\n");
   DEBUG("  back to field plate units using the %s\n", newmodel ? "new model" : "old model");
   DEBUG("expected             calculated                 error (x/y/t)\n");
   DEBUG("===================================================================\n");
   for (counter = 0; counter < m_tdFfpiSStruct->numMarks; counter++)
   {
      DEBUG("%9.1f,%9.1f   %9.1f,%9.1f  %6.1f,%6.1f %6.2f\n",
            fiducialArray[counter][_XI],
            fiducialArray[counter][_YI],
            cal[counter][_XI],
            cal[counter][_YI],
            xErr[counter], yErr[counter],
            sqrt(xErr[counter] * xErr[counter] +
                 yErr[counter] * yErr[counter]));
   }
   DEBUG("RMS                                            %6.1f,%6.1f %6.2f\n", xrms, yrms, rrms);
   DEBUG("===================================================================\n");
}

void CSurveyAction::tdFfpiSurveyCheckCoeffs(const double *current, const double *newcoeffs,
                                            int plate, const double plateTheta, drama::sds::Id *paramId)
{
   double cXz;     /* Current coefficients - X zero point */
   double cYz;     /* Current coefficients - Y zero point */
   double cXs;     /* Current coefficients - X Scale */
   double cYs;     /* Current coefficients - X Scale */
   double cPerp;   /* Current coefficients -  nonperpendicularity (radians) */
   double cOrient; /* Current coefficients - orientation (radians) */

   double nXz;     /* New coefficients - X zero point */
   double nYz;     /* New coefficients - Y zero point */
   double nXs;     /* New coefficients - X Scale */
   double nYs;     /* New coefficients - X Scale */
   double nPerp;   /* New coefficients -  nonperpendicularity (radians) */
   double nOrient; /* New coefficients - orientation (radians) */

   slaDcmpf((double *)current, &cXz, &cYz, &cXs, &cYs, &cPerp, &cOrient);
   slaDcmpf((double *)newcoeffs, &nXz, &nYz, &nXs, &nYs, &nPerp, &nOrient);

   DEBUG("C_SURVEY: Old Coefficents array = %.1f, %f, %f, %.1f, %f, %f\n",
         current[0], current[1], current[2], current[3], current[4], current[5]);
   DEBUG("C_SURVEY: New Coefficents array = %.1f, %f, %f, %.1f, %f, %f\n",
         newcoeffs[0], newcoeffs[1], newcoeffs[2],
         newcoeffs[3], newcoeffs[4], newcoeffs[5]);
   DEBUG("C_SURVEY: Decomposition follows\n");
   DEBUG("Which        X-Zero      Y-Zero      X-Scale    Y-Scale  Orientation Non-perp \n");
   DEBUG("Current      %8.1f %8.1f %10.8f %10.8f %8.4f %8.4f\n",
         cXz, cYz, cXs, cYs, cOrient * 180.0 / PI, cPerp * 180.0 / PI);
   DEBUG("New          %8.1f %8.1f %10.8f %10.8f %8.4f %8.4f\n",
         nXz, nYz, nXs, nYs, nOrient * 180.0 / PI, nPerp * 180.0 / PI);
   DEBUG("Change       %8.1f %8.1f %10.8f %10.8f %8.6f %8.6f\n",
         nXz - cXz,
         nYz - cYz,
         fabs(nXs / cXs) - 1.0,
         fabs(nYs / cYs) - 1.0,
         (nOrient - cOrient) * 180.0 / PI,
         (nPerp - cPerp) * 180.0 / PI);
   DEBUG("Rotation: Applied=%8.4f, Apparent = %8.4f (milli-degrees), Plate %d\n",
         (plateTheta / PI * 180.0 * 1000.0),
         ((plateTheta + nOrient) / PI * 180.0 * 1000.0),
         plate);

   if (paramId)
   {
      int counter;
      std::vector<unsigned long> dims;
      dims.push_back(6);

      drama::sds::Id newId = paramId->Copy();
      drama::sds::Id newParamId(newId.CreateChildItem("checkdata", SDS_STRUCT));
      drama::sds::Id oldCoeffs(newParamId.CreateChildArray("oldCoeffs", SDS_DOUBLE, dims));
      drama::sds::ArrayWriteHelper<double> oldCoeffarray;
      oldCoeffs.ArrayAccess(&oldCoeffarray);
      for (counter = 0; counter < (int)oldCoeffarray.Size(); ++counter)
      {
         oldCoeffarray[counter] = current[counter];
      }
      drama::sds::Id newCoeffs(newParamId.CreateChildArray("newCoeffs", SDS_DOUBLE, dims));
      drama::sds::ArrayWriteHelper<double> newCoeffarray;
      newCoeffs.ArrayAccess(&newCoeffarray);
      for (counter = 0; counter < (int)newCoeffarray.Size(); ++counter)
      {
         newCoeffarray[counter] = newcoeffs[counter];
      }

      drama::sds::Id idOldDecomp(newParamId.CreateChildItem("oldDecomp", SDS_STRUCT));
      idOldDecomp.Put("X-Zero", cXz);
      idOldDecomp.Put("Y-Zero", cYz);
      idOldDecomp.Put("X-Scale", cXs);
      idOldDecomp.Put("Y-Scale", cYs);
      idOldDecomp.Put("Orientation", cOrient * 180.0 / PI);
      idOldDecomp.Put("Non-perp", cPerp * 180.0 / PI);

      drama::sds::Id idNewDecomp(newParamId.CreateChildItem("newDecomp", SDS_STRUCT));
      idNewDecomp.Put("X-Zero", nXz);
      idNewDecomp.Put("Y-Zero", nYz);
      idNewDecomp.Put("X-Scale", nXs);
      idNewDecomp.Put("Y-Scale", nYs);
      idNewDecomp.Put("Orientation", nOrient * 180.0 / PI);
      idNewDecomp.Put("Non-perp", nPerp * 180.0 / PI);

      drama::sds::Id idChange(newParamId.CreateChildItem("change", SDS_STRUCT));
      idChange.Put("X-Zero", nXz - cXz);
      idChange.Put("Y-Zero", nYz - cYz);
      idChange.Put("X-Scale", fabs(nXs / cXs) - 1.0);
      idChange.Put("Y-Scale", fabs(nYs / cYs) - 1.0);
      idChange.Put("Orientation", (nOrient - cOrient) * 180.0 / PI);
      idChange.Put("Non-perp", (nPerp - cPerp) * 180.0 / PI);

      newParamId.Put("PlateTheta", plateTheta);
      newParamId.Put("RotApplied", (plateTheta / PI * 180.0 * 1000.0));
      newParamId.Put("RotApparent", (plateTheta + nOrient) / PI * 180.0 * 1000.0);
      paramId->ShallowCopy(&newId, true);
   }
}

bool CSurveyAction::OffsetAndDisplayResults(tdFfpiTaskType *details, const double platetheta, const short fitType, const short centerFid, double measuredArray[][2], double fiducialArray[][2],
                                            double *const coeffs, drama::sds::Id *paramId)
{
   double xrms;
   double yrms;
   double rrms;
   double xrms2;
   double yrms2;
   double rrms2;
   double diffCoeffs[6];
   double origNewCoeffs[6];
   double calculatedNewModel[NUM_FIDUCIALS][2]; /* Positions calculated with new model */
   double calculatedNewModelNoOffset[NUM_FIDUCIALS][2];
   double calculatedOrigModel[NUM_FIDUCIALS][2]; /* Positions calculated with original model */
   int j;
   int adjustedCenter = 0;

   for (j = 0; j < 6; ++j)
      origNewCoeffs[j] = coeffs[j];

   slaFitxy(fitType, m_tdFfpiSStruct->numMarks, fiducialArray, measuredArray, diffCoeffs, &j);
   slaPxy(m_tdFfpiSStruct->numMarks, fiducialArray, measuredArray, details->convert.invCoeffs, calculatedOrigModel, &xrms2, &yrms2, &rrms2);

   slaPxy(m_tdFfpiSStruct->numMarks, fiducialArray, measuredArray, diffCoeffs, calculatedNewModel, &xrms, &yrms, &rrms);
   slaPxy(m_tdFfpiSStruct->numMarks, fiducialArray, measuredArray, diffCoeffs, calculatedNewModelNoOffset, &xrms, &yrms, &rrms);

   if ((centerFid >= 0) && (centerFid < m_tdFfpiSStruct->numMarks))
   {
      double xCen = calculatedNewModel[centerFid][_XI];
      double yCen = calculatedNewModel[centerFid][_YI];
      if ((fabs(xCen) > 0.1) || (fabs(yCen) > 0.1))
      {
         double x, y;
         double xErr, yErr;
         adjustedCenter = 1;

         DEBUG("C_SURVEY: Adjusting offset by %.1f %.1f to center the center fiducial (index %d)\n",
               xCen, yCen, centerFid);
         DEBUG("C_SURVEY: Original RMS error is %6.1f,%6.1f %6.2f\n", xrms, yrms, rrms);

         slaXy2xy(0, 0, coeffs, &x, &y);
         xErr = x - measuredArray[centerFid][_XI];
         yErr = y - measuredArray[centerFid][_YI];
         DEBUG("C_SURVEY: Without adjusted coeffs, 0,0 -> %6.1f, %6.1f (error %3.1f, %3.1f)\n", x, y, xErr, yErr);

         diffCoeffs[0] -= xCen;
         diffCoeffs[3] -= yCen;
         slaInvf(diffCoeffs, coeffs, &j);

         slaXy2xy(0, 0, coeffs, &x, &y);
         xErr = x - measuredArray[centerFid][_XI];
         yErr = y - measuredArray[centerFid][_YI];
         DEBUG("C_SURVEY: With zero adjusted coeffs, 0,0 -> %6.1f, %6.1f (error %3.1f, %3.1f)\n", x, y, xErr, yErr);

         /*
          *  Recalculate the new positions with the adjusted offset.
          */
         slaPxy(m_tdFfpiSStruct->numMarks, fiducialArray, measuredArray,
                diffCoeffs, calculatedNewModel, &xrms, &yrms, &rrms);
      }
      else
      {
         DEBUG("C_SURVEY: Center fiducial (index %d) is centered - no adjustment needed.\n", centerFid);
      }
   }
   else
   {
      DEBUG("C_SURVEY: Don't have center fiducial so can't ensure offset is correct.\n");
   }

   DEBUG("C_SURVEY: Survey did a %d component fit.\n", fitType);

   DisplayResultsBasic(0, fiducialArray, calculatedOrigModel,
                       xrms2, yrms2, rrms2, paramId);
   if (adjustedCenter)
      DisplayResultsBasic(10, fiducialArray, calculatedNewModelNoOffset,
                          xrms, yrms, rrms, paramId);

   DisplayResultsBasic(1, fiducialArray, calculatedNewModel,
                       xrms, yrms, rrms, paramId);

   DisplayResultsInEncoderUnits(details, platetheta, 0,
                                details->convert.coeffs,
                                measuredArray);
   if (adjustedCenter)
      DisplayResultsInEncoderUnits(details, platetheta,
                                   10, origNewCoeffs,
                                   measuredArray);
   DisplayResultsInEncoderUnits(details, platetheta, 1, coeffs,
                                measuredArray);

   tdFfpiSurveyCheckCoeffs(details->convert.coeffs, coeffs,
                           details->currentPlate, platetheta, paramId);

   DEBUG("C_SURVEY: With old model:XRMS = %5.2f, YRMS = %5.2f, TOTAL RMS = %5.2f\n",
         xrms2, yrms2, rrms2);
   DEBUG("With new model:XRMS = %5.2f, YRMS = %5.2f, TOTAL RMS = %5.2f\n",
         xrms, yrms, rrms);


   if (rrms > RMS_WARNING)
   {
      DEBUG("C_SURVEY: Warning - the SURVEY RMS of %6.1f microns seems too large\n", rrms);
      return false;
   }
   return true;
}

void CSurveyAction::DisplayResultsInEncoderUnits(tdFfpiTaskType *details, const double plateTheta,
                                                 const int newmodel, const double *const coeffs, double measuredArray[][2])
{
   int counter;
   double savedCoeffs[6];
   int j;
   auto thisTask = _theTask.lock()->TaskPtrAs<TdFCanTask>();
   for (j = 0; j < 6; j++)
   {
      savedCoeffs[j] = details->convert.coeffs[j];
      details->convert.coeffs[j] = coeffs[j];
   }

   if (newmodel == 10)
      DEBUG("C_SURVEY: Encoder units - original new model\n");
   else
      DEBUG("C_SURVEY: Encoder units - %s\n", newmodel ? "new model" : "old model");
   DEBUG("#########################\n");
   DEBUG("    The encoder positions the fiducials were expected and found at.\n");
   DEBUG("expected             found                  error (x/y/t)\n");
   DEBUG("===================================================================\n");
   for (counter = 0; counter < m_tdFfpiSStruct->numMarks; counter++)
   {
      double encExpX, encExpY;
      double xErr, yErr;
      /* Convert original positions to the encoder value we
         would have searched at */
      thisTask->tdFfpiConvertFromFP(m_tdFfpiSStruct->x[counter], m_tdFfpiSStruct->y[counter],
                                    plateTheta, _FULL, &encExpX, &encExpY);

      xErr = encExpX - measuredArray[counter][_XI];
      yErr = encExpY - measuredArray[counter][_YI];

      DEBUG("%9.1f,%9.1f   %9.1f,%9.1f  %6.1f,%6.1f %6.2f\n",
            encExpX, encExpY,
            measuredArray[counter][_XI],
            measuredArray[counter][_YI],
            xErr, yErr, sqrt(xErr * xErr + yErr * yErr));
   }
   DEBUG("==============================================================\n");
   /*
    * Revert the coefficents.
    */
   for (j = 0; j < 6; j++)
   {
      details->convert.coeffs[j] = savedCoeffs[j];
   }
}

void CSurveyAction::SetCoeffs(tdFfpiTaskType *details, const double *const expectedX, const double *const expectedY,
                              const double *const measuredX, const double *const measuredY,
                              const short fitType, double *const coeffs, const double plateTheta, drama::sds::Id *paramId)
{
   short centerFid = -1; /* Index of center fiducial */
   double expectedArray[m_tdFfpiSStruct->numMarks][2];
   double measuredArray[m_tdFfpiSStruct->numMarks][2];
   double fiducialArray[m_tdFfpiSStruct->numMarks][2];

   DEBUG("C_SURVEY: Updating gantry field-plate/encoder transformation matrix...\n");
   int j;
   for (j = 0; j < m_tdFfpiSStruct->numMarks; j++)
   {
      expectedArray[j][_XI] = expectedX[j];
      expectedArray[j][_YI] = expectedY[j];
      measuredArray[j][_XI] = measuredX[j];
      measuredArray[j][_YI] = measuredY[j];

      fiducialArray[j][_XI] = m_tdFfpiSStruct->x[j];
      fiducialArray[j][_YI] = m_tdFfpiSStruct->y[j];

      if ((labs(m_tdFfpiSStruct->x[j]) < 5000) && (labs(m_tdFfpiSStruct->y[j]) < 5000))
         centerFid = j;
   }

   slaFitxy(fitType,                   /* Type of model to use (see slaLib docs) */
            m_tdFfpiSStruct->numMarks, /* Number of marks surveyed             */
            measuredArray,             /* Array of measured values             */
            expectedArray,             /* Array of expected values             */
            coeffs,                    /* New coeffs array                     */
            &j);                       /* Modified status                      */

   if (j != 0)
   {
      DEBUG("C_SURVEY: Error calculating coeffs array.\n");
      return;
   }

   DEBUG("C_SURVEY: First pass array = %.1f, %.6f, %.6f, %.1f, %.6f, %.6f\n",
         coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5]);

   if (m_tdFfpiSStruct->area == _COEFFS)
      *paramId = (drama::sds::Id::CreateArgStruct("COEFFS"));

   bool checkFlag = OffsetAndDisplayResults(details, plateTheta, fitType, centerFid,
                                            measuredArray, fiducialArray, coeffs, paramId);
   if (checkFlag == false)
   {
      return;
   }

   for (j = 0; j < 6; j++)
      details->convert.coeffs[j] = coeffs[j];

   slaInvf(details->convert.coeffs, details->convert.invCoeffs, &j);
   if (j != 0)
   {
      int i;
      for (i = 0; i < 6; i++)
         details->convert.invCoeffs[i] = details->convert.coeffs[i];
   }

   DEBUG("C_SURVEY: Actual New array = %.1f, %.6f, %.6f, %.1f, %.6f, %.6f\n",
         coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5]);
   DEBUG("C_SURVEY: Diff array = %.1f, %.6f, %.6f, %.1f, %.6f, %.6f\n",
         details->convert.invCoeffs[0],
         details->convert.invCoeffs[1],
         details->convert.invCoeffs[2],
         details->convert.invCoeffs[3],
         details->convert.invCoeffs[4],
         details->convert.invCoeffs[5]);
}
void CSurveyAction::ConstructReturnValue(tdFfpiTaskType *details, const double *const expectedX, const double *const expectedY, const double *const measuredX,
                                         const double *const measuredY, const double *const coeffs, const double ha,
                                         const double dec, drama::sds::Id *paramId)
{
   time_t now;
   std::vector<unsigned long> dims;
   dims.push_back(m_tdFfpiSStruct->numMarks);
   time(&now);

   drama::sds::Id id = paramId->Copy();
   if (m_tdFfpiSStruct->area == _ALL)
      id = (drama::sds::Id::CreateArgStruct("ALL"));
   else if (m_tdFfpiSStruct->area == _TEMP)
      id = (drama::sds::Id::CreateArgStruct("TEMP"));
   else if (m_tdFfpiSStruct->area == _FLEX)
      id = (drama::sds::Id::CreateArgStruct("FLEX"));
   else if (!paramId) /* _COEFFS */
      id = (drama::sds::Id::CreateArgStruct("COEFFS"));

   id.Put("fitTime", (INT32)(now));
   id.Put("plate", details->currentPlate);
   id.Put("instrument", "2dF");
   id.Put("gantry", "FPI");

   id.Put("numMarks", m_tdFfpiSStruct->numMarks);

   drama::sds::Id tmpId = id.CreateChildArray("expectedX", SDS_DOUBLE, dims);
   drama::sds::ArrayWriteHelper<double> arrayHelper;
   tmpId.ArrayAccess(&arrayHelper);

   for (int index = 0; index < (int)arrayHelper.Size(); index++)
   {
      arrayHelper[index] = expectedX[index];
   }

   tmpId = id.CreateChildArray("expectedY", SDS_DOUBLE, dims);
   tmpId.ArrayAccess(&arrayHelper);
   for (int index = 0; index < (int)arrayHelper.Size(); index++)
   {
      arrayHelper[index] = expectedY[index];
   }

   tmpId = id.CreateChildArray("measuredX", SDS_DOUBLE, dims);
   tmpId.ArrayAccess(&arrayHelper);
   for (int index = 0; index < (int)arrayHelper.Size(); index++)
   {
      arrayHelper[index] = measuredX[index];
   }

   tmpId = id.CreateChildArray("measuredY", SDS_DOUBLE, dims);
   tmpId.ArrayAccess(&arrayHelper);
   for (int index = 0; index < (int)arrayHelper.Size(); index++)
   {
      arrayHelper[index] = measuredY[index];
   }

   dims[0] = 6;
   tmpId = id.CreateChildArray("coeffs", SDS_DOUBLE, dims);
   tmpId.ArrayAccess(&arrayHelper);
   for (int index = 0; index < (int)arrayHelper.Size(); index++)
   {
      arrayHelper[index] = coeffs[index];
   }

   id.Put("ha", ha);
   id.Put("dec", dec);
   id.Put("temp", m_tdFfpiSStruct->temp); /* Just dummy at moment */
   paramId->ShallowCopy(&id, true);
}

void CSurveyAction::ActionComplete(tdFfpiTaskType *details, const double *const expectedX, const double *const expectedY,
                                   const double *const measuredX, const double *const measuredY,
                                   const double ha, const double dec, const short fitType, const double plateTheta,
                                   drama::sds::Id *paramId)
{
   double coeffs[6];
   SetCoeffs(details, expectedX, expectedY, measuredX, measuredY, fitType, coeffs, plateTheta, paramId);

   ConstructReturnValue(details, expectedX, expectedY, measuredX, measuredY, coeffs, ha, dec, paramId);
}

void CSurveyAction::ActionThread(const drama::sds::Id &Arg)
{
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();
   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   if (details == nullptr)
   {
      DramaTHROW(TDFCANTASK__NOTINIT, "C_SURVEY: the structure pointer is null, please initialise the task!");
   }
   if (details->inUse)
   {
      DramaTHROW(TDFCANTASK__IN_USE, "C_SURVEY: TdFCanTask is running other actions.");
   }
   if (!Arg)
   {
      DramaTHROW(TDFCANTASK__NO_ARGUMENTS, "C_SURVEY: No input argument is provided.");
   }
   m_tdFfpiSStruct = new tdFfpiStype();
   drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
   drama::gitarg::String AreaArg(this, Arg, "area", 1, "", NoFlags);
   std::string strArea = AreaArg;
   if (strArea != "ALL" && strArea != "COEFFS" && strArea != "TEMP" && strArea != "FLEX")
   {
      DramaTHROW(TDFCANTASK__INV_INPUT_ARGUMENT, "C_SURVEY: TdFCanTask can only take \"ALL\", \"COEFFS\", \"TEMP\",\"FLEX\" type.");
   }
   if (strArea == "ALL")
      m_tdFfpiSStruct->area = _ALL;
   else if (strArea == "FLEX")
      m_tdFfpiSStruct->area = _FLEX;
   else if (strArea == "TEMP")
      m_tdFfpiSStruct->area = _TEMP;
   else
      m_tdFfpiSStruct->area = _COEFFS;

   drama::gitarg::Int<3, NUM_FIDUCIALS> NumArg(this, Arg, "numMarks", 2, 3, NoFlags);
   int iNum = NumArg;
   m_tdFfpiSStruct->numMarks = iNum;

   drama::gitarg::Id XArg(this, Arg, "x", 3, "", NoFlags);
   std::vector<unsigned long> checkDims;
   XArg.GetDims(&checkDims);
   if (checkDims.size() != 1 && iNum != (int)checkDims[0])
   {
      DramaTHROW_S(TDFCANTASK__INV_INPUT_ARGUMENT, "C_SURVEY: the input of X coordinates is invalid. The number of fidual is %d and the number of X coordinates is %d.\n", iNum, (int)(checkDims[0]));
   }
   drama::sds::ArrayReadHelper<long int> xArray;
   XArg.ArrayAccess(&xArray, &checkDims);
   for (int i = 0; i < iNum; i++)
   {
      m_tdFfpiSStruct->x[i] = xArray[i];
      DEBUG("%ld ", m_tdFfpiSStruct->x[i]);
   }
   DEBUG("\n");

   drama::gitarg::Id YArg(this, Arg, "y", 4, "", NoFlags);
   YArg.GetDims(&checkDims);
   if (checkDims.size() != 1 && iNum != checkDims[0])
   {
      DramaTHROW_S(TDFCANTASK__INV_INPUT_ARGUMENT, "C_SURVEY: the input of Y coordinates is invalid. The number of fidual is %d and the number of Y coordinates is %d.\n", iNum, (int)(checkDims[0]));
   }
   drama::sds::ArrayReadHelper<long int> yArray;
   YArg.ArrayAccess(&yArray, &checkDims);
   for (int i = 0; i < iNum; i++)
   {
      m_tdFfpiSStruct->y[i] = yArray[i];
      DEBUG("%ld ", m_tdFfpiSStruct->y[i]);
   }
   DEBUG("\n");

   drama::gitarg::Real<-30, 40> TempArg(this, Arg, "temp", 5, 0.0, NoFlags);
   double temp = TempArg;
   m_tdFfpiSStruct->temp = temp;

   double expectedX[NUM_FIDUCIALS], expectedY[NUM_FIDUCIALS];
   double measuredX[NUM_FIDUCIALS], measuredY[NUM_FIDUCIALS];
   long int offsetX, offsetY;

   short curFid, atFid, recordedFid, fitType;
   int nFailures;
   drama::ParSys parSysId(ThisTask->TaskPtr());

   double ha, dec;
   double plateTheta;

   if (m_tdFfpiSStruct->reset == YES)
   {
      if (m_tdFfpiSStruct->area != _COEFFS)
      {
         DramaTHROW(TDFCANTASK__INV_INPUT_ARGUMENT, "C_SURVEY: The survey code currently only supports the COEFFS flag");
      }
      MessageUser("C_SURVEY: Starting FPI Survey of %d marks", m_tdFfpiSStruct->numMarks);
      fitType = 6;
      nFailures = 0;
      curFid = 0;
      atFid = recordedFid = NO;
      offsetX = offsetY = 0;
      parSysId.Get("HA", &ha);
      parSysId.Get("DEC", &dec);
      parSysId.Get("PLATE_THETA", &plateTheta);
      parSysId.Put("SURVEY_PROG", 0.0);
      m_tdFfpiSStruct->reset = NO;
   }
   bool flag = true;

   while (!atFid && !recordedFid)
   {
      if (!atFid)
      {
         SearchForFid(offsetX, offsetY, curFid, &atFid);
      }
      else if (!recordedFid)
      {
         if (m_tdFfpiSStruct->found != YES)
         {
            if (nFailures < MAX_FAILURES)
            {
               bool checkFound = FidNotFound(expectedX, expectedY, curFid, &atFid, &recordedFid);
               if (!checkFound)
               {
                  flag = false;
                  break;
               }
            }
            else
            {
               flag = false;
               MessageUser("C_SURVEY: Search for fiducials index %d, at %ld,%ld failed\n",
                           curFid, m_tdFfpiSStruct->x[curFid], m_tdFfpiSStruct->y[curFid]);
               MessageUser("C_SURVEY: Too many failures to find fiducials (%d), aborting survey\n",
                           nFailures);
               break;
            }
         }
         else
         {
            /* found ok */
            RecordFid(expectedX, expectedY, measuredX, measuredY, &offsetX, &offsetY, &curFid, &atFid, &recordedFid);
         }
      }
   }
   drama::sds::Id newArg;
   if (flag)
   {
      // newArg = drama::sds::Id::CreateArgStruct();

      ActionComplete(details, expectedX, expectedY, measuredX, measuredY, ha, dec, fitType, plateTheta, &newArg);
   }
   SetReturnArg(&newArg);
   MessageUser("C_SURVEY: - Action complete.");
}

//  ------------------------------------------------------------------------------------------------

//                                    M a i n  P r o g r a m

int main()
{
   //  See the comments to BlockSIGUSR2() for an explanation of this bit of housekeeping.
   //  For testing with the version of CML that doesn't use signals, this is commented out.
   //  Actually, I suspect it should never be re-enabled.

   // BlockSIGUSR2();

   //  Create the main task object. (Note - don't use 'TheTask' as a variable name - it's
   //  defined as a macro, and things will get confusing.)

   TdFCanTask ThisTask("TdFCanTask");

   //  Do the initial setup of the amplifiers at this point, in the main thread. (I thought this
   //  might make a difference to the CML hangs, but it didn't.)

   // if (ThisTask.SetupAmps();

   //  Enter the DRAMA main loop.
   // const uint SIM_X_SIZE = 1088;
   // const uint SIM_Y_SIZE = 1456;
   // vimbacam::Task camTask(
   //     "10.88.16.199", false, vimbacam::PixelSize::PixelSize8bits,
   //     "Vimba", SIM_X_SIZE, SIM_Y_SIZE,
   //     gcam::BytesPerPixel::oneByte, /* IGNORED, but we had to supply something */
   //     "FPIVIMBACAM",                /* Log sys name */
   //     vimbacam::VimbaCamVersion,    /* Task version */
   //     vimbacam::VimbaCamDate,       /* Task date */
   //     "FPI Vimba Camera task");

   ThisTask.RunDrama();

   //  Housekeeping needed before CML destructors are run. See BlockSIGUSR2() comments.

   //  For testing with the version of CML that doesn't use signals, this is commented out.
   // UnblockSIGUSR2();

   return 0;
}

//  ------------------------------------------------------------------------------------------------

//                           P r o g r a m m i n g   N o t e s

/*

   o  It isn't clear at the moment what more the G_INIT action involves other than a homing of
      the axis/axes, so it isn't clear what this does that G_HOME doesn't.

   o  INITIALISE is traditionally a GIT action. Should this task be coded using the DRAMA-supplied
      GIT infrastructure, ie as a Git Task explicitly?

   o  I can't use MessageUser() in a routine like SetupAmps() which is part of the main task class
      rather than part of the code for an action. Instead, I let it set an error string if
      something goes wrong and let the calling action routine grab that and output it through
      MessageUser(). Is there a better way? Surely there is. What about having a sub-class of
      TAction that includes routines like SetupAmp(), and then having the action handlers inherit
      from that? But if the Amplifier objects (X1Amp, etc) are to be shared between actions, they
      need to be part of the task (I suppose they could be static items in that intermediate
      subclass of TAction, but that seems inelegant) and accessed through accessor routines. Is
      that better? (Temperamentally, I prefer not to have too much inheritance if I can avoid it.
      Making Actions a class is all very object-orienty, but it does introduce this sort of
      complexity.) But all this would be fine if I could call MessageUser directly from a routine
      like SetupAmps().

   o  If a linkage needs all move limits (and other things) to be the same for all amps, can
      we sensibly have a linkage of quite disparate amp types? (Eg X,Y,Jaw.) Is the answer to
      do with the units and the scaling between the ones the CML::Amps work in and the actual
      values?

   o  At some point, the _NT versions of the actions can be removed, but for the moment, it's
      handy to be able to test them occasionally.

   o  As of 23/3/18, the position regarding SIGUSR2 is this: there is the traditional, long-trusted,
      AAO-written, version of sem_timedwait() used - only under OS X - by CML. With this version in
      use, there are issues with DRAMA's blocking of all signals pretty much all the time,
      particularly in action handling threads. This shows up mainly when waiting for a move in
      MoveAxes() to complete when using WaitMoveDone(). The first move completes properly, but
      the wait for any second move hangs. Using WaitLinkedHome() seems to work most - perhaps even
      all - of the time. However, node guarding stops at the end of the first move and never seems
      to start again. Unblocking SIGUSR2 at the start of each such thread, including the main
      thread, and NEVER blocking it again, seems to fix this. However, even this didn't stop a
      hang in MoveAxes() if a KickNotifier was declared in the code just before the wait call.
      Moving the declaration to before the move was started (a few code lines earlier) helped,
      not not always. At that point, a new, lashed-up version of CML was produced with threads
      code that re-implemented all the sem_ calls using pthread conditions and mutexes. With this,
      all the hangs in WaitMoveDone() seem to have vanished (this needs soak testing - there has
      been one case where there might have been such a hang). However, this didn't fix the
      KickNotify issue. Finally, introducing a new version of KickNotify from Tony did seem to
      fix the problem (although it still isn't clear how best to interrupt the wait from the
      KickNotifier). Note that in all this there are a number of combinations of the various
      fixes that haven't been tried. It might be, for example, that we could go back to the
      signals-based CML, so long as we have the latest KickNotifier version, and so long as we
      always unblock SIGUSR2 and never reblock it, and still have everything working.
*/
