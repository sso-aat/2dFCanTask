/*
 * @Description:
 * @Author: lliu
 * @version:
 * @Date: 2024-08-24 15:59:31
 * @LastEditors: lliu
 * @LastEditTime: 2024-08-28 20:54:21
 */
#include "TdFGripperTask.h"
#include <memory>

static const std::string G_AmpNames[MAX_TDF_AMPS] = {
    "2dFsimGantryX1", "2dFsimGantryX2", "2dFsimGantryY", "2dFsimGantryZ", "2dFsimGantryJaw", "2dFsimGantryTheta"};

//  The configuration file that defines the set of CANBuses and simulation settings to be used.

static const std::string CONFIGURATION_FILE = "$TDFCTASK_DIR/TdFGripperTask.conf";

extern const char *const TdFGripperTaskVersion;
extern const char *const TdFGripperTaskDate;

typedef struct
{
    AmpId AxisId;    //  The axis in question.
    double Position; //  The demanded position.
    double Velocity; //  The demanded velocity. 0 => not specified.
} AxisDemand;

class InitialiseAction : public drama::thread::TAction
{
public:
    InitialiseAction(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
    ~InitialiseAction() {}

private:
    void ActionThread(const drama::sds::Id &) override;
};

class GHomeActionNT : public drama::thread::TAction
{
public:
    GHomeActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
    ~GHomeActionNT() {}

private:
    void ActionThread(const drama::sds::Id &) override;
};

class GMoveAxisActionNT : public drama::thread::TAction
{
public:
    GMoveAxisActionNT(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask) {}
    ~GMoveAxisActionNT() {}

private:
    void ActionThread(const drama::sds::Id &) override;
};

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

// P_SetActionNT is a thread version of SET action, it sets a signal parameter of tdfgripper task
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

// P_ReportActionNT is a thread version of Report action, it reports the value of a gripper task parameter
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

// P_SaveDefsActionNT is a thread version of SaveDef action, it saves Def into the tdfgripperDef.sds
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
    }

private:
    std::shared_ptr<tdFgrip_SFtype> m_tdFgripperSFStruct;
    std::weak_ptr<drama::Task> _theTask;
    std::shared_ptr<tdFgripperTaskType> details = nullptr;

private:
    void ActionThread(const drama::sds::Id &) override;

    bool MoveToSearchPosition(const long searchX, const long searchY, short *const atSearchXY,
                              short *const searchStarted);
    bool PerformCentroid(const shared_ptr<tdFgrip_CENtype> &cenWin, const double settletime, short *const centroided);

    bool CheckCentroid(short *const attempts, long *const searchX, long *const searchY, short *const atSearchXY,
                       short *const centroided, short *const foundIt, int *const i, int *const j, int *const k, short *const checkedCentroid,
                       short *const centroidRepeated, short *const repeatChecked);

    void CheckCent_ObjectHasNotYetBeenSeen(long *const searchX, long *const searchY, short *const atSearchXY, short *const centroided,
                                           short *const attempts, short *const foundIt, int *const i, int *const j, int *const k,
                                           short *const checkedCentroid, short *const centroidRepeated, short *const repeatChecked);

    void CheckCent_ObjectNotFound(long *const searchX, long *const searchY, short *const atSearchXY, short *const centroided,
                                  int *const i, int *const j, int *const k, short *const checkedCentroid,
                                  short *const centroidRepeated, short *const repeatChecked);

    void CheckCent_ObjectFound(long *const searchX, long *const searchY, short *const atSearchXY,
                               short *const centroided, short *const attempts, short *const foundIt, short *const checkedCentroid);

    bool CheckRepeatCentroid(short *const centroidRepeated, short *const repeatChecked);

    bool ActionComplete_CalculateMean(long *const searchX, long *const searchY);
    void ActionComplete_FoundItCheck(long *const searchX, long *const searchY, short *const foundIt);
    void ActionComplete(long searchX, long searchY, short &foundIt);
};

class CCentroidAction : public drama::thread::TAction
{
public:
    CCentroidAction(std::weak_ptr<drama::Task> theTask) : drama::thread::TAction(theTask), _theTask(theTask) {}
    ~CCentroidAction() {}

private:
    std::shared_ptr<tdFgrip_CENtype> m_tdFgripperCENStruct;
    std::weak_ptr<drama::Task> _theTask;
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
    }

private:
    std::shared_ptr<tdFgrip_ZCtype> m_tdFgripperZCStruct;
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
    }

private:
    std::shared_ptr<tdFgrip_SHtype> m_tdFgripperSHStruct;
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
    }

private:
    std::shared_ptr<tdFgrip_Stype> m_tdFgripperSStruct;
    std::weak_ptr<drama::Task> _theTask;
    std::shared_ptr<dFgripperTaskType> details = nullptr;
    const int MAX_FAILURES = 4;
    const int MIN_MARKS = 4;
    const double RMS_WARNING = 20.0;

private:
    void ActionThread(const drama::sds::Id &) override;
    bool FidNotFound(double *const expectedX, double *const expectedY,
                     const short curFid, short *const atFid, short *const recordedFid);
    bool SearchForFid(const long offsetX, const long offsetY, const short curFid, short *const atFid);
    void RecordFid(double *const expectedX, double *const expectedY, double *const measuredX,
                   double *const measuredY, long *const offsetX, long *const offsetY,
                   short *const curFid, short *const atFid, short *const recordedFid);
    void ActionComplete(const double *const expectedX, const double *const expectedY,
                        const double *const measuredX, const double *const measuredY,
                        const double ha, const double dec, const short fitType, const double plateTheta,
                        drama::sds::Id *paramId);
    void SetCoeffs(const double *const expectedX, const double *const expectedY,
                   const double *const measuredX, const double *const measuredY,
                   const short fitType, double *const coeffs, const double plateTheta, drama::sds::Id *paramId);
    bool OffsetAndDisplayResults(const double platetheta, const short fitType, const short centerFid, double measuredArray[][2], double fiducialArray[][2],
                                 double *const coeffs, drama::sds::Id *paramId);
    void tdFgripperSurveyCheckCoeffs(const double *current, const double *newcoeffs,
                                     int plate, const double plateTheta, drama::sds::Id *paramId);
    void DisplayResultsBasic(const int newmodel, double fiducialArray[][2],
                             double cal[][2], const double xrms, const double yrms, const double rrms, drama::sds::Id *paramId);
    void DisplayResultsInEncoderUnits(const double plateTheta,
                                      const int newmodel, const double *const coeffs, double measuredArray[][2]);
    void ConstructReturnValue(const double *const expectedX, const double *const expectedY, const double *const measuredX,
                              const double *const measuredY, const double *const coeffs, const double ha,
                              const double dec, drama::sds::Id *paramId);
};

class TdFGripperTask : public drama::Task
{
public:
    //  Constructor
    TdFGripperTask(const std::string &taskName);
    //  Destructor
    ~TdFGripperTask();
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

    // Initialise the gripper struct
    bool InitialisegripperMainStruct(void);
    // returns the pointer of the gripper task
    std::shared_ptr<tdFgripperTaskType> tdFgripperGetMainStruct() { return I_tdFgripperMainStruct; }
    void tdFgripperSetMainStruct(std::shared_ptr<tdFgripperTaskType> tdFgripperMainStruct) { I_tdFgripperMainStruct = tdFgripperMainStruct; }
    bool tdFgripperDefRead(short loadingFiles, short check);
    bool SetParameter(string &ParameterName, string &ParameterValue);
    bool tdFgripperDefWrite(short savingFiles, short check = 1);

    bool tdFgripperIlocks(long int ilocks);
    bool tdFgripperUpdatePos(short updateIdeal, short useDpr, short displayText);
    // lliu added on 06-06-2024 to compute the theata based on the gantry position;
    double tdFautoThetaPos(long int x, long int y);
    // lliu added on 06-06-2024 to check if a position is forbidden
    int tdFforbidden(double x, double y, double theta, double rq, double ri,
                     double ro, double hw, double jhwp, double jhwm, double jl, double clearance,
                     int *inside, int *outside);
    // lliu added on 26-06-2024 to check if the position of an image before and after centorid
    void tdFgripperPreExp();
    void tdFgripperPostExp();

    void tdFgripperConvertFromFP(int displayText, long int xFp, long int yFp, long int zFp, double tFp,
                                 long int jFp, double plateTheta, short level, double *xCon,
                                 double *yCon, double *zCon, double *tCon, double *jCon);
    void tdFgripperConvertFromEnc(int displayText, int xEnc, int yEnc, int zEnc, int tEnc,
                                  int jEnc, double plateTheta, short level, double *xCon,
                                  double *yCon, double *zCon, double *tCon, double *jCon);

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
    bool tdFgripperReadFile(drama::sds::Id &defId);
    // Write the parameters into a file
    bool tdFgripperWriteFile(drama::sds::Id &defId);
    // Read the flex file
    bool tdFgripperReadFlexFile(drama::sds::Id &defId);

    void tdFgripperFlexure(long int x, long int y,
                           double ha, double dec, TdfFlexType *pars, long int *dx, long int *dy);
    double GetZAdjustment(const int debug, const double x, const double y, tdFzmap *const zmap);
    void CalcGripperToFPIdistortion(const double x, const double y, double *const dx, double *const dy);

    void tdFstateBitSet(unsigned char bit);
    void tdFstateBitClear(unsigned char bit);

    // lliu added on 17-05-2024 to check the target poisiton
    void tdFgripperPositionCheck(int Index, CML::uunit &Position);
    void tdFgripperEncPos(short useDpr, short displayText, tdFencPos *position);
    // --------------------------------------------------------------------------
    // Gantry movement related actions
    // --------------------------------------------------------------------------
    //  The action handler for the INITIALISE action.
    InitialiseAction I_InitialiseActionObj;

    // The new action handler for G_EXIT action.
    GEXITActionNT I_GExitActionNTObj;
    // The new action handler for G_RESET action.
    GRESETActionNT I_GResetActionNTObj;

    // --------------------------------------------------------------------------
    // Gantry movement related actions
    // --------------------------------------------------------------------------
    //  The action handler for the G_MOVE_AXIS_NT (threaded) action.
    GMoveAxisActionNT I_GMoveAxisActionNTObj;
    //  action handler for the G_HOME_NT (threaded) action
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

    std::shared_ptr<tdFgripperTaskType> I_tdFgripperMainStruct = nullptr;
    drama::ParSys I_TdFGripperTaskParSys;
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
            else if (TcsUtil::MatchCaseBlind(AxisName, "Y"))
            {
                Demand.AxisId = Y_AMP;
                AxisDemands.push_back(Demand);
            }
            else if (TcsUtil::MatchCaseBlind(AxisName, "Z"))
            {
                Demand.AxisId = Z_AMP;
                AxisDemands.push_back(Demand);
            }
            else if (TcsUtil::MatchCaseBlind(AxisName, "Jaw"))
            {
                Demand.AxisId = JAW_AMP;
                AxisDemands.push_back(Demand);
            }
            else if (TcsUtil::MatchCaseBlind(AxisName, "Theta"))
            {
                Demand.AxisId = THETA_AMP;
                AxisDemands.push_back(Demand);
            }
            else
            {
                Error = "Invalid axis specification: " + AxesList[Index];
                OKSoFar = false;
                break;
            }
        }
    }
    if (!OKSoFar)
        AxisDemands.clear();

    return AxisDemands;
}

static bool WhichAxes(const std::string &Axes, bool &X, bool &Y, bool &Z, bool &Theta, bool &Jaw)
{
    X = Y = Z = Theta = Jaw = false;

    std::string AxisNames[] = {"X", "Y", "Z", "JAW", "THETA"};
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

void TdFGripperTask::tdFgripperEncPos(short useDpr, short displayText, tdFencPos *position)
{
    if (useDpr)
    {
        if (displayText)
            DEBUG("Assuming current encoder position is previous target position.\n");
        position->x = I_tdFgripperMainStruct->atEnc.x;
        position->y = I_tdFgripperMainStruct->atEnc.y;
        position->z = I_tdFgripperMainStruct->atEnc.z;
        position->theta = I_tdFgripperMainStruct->atEnc.theta;
        position->jaw = I_tdFgripperMainStruct->atEnc.jaw;
    }
    else
    {
        position->x = I_tdFgripperMainStruct->toEnc.x;
        position->y = I_tdFgripperMainStruct->toEnc.y;
        position->z = I_tdFgripperMainStruct->toEnc.z;
        position->theta = I_tdFgripperMainStruct->toEnc.theta;
        position->jaw = I_tdFgripperMainStruct->toEnc.jaw;
    }
}

void TdFGripperTask::tdFgripperPreExp()
{
    tdFimagePos *details = &I_tdFgripperMainStruct->imagePos;
    if (details->enable)
        tdFgripperEncPos(YES, YES, &details->enc);
    else
        DEBUG("tdFgripperPreExp cannot be implemented because details->enable is set to NO.\n");
}

void TdFGripperTask::tdFgripperPostExp()
{
    tdFimagePos *details = &I_tdFgripperMainStruct->imagePos;
    if (details->enable)
    {
        tdFencPos now;
        double atFpX, atFpY, atFpZ, atFpJ, plateTheta;
        tdFgripperEncPos(YES, YES, &now);
        details->enc.theta = (now.theta + details->enc.theta) / 2;
        details->enc.x = (now.x + details->enc.x) / 2;
        details->enc.y = (now.y + details->enc.y) / 2;
        details->enc.z = (now.z + details->enc.z) / 2;
        details->enc.jaw = (now.jaw + details->enc.jaw) / 2;
        I_TdFGripperTaskParSys.Get("PLATE_THETA", &plateTheta);
        tdFgripperConvertFromEnc(details->displayText & _DEBUG,
                                 details->enc.x, details->enc.y,
                                 details->enc.z, details->enc.theta,
                                 details->enc.jaw, plateTheta, _FULL,
                                 &atFpX, &atFpY, &atFpZ,
                                 &details->p.theta, &atFpJ);

        details->p.x = doubleToLong(atFpX);
        details->p.y = doubleToLong(atFpY);
        details->p.z = doubleToLong(atFpZ);
        details->p.jaw = doubleToLong(atFpJ);
        DEBUG("Average gantry position during image is x:%ld, y:%ld, z:%ld, theta:%g.\n",
              details->p.x, details->p.y);
    }
}

void TdFGripperTask::tdFgripperConvertFromFP(int displayText, long int xFp, long int yFp, long int zFp, double tFp,
                                             long int jFp, double plateTheta, short level, double *xCon,
                                             double *yCon, double *zCon, double *tCon, double *jCon)
{
    double x_fp, y_fp, z_fp, t_fp, j_fp,
        x_enc, y_enc, z_enc, t_enc, j_enc,
        ha, dec;
    long int dx = 0,
             dy = 0;
    double ddx = 0;
    double ddy = 0;

    short debugging = 0;
    if (displayText)
    {
        I_TdFGripperTaskParSys.Get("DEBUG_CVT", &debugging);
        if (debugging)
            DEBUG("FromFP:Converting %ld, %ld, level %d",
                  xFp, yFp, level);
    }

    I_TdFGripperTaskParSys.Get("HA", &ha);
    I_TdFGripperTaskParSys.Get("DEC", &dec);

    x_fp = (double)(xFp * cos(plateTheta) - yFp * sin(plateTheta));
    y_fp = (double)(yFp * cos(plateTheta) + xFp * sin(plateTheta));
    t_fp = tFp - plateTheta;
    j_fp = (double)jFp;

    if (debugging)
    {
        DEBUG("  FromFP:Due to field plate rotation of %.2f degrees - %ld,%ld to %.1f, %.1f",
              plateTheta * 180.0 / PI, xFp, yFp, x_fp, y_fp);
    }
    /*
     *  Apply z-map correction, if required.
     */
    if (level == _FULL_NO_ZMAP)
        z_fp = zFp;
    else
        z_fp = zFp + GetZAdjustment(debugging, xFp, yFp, &I_tdFgripperMainStruct->convert.zmap);

    if ((level == _ALL) || (level == _COEFFS))
    {
        *xCon = x_fp;
        *yCon = y_fp;
        *zCon = z_fp;
        *tCon = t_fp;
        *jCon = j_fp;
        if (debugging)
            DEBUG("  FromFP:Conversion complete %.1f, %.1f - level _ALL/_COEFFS",
                  *xCon, *yCon);
        return;
    }

    /*
     * Remove the Gripper to FPI distortion.
     */
    CalcGripperToFPIdistortion(debugging,
                               0, x_fp, y_fp, &ddx, &ddy);
    x_fp += ddx;
    y_fp += ddy;
    if (debugging)
        DEBUG("  FromFP:Adding distortion correction of %.1f, %.1f -> %.1f, %.1f",
              ddx, ddy, x_fp, y_fp);

    /*
     * Adjust for offset of plate one center fiducial from plate 2 center fiducial.
     */
    if (I_tdFgripperMainStruct->configPlate == 1)
    {
        if (I_tdFgripperMainStruct->plateOneDontRemove)
        {
            if (debugging)
                DEBUG("  FromFP:Converting for plate 1, but offset removal disabled");
        }
        else
        {
            double t_x = x_fp;
            double t_y = y_fp;

            x_fp = t_x - (double)(*(I_tdFgripperMainStruct->pars.plt1CenterFidOffsetX));
            y_fp = t_y - (double)(*(I_tdFgripperMainStruct->pars.plt1CenterFidOffsetY));
            if (debugging)
                DEBUG("  FromFP:Plate 1 position changed by %d, %d from %.1f, %.1f to %.1f, %.1f",
                      -1 * (*(I_tdFgripperMainStruct->pars.plt1CenterFidOffsetX)),
                      -1 * (*(I_tdFgripperMainStruct->pars.plt1CenterFidOffsetY)),
                      t_x, t_y, x_fp, y_fp);
        }
    }
    else if (debugging)
        DEBUG("  FromFP:Conversion is for plate 0 - no offset removal");

    slaXy2xy(x_fp, y_fp,                             /* Field plate coordinates */
             I_tdFgripperMainStruct->convert.coeffs, /* Transformation matrix   */
             &x_enc, &y_enc);                        /* Encoder coordinates     */
    t_enc = (double)(1000000) * t_fp - (double)details->convert.thetaShift;
    j_enc = j_fp - (double)I_tdFgripperMainStruct->convert.jawShift;
    z_enc = z_fp - (double)I_tdFgripperMainStruct->convert.zShift;

    if (debugging)
        DEBUG("  FromFP:Lin model, converted %.1f, %.1f to %.1f, %.1f",
              x_fp, y_fp, x_enc, y_enc);
    if (level == _TEMP)
    {
        *xCon = x_enc;
        *yCon = y_enc;
        *zCon = z_enc;
        *tCon = t_enc;
        *jCon = j_enc;
        if (debugging)
            DEBUG("  FromFP:Conversion complete %.1f, %.1f - level _TEMP",
                  *xCon, *yCon);
        return;
    }

    if (level == _FLEX)
    {
        *xCon = x_enc;
        *yCon = y_enc;
        *zCon = z_enc;
        *tCon = t_enc;
        *jCon = j_enc;
        if (debugging)
            DEBUG("  FromFP:Conversion complete %.1f, %.1f - level _FLEX",
                  *xCon, *yCon);
        return;
    }

    /*
     *  Compensate for system deflections.
     */
    tdFgripperFlexure((long int)x_enc, (long int)y_enc, ha, dec,
                      &I_tdFgripperMainStruct->convert.flex, &dx, &dy);

    if (debugging)
        DEBUG("  FromFP:Removing flexure of %ld, %ld microns",
              dx, dy);
    *xCon = x_enc - (double)dx;
    *yCon = y_enc - (double)dy;
    *zCon = z_enc;
    *tCon = t_enc;
    *jCon = j_enc;

    if (debugging)
        DEBUG("  FromFP:Conversion complete - full model %.1f, %.1f %.1f",
              *xCon, *yCon, *zCon);
}

void TdFGripperTask::CalcGripperToFPIdistortion(const double x, const double y, double *const dx,
                                                double *const dy)
{
    double x_s;
    double y_s;
    double x_p2;
    double y_p2;
    double x_p3;
    double y_p3;
    double x_p4;
    double y_p4;
    double angle;
    double angle_sine;
    double angle_cosine;

    /*
     * Scale X/Y down.
     */
    x_s = x / distP18;
    y_s = y / distP18;
    /*
     * Work out the values we want.
     */
    x_p2 = x_s * x_s;
    y_p2 = y_s * y_s;
    x_p3 = x_p2 * x_s;
    y_p3 = y_p2 * y_s;
    x_p4 = x_p3 * x_s;
    y_p4 = y_p3 * y_s;
    /*
     * Work out the angle - I am unclear if atan2() handles the zero point
     * nicely on VxWorks- so I do it by hand.
     */
    if (0 == x)
    {
        if (0 == y)
        {
            angle = 0;
            angle_sine = 0;
            angle_cosine = 1;
        }
        else if (y < 0)
        {
            angle = -M_PI_2;
            angle_sine = -1;
            angle_cosine = 0;
        }
        else
        {
            angle = M_PI_2;
            angle_sine = 1;
            angle_cosine = 0;
        }
    }
    else
    {
        angle = atan2(x, y);
        angle_sine = sin(angle);
        angle_cosine = cos(angle);
    }
    /*
     * Now do the distortion calculation.
     */

    *dx =
        distP1 * x_p2 + distP2 * y_p2 +
        distP3 * x_p3 + distP4 * y_p3 +
        distP5 * x_p4 + distP6 * y_p4 +
#if 0
        distP7 * angle_sine +
        distP8 * angle_cosine;
#endif
        distP7;

    *dy =
        distP9 * x_p2 + distP10 * y_p2 +
        distP11 * x_p3 + distP12 * y_p3 +
        distP13 * x_p4 + distP14 * y_p4 +
#if 0
        distP15 * angle_sine +
        distP16 * angle_cosine;
#endif
        distP15;

    *dx *= -1;
    *dy *= -1;
}

double TdFGripperTask::GetZAdjustment(const int debug, const double x, const double y, tdFzmap *const zmap)
{
    double zadj_x = 0, zadj_y = 0;
    double result;
    /*
     * If we remove the centre position z value from each of the
     * extremes, then we are dealing with a simple line slope issue, with
     * the slope different in each of the four directions.
     */
    if (x == 0)
    {
        zadj_x = 0;
    }
    else if (x > 0)
    {
        zadj_x = x * (zmap->_xp - zmap->_cen) / (double)(FIELD_RAD);
    }
    else if (x < 0)
    {
        /* We don't want x to get negative for the calc, so negate it */
        zadj_x = -1 * x * (zmap->_xm - zmap->_cen) / (double)(FIELD_RAD);
    }

    if (y == 0)
    {
        zadj_y = 0;
    }
    else if (y > 0)
    {
        zadj_y = y * (zmap->_yp - zmap->_cen) / (double)(FIELD_RAD);
    }
    else if (y < 0)
    {
        zadj_y = -1 * y * (zmap->_ym - zmap->_cen) / (double)(FIELD_RAD);
    }

    /*
     * We add our two adjustments together and the centre position
     * value and return the result.
     */
    result = zadj_x + zadj_y + zmap->_cen;

    if (debug)
    {
        DEBUG("GetZAdustmnet for %.1f,%.1f = %.1f, %.1f, result %.1f",
              x, y, zadj_x, zadj_y, result);
    }

    return result;
}

int TdFGripperTask::tdFforbidden(double x, double y, double theta, double rq, double ri,
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
                x -= 2. * hw;         /* Next sector may be the  */
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
                y += 2. * hw;         /* Previous sector can be   */
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

TdFGripperTask::TdFGripperTask(const std::string &taskName) : drama::Task(taskName, GLOBAL_BUFFER_SPACE),
                                                              I_InitialiseActionObj(TaskPtr()),
                                                              I_CanAccessInitialised(false),
                                                              I_TdFGripperTaskParSys(TaskPtr()),
                                                              I_GParkGantryActionNTObj(TaskPtr()),
                                                              I_GUnParkActionNTObj(TaskPtr()),
                                                              I_GHomeActionNTObj(TaskPtr()),
                                                              I_GMoveOffsetActionNTObj(TaskPtr()),
                                                              I_GMoveActionNTObj(TaskPtr()),
                                                              I_GMoveAxisActionNTObj(TaskPtr()),
                                                              I_GExitActionNTObj(TaskPtr()),
                                                              I_GResetActionNTObj(TaskPtr()),
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
                                                              I_pCameraPath(TaskPtr(), "VIMBACAM", "", "/home/twodf/Codes/vimbacam/vimbacam")
{
    //  cml is a global defined by the CML library. Logging everything is a very good idea,
    //  even for a production system, and it's essential during development. Note that the
    //  logs produced are timestamped in the same way as the CANOpen.log files produced by
    //  the simulator, and can be compared for diagnostic purposes.

    CML::cml.SetDebugLevel(CML::LOG_EVERYTHING);

    //  Set up the various actions. Note that 'INITIALISE' which initialises the task as a
    //  whole, is quite different to 'G_INIT' which initialises the 2dF gantry. (There
    //  will later be an 'F_INIT' which initialises the gripper gantry, and others.)

    Add("INITIALISE", drama::MessageHandlerPtr(&I_InitialiseActionObj, drama::nodel()));
    Add("G_MOVE_AXIS_NT", drama::MessageHandlerPtr(&I_GMoveAxisActionNTObj, drama::nodel()));
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
    // add new Set action which sets the gripper task parameters;
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

    // add new Report action which reports the parameters of gripper task
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
        I_TdFGripperTaskParSys.Create("PMAC_LIM_X_POS", (INT32)(528000));
        I_TdFGripperTaskParSys.Create("PMAC_LIM_X_NEG", (INT32)(-16300));
        I_TdFGripperTaskParSys.Create("PMAC_LIM_Y_POS", (INT32)(537400));
        I_TdFGripperTaskParSys.Create("PMAC_LIM_Y_NEG", (INT32)(-3400));
        drama::Buffers buffer(2000, 2, 2000000, 1);
        I_pCameraPath.SetBuffers(buffer);
        std::string LoadArg = "-i 10.88.16.199 -n VIMBACAM -p 8bit";
        I_pCameraPath.SetArgument(LoadArg);
    }
}

TdFGripperTask::~TdFGripperTask()
{

    I_X1Amp = I_X2Amp = I_YAmp = I_ZAmp = I_ThetaAmp = I_JawAmp = nullptr;
}

bool TdFGripperTask::tdFgripperDefWrite(short savingFiles, short check)
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
        drama::sds::Id defId;
        if (tdFgripperWriteFile(defId))
        {
            if (defId)
            {
                fName = pathName + "tdFgripperDefsSave.sds";
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

bool TdFGripperTask::tdFgripperDefRead(short loadingFiles, short check)
{
    string pathName = TDFPT_PARAM_DIR;
    string fName;

    if (!boost::filesystem::exists(pathName) || !boost::filesystem::is_directory(pathName))
    {
        I_ErrorString += "The folder directory is not correct.\n";
        DramaTHROW(TDFGRIPPERTASK__NO_ENV_VAR, "The folder directory is not correct.");
    }
    if (loadingFiles & DEFS_FILE)
    {
        fName = pathName + "tdFgripperDefs.sds";
        if (!boost::filesystem::exists(fName) || !boost::filesystem::is_regular_file(fName))
        {
            I_ErrorString += "tdFgripperDefs.sds doesn't exist.\n";
            DramaTHROW(TDFGRIPPERTASK__NO_ENV_VAR, "tdFgripperDefs.sds doesn't exist.");
        }
        drama::sds::Id defId(drama::sds::Id::FromFile(fName));
        bool readRes = tdFgripperReadFile(defId);
        if (!readRes)
        {
            I_ErrorString += "Failed to read tdFgripperDefs.sds\n";
            DEBUG("\nREAD tdFgripperDefs.sds: %s", I_ErrorString.c_str());
            DramaTHROW(TDFGRIPPERTASK__READ_ERROR, "Failed to read tdFgripperDefs.sds.");
        }
        if (check & SHOW)
        {
            DEBUG("\nREAD tdFgripperDefs.sds: %s", fName.c_str());
            DEBUG("\nThe parameter array is:\n");
            defId.List();
        }
    }
    if (loadingFiles & FLEX_FILE)
    {
        fName = pathName + "tdFgripperFlex.sds";
        if (!boost::filesystem::exists(fName) || !boost::filesystem::is_regular_file(fName))
        {
            I_ErrorString += "tdFgripperFlex.sds doesn't exist.\n";
            DramaTHROW(TDFGRIPPERTASK__NO_ENV_VAR, "tdFgripperFlex.sds doesn't exist.");
        }
        drama::sds::Id defId(drama::sds::Id::FromFile(fName));
        bool readRes = tdFgripperReadFlexFile(defId);
        if (!readRes)
        {
            I_ErrorString += "Failed to read tdFgripperFlex.sds\n";
            DEBUG("READ tdFgripperFlex.sds: %s", I_ErrorString.c_str());
            DramaTHROW(TDFGRIPPERTASK__READ_ERROR, "Failed to read tdFgripperFlex.sds.");
        }
        if (check & SHOW)
        {
            DEBUG("\nREAD tdFgripperFlex.sds: %s", fName.c_str());
            DEBUG("\nThe parameter array is :\n");
            defId.List();
        }
    }

    return true;
}

bool TdFGripperTask::tdFgripperWriteFile(drama::sds::Id &defId)
{
    std::shared_ptr<tdFgripperTaskType> details = tdFgripperGetMainStruct();
    drama::sds::Id tmpId, tmp2Id;
    double dParam;
    unsigned long int dimVal = 6;
    long int lIntParam;
    short shortParam;
    std::vector<unsigned long int> dims;
    dims.push_back(dimVal);

    if (details == nullptr)
    {
        DramaTHROW(TDFGRIPPERTASK__MALLOCERR, "Malloc error.");
    }
    defId = drama::sds::Id::CreateTopLevel(
        "tdFgripperDefs", SDS_STRUCT);
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
    lIntParam = I_TdFGripperTaskParSys.GetInt("XY_VEL");
    tmpId.Put("XY_VEL", (INT32)lIntParam);

    lIntParam = (long int)stepSize;
    lIntParam = I_TdFGripperTaskParSys.GetInt("STEP_SIZE");
    tmpId.Put("STEP_SIZE", (INT32)lIntParam);

    lIntParam = (long int)maxError;
    lIntParam = I_TdFGripperTaskParSys.GetInt("MAX_ERROR");
    tmpId.Put("MAX_ERROR", (INT32)lIntParam);

    lIntParam = (long int)posTol;
    lIntParam = I_TdFGripperTaskParSys.GetInt("POS_TOL");
    tmpId.Put("POS_TOL", (INT32)lIntParam);

    shortParam = (short)attempts;
    shortParam = I_TdFGripperTaskParSys.GetInt("POS_ATTEMPTS");
    tmpId.Put("POS_ATTEMPTS", shortParam);

    lIntParam = (long int)fibInImgThres;
    lIntParam = I_TdFGripperTaskParSys.GetInt("FIBRE_IN_IMAGE");
    tmpId.Put("FIBRE_IN_IMAGE", (INT32)lIntParam);

    dParam = (double)settleTime;
    dParam = I_TdFGripperTaskParSys.GetDouble("SETTLE_TIME");
    tmpId.Put("SETTLE_TIME", dParam);

    lIntParam = (long int)timeoutFac;
    lIntParam = I_TdFGripperTaskParSys.GetInt("TIMEOUT_FAC");
    tmpId.Put("TIMEOUT_FAC", (INT32)lIntParam);

    lIntParam = (long int)xaccuracy;
    lIntParam = I_TdFGripperTaskParSys.GetInt("X_ACCURACY");
    tmpId.Put("X_ACCURACY", (INT32)lIntParam);

    lIntParam = (long int)yaccuracy;
    lIntParam = I_TdFGripperTaskParSys.GetInt("Y_ACCURACY");
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
    lIntParam = I_TdFGripperTaskParSys.GetInt("VFG_OP_ENABLE");
    tmpId.Put("VFG_OP_ENABLE", (INT32)lIntParam);

    dParam = (double)zerocamCenWait;
    dParam = I_TdFGripperTaskParSys.GetDouble("ZEROCAM_CENWAIT");
    tmpId.Put("ZEROCAM_CENWAIT", dParam);

    lIntParam = I_TdFGripperTaskParSys.GetInt("PMAC_LIM_X_POS");
    tmpId.Put("PMAC_LIM_X_POS", (INT32)lIntParam);

    lIntParam = I_TdFGripperTaskParSys.GetInt("PMAC_LIM_X_NEG");
    tmpId.Put("PMAC_LIM_X_NEG", (INT32)lIntParam);

    lIntParam = I_TdFGripperTaskParSys.GetInt("PMAC_LIM_Y_POS");
    tmpId.Put("PMAC_LIM_Y_POS", (INT32)lIntParam);

    lIntParam = I_TdFGripperTaskParSys.GetInt("PMAC_LIM_Y_NEG");
    tmpId.Put("PMAC_LIM_Y_NEG", (INT32)lIntParam);

    lIntParam = (long int)shortZeroX;
    lIntParam = I_TdFGripperTaskParSys.GetInt("PLT1_CFID_OFF_X");
    tmpId.Put("PLT1_CFID_OFF_X", (short)lIntParam);

    lIntParam = (long int)shortZeroY;
    lIntParam = I_TdFGripperTaskParSys.GetInt("PLT1_CFID_OFF_Y");
    tmpId.Put("PLT1_CFID_OFF_Y", (short)lIntParam);

    tdFgripperSetMainStruct(details);
    return true;
}

bool TdFGripperTask::tdFgripperReadFile(drama::sds::Id &defId)
{
    std::shared_ptr<tdFgripperTaskType> details = tdFgripperGetMainStruct();

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
        I_ErrorString += "Failed to read the tdFgripperDef.sds.\n";
        DramaTHROW(TDFGRIPPERTASK__READ_ERROR, "Failed to read the tdFgripperDef.sds.");
    }

    /*
     *  Set camera constant parameters.
     */
    details->freeImg.shutter = GCAM_OPEN;
    details->freeImg.updateTime = 0.005;
    details->freeImg.exposureTime = 0.001667;
    details->freeImg.camNo = 1;
    details->freeImg.xMax = CAM_X_SPAN;
    details->freeImg.yMax = CAM_Y_SPAN;
    details->freeImg.PixelSize = PIXEL_SIZE;

    /*
     *  Read free-focus image details.
     */
    tmpId = defId.Find("freeImage");
    if (tmpId)
    {
        tmpId.Get("bias", &sParam);
        DEBUG("The bias of freeImage is %d\n", (int)sParam);
        details->freeImg.bias = (int)sParam;

        tmp2Id = tmpId.Find("camCoeffs");
        if (tmp2Id)
        {
            tmp2Id.Get(6 * sizeof(double), details->freeImg.camCoeffs, &actlen, 0);
            if (actlen != 6 * sizeof(double)) // this is not correct actlen returns bytes values
            {
                I_ErrorString += "The field of camCoeffs is invalid.\n";
                DramaTHROW(TDFGRIPPERTASK__READ_ERROR, "The field of camCoeffs is invalid.");
            }

            // details->freeImg.camCoeffs[0] = 97.69348;
            // details->freeImg.camCoeffs[1] = 0.00672548;
            // details->freeImg.camCoeffs[2] = -0.213328;
            // details->freeImg.camCoeffs[3] = 169.0098;
            // details->freeImg.camCoeffs[4] = -0.2252;
            // details->freeImg.camCoeffs[5] = -0.003135;
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
            DramaTHROW(TDFGRIPPERTASK__READ_ERROR, "Cannot find the field of camCoeffs.");
        }
    }
    else
    {
        I_ErrorString += "Cannot find the field of freeImage.\n";
        DramaTHROW(TDFGRIPPERTASK__READ_ERROR, "Cannot find the field of freeImage.");
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

    tmpId = defId.Find("normWindow");
    if (tmpId)
    {

        tmpId.Get("xCen", &dParam);
        DEBUG("xCen from file is %f\n", dParam);
        details->normWin.xCen = dParam;
        // DEBUG("normWin.xCen is %f\n", details->normWin.xCen);

        tmpId.Get("yCen", &dParam);
        DEBUG("yCen from file is %f\n", dParam);
        details->normWin.yCen = dParam;
        // DEBUG("normWin.yCen is %f\n", details->normWin.yCen);

        tmpId.Get("xSpan", &sParam);
        DEBUG("xSpan from file is %hd\n", sParam);
        details->normWin.xSpan = (int)sParam;
        // DEBUG("normWin.xSpan is %d\n", details->normWin.xSpan);

        tmpId.Get("ySpan", &sParam);
        DEBUG("ySpan from file is %hd\n", sParam);
        details->normWin.ySpan = (int)sParam;
        // DEBUG("normWin.ySpan is %d\n", details->normWin.ySpan);
    }

    /*
     *  Read search window details.
     */
    tmpId = defId.Find("searchWindow");
    if (tmpId)
    {

        tmpId.Get("xCen", &dParam);
        details->searchWin.xCen = dParam;

        tmpId.Get("yCen", &dParam);
        details->searchWin.yCen = dParam;

        tmpId.Get("xSpan", &sParam);
        details->searchWin.xSpan = (int)sParam;

        tmpId.Get("ySpan", &sParam);
        details->searchWin.ySpan = (int)sParam;
    }

    /*
     *  Get parameters used to convert between field-plate and encoder units.
     */
    tmpId = defId.Find("conversion");
    if (tmpId)
    {
        tmp2Id = tmpId.Find("coeffs");
        if (tmp2Id)
        {
            tmp2Id.Get(6 * sizeof(double), details->convert.coeffs, &actlen, 0);
            if (actlen != 6 * sizeof(double)) //
            {
                I_ErrorString += "The field of conversion coeffs is invalid.\n";
                DramaTHROW(TDFGRIPPERTASK__READ_ERROR, "The field of conversion coeffs is invalid.");
            }
            // SdsFreeId(tmpId, status);
            // SdsFreeId(tmp2Id, status);
            // details->convert.coeffs[0] = 47386;
            // details->convert.coeffs[1] = 1;
            // details->convert.coeffs[2] = 0;
            // details->convert.coeffs[3] = 65974;
            // details->convert.coeffs[4] = 0;
            // details->convert.coeffs[5] = 1;

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
            DramaTHROW(TDFGRIPPERTASK__READ_ERROR, "Cannot find the field of coeffs.");
        }
    }
    else
    {
        I_ErrorString += "Cannot find the field of conversion.\n";
        DramaTHROW(TDFGRIPPERTASK__READ_ERROR, "Cannot find the field of conversion.");
    }

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
    tmpId = defId.Find("parameters");
    if (tmpId)
    {
        tmpId.Get("XY_VEL", &lIntParam);
        if (lIntParam)
        {

            DEBUG("XY_VEL from file is %ld\n", lIntParam);
            I_TdFGripperTaskParSys.Put("XY_VEL", (INT32)lIntParam);
            lIntParam = 0;
        }

        tmpId.Get("STEP_SIZE", &lIntParam);
        if (lIntParam)
        {

            DEBUG("STEP_SIZE from file is %ld\n", lIntParam);
            I_TdFGripperTaskParSys.Put("STEP_SIZE", (INT32)lIntParam);
            lIntParam = 0;
        }

        tmpId.Get("MAX_ERROR", &lIntParam);
        if (lIntParam)
        {
            I_TdFGripperTaskParSys.Put("MAX_ERROR", (INT32)lIntParam);
            lIntParam = 0;
        }

        tmpId.Get("POS_TOL", &lIntParam);
        if (lIntParam)
        {
            I_TdFGripperTaskParSys.Put("POS_TOL", (INT32)lIntParam);
            lIntParam = 0;
        }

        tmpId.Get("POS_ATTEMPTS", &sParam);
        if (sParam)
        {
            I_TdFGripperTaskParSys.Put("POS_ATTEMPTS", (short)sParam);
            // DEBUG("attempts is %hd\n", attempts);
            sParam = 0;
        }

        tmpId.Get("FIBRE_IN_IMAGE", &lIntParam);
        if (lIntParam)
        {

            I_TdFGripperTaskParSys.Put("FIBRE_IN_IMAGE", (INT32)lIntParam);
            // DEBUG("fibInImgThres is %d\n", fibInImgThres);
            lIntParam = 0;
        }

        tmpId.Get("SETTLE_TIME", &dParam);
        if (dParam)
        {
            I_TdFGripperTaskParSys.Put("SETTLE_TIME", (double)dParam);
            // DEBUG("settleTime is %f\n", settleTime);
            dParam = 0.0;
        }

        tmpId.Get("X_ACCURACY", &lIntParam);
        if (lIntParam)
        {
            I_TdFGripperTaskParSys.Put("X_ACCURACY", (INT32)lIntParam);
            // DEBUG("xaccuracy is %d\n", xaccuracy);
            lIntParam = 0;
        }

        tmpId.Get("Y_ACCURACY", &lIntParam);
        if (lIntParam)
        {
            I_TdFGripperTaskParSys.Put("Y_ACCURACY", (INT32)lIntParam);
            // DEBUG("yaccuracy is %d\n", yaccuracy);
            lIntParam = 0;
        }

        tmpId.Get("TIMEOUT_FAC", &lIntParam);
        if (lIntParam)
        {
            I_TdFGripperTaskParSys.Put("TIMEOUT_FAC", (INT32)lIntParam);
            // DEBUG("timeoutFac is %d\n", timeoutFac);
            lIntParam = 0;
        }

        tmpId.Get("DPR_FEEDBACK", &strParam);
        if (strParam.empty() == false)
        {
            details->dprFeedback = strcmp("NO", strParam.c_str()) ? YES : NO;
            // DEBUG("details->dprFeedback is %hd\n", details->dprFeedback);
            strParam = "";
        }

        tmpId.Get("VFG_OP_ENABLE", &lIntParam);
        if (lIntParam)
        {
            I_TdFGripperTaskParSys.Put("VFG_OP_ENABLE", (INT32)lIntParam);
            // DEBUG("overlayPlaneEnabled is %d\n", overlayPlaneEnabled);
            lIntParam = 0;
        }

        tmpId.Get("ZEROCAM_CENWAIT", &dParam);
        if (dParam)
        {
            I_TdFGripperTaskParSys.Put("ZEROCAM_CENWAIT", (double)dParam);
            // DEBUG("zerocamCenWait is %f\n", zerocamCenWait);
            dParam = 0.0;
        }

        tmpId.Get("PMAC_LIM_X_POS", &lIntParam);
        if (lIntParam)
        {
            I_TdFGripperTaskParSys.Put("PMAC_LIM_X_POS", (INT32)lIntParam);
            // DEBUG("copleyXPosLim is %d\n", copleyXPosLim);
            lIntParam = 0;
        }

        tmpId.Get("PMAC_LIM_X_NEG", &lIntParam);
        if (lIntParam)
        {
            I_TdFGripperTaskParSys.Put("PMAC_LIM_X_NEG", (INT32)lIntParam);
            // DEBUG("copleyXNegLim is %d\n", copleyXNegLim);
            lIntParam = 0;
        }

        tmpId.Get("PMAC_LIM_Y_POS", &lIntParam);
        if (lIntParam)
        {
            I_TdFGripperTaskParSys.Put("PMAC_LIM_Y_POS", (INT32)lIntParam);
            lIntParam = 0;
        }

        tmpId.Get("PMAC_LIM_Y_NEG", &lIntParam);
        if (lIntParam)
        {
            I_TdFGripperTaskParSys.Put("PMAC_LIM_Y_NEG", (INT32)lIntParam);
            lIntParam = 0;
        }

        tmpId.Get("PLT1_CFID_OFF_X", &lIntParam);
        if (lIntParam)
        {
            I_TdFGripperTaskParSys.Put("PLT1_CFID_OFF_X", (short)lIntParam);
            // DEBUG("shortZeroX is %hd\n", shortZeroX);
            lIntParam = 0;
        }

        tmpId.Get("PLT1_CFID_OFF_Y", &lIntParam);
        if (lIntParam)
        {
            I_TdFGripperTaskParSys.Put("PLT1_CFID_OFF_Y", (short)lIntParam);
            // DEBUG("shortZeroY is %hd\n", shortZeroY);
            lIntParam = 0;
        }
    }
    tdFgripperSetMainStruct(details);
    return true;
}

bool TdFGripperTask::tdFgripperReadFlexFile(drama::sds::Id &defId)
{
    if (!defId)
    {
        DEBUG("The sds::Id is invalid");
        I_ErrorString += "Failed to read the tdFgripperFlex.sds.\n";
        DramaTHROW(TDFGRIPPERTASK__READ_ERROR, "Failed to read the tdFgripperFlex.sds.");
    }

    drama::sds::Id id;
    unsigned long int actlen;
    id = defId.Find("offset");
    if (id)
    {
        id.Get(sizeof(double), &I_tdFgripperMainStruct->convert.flex.offset, &actlen, 0);
        if (actlen != sizeof(double))
        {
            I_ErrorString += "The field of offset is invalid.\n";
            DramaTHROW(TDFGRIPPERTASK__READ_ERROR, "The field of offset is invalid.");
        }
        actlen = 0;
    }
    else
    {
        I_ErrorString += "Cannot find the field of offset.\n";
        DramaTHROW(TDFGRIPPERTASK__READ_ERROR, "Cannot find the field of offset.");
    }
    id = defId.Find("length");
    if (id)
    {
        id.Get(sizeof(double), &I_tdFgripperMainStruct->convert.flex.length, &actlen, 0);
        if (actlen != sizeof(double))
        {
            I_ErrorString += "The field of length is invalid.\n";
            DramaTHROW(TDFGRIPPERTASK__READ_ERROR, "The field of length is invalid.");
        }
        actlen = 0;
    }
    else
    {
        I_ErrorString += "Cannot find the field of length.\n";
        DramaTHROW(TDFGRIPPERTASK__READ_ERROR, "Cannot find the field of length.");
    }
    id = defId.Find("k");
    if (id)
    {
        id.Get(sizeof(double), &I_tdFgripperMainStruct->convert.flex.k, &actlen, 0);
        if (actlen != sizeof(double))
        {
            I_ErrorString += "The field of length is k.\n";
            DramaTHROW(TDFGRIPPERTASK__READ_ERROR, "The field of k is invalid.");
        }
        actlen = 0;
    }
    else
    {
        I_ErrorString += "Cannot find the field of k.\n";
        DramaTHROW(TDFGRIPPERTASK__READ_ERROR, "Cannot find the field of k.");
    }

    return true;
}

bool TdFGripperTask::tdFgripperIlocks(long int ilocks)
{

    if (ilocks & ILOCK__TASK_INIT)
    {
        if (I_tdFgripperMainStruct->Initialised == NO)
        {
            I_ErrorString += "The task has not been initialised.\n";
            DEBUG("The task has not been initialised.\n");
            return false;
        }
    }

    if (ilocks & ILOCK__GAN_PARKED)
    {
        if (I_TdFGripperTaskParSys.GetString("PARKED") == "YES")
        {
            I_ErrorString += "The gantry has parked.\n";
            DEBUG("The gantry has parked.\n");
            return false;
        }
    }

    if (ilocks & ILOCK__GAN_NOT_PARKED)
    {
        if (I_TdFGripperTaskParSys.GetString("PARKED") == "NO")
        {
            I_ErrorString += "The gantry has not parked.\n";
            DEBUG("The gantry has not parked.\n");
            return false;
        }
    }
    DEBUG("Lock check has completed.\n");
    return true;
}

void TdFGripperTask::tdFgripperFlexure(long int x, long int y,
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

void TdFGripperTask::tdFstateBitSet(unsigned char bit)
{
    unsigned short oldval, newval = 0;
    oldval = ushortZero;
    newval = oldval | bit;

    /* Only update the parameter (triggering monitors) if value has changed*/
    if (newval != oldval)
        ushortZero = newval;
}

void TdFGripperTask::tdFstateBitClear(unsigned char bit)
{
    unsigned short oldval, newval = 0;
    oldval = ushortZero;

    newval = oldval & (~bit);

    /* Only update the parameter (triggering monitors) if value has changed*/
    if (newval != oldval)
        ushortZero = newval;
}

void TdFGripperTask::tdFgripperConvertFromEnc(int displayText, int xEnc, int yEnc, int zEnc, int tEnc,
                                              int jEnc, double plateTheta, short level, double *xCon,
                                              double *yCon, double *zCon, double *tCon, double *jCon)
{

    double x_enc, y_enc, z_enc, t_enc, j_enc,
        x_fp, y_fp, z_fp, t_fp, j_fp,
        ha, dec;
    long int dx = 0,
             dy = 0,
             dz = 0;
    double ddx = 0;
    double ddy = 0;

    int n;

    short debugging = 0;

    /*
     *  If the display flag is non-zero, then check if we are to add
     * debugging details for the conversion.
     */
    if (displayText)
    {
        I_TdFGripperTaskParSys.Get("DEBUG_CVT", &debugging);
        if (debugging)
            DEBUG("FromENC:Converting %d, %d, level %d",
                  xEnc, yEnc, level);
    }

    I_TdFGripperTaskParSys.Get("HA", &ha);
    I_TdFGripperTaskParSys.Get("DEC", &dec);

    if ((level == _ALL) || (level == _FLEX))
    {
        *xCon = (double)xEnc;
        *yCon = (double)yEnc;
        *zCon = (double)zEnc;
        *tCon = (double)tEnc;
        *jCon = (double)jEnc;
        if (debugging)
            DEBUG("  FromENC:Conversion complete (nothing done) - level _ALL/_FLEX");
        return;
    }

    tdFgripperFlexure((long int)xEnc, (long int)yEnc,
                      ha, dec, &I_tdFgripperMainStruct->convert.flex,
                      &dx, &dy);

    if (debugging)
        DEBUG("  FromENC:Adding flexure of %ld, %ld microns", dx, dy);
    x_enc = (double)xEnc + (double)dx;
    y_enc = (double)yEnc + (double)dy;
    z_enc = (double)zEnc + (double)dz;
    t_enc = (double)tEnc;
    j_enc = (double)jEnc;
    /*
     *  This is all we want if the level is _TEMP.
     */
    if (level == _TEMP)
    {
        *xCon = x_enc;
        *yCon = y_enc;
        *zCon = z_enc;
        *tCon = t_enc;
        *jCon = j_enc;
        if (debugging)
            DEBUG("  FromENC:Conversion complete %.1f, %.1f - level _TEMP",
                  *xCon, *yCon);
        return;
    }

    if (level == _COEFFS)
    {
        *xCon = x_enc;
        *yCon = y_enc;
        *zCon = z_enc;
        *tCon = t_enc;
        *jCon = j_enc;
        if (debugging)
            DEBUG("  FromENC:Conversion complete %.1f, %.1f - level _COEFFS",
                  *xCon, *yCon);
        return;
    }

    /*
     *  Translate/rotate/scale to non-deflected coordinates for set temperature.
     */
    slaXy2xy(x_enc, y_enc,                              /* Encoder coordinates     */
             I_tdFgripperMainStruct->convert.invCoeffs, /* Transformation matrix   */
             &x_fp, &y_fp);                             /* Field-plate coordinates */
    z_fp = z_enc + (double)I_tdFgripperMainStruct->convert.zShift;
    j_fp = j_enc + (double)I_tdFgripperMainStruct->convert.jawShift;
    t_fp = (t_enc + (double)I_tdFgripperMainStruct->convert.thetaShift) / 1000000.0;

    if (debugging)
        DEBUG("  FromENC:Lin model, converted %.1f, %.1f to %.1f, %.1f",
              x_enc, y_enc, x_fp, y_fp);

    if (I_tdFgripperMainStruct->configPlate == 1)
    {
        if (I_tdFgripperMainStruct->plateOneDontRemove)
        {
            if (debugging)
                DEBUG("  FromEnc:Converting for plate 1, but offset removal disabled");
        }
        else
        {
            double t_x = x_fp;
            double t_y = y_fp;
            x_fp = t_x + (double)(*(I_tdFgripperMainStruct->pars.plt1CenterFidOffsetX));
            y_fp = t_y + (double)(*(I_tdFgripperMainStruct->pars.plt1CenterFidOffsetY));
            if (debugging)
                DEBUG("  FromEnc:Plate 1 position changed by %d, %d from %.1f, %.1f to %.1f, %.1f",
                      *(I_tdFgripperMainStruct->pars.plt1CenterFidOffsetX),
                      *(I_tdFgripperMainStruct->pars.plt1CenterFidOffsetY),
                      t_x, t_y, x_fp, y_fp);
        }
    }
    else if (debugging)
        DEBUG("  FromENC:Conversion is for plate 0 - no offset removal");
    /*
     *  Remove rotation due to field plate rotation.
     */
    *xCon = x_fp * cos(-plateTheta) - y_fp * sin(-plateTheta);
    *yCon = y_fp * cos(-plateTheta) + x_fp * sin(-plateTheta);
    *tCon = t_fp + plateTheta;
    *jCon = j_fp;

    if (debugging)
    {
        DEBUG("  FromENC:Due to field plate rotation of %.2f degrees - %.1f,%.1f to %.1f, %.1f",
              plateTheta * 180.0 / PI, x_fp, y_fp, *xCon, *yCon);
    }

    /*
     *  Remove the zmap correction.
     */
    *zCon = z_fp - GetZAdjustment(debugging, *xCon, *yCon, &I_tdFgripperMainStruct->convert.zmap);

    /*
     *  Normalize theta reading to range -2*PI to 2*PI.
     */
    n = *tCon / (2 * PI);
    *tCon -= n * 2 * PI;

    /*
     * Add the Gripper to FPI distortion.
     */
    CalcGripperToFPIdistortion(debugging,
                               1, details, *xCon, *yCon, &ddx, &ddy, status);
    *xCon -= ddx;
    *yCon -= ddy;
    if (debugging)
    {
        DEBUG("  FromENC:Removing distortion correction of %.1f, %.1f -> %.1f, %.1f",
              ddx, ddy, *xCon, *yCon);
        DEBUG("  FromENC:Conversion complete - full model %.1f, %.1f %.1f",
              *xCon, *yCon, *zCon);
    }
}

bool TdFGripperTask::tdFgripperUpdatePos(short updateIdeal, short useDpr, short displayText)
{
    double atFpX, atFpY, atFpZ, atFpJ,
        oldFpT, newFpT, plateTheta;
    long int oldFpX, oldFpY, oldFpZ, oldFpJ,
        newFpX, newFpY, newFpZ, newFpJ;

    if (I_tdFgripperMainStruct == nullptr)
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
                    I_tdFgripperMainStruct->atEnc.x = (int)CurrentPosition;
                    DEBUG("\nNow the encoder position of X axis is %d", I_tdFgripperMainStruct->atEnc.x);
                }
                else if (Index == 2)
                {
                    I_tdFgripperMainStruct->atEnc.y = (int)CurrentPosition;
                    DEBUG("\nNow the encoder position of Y axis is %d", I_tdFgripperMainStruct->atEnc.y);
                }
                else if (Index == 3)
                {
                    I_tdFgripperMainStruct->atEnc.z = (int)CurrentPosition;
                    DEBUG("\nNow the encoder position of Y axis is %d", I_tdFgripperMainStruct->atEnc.z);
                }
                else if (Index == 4)
                {
                    I_tdFgripperMainStruct->atEnc.jaw = (int)CurrentPosition;
                    DEBUG("\nNow the encoder position of Y axis is %d", I_tdFgripperMainStruct->atEnc.jaw);
                }
                else
                {
                    I_tdFgripperMainStruct->atEnc.theta = (int)CurrentPosition;
                    DEBUG("\nNow the encoder position of Y axis is %d", I_tdFgripperMainStruct->atEnc.theta);
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
        I_tdFgripperMainStruct->atEnc.x = I_tdFgripperMainStruct->toEnc.x;
        I_tdFgripperMainStruct->atEnc.y = I_tdFgripperMainStruct->toEnc.y;
    }

    {
        /*
         * Update the parameters which contain the encoder values - but only if they
         * have actually changed (to avoid flooding parameter monitors)
         */
        ixPark = I_TdFGripperTaskParSys.GetInt("X");
        if (ixPark != I_tdFgripperMainStruct->atEnc.x)
        {
            I_TdFGripperTaskParSys.Put("X", (long int)I_tdFgripperMainStruct->atEnc.x);
            I_TdFGripperTaskParSys.Put("XCopley", (long int)I_tdFgripperMainStruct->atEnc.x);
        }
        iyPark = I_TdFGripperTaskParSys.GetInt("Y");
        if (iyPark != I_tdFgripperMainStruct->atEnc.y)
        {
            I_TdFGripperTaskParSys.Put("Y", (long int)I_tdFgripperMainStruct->atEnc.y);
            I_TdFGripperTaskParSys.Put("YCopley", (long int)I_tdFgripperMainStruct->atEnc.y);
        }
    }
    double dPlateTheta = I_TdFGripperTaskParSys.GetDouble("PLATE_THETA");
    tdFgripperConvertFromEnc(I_tdFgripperMainStruct->atEnc.x,
                             I_tdFgripperMainStruct->atEnc.y,
                             (double)dPlateTheta, _FULL,
                             &atFpX, &atFpY);

    if (displayText)
        DEBUG("\ntdFgripper:Converted encoder values %d, %d to plate units %f, %f.\n", I_tdFgripperMainStruct->atEnc.x, I_tdFgripperMainStruct->atEnc.y, atFpX, atFpY);

    oldFpX = (long int)(ixPark);
    oldFpX = I_TdFGripperTaskParSys.GetLong("X");
    oldFpY = (long int)(iyPark);
    oldFpY = I_TdFGripperTaskParSys.GetLong("Y");
    newFpX = doubleToLong(atFpX);
    newFpY = doubleToLong(atFpY);

    if (oldFpX != newFpX)
    {
        xPark = newFpX;
        xCopleyPark = newFpX;
        I_TdFGripperTaskParSys.Put("X", (long int)newFpX);
        I_TdFGripperTaskParSys.Put("XCopley", (long int)newFpX);
    }
    if (oldFpY != newFpY)
    {
        yPark = newFpY;
        yCopleyPark = newFpY;
        I_TdFGripperTaskParSys.Put("Y", (long int)newFpY);
        I_TdFGripperTaskParSys.Put("YCopley", (long int)newFpY);
    }
    if ((newFpX >= (GRIPPER_CLEAR_X - 1000)) && (newFpY <= (GRIPPER_CLEAR_Y + 1000)))
        tdFstateBitSet(SAFE);
    else
        tdFstateBitClear(SAFE);

    if (updateIdeal)
    {
        I_tdFgripperMainStruct->ideal.x = newFpX;
        I_tdFgripperMainStruct->ideal.y = newFpY;
    }
    DEBUG("tdFgripper:I_tdFgripperMainStruct->ideal values: %ld, %ld.\n", I_tdFgripperMainStruct->ideal.x, I_tdFgripperMainStruct->ideal.y);
    return true;
}

bool CSearchAction::MoveToSearchPosition(const long searchX, const long searchY, short *const atSearchXY,
                                         short *const searchStarted)
{
    *searchStarted = YES;
    *atSearchXY = YES;
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
    drama::ParSys parSysId(ThisTask->TaskPtr());
    double plateTheta, toXenc, toYenc;

    if (0) // if (!(ThisTask->SetupAmps()))
    {
        DEBUG("MoveToSearchPosition: Failed to set up the amplifiers %s.\n", ThisTask->GetError().c_str());
        return false;
    }
    else
    {
        std::string Error;
        std::string Axes = "X,Y";
        std::string Velocities = "1,1";

        parSysId.Get("PLATE_THETA", &plateTheta);
        ThisTask->tdFgripperConvertFromFP(searchX, searchY, plateTheta, _FULL,
                                          &toXenc, &toYenc);

        details->ideal.x = searchX;
        details->ideal.y = searchY;
        details->toEnc.x = (int)doubleToLong(toXenc);
        details->toEnc.y = (int)doubleToLong(toYenc);

        std::string Positions = to_string(doubleToLong(toXenc)) + "," + to_string(doubleToLong(toYenc));
        printf("MoveToSearchPosition: the position is %s.\n", Positions.c_str());
        std::vector<AxisDemand> AxisDemands = GetDemands(Axes, Positions, Velocities, Error);
        if (!(ThisTask->MoveAxes(AxisDemands, false)))
        {
            DEBUG("MoveToSearchPosition: failed to move to the position %s.\n", ThisTask->GetError().c_str());
            return false;
        }
        std::this_thread::sleep_for(2000ms);
        if (!ThisTask->tdFgripperUpdatePos(YES, YES, YES))
        {
            MessageUser("MoveToSearchPosition: failed to Update the current position of gantry.");
        }
    }
    return true;
}

bool CSearchAction::PerformCentroid(const shared_ptr<tdFgrip_CENtype> &cenWin, const double settletime, short *const centroided)
{
    *centroided = YES;
    if (settletime > 0.0)
    {
        std::this_thread::sleep_for(std::chrono::seconds(int(settletime)));
    }
    drama::Path thisTaskPath(_theTask);

    MessageUser("C_SEARCH PerformCentroid: - grabbing image");
    MessageUser("C_SEARCH PerformCentroid:  Window size -> Max %ld %ld off %ld %ld, dim %ld %ld",
                cenWin->window.MaxX, cenWin->window.MaxY,
                cenWin->window.Xoffset, cenWin->window.Yoffset,
                cenWin->window.Xdim, cenWin->window.Ydim);

    drama::sds::Id messageArg(drama::sds::Id::CreateArgStruct());
    drama::sds::IdPtr returnedArg;

    messageArg.Put("Image", "FREE");
    messageArg.Put("Window", "FULL");

    // reorganise a class or a function to implement centroid;
    try
    {
        thisTaskPath.Obey(this, "C_CENTROID", messageArg, &returnedArg);
    }
    catch (std::exception &what)
    {
        DEBUG("C_SEARCH PerformCentroid: Vimba raises an error.\n");
        return false;
    }

    if (!(*returnedArg))
    {
        DEBUG("C_SEARCH PerformCentroid: Failed to perform centroid and return an image.\n");
        return false;
    }

    returnedArg->Get("XERR", &m_tdFgripperSFStruct->xErr);
    returnedArg->Get("YERR", &m_tdFgripperSFStruct->yErr);
    returnedArg->Get("FIBREINIMAGE", &m_tdFgripperSFStruct->centroidOK);
    return true;
}

bool TdFGripperTask::SetupAmps(void)
{

    //  See if all the amps have already been setup successfully.

    // bool AllFound = I_X1Amp && I_X2Amp && I_YAmp && I_ZAmp && I_ThetaAmp && I_JawAmp;
    bool AllFound = AllFound = I_X1Amp && I_X2Amp && I_YAmp && I_ZAmp && I_ThetaAmp && I_JawAmp;

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
            {&I_X1Amp, &I_X2Amp, &I_YAmp, &I_ZAmp, &I_ThetaAmp, &I_JawAmp};

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

double TdFGripperTask::tdFautoThetaPos(long int x, long int y)
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

bool TdFGripperTask::DisableAmps()
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
    if (I_ZAmp)
    {
        Err = I_CanAccess.GetAmp(G_AmpNames[3])->Disable();
        if (Err)
        {
            I_ErrorString = "Error disabling Z amplifier. ";
            I_ErrorString += Err->toString();
            return DisableAllAmps;
        }
        I_ZAmp = nullptr;
        DEBUG("Z amplifier disabled!\n");
    }
    if (I_ThetaAmp)
    {
        Err = I_CanAccess.GetAmp(G_AmpNames[5])->Disable();
        if (Err)
        {
            I_ErrorString = "Error disabling Theta amplifier. ";
            I_ErrorString += Err->toString();
            return DisableAllAmps;
        }
        I_ThetaAmp = nullptr;
        DEBUG("Theta amplifier disabled!\n");
    }
    if (I_JawAmp)
    {
        Err = I_CanAccess.GetAmp(G_AmpNames[4])->Disable();
        if (Err)
        {
            I_ErrorString = "Error disabling Jaw amplifier. ";
            I_ErrorString += Err->toString();
            return DisableAllAmps;
        }
        I_JawAmp = nullptr;
        DEBUG("Jaw amplifier disabled!\n");
    }
    DEBUG("All amplifiers disabled!\n");
    return true;
}

bool TdFGripperTask::HomeAxes(bool HomeX, bool HomeY, bool HomeZ, bool HomeTheta, bool HomeJaw)
{

    bool ReturnOK = false;

    CML::Linkage *HomeLinkage = I_CanAccess.GetLinkage(MAX_TDF_AMPS, G_AmpNames);
    if (HomeLinkage)
    {

        const CML::Error *Err = NULL;
        I_tdFgripperMainStruct->inUse = YES;

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
            I_tdFgripperMainStruct->toEnc.x = 0;
        }
        if (HomeY)
        {
            HomeFlags[Y_AMP] = true;
            I_tdFgripperMainStruct->toEnc.x = 0;
        }

        //  Set all the amps in the linkage to a pre-programmed halt mode, specifying that a
        //  halted axis will be 'floppy'.

        for (int Index = 0; Index < MAX_TDF_AMPS; Index++)
        {
            Err = (*HomeLinkage)[Index].SetHaltMode(CML::HALT_DISABLE);
            if (Err)
                DEBUG("Setting halt mode %s\n", Err->toString());
        }

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
        }
        if (Err == NULL)
        {
            ReturnOK = true;
            I_TdFGripperTaskParSys.Put("PARKED", "YES");
        }

        I_tdFgripperMainStruct->inUse = NO;
    }
    DEBUG("HomeAxes returns\n");
    return ReturnOK;
}

bool TdFGripperTask::SetParameter(string &ParameterName, string &ParameterValue)
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
            if (strcmp(ParameterName.c_str(), "X") == 0)
            {
                I_TdFGripperTaskParSys.Put("X", (INT32)PosVal);
            }
            else
            {
                I_TdFGripperTaskParSys.Put("Y", (INT32)PosVal);
            }
        }
    }
    else if (strcmp(ParameterName.c_str(), "XY_VEL") == 0)
    {
        int VelVal = stoi(ParameterValue);
        I_TdFGripperTaskParSys.Put("XY_VEL", (INT32)VelVal);
        DEBUG("Setting up the velocity %d\n", (int)xyVel);
    }
    else if ((strcmp(ParameterName.c_str(), "X_ACCURACY") == 0) ||
             (strcmp(ParameterName.c_str(), "Y_ACCURACY") == 0))
    {
        int accuracy = stoi(ParameterValue);
        if (strcmp(ParameterName.c_str(), "X_ACCURACY") == 0)
        {
            I_TdFGripperTaskParSys.Put("X_ACCURACY", (INT32)accuracy);
            DEBUG("Setting up the X_ACCURACY %d\n", (int)xaccuracy);
        }
        else
        {
            I_TdFGripperTaskParSys.Put("Y_ACCURACY", (INT32)accuracy);
            DEBUG("Setting up the Y_ACCURACY %d\n", (int)yaccuracy);
        }
    }
    else if (strcmp(ParameterName.c_str(), "PLATE_THETA") == 0)
    {
        double ThetaVal = stod(ParameterValue);
        I_TdFGripperTaskParSys.Put("PLATE_THETA", (double)ThetaVal);
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
            I_TdFGripperTaskParSys.Put("PARKED", std::string(ParameterValue));
        }

        DEBUG("Setting up the PARKED Mode %s\n", ParameterValue.c_str());
    }
    else if ((strcmp(ParameterName.c_str(), "HA") == 0) ||
             (strcmp(ParameterName.c_str(), "DEC") == 0))
    {
        double dVal = stod(ParameterValue);
        if (strcmp(ParameterName.c_str(), "HA") == 0)
        {
            I_TdFGripperTaskParSys.Put("HA", (double)dVal);
        }
        else
        {
            I_TdFGripperTaskParSys.Put("DEC", (double)dVal);
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
            I_TdFGripperTaskParSys.Put("STEP_SIZE", (INT32)(iVal));
        }
        else if (strcmp(ParameterName.c_str(), "MAX_ERROR") == 0)
        {
            I_TdFGripperTaskParSys.Put("MAX_ERROR", (INT32)(iVal));
        }
        else if (strcmp(ParameterName.c_str(), "FIBRE_IN_IMAGE") == 0)
        {
            I_TdFGripperTaskParSys.Put("FIBRE_IN_IMAGE", (INT32)(iVal));
        }
        else if (strcmp(ParameterName.c_str(), "POS_TOL") == 0)
        {
            I_TdFGripperTaskParSys.Put("POS_TOL", (INT32)(iVal));
        }
        else if (strcmp(ParameterName.c_str(), "TIMEOUT_FAC") == 0)
        {
            I_TdFGripperTaskParSys.Put("TIMEOUT_FAC", (INT32)(iVal));
        }
        else if (strcmp(ParameterName.c_str(), "VFG_OP_ENABLE") == 0)
        {
            I_TdFGripperTaskParSys.Put("VFG_OP_ENABLE", (INT32)(iVal));
        }
        DEBUG("Setting up the %s to be %d\n", ParameterName.c_str(), iVal);
    }
    else if ((strcmp(ParameterName.c_str(), "PLT1_CFID_OFF_X") == 0) ||
             (strcmp(ParameterName.c_str(), "PLT1_CFID_OFF_Y") == 0))
    {
        int iVal = stoi(ParameterValue);
        if (strcmp(ParameterName.c_str(), "PLT1_CFID_OFF_X") == 0)
        {
            I_TdFGripperTaskParSys.Put("PLT1_CFID_OFF_X", (short)(iVal));
        }
        else
        {
            I_TdFGripperTaskParSys.Put("PLT1_CFID_OFF_Y", (short)(iVal));
        }

        DEBUG("Setting up the %s to be %d\n", ParameterName.c_str(), iVal);
    }
    else if (strcmp(ParameterName.c_str(), "POS_ATTEMPTS") == 0)
    {
        int iVal = stoi(ParameterValue);
        I_TdFGripperTaskParSys.Put("POS_ATTEMPTS", (short)(iVal));
        DEBUG("Setting up the %s to be %d\n", ParameterName.c_str(), iVal);
    }
    else if (strcmp(ParameterName.c_str(), "SETTLE_TIME") == 0)
    {
        double dVal = stod(ParameterValue);
        I_TdFGripperTaskParSys.Put("SETTLE_TIME", (double)(dVal));
        DEBUG("Setting up the %s to be %f\n", ParameterName.c_str(), dVal);
    }
    else if (strcmp(ParameterName.c_str(), "ZEROCAM_CENWAIT") == 0)
    {
        double dVal = stod(ParameterValue);
        I_TdFGripperTaskParSys.Put("ZEROCAM_CENWAIT", (double)(dVal));
        DEBUG("Setting up the %s to be %f\n", ParameterName.c_str(), dVal);
    }
    else if (strcmp(ParameterName.c_str(), "GANTRY_LAMPS") == 0)
    {
        I_TdFGripperTaskParSys.Put("GANTRY_LAMPS", std::string(ParameterValue));
        DEBUG("Setting up the %s to be %s\n", ParameterName.c_str(), ParameterValue.c_str());
    }
    else if ((strcmp(ParameterName.c_str(), "PMAC_LIM_X_POS") == 0) ||
             (strcmp(ParameterName.c_str(), "PMAC_LIM_X_NEG") == 0) ||
             (strcmp(ParameterName.c_str(), "PMAC_LIM_Y_POS") == 0) ||
             (strcmp(ParameterName.c_str(), "PMAC_LIM_Y_NEG") == 0))
    {
        int iVal = stoi(ParameterValue);
        if (strcmp(ParameterName.c_str(), "PMAC_LIM_X_POS") == 0)
        {
            I_TdFGripperTaskParSys.Put("PMAC_LIM_X_POS", (INT32)(iVal));
        }
        else if (strcmp(ParameterName.c_str(), "PMAC_LIM_X_NEG") == 0)
        {

            I_TdFGripperTaskParSys.Put("PMAC_LIM_X_NEG", (INT32)(iVal));
        }
        else if (strcmp(ParameterName.c_str(), "PMAC_LIM_Y_POS") == 0)
        {

            I_TdFGripperTaskParSys.Put("PMAC_LIM_Y_POS", (INT32)(iVal));
        }
        else if (strcmp(ParameterName.c_str(), "PMAC_LIM_Y_NEG") == 0)
        {

            I_TdFGripperTaskParSys.Put("PMAC_LIM_Y_NEG", (INT32)(iVal));
        }
        DEBUG("Setting up the %s to be %d\n", ParameterName.c_str(), iVal);
    }
    else if (strcmp(ParameterName.c_str(), "DEBUG_CENTROID") == 0)
    {
        int iVal = stoi(ParameterValue);
        I_TdFGripperTaskParSys.Put("DEBUG_CENTROID", (unsigned short)(iVal));
        DEBUG("Setting up the %s to be %d\n", ParameterName.c_str(), iVal);
    }
    return true;
}

bool TdFGripperTask::SetupHomeConfig(CML::HomeConfig HomeConfigs[], int Index, AmpId Amp)
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

CML::Amp *TdFGripperTask::GetAmp(AmpId AxisId)
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

bool TdFGripperTask::SetupLinkage(CML::Linkage &TheLinkage, unsigned int NumberAmps, CML::Amp *LinkedAmps[], double Limits[])
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

void TdFGripperTask::tdFgripperPositionCheck(int Index, CML::uunit &Position)
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

bool TdFGripperTask::MoveAxes(const std::vector<AxisDemand> &AxisDemands, bool MoveOffset /*=false*/,
                              drama::thread::TAction * /*ThisAction*/)
{

    bool ReturnOK = false;

    CML::Linkage *MoveLinkage = I_CanAccess.GetLinkage(MAX_TDF_AMPS, G_AmpNames);
    if (MoveLinkage)
    {

        const CML::Error *Err = NULL;

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

            tdFgripperPositionCheck(Index, TargetPosition);
            Targets[Index] = TargetPosition;
            TargetPoint[Index] = TargetPosition;
            DEBUG("Target for axis %d is %f\n", Index, TargetPosition);
            if (Index == 0 || Index == 1)
            {
                I_tdFgripperMainStruct->toEnc.x = doubleToLong(TargetPosition);
            }
            else if (Index == 2)
            {
                I_tdFgripperMainStruct->toEnc.y = doubleToLong(TargetPosition);
            }
            MoveFlags[Index] = Found;
        }
        TargetPoint.setDim(MAX_TDF_AMPS);

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
                DEBUG("Enabling axis %d\n", Index);
                Err = (*MoveLinkage)[Index].Enable();
                if (Err)
                    DEBUG("Enabling %s\n", Err->toString());
            }
        }

        if (Err == NULL)
        {
            Err = MoveLinkage->MoveTo(TargetPoint);
            if (Err)
                DEBUG("Moving to target %s\n", Err->toString());
        }

        if (Err == NULL)
        {
            DEBUG("Waiting\n");

            Err = MoveLinkage->WaitMoveDone(20000);
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

void InitialiseAction::ActionThread(const drama::sds::Id &)
{

    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
    drama::sds::Id parSysId(drama::sds::Id::CreateFromSdsIdType((long)(DitsGetParId())));

    ThisTask->ClearError();

    std::shared_ptr<tdFgripperTaskType> details = (ThisTask->tdFgripperGetMainStruct());
    if (details != nullptr && details->Initialised == YES)
    {
        MessageUser("INITIALISE: TdFGripperTask is already initialised.");
        return;
    }
    details = std::make_shared<tdFgripperTaskType>();
    if (!details)
    {
        DEBUG("Fail to allocate memory to tdFgripperTaskType\n");
        return;
    }
    else
    {
        DEBUG("Succeed to allocate memory to tdFgripperTaskType\n");

        parSysId.Put("ENQ_VER_NUM", TdFGripperTaskVersion);
        parSysId.Put("ENQ_VER_DATE", TdFGripperTaskDate);

        drama::sds::Id id;        /* Parameter id                    */
        unsigned long int length; /* Length of parameter item        */
        short check = SHOW;
        /*
         *  For parameters we want quick access to, get pointers to them
         *  (we  can't do data conversion when getting such parameters)
         */

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

        bool defRead = ThisTask->tdFgripperDefRead(DEFS_FILE | FLEX_FILE,
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

        ThisTask->tdFGetCameraPath().GetPath(this);
        try
        {
            ThisTask->tdFGetCameraPath().Obey(this, "INITIALISE");
        }
        catch (...)
        {
            MessageUser("INITIALISE: failed to initialise the camera.");
            DramaTHROW(TDFGRIPPERTASK__CAMERA, "INITIALISE: failed to initialise the camera.");
        }

        MessageUser("INITIALISE: camera initialisation completes.\n");

        details->cameraInit = YES;

        if (!(ThisTask->SetupAmps()))
        {
            DEBUG("SetupAmps fails\n");
            MessageUser("INITIALISE: " + ThisTask->GetError());
            return;
        }
        else
        {
            DEBUG("Setup amps OK\n");
            MessageUser("INITIALISE: Task initialised from configuration file " + CONFIGURATION_FILE);
        }
        ThisTask->tdFgripperSetMainStruct(details);
    }
}

void GMoveAxisActionNT::ActionThread(const drama::sds::Id &Arg)
{

    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
    ThisTask->ClearError();

    std::shared_ptr<tdFgripperTaskType> details = (ThisTask->tdFgripperGetMainStruct());
    if (details == nullptr || details->Initialised == NO)
    {
        MessageUser("G_MOVE_AXIS_NT: TdFGripperTask is not initialised. Please use \"ditscmd TdFGripperTask INITIALISE\" to initialise the task.");
        return;
    }
    if (!(ThisTask->tdFgripperIlocks(ILOCK__GAN_PARKED)))
    {
        MessageUser("G_MOVE_AXIS_NT: the gantry is parked, please unpark the gantry first.");
        return;
    }
    if (details->inUse)
    {
        MessageUser("G_MOVE_AXIS_NT: TdFGripperTask is running other actions.\n");
        return;
    }

    std::string Axes;
    std::string Positions;
    std::string Velocities;
    std::string OffsetFlag = "";
    if (Arg)
    {
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
        double plateTheta;
        drama::ParSys parSysId(ThisTask->TaskPtr());
        parSysId.Get("PLATE_THETA", &plateTheta);
        long int toX, toY;
        double toXenc = 0.0, toYenc = 0.0;
        unsigned int NumberAxes = AxisDemands.size();
        for (unsigned int Index = 0; Index < NumberAxes; Index++)
        {
            if (MoveBackward == true)
            {

                AxisDemands[Index].Position *= (-1.0);
            }
            if (MoveOffset == true)
            {
                AxisDemands[Index].Position += (Index < 2 ? details->ideal.x : details->ideal.y);
            }
            if (Index < 2)
            {
                toX = doubleToLong(AxisDemands[Index].Position);
                details->ideal.x = toX;
            }
            if (Index == 2)
            {
                toY = doubleToLong(AxisDemands[Index].Position);
                details->ideal.y = toY;
            }
            DEBUG("AxisId: %d, plate position %f, velocity %f\n", AxisDemands[Index].AxisId,
                  AxisDemands[Index].Position, AxisDemands[Index].Velocity);
        }
        ThisTask->tdFgripperSetMainStruct(details);
        ThisTask->tdFgripperConvertFromFP(toX, toY, plateTheta, _FULL,
                                          &toXenc, &toYenc);
        for (unsigned int Index = 0; Index < NumberAxes; Index++)
        {
            if (Index < 2)
            {
                AxisDemands[Index].Position = toXenc;
            }
            else if (Index == 2)
            {
                AxisDemands[Index].Position = toYenc;
            }
            DEBUG("AxisId: %d, encoder position %f, velocity %f\n", AxisDemands[Index].AxisId,
                  AxisDemands[Index].Position, AxisDemands[Index].Velocity);
        }

        if (!(ThisTask->SetupAmps()))
        {
            MessageUser("G_MOVE_AXIS_NT: " + ThisTask->GetError());
        }
        else
        {
            if (!(ThisTask->MoveAxes(AxisDemands, false)))
            {
                MessageUser("G_MOVE_AXIS_NT: " + ThisTask->GetError());
            }
            else
            {
                MessageUser("G_MOVE_AXIS_NT: Move complete");
                MessageUser("G_MOVE_AXIS_NT: Waiting to receive signals about the current position of the gantry.");
                std::this_thread::sleep_for(2000ms);
                if (!(ThisTask->tdFgripperUpdatePos(YES, details->dprFeedback, YES)))
                {
                    MessageUser("G_MOVE_AXIS_NT: Failed to update the position of the gantry");
                }
            }
        }

        details->inUse = NO;
        ThisTask->tdFgripperSetMainStruct(details);
    }
}

void GParkGantryActionNT::ActionThread(const drama::sds::Id &Arg)
{

    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
    ThisTask->ClearError();

    std::shared_ptr<tdFgripperTaskType> details = (ThisTask->tdFgripperGetMainStruct());
    drama::ParSys parSysId(ThisTask->TaskPtr());

    if (details == nullptr || details->Initialised == NO)
    {
        MessageUser("G_PARK_NT: TdFGripperTask is not initialised. Please use \"ditscmd TdFGripperTask INITIALISE\" to initialise the task.");
        return;
    }
    if (!(ThisTask->tdFgripperIlocks(ILOCK__GAN_PARKED)))
    {
        MessageUser("G_PARK_NT: the gantry is already parked.");
        return;
    }
    if (details->inUse)
    {
        MessageUser("G_PARK_NT: TdFGripperTask is running other actions.\n");
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
            ThisTask->tdFgripperSetMainStruct(details);
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
                if (!(ThisTask->tdFgripperUpdatePos(YES, details->dprFeedback, YES)))
                {
                    MessageUser("G_PARK_NT: Failed to update the position of gantry.");
                }
            }
            details->inUse = NO;
            ThisTask->tdFgripperSetMainStruct(details);
        }
    }
    else
    {
        MessageUser("G_PARK_NT: Invalid axis specification");
    }
}

void GHomeActionNT::ActionThread(const drama::sds::Id &Arg)
{

    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
    ThisTask->ClearError();
    // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());
    std::shared_ptr<tdFgripperTaskType> details = ThisTask->tdFgripperGetMainStruct();
    drama::ParSys parSysId(ThisTask->TaskPtr());

    if (details == nullptr || details->Initialised == NO)
    {
        MessageUser("G_HOME_NT: TdFGripperTask is not initialised. Please use \"ditscmd TdFGripperTask INITIALISE\" to initialise the task.");
        return;
    }
    if (!(ThisTask->tdFgripperIlocks(ILOCK__GAN_PARKED)))
    {
        MessageUser("G_HOME_NT: the gantry is already homed.");
        return;
    }
    if (details->inUse)
    {
        MessageUser("G_HOME_NT: TdFGripperTask is running other actions.\n");
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
            ThisTask->tdFgripperSetMainStruct(details);
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
                if (!(ThisTask->tdFgripperUpdatePos(YES, details->dprFeedback, YES)))
                {
                    MessageUser("G_HOME_NT: Failed to update the position of gantry.");
                }
            }
            details->inUse = NO;
            ThisTask->tdFgripperSetMainStruct(details);
        }
    }
    else
    {
        MessageUser("G_HOME_NT: Invalid axis specification");
    }
}

void GUnParkActionNT::ActionThread(const drama::sds::Id &Arg)
{
    std::string Axes("");
    std::string Positions;
    std::string Velocities;

    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
    ThisTask->ClearError();
    // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());

    std::shared_ptr<tdFgripperTaskType> details = ThisTask->tdFgripperGetMainStruct();
    drama::ParSys parSysId(ThisTask->TaskPtr());

    if (details == nullptr || details->Initialised == NO)
    {
        MessageUser("G_UNPARK_NT: TdFGripperTask is not initialised. Please use \"ditscmd TdFGripperTask INITIALISE\" to initialise the task.");
        return;
    }
    if (!(ThisTask->tdFgripperIlocks(ILOCK__GAN_NOT_PARKED)))
    {
        MessageUser("G_UNPARK_NT: the gantry is already not-parked.");
        return;
    }
    if (details->inUse)
    {
        MessageUser("G_UNPARK_NT: TdFGripperTask is running other actions.\n");
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
            ThisTask->tdFgripperSetMainStruct(details);
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
                if (!(ThisTask->tdFgripperUpdatePos(YES, details->dprFeedback, YES)))
                {
                    MessageUser("G_UNPARK_NT: Failed to update the position of gantry.");
                }
            }

            details->inUse = NO;
            ThisTask->tdFgripperSetMainStruct(details);
        }
    }
    else
    {
        MessageUser("G_UNPARK_NT: Invalid axis specification");
    }
}

void GMoveOffsetActionNT::ActionThread(const drama::sds::Id &Arg)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
    ThisTask->ClearError();

    std::shared_ptr<tdFgripperTaskType> details = ThisTask->tdFgripperGetMainStruct();
    if (details == nullptr || details->Initialised == NO)
    {
        MessageUser("G_MOVEOFFSET_NT: TdFGripperTask is not initialised. Please use \"ditscmd TdFGripperTask INITIALISE\" to initialise the task.");
        return;
    }
    if (!(ThisTask->tdFgripperIlocks(ILOCK__GAN_PARKED)))
    {
        MessageUser("G_MOVEOFFSET_NT: the gantry is parked.");
        return;
    }
    if (details->inUse)
    {
        MessageUser("G_MOVEOFFSET_NT: TdFGripperTask is running other actions.\n");
        return;
    }

    std::string Axes;
    std::string Positions;
    std::string Velocities;
    std::string OffsetFlag = "";
    if (Arg)
    {
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

        double plateTheta;
        drama::ParSys parSysId(ThisTask->TaskPtr());
        parSysId.Get("PLATE_THETA", &plateTheta);
        long int toX, toY;
        double toXenc = 0.0, toYenc = 0.0;

        for (unsigned int Index = 0; Index < NumberAxes; Index++)
        {
            if (MoveBackward == true)
                AxisDemands[Index].Position *= (-1.0);

            AxisDemands[Index].Position += (Index < 2 ? details->ideal.x : details->ideal.y);

            if (Index < 2)
            {
                toX = doubleToLong(AxisDemands[Index].Position);
                details->ideal.x = toX;
            }
            if (Index == 2)
            {
                toY = doubleToLong(AxisDemands[Index].Position);
                details->ideal.y = toY;
            }
            DEBUG("AxisId: %d, plate position %f, velocity %f\n", AxisDemands[Index].AxisId,
                  AxisDemands[Index].Position, AxisDemands[Index].Velocity);
        }
        ThisTask->tdFgripperSetMainStruct(details);
        ThisTask->tdFgripperConvertFromFP(toX, toY, plateTheta, _FULL,
                                          &toXenc, &toYenc);
        for (unsigned int Index = 0; Index < NumberAxes; Index++)
        {
            if (Index < 2)
            {
                AxisDemands[Index].Position = toXenc;
            }
            else if (Index == 2)
            {
                AxisDemands[Index].Position = toYenc;
            }
            DEBUG("AxisId: %d, encoder position %f, velocity %f\n", AxisDemands[Index].AxisId,
                  AxisDemands[Index].Position, AxisDemands[Index].Velocity);
        }

        if (!(ThisTask->SetupAmps()))
        {
            MessageUser("G_MOVEOFFSET_NT: " + ThisTask->GetError());
        }
        else
        {
            if (!(ThisTask->MoveAxes(AxisDemands, false)))
            {
                MessageUser("G_MOVEOFFSET_NT: " + ThisTask->GetError());
            }
            else
            {
                MessageUser("G_MOVEOFFSET_NT: Move complete.");
                MessageUser("G_MOVEOFFSET_NT: Waiting to receive signals about the current position of the gantry.");
                std::this_thread::sleep_for(2000ms);
                if (!(ThisTask->tdFgripperUpdatePos(YES, details->dprFeedback, YES)))
                {
                    MessageUser("G_MOVEOFFSET_NT: Failed to update the position of gantry.");
                }
            }
        }
        details->inUse = NO;
        ThisTask->tdFgripperSetMainStruct(details);
    }
}

void GEXITActionNT::ActionThread(const drama::sds::Id &)
{

    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
    ThisTask->ClearError();

    std::shared_ptr<tdFgripperTaskType> details = ThisTask->tdFgripperGetMainStruct();
    if (details == nullptr || details->Initialised == NO)
    {
        MessageUser("G_EXIT: TdFGripperTask is not initialised. Please use \"ditscmd TdFGripperTask INITIALISE\" to initialise the task.");
        return;
    }
    if (details->inUse)
    {
        MessageUser("G_EXIT: TdFGripperTask is running other actions.\n");
        return;
    }
    details->inUse = YES;
    ThisTask->tdFgripperSetMainStruct(details);
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
    try
    {
        ThisTask->tdFGetCameraPath().Obey(this, "EXIT");
    }
    catch (...)
    {
        DEBUG("Failed to exit camera\n");
        DramaTHROW(TDFGRIPPERTASK__CAMERA, "G_EXIT: fail to exit the camera.");
    }
    DEBUG("EXIT camera OK\n");
    details->inUse = NO;
    details->Initialised = NO;
    details->cameraInit = NO;
    ThisTask->tdFgripperSetMainStruct(details);
    return;
}

void GMOVEActionNT::ActionThread(const drama::sds::Id &Arg)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
    ThisTask->ClearError();
    // drama::Task::guardType DramaLock(std::shared_ptr<drama::Task>(ThisTask)->Lock());

    std::shared_ptr<tdFgripperTaskType> details = ThisTask->tdFgripperGetMainStruct();

    if (details == nullptr || details->Initialised == NO)
    {
        MessageUser("G_MOVE_NT: TdFGripperTask is not initialised. Please use \"ditscmd TdFGripperTask INITIALISE\" to initialise the task.");
        return;
    }
    if (!(ThisTask->tdFgripperIlocks(ILOCK__GAN_PARKED)))
    {
        MessageUser("G_MOVE_NT: the gantry is parked, please unpark the gantry first.");
        return;
    }
    if (details->inUse)
    {
        MessageUser("G_MOVE_NT: TdFGripperTask is running other actions.\n");
        return;
    }

    std::string Axes;
    std::string Positions;
    std::string Velocities;
    std::string OffsetFlag;
    if (Arg)
    {
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
        double plateTheta;
        drama::ParSys parSysId(ThisTask->TaskPtr());
        parSysId.Get("PLATE_THETA", &plateTheta);
        long int toX, toY;
        double toXenc = 0.0, toYenc = 0.0;
        details->inUse = YES;
        unsigned int NumberAxes = AxisDemands.size();
        for (unsigned int Index = 0; Index < NumberAxes; Index++)
        {
            if (MoveBackward)
                AxisDemands[Index].Position *= (-1.0);
            if (MoveOffset == true)
            {
                AxisDemands[Index].Position += (Axes == "X" ? details->ideal.x : details->ideal.y);
            }
            if (Axes == "X")
            {
                toX = doubleToLong(AxisDemands[Index].Position);
                details->ideal.x = toX;
            }
            else if (Axes == "Y")
            {
                toY = doubleToLong(AxisDemands[Index].Position);
                details->ideal.y = toY;
            }
            DEBUG("AxisId: %d, plate position %f, velocity %f\n", AxisDemands[Index].AxisId,
                  AxisDemands[Index].Position, AxisDemands[Index].Velocity);
        }
        ThisTask->tdFgripperSetMainStruct(details);
        ThisTask->tdFgripperConvertFromFP(toX, toY, plateTheta, _FULL,
                                          &toXenc, &toYenc);
        for (unsigned int Index = 0; Index < NumberAxes; Index++)
        {
            if (Axes == "X")
            {
                AxisDemands[Index].Position = toXenc;
            }
            else if (Axes == "Y")
            {
                AxisDemands[Index].Position = toYenc;
            }
            DEBUG("AxisId: %d, encoder position %f, velocity %f\n", AxisDemands[Index].AxisId,
                  AxisDemands[Index].Position, AxisDemands[Index].Velocity);
        }

        if (!(ThisTask->SetupAmps()))
        {
            MessageUser("G_MOVE_NT: " + ThisTask->GetError());
        }
        else
        {
            if (!(ThisTask->MoveAxes(AxisDemands, false)))
            {
                MessageUser("G_MOVE_NT: " + ThisTask->GetError());
            }
            else
            {
                MessageUser("G_MOVE_NT: Move complete");
                MessageUser("G_MOVE_NT: Waiting to receive signals about the current position of the gantry.");
                std::this_thread::sleep_for(2000ms);
                if (!(ThisTask->tdFgripperUpdatePos(YES, details->dprFeedback, YES)))
                {
                    MessageUser("G_MOVE_NT: Failed to update the position of gantry.");
                }
                details->inUse = NO;
                ThisTask->tdFgripperSetMainStruct(details);
            }
        }
    }
}

void GRESETActionNT::ActionThread(const drama::sds::Id &)
{

    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
    ThisTask->ClearError();

    drama::sds::Id parSysId(drama::sds::Id::CreateFromSdsIdType((long)(DitsGetParId())));

    parSysId.Put("ENQ_VER_NUM", TdFGripperTaskVersion);
    parSysId.Put("ENQ_VER_DATE", TdFGripperTaskDate);
    std::shared_ptr<tdFgripperTaskType> details = ThisTask->tdFgripperGetMainStruct();
    if (details == nullptr)
    {
        details = std::make_shared<tdFgripperTaskType>();
        ThisTask->tdFgripperSetMainStruct(details);
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

    bool defRead = ThisTask->tdFgripperDefRead(DEFS_FILE | FLEX_FILE,
                                               check);
    if (defRead == false)
    {
        details->inUse = NO;
        ThisTask->tdFgripperSetMainStruct(details);
        return;
    }

    details->ipsMode = 0;
    details->dprAddress = ERROR;
    details->toEnc.x = 0;
    details->toEnc.y = 0;

    details->Initialised = YES;
    DEBUG("Reset tdFgripperTaskType OK\n");

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
    ThisTask->tdFgripperSetMainStruct(details);
}

void PTELPOSActionNT::ActionThread(const drama::sds::Id &Arg)
{

    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
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

void PSetActionNT::ActionThread(const drama::sds::Id &Arg)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
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

void PResetLockActionNT::ActionThread(const drama::sds::Id &)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());

    ThisTask->ClearError();
    std::shared_ptr<tdFgripperTaskType> details = ThisTask->tdFgripperGetMainStruct();
    if (details != nullptr)
    {
        details->inUse = NO;
        ThisTask->tdFgripperSetMainStruct(details);
        MessageUser("P_RESET_LOCK: Reset internal lock completed.\n");
    }
    else
    {
        MessageUser("P_RESET_LOCK: the structure pointer is null, please initialise the task first.\n");
    }
}

void PSetCoeffsActionNT::ActionThread(const drama::sds::Id &Arg)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());

    ThisTask->ClearError();
    std::shared_ptr<tdFgripperTaskType> details = ThisTask->tdFgripperGetMainStruct();

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
                    if (!ThisTask->tdFgripperDefWrite(DEFS_FILE + FLEX_FILE, check))
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
        ThisTask->tdFgripperSetMainStruct(details);
    }
    else
    {
        MessageUser("P_SET_COEFFS: the structure pointer is null, please initialise the task first.\n");
    }
}

void PSetImageActionNT::ActionThread(const drama::sds::Id &Arg)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());

    ThisTask->ClearError();
    std::shared_ptr<tdFgripperTaskType> details = ThisTask->tdFgripperGetMainStruct();
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

        if (!ThisTask->tdFgripperDefWrite(DEFS_FILE, check))
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

    ThisTask->tdFgripperSetMainStruct(details);
}

void PSetWindowActionNT::ActionThread(const drama::sds::Id &Arg)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
    ThisTask->ClearError();

    std::shared_ptr<tdFgripperTaskType> details = ThisTask->tdFgripperGetMainStruct();
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
        if (!ThisTask->tdFgripperDefWrite(DEFS_FILE, 0))
        {
            MessageUser("P_SET_WINDOW: Save Window failed " + ThisTask->GetError());
        }
        else
            MessageUser("P_SET_WINDOW: Set Window completed.\n");
        ThisTask->tdFgripperSetMainStruct(details);
    }
    else
    {
        MessageUser("P_SET_WINDOW: No input arguments.\n");
    }
}

void PSetVelActionNT::ActionThread(const drama::sds::Id &Arg)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
    ThisTask->ClearError();
    long int iFeedrate;

    if (Arg)
    {
        drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
        drama::gitarg::Int<GRIPPER_XY_FR_MIN, GRIPPER_XY_FR_MAX> feedRateArg(this, Arg, "feedrate", 1, 0, NoFlags);
        iFeedrate = feedRateArg;
        drama::ParId paramFeedRate(GetTask(), "XY_VEL");
        MessageUser("P_SET_VEL: before the setting, the value of XY_VEL is: \n");
        paramFeedRate.List();
        paramFeedRate.Put((int)iFeedrate);
        MessageUser("P_SET_VEL: after the setting, the value of XY_VEL is: \n");
        paramFeedRate.List();

        if (!ThisTask->tdFgripperDefWrite(DEFS_FILE, 0))
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

void PSetPlateActionNT::ActionThread(const drama::sds::Id &Arg)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());

    ThisTask->ClearError();
    long int iPlate;

    std::shared_ptr<tdFgripperTaskType> details = ThisTask->tdFgripperGetMainStruct();
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
        ThisTask->tdFgripperSetMainStruct(details);
    }
    else
    {
        MessageUser("P_SET_PLATE: No input arguments.\n");
    }
}

void PUpdateFlexActionNT::ActionThread(const drama::sds::Id &)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());

    ThisTask->ClearError();
    short check = SHOW;
    if (!ThisTask->tdFgripperDefRead(FLEX_FILE, check))
    {
        DEBUG("P_UPDATE_FLEX: failed to read tdfGripperFlex.sds.\n");
        MessageUser("P_UPDATE_FLEX: failed to read tdfGripperFlex.sds " + ThisTask->GetError());
    }
    else
    {
        MessageUser("P_UPDATE_FLEX: Update Flex completed.\n");
    }
}

void PUpdateActionNT::ActionThread(const drama::sds::Id &Arg)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
    ThisTask->ClearError();

    std::shared_ptr<tdFgripperTaskType> details = ThisTask->tdFgripperGetMainStruct();
    if (details == nullptr || (details && details->Initialised == NO))
    {
        MessageUser("P_UPDATE: the structure pointer is null, please initialise the task first.\n");
        return;
    }
    if (!ThisTask->tdFgripperIlocks(ILOCK__TASK_INIT))
    {
        MessageUser("Unable to execute this action: " + ThisTask->GetError());
        return;
    }

    MessageUser("P_POLL_POSE: this action is not supported any more.\n");
    return;
}

void PReportActionNT::ActionThread(const drama::sds::Id &Arg)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
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

void PReportLocksActionNT::ActionThread(const drama::sds::Id &)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
    ThisTask->ClearError();
    std::shared_ptr<tdFgripperTaskType> details = ThisTask->tdFgripperGetMainStruct();
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

void PReportImageActionNT::ActionThread(const drama::sds::Id &)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());

    ThisTask->ClearError();
    std::shared_ptr<tdFgripperTaskType> details = ThisTask->tdFgripperGetMainStruct();
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

void PReportWindowActionNT::ActionThread(const drama::sds::Id &Arg)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());

    ThisTask->ClearError();
    std::shared_ptr<tdFgripperTaskType> details = ThisTask->tdFgripperGetMainStruct();
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

void PSaveDefsActionNT::ActionThread(const drama::sds::Id &)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
    ThisTask->ClearError();
    if (!ThisTask->tdFgripperDefWrite(DEFS_FILE + FLEX_FILE, 0))
    {
        MessageUser("P_SAVE_DEFS: Save DEFS_FILE failed " + ThisTask->GetError());
    }
    MessageUser("P_SAVE_DEFS: action completed.\n");
}

void CSearchAction::ActionThread(const drama::sds::Id &Arg)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
    ThisTask->ClearError();

    details = ThisTask->tdFgripperGetMainStruct();
    if (details == nullptr)
    {
        DramaTHROW(TDFGRIPPERTASK__NOTINIT, "C_SEARCH: the structure pointer is null, please initialise the task!");
    }
    if (details->inUse)
    {
        DramaTHROW(TDFGRIPPERTASK__IN_USE, "C_SEARCH: TdFGripperTask is running other actions.");
    }

    // use a shared_pointer instead
    m_tdFgripperSFStruct = std::make_shared<tdFgrip_SFtype>();
    if (m_tdFgripperSFStruct == nullptr)
    {
        DramaTHROW(TDFGRIPPERTASK__MALLOCERR, "C_SEARCH: failed to allocate memory to the SEARCH data structure.");
    }
    if (!Arg)
    {
        DramaTHROW(TDFGRIPPERTASK__NO_ARGUMENTS, "C_SEARCH: no input argument.");
    }
    details->inUse = YES;
    ThisTask->tdFgripperSetMainStruct(details);
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

    parSysId.Get("STEP_SIZE", &m_tdFgripperSFStruct->stepSize);
    m_tdFgripperSFStruct->stepSize = 200;
    parSysId.Get("MAX_ERROR", &m_tdFgripperSFStruct->maxError);
    m_tdFgripperSFStruct->maxError = 200;
    parSysId.Get("POS_TOL", &m_tdFgripperSFStruct->tolerance);
    m_tdFgripperSFStruct->tolerance = 10;
    parSysId.Get("POS_ATTEMPTS", &m_tdFgripperSFStruct->attempts);
    parSysId.Get("SETTLE_TIME", &settletime);
    parSysId.Get("PLATE_THETA", &platetheta);

    m_tdFgripperSFStruct->searchStartX = searchStartX;
    m_tdFgripperSFStruct->searchStartY = searchStartY;
    m_tdFgripperSFStruct->resultsValid = 0;

    auto cenWin = std::make_shared<tdFgrip_CENtype>();
    int i = 1, j = 1, k = 0;   /* Used to determine next search point  */
    short atSearchXY = NO,     /* Flag - above fibre-end location      */
        centroided = NO,       /*      - fibre-end centroided          */
        checkedCentroid = NO,  /*      - analysed last centroid        */
        iAttempts = 0,         /* Current number of positioning trys   */
        foundIt = NO,          /* Have we found the fibre              */
        searchStarted = NO,    /* Have we completed our first move     */
        centroidRepeated = NO, /* Have we repeated the centroid        */
        repeatChecked = NO;    // m_tdFgripperSFStruct->statCheck ? NO : YES; /* Has the repeated centroid been checks */

    double theta = ThisTask->tdFautoThetaPos(m_tdFgripperSFStruct->searchStartX, m_tdFgripperSFStruct->searchStartY);
    forbidden = ThisTask->tdFforbidden(m_tdFgripperSFStruct->searchStartX, m_tdFgripperSFStruct->searchStartY,
                                       theta, QUADRANT_RADIUS, INSIDE_RADIUS,
                                       OUTSIDE_RADIUS, HALF_GUIDE_EXPAN,
                                       JAW_HWP, JAW_HWM, JAW_LENGTH, 0, &inside, &outside);
    if (forbidden || outside)
    {
        details->inUse = NO;
        DramaTHROW_S(TDFGRIPPERTASK__INV_POS, "C_SEARCH: Initial search position (%d,%d) invalid.\n", m_tdFgripperSFStruct->searchStartX, m_tdFgripperSFStruct->searchStartX);
    }
    details->imagePos.enable = 1;
    details->imagePos.displayText = 1;
    details->imagePos.useDpr = details->dprFeedback;

    ThisTask->tdFgripperUpdatePos(YES, details->dprFeedback, YES);

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
    drama::Path taskPath(ThisTask->TaskPtr());
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
            // atSearchXY = YES;
        }
        else if (!centroided)
        {
            details->inUse = NO;
            ThisTask->tdFgripperSetMainStruct(details);
            if (!PerformCentroid(cenWin, settletime, &centroided))
            {
                MessageUser("C_SEARCH: Fail to perform centroid at position (%d,%d).\n", searchX, searchY);
                flag = false;
                break;
            }
            details->inUse = YES;
            ThisTask->tdFgripperSetMainStruct(details);
        }
        else if (!checkedCentroid)
        {

            if (!CheckCentroid(&iAttempts, &searchX, &searchY, &atSearchXY, &centroided,
                               &foundIt, &i, &j, &k, &checkedCentroid, &centroidRepeated, &repeatChecked))
            {
                MessageUser("C_SEARCH: Fail to check centroid at position (%d,%d).\n", searchX, searchY);
                flag = false;
                break;
            }
        }
        else if (!centroidRepeated)
        {
            details->inUse = NO;
            ThisTask->tdFgripperSetMainStruct(details);
            if (!PerformCentroid(cenWin, settletime, &centroidRepeated))
            {
                MessageUser("C_SEARCH: Fail to perform centroid again at position (%d,%d).\n", searchX, searchY);
                flag = false;
                break;
            }
            details->inUse = YES;
            ThisTask->tdFgripperSetMainStruct(details);
        }
        else if (!repeatChecked)
        {
            if (!CheckRepeatCentroid(&centroidRepeated, &repeatChecked))
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
        ActionComplete(searchX, searchY, foundIt);

        newArg = drama::sds::Id::CreateArgStruct();
        double expXenc, expYenc, atXenc, atYenc;
        ThisTask->tdFgripperConvertFromFP(m_tdFgripperSFStruct->searchStartX, m_tdFgripperSFStruct->searchStartY,
                                          platetheta, _FULL, &expXenc, &expYenc);
        // expXenc = m_tdFgripperSFStruct->searchStartX;
        // expYenc = m_tdFgripperSFStruct->searchStartY;

        ThisTask->tdFgripperConvertFromFP(searchX, searchY, platetheta, _FULL,
                                          &atXenc, &atYenc);
        // atXenc = searchX;
        // atYenc = searchY;

        newArg.Put("FOUND", foundIt);
        newArg.Put("MEASUREDX", searchX);
        newArg.Put("MEASUREDY", searchY);
        newArg.Put("EncoderX", atXenc);
        newArg.Put("EncoderY", atYenc);
        newArg.Put("STARTX", m_tdFgripperSFStruct->searchStartX);
        newArg.Put("STARTY", m_tdFgripperSFStruct->searchStartY);
        newArg.Put("XErr", m_tdFgripperSFStruct->xErr);
        newArg.Put("YErr", m_tdFgripperSFStruct->yErr);
        newArg.Put("XEncoderErr", doubleToLong(atXenc - expXenc));
        newArg.Put("YEncoderErr", doubleToLong(atYenc - expYenc));
    }
    SetReturnArg(&newArg);
    details->inUse = NO;
    ThisTask->tdFgripperSetMainStruct(details);
    MessageUser("C_SEARCH: action completed.\n");
}

void CSearchAction::CheckCent_ObjectHasNotYetBeenSeen(long *const searchX, long *const searchY, short *const atSearchXY,
                                                      short *const centroided, short *const attempts, short *const foundIt, int *const i, int *const j, int *const k,
                                                      short *const checkedCentroid, short *const centroidRepeated, short *const repeatChecked)
{
    // needs to fix in the PerformCentroid function;
    if (m_tdFgripperSFStruct->centroidOK == YES)
    {
        CheckCent_ObjectFound(searchX, searchY, atSearchXY, centroided, attempts, foundIt, checkedCentroid);
    }
    else
    {
        CheckCent_ObjectNotFound(searchX, searchY, atSearchXY, centroided, i, j, k, checkedCentroid, centroidRepeated, repeatChecked);
    }
}

bool CSearchAction::CheckRepeatCentroid(short *const centroidRepeated, short *const repeatChecked)
{
    if (m_tdFgripperSFStruct->centroidOK == YES)
    {
        if (m_tdFgripperSFStruct->resultsValid == 0)
            MessageUser("CheckRepeatCentroid: Found object within tolerance - will repeat centroid 10 times");

        m_tdFgripperSFStruct->resultsX[m_tdFgripperSFStruct->resultsValid] = details->imagePos.p.x - m_tdFgripperSFStruct->xErr;
        m_tdFgripperSFStruct->resultsY[m_tdFgripperSFStruct->resultsValid] =
            details->imagePos.p.y - m_tdFgripperSFStruct->yErr;
        ++(m_tdFgripperSFStruct->resultsValid);

        int index = m_tdFgripperSFStruct->resultsValid - 1;
        MessageUser("CheckRepeatCentroid: Search Object found at position %ld, %ld (in tolerance image %d)",
                    m_tdFgripperSFStruct->resultsX[index], m_tdFgripperSFStruct->resultsY[index], m_tdFgripperSFStruct->resultsValid);

#define SEARCH_TOL 4 /* A difference of this amount in 3 centroids is ignored */
        if (m_tdFgripperSFStruct->resultsValid == 3)
        {
            if ((labs(m_tdFgripperSFStruct->resultsX[0] - m_tdFgripperSFStruct->resultsX[1]) <= SEARCH_TOL) &&
                (labs(m_tdFgripperSFStruct->resultsX[0] - m_tdFgripperSFStruct->resultsX[2]) <= SEARCH_TOL) &&
                (labs(m_tdFgripperSFStruct->resultsY[0] - m_tdFgripperSFStruct->resultsY[1]) <= SEARCH_TOL) &&
                (labs(m_tdFgripperSFStruct->resultsY[0] - m_tdFgripperSFStruct->resultsY[2]) <= SEARCH_TOL))
            {
                *repeatChecked = YES;
                MessageUser("CheckRepeatCentroid: First three images give the same result - won't do the other seven");
            }
            else
            {
                *centroidRepeated = NO;

                MessageUser("CheckRepeatCentroid: First three images on thsi object give different results - will do 10");
                MessageUser("CheckRepeatCentroid: Deltas %ld,%ld - %ld,%ld", labs(m_tdFgripperSFStruct->resultsX[0] - m_tdFgripperSFStruct->resultsX[1]),
                            labs(m_tdFgripperSFStruct->resultsY[0] - m_tdFgripperSFStruct->resultsY[1]), labs(m_tdFgripperSFStruct->resultsX[0] - m_tdFgripperSFStruct->resultsX[2]),
                            labs(m_tdFgripperSFStruct->resultsY[0] - m_tdFgripperSFStruct->resultsY[2]));
            }
        }
        else if (m_tdFgripperSFStruct->resultsValid >= SF_IMAGES)
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
        (*searchY += (*i % 2) ? m_tdFgripperSFStruct->stepSize : -m_tdFgripperSFStruct->stepSize);
    else
        (*searchX += (*i % 2) ? m_tdFgripperSFStruct->stepSize : -m_tdFgripperSFStruct->stepSize);
    (*j)++;

    dist = (ABS(*searchX - m_tdFgripperSFStruct->searchStartX) >
            ABS(*searchY - m_tdFgripperSFStruct->searchStartY))
               ? ABS(*searchX - m_tdFgripperSFStruct->searchStartX)
               : ABS(*searchY - m_tdFgripperSFStruct->searchStartY);
    MessageUser("CheckCent_ObjectNotFound: Current search distance is %ld (max %ld). i=%d(%d),j=%d(%d),k=%d(%d)",
                dist, m_tdFgripperSFStruct->maxError, *i, pi, *j, pj, *k, pk);

    if (dist > m_tdFgripperSFStruct->maxError)
    {
        MessageUser("Completed search to distance of %ld microns without finding object.", m_tdFgripperSFStruct->maxError);
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

void CSearchAction::CheckCent_ObjectFound(long *const searchX, long *const searchY, short *const atSearchXY,
                                          short *const centroided, short *const attempts, short *const foundIt, short *const checkedCentroid)
{
    double error;
    double xErr;
    double yErr;
    *foundIt = YES;
    error = sqrt(SQRD((double)m_tdFgripperSFStruct->xErr) + SQRD((double)m_tdFgripperSFStruct->yErr));

    MessageUser("CheckCent_ObjectFound: Search Image found, error = %.1f (%ld,%ld)", error, m_tdFgripperSFStruct->xErr, m_tdFgripperSFStruct->yErr);

    MessageUser("CheckCent_ObjectFound: Robot position error is %ld, %ld", details->imagePos.p.x - *searchX,
                details->imagePos.p.y - *searchY);

    xErr = m_tdFgripperSFStruct->xErr - (details->imagePos.p.x - *searchX);
    yErr = m_tdFgripperSFStruct->yErr - (details->imagePos.p.y - *searchY);
    error = sqrt(SQRD(xErr) + SQRD(yErr));
    MessageUser("CheckCent_ObjectFound: After subtracting robot pos error, error = %.1f (%ld,%ld)",
                error, (long)xErr, (long)yErr);
    if ((error <= (double)m_tdFgripperSFStruct->tolerance) ||
        (*attempts >= m_tdFgripperSFStruct->attempts))
    {
        *checkedCentroid = YES;
        *searchX = details->imagePos.p.x - m_tdFgripperSFStruct->xErr;
        *searchY = details->imagePos.p.y - m_tdFgripperSFStruct->yErr;
        m_tdFgripperSFStruct->resultsX[0] = *searchX;
        m_tdFgripperSFStruct->resultsY[0] = *searchY;
        m_tdFgripperSFStruct->resultsValid = 1;
        MessageUser("CheckCent_ObjectFound: Search Object found at position %ld, %ld (in tolerance image %d)",
                    m_tdFgripperSFStruct->resultsX[0], m_tdFgripperSFStruct->resultsY[0], m_tdFgripperSFStruct->resultsValid);
    }
    else
    {
        MessageUser("CheckCent_ObjectFound: Object was outside tolerance (error %g, tolerance %ld), try to get closer",
                    error, m_tdFgripperSFStruct->tolerance);
        *atSearchXY = *centroided = NO;
        *searchX = details->imagePos.p.x - m_tdFgripperSFStruct->xErr;
        *searchY = details->imagePos.p.y - m_tdFgripperSFStruct->yErr;
        (*attempts)++;
    }
}

bool CSearchAction::CheckCentroid(short *const attempts, long *const searchX, long *const searchY, short *const atSearchXY,
                                  short *const centroided, short *const foundIt, int *const i, int *const j, int *const k, short *const checkedCentroid,
                                  short *const centroidRepeated, short *const repeatChecked)
{
    if (*attempts == 0)
    {
        CheckCent_ObjectHasNotYetBeenSeen(searchX, searchY,
                                          atSearchXY, centroided, attempts, foundIt, i, j, k,
                                          checkedCentroid, centroidRepeated, repeatChecked);
    }
    else if (m_tdFgripperSFStruct->centroidOK != YES)
    {
        MessageUser("CheckCentroid: Fibre was found, but is no longer in field of view");
        return false;
    }
    else
    {
        CheckCent_ObjectFound(searchX, searchY, atSearchXY, centroided,
                              attempts, foundIt, checkedCentroid);
    }
    return true;
}

bool CSearchAction::ActionComplete_CalculateMean(long *const searchX, long *const searchY)
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
        xSum += (double)m_tdFgripperSFStruct->resultsX[i];
        ySum += (double)m_tdFgripperSFStruct->resultsY[i];
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
        d = ((double)m_tdFgripperSFStruct->resultsX[i] - xMean);
        xDev += d * d;
        d = ((double)m_tdFgripperSFStruct->resultsY[i] - yMean);
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
        if ((m_tdFgripperSFStruct->resultsX[i] >= minX) &&
            (m_tdFgripperSFStruct->resultsX[i] <= maxX) &&
            (m_tdFgripperSFStruct->resultsY[i] >= minY) &&
            (m_tdFgripperSFStruct->resultsY[i] <= maxY))
        {
            xSum += (double)m_tdFgripperSFStruct->resultsX[i];
            ySum += (double)m_tdFgripperSFStruct->resultsY[i];
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
        return true;
    }

    else if (usedCounter == 1)
    {
        MessageUser("ActionComplete_CalculateMean: After search rejected outlier image centroids, there was only one value left.");
        return false;
    }
    else
    {
        MessageUser("ActionComplete_CalculateMean: After search rejected outliers image centroids, there were no values left.");
        return false;
    }
}

void CSearchAction::ActionComplete_FoundItCheck(long *const searchX, long *const searchY, short *const foundIt)
{
    long int dist;

    if (m_tdFgripperSFStruct->resultsValid != 3)
    {
        if (!(ActionComplete_CalculateMean(searchX, searchY)))
            return;
    }

    if (ABS(*searchX - m_tdFgripperSFStruct->searchStartX) >
        ABS(*searchY - m_tdFgripperSFStruct->searchStartY))
        dist = ABS(*searchX - m_tdFgripperSFStruct->searchStartX);
    else
        dist = ABS(*searchY - m_tdFgripperSFStruct->searchStartY);

    if (dist > m_tdFgripperSFStruct->maxError)
    {
        MessageUser("ActionComplete_FoundItCheck: Search found object outside search distance - so it is considered not found");
        *foundIt = NO;
    }
}

void CSearchAction::ActionComplete(long searchX, long searchY, short &foundIt)
{
    if (foundIt == YES)
    {
        ActionComplete_FoundItCheck(&searchX, &searchY, &foundIt);
    }

    MessageUser("ActionComplete: Given fibre location  = %ld,%ld", m_tdFgripperSFStruct->searchStartX, m_tdFgripperSFStruct->searchStartY);
    MessageUser("ActionComplete: Current encoder location = %d,%d", details->imagePos.enc.x, details->imagePos.enc.y);
    if (foundIt == YES)
    {
        MessageUser("ActionComplete: Final fibre position  = %ld,%ld", searchX, searchY);
        MessageUser("ActionComplete: Offset from expected     = %ld,%ld", doubleToLong(m_tdFgripperSFStruct->searchStartX - searchX), doubleToLong(m_tdFgripperSFStruct->searchStartY - searchY));
        MessageUser("ActionComplete: Gantry/image-centroid offset = %ld,%ld", m_tdFgripperSFStruct->xErr, m_tdFgripperSFStruct->yErr);
    }
    else
        MessageUser("ActionComplete: Image not found within %ld microns from given location", m_tdFgripperSFStruct->maxError);
}

void CCentroidAction::ActionThread(const drama::sds::Id &Arg)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
    ThisTask->ClearError();
    std::shared_ptr<tdFgripperTaskType> details = ThisTask->tdFgripperGetMainStruct();
    if (details == nullptr)
    {
        DramaTHROW(TDFGRIPPERTASK__NOTINIT, "C_CENTROID: the structure pointer is null, please initialise the task!");
    }
    if (details->inUse)
    {
        DramaTHROW(TDFGRIPPERTASK__IN_USE, "C_CENTROID: TdFGripperTask is running other actions.");
    }
    if (!Arg) // throw out if there is no argument quickly
    {
        DramaTHROW(TDFGRIPPERTASK__NO_ARGUMENTS, "C_CENTROID: No input argument is provided.");
    }

    drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
    drama::gitarg::String ImageArg(this, Arg, "Image", 1, "FREE", NoFlags);
    string strImage = "FREE";
    if (ImageArg != "FREE" && ImageArg != "free")
    {
        DramaTHROW(TDFGRIPPERTASK__INV_INPUT_ARGUMENT, "C_CENTROID: TdFGripperTask can only take \"FREE\" Image type.");
    }
    drama::gitarg::String WindowArg(this, Arg, "Window", 2, "FULL", NoFlags);
    if (WindowArg != "FULL" && WindowArg != "NORM" && WindowArg != "SEARCH")
    {
        DramaTHROW(TDFGRIPPERTASK__INV_INPUT_ARGUMENT, "C_CENTROID: TdFGripperTask can only take \"FULL\", \"NORM\", \"SEARCH\" Window type.");
    }

    string strWindow = WindowArg;
    auto cenData = std::make_shared<tdFgrip_CENtype>();
    drama::ParSys parSysId(ThisTask->TaskPtr());
    sprintf(cenData->saveName, "%s_%ld", "Centroid", (long int)time(0));
    cenData->window.MaxX = details->freeImg.xMax;
    cenData->window.MaxY = details->freeImg.yMax;
    cenData->window.PixelSize = details->freeImg.PixelSize;
    cenData->img = &details->freeImg;

    details->imagePos.enable = 1;
    details->imagePos.displayText = 1;
    details->imagePos.useDpr = YES;
    details->inUse = YES;
    ThisTask->tdFgripperSetMainStruct(details);

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

    ThisTask->tdFgripperPreExp();
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

    messageArg.Put("CENTROID", strWindowType);
    messageArg.Put("BIAS", cenData->img->bias);
    messageArg.Put("EXPOSURE_TIME", cenData->img->exposureTime);
    messageArg.Put("SHUTTER_OPEN", (int)cenData->img->shutter);
    messageArg.Put("UPDATE", cenData->img->updateTime);

    try
    {
        ThisTask->tdFGetCameraPath().Obey(this, "CENTRECT", messageArg, &returnedArg);
    }
    catch (std::exception &what)
    {
        details->inUse = NO;
        ThisTask->tdFgripperSetMainStruct(details);
        DramaTHROW(TDFGRIPPERTASK__NO_IMAGE, "C_CENTROID: vimba failed to take centroid.");
    }

    drama::sds::Id newArg;
    if (*returnedArg)
    {
        newArg = drama::sds::Id::CreateArgStruct();
        double xCent, yCent, xFull, yFull, dFWHM;
        short centroidOK;
        returnedArg->Get("XVALUE", &xCent);
        returnedArg->Get("YVALUE", &yCent);
        returnedArg->Get("XFULL", &xFull);
        returnedArg->Get("YFULL", &yFull);
        returnedArg->Get("FWHM", &dFWHM);
        returnedArg->Get("FIBREINIMAGE", &centroidOK);

        MessageUser("C_CENTROID:  result \nCentroid:  %lf %lf \nWinodw Size: %lf %lf\nFull Width Half Max: %lf",
                    xCent - (double)cenData->window.Xoffset, yCent - (double)cenData->window.Yoffset, xFull, yFull, dFWHM);
        double oldXerr, oldYerr, cosT, sinT;
        double plateTheta;
        double xErr = 0.0, yErr = 0.0;
        parSysId.Get("PLATE_THETA", &plateTheta);
        cosT = cos(-plateTheta);
        sinT = sin(-plateTheta);

        slaXy2xy(xCent, yCent, cenData->img->camCoeffs, &xErr, &yErr);
        oldXerr = xErr;
        oldYerr = yErr;
        xErr = oldXerr * cosT - oldYerr * sinT;
        yErr = oldYerr * cosT + oldXerr * sinT;
        MessageUser("C_CENTROID: Fibre-end %s:  %ld,%ld (microns) %.3f,%.3f (pixels)",
                    centroidOK ? "in image" : "NOT in image",
                    doubleToLong(xErr), doubleToLong(yErr), xCent, yCent);
        MessageUser("C_CENTROID: (centroid for plate theta = %.3f)", plateTheta);
        if (details->imagePos.enable)
        {
            ThisTask->tdFgripperPostExp();
            MessageUser("C_CENTROID: Actual gantry position is x:%ld, y:%ld",
                        details->imagePos.p.x, details->imagePos.p.y);
        }
        newArg.Put("XFULL", xFull);
        newArg.Put("YFULL", yFull);
        newArg.Put("XERR", doubleToLong(xErr));
        newArg.Put("YERR", doubleToLong(yErr));
        newArg.Put("FIBREINIMAGE", centroidOK);
        SetReturnArg(&newArg);
        MessageUser("C_CENTROID: Set the return parameters successfully.\n");
    }

    details->inUse = NO;
    ThisTask->tdFgripperSetMainStruct(details);
    MessageUser("C_CENTROID: - Action complete.");
}

void CImageAction::ActionThread(const drama::sds::Id &Arg)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
    ThisTask->ClearError();
    std::shared_ptr<tdFgripperTaskType> details = ThisTask->tdFgripperGetMainStruct();
    if (details == nullptr)
    {
        DramaTHROW(TDFGRIPPERTASK__NOTINIT, "C_IMAGE: the structure pointer is null, please initialise the task!");
    }
    if (details->inUse)
    {
        DramaTHROW(TDFGRIPPERTASK__IN_USE, "C_IMAGE: TdFGripperTask is running other actions.");
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
    ThisTask->tdFgripperSetMainStruct(details);
    auto cenData = std::make_shared<tdFgrip_CENtype>();
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

    /// setBuffer class and set the buffer size before getting the path;
    // buffer global size in the task initialisation;
    // set buffer on the path

    ThisTask->tdFGetCameraPath().GetPath(this);

    drama::sds::Id messageArg(drama::sds::Id::CreateArgStruct());
    drama::sds::IdPtr returnedArg;

    messageArg.Put("EXPOSURE_TIME", cenData->img->exposureTime);
    messageArg.Put("SHUTTER_OPEN", (int)cenData->img->shutter);
    messageArg.Put("UPDATE", cenData->img->updateTime);
    try
    {
        ThisTask->tdFGetCameraPath().Obey(this, "GRAB", messageArg, &returnedArg);
    }
    catch (...)
    {
        details->inUse = NO;
        ThisTask->tdFgripperSetMainStruct(details);
        DramaTHROW(TDFGRIPPERTASK__NO_IMAGE, "C_IMAGE: no image was obtained by the camera");
    }

    if (!(*returnedArg))
    {
        details->inUse = NO;
        ThisTask->tdFgripperSetMainStruct(details);
        DramaTHROW(TDFGRIPPERTASK__NO_IMAGE, "C_IMAGE: No image is taken, please check the camera.");
    }
    drama::sds::Id DataArray = returnedArg->Find("DATA_ARRAY");
    if (DataArray)
    {
        MessageUser("C_IMAGE: - Print out Image data.");
        DataArray.List();
    }

    details->inUse = NO;
    ThisTask->tdFgripperSetMainStruct(details);
    MessageUser("C_IMAGE: - Action complete.");
}

void CZeroCamAction::ActionThread(const drama::sds::Id &)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
    ThisTask->ClearError();
    std::shared_ptr<tdFgripperTaskType> details = ThisTask->tdFgripperGetMainStruct();
    if (details == nullptr)
    {
        DramaTHROW(TDFGRIPPERTASK__NOTINIT, "C_ZEROCAM: the structure pointer is null, please initialise the task!");
    }
    if (details->inUse)
    {
        DramaTHROW(TDFGRIPPERTASK__IN_USE, "C_ZEROCAM: TdFGripperTask is running other actions.");
    }

    m_tdFgripperZCStruct = std::make_shared<tdFgrip_ZCtype>();
    if (m_tdFgripperZCStruct == nullptr)
    {
        std::cout << "C_ZEROCAM: m_tdFgripperZCStruct is nullptr\n";
        return;
    }
    drama::ParSys parSysId(ThisTask->TaskPtr());
#define MAX_POINTS 25
    GCamWindowType cenWin;
    double grid[MAX_POINTS][2], measured[MAX_POINTS][2], settleTime, plateTheta;
    long int cenX, cenY;
    bool atNextPoint, centroided, doneGrid, settled;
    short curPoint;

    std::vector<std::string> fitErrors = {"illegal itype",
                                          "insufficent data", "singular solution"};
#define FITERR_ILL_ITYPE 0
#define FITERR_INSUFF_DATA 1
#define FITERR_SINGULAR 2

    if (m_tdFgripperZCStruct->reset == YES)
    {
        long int i, j, k, nextX, nextY;
        details->imagePos.enable = 1;
        details->imagePos.displayText = YES;
        details->imagePos.useDpr = details->dprFeedback;

        ThisTask->tdFgripperUpdatePos(YES, details->dprFeedback, YES);
        cenX = details->ideal.x;
        cenY = details->ideal.y;
        parSysId.Get("SETTLE_TIME", &settleTime);
        parSysId.Get("PLATE_THETA", &plateTheta);
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
        m_tdFgripperZCStruct->reset = NO;
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
            ThisTask->tdFgripperPreExp();
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
            try
            {
                ThisTask->tdFGetCameraPath().Obey(this, "CENTRECT", messageArg, &returnedArg);
            }
            catch (...)
            {
                DramaTHROW(TDFGRIPPERTASK__CAMERA, "C_ZEROCAM: the camera failed to take centroid.");
            }
            if (*returnedArg)
            {
                double xCent, yCent, xFull, yFull, dFWHM;
                short centroidOK = YES;
                returnedArg->Get("XVALUE", &xCent);
                returnedArg->Get("YVALUE", &yCent);
                returnedArg->Get("XFULL", &xFull);
                returnedArg->Get("YFULL", &yFull);
                returnedArg->Get("FWHM", &dFWHM);
                returnedArg->Get("FIBREINIMAGE", &centroidOK);
                MessageUser("C_ZEROCAM: centroid result \nError:  %lf %lf \nWinodw Size: %lf %lf\nFull Width Half Max: %lf",
                            xCent, yCent, xFull, yFull, dFWHM);
                m_tdFgripperZCStruct->centroidOK = centroidOK ? YES : NO;
                m_tdFgripperZCStruct->xFull = xFull;
                m_tdFgripperZCStruct->yFull = yFull;
            }
            ThisTask->tdFgripperPostExp();
        }
        else if (!doneGrid)
        {
            if (!atNextPoint)
            {
                atNextPoint = YES;
                centroided = NO;
                long int XCoor = cenX + doubleToLong(grid[curPoint][_XI]);
                long int YCoor = cenY + doubleToLong(grid[curPoint][_YI]);

                double toXenc = 0.0, toYenc = 0.0;
                ThisTask->tdFgripperConvertFromFP(XCoor, YCoor, plateTheta, _FULL,
                                                  &toXenc, &toYenc);
                details->ideal.x = XCoor;
                details->ideal.y = YCoor;

                details->toEnc.x = (int)doubleToLong(toXenc);
                details->toEnc.y = (int)doubleToLong(toYenc);

                MessageUser("C_ZEROCAM: About to move to next grid point (%ld,%ld)", toXenc, toYenc);
                drama::Path thisTaskPath(_theTask);
                thisTaskPath.GetPath(this);
                drama::sds::Id messageArg(drama::sds::Id::CreateArgStruct());
                std::string strAxis, strCoordinates, strVelocity;
                strAxis = "X,Y";
                strCoordinates = to_string(doubleToLong(XCoor)) + "," + to_string(doubleToLong(YCoor));
                strVelocity = "1,1";
                messageArg.Put("AXES", strAxis);
                messageArg.Put("POSITIONS", strCoordinates);
                messageArg.Put("VELOCITIES", strVelocity);
                try
                {
                    thisTaskPath.Obey(this, "G_MOVE_AXIS_NT", messageArg);
                }
                catch (...)
                {
                    DramaTHROW(TDFGRIPPERTASK__NO_GANTRY_MOVEMENT, "C_ZEROCAM: the gantry failed to move to the position.");
                }

                if ((*details->pars.zeroCamCenWait) > 0)
                    settled = NO;
            }
            else
            {
                if (m_tdFgripperZCStruct->centroidOK != YES)
                {
                    flag = false;
                    break;
                }
                else
                {
                    measured[curPoint][_XI] = m_tdFgripperZCStruct->xFull;
                    measured[curPoint][_YI] = m_tdFgripperZCStruct->yFull;
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
    ThisTask->tdFgripperSetMainStruct(details);
    MessageUser("C_ZEROCAM: - Action complete.");
}

void CShiftCoAction::ActionThread(const drama::sds::Id &Arg)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
    ThisTask->ClearError();
    std::shared_ptr<tdFgripperTaskType> details = ThisTask->tdFgripperGetMainStruct();
    if (details == nullptr)
    {
        DramaTHROW(TDFGRIPPERTASK__NOTINIT, "C_SHIFT_COEFFS: the structure pointer is null, please initialise the task!");
    }
    if (details->inUse)
    {
        DramaTHROW(TDFGRIPPERTASK__IN_USE, "C_SHIFT_COEFFS: TdFGripperTask is running other actions.");
    }

    m_tdFgripperSHStruct = std::make_shared<tdFgrip_SHtype>();
    drama::ParSys parSysId(ThisTask->TaskPtr());
    if (!Arg)
    {
        DramaTHROW(TDFGRIPPERTASK__NO_ARGUMENTS, "C_SHIFT_COEFFS: No input argument is provided.");
    }
    int expX, expY;
    drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
    drama::gitarg::Int<XMIN, XMAX> XFArg(this, Arg, "expX", 1, 0, NoFlags);
    expX = XFArg;
    m_tdFgripperSHStruct->expX = expX;
    drama::gitarg::Int<YMIN, YMAX> YFArg(this, Arg, "expY", 2, 0, NoFlags);
    expY = YFArg;
    m_tdFgripperSHStruct->expY = expY;

    short doneSearch;
    if (m_tdFgripperSHStruct->reset == YES)
    {
        doneSearch = NO;
        m_tdFgripperSHStruct->reset = NO;
        m_tdFgripperSHStruct->found = NO;
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
        try
        {
            thisTaskPath.Obey(this, "C_SEARCH", messageArg, &returnedArg);
        }
        catch (...)
        {
            DramaTHROW(TDFGRIPPERTASK__SEARCH, "C_SHIFT_COEFFS: errors occured during the search.");
        }
        if (*returnedArg)
        {
            returnedArg->Get("FOUND", &m_tdFgripperSHStruct->found);
            returnedArg->Get("MEASUREDX", &m_tdFgripperSHStruct->measuredX);
            returnedArg->Get("MEASUREDY", &m_tdFgripperSHStruct->measuredY);
        }
    }
    if (doneSearch)
    {
        double expXenc, expYenc, atXenc, atYenc,
            plateTheta, dx, dy;

        parSysId.Get("PLATE_THETA", &plateTheta);
        ThisTask->tdFgripperConvertFromFP(m_tdFgripperSHStruct->expX, m_tdFgripperSHStruct->expY,
                                          plateTheta, _FULL,
                                          &expXenc, &expYenc);
        // expXenc = m_tdFgripperSHStruct->expX;
        // expYenc = m_tdFgripperSHStruct->expY;
        ThisTask->tdFgripperConvertFromFP(m_tdFgripperSHStruct->measuredX, m_tdFgripperSHStruct->measuredY,
                                          plateTheta, _FULL,
                                          &atXenc, &atYenc);
        // atXenc = m_tdFgripperSHStruct->measuredX;
        // atYenc = m_tdFgripperSHStruct->measuredY;
        dx = atXenc - expXenc;
        dy = atYenc - expYenc;

        if (m_tdFgripperSHStruct->found == NO)
        {
            MessageUser("C_SHIFT_COEFFS: Search for image at %ld,%ld failed.\n", expX, expY);
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
        MessageUser("C_SHIFT_COEFFS: Measured shift = %.1f,%.1f", dx, dy);
        MessageUser("C_SHIFT_COEFFS: New COEFFS array is %f,%f,%f,%f,%f,%f",
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
    ThisTask->tdFgripperSetMainStruct(details);
    MessageUser("C_SHIFT_COEFFS: - Action complete.");
}

bool CSurveyAction::FidNotFound(double *const expectedX, double *const expectedY,
                                const short curFid, short *const atFid, short *const recordedFid)
{
    DEBUG("C_SURVEY: Search for fiducial index %d, at %ld,%ld failed\n", curFid,
          m_tdFgripperSStruct->x[curFid], m_tdFgripperSStruct->y[curFid]);

    --m_tdFgripperSStruct->numMarks;
    if ((m_tdFgripperSStruct->numMarks) < MIN_MARKS)
    {
        DEBUG("C_SURVEY: Insufficent marks found to do a survey, have %d, need %d\n",
              m_tdFgripperSStruct->numMarks, MIN_MARKS);
        return false;
    }

    drama::ParSys parSysId(_theTask);
    if (curFid == m_tdFgripperSStruct->numMarks)
    {
        *recordedFid = YES;
        parSysId.Put("SURVEY_PROG", 100.0);
        return true;
    }
    for (int i = curFid; i < m_tdFgripperSStruct->numMarks; ++i) /// why is no larger than numMarks here?
    {
        m_tdFgripperSStruct->x[i] = m_tdFgripperSStruct->x[i + 1];
        m_tdFgripperSStruct->y[i] = m_tdFgripperSStruct->y[i + 1];
        expectedX[i] = expectedX[i + 1];
        expectedY[i] = expectedY[i + 1];
    }

    parSysId.Put("SURVEY_PROG",
                 100.0 * (curFid + 1.0) / (double)m_tdFgripperSStruct->numMarks);
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

    auto thisTask = _theTask.lock()->TaskPtrAs<TdFGripperTask>();
    thisTask->tdFgripperConvertFromFP(m_tdFgripperSStruct->x[*curFid], m_tdFgripperSStruct->y[*curFid],
                                      plateTheta, m_tdFgripperSStruct->area,
                                      &expectedX[*curFid], &expectedY[*curFid]);
    // expectedX[*curFid] = m_tdFgripperSStruct->x[*curFid];
    // expectedY[*curFid] = m_tdFgripperSStruct->y[*curFid];

    thisTask->tdFgripperConvertFromEnc(m_tdFgripperSStruct->xEnc, m_tdFgripperSStruct->yEnc,
                                       plateTheta, m_tdFgripperSStruct->area,
                                       &measuredX[*curFid], &measuredY[*curFid]);
    // measuredX[*curFid] = m_tdFgripperSStruct->xEnc;
    // measuredY[*curFid] = m_tdFgripperSStruct->yEnc;
    if (m_tdFgripperSStruct->area == _COEFFS)
    {
        double xErr, yErr, absErr;
        double expEncX, expEncY;
        thisTask->tdFgripperConvertFromFP(m_tdFgripperSStruct->x[*curFid], m_tdFgripperSStruct->y[*curFid],
                                          plateTheta, _FULL,
                                          &expEncX, &expEncY);
        // expEncX = m_tdFgripperSStruct->x[*curFid];
        // expEncY = m_tdFgripperSStruct->y[*curFid];

        xErr = expEncX - m_tdFgripperSStruct->xEnc;
        yErr = expEncY - m_tdFgripperSStruct->yEnc;
        absErr = sqrt(SQRD(xErr) + SQRD(yErr));
        DEBUG("C_SURVEY: Fiducial index %d found %.1f (x:%.1f, y:%.1f) microns from the expected position\n", *curFid, absErr, xErr, yErr);
    }

    *offsetX += m_tdFgripperSStruct->dx;
    *offsetY += m_tdFgripperSStruct->dy;

    if (*curFid == m_tdFgripperSStruct->numMarks - 1)
    {
        *recordedFid = YES;
        parSysId.Put("SURVEY_PROG", 100.0);
    }
    else
    {
        parSysId.Put("SURVEY_PROG",
                     100.0 * (*curFid + 1.0) / (double)m_tdFgripperSStruct->numMarks);
        *atFid = NO;
        (*curFid)++;
    }
}

bool CSurveyAction::SearchForFid(const long offsetX, const long offsetY,
                                 const short curFid, short *const atFid)
{
    *atFid = YES;
    drama::Path thisTaskPath(_theTask);
    thisTaskPath.GetPath(this);
    drama::sds::Id messageArg(drama::sds::Id::CreateArgStruct());
    long int startX, startY;
    startX = m_tdFgripperSStruct->x[curFid] + offsetX;
    startY = m_tdFgripperSStruct->y[curFid] + offsetY;
    messageArg.Put("XF", startX);
    messageArg.Put("YF", startY);
    drama::sds::IdPtr returnedArg;

    DEBUG("C_SURVEY: Will search for fiducial index %d at %7ld, %7ld - search at %7ld, %7ld\n",
          curFid, m_tdFgripperSStruct->x[curFid], m_tdFgripperSStruct->y[curFid], startX, startY);
    try
    {
        thisTaskPath.Obey(this, "C_SEARCH", messageArg, &returnedArg);
    }
    catch (...)
    {
        DramaTHROW(TdFGripperTask__SEARCH, "C_SURVEY: errors occured during the search.\n");
    }
    if (*returnedArg)
    {
        returnedArg->Get("FOUND", &m_tdFgripperSStruct->found);
        if (m_tdFgripperSStruct->found == NO)
        {
            m_tdFgripperSStruct->xEnc = m_tdFgripperSStruct->yEnc = 0;
        }
        else
        {
            returnedArg->Get("EncoderX", &m_tdFgripperSStruct->xEnc);
            returnedArg->Get("EncoderY", &m_tdFgripperSStruct->yEnc);
            returnedArg->Get("XEncoderErr", &m_tdFgripperSStruct->dx);
            returnedArg->Get("YEncoderErr", &m_tdFgripperSStruct->dy);
        }
    }
    else
    {
        return false;
    }

    return true;
}

void CSurveyAction::DisplayResultsBasic(const int newmodel, double fiducialArray[][2],
                                        double cal[][2], const double xrms, const double yrms, const double rrms, drama::sds::Id *paramId)
{
    int counter;

    std::vector<double> xErr(NUM_FIDUCIALS, INT_MIN);
    std::vector<double> yErr(NUM_FIDUCIALS, INT_MIN);
    std::vector<double> xExpected(NUM_FIDUCIALS, INT_MIN);
    std::vector<double> yExpected(NUM_FIDUCIALS, INT_MIN);
    std::vector<double> xCalculated(NUM_FIDUCIALS, INT_MIN);
    std::vector<double> yCalculated(NUM_FIDUCIALS, INT_MIN);

    /// this might be an error?
    for (counter = 0; counter < m_tdFgripperSStruct->numMarks; counter++)
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
        dims.push_back(m_tdFgripperSStruct->numMarks);
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
    for (counter = 0; counter < m_tdFgripperSStruct->numMarks; counter++)
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

void CSurveyAction::tdFgripperSurveyCheckCoeffs(const double *current, const double *newcoeffs,
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

bool CSurveyAction::OffsetAndDisplayResults(const double platetheta, const short fitType, const short centerFid, double measuredArray[][2], double fiducialArray[][2],
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

    slaFitxy(fitType, m_tdFgripperSStruct->numMarks, fiducialArray, measuredArray, diffCoeffs, &j);
    slaPxy(m_tdFgripperSStruct->numMarks, fiducialArray, measuredArray, details->convert.invCoeffs, calculatedOrigModel, &xrms2, &yrms2, &rrms2);

    slaPxy(m_tdFgripperSStruct->numMarks, fiducialArray, measuredArray, diffCoeffs, calculatedNewModel, &xrms, &yrms, &rrms);
    slaPxy(m_tdFgripperSStruct->numMarks, fiducialArray, measuredArray, diffCoeffs, calculatedNewModelNoOffset, &xrms, &yrms, &rrms);

    if ((centerFid >= 0) && (centerFid < m_tdFgripperSStruct->numMarks))
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
            slaPxy(m_tdFgripperSStruct->numMarks, fiducialArray, measuredArray,
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

    DisplayResultsInEncoderUnits(platetheta, 0,
                                 details->convert.coeffs,
                                 measuredArray);
    if (adjustedCenter)
        DisplayResultsInEncoderUnits(platetheta,
                                     10, origNewCoeffs,
                                     measuredArray);
    DisplayResultsInEncoderUnits(platetheta, 1, coeffs,
                                 measuredArray);

    tdFgripperSurveyCheckCoeffs(details->convert.coeffs, coeffs,
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

void CSurveyAction::DisplayResultsInEncoderUnits(const double plateTheta,
                                                 const int newmodel, const double *const coeffs, double measuredArray[][2])
{
    int counter;
    double savedCoeffs[6];
    int j;
    auto thisTask = _theTask.lock()->TaskPtrAs<TdFGripperTask>();
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
    for (counter = 0; counter < m_tdFgripperSStruct->numMarks; counter++)
    {
        double encExpX, encExpY;
        double xErr, yErr;
        /* Convert original positions to the encoder value we
           would have searched at */
        thisTask->tdFgripperConvertFromFP(m_tdFgripperSStruct->x[counter], m_tdFgripperSStruct->y[counter],
                                          plateTheta, _FULL, &encExpX, &encExpY);
        // encExpX = m_tdFgripperSStruct->x[counter];
        // encExpY = m_tdFgripperSStruct->y[counter];

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

void CSurveyAction::SetCoeffs(const double *const expectedX, const double *const expectedY,
                              const double *const measuredX, const double *const measuredY,
                              const short fitType, double *const coeffs, const double plateTheta, drama::sds::Id *paramId)
{
    short centerFid = -1; /* Index of center fiducial */
    double(*expectedArray)[2] = new double[m_tdFgripperSStruct->numMarks][2];
    double(*measuredArray)[2] = new double[m_tdFgripperSStruct->numMarks][2];
    double(*fiducialArray)[2] = new double[m_tdFgripperSStruct->numMarks][2];

    DEBUG("C_SURVEY: Updating gantry field-plate/encoder transformation matrix...\n");
    int j;
    for (j = 0; j < m_tdFgripperSStruct->numMarks; j++)
    {
        expectedArray[j][_XI] = expectedX[j];
        expectedArray[j][_YI] = expectedY[j];
        measuredArray[j][_XI] = measuredX[j];
        measuredArray[j][_YI] = measuredY[j];

        fiducialArray[j][_XI] = m_tdFgripperSStruct->x[j];
        fiducialArray[j][_YI] = m_tdFgripperSStruct->y[j];

        if ((labs(m_tdFgripperSStruct->x[j]) < 5000) && (labs(m_tdFgripperSStruct->y[j]) < 5000))
            centerFid = j;
    }

    slaFitxy(fitType,                       /* Type of model to use (see slaLib docs) */
             m_tdFgripperSStruct->numMarks, /* Number of marks surveyed             */
             measuredArray,                 /* Array of measured values             */
             expectedArray,                 /* Array of expected values             */
             coeffs,                        /* New coeffs array                     */
             &j);                           /* Modified status                      */

    if (j != 0)
    {
        DEBUG("C_SURVEY: Error calculating coeffs array.\n");
        return;
    }

    DEBUG("C_SURVEY: First pass array = %.1f, %.6f, %.6f, %.1f, %.6f, %.6f\n",
          coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4], coeffs[5]);

    if (m_tdFgripperSStruct->area == _COEFFS)
        *paramId = (drama::sds::Id::CreateArgStruct("COEFFS"));

    bool checkFlag = OffsetAndDisplayResults(plateTheta, fitType, centerFid,
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

    delete[] expectedArray;
    delete[] measuredArray;
    delete[] fiducialArray;
}
void CSurveyAction::ConstructReturnValue(const double *const expectedX, const double *const expectedY, const double *const measuredX,
                                         const double *const measuredY, const double *const coeffs, const double ha,
                                         const double dec, drama::sds::Id *paramId)
{
    time_t now;
    std::vector<unsigned long> dims;
    dims.push_back(m_tdFgripperSStruct->numMarks);
    time(&now);

    drama::sds::Id id = paramId->Copy();
    if (m_tdFgripperSStruct->area == _ALL)
        id = (drama::sds::Id::CreateArgStruct("ALL"));
    else if (m_tdFgripperSStruct->area == _TEMP)
        id = (drama::sds::Id::CreateArgStruct("TEMP"));
    else if (m_tdFgripperSStruct->area == _FLEX)
        id = (drama::sds::Id::CreateArgStruct("FLEX"));
    else if (!paramId) /* _COEFFS */
        id = (drama::sds::Id::CreateArgStruct("COEFFS"));

    id.Put("fitTime", (INT32)(now));
    id.Put("plate", details->currentPlate);
    id.Put("instrument", "2dF");
    id.Put("gantry", "GRIPPER");

    id.Put("numMarks", m_tdFgripperSStruct->numMarks);

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
    id.Put("temp", m_tdFgripperSStruct->temp); /* Just dummy at moment */
    paramId->ShallowCopy(&id, true);
}

void CSurveyAction::ActionComplete(const double *const expectedX, const double *const expectedY,
                                   const double *const measuredX, const double *const measuredY,
                                   const double ha, const double dec, const short fitType, const double plateTheta,
                                   drama::sds::Id *paramId)
{
    double coeffs[6];
    SetCoeffs(expectedX, expectedY, measuredX, measuredY, fitType, coeffs, plateTheta, paramId);

    ConstructReturnValue(expectedX, expectedY, measuredX, measuredY, coeffs, ha, dec, paramId);
}

void CSurveyAction::ActionThread(const drama::sds::Id &Arg)
{
    auto ThisTask(GetTask()->TaskPtrAs<TdFGripperTask>());
    ThisTask->ClearError();
    details = ThisTask->tdFgripperGetMainStruct();
    if (details == nullptr)
    {
        DramaTHROW(TDFGRIPPERTASK__NOTINIT, "C_SURVEY: the structure pointer is null, please initialise the task!");
    }
    if (details->inUse)
    {
        DramaTHROW(TDFGRIPPERTASK__IN_USE, "C_SURVEY: TdFGripperTask is running other actions.");
    }
    if (!Arg)
    {
        DramaTHROW(TDFGRIPPERTASK__NO_ARGUMENTS, "C_SURVEY: No input argument is provided.");
    }
    m_tdFgripperSStruct = std::make_shared<tdFgrip_Stype>();
    drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
    drama::gitarg::String AreaArg(this, Arg, "area", 1, "", NoFlags);
    std::string strArea = AreaArg;
    if (strArea != "ALL" && strArea != "COEFFS" && strArea != "TEMP" && strArea != "FLEX")
    {
        DramaTHROW(TDFGRIPPERTASK__INV_INPUT_ARGUMENT, "C_SURVEY: TdFGripperTask can only take \"ALL\", \"COEFFS\", \"TEMP\",\"FLEX\" type.");
    }
    if (strArea == "ALL")
        m_tdFgripperSStruct->area = _ALL;
    else if (strArea == "FLEX")
        m_tdFgripperSStruct->area = _FLEX;
    else if (strArea == "TEMP")
        m_tdFgripperSStruct->area = _TEMP;
    else
        m_tdFgripperSStruct->area = _COEFFS;

    drama::gitarg::Int<3, NUM_FIDUCIALS> NumArg(this, Arg, "numMarks", 2, 3, NoFlags);
    int iNum = NumArg;
    m_tdFgripperSStruct->numMarks = iNum;

    drama::gitarg::String XStr(this, Arg, "x", 3, "", NoFlags);
    std::vector<std::string> XCoordinates = SplitString(XStr);
    if (iNum != (int)XCoordinates.size())
    {
        DramaTHROW_S(TDFGRIPPERTASK__INV_INPUT_ARGUMENT, "C_SURVEY: the input of X coordinates is invalid. The number of fidual is %d and the number of X coordinates is %d.\n", iNum, (int)(XCoordinates.size()));
    }
    for (int index = 0; index < iNum; index++)
    {
        m_tdFgripperSStruct->x[index] = stol(XCoordinates[index]);
        DEBUG("%ld ", m_tdFgripperSStruct->x[index]);
    }
    DEBUG("\n");

    drama::gitarg::String YStr(this, Arg, "y", 4, "", NoFlags);
    std::vector<std::string> YCoordinates = SplitString(YStr);
    if (iNum != (int)YCoordinates.size())
    {
        DramaTHROW_S(TDFGRIPPERTASK__INV_INPUT_ARGUMENT, "C_SURVEY: the input of Y coordinates is invalid. The number of fidual is %d and the number of Y coordinates is %d.\n", iNum, (int)(YCoordinates.size()));
    }
    for (int index = 0; index < iNum; index++)
    {
        m_tdFgripperSStruct->y[index] = stol(YCoordinates[index]);
        DEBUG("%ld ", m_tdFgripperSStruct->y[index]);
    }
    DEBUG("\n");

    drama::gitarg::Real<-30, 40> TempArg(this, Arg, "temp", 5, 0.0, NoFlags);
    double temp = TempArg;
    m_tdFgripperSStruct->temp = temp;

    double expectedX[NUM_FIDUCIALS], expectedY[NUM_FIDUCIALS];
    double measuredX[NUM_FIDUCIALS], measuredY[NUM_FIDUCIALS];
    long int offsetX, offsetY;

    short curFid, atFid, recordedFid, fitType;
    int nFailures;
    drama::ParSys parSysId(ThisTask->TaskPtr());

    double ha, dec;
    double plateTheta;

    if (m_tdFgripperSStruct->reset == YES)
    {
        if (m_tdFgripperSStruct->area != _COEFFS)
        {
            DramaTHROW(TDFGRIPPERTASK__INV_INPUT_ARGUMENT, "C_SURVEY: The survey code currently only supports the COEFFS flag");
        }
        MessageUser("C_SURVEY: Starting Gripper Survey of %d marks", m_tdFgripperSStruct->numMarks);
        fitType = 6;
        nFailures = 0;
        curFid = 0;
        atFid = recordedFid = NO;
        offsetX = offsetY = 0;
        parSysId.Get("HA", &ha);
        parSysId.Get("DEC", &dec);
        parSysId.Get("PLATE_THETA", &plateTheta);
        parSysId.Put("SURVEY_PROG", 0.0);
        m_tdFgripperSStruct->reset = NO;
    }
    bool flag = true;

    while (!atFid || !recordedFid)
    {
        if (!atFid)
        {
            bool checkFound = SearchForFid(offsetX, offsetY, curFid, &atFid);
            if (!checkFound)
            {
                flag = false;
                break;
            }
        }
        else if (!recordedFid)
        {
            if (m_tdFgripperSStruct->found != YES)
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
                                curFid, m_tdFgripperSStruct->x[curFid], m_tdFgripperSStruct->y[curFid]);
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
        ActionComplete(expectedX, expectedY, measuredX, measuredY, ha, dec, fitType, plateTheta, &newArg);
    }
    SetReturnArg(&newArg);
    ThisTask->tdFgripperSetMainStruct(details);
    MessageUser("C_SURVEY: - Action complete.");
}

int main()
{
    drama::CreateRunDramaTask<TdFGripperTask>("TdFGripperTask");
    return 0;
}