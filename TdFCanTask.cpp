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

class InitialiseAction : public drama::MessageHandler
{
public:
   InitialiseAction() {}
   ~InitialiseAction() {}

private:
   drama::Request MessageReceived() override;
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

// G_HOME_NT is a non-threaded version of G_HOME
class GHomeActionNT : public drama::MessageHandler
{
public:
   GHomeActionNT() {}
   ~GHomeActionNT() {}

private:
   drama::Request MessageReceived() override;
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

//  G_MOVE_AXIS_NT is a non-threaded version of G_MOVE_AXIS, purely for testing purposes.

class GMoveAxisActionNT : public drama::MessageHandler
{
public:
   GMoveAxisActionNT() {}
   ~GMoveAxisActionNT() {}

private:
   drama::Request MessageReceived() override;
};

//  G_INIT_NT is a non-threaded version of G_INIT, purely for testing purposes.

class GInitActionNT : public drama::MessageHandler
{
public:
   GInitActionNT() {}
   ~GInitActionNT() {}

private:
   drama::Request MessageReceived() override;
};

// G_Park_Gantry_NT is a newly-added action to park the gantry
class GParkGantryActionNT : public drama::MessageHandler
{
public:
   GParkGantryActionNT() {}
   ~GParkGantryActionNT() {}

private:
   drama::Request MessageReceived() override;
};

// G_UNPARK_NT is a non-thread version of UnParkGantry action
class GUnParkActionNT : public drama::MessageHandler
{
public:
   GUnParkActionNT() {}
   ~GUnParkActionNT() {}

private:
   drama::Request MessageReceived() override;
};

// G_MOVEOFFSET_NT is a non-thread version of MoveOffset action
class GMoveOffsetActionNT : public drama::MessageHandler
{
public:
   GMoveOffsetActionNT() {}
   ~GMoveOffsetActionNT() {}

private:
   drama::Request MessageReceived() override;
};

// G_EXIT is a non-thread version of Exit action
class GEXITActionNT : public drama::MessageHandler
{
public:
   GEXITActionNT() {}
   ~GEXITActionNT() {}

private:
   drama::Request MessageReceived() override;
};

// G_MOVEActionNT is a non-thread version of Move action, it moves a particular axis
class GMOVEActionNT : public drama::MessageHandler
{
public:
   GMOVEActionNT() {}
   ~GMOVEActionNT() {}

private:
   drama::Request MessageReceived() override;
};

// G_RESETActionNT is a non-thread version of Move action, it moves a particular axis
class GRESETActionNT : public drama::MessageHandler
{
public:
   GRESETActionNT() {}
   ~GRESETActionNT() {}

private:
   drama::Request MessageReceived() override;
};

// P_TELPOSActionNT is a non-thread version of TELPOS action, it sets the current position of telescope
class PTELPOSActionNT : public drama::MessageHandler
{
public:
   PTELPOSActionNT() {}
   ~PTELPOSActionNT() {}

private:
   drama::Request MessageReceived() override;
};

// P_SetActionNT is a non-thread version of SET action, it sets a signal parameter of tdffpi task
class PSetActionNT : public drama::MessageHandler
{
public:
   PSetActionNT() {}
   ~PSetActionNT() {}

private:
   drama::Request MessageReceived() override;
};

// P_ResetLockActionNT is a non-thread version of RESET_LOCK action, it resets the internal lock
class PResetLockActionNT : public drama::MessageHandler
{
public:
   PResetLockActionNT() {}
   ~PResetLockActionNT() {}

private:
   drama::Request MessageReceived() override;
};

// P_SetCoffsActionNT is a non-thread version of SET_COFFS action, it resets the image cofficients
class PSetCoffsActionNT : public drama::MessageHandler
{
public:
   PSetCoffsActionNT() {}
   ~PSetCoffsActionNT() {}

private:
   drama::Request MessageReceived() override;
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

private:
   //  Set up the homing configuration for a specified amplifier.
   bool SetupHomeConfig(CML::HomeConfig HomeConfigs[], int Index, AmpId Amp);
   //  Return a pointer to the amplifier corresponding to a specified axis/amp.
   CML::Amp *GetAmp(AmpId AxisId);
   //  Perform standard setup for a linkage, given a list of amplifiers to link.
   bool SetupLinkage(CML::Linkage &TheLinkage, unsigned int NumberAmps,
                     CML::Amp *LinkedAmps[], double Limits[] = NULL);

   bool tdFfpiReadFile(drama::sds::Id &defId);
   bool tdFfpiWriteFile(drama::sds::Id &defId);

   //  The action handler for the INITIALISE action.
   InitialiseAction I_InitialiseActionObj;
   //  The action handler for the G_INIT action.
   GInitAction I_GInitActionObj;
   //  A non-threaded version of G_INIT, called G_INIT_NT
   GInitActionNT I_GInitActionNTObj;
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
   // The new action handler for G_EXIT action.
   GEXITActionNT I_GExitActionNTObj;
   // The new action handler for G_MOVE_NT action.
   GMOVEActionNT I_GMoveActionNTObj;
   // The new action handler for G_RESET action.
   GRESETActionNT I_GResetActionNTObj;

   // The new action handler for TELPOS action;
   PTELPOSActionNT I_PTelposActionNTObj;
   // The new action handler for Set action;
   PSetActionNT I_PSetActionNTObj;
   // The new action handler for ResetLock action
   PResetLockActionNT I_PResetLockNTObj;
   // The newe action handler for SetCoffs action
   PSetCoffsActionNT I_PSetCoffsActionNTObj;

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
   // drama::ParSys I_TdFCanTaskParSys;

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
   drama::Parameter<INT32> copleyXPosLim;
   drama::Parameter<INT32> copleyXNegLim;
   drama::Parameter<INT32> copleyYPosLim;
   drama::Parameter<INT32> copleyYNegLim;

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
                                                      I_GInitActionObj(TaskPtr()),
                                                      I_GMoveAxisActionObj(TaskPtr()),
                                                      I_GHomeActionObj(TaskPtr()),
                                                      I_CanAccessInitialised(false),
                                                      // I_TdFCanTaskParSys(TaskPtr()),
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
                                                      copleyXPosLim(TaskPtr(), "Copley_LIM_X_POS", 528000),
                                                      copleyXNegLim(TaskPtr(), "Copley_LIM_X_NEG", -16300),
                                                      copleyYPosLim(TaskPtr(), "Copley_LIM_Y_POS", 537400),
                                                      copleyYNegLim(TaskPtr(), "Copley_LIM_Y_NEG", -3400),
                                                      shortZeroX(TaskPtr(), "PLT1_CFID_OFF_X", 0),
                                                      shortZeroY(TaskPtr(), "PLT1_CFID_OFF_Y", 0),
                                                      I_X1Amp(NULL),
                                                      I_X2Amp(NULL),
                                                      I_YAmp(NULL),
                                                      I_ZAmp(NULL),
                                                      I_ThetaAmp(NULL),
                                                      I_JawAmp(NULL),
                                                      I_tdFfpiMainStruct(nullptr)
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
   Add("G_EXIT", drama::MessageHandlerPtr(&I_GMoveOffsetActionNTObj, drama::nodel()));
   // add new Move Axis action, which only moves a particular axis
   Add("G_MOVE_NT", drama::MessageHandlerPtr(&I_GMoveActionNTObj, drama::nodel()));
   // add new Reset action, which disables the amplifiers and setup them again.
   Add("G_RESET", drama::MessageHandlerPtr(&I_GResetActionNTObj, drama::nodel()));

   // add new TOLPOS action which updates the current recorded telescope orientation
   Add("P_TELPOS", drama::MessageHandlerPtr(&I_PTelposActionNTObj, drama::nodel()));
   // add new Set action which set the FPI task parameters;
   Add("P_SET", drama::MessageHandlerPtr(&I_PSetActionNTObj, drama::nodel()));
   // add new Reset Lock action which reset the lock;
   Add("P_RESETLOCK", drama::MessageHandlerPtr(&I_PResetLockNTObj, drama::nodel()));
   //
   Add("P_SETCOFFS", drama::MessageHandlerPtr(&I_PSetCoffsActionNTObj, drama::nodel()));
   Add("EXIT", &drama::SimpleExitAction);
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
   string fName;
   if (savingFiles & DEFS_FILE)
   {
      tdFfpiTaskType *details = tdFfpiGetMainStruct();
      drama::sds::Id defId;
      if(tdFfpiWriteFile(defId))
      {
         if(defId)
         {
            fName = pathName + "tdFfpiDefs.sds";
            defId.Write(fName);
         }else{
            I_ErrorString += "The created sds structure is invalid.\n";
            return false;
         }
      }else{
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
      return false;
   }
   if (loadingFiles & DEFS_FILE)
   {
      fName = pathName + "tdFfpiDefs.sds";
      if (!boost::filesystem::exists(fName) || !boost::filesystem::is_regular_file(fName))
      {
         I_ErrorString += "tdFfpiDefs.sds doesn't exist.\n";
         DramaTHROW(TDFCANTASK__NO_ENV_VAR, "tdFfpiDefs.sds doesn't exist.");
         return false;
      }
      // This is the drama version
      // SdsRead(fName.c_str(), &defId, status);
      drama::sds::Id defId(drama::sds::Id::FromFile(fName));
      bool readRes = tdFfpiReadFile(defId);
      if (!readRes)
      {
         I_ErrorString += "Failed to read tdFfpiDefs.sds\n";
         DEBUG("READ tdFfpiDefs.sds: %s", I_ErrorString.c_str());
         DramaTHROW(TDFCANTASK__READ_ERROR, "Failed to read tdFfpiDefs.sds.");
         return false;
      }
      if (check & SHOW)
      {
         DEBUG("READ tdFfpiDefs.sds: %s", fName.c_str());
      }
      return true;
   }
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

   if(details == nullptr)
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
   tmpId.Put("XY_VEL", lIntParam);

   lIntParam = (long int)stepSize;
   tmpId.Put("STEP_SIZE", lIntParam);

   lIntParam = (long int)maxError;
   tmpId.Put("MAX_ERROR", lIntParam);

   lIntParam = (long int)posTol;
   tmpId.Put("POS_TOL", lIntParam);

   lIntParam = (long int)attempts;
   tmpId.Put("POS_ATTEMPTS", lIntParam);

   lIntParam = (long int)fibInImgThres;
   tmpId.Put("FIBRE_IN_IMAGE", lIntParam);

   dParam = (double)settleTime;
   tmpId.Put("SETTLE_TIME", dParam);

   lIntParam = (long int)timeoutFac;
   tmpId.Put("TIMEOUT_FAC", lIntParam);

   lIntParam = (long int)xaccuracy;
   tmpId.Put("X_ACCURACY", lIntParam);

   lIntParam = (long int)yaccuracy;
   tmpId.Put("Y_ACCURACY", lIntParam);

   if(details->dprFeedback)
   {
      tmpId.Put("DPR_FEEDBACK", "YES");
   }
   else
   {
      tmpId.Put("DPR_FEEDBACK", "NO");
   }

   lIntParam = (long int)overlayPlaneEnabled;
   tmpId.Put("VFG_OP_ENABLE", lIntParam);

   dParam = (double)zerocamCenWait;
   tmpId.Put("ZEROCAM_CENWAIT", dParam);

   lIntParam = (long int)copleyXPosLim;
   tmpId.Put("COPLEY_LIM_X_POS", lIntParam);

   lIntParam = (long int)copleyXNegLim;
   tmpId.Put("COPLEY_LIM_X_NEG", lIntParam);

   lIntParam = (long int)copleyYPosLim;
   tmpId.Put("COPLEY_LIM_Y_POS", lIntParam);

   lIntParam = (long int)copleyYNegLim;
   tmpId.Put("COPLEY_LIM_Y_NEG", lIntParam);

   lIntParam = (long int)shortZeroX;
   tmpId.Put("PLT1_CFID_OFF_X", lIntParam);

   lIntParam = (long int)shortZeroY;
   tmpId.Put("PLT1_CFID_OFF_Y", lIntParam);
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
      I_ErrorString += "Malloc error.\n";
      DramaTHROW(TDFCANTASK__MALLOCERR, "Malloc error.");
      // return false;
   }

   /*
    *  Set camera constant parameters.
    */
   details->freeImg.shutter = GCAM_OPEN;
   details->freeImg.updateTime = 0.0;
   details->freeImg.exposureTime = 0.1;
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
      details->normWin.xCen = dParam;

      // ArgGetd(tmpId, "yCen", &dParam, status);
      tmpId.Get("yCen", &dParam);
      details->normWin.yCen = dParam;

      // ArgGets(tmpId, "xSpan", &sParam, status);
      tmpId.Get("xSpan", &sParam);
      details->normWin.xSpan = (int)sParam;

      // ArgGets(tmpId, "ySpan", &sParam, status);
      tmpId.Get("ySpan", &sParam);
      details->normWin.ySpan = (int)sParam;
      // SdsFreeId(tmpId, status);
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
            return false;
         }
         // SdsFreeId(tmpId, status);
         // SdsFreeId(tmp2Id, status);
      }
      else
      {
         I_ErrorString += "Cannot find the field of coeffs.\n";
         DramaTHROW(TDFCANTASK__READ_ERROR, "Cannot find the field of coeffs.");
         // return false;
      }
   }
   else
   {
      I_ErrorString += "Cannot find the field of conversion.\n";
      DramaTHROW(TDFCANTASK__READ_ERROR, "Cannot find the field of conversion.");
      // return false;
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
         xyVel = lIntParam;
         DEBUG("xyVel is %d\n", xyVel);
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
         stepSize = lIntParam;
         DEBUG("stepSize is %d\n", stepSize);
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
         DEBUG("maxError is %d\n", maxError);
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
         DEBUG("posTol is %d\n", posTol);
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
         DEBUG("attempts is %hd\n", attempts);
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
         DEBUG("fibInImgThres is %d\n", fibInImgThres);
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
         DEBUG("settleTime is %f\n", settleTime);
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
         DEBUG("xaccuracy is %d\n", xaccuracy);
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
         DEBUG("yaccuracy is %d\n", yaccuracy);
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
         DEBUG("timeoutFac is %d\n", timeoutFac);
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
         details->dprFeedback = strcmp("NO", strParam.c_str()) ? YES : NO;
         DEBUG("details->dprFeedback is %hd\n", details->dprFeedback);
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
         DEBUG("overlayPlaneEnabled is %d\n", overlayPlaneEnabled);
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
         DEBUG("zerocamCenWait is %f\n", zerocamCenWait);
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
         copleyXPosLim = lIntParam;
         DEBUG("copleyXPosLim is %d\n", copleyXPosLim);
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
         copleyXNegLim = lIntParam;
         DEBUG("copleyXNegLim is %d\n", copleyXNegLim);
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
         copleyYPosLim = lIntParam;
         DEBUG("copleyYPosLim is %d\n", copleyYPosLim);
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
         copleyYNegLim = lIntParam;
         DEBUG("copleyYNegLim is %d\n", copleyYNegLim);
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
         DEBUG("shortZeroX is %hd\n", shortZeroX);
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
         DEBUG("shortZeroY is %hd\n", shortZeroY);
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

      DEBUG("Setting amp addrs\n");

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
         DEBUG("I_tdFfpiMainStruct fail to initialise\n");
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

bool TdFCanTask::DisableAmps()
{
   bool DisableAllAmps = false;
   const CML::Error *Err = NULL;

   if (!I_CanAccessInitialised)
   {
      DEBUG("CanAccess is not inlitialised!\n");
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
      }
      if (HomeY)
         HomeFlags[Y_AMP] = true;

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
         ReturnOK = true;
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
      DEBUG("Setting up the mode %s\n", ParameterValue.c_str());
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
            DEBUG("Setting up the X %d\n", xPark);
         }
         else
         {
            yPark = PosVal;
            DEBUG("Setting up the Y %d\n", yPark);
         }
      }
   }
   else if (strcmp(ParameterName.c_str(), "XY_VEL") == 0)
   {
      int VelVal = stoi(ParameterValue);
      xyVel = VelVal;
      DEBUG("Setting up the velocity %d\n", xyVel);
   }
   else if ((strcmp(ParameterName.c_str(), "X_ACCURACY") == 0) ||
            (strcmp(ParameterName.c_str(), "Y_ACCURACY") == 0))
   {
      int accuracy = stoi(ParameterValue);
      if (strcmp(ParameterName.c_str(), "X_ACCURACY") == 0)
      {
         xaccuracy = accuracy;
         DEBUG("Setting up the X_ACCURACY %d\n", xaccuracy);
      }
      else
      {
         yaccuracy = accuracy;
         DEBUG("Setting up the Y_ACCURACY %d\n", yaccuracy);
      }
   }
   else if (strcmp(ParameterName.c_str(), "PLATE_THETA") == 0)
   {
      double ThetaVal = stod(ParameterValue);
      plateTheta = ThetaVal;
      DEBUG("Setting up the plate theta %f\n", plateTheta);
   }
   else if (strcmp(ParameterName.c_str(), "PARKED") == 0)
   {
      if (tdfCanMode != "NOT_PROTECTED")
      {
         I_ErrorString += "In Protected Mode, cannot set up the Parked Mode!";
         return false;
      }
      // validate the value, restrict the value to YES & NO;
      if (ParameterValue == "YES" || ParameterValue == "NO")
         tdfCanParked = ParameterValue;
      DEBUG("Setting up the PARKED Mode %s\n", ParameterValue.c_str());
   }
   else if ((strcmp(ParameterName.c_str(), "HA") == 0) ||
            (strcmp(ParameterName.c_str(), "DEC") == 0))
   {
      double dVal = stod(ParameterValue);
      if (strcmp(ParameterName.c_str(), "HA") == 0)
         dzeroHA = dVal;
      else
      {
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
         stepSize = iVal;
      else if (strcmp(ParameterName.c_str(), "MAX_ERROR") == 0)
         maxError = iVal;
      else if (strcmp(ParameterName.c_str(), "FIBRE_IN_IMAGE") == 0)
         fibInImgThres = iVal;
      else if (strcmp(ParameterName.c_str(), "POS_TOL") == 0)
         posTol = iVal;
      else if (strcmp(ParameterName.c_str(), "VFG_OP_ENABLE") == 0)
         timeoutFac = iVal;
      else if (strcmp(ParameterName.c_str(), "VFG_OP_ENABLE") == 0)
         overlayPlaneEnabled = iVal;

      DEBUG("Setting up the %s to be %d\n", ParameterName.c_str(), iVal);
   }
   else if ((strcmp(ParameterName.c_str(), "PLT1_CFID_OFF_X") == 0) ||
            (strcmp(ParameterName.c_str(), "PLT1_CFID_OFF_Y") == 0))
   {
      int iVal = stoi(ParameterValue);
      if (strcmp(ParameterName.c_str(), "PLT1_CFID_OFF_X") == 0)
         shortZeroX = iVal;
      else
         shortZeroY = iVal;
      DEBUG("Setting up the %s to be %d\n", ParameterName.c_str(), iVal);
   }
   else if (strcmp(ParameterName.c_str(), "POS_ATTEMPTS") == 0)
   {
      int iVal = stoi(ParameterValue);
      attempts = iVal;
      DEBUG("Setting up the %s to be %d\n", ParameterName.c_str(), iVal);
   }
   else if (strcmp(ParameterName.c_str(), "SETTLE_TIME") == 0)
   {
      double dVal = stod(ParameterValue);
      settleTime = dVal;
      DEBUG("Setting up the %s to be %f\n", ParameterName.c_str(), dVal);
   }
   else if (strcmp(ParameterName.c_str(), "ZEROCAM_CENWAIT") == 0)
   {
      double dVal = stod(ParameterValue);
      zerocamCenWait = dVal;
      DEBUG("Setting up the %s to be %f\n", ParameterName.c_str(), dVal);
   }
   else if (strcmp(ParameterName.c_str(), "GANTRY_LAMPS") == 0)
   {
      tdfGantryLamp = ParameterValue;
      DEBUG("Setting up the %s to be %s\n", ParameterName.c_str(), ParameterValue);
   }
   else if ((strcmp(ParameterName.c_str(), "Copley_LIM_X_POS") == 0) ||
            (strcmp(ParameterName.c_str(), "Copley_LIM_X_NEG") == 0) ||
            (strcmp(ParameterName.c_str(), "Copley_LIM_Y_POS") == 0) ||
            (strcmp(ParameterName.c_str(), "Copley_LIM_Y_NEG") == 0))
   {
      int iVal = stoi(ParameterValue);
      if (strcmp(ParameterName.c_str(), "Copley_LIM_X_POS") == 0)
         copleyXPosLim = iVal;
      else if (strcmp(ParameterName.c_str(), "Copley_LIM_X_NEG") == 0)
         copleyXNegLim = iVal;
      else if (strcmp(ParameterName.c_str(), "Copley_LIM_Y_POS") == 0)
         copleyYPosLim = iVal;
      else if (strcmp(ParameterName.c_str(), "Copley_LIM_Y_NEG") == 0)
         copleyYNegLim = iVal;

      DEBUG("Setting up the %s to be %d\n", ParameterName.c_str(), iVal);
   }
   else if (strcmp(ParameterName.c_str(), "DEBUG_CENTROID") == 0)
   {
      int iVal = stoi(ParameterValue);
      ushortZeroCentroid = iVal;
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
         Targets[Index] = TargetPosition;
         TargetPoint[Index] = TargetPosition;
         DEBUG("Target for axis %d is %f\n", Index, TargetPosition);

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
drama::Request InitialiseAction::MessageReceived()
{

   UnblockSIGUSR2();

   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   // drama::ParSys parSys(GetTask());
   drama::sds::Id parSysId(drama::sds::Id::CreateFromSdsIdType((long)(DitsGetParId())));

   ThisTask->ClearError();
   if (!(ThisTask->InitialisefpiMainStruct()))
   {
      DEBUG("Fail to allocate memory to fpiMainStruct\n");
      return drama::RequestCode::End;
   }
   else
   {
      DEBUG("Succeed to allocate memory to fpiMainStruct\n");

      parSysId.Put("ENQ_VER_NUM", TdFCanTaskVersion);
      parSysId.Put("ENQ_VER_DATE", TdFCanTaskDate);

      drama::sds::Id id;        /* Parameter id                    */
      unsigned long int length; /* Length of parameter item        */
      short check;
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
         return drama::RequestCode::End;
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
         return drama::RequestCode::End;
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
         return drama::RequestCode::End;
      }
      length = 0;
      // SdsFreeId(id, &status);

      details->inUse = YES;

      bool defRead = ThisTask->tdFfpiDefRead(DEFS_FILE | FLEX_FILE,
                                             check);
      if (defRead == false)
      {
         details->inUse = NO;
         return drama::RequestCode::End;
      }

      details->ipsMode = 0;
      details->dprAddress = ERROR;
      details->toEnc.x = 0;
      details->toEnc.y = 0;
   }

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

   return drama::RequestCode::End;
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

   std::string Axes("");
   if (Arg)
   {
      drama::gitarg::String AxesArg(Arg, "AXES", 1);
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
      auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
      ThisTask->ClearError();
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

drama::Request GInitActionNT::MessageReceived()
{

   UnblockSIGUSR2();

   drama::sds::Id Arg = GetEntry().Argument();

   std::string Axes("");
   if (Arg)
   {
      drama::gitarg::String AxesArg(Arg, "AXES", 1);
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
      auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
      ThisTask->ClearError();
      if (!(ThisTask->SetupAmps()))
      {
         MessageUser("G_INIT_NT: " + ThisTask->GetError());
      }
      else
      {
         if (!(ThisTask->HomeAxes(UseX, UseY, UseZ, UseTheta, UseJaw)))
         {
            MessageUser("G_INIT_NT: " + ThisTask->GetError());
         }
         else
         {
            MessageUser("G_INIT_NT: Axes homed");
         }
      }
   }
   else
   {
      MessageUser("G_INIT_NT: Invalid axis specification");
   }
   // We never re-enable the blocking of SIGUSR2 - it causes too many problems.
   // BlockSIGUSR2();

   return drama::RequestCode::End;
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
         drama::gitarg::String AxesArg(Arg, "AXES", 1, "", NoFlags);
         Axes = AxesArg;
         drama::gitarg::String PositionsArg(Arg, "POSITIONS", 2, "", NoFlags);
         Positions = PositionsArg;
         drama::gitarg::String VelocitiesArg(Arg, "VELOCITIES", 3, "", NoFlags);
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
      auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
      ThisTask->ClearError();
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

drama::Request GMoveAxisActionNT::MessageReceived()
{

   UnblockSIGUSR2();

   drama::sds::Id Arg = GetEntry().Argument();

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
         drama::gitarg::String AxesArg(Arg, "AXES", 1, "", NoFlags);
         Axes = AxesArg;
         drama::gitarg::String PositionsArg(Arg, "POSITIONS", 2, "", NoFlags);
         Positions = PositionsArg;
         drama::gitarg::String VelocitiesArg(Arg, "VELOCITIES", 3, "", NoFlags);
         Velocities = VelocitiesArg;
         drama::gitarg::String OffsetFlagArg(Arg, "OFFSET", 4, "", NoFlags);
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
      MessageUser("MOVE_AXES: " + Error);
   }
   else
   {
      unsigned int NumberAxes = AxisDemands.size();
      for (unsigned int Index = 0; Index < NumberAxes; Index++)
      {
         if (MoveBackward == true)
            AxisDemands[Index].Position *= (-1.0);
         DEBUG("AxisId: %d, position %f, velocity %f\n", AxisDemands[Index].AxisId,
               AxisDemands[Index].Position, AxisDemands[Index].Velocity);
      }
      auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
      ThisTask->ClearError();
      if (!(ThisTask->SetupAmps()))
      {
         MessageUser("MOVE_AXES: " + ThisTask->GetError());
      }
      else
      {
         if (!(ThisTask->MoveAxes(AxisDemands, MoveOffset)))
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

   return drama::RequestCode::End;
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

   std::string Axes("");
   if (Arg)
   {
      drama::gitarg::String AxesArg(Arg, "AXES", 1);
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
      auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
      ThisTask->ClearError();
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

drama::Request GParkGantryActionNT::MessageReceived()
{

   UnblockSIGUSR2();
   std::string Axes("");
   std::string Positions;
   std::string Velocities;
   drama::sds::Id Arg = GetEntry().Argument();
   if (Arg)
   {
      drama::gitarg::String AxesArg(Arg, "AXES", 1);
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
      auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
      ThisTask->ClearError();
      if (!(ThisTask->SetupAmps()))
      {
         MessageUser("G_PARK_NT: " + ThisTask->GetError());
      }
      else
      {
         std::string Error;
         std::vector<AxisDemand> AxisDemands = GetDemands(Axes, Positions, Velocities, Error);
         if (!(ThisTask->MoveAxes(AxisDemands)))
         {
            MessageUser("G_PARK_NT: " + ThisTask->GetError());
         }
         else
         {
            MessageUser("G_PARK_NT: Axes homed");
         }
      }
   }
   else
   {
      MessageUser("G_PARK_NT: Invalid axis specification");
   }

   return drama::RequestCode::End;
}

//  ------------------------------------------------------------------------------------------------

//         G  Home   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
// Currently the G_HOME Action doesn't work properly. It will throw out an error when the G_HOME is invoked, indicating
// there should have been an interlock check. It is very strange that G_HOME function uses the same code as G_INIT_NT.
// G_HOME fails to process the Home action whilst G_INIT_NT functions properly. The error may result from the thread version drama?
// Need to check if the non-threaded version works.

drama::Request GHomeActionNT::MessageReceived()
{

   UnblockSIGUSR2();
   std::string Axes("");
   drama::sds::Id Arg = GetEntry().Argument();
   if (Arg)
   {
      drama::gitarg::String AxesArg(Arg, "AXES", 1);
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
      auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
      ThisTask->ClearError();
      if (!(ThisTask->SetupAmps()))
      {
         MessageUser("G_HOME_NT: " + ThisTask->GetError());
      }
      else
      {
         if (!(ThisTask->HomeAxes(X, Y, Z, Theta, Jaw)))
         {
            MessageUser("G_HOME_NT: " + ThisTask->GetError());
         }
         else
         {
            MessageUser("G_HOME_NT: Axes homed");
         }
      }
   }
   else
   {
      MessageUser("G_HOME_NT: Invalid axis specification");
   }
   return drama::RequestCode::End;
}

//  ------------------------------------------------------------------------------------------------

//         G  UnPark   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
drama::Request GUnParkActionNT::MessageReceived()
{
   UnblockSIGUSR2();
   std::string Axes("");
   std::string Positions;
   std::string Velocities;

   drama::sds::Id Arg = GetEntry().Argument();
   if (Arg)
   {
      drama::gitarg::String AxesArg(Arg, "AXES", 1);
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
      auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
      ThisTask->ClearError();
      if (!(ThisTask->SetupAmps()))
      {
         MessageUser("G_UNPARK_NT: " + ThisTask->GetError());
      }
      else
      {
         std::string Error;
         std::vector<AxisDemand> AxisDemands = GetDemands(Axes, Positions, Velocities, Error);
         if (!(ThisTask->MoveAxes(AxisDemands)))
         {
            MessageUser("G_UNPARK_NT: " + ThisTask->GetError());
         }
         else
         {
            MessageUser("G_UNPARK_NT: Axes unparked");
         }
      }
   }
   else
   {
      MessageUser("G_HOME_NT: Invalid axis specification");
   }
   return drama::RequestCode::End;
}

//  ------------------------------------------------------------------------------------------------

//         G  MoveOffset   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
// The G_MOVEOFFSET Action moves an offset amount from its current position. This is a non-thread version.

drama::Request GMoveOffsetActionNT::MessageReceived()
{
   UnblockSIGUSR2();

   drama::sds::Id Arg = GetEntry().Argument();

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
         drama::gitarg::String AxesArg(Arg, "AXES", 1, "", NoFlags);
         Axes = AxesArg;
         drama::gitarg::String PositionsArg(Arg, "POSITIONS", 2, "", NoFlags);
         Positions = PositionsArg;
         drama::gitarg::String VelocitiesArg(Arg, "VELOCITIES", 3, "", NoFlags);
         Velocities = VelocitiesArg;
         drama::gitarg::String OffsetFlagArg(Arg, "OFFSET", 4, "", NoFlags);
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
      unsigned int NumberAxes = AxisDemands.size();
      for (unsigned int Index = 0; Index < NumberAxes; Index++)
      {
         if (MoveBackward == true)
            AxisDemands[Index].Position *= (-1.0);
         DEBUG("AxisId: %d, offset position %f, velocity %f\n", AxisDemands[Index].AxisId,
               AxisDemands[Index].Position, AxisDemands[Index].Velocity);
      }
      auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
      ThisTask->ClearError();
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
            MessageUser("G_MOVEOFFSET_NT: Move complete");
         }
      }
   }

   return drama::RequestCode::End;
}

//  ------------------------------------------------------------------------------------------------

//         G  Exit   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
drama::Request GEXITActionNT::MessageReceived()
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();
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
   return drama::RequestCode::End;
}

//  ------------------------------------------------------------------------------------------------

//         G  Move   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
drama::Request GMOVEActionNT::MessageReceived()
{
   UnblockSIGUSR2();
   drama::sds::Id Arg = GetEntry().Argument();

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
         drama::gitarg::String AxesArg(Arg, "AXES", 1, "", NoFlags);
         Axes = AxesArg;
         drama::gitarg::String PositionsArg(Arg, "POSITIONS", 2, "", NoFlags);
         Positions = PositionsArg;
         drama::gitarg::String VelocitiesArg(Arg, "VELOCITIES", 3, "", NoFlags);
         Velocities = VelocitiesArg;
         drama::gitarg::String OffsetFlagArg(Arg, "OFFSET", 4, "", NoFlags);
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

   std::string Error;
   std::vector<AxisDemand> AxisDemands = GetDemands(Axes, Positions, Velocities, Error);
   int NumberAxes = AxisDemands.size();
   if (NumberAxes <= 1)
   {
      MessageUser("G_MOVE_NT can only work on one axis. If more than one axes need to move, please choose G_MOVE_AXIS_NT." + Error);
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
      auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
      ThisTask->ClearError();
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
         }
      }
   }
   return drama::RequestCode::End;
}

//  ------------------------------------------------------------------------------------------------

//         G  Reset   A c t i o n  N T  : :  M e s s a g e  R e c e i v e d
drama::Request GRESETActionNT::MessageReceived()
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();

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

   return drama::RequestCode::End;
}

drama::Request PTELPOSActionNT::MessageReceived()
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();

   drama::ParId paramHA(GetTask(), "HA");
   drama::ParId paramDEC(GetTask(), "DEC");

   drama::sds::Id Arg = GetEntry().Argument();
   string HA = "";
   string DEC = "";
   if (Arg)
   {
      try
      {
         drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
         drama::gitarg::String HAArg(Arg, "HA", 1, "", NoFlags);
         HA = HAArg;
         drama::gitarg::String DECArg(Arg, "DEC", 2, "", NoFlags);
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
      DEBUG("\nBefore the update, the current DEC is:\n");
      paramDEC.List();

      paramHA.Put(dHA);
      paramDEC.Put(dDEC);

      DEBUG("After the update, the current HA is:\n");
      paramHA.List();
      DEBUG("\nAfter the update, the current DEC is:\n");
      paramDEC.List();
   }
   else
   {
      DEBUG("P_TELPOS please check your input arguments\n");
      MessageUser("P_TELPOS: " + ThisTask->GetError());
   }
   return drama::RequestCode::End;
}

drama::Request PSetActionNT::MessageReceived()
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();

   drama::sds::Id Arg = GetEntry().Argument();
   drama::ParSys parSys(GetTask()); // this is duplicated;
   drama::sds::Id parSysId(drama::sds::Id::CreateFromSdsIdType((long)(DitsGetParId())));

   string ParameterName = "";
   string ParameterValue = "";
   if (Arg)
   {
      try
      {
         drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
         drama::gitarg::String ParameterNameArg(Arg, "ParameterName", 1, "", NoFlags);
         ParameterName = ParameterNameArg;
         drama::gitarg::String ParameterValueArg(Arg, "ParameterValue", 2, "", NoFlags);
         ParameterValue = ParameterValueArg;
      }
      catch (exception &e)
      {
         MessageUser("P_SET: " + string(e.what()));
      }
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
   return drama::RequestCode::End;
}

drama::Request PResetLockActionNT::MessageReceived()
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();
   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   if (details != nullptr)
   {
      details->inUse = NO;
      MessageUser("P_RESETLOCK: Reset internal lock completed.\n");
   }
   else
   {
      MessageUser("P_RESETLOCK: the structure pointer is null, please initialise the task first.\n");
   }

   return drama::RequestCode::End;
}

drama::Request PSetCoffsActionNT::MessageReceived()
{
   UnblockSIGUSR2();
   auto ThisTask(GetTask()->TaskPtrAs<TdFCanTask>());
   ThisTask->ClearError();
   tdFfpiTaskType *details = ThisTask->tdFfpiGetMainStruct();
   drama::sds::Id Arg = GetEntry().Argument();
   string CoffsVal;
   string SaveFlag;
   int j = 0;
   if (details != nullptr)
   {
      if (Arg)
      {
         try
         {
            drama::gitarg::Flags NoFlags = drama::gitarg::Flags::NoFlagSet;
            drama::gitarg::String CoffsValArg(Arg, "CoffsVal", 1, "", NoFlags);
            CoffsVal = CoffsValArg;
            drama::gitarg::String SaveFlagArg(Arg, "ParameterValue", 2, "", NoFlags);
            SaveFlag = SaveFlagArg;
         }
         catch (exception &e)
         {
            MessageUser("P_SETCOFFS: " + string(e.what()));
            return drama::RequestCode::End;
         }
         std::vector<string> ParameterVec = SplitString(CoffsVal);
         if ((int)ParameterVec.size() == 6)
         {
            for (int index = 0; index < 6; index++)
            {
               details->convert.coeffs[index] = stod(ParameterVec[index]);
            }
            slaInvf(details->convert.coeffs, details->convert.invCoeffs, &j);
            if (j != 0)
            {
               for (int index = 0; index < 6; index++)
                  details->convert.invCoeffs[index] = details->convert.coeffs[index];
            }
            if (SaveFlag == "SAVE" || SaveFlag == "save")
            {
               ThisTask->tdFfpiDefWrite(1, 0);
            }
            MessageUser("P_SETCOFFS: Set Coffs completed.\n");
         }
         else
         {
            MessageUser("P_SETCOFFS: Invalid input arguments. Require 6 input arguments.\n");
         }
      }
      else
      {
         MessageUser("P_SETCOFFS: No i.nput arguments.\n");
      }
   }
   else
   {
      MessageUser("P_SETCOFFS: the structure pointer is null, please initialise the task first.\n");
   }
   return drama::RequestCode::End;
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
