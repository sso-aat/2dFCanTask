//
//                                   C a n  A c c e s s . h
//
//  Function:
//     Handles details of access to CANBus devices, in particular handling simulation.
//
//  Description:
//     A CanAccess object handles the details of access to named I/O items and amplifiers.
//     For a named item (I/O or amplifier) it can return the address of a CML::CanOpen
//     object that provides access to the CANBus/CANOpen system for that item. It can also
//     return the address of the CANBusConfigurator that knows about the details of the item.
//     For a named amplifier, it will return the address of a CML::Amp object that can be used
//     to control the amplifier. For a named I/O item, it will return the address of a
//     CML::IOModule object that can be used to access the item, together with the start and
//     number of bits for that item.
//
//     The CanAccess code sits on top of the CANNetworkConfigurator and CANBusConfigurator code,
//     and also handles details of which devices are simulated and which are not. The CML::Amp
//     and CML::IOModule items it returns have already been initialised to work with either
//     a real or simulated CANBus. This means that all details of simulation are handled at this
//     level. The IsSimulated() enquiry routine can tell the caller whether or not a named amp
//     or I/O item is being handled through simulation, but the caller should generally only
//     use this for display purposes - all items should be handled the same way by the caller,
//     regardless of their simulation state.
//
//     Note the CML restriction that all amplifiers run as a group through a CML::Linkage need
//     to be on the same CANBus and running through the same CML::CanOpen control object. This
//     is not strictly a simulation issue, although simulated objects do not use the same CanOpen
//     objects as do non-simulated objects, so this means all the amplifiers in a linkage must
//     have the same simulation status. Higher level code may have to allow for this.
//
//  Author(s): Keith Shortridge, Keith@KnaveAndVarlet.com.au.
//
//  History:
//      9th Mar 2018. Original version. KS.
//     29th Mar 2018. Added include of string.h to compile under Linux. KS.
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
//     "@(#) $Id: ACMM:2dFCanTask/CanAccess.h,v 1.5 23-Aug-2021 08:14:08+10 tjf $"

#include "CANNetworkConfigurator.h"
#include "CML.h"
#include "SimCanInterface.h"
#include <map>
#include <string>
#include <list>
#include <string.h>

//  Assume for the moment that if we're running on OS X we're running in simulation and don't
//  have access to the AnaGateCAN code for access to the real hardware. By the same token, if
//  we're not running on OS X, assume we do have that access.

#ifndef __APPLE__
#define USE_CAN_ANAGATE
#endif

//  ------------------------------------------------------------------------------------------------

//                                  S t r u c t u r e s

//  A CanAccessCANDetails structure contains details about one of the - possibly multiple -
//  CANBuses in use. (Most systems will only have one, but some instruments, like GHOST,
//  use more.) This is filled in by CanGetDetails() on the basis of what it finds in the
//  file whose name it is passed. We don't need a constructor, as all the items are strings
//  with their own default constructors.

struct CanAccessBusDetails {
   std::string Name;            // The CANBus name, eg "CAN0"
   std::string IPAddress;       // The IP address, if for an internet bus. Otherwise, empty string.
   std::string SimName;         // The name of the simulated CAN bus, often the same as Name.
   std::string SimServer;       // The simulator server, if multiple buses being used. Else empty.
   std::string IniFileName;     // The name of the .ini file with the bus configuration.
};

//  A CanAccessModuleDetails structure contains details of each of the CML::IOModule objects
//  allocated on initialisation. Each time we need to access a named I/O item, we need to know
//  which CML::IOModule to use. We will know the CANBus name, the nodeId from the configurators,
//  and the CanAccess layer will know about whether a named item is being simulated or not.
//  Given those pieces of information - CANBus name, NodeId and simulation status, we can search
//  a list of such structures to locate the CML::IOModule we need to use. There is a constructor
//  that sets sensible default values.

struct CanAccessModuleDetails {
   std::string BusName;          // The CANBus name.
   unsigned int NodeId;          // The node ID number.
   bool Simulated;               // True if this IOModule is handled through simulation.
   CML::IOModule* IOModulePtr;   // The IOModule address.
   CanAccessModuleDetails (void) {
      BusName = "";
      NodeId = 0;
      Simulated = false;
      IOModulePtr = NULL;
   }
};

//  ------------------------------------------------------------------------------------------------

//                      C a n  A c c e s s   C l a s s  D e f i n i t i o n
//
//  A CanAccess object handles the details of access to named I/O items and amplifiers. It
//  needs to be set up with a call to Initialise(), giving it the name of a file that contains
//  an overall description of the various CANBuses (and their simulated equivalents), together
//  with the names of the .ini configuration files that describe each CANBus and its nodes in
//  detail. This file also specifies which ampliifers and I/O items are being simulated and
//  which are not. After that, it creates CML::Amp and CML::IOModule objects that can be used
//  to control the various CANBus items, and CanBus Configurators that can supply more details
//  about them.

class CanAccess {
public:
   CanAccess(void);
   ~CanAccess();
   //  Initialise, given a file consting a description of the CANBus and simulation details.
   bool Initialise(const std::string& FileName);
   //  Get a CML::Linkage object that links all the named amplifiers.
   CML::Linkage* GetLinkage (int NumberAmps, const std::string AmpNameList[]);
   //  Get a CML::Amp object that can be used to access a named amplifier.
   CML::Amp* GetAmp(const std::string& AmpName);
   //  Get a CML::IOModule object that can be used to access a named I/O item.
   CML::IOModule* GetModule(const std::string& IOItemName);
   //  Get the configurator than handles a named amplifier or I/O item.
   CANBusConfigurator* GetConfigurator(const std::string& Name);
   //  Returns true if a named amplifier or I/O item is currently being simulated.
   bool IsSimulated (const std::string& Name);
   //  Get string describing latest error.
   const std::string& GetError(void) { return I_ErrorString; }
private:
   //  Release allocated items.
   void Release(void);
   //  Read and parse the file consting a description of the CANBus and simulation details.
   bool GetCanDetails (const std::string& FileName, std::list<CanAccessBusDetails>& DetailsList);
   //  A pointer to the network configurator we use to coordinate all the various CANBuses.
   CANNetworkConfigurator* I_NetworkConfigurator;
   //  A string describing the latest error.
   std::string I_ErrorString;
   //  A map giving the names of the amplifiers and pointers to the CML::Amp objects used for them.
   std::map<std::string,CML::Amp*> I_AmpMap;
   //  A list of structures holding details of all the CML::IOModules in use.
   std::list<CanAccessModuleDetails> I_ModuleDetailsList;
   //  The single CML::Linkage supported at the moment.
   CML::Linkage* I_Linkage;
   //  True if CANBus items are simulated by default.
   bool I_SimulateByDefault;
   //  A list of CANBus named items that are exceptions to the default simulation.
   std::vector<std::string> I_ExceptionList;
   
};

