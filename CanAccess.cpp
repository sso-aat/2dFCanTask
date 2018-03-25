//
//                                   C a n  A c c e s s . c p p
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
//     More details can be found in the corresponding header file, CanAccess.h. In particular,
//     note that simulated and non-simulated amplifiers cannot be combined in the same CML
//     Linkage.
//
//  Author(s): Keith Shortridge, Keith@KnaveAndVarlet.com.au.
//
//  History:
//      9th Mar 2018. Original version. KS.
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
//     "@(#) $Id: ACMM:2dFCanTask/CanAccess.cpp,v 1.1 26-Mar-2018 08:48:13+10 ks $"

#include "CanAccess.h"

//  We use TcsUtil only for ExpandFileName(). We really need to sort out these utilities and
//  where to find them.

#include "TcsUtil.h"

//  We use a try-catch block and exceptions to simplify the code in Initialise().

#include <exception>

//  We use these often enough to make it worth while using 'using' for them. Note that
//  SimCanInterface really wasn't ever meant to be part of CML - I think there's been a rather
//  careless use of namespace declarations in the SimCanOpen code, probably involving
//  CML_NAMESPACE_USE().

using CML::SimCanInterface;
using CML::CanOpen;
using CML::Error;

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

#undef DEBUG
#define DEBUG printf

//  ------------------------------------------------------------------------------------------------

//                      C a n  A c c e s s  : :  C o n s t r u c t o r
//
//  The only constructor takes no arguments.

CanAccess::CanAccess(void) {
   I_ErrorString = "";
   I_NetworkConfigurator = NULL;
   I_ExceptionList.clear();
   I_SimulateByDefault = true;
}

//  ------------------------------------------------------------------------------------------------

//                        C a n  A c c e s s  : :  D e s t r u c t o r
//
//  The destructor releases any resources used by the CanAccess object.

CanAccess::~CanAccess() {

  //  Everything that needs to be released is handled by Release().
  
  Release();
}

//  ------------------------------------------------------------------------------------------------

//                        C a n  A c c e s s  : :  R e l e a s e
//
//  An internal routine that deletes all allocated items. This is called from the destructor,
//  but also from Initialise() to tidy up in the event of problems during initialisation.

void CanAccess::Release (void) {
   
   DEBUG ("CanAccess::Release() called\n");
   
   //  The network configurator.
   
   if (I_NetworkConfigurator) delete I_NetworkConfigurator;
   I_NetworkConfigurator = NULL;
   
   //  The set of amplifiers.
   
   for (std::map<std::string,CML::Amp*>::iterator Iter = I_AmpMap.begin();
                                                    Iter != I_AmpMap.end(); Iter++) {
      if (Iter->second) delete Iter->second;
   }
   I_AmpMap.clear();
   
   //  The set of I/O modules.
   
   for (std::list<CanAccessModuleDetails>::iterator Iter = I_ModuleDetailsList.begin();
                                                    Iter != I_ModuleDetailsList.end(); Iter++) {
      if (Iter->IOModulePtr) delete Iter->IOModulePtr;
   }
   I_ModuleDetailsList.clear();
}

//  ------------------------------------------------------------------------------------------------

//                        C a n  A c c e s s  : :  I n i t i a l i s e
//
//  Initialises the CanAccess object, reading details of the CANBus configuration from a named
//  configuration file. The FileName passed can contain names of environment variables preceded
//  by '$'. eg $THE_DIR/TheConfFile.conf. It creates CML Amp and IOModule objects that can be
//  used to control all the CANBus items described by the configuration, and sets up the various
//  internal tables that will allow it to supply these when requested.

bool CanAccess::Initialise(const std::string& FileName) {

   bool ReturnOK = true;
   
   //  We need a network configurator, which will itself create the individual configurators that
   //  encapsulate the details of the various CANbus configurations (the nodes they control, and
   //  the details of how those nodes work). Each CANBus has its own configurator, and they are
   //  coordinated by the network configurator. Having created the network configurator, we
   //  tell it what it needs to know to create the individual configurators.
   
   I_NetworkConfigurator = new CANNetworkConfigurator;

   //  For each CANBus, in the general case, we need to create a CanOpen object that will work
   //  with the real CANBus and one that will work with the simulated CANBus, and we need to
   //  create the interfaces (real and simulated) they will use.
   
   int NumberOfCanBuses = 0;
   
   //  A lot of things can go wrong in this code, so the easiest thing is to put it in a try
   //  block and raise an exception when something goes wrong.
   
   try {
   
      //  GetCanDetails() not only returns the details for each CANBus, it also reads the details
      //  from the file about which items are being simulated. Once this has been called, calls
      //  to IsSimulated() will return the correct answer.
      
      list<CanAccessBusDetails> CANBusDetailsList;
      if (!GetCanDetails (FileName,CANBusDetailsList)) {
         throw std::exception();
      }

      for (list<CanAccessBusDetails>::iterator Iter = CANBusDetailsList.begin();
                                                       Iter != CANBusDetailsList.end(); Iter++) {
         AnaGateCan* AnagateInterface = NULL;
         CanOpen* CanOpenInterface = NULL;
         SimCanInterface* SimInterface = NULL;
         CanOpen* SimCanOpenInterface = NULL;
         
         //  Create and open the CanOpen interface for the real CANBus, create the interface - here
         //  we assume this will be an Anagate interface with an IP address - and open the simulator
         //  CanOpen interface and its SimCanOpen interface. Note the CanOpen is an interface between
         //  high-level software (like this) and the CANOpen/CANBus system, and the Anagate or
         //  SimCanInterface it uses is its low-level interface to the actual hardware.
         
         if (Iter->IPAddress != "") {
            AnagateInterface = new AnaGateCan(Iter->IPAddress.c_str());
            CanOpenInterface = new CanOpen;
            const Error *Err = CanOpenInterface->Open(*AnagateInterface);
            if (Err) {
               I_ErrorString = "Error opening CanOpen interface at " + Iter->IPAddress + ". ";
               I_ErrorString += Err->toString();
               throw std::exception();
            }
         }
         if (Iter->SimName != "") {
            SimInterface = new SimCanInterface(Iter->SimName.c_str(),Iter->SimServer.c_str());
            SimCanOpenInterface = new CanOpen;
            const Error *Err = SimCanOpenInterface->Open(*SimInterface);
            if (Err) {
               I_ErrorString = "Error opening simulated interface " + Iter->SimName + ". ";
               I_ErrorString += Err->toString();
               throw std::exception();
            }
         }
         
         //  Initialise the network configurator's records for this CANBus. This will create a
         //  CANBusConfigurator which acts as a repository for most of the information we need
         //  about the layout of that CANBus.
         
         if (!I_NetworkConfigurator->Initialise(Iter->Name,Iter->IniFileName)) {
            I_ErrorString = "Error opening .ini file " + Iter->IniFileName;
            throw std::exception();
         }
         
         //  Get a pointer to the Configurator from the network configurator.
         
         std::weak_ptr<CANBusConfigurator> TheConfiguratorPtr =
                                               I_NetworkConfigurator->GetConfigurator(Iter->Name);
         if (!TheConfiguratorPtr.expired()) {
         
            //  Use the configurator to tell us about the nodes we are supporting.
            
            std::shared_ptr<CANBusConfigurator> TheConfigurator(TheConfiguratorPtr);

            //  First, the amplifier nodes. Go through each amplifier node known to this
            //  configurator.
            
            unsigned int AmpNodes = TheConfigurator->GetAmpNodeCount();
            for (unsigned int AmpNode = 0; AmpNode < AmpNodes; AmpNode++) {
               AmpNodeConfigType Config;
               TheConfigurator->GetAmpNodeConfig(AmpNode,&Config);
               
               //  Create a CML::Amp object to use with this amplifier node and initialise it
               //  with the relevant CanOpen interface and node Id.
               
               CML::Amp* NewAmpPtr = new CML::Amp;
               const Error* Err = NULL;
               if (IsSimulated(Config.Name)) {
                  if (SimCanOpenInterface == NULL) {
                     I_ErrorString = "Amp " + Config.Name +
                                   " is simulated, but no simulated interface was specified";
                     throw std::exception();
                  } else {
                     Err = NewAmpPtr->Init(*SimCanOpenInterface,Config.NodeId);
                  }
               } else {
                  if (CanOpenInterface == NULL) {
                     I_ErrorString = "Amp " + Config.Name +
                                    " is not simulated, but no real interface was specified";
                     throw std::exception();
                  } else {
                     Err = NewAmpPtr->Init(*CanOpenInterface,Config.NodeId);
                  }
               }
               if (Err) {
                  I_ErrorString = "Error initialising amp " + Config.Name + " ";
                  I_ErrorString += Err->toString();
                  throw std::exception();
               }
               I_AmpMap[Config.Name] = NewAmpPtr;
               //DEBUG ("Amp %s created at %p\n",Config.Name.c_str(),(void*)NewAmpPtr);
            }
            
            //  Similarly for the I/O module nodes.
            
            unsigned int IONodes = TheConfigurator->GetIONodeCount();
            for (unsigned int IONode = 0; IONode < IONodes; IONode++) {
               IONodeConfigType Config;
               TheConfigurator->GetIONodeConfig(IONode,&Config);
               
               //  Create a new CML::IOModule object to use with the node, and initialise it.
               //  Simulation is not at the I/O module level, but at the level of individual items
               //  handled by the node, so we need to create two IOmodules, one for each case.
               //  First, the simulated module - assuming we have support for simulation.
               
               const Error* Err = NULL;
               CanAccessModuleDetails ModuleDetails;
               CML::IOModule* NewIOModulePtr = NULL;
               
               if (SimCanOpenInterface) {
                  NewIOModulePtr = new CML::IOModule;
                  Err = NewIOModulePtr->Init(*SimCanOpenInterface,Config.NodeId);
                  if (Err) {
                     I_ErrorString = "Error initialising simulated IO module bus " +
                                                                            Iter->Name + ". ";
                     I_ErrorString += Err->toString();
                     throw std::exception();
                  }
                  ModuleDetails.BusName = Iter->Name;
                  ModuleDetails.NodeId = Config.NodeId;
                  ModuleDetails.IOModulePtr = NewIOModulePtr;
                  ModuleDetails.Simulated = true;
                  I_ModuleDetailsList.push_back(ModuleDetails);
               }

               //  Then the real one - assuming we have support for real hardware.
               
               if (CanOpenInterface) {
                  NewIOModulePtr = new CML::IOModule;
                  Err = NewIOModulePtr->Init(*CanOpenInterface,Config.NodeId);
                  if (Err) {
                     I_ErrorString = "Error initialising IO module bus " + Iter->Name + ". ";
                     I_ErrorString += Err->toString();
                     throw std::exception();
                  }
                  ModuleDetails.BusName = Iter->Name;
                  ModuleDetails.NodeId = Config.NodeId;
                  ModuleDetails.IOModulePtr = NewIOModulePtr;
                  ModuleDetails.Simulated = false;
                  I_ModuleDetailsList.push_back(ModuleDetails);
               }
            }
         }
         NumberOfCanBuses++;
      }
   } catch (std::exception Except) {
      ReturnOK = false;
   }
   
   if (!ReturnOK) Release();
   
   return ReturnOK;
}

//  ------------------------------------------------------------------------------------------------

//                        C a n  A c c e s s  : :  G e t  C a n  D e t a i l s
//
//  An internal routine that reads and parses the configuration file passed to Initialise()
//  and returns a list of structures describing the CANBus configuration, with one entry for
//  each actual CANBus. It also reads from the file the names of the CANBus items (amplifiers, I/O
//  items) that are to be simulated and which are not. The filename passed may include the names
//  of environment variables preceded by '$', for example "$TDFSIM_DIR/2dFSim.conf". This file is
//  read by this routine, and should have the following format:
//
//  Blank lines and lines beginning with # or * are ignored.
//
//  Other lines must start with one of the following keywords:
//
//  o  CANBus
//         A CANBus line describes a CANBus used by the instrument. It has the following format:
//
//     CANBus Name IniFilename IPAddr SimName SimServer
//         where:
//         Name        is the general name of the CANBus, eg CAN0.
//         IniFilename is the name of the .ini file that describes the detailed layout of the
//                     CANBus. This may include environment variables preceded by '$'.
//         IPAddr      is the IP address of the CANBus interface, assuming this is an internet
//                     interface.
//         SimName     is the name used for the simulated version of the CANBus - usually CAN0 etc.
//                     SimName can be the same as Name, and usually is.
//         SimServer   is the name used by the XWing server for the CANBus. This is only needed
//                     when there are multiple CANBuses being simulated by multiple simulator
//                     tasks, and can be omitted if this is not the case.
//
//  o  SimulateAll
//         Indicates that all CANBus items are to be simulated.
//
//  o  SimulateNone
//         Indicates that no CANBus items are to be simulated.
//
//  o  SimulateAllExcept:
//         Indicates that most CANBus items are to be simulated. A list of items to be run using
//         real hardware must follow, one named item to a line. Anything following that single
//         name is taken as a comment. The list should end with either the end of the file or
//         a line that starts with the keyword: End.
//
//  o  SimulateNoneExcept:
//         Indicates that most CANBus items are to controlled using real hardware. A list of items
//         to be simulated must follow, one named item to a line. Anything following that single
//         name is taken as a comment. The list should end with either the end of the file or
//         a line that starts with the keyword: EndExceptions.
//
//  o  EndExceptions
//         Is used only to end a list of CANBus item names following a SimulateNoneExcept: or
//         a SimulateAllExcept: keyword.
//
//  Keywords are not case-sensitive, but almost everything else - filenames, CANBus item names,
//  CANBus names, server names - are case-sensitive. It's probably best to just treat the whole
//  file as case-sensitive.

bool CanAccess::GetCanDetails (
   const std::string& FileName,
   list<CanAccessBusDetails>& DetailsList)
{
   bool ReadOK = false;
   
   //  The Details list is returned to the caller. The exceptions list (those named CANBus items
   //  which do not follow the default simulation setting) is used internally by this routine
   //  and IsSimulated().
   
   DetailsList.clear();
   I_ExceptionList.clear();
   
   //  Open the file.
   
   std::string ExpandedName = FileName;
   TcsUtil::ExpandFileName(FileName,ExpandedName);
   FILE* ConfFile = fopen(ExpandedName.c_str(),"r");
   if (ConfFile == NULL) {
      I_ErrorString = "Unable to open configuration file '" + FileName + "' : ";
      I_ErrorString += strerror(errno);
   } else {
   
      //  Work through each line in the file.
      
      bool InExceptionList = false;
      for (;;) {
         char Buffer[256];
         if (fgets(Buffer,sizeof(Buffer),ConfFile) == NULL) {
            if (ferror(ConfFile)) {
               I_ErrorString = "Error reading configuration file '" + FileName + "' : ";
               I_ErrorString += strerror(errno);
               break;
            } else if (feof(ConfFile)) {
               ReadOK = true;
               break;
            }
         } else {
         
            //  Split the line we've read into tokens. TcsUtil::Tokenize() will ignore
            //  anything following one of the # or * comment characters, and will honour
            //  character strings enclosed in either 'quotes' or "quotes". Note that
            //  fgets leaves the terminating end of line character in the buffer, and
            //  we'd rather get rid of that.
            
            int Len = strlen(Buffer);
            if (Buffer[Len - 1] == '\n') Buffer[Len - 1] = '\0';
            std::string Line = Buffer;
            std::vector<std::string> Tokens;
            TcsUtil::Tokenize (Line,Tokens," ","\"\'","#*");
            if (Tokens.size() > 0) {
            
               //  This wasn't a blank line. If we're in an exception list, all we care about
               //  is the single first item, which we add to the list of exceptions.
               
               if (InExceptionList) {
                  I_ExceptionList.push_back(Tokens[0]);
               } else {
               
                  //  Otherwise, we look for the various keywords. The only complicated one is
                  //  a CANBus line, but Tokenize() has already done the hard work for that.
                  //  All we have to do is add its details to the Details list. Other keywords
                  //  are trivial.
                  
                  if (TcsUtil::MatchCaseBlind(Tokens[0],"CANBus")) {
                     CanAccessBusDetails Details;
                     if (Tokens.size() < 5) {
                        I_ErrorString = "CANBus line has insufficient arguments: '" + Line + "'";
                        break;
                     }
                     Details.Name = Tokens[1];
                     Details.IniFileName = Tokens[2];
                     std::string ExpandedName;
                     if (TcsUtil::ExpandFileName(Details.IniFileName,ExpandedName)) {
                        Details.IniFileName = ExpandedName;
                     }
                     Details.IPAddress = Tokens[3];
                     Details.SimName = Tokens[4];
                     if (Tokens.size() > 5) {
                        Details.SimServer = Tokens[5];
                     }
                     DetailsList.push_back(Details);
                  } else if (TcsUtil::MatchCaseBlind(Tokens[0],"SimulateAll")) {
                     I_SimulateByDefault = true;
                  } else if (TcsUtil::MatchCaseBlind(Tokens[0],"SimulateNone")) {
                     I_SimulateByDefault = false;
                  } else if (TcsUtil::MatchCaseBlind(Tokens[0],"SimulateAllExcept:")) {
                     I_SimulateByDefault = true;
                     InExceptionList = true;
                  } else if (TcsUtil::MatchCaseBlind(Tokens[0],"SimulateNoneExcept:")) {
                     I_SimulateByDefault = false;
                     InExceptionList = true;
                  } else if (TcsUtil::MatchCaseBlind(Tokens[0],"EndExceptions")) {
                     if (!InExceptionList) {
                        I_ErrorString = "EndExceptions encountered outside list of excepted items.";
                        break;
                     }
                     InExceptionList = false;
                  } else {
                     I_ErrorString = "Unrecognised line in configuration file: " + Line;
                     break;
                  }
               }
            }
         }
      }
   }
   return ReadOK;
}

//  ------------------------------------------------------------------------------------------------

//                        C a n  A c c e s s  : :  G e t  A m p
//
//  Returns a pointer to the CML::Amp object that controls a named amplifier. This will already
//  have been initialised (a call to Init()) with the appropriate CANBus interface and NodeId.

CML::Amp* CanAccess::GetAmp(const std::string& AmpName) {
   CML::Amp* TheAmp = NULL;
   for (std::map<std::string,CML::Amp*>::iterator Iter = I_AmpMap.begin();
                                                    Iter != I_AmpMap.end(); Iter++) {
      if (Iter->first == AmpName) {
         TheAmp = Iter->second;
         break;
      }
   }
   if (TheAmp == NULL) {
      I_ErrorString = "Configuration has no record of an amplifier called " + AmpName;
   }
   //DEBUG ("Amp %s located at %p\n",AmpName.c_str(),(void*)TheAmp);
   return TheAmp;
}

//  ------------------------------------------------------------------------------------------------

//                        C a n  A c c e s s  : :  G e t  I O  M o d u l e
//
//  Returns a pointer to the CML::IOModule object that controls the node used for a named I/O item.
//  This will already have been initialised (a call to Init()) with the appropriate CANBus
//  interface and NodeId. Note that the same IOModule object can handle a number of different
//  named items. The start bit and bit count for the item can be obtained through a call to
//  the appropriate configurator.

CML::IOModule* CanAccess::GetModule(const std::string& IOItemName) {
   CML::IOModule* TheIOModule = NULL;
   unsigned int NodeId,StartBit,Bits;
   IODataType Type;
   std::string CANBusName;
   std::weak_ptr<CANBusConfigurator> TheConfiguratorPtr =
      I_NetworkConfigurator->GetIOItemData(IOItemName,&NodeId,&Type,&StartBit,&Bits,CANBusName);
   if (!TheConfiguratorPtr.expired()) {

      bool Simulated = IsSimulated(IOItemName);
      for (std::list<CanAccessModuleDetails>::iterator Iter = I_ModuleDetailsList.begin();
                                                       Iter != I_ModuleDetailsList.end(); Iter++) {
         if (Iter->BusName == CANBusName && Iter->NodeId == NodeId &&
                                                             Iter->Simulated == Simulated) {
            TheIOModule = Iter->IOModulePtr;
            break;
         }
      }
   }
   if (TheIOModule == NULL) {
      I_ErrorString = "Configuration has no record of an I/O item called " + IOItemName;
   }
   return TheIOModule;
}

//  ------------------------------------------------------------------------------------------------

//                     C a n  A c c e s s  : :  G e t  C o n f i g u r a t o r
//
//  Returns a pointer to the CANBusConfigurator object that controls the node used for a named
//  amplifier or I/O item. This assumes that names of all such items are unique within the
//  current configuration. (Since these names are somewhat arbitrary - they are set by the .ini
//  configuration files - there should be no real difficulty arranging this.)

CANBusConfigurator* CanAccess::GetConfigurator(const std::string& Name) {
   CANBusConfigurator* ConfigPtr = NULL;
   
   //
   std::weak_ptr<CANBusConfigurator> TheConfiguratorPtr =
                                          I_NetworkConfigurator->GetConfiguratorForAmp(Name);
   if (TheConfiguratorPtr.expired()) {
      TheConfiguratorPtr = I_NetworkConfigurator->GetConfiguratorForIOItem(Name);
   }
   if (!TheConfiguratorPtr.expired()) {
   
      //  We want an 'ordinary' pointer, because that's how the interface was defined. We need
      //  to get that from the weak_ptr we have, which means converting to a shared_ptr and
      //  then getting access to the raw pointer it contains.
      
      std::shared_ptr<CANBusConfigurator> TheConfigurator(TheConfiguratorPtr);
      ConfigPtr = TheConfigurator.get();
   }
   if (ConfigPtr == NULL) {
      I_ErrorString = "Configuration has no record of an amplifier or I/O item called " + Name;
   }
   return ConfigPtr;
}

//  ------------------------------------------------------------------------------------------------

//                     C a n  A c c e s s  : :  I s  S i m u l a t e d
//
//  Returns true if a specified amplifier or I/O item is currently being handled through simulation.

bool CanAccess::IsSimulated (const std::string& Name) {

   //  Assume the default.
   
   bool Simulated = I_SimulateByDefault;
   
   //  See if this name is in the list of exceptions.
   
   unsigned int Len = I_ExceptionList.size();
   for (unsigned int I = 0; I < Len; I++) {
      if (I_ExceptionList[I] == Name) {
         Simulated = !Simulated;
         break;
      }
   }
   
   return Simulated;
}


