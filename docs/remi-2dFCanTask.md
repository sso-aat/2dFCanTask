# SSO AAT ops-305-2df-simulator 2dFCanTask DRAMA

<div align="right">2024-02-28 rev E</div>

# Introduction

In March 2018, Keith Shortridge wrote and tested 2dFCanTask, a DRAMA C++ program, against the 2dF hardware simulator.  We reproduce this work with upgraded operating systems, libraries, and DRAMA.  A motion test program written by Lew Waller was used to watch amplifier telemetry.

# Results

In 2021;

* verified the operational state of the 2dF hardware simulator, with no critical damage during storage, some missing parts, and safe operation,
* learning of AAO software practice, AAO software libraries, DRAMA, AnaGate CAN uno, and Copley axis amplifiers,
* characterised network performance impacts on axis operations,
* identified likely network security vulnerabilities of the AnaGate CAN uno,
* identified software license considerations,
* identified software regressions in multithreaded message handling due to changes in DRAMA since 2018,
* determined that the robot is fast.

# Environment Preparation

* select virtual machines aatlie and aatlij, update to Ubuntu 20.04, and maintain continuous critical updates,
* clone, configure, build, and install ACMM,
* clone, configure, build, and install DRAMA and DRAMA dependencies,
  * DramaConfig ACMM module,
  * DramaErs ACMM module,
  * imp ACMM module,
  * sds ACMM module,
  * DramaDits ACMM module,
  * DramaGit ACMM module,
  * DramaDul ACMM module,
  * Drama2 ACMM module,
* create an `IMP_Startup` file in the home directory containing;

        use port 2123

* clone, configure and build Lew Waller's test program dependencies,
* clone, configure and build 2dFCanTask dependencies,
  * AnaGateAPI CAN download,
  * CANopen_CML ACMM module,
  * CanInterfacesAAO ACMM module,
  * 2dFCanSimulator ACMM module,
  * AAOCANBusIniConfigurator ACMM module,
  * GenInstSimulator ACMM module,
  * SimCanOpen ACMM module,
  * SimSerialLine ACMM module,
  * TcsUtil ACMM module,
  * XWing ACMM module,
* clone, and fail to build 2dFCanTask,
* update 2dFCanTask for changes to DRAMA since 2018 [Tony Farrell],

# Test #1 - X1, X2, and Y-Axis

In 2024, a test using all three operational axes.  Outcome is remote control of position through DRAMA and CANopen.  Refer to the following diagram;

![diagram](remi-2dFCanTask.svg "diagram")

## Preparation

* clone, fork, configure and build 2dFCanTask for three axis testing [James Cameron],
* test 2dFCanTask using DRAMA DITS commands via terminal,

## Commits

On branch xy-only of 2dFCanTask commits were obtained from earlier work;

* cb35038 ("Use SimulateNone")
* 5da8ac3 ("Remove all but X and Y axis from code")
* 297195d ("Bringup X and Y")

## Method

* position the hardware simulator X and Y axes about 10cm from home,
* power the hardware simulator,
* power the AnaGate CAN uno,
* turn on the hardware enable switches for the X and Y axes, by moving the switch to the left,
* disable the X1 X2 link by moving the switch to the left,
* start configured DRAMA shell environment

        ~drama/dramastart host bash
        source $DRAMA/drama.sh

* configure 2dFCanTask directory

        export TDFCTASK_DIR=.

* start 2dFCanTask

        cd /var/tmp/build-1/2dFCanTask
        ./2dFCanTask &

* start test web app

        cd /var/tmp/build-1/2dFCanTask
        ./remi-2dFCanTask &

* at end of test, exit and stop DRAMA network task

        ditscmd TdFCanTask EXIT
        exit

* open web browser on phone or laptop,

        http://aatlij:8081/

![diagram](qr.svg "diagram")

* home all three axes, verifying both X1 and X2 move in tandem,
* command X and Y axes at will, verifying movement,

# Resources

## Development Machines

* aatpc15.aao.gov.au, Ubuntu 20.04,

* aatlij.aao.gov.au, Ubuntu 22.04,

## Development Filesystems

* aatpc15:src/2dFCanTask

* jcameron@aatpc15:ops-305-2df-simulator/remi-2dFCanTask (a git repository)
  * jcameron@aatliz:ops-305-2df-simulator/remi-2dFCanTask

## Test Machines

* aatlij.aao.gov.au, Ubuntu 22.04,

## Test Filesystems

* jcameron@aatlij:/var/tmp/build-1

## Hardware Simulator

* anagate-can-uno-2df-simulator.aao.gov.au 10.88.16.71,

* web app status option shows TCP and CAN counters,

## Tools

* mosh,

* emacs,

* tramp,

* magit,

* git,

* ACMM,

* LewsCAN_TestPrograms aka motion_test [Lew Waller], for simultaneous access for diagnostic purposes,

        motion_test -b 1000000 -d 10.88.16.71 3

* tcpdump,

* wireshark,

* Python Remi.

	pip3 install remi

# References

* 2dF Testbed Software.pdf, Control Task and Simulator: State of Play, Keith Shortridge, K&V, 30th March 2018,

* Drama2_CAN_Template.pdf, Using the DRAMA2 AAO CAN Task Template, Tony Farrell, 12th August 2021,

* 2dFsimHardwareInterfaceTechRef V1.7.pdf, 2dFsim Hardware Interface Technical Reference, April 2017, AAT-EL-2DFSIM-001-HI-TR, Vijay Nichani, Lew Waller, and Chris McCowage,

* HECTOR Instrument Computer Linux OS Setup, AAO Confluence, https://confluence.aao.org.au/display/AAOSOFT/HECTOR+Instrument+Computer+Linux+OS+Setup 30th June 2021, Tony Farrell.
