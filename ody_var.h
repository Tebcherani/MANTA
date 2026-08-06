/*----------------------------------------------------------------------*
   Copyright (C) 1994-1998, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: ody_var.h,v 2.23 1998/02/03 04:25:07 bamoran Exp $
 *----------------------------------------------------------------------*/

#ifndef ODY_VAR_H
#define ODY_VAR_H

#include "ody_defs.h"
#include "ody_types.h"

extern const struct behavior bTemplate[];
extern const struct sensor sss[];
extern const int num_behaviors, num_sensors;

EXTERN struct vehicle vvv, *vvp;

/* Globals of serial port file numbers (Perhaps use a structure) */
EXTERN int Sail1;                /* sail 1 port number */
EXTERN int AltimeterSonar;        /* Altimeter sonar port number */

EXTERN int AltimeterPortNumber;        /* 1 or 85 for /t1 or /t85 */
EXTERN int RunNumber;
EXTERN int DeadManSwitchEnabled;  /* for aborting mission from RF modem */
EXTERN int NewMissionFileEnabled; /* for reading new mission during run */
EXTERN char MissionFileName[60];  

/*---------------------------------------------------------------------*
 | For each vehicle system, we have four flags: Error, Status, Use, and
 | Monitor.   Error flags refer primarily to initialization, and will 
 | prevent a mission from proceeeding for a critical mission system. 
 | Status flags refer to sensor errors that occur during the main loop.
 |<In general, these won't turn off the device, because in most cases 
 | we could get in more trouble by simply turning the system off instead
 | of hoping the problem was a temporary glitch
 *---------------------------------------------------------------------*/
				/* kvh compass */
extern int ErrorKVH_DGC, StatusKVH_DGC, UseKVH_DGC, MonitorKVH_DGC;
				/* kvh inclinometer */
extern int ErrorKVH_DGI, StatusKVH_DGI, UseKVH_DGI, MonitorKVH_DGI;

				/* Paroscientific Digiquartz */
extern int ErrorParosci, StatusParosci, UseParosci, MonitorParosci;

				/* serial ascii instrumentation loop */
EXTERN int ErrorSail1, StatusSail1, UseSail1, MonitorSail1;
				/* MITSG Watchdog board */
extern int ErrorWatchdog, StatusWatchdog, UseWatchdog, MonitorWatchdog;
				/* MITSG thruster */
extern int ErrorThruster, StatusThruster, UseThruster, MonitorThruster;

				/* MITSG/WHOI fins */
extern int ErrorRudder, StatusRudder, UseRudder, MonitorRudder;
extern int ErrorElevator, StatusElevator, UseElevator, MonitorElevator;
extern int HallEffectIndex, FinHECalibrate;
				/* simulator mode */
extern int UseSim, MonitorSim;

EXTERN int MonitorDataLogging;
EXTERN int MonitorLayeredControl;

EXTERN int DebugMode; 
EXTERN int CommsDebugMode; 
EXTERN int TimingDebugMode;	/* send chars to a serial port for timing
				   check */

EXTERN int Countdown;		/* prior to start, in seconds */
EXTERN int WatchdogTimeoutMinutes;
EXTERN int WatchdogTimeoutHours;

EXTERN double CommandedSpeed;
EXTERN double CommandedDepth;
EXTERN double CommandedHeading;
EXTERN double CommandedHeadingDegrees;
EXTERN int CommandedTime;

EXTERN int UseKalmanFilter;

#if defined _NRAD
extern int ErrorNRaD, StatusNRaD, UseNRaD, MonitorNRaD;
#elif defined _EDC		/* bamoran 08/22/95 */
extern int ErrorEDC, StatusEDC, UseEDC, MonitorEDC;
#endif

#ifdef WHOI_LATCH
extern int ErrorLatchMotor, StatusLatchMotor, UseLatchMotor, MonitorLatchMotor;
#endif

#ifdef AEL_BATCON		/* Al Bradley's battery controller */
extern int ErrorBatCon, StatusBatCon, UseBatCon, MonitorBatCon;
#endif


#ifdef EDC_COUPLE		/* dummy load as of 10/30/96 */
extern int ErrorLoad, StatusLoad, UseLoad, MonitorLoad;
#endif

#ifdef EDGEPS8000
extern int LBLTransponderMode;  /* If mode = 0, then in pinger mode */
extern int ErrorLBL, MonitorLBL, UseLBL;
#endif
				/* Tritech ST-200 */
EXTERN int ErrorAltimeter, StatusAltimeter, UseAltimeter, MonitorAltimeter;
EXTERN int OldAltimeter;	/* 0 = new, 1 = old altimeters */
EXTERN int OldAltimeterID;	/* 1 for 200kHz, 2 for Stbd 500 kHz, 3 for
				   Port 500kHz */ 
EXTERN int UsePort;		/* for old sonars, enable one by one */
EXTERN int UseStarboard;
EXTERN int UseOld200;
EXTERN int AltimeterFrequency;	/* 500 vs 200 kHz (applies to new only) */

#ifdef SBE_25
extern int UseCondTemp, MonitorCondTemp, StatusCondTemp, ErrorCondTemp;
#endif

#ifdef SONTEK_ADV		/* SonTek ADV Ocean */
extern int UseADV, MonitorADV, StatusADV, ErrorADV;
#endif

#ifdef PSA_916			/* Datasonics PSA_916 Altimeter */
EXTERN int UsePSA, MonitorPSA, StatusPSA, ErrorPSA;
#endif

#ifdef ODY_CAMERA
extern int Camera, CameraOnTime, CameraOffTime;
extern int UseCamera, MonitorCamera, StatusCamera, ErrorCamera;
#endif

				/* ody_modem and whoi_usbl */
#if    defined ODY_MODEM || defined WHOI_USBL
EXTERN int UseModem, MonitorModem, StatusModem, ErrorModem;
EXTERN int ModemInterval;
#ifndef WHOI_USBL
EXTERN FILE *modemlog;		/* modemlog only for ODY_MODEM */
#endif
#endif

#ifdef ODY_SIDESCAN		/* IOS Doppler Sidescan Sonar */
extern int UseSideScan, MonitorSideScan, StatusSideScan, ErrorSideScan;
#endif

#ifdef RDI_WH			/* RDI Workhorse ADCP */
extern int UseRDI, MonitorRDI, StatusRDI, ErrorRDI;
#endif

#endif /* ODY_VAR_H */
