/***************************************************************************

    file                 : driver.h
    created              : Thu Dec 20 01:20:19 CET 2002
    copyright            : (C) 2007 Andrew Sumner, 2004 Remi Coulom, 2002-2004 Bernhard Wymann
    version              : $Id$

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _DRIVER_H_
#define _DRIVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>

#include "linalg.h"
#include "opponent.h"
#include "pit.h"
#include "learn.h"
#include "strategy.h"
#include "cardata.h"
#include "correct.h"

#define HYMIE_SECT_PRIV "hymie"
#define HYMIE_ATT_FUELPERLAP "fuelperlap"
#define HYMIE_ATT_FUELPERSECOND "fuelpersecond"
#define HYMIE_ATT_MUFACTOR "mufactor"

class Opponents;
class Opponent;
class Pit;
class AbstractStrategy;

#define MaxSegments 3000
#define MaxDivs 10000
enum { TEAM_KAOS=1, TEAM_CONTROL=2 };
enum { AVOID_FRONT=1, AVOID_SIDE=2, AVOID_BACK=4, AVOID_LEFT=8, AVOID_RIGHT=16, AVOID_HOLD=32, AVOID_INSIDE=64, AVOID_LETPASS=128, AVOID_SIDE_COLL=256, AVOID_ALIGNED=512, AVOID_TEAM=1024, AVOID_SWITCH=2048 };

class Driver {
 public:
  Driver(int index);
  ~Driver();

  // Callback functions called from TORCS.
  void initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s);
  void newRace(tCarElt* car, tSituation *s);
  void drive(tSituation *s);
  int pitCommand(tSituation *s);
  void endRace(tSituation *s);

  tCarElt *getCarPtr() { return car; }
  tTrack *getTrackPtr() { return track; }
  int getAvoiding() { return (avoiding ? !(avoiding & AVOID_ALIGNED) : 0); }
  int getCorrecting() { return correct->getCorrecting(); }
  int getAllowCorrecting() { return correct->getAllowCorrecting(); }
  double getCorrectTimer() { return correct->getCorrectTimer(); }
  double getSpeed() { return mycardata->getSpeedInTrackDirection(); /*speed;*/ }
  double getNextLeft() { return nextleft; }
  double getLaneLeft(tCarElt *pcar, double distance, double Time);
  double getCarRInverse(tCarElt *pcar, double distance);
  int getCarDiv(tCarElt *pcar, double distance);
  double getWidth() { return mycardata->getWidthOnTrack(); }
  double getLength() { return mycardata->getLengthOnTrack(); }
  void setCorrecting(int n) { correct->setCorrecting(n); }
  void setAllowCorrecting(int n) { correct->setAllowCorrecting(n); }
  double getSimTime() { return currentsimtime; }
  double getSideMargin();
  double getSlowestSpeed(double distance);
  double getPitEntryOffset() { return PitEntryOffset; }
  int getStrategy();
  int Alone() { return alone; }
  int Chased();

  tPosd *getCornerPos() { return corner1; }
  tPosd *getCornerPos2() { return corner2; }
  double getSpeedAngle() { return speedangle; }
  double getNextWidth() { return nextwidth; }

 private:
  // Utility functions.
  bool isStuck();
  void update(tSituation *s);
  double getAccel();
  double getDistToSegEnd();
  double getDistToCorner();
  double getBrake(double brake);
  int getGear();
  double getSteer(tSituation *s);
  double getClutch();
  v2d getTargetPoint();
  v2d getTargetPoint(double offset, double lookahead);
  double getOffset();
  double AvoidBrake(double speed);
  double brakedist(double allowedspeed, double mu);

  double filterOverlap(double accel);
  double filterABS(double brake);
  double filterBPit(double brake);
  double filterBrakeSpeed(double brake);
  double filterTurnSpeed(double brake);

  double filterTCL(double accel, char slow);
  double filterTrk(double accel);

  double filterSColl(double steer);

  double filterTCL_RWD();
  double filterTCL_FWD();
  double filterTCL_4WD();
  void initTCLfilter();

  void initCa();
  void initCw();
  void initCR();
  void initTireMu();

  int isAlone();

  // Per robot global data.
  int stuck;
  float speedangle;  // the angle of the speed vector relative to trackangle, > 0.0 points to right.
  int pitpos;
  float mass;    // Mass of car + fuel.
  float fuelperlap;
  double ToLeft;
  float myoffset;   // Offset to the track middle.
  tCarElt *car;   // Pointer to tCarElt struct.

  Opponents *opponents; // The container for opponents.
  Opponent *opponent;  // The array of opponents.

  Pit *pit;      // Pointer to the pit instance.
  AbstractStrategy *strategy;  // Pit stop strategy.
  Correct *correct;

  static Cardata *cardata;  // Data about all cars shared by all instances.
  SingleCardata *mycardata;  // Pointer to "global" data about my car.
  static double currentsimtime; // Store time to avoid useless updates.
  double deltaTime;
  double lastLTleft, avoid_diverge, lastTleft;

  double currentspeedsqr; // Square of the current speed_x.
  float clutchtime;  // Clutch timer.
  float oldlookahead;  // Lookahead for steering in the previous step.

  SegLearn *learn;
  int alone;
  int rl;

  // Data that should stay constant after first initialization.
  int MAX_UNSTUCK_COUNT;
  int INDEX;
  int carindex;
  double CARMASS;  // Mass of the car only [kg].
  double CA;   // Aerodynamic downforce coefficient.
  double FCA;  // front downforce
  double RCA;  // rear downforce
  double FWA;  // front wing angle
  double CW;   // Aerodynamic drag coefficient.
  double CR;   // Front/rear weight repartition
  double brakeCW;   // Aerodynamic drag coefficient for braking.
  double TIREMU;  // Friction coefficient of tires.
  double (Driver::*GET_DRIVEN_WHEEL_SPEED)();
  float OVERTAKE_OFFSET_INC;  // [m/timestep]
  double MU_FACTOR;    // [-]

  // Class constants.
  static const float MAX_UNSTUCK_ANGLE;
  static const float MIN_UNSTUCK_ANGLE;
  static const float UNSTUCK_TIME_LIMIT;
  static const float MAX_UNSTUCK_SPEED;
  static const float MIN_UNSTUCK_DIST;
  static const float G;
  static const float FULL_ACCEL_MARGIN;
  static const float SHIFT;
  static const float SHIFT_MARGIN;
  static float ABS_SLIP;
  static float ABS_RANGE ;
  static const float ABS_MINSPEED;
  static const float TCL_SLIP;
  static const float LOOKAHEAD_CONST;
  static const float LOOKAHEAD_FACTOR;
  static const float WIDTHDIV;
  static const float SIDECOLL_MARGIN;
  static const float BORDER_OVERTAKE_MARGIN;
  static const float OVERTAKE_OFFSET_SPEED;
  static const float PIT_LOOKAHEAD;
  static const float PIT_BRAKE_AHEAD;
  static const float PIT_MU;
  static const float MAX_SPEED;
  static const float TCL_RANGE;
  static const float MAX_FUEL_PER_METER;
  static const float CLUTCH_SPEED;
  static const float CENTERDIV;
  static const float DISTCUTOFF;
  static const float CATCH_FACTOR;
  static const float CLUTCH_FULL_MAX_TIME;
  static const float USE_LEARNED_OFFSET_RANGE;
  static const float OVERTAKE_TIME_CUTOFF;

  double WingRInverse;
  double TireAccel1;
  double MaxBrake;
  double SlipLimit;
  double TSlipLimit;
  double SteerSkid;
  double SteerGain;
  double SteerGain2;
  double SecurityZ;
  double CurveAccel;
  double CurveFactor;
  double RevsChangeDown;
  double RevsChangeDownMax;
  double RevsChangeUp;
  double FlyingCaution;
  double FlyingWidth;
  double TrackFriction;
  double PitDamage;
  double IntMargin;
  double ExtMargin;
  double AvoidSpeed;
  double CAModifier;
  double SteerLimit;
  double EdgeAllowance;
  double FuelSpeedup;
  double BrakeShift;
  double AvoidMargin;
  double ABSFactor;
  double BrakePressure;
  double Learning;
  double RaceLearning;
  double MaxIncFactor;
  double TurnAccel;
  double TurnDecel;
  double BTBoost, AvoidBTBoost;
  double TeamWaitTime;
  double SteerGainDiv;
  double PitEntryOffset;
  char * pszCarName; 
  double wheelz[4];

  double ABS;
  double TractionHelp;
  double wheelbase;
  double wheeltrack;

  double Width;
  double Length;
  double tSegDist[MaxSegments];
  double tElemLength[MaxSegments];
  double tLSlope[MaxSegments];
  double tRSlope[MaxSegments];
  double tLDelta[MaxSegments];
  double tRDelta[MaxSegments];
  double tRadius[MaxSegments];
  double tSegArc[MaxSegments];
  tTrackSeg *tSegment[MaxSegments];
  //float tAvoidRight[MaxSegments];
  //float tAvoidLeft[MaxSegments];
  double Flying[MaxDivs];
  double tx[3][MaxDivs];
  double ty[3][MaxDivs];
  double tRInverse[3][MaxDivs];
  double tMaxSpeed[MaxDivs];
  double tLSlowSpeed[MaxDivs];
  double tRSlowSpeed[MaxDivs];
  double tSpeedAdjust[MaxDivs];
  double tLaneAdjust[MaxDivs];
  double tSpeed[MaxDivs];
  double txLeft[3][MaxDivs];
  double tyLeft[3][MaxDivs];
  double txRight[3][MaxDivs];
  double tyRight[3][MaxDivs];
  double tLane[3][MaxDivs];
  double tLaneLMargin[MaxDivs];
  double tLaneRMargin[MaxDivs];
  double tFriction[MaxDivs];
  double lastX[10], lastY[10];
  double ClutchTime;
  double AccelCmd;
  double BrakeCmd;
  double KBrakeCmd;
  double TargetSpeed;
  double BTargetSpeed;
  double KTargetSpeed;
  double LTargetSpeed;
  double RTargetSpeed;
  double angle;
  double correct_meld;
  double correct_diff;
  double align_hold;
  double avoid_hold;
  double nextleft;
  double prevleft;
  double k1999steer;
  double k1999steer_a;
  double k1999steer_c;
  double k1999steerL;
  double k1999steerR;
  double laststeer, lastlaststeer, lastTC, KTC, KTCdiff;
  double lastyaw;
  double bc_timer;
  double lastksteer;
  double lastbrake;
  double lastaccel;
  double slowslip;
  double speedfactor;
  double aero;
  double brake_collision;
  double CosAngleError, SinAngleError, Error;
  double sidemargin, frontmargin;
  double BTAccelCmd;
  double BTBrakeCmd;
  double cornerdist;
  double cornerlimit;
  double flying_timer;
  double lastLoffset;
  double lastRoffset;
  double prevwidth;
  double nextwidth;
  double skill_level;
  double cur_speed_adjust;
  double trg_speed_adjust;
  double speed_adjust_limit;
  double speed_adjust_timer;

  float brake_avoid;
  float last_speedy;
  float last_lapfuel;
  float reversehold;
  float stuckhold;
  float tSegHeight[MaxSegments];
  float tSegLength[MaxSegments];
  int tSegIndex[MaxSegments];
  int tSegID[MaxSegments];
  int tDivSeg[MaxDivs];
  int tSegDivStart[MaxSegments];
  int tLearnCount[MaxDivs];
  //int tfConst[MaxDivs];
  int Braking[MaxDivs];
  int raceline;
  int fStuck;
  int NoAccelRelax;
  int LearnLimit;
  int K1999Brakes;
  int LoadSVG;
  int SaveSVG;
  int NoTeamWaiting;
  int OverrideLearning;
  int Iterations;
  int Divs;
  int Segs;
  int LearningLoaded;
  int segtype;
  int nextdiv;
  int thisdiv;
  int avoiding;
  int racetype;
  int prefer_side;
  int lastdiv;
  int lastgear;
  int lastsegid;
  int last_damage;
  int last_lap;
  int old_avoiding;
  int segupdated;
  v2d k1999pt;
  v2d nextpt;
  int start_finished;
  int useBTSpeed;
  int lastcornertype;
  int reversetype;
  int fStuckForward;
  int side_count;
  unsigned int random_seed;
  tTrackSeg *cseg;

  int fDirt;
  int last_flying;
  int drivetrain;

  tPosd corner1[4];
  tPosd corner2[4];

  void UpdateTxTy(int i);
  void SetSegmentInfo(const tTrackSeg *pseg, double d, int i, double l);
  void SplitTrack(tTrack *ptrack);
  void ComputeFullPath();
  void Optimize();
  double GetRInverse(int prev, double x, double y, int next, int rline);
  void AdjustRadius(int prev, int i, int next, double TargetRInverse, double Security = -1);
  double InverseFriction(double a);
  double GetControl(double At, double An, double v, double mass);
  void Smooth(int Step);
  void StepInterpolate(int iMin, int iMax, int Step);
  void Interpolate(int Step);
  void K1999InitTrack(tTrack* track, void **carParmHandle, tSituation *p);
  double K1999Steer(tSituation *s);
  double AvoidSteer(tSituation *s);
  double CheckAvoidSteer(tSituation *s, double steercmd);
  void setCorrecting();
  void FindPreferredSide();
  double getDistToSegStart(tCarElt *car, int useovertake);
  tTrackSeg *getNextCornerSeg();
  double getAllowedSpeed(tTrackSeg *seg, int accel);
  double filterTeam(double accel);
  double getAvoidSpeed(double toLeft);
  int CheckFlying();
  void InitSegInfo();
  void initRadius();
  void initWheelPos();
  void loadLearning();
  void saveLearning();
  void loadSVG();
  void saveSVG();
  void updateSegAvoidance();
  int rearWheelSlip();
  void SetRandomSeed(unsigned int seed);
  unsigned int getRandom();

  // Track variables.
  tTrack* track;
};

#endif // _DRIVER_H_

