/***************************************************************************

    file                 : driver.h
    created              : 9 Apr 2006
    copyright            : (C)2006 Tim Foden - (C)2014 Xavier BERTAUX
    email                : bertauxx@yahoo.fr
    version              : $Id: driver.h 5631 2014-12-27 21:32:55Z torcs-ng $

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

#include <track.h>
#include <car.h>
#include <robot.h>

#include "../../../modules/simu/simuv4/aero.h"

#include "MyTrack.h"
#include "Shared.h"
#include "ClothoidPath.h"
#include "OptimisedPath.h"
#include "PitPath.h"
#include "Opponent.h"
#include "PidController.h"
#include "LearnedGraph.h"
#include "AveragedData.h"
#include "LinearRegression.h"
#include "PtInfo.h"
#include "Strategy.h"
#include "teammanager.h"

#define SECT_PRIV               "private"
#define PRV_SHIFT               "shift"
#define PRV_MU_SCALE            "mu scale"
#define PRV_RAIN_MU             "mu scale rain"
#define PRV_ACCEL_DELTA         "accel delta"
#define PRV_ACCEL_DELTA_RAIN    "accel delta rain"
#define PRV_BRAKESCALE          "brake scale"
#define PRV_BRAKEFORCE          "brake force"
#define PRV_FLY_HEIGHT          "fly height"
#define PRV_FACTOR              "factor"
#define PRV_AERO_MOD            "aero mod"
#define PRV_BUMP_MOD            "bump mod"
#define PRV_SIDE_MOD            "side mod"
#define PRV_KZ_SCALE            "kz scale"
#define PRV_BUMP_FACTOR         "bump factor"
#define PRV_BUMP_FACTOR_LEFT    "bump factor left"
#define PRV_BUMP_FACTOR_RIGHT   "bump factor right"
#define PRV_CLUTCH_DELTA        "clutch delta"
#define PRV_CLUTCH_RANGE        "clutch range"
#define PRV_CLUTCH_MAX          "clutch max"
#define PRV_CLUTCH_RELEASE      "clutch release"
#define PRV_STEER_K_ACC         "steer k acc"
#define PRV_STEER_K_DEC         "steer k dec"
#define PRV_AVOID_WIDTH         "avoid width"
#define PRV_SPDC_NORMAL         "spd ctrl normal"
#define PRV_SPDC_TRAFFIC        "spd ctrl traffic"
#define PRV_SPDC_EXTRA          "spd ctrl extra"
#define PRV_STEER_CTRL          "steer ctrl"
#define PRV_STAY_TOGETHER       "stay together"
#define PRV_PITSTRAT            "pitstrat"
#define PRV_PITSTOP             "chkpitstop"
#define PRV_CHKQUALIF           "chkqualiftime"
#define PRV_PIT_ENTRY_OFFS      "pit entry offset"
#define PRV_PIT_EXIT_OFFS       "pit exit offset"
#define PRV_MAX_BRAKING         "max braking"
#define PRV_FUELPERMETERS       "fuel per meters"
#define PRV_FUELPERLAPS         "fuel per lap"
#define PRV_RESERVE             "reserve"
#define PRV_DAMAGE              "damage"
#define PRV_FULL_FUEL           "full fuel"
#define PRV_VERBOSE             "strategyverbose"
#define PRV_NEED_SIN            "use sin long"
#define PRV_USED_ACC            "acc exit"
#define PRV_SKILL_OFFSET        "offset skill"
#define PRV_BRAKE_LIMIT         "brake limit"
#define PRV_BRAKE_LIMIT_BASE    "brake limit base"
#define PRV_BRAKE_LIMIT_SCALE   "brake limit scale"
#define PRV_SPEED_LIMIT_BASE    "speed limit base"
#define PRV_SPEED_LIMIT_SCALE   "speed limit scale"
#define PRV_TCL_SLIP            "tcl slip"
#define PRV_TCL_RANGE           "tcl range"
#define PRV_TCL_FACTOR          "tcl factor"
#define PRV_ABS_SLIP            "abs slip"
#define PRV_ABS_RANGE           "abs range"

#define NBR_BRAKECOEFF 50                                   // Number of brake coeffs

const double	SPD_MIN = 0;
const double	SPD_MAX = 120;
const int	SPD_N = 20;
const double	SPD_STEP = (SPD_MAX - SPD_MIN) / SPD_N;
const double	K_MIN = -0.1;
const double	K_MAX = 0.1;
const int	K_N = 100;
const double	K_STEP = (K_MAX - K_MIN) / K_N;

// The "SHADOW" logger instance.
extern GfLogger* PLogSHADOW;
#define LogSHADOW (*PLogSHADOW)

class   AbstractStrategy;
class   SimpleStrategy;

//==========================================================================*
// Speed Dreams-Interface
//--------------------------------------------------------------------------*
static const int MAX_NBBOTS = 100;
static const int MAXNBBOTS = MAX_NBBOTS;         // Number of drivers/robots
static const int BUFSIZE = 256;

enum { SHADOW_TRB1=1, SHADOW_SC, SHADOW_SRW, SHADOW_LS1, SHADOW_LS2, SHADOW_36GP,
       SHADOW_67GP, SHADOW_RS, SHADOW_LP1, SHADOW_MPA1, SHADOW_MPA11, SHADOW_MPA12 };

class TDriver
{
public:
  enum	// paths
  {
    PATH_NORMAL,
    PATH_LEFT,
    PATH_RIGHT,

    N_PATHS
  };

  enum
  {
    STEER_SPD_MAX = 20,
    STEER_K_MAX = 41,
    HIST = 20
  };

public:
  TDriver(int index, const int robot_type);       //  Constructor
  ~TDriver();                                     // Destructor

  void	SetShared( Shared* pShared );
  void	InitTrack(tTrack* track, void* carHandle, void** carParmHandle, tSituation* s);
  void	NewRace(tCarElt* car, tSituation* s );

  void	GetPtInfo( int path, double pos, PtInfo& pi ) const;
  void	GetPosInfo( double pos, PtInfo& pi, double u, double v ) const;
  void	GetPosInfo( double pos, PtInfo& pi ) const;
  double	CalcPathTarget( double pos, double offs, double s ) const;
  double	CalcPathTarget( double pos, double offs ) const;
  Vec2d	CalcPathTarget2( double pos, double offs ) const;
  double	CalcPathOffset( double pos, double s, double t ) const;
  void	CalcBestPathUV( double pos, double offs, double& u, double& v ) const;
  double	CalcBestSpeed( double pos, double offs ) const;
  void	GetPathToLeftAndRight( const CarElt* pCar, double& toL, double& toR ) const;

  double filterTCL(double accel);
  double filterTrk(double accel);
  double filterAccel(double Accel);

  void initCa();
  //void initCa_MPA1();
  //void initCa_MPA11();
  void initCw();
  void initCR();

  void initDriveTrain();
  void initWheelRadius();
  void initTireMu();
  void initWheelPos();
  void initBrake();

  double	SteerAngle0( tCarElt* car, PtInfo& pi, PtInfo& aheadPi );
  double	SteerAngle1( tCarElt* car, PtInfo& pi, PtInfo& aheadPi );
  double	SteerAngle2( tCarElt* car, PtInfo& pi, PtInfo& aheadPi );
  double	SteerAngle3( tCarElt* car, PtInfo& pi, PtInfo& aheadPi );
  double	SteerAngle4( tCarElt* car, PtInfo& pi, PtInfo& aheadPi );

  void	SpeedControl0( double targetSpd, double spd0, double& acc, double& brk );
  void	SpeedControl1( double targetSpd, double spd0, double& acc, double& brk );
  void	SpeedControl2( double targetSpd, double spd0, double& acc, double& brk );
  void	SpeedControl3( double targetSpd, double spd0, double& acc, double& brk );
  void	SpeedControl4( double targetSpd, double spd0, CarElt* car,
                       double& acc, double& brk );
  void  SpeedControl5(double targetSpd, double spd0, CarElt* car,
                        double& acc, double& brk );
  void  SpeedControl6(double targetSpd, double spd0, CarElt* car,
                        double& acc, double& brk );
  void	SpeedControl( int which, double targetSpd, double spd0,
                      CarElt* car, double& acc, double& brk );

  void	Drive(tSituation* s );
  int	PitCmd(tSituation* s );
  void	EndRace(tSituation* s );
  void	Shutdown();

  static int          NBBOTS;                         // Nbr of cars
  double              CurrSimTime;                    // Current simulation time
  static const char   *robot_name;
  double              Frc;                            // Friction coefficient
  //bool                UseBrakeLimit;                // Enable/disable brakelimit

  double              wheelRadius;

  static const char*  ROBOT_DIR;                      // Sub path to dll
  static const char*  DEFAULTCARTYPE;                 // Default car type

  static int          RobotType;
  static bool         AdvancedParameters;
  static bool         UseOldSkilling;
  static bool         UseSCSkilling;
  static bool         UseMPA1Skilling;
  static float        SkillingFactor;
  static bool         UseBrakeLimit;
  static bool         UseGPBrakeLimit;
  static bool         UseRacinglineParameters;
  static bool         UseWingControl;
  static float        BrakeLimit;
  static float        BrakeLimitScale;
  static float        BrakeLimitBase;
  static float        SpeedLimitScale;
  static float        SpeedLimitBase;
  static bool         FirstPropagation;
  static bool         Learning;

  int                 INDEX;                           // index of own driver
  int                 m_Extended;                      // Information if this robot is extended (oExtended = 1) or not (oExtended = 0).
  int                 m_TestPitStop;                   // Test pit stop

  void    CalcSkilling();
  double  CalcFriction(const double Crv);
  double  CalcCrv(double Crv);
  double  CalcHairpin(double Speed, double AbsCrv);

  void    (TDriver::*CalcSkillingFoo)();
  double  (TDriver::*CalcFrictionFoo)(const double Crv);
  double  (TDriver::*CalcCrvFoo)(double Crv);
  double  (TDriver::*CalcHairpinFoo)(double Speed, double AbsCrv);

  void    CalcSkill();
  void    CalcSkilling_shadow();
  void    CalcSkilling_shadow_LS1();
  void    CalcSkilling_shadow_LS2();
  void    CalcSkilling_shadow_MPA1();
  void    CalcSkilling_shadow_SC();
  void    CalcSkilling_shadow_LP1();

  void    AdjustSkilling(void* pCarHandle);
  void    AdjustBrakes(void *pCarHandle);

  void    GetSkillingParameters();

  double  CalcFriction_shadow_Identity(double Crv);
  double  CalcFriction_shadow_LS2(double Crv);
  double  CalcFriction_shadow_LP1(double Crv);
  double  CalcFriction_shadow_REF(double Crv);

  double  CalcCrv_shadow(double Crv);
  double  CalcCrv_shadow_SC(double Crv);
  double  CalcCrv_shadow_Identity(double Crv);
  double  CalcCrv_shadow_36GP(double Crv);
  double  CalcCrv_shadow_LP1(double Crv);

  double  CalcHairpin_shadow_Identity(double Speed, double AbsCrv);
  double  CalcHairpin_shadow(double Speed, double AbsCrv);

  void    UseFilterAccel() { m_UseFilterAccel = true; };
  void    UseAccelOut() { m_UseAccelOut = true; };
  void    SetBotName(const char* Value);

  void    ScaleSide(float FactorMu, float FactorBrake);
  void    SideBorderOuter(float Factor);
  void    SideBorderInner(float Factor);
  void    LearnBraking(double Pos);                   // Learn braking parameters
  void    DetectFlight();
  double  FlightControl(double Steer);                // Prepare landing
  double  TyreConditionFront();
  double  TyreConditionRear();
  double  TyreTreadDepthFront();
  double  TyreTreadDepthRear();

  bool        m_UseFilterAccel;
  bool        m_UseAccelOut;

  double      m_BrakeCoeff[NBR_BRAKECOEFF+1];             // Brake coefficients
  int         m_LastBrakeCoefIndex;                       // Index of last brake coef.
  double      m_LastTargetSpeed;                          // Last target speed

  double      m_LastBrake;                                // Last brake command
  double      m_LastAccel;
  int         m_LastPosIdx;                               // Last brake position

private:
  void    ProcessOtherCars( int index, tCarElt* car, double spd, tSituation* s );
  void    AvoidOtherCars( int index, tCarElt* car, double k, double& carTargetSpd, tSituation* s, bool& inTraffic, bool& lapper );
  int     CalcGear( tCarElt* car, double& acc );
  float   getClutch(float flutch);
  float   startAutomatic(float clutch);
  double  ApplyAbs( tCarElt* car, double brake );
  double  filterBrake(double Brake);
  double  filterDrifting(double Acc);
  void    Meteorology();
  int     GetWeather();
  bool    CheckPitSharing();

  void            SetRandomSeed(unsigned int Seed);
  unsigned int    getRandom();

private:
  enum	// drive types
  {
    DT_RWD, DT_FWD, DT_4WD
  };

  enum
  {
    MAX_OPP = 100
  };

private:
  Shared*	m_pShared;
  MyTrack 	m_track;
  OptimisedPath	m_path[N_PATHS];
  PitPath	m_pitPath[N_PATHS];

  CarModel	m_cm;
  CarModel      m_cm2;

  tCarElt       *car;                           // Pointer to tCarElt struct.

  // Track variables.
  tTrack        *track;
  tWing         carWing[2];

  double        wheelz[4];

  char*         m_BotName;                      // Name of driver
  const char*   m_TeamName;                     // Name of team
  int           m_RaceNumber;                   // Race number
  const char*   m_CarType;

  double	FLY_HEIGHT;
  Array<double>	FACTORS;
  double	BUMP_MOD;
  int		SPDC_NORMAL;
  int		SPDC_TRAFFIC;
  int           SPDC_EXTRA;
  int           STEER_CTRL;
  double	STEER_K_ACC;
  double	STEER_K_DEC;
  double	STAY_TOGETHER;			// dist in m.
  double	AVOID_WIDTH;			// in m.
  double	PIT_ENTRY_OFFSET;		// dist in m.
  double	PIT_EXIT_OFFSET;		// dist in m.

  tSituation    *m_Situation;                   // situation
  int		m_driveType;
  double	m_gearUpRpm;			// for gear changing.

  double        m_XXX;

  float         m_SideScaleMu;
  float         m_SideScaleBrake;
  float         m_SideBorderOuter;
  float         m_SideBorderInner;
  bool          m_Rain;
  double        m_RainIntensity;
  double        m_ScaleMuRain;
  double        m_ScaleBrakeRain;
  float         m_DeltaAccel;                           //
  float         m_DeltaAccelRain;                       //
  int           m_WeatherCode;          // Track specific weather
  int           m_DryCode;              // Track specific dry weather

  bool          HasABS;
  bool          HasESP;
  bool          HasTCL;
  bool          HasTYC;

  double        m_TclRange;                            // TCL range
  double        m_TclSlip;                             // Max TCL slip
  double        m_TclFactor;                           // TCL scale
  float         m_AbsSlip;
  float         m_AbsRange;

  double	m_DriftAngle;                          // Drifting angle
  double	m_AbsDriftAngle;                       // fabs(Drifting angle)
  double	m_LastAbsDriftAngle;                   // Historie
  double	m_CosDriftAngle2;
  double	m_DriftFactor;                         // Drifting acceleration factor

  double        m_ClutchMax;
  double        m_ClutchDelta;
  double        m_ClutchRange;
  double        m_ClutchRelease;

  double        suspHeight;

  float         m_Shift;

  PidController	m_lineControl;			// controller for line error.
  PidController	m_velAngControl;		// controller for direction of car.
  PidController	m_angControl;			// controller for attack angle error.
  PidController m_speedController;      // controller for speed.
  double	m_prevYawError;
  double	m_prevLineError;
  double        m_Jumping;              // Car is jumping
  double        m_JumpOffset;           // Offset for calculation of jumps
  bool          m_FirstJump;
  int           m_Flying;               // Flag prepare landing
  int		m_nCars;
  int		m_myOppIdx;
  Opponent	*m_opp;                  // info about other cars.
  double	m_avgAY;
  bool          m_raceStart;
  double	m_avoidS;				// where we are LR->T (0..1).
  double	m_avoidSVel;
  double	m_avoidT;				// where we are L->R (-1..1).
  double	m_avoidTVel;
  double	m_avoidU;
  double	m_avoidV;
  double	m_attractor;			// where we want to be.
  int		m_followPath;			// path we want to follow;

  LinearRegression	m_accBrkCoeff;                      //
  double	m_brkCoeff[50];
  double	m_steerCoeff[STEER_SPD_MAX][STEER_K_MAX];

  int		m_lastB;
  double	m_lastBrk;
  double	m_lastTargV;
  double        m_maxbrkPressRatio;

  LearnedGraph	m_maxAccel;
  double	m_angle[SPD_N][K_N];

  LearnedGraph	m_steerGraph;
  AveragedData	m_steerAvg;

  double        m_FuelNeeded;
  double        m_RepairNeeded;
  //LinearAttractor	m_avoidX;
  //LinearAttractor	m_avoidY;

  Vec2d		m_lastPts[HIST];
  double	m_lastSpd;
  double	m_lastAng;

  SimpleStrategy  *m_Strategy;                           // Pit strategy

  bool            m_CrvComp;   			               // Crv compensation
  float           clutchtime;                            // Clutch timer.
  bool            Skilling;                              // Skilling on/off
  double          Skill;                                 // Skilling
  double          SkillMax;                              // Max skilling
  double          SkillDriver;                           // Individual skilling level
  double          SkillGlobal;                           // Global skilling level
  double          SkillScale;                            // Track skilling level
  double          SkillOffset;                           // Hymie skilling level
  double          DriverAggression;
  double          SkillAdjustTimer;                      // Timer
  double          SkillAdjustLimit;                      // Limit

  double          BrakeAdjustTarget;                     //
  double          BrakeAdjustPerc;                       //
  double          DecelAdjustTarget;                     //
  double          DecelAdjustPerc;                       //

  int             m_raceType;

  unsigned int    RandomSeed;                            // seed of generator

  double          CA;
  double          CW;
  double          CR;
  double          CX;
  double          CAFWING;
  double          CARWING;
  double          CAFGROUNDEFFECT;
  double          CARGROUNDEFFECT;

  float           TIREMU;
  float           TIREMUF;
  float           TIREMUR;

  float           (TDriver::*GET_DRIVEN_WHEEL_SPEED)();

  double          BrakeMaxPressRatio;
  double          BrakeMaxTqFront;
  double          BrakeMaxTqRear;
  double          BrakeForce;

  double          FrontWingAreaCd;
  double          RearWingAreaCd;

  bool            HasTrainFWD;
  bool            HasTrainRWD;

  bool            WingControl;			// Enable wing control
  double          WingCD;
  double          WingAngleFront;         // Front wing angle of attack
  double          WingAngleRear;          // Rear wing angle of attack
  double          WingAngleRearMin;       // Min rear wing angle of attack
  double          WingAngleRearMax;       // Max rear wing angle of attack
  double          WingAngleRearBrake;		// Air brake
  double          AirBrakeLatchTime;
  float           OversteerASR;

  static const float SHIFT_UP;
  static const float SHIFT_DOWN;
  static const float SHIFT_MARGIN;
  static const float CLUTCH_SPEED;
  static const float ABS_MINSPEED;
  static const float ABS_SLIP;
  static const float ABS_RANGE;
};

#endif
