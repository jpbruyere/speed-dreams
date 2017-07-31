/***************************************************************************

    file                 : driver.cpp
    created              : 9 Apr 2006
    copyright            : (C)2006 Tim Foden - (C)2014 Xavier BERTAUX
    email                : bertauxx@yahoo.fr
    version              : $Id: driver.cpp 5631 2014-12-27 21:32:55Z torcs-ng $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include <portability.h>
#include <robottools.h>

#include "Driver.h"
#include "Quadratic.h"
#include "Utils.h"
#include "Avoidance.h"
#include "GenericAvoidance.h"
#include "Strategy.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define FLY_COUNT		20

#define	STEER_SPD_IDX(x)	(int(floor((x) / 5)))
#define	STEER_K_IDX(k)		(MX(0, MN(int(20 + floor((k) * 500 + 0.5)), 40)))
#define DOUBLE_NORM_PI_PI(x) 				\
{ \
  while ((x) > PI) { (x) -= 2*PI; } \
  while ((x) < -PI) { (x) += 2*PI; } \
  }

//==========================================================================*
// Statics
//--------------------------------------------------------------------------*
//int TDriver::NBBOTS = MAX_NBBOTS;                   // Nbr of drivers/robots
//const char* TDriver::ROBOT_DIR = "drivers/shadow";  // Sub path to dll
//const char* TDriver::DEFAULTCARTYPE  = "car1-trb1"; // Default car type
const char* TDriver::robot_name="shadow_trb1";
int   TDriver::RobotType = 0;
bool  TDriver::AdvancedParameters = false;         // Advanced parameters
bool  TDriver::UseOldSkilling = false;             // Use old skilling
bool  TDriver::UseSCSkilling = false;              // Use supercar skilling
bool  TDriver::UseMPA1Skilling = false;            // Use mpa1 car skilling
float TDriver::SkillingFactor = 0.1f;              // Skilling factor for career-mode
bool  TDriver::UseBrakeLimit = false;              // Use brake limit
bool  TDriver::UseGPBrakeLimit = false;            // Use brake limit GP36
bool  TDriver::UseRacinglineParameters = false;    // Use racingline parameters
bool  TDriver::UseWingControl = false;             // Use wing control parameters
float TDriver::BrakeLimit = -6;                    // Brake limit
float TDriver::BrakeLimitBase = 0.025f;            // Brake limit base
float TDriver::BrakeLimitScale = 25;               // Brake limit scale
float TDriver::SpeedLimitBase = 0.025f;            // Speed limit base
float TDriver::SpeedLimitScale = 25;               // Speed limit scale
bool  TDriver::FirstPropagation = true;            // Initialize
bool  TDriver::Learning = false;                   // Initialize

const float TDriver::SHIFT_UP = 0.99f;             // [-] (% of rpmredline)
const float TDriver::SHIFT_DOWN = 120;
const float TDriver::SHIFT_MARGIN = 4.0f;          // [m/s] Avoid oscillating gear changes.
const float TDriver::CLUTCH_SPEED = 0.5f;
const float TDriver::ABS_MINSPEED = 3.0f;            // [m/s] Below this speed the ABS is disabled (numeric, division by small numbers).
const float TDriver::ABS_SLIP = 2.5f;
const float TDriver::ABS_RANGE = 5.0f;

//double TDriver::LengthMargin;                      // safety margin long.
//bool TDriver::Qualification;                       // Global flag
/*static const char *WheelSect[4] =                  // TORCS defined sections
{SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
static const char *WingSect[2] =
{SECT_FRNTWING, SECT_REARWING};*/
//==========================================================================*

//==========================================================================*
// Buffers
//
// Q: qualifying
// F: Free
// L: Avoid to left
// R: Avoid to right
//--------------------------------------------------------------------------*
/*#define BUFLEN 256
static char PathToWriteToBuffer[BUFLEN];         // for path we write to*/
//static char PathFilenameBuffer[256];          // for path and filename
/*static char TrackNameBuffer[BUFLEN];             // for track name
static char TrackLoadQualifyBuffer[BUFLEN];      // for track filename Q
static char TrackLoadBuffer[BUFLEN];             // for track filename F
static char TrackLoadLeftBuffer[BUFLEN];         // for track filename L
static char TrackLoadRightBuffer[BUFLEN];        // for track filename R
static char PitLoadBuffer[BUFLEN];               // for pit track filename F
static char PitLoadLeftBuffer[BUFLEN];           // for pit track filename L
static char PitLoadRightBuffer[BUFLEN];          // for pit track filename R*/
//==========================================================================*

//==========================================================================*
// Skilling: Randomness
//--------------------------------------------------------------------------*
#define RANDOM_SEED 0xfded
#define RANDOM_A    1664525
#define RANDOM_C    1013904223
//==========================================================================*

//==========================================================================*
// Skilling: Initialize Randomness
//--------------------------------------------------------------------------*
void TDriver::SetRandomSeed(unsigned int Seed)
{
  RandomSeed = Seed ? Seed : RANDOM_SEED;
  return;
}
//==========================================================================*

//==========================================================================*
// Skilling: Get Randomness
//--------------------------------------------------------------------------*
unsigned int TDriver::getRandom()
{
  RandomSeed = RANDOM_A * RandomSeed + RANDOM_C;
  return (RandomSeed >> 16);
}
//==========================================================================*

static const double	s_sgMin[] = { 0.00,  10 };
static const double	s_sgMax[] = { 0.03, 100 };
static const int	s_sgSteps[] = { 10,  18 };

TDriver::TDriver(int Index, const int robot_type):
  m_CarType(0),
  m_driveType(DT_RWD),
  m_gearUpRpm(9000),

  m_XXX(0),

  m_SideScaleMu(0.97f),
  m_SideScaleBrake(0.97f),
  m_SideBorderOuter(0.2f),
  m_SideBorderInner(0.2f),

  m_Rain(false),
  m_RainIntensity(0),
  m_ScaleMuRain(0.85f),
  m_ScaleBrakeRain(0.75f),
  m_DeltaAccel(0.05f),
  m_DeltaAccelRain(0.025f),
  m_WeatherCode(0),
  m_DryCode(0),

  HasABS(false),
  HasESP(false),
  HasTCL(false),
  HasTYC(false),

  m_TclRange(5.0),
  m_TclSlip(1.6f),
  m_TclFactor(1.0),
  m_AbsSlip(2.5f),
  m_AbsRange(5.0f),

  m_DriftAngle(0.0),
  m_AbsDriftAngle(0.0),
  m_LastAbsDriftAngle(0.0),
  m_CosDriftAngle2(1.0),
  m_DriftFactor(1.0),

  m_ClutchMax(0.5f),
  m_ClutchDelta(0.009f),
  m_ClutchRange(0.82f),
  m_ClutchRelease(0.5f),
  suspHeight(0),

  m_Shift(0.98f),

  m_prevYawError(0),
  m_prevLineError(0),
  m_Flying(0),
  m_avgAY(0),
  m_raceStart(false),
  m_avoidS(1),
  m_avoidT(0),
  m_followPath(PATH_NORMAL),
  m_lastB(0),
  m_lastBrk(0),
  m_lastTargV(0),
  m_maxbrkPressRatio(0.85f),
  m_maxAccel(0, 150, 30, 1),
  m_steerGraph(2, s_sgMin, s_sgMax, s_sgSteps, 0),
  m_steerAvg(19, 0.001f, 0.02f, 15, 20, 95),

  m_FuelNeeded(0),

  m_Strategy(NULL),

  m_raceType(0)
{
  INDEX = Index;

  switch(robot_type)
    {
    case SHADOW_TRB1:
      robot_name = "shadow_trb1";
      Frc = 0.95;
      break;

    case SHADOW_SC:
      robot_name = "shadow_sc";
      Frc = 0.90;
      break;

    case SHADOW_SRW:
      robot_name = "shadow_srw";
      WingControl = true;
      Frc = 0.95;
      break;

    case SHADOW_LS1:
      robot_name = "shadow_ls1";
      Frc = 0.85;
      break;

    case SHADOW_LS2:
      robot_name = "shadow_ls2";
      Frc = 0.85;
      break;

    case SHADOW_36GP:
      robot_name = "shadow_36GP";
      Frc = 0.75;
      break;

    case SHADOW_67GP:
      robot_name = "shadow_67GP";
      Frc = 0.75;
      break;

    case SHADOW_RS:
      robot_name = "shadow_rs";
      Frc = 0.98;
      break;

    case SHADOW_LP1:
      robot_name = "shadow_lp1";
      Frc = 0.90;
      break;

    case SHADOW_MPA1:
      robot_name = "shadow_mpa1";
      Frc = 0.88;
      break;

    case SHADOW_MPA11:
      robot_name = "shadow_mpa11";
      Frc = 0.88;
      break;

    case SHADOW_MPA12:
      robot_name = "shadow_mpa12";
      Frc = 0.88;
      break;
    }

  for( int i = 0; i < 50; i++ )
    {
      m_brkCoeff[i] = 0.5;//0.12;
    }

  for( int i = 0; i < STEER_SPD_MAX; i++ )
    {
      for( int j = 0; j < STEER_K_MAX; j++ )
        {
          m_steerCoeff[i][j] = 0;
        }
    }

  memset( m_angle, 0, sizeof(m_angle) );
}

TDriver::~TDriver()
{
  LogSHADOW.debug("\n#TDriver::~TDriver() >>>\n\n");
  delete [] m_opp;

  if (m_Strategy != NULL)
    delete m_Strategy;

  LogSHADOW.debug("\n#<<< TDriver::~TDriver()\n\n");
}

static void* MergeParamFile( void* hParams, const char* fileName )
{
  void*	hNewParams = GfParmReadFile(fileName, GFPARM_RMODE_STD);
  if( hNewParams == NULL )
    return hParams;

  if( hParams == NULL )
    return hNewParams;

  return GfParmMergeHandles(hParams, hNewParams,
                            GFPARM_MMODE_SRC    | GFPARM_MMODE_DST |
                            GFPARM_MMODE_RELSRC | GFPARM_MMODE_RELDST);
}

void TDriver::SetShared( Shared* pShared )
{
  m_pShared = pShared;
}

//==========================================================================*

//==========================================================================*
// Adjust skilling depending on the car types to drive
//--------------------------------------------------------------------------*
void TDriver::AdjustSkilling(void* pCarHandle)
{
  // Adjust skilling ...
  if ((Skill < 0) || (!Skilling))
    {
      Skilling = false;
      Skill = 1.0;
      LogSHADOW.debug("#No skilling: Skill %g\n", Skill);
      //Param.Tmp.oSkill = oSkill;
    }
  else
    {
      SkillOffset = MAX(0.0,MIN(10.0, GfParmGetNum(pCarHandle, SECT_PRIV,
                                                   "offset skill", (char *) NULL, (float) SkillOffset)));
      LogSHADOW.debug("#SkillOffset: %g\n", SkillOffset);
      SkillScale = MAX(0.0,MIN(10.0, GfParmGetNum(pCarHandle, SECT_PRIV,
                                                  "scale skill", (char *) NULL, (float)SkillScale)));
      LogSHADOW.debug("#SkillScale: %g\n", SkillScale);

      //LookAhead = LookAhead / (1+SkillGlobal/24);
      //LookAheadFactor = LookAheadFactor / (1+SkillGlobal/24);

      //CalcSkilling();

      //Param.Tmp.oSkill = 1.0 + oSkill;
      LogSHADOW.debug("\n#>>>Skilling: Skill %g SkillGlobal %g SkillDriver %g LookAhead %g "
                      "LookAheadFactor %g effSkill:%g\n\n", Skill, SkillGlobal, SkillDriver/*,
                                                        LookAhead, LookAheadFactor, Param.Tmp.oSkill*/);
    }
};

//==========================================================================*
//==========================================================================*
// Get skilling parameters
//--------------------------------------------------------------------------*
void TDriver::GetSkillingParameters()
{
  // Global skilling from Andrew Sumner ...
  // Check if skilling is enabled
  int SkillEnabled = 0;
  char	PathFilenameBuffer[512];

  // Do not skill if optimisation is working
  //if (!GeneticOpti)
  {
    snprintf(PathFilenameBuffer, 512, "%sdrivers/%s/default.xml", GetDataDir(), robot_name);
    LogSHADOW.debug("#PathFilename: %s\n", PathFilenameBuffer); // itself
    void* SkillHandle = GfParmReadFile(PathFilenameBuffer, GFPARM_RMODE_REREAD);
    if (SkillHandle)
      {
        SkillEnabled = (int) MAX(0,MIN(1,(int) GfParmGetNum(SkillHandle, "skilling", "enable", (char *) NULL, 0.0)));
        LogSHADOW.debug("#SkillEnabled %d\n", SkillEnabled);
        //TeamEnabled = GfParmGetNum(SkillHandle, "team", "enable", 0, (float)TeamEnabled) != 0;
        //LogSHADOW.debug("#TeamEnabled %d\n", TeamEnabled);
      }

    GfParmReleaseHandle(SkillHandle);
  }

  if (SkillEnabled > 0)                          // If skilling is enabled
    {                                              // Get Skill level
      Skilling = true;                           // of TORCS-Installation
      LogSHADOW.debug("#Skilling: On\n");

      void* SkillHandle = NULL;

      snprintf(PathFilenameBuffer, 256, "%sconfig/raceman/extra/skill.xml", GetLocalDir());
      LogSHADOW.debug("#skill.xml: %s\n", PathFilenameBuffer);
      SkillHandle = GfParmReadFile(PathFilenameBuffer, GFPARM_RMODE_REREAD);
      if (SkillHandle)
        {
          SkillGlobal = MAX(0.0,MIN(30.0,GfParmGetNum(SkillHandle, "skill", "level", (char *) NULL, 30.0)));
          LogSHADOW.debug("#LocalDir: SkillGlobal: %g\n", SkillGlobal);
        }
      else
        {
          snprintf(PathFilenameBuffer, 256, "%sconfig/raceman/extra/skill.xml", GetDataDir());
          LogSHADOW.debug("#skill.xml: %s\n", PathFilenameBuffer);

          SkillHandle = GfParmReadFile(PathFilenameBuffer, GFPARM_RMODE_REREAD);
          if (SkillHandle)
            {
              SkillGlobal = MAX(0.0,MIN(30.0, GfParmGetNum(SkillHandle, "skill", "level", (char *) NULL, 30.0)));
              LogSHADOW.debug("#DataDir: SkillGlobal: %g\n", SkillGlobal);
            }
        }

      // Get individual skilling
      snprintf(PathFilenameBuffer, 256, "%sdrivers/%s/%d/skill.xml", GetDataDir(), robot_name, INDEX);
      LogSHADOW.debug("#PathFilename: %s\n", PathFilenameBuffer); // itself
      SkillHandle = GfParmReadFile(PathFilenameBuffer, GFPARM_RMODE_REREAD);

      if (SkillHandle)
        {
          SkillDriver = GfParmGetNum(SkillHandle, "skill", "level", 0, 0.0);
          SkillDriver = MIN(1.0, MAX(0.0, SkillDriver));
          LogSHADOW.debug("#SkillDriver: %g\n", SkillDriver);

          DriverAggression = GfParmGetNum(SkillHandle, "skill", "aggression", (char *)NULL, 0.0);
          LogSHADOW.debug("#DriverAggression: %g\n", DriverAggression);

          SkillDriver = (float)((SkillGlobal + SkillDriver * 2) * (1.0 + SkillDriver));
          Skill = (float)((SkillGlobal + SkillDriver * 2) * (1.0 + SkillDriver));
        }

      GfParmReleaseHandle(SkillHandle);
    }
  else
    {
      Skilling = false;
      LogSHADOW.debug("#Skilling: Off\n");
    }
  // ... Global skilling from Andrew Sumner
};

// Called for every track change or new race.
void TDriver::InitTrack( tTrack* pTrack, void* pCarHandle, void** ppCarParmHandle, tSituation* pS )
{
  void*	hCarParm = 0;
  char	buf[1024];
  char	trackName[256];
  track = pTrack;

  // Initialize the base param path
  //const char* BaseParamPath = TDriver::robot_name;
  //const char* PathFilename = PathFilenameBuffer;

  SkillGlobal = Skill = DecelAdjustPerc = DriverAggression = SkillAdjustLimit = 0.0;
  GetSkillingParameters();
  LogSHADOW.debug("#Skill Driver = %f\n", SkillDriver);

  strncpy( trackName, strrchr(track->filename, '/') + 1, sizeof(trackName) );
  *strrchr(trackName, '.') = '\0';

  if (track->length < 2000)
    RtTeamManagerLaps(3);
  else if (track->length < 3000)
    RtTeamManagerLaps(2);
  //
  //	set up race type array.
  //

  const char*	raceType[] = { "practice", "qualify", "race" };
  m_raceType = pS->_raceType;
  LogSHADOW.info("#RaceType = %d\n", m_raceType);

  m_WeatherCode = GetWeather();

  //
  //	set up the base param path.
  //

  snprintf(buf, BUFSIZE, "drivers/%s/%s/default.xml", robot_name, m_CarType);
  LogSHADOW.info("#Path for default.xml = drivers/%s/%s/default.xml\n", robot_name, m_CarType);

  *ppCarParmHandle = GfParmReadFile(buf, GFPARM_RMODE_STD);

  //
  //	ok, lets read/merge the car parms.
  //

  Meteorology();

  if (!m_Rain)
    {
      // default params for car type (e.g. clkdtm)
      snprintf( buf, BUFSIZE, "drivers/%s/%s/%s.xml", robot_name, m_CarType, trackName );
      LogSHADOW.info("#Override params for car type with params of track: %s\n", buf);
      hCarParm = MergeParamFile(hCarParm, buf);
    }

  if (m_Rain)
    {
      // Override params for car type with params of track and weather
      snprintf(buf, BUFSIZE, "drivers/%s/%s/%s-%d.xml", robot_name, m_CarType, trackName, m_WeatherCode);
      LogSHADOW.info("#Override params for car type with params of track and weather: %s\n", buf);
      hCarParm = MergeParamFile(hCarParm, buf);
    }

  // override params for car type on track of specific race type.
  snprintf( buf, sizeof(buf), "drivers/%s/%s/track-%s-%s.xml", robot_name, m_CarType, trackName, raceType[pS->_raceType] );
  LogSHADOW.info("#Override params for car type with params of track and race type: %s\n", buf);
  hCarParm = MergeParamFile(hCarParm, buf);

  // setup the car param handle to be returned.
  *ppCarParmHandle = hCarParm;
  LogSHADOW.debug("Load track settings ....\n");

  // get the private parameters now.
  m_ScaleMuRain = (double)(GfParmGetNum(hCarParm, SECT_PRIV, PRV_RAIN_MU, NULL, (tdble) m_ScaleMuRain));
  LogSHADOW.info("#Scale Mu Rain: %g\n", m_ScaleMuRain);

  m_cm.AERO = (int)GfParmGetNum(hCarParm, SECT_PRIV, PRV_AERO_MOD, 0, 0);
  m_cm.MU_SCALE = GfParmGetNum(hCarParm, SECT_PRIV, PRV_MU_SCALE, NULL, 0.9f);
  m_cm.SKILL = SkillDriver / 10;
  if (m_raceType == 1)
    {
      m_cm.MU_SCALE = m_cm.MU_SCALE + 0.02;
      LogSHADOW.info("#Scale Mu Qualification\n");
    }

  m_cm.KZ_SCALE = GfParmGetNum(hCarParm, SECT_PRIV, PRV_KZ_SCALE, NULL, 0.43f);
  m_cm.BUMP_FACTOR = GfParmGetNum(hCarParm, SECT_PRIV, PRV_BUMP_FACTOR, NULL, 1.0);
  m_cm.BUMP_FACTORLEFT = GfParmGetNum(hCarParm, SECT_PRIV, PRV_BUMP_FACTOR_LEFT, NULL, 1.0);
  m_cm.BUMP_FACTORRIGHT = GfParmGetNum(hCarParm, SECT_PRIV, PRV_BUMP_FACTOR_RIGHT, NULL, 1.0);
  m_cm.NEEDSINLONG = (GfParmGetNum(hCarParm, SECT_PRIV, PRV_NEED_SIN, NULL, 0) != 0);
  m_cm.USEDACCEXIT = (GfParmGetNum(hCarParm, SECT_PRIV, PRV_USED_ACC, NULL, 0) != 0);
  m_cm.BRAKESCALE = GfParmGetNum(hCarParm, SECT_PRIV, PRV_BRAKESCALE, NULL, 1.0);

  FACTORS.RemoveAll();

  for( int i = 0; ; i++ )
  {
      snprintf( buf, sizeof(buf), "%s %d", PRV_FACTOR, i );
      double	factor = GfParmGetNum(hCarParm, SECT_PRIV, buf, 0, -1);
      LogSHADOW.debug("FACTOR %g\n", factor );
      if( factor == -1 )
        break;

      FACTORS.Add( factor );
  }

  if( FACTORS.GetSize() == 0 )
    FACTORS.Add( 1.005 );


  FLY_HEIGHT       = GfParmGetNum(hCarParm, SECT_PRIV, PRV_FLY_HEIGHT, "m", 0.15f);
  BUMP_MOD         = GfParmGetNum(hCarParm, SECT_PRIV, PRV_BUMP_MOD, 0, 0);
  SPDC_NORMAL      = int(GfParmGetNum(hCarParm, SECT_PRIV, PRV_SPDC_NORMAL, 0, 2));
  SPDC_TRAFFIC     = int(GfParmGetNum(hCarParm, SECT_PRIV, PRV_SPDC_TRAFFIC, 0, 2));
  SPDC_EXTRA       = int(GfParmGetNum(hCarParm, SECT_PRIV, PRV_SPDC_EXTRA, 0, 3));
  STEER_CTRL       = int(GfParmGetNum(hCarParm, SECT_PRIV, PRV_STEER_CTRL, 0, 0));
  AVOID_WIDTH      = GfParmGetNum(hCarParm, SECT_PRIV, PRV_AVOID_WIDTH, 0, 0.5);
  STAY_TOGETHER    = GfParmGetNum(hCarParm, SECT_PRIV, PRV_STAY_TOGETHER, 0, 0);
  STEER_K_ACC      = GfParmGetNum(hCarParm, SECT_PRIV, PRV_STEER_K_ACC, 0, 0);
  STEER_K_DEC      = GfParmGetNum(hCarParm, SECT_PRIV, PRV_STEER_K_DEC, 0, 0);
  PIT_ENTRY_OFFSET = GfParmGetNum(hCarParm, SECT_PRIV, PRV_PIT_ENTRY_OFFS, 0, 0);
  PIT_EXIT_OFFSET  = GfParmGetNum(hCarParm, SECT_PRIV, PRV_PIT_EXIT_OFFS, 0, 0);

  m_ClutchDelta = GfParmGetNum(hCarParm, SECT_PRIV, PRV_CLUTCH_DELTA,0,(float)m_ClutchDelta);
  LogSHADOW.debug("#m_ClutchDelta %g\n", m_ClutchDelta);

  m_ClutchMax = GfParmGetNum(hCarParm, SECT_PRIV, PRV_CLUTCH_MAX,0,(float)m_ClutchMax);
  LogSHADOW.debug("#m_ClutchMax %g\n",m_ClutchMax);

  m_ClutchRange = GfParmGetNum(hCarParm, SECT_PRIV, PRV_CLUTCH_RANGE,0,(float)m_ClutchRange);
  LogSHADOW.debug("#m_ClutchRange %g\n",m_ClutchRange);

  m_ClutchRelease = GfParmGetNum(hCarParm, SECT_PRIV, PRV_CLUTCH_RELEASE,0, (float)m_ClutchRelease);
  LogSHADOW.debug("#m_ClutchRelease %g\n",m_ClutchRelease);

  m_Shift = GfParmGetNum(hCarParm, SECT_PRIV, PRV_SHIFT, 0, (float)m_Shift);
  LogSHADOW.debug("#m_Shift %g\n",m_Shift);

  m_TclSlip = GfParmGetNum(hCarParm, SECT_PRIV, PRV_TCL_SLIP, 0, m_TclSlip);
  LogSHADOW.debug("#m_TclSlip %g\n",m_TclSlip);

  m_TclRange = GfParmGetNum(hCarParm, SECT_PRIV, PRV_TCL_RANGE, 0, m_TclRange);
  LogSHADOW.debug("#m_TclRange %g\n",m_TclRange);

  m_TclFactor = GfParmGetNum(hCarParm, SECT_PRIV, PRV_TCL_FACTOR, 0, m_TclFactor);
  LogSHADOW.debug("#m_TclFactor %g\n",m_TclFactor);

  m_AbsSlip = GfParmGetNum(hCarParm, SECT_PRIV, PRV_ABS_SLIP, 0, m_AbsSlip);
  LogSHADOW.debug("#m_AbsSlip %g\n",m_AbsSlip);

  m_AbsRange = GfParmGetNum(hCarParm, SECT_PRIV, PRV_ABS_RANGE, 0, m_AbsRange);
  LogSHADOW.debug("#m_AbsRange %g\n",m_AbsRange);

  m_DeltaAccel = GfParmGetNum(hCarParm, SECT_PRIV, PRV_ACCEL_DELTA , 0, (float)m_DeltaAccel);
  m_DeltaAccelRain = GfParmGetNum(hCarParm, SECT_PRIV, PRV_ACCEL_DELTA_RAIN, 0, (float)m_DeltaAccelRain);
  LogSHADOW.debug("FLY_HEIGHT %g\n", FLY_HEIGHT );
  LogSHADOW.debug( "BUMP_MOD %d\n", BUMP_MOD );

  /*m_TclSlip = GfParmGetNum(hCarParm, SECT_PRIV, PRV_TCL_SLIP , 0, (float)m_TclSlip);
  m_TclRange = GfParmGetNum(hCarParm, SECT_PRIV, PRV_TCL_RANGE , 0, (float)m_TclRange);
  m_AbsSlip = GfParmGetNum(hCarParm, SECT_PRIV, PRV_ABS_SLIP , 0, (float)m_AbsSlip);
  m_AbsRange = GfParmGetNum(hCarParm, SECT_PRIV, PRV_ABS_RANGE , 0, (float)m_AbsRange);*/
  LogSHADOW.debug( "#Car TCL SLIP = %g - TCL RANGE = %g - ABS SLIP = %g - ABS RANGE = %g\n", m_TclSlip, m_TclRange, m_AbsSlip, m_AbsRange );

  AdjustBrakes(hCarParm);

  const char *enabling;

  HasTYC = false;
  enabling = GfParmGetStr(hCarParm, SECT_FEATURES, PRM_TIRETEMPDEG, VAL_NO);
  if (strcmp(enabling, VAL_YES) == 0)
    {
      HasTYC = true;
      LogSHADOW.info("#Car has TYC yes\n");
    }
  else
    LogSHADOW.info("#Car has TYC no\n");

  HasABS = false;
  enabling = GfParmGetStr(hCarParm, SECT_FEATURES, PRM_ABSINSIMU, VAL_NO);
  if (strcmp(enabling, VAL_YES) == 0)
    {
      HasABS = true;
      LogSHADOW.info("#Car has ABS yes\n");
    }
  else
    LogSHADOW.info("#Car has ABS no\n");

  HasESP = false;
  enabling = GfParmGetStr(hCarParm, SECT_FEATURES, PRM_ESPINSIMU, VAL_NO);
  if (strcmp(enabling, VAL_YES) == 0)
    {
      HasESP = true;
      LogSHADOW.info("#Car has ESP yes\n");
    }
  else
    LogSHADOW.info("#Car has ESP no\n");

  HasTCL = false;
  enabling = GfParmGetStr(hCarParm, SECT_FEATURES, PRM_TCLINSIMU, VAL_NO);
  if (strcmp(enabling, VAL_YES) == 0)
    {
      HasTCL = true;
      LogSHADOW.info("#Car has TCL yes\n");
    }
  else
    LogSHADOW.info("#Car has TCL no\n");

  // For test of simu options override switches here
  /*
    oCarHasABS = true;
    oCarHasTCL = true;
    oCarHasESP = true;
    */

  MyTrack::SideMod	sideMod;
  sideMod.side = -1;
  const char*	pStr = GfParmGetStr(hCarParm, SECT_PRIV, PRV_SIDE_MOD, "");
  if( pStr == 0 || sscanf(pStr, "%d , %d , %d", &sideMod.side, &sideMod.start, &sideMod.end) != 3 )
    {
      sideMod.side = -1;
    }

  LogSHADOW.debug( "SIDE MOD %d %d %d\n", sideMod.side, sideMod.start, sideMod.end );
  LogSHADOW.debug( "STAY_TOGETHER %g\n", STAY_TOGETHER );

  m_track.NewTrack( track, false, &sideMod );

  m_Situation = pS;

  // Create pitting strategy
  m_Strategy = new SimpleStrategy();
  m_Strategy->Driver = this;

  m_Strategy->setFuelAtRaceStart(track, ppCarParmHandle, pS, INDEX);                               //   strategy
  m_AbsSlip = ABS_SLIP;
}

// Start a new race.
void TDriver::NewRace( tCarElt* pCar, tSituation* pS )
{
  LogSHADOW.debug("Shadow NewRace ...\n");
  m_nCars = pS->_ncars;
  this->car = pCar;
  m_myOppIdx = -1;
  clutchtime = 0.0f;

  // Skilling from Andrew Sumner ...
  SkillAdjustTimer = -1;
  SkillAdjustLimit = 0.0;
  BrakeAdjustTarget = DecelAdjustTarget = 1.0f;
  BrakeAdjustPerc = DecelAdjustPerc = 1.0f;

  m_opp = new Opponent[m_nCars];
  for( int i = 0; i < m_nCars; i++ )
    {
      m_opp[i].Initialise( &m_track, pS->cars[i] );

      if( pS->cars[i] == pCar )
        m_myOppIdx = i;
    }

  m_cm.MASS = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_MASS, NULL, 1000.0);
  suspHeight = GfParmGetNum(car->_carHandle, SECT_REARLFTSUSP, PRM_SUSPCOURSE, NULL, 0.0f);

  initCa();

  LogSHADOW.debug( "CA %g   CA_FW %g   CA_RW %g   CA_GE %g\n", m_cm.CA, m_cm.CA_FW, m_cm.CA_RW, m_cm.CA_GE );

  m_cm.TYRE_MU   = 9999;
  m_cm.TYRE_MU_F = 9999;
  m_cm.TYRE_MU_R = 9999;

  initTireMu();

  LogSHADOW.debug( "CARMASS %g   TYRE_MU %g   TYRE_MU_F %g   TYRE_MU_R %g \n",
                   m_cm.MASS, m_cm.TYRE_MU, m_cm.TYRE_MU_F, m_cm.TYRE_MU_R );

  LogSHADOW.debug( "MU_SC %g   KZ_SCALE %g   FLY_HEIGHT %g\n",
                   m_cm.MU_SCALE, m_cm.KZ_SCALE, FLY_HEIGHT );

  char*	pTrackName = strrchr(m_track.GetTrack()->filename, '/') + 1;
  char	buf[1024];

  snprintf( buf, 256, "drivers/%s/%s.spr", robot_name, pTrackName );

  initCw();
  LogSHADOW.debug(" CW = %.3f\n", m_cm.CD_BODY);
  initCR();

  initDriveTrain();
  initWheelRadius();
  initWheelPos();
  initBrake();

  if(HasTYC)
    {
      m_cm.HASTYC = true;
      m_cm.TYRECONDITIONFRONT  = TyreConditionFront();
      m_cm.TYRECONDITIONREAR   = TyreConditionRear();
      LogSHADOW.debug("Tyre Front = %d - Tyre Rear = %d\n", m_cm.TYRECONDITIONFRONT, m_cm.TYRECONDITIONREAR);
    }
  else
    m_cm.HASTYC = false;

  m_cm.BRAKEFORCE = BrakeForce;

  m_cm.FUEL = 0;//pCar->_fuel;

  m_cm.WIDTH = car->_dimension_y;

  m_cm2 = m_cm;
  m_cm2.MU_SCALE = MN(1.5, m_cm.MU_SCALE);

  LogSHADOW.debug( "SPDC N %d    SPDC T %d\n", SPDC_NORMAL, SPDC_TRAFFIC );

  if( m_pShared->m_path[PATH_NORMAL].GetFactors() != FACTORS || m_pShared->m_pTrack != m_track.GetTrack())
    {
      LogSHADOW.debug("m_Pshared first test\n");
      if( m_pShared->m_pTrack != m_track.GetTrack())
        {
          m_pShared->m_pTrack = m_track.GetTrack();
          m_pShared->m_teamInfo.Empty();
        }

      double	w = AVOID_WIDTH;

      m_pShared->m_path[PATH_NORMAL].SetFactors( FACTORS );
      m_pShared->m_path[PATH_NORMAL].MakeSmoothPath( &m_track, m_cm, ClothoidPath::Options((int)BUMP_MOD));
      m_pShared->m_path[PATH_LEFT].SetFactors( FACTORS );
      m_pShared->m_path[PATH_LEFT].MakeSmoothPath( &m_track, m_cm2, ClothoidPath::Options((int)BUMP_MOD, 999, -w));
      m_pShared->m_path[PATH_RIGHT].SetFactors( FACTORS );
      m_pShared->m_path[PATH_RIGHT].MakeSmoothPath( &m_track, m_cm2, ClothoidPath::Options((int)BUMP_MOD, -w, 999));
    }

  LogSHADOW.debug("m_Pshared passed\n");

  m_path[PATH_NORMAL] = m_pShared->m_path[PATH_NORMAL];
  m_path[PATH_NORMAL].CalcMaxSpeeds( m_cm );
  m_path[PATH_NORMAL].PropagateBreaking( m_cm );

  m_path[PATH_LEFT] = m_pShared->m_path[PATH_LEFT];
  m_path[PATH_LEFT].CalcMaxSpeeds( m_cm2 );
  m_path[PATH_LEFT].PropagateBreaking( m_cm2 );

  m_path[PATH_RIGHT] = m_pShared->m_path[PATH_RIGHT];
  m_path[PATH_RIGHT].CalcMaxSpeeds( m_cm2 );
  m_path[PATH_RIGHT].PropagateBreaking( m_cm2 );

  LogSHADOW.debug("m_path passed\n");

  m_pitPath[PATH_NORMAL].MakePath( car, &m_path[PATH_NORMAL], m_cm,  PIT_ENTRY_OFFSET, PIT_EXIT_OFFSET );
  m_pitPath[PATH_LEFT].  MakePath( car, &m_path[PATH_LEFT],   m_cm2, PIT_ENTRY_OFFSET, PIT_EXIT_OFFSET );
  m_pitPath[PATH_RIGHT]. MakePath( car, &m_path[PATH_RIGHT],  m_cm2, PIT_ENTRY_OFFSET, PIT_EXIT_OFFSET );

  LogSHADOW.debug("m_pit Path passed\n");

  m_Flying = 0;
  m_raceStart = true;

  m_avoidS = 0;
  m_avoidT = 0;

  m_accBrkCoeff.Clear();
  m_accBrkCoeff.Sample( 0, 0 );
  m_accBrkCoeff.Sample( 1, 0.5 );

  TeamInfo::Item*	pItem = new TeamInfo::Item();
  pItem->index = car->index;
  pItem->teamName = car->_teamname;
  pItem->damage = car->_dammage;
  pItem->pitState = TeamInfo::PIT_NOT_SHARED;
  pItem->pOther = 0;
  pItem->pCar = car;
  m_pShared->m_teamInfo.Add( car->index, pItem );
  LogSHADOW.debug("End Shadow NewRace\n");
}

void TDriver::GetPtInfo( int path, double pos, PtInfo& pi ) const
{

  if( m_Strategy->needPitstop(car, m_Situation) && m_pitPath[path].ContainsPos(pos) )
    m_pitPath[path].GetPtInfo( pos, pi );
  else
    m_path[path].GetPtInfo( pos, pi );
}

void InterpPtInfo( PtInfo& pi0, const PtInfo& pi1, double t )
{
  pi0.k	= Utils::InterpCurvature(pi0.k, pi1.k, t);
  double	deltaOAng = pi1.oang - pi0.oang;
  NORM_PI_PI(deltaOAng);
  pi0.oang = pi0.oang + deltaOAng * t;
  pi0.offs = pi0.offs * (1 - t) + pi1.offs * t;
  pi0.spd	 = pi0.spd  * (1 - t) + pi1.spd  * t;
}

void TDriver::GetPosInfo( double pos, PtInfo& pi, double u, double v ) const
{
  GetPtInfo( PATH_NORMAL, pos, pi );

  PtInfo	piL, piR;

  if( u != 1 )
    {
      GetPtInfo( PATH_LEFT,  pos, piL );
      GetPtInfo( PATH_RIGHT, pos, piR );

      double	s = u;
      double	t = (v + 1) * 0.5;

      InterpPtInfo( piL, pi, s );
      InterpPtInfo( piR, pi, s );

      pi = piL;

      InterpPtInfo( pi, piR, t );
    }
}

void TDriver::GetPosInfo( double pos, PtInfo& pi ) const
{
  GetPosInfo(pos, pi, m_avoidS, m_avoidT);
}

double TDriver::CalcPathTarget( double pos, double offs, double s ) const
{
  PtInfo	pi, piL, piR;
  GetPtInfo( PATH_NORMAL, pos, pi );
  GetPtInfo( PATH_LEFT, pos, piL );
  GetPtInfo( PATH_RIGHT, pos, piR );

  InterpPtInfo( piL, pi, s );
  InterpPtInfo( piR, pi, s );

  double	t = (offs - piL.offs) / (piR.offs - piL.offs);

  return MX(-1, MN(t, 1)) * 2 - 1;
}

double TDriver::CalcPathTarget( double pos, double offs ) const
{
  return CalcPathTarget(pos, offs, m_avoidS);
}

Vec2d TDriver::CalcPathTarget2( double pos, double offs ) const
{
  PtInfo	pi, piL, piR;
  GetPtInfo( PATH_NORMAL, pos, pi );
  GetPtInfo( PATH_LEFT, pos, piL );
  GetPtInfo( PATH_RIGHT, pos, piR );

  double	s = m_avoidS;

  InterpPtInfo( piL, pi, s );
  InterpPtInfo( piR, pi, s );

  double	t = (offs - piL.offs) / (piR.offs - piL.offs);

  return Vec2d(MX(-1, MN(t, 1)) * 2 - 1, 1);
}

double TDriver::CalcPathOffset( double pos, double s, double t ) const
{
  PtInfo	pi, piL, piR;
  GetPtInfo( PATH_NORMAL, pos, pi );
  GetPtInfo( PATH_LEFT, pos, piL );
  GetPtInfo( PATH_RIGHT, pos, piR );

  InterpPtInfo( piL, pi, s );
  InterpPtInfo( piR, pi, s );

  InterpPtInfo( piL, piR, (t + 1) * 0.5 );

  return piL.offs;
}

void TDriver::CalcBestPathUV( double pos, double offs, double& u, double& v ) const
{
  PtInfo	pi, piL, piR;
  GetPtInfo( PATH_NORMAL, pos, pi );

  if( fabs(offs - pi.offs) < 0.01 )
    {
      u = 1;
      v = 0;
      return;
    }

  GetPtInfo( PATH_LEFT,  pos, piL );
  GetPtInfo( PATH_RIGHT, pos, piR );

  double	doffs = offs - pi.offs;
  if( doffs < 0 )
    {
      double	den = piL.offs - pi.offs;
      if( fabs(den) > 0.001 )
        u = 1 - MN(1, doffs / den);
      else
        u = 0;
      v = -1;
    }
  else
    {
      double	den = piR.offs - pi.offs;
      if( fabs(den) > 0.001 )
        u = 1 - MN(1, doffs / den);
      else
        u = 0;
      v = 1;
    }
}

double TDriver::CalcBestSpeed( double pos, double offs ) const
{
  double	u, v;
  CalcBestPathUV( pos, offs, u, v );

  PtInfo	pi;
  GetPosInfo( pos, pi, u, v );

  return pi.spd;
}

void TDriver::GetPathToLeftAndRight( const CarElt* pCar, double& toL, double& toR ) const
{
  double	pos = pCar->_distFromStartLine;
  double	offs = -pCar->_trkPos.toMiddle;

  PtInfo	pi;
  GetPtInfo( PATH_LEFT, pos, pi );
  toL = -(pi.offs - offs);
  GetPtInfo( PATH_RIGHT, pos, pi );
  toR = pi.offs - offs;
}

double CalcMaxSlip( double fRatio )
{
  //	double	stmp = MIN(slip, 1.5f);
  //	Bx = wheel->mfB * stmp;
  //	F = sin(wheel->mfC * atan(Bx * (1.0f - wheel->mfE) +
  //								wheel->mfE * atan(Bx))) *
  //			(1.0f + stmp * simSkidFactor[car->carElt->_skillLevel]);
  //
  //	f = sin(C + atan(Bx * (1 - E) + E * atan(Bx)))
  //
  //	s = 0, f = 0
  //	s = 0.75, f = 0.75
  //	s = 1.25, f = 1
  //	s = 10, f = 1
  //
  //	sa = yaw - velAng;
  //	slip = tan(sa) * vel;
  //	sr = slip / vel == tan(sa) * vel / vel == tan(sa)
  return 0;
}

double TDriver::SteerAngle0( tCarElt* car, PtInfo& pi, PtInfo& aheadPi )
{
  // work out current car speed.
  double	spd0 = hypot(car->_speed_x, car->_speed_y);

  // get current pos on track.
  double	pos = m_track.CalcPos(car);//, car->_dimension_x * 0.025);

  // look this far ahead.
  double	aheadDist = car->_dimension_x * 0.5 + spd0 * 0.02;
  double	aheadPos = m_track.CalcPos(car, aheadDist);

  // get info about pts on track.
  GetPosInfo( pos, pi );
  GetPosInfo( aheadPos, aheadPi );
  LogSHADOW.debug("SHADOW SteerAngle0\n");

  PtInfo	piOmega;
  double	aheadOmega = car->_dimension_x * 0.5 + spd0 * 0.02;// * 10;
  double	aheadOmegaPos = m_track.CalcPos(car, aheadOmega);
  GetPosInfo( aheadOmegaPos, piOmega );

  // work out basic steering angle.
  double velAng = atan2(car->_speed_Y, car->_speed_X);
  double	angle = aheadPi.oang - car->_yaw;
  double	delta = pi.offs + car->_trkPos.toMiddle;

  NORM_PI_PI(angle);
  double	avgK = (pi.k + piOmega.k) * 0.5;
  double omega = car->_speed_x * avgK;//aheadPi.k;
  double	o2 = (aheadPi.k - pi.k) * spd0 / aheadDist;
  //	if( fabs(pi.k - aheadPi.k) < 0.0025 )
  {
    //		double omega = spd0 * avgK;//aheadPi.k;
    //		angle += 0.02 * (omega - car->_yaw_rate);
    //		angle += 0.04 * (omega - car->_yaw_rate);
    //		angle += 0.05 * (omega - car->_yaw_rate);
    angle += 0.08 * (omega - car->_yaw_rate);
    //		angle += 0.10 * (omega - car->_yaw_rate);
    //		angle += 0.12 * (omega - car->_yaw_rate);
  }

  angle += o2 * 0.08;

  if( car->_accel_x > 0 )
    angle += avgK * STEER_K_ACC;
  else
    angle += avgK * STEER_K_DEC;

  velAng = atan2(car->_speed_Y, car->_speed_X);
  double	ang = car->_yaw - velAng;
  NORM_PI_PI(ang);

  int	k = int(floor((pi.k - K_MIN) / K_STEP));
  int	s = int(floor((spd0 - SPD_MIN) / SPD_STEP));

  if( k >= 0 && k < K_N && s >= 0 && s < SPD_N )
    {
      double	ae = 0;
      ae = m_angle[s][k] - ang;
    }

  // control offset from path.
  m_lineControl.m_p = 1.0;
  m_lineControl.m_d = 10;
  const double SC = 0.15;                             //0.15;//0.15;//0.2;//0.15;
  angle -= SC * atan(m_lineControl.Sample(delta));

  return angle;
}

double TDriver::SteerAngle1( tCarElt* car, PtInfo& pi, PtInfo& aheadPi )
{
  LogSHADOW.debug("SHADOW SteerAngle1\n");
  // work out current car speed.
  double	spd0 = hypot(car->_speed_x, car->_speed_y);

  // calc x,y coords of mid point on frt axle.
  double	midPt = 1.37;

  double	x = car->pub.DynGCg.pos.x + midPt * cos(car->_yaw);
  double	y = car->pub.DynGCg.pos.y + midPt * sin(car->_yaw);

  tTrkLocPos	trkPos;
  RtTrackGlobal2Local(car->_trkPos.seg, (tdble)x, (tdble)y, &trkPos, 0);
  double	toMiddle = trkPos.toMiddle;

  // get curret pos on track.
  double	pos = m_track.CalcPos(trkPos);

  // look this far ahead.
  double	aheadDist = spd0 * 0.02;
  double	aheadPos = m_track.CalcPos(trkPos, aheadDist);

  // get info about pts on track.
  GetPosInfo( pos, pi );
  GetPosInfo( aheadPos, aheadPi );

  double	angle = aheadPi.oang - car->_yaw;
  NORM_PI_PI(angle);
  double	avgK = aheadPi.k;
  double omega = car->_speed_x * avgK;//aheadPi.k;
  angle += 0.08 * (omega - car->_yaw_rate);

  // control offset from path.
  m_lineControl.m_p = 1.0;
  m_lineControl.m_d = 10;
  m_lineControl.m_i = 0;//0.02;
  m_lineControl.m_totalRate = 0;
  m_lineControl.m_maxTotal = 2;

  const double SC = MN(1, 8.5 / spd0);
  double	delta = pi.offs + toMiddle;

  angle -= SC * tanh(m_lineControl.Sample(delta));

  return angle;
}

double TDriver::SteerAngle2( tCarElt* car, PtInfo& pi, PtInfo& aheadPi )
{
  LogSHADOW.debug("SHADOW SteerAngle2\n");
  // work out current car speed.
  double	spd0 = hypot(car->_speed_x, car->_speed_y);

  // calc x,y coords of mid point on frt axle.
  double	midPt = 1.37;
  double	x = car->pub.DynGCg.pos.x + midPt * cos(car->_yaw);
  double	y = car->pub.DynGCg.pos.y + midPt * sin(car->_yaw);

  // static double	oldX = x;                   // Removed 5th April 2015 - Not Used
  // static double	oldY = y;                   // Removed 5th April 2015 - Not Used
  // double	velX = (x - oldX) / 0.02;           // Removed 5th April 2015 - Not Used
  // double	velY = (y - oldY) / 0.02;           // Removed 5th April 2015 - Not Used
  // oldX = x;                                    // Removed 5th April 2015 - Not Used
  // oldY = y;                                    // Removed 5th April 2015 - Not Used

  tTrkLocPos	trkPos;
  RtTrackGlobal2Local(car->_trkPos.seg, (tdble) x, (tdble) y, &trkPos, 0);
  double	toMiddle = trkPos.toMiddle;


  // get curret pos on track.
  double	pos = m_track.CalcPos(trkPos);
  double	aheadDist = spd0 * 0.02;
  double	aheadPos = m_track.CalcPos(trkPos, aheadDist);

  // get info about pts on track.
  GetPosInfo( pos, pi );
  GetPosInfo( aheadPos, aheadPi );

  double	angle = aheadPi.oang - car->_yaw;
  NORM_PI_PI(angle);

  double	velAng = atan2(car->_speed_Y, car->_speed_X);
  double	velAngCtrl = aheadPi.oang - velAng;
  NORM_PI_PI(velAngCtrl);
  m_velAngControl.m_p = 1;//0.5;//1;
  m_velAngControl.m_d = 10;//25;
  velAngCtrl = m_velAngControl.Sample(velAngCtrl);
  angle += tanh(velAngCtrl);

  double	avgK = aheadPi.k;
  double omega = car->_speed_x * avgK;//aheadPi.k;
  angle += 0.02 * (omega - car->_yaw_rate);

  // control offset from path.
  m_lineControl.m_p = 1.0;
  m_lineControl.m_d = 10;
  const double SC = 0.15;
  double	delta = pi.offs + toMiddle;

  angle -= SC * tanh(m_lineControl.Sample(delta));

  return angle;
}

double TDriver::SteerAngle3( tCarElt* car, PtInfo& pi, PtInfo& aheadPi )
{
  LogSHADOW.debug("SHADOW SteerAngle3\n");
  // work out current car speed.
  double	spd0 = hypot(car->_speed_x, car->_speed_y);

  // get curret pos on track.
  double	pos = m_track.CalcPos(car);

  // look this far ahead.
  double	aheadTime = 0.25;
  double	aheadDist = spd0 * aheadTime;

  double	aheadPos = m_track.CalcPos(car, aheadDist);

  // get info about pts on track.
  GetPosInfo( pos, pi );
  GetPosInfo( aheadPos, aheadPi );

  // we are trying to control 4 things with the steering...
  //	1. the distance of the car from the driving line it is to follow.
  //	2. the gradient of the distance of car from driving line.
  //	3. the angle of the car (yaw).
  //	4. the rotation speed (omega) of the car.

  // we want the angle of the car to be aheadPi.oang in aheadTime seconds.
  //	our current rotation speed is car->_yaw_rate.  our current rotation
  //	angle is car->_yaw.

  // current yaw rate.
  double	yawU = car->_yaw_rate;

  // future yaw rate required ahead.
  // double	yawV = aheadPi.k * spd0;        // Removed 5th April 2015 - Not Used

  // future yaw required ahead (assuming current yaw to be 0).
  double	yawS = aheadPi.oang - car->_yaw;

  NORM_PI_PI(yawS);

  // acceleration to apply.
  double	yawA = 2 * (yawS - yawU * aheadTime) / (aheadTime * aheadTime);
  double	yaw1s = yawU + 0.5 * yawA;

  // angle to steer to get desired yaw angle after timestep.
  double	dist1s = spd0;
  double	len = 2.63;	// dist between front/back wheel centres.
  double	radiusRear = dist1s / yaw1s;
  double	angle = atan(len / radiusRear);

  if( spd0 < 1 )
    angle = 0;

  // control offset from path.
  const double SC1 = 1;
  m_lineControl.m_p = SC1 * 0.25;	// 1.0 == oscillates
  m_lineControl.m_d = SC1 * 2.5;	// 9.5 == oscillates
  double	delta = pi.offs + car->_trkPos.toMiddle;
  const double SC2 = 1.0 / SC1;
  angle -= SC2 * atan(m_lineControl.Sample(delta));

  return angle;
}

double TDriver::SteerAngle4( tCarElt* car, PtInfo& pi, PtInfo& aheadPi )
{
  LogSHADOW.debug("SHADOW SteerAngle4\n");
  // work out current car speed.
  double	spd0 = hypot(car->_speed_x, car->_speed_y);

  // get curret pos on track.
  double	pos = m_track.CalcPos(car);

  // look this far ahead.
  double	aheadDist = car->_dimension_x * 0.5 + spd0 * 0.02;
  double	aheadPos = m_track.CalcPos(car, aheadDist);

  // get info about pts on track.
  GetPosInfo( pos, pi );
  GetPosInfo( aheadPos, aheadPi );

  //
  //	deal with yaw.
  //

  double	yawError = aheadPi.oang - car->_yaw;
  NORM_PI_PI(yawError);
  double	yawERate = yawError - m_prevYawError;

  // PID with no integration term.
  const double Y_PROP = 0.1;//1;//2;//1.0;
  const double Y_DIFF = 2.5;//1;//10;
  const double Y_SC = 1;
  double angle = Y_SC * atan((Y_PROP * yawError + Y_DIFF * yawERate) / Y_SC);

  //
  //	deal with line.
  //

  double	lineError = -(pi.offs + car->_trkPos.toMiddle);
  double	lineERate = lineError - m_prevLineError;
  m_prevLineError = lineError;

  // PID with no integration term.
  const double L_PROP = 0;//0.15;//005;
  const double L_DIFF = 0;//1.5;	// 2 -- osc;
  const double L_SC = 0.15;
  angle += L_SC * atan((L_PROP * lineError + L_DIFF * lineERate) / L_SC);

  return angle;
}

void TDriver::SpeedControl0( double	targetSpd, double spd0,	double&	acc, double& brk )
{
  if( spd0 > targetSpd )
    {
      if( spd0 - 2 > targetSpd )
        {
          if( spd0 - 2 < targetSpd )
            brk = 0.07;
          else if( spd0 - 3 < targetSpd )
            brk = 0.14;
          else if( spd0 - 4 < targetSpd )
            brk = 0.20;
          else if( spd0 - 5 < targetSpd )
            brk = 0.35;
          else if( spd0 - 6 < targetSpd )
            brk = 0.5;
          else
            brk = 0.5;//targetSpd == 0 ? 0.5 : 0.75;

          acc = 0;
        }
      else
        {
          if( targetSpd > 1 )
            // slow naturally.
            acc = MN(acc, 0.25);
          else
            {
              acc = 0;
              brk = 0.1;
            }
        }
    }

  if( m_lastBrk > 0 && brk > 0 && spd0 - 6 < targetSpd )
    {

      {
        brk = 0;
        acc = 0.25;
      }
    }

  m_lastBrk = brk;
  m_lastTargV = 0;
}

void TDriver::SpeedControl1( double	targetSpd, double spd0, double&	acc, double& brk )
{
  if( spd0 > targetSpd )
    {
      if( spd0 - 1 > targetSpd )
        {
          if( spd0 - 2 < targetSpd )
            brk = 0.07;
          else if( spd0 - 3 < targetSpd )
            brk = 0.14;
          else if( spd0 - 4 < targetSpd )
            brk = 0.20;
          else if( spd0 - 5 < targetSpd )
            brk = 0.25;
          else if( spd0 - 5 < targetSpd )
            brk = 0.5;
          else
            brk = 0.5;//targetSpd == 0 ? 0.5 : 0.75;

          acc = 0;
        }
      else
        {
          if( targetSpd > 1 )
            // slow naturally.
            acc = MN(acc, 0.25);
          else
            {
              acc = 0;
              brk = 0.1;
            }
        }
    }

  m_lastTargV = 0;
}

void TDriver::SpeedControl2( double targetSpd, double spd0, double& acc, double& brk )
{
  if( m_lastBrk && m_lastTargV )
    {
      if( m_lastBrk > 0 )
        {
          double	err = m_lastTargV - spd0;
          m_accBrkCoeff.Sample( err, m_lastBrk );
        }
      m_lastBrk = 0;
      m_lastTargV = 0;
    }

  if( spd0 > targetSpd )
    {
      {
        double	MAX_BRK = 0.5;
        double	err = spd0 - targetSpd;
        brk = MX(0, MN(m_accBrkCoeff.CalcY(err), MAX_BRK));
        acc = 0;

        m_lastBrk = brk;
        m_lastTargV = 0;

        if( brk > 0 )
          {
            if( targetSpd > 0 )
              m_lastTargV = spd0;
          }

      }
    }
}

void TDriver::SpeedControl3( double	targetSpd, double spd0, double&	acc, double& brk )
{
  if( m_lastBrk && m_lastTargV )
    {
      double	err = spd0 - m_lastTargV;
      m_brkCoeff[m_lastB] += err * 0.001;
      m_lastBrk = 0;
      m_lastTargV = 0;
    }

  if( spd0 > targetSpd )
    {
      {
        int		b = int(floor(spd0 / 2));
        double	MAX_BRK = 0.5;
        brk = MX(0, MN(m_brkCoeff[b] * (spd0 - targetSpd), MAX_BRK));
        acc = 0;
        m_lastB = b;
        m_lastBrk = brk;
        m_lastTargV = 0;

        if( brk > 0 && brk < MAX_BRK)
          {
            if( targetSpd > 0 )
              m_lastTargV = targetSpd;
          }
      }
    }
}

void TDriver::SpeedControl4( double targetSpd, double spd0, tCarElt* car, double& acc, double& brk )
{
  if( m_lastBrk && m_lastTargV )
    {
      if( m_lastBrk > 0 || (car->ctrl.accelCmd == -m_lastBrk) )
        {
          double	err = m_lastTargV - spd0;
          m_accBrkCoeff.Sample( err, m_lastBrk );
        }
      m_lastBrk = 0;
      m_lastTargV = 0;
    }

  if( targetSpd < 100 )
    {
      {
        double	MAX_BRK = 0.5;
        double	err = spd0 - targetSpd;
        double	t = m_accBrkCoeff.CalcY(err);

        if( t > 0 )
          {
            brk = MN(t, MAX_BRK);
            acc = 0;
          }
        else
          {
            brk = 0;
            acc = MN(-t, 1);
          }
        m_lastBrk = t;
        m_lastTargV = 0;

        if( t > -1 && t < MAX_BRK )
          {
            if( targetSpd > 0 )
              m_lastTargV = spd0;
          }
      }
    }
}

void TDriver::SpeedControl5( double targetSpd, double spd0, tCarElt* car, double& acc, double& brk )
{
  // Set parameters
  m_speedController.m_p = 0.02;
  m_speedController.m_d = 0.0;
  // Run controller
  double speeddiff =  spd0 - targetSpd;
  acc += m_speedController.Sample(speeddiff);

  if (acc > 1.0)
    {
      acc = 1.0;
    }
}

void TDriver::SpeedControl6( double targetSpd, double spd0, tCarElt* car, double& acc, double& brk )
{
  int	B = (int) MIN(NBR_BRAKECOEFF,(floor(spd0/2)));
  double Diff = 2 * m_brkCoeff[B] * (spd0 - targetSpd);

  brk = m_speedController.Sample(Diff*Diff*Diff);
  brk = MIN(m_maxbrkPressRatio,MAX(0.0, brk));

  if (Diff < 0)
    {
      brk = 0;
    }
  else if ((brk > 0) && (Diff < 0.1))
    {
      brk = 0;
      acc = 0.06;
    }

  if (brk > 0)
    {
      acc = 0;
      LogSHADOW.debug("#Diff: %.3f m/s B: %.3f %% T: %.1f R: %.3f %%\n",
                      Diff, brk*100, m_speedController.m_total, m_maxbrkPressRatio);
    }

  m_lastTargV = targetSpd;
}

void TDriver::SpeedControl(int which, double targetSpd, double spd0, CarElt* car, double& acc, double& brk )
{
  switch( which )
    {
    case 0:		SpeedControl0(targetSpd, spd0, acc, brk);		break;
    case 1:		SpeedControl1(targetSpd, spd0, acc, brk);		break;
    case 2:		SpeedControl2(targetSpd, spd0, acc, brk);		break;
    case 3:		SpeedControl3(targetSpd, spd0, acc, brk);		break;
    case 4:		SpeedControl4(targetSpd, spd0, car, acc, brk);	break;
    case 5:     SpeedControl5(targetSpd, spd0, car, acc, brk);  break;
    case 6:     SpeedControl6(targetSpd, spd0, car, acc, brk);  break;
    default:	SpeedControl3(targetSpd, spd0, acc, brk);		break;
    }
}

void TDriver::Drive( tSituation* s )
{
  DetectFlight();

  if( m_raceStart || s->currentTime <= 0.5 )
    {
      if( m_raceStart )
        {
          LogSHADOW.debug("SHADOW m_raceStart\n");
          Vec2d	thisPt(car->_pos_X, car->_pos_Y);

          for( int i = 0; i + 1 < HIST; i++ )
            m_lastPts[i] = m_lastPts[i + 1];

          m_lastSpd = 0;
          m_lastAng = 0;
          m_steerGraph.SetBeta( 0.1 );
        }

      m_raceStart = false;
      m_avoidS = 0;
      m_avoidSVel = 0;
      m_avoidT = CalcPathTarget(m_track.CalcPos(car), -car->_trkPos.toMiddle);
      LogSHADOW.debug("SHADOW m_avoidT\n");
      m_avoidTVel = 0;
    }

  CurrSimTime = s->currentTime;

  double	carFuel = car->_fuel;

  if(HasTYC)
    {
      m_cm.TYRECONDITIONFRONT  = TyreConditionFront();
      m_cm.TYRECONDITIONREAR   = TyreConditionRear();
      LogSHADOW.debug("Tyre Front = %3f - Tyre Rear = %3f\n", m_cm.TYRECONDITIONFRONT, m_cm.TYRECONDITIONREAR);

      m_cm2.TYRECONDITIONFRONT  = TyreConditionFront();
      m_cm2.TYRECONDITIONREAR   = TyreConditionRear();
    }

  if((fabs(m_cm.FUEL - carFuel) > 5) || (fabs(m_cm.DAMAGE - car->_dammage) > 250))
    {
      m_cm.FUEL	= 5 * floor(carFuel / 5);
      m_cm.DAMAGE	= car->_dammage;

      m_cm2.FUEL = m_cm.FUEL;
      m_cm2.DAMAGE = m_cm.DAMAGE;

      m_path[PATH_NORMAL].CalcMaxSpeeds( m_cm );
      m_path[PATH_NORMAL].PropagateBreaking( m_cm );
      m_path[PATH_NORMAL].PropagateAcceleration( m_cm );
      m_path[PATH_LEFT].CalcMaxSpeeds( m_cm2 );
      m_path[PATH_LEFT].PropagateBreaking( m_cm2 );
      m_path[PATH_LEFT].PropagateAcceleration( m_cm2 );
      m_path[PATH_RIGHT].CalcMaxSpeeds( m_cm2 );
      m_path[PATH_RIGHT].PropagateBreaking( m_cm2 );
      m_path[PATH_RIGHT].PropagateAcceleration( m_cm2 );
      LogSHADOW.debug("SHADOW m_path calculation\n");
    }

  // get curret pos on track.

  PtInfo	pi, aheadPi;
  double angle = 0.0;

  switch (STEER_CTRL)
    {
    case 0:		angle = SteerAngle0(car, pi, aheadPi);		break;
    case 1:		angle = SteerAngle1(car, pi, aheadPi);		break;
    case 2:		angle = SteerAngle2(car, pi, aheadPi);		break;
    case 3:		angle = SteerAngle3(car, pi, aheadPi);		break;
    case 4:		angle = SteerAngle4(car, pi, aheadPi);          break;
    default:            angle = SteerAngle0(car, pi, aheadPi);		break;
    }

  double	steer = angle / car->_steerLock;

  m_DriftAngle = atan2(car->_speed_Y, car->_speed_X) - car->_yaw;
  DOUBLE_NORM_PI_PI(m_DriftAngle);
  m_AbsDriftAngle = fabs(m_DriftAngle);
  m_CosDriftAngle2 = (float) cos(MAX(MIN(m_AbsDriftAngle * 2, PI),-PI));

  // work out current car speed.
  double	spd0 = hypot(car->_speed_x, car->_speed_y);

  {
    Vec2d	thisPt(car->_pos_X, car->_pos_Y);
    if( (thisPt - m_lastPts[0]).len() > 0.1 && car->ctrl.accelCmd == 1.0 )
      {
        double	x[2];
        x[0] = Utils::CalcCurvature(m_lastPts[0], m_lastPts[HIST / 2], thisPt);
        x[0] = fabs(x[0]);
        x[1] = m_lastSpd;
        m_steerGraph.Learn( x, fabs(m_lastAng) );
        m_steerAvg.AddValue( x[0], x[1], fabs(m_lastAng) );
      }

    for( int i = 0; i + 1 < HIST; i++ )
      {
        m_lastPts[i] = m_lastPts[i + 1];
      }

    m_lastPts[HIST - 1] = thisPt;
    m_lastSpd = spd0;
    m_lastAng = m_lastAng * 0.75 + angle * 0.25;
  }

  // const double G = 9.81;            // Removed 5th April 2015 - Not Used

  double	acc = 1.0;
  double	brk = 0;

  double	targetSpd = pi.spd;

  bool	close = false;
  bool	lapper = false;
  AvoidOtherCars( INDEX, car, pi.k, targetSpd, s, close, lapper );
  //targetSpd = CalcSkill(m_Situation, targetSpd);


  LogSHADOW.debug("SHADOW AvoidOtherCars\n");
  if( close )
    {
      SpeedControl( SPDC_TRAFFIC, targetSpd, spd0, car, acc, brk );
      LogSHADOW.debug("#Drive Close SpeedControl = close\n");
    }
  else if( m_Strategy->needPitstop(car, s) )
    {
      SpeedControl( SPDC_TRAFFIC, targetSpd, spd0, car, acc, brk );
      LogSHADOW.debug("#Drive needPitStop SpeedControl = m_pitControl\n");
    }
  else
    {
      if( m_avoidS == 1 )
        {
          SpeedControl( SPDC_NORMAL, targetSpd, spd0, car, acc, brk );
          LogSHADOW.debug("#Drive Avoid SpeedControl = m_avoidS - %d\n", SPDC_NORMAL);
        }
      else if (car->_trkPos.seg->type == TR_STR)
        {
          SpeedControl( SPDC_EXTRA, targetSpd, spd0, car, acc, brk );
          LogSHADOW.debug("#Drive Extra SpeedControl = %d\n", SPDC_EXTRA);
        }
      else
        {
          SpeedControl( SPDC_NORMAL, targetSpd, spd0, car, acc, brk );
          LogSHADOW.debug("#Drive Normal SpeedControl = SPDC_NORMAL - track seg type = %d\n", track->seg->type);
        }
    }

  if( lapper )
    acc = MN(acc, 0.9);

  double	delta = pi.offs + car->_trkPos.toMiddle;

  // out of control??
  double	skidAng = atan2(car->_speed_Y, car->_speed_X) - car->_yaw;
  NORM_PI_PI(skidAng);

  if( car->_speed_x > 5 && fabs(skidAng) > 0.2 )
    {
      acc = MN(acc, 0.25 + 0.75 * cos(skidAng));
    }

  if( car->_speed_x > 5 )
    {
      skidAng = MN(skidAng * 2, PI);
      brk *= MX(0, fabs(cos(skidAng)));
    }

  // too far off line?
  double	fDelta = fabs(delta);
  const double	offDist = 2;//0.5;
  if( fDelta > offDist && car->_distRaced > 50 )
    {
      double	mx = MX(1.0 - (fDelta - offDist) * 0.2, 0.4);
      acc = MN(mx, acc);
    }

  if( car->ctrl.clutchCmd > 0 )
    {
      double	wr = 0;
      int		count = 0;

      if( m_driveType == DT_FWD || m_driveType == DT_4WD )
        {
          wr += car->_wheelRadius(FRNT_LFT) + car->_wheelRadius(FRNT_RGT);
          count += 2;
        }

      if( m_driveType == DT_RWD || m_driveType == DT_4WD )
        {
          wr += car->_wheelRadius(REAR_LFT) + car->_wheelRadius(REAR_RGT);
          count += 2;
        }
      wr /= count;
      double	gr = car->_gearRatio[car->_gear + car->_gearOffset];
      double	rpmForSpd = gr * car->_speed_x / wr;
      double	rpm = car->_enginerpm;

      if( car->ctrl.clutchCmd > 0.5 )
        {
          car->ctrl.clutchCmd = getClutch(car->ctrl.clutchCmd);
        }
      else if( car->ctrl.clutchCmd == 0.5 )
        {
          if( rpmForSpd / rpm > 0.82 )
            car->ctrl.clutchCmd = getClutch(car->ctrl.clutchCmd);
        }
      else
        {
          car->ctrl.clutchCmd = getClutch(car->ctrl.clutchCmd);
          if( car->ctrl.clutchCmd < 0 )
            car->ctrl.clutchCmd = 0;
        }
    }

  int gear = CalcGear(car, acc);

  // facing the wrong way on the track??
  double	dirAng = pi.oang - car->_yaw;
  NORM_PI_PI(dirAng);
  if( gear > 0 && fabs(dirAng) > 75 * PI / 180 )//car->_speed_x < 0 )
    {
      if( dirAng * car->_trkPos.toMiddle < 0 )
        {
          gear = -1;
          acc = 0.5;
          brk = 0;
          steer = -SGN(dirAng);
        }
    }

  if( gear > 0 && car->_speed_x < -0.01 )
    {
      gear = 1;
      brk = car->_speed_x < -0.5 ? 0.25 : 0;
      acc = 0.25;
    }

  if( gear == 1 && car->_speed_x >= -0.01 && car->_speed_x < 10 &&
      acc == 1.0 && brk == 0 )
    {
      double	rpm = car->_enginerpm;
      double	clutch = (850 - rpm) / 400;
      if( car->_speed_x < 0.05 )
        clutch = getClutch((tdble) clutch);

      car->ctrl.clutchCmd = (tdble) MX(0, MN(clutch, 0.9));
    }

  if( fabs(pi.offs) > 5 )
    {
      //		acc = MN(acc, 0.25);
    }

  double	toL = pi.toL - car->_trkPos.toMiddle;
  double	toR = pi.toR + car->_trkPos.toMiddle;

  if( toL < -1 || toR < -1 )
    {
      // off track.
      //		acc = MN(acc, 0.2);
    }

  if( car->ctrl.accelCmd == 1 && car->ctrl.brakeCmd == 0 )
    {
      m_maxAccel.Learn( car->_speed_x, car->_accel_x );
    }

  if( fabs(pi.k * spd0 - car->_yaw_rate) < 0.02 )
    {
      double	velAng = atan2(car->_speed_Y, car->_speed_X);
      double	ang = car->_yaw - velAng;
      NORM_PI_PI(ang);

      int	k = int(floor((pi.k - K_MIN) / K_STEP));
      int	s = int(floor((spd0 - SPD_MIN) / SPD_STEP));
      double	ae = 0;
      if( k >= 0 && k < K_N && s >= 0 && s < SPD_N )
        {
          ae = ang - m_angle[s][k];
          m_angle[s][k] += ae * 0.1;
          ae = m_angle[s][k] - ang;
        }
    }

  CalcSkill();


  if (!HasESP && !HasABS)
    brk = ApplyAbs(car, brk);

  steer = FlightControl(steer);

  brk *= BrakeAdjustPerc;

  //acc *= BrakeAdjustPerc;

  acc = filterDrifting(acc);

  if (!HasTCL)
    acc = filterTCL(acc);

  acc = filterAccel(acc);

  // set up the values to return
  car->ctrl.steer = (tdble) steer;
  car->ctrl.gear = gear;
  car->ctrl.accelCmd = (tdble) acc;
  car->ctrl.brakeCmd = (tdble) brk;
  m_LastBrake = brk;
  m_LastAccel = acc;
  m_LastAbsDriftAngle = m_AbsDriftAngle;

  m_Strategy->update(car, s);

  /*if (!Qualification)								// Don't use pit while
                m_Strategy->CheckPitState(0.6f);				//  qualification*/
}

// Pitstop callback.
int	TDriver::PitCmd( tSituation* s )
{
  return false;
}


// End of the current race.
void TDriver::EndRace(tSituation* s )
{
}


// Called before the module is unloaded.
void TDriver::Shutdown()
{
}

void TDriver::AvoidOtherCars(int index, tCarElt* car, double k, double& carTargetSpd, tSituation* s, bool& close, bool& lapper)
{
  m_pShared->m_teamInfo.GetAt(car->index)->damage = car->_dammage;

  // double	trackLen = m_track.GetLength();     // Removed 5th April 2015 - Not Used
  // double	myPos = RtGetDistFromStart(car);    // Removed 5th April 2015 - Not USed
  double	mySpd = hypot(car->_speed_X, car->_speed_Y);
  if( fabs(mySpd) < 0.01 )
    mySpd = 0.01;

  double	myDirX = car->_speed_X / mySpd;
  double	myDirY = car->_speed_Y / mySpd;
  // int		myIdx = 0;                          // Removed 5th April 2015 - Not Used

  for( int i = 0; i < m_nCars; i++ )
    {
      m_opp[i].UpdatePath();
      m_opp[i].UpdateSit( car, &m_pShared->m_teamInfo, myDirX, myDirY );
    }

  const Opponent::Sit&	mySit = m_opp[m_myOppIdx].GetInfo().sit;

  for( int i = 0; i < m_nCars; i++ )
    {
      m_opp[i].ProcessMyCar( s, &m_pShared->m_teamInfo, car, mySit, *this,
                             m_maxAccel.CalcY(car->_speed_x), i );
    }

  // accumulate all the collision the flags...
  Avoidance::Info	ai;
  double	minCatchTime = 99;
  double	minCatchAccTime = 99;
  double	minVCatTime = 99;
  lapper = false;

  double	width = m_track.GetWidth();
  ai.aheadSpan = Span(-width / 2, width / 2);
  ai.sideSpan = ai.aheadSpan;

  PtInfo	pi;
  GetPtInfo( PATH_NORMAL, car->_distFromStartLine, pi );
  ai.bestPathOffs = pi.offs;

  for( int i = 0; i < m_nCars; i++ )
    {
      Opponent::Info&	oi = m_opp[i].GetInfo();
      CarElt*			oCar = m_opp[i].GetCar();

      ai.flags |= oi.flags;

      if( oi.GotFlags(Opponent::F_FRONT) )
        {
          if( oi.flags & (Opponent::F_COLLIDE || Opponent::F_CATCHING || Opponent::F_CATCHING_ACC) )
            {
            }

          if( oi.GotFlags(Opponent::F_COLLIDE) &&	oi.catchDecel > 12.5 * car->_trkPos.seg->surface->kFriction )
            {
              ai.spdF = MN(ai.spdF, oi.catchSpd);
            }

          if( oi.flags & (Opponent::F_COLLIDE || Opponent::F_CATCHING) )
            minCatchTime = MN(minCatchTime, oi.catchTime);

          if( oi.flags & Opponent::F_CATCHING_ACC )
            minCatchAccTime = MN(minCatchAccTime, oi.catchAccTime);

          if( oi.sit.rdVX < 0 )
            {
              double	vCatTime = -(oi.sit.rdPX - oi.sit.minDX) / oi.sit.rdVX;

              if( vCatTime > 0 )
                minVCatTime = MN(minVCatTime, vCatTime);
            }

          bool ignoreTeamMate = oi.GotFlags(Opponent::F_TEAMMATE) && (car->_laps < oCar->_laps || car->_dammage + 200 >= oi.tmDamage);

          oi.avoidLatchTime = MX(0, oi.avoidLatchTime - s->deltaTime);

          //double	maxSpdK = 15.0 / (110 * 110);
          double maxSpdK = m_cm.CalcMaxSpeedCrv();
          //double	colTime = fabs(k) > maxSpdK ? 0.5 : 0.7;
          double colTime = fabs(k) > maxSpdK ? 1.0 : 1.2;
          //double	catTime = fabs(k) > maxSpdK ? 0.5 :	2.5;
          double catTime = fabs(k) > maxSpdK ? 1.0 : 3.0;
          //double	cacTime = fabs(k) > maxSpdK ? 0.5 : 2.5;
          double cacTime = fabs(k) > maxSpdK ? 1.0 : 3.0;

          bool	catching = ( oi.catchTime < colTime && oi.GotFlags(Opponent::F_COLLIDE))  ||
              ( oi.catchTime    < catTime && oi.GotFlags(Opponent::F_CATCHING)) ||
              ( oi.catchAccTime < cacTime && oi.GotFlags(Opponent::F_CATCHING_ACC));

          if( !ignoreTeamMate && (oi.avoidLatchTime > 0 || catching || oi.GotFlags(Opponent::F_DANGEROUS)))
            {
              double	toL, toR;
              GetPathToLeftAndRight( oCar, toL, toR );
              toL += oi.sit.tVY * oi.catchTime;
              toR -= oi.sit.tVY * oi.catchTime;
              bool	spaceL = toL > oi.sit.minDY;// + 0.25;
              bool	spaceR = toR > oi.sit.minDY;// + 0.25;
              bool	avoidL = oi.sit.rdPY < 0 && spaceR;
              bool	avoidR = oi.sit.rdPY > 0 && spaceL;

              if( catching )
                oi.avoidLatchTime = fabs(k) < maxSpdK ? 0.5 : 0.1;

              if( fabs(k) < maxSpdK )
                {
                  if( !avoidL && !avoidR )
                    {
                      avoidL = !spaceL && spaceR;
                      avoidR = !spaceR && spaceL;
                    }
                }

              if( avoidL )
                {
                  ai.avoidAhead |= Opponent::F_LEFT;
                  ai.aheadSpan.ExcludeLeftOf(-oCar->_trkPos.toMiddle + oi.sit.minDY);
                }
              if( avoidR )
                {
                  ai.avoidAhead |= Opponent::F_RIGHT;
                  ai.aheadSpan.ExcludeRightOf(-oCar->_trkPos.toMiddle - oi.sit.minDY);
                }

              if( avoidL )
                ai.minLDist = MN(oi.sit.ragVX, ai.minLDist);

              if( avoidR )
                ai.minRDist = MN(oi.sit.ragVX, ai.minRDist);
            }
        }

      if( oi.GotFlags(Opponent::F_TO_SIDE) )
        {
          int	av = oi.sit.rdPY < 0 ? Opponent::F_LEFT : Opponent::F_RIGHT;

          ai.avoidToSide |= av;

          if( oi.sit.rdPY < 0 )
            {
              ai.minLSideDist = MN(ai.minLSideDist, -oi.sit.rdPY - oi.sit.minDY);
              ai.sideSpan.ExcludeLeftOf(-oCar->_trkPos.toMiddle + oi.sit.minDY);
            }
          else
            {
              ai.minRSideDist = MN(ai.minRSideDist,  oi.sit.rdPY - oi.sit.minDY);
              ai.sideSpan.ExcludeRightOf(-oCar->_trkPos.toMiddle - oi.sit.minDY);
            }

        }

      if( oi.GotFlags(Opponent::F_AHEAD) )
        {
          if( ai.pClosestAhead == 0 ||
              ai.pClosestAhead->sit.rdPX > oi.sit.rdPX )
            {
              ai.pClosestAhead = &oi;
            }
        }

      bool	treatTeamMateAsLapper =
          oi.GotFlags(Opponent::F_TEAMMATE | Opponent::F_REAR) &&
          oi.sit.relPos > -25 &&
          car->_laps == oCar->_laps &&
          car->_dammage > oi.tmDamage + 300;

      if( STAY_TOGETHER > 50 &&
          oi.GotFlags(Opponent::F_TEAMMATE | Opponent::F_REAR) &&
          oi.sit.relPos < -25 && oi.sit.relPos > -STAY_TOGETHER &&
          car->_dammage + 2000 > oi.tmDamage )
        {
          lapper = true;
        }

      if( oi.GotFlags(Opponent::F_LAPPER) || treatTeamMateAsLapper )
        {
          int	av = oi.sit.rdPY < 0 ? Opponent::F_LEFT : Opponent::F_RIGHT;
          ai.avoidLapping |= av;
          lapper = true;
        }
    }

  ai.k = k;
  ai.nextK = k;

  LogSHADOW.debug("ss %5.1f %5.1f  as %5.1f %5.1f  bpo %5.1f\r", ai.sideSpan.a, ai.sideSpan.b, ai.aheadSpan.a, ai.aheadSpan.b, ai.bestPathOffs );

  // double	pos = car->_distFromStartLine;          // Removed 5 April 2015 - Not Used
  int		carIdx = m_track.IndexFromPos(m_track.CalcPos(car));
  ai.k = 	m_path[PATH_NORMAL].GetAt(carIdx).k;
  int		NSEG = m_track.GetSize();

  for( int i = 1; i < NSEG; i++ )
    {
      int	idx = (carIdx + i) % NSEG;
      double	thisK = m_path[PATH_NORMAL].GetAt(idx).k;

      if( fabs(thisK) > 0.01 )
        {
          ai.nextK = thisK;
          break;
        }
    }

  GenericAvoidance		ga;

  // int		priority = ga.priority(ai, car);        // Removed 5th April 2015 - Not Used
  Vec2d	target = ga.calcTarget(ai, car, *this);

  carTargetSpd = MN(carTargetSpd, ai.spdF);
  close = (ai.flags & Opponent::F_CLOSE) != 0;

  if( m_Flying )
    return;

  double	w = AVOID_WIDTH * 2 + width;
  double	scale = 25.0 / w;
  double	avoidSMaxA = 0.00075 * scale;
  double	avoidSMaxV = 0.005 * scale;

  double	avoidTMaxA = 0.0003 * scale;
  double	avoidTMaxV = 0.2 * scale;

  m_attractor = target.x;

  double	targetS = 1 - target.y;
  if(( m_avoidS != 1 && m_attractor == 0) || (m_avoidS != targetS && m_attractor != 0))
    {
      targetS = (m_attractor == 0) ? 1 : 0;//0.35;
      double	avoidA = targetS > m_avoidS ? avoidSMaxA : -avoidSMaxA;

      double	dist = targetS - m_avoidS;

      if( fabs(dist) < 0.0005 )
        m_avoidSVel = 0;
      else
        {
          double	slowS = (m_avoidSVel * m_avoidSVel) / (2 * avoidSMaxA);

          if( fabs(dist) <= slowS )
            {
              avoidA = -(m_avoidSVel * m_avoidSVel) / (2 * dist);
            }

          m_avoidSVel += avoidA;
        }
    }
  else
    m_avoidSVel = 0;

  if( m_avoidSVel > avoidSMaxV )
    m_avoidSVel = avoidSMaxV;
  else if( m_avoidSVel < -avoidSMaxV )
    m_avoidSVel = -avoidSMaxV;

  double	oldAvoidS = m_avoidS;
  m_avoidS += m_avoidSVel;

  if( m_avoidS < 0.0005 && m_avoidSVel < 0 )
    {
      m_avoidS = 0;
      m_avoidSVel = 0;
    }
  else if( m_avoidS >= 0.9995 && m_avoidSVel > 0 )
    {
      m_avoidS = 1;
      m_avoidSVel = 0;
    }
  else if(( oldAvoidS < targetS && m_avoidS >= targetS ) ||
          ( oldAvoidS > targetS && m_avoidS <= targetS ) ||
          ( fabs(targetS - m_avoidS) < 0.0005 ))
    {
      m_avoidS = targetS;
      m_avoidSVel = 0;
    }

  double	attractT = m_attractor;
  double	avoidA = 0;

  if( attractT != m_avoidT )
    {
      double	tMaxA = avoidTMaxA / MX(0.2, 1 - m_avoidS);
      avoidA = attractT > m_avoidT ? tMaxA : -tMaxA;

      double	dist = attractT - m_avoidT;
      double	slowS = (m_avoidTVel * m_avoidTVel) / (2 * avoidTMaxA);

      if( dist * m_avoidTVel > 0 && fabs(dist) <= slowS )
        {
          avoidA = -(m_avoidTVel * m_avoidTVel) / (2 * dist);
        }

      if( avoidA > avoidTMaxA )
        avoidA = avoidTMaxA;
      else if( avoidA < -avoidTMaxA )
        avoidA = -avoidTMaxA;

      m_avoidTVel += avoidA;
    }
  else
    m_avoidTVel = 0;

  double	tMaxV = avoidTMaxV / MX(0.2, 1 - m_avoidS);

  if( m_avoidTVel > tMaxV )
    m_avoidTVel = tMaxV;
  else if( m_avoidTVel < -tMaxV )
    m_avoidTVel = -tMaxV;

  double	oldAvoidT = m_avoidT;
  m_avoidT += m_avoidTVel;

  if( m_avoidT < -1 )
    {
      m_avoidT = -1;
      m_avoidTVel = 0;
    }
  else if( m_avoidT > 1 )
    {
      m_avoidT = 1;
      m_avoidTVel = 0;
    }
  else if(( oldAvoidT < attractT && m_avoidT >= attractT ) || ( oldAvoidT > attractT && m_avoidT <= attractT ))
    {
      m_avoidT = attractT;
      m_avoidTVel = 0;
    }
}

int TDriver::CalcGear( tCarElt* car, double& acc )
{
  if( car->_gear <= 0 )
    return 1;
#if 0
  const int	MAX_GEAR = car->_gearNb - 1;

  double	gr_dn = car->_gear > 1 ? car->_gearRatio[car->_gear + car->_gearOffset - 1] : 1e5;
  double	gr_this = car->_gearRatio[car->_gear + car->_gearOffset];

  double	wr = wheelRadius;
  double	rpm = gr_this * car->_speed_x / wr;

  double	rpmUp = m_gearUpRpm;
  double	rpmDn = rpmUp * gr_this / gr_dn;

  if( car->_gear < MAX_GEAR && rpm > rpmUp )
    {
      return car->_gear + 1;
    }
  else if( car->_gear > 1 && rpm < rpmDn )
    {
      return car->_gear - 1;
    }
#else
  {
    /*BT gear changing */
    float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
    float omega = (car->_enginerpmRedLine * m_Shift) /gr_up;
    float wr = (tdble) wheelRadius;

    if (omega * wr * m_Shift < car->_speed_x)
      {
        return car->_gear + 1;
      } else
      {
        float gr_down = car->_gearRatio[car->_gear + car->_gearOffset - 1];
        omega = (car->_enginerpmRedLine * m_Shift) /gr_down;

        if (car->_gear > 1 && omega * wr * m_Shift > car->_speed_x + SHIFT_MARGIN)
          {
            return car->_gear - 1;
          }
      }
  }
#endif
  return car->_gear;
}

// Compute the clutch value.
float TDriver::getClutch(float flutch)
{
  float m_Clutch = flutch;
#if 0
  float speedr;
  float omega;
  float wr = wheelRadius;
  //int PrevGear = car->_gear - 1;
  omega = car->_enginerpmRedLine/car->_gearRatio[car->_gear + car->_gearOffset];
  speedr = (CLUTCH_SPEED + MAX(0.0f, car->_speed_x))/fabs(wr*omega);

  if (1 || car->_gearCmd > 1)
    {
      float maxtime = MAX(0.06f, 0.32f - ((float) car->_gearCmd / 65.0f));

      if (car->_gear != car->_gearCmd)
        clutchtime = maxtime;

      if (clutchtime > 0.0f)
        clutchtime -= (float) (RCM_MAX_DT_ROBOTS * (0.02f + ((float) car->_gearCmd / 8.0f)));

      return 2.0f * clutchtime;
    } else
    {
      float drpm = car->_enginerpm - car->_enginerpmRedLine/2.0f;
      float ctlimit = 0.9f;

      if (car->_gearCmd > 1)
        ctlimit -= 0.15f + (float) car->_gearCmd/13.0f;

      clutchtime = MIN(ctlimit, clutchtime);

      if (car->_gear != car->_gearCmd)
        clutchtime = 0.0f;

      float clutcht = (ctlimit - clutchtime) / ctlimit;

      if (car->_gear == 1 && car->_accelCmd > 0.0f)
        {
          clutchtime += (float) RCM_MAX_DT_ROBOTS;
        }

      if (car->_gearCmd == 1 || drpm > 0)
        {

          if (car->_gearCmd == 1)
            {
              // Compute corresponding speed to engine rpm.
              //float omega = car->_enginerpmRedLine/car->_gearRatio[car->_gear + car->_gearOffset];
              //float wr = car->_wheelRadius(2);
              //speedr = (CLUTCH_SPEED + MAX(0.0f, car->_speed_x))/fabs(wr*omega);
              float clutchr = MAX(0.0f, (1.0f - speedr*2.0f*drpm/car->_enginerpmRedLine)) *
                  (car->_gearCmd == 1 ? 0.95f : (0.7f - (float)(car->_gearCmd)/30.0f));
              return MIN(clutcht, clutchr);
            } else
            {
              // For the reverse gear.
              clutchtime = 0.0f;
              return 0.0f;
            }
        } else
        {
          return clutcht;
        }
    }
#else
  if(m_Clutch > 0)
    {
      if (car->_gear < 2)
        m_Clutch = startAutomatic(m_Clutch);

      m_Clutch = (tdble)(MIN(m_ClutchMax, m_Clutch));
      if(m_Clutch == m_ClutchMax)
        {
          if((car->_gear + car->_gearOffset)* car->_speed_x
             / (wheelRadius * car->_enginerpm) > m_ClutchRange)
            {
              m_Clutch = (tdble)(m_ClutchMax - 0.01);
            }
          else
            m_Clutch -= (tdble)(m_ClutchDelta / 10);
        }
      else
        {
          m_Clutch -= (tdble) m_ClutchDelta;
          m_Clutch = (tdble)(MAX(0.0, m_Clutch));
        }
    }

  return m_Clutch;
#endif
}

float TDriver::startAutomatic(float clutch)
{
  float m_Clutch = clutch;
  if ((car->_gearCmd == 1) && (TDriver::CurrSimTime < 20))
    {
      if (car->_enginerpm < 0.94)
        m_Clutch += (tdble)(m_ClutchDelta);
      else if (car->_enginerpm > 1.1 * 0.94)
        m_Clutch -= (tdble)(m_ClutchDelta * m_ClutchRelease);
    }

  return m_Clutch;
}

double TDriver::filterBrake(double Brake)
{
  /*BrakeRight = 1.0f;
  BrakeLeft = 1.0f;
  BrakeFront = 1.0f;
  BrakeRear = 1.0f;

  // EPS system for use with SimuV4 ...
  if((CarSpeedLong > SLOWSPEED) && (Brake > 0.0))
  {
    Brake *= (float) MAX(0.1, oCosDriftAngle2);
    if (oDriftAngle > 4.0/180.0*PI)
    {
      BrakeLeft = 1.0f + oBrakeCorrLR;
      BrakeRight = 1.0f - oBrakeCorrLR;
      BrakeFront = 1.0f + oBrakeCorrFR;
      BrakeRear = 1.0f - oBrakeCorrFR;
      LogSHADOW.debug("#BL+ BR- %.3f deg\n",oDriftAngle*180/PI);
    }
    else if (oDriftAngle > 2.0/180.0*PI)
    {
      oBrakeLeft = 1.0f + oBrakeCorrLR;
      oBrakeRight = 1.0f - oBrakeCorrLR;
      oBrakeFront = 1.0f;
      oBrakeRear = 1.0f;
      LogSHADOW.debug("#BL+ BR- %.3f deg\n",oDriftAngle*180/PI);
    }
    else if (oDriftAngle < -4.0/180.0*PI)
    {
      oBrakeRight = 1.0f + oBrakeCorrLR;
      oBrakeLeft = 1.0f - oBrakeCorrLR;
      oBrakeFront = 1.0f + oBrakeCorrFR;
      oBrakeRear = 1.0f - oBrakeCorrFR;
      LogSHADOW.debug("#BL- BR+ %.3f deg\n",oDriftAngle*180/PI);
    }
    else if (oDriftAngle < -2.0/180.0*PI)
    {
      oBrakeRight = 1.0f + oBrakeCorrLR;
      oBrakeLeft = 1.0f - oBrakeCorrLR;
      oBrakeFront = 1.0f;
      oBrakeRear = 1.0f;
      LogSHADOW.debug("#BL- BR+ %.3f deg\n",oDriftAngle*180/PI);
    }
    else
    {
      oBrakeRight = 1.0f;
      oBrakeLeft = 1.0f;
      oBrakeFront = 1.0f;
      oBrakeRear = 1.0f;
      //LogSHADOW.debug("#BR = BL %.3f�\n",oDriftAngle*180/PI);
    }
  }
  // ... EPS system for use with SimuV4

  // Limit the brake press at start of braking
  if (oLastAccel > 0)
    return MIN(0.10,Brake);*/

  return Brake;
}

double TDriver::filterAccel(double Accel)
{
  if (m_Rain)
    {
      if (Accel > m_LastAccel + m_DeltaAccelRain)
        Accel = MIN(1.0, m_LastAccel + m_DeltaAccelRain);
    }
  else
    {
      if (Accel > m_LastAccel + m_DeltaAccel)
        Accel = MIN(1.0, m_LastAccel + m_DeltaAccel);
    }
  return Accel;
}

double TDriver::ApplyAbs( tCarElt* car, double brake )
{
  if( car->_speed_x < 10 )
    return brake;

  if (car->_speed_x < ABS_MINSPEED)
      return brake;

  float trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
  double angle = trackangle - car->_yaw;
  NORM_PI_PI(angle);
  float origbrake = brake;
  float rearskid = MAX(0.0f, MAX(car->_skid[2], car->_skid[3]) - MAX(car->_skid[0], car->_skid[1]));
  int i;
  float slip = 0.0f;

  for (i = 0; i < 4; i++)
  {
      slip += car->_wheelSpinVel(i) * car->_wheelRadius(i);
  }

  slip *= 1.0f + MAX(rearskid, MAX(fabs(car->_yaw_rate) / 5, fabs(angle) / 6));
  slip = car->_speed_x - slip / 4.0f;

  if (slip > m_AbsSlip)
  {
      brake = brake - MIN(brake, (slip - m_AbsSlip) / m_AbsRange);
  }

  if (car->_speed_x > 5.0)
  {
      double skidAng = atan2(car->_speed_Y, car->_speed_X) - car->_yaw;
      NORM_PI_PI(skidAng);
      skidAng = MIN(skidAng * 2, PI);
      brake *= MAX(0, fabs(cos(skidAng)));
  }

  brake = MAX(brake, MIN(origbrake, 0.1f));

  return brake;
}

tdble F(tWing* wing)
{
  return 1 - exp( pow(-(wing->a / wing->b),wing->c));
}

tdble CliftFromAoA(tWing* wing)
{
  tdble angle = (tdble) (wing->angle * 180/PI);
  wing->Kz_org = 4.0f * wing->Kx;

  double s = 0;

  if (angle <= wing->AoAatMax)
    {
      wing->a = wing->f * (angle + wing->AoAOffset);
      s = sin(wing->a/180.0*PI);

      return (tdble)(s * s * (wing->CliftMax + wing->d) - wing->d);
    }
  else
    {
      wing->a = (angle - wing->AoAatMax - 90.0f);

      return (tdble)(wing->CliftMax - F(wing) * (wing->CliftMax - wing->CliftAsymp));
    }
}

// Compute aerodynamic downforce coefficient CA.
void TDriver::initWheelPos()
{
  for (int i=0; i<4; i++)
    {
      char const *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
      float rh = 0.0;
      rh = GfParmGetNum(car->_carHandle,WheelSect[i],PRM_RIDEHEIGHT,(char *)NULL, 0.10f);
      wheelz[i] = (-rh / 1.0 + car->info.wheel[i].wheelRadius) - 0.01;
    }
}

void TDriver::initBrake()
{
  LogSHADOW.debug("\n#Init Brake >>>\n\n");

  float DiameterFront = GfParmGetNum(car->_carHandle, (char*) SECT_FRNTRGTBRAKE,
                                     PRM_BRKDIAM, (char*)NULL, 0.2f);
  float DiameterRear = GfParmGetNum(car->_carHandle, (char*) SECT_REARRGTBRAKE,
                                    PRM_BRKDIAM, (char*)NULL, 0.2f);
  LogSHADOW.debug("#Brake diameter    : %0.3f m / %0.3f m\n", DiameterFront, DiameterRear);

  float AreaFront = GfParmGetNum(car->_carHandle, (char*) SECT_FRNTRGTBRAKE,
                                 PRM_BRKAREA, (char*)NULL, 0.002f);
  float AreaRear = GfParmGetNum(car->_carHandle, (char*) SECT_REARRGTBRAKE,
                                PRM_BRKAREA, (char*)NULL, 0.002f);
  LogSHADOW.debug("#Brake area        : %0.5f m2 / %0.5f m2\n", AreaFront, AreaRear);

  float MuFront = GfParmGetNum(car->_carHandle, (char*) SECT_FRNTRGTBRAKE,
                               PRM_MU, (char*)NULL, 0.30f);
  float MuRear = GfParmGetNum(car->_carHandle, (char*) SECT_REARRGTBRAKE,
                              PRM_MU, (char*)NULL, 0.30f);
  LogSHADOW.debug("#Brake mu          : %0.5f / %0.5f\n", MuFront, MuRear);

  float Rep = GfParmGetNum(car->_carHandle, (char*) SECT_BRKSYST,
                           PRM_BRKREP, (char*)NULL, 0.5);
  float Press = GfParmGetNum(car->_carHandle, (char*) SECT_BRKSYST,
                             PRM_BRKPRESS, (char*)NULL, 1000000);

  LogSHADOW.info("#################################\n");
  LogSHADOW.info("#Shadow Brake repartition : %0.2f\n", Rep);
  LogSHADOW.info("#Shadow Brake pressure    : %0.0f\n", Press);
  LogSHADOW.info("#################################\n");

  float MaxPressRatio = GfParmGetNum(car->_carHandle, SECT_PRIV,
                                     PRV_MAX_BRAKING, (char*)NULL, (float) m_maxbrkPressRatio);
  LogSHADOW.debug("#Shadow2 Max press ratio   : %0.7f\n", MaxPressRatio);

  float BrakeCoeffFront = (float) (DiameterFront * 0.5 * AreaFront * MuFront);
  float BrakeCoeffRear = (float) (DiameterRear * 0.5 * AreaRear * MuRear);
  LogSHADOW.debug("#Shadow2 Brake coefficient : %0.7f / %0.7f\n", BrakeCoeffFront, BrakeCoeffRear);

  BrakeMaxTqFront = MaxPressRatio * BrakeCoeffFront * Press * Rep;
  LogSHADOW.debug("#Shadow2 Brake torque front: %0.2f\n", BrakeMaxTqFront);

  BrakeMaxTqRear = MaxPressRatio * BrakeCoeffRear * Press * (1 - Rep);
  LogSHADOW.debug("#Shadow2 Brake torque rear : %0.2f\n", BrakeMaxTqRear);

  BrakeForce = (3 * BrakeMaxTqFront * (car->_wheelRadius(FRNT_LFT) + car->_wheelRadius(FRNT_RGT))
                + BrakeMaxTqRear * (car->_wheelRadius(REAR_LFT) + car->_wheelRadius(REAR_RGT))) / 4;
  LogSHADOW.debug("#Shadow2 Brake force       : %0.2f\n", BrakeForce);
  LogSHADOW.debug("\n#<<< Shadow2 Init Brake\n\n");
}

//==========================================================================*

//==========================================================================*
// Adjust brakes
//--------------------------------------------------------------------------*
void TDriver::AdjustBrakes(void *pCarHandle)
{
  if ((TDriver::UseBrakeLimit) || (TDriver::UseGPBrakeLimit))
    {
      TDriver::BrakeLimit = GfParmGetNum(pCarHandle, SECT_PRIV, PRV_BRAKE_LIMIT, 0, (float) TDriver::BrakeLimit);
      LogSHADOW.debug("#BrakeLimit %g\n", TDriver::BrakeLimit);

      TDriver::BrakeLimitBase = GfParmGetNum(pCarHandle, SECT_PRIV,PRV_BRAKE_LIMIT_BASE,0, (float) TDriver::BrakeLimitBase);
      LogSHADOW.debug("#BrakeLimitBase %g\n", TDriver::BrakeLimitBase);

      TDriver::BrakeLimitScale = GfParmGetNum(pCarHandle, SECT_PRIV, PRV_BRAKE_LIMIT_SCALE, 0, (float) TDriver::BrakeLimitScale);
      LogSHADOW.debug("#BrakeLimitScale %g\n", TDriver::BrakeLimitScale);

      TDriver::SpeedLimitBase = GfParmGetNum(pCarHandle, SECT_PRIV, PRV_SPEED_LIMIT_BASE, 0, (float) TDriver::SpeedLimitBase);
      LogSHADOW.debug("#SpeedLimitBase %g\n", TDriver::SpeedLimitBase);

      TDriver::SpeedLimitScale = GfParmGetNum(pCarHandle, SECT_PRIV, PRV_SPEED_LIMIT_SCALE, 0, (float) TDriver::SpeedLimitScale);
      LogSHADOW.debug("#SpeedLimitScale %g\n", TDriver::SpeedLimitScale);
    }
};

void TDriver::initCa()
{
  LogSHADOW.debug("\n#Shadow Init InitCA >>>\n\n");

  const char *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
  const char *WingSect[2] = {SECT_FRNTWING, SECT_REARWING};

  bool WingTypeProfile = false;

  float FrontWingArea = GfParmGetNum(car->_carHandle, SECT_FRNTWING, PRM_WINGAREA, (char*) NULL, 0.0);
  float FrontWingAngle = GfParmGetNum(car->_carHandle, SECT_FRNTWING, PRM_WINGANGLE, (char*) NULL, 0.0);

  LogSHADOW.debug("#Shadow FrontWingAngle %g\n", FrontWingAngle * 180 / PI);
  float RearWingArea = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGAREA, (char*) NULL, 0.0f);
  float RearWingAngle = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGANGLE, (char*) NULL, 0.0f);
  LogSHADOW.debug("#Shadow RearWingAngle %g\n", RearWingAngle * 180 / PI);

  WingAngleFront = FrontWingAngle;
  WingAngleRear = RearWingAngle;
  if (WingControl)
    {
      WingAngleRearMin = RearWingAngle;
      WingAngleRearMax = 2.5f * RearWingAngle;
      WingAngleRearBrake = (float) (0.9 * PI_4);
    }
  else
    {
      WingAngleRearMin = RearWingAngle;
      WingAngleRearMax = RearWingAngle;
      WingAngleRearBrake = RearWingAngle;
    }

  float FrontWingAreaCd = FrontWingArea * sin(FrontWingAngle);
  float RearWingAreaCd = RearWingArea * sin(RearWingAngle);
  float WingCd = (float) (1.23 * (FrontWingAreaCd + RearWingAreaCd));
  WingCD = WingCd;

  float FCL = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FCL, (char*) NULL, 0.0f);
  float RCL = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_RCL, (char*) NULL, 0.0f);

  float H = 0.0;
  int I;
  for (I = 0; I < 4; I++)
    H += GfParmGetNum(car->_carHandle, WheelSect[I], PRM_RIDEHEIGHT, (char*) NULL, 0.20f);

  H *= 1.5;
  H = H*H;
  H = H*H;
  H = (float) (2.0 * exp(-3.0 * H));
  CA = H * (FCL + RCL) + 4.0 * WingCd;
  CAFWING = 4 * 1.23 * FrontWingAreaCd;
  CARWING = 4 * 1.23 * RearWingAreaCd;
  CAFGROUNDEFFECT = H * FCL;
  CARGROUNDEFFECT = H * RCL;

  //>>> simuv4
  double CliftFrnt = 0;
  double CliftRear = 0;
  double MeanCliftFromAoA = 0;
  const char* w = NULL;
  int WingType = 0;
  double phi;
  double sinphi;
  double sinphi2;

  for (int index = 0; index < 2; index++)
    {
      w = GfParmGetStr(car->_carHandle, WingSect[index], PRM_WINGTYPE, "FLAT");

      if (strncmp(w,"FLAT",4) == 0)
        WingType = 0;
      else if (strncmp(w, "PROFILE", 7) == 0)
        WingType = 1;
      // ...

      if (WingType == 1)
        {
          tWing *wing = &(carWing[index]);
          wing->WingType = WingType;

          WingTypeProfile = true;

          if (index == 0)
            {
              wing->angle = FrontWingAngle;
            }
          else
            {
              wing->angle = RearWingAngle;
            }

          /* [deg] Angle of Attack at the maximum of coefficient of lift */
          wing->AoAatMax = GfParmGetNum(car->_carHandle, WingSect[index], PRM_AOAATMAX, (char*) "deg", 90);
          //fprintf(stderr,"AoAatMax: %g\n",wing->AoAatMax);

          /* [deg] Angle of Attack at coefficient of lift = 0 (-30 < AoAatZero < 0) */
          wing->AoAatZero = GfParmGetNum(car->_carHandle, WingSect[index], PRM_AOAATZERO, (char*) "deg", 0);
          //fprintf(stderr,"AoAatZero: %g\n",wing->AoAatZero);
          wing->AoAatZRad = (tdble) (wing->AoAatZero/180*PI);

          /* [deg] Offset for Angle of Attack */
          wing->AoAOffset = GfParmGetNum(car->_carHandle, WingSect[index], PRM_AOAOFFSET, (char*) "deg", 0);
          //fprintf(stderr,"AoAOffset: %g\n",wing->AoAOffset);

          /* Maximum of coefficient of lift (0 < CliftMax < 4) */
          wing->CliftMax = GfParmGetNum(car->_carHandle, WingSect[index], PRM_CLMAX, (char*)NULL, 4);
          //fprintf(stderr,"CliftMax: %g\n",wing->CliftMax);

          /* Coefficient of lift at Angle of Attack = 0 */
          wing->CliftZero = GfParmGetNum(car->_carHandle, WingSect[index], PRM_CLATZERO, (char*)NULL, 0);
          //fprintf(stderr,"CliftZero: %g\n",wing->CliftZero);

          /* Asymptotic coefficient of lift at large Angle of Attack */
          wing->CliftAsymp = GfParmGetNum(car->_carHandle, WingSect[index], PRM_CLASYMP, (char*)NULL, wing->CliftMax);
          //fprintf(stderr,"CliftAsymp: %g\n",wing->CliftAsymp);

          /* Delay of decreasing */
          wing->b = GfParmGetNum(car->_carHandle, WingSect[index], PRM_DELAYDECREASE, (char*)NULL, 20);
          //fprintf(stderr,"b: %g\n",wing->b);

          /* Curvature of start of decreasing */
          wing->c = GfParmGetNum(car->_carHandle, WingSect[index], PRM_CURVEDECREASE, (char*)NULL, 2);
          //fprintf(stderr,"c: %g\n",wing->c);

          /* Scale factor for angle */
          wing->f = (tdble) (90.0 / (wing->AoAatMax + wing->AoAOffset));
          //fprintf(stderr,"f: %g\n",wing->f);
          phi = wing->f * (wing->AoAOffset);
          //fprintf(stderr,"phi: %g deg\n",phi);
          phi *= PI / 180;
          //fprintf(stderr,"phi: %g rad\n",phi);
          sinphi = sin(phi);
          //fprintf(stderr,"sinphi: %g\n",sinphi);
          sinphi2 = sinphi * sinphi;

          /* Scale at AoA = 0 */
          wing->d = (tdble) (1.8f * (sinphi2 * wing->CliftMax - wing->CliftZero));

          if (index == 0)
            {
              CliftFrnt = CliftFromAoA(wing);

              FrontWingAreaCd = FrontWingArea * sin(FrontWingAngle - wing->AoAatZRad);
              CAFWING = CliftFrnt * 1.23 * FrontWingAreaCd;
              MeanCliftFromAoA = CliftFrnt;
            }
          else
            {
              CliftRear = CliftFromAoA(wing);

              RearWingAreaCd = RearWingArea * sin(RearWingAngle - wing->AoAatZRad);
              CARWING = CliftRear * 1.23 * RearWingAreaCd;
              if (CliftFrnt > 0)
                {
                  MeanCliftFromAoA += CliftRear;
                  MeanCliftFromAoA /= 2;
                }
              else
                MeanCliftFromAoA = CliftRear;
            }

        }
    }

  if (WingTypeProfile)
    {
      WingCd = (float) (1.23 * (FrontWingAreaCd + RearWingAreaCd));
      WingCD = WingCd;
      CA = H * (FCL + RCL) + MeanCliftFromAoA * WingCd;
    }

  m_cm.CA = CA;
  m_cm.CA_FW = CAFWING;
  m_cm.CA_RW = CARWING;
  m_cm.CA_GE = CARGROUNDEFFECT;
  m_cm.CD_WING = WingCD;
  //<<< simuv4

  LogSHADOW.debug("\n#<<< Shadow Init InitCa\n\n");
}

// Compute aerodynamic drag coefficient CW.
void TDriver::initCw()
{
  LogSHADOW.debug("\n#Shadow Init InitCw >>>\n\n");

  CX = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_CX, (char*) NULL, 0.0f);
  float frontarea = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FRNTAREA, (char*) NULL, 0.0f);
  CW = 0.645f * CX * frontarea;

  LogSHADOW.debug("\n#Shadow CX = %.3f - Front Area = %.3f - CW = %.3f\n", CX, frontarea, CW);
  LogSHADOW.debug("\n#<<< Shadow Init InitCw\n\n");
  m_cm.CD_BODY = CW;
  m_cm.CD_CX = CX;
}

// Compute Front/Rear Repartition coefficient CR
void TDriver::initCR()
{
  LogSHADOW.debug("\n#SHADOW Front/Rear Repartition >>>\n\n");
  CR = GfParmGetNum(car->_carHandle, (char *)SECT_CAR, (char *)PRM_FRWEIGHTREP, (char *)NULL, 0.50);
  LogSHADOW.debug("<<< #SHADOW Front/Rear Repartition = %.3f\n\n", CR);
}

// Init drive train
void TDriver::initDriveTrain()
{
  LogSHADOW.debug("\n#SHADOW initDriveTrain >>>\n");

  m_driveType = DT_RWD;                                   // Assume rear wheel drive
  HasTrainRWD = true;
  const char* Train = GfParmGetStr(car->_carHandle, SECT_DRIVETRAIN, PRM_TYPE, VAL_TRANS_RWD);

  if (strcmp(Train, VAL_TRANS_FWD) == 0)                  //   If front wheel drive
    {
      m_driveType = DT_FWD;                               //   change mode
      HasTrainFWD = true;
      HasTrainRWD = false;
    }
  else if (strcmp(Train, VAL_TRANS_4WD) == 0)             //   and if all wheel drive
    {
      m_driveType = DT_4WD;                               //   too
      HasTrainFWD = true;
      HasTrainRWD = true;
    }

  LogSHADOW.debug("\n#<<< SHADOW initDriveTrain\n\n");
}

// Calculate mean wheel radius
void TDriver::initWheelRadius()
{
  LogSHADOW.debug("\n#SHADOW initWheelRadius >>>\n");

  int Count = 0;
  wheelRadius = 0.0;

  if(m_driveType == DT_FWD || m_driveType == DT_4WD)
    {
      wheelRadius += car->_wheelRadius(FRNT_LFT) + car->_wheelRadius(FRNT_RGT);
      Count += 2;
    }

  if(m_driveType == DT_RWD || m_driveType == DT_4WD)
    {
      wheelRadius += car->_wheelRadius(REAR_LFT) + car->_wheelRadius(REAR_RGT);
      Count += 2;
    }

  wheelRadius /= Count;

  LogSHADOW.debug("\n#<<< SHADOW initWheelRadius\n\n");
}

// Init the friction coefficient of the the tires.
void TDriver::initTireMu()
{
  LogSHADOW.debug("\n#Shadow InitTireMu >>>\n\n");
  char const *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
  TIREMUF = FLT_MAX;
  int I;

  for (I = 0; I < 2; I++)
    TIREMUF = MIN(TIREMUF, GfParmGetNum(car->_carHandle, WheelSect[I],
                                        PRM_MU, (char*) NULL, 1.0f));
  TIREMUR = FLT_MAX;
  for (I = 2; I < 4; I++)
    TIREMUR = MIN(TIREMUR, GfParmGetNum(car->_carHandle, WheelSect[I],
                                        PRM_MU, (char*) NULL, 1.0f));

  TIREMU = MIN(TIREMUF, TIREMUR);

  LogSHADOW.debug("\n#<<< Shadow TIREMUF = %.3f - TIREMUR = %.3f - TIREMU = %.3f\n", TIREMUF, TIREMUR, TIREMU);
  LogSHADOW.debug("\n#<<< Shadow InitTireMu\n\n");

  m_cm.TYRE_MU = TIREMU;
  m_cm.TYRE_MU_F = TIREMUF;
  m_cm.TYRE_MU_R = TIREMUR;
}

double TDriver::filterTCL(double Accel)                                     // Tracktion control
{
  if(fabs(car->_speed_x) < 0.001)                                         // Only if driving faster
    return Accel;

  double Spin = 0;                                                        // Initialize spin
  double Wr = 0;                                                          // wheel radius
  int Count = 0;                                                          // count impellers

  if(HasTrainFWD)                                                         // If front wheels
    {                                                                       //   are impellers
      double WSL = car->_wheelSpinVel(FRNT_LFT);                          // Get spin velocity
      double WSR = car->_wheelSpinVel(FRNT_RGT);
      if (WSL > WSR)                                                      // Depending on max
        Spin += 2 * WSL + WSR;                                          // calc weighted spin
      else
        Spin += WSL + 2 * WSR;
      Wr += car->_wheelRadius(FRNT_LFT)+ car->_wheelRadius(FRNT_RGT);     // measure radius
      Count += 3;                                                         // and count weights
    }

  if(HasTrainRWD)                                                         // If rear wheels
    {                                                                       //   are impellers
      double WSL = car->_wheelSpinVel(REAR_LFT);
      double WSR = car->_wheelSpinVel(REAR_RGT);
      if (WSL > WSR)
        Spin += 2 * WSL + WSR;
      else
        Spin += WSL + 2 * WSR;
      Wr += car->_wheelRadius(REAR_LFT)+ car->_wheelRadius(REAR_RGT);
      Count += 3;
    }
  Spin /= Count;                                 // Calculate spin
  Wr /= Count;                                   // and radius

  double Slip = Spin * Wr - car->_speed_x;        // Calculate slip

  if (m_Rain)
    Slip *= m_TclFactor * (m_RainIntensity * 0.25 + 1);

  float AccelScale = 0.05f;
  if (m_Rain)
    AccelScale = 0.01f;

#if 0
  if (Slip > m_TclSlip)                           // Decrease accel if needed
    {
      float MinAccel = (float) (AccelScale * Accel);
      Accel -= MIN(Accel, (Slip - m_TclSlip)/ m_TclRange);
      Accel = MAX(MinAccel, Accel);
    }

  return MIN(1.0, Accel);
#else
    //if (simtime < 0.7)
        //return accel;

    Accel = MIN(1.0f, Accel);
    float accel1 = Accel, accel2 = Accel, accel3 = Accel, accel4 = Accel;

    if (car->_speed_x > 10.0f)
    {
    }
        float this_tcl_slip = (car->_speed_x > 10.0f ? m_TclSlip : m_TclRange/10);
        if (Slip > this_tcl_slip)
        {
            float friction = MIN(car->_wheelSeg(REAR_RGT)->surface->kFriction, car->_wheelSeg(REAR_LFT)->surface->kFriction);

            if (friction >= 0.95f)
                friction = pow(friction+0.06f, 3.0f);
            else
                friction = pow(friction, 3.0f);

            float this_slip = MIN(m_TclRange, (m_TclRange/2) * friction);
            //accel3 = accel3 - MIN(accel3, (slip - tcl_slip) / tcl_range);
            accel3 = accel3 - MIN(accel3, (Slip - this_slip) / m_TclRange);
        }

#if 1
        //if (raceType != RM_TYPE_QUALIF)
        {
            double height = 0.0;
            tTrkLocPos wp;
            double wx = car->pub.DynGCg.pos.x;
            double wy = car->pub.DynGCg.pos.y;
            RtTrackGlobal2Local(car->_trkPos.seg, wx, wy, &wp, TR_LPOS_SEGMENT);
            height = car->_pos_Z - RtTrackHeightL(&wp) - car->_wheelRadius(REAR_LFT) - suspHeight*2;

            if (height > 0.0)
            {
                accel1 = MAX(accel3, 1.0);
                accel2 = MAX(accel3, 1.0);
                accel3 = MAX(accel3, 1.0);
                accel4 = MAX(accel3, 1.0);
                accel1 = MIN(accel1, 0.2);
            }
        }
#endif
#endif

    return MIN(accel1, MIN(accel2, MIN(accel3, accel4)));
}

//==========================================================================*
// Filter Drifting
//--------------------------------------------------------------------------*
double TDriver::filterDrifting(double Acc)
{
  if(car->_speed_x < 5.0)
    return Acc;

  double Drifting = m_AbsDriftAngle;
  double DriftFactor = m_DriftFactor;

  if (m_Rain)
    {
      Drifting *= 1.5;
      DriftFactor *= 2;
    }

  // Decrease accelleration while drifting
  double DriftAngle = MAX(MIN(Drifting * 1.75, PI - 0.01),-PI + 0.01);
  if (m_AbsDriftAngle > m_LastAbsDriftAngle)
    Acc /= MAX(1.0,(DriftFactor * 150 * ( 1 - cos(DriftAngle))));
  else
    Acc /= MAX(1.0,(DriftFactor * 50 * ( 1 - cos(DriftAngle))));

  return MIN(1.0, Acc);
}

//==========================================================================*
// Learn brake at current speed
//--------------------------------------------------------------------------*
void TDriver::LearnBraking(double Pos)
{
  if (TDriver::Learning)
    {
      double Err = 0.0;
      if(m_LastBrake && m_LastTargetSpeed)
        {
          /*int PosIdx = oTrackDesc.IndexFromPos(Pos);
            if (PosIdx != oLastPosIdx)
            {
                double TargetSpeed = oTrackDesc.InitialTargetSpeed(PosIdx);
                Err = (float) (oCurrSpeed - TargetSpeed);
                if (fabs(Err) > 8.0)
                {
                    double Delta = - Sign(Err) * MAX(0.01,(fabs(Err) - 8.0)/50.0);
                    oTrackDesc.LearnFriction(PosIdx, Delta, 0.9);
                    oLastPosIdx = PosIdx;
                }
            }*/

          m_BrakeCoeff[m_LastBrakeCoefIndex] += (float)(Err * 0.002);
          m_BrakeCoeff[m_LastBrakeCoefIndex] = (float) MAX(0.5f, MIN(2.0, m_BrakeCoeff[m_LastBrakeCoefIndex]));
        }
    }
}

//==========================================================================*
// Detect flight
//--------------------------------------------------------------------------*
void TDriver::DetectFlight()
{
  double H[4];
  m_Jumping = -1.0;

  if (m_FirstJump)
    m_JumpOffset = 0.0;

  for (int I = 0; I < 4; I++)
    {
      tTrkLocPos Wp;
      float Wx = car->pub.DynGCg.pos.x;
      float Wy = car->pub.DynGCg.pos.y;
      RtTrackGlobal2Local(car->_trkPos.seg, Wx, Wy, &Wp, TR_LPOS_SEGMENT);
      H[I] = car->_pos_Z - RtTrackHeightL(&Wp) - car->_wheelRadius(I) + m_JumpOffset;
      if (m_Jumping < H[I])
        m_Jumping = H[I];
    }

  if (m_FirstJump)
    {
      m_JumpOffset = - m_Jumping - 0.03;
      LogSHADOW.debug("#JumpOffset: %g\n", m_JumpOffset);
      m_FirstJump = false;
    }

  if (m_Jumping > FLY_HEIGHT)
    {
      m_Flying = MIN(FLY_COUNT, m_Flying + (FLY_COUNT / 2));
    }
  else if (m_Flying > 0)
    {
      m_Flying--;
    }

  if ((m_Jumping > 0) || (m_Flying > 0))
    LogSHADOW.debug("#Jumping: %g %d\n", m_Jumping, m_Flying);
}

//==========================================================================*
// Prepare landing
//--------------------------------------------------------------------------*
double TDriver::FlightControl(double Steer)
{
  if ( m_Flying)
    {
      // Steer in direction of car movement
      double Angle = m_lastSpd - car->_yaw;
      NORM_PI_PI(Angle);
      int F = FLY_COUNT -  m_Flying;
      double T = MAX(0, MIN(1.0 * F / FLY_COUNT, 1));
      Steer = Steer * T + (1 - T) * Angle / car->_steerLock;
    }

  return Steer;
}

//==========================================================================*
// Calculate the skilling
//--------------------------------------------------------------------------*
void TDriver::CalcSkilling()
{
  (this->*CalcSkillingFoo)();
}
//==========================================================================*

//==========================================================================*
// Calculate the friction
//--------------------------------------------------------------------------*
double TDriver::CalcFriction(const double Crv)
{
  return (this->*CalcFrictionFoo)(Crv);
}
//==========================================================================*

//==========================================================================*
// Calculate the crv
//--------------------------------------------------------------------------*
double TDriver::CalcCrv(double Crv)
{
  return (this->*CalcCrvFoo)(Crv);
}
//==========================================================================*

//==========================================================================*
// Calculate the hairpin
//--------------------------------------------------------------------------*
double TDriver::CalcHairpin(double Speed, double AbsCrv)
{
  return (this->*CalcHairpinFoo)(Speed, AbsCrv);
}
//==========================================================================*
/*
//==========================================================================*
// shadow
//--------------------------------------------------------------------------*
double TDriver::CalcCrv_shadow(double Crv)
{
  double Offset = 800;

  if (oCrvComp)
  {
    if (Crv < 0.01)
      return 1.0;
    else
      return ((1+Crv) * (200 + Offset)/(1/Crv + Offset));
  }
  else
    return 1.0;
}

//==========================================================================*
*/
//==========================================================================*
// shadow
//--------------------------------------------------------------------------*
double TDriver::CalcCrv_shadow_LP1(double Crv)
{
  double Offset = 800;

  if (m_CrvComp)
    {
      if (Crv < 0.01)
        return 1.0;
      else
        return ((1+Crv) * (200 + Offset)/(1/Crv + Offset));
    }
  else
    return 1.0;
}

//==========================================================================*

//==========================================================================*
// If not used for a carset
//--------------------------------------------------------------------------*
double TDriver::CalcCrv_shadow_Identity(double Crv)
{
  return 1.0;
}
//==========================================================================*
//==========================================================================*
// simplix_sc
//--------------------------------------------------------------------------*
double TDriver::CalcCrv_shadow_SC(double Crv)
{
  double Offset = 1300;

  if (m_CrvComp)
    {
      if (Crv < 0.0085)
        return 1.0;
      else
        return ((1+Crv) * (400 + Offset)/(1/Crv + Offset));
    }
  else
    return 1.0;
}

//==========================================================================*
//==========================================================================*
// shadow_36GP
//--------------------------------------------------------------------------*
double TDriver::CalcCrv_shadow_36GP(double Crv)
{
  double Offset = 1300;

  if (m_CrvComp)
    {
      if (Crv < 0.0085)
        return 1.0;
      else
        return MIN(1.5,MAX(1.0,((1+Crv) * (400 + Offset)/(1/Crv + Offset))));
    }
  else
    return 1.0;
}

//==========================================================================*

//==========================================================================*
// If not used for a carset
//--------------------------------------------------------------------------*
double TDriver::CalcHairpin_shadow_Identity(double Speed, double AbsCrv)
{
  return Speed;
}

//==========================================================================*

//==========================================================================*
// Old Default function
//--------------------------------------------------------------------------*
double TDriver::CalcHairpin_shadow(double Speed, double AbsCrv)
{

  if (TDriver::UseGPBrakeLimit)
    {
      if (fabs(AbsCrv) > 1/15.0)
        Speed *= 0.20;                             // Filter hairpins
      else if (fabs(AbsCrv) > 1/25.0)
        Speed *= 0.30;                             // Filter hairpins
      else if (fabs(AbsCrv) > 1/40.0)
        Speed *= 0.70;                             // Filter hairpins
      else if (fabs(AbsCrv) > 1/45.0)
        Speed *= 0.84;                             // Filter hairpins
      else if (Speed > 112)                        // (111,11 m/s = 400 km/h)
        Speed = 112;
    }
  else
    {
      if (fabs(AbsCrv) > 1/40.0)
        Speed *= 0.70;                             // Filter hairpins
      else if (fabs(AbsCrv) > 1/45.0)
        Speed *= 0.84;                             // Filter hairpins
      else if (Speed > 112)                        // (111,11 m/s = 400 km/h)
        Speed = 112;
    }

  if (AbsCrv < 1/10.0)
    {
      if (TDriver::UseGPBrakeLimit)
        {
          if (Speed < 6.0)
            Speed = 6.0;
        }
      else
        {
          if (Speed < 12.0)
            Speed = 12.0;
        }
    }
  else
    {
      if (TDriver::UseGPBrakeLimit)
        {
          if (Speed < 3.0)
            Speed = 3.0;
        }
      else
        {
          if (Speed < 12.0)
            Speed = 12.0;
        }
    }

  return Speed;
}

//==========================================================================*
//==========================================================================*
// If not used for a carset
//--------------------------------------------------------------------------*
double TDriver::CalcFriction_shadow_Identity(const double Crv)
{
  return 1.0;
}

//==========================================================================*
//==========================================================================*
// shadow_ls2
//--------------------------------------------------------------------------*
double TDriver::CalcFriction_shadow_LS2(const double Crv)
{
  double AbsCrv = fabs(Crv);

  if (AbsCrv > 1/12.0)
    m_XXX = 0.60;
  else if ((AbsCrv > 1/15.0) && (m_XXX > 0.70))
    m_XXX = 0.70;
  else if ((AbsCrv > 1/18.0) && (m_XXX > 0.80))
    m_XXX = 0.80;
  else if ((AbsCrv > 1/19.0) && (m_XXX > 0.90))
    m_XXX = 0.90;
  else if ((AbsCrv > 1/20.0) && (m_XXX > 0.99))
    m_XXX = 0.99;
  else
    m_XXX = MIN(1.0, m_XXX + 0.0003);

  double FrictionFactor = 1.0;

  if (AbsCrv > 0.10)
    FrictionFactor = 0.84;
  else if (AbsCrv > 0.045)
    FrictionFactor = 0.85;
  else if (AbsCrv > 0.03)
    FrictionFactor = 0.86;
  else if (AbsCrv > 0.012)
    FrictionFactor = 1.0;
  else if (AbsCrv > 0.01)
    FrictionFactor = 1.01;
  else if (AbsCrv > 0.0075)
    FrictionFactor = 1.015;
  else if (AbsCrv > 0.005)
    FrictionFactor = 1.025;

  return m_XXX * FrictionFactor;
}

//==========================================================================*
//==========================================================================*
// shadow_LP1
//--------------------------------------------------------------------------*
double TDriver::CalcFriction_shadow_LP1(const double Crv)
{
  double AbsCrv = fabs(Crv);

  if (AbsCrv > 1/12.0)
    m_XXX = 0.60;
  else if ((AbsCrv > 1/15.0) && (m_XXX > 0.65))
    m_XXX = 0.65;
  else if ((AbsCrv > 1/18.0) && (m_XXX > 0.75))
    m_XXX = 0.75;
  else if ((AbsCrv > 1/19.0) && (m_XXX > 0.83))
    m_XXX = 0.83;
  else if ((AbsCrv > 1/20.0) && (m_XXX > 0.90))
    m_XXX = 0.90;
  else
    m_XXX = MIN(1.0, m_XXX + 0.0003);

  double FrictionFactor = 0.95;

  if (AbsCrv > 0.10)
    FrictionFactor = 0.44;
  else if (AbsCrv > 0.05)
    FrictionFactor = 0.53;
  else if (AbsCrv > 0.045)
    FrictionFactor = 0.74;
  else if (AbsCrv > 0.03)
    FrictionFactor = 0.83;
  else if (AbsCrv > 0.02)
    FrictionFactor = 0.92;
  else if (AbsCrv > 0.01)
    FrictionFactor = 0.93;
  else if (AbsCrv > 0.005)
    FrictionFactor = 0.95;

  return FrictionFactor * m_XXX;
}

//==========================================================================*
//==========================================================================*
// shadow_REF
//--------------------------------------------------------------------------*
double TDriver::CalcFriction_shadow_REF(const double Crv)
{
  double AbsCrv = fabs(Crv);

  if (AbsCrv > 1/12.0)
    m_XXX = 0.60;
  else if ((AbsCrv > 1/15.0) && (m_XXX > 0.65))
    m_XXX = 0.65;
  else if ((AbsCrv > 1/18.0) && (m_XXX > 0.75))
    m_XXX = 0.75;
  else if ((AbsCrv > 1/19.0) && (m_XXX > 0.83))
    m_XXX = 0.83;
  else if ((AbsCrv > 1/20.0) && (m_XXX > 0.90))
    m_XXX = 0.90;
  else
    m_XXX = MIN(1.0, m_XXX + 0.0003);

  double FrictionFactor = 0.95;

  if (AbsCrv > 0.10)
    FrictionFactor = 0.44;
  else if (AbsCrv > 0.05)
    FrictionFactor = 0.53;
  else if (AbsCrv > 0.045)
    FrictionFactor = 0.74;
  else if (AbsCrv > 0.03)
    FrictionFactor = 0.83;
  else if (AbsCrv > 0.02)
    FrictionFactor = 0.92;
  else if (AbsCrv > 0.01)
    FrictionFactor = 0.93;
  else if (AbsCrv > 0.005)
    FrictionFactor = 0.95;

  return FrictionFactor * m_XXX;
}

//==========================================================================*
//==========================================================================*
// Skilling
//--------------------------------------------------------------------------*
void TDriver::CalcSkill()
{
  if (Skilling && (RM_TYPE_PRACTICE != m_Situation->_raceType)
      && m_Strategy->OutOfPitlane())
    {
      if ((SkillAdjustTimer == -1.0) || (CurrSimTime - SkillAdjustTimer > SkillAdjustLimit))
        {
          double Rand1 = (double) getRandom() / 65536.0;
          double Rand2 = (double) getRandom() / 65536.0;
          double Rand3 = (double) getRandom() / 65536.0;

          // acceleration to use in current time limit
          DecelAdjustTarget = (Skill / 4 * Rand1);

          // brake to use
          BrakeAdjustTarget = MAX(0.85, 1.0 - MAX(0.0, Skill/15 * (Rand2 - 0.85)));
          LogSHADOW.debug("Brake Adjust Target = %.2f\n", BrakeAdjustTarget);

          // how long this skill mode to last for
          SkillAdjustLimit = 5.0 + Rand3 * 50.0;
          SkillAdjustTimer = CurrSimTime;

          if (DecelAdjustPerc < DecelAdjustTarget)
            DecelAdjustPerc += MIN(m_Situation->deltaTime*4, DecelAdjustTarget - DecelAdjustPerc);
          else
            DecelAdjustPerc -= MIN(m_Situation->deltaTime*4, DecelAdjustPerc - DecelAdjustTarget);

          if (BrakeAdjustPerc < BrakeAdjustTarget)
            BrakeAdjustPerc += MIN(m_Situation->deltaTime*2, BrakeAdjustTarget - BrakeAdjustPerc);
          else
            BrakeAdjustPerc -= MIN(m_Situation->deltaTime*2, BrakeAdjustPerc - BrakeAdjustTarget);
        }
      LogSHADOW.debug("CalcSkill DAP: %g (%g)\n ", DecelAdjustPerc,(1 - DecelAdjustPerc/10));

    }
}

//==========================================================================*
//==========================================================================*
// shadow_TRB1
// shadow_GP36
//--------------------------------------------------------------------------*
void TDriver::CalcSkilling_shadow()
{
  SkillGlobal = SkillGlobal/10.0;
  SkillDriver = SkillDriver/3.0;
  Skill = SkillScale * (SkillGlobal + SkillDriver) + SkillOffset;
}

//==========================================================================*
//==========================================================================*
// shadow_ls1
//--------------------------------------------------------------------------*
void TDriver::CalcSkilling_shadow_LS1()
{
  SkillGlobal = SkillGlobal/10.0;
  SkillDriver = SkillDriver/3.0;
  Skill = SkillScale * (SkillGlobal + SkillDriver) + SkillOffset;
}
//==========================================================================*
//==========================================================================*
// shadow_ls2
//--------------------------------------------------------------------------*
void TDriver::CalcSkilling_shadow_LS2()
{
  SkillGlobal = SkillGlobal/10.0;
  SkillDriver = SkillDriver/3.0;
  Skill = SkillScale * (SkillGlobal + SkillDriver) + SkillOffset;
}
//==========================================================================*
//==========================================================================*
// shadow_MPA1
//--------------------------------------------------------------------------*
void TDriver::CalcSkilling_shadow_MPA1()
{
  SkillGlobal = SkillGlobal/10.0;
  SkillDriver = SkillDriver/3.0;
  Skill = SkillScale * (SkillGlobal + SkillDriver) + SkillOffset;
}
//==========================================================================*
//==========================================================================*
// shadow_SC
//--------------------------------------------------------------------------*
void TDriver::CalcSkilling_shadow_SC()
{
  SkillScale = SkillScale/50.0;
  SkillDriver = SkillDriver / ((50.0 - SkillGlobal)/40.0);
  Skill = SkillScale * (SkillGlobal + SkillDriver * 2)
      * (1.0 + SkillDriver) + SkillOffset;
}

//==========================================================================*
//==========================================================================*
// shadow_lp1
//--------------------------------------------------------------------------*
void TDriver::CalcSkilling_shadow_LP1()
{
  SkillGlobal = SkillGlobal/10.0;
  SkillDriver = SkillDriver/3.0;
  Skill = SkillScale * (SkillGlobal + SkillDriver) + SkillOffset;
}
//==========================================================================*
//==========================================================================*
// Set name of robot (and other appendant features)
//--------------------------------------------------------------------------*
void TDriver::SetBotName(const char* Value)
{
  m_CarType = Value;
  LogSHADOW.debug("#Car Name    : %s\n" , m_CarType);
};
//==========================================================================*
//==========================================================================*
// Set scaling factor for avoiding racinglines
//--------------------------------------------------------------------------*
void TDriver::ScaleSide(float FactorMu, float FactorBrake)
{
  m_SideScaleMu = FactorMu;
  m_SideScaleBrake = FactorBrake;
}
//==========================================================================*
//==========================================================================*
// Set additional border to outer side
//--------------------------------------------------------------------------*
void TDriver::SideBorderOuter(float Factor)
{
  m_SideBorderOuter = Factor;
}
//==========================================================================*
//==========================================================================*
// Set additional border to inner side
//--------------------------------------------------------------------------*
void TDriver::SideBorderInner(float Factor)
{
  m_SideBorderInner = Factor;
}
//==========================================================================*
//==========================================================================*
// Meteorology
//--------------------------------------------------------------------------*
void TDriver::Meteorology()
{
  tTrackSeg *Seg;
  tTrackSurface *Surf;
  m_RainIntensity = 0;
  m_WeatherCode = GetWeather();
  Seg = track->seg;

  for ( int I = 0; I < track->nseg; I++)
    {
      Surf = Seg->surface;
      m_RainIntensity = MAX(m_RainIntensity, Surf->kFrictionDry / Surf->kFriction);
      GfLogDebug("# %.4f, %.4f %s\n",Surf->kFriction, Surf->kRollRes, Surf->material);
      Seg = Seg->next;
    }

  m_RainIntensity -= 1;

  if (m_RainIntensity > 0)
    {
      m_Rain = true;
      m_cm.MU_SCALE *= m_ScaleMuRain;
      m_cm.BRAKESCALE *= m_ScaleBrakeRain;
      m_TclSlip = MIN(m_TclSlip, 2.0);
      //Param.Fix.oBorderOuter += 0.5;
      //Param.oCarParam.oScaleMinMu = 1.0;
    }
  else
    m_Rain = false;
}

//==========================================================================*
// Estimate weather
//--------------------------------------------------------------------------*
int TDriver::GetWeather()
{
  return (track->local.rain << 4) + track->local.water;
};

//==========================================================================*
//==========================================================================*
// Check if pit sharing is activated
//--------------------------------------------------------------------------*
bool TDriver::CheckPitSharing()
{
  const tTrackOwnPit* OwnPit = car->_pit;           // Get my pit

  if (OwnPit == NULL)                               // If pit is NULL
    {                                                 // nothing to do
      LogSHADOW.debug("\n\n#Pit = NULL\n\n");       // here
      return false;
    }

  if (OwnPit->freeCarIndex > 1)
    {
      LogSHADOW.debug("\n\n#PitSharing = true\n\n");
      return true;
    }
  else
    {
      LogSHADOW.debug("\n\n#PitSharing = false\n\n");
      return false;
    }
}

double TDriver::TyreConditionFront()
{
    LogSHADOW.debug("Tyre Condition 0 = %.f - Tyre Condition 1 = %.f\n", car->_tyreCondition(0), car->_tyreCondition(1));
    return MIN(car->_tyreCondition(0), car->_tyreCondition(1));
}

double TDriver::TyreConditionRear()
{
    LogSHADOW.debug("Tyre Condition 2 = %.f - Tyre Condition 3 = %.f\n", car->_tyreCondition(2), car->_tyreCondition(3));
    return MIN(car->_tyreCondition(2), car->_tyreCondition(3));
}

double TDriver::TyreTreadDepthFront()
{
  double Right = (car->_tyreTreadDepth(0) - car->_tyreCritTreadDepth(0));
  double Left = (car->_tyreTreadDepth(1) - car->_tyreCritTreadDepth(1));
  return 100 * MIN(Right, Left);
}

double TDriver::TyreTreadDepthRear()
{
  double Right = (car->_tyreTreadDepth(2) - car->_tyreCritTreadDepth(2));
  double Left = (car->_tyreTreadDepth(3) - car->_tyreCritTreadDepth(3));
  return 100 * MIN(Right, Left);
}

