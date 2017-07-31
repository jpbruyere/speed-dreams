/***************************************************************************

    file                 : driver.cpp
    created              : Thu Dec 20 01:21:49 CET 2002
    copyright            : (c) 2005-2006 Andrew Sumner, 2000 Remi Coulom, 2004 Bernhard Wymann
    email                : andrew@onepixel.com.au
    version              : $Id$

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   It's the same old story. Nobody cares for a robot. Just wind him up,  *
 *   turn him loose, and grease him every thousand miles.                  *
 *
 *   My this has been a lot of work.
 *                                                                         *
 ***************************************************************************/

//#define DEBUGMSG
#include "driver.h"
#include "name.h"

#ifdef WIN32
#define snprintf _snprintf
#endif
//#define OVERTAKE_DEBUG_MSG 1
//#define STEER_DEBUG_MSG 1
//#define STATE_DEBUG_MSG 1
//#define SPEED_DEBUG_MSG 1
#define CONTROL_SKILL

const float Driver::MAX_UNSTUCK_ANGLE = (float)(15.0/180.0*PI);  // [radians] If the angle of the car on the track is smaller, we assume we are not stuck.
const float Driver::MIN_UNSTUCK_ANGLE = 2.4f;
const float Driver::UNSTUCK_TIME_LIMIT = 1.2f;    // [s] We try to get unstuck after this time.
const float Driver::MAX_UNSTUCK_SPEED = 5.0f;    // [m/s] Below this speed we consider being stuck.
const float Driver::MIN_UNSTUCK_DIST = 3.0f;    // [m] If we are closer to the middle we assume to be not stuck.
const float Driver::G = 9.81f;        // [m/(s*s)] Welcome on Earth.
const float Driver::FULL_ACCEL_MARGIN = 1.0f;    // [m/s] Margin reduce oscillation of brake/acceleration.
const float Driver::SHIFT = 0.9f;       // [-] (% of rpmredline) When do we like to shift gears.
const float Driver::SHIFT_MARGIN = 4.0f;     // [m/s] Avoid oscillating gear changes.
float Driver::ABS_SLIP = 1.8f;      // [m/s] range [0..10]
float Driver::ABS_RANGE = 5.0f;      // [m/s] range [0..10]
const float Driver::ABS_MINSPEED = 3.0f;     // [m/s] Below this speed the ABS is disabled (numeric, division by small numbers).
const float Driver::TCL_SLIP = 1.6f;      // [m/s] range [0..10]
const float Driver::TCL_RANGE = 10.0f;      // [m/s] range [0..10]
#define K1999AVOIDSTEER 0
#if K1999AVOIDSTEER
const float Driver::LOOKAHEAD_CONST = 8.0f;//17.0f;    // [m]
const float Driver::LOOKAHEAD_FACTOR = 0.10f;    // [-]
#else
const float Driver::LOOKAHEAD_CONST = 16.0f;//17.0f;    // [m]
const float Driver::LOOKAHEAD_FACTOR = 0.14f; // 0.33f;    // [-]
#endif
const float Driver::WIDTHDIV = 3.0f;      // [-] Defines the percentage of the track to use (2/WIDTHDIV).
const float Driver::SIDECOLL_MARGIN = 1.0f;     // [m] Distance between car centers to avoid side collisions.
const float Driver::BORDER_OVERTAKE_MARGIN = 0.5f;   // [m]
const float Driver::OVERTAKE_OFFSET_SPEED = 14.0f;   // [m/s] Offset change speed.
const float Driver::PIT_LOOKAHEAD = 6.0f;     // [m] Lookahead to stop in the pit.
const float Driver::PIT_BRAKE_AHEAD = 200.0f;    // [m] Workaround for "broken" pitentries.
const float Driver::PIT_MU = 0.4f;       // [-] Friction of pit concrete.
const float Driver::MAX_SPEED = 84.0f;      // [m/s] Speed to compute the percentage of brake to apply.
const float Driver::MAX_FUEL_PER_METER = 0.0008f;   // [liter/m] fuel consumtion.
const float Driver::CLUTCH_SPEED = 5.0f;     // [m/s]
const float Driver::CENTERDIV = 0.1f;      // [-] (factor) [0.01..0.6].
const float Driver::DISTCUTOFF = 55.0f;     // [m] How far to look, terminate while loops.
const float Driver::OVERTAKE_TIME_CUTOFF = 2.5f;     // [m] How far to look, terminate while loops.
const float Driver::CATCH_FACTOR = 18.0f;     // [-] select MIN(catchdist, dist*CATCH_FACTOR) to overtake.
const float Driver::CLUTCH_FULL_MAX_TIME = 0.7f;   // [s] Time to apply full clutch.
const float Driver::USE_LEARNED_OFFSET_RANGE = 0.2f;  // [m] if offset < this use the learned stuff
static const float STEER_DIRECTION_GAIN = 1.0f; ///< [-] Gain to apply for basic steerin
static const float STEER_PREDICT_GAIN = 0.11f;
static const float STEER_DRIFT_GAIN = 0.01f; ///< [-] Gain for drift compensation
static const float STEER_AVOIDANCE_GAIN = 0.01f; ///< [lograd/m] Gain for border avoidance steerin

static const double DivLength = 3.0;   // Length of path elements in meters
static const double MAXINCFACTOR = 2.6;
 
static const double SecurityR = 100.0; // Security radius
static double SideDistExt = 2.0; // Security distance wrt outside
static double SideDistInt = 1.0; // Security distance wrt inside 

static int telemetry[128] = {0};
static int pitstatus[128] = {0};
//static int pitdamage[128] = {0};
//static double pittime[128] = {0.0};

// Static variables.
Cardata *Driver::cardata = NULL;
double Driver::currentsimtime;

enum { DRWD = 0, DFWD = 1, D4WD = 2 };
enum { FLYING_FRONT = 1, FLYING_BACK = 2, FLYING_BOTH = 3 };

#define LINE_START 0
enum { LINE_MID = LINE_START, LINE_RL, LINE_RIGHT, LINE_LEFT };
//enum { LINE_RIGHT=0, LINE_LEFT, LINE_RL };

static double tAvoidRight[MaxDivs];
static double tAvoidLeft[MaxDivs];
static double tSegRadius[MaxSegments];
static double tAvoidBoost[MaxDivs];

/////////////////////////////////////////////////////////////////////////////
// Some utility macros and functions
/////////////////////////////////////////////////////////////////////////////
#ifndef ROB_SECT_ARBITRARY
static void RtGetCarindexString( int index, const char *, char, char *dest, int destLength )
{
 snprintf( dest, destLength, "%d", index );
}
#endif
#ifdef VERBOSE
#include <iostream.h>
#define OUTPUT(x) do {(std::cout << "HYMIE: " << x << '\n').flush();} while(0)
#else
#define OUTPUT(x)
#endif
 
#define FATAL(x) do{if (x){OUTPUT("Fatal error: " << #x); exit(1);}}while(0)
#define SEGTYPE2(seg, n) (SEGTYPE(seg) == TR_STR ? TR_STR \
           : !n || seg->length*0.5>getDistToSegStart(car, 0) ? seg->type : seg->next->type) 

//#define SEGTYPENEXT(seg) (seg->type == TR_STR || seg->radius >= 130.0 || (tSegArc[seg->id]*(seg->radius*6.5) <= seg->radius) ? TR_STR : seg->type) 
//#define SEGTYPE(seg) (seg->type != TR_STR && (SEGTYPENEXT(seg) == seg->type || (tSegArc[seg->id]*(seg->radius*6.5) > seg->radius && seg->radius <= 120.0 /*&& tSegLength[tSegID[seg->id]] > 10.0*/)) ? seg->type : TR_STR)
#define SEGTYPENEXT(seg) (seg->type == TR_STR || seg->radius >= 130.0 || (tSegLength[seg->id] <= seg->radius) ? TR_STR : seg->type) 
#define SEGTYPE(seg) (seg->type != TR_STR && (SEGTYPENEXT(seg) == seg->type || (tSegLength[seg->id] > seg->radius && seg->radius <= 120.0 /*&& tSegLength[tSegID[seg->id]] > 10.0*/)) ? seg->type : TR_STR)
#define STRAIGHT_RADIUS 2000.0

#define RANDOM_SEED 0xfded
#define RANDOM_A    1664525
#define RANDOM_C    1013904223

void Driver::SetRandomSeed(unsigned int seed)
{
 random_seed = seed ? seed : RANDOM_SEED;
 return;
}

unsigned int Driver::getRandom()
{
 random_seed = RANDOM_A * random_seed + RANDOM_C;
 return (random_seed >> 16);
}

static double Mag(double x, double y)
{
 return sqrt(x * x + y * y);
}
 
/////////////////////////////////////////////////////////////////////////////
// Update tx and ty arrays
/////////////////////////////////////////////////////////////////////////////
void Driver::UpdateTxTy(int i)
{
 tx[rl][i] = tLane[rl][i] * txRight[rl][i] + (1 - tLane[rl][i]) * txLeft[rl][i];
 ty[rl][i] = tLane[rl][i] * tyRight[rl][i] + (1 - tLane[rl][i]) * tyLeft[rl][i];
}                                                                               

/////////////////////////////////////////////////////////////////////////////
// Set segment info
/////////////////////////////////////////////////////////////////////////////
void Driver::InitSegInfo()
{
 //if (rl != LINE_RL)
 // return;

 memset(tSegLength, 0, sizeof(tSegLength));
 memset(tSegRadius, 0, sizeof(tSegRadius));
 memset(tRadius, 0, sizeof(tRadius));
 memset(tSegID, 0, sizeof(tSegID));

 tTrackSeg *seg = track->seg;
 tTrackSeg *nseg = seg;
 int last_type = nseg->type;
 double last_radius = nseg->radius;
 double first_radius = nseg->radius;
 double curlength = 0.0f;
 int curid = 0;

 initRadius();

 {
  memset(tSegArc, 0, sizeof(tSegArc));
  double arc_total = fabs(nseg->arc);
  double length_total = nseg->length;
  int arc_count = 1;
  tTrackSeg *firstseg = nseg;

  do {
   if (last_type == nseg->type && last_type != TR_STR && nseg->radius == last_radius)
   {
    arc_count++;
    arc_total += fabs(nseg->arc);
    length_total += nseg->length;
   }
   else
   {
    if (last_type != TR_STR)
    {
     double arc = (arc_total/arc_count);
     arc -= arc/10;
     arc *= ((double)arc_count * (arc*1.2));
 
     for (;firstseg != nseg; firstseg = firstseg->next)
     {
      tSegArc[firstseg->id] = arc;
      tSegLength[firstseg->id] = length_total;
     }
    }
    
    firstseg = nseg;
    last_radius = nseg->radius;
    last_type = nseg->type;
    arc_count = 1;
    arc_total = fabs(nseg->arc);
    length_total = nseg->length;
   }

   nseg = nseg->next;
  } while (nseg != seg);

  if (last_type != TR_STR)
  {
   double arc = (arc_total/(arc_count));
   arc *= ((double)arc_count * (arc*1.2));
 
   for (;firstseg != nseg; firstseg = firstseg->next)
   {
    tSegArc[firstseg->id] = arc;
    tSegLength[firstseg->id] = length_total;
   }
  }
 }

 do {
  if (last_type == nseg->type && (fabs(first_radius-nseg->radius) < nseg->radius/2 || (last_radius <= 70.0 && first_radius <= 70.0)))
  {
   curlength += nseg->length;
   tSegID[nseg->id] = curid;
  }
  else
  {
   tSegLength[curid] = curlength;
   curid++;
   tSegID[nseg->id] = curid;
   curlength = nseg->length;
   first_radius = nseg->radius;
  }

  tSegRadius[nseg->id] = nseg->radius;
  if (!nseg->radius)
   tSegRadius[nseg->id] = STRAIGHT_RADIUS;
  else if (tSegLength[nseg->id] < nseg->radius)
   tSegRadius[nseg->id] += nseg->radius - tSegLength[nseg->id];

  double delta = tLDelta[nseg->id] * 0.5 + tRDelta[nseg->id] * 0.5;
  if (delta < -0.03)
   tSegRadius[nseg->id] -= MIN(tSegRadius[nseg->id]/3, fabs(delta) * 80);
#if 0
  else
   tSegRadius[nseg->id] = MAX(nseg->radius * 0.7, tSegRadius[nseg->id] - (tSegArc[nseg->id] * (nseg->radius*6.5)) * 0.3);
#endif

  last_type = nseg->type;
  last_radius = nseg->radius;
//fprintf(stderr,"%d: %c%c %.3f SegArc=%.3f arc=%.3f\n",nseg->id,(nseg->type==TR_STR?'s':'c'),(SEGTYPE(nseg)==TR_STR?'s':'c'),nseg->radius,tSegArc[nseg->id],nseg->arc);fflush(stderr);
//#define SEGTYPE(seg) (seg->type != TR_STR && (SEGTYPENEXT(seg) == seg->type || (tSegArc[seg->id]*(seg->radius*6.5) > seg->radius && seg->radius <= 120.0 && tSegLength[tSegID[seg->id]] > 10.0)) ? seg->type : TR_STR)
  nseg = nseg->next;
 }
 while (nseg != seg);

 tSegLength[curid] = curlength;

 int i;
 last_type = TR_STR;
 double last_lmargin = 0.0, last_rmargin = 0.0;
 // calculate radii from an overtaking viewpoint
 for (i=0; i<=Divs; i++)
 {
  seg = tSegment[tDivSeg[i]];

  int prevdiv = i - 1;
  if (prevdiv < 0) prevdiv = Divs;

  tAvoidLeft[i] = tAvoidRight[i] = AvoidMargin/2;
  tLSlowSpeed[i] = tRSlowSpeed[i] = 0.0;

  //if (SEGTYPE(seg)!= TR_STR)
  if (seg->type != TR_STR && seg->radius <= 200.0)
  {
   double distratio = (((i - tSegDivStart[tDivSeg[i]]) * DivLength) + DivLength/2) * 1.2f / seg->length;
   double camber = ((seg->vertex[TR_SR].z-seg->vertex[TR_SL].z) * distratio) + ((seg->vertex[TR_ER].z-seg->vertex[TR_EL].z) * (1.0-distratio));
   double camber1 = ((seg->vertex[TR_SR].z-seg->vertex[TR_SL].z));
   double camber2=  ((seg->vertex[TR_ER].z-seg->vertex[TR_EL].z));
   double cambermargin = 0.0;

   if (seg->type == TR_RGT)
   {
    camber = -camber;
    camber2 = -camber2;
    camber1 = -camber1;
   }
   if (camber2 < camber1)
    camber -= (camber1 - camber2) * 1.5;
   else if (camber2 > camber1)
    camber += (camber2 - camber1) * 0.4;

#if 1
   if (camber < -0.2 && seg->radius <= 120.0)
   {
    // bad camber. slow us down.
    cambermargin = fabs(camber) * 2;
   }
   else if (camber > 0.4 && seg->radius <= 120.0 && tSegArc[seg->id]*(seg->radius*6.5) > seg->radius)
   {
    // good camber, speed us up.
    cambermargin = -(fabs(camber)*0.5);
   }
#endif

   {
    if (seg->type == TR_RGT)
    {
     tAvoidLeft[i] = MAX(car->_dimension_y*0.7, AvoidMargin*0.7);
     tAvoidRight[i] = (210.0 - MIN(210.0, (tSegArc[seg->id]*(seg->radius*6.5)))) / 100;
     tAvoidRight[i] = AvoidMargin + tAvoidRight[i] * tAvoidRight[i];
     tAvoidRight[i] += cambermargin;
     last_type = TR_RGT;
    }
    else if (seg->type == TR_LFT)
    {
     tAvoidRight[i] = MAX(car->_dimension_y*0.7, AvoidMargin*0.7);
     tAvoidLeft[i] = (210.0 - MIN(210.0, (tSegArc[seg->id]*(seg->radius*6.5)))) / 100;
     tAvoidLeft[i] = AvoidMargin + tAvoidLeft[i] * tAvoidLeft[i];
     tAvoidLeft[i] += cambermargin;
     last_type = TR_LFT;
    }
   }
  }
  else
  {
   if (last_type == TR_RGT && tAvoidLeft[i] < last_lmargin-0.2)
    tAvoidLeft[i] = last_lmargin-0.2;
   else if (last_type == TR_LFT && tAvoidRight[i] < last_rmargin-0.2)
    tAvoidRight[i] = last_rmargin-0.2;
   else
    last_type = TR_STR;
  }

  last_lmargin = tAvoidLeft[i];
  last_rmargin = tAvoidRight[i];
 }

 for (i=0; i<=Divs; i++)
 {
  tAvoidLeft[i] = MIN(tAvoidLeft[i], tLane[rl][i] * seg->width - 3.0); 
  tAvoidRight[i] = MIN(tAvoidRight[i], seg->width - (tLane[rl][i]*seg->width + 3.0));
 }
}

void Driver::SetSegmentInfo(const tTrackSeg *pseg, double d, int i, double l)
{
 if (pseg)
 {
  FATAL(pseg->id >= MaxSegments);
  tSegDist[pseg->id] = d;
  tSegIndex[pseg->id] = i;
  tElemLength[pseg->id] = l;
  if (pseg->id >= Segs)
   Segs = pseg->id + 1;

  v3d l, r, m;

  // following imported from berniw
  l.x = r.x = l.y = r.y = 0.0;
  double dzl = (pseg->vertex[TR_EL].z - pseg->vertex[TR_SL].z) / pseg->length;
  double dzr = (pseg->vertex[TR_ER].z - pseg->vertex[TR_SR].z) / pseg->length;
  l.z = pseg->vertex[TR_SL].z + dzl * pseg->length;
  r.z = pseg->vertex[TR_SR].z + dzr * pseg->length;

  m = (l+r)/2.0;
  if (fabs(r.z-m.z) > fabs(l.z-m.z))
   tSegHeight[pseg->id] = r.z;
  else
   tSegHeight[pseg->id] = l.z;
 }
}

/////////////////////////////////////////////////////////////////////////////
// Split the track into small elements
// ??? constant width supposed
/////////////////////////////////////////////////////////////////////////////
void Driver::SplitTrack(tTrack *ptrack)
{
 Segs = 0;
 OUTPUT("Analyzing track...");
 tTrackSeg *psegCurrent = ptrack->seg;

 double Distance = 0;
 double Angle = psegCurrent->angle[TR_ZS];
 double xPos = (psegCurrent->vertex[TR_SL].x +
                psegCurrent->vertex[TR_SR].x) / 2;
 double yPos = (psegCurrent->vertex[TR_SL].y +
                psegCurrent->vertex[TR_SR].y) / 2;

 int i = 0;

 for (i=0; i<Divs; i++)
  tLane[rl][i] = 0.5;

 i = 0;
 if (rl == LINE_RL)
 memset(tSegment, 0, sizeof(tSegment));

 do
 {
  int Divisions = 1 + int(psegCurrent->length / DivLength);
  double Step = psegCurrent->length / Divisions;
  double lmargin = 0.0, rmargin = 0.0;

  SetSegmentInfo(psegCurrent, Distance + Step, i, Step);
  tSegDivStart[psegCurrent->id] = i;

  if (rl == LINE_RL)
  {
   for (int side=0; side<2; side++)
   {
    tTrackSeg *psegside = psegCurrent->side[side];
    double margin = 0.0;
 
    while (psegside != NULL)
    {
     if (psegside->style == TR_WALL || psegside->style == TR_FENCE)
      margin = MAX(0.0, margin - (psegCurrent->type == TR_STR ? 0.5 : 1.0));
 
     if (psegside->style != TR_PLAN ||
         psegside->surface->kFriction < psegCurrent->surface->kFriction*0.8 ||
         psegside->surface->kRoughness > MAX(0.02, psegCurrent->surface->kRoughness*1.2) ||
         psegside->surface->kRollRes > MAX(0.005, psegCurrent->surface->kRollRes*1.2))
      break;
 
     if (track->pits.type != TR_PIT_NONE)
     {
      if (((side == TR_SIDE_LFT && track->pits.side == TR_LFT) ||
           (side == TR_SIDE_RGT && track->pits.side == TR_RGT)))
      {
       double pitstart = track->pits.pitEntry->lgfromstart - 50.0;
       double pitend = track->pits.pitExit->lgfromstart + track->pits.pitExit->length + 50.0;
       if (pitstart > pitend)
       {
        if (psegCurrent->lgfromstart >= pitstart)
         pitend += track->length;
        else
         pitstart -= track->length;
       }
       if (psegCurrent->lgfromstart >= pitstart && psegCurrent->lgfromstart <= pitend)
        break;
      }
     }

     double thiswidth = MIN(psegside->startWidth, psegside->endWidth) * 1.0;
     if (psegCurrent->type == TR_STR)
     if ((side == TR_SIDE_LFT && (psegCurrent->type == TR_RGT || psegCurrent->next->type != TR_LFT)) ||
         (side == TR_SIDE_RGT && (psegCurrent->type == TR_LFT || psegCurrent->next->type != TR_RGT)))
      thiswidth *= 0.6;
     margin += thiswidth;
     psegside = psegside->side[side];
    }

    margin = MAX(0.0, margin - car->_dimension_y*0.55);

    if (margin > 0.0)
    {
     margin /= psegCurrent->width;
     if (side == TR_SIDE_LFT)
      lmargin += margin;
     else
      rmargin += margin;
    }
   }
  }

  for (int j = Divisions; --j >= 0;)
  {
   double cosine = cos(Angle);
   double sine = sin(Angle);
   
   if (psegCurrent->type == TR_STR)
   {
    xPos += cosine * Step;
    yPos += sine * Step;
   }
   else
   {
    double r = psegCurrent->radius;
    double Theta = psegCurrent->arc / Divisions;
    double L = 2 * r * sin(Theta / 2);
    double x = L * cos(Theta / 2);
    double y;
    if (psegCurrent->type == TR_LFT)
    {
     Angle += Theta;
     y = L * sin(Theta / 2);
    }
    else
    {
     Angle -= Theta;
     y = -L * sin(Theta / 2);
    }
    xPos += x * cosine - y * sine;
    yPos += x * sine + y * cosine;
   }

   double dx = -psegCurrent->width * sin(Angle) / 2;
   double dy = psegCurrent->width * cos(Angle) / 2;
   txLeft[rl][i] = xPos + dx;
   tyLeft[rl][i] = yPos + dy;
   txRight[rl][i] = xPos - dx;
   tyRight[rl][i] = yPos - dy;
   tLane[rl][i] = 0.5;
   tLaneLMargin[i] = lmargin - (tLaneAdjust[i] > 0.0 ? tLaneAdjust[i] : 0.0);
   tLaneRMargin[i] = rmargin + (tLaneAdjust[i] < 0.0 ? tLaneAdjust[i] : 0.0);

   {
    tFriction[i] = psegCurrent->surface->kFriction * TrackFriction;
    if (tFriction[i] < 1) // ??? ugly trick for dirt
    {
     //tFriction[i] *= 0.90;
     fDirt = 1;
     //IntMargin = -1.5;
     //ExtMargin = 0.0;
    }
   }

   UpdateTxTy(i);

   Distance += Step;
   {
    tDivSeg[i] = psegCurrent->id;
    tSegment[psegCurrent->id] = psegCurrent;
   }
   i++;
   FATAL(i > MaxDivs);
  }

  psegCurrent = psegCurrent->next;
 }
 while (psegCurrent != ptrack->seg);

 Divs = i - 1;
 Width = psegCurrent->width;
 Length = Distance;

 if (rl == LINE_RL)
 {
  // evaluate changes in slope, used later for flying mitigation
  memset(tLSlope, 0, sizeof(tLSlope));
  memset(tRSlope, 0, sizeof(tRSlope));
  psegCurrent = ptrack->seg;
  do {
   int i = psegCurrent->id, j = psegCurrent->prev->id;
   double length = psegCurrent->length + (psegCurrent->length > 80.0f ? psegCurrent->length : 0.0f);

   if (length > FlyingWidth || psegCurrent->type != TR_STR)
   {
    tLSlope[i] = (psegCurrent->vertex[TR_EL].z - psegCurrent->vertex[TR_SL].z) * (10.0f / length);
    tRSlope[i] = (psegCurrent->vertex[TR_ER].z - psegCurrent->vertex[TR_SR].z) * (10.0f / length);
   }
   else
    tLSlope[i] = tRSlope[i] = 0.0f;
  
   if (tLSlope[i] < 0.0) tLSlope[i] *= 1.0f + MIN(1.0, fabs(tLSlope[j]-tLSlope[i])*0.3);
   if (tRSlope[i] < 0.0) tRSlope[i] *= 1.0f + MIN(1.0, fabs(tRSlope[j]-tRSlope[i])*0.3);
 
   psegCurrent = psegCurrent->next;
  }
  while (psegCurrent != ptrack->seg);

  psegCurrent = ptrack->seg;
  do {
   int i = psegCurrent->id;
   tTrackSeg *psegNext = psegCurrent->next;
   int j = psegNext->id;
 
   tLDelta[i] = atan((tLSlope[j] - tLSlope[i]) / 4.0);
   tRDelta[i] = atan((tRSlope[j] - tRSlope[i]) / 4.0);
 
   psegCurrent = psegCurrent->next;
  }
  while (psegCurrent != ptrack->seg);
 }

 if (0 && rl == LINE_RL)
 {
  for (i=0; i<Divs; i++)
  {
   double delta = MIN(tLDelta[tDivSeg[i]], tRDelta[tDivSeg[i]]);
   if (FlyingCaution > 0.0 && delta < -0.08)
   {
    // look for dangerous bumps in the track and slow the car down for them
    double gamma = fabs(delta) * 2.0f;
    if (gamma * (0.10+FlyingCaution*0.90) > 0.02)
    {
     int j;
     Flying[i] = 1.0;
     for (j=0; j<8; j++)
     {
      int div = ((i-j)+Divs)%Divs;
      tLaneLMargin[div] += FlyingCaution * 40;
      tLaneRMargin[div] += FlyingCaution * 40;
     }
     for (j=1; j<8; j++)
     {
      int div = ((i+j)+Divs)%Divs;
      tLaneLMargin[div] += FlyingCaution * 40;
      tLaneRMargin[div] += FlyingCaution * 40;
     }
    }
   }
  }
 }

 OUTPUT("Position of the last point (should be (0, 0))");
 OUTPUT("xPos = " << xPos);
 OUTPUT("yPos = " << yPos);
 OUTPUT("Number of path elements : " << Divs);
 OUTPUT("Segs = " << Segs);
 OUTPUT("Track length : " << Length);
 OUTPUT("Width : " << Width);
}

/////////////////////////////////////////////////////////////////////////////
// Compute the inverse of the radius
/////////////////////////////////////////////////////////////////////////////
double Driver::GetRInverse(int prev, double x, double y, int next, int rline)
{
 double x1 = tx[rline][next] - x;
 double y1 = ty[rline][next] - y;
 double x2 = tx[rline][prev] - x;
 double y2 = ty[rline][prev] - y;
 double x3 = tx[rline][next] - tx[rline][prev];
 double y3 = ty[rline][next] - ty[rline][prev];
 
 double det = x1 * y2 - x2 * y1;
 double n1 = x1 * x1 + y1 * y1;
 double n2 = x2 * x2 + y2 * y2;
 double n3 = x3 * x3 + y3 * y3;
 double nnn = sqrt(n1 * n2 * n3);
 
 return 2 * det / nnn;
}

/////////////////////////////////////////////////////////////////////////////
// Change lane value to reach a given radius
/////////////////////////////////////////////////////////////////////////////
void Driver::AdjustRadius(int prev, int i, int next, double TargetRInverse, double Security)
{
 double OldLane = tLane[rl][i];

 char first_time = 0;
 if (Security == -1)
 {
  first_time = 1;
  Security = SecurityZ;
 }
 
 //
 // Start by aligning points for a reasonable initial lane
 //
 tLane[rl][i] = (-(ty[rl][next] - ty[rl][prev]) * (txLeft[rl][i] - tx[rl][prev]) +
                  (tx[rl][next] - tx[rl][prev]) * (tyLeft[rl][i] - ty[rl][prev])) /
                ( (ty[rl][next] - ty[rl][prev]) * (txRight[rl][i] - txLeft[rl][i]) -
                  (tx[rl][next] - tx[rl][prev]) * (tyRight[rl][i] - tyLeft[rl][i]));
#if 0
 if (tLane[i] < -0.2)
  tLane[i] = -0.2;
 else if (tLane[i] > 1.2)
  tLane[i] = 1.2;
#else
 if (rl == LINE_RL)
 {
  if (tLane[rl][i] < -0.2-tLaneLMargin[i])
   tLane[rl][i] = -0.2-tLaneLMargin[i];
  else if (tLane[rl][i] > 1.2+tLaneRMargin[i])
   tLane[rl][i] = 1.2+tLaneRMargin[i];
 }
 if (rl == LINE_LEFT)
 {
  double margin = 4.0 / track->width;
  tLane[rl][i] = MAX(-0.2, MIN(margin, tLane[rl][i]));
 }
 else if (rl == LINE_RIGHT)
 {
  double margin = 1.0 - (4.0 / track->width);
  tLane[rl][i] = MIN(1.2, MAX(margin, tLane[rl][i]));
 }
 else if (rl == LINE_MID)
 {
  double margin = -1.2;
  tLane[rl][i] = MIN(1.2, MAX(margin, tLane[rl][i]));
 }
#endif
 UpdateTxTy(i);
 
 //
 // Newton-like resolution method
 //
 const double dLane = 0.0001;
 //double SInt = IntMargin - (CurveFactor > 0.0 ? CurveFactor * 5 : CurveFactor * 2);
 double SInt = (rl == LINE_RL ? (IntMargin - (CurveFactor > 0.0 ? CurveFactor * 5 : CurveFactor * 2)) : MAX(4.0, Width * 0.5 - 2.0));
 double SExt = (rl == LINE_RL ? ExtMargin : Width * 0.5);
 //tTrackSeg *seg = tSegment[tDivSeg[i]];
 //if (seg->type != TR_STR && seg->radius <= 60.0)
 // SInt += (65.0 - seg->radius) / 70.0;
 
 double dx = dLane * (txRight[rl][i] - txLeft[rl][i]);
 double dy = dLane * (tyRight[rl][i] - tyLeft[rl][i]);
 
 double dRInverse = GetRInverse(prev, tx[rl][i] + dx, ty[rl][i] + dy, next, rl);
 int pi = i - 1;
 if (pi < 0)
  pi += Divs;

 if (dRInverse > 0.000000001)
 {
  tLane[rl][i] += (dLane / dRInverse) * TargetRInverse;
 
  double ExtLane = (SExt + Security) / Width;
  double IntLane = (SInt + Security) / Width;
  if (ExtLane > 0.5)
   ExtLane = 0.5;
  if (IntLane > 0.5)
   IntLane = 0.5;
  if (rl == LINE_RL)
  {
   if (TargetRInverse >= 0.0)
   {
    IntLane -= tLaneLMargin[i];
    ExtLane -= tLaneRMargin[i];
   }
   else
   {
    ExtLane -= tLaneLMargin[i];
    IntLane -= tLaneRMargin[i];
   }
  }
 
  if (TargetRInverse >= 0.0)
  {
   if (tLane[rl][i] < IntLane)
    tLane[rl][i] = IntLane;
   if (1 - tLane[rl][i] < ExtLane)
   {
    if (1 - OldLane < ExtLane)
     tLane[rl][i] = MIN(OldLane, tLane[rl][i]);
    else
     tLane[rl][i] = 1 - ExtLane;
   }
  }
  else
  {
   if (tLane[rl][i] < ExtLane)
   {
    if (OldLane < ExtLane)
     tLane[rl][i] = MAX(OldLane, tLane[rl][i]);
    else
     tLane[rl][i] = ExtLane;
   }
   if (1 - tLane[rl][i] < IntLane)
    tLane[rl][i] = 1 - IntLane;
  }
 }

 //if (first_time)
 // tLane[i] += tLaneAdjust[i];

 UpdateTxTy(i);
}

/////////////////////////////////////////////////////////////////////////////
// Smooth path
/////////////////////////////////////////////////////////////////////////////
void Driver::Smooth(int Step)
{
 int prev = ((Divs - Step) / Step) * Step;
 int prevprev = prev - Step;
 int next = Step;
 int nextnext = next + Step;
 
 for (int i = 0; i <= Divs - Step; i += Step)
 {
  {
   double ri0 = GetRInverse(prevprev, tx[rl][prev], ty[rl][prev], i, rl);
   double ri1 = GetRInverse(i, tx[rl][next], ty[rl][next], nextnext, rl);
   double lPrev = Mag(tx[rl][i] - tx[rl][prev], ty[rl][i] - ty[rl][prev]);
   double lNext = Mag(tx[rl][i] - tx[rl][next], ty[rl][i] - ty[rl][next]);

   double TargetRInverse = (lNext * ri0 + lPrev * ri1) / (lNext + lPrev);
  
   double Security = lPrev * lNext / (8 * SecurityR);

   if (CurveFactor && ri0 * ri1 > 0)
   {
    double ac1 = fabs(ri0);
    double ac2 = fabs(ri1);
    {
     if (ac1 < ac2)
      ri0 += CurveFactor * (ri1 - ri0);
     else if (ac2 < ac1)
      ri1 += CurveFactor * (ri0 - ri1);
    }

    TargetRInverse = (lNext * ri0 + lPrev * ri1) / (lNext + lPrev);
   }

   AdjustRadius(prev, i, next, TargetRInverse, Security);
  }
 
  prevprev = prev;
  prev = i;
  next = nextnext;
  nextnext = next + Step;
  if (nextnext > Divs - Step)
   nextnext = 0;
 }
}

/////////////////////////////////////////////////////////////////////////////
// Interpolate between two control points
/////////////////////////////////////////////////////////////////////////////
void Driver::StepInterpolate(int iMin, int iMax, int Step)
{
 int next = (iMax + Step) % Divs;
 if (next > Divs - Step)
  next = 0;
 
 int prev = (((Divs + iMin - Step) % Divs) / Step) * Step;
 if (prev > Divs - Step)
  prev -= Step;
 
 double ir0 = GetRInverse(prev, tx[rl][iMin], ty[rl][iMin], iMax % Divs, rl);
 double ir1 = GetRInverse(iMin, tx[rl][iMax % Divs], ty[rl][iMax % Divs], next, rl);

 for (int k = iMax; --k > iMin;)
 {
  double x = double(k - iMin) / double(iMax - iMin);
  double TargetRInverse = x * ir1 + (1 - x) * ir0;
  AdjustRadius(iMin, k, iMax % Divs, TargetRInverse);
 }
}
 
/////////////////////////////////////////////////////////////////////////////
// Calls to StepInterpolate for the full path
/////////////////////////////////////////////////////////////////////////////
void Driver::Interpolate(int Step)
{
 if (Step > 1)
 {
  int i;
  for (i = Step; i <= Divs - Step; i += Step)
   StepInterpolate(i - Step, i, Step);
  StepInterpolate(i - Step, Divs, Step);
 }
}

void Driver::K1999InitTrack(tTrack* track, void **carParmHandle, tSituation *p)
{
 WingRInverse = 0.0018;
 /*
 TireAccel1 = 17.0;
 MaxBrake = 14.0;
 SlipLimit = 2.30;
 SteerSkid = 0.08;
 CurveFactor = 0.05;
 */
 TireAccel1 = 20.8;
 MaxBrake = 12.2;
 BrakeShift = 0.0f;
 TSlipLimit = 0.0;
 SlipLimit = 2.00;
 SteerSkid = 0.02;
 CurveAccel = 0.0;
 IntMargin = SideDistInt;
 ExtMargin = SideDistExt;
 SecurityZ = 0.0f;
 CurveFactor = 0.15;
 RevsChangeDown = 0.75;
 RevsChangeDownMax = 0.853;
 RevsChangeUp = 0.96;
 TrackFriction = 1.0;
 SteerGain = 1.2;
 SteerGain2 = 1.0;
 SteerGainDiv = 25.0;
 FlyingCaution = 0.0;
 FlyingWidth = 1.0;
 TurnAccel = 1.0;
 ABSFactor = 0.8;
 AvoidSpeed = 0.0;
 CAModifier = 0.0;
 SteerLimit = 0.72;//1.04;
 EdgeAllowance = 0.75;
 AvoidMargin = 1.5;
 FuelSpeedup = 0.0;
 TurnDecel = 1.0;
 BrakePressure = 1.0;
 BTBoost = 1.0;
 AvoidBTBoost = 0.98;
 Learning = 0.0;
 RaceLearning = 0.0;
 LearnLimit = 0;
 OverrideLearning = 0;
 NoTeamWaiting = 0;
 Iterations = 100;
 K1999Brakes = 0;
 NoAccelRelax = 0;
 TeamWaitTime = 1.5;
 MaxIncFactor = MAXINCFACTOR;
 ClutchTime = 0.9;
 PitEntryOffset = 0.0;
 LoadSVG = 0;
 SaveSVG = 0;

 float value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "WingRInverse", (char*) NULL, WingRInverse)))
  WingRInverse = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "TireAccel", (char*) NULL, TireAccel1)))
  TireAccel1 = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "MaxBrake", (char*) NULL, MaxBrake)))
  MaxBrake = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "ClutchTime", (char*) NULL, ClutchTime)))
  ClutchTime = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "BrakeShift", (char*) NULL, BrakeShift)))
  BrakeShift = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "SlipLimit", (char*) NULL, SlipLimit)))
  SlipLimit = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "TSlipLimit", (char*) NULL, TSlipLimit)))
  TSlipLimit = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "SteerSkid", (char*) NULL, SteerSkid)))
  SteerSkid = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "SteerLimit", (char*) NULL, SteerLimit)))
#if K1999AVOIDSTEER
  SteerLimit = (value < 1.0 ? MAX(1.0, value + 0.3) : value);
#else
  SteerLimit = value;
#endif
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "CurveAccel", (char*) NULL, CurveAccel)))
  CurveAccel = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "CurveFactor", (char*) NULL, CurveFactor)))
  CurveFactor = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "RevsChangeDown", (char*) NULL, RevsChangeDown)))
  RevsChangeDown = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "RevsChangeDownMax", (char*) NULL, RevsChangeDownMax)))
  RevsChangeDownMax = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "RevsChangeUp", (char*) NULL, RevsChangeUp)))
  RevsChangeUp = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "TrackFriction", (char*) NULL, TrackFriction)))
  TrackFriction = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "PitDamage", (char*) NULL, 5500)))
  strategy->setPitDamage((int) value);
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "FlyingCaution", (char*) NULL, FlyingCaution)))
  FlyingCaution = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "FlyingWidth", (char*) NULL, FlyingWidth)))
  FlyingWidth = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "SteerGain", (char*) NULL, SteerGain)))
  SteerGain = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "SteerGain2", (char*) NULL, SteerGain2)))
  SteerGain2 = MAX(value, 1.0);
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "IntMargin", (char*) NULL, IntMargin)))
  IntMargin = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "ExtMargin", (char*) NULL, ExtMargin)))
  ExtMargin = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "Security", (char*) NULL, SecurityZ)))
  SecurityZ = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "AvoidSpeed", (char*) NULL, AvoidSpeed)))
  AvoidSpeed = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "ABSFactor", (char*) NULL, ABSFactor)))
  ABSFactor = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "CAModifier", (char*) NULL, CAModifier)))
  CAModifier = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "TurnAccel", (char*) NULL, TurnAccel)))
  TurnAccel = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "EdgeAllowance", (char*) NULL, EdgeAllowance)))
  EdgeAllowance = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "FuelSpeedup", (char*) NULL, FuelSpeedup)))
  FuelSpeedup = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "AvoidMargin", (char*) NULL, AvoidMargin)))
  AvoidMargin = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "TurnDecel", (char*) NULL, TurnDecel)))
  TurnDecel = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "Learning", (char*) NULL, 0.0)))
  Learning = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "RaceLearning", (char*) NULL, 0.0)))
  RaceLearning = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "LearnLimit", (char*) NULL, 0.0)))
  LearnLimit = (int) value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "NoAccelRelax", (char*) NULL, 0.0)))
  NoAccelRelax = (int) value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "SaveSVG", (char*) NULL, 0.0)))
  SaveSVG = (int) value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "LoadSVG", (char*) NULL, 0.0)))
  LoadSVG = (int) value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "NoTeamWaiting", (char*) NULL, 0.0)))
  NoTeamWaiting = (int) value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "Iterations", (char*) NULL, 0.0)))
  Iterations = (int) value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "K1999Brakes", (char*) NULL, 0.0)))
  K1999Brakes = (int) value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "OverrideLearning", (char*) NULL, 0.0)))
  OverrideLearning = (int) value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "AvoidBTBoost", (char*) NULL, 0.9)))
  AvoidBTBoost = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "BTBoost", (char*) NULL, 1.0)))
  BTBoost = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "TeamWaitTime", (char*) NULL, 0.0)))
  TeamWaitTime = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "ABS_SLIP", (char*) NULL, 1.8)))
  ABS_SLIP = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "ABS_RANGE", (char*) NULL, 5.0)))
  ABS_RANGE = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "MaxIncFactor", (char*) NULL, MaxIncFactor)))
  MaxIncFactor = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "SteerGainDiv", (char*) NULL, SteerGainDiv)))
  SteerGainDiv = value;
 if (0.0 != (value = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "PitEntryOffset", (char*) NULL, PitEntryOffset)))
  PitEntryOffset = value;
 CA += CAModifier;
 if (!TSlipLimit)
  TSlipLimit = SlipLimit;

 memset(tAvoidBoost, 0, sizeof(BTBoost));

 tTrackSeg *seg;

 int i;

 for (rl = LINE_START; rl <= LINE_RL; rl++)
 {
  //
  // split track
  //
  //if (rl == LINE_START)
  SplitTrack(track);

  if (!learn || rl == LINE_RL)
   learn = new SegLearn(track, Learning, LearnLimit);
  LearningLoaded = 0;
  if (rl == LINE_RL)
  {
   if (Learning)
    loadLearning();
   if (LoadSVG)
    loadSVG();
  }

  //if (rl == LINE_RL)
   InitSegInfo();

  //if (rl == LINE_RL)
   Optimize();

  if (rl == LINE_RL && Learning)
   loadLearning();
  //if (LoadSVG)
  // loadSVG();

 }

 rl = LINE_RL;

 for (i=0; i<Divs; i++)
 {
  seg = tSegment[tDivSeg[i]];
  int prev = ((i - 1) + Divs) % Divs;
  tSpeed[i] = MIN(tSpeed[i], tSpeed[i]);
  tSpeed[i] = MIN(tSpeed[i], tSpeed[i]);
  if (SEGTYPE(seg) == TR_STR && tSpeed[i] > tSpeed[prev])
  {
   tSpeed[i] = MAX(tSpeed[i], tSpeed[prev] + (tSpeed[i] - tSpeed[prev]));
   tSpeed[i] = MAX(tSpeed[i], tSpeed[prev] + (tSpeed[i] - tSpeed[prev]));
  }
  double lane2left = tLane[rl][i] * seg->width;
  double lane2right = seg->width - lane2left;

  if (SEGTYPE(seg) == TR_RGT)
   tAvoidLeft[i] = MIN(tAvoidLeft[i], lane2left);
  else if (SEGTYPE(seg) == TR_LFT)
   tAvoidRight[i] = MIN(tAvoidRight[i], lane2right);
  else
  {
   tAvoidLeft[i] = MIN(tAvoidLeft[i], lane2left + 1.0);
   tAvoidRight[i] = MIN(tAvoidRight[i], lane2right + 1.0);
  }

  tMaxSpeed[i] = tSpeed[i];
  if (tSpeedAdjust[i] != 0.0)
  {
   tMaxSpeed[i] = tSpeedAdjust[i];
   tSpeedAdjust[i] = 0.0;
  }

  tLearnCount[i] = 0;
 }

 if (SaveSVG)
  saveSVG();

 rl = LINE_RL;
}


void Driver::ComputeFullPath()
{
 int i;

 {
  //
  // Smoothing loop
  //
  int MaxStep = 1;
  MaxStep = (rl == LINE_MID ? 16 : (LoadSVG ? (LoadSVG-1) : 132));
  for (int Step = MaxStep /*128*/; (Step /= 2) > 0;)
  {
   OUTPUT("Step = " << Step);
   for (i = Iterations * int(sqrt((float) Step)); --i >= 0;)
   {
    Smooth(Step);
    //if (i % 30 == 0)
    // EstimateSpeed(Step);
   }
   Interpolate(Step);
  }

  memset(Flying, 0, sizeof(Flying));
  double lastspeed = 0.0;
 
  //
  // Compute curvature and speed along the path
  //
  for (i = Divs; --i >= 0;)
  {
   double TireAccel = TireAccel1 * tFriction[i];
   if (0 && rl != LINE_RL) 
   {
    tTrackSeg *seg = (tSegment[tDivSeg[i]]);
    if (SEGTYPE(seg) != TR_STR)
     TireAccel = (TireAccel1 - 3.0) * tFriction[i];
   }
   int next = (i + 1) % Divs;
   int prev = (i - 1 + Divs) % Divs;
   int nnext = (i + 5) % Divs;
   tTrackSeg *tseg = (tSegment[tDivSeg[nnext]]);
   tTrackSeg *seg = tSegment[tDivSeg[i]];
   int iscorner = SEGTYPE(tseg);
   
   if (rl == LINE_LEFT)
   {
    if (iscorner == TR_RGT)
     TireAccel *= MAX(0.7, (1.0f - ((120.0f-seg->radius)/120.0f) * 0.5f));
    else 
     TireAccel *= 1.0f + (seg->width - 8.0) * 0.018;
     //TireAccel *= 2.5 + seg->width/10;
   }
   else if (rl == LINE_RIGHT)
   {
    if (iscorner == TR_LFT)
     TireAccel *= MAX(0.7, (1.0f - ((120.0f-seg->radius)/120.0f) * 0.5f));
    else 
     TireAccel *= 1.0f + (seg->width - 8.0) * 0.018;
     //TireAccel *= 2.5 + seg->width/10;
   }

   double rInverse = GetRInverse(prev, tx[rl][i], ty[rl][i], next, rl);

   tRInverse[rl][i] = rInverse;

   double MaxSpeed;
   double wingrinverse = WingRInverse;

   double sRInverse = rInverse;// * (1.0 + fabs(rInverse)*20) * 0.87;
   double delta = tLDelta[tDivSeg[i]] * (1.0 - tLane[rl][i]) + tRDelta[tDivSeg[i]] * tLane[rl][i];
   if (fabs(sRInverse) > fabs(rInverse) * 1.0)
    sRInverse = rInverse * 0.7;
   if (fabs(sRInverse) > wingrinverse * 1.01)
   {
    double tmpRInverse = sRInverse * (1.0 - fabs(sRInverse) * CurveAccel);
    MaxSpeed = sqrt(TireAccel / (fabs(tmpRInverse) - wingrinverse));
    //if (0)
     //MaxSpeed *= MAX(1.0, (1.28 - fabs(sRInverse)*15));   // speed car up on faster bends
 
    // adjust speed based on the camber of the corner
#if 1
    if (fabs(rInverse) > 0.002)
    {
     double distratio = (((i - tSegDivStart[tDivSeg[i]]) * DivLength) + DivLength/2) * 1.2f / seg->length;
     double camber = (((seg->vertex[TR_SR].z-seg->vertex[TR_SL].z) * distratio) + ((seg->vertex[TR_ER].z-seg->vertex[TR_EL].z) * (1.0-distratio))) / seg->width;
     double camber1 = ((seg->vertex[TR_SR].z-seg->vertex[TR_SL].z)) / seg->width;
     double camber2=  ((seg->vertex[TR_ER].z-seg->vertex[TR_EL].z)) / seg->width;

     if (rInverse < 0.0)
     {
      camber = -camber;
      camber2 = -camber2;
      camber1 = -camber1;
     }
     if (camber2 < camber1)
      camber -= (camber1 - camber2) * 3.0;
     else if (camber2 > camber1)
      camber += (camber2 - camber1) * 0.4;

     if (camber < -0.02) // && seg->radius <= 220.0 && tSegArc[seg->id]*(seg->radius*6.5) > seg->radius)
     {
      // bad camber. slow us down.
      MaxSpeed -= MIN(MaxSpeed/4, fabs(camber) * 20);
      if (rInverse > 0.0)
       tRSlowSpeed[i] = fabs(camber) * 20;
      else
       tLSlowSpeed[i] = fabs(camber) * 20;
      if (rl == LINE_RL)
      {
       if (SEGTYPE(seg) == TR_RGT && seg->radius <= 80.0f)
        tAvoidLeft[i] += fabs(camber) * 300.0f;
       else if (SEGTYPE(seg) == TR_LFT && seg->radius <= 80.0f)
        tAvoidRight[i] += fabs(camber) * 300.0f;
      }
     }
     else if (camber > 0.02)
     {
      // good camber, speed us up.
      MaxSpeed += camber * 10;
      if (seg->type == TR_LFT)
       tRSlowSpeed[i] = -(fabs(camber) * 20);
      else
       tLSlowSpeed[i] = -(fabs(camber) * 20);

      if (rl == LINE_RL)
      {
       if (SEGTYPE(seg) == TR_RGT)
        tAvoidLeft[i] = MAX(0.0f, tAvoidLeft[i] - camber);
       else if (SEGTYPE(seg) == TR_LFT)
        tAvoidRight[i] = MAX(0.0f, tAvoidRight[i] - camber);
      }
     }
    }
#endif
  
    // adjust speed for uphill/downhill corners
    //double slope = tLSlope[tDivSeg[i]] * (1.0 - tLane[rl][i]) + tRSlope[tDivSeg[i]] * tLane[rl][i];
    if (delta < -0.03f && fabs(rInverse) > 0.002 && MaxSpeed >= 30.0)
    {
     MaxSpeed -= MIN(MaxSpeed-30.0/10, fabs(delta) * 200);
     if (rInverse > 0.0)
      tRSlowSpeed[i] = fabs(delta) * 300;
     else
      tLSlowSpeed[i] = fabs(delta) * 300;
#if 0
     if (rInverse > 0.0)
      rInverse -= delta * 0.035f;
     else
      rInverse += delta * 0.035f;
#endif
    }
    //else if (delta > 0.01)
    // MaxSpeed += delta * 10;
   }
   else
    MaxSpeed = 10000;

   if (tRInverse[rl][i] != rInverse)
   {
    tRInverse[rl][i] = rInverse;
   }


#if 1
   if (rl == LINE_RL && FlyingCaution > 0.0f && fabs(delta) > 0.08)
   {
    // look for dangerous bumps in the track and slow the car down for them
    double gamma = fabs(delta) * 2.0f;
    gamma += fabs(rInverse) * 100;
    if (gamma * (0.10+FlyingCaution*0.90) > 0.02 && (MaxSpeed * gamma) * 10 > 20)
    {
     double OldMaxSpeed = MaxSpeed;
     if (MaxSpeed == 10000)
      OldMaxSpeed = 80.0f;
     if (MaxSpeed > 50)
      OldMaxSpeed = MIN(OldMaxSpeed, 50 + (50 * 1.0/FlyingCaution));
 
     if (OldMaxSpeed > 15.0f)
     {
      double caution = MAX(0.1f, 1.0f-(FlyingCaution * 0.9f));
      double arc1 = (fabs(seg->arc));
      double arc2 = (fabs(fabs(tSegment[(Segs+(tDivSeg[i]-2))%Segs]->arc)));
      double arc3 = (fabs(fabs(tSegment[(Segs+(tDivSeg[i]-2))%Segs]->arc)));
      double arc = MAX(arc1, MAX(arc2, arc3))*0.5f;
      arc = fabs(tRInverse[rl][i]*40.0f);
      double NewMaxSpeed = MIN(OldMaxSpeed, (25.0f + (1.0f / (1.0f-caution)) / ((gamma+arc))*(0.1f+caution*0.9f)));
      if (NewMaxSpeed < MaxSpeed)
      {
       Flying[i] = MaxSpeed - NewMaxSpeed;
       for (int j=1; j<11; j++)
       {
        int div = ((i-j)+Divs)%Divs;
        Flying[div] = MAX(Flying[div], MAX(1.0, (MaxSpeed - NewMaxSpeed) - (double)j/5));
       }
       MaxSpeed = NewMaxSpeed;
      }
     }
    }
   }
#endif

   if ((rInverse > 0.0 && tLane[rl][next] > tLane[rl][i]) || 
       (rInverse < 0.0 && tLane[rl][next] < tLane[rl][i]))
    MaxSpeed *= TurnAccel;

   if (rl == LINE_RL)
    tSpeed[i] = tMaxSpeed[i] = lastspeed = MaxSpeed;
  }
 
  //
  // Anticipate braking
  //

  if (rl != LINE_RL)
   return;

  for (i = Divs-1; --i >= 0;)
  {
   int next = (i+1) % Divs;

   if (tSpeed[i] > tSpeed[next])
   {
    tTrackSeg *seg = tSegment[tDivSeg[i]];
    double delta = tLDelta[tDivSeg[i]] * (1.0 - tLane[rl][i]) + tRDelta[tDivSeg[i]] * tLane[rl][i];
    double distratio = (((i - tSegDivStart[tDivSeg[i]]) * DivLength) + DivLength/2) * 1.2f / seg->length;
    double camber = (((seg->vertex[TR_SR].z-seg->vertex[TR_SL].z) * distratio) + ((seg->vertex[TR_ER].z-seg->vertex[TR_EL].z) * (1.0-distratio))) / seg->width;
    double camber1 = ((seg->vertex[TR_SR].z-seg->vertex[TR_SL].z)) / seg->width;
    double camber2=  ((seg->vertex[TR_ER].z-seg->vertex[TR_EL].z)) / seg->width;

    if (seg->type == TR_RGT)
    {
     camber = -camber;
     camber2 = -camber2;
     camber1 = -camber1;
    }
    if (camber2 < camber1)
     camber -= (camber1 - camber2) * 3.0;
    else if (camber2 > camber1)
     camber += (camber2 - camber1) * 0.4;

    double factor = 1.0;
    if (camber < -0.01)
     factor += MAX(-0.3, camber * 5);
    if (delta < -0.02 && fabs(tRInverse[rl][next]) > 0.0024)
     factor += MAX(-0.3, delta * 5);

    tSpeed[i] = MIN(tSpeed[i], tSpeed[next] + MAX(0.1, 
       ((0.1 - MIN(0.085, fabs(tRInverse[rl][next])*8)) 
        * MAX(MaxBrake/4.0, MaxBrake / (tSpeed[next]/20))) * factor));
   }
  }
#if 0
  int j;
  for (j = 100; --j >= 0;)
  for (i = Divs; --i >= 0;)
  {
   double TireAccel = TireAccel1 * tFriction[i];
   if (rl != LINE_RL) 
    TireAccel = (TireAccel1 - 2.0) * tFriction[i];
   int prev = (i - 1 + Divs) % Divs;
   int nnext = (i + 5 + Divs) % Divs;
   tTrackSeg *tseg = (tSegment[tDivSeg[nnext]]);
   tTrackSeg *seg = (tSegment[tDivSeg[i]]);
   int iscorner = SEGTYPE(tseg);
   
   if (rl == LINE_LEFT)
   {
    if (iscorner == TR_RGT)
     TireAccel *= MAX(0.7, (1.0f - ((120.0f-seg->radius)/120.0f) * 0.5f));
#if 0
    if (tseg->type == TR_RGT || seg->type == TR_RGT)
    {
     if (iscorner != TR_STR)
      TireAccel *= 0.85;
     else
      TireAccel *= 1.5;
    }
#endif
    else 
     TireAccel *= 1.0f + (seg->width - 8.0) * 0.018;
     //TireAccel *= 2.5 + seg->width/10;
   }
   else if (rl == LINE_RIGHT)
   {
    if (iscorner == TR_LFT)
     TireAccel *= MAX(0.7, (1.0f - ((120.0f-seg->radius)/120.0f) * 0.5f));
#if 0
    if (tseg->type == TR_LFT || seg->type == TR_LFT)
    {
     if (iscorner != TR_STR)
      TireAccel *= 0.85;
     else
      TireAccel *= 1.5;
    }
#endif
    else 
     TireAccel *= 1.0f + (seg->width - 8.0) * 0.018;
     //TireAccel *= 2.5 + seg->width/10;
   }
 
   double dx = tx[rl][i] - tx[rl][prev];
   double dy = ty[rl][i] - ty[rl][prev];
   double dist = Mag(dx, dy);
 
   //double Speed = (tSpeed[i] + tSpeed[i] + tSpeed[prev]) / 3;
   double Speed = (tSpeed[i] + tSpeed[prev]) / 2;
 
   //double LatA = tSpeed[i] * tSpeed[i] *
   double rInverse = (fabs(tRInverse[rl][prev]) + fabs(tRInverse[rl][i])) / 2;
   //rInverse *= fabs(rInverse)*295;

   double LatA = Speed * Speed * rInverse;
                 //(fabs(tRInverse[rl][prev]) + fabs(tRInverse[rl][i])) / 2;
 
   double TanA = TireAccel * TireAccel +
                 WingRInverse * Speed * Speed - LatA * LatA;


   // brake earlier or later depending on rl and situation
   double maxbrake = MaxBrake;
   if (TanA < 0.0)
    TanA = 0.0;
   if (TanA > maxbrake * tFriction[i])
   {
    TanA = (maxbrake * tFriction[i]);
   }
 
   double Time = dist / Speed;
   double MaxSpeed = tSpeed[i] + TanA * Time;

   tSpeed[prev] = MIN(MaxSpeed, tMaxSpeed[prev]);
  }                                                                              
#endif
  if (BrakeShift != 0.0f)
  {
   for (int j=0; j<(int) BrakeShift; j++)
   for (i = 1; ++i < Divs;)
   {
    int prev = (i - 1 + Divs) % Divs;
  
    if (tSpeed[i] < tSpeed[prev] - (1.0 - MIN(0.9, ((float) j+0.01)/BrakeShift)))
     tSpeed[prev] = ((tSpeed[i] * (1.5 + fabs(tRInverse[rl][i]) * 100.0)) + tSpeed[prev]) / (2.5 + fabs(tRInverse[rl][i]) * 100.0);
   }
  }
 }

 if (rl == LINE_RL)
 {
  for (i=1; i<Divs; i++)
  {
   int prev = i-1;
   if (tMaxSpeed[prev] > tMaxSpeed[i] + 0.3)
    Braking[prev] = 1;
  }
 }
} 

double Driver::InverseFriction(double a)
{
 a = MIN(a, 1.3);
 return 2.0 * a / (1.3 - a);
}

double Driver::GetControl(double At, double An, double v, double mass)
{
 int fAdjustP = 0;
 double At1 = 0;
 double P1 = 0;
 double At0 = 0;
 double P0 = 0;
 double alpha = 0, vc = 0;
 int Loops = 0;

 while(1)
 {
  if (++Loops >= 40)
  {
   OUTPUT("Control problem");
   return At0;
  }

  double A = Mag(At, An);
  if (A > 0)
  {
   double g = 32.2;
   double PM = 1e5;
   double L;
   L = InverseFriction(A / g);

   double SinTheta = An / A;
   double CosTheta = At / A;
   alpha = atan((L * SinTheta) / (L * CosTheta + v));
   vc = (L * CosTheta + v) / cos(alpha);
   double x = cos(alpha) * CosTheta + sin(alpha) * SinTheta;
   double P = A * mass * vc * x;
   
   if (At < 0 || (!fAdjustP && P < PM) || (fAdjustP && P < PM && P > 0.999 * PM))
    break;

   if (!fAdjustP)
   {
    fAdjustP = 1;
    At1 = At;
    P1 = P;
   }
   else
   {
    if (P >= PM)
    {
     At1 = At;
     P1 = P;
    }
    else
    {
     At0 = At;
     P0 = P;
    }
   }

   At = At0 + (0.9995 * PM - P0) * (At1 - At0) / (P1 - P0);
  }
  else
  {
   alpha = 0;
   vc = 0;
  }
 }

 return At;
}

void Driver::Optimize()
{
 ComputeFullPath();
#if 0
 const double LaneMinInt = SideDistInt / track->width;
 const double LaneMinExt = SideDistExt / track->width;
 const double LaneMaxInt = 1 - LaneMinInt;
 const double LaneMaxExt = 1 - LaneMinExt;

 int IndexStep = 128;
 double LaneStep = 0.02;

 for (int Pass = 0; Pass <= 1; Pass++, LaneStep /= 2, IndexStep /= 2)
 {
fprintf(stderr,"Pass = %d\n",Pass);fflush(stderr);
  int i;
  int Tries[MaxDivs];
  for (i = Divs; --i >= 0;)
   Tries[i] = 0;

  for (i = 0; i < Divs - IndexStep; i += IndexStep)
  {
fprintf(stderr,"%d: a\n",i);fflush(stderr);
   ComputeFullPath();
fprintf(stderr,"%d: b\n",i);fflush(stderr);
   double s0 = EstimateSpeed();
fprintf(stderr,"%d: c\n",i);fflush(stderr);
   OUTPUT("Optimizing lane number " << i << " / " << Divs << " (pass " << Pass << ")");
   OUTPUT("Reference estimated lap speed = " << s0 << " mph");
   int OldfConst = tfConst[i];
   tfConst[i] = 1;
   double RefLane = tLane[rl][i];
   double BestLane = tLane[rl][i];

   int Dir = -1;
   int j = 0;
   while(1)
   {
    j++;
    double l = RefLane + j * Dir * LaneStep;
    if ((tRInverse[rl][i] > 0 && (l < LaneMinInt || l > LaneMaxExt)) ||
        (tRInverse[rl][i] < 0 && (l > LaneMaxInt || l < LaneMinInt)))
    break;
    tLane[rl][i] = l;
    UpdateTxTy(i);
    ComputeFullPath();
    double s = EstimateSpeed();
    
    if (s > s0)
    { 
     BestLane = tLane[rl][i];
     s0 = s;
    }
    else
    {
     if (j == 1 && Dir < 0)
      Dir = 1;
     else 
      break;
     j = 0;
    }
   }
   tLane[rl][i] = BestLane;
   UpdateTxTy(i);
   Tries[i]++;

   if (BestLane == RefLane)
    tfConst[i] = OldfConst;
   else if (Tries[i] <= 3 && i > 0)
    i -= 2 * IndexStep;
  }
 }
#endif
}

double Driver::getDistToSegStart(tCarElt *car, int useovertake)
{
 if (!useovertake)
 {
  //return car->_distFromStartLine - car->_trkPos.seg->lgfromstart;
  return car->_distFromStartLine - car->_trkPos.seg->lgfromstart;
 }

 //if (TR_STR == SEGTYPE(car->_trkPos.seg))
 if (car->_trkPos.seg->type == TR_STR)
  return car->_trkPos.seg->lgfromstart + car->_trkPos.toStart;
  //return tSegStart[car->_trkPos.seg->id] + car->_trkPos.toStart;
 else
  return car->_trkPos.seg->lgfromstart + car->_trkPos.toStart*car->_trkPos.seg->radius;
  //return tSegStart[car->_trkPos.seg->id] + car->_trkPos.toStart*car->_trkPos.seg->radius;
}

tTrackSeg * Driver::getNextCornerSeg()
{
 tTrackSeg *seg = car->_trkPos.seg;

 do
 {
  if (SEGTYPE(seg) != TR_STR && seg->type == prefer_side)
   break;
  seg = seg->next;
 } while (seg->next != car->_trkPos.seg);

 return seg;
}


double Driver::getDistToCorner()
{
 tTrackSeg *seg = car->_trkPos.seg;
 double dist = 0.0;
 cseg = NULL;

 do
 {
  if (seg->type == TR_STR)
  {
   if (seg == car->_trkPos.seg)
    dist = (seg->length+seg->lgfromstart) - car->_distFromStartLine;
   else
    dist += seg->length;
  }
  else
  {
   if (seg == car->_trkPos.seg)
   {
    dist = (seg->length+seg->lgfromstart) - car->_distFromStartLine;
    if (dist < seg->length * 0.5f && SEGTYPE2(seg, 1) != TR_STR && seg->radius < 80.0)
    {
     dist = 0.0;
     cseg = seg;
     break;
    }
   }
   else
   {
    dist += seg->length * 0.5;// * seg->radius;
   }
  }
  seg = seg->next;
 }
 while (TR_STR == SEGTYPE(seg) && seg->next != car->_trkPos.seg);

 if (SEGTYPE(seg) != TR_STR)
  cseg = seg;

 dist += seg->length * 0.5;

 if (FlyingCaution)
 {
  // lets see if there's a bump somewhere between us and the corner
  double bdist = DivLength;
  for (int i = nextdiv; (bdist == DivLength || i != nextdiv) ; i++)
  {
   if (i >= Divs)
    i = 0;

   if (Flying[i])
    break;

   bdist += DivLength;
   if (bdist >= dist)
    break;
  }

  dist = MIN(dist, bdist);
 }

 return dist;
}

void Driver::FindPreferredSide()
{
 tTrackSeg *seg = car->_trkPos.seg;;
 prefer_side = SEGTYPE(seg);
 double next_radius = tSegRadius[tDivSeg[nextdiv]];
 //float lane2left = tLane[rl][nextdiv] * track->width;
 double curlane = car->_trkPos.toLeft / track->width;
 int next = (Divs+nextdiv+5) % Divs;

 if (prefer_side == TR_STR)
 {
  for (int i=1; i<(int) car->_speed_x * 3; i++)
  {
   int n = (nextdiv+i+Divs) % Divs;
   seg = tSegment[tDivSeg[n]];
   int type = SEGTYPE(seg);

   if (type != TR_STR)
    break;
  }

  prefer_side = SEGTYPE(seg);
  next_radius = tSegRadius[seg->id];
 }
 else if (prefer_side == TR_LFT && MAX(tLane[rl][next], curlane) > 0.4 && tLane[rl][next] > tLane[rl][thisdiv])
 {
  for (int i=1; i<(int) (car->_speed_x * 1.5); i++)
  {
   int n = (nextdiv+Divs+i) % Divs;
   seg = tSegment[tDivSeg[n]];
   int type = SEGTYPE(seg);

   if (type != prefer_side && type != TR_STR)
   {
    prefer_side = type;
    break;
   }
  }
  if (prefer_side == TR_LFT)
   prefer_side = TR_STR;
 }
 else if (prefer_side == TR_RGT && MIN(tLane[rl][next], curlane) < 0.6 && tLane[rl][next] < tLane[rl][thisdiv])
 {
  for (int i=1; i<(int) (car->_speed_x * 1.5); i++)
  {
   int n = (nextdiv+Divs+i) % Divs;
   seg = tSegment[tDivSeg[n]];
   int type = SEGTYPE(seg);

   if (type != prefer_side && type != TR_STR)
   {
    prefer_side = type;
    break;
   }
  }
  if (prefer_side == TR_RGT)
   prefer_side = TR_STR;
 }
}

void Driver::initRadius()
{
 double lastturnarc = 0.0;
 int lastsegtype = TR_STR;

 tTrackSeg *currentseg, *startseg = track->seg;
 currentseg = startseg;

 do {
  if (currentseg->type == TR_STR) 
  {
   lastsegtype = TR_STR;
   tRadius[currentseg->id] = FLT_MAX;
  } 
  else 
  {
   if (currentseg->type != lastsegtype) 
   {
    double arc = 0.0;
    tTrackSeg *s = currentseg;
    lastsegtype = currentseg->type;

    while (s->type == lastsegtype && arc < PI/2.0) {
     arc += s->arc;
     s = s->next;
    }
    lastturnarc = arc/(PI/2.0);
   }
   tRadius[currentseg->id] = ((currentseg->radius + currentseg->width/2.0)/lastturnarc) * 1.3;
  }
  currentseg = currentseg->next;
 } while (currentseg != startseg);
}

void Driver::initWheelPos()
{
 for (int i=0; i<4; i++)
 {
  //wheelz[i] = car->priv.wheel[i].relPos.z;

  char const *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
  //char *SuspSect[4] = {SECT_FRNTRGTSUSP, SECT_FRNTLFTSUSP, SECT_REARRGTSUSP, SECT_REARLFTSUSP};
  float rh = 0.0; //, bc = 0.0, rimdiam = 0.0, tireratio = 0.0, tirewidth = 0.0;
  rh = GfParmGetNum(car->_carHandle,WheelSect[i],PRM_RIDEHEIGHT,(char *)NULL, 0.10f);
  //bc = GfParmGetNum(car->_carHandle,SuspSect[i],PRM_BELLCRANK,(char *)NULL, 1.50f);
  //rimdiam = GfParmGetNum(car->_carHandle, WheelSect[i], PRM_RIMDIAM, (char*)NULL, 0.33f);
  //tirewidth = GfParmGetNum(car->_carHandle, WheelSect[i], PRM_TIREWIDTH, (char*)NULL, 0.33f);
  //tireratio = GfParmGetNum(car->_carHandle, WheelSect[i], PRM_TIRERATIO, (char*)NULL, 0.33f);

  //float radius = rimdiam / 2.0 + tirewidth * tireratio;

  // 1.0 is supposed to be bellcrank, but using bellcrank seems to produce wrong values.
  // Going with this for now and hoping that simuv3 doesn't change it too much.
  wheelz[i] = (-rh / 1.0 + car->info.wheel[i].wheelRadius) - 0.01;
 }
}

int Driver::CheckFlying()
{
 int i = 0;
 if (car->_speed_x < 20)
  return 0;

 if (car->priv.wheel[0].relPos.z < wheelz[0] &&
     car->priv.wheel[1].relPos.z < wheelz[1])
 {
  i += FLYING_FRONT;
 }
 if (car->priv.wheel[2].relPos.z < wheelz[2]-0.05 &&
     car->priv.wheel[3].relPos.z < wheelz[3]-0.05)
 {
  i += FLYING_BACK;
 }
 if (last_flying &&
     ((car->priv.wheel[0].relPos.z < wheelz[0] &&
       car->priv.wheel[2].relPos.z < wheelz[1]) ||
      (car->priv.wheel[1].relPos.z < wheelz[2] &&
       car->priv.wheel[3].relPos.z < wheelz[3])))
 {
  i = MAX(i, last_flying);
 }

 return i;
}

void Driver::loadSVG()
{
 const int BUFSIZE = 256;
 char buffer[BUFSIZE];

 char *trackname = strrchr(track->filename, '/') + 1;
 sprintf(buffer, "drivers/%s/svg/%s.svg", DRIVERNAME, trackname);
 FILE *fp = NULL;

 if (!(fp = fopen(buffer, "r")))
 {
  fprintf(stderr,"Failed to open SVG file %s\n",buffer); fflush(stderr);
  LoadSVG = 0;
  return;
 }

 fseek(fp, 0, SEEK_END);
 int filelen = ftell(fp);
 fseek(fp, 0, SEEK_SET);
 char *line = new char[filelen+1];
 char *data = NULL;

 // find the line that has the path on it
 while (fgets(line, filelen, fp))
 {
  char *p = strchr(line, '=');
  if (p)
  {
   char *q = strstr(line, "z \"");
   if (q)
   {
    data = new char[strlen(line)];
    strncpy(data, p+2, q-p);
    break;
   }
  }
 }

 fclose(fp);
 delete line;

 if (data)
 {
  // parse the line into the array of points
  v2d *point = new v2d[Divs];
  int i, count = 0;
  int str_size = strlen(data);
  char *tmp = new char[str_size+1];
  memset(tmp, 0, str_size+1);
  char *ptmp = tmp;

  for (i=0; i<str_size && count<Divs; i++)
  {
   if (data[i] == ',')
   {
    point[count].x = (float)atof(tmp);
    memset(tmp, 0, str_size+1);
    ptmp = tmp;
   }
   else if (data[i] == 'L' || data[i] == 'z')
   {
    point[count].y = (float)atof(tmp);
    memset(tmp, 0, str_size+1);
    ptmp = tmp;
    count++;
   }
   else if ((data[i] >= '0' && data[i] <= '9') || data[i] == '.')
   {
    *ptmp = data[i];
    ptmp++;
   }
  }

  delete tmp;
  delete data;

  if (count < Divs)  // didn't find right number of points
  {
   delete point;
   fprintf(stderr,"LoadSVG: Count %d < Divs %d\n",count,Divs);fflush(stderr);
   LoadSVG = 0;
   return;
  }

  for (i=0; i<Divs; i++)
  {
   // copy into the tx/ty arrays
   tx[rl][i] = point[i].x;
   ty[rl][i] = point[i].y;
  }

  delete point;
 }

 //LoadSVG = 0;
 return;
}

void Driver::saveSVG()
{
 const int BUFSIZE = 256;
 char buffer[BUFSIZE];

 char *trackname = strrchr(track->filename, '/') + 1;
 sprintf(buffer, "drivers/%s/svg/%s.svg", DRIVERNAME, trackname);
 FILE *fp = NULL;

 if (!(fp = fopen(buffer, "w")))
  return;

 // write svg header
 fprintf(fp, "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n");
 fprintf(fp, "<svg\n");
 fprintf(fp, "  width=\"30cm\"\n  height=\"30cm\"\n  viewBox=\"0 0 3000 3000\"\n");
 fprintf(fp, "  xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\">\n");

 // write track borders
 tTrackSeg *first = track->seg;
 tTrackSeg *prev = track->seg->prev;
 tTrackSeg *seg = first;

#if 1
 int i;
 for (i=0; i<track->nseg; i++)
 {
  if (1 || seg->type == TR_STR)
  {
   fprintf(fp, "  <path d=\"M %.3f,%.3f L %.3f,%.3f\"\n", seg->vertex[TR_SL].x, seg->vertex[TR_SL].y, seg->vertex[TR_EL].x, seg->vertex[TR_EL].y);
   fprintf(fp, "    style=\"fill:none; stroke:grey; stroke-width:1\"/>\n");
   fprintf(fp, "  <path d=\"M %.3f,%.3f L %.3f,%.3f\"\n", seg->vertex[TR_SR].x, seg->vertex[TR_SR].y, seg->vertex[TR_ER].x, seg->vertex[TR_ER].y);
   fprintf(fp, "    style=\"fill:none; stroke:grey; stroke-width:1\"/>\n");
  }
  else
  {
   fprintf(fp, "  <path d=\"M %.3f,%.3f A %.3f,%.3f 0 0,%d %.3f,%.3f\"\n", seg->vertex[TR_SL].x, seg->vertex[TR_SL].y, seg->radiusl, seg->radiusl, seg->vertex[TR_EL].x, seg->vertex[TR_EL].y);
   fprintf(fp, "    style=\"fill:none; stroke:grey; stroke-width:1\"/>\n");
   fprintf(fp, "  <path d=\"M %.3f,%.3f A %.3f,%.3f 0 0,%d %.3f,%.3f\"\n", seg->vertex[TR_SR].x, seg->vertex[TR_SR].y, seg->radiusl, seg->radiusl, seg->vertex[TR_ER].x, seg->vertex[TR_ER].y);
   fprintf(fp, "    style=\"fill:none; stroke:grey; stroke-width:1\"/>\n");
  }

  prev = seg;
  seg = seg->next;
 }
#endif
 
 // now write the path in red
 fprintf(fp, "  <path\n    d=\"M %.3f,%.3f", tx[rl][0], ty[rl][0]);

 for (i=1; i<Divs; i++)
 {
  fprintf(fp, " L %.3f,%.3f", tx[LINE_RL][i], ty[LINE_RL][i]);
 }
 fprintf(fp, " z \"\n");
 fprintf(fp, "    style=\"fill:none; stroke:red; stroke-width:1\"/>\n");
 fprintf(fp, "</svg>\n");
 fclose(fp);
}

void Driver::saveLearning()
{
 const int BUFSIZE = 256;
 char buffer[BUFSIZE];
 char indexstr[32];

 char *trackname = strrchr(track->filename, '/') + 1;

 RtGetCarindexString(INDEX, "hymie", INDEX < 1 || INDEX > 10, indexstr, 32);

 switch (racetype)
 {
  case RM_TYPE_QUALIF:
   snprintf(buffer, BUFSIZE, "drivers/%s/%s/qualifying/%s.learn", DRIVERNAME, indexstr, trackname);
   break;
  case RM_TYPE_PRACTICE:
   snprintf(buffer, BUFSIZE, "drivers/%s/%s/practice/%s.learn", DRIVERNAME, indexstr, trackname);
   break;
  case RM_TYPE_RACE:
   sprintf(buffer, "drivers/%s/%s/%s.learn", DRIVERNAME, indexstr, trackname);
   break;
 }

 FILE *fp;

 if (NULL != (fp = fopen(buffer, "r")))
 {
  fclose(fp);
  if (!OverrideLearning)
   return;
 }
 if (!(fp = fopen(buffer, "w")))
 {
  sprintf(buffer, "drivers/%s/%s/%s.learn", DRIVERNAME, indexstr, trackname);
  if (NULL != (fp = fopen(buffer, "r")))
  {
   fclose(fp);
   if (!OverrideLearning)
    return;
  }
  if (!(fp = fopen(buffer, "w")))
   return;
 }

 int i;
 for (i=0; i<Divs; i++)
  fprintf(fp, "%.8lf\n", tMaxSpeed[i]+tSpeedAdjust[i]);

 //for (i=0; i<Divs; i++)
 // fprintf(fp, "%.8lf\n", tLaneAdjust[i]);

 fclose(fp);

 return;
}

void Driver::loadLearning()
{
 const int BUFSIZE = 256;
 char buffer[BUFSIZE];
 char indexstr[32];
 buffer[0] = 0;

 char *trackname = strrchr(track->filename, '/') + 1;

 RtGetCarindexString(INDEX, "hymie", INDEX < 1 || INDEX > 10, indexstr, 32);

 switch (racetype)
 {
  case RM_TYPE_QUALIF:
   snprintf(buffer, BUFSIZE, "drivers/%s/%s/qualifying/%s.learn", DRIVERNAME, indexstr, trackname);
   break;
  case RM_TYPE_PRACTICE:
   snprintf(buffer, BUFSIZE, "drivers/%s/%s/practice/%s.learn", DRIVERNAME, indexstr, trackname);
   break;
  case RM_TYPE_RACE:
   sprintf(buffer, "drivers/%s/%s/%s.learn", DRIVERNAME, indexstr, trackname);
   break;
 }

 FILE *fp;

 if (!(fp = fopen(buffer, "r")))
 {
  sprintf(buffer, "drivers/%s/%s/%s.learn", DRIVERNAME, indexstr, trackname);
  if (!(fp = fopen(buffer, "r")))
  {
   return;
  }
 }

 int i;
 memset(tSpeedAdjust, 0, sizeof(tSpeedAdjust));
 memset(tLaneAdjust, 0, sizeof(tLaneAdjust));

 for (i=0; i<Divs; i++)
 {
  char buf[16];
  if (!fgets(buf, 16, fp))
  {
   fclose(fp);
   return;
  }
  tSpeedAdjust[i] = atof(buf);
 }

 for (i=0; i<Divs; i++)
 {
  char buf[16];
  if (!fgets(buf, 16, fp))
  {
   fclose(fp);
   if (!OverrideLearning)
    Learning = MAX(RaceLearning, 0.0);
   break;
  }
  tLaneAdjust[i] = atof(buf);
  tLaneLMargin[i] -= (tLaneAdjust[i] > 0.0 ? tLaneAdjust[i] : 0.0);
  tLaneRMargin[i] += (tLaneAdjust[i] < 0.0 ? tLaneAdjust[i] : 0.0);
 }

 fclose(fp);
 if (!OverrideLearning)
  Learning = MAX(RaceLearning, 0.0);
 LearningLoaded = 1;
 return;
}

double Driver::K1999Steer(tSituation *s)
{
 //memset(&(car->ctrl), 0, sizeof(tCarCtrl));

 // 
 // Find index in data arrays
 //
 int SegId = car->_trkPos.seg->id;
 tTrackSeg *seg = car->_trkPos.seg;
 double dist = car->_trkPos.toStart;
 if (dist < 0)
  dist = 0;
 if (car->_trkPos.seg->type != TR_STR)
  dist *= car->_trkPos.seg->radius;
 int Index = tSegIndex[SegId] + int(dist / tElemLength[SegId]);
 //double d = tSegDist[SegId] + dist;

 Index = (Index + Divs - 5) % Divs;
 int Next = Index;
 double Time = deltaTime*3;//0.01;
 double X = car->_pos_X + car->_speed_X * Time / 2;
 double Y = car->_pos_Y + car->_speed_Y * Time / 2;

 while(1)
 {
  Next = (Index + 1) % Divs;
  if ((tx[rl][Next] - tx[rl][Index]) * (X - tx[rl][Next]) +
      (ty[rl][Next] - ty[rl][Index]) * (Y - ty[rl][Next]) < -0.1)
   break;
  Index = Next;
 }
 //if ((avoiding && !(avoiding & AVOID_ALIGNED)) || getCorrecting())
 // Next = (Next + 2) % Divs;
 int NextNext = (Next + 1) % Divs;

 thisdiv = Index;
 nextdiv = Next;

 if (lastLTleft == -1000.0)
  lastLTleft = tLane[rl][nextdiv];

 double c0 = (tx[rl][Next] - tx[rl][Index]) * (tx[rl][Next] - X) +
             (ty[rl][Next] - ty[rl][Index]) * (ty[rl][Next] - Y);
 double c1 = (tx[rl][Next] - tx[rl][Index]) * (X - tx[rl][Index]) +
             (ty[rl][Next] - ty[rl][Index]) * (Y - ty[rl][Index]);
 {
  double sum = c0 + c1;
  c0 /= sum;
  c1 /= sum;
 }

 k1999pt.x = (float)tx[rl][Next];
 k1999pt.y = (float)ty[rl][Next];
 double lane2left = tLane[rl][nextdiv] * seg->width;

 //
 // Find target curvature (for the inside wheel)
 //
 if (1 || (fabs(angle) < 0.4 && fabs(car->_yaw_rate) < ((avoiding || getCorrecting()) ? 1.0 : 2.4)))
 {
  double modifier = 10.0f;//avoiding ? 2.0 : getCorrecting() ? 6.0 : 10.0f);
  if (/*SteerGain2 &&*/ fabs(angle) < 0.4 && fabs(car->_yaw_rate) < 1.0)
   c0 /= (1.0 + fabs(tRInverse[rl][Next]) * modifier);//SteerGain2;
  if ((prefer_side == TR_LFT && nextleft > lane2left) || (prefer_side == TR_RGT && nextleft < seg->width-lane2left))
   c0 /= 1.0 + (modifier*0.1);
 }

 double TargetCurvature = ((1 - c0) * tRInverse[rl][Next] + c0 * tRInverse[rl][Index]);
 if (fabs(TargetCurvature) > 0.01)
 {
  double r = 1 / TargetCurvature;
  if (r > 0)
   r -= wheeltrack / 2;
  else
   r += wheeltrack / 2;
  TargetCurvature = 1 / r;
 }

 //if (car->_accel_x > 0.0)
 // TargetCurvature *= 1.0 + (car->_accel_x/10);
 if (car->_accel_x > -5.0 && SteerGainDiv < 1000)
  TargetCurvature *= 1.0 + ((car->_accel_x+5.0))/SteerGainDiv;
 KTCdiff = TargetCurvature - KTC;
 KTC = TargetCurvature;

 {
  //
  // Find target speed
  //
  double tanksize = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_TANK, (char*) NULL, 100.0f);
  //int avoid = ((avoiding && !(avoiding & AVOID_ALIGNED)) || getCorrecting() || brake_collision || align_hold > currentsimtime || !alone);
  double speednext, speedthis;
  if (rl == LINE_RL)
  {
   speednext = tMaxSpeed[Next]+tSpeedAdjust[Next]; speedthis = tMaxSpeed[Index]+tSpeedAdjust[Index];
  }
  else
  {
   speednext = tSpeed[Next]; speedthis = tSpeed[Index];
   if (rl == LINE_LEFT)
   {
    speednext -= tLSlowSpeed[Next];
    speedthis -= tLSlowSpeed[Index];
   }
   else
   {
    speednext -= tRSlowSpeed[Next];
    speedthis -= tRSlowSpeed[Index];
   }
  }
#if 0
  if (speednext < speedthis && car->_fuel < tanksize/2)
  {
   speednext += (speedthis-speednext) * (1.0 - (car->_fuel/(tanksize*0.9)));
  }
#endif

  double targspeed = ((1 - c0) * speednext + c0 * speedthis);
  //if (tMaxSpeed[Next] < tMaxSpeed[Index])
  if (targspeed < car->_speed_x)// && racetype != RM_TYPE_QUALIF)
   targspeed = MIN(MAX(car->_speed_x-0.02, targspeed), targspeed + (tanksize-car->_fuel)/(tanksize*20));

  if (FuelSpeedup > 0.0f && (car->_tank - car->_fuel) > 5.0)
  {
   targspeed += (targspeed/100 * ((CARMASS + (car->_tank-5.0)) / mass)) * FuelSpeedup;
  }

#ifdef CONTROL_SKILL
  if (RM_TYPE_PRACTICE != racetype)
  {
   double level = skill_level + 1.5;
   double speedadjust = MIN(level*2, targspeed / (100 - level*2) * 27);
   targspeed -= speedadjust + cur_speed_adjust;

   if (currentsimtime - speed_adjust_timer > speed_adjust_limit)
   {
    double rand1 = (double) getRandom() / 65536.0;
    double rand2 = (double) getRandom() / 65536.0;
    double rand3 = (double) getRandom() / 65536.0;

    speed_adjust_limit = 5.0 + rand1 * 30;
    speed_adjust_timer = currentsimtime;
    trg_speed_adjust = rand2 * (speedadjust/12 + 0.1);

    if (rand3 < 0.15)
     trg_speed_adjust = -(trg_speed_adjust)/2;
   }

   if (cur_speed_adjust < trg_speed_adjust)
    cur_speed_adjust += MIN(deltaTime/4, trg_speed_adjust - cur_speed_adjust);
   else
    cur_speed_adjust -= MIN(deltaTime/4, cur_speed_adjust - trg_speed_adjust);
  }
#endif

  if (rl == LINE_RL)
   KTargetSpeed = TargetSpeed = targspeed;
  else if (rl == LINE_LEFT)
   LTargetSpeed = targspeed;
  else if (rl == LINE_RIGHT)
   RTargetSpeed = targspeed;
 }

 //
 // Steering control
 //
 Error = 0;
 double VnError = 0;
 double Skid = 0;
 double steer = 0;
 CosAngleError = 1;
 SinAngleError = 0;
 {
  //
  // Ideal value
  //
  if (1)
  {
   steer = atan(wheelbase * TargetCurvature) / car->_steerLock;
   double steergain = (((avoiding && !(avoiding & AVOID_ALIGNED)) || (getCorrecting() && (getCorrectTimer() > 3.0))) ? SteerGain - 0.3 : SteerGain);
   if (car->_speed_x > 60)
    steergain = MAX(MIN(1.1, SteerGain), steergain - (car->_speed_x-60.0)/40);
   if (car->_speed_x < TargetSpeed-10.0 && segtype != TR_STR)
    steergain = MIN(steergain, 1.1);
#if 1
   //if (racetype != RM_TYPE_QUALIF)
   {
    if (segtype == TR_RGT)
    {
     if (car->_trkPos.toRight < 4.0)
      steergain = MAX(1.0, steergain - (car->_trkPos.toRight/17));
    }
    else if (segtype == TR_LFT)
    {
     if (car->_trkPos.toLeft < 4.0)
      steergain = MAX(1.0, steergain - (car->_trkPos.toLeft/17));
    }
   }
#endif
#if 0
   if (segtype == TR_LFT && nextleft - car->_dimension_y * 0.2 < 0.0)
    steergain = MAX(1.0, steergain - fabs(nextleft - car->_dimension_y * 0.5));
   else if (segtype == TR_RGT && nextleft + car->_dimension_y * 0.2 > seg->width)
    steergain = MAX(1.0, steergain - ((nextleft + car->_dimension_y * 0.5) - seg->width));
   if (seg->type != TR_LFT && nextleft - car->_dimension_y * 0.5 < 0.0)
    steergain += fabs(nextleft - car->_dimension_y * 0.5) * 0.5;
   else if (seg->type != TR_RGT && nextleft + car->_dimension_y * 0.5 > seg->width)
    steergain += ((nextleft + car->_dimension_y * 0.5) - seg->width) * 0.5;
#endif

   steer = atan(wheelbase * steergain * TargetCurvature) / car->_steerLock;
  }

  //
  // Servo system to stay on the pre-computed path
  //
  {
   double dx = tx[rl][Next] - tx[rl][Index];
   double dy = ty[rl][Next] - ty[rl][Index];
   Error = (dx * (Y - ty[rl][Index]) - dy * (X - tx[rl][Index])) / Mag(dx, dy);
  }  

  int Prev = (Index + Divs - 1) % Divs;
  double txPrev = tx[rl][Prev];
  double tyPrev = ty[rl][Prev];
  double txIndex = tx[rl][Index];
  double tyIndex = ty[rl][Index];
  double txNext = tx[rl][Next];
  double tyNext = ty[rl][Next];
  double txNextNext = tx[rl][NextNext];
  double tyNextNext = ty[rl][NextNext];

  double Prevdx = txNext - txPrev;
  double Prevdy = tyNext - tyPrev;
  double Nextdx = txNextNext - txIndex;
  double Nextdy = tyNextNext - tyIndex;
  double dx = c0 * Prevdx + (1 - c0) * Nextdx;
  double dy = c0 * Prevdy + (1 - c0) * Nextdy;
  double n = Mag(dx, dy);
  dx /= n;
  dy /= n;
  double speed = Mag(car->_speed_Y, car->_speed_X);
  double sError = (dx * car->_speed_Y - dy * car->_speed_X) / (speed + 0.01);
  double cError = (dx * car->_speed_X + dy * car->_speed_Y) / (speed + 0.01);
  VnError = asin(sError);
  if (cError < 0)
   VnError = PI - VnError;

  if (rl == LINE_RL && RaceLearning && !Flying[thisdiv] && !getCorrecting() && !avoiding)
  {
   int pdiv1 = ((thisdiv - (int) (getSpeed()*0.4)) + Divs) % Divs;
   int pdiv2 = ((thisdiv - (int) (getSpeed()*0.4)) + Divs) % Divs;
   if (steer > 0.0)
   {
    if (!Flying[pdiv2] && Error > -0.5 && nextleft < MIN(lane2left + 0.6, seg->width-getWidth()*0.8))
    {
     tSpeedAdjust[pdiv2] += MIN(2.0, 0.2 + fabs(-0.5 - Error)) * RaceLearning;
    }
    else if (steer>0.1 && Error<-0.7 && (car->_trkPos.toLeft>lane2left+2.5 || (car->_trkPos.toLeft>lane2left+0.6 && car->_trkPos.toRight<getWidth()*0.7)) && (segtype != TR_RGT || car->_trkPos.toRight > 3.0))
    {
     tMaxSpeed[pdiv1] -= MIN(3.5, fabs(-0.7 - Error)*(0.3+MIN(1.5, steer*2))) * RaceLearning;
     if (LearningLoaded)
      tSpeedAdjust[pdiv1] = MAX(tSpeedAdjust[pdiv1], 0);
    }
   }
   else if (k1999steer < 0.0)
   {
    if (!Flying[pdiv2] && Error < 0.5 && nextleft > MAX(lane2left - 0.6, getWidth()*0.8))
    {
     tSpeedAdjust[pdiv2] += MIN(2.0, 0.2 + fabs(0.5 - Error)) * RaceLearning;
    }
    else if (k1999steer<-0.1 && Error>0.7 && (car->_trkPos.toLeft<lane2left-2.5 || (car->_trkPos.toLeft<lane2left-0.6 && car->_trkPos.toLeft<getWidth()*0.7)) && (segtype != TR_LFT || car->_trkPos.toLeft > 3.0))
    {
     tSpeedAdjust[pdiv1] -= MIN(3.5, fabs(0.7 - Error)*(0.3+MIN(1.5, fabs(steer)*2))) * RaceLearning;
     if (LearningLoaded)
      tSpeedAdjust[pdiv1] = MAX(tSpeedAdjust[pdiv1], 0);
    }
   }
  }

  double sg2 = SteerGain2;
  steer -= (atan(Error * (300 / (speed + 300)) / (15 / sg2)) + VnError) / car->_steerLock;
  if (getCorrecting() || (avoiding && !(avoiding & AVOID_ALIGNED)))
   Error *= MAX(0.7, (1.0 - fabs(lastksteer - laststeer) * 0.5));

  //
  // Steer into the skid
  //
  double vx = car->_speed_X;
  double vy = car->_speed_Y;
  double yaw = lastyaw;
  double dirx = cos(yaw);
  double diry = sin(yaw);
  CosAngleError = dx * dirx + dy * diry;
  SinAngleError = dx * diry - dy * dirx;
  Skid = (dirx * vy - vx * diry) / (speed + 0.1);
  if (Skid > 0.9)
   Skid = 0.9;
  if (Skid < -0.9)
   Skid = -0.9;
  steer += (asin(Skid) / car->_steerLock) * 0.9;

  double yr = speed * TargetCurvature;
  double diff = car->_yaw_rate - yr;
  //double diff = (car->_yaw_rate-(car->_yaw_rate-lastyr)/4) - yr;
  double steerskid = SteerSkid;
  if (car->_accel_x < -10.0)
   steerskid += (car->_accel_x+10.0)/300;
  if ((flying_timer > currentsimtime - 2.0) && 
      (segtype == TR_STR ||
       (segtype == TR_RGT && nextleft > 4.0f) ||
       (segtype == TR_LFT && nextleft < seg->width - 4.0f) ||
       (flying_timer < currentsimtime - 2.0 && (nextleft <= MIN(0.0, lane2left) || nextleft >= MAX(seg->width, lane2left)))))
  {
   steerskid = MAX(SteerSkid, MIN(0.24, SteerSkid + MIN(0.2, fabs(laststeer-lastksteer) * 0.7)));
   if (flying_timer > currentsimtime - 3.5)
   {
    steerskid= MAX((0.20 + (3.5 - (currentsimtime - flying_timer)) * 0.08), SteerSkid);
   }
   else if ((nextleft <= 0.0 || nextleft >= seg->width) && steerskid > SteerSkid)
    steerskid = MAX(steerskid * (0.3 - fabs(angle)*0.5), SteerSkid);
  }
#if 0
  else if ((getCorrecting() || getAllowCorrecting()) && getCorrectTimer() > 0.0)
   steerskid += fabs(correct_diff) / 10;
  else if (align_hold > currentsimtime)
   steerskid += (align_hold-currentsimtime)/50.0;
#endif

  steer -= (steerskid * (1 + fDirt) * (100 / (speed + 90)) * diff) / car->_steerLock;
  if ((last_flying & FLYING_FRONT) && fabs(angle) < 0.3 && nextleft > 0.5f && nextleft < seg->width - 0.5f)
  {
   steer = MAX(-0.1, MIN(0.1, steer));
  }

#if 1
  if (fabs(car->_yaw_rate) < 1.8 && getSpeed() < 10.0 && fabs(angle) > 1.6)
  {
   if ((angle > 1.6f && car->_trkPos.toRight > angle*2 && steer < -0.1) || 
       (angle < -1.4f && car->_trkPos.toLeft > fabs(angle)*2 && steer > 0.1))
    steer = -steer;
   if ((angle >= 1.6 && car->_trkPos.toRight < angle*2 && steer > 0.1) || 
       (angle <= -1.6 && car->_trkPos.toLeft < fabs(angle)*2 && steer < -0.1))
    steer = -steer;
  }
  else 
#endif
  if (fabs(car->_yaw_rate) >= 1.8 && getSpeed() > 10.0)
  {
   AccelCmd = 0.0;
   BrakeCmd = 1.0;
  }
   
   // this applies only to the lumbering, understeering Viper...
   if (fabs(steer) < 0.35
    //&& !(avoiding || (avoiding & AVOID_ALIGNED)) 
    && (segtype != TR_STR )
    // || (segtype == TR_LFT && car->_trkPos.toLeft > 3.0) 
    // || (segtype == TR_RGT && car->_trkPos.toRight > 3.0)))
     )
    steer *= (1.0 + (0.35 - fabs(steer)) * 1.3);

  if (fabs(steer) > 0.9 && fabs(laststeer) > 0.9 && lastgear >= 1 && fabs(laststeer-car->_yaw_rate) < fabs(laststeer))
   steer = laststeer;
 }

 if (rl == LINE_RL)
 {
  //
  // Handle getting unstuck
  //
  if (reversehold > s->currentTime && reversehold < s->currentTime + 1.2 &&
      ((nextleft > -1.0 && angle > 0.3 && nextleft < seg->width - 1.5) || 
       (nextleft < seg->width + 1.0 && angle < -0.3 && nextleft > 1.5)))
   reversehold = 0.0f;
 
  int stuck = isStuck();
 
  if (!pit->getPitstop() && !pit->getInPit() &&
      (stuck || reversehold > s->currentTime) && stuckhold < s->currentTime && car->_speed_x < 1.0 &&
      (fabs(angle)>1.4 || nextleft < 1.5 || nextleft > seg->width-1.5)) 
  {
   if (fStuckForward)
    steer = -laststeer;
   else if (fStuck)
   {
    if (laststeer > 0)
     steer = MAX(laststeer, fabs(steer));
    else
     steer = MIN(laststeer, -(fabs(steer)));
   }
   else
    steer = -steer;
   if (fabs(steer) * 4 > fabs(car->_speed_x))
    steer *= 0.6;

   car->ctrl.gear = car->_gearCmd = -1; // reverse gear
   if (reversehold < s->currentTime)
    reversehold = (float)(s->currentTime + 2.5);
   //car->ctrl.accelCmd = 0.7f; // 70% accelerator pedal
   AccelCmd = BTAccelCmd = 0.6;
   car->_clutchCmd = 0.0f;
   car->ctrl.brakeCmd = BrakeCmd = BTBrakeCmd = 0.0; // no brakes
   fStuck = 1;
   fStuckForward = 0;
   avoiding = 0;
   setCorrecting(0);
   setAllowCorrecting(0);
  } 
  else 
  {
   //float last_lane = car->_trkPos.toLeft / car->_trkPos.seg->width;
   if (stuck ||
       (car->_speed_x < 30.0 && !pit->getInPit() &&
        (((CosAngleError < 0.7 || (fStuck && CosAngleError < 0.9)) &&
          SinAngleError * Error > 0))))
   {
    /*if (fStuck && !fStuckForward && car->_gear == -1)
     steer = -laststeer;
    else */if (fStuckForward && fabs(laststeer) > 0.8 && car->_gear > -1)
     steer = laststeer;
    if (!pit->getInPit() && MIN(car->_trkPos.toLeft, car->_trkPos.toRight) < -2.0 && car->_speed_x < 3.0)
     steer *= 0.5;

    fStuckForward = 1;
    if (car->_speed_x > -2.0)
    {
     AccelCmd = BTAccelCmd = 0.6;
     car->_brakeCmd = BTBrakeCmd = BrakeCmd = 0.0f;
    }
    else
    {
     AccelCmd = BTAccelCmd = 0.0;
     car->_brakeCmd = BTBrakeCmd = BrakeCmd = 0.5;
    }
    car->_clutchCmd = 0.1f;
    if (!car->_gearCmd)
     car->_gearCmd = 1;
   }
   else
    fStuckForward = 0;
   fStuck = 0;
  }

  if (Learning > 0.0 && !fStuck && !fStuckForward && !avoiding && !getCorrecting() && align_hold < currentsimtime && alone && car->_speed_x > 15.0)
  {
   double lane2left = tLane[rl][thisdiv] * seg->width;
   double dr = 0.0;
   double limit = 2.5;
   double speed = Mag(car->_speed_Y, car->_speed_X);
   Error = (atan(Error * (300 / (speed + 300)) / 15) + VnError);
   int div = ((thisdiv-(int)((car->_speed_x*(car->_speed_x/9))/30)) + Divs) % Divs;
#if 0
   // BABIS style learning
   div = (tSegIndex[SegId] + int(dist / tElemLength[SegId])) % Divs;
   if (tLearnCount[div] < LearnLimit && lastdiv < thisdiv)
   {
    double speedinc = -1*(fabs(Error)-0.1) * Learning;
    int prev = ((div - 7) + Divs) % Divs;
    if (speedinc >= 0.1)
    {
     speed = (tMaxSpeed[div]*speedinc);
     tMaxSpeed[div] += speed;
     tLearnCount[div]++;
    }
    else
    {
     if (fabs(speed) < (tMaxSpeed[prev]/4))
     {
      tMaxSpeed[prev] += speed;
      tLearnCount[div]++;
     }
    }
   }
#else
   // My own hack
 
   if (segtype != TR_STR)
    limit -= MIN(2.4, ((140.0 - seg->radius) * ((140.0-seg->radius)/20)) / 250.0);
 
   if (lastcornertype == TR_RGT)
   {
    double outside = MAX(MIN(lane2left, 1.5), lane2left - limit);
    dr = car->_trkPos.toLeft - outside;
   }
   else if (lastcornertype == TR_LFT)
   {
    double outside = MIN(MAX(lane2left, seg->width - 1.5), lane2left + limit);
    dr = outside - car->_trkPos.toLeft;
   }
 
   if (dr > 0.0)
    dr = MIN(1.0, dr / 10);
   else
    dr = MAX(-1.0, dr / 5);
 
   if (tLearnCount[div] < LearnLimit)
   {
    tMaxSpeed[div] += dr * Learning;
    tLearnCount[div]++;
   }
#endif
  }
 }

 return steer;
} 

double Driver::AvoidSteer(tSituation *s)
{
 tTrackSeg *seg = car->_trkPos.seg;
 int next = nextdiv;
 next = (Divs+nextdiv+4) % Divs;
 double lookahead;
 double length = getDistToSegEnd();
 double nextright = seg->width - nextleft;
 double nlane2left = tLane[rl][next] * seg->width;
 double lane2left = tLane[rl][nextdiv] * seg->width;
 double lane2right = seg->width - lane2left;
#if 0
 double llane2left = ((tLane[LINE_LEFT][thisdiv] + tLane[LINE_LEFT][nextdiv])/2) * seg->width;
 double rlane2left = ((tLane[LINE_RIGHT][thisdiv] + tLane[LINE_RIGHT][nextdiv])/2) * seg->width;
#endif

 double speedfactor = 0.33;
 double speed = car->_speed_x;
 if (!pit->getInPit())
 {
  if (getCorrecting() && ((seg->type != TR_STR && seg->radius <= 200.0) || fabs(tRInverse[rl][nextdiv]) > 0.002) &&
      (((seg->type == TR_LFT || tRInverse[rl][nextdiv] > 0.002) && nextleft > lane2left + 2.0 && nlane2left > lane2left) ||
       ((seg->type == TR_RGT || tRInverse[rl][nextdiv] < -0.002) && nextleft < lane2left - 2.0 && nlane2left < lane2left)))
   speedfactor += MIN(0.7, fabs(nextleft-lane2left)/15);
  speed = ((car->_speed_x+car->_accel_x/5)-fabs(laststeer));
 }

 lookahead = LOOKAHEAD_CONST * (getCorrecting() ? 1.4 : 1.0) + speed * speedfactor;
#if 0
#ifdef K1999AVOIDSTEER
 if (seg->type != TR_STR && seg->radius <= 60.0)
  lookahead += (70.0-seg->radius) / 10.0;
#endif

 if ((segtype == TR_LFT && seg->radius <= 80.0 && nextleft < 90.0 - seg->radius / 10) ||
     (segtype == TR_RGT && seg->radius <= 80.0 && nextright < 90.0 - seg->radius / 10))
 {
  lookahead *= (1.0 - (90.0 - seg->radius)/150);
 }
 else if ((avoiding & AVOID_SIDE_COLL) && ((segtype == TR_LFT && (avoiding & AVOID_LEFT)) || (segtype == TR_RGT && (avoiding & AVOID_RIGHT))))
 {
  lookahead *= 0.8;
 }
 else if ((avoiding & AVOID_SIDE) && ((segtype == TR_LFT && (avoiding & AVOID_LEFT)) || (segtype == TR_RGT && (avoiding & AVOID_RIGHT))))
 {
  lookahead *= 0.9;
 }
#endif

 // prevent snap-back
 //double cmplookahead = oldlookahead - MIN(50.0, car->_speed_x)*RCM_MAX_DT_ROBOTS;
 if (!pit->getInPit() || car->_speed_x < 20.0)
 {
  double cmplookahead = oldlookahead - (car->_speed_x*RCM_MAX_DT_ROBOTS)/2;
  lookahead = MIN(lookahead, cmplookahead);
 }

 oldlookahead = (float)lookahead;

 while (length < lookahead)
 {
  seg = seg->next;
  length += seg->length;
 }

 length = lookahead - length + seg->length;
 double fromstart = seg->lgfromstart;
 fromstart += length;

 v2d t, target;
 t.x = (seg->vertex[TR_SL].x + seg->vertex[TR_SR].x)/2.0f;
 t.y = (seg->vertex[TR_SL].y + seg->vertex[TR_SR].y)/2.0f;

 if ( seg->type == TR_STR) {
  v2d d, n;
  n.x = (seg->vertex[TR_EL].x - seg->vertex[TR_ER].x)/seg->length;
  n.y = (seg->vertex[TR_EL].y - seg->vertex[TR_ER].y)/seg->length;
  n.normalize();
  d.x = (seg->vertex[TR_EL].x - seg->vertex[TR_SL].x)/seg->length;
  d.y = (seg->vertex[TR_EL].y - seg->vertex[TR_SL].y)/seg->length;
  target = t + d*(float)length + (float)myoffset*n;
 } else {
  v2d c, n;
  c.x = seg->center.x;
  c.y = seg->center.y;
  double arc = length/seg->radius;
  double arcsign = (seg->type == TR_RGT) ? -1.0 : 1.0;
  arc = arc*arcsign;
  t = t.rotate(c, (float)arc);

  n = c - t;
  n.normalize();
  target = t + (float)(arcsign*myoffset)*n;
 }
 seg = car->_trkPos.seg;

#if !K1999AVOIDSTEER 
 //memset(&(car->ctrl), 0, sizeof(tCarCtrl));
 //tTrackSeg *nseg = tSegment[tDivSeg[next]];
 //int ntype = nseg->type;
 //double ksteer = k1999steer;
 //double nleft = nextleft;//car->_trkPos.toLeft + (nextleft-car->_trkPos.toLeft)/2;
 //double newleft = nleft - (nleft - (seg->width/2 - myoffset)) * 0.45;

#if 0
 if (newleft < lane2left)
 {
  double factor = 1.0 - ((newleft - llane2left) / (lane2left - llane2left));
  ksteer = k1999steerL + (k1999steer-k1999steerL)*factor;
 }
 else if (newleft > lane2left)
 {
  double factor = ((rlane2left - newleft) / (rlane2left - lane2left));
  ksteer = k1999steerR + (k1999steer-k1999steerR)*factor;
 }
#endif


 double targetAngle = atan2(target.y - car->_pos_Y, target.x - car->_pos_X);
 double yr = car->_yaw_rate;// * MIN(0.5, MAX(0.02, fabs(laststeer)*1.5));
 //targetAngle -= (car->_yaw + yr);
 //targetAngle -= (lastyaw + yr);
 double yaw = car->_yaw;
 //if ((avoiding & AVOID_LEFT))
 // yaw = MAX(yaw, yaw + (yaw - lastyaw));
 //else if (avoiding & AVOID_RIGHT)
 // yaw = MIN(yaw, yaw + (yaw - lastyaw));
 targetAngle -= yaw + yr/20;
 NORM_PI_PI(targetAngle);
 k1999steer_a = targetAngle / car->_steerLock;

 if (fabs(car->_speed_y) > 1.5
     && fabs(car->_speed_y-angle) > fabs(car->_speed_y)
     && fabs(car->_speed_y-car->_yaw_rate) > fabs(car->_speed_y) 
     && fabs(car->_speed_y * (car->_speed_x/10)) > 1.0)
 {
  k1999steer_a += car->_speed_y * (1.0 - MIN(60.0, MAX(25.0, car->_speed_x)) / 80.0) / MAX(5, 20-fabs(car->_speed_y));
 }

 if (((angle > 0.8 || lastbrake > 0.7 || fabs(car->_yaw_rate) > 1.4) && k1999steer_a > 0.25) ||
     ((angle < -0.8 || lastbrake > 0.7 || fabs(car->_yaw_rate) > 1.4) && k1999steer_a < -0.25))
  k1999steer_a *= 0.8f;
#else
 // 
 // Find index in data arrays
 //
 int SegId = car->_trkPos.seg->id;
 double dist = car->_trkPos.toStart;
 if (dist < 0)
  dist = 0;
 if (car->_trkPos.seg->type != TR_STR)
  dist *= car->_trkPos.seg->radius;
 int Index = tSegIndex[SegId] + int(dist / tElemLength[SegId]);
 double d = tSegDist[SegId] + dist;
 Index = (Index + Divs - 5) % Divs;
 int Next = (Index + (int) (lookahead*2/DivLength)) % Divs;
 //static const double Time = 0.01;
 double Time = deltaTime*(getCorrecting() ? (2.85 + (fabs(lastksteer-laststeer))) : 2.8);//MAX(1.0, 3.0 - (CW/4 + FWA*7));
 if (pit->getInPit())
  Time = deltaTime * 3.0;
 //if ((seg->radius <= 200.0 && ((seg->type == TR_LFT && (avoiding & AVOID_LEFT)) || (seg->type == TR_RGT && (avoiding & AVOID_RIGHT)))))
 // Time *= 0.6;
 double X = car->_pos_X + car->_speed_X * Time / 2;
 double Y = car->_pos_Y + car->_speed_Y * Time / 2;

 //if ((avoiding && !(avoiding & AVOID_ALIGNED)) || getCorrecting())
 // Next = (Next + 2) % Divs;
 
 //Next = (nextdiv+2) % Divs;
 //int NextNext = (nextdiv + MAX(3, MIN(8, (int) (car->_speed_x/10)))) % Divs;
 Next = (nextdiv);// + (int) (getSpeed()/10)) % Divs;;
 int NextNext = (Next + 1) % Divs;

 int Prev = (Index + Divs - 1) % Divs;
 //int line = (fabs(tRInverse[rl][Next]) < fabs(tRInverse[rl][Next]) ? LINE_LEFT : LINE_RIGHT);
 int line = (fabs(tRInverse[rl][Next]) > 0.0 ? LINE_LEFT : LINE_RIGHT);
 int nline = (line == LINE_LEFT ? LINE_RIGHT : LINE_LEFT);
 line = LINE_MID;

 double newleft = seg->width * 0.5 - myoffset ;
 double newlane = newleft / seg->width;
 double c0, c1;
 //double txPrev = tx[line][Prev], tyPrev = ty[line][Prev];
 //double txNext = tx[line][Next], tyNext = ty[line][Next];
 //double txIndex = tx[line][Index], tyIndex = ty[line][Index];
 int prevxy = (getCorrecting() ? 2 : 3);

 //if ((avoiding & AVOID_SWITCH) && segtype != TR_STR && cornerdist > cornerlimit)
 // prevxy++;
 if (getCorrecting() || (seg->type == TR_LFT && newleft > nextleft) || (seg->type == TR_RGT && newleft < nextleft))
 {
  if (segtype != TR_STR && getCorrecting())
   prevxy -= 2;
  else
   prevxy--;
 }
 //else if (fabs(tRInverse[rl][nextdiv]) < 0.002)

 //double offset = (car->_trkPos.seg->width/2) - (tLane[rl][Next] * car->_trkPos.seg->width);
 int idx = tSegIndex[SegId] + int(dist / tElemLength[SegId]);
 double ndist1 = ((double) (nextdiv - idx) * DivLength);
 if (nextdiv < idx)
  ndist1 = ((double) ((nextdiv+Divs) - idx) * DivLength);
 double ndist2 = ((double) (NextNext - idx) * DivLength);
 if (NextNext < idx)
  ndist2 = ((double) ((NextNext+Divs) - idx) * DivLength);

 v2d target1, target2;
 target1.x = newlane * txRight[line][Next] + (1 - newlane) * txLeft[line][Next];
 target1.y = newlane * tyRight[line][Next] + (1 - newlane) * tyLeft[line][Next];
 target2.x = newlane * txRight[line][NextNext] + (1 - newlane) * txLeft[line][NextNext];
 target2.y = newlane * tyRight[line][NextNext] + (1 - newlane) * tyLeft[line][NextNext];
  
 prevxy = 1;
 int nprev = (pit->getInPit() ? 3 : 3);

 double txPrev = lastX[prevxy], tyPrev = lastY[prevxy];
 double txIndex = lastX[0];
 double tyIndex = lastY[0];
 double txNext = target1.x;
 double tyNext = target1.y;
 double txNextNext = target2.x;
 double tyNextNext = target2.y;

 double thislane = car->_trkPos.toLeft / seg->width;
 double nextnextlane = newlane + (newlane - thislane);

 c0 = (txNext - txIndex) * (txNext - X) +
      (tyNext - tyIndex) * (tyNext - Y);
 c1 = (txNext - txIndex) * (X - txIndex) +
      (tyNext - tyIndex) * (Y - tyIndex);
 {
  double sum = c0 + c1;
  c0 /= sum;
  c1 /= sum;
 }

 //
 // Find target curvature (for the inside wheel)
 //
 //double RIidx = (tRInverse[rl][Index]  + (tRInverse[line][Index])*3)/3;
 //double RInxt = (tRInverse[rl][Next] + (tRInverse[rl][Next])*3)/3;
 double RIidx = (tRInverse[line][Index]  + tRInverse[line][Index])/2;
 double RInxt = (tRInverse[line][Next] + tRInverse[line][Next])/2;
 double TargetCurvature = ((1 - c0) * RInxt + c0 * RIidx) * 1.2;
#if 1
 //if (seg->type != TR_STR || (prefer_side != TR_STR && cornerdist < cornerlimit 
 //    && ((prefer_side == TR_LFT && tRInverse[rl][next] > 0.0) || (prefer_side == TR_RGT && tRInverse[rl][next] < 0.0))))
 if ((TargetCurvature < 0.0 && KTC < TargetCurvature) || (TargetCurvature > 0.0 && KTC > TargetCurvature))
 {
#if 0
  int type = (seg->type != TR_STR ? segtype : prefer_side);
  double curvefactor = MIN(1.6, MAX(0.0, (type == TR_LFT ? car->_trkPos.toLeft / seg->width : car->_trkPos.toRight / seg->width) * 2));
  double curve = ((1 - c0) * tRInverse[rl][Next]*curvefactor + c0 * tRInverse[rl][Index]*curvefactor);
  TargetCurvature = (TargetCurvature + curve*2) / 2;
#else
  TargetCurvature += KTC/2;
#endif
 }
#endif

 if (fabs(TargetCurvature) > 0.01)
 {
  double r = 1 / TargetCurvature;
  if (r > 0)
   r -= wheeltrack / 2;
  else
   r += wheeltrack / 2;
  TargetCurvature = 1 / r;
 }

 if (car->_accel_x > -5.0 && SteerGainDiv < 1000)
  TargetCurvature *= 1.0 + ((car->_accel_x+5.0))/SteerGainDiv;
 /*
 if (!(avoiding & AVOID_SIDE_COLL) ||
     (KTC > TargetCurvature && !((avoiding & AVOID_SIDE) && (avoiding & AVOID_LEFT))) ||
     (KTC < TargetCurvature && !((avoiding & AVOID_SIDE) && (avoiding & AVOID_RIGHT))))
  TargetCurvature = TargetCurvature + (KTC - TargetCurvature)/10;
 else if ((segtype == TR_RGT && (avoiding & AVOID_RIGHT)) ||
          (segtype == TR_LFT && (avoiding & AVOID_LEFT)))
          */
 TargetCurvature = TargetCurvature + (KTC - TargetCurvature)/100;

 /*
 if (TargetCurvature > lastTC && TargetCurvature - lastTC > KTCdiff)
  TargetCurvature = lastTC + (TargetCurvature - lastTC) / 4;
 else if (TargetCurvature < lastTC && TargetCurvature - lastTC < KTCdiff)
  TargetCurvature = lastTC - (lastTC - TargetCurvature) / 4;
 lastTC = TargetCurvature;
 */

 //
 // Steering control
 //
 Error = 0;
 double VnError = 0;
 double Skid = 0;
 double steer = 0.0;
 CosAngleError = 1;
 SinAngleError = 0;
 
 {
  //
  // Ideal value
  //
  if (1)
  {
   steer = atan(wheelbase * TargetCurvature) / car->_steerLock;
   double steergain = SteerGain;
   if (!getCorrecting())
    steergain = MAX(1.1, SteerGain - 0.2);
   //if ((segtype == TR_RGT && (avoiding & AVOID_RIGHT)) || (segtype == TR_LFT && (avoiding & AVOID_LEFT)))
   // steergain = MAX(1.1, steergain - 0.4);
   if (car->_speed_x > 60)
    steergain = MAX(MIN(1.1, SteerGain), steergain - (car->_speed_x-60.0)/40);
#if 1
   //if (racetype != RM_TYPE_QUALIF)
   {
    if (segtype == TR_RGT)
    {
     if (car->_trkPos.toRight < 4.0)
      steergain = MAX(1.0, steergain - (car->_trkPos.toRight/17));
    }
    else if (segtype == TR_LFT)
    {
     if (car->_trkPos.toLeft < 4.0)
      steergain = MAX(1.0, steergain - (car->_trkPos.toLeft/17));
    }
   }
#endif
   steer = atan(wheelbase * steergain * TargetCurvature) / car->_steerLock;
   
   // this applies only to the lumbering, understeering Viper...
   if (fabs(steer) < 0.35 
    && !(avoiding || (avoiding & AVOID_ALIGNED)) 
    && (segtype != TR_STR || (segtype == TR_LFT && car->_trkPos.toLeft > 3.0) 
     || (segtype == TR_RGT && car->_trkPos.toRight > 3.0)))
    steer *= (1.0 + (0.35 - fabs(steer)) * 1.3);
  }

  //
  // Servo system to stay on the pre-computed path
  //
  {
   double dx = txNext - txIndex;
   double dy = tyNext - tyIndex;
   Error = (dx * (Y - tyIndex) - dy * (X - txIndex)) / Mag(dx, dy);
  }  

  double Prevdx = txNext - txPrev;
  double Prevdy = tyNext - tyPrev;
  double Nextdx = txNextNext - txIndex;
  double Nextdy = tyNextNext - tyIndex;
  double dx = c0 * Prevdx + (1 - c0) * Nextdx;
  double dy = c0 * Prevdy + (1 - c0) * Nextdy;
  double n = Mag(dx, dy);
  dx /= n;
  dy /= n;
  double speed = Mag(car->_speed_Y, car->_speed_X);
  double sError = (dx * car->_speed_Y - dy * car->_speed_X) / (speed + 0.01);
  double cError = (dx * car->_speed_X + dy * car->_speed_Y) / (speed + 0.01);
  VnError = asin(sError);
  if (cError < 0)
   VnError = PI - VnError;

  steer -= (atan(Error * (300 / (speed + 300)) / (15 / SteerGain2)) + VnError) / car->_steerLock;

  //
  // Steer into the skid
  //
  double vx = car->_speed_X;
  double vy = car->_speed_Y;
  double dirx = cos(car->_yaw);
  double diry = sin(car->_yaw);
  CosAngleError = dx * dirx + dy * diry;
  SinAngleError = dx * diry - dy * dirx;
  Skid = (dirx * vy - vx * diry) / (speed + 0.1);
  if (Skid > 0.9)
   Skid = 0.9;
  if (Skid < -0.9)
   Skid = -0.9;
  steer += (asin(Skid) / car->_steerLock) * 0.9;

  double yr = speed * TargetCurvature;
  double diff = car->_yaw_rate - yr;
  //double diff = (car->_yaw_rate-(car->_yaw_rate-lastyr)/4) - yr;
  double steerskid = SteerSkid;
  if ((flying_timer > currentsimtime - 3.5) && 
      (segtype == TR_STR ||
       (segtype == TR_RGT && nextleft > 4.0f) ||
       (segtype == TR_LFT && nextleft < seg->width - 4.0f) ||
       (flying_timer < currentsimtime - 3.5 && (nextleft <= 0.0 || nextleft >= seg->width))))
  {
   steerskid = MAX(SteerSkid, MIN(0.24, SteerSkid + MIN(0.2, fabs(laststeer-lastksteer) * 0.7)));
   if (flying_timer > currentsimtime - 3.5)
   {
    steerskid= MAX((0.20 + (3.5 - (currentsimtime - flying_timer)) * 0.08), SteerSkid);
   }
   else if ((nextleft <= 0.0 || nextleft >= seg->width) && steerskid > SteerSkid)
    steerskid = MAX(steerskid * (0.3 - fabs(angle)*0.5), SteerSkid);
  }
  else if ((getCorrecting() || getAllowCorrecting()) && getCorrectTimer() > 0.0)
   steerskid = MAX(steerskid, 0.25 - (CORRECT_TIMER-(getCorrectTimer()))/30);

  steer -= (steerskid * (1 + fDirt) * (100 / (speed + 90)) * diff) / car->_steerLock;
  if ((last_flying & FLYING_FRONT) && fabs(angle) < 0.3 && nextleft > 0.5f && nextleft < seg->width - 0.5f)
  {
   steer = MAX(-0.1, MIN(0.1, steer));
  }

#if 1
  if (fabs(car->_yaw_rate) < 1.8 && getSpeed() < 10.0 && fabs(angle) > 1.6)
  {
   if ((angle > 1.6f && car->_trkPos.toRight > angle*2 && steer < -0.1) || 
       (angle < -1.4f && car->_trkPos.toLeft > fabs(angle)*2 && steer > 0.1))
    steer = -steer;
   if ((angle >= 1.6 && car->_trkPos.toRight < angle*2 && steer > 0.1) || 
       (angle <= -1.6 && car->_trkPos.toLeft < fabs(angle)*2 && steer < -0.1))
    steer = -steer;
  }
  else 
#endif
  if (fabs(car->_yaw_rate) >= 1.8 && getSpeed() > 10.0)
  {
   AccelCmd = 0.0;
   BrakeCmd = 1.0;
  }

  if (fabs(steer) > 0.9 && fabs(laststeer) > 0.9 && lastgear >= 1 && fabs(laststeer-car->_yaw_rate) < fabs(laststeer))
   steer = laststeer;

  k1999steer_a = steer;
 }
#endif // K1999AVOIDSTEER

 if (!pit->getInPit()) 
 {
  int csegtype = seg->type;
  double segradius = seg->radius;

  if (segtype == TR_STR && cornerdist < cornerlimit && cseg)
  {
   csegtype = cseg->type;
   segradius = cseg->radius;
  }
 
  double oldsteer = k1999steer_a;
  // if we need to include some of the raceline steering...
#if 1
  if (getCorrecting())
  {
   double meldfactor = 0.2;
                       //((avoiding & AVOID_SIDE_COLL) ? MAX(0.25, 0.6 - (sidemargin/20)) : 
                       // (avoiding & AVOID_SIDE) ? MAX(0.2, 0.4 - (sidemargin/20)) : 0.2);
   /*
   if ((avoiding & AVOID_SIDE) && segtype != TR_STR)
   {
    if ((segtype == TR_LFT && (avoiding & AVOID_LEFT)) || (segtype == TR_RGT && (avoiding & AVOID_RIGHT)))
     meldfactor = MIN(meldfactor, 2.0);
    else if ((segtype == TR_LFT && (avoiding & AVOID_RIGHT)) || (segtype == TR_RGT && (avoiding & AVOID_LEFT)))
     meldfactor *= 1.5;
   }
   */

   double lsteerdiff = (laststeer - lastksteer);
   double csteerdiff = (k1999steer_a - k1999steer);

   if (lsteerdiff < 0.0 && csteerdiff < lsteerdiff)
    k1999steer_a = MAX(k1999steer_a, (k1999steer + lsteerdiff) - deltaTime * meldfactor);
   else if (lsteerdiff > 0.0 && csteerdiff > lsteerdiff)
    k1999steer_a = MIN(k1999steer_a, (k1999steer + lsteerdiff) + deltaTime * meldfactor);
  }
#endif
#if 0
  if ((csegtype != TR_STR))
  {
   {
    // basic factor due to corner radius
    factor = MAX(0.03, 0.17 - (segradius*0.002));
    if (getCorrecting() || getAllowCorrecting())
     factor *= 3.0;
    if ((csegtype == TR_LFT && lastksteer > laststeer) || (csegtype == TR_RGT && lastksteer < laststeer))
     factor *= 3.0;

    if (segradius < 25.0)
     factor *= 4.0;

    if (factor > correct_meld)
    {
     if (segradius < 25.0)
      factor = correct_meld = correct_meld + (deltaTime * 16);
     else
      factor = correct_meld = correct_meld + deltaTime * 8;
    }
    else
     correct_meld = factor;
   }
   if (seg->type == TR_STR)
   {
    factor *= 0.7;
    correct_meld = factor;
   }
   if ((avoiding & AVOID_FRONT) && frontmargin > 3.0)
    factor = MIN(0.5, factor * 1.0 + (frontmargin-3.0)/5);
   if ((!(avoiding & AVOID_FRONT) || frontmargin < 3.0) && sidemargin < 4.0 && 
       ((k1999steer > k1999steer_a && (avoiding & AVOID_LEFT)) || (k1999steer<k1999steer_a && (avoiding & AVOID_RIGHT))))
   {
    factor = correct_meld = MAX(0.0, correct_meld - correct_meld / (sidemargin/3));
    if ((avoiding & AVOID_TEAM) || sidemargin <= 2.0)
     factor = correct_meld = 0.0;
   }
   else if (sidemargin < 8.0 && (avoiding & AVOID_SIDE) && 
            (((avoiding & AVOID_LEFT) && k1999steer > k1999steer_a) ||
             ((avoiding & AVOID_RIGHT) && k1999steer < k1999steer_a)))
   {
    factor *= sidemargin/10;
    correct_meld = factor;
   }
#if 0
   if (!(avoiding & AVOID_SIDE_COLL) && (correcting || sidemargin > 6.0))
    factor *= 1.5;

   // increase as its a sharp corner
   if (segradius < 25.0)
    factor += (30.0-segradius)/25;
   // decrease as we're close to another car.
   if (sidemargin < 8.0 && ((k1999steer > k1999steer_a && (avoiding & AVOID_LEFT)) || (k1999steer<k1999steer_a && (avoiding & AVOID_RIGHT))))
   {
    factor = 0.0;
   }
   // decrease as we're on a straight atm
   if (segtype == TR_STR)
    factor *= MAX(0.01, (1.0 - ((double)d/12)));
   factor = MAX(0.0, MIN(0.4, factor));
#endif

   // meld with raceline
   k1999steer_a += (k1999steer - k1999steer_a) * factor;
//fprintf(stderr,"->%.3f (%.3f)",k1999steer_a,correct_meld);
  }
  else
   correct_meld = 0.0;
#endif
//fprintf(stderr,"B%.3f ",k1999steer_a);


#if 0
  if (nextleft < llane2left && (seg->type != TR_LFT || seg->radius > 100.0))
   k1999steer_a = MIN(k1999steer_a, MAX(k1999steerL, k1999steer));
  else if (nextleft > rlane2left && (seg->type != TR_RGT || seg->radius > 100.0))
   k1999steer_a = MAX(k1999steer_a, MIN(k1999steerR, k1999steer));
#endif
//fprintf(stderr,"C%.3f ",k1999steer_a);

  // make sure the change isn't too sudden
  double limitfactor = SteerLimit;//1.0 - MIN(0.7, 3.2 - MIN(3.2, CA)) * 0.08;
  limitfactor += (1.0-limitfactor) * (0.6+(0.19*2));
  limitfactor -= ((car->_speed_x < 30 ? 0.0f : (MIN(75.0f, MAX(40.0, car->_speed_x)) - 30.0f) * 0.006));
  limitfactor = MAX(limitfactor, 0.01f) * 1.3;
  //double aerofactor = 0.09 * 4;

  if (!(currentsimtime < 20.0 ||
       fabs(k1999steer_a-laststeer) > fabs(oldsteer-laststeer) ||
       (k1999steer > oldsteer && (avoiding & AVOID_LEFT)) ||
       (k1999steer < oldsteer && !(avoiding & AVOID_RIGHT))))
   limitfactor *= 2;

  double lftlimit = (SEGTYPE(seg) == TR_LFT && prefer_side == TR_LFT && (nextleft > 6.0f || (avoiding & AVOID_SIDE)) && !(avoiding&AVOID_LEFT) ? 0.08: seg->type != TR_RGT && prefer_side == TR_LFT ? 0.07 : 0.025) * (limitfactor);
  double rgtlimit = (segtype == TR_RGT && prefer_side == TR_RGT && (nextright > 6.0f || (avoiding & AVOID_SIDE)) && !(avoiding&AVOID_RIGHT) ? 0.08 : segtype != TR_LFT && prefer_side == TR_RGT ? 0.07 : 0.025) * (limitfactor);

  {
   if (k1999steer_a != oldsteer)
    k1999steer_a = MAX(oldsteer - rgtlimit, MIN(oldsteer + lftlimit, k1999steer_a));

#if 0
   int avoidside = ((avoiding & AVOID_TEAM) || (avoiding & AVOID_SIDE_COLL));

   if (avoiding && 
       ((!(avoiding & AVOID_LEFT) && k1999steer > k1999steer_a && k1999steerL > k1999steer_a) ||
        (!(avoiding & AVOID_RIGHT) && k1999steer < k1999steer_a && k1999steerR < k1999steer_a)))
    avoidside = 2;

   if (avoiding && (!(avoiding & AVOID_ALIGNED) || avoidside))
   {
    double ratio = 0.7; //1.0;

    if (!(avoiding & AVOID_SIDE))
     ratio = 0.7;
    if (segtype == TR_STR && avoidside != 2)
     ratio *= 0.7;

    if ((avoiding & AVOID_RIGHT) && !(avoiding & AVOID_LEFT) && k1999steer_a < k1999steerL && (segtype == TR_LFT || sidemargin < 2.0 || avoidside))
     k1999steer_a += MIN(lftlimit*ratio, k1999steerL-k1999steer_a);
    else if ((avoiding & AVOID_LEFT) && !(avoiding & AVOID_RIGHT) && k1999steer_a > k1999steerR && (segtype == TR_RGT || sidemargin < 2.0 || avoidside))
     k1999steer_a -= MIN(rgtlimit*ratio, k1999steer_a-k1999steerR);
   }
//fprintf(stderr,"D%.3f (l%.3f r%.3f) ",k1999steer_a,k1999steerL,k1999steerR);

   if (k1999steer_a != oldsteer)
    k1999steer_a = MAX(oldsteer - rgtlimit*1.5, MIN(oldsteer + lftlimit*1.5, k1999steer_a));
//fprintf(stderr,"E%.3f ",k1999steer_a);
#endif
  }

#if 1
  if ((avoiding && !(avoiding & AVOID_ALIGNED)) || getCorrecting())
  {
   double nextright = seg->width - nextleft;
   if (seg->type == TR_RGT && tSegRadius[tDivSeg[nextdiv]] < 150.0 && nextleft < tAvoidLeft[next])
    TargetSpeed *= 1.0 - MAX(0.0, MIN(0.3, (nextleft / lane2left) * 0.4) - angle*0.8) * 0.75;
   else if (seg->type == TR_LFT && tSegRadius[tDivSeg[nextdiv]] < 150.0 && nextright < tAvoidRight[next])
    TargetSpeed *= 1.0 - MAX(0.0, MIN(0.3, (nextright / lane2right) * 0.4) + angle*0.8) * 0.75;
  }
#endif

  if (avoiding && !old_avoiding && fabs(k1999steer_a - laststeer) > MAX(10.0, (100.0-car->_speed_x)+MIN(0.0, car->_accel_x*2)) * ((avoiding & AVOID_SIDE_COLL) ? 0.012 : ((avoiding & AVOID_SIDE) ? 0.006 : 0.003)))
  {
   if (avoiding & AVOID_SIDE_COLL)
   {
    k1999steer_a = laststeer + (k1999steer_a-laststeer)/5;
//fprintf(stderr,"F%.3f ",k1999steer_a);
   }
   else
   {
    // too sharp a steering change - abort the overtake
    avoiding = 0;
    setCorrecting(1);
    k1999steer_a = laststeer + (k1999steer_a-laststeer)/3;
//fprintf(stderr,"G%.3f ",k1999steer_a);
   }
  }

  if (avoiding)
  {
   setCorrecting(0);
   correct->setAligned(0);
  }
  else if (old_avoiding)
  {
   setCorrecting(1);
  }

  if ((last_flying & FLYING_FRONT) && fabs(k1999steer_a) > fabs(laststeer))
   k1999steer_a = laststeer;
//fprintf(stderr,"I%.3f ",k1999steer_a);

#if 1
 if (!(avoiding & AVOID_SIDE) && fabs(tRInverse[LINE_RL][nextdiv]) > WingRInverse)
 {
  // prevent going too far away from the raceline too quickly
  double factor = MAX(deltaTime*10, deltaTime*50 - (fabs(tRInverse[LINE_RL][nextdiv]) - WingRInverse));

  if (fabs(k1999steer_a - k1999steer) > fabs(correct_diff))
  {
   if (k1999steer_a > k1999steer)
    k1999steer_a = MIN(k1999steer_a, k1999steer + (correct_diff + factor));
   else
    k1999steer_a = MAX(k1999steer_a, k1999steer - (correct_diff + factor));
  }
 }
#endif

  if (!pit->getInPit())
  {
   if ((car->_trkPos.toLeft < -2.0 && nextleft/seg->width < tLane[rl][nextdiv]-0.1) || 
       (car->_trkPos.toRight < -2.0 && nextleft/seg->width > tLane[rl][nextdiv]+0.1))
    k1999steer_a *= 0.7;

   if ((avoiding))// && !(avoiding & AVOID_SIDE))
   {
    double factor = (getCorrecting() ? 1.6 : 0.5 + MAX(fabs(angle), fabs(car->_yaw_rate)/2));
    if (car->_accel_x < 0.0)
     factor = MAX(0.1, factor + car->_accel_x/5);

    if (k1999steer_a < k1999steer)// && k1999steer_a - k1999steer < correct_diff)
    {
     k1999steer_a = MAX(k1999steer_a, (k1999steer - fabs(correct_diff)) - rgtlimit*factor);
     k1999steer_a = MIN(k1999steer_a, (k1999steer - fabs(correct_diff)) + lftlimit*(factor*1.6));
    }
    else if (k1999steer_a > k1999steer)// && k1999steer_a - k1999steer > correct_diff)
    {
     k1999steer_a = MIN(k1999steer_a, (k1999steer + correct_diff) + lftlimit*factor);
     k1999steer_a = MAX(k1999steer_a, (k1999steer - fabs(correct_diff)) - rgtlimit*(factor*1.6));
    }
   }
  }
 }

 return k1999steer_a;
} 

Driver::Driver(int index)
 : stuck(0),
 speedangle(0.0),
 mass(0.0),
 fuelperlap(0.0),
 ToLeft(0.0),
 myoffset(0.0),
 car(NULL),
 opponents(NULL),
 opponent(NULL),
 pit(NULL),
 strategy(NULL),
 correct(NULL),
 mycardata(NULL),
 deltaTime(0.0),
 lastLTleft(0.0),
 avoid_diverge(0.0),
 lastTleft(0.0),
 currentspeedsqr(0.0),
 clutchtime(0.0),
 oldlookahead(0.0),
 learn(NULL),
 alone(0),
 rl(0),
 MAX_UNSTUCK_COUNT(0),
 INDEX(0),
 carindex(0),
 CARMASS(0.0),
 CA(0.0),
 FCA(0.0),
 RCA(0.0),
 FWA(0.0),
 CW(0.0),
 CR(0.0),
 brakeCW(0.0),
 TIREMU(0.0),
 GET_DRIVEN_WHEEL_SPEED(NULL),
 OVERTAKE_OFFSET_INC(0.0),
 MU_FACTOR(0.0),
 WingRInverse(0.0),
 TireAccel1(0.0),
 MaxBrake(0.0),
 SlipLimit(0.0),
 TSlipLimit(0.0),
 SteerSkid(0.0),
 SteerGain(0.0),
 SteerGain2(0.0),
 SecurityZ(0.0),
 CurveAccel(0.0),
 CurveFactor(0.0),
 RevsChangeDown(0.0),
 RevsChangeDownMax(0.0),
 RevsChangeUp(0.0),
 FlyingCaution(0.0),
 FlyingWidth(0.0),
 TrackFriction(0.0),
 PitDamage(0.0),
 IntMargin(0.0),
 ExtMargin( 0.0 ),
 AvoidSpeed( 0.0 ),
 CAModifier( 0.0 ),
 SteerLimit( 0.0 ),
 EdgeAllowance( 0.0 ),
 FuelSpeedup( 0.0 ),
 BrakeShift( 0.0 ),
 AvoidMargin( 0.0 ),
 ABSFactor( 0.0 ),
 BrakePressure( 0.0 ),
 Learning( 0.0 ),
 RaceLearning( 0.0 ),
 MaxIncFactor( 0.0 ),
 TurnAccel( 0.0 ),
 TurnDecel( 0.0 ),
 BTBoost( 0.0 ),
 AvoidBTBoost( 0.0 ),
 TeamWaitTime( 0.0 ),
 SteerGainDiv( 0.0 ),
 pszCarName( NULL ),
 ABS( 0.0 ),
 TractionHelp( 0.0 ),
 wheelbase( 0.0 ),
 wheeltrack( 0.0 ),
 Width( 0.0 ),
 Length( 0.0 ),
 ClutchTime( 0.0 ),
 AccelCmd( 0.0 ),
 BrakeCmd( 0.0 ),
 KBrakeCmd( 0.0 ),
 TargetSpeed( 0.0 ),
 BTargetSpeed( 0.0 ),
 KTargetSpeed( 0.0 ),
 LTargetSpeed( 0.0 ),
 RTargetSpeed( 0.0 ),
 angle( 0.0 ),
 correct_meld( 0.0 ),
 correct_diff( 0.0 ),
 align_hold( 0.0 ),
 avoid_hold( 0.0 ),
 nextleft( 0.0 ),
 prevleft( 0.0 ),
 k1999steer( 0.0 ),
 k1999steer_a( 0.0 ),
 k1999steer_c( 0.0 ),
 k1999steerL( 0.0 ),
 k1999steerR( 0.0 ),
 laststeer( 0.0 ),
 lastlaststeer( 0.0 ),
 lastTC( 0.0 ),
 KTC( 0.0 ),
 KTCdiff( 0.0 ),
 lastyaw( 0.0 ),
 bc_timer( 0.0 ),
 lastksteer( 0.0 ),
 lastbrake( 0.0 ),
 lastaccel( 0.0 ),
 slowslip( 0.0 ),
 speedfactor( 0.0 ),
 aero( 0.0 ),
 brake_collision( 0.0 ),
 CosAngleError( 0.0 ),
 SinAngleError( 0.0 ),
 Error( 0.0 ),
 sidemargin( 0.0 ),
 frontmargin( 0.0 ),
 BTAccelCmd( 0.0 ),
 BTBrakeCmd( 0.0 ),
 cornerdist( 0.0 ),
 cornerlimit( 0.0 ),
 flying_timer( 0.0 ),
 lastLoffset( 0.0 ),
 lastRoffset( 0.0 ),
 prevwidth( 0.0 ),
 nextwidth( 0.0 ),
 skill_level( 0.0 ),
 cur_speed_adjust( 0.0 ),
 trg_speed_adjust( 0.0 ),
 speed_adjust_limit( 0.0 ),
 speed_adjust_timer( 0.0 ),
 brake_avoid( 0.0 ),
 last_speedy( 0.0 ),
 last_lapfuel(0.0),
 reversehold( 0.0 ),
 stuckhold( 0.0 ),
 raceline( 0 ),
 fStuck( 0 ),
 NoAccelRelax( 0 ),
 LearnLimit( 0 ),
 K1999Brakes( 0 ),
 LoadSVG( 0 ),
 SaveSVG( 0 ),
 NoTeamWaiting( 0 ),
 OverrideLearning( 0 ),
 Iterations( 0 ),
 Divs( 0 ),
 Segs( 0 ),
 LearningLoaded( 0 ),
 segtype( 0 ),
 nextdiv( 0 ),
 thisdiv( 0 ),
 avoiding( 0 ),
 racetype( 0 ),
 prefer_side( 0 ),
 lastdiv( 0 ),
 lastgear( 0 ),
 lastsegid( 0 ),
 last_damage( 0 ),
 last_lap( 0 ),
 old_avoiding( 0 ),
 segupdated( 0 ),
 start_finished( 0 ),
 useBTSpeed( 0 ),
 lastcornertype( 0 ),
 reversetype( 0 ),
 fStuckForward( 0 ),
 side_count( 0 ),
 random_seed( 0 ),
 cseg( NULL ),
 
 fDirt( 0 ),
 last_flying( 0 ),
 drivetrain( 0 )
{
 INDEX = index;
 int xx, yy;

 for (xx = 0; xx < 4; ++xx)
   wheelz[ xx ] = 0.0;

 for (xx = 0; xx < MaxSegments; ++xx) 
 {
   tSegDist[xx] = 0.0;
   tElemLength[xx] = 0.0;
   tLSlope[xx] = 0.0;
   tRSlope[xx] = 0.0;
   tLDelta[xx] = 0.0;
   tRDelta[xx] = 0.0;
   tRadius[xx] = 0.0;
   tSegArc[xx] = 0.0;
   tSegment[xx] = NULL;
// tAvoidRight[xx] = 0.0;
// tAvoidLeft[xx] = 0.0;

   tSegHeight[xx] = 0.0;
   tSegLength[xx] = 0.0;
   tSegIndex[xx] = 0;
   tSegID[xx] = 0;
   tSegDivStart[xx] = 0;
 }

 for (xx = 0; xx < MaxDivs; ++xx) 
 {
   Flying[xx] = 0.0;
   tMaxSpeed[xx] = 0.0;
   tLSlowSpeed[xx] = 0.0;
   tRSlowSpeed[xx] = 0.0;
   tSpeedAdjust[xx] = 0.0;
   tLaneAdjust[xx] = 0.0;
   tSpeed[xx] = 0.0;
   tLaneLMargin[xx] = 0.0;
   tLaneRMargin[xx] = 0.0;
   tFriction[xx] = 0.0;

   tDivSeg[xx] = 0;
   tLearnCount[xx] = 0;
   //tfConst[xx] = 0;
   Braking[xx] = 0;
 }

 for (xx = 0; xx < 3; ++xx) 
 {
  for (yy = 0; yy < MaxDivs; ++yy) 
  {
   tx[xx][yy] = 0.0f;
   ty[xx][yy] = 0.0f;
   tRInverse[xx][yy] = 0.0;
   txLeft[xx][yy] = 0.0;
   tyLeft[xx][yy] = 0.0;
   txRight[xx][yy] = 0.0;
   tyRight[xx][yy] = 0.0;
   tLane[xx][yy] = 0.0;
  }
 }

 for (xx = 0; xx < 10; ++xx) 
 {
   lastX[xx] = 0.0;
   lastY[xx] = 0.0;
 }

 k1999pt.x = 0.0;
 k1999pt.y = 0.0;
 nextpt.x = 0.0;
 nextpt.y = 0.0;

 for (xx = 0; xx < 4; ++xx) 
 {
   corner1[xx].x = 0.0;
   corner1[xx].y = 0.0;
   corner1[xx].z = 0.0;
   corner1[xx].ax = 0.0;
   corner1[xx].ay = 0.0;
   corner1[xx].az = 0.0;

   corner2[xx].x = 0.0;
   corner2[xx].y = 0.0;
   corner2[xx].z = 0.0;
   corner2[xx].ax = 0.0;
   corner2[xx].ay = 0.0;
   corner2[xx].az = 0.0;
 }
}


Driver::~Driver()
{
 if (Learning && racetype != RM_TYPE_RACE)
  saveLearning();

 delete opponents;
 delete pit;
 delete strategy;
 delete learn;
 if (cardata != NULL) {
  delete cardata;
  cardata = NULL;
 }
 if (correct != NULL)
 {
  delete correct;
  correct = NULL;
 }
}

// Called for every track change or new race.
void Driver::initTrack(tTrack* t, void *carHandle, void **carParmHandle, tSituation *s)
{
 track = t;

 const int BUFSIZE = 256;
 char buffer[BUFSIZE];
 char indexstr[32];

 RtGetCarindexString(INDEX, "hymie", INDEX < 1 || INDEX > 10, indexstr, 32);

#ifdef CONTROL_SKILL
 // load the skill level
 snprintf(buffer, BUFSIZE, "config/raceman/extra/skill.xml");
 void *skillHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
 if (skillHandle)
  skill_level = GfParmGetNum(skillHandle, "skill", "level", (char *) NULL, 10.0);
 else
  skill_level = 0.0;
#endif

 // load the default setup.  Track specific ones will override this.
 snprintf(buffer, BUFSIZE, "drivers/%s/%s/default.xml", DRIVERNAME, indexstr);
 *carParmHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
 
 char* trackname = strrchr(track->filename, '/') + 1;
 void *newHandle;

 // Load setup for the track
 snprintf(buffer, BUFSIZE, "drivers/%s/%s/%s", DRIVERNAME, indexstr,trackname);
 newHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
 if (newHandle)
 {
  if (*carParmHandle)
   *carParmHandle = GfParmMergeHandles(*carParmHandle, newHandle, (GFPARM_MMODE_SRC|GFPARM_MMODE_DST|GFPARM_MMODE_RELSRC|GFPARM_MMODE_RELDST));
  else
   *carParmHandle = newHandle;
 }

 // Load setup for the racetype
 switch (s->_raceType) {
  case RM_TYPE_QUALIF:
   snprintf(buffer, BUFSIZE, "drivers/%s/%s/qualifying/%s", DRIVERNAME, indexstr, trackname);
   break;
  case RM_TYPE_PRACTICE:
   snprintf(buffer, BUFSIZE, "drivers/%s/%s/practice/%s", DRIVERNAME, indexstr, trackname);
   break;
  case RM_TYPE_RACE:
   snprintf(buffer, BUFSIZE, "drivers/%s/%s/race/%s", DRIVERNAME, indexstr, trackname);
   break;
  default:
   break;
 }

 newHandle = GfParmReadFile(buffer, GFPARM_RMODE_STD);
 if (newHandle)
 {
  if (*carParmHandle)
   *carParmHandle = GfParmMergeHandles(*carParmHandle, newHandle, (GFPARM_MMODE_SRC|GFPARM_MMODE_DST|GFPARM_MMODE_RELSRC|GFPARM_MMODE_RELDST));
  else
   *carParmHandle = newHandle;
 }

#ifdef CONTROL_SKILL
 skill_level += GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, "Skill", (char *) NULL, 0.0);
#endif

 // Create a pit stop strategy object.
 strategy = new SimpleStrategy();

 // Init fuel.
 strategy->setFuelAtRaceStart(t, carParmHandle, s);

 // Load and set parameters.
 MU_FACTOR = GfParmGetNum(*carParmHandle, HYMIE_SECT_PRIV, HYMIE_ATT_MUFACTOR, (char*)NULL, 0.69f);

 //K1999InitTrack(t, carParmHandle, s);
}


// Start a new race.
void Driver::newRace(tCarElt* car, tSituation *s)
{
 deltaTime = (float) RCM_MAX_DT_ROBOTS;
 MAX_UNSTUCK_COUNT = int(UNSTUCK_TIME_LIMIT/deltaTime);
 OVERTAKE_OFFSET_INC = (float)(OVERTAKE_OFFSET_SPEED*deltaTime);
 stuck = last_flying = start_finished = 0;
 alone = 1;
 clutchtime = lastyaw = bc_timer = 0.0;
 oldlookahead = 0.0;
 this->car = car;
#ifdef CONTROL_SKILL
#ifdef _driveSkill
 skill_level = car->_driveSkill;
#endif
#endif
 CARMASS = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_MASS, NULL, 1000.0);
 myoffset = ToLeft = lastTC = KTC = 0.0;
 initCa();
 initCw();
 initCR();
 initTireMu();
 aero = (((CA * 0.45 + CW * 0.16) / 9.0f) 
         - ((float) CARMASS-950.0f) * 0.0001 /*0.00007*/ 
         + MAX(0.0f, (track->width - 8.0f) * 0.0014f))
         * (TIREMU/1.6f);
 aero = MIN(0.2f, aero);

 racetype = s->_raceType;
 K1999InitTrack(track, &car->_carHandle, s);
 initTCLfilter();
 initWheelPos();

 memset(telemetry, 0, sizeof(telemetry));

 // Create just one instance of cardata shared by all drivers.
 if (cardata == NULL) {
  cardata = new Cardata(s);
 }
 mycardata = cardata->findCar(car);

 if (NULL == correct)
 {
  correct = new Correct(car);
 }

 currentsimtime = s->currentTime;

 // initialize the list of opponents.
 opponents = new Opponents(s, this, cardata);
 opponent = opponents->getOpponentPtr();

 // create the pit object.
 pit = new Pit(s, this);

 wheelbase = (car->priv.wheel[FRNT_RGT].relPos.x +
              car->priv.wheel[FRNT_LFT].relPos.x -
              car->priv.wheel[REAR_RGT].relPos.x -
              car->priv.wheel[REAR_LFT].relPos.x) / 2;
 wheeltrack = (car->priv.wheel[FRNT_LFT].relPos.y +
               car->priv.wheel[REAR_LFT].relPos.y -
               car->priv.wheel[FRNT_RGT].relPos.y -
               car->priv.wheel[REAR_RGT].relPos.y) / 2;

 ABS = 1;
 TractionHelp = 1;
 fStuckForward = fStuck = avoiding = old_avoiding = 0;
 setCorrecting(1);
 align_hold = avoid_hold = flying_timer = correct_meld = reversehold = 0.0;
 lastbrake = lastaccel = laststeer = lastksteer = 0.0f;
 prevleft = car->_trkPos.toLeft;
 last_damage = car->_dammage;
 segupdated = 0;
 reversetype = 1;
 slowslip = 1.0;
 lastsegid = tSegID[car->_trkPos.seg->id];

 sidemargin = 1000;// + (track->width > 12.0f ? (track->width - 12.0f) * 0.05f : 0.0f);
 lastgear = 0;
 stuckhold = 3.0f;
 useBTSpeed = 0;
 lastdiv = 0;
 lastcornertype = TR_STR;
 int i;
 for (i=0; i<10; i++)
 {
  lastX[i] = car->_pos_X;
  lastY[i] = car->_pos_Y;
 }
 for (i=0; i<4; i++)
 {
  corner2[i].ax = corner1[i].ax = car->_corner_x(i);
  corner2[i].ax = corner1[i].ay = car->_corner_y(i);
 }
 lastLoffset = car->_trkPos.toLeft - getWidth()/2;
 lastRoffset = car->_trkPos.toRight - getWidth()/2;
 lastLTleft = -1000.0;
 lastTleft = car->_trkPos.toLeft;
 avoid_diverge = 0.0;
 last_lapfuel = 0.0;
 last_lap = car->_laps;

 speed_adjust_limit = 3.0;
 cur_speed_adjust = trg_speed_adjust = speed_adjust_timer = 0.0;
 carindex = 0;

 for (i = 0; i < s->_ncars; i++)
 {
  if (s->cars[i] == car)
  {
   carindex = i;
   break;
  }
 }
 SetRandomSeed(carindex);
}

// Drive during race.
void Driver::drive(tSituation *s)
{
 //memset(&car->ctrl, 0, sizeof(tCarCtrl));
 car->_steerCmd = car->_brakeCmd = car->_accelCmd = car->_clutchCmd = 0.0;

 segtype = SEGTYPE(car->_trkPos.seg);
 rl = LINE_RL;
 speedfactor = 1.0;

 update(s);
 correct->update(s->currentTime, (!avoiding && !getCorrecting()), segtype);

 {
  reversetype = 1;
  updateSegAvoidance();
  car->_steerCmd = (float)getSteer(s);
  if (!fStuck)
  {
   if (!fStuckForward)
    stuckhold = (float)(s->currentTime + 3.0);
   car->_gearCmd = getGear();
   car->_accelCmd = (float)filterOverlap(filterTCL(filterTrk(getAccel()), 0));
   if (brake_collision && fabs(angle) < 0.5)
   {
    double bc = brake_collision;
    brake_collision = 0.0;
    double kBrake = filterABS(filterBrakeSpeed(filterBPit(getBrake(KBrakeCmd)))) * 0.7;
    brake_collision = bc;
    car->_brakeCmd = (float)MAX(kBrake, filterABS(filterBrakeSpeed(filterBPit(getBrake(BrakeCmd)))));
   }
   else
   {
    car->_brakeCmd = (float)filterABS(filterBrakeSpeed(filterBPit(getBrake(BrakeCmd))));
   }

   if (car->_brakeCmd > 0.0f) {
    car->_accelCmd = 0.0f;
    //if (car->_brakeCmd > 0.8f && lastbrake > 0.8f && brake_collision && !avoiding && !correcting && fabs(car->_yaw_rate-car->_steerCmd*2) > 0.6)
     //align_hold = MAX(align_hold, currentsimtime + 5.0);

    if (0 && racetype == RM_TYPE_QUALIF && car->_speed_x < TargetSpeed + 3.0)
    {
     car->_brakeCmd = 0.0;
     car->_accelCmd = 0.2f;
    }
    else if (!fStuck && !fStuckForward && car->_brakeCmd > 0.5 && MAX(fabs(car->_yaw_rate), fabs(angle)*1.4) > 1.2 && (brake_collision || getCorrecting() || avoiding))
    {
     car->_brakeCmd = (float)MAX(0.2, car->_brakeCmd - (MAX(fabs(car->_yaw_rate), fabs(angle)*1.4) - 1.1));
    }
   }
   else if (!fStuck && !fStuckForward && car->_speed_x > 5.0)
   {
    BTAccelCmd = filterOverlap(filterTCL(filterTrk((BTAccelCmd)), 0));
    if ((((!getCorrecting() && !avoiding) || ((getCorrecting() || getAllowCorrecting()) && fabs(nextleft-car->_trkPos.seg->width*tLane[rl][nextdiv]) < 1.0)) && BTAccelCmd > car->_accelCmd) ||
        ((getCorrecting() || avoiding) && fabs(tRInverse[rl][nextdiv]) > WingRInverse && BTAccelCmd < car->_accelCmd))
     car->_accelCmd = (float)BTAccelCmd;
   }

   // reduce acceleration if seriously slipping sideways.
   double speed = Mag(car->_speed_Y, car->_speed_X);
   if ((avoiding || getCorrecting() || align_hold > currentsimtime + 3.0) && speed > getSpeed() + 1.0)
    car->_accelCmd = (float)MAX(0.0, MIN(car->_accelCmd, car->_accelCmd - (speed-getSpeed())/4));

   if (0 != last_flying)
   {
    if ((last_flying & FLYING_BACK) && FlyingCaution)
    {
     car->_accelCmd = MIN(car->_accelCmd, 0.7f);
     if ((last_flying & FLYING_FRONT))
      car->_accelCmd = (float)MIN(car->_accelCmd, 0.2);
    }

    if (nextleft > 0.0 && nextleft < car->_trkPos.seg->width && fabs(angle) < 0.4)
     flying_timer = currentsimtime;
   }

   if (car->_accelCmd > 0.0 && !pit->getInPit() && (fabs(angle) > 1.3 || fabs(car->_yaw_rate) > 2.0 || 
        (fabs(car->_steerCmd) >= 0.8 && (avoiding || getCorrecting() || align_hold > currentsimtime))))
   {
    car->_accelCmd = (float)MIN(car->_accelCmd, 0.5);
   }

   if (!pit->getInPit() && !brake_collision && currentsimtime > 3 && getSpeed() < 5.0 && car->_gearCmd == 1)
   {
    // make sure if we've spun out we actually get started again
    car->_brakeCmd = 0.0;
    car->_accelCmd = (float)MAX(car->_accelCmd, 0.8);
   }


   double lane2left = tLane[rl][thisdiv] * car->_trkPos.seg->width;
   if (!pit->getInPit() && align_hold > currentsimtime + 3.0 &&
       ((segtype != TR_STR && 
         ((segtype == TR_LFT && car->_trkPos.toLeft > lane2left + 3.0) ||
          (segtype == TR_RGT && car->_trkPos.toLeft < lane2left - 3.0))) ||
        (fabs((float) car->_gear) == 1 && fabs(car->_speed_x) < 15.0 &&
         (fabs(car->_steerCmd) > 0.7 || fabs(angle) > 0.8 || fabs(car->_yaw_rate) + fabs(angle)/2 > 1.8))))
   {
    car->_accelCmd = (float)filterTCL(car->_accelCmd, 1);
   }
//fprintf(stderr,"%s: %d%d bc%d a%.3f b%.3f A%.3f\n",car->_name,getCorrecting(),avoiding,brake_collision,car->_accelCmd,car->_brakeCmd,angle);fflush(stderr);

   if (!NoAccelRelax && (!pit->getInPit() || car->_speed_x > 35.0))
   {
    if (currentsimtime > 4.0 && car->_accelCmd > lastaccel)
    {
     if (car->_gearCmd > 1)
      RELAXATION(car->_accelCmd, lastaccel, 40.0f);
     else
      lastaccel = car->_accelCmd;
    }
    else if (car->_accelCmd < lastaccel)
    {
     double accel = car->_accelCmd;
     RELAXATION(lastaccel, accel, 40.0);
    }
   }

   car->_accelCmd = (float)filterTeam(car->_accelCmd);

#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"%s: %d brk=%.3f acc=%.3f coll=%.2f\n",car->_name,car->_lightCmd, car->_brakeCmd,car->_accelCmd,brake_collision,car->_wheelSlipSide(0),car->_wheelSlipSide(1), car->_speed_x,TargetSpeed);fflush(stderr);
   //slip += (car->_wheelSpinVel(i) * car->_wheelRadius(i));
#endif
   car->_clutchCmd = (float)getClutch();

   if (fabs(angle) > 1.6f && 0)
   {
    if (car->_steerCmd >= 0.0f)
     car->_steerCmd = MAX(car->_steerCmd, 0.5f);
    else
     car->_steerCmd = MIN(car->_steerCmd, -0.5f);
    if (avoiding)
     laststeer = car->_steerCmd = -car->_steerCmd;
   }
  }

  if (fabs(car->_speed_x) < 5.0 && car->_gearCmd < 0)
   car->_accelCmd = (float)MAX(car->_accelCmd, 0.75);

  //if (car->_accelCmd >= 0.5 && fabs(car->_steerCmd) > 0.2)
  // car->_steerCmd *= (1.0 + fabs(car->_steerCmd) * MIN(1.0, car->_accelCmd*0.75));
 }

 if (!brake_collision && car->_brakeCmd > 0.0 && car->_brakeCmd < 0.4)
 {
  // this is probably only useful for the chronically understeering clk, as
  // it pushes the rear wheels around at the same time as applying brakes.
  // with an oversteering car it'd probably be disastrous.
  car->_accelCmd = (float)MIN(car->_brakeCmd, MIN(0.25, fabs(car->_steerCmd) / 6));
 }

#ifdef DEBUGTHIS
fprintf(stderr,"done: accel=%.3f steer=%.3f\n",car->_accelCmd,car->_steerCmd);fflush(stderr);
#endif
 laststeer = car->_steerCmd;
 lastgear = car->_gearCmd;
 lastbrake = car->_brakeCmd;
 lastaccel = car->_accelCmd;
 prevleft = car->_trkPos.toLeft;
 old_avoiding = avoiding;
 lastyaw = car->_yaw;
 if (segtype != TR_STR)
  lastcornertype = segtype;

 int i;
 if (lastdiv != thisdiv && (thisdiv > lastdiv || (lastdiv > Divs - 100 && thisdiv < 100)))
 {
  for (i=9; i>0; i--)
  {
   lastX[i] = lastX[i-1];
   lastY[i] = lastY[i-1];
  }
 }
 lastX[0] = car->_pos_X;
 lastY[0] = car->_pos_Y;

 for (i=0; i<4; i++)
 {
  corner2[i].ax = corner1[i].ax;
  corner2[i].ay = corner1[i].ay;
  corner1[i].ax = car->_corner_x(i);
  corner1[i].ay = car->_corner_y(i);
 }
 lastdiv = thisdiv;
 lastLoffset = car->_trkPos.toLeft - getWidth()/2;
 lastRoffset = car->_trkPos.toRight - getWidth()/2;
 if (currentsimtime > 2.0)
  start_finished = 1;
 telemetry[carindex] = avoiding;
 correct_diff = car->_steerCmd - k1999steer;

 if (car->_laps != last_lap)
 {
  fuelperlap = MAX(fuelperlap, last_lapfuel - car->_fuel);
  last_lap = car->_laps;
  last_lapfuel = car->_fuel;
 }

 // a nice finishing touch
 if (s->_totLaps == car->_laps && car->_pos > 4)
 {
  if (car->_pos == 1)
   car->_lightCmd = RM_LIGHT_HEAD1;
  else
   car->_lightCmd = RM_LIGHT_HEAD2;
 }
}


// Set pitstop commands.
int Driver::pitCommand(tSituation *s)
{
 car->_pitRepair = strategy->pitRepair(car, s, opponents);
 car->_pitFuel = strategy->pitRefuel(car, s);
 // This should be the only place where the pit stop is set to false!
 pit->setPitstop(false);
 return ROB_PIT_IM; // return immediately.
}


// End of the current race.
void Driver::endRace(tSituation *s)
{
 // Nothing at the moment.
}


/***************************************************************************
 *
 * utility functions
 *
 ***************************************************************************/

double Driver::getLaneLeft(tCarElt *pcar, double distance, double Time)
{
 int SegId = pcar->_trkPos.seg->id;
 tTrackSeg *seg = pcar->_trkPos.seg;
 double dist = pcar->_trkPos.toStart;
 if (dist < 0)
  dist = 0;
 if (pcar->_trkPos.seg->type != TR_STR)
  dist *= pcar->_trkPos.seg->radius;
 int Index = tSegIndex[SegId] + int(dist / tElemLength[SegId]);
 Index = (Index + Divs - 5) % Divs;
 int Next;
 //static const double Time = 0.01;
 double X = pcar->_pos_X + pcar->_speed_X * Time / 2;
 double Y = pcar->_pos_Y + pcar->_speed_Y * Time / 2;

 while(1)
 {
  Next = (Index + 1) % Divs;
  if ((tx[rl][Next] - tx[rl][Index]) * (X - tx[rl][Next]) +
      (ty[rl][Next] - ty[rl][Index]) * (Y - ty[rl][Next]) < 0)
   break;
  Index = Next;
 }

 if (distance > DivLength)
  Index = (Index + (int) (distance / DivLength)) % Divs;

//fprintf(stderr,"GLL %s-%s: dist=%.3f Index=%d/%d toleft=%.3f (%.3f)\n",car->_name,pcar->_name,distance,thisdiv,Index,tLane[rl][Index]*seg->width,nextleft);

 return tLane[rl][Index] * seg->width;
}

double Driver::getCarRInverse(tCarElt *pcar, double distance)
{
 if (pcar->_speed_x < 7.0)
  return 0.0;

 int SegId = pcar->_trkPos.seg->id;
 //tTrackSeg *seg = pcar->_trkPos.seg;
 double dist = pcar->_trkPos.toStart;
 if (dist < 0)
  dist = 0;
 if (pcar->_trkPos.seg->type != TR_STR)
  dist *= pcar->_trkPos.seg->radius;
 int Index = tSegIndex[SegId] + int(dist / tElemLength[SegId]);
 Index = (Index + Divs - 5) % Divs;
 int Next;
 static const double Time = 0.01;
 double X = pcar->_pos_X + pcar->_speed_X * Time / 2;
 double Y = pcar->_pos_Y + pcar->_speed_Y * Time / 2;

 while(1)
 {
  Next = (Index + 1) % Divs;
  if ((tx[rl][Next] - tx[rl][Index]) * (X - tx[rl][Next]) +
      (ty[rl][Next] - ty[rl][Index]) * (Y - ty[rl][Next]) < 0)
   break;
  Index = Next;
 }

 if (distance > DivLength)
  Index = (Index + (int) (distance / DivLength)) % Divs;

//fprintf(stderr,"GLL %s-%s: dist=%.3f Index=%d/%d toleft=%.3f (%.3f)\n",car->_name,pcar->_name,distance,thisdiv,Index,tLane[rl][Index]*seg->width,nextleft);

 return tRInverse[rl][Index];
}

int Driver::getCarDiv(tCarElt *pcar, double distance)
{
 int SegId = pcar->_trkPos.seg->id;
 //tTrackSeg *seg = pcar->_trkPos.seg;
 double dist = pcar->_trkPos.toStart;
 if (dist < 0)
  dist = 0;
 if (pcar->_trkPos.seg->type != TR_STR)
  dist *= pcar->_trkPos.seg->radius;
 int Index = tSegIndex[SegId] + int(dist / tElemLength[SegId]);
 Index = (Index + Divs - 5) % Divs;
 int Next;
 static const double Time = 0.01;
 double X = pcar->_pos_X + pcar->_speed_X * Time / 2;
 double Y = pcar->_pos_Y + pcar->_speed_Y * Time / 2;

 while(1)
 {
  Next = (Index + 1) % Divs;
  if ((tx[rl][Next] - tx[rl][Index]) * (X - tx[rl][Next]) +
      (ty[rl][Next] - ty[rl][Index]) * (Y - ty[rl][Next]) < 0)
   break;
  Index = Next;
 }

 if (distance > DivLength)
  Index = (Index + (int) (distance / DivLength)) % Divs;

//fprintf(stderr,"GLL %s-%s: dist=%.3f Index=%d/%d toleft=%.3f (%.3f)\n",car->_name,pcar->_name,distance,thisdiv,Index,tLane[rl][Index]*seg->width,nextleft);

 return Index;
}

double Driver::AvoidBrake(double speed)
{
 //float thisspeed = speed;
 double closestdspd = 1000.0, closestdist = 1000.0, closestti = 1000.0;
 double oldspeed = speed;
 int closest = -1, i;
 int nindex = (thisdiv+1) % Divs;
 tTrackSeg *seg = car->_trkPos.seg;
 double lane2left = tLane[rl][thisdiv] * seg->width;
 if (fabs(angle) > 1.4)
  return speed;

 for (i = 0; i < opponents->getNOpponents(); i++) 
 {
  if (opponent[i].car == car) continue;
  tCarElt *ocar = opponent[i].car;

  int team = opponent[i].getTeam();
  int coll = (opponent[i].getState() & OPP_COLL);
  if (ocar->_state >= RM_CAR_STATE_PIT) continue;

  double dist = opponent[i].getBrakeDistance();
  tTrackSeg *oseg = ocar->_trkPos.seg;
  tTrackSeg *tseg = oseg;
  int tsegtype = SEGTYPE(tseg);
  if (dist < 7.0 && (oseg->type == TR_STR || (seg->type != TR_STR && seg->radius < seg->radius)))
   tseg = seg;
  double trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
  double oppangle = (trackangle - ocar->_yaw);
  NORM_PI_PI(oppangle);
  double sidedistance = fabs(nextleft - opponent[i].getNextLeft()) - (getWidth()/2 + opponent[i].getWidth()/2);
  double odist = dist;
  if ((team & TEAM_CONTROL) && (avoiding || getCorrecting() || !alone))
   team = TEAM_KAOS;
  double dx = opponent[i].car->_speed_X - car->_speed_X;
  double dy = opponent[i].car->_speed_Y - car->_speed_Y;
  double accel = MAX(-8.0, MIN(2.0, 
     (car->_accel_x > 0.0 && !(opponent[i].getState() & OPP_COLL_URGENT) && dist > 1.5 
      ? car->_accel_x / ((team & TEAM_CONTROL) ? 0.5 : 1.5) 
      : (car->_accel_x < -2.0 ? (car->_accel_x+2.0)/((team & TEAM_CONTROL)
        ? 20.5 : 1.5) : 0.0))));
  accel = 0;
  double dspd = car->_speed_x + (car->_speed_X/getSpeed()*dx + car->_speed_Y/getSpeed()*dy) + MIN(0.0, opponent[i].car->_accel_x/2);
  if (!(team & TEAM_CONTROL) || avoiding || getCorrecting())
   dspd += MIN(0.0, opponent[i].car->_accel_x/2);
  int pull_back = 0;

  if (!coll && dist > MIN(150.0, (TargetSpeed-dspd) * 8))
  {
   continue;
  }

  double ospeed = opponent[i].getSpeed() + (accel/5);//(dist > 2.0 ? ocar->_accel_x/(team==TEAM_CONTROL ? 8 : 2) : 0.0);
  //double diffspeed = (getSpeed() - ospeed);

  if (!coll && opponent[i].getDistance() < 0.0)
  {
   continue;
  }

  if (!coll && MIN(ocar->_trkPos.toRight, ocar->_trkPos.toLeft) < -2.0 && MIN(car->_trkPos.toRight, car->_trkPos.toLeft) > 1.0)
  {
   continue;
  }

  double lowestrad = 1000.0;
  int start = (tseg == oseg ? (int) (dist/DivLength) : 0);
  for (int j=start; j<MAX(4, (int) (car->_speed_x/DivLength)); j++)
  {
   int div = (nextdiv+j) % Divs;
   tTrackSeg *lseg = tSegment[tDivSeg[div]];
   if (SEGTYPE(lseg) != TR_STR)
    lowestrad = MIN(lseg->radius, lowestrad);
  }

  if (!coll && fabs(opponent[i].getNextLeft() - nextleft) > 10.0 && lowestrad >= 25.0)
  {
   continue;
  }

  double n2left = car->_trkPos.toLeft;
  if (/*lastaccel >= 0.8 &&*/ dist < 4.0 && 
      ((n2left < lane2left && nextleft < car->_trkPos.toLeft-0.1 && n2left < (seg->type == TR_LFT ? 1.0 : 2.5)) ||
       (n2left > lane2left && nextleft > car->_trkPos.toLeft+0.1 && n2left > seg->width-(seg->type == TR_RGT ? 1.0 : 2.5))))
  {
   // running off the track while closely following someone - slow down.
   dspd = MIN(dspd, MIN(ospeed, MIN(TargetSpeed, car->_speed_x))) - 1.0;
   coll = 4;
  }

  if (coll & OPP_COLL_URGENT)
   dspd -= 2.0;
#if 0
  if (diffspeed > 0.0 && dist > 3.0 && diffspeed < dist*(team==TEAM_CONTROL?0.8:0.55))
   dspd += dist*(team==TEAM_CONTROL?0.8:0.55)-diffspeed;
#endif

  double safedist = 0.0;
#if 0
  if (lowestrad <= 80.0)
   safedist += (90.0 - tsegtype) / 80.0;//40.0;
  else if (tMaxSpeed[thisdiv] > tMaxSpeed[nextdiv] + 0.3)
   safedist += fabs(tMaxSpeed[thisdiv] - tMaxSpeed[nextdiv]) / 2;
  if (car->_speed_x > 60.0 && !coll && dspd < car->_speed_x - 5.0)
   safedist += ((car->_speed_x-55.0) / 10.0) * (car->_speed_x - dspd);
#endif

  int cornercoll = 0, inside = 0;
  int closing = opponent[i].getClosing();

  double width = MAX(ocar->_dimension_y, opponent[i].getWidth());
  double brkfact = (!(team & TEAM_CONTROL) && ocar->_pos > car->_pos) ? 2 : (!(team & TEAM_CONTROL) ? 1.5 : 1.0);
  double oppmax;
  double oppmin;
  if (lowestrad < 1000.0)
  {
   oppmax = opponent[i].getNextLeft() + width * 0.5 + car->_dimension_y/2 + 1.0 + MAX(0.0, (140.0-lowestrad) / 70);
   oppmin = opponent[i].getNextLeft() - (width * 0.5 + car->_dimension_y/2 + 1.0 + MAX(0.0, (140.0-lowestrad) / 70));
  }
  else
  {
   oppmax = opponent[i].getNextLeft() + width * 0.5 + car->_dimension_y/2 + 1.0;
   oppmin = opponent[i].getNextLeft() - (width * 0.5 + car->_dimension_y/2 + 1.0);
  }

  double tradius = MIN(lowestrad, tseg->radius);

#if 1
  if (fabs(nextleft-ocar->_trkPos.toLeft) > 1.5 && closing == CLOSING_FAST &&  
      (//closing_fast ||
       // checking for being trapped inside the other car on the corner
       (0 != (inside = (
        (tsegtype == TR_LFT && nextleft < ocar->_trkPos.toLeft && fabs(nextleft-ocar->_trkPos.toLeft) < MAX(3.0, (90.0-tradius)/9) - (ocar->_trkPos.toLeft-4.0)/4) ||
        (tsegtype == TR_RGT && nextleft > ocar->_trkPos.toLeft && fabs(nextleft-ocar->_trkPos.toLeft) < MAX(3.0, (90.0-tradius)/9) - (ocar->_trkPos.toRight-4.0)/4)))) ||
       // checking for other car being near rl and we're close enough for it to be trouble
       (tsegtype == TR_LFT && tradius <= 80.0 && ocar->_trkPos.toLeft < 6.0 && fabs(nextleft-ocar->_trkPos.toLeft) < (85.0-tradius)*(85.0-tradius)/600) ||
       (tsegtype == TR_RGT && tradius <= 80.0 && ocar->_trkPos.toRight < 6.0 && fabs(nextleft-ocar->_trkPos.toLeft) < (85.0-tradius)*(85.0-tradius)/600)))
  {
   cornercoll = 1;
  }
#endif

  if (coll && cornercoll)
  {
#if 0
   if (dist > 2.0 && dspd < TargetSpeed && !inside)
   {
    // slow down less if we're further away
    dspd += (dist) * ((100.0-dist)/150);
   }
#endif
   if (dist > -1.0 && (dist < 0.3 || (inside && dist < 1.2)))
   {
    // too close - back off!
    double speed = MIN(dspd, MIN(TargetSpeed, MIN(ospeed, car->_speed_x)));
    if (!inside && (segtype == TR_STR || (segtype == TR_LFT && car->_trkPos.toLeft > ocar->_trkPos.toLeft+1.0) || (segtype == TR_RGT && car->_trkPos.toLeft < ocar->_trkPos.toLeft-1.0)))
     dspd = speed - (1.0-dist)*3;
    else
     dspd = speed - (((inside ? 2.0 : 1.0)-dist) + (140.0-lowestrad) / 30)*0.7;
    pull_back = 1;
    safedist = dist+1;
   }
  }

  if (tsegtype != TR_STR && (tradius <= 30.0 || (tradius <= 60.0 && !(team & TEAM_CONTROL) && cornercoll)) 
      && dist < 15.0 && sidedistance < MIN(12.0, MAX(4.0, ospeed)))
  {
#if 0
   if (dist < (team == TEAM_CONTROL ? 2.0 : 3.0))
    dspd = MIN(dspd, ospeed);
   else if (team != TEAM_CONTROL)
   {
    dspd = MIN(dspd, ospeed + (dist-3.0)*0.3);
    if (tradius <= 30.0)
     dspd = MIN(dspd, ospeed + (dist-3.0)*0.1);
   }

   if (dist < 0.6 || cornercoll)
    dspd = dspd * 0.9;
#endif
   if (dist < 2.0 || !(team & TEAM_CONTROL))
    coll = 3;
  }

  if (0 && car->_speed_x > 5.0 && tradius <= 120.0 && tseg->type != TR_STR && (cornercoll || tradius <= 40.0))//((tseg->type == TR_LFT && ocar->_trkPos.toMiddle > 0.0) || (tseg->type == TR_RGT && ocar->_trkPos.toMiddle < 0.0)))
  {
   dist = MAX(0.1, dist - (fabs(ocar->_trkPos.toMiddle)/tseg->width * (130.0-tradius) / 50));
   if (tradius <= 30.0)
   {
    dist -= (40.0-tradius) / 12.0;
    dspd -= 1.0 * brkfact;
   }
   else if ((avoiding || !(team & TEAM_CONTROL)) && tradius <= 60.0 && tMaxSpeed[nindex] < MIN(tSpeed[nindex], car->_speed_x)+0.3)
    dspd -= ((odist-dist)/4) * brkfact;
  }

  double radfactor = ((team & TEAM_CONTROL) ? 0.5 : 0.6);
  if (cornercoll && car->_speed_x > 5.0 && dist < safedist && getSpeed() > (tseg->type == TR_STR ? 30.0 : tradius * radfactor))
  {
   coll = 2;
  }

  if ((avoiding & AVOID_TEAM) && opponent[i].getDistance() > 0.1 && opponent[i].getDistance() <= ocar->_dimension_x && seg->radius <= 100.0 && sidedistance < 6.0)
  {
   coll = 1;
   pull_back = 1;
   //if (currentsimtime < 20.0)
   // pull_back = 2;
   if (((seg->type == TR_LFT && ocar->_trkPos.toLeft < car->_trkPos.toLeft) ||
       (seg->type == TR_RGT && ocar->_trkPos.toLeft > car->_trkPos.toLeft)))
    //dspd = MIN(dspd, MIN(getSpeed()+1.0, ospeed) - MAX(0.2, (3.0 - (sidedistance/2))));//pull_back);
    dspd = MIN(dspd, ospeed - 1.0);
   else
    dspd = MIN(dspd, ospeed - 0.5);
  }

  if (!coll && !pit->getInPit())/* && 
      //(team != TEAM_CONTROL || currentsimtime > 7.0 || ocar->_speed_x > 5) &&
       (sidedistance > 0.5 || 
        (currentsimtime < 3.0) || 
        ((car->_speed_x < opponent[i].car->_speed_x && !(opponent[i].getState() & OPP_SIDE))) || 
        (dist > (tsegtype == TR_STR && tMaxSpeed[nextdiv] > tSpeed[thisdiv] - 0.8 ? 1.0 : 1.6))))*/
   continue;

#if 0
  if (dist < safedist && team != TEAM_CONTROL && 
      (cornercoll || 
       (fabs(nextleft-ocar->_trkPos.toLeft) < 90.0-tradius/5) && 
        fabs(car->_trkPos.toLeft-ocar->_trkPos.toLeft) > fabs(nextleft-opponent[i].getNextLeft()) &&
        ((tseg->type == TR_LFT && ocar->_trkPos.toLeft < 10.0) || 
         (tseg->type == TR_RGT && ocar->_trkPos.toRight < 10.0))))
   dspd = MAX(ocar->_speed_x, dspd - (90.0 - tradius) / 40.0);
#endif

  int ndiv1 = (nextdiv+1) % Divs;
  int ndiv2 = (nextdiv+2) % Divs;
  int ndiv3 = (nextdiv+3) % Divs;
  if (dist < 2.0 && dist > -2.0)
  {
   if (FlyingCaution && (Flying[thisdiv]+Flying[nextdiv]+Flying[ndiv1]+Flying[ndiv2]+Flying[ndiv3]))
   {
    dspd -= FlyingCaution * 70.0;
    pull_back = 2;
   }
   else
   {
   // if ((tSegment[tDivSeg[thisdiv]]->type != TR_STR && tSegment[tDivSeg[thisdiv]]->radius <= 25.0) ||
   //         (tSegment[tDivSeg[nextdiv]]->type != TR_STR && tSegment[tDivSeg[nextdiv]]->radius <= 25.0) ||
   //         (tSegment[tDivSeg[ndiv2]]->type != TR_STR && tSegment[tDivSeg[ndiv2]]->radius <= 25.0) ||
   //         (tSegment[tDivSeg[ndiv3]]->type != TR_STR && tSegment[tDivSeg[ndiv3]]->radius <= 25.0))
    if (tradius <= 25.0)
    {
     //dspd -= 2.0 * brkfact;
    }
   }
   //else if ((car->_trkPos.seg->type != TR_STR && tMaxSpeed[thisdiv] > tMaxSpeed[nextdiv] + 0.6 && tMaxSpeed[thisdiv] > 50.0 && car->_trkPos.seg->radius <= 110.0))
   // dspd -= 2.0;
  }

  /*
  if (opponent[i].getState() & OPP_COLL_URGENT)
  {
   pull_back = 1;
   dspd -= (team & TEAM_CONTROL) ? 1.0 : 2.0;
  }
  */
#if 0
  if (team != TEAM_CONTROL && (dist < 3.0 || (dist<10.0 && diffspeed > 5.0)) && (diffspeed > dist/2 || tMaxSpeed[thisdiv]-tMaxSpeed[nindex] > 0.3))
   dspd -= (MAX(0.5, 3.0 - dist) / 8.0) * brkfact;
  else if (!(opponent[i].getState() & OPP_COLL_URGENT) && dist > 3.0 && (getSpeed()-ospeed < dist/2))
   dspd += MAX(0.0, (TargetSpeed-dspd)*(dist+3.0)/100.0);
  if (odist < (team == TEAM_CONTROL ? 0.3 : 1.1))
   dspd = MIN(dspd, ospeed*0.95);
#endif
  double t_impact = opponent[i].getMaxTimeImpact();
  if (!pull_back)
  {
   if (dist <= 0.5)
    dspd = MIN(dspd, ospeed);
   else
    dspd = MAX(dspd, ospeed);
   if (t_impact > 3.0)
    dspd += MIN((t_impact-3.0) * 2, (car->_speed_x-dspd) * MIN(1.0, (t_impact-3.0) / 10));
   if (!coll && t_impact > 1.0 && dist > 3.0)
    dspd = MAX(dspd, opponent[i].getSpeed());
    //dspd += (opponent[i].getTimeImpact()-3.0) * 0.4;
  }
  else
   dspd = MAX(dspd, ospeed-3.0);

  if (t_impact > 0.8 && !pull_back && car->_speed_x > dspd)// < 2)
  {
   dspd += (t_impact-0.5) * 6 + dist/6;//6;
#if 0
   double brakelimit = (1.0 - MAX(fabs(angle), MAX(fabs(laststeer), fabs(car->_yaw_rate)))*1.1);
   if (car->_speed_x < 45.0)
    brakelimit += (45.0-car->_speed_x)/150.0;
   dspd = MAX(dspd, car->_speed_x-brakelimit);
#endif
  }

  if (dspd < closestdspd && (pull_back || dist > -2.0) && MAX(car->_speed_x, getSpeed()) >= dspd)
  {
//fprintf(stderr,"BRAKE: %s->%s TS=%.3f sp=%.3f osp=%.3f dspd=%.3f pb=%d",car->_name,ocar->_name,TargetSpeed,car->_speed_x,ocar->_speed_x,dspd,pull_back);
   closest = i;
   closestdspd = dspd;
   closestdist = dist;
   closestti = t_impact;
  }
 }
 if (closest != -1 && (closestdspd < TargetSpeed || (MIN(closestdist, closestti) < 1.0 && closestdspd < car->_speed_x)))
 {
  double yr = car->_yaw_rate * 0.25;//0.1;
  if (fabs(angle) > 0.4 && closestdist > 2.0)
   closestdspd += MIN(car->_speed_x, speed-closestdspd) * MIN(1.0, fabs(-angle + yr) / 20);
  brake_collision = MAX(0.08, MIN(closestti, closestdist*1.5));
  if (speed > closestdspd || closestdist < 0.8)
  {
   if (closestdist < 0.8)
   {
    speed = MIN(speed, MIN(closestdspd - (0.9-closestdist)*4, TargetSpeed - (0.9-closestdist) * 4));
//fprintf(stderr," speedA=%.3f\n",speed);
   }
   else if (tMaxSpeed[thisdiv] > tMaxSpeed[nindex] - 0.5)
   {
    speed = MIN(speed, closestdspd);
//fprintf(stderr," speedB=%.3f\n",speed);
   }
   else
   {
    speed = closestdspd;
//fprintf(stderr," speedC=%.3f\n",speed);
   }
  }
  else if (tMaxSpeed[thisdiv] > tMaxSpeed[nindex] - 0.5)
  {
   speed = MIN(speed, closestdspd);
//fprintf(stderr," speedD=%.3f\n",speed);
  }
  else
  {
   speed = closestdspd;
//fprintf(stderr," speedE=%.3f\n",speed);
  }
 }

 if (oldspeed > speed && !brake_collision)
  brake_collision = 10.0;

 if (brake_collision)
  bc_timer = currentsimtime;

 return speed;
}



// Compute the length to the end of the segment.
double Driver::getDistToSegEnd()
{
 if (car->_trkPos.seg->type == TR_STR) {
  return car->_trkPos.seg->length - car->_trkPos.toStart;
 } else {
  return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
 }
}

// Compute the allowed speed on a segment.
double Driver::getAllowedSpeed(tTrackSeg *segment, int accel)
{
 double mu = segment->surface->kFriction*TIREMU*MU_FACTOR;
 double r = tRadius[segment->id];
 int avoid = ((avoiding && !(avoiding & AVOID_ALIGNED)) || getCorrecting() || align_hold > currentsimtime);
 double br = learn->getBaseRadius(segment, avoid);
 double dr = (avoid ? learn->getRadius(segment, avoid) : 0.0);
 double lane2left = tLane[rl][thisdiv]*segment->width;
 double pleft = prevleft;
 if (SEGTYPE(segment) == TR_LFT)
  pleft -= (130.0 - segment->radius) / 1400.0;
 else if (SEGTYPE(segment) == TR_RGT)
  pleft += (130.0 - segment->radius) / 1400.0;
 double nleft = car->_trkPos.toLeft + (car->_trkPos.toLeft - pleft) * 20;
 int fastspeed = 1;
 double sideslip = (car->_wheelSlipSide(0) + car->_wheelSlipSide(1) + car->_wheelSlipSide(2) + car->_wheelSlipSide(3)) / 4;
 if (avoid && (getCorrecting() || align_hold > currentsimtime))
 {
  switch (segtype)
  {
   case TR_STR:
    if (car->_speed_x < TargetSpeed - fabs(sideslip)/2)
     avoid = 0;
    break;
   case TR_RGT:
    if (car->_trkPos.toRight < segment->width*0.6
        && nleft-(car->_trkPos.toLeft-nleft)*15 > tAvoidLeft[nextdiv]
        && car->_speed_x < TargetSpeed - (2.0 + fabs(sideslip)/2))
     avoid = 0;
    break;
   case TR_LFT:
    if (car->_trkPos.toLeft < segment->width*0.6 
        && nleft+(nleft-car->_trkPos.toLeft)*15 < segment->width - tAvoidRight[nextdiv]
        && car->_speed_x < TargetSpeed - (2.0 + fabs(sideslip)/2))
     avoid = 0;
    break;
  }
 }

 if (avoid)
 {
  fastspeed = (fabs(car->_trkPos.toLeft-lane2left)<1.0 || 
               (segment->type == TR_LFT && car->_trkPos.toLeft < lane2left && car->_trkPos.toLeft > 1.0) ||
               (segment->type == TR_RGT && car->_trkPos.toLeft < lane2left && car->_trkPos.toRight > 1.0));
 }

 double speed = sqrt((mu*G*r)/(1.0 - MIN(1.0, r*CA*mu/mass)));// * (fastspeed ? 1.3 : 1.1);

 if (avoid && dr < 0.0)
 {
  if (dr > 0.0)
   dr = MIN(dr, Learning * 7);
  else if (dr < 0.0 && Learning)
   dr = MAX(dr, -(TargetSpeed * (0.1 + Learning/8)));
 }

 if (accel && TargetSpeed > speed)
 {
  r = tRadius[segment->id];
  learn->incrBaseRadius(segment, (TargetSpeed - speed)*2.5);
  br = learn->getBaseRadius(segment, avoid);
  r += MAX(-r/2, br+dr/2);
  speed = sqrt((mu*G*r)/(1.0 - MIN(1.0, r*CA*mu/mass)));// * (fastspeed ? 1.3 : 1.1);
 }
 if (dr >= 0.0)
  speed = MAX(speed, TargetSpeed);

 speedfactor = 1.0;
 if (!avoid &&
     ((segtype == TR_STR && ((lastcornertype == TR_RGT && nleft >= lane2left) || (lastcornertype == TR_LFT && nleft <= lane2left))) ||
      (segtype == TR_LFT && (nleft <= lane2left + 1.0 && (nleft < lane2left+0.5 || nleft > tAvoidLeft[nextdiv]))) ||
      (segtype == TR_RGT && (nleft >= lane2left - 1.0 && (nleft > lane2left-0.5 || segment->width-nleft > tAvoidRight[nextdiv])))))
 {
//if (accel)
//fprintf(stderr,"%s: here! %d %d %d %d\n",car->_name,!(avoiding || getCorrecting()),(segtype == TR_STR && ((lastcornertype == TR_RGT && nleft >= lane2left) || (lastcornertype == TR_LFT && nleft <= lane2left))),(segtype == TR_LFT && nleft <= lane2left + 1.0),(segtype == TR_RGT && nleft >= lane2left - 1.0));
  speed *= (fastspeed ? 1.3 : 1.1);
 }
 else if (0 && avoid)
 {
  int dir = TR_LFT;
  if (segtype == TR_LFT || (segtype == TR_STR && lastcornertype == TR_LFT))
   dir = TR_RGT;

  if ((dir == TR_LFT && nleft < car->_trkPos.toLeft && car->_speed_x >= TargetSpeed-10.0 && (angle < 0.1 || nleft < tAvoidLeft[nextdiv]+6.0)) || 
      (dir == TR_RGT && nleft > car->_trkPos.toLeft && car->_speed_x >= TargetSpeed-10.0 && (angle > -0.1 || segment->width-nleft < tAvoidRight[nextdiv]+6.0)))
  {
   if (dir == TR_LFT)
   {
    speedfactor = ((1.0 - (nleft/lane2left)) * nleft) * 0.1;
    if (segment->radius < 400.0 && segment->type != TR_STR)
     speedfactor *= 1.0 + pow((400.0-segment->radius)/290.0,5)/(MAX(1.0,segment->width-7.0));
    speedfactor = 1.0 - (speedfactor * speedfactor);
   }
   else
   {
    speedfactor = ((1.0 - ((segment->width-nleft)/(segment->width-lane2left))) * (segment->width-nleft)) * 0.1;
    if (segment->radius < 400.0 && segment->type != TR_STR)
     speedfactor *= 1.0 + pow((400.0-segment->radius)/290.0,5)/(MAX(1.0,segment->width-7.0));
    speedfactor = 1.0 - (speedfactor * speedfactor);
   }
   if (dir == TR_RGT)
    speedfactor -= angle*0.15;
   else if (dir == TR_LFT)
    speedfactor += angle*0.15;
   speedfactor = MIN(1.0, MAX(0.7, speedfactor));
   if (speed >= TargetSpeed)
    speed *= speedfactor;
   TargetSpeed *= speedfactor;
  }
 }

 if (!avoid && !accel)
 {
  return speed;
 }


 useBTSpeed = 0;
 int close_enough = 0;
 double tlane2left = tLane[rl][thisdiv] * track->width;
 double nextright = segment->width - nleft;
 lane2left = tLane[rl][(thisdiv+1)%Divs] * track->width;

 int danger = (fabs(sideslip) > 15.0 || 
               (segtype == TR_LFT && nleft > segment->width - MAX(3.0, 3.0 + (100.0-segment->radius)/15)) ||
               (segtype == TR_RGT && nleft < MAX(3.0, (100.0-segment->radius)/15)) ||
               (nleft < MAX(car->_dimension_y*0.7, tAvoidLeft[nextdiv]) && nleft < tlane2left) ||
               (nextright < MAX(car->_dimension_y*0.7, tAvoidRight[nextdiv]) && nleft > tlane2left));
 double boost = (!avoid ? BTBoost : (!danger ? AvoidBTBoost - tAvoidBoost[nextdiv] : (AvoidBTBoost-0.1)-tAvoidBoost[nextdiv]));
 if (avoid && fabs(sideslip) > 10)
  boost -= fabs(sideslip)/100;
 speed *= boost;
 if (!avoid)// || boost < 1.0)
  TargetSpeed *= boost;
 else
  speed = TargetSpeed;

 if (!accel)
 {
  if (avoid)
   return MIN(speed, TargetSpeed);
  else
   return speed;
 }

 speed += dr;
 TargetSpeed += dr;

 if (((getCorrecting() || (avoiding && getAllowCorrecting() && !(avoiding & AVOID_ALIGNED))) && dr >= 0.0 && 
      fabs(angle) < 0.4 && fabs(laststeer) < 0.4 &&
      fabs(car->_trkPos.toLeft - lane2left) < 1.0 &&
      fabs(nleft - lane2left) < 1.0 &&
      fabs(nleft - car->_trkPos.toLeft) < fabs(lane2left-tlane2left)*1.5 &&
      fabs(laststeer - lastksteer) < 0.1))
  close_enough = 1;

 if (close_enough || !avoid)
  speed = MAX(speed, TargetSpeed);

 if (dr < 0.0 || speedfactor < 1.0 || (avoid && !close_enough))
 {
  if ((speed < TargetSpeed && dr <= 0.0) || speedfactor < 1.0)
   useBTSpeed = 1;
  if (speedfactor < 1.0 && MIN(speed, TargetSpeed) < car->_speed_x)
  {
   return MAX(MIN(speed, TargetSpeed), car->_speed_x * speedfactor);
  }
  return MIN(speed, TargetSpeed);
 }
 else
 {
  speed = MIN(500.0, speed);

  if (TargetSpeed < speed && !brake_collision)
   useBTSpeed = 2;

  if (FlyingCaution && (Flying[thisdiv] || Flying[nextdiv]))
   return TargetSpeed;
  return MAX(speed, TargetSpeed);
 }
}

// slows us down to allow a team-member to catch up
double Driver::filterTeam(double accel)
{
 if (accel <= 0.9 || avoiding || getCorrecting() || NoTeamWaiting) return accel;

 double minaccel = accel;
 double closest = -10000.0;
 int i;

 // first filter for following closely
 for (i = 0; i < opponents->getNOpponents(); i++)
 {
  if (opponent[i].car == car) continue;
  if ((opponent[i].getTeam() != TEAM_CONTROL)) continue;

  if (opponent[i].getDistance() - opponent[i].car->_dimension_x > 3.0 || opponent[i].getDistance() < car->_dimension_x)
   break;

  minaccel = accel = MIN(accel, opponent[i].car->_accelCmd);
 }
 
 // now filter to wait for catching up
 for (i = 0; i < opponents->getNOpponents(); i++)
 {
  if (opponent[i].car == car) continue;
  if (opponent[i].getTeam() & TEAM_CONTROL) continue;

  if (opponent[i].getDistance() < 0.0 && opponent[i].getDistance() > closest)
   closest = opponent[i].getDistance();

  if (opponent[i].car->_pos < car->_pos)
  {
   if (opponent[i].getDistance() < -150.0)
    return accel;
  }

  if (opponent[i].car->_pos >= car->_pos + 2 && 
      opponent[i].car->_laps == car->_laps &&
      opponent[i].getDistance() > -(car->_speed_x*2) &&
      opponent[i].getDistance() < 0.0)
  {
   return accel;
  }
 }

 for (i = 0; i < opponents->getNOpponents(); i++) 
 {
  if (opponent[i].car->_state >= RM_CAR_STATE_PIT) continue;
  if (opponent[i].car == car) continue;
  if (!(opponent[i].getTeam() & TEAM_CONTROL))
   continue;
  if (opponent[i].getDistance() > -25.0)
   continue;

  double time_behind = fabs(opponent[i].getDistance()) / opponent[i].car->_speed_x;

  if ((opponent[i].getTeam() & TEAM_CONTROL) && 
      opponent[i].car->_laps >= car->_laps &&
      opponent[i].car->_dammage < car->_dammage + 2000 &&
      ((time_behind <= TeamWaitTime && time_behind > 0.4) ||
       (opponent[i].getDistance() < 0.0 && opponent[i].getDistance() > -(car->_speed_x * TeamWaitTime))) &&
      opponent[i].getDistance() > closest &&
      opponent[i].getDistance() < -25.0)
  {
   return MIN(accel, 0.9);
  }
 }


 return minaccel;
}

double Driver::getAvoidSpeed(double toLeft)
{
 tTrackSeg *seg = car->_trkPos.seg;

 double speed1 = MIN(TargetSpeed + 2.0, tSpeed[nextdiv]);
 double tlane2left = tLane[LINE_RL][thisdiv] * seg->width;
 double nlane2left = tLane[LINE_RL][nextdiv] * seg->width;
 double mlane2left = tLane[LINE_MID][nextdiv] * seg->width;

 int side = seg->type;
 if (side == TR_STR)
 {
  if (tRInverse[LINE_RL][nextdiv] < -0.002)
   side = TR_RGT;
  else if (tRInverse[LINE_RL][nextdiv] > 0.002)
   side = TR_LFT;
 }
 
 // if outside the raceline or significantly inside approaching a corner, 
 // reduce speed as required.
 if (side != TR_STR &&
     (tSpeed[nextdiv] < tSpeed[thisdiv] ||
      fabs(tRInverse[LINE_MID][nextdiv]) > WingRInverse))
 {
  double dirdiverge = fabs((car->_trkPos.toLeft-nextleft) - (tlane2left - nlane2left)) * 2;
  if ((side == TR_LFT && (toLeft >= nlane2left+3.0 || 
       ((tlane2left > nlane2left || tSpeed[nextdiv] < tSpeed[thisdiv]) &&
        toLeft <= nlane2left - 4.0))) ||
      (side == TR_RGT && (toLeft <= nlane2left-3.0 || 
       ((tlane2left < nlane2left || tSpeed[nextdiv] < tSpeed[thisdiv]) &&
        toLeft >= nlane2left + 4.0))) ||
      (dirdiverge > 0.4))
  {
   double factor = 1.0;
   if ((side == TR_LFT && toLeft > track->width*0.5 && toLeft > tLane[LINE_RL][nextdiv]+1.0) ||
       (side == TR_RGT && toLeft < track->width*0.5 && toLeft < tLane[LINE_RL][nextdiv]-1.0))
   {
    factor = MAX(0.7, MIN(1.0, (side == TR_LFT ? (1.0 - toLeft/seg->width)/2 : (toLeft/seg->width)/2)));
   }
   //speed1 = MIN(speed1, sqrt(((TireAccel1-0.5) * tFriction[nextdiv] * factor) / (fabs(tRInverse[LINE_MID][nextdiv]) - WingRInverse)));
   double ri = fabs(tRInverse[LINE_RL][nextdiv]) * 1.5;
   speed1 = TargetSpeed - (TargetSpeed * ri) / (factor*factor);

   if (speed1 < TargetSpeed && (getAllowCorrecting() || getCorrecting()))
    speed1 = (speed1 + TargetSpeed) / 2;
#ifdef SPEED_DEBUG_MSG
fprintf(stderr,"%s: TS=%.3f (%.3f) spd1=%.3f -> ",car->_name,TargetSpeed,tSpeed[nextdiv],speed1);
#endif
   if (side == TR_LFT)
    speed1 -= tRSlowSpeed[nextdiv];
   else
    speed1 -= tLSlowSpeed[nextdiv];

   if ((side == TR_LFT && toLeft < nlane2left && tlane2left < nlane2left) ||
       (side == TR_RGT && toLeft > nlane2left && tlane2left > nlane2left))
   {
    //speed1 *= 1.0 + fabs(toLeft - nlane2left) / 10;
    speed1 = TargetSpeed;
   }

#ifdef SPEED_DEBUG_MSG
fprintf(stderr,"%.3f\n",speed1);fflush(stderr);
#endif
  }
 }

 double speed2 = speed1;

 if (!getAllowCorrecting() && !getCorrecting())
 {
  // find tightest curves looking ahead according to speed
  double rInverse = fabs(tRInverse[LINE_MID][nextdiv]);
  //double startInverse = rInverse;
  int maxinv = nextdiv;

  for (int i=0; i<(int) (car->_speed_x / 3); i++)
  {
   int Nnextdiv = (nextdiv + i) % Divs;
   double thisRInverse = fabs(tRInverse[LINE_MID][Nnextdiv]);

   if ((thisRInverse > 0.0 && tlane2left < nlane2left) ||
       (thisRInverse < 0.0 && tlane2left > nlane2left))
    thisRInverse *= 0.75;

   if (thisRInverse > rInverse * 1.1)
   {
    maxinv = Nnextdiv;
    rInverse = thisRInverse;
   }
  }
 
  // work out speed according to future stuff
  if (maxinv != nextdiv && rInverse > WingRInverse)
  {
   double factor = 1.0;
   mlane2left = tLane[LINE_MID][maxinv] * seg->width;
   if ((prefer_side == TR_LFT && toLeft > mlane2left*1.4 && toLeft > tLane[LINE_RL][maxinv]+1.0) ||
       (prefer_side == TR_RGT && toLeft < mlane2left*0.6 && toLeft < tLane[LINE_RL][maxinv]-1.0))
   {
    factor = MAX(0.6, MIN(1.0, (prefer_side == TR_LFT ? (1.0 - toLeft/seg->width)/2 : (toLeft/seg->width)/2)));
   }
   //speed2 = MIN(speed2, sqrt(((TireAccel1) * tFriction[maxinv] * factor) / (rInverse - WingRInverse)));
   speed2 = (tSpeed[maxinv] + speed2*2) / 3;

#ifdef SPEED_DEBUG_MSG
fprintf(stderr,"%s: TS=%.3f spd2=%.3f -> ",car->_name,TargetSpeed,speed2);
#endif
   if (tRInverse[LINE_RL][maxinv] > 0.0)
    speed2 -= tRSlowSpeed[maxinv];
   else
    speed2 -= tLSlowSpeed[maxinv];
#ifdef SPEED_DEBUG_MSG
fprintf(stderr,"%.3f\n",speed2);fflush(stderr);
#endif
  }
 }

 // return whichever speed is less.
 double speed = MIN(speed1, speed2) + AvoidSpeed;
 speed = MAX(speed, TargetSpeed * 0.85);
 return speed;
}

#if 0
double Driver::getAvoidSpeed(tTrackSeg *seg)
{
 if (!seg)
  seg = car->_trkPos.seg;

 double mu = seg->surface->kFriction;
 double maxlookahead = getSpeed() * getSpeed() / (2.0 * mu * G);
 double lookahead = 0.0;
 double radius = tSegRadius[seg->id];
 double nextradius = STRAIGHT_RADIUS;
 if (!radius)
  radius = STRAIGHT_RADIUS;

 if (seg->type == TR_STR && fabs(tRInverse[rl][nextdiv]) > 0.0026)
 {
  radius = MAX(35.0, 200.0 - (fabs(tRInverse[rl][nextdiv]) * 7500.0));
  double toleft = car->_trkPos.toLeft / seg->width;
  double delta = tLDelta[seg->id] * (1.0 - toleft) + tRDelta[seg->id] * toleft;
  if (delta < -0.03)
   radius -= MIN(radius/3, fabs(delta) * 30);
 }
  
 while (lookahead < maxlookahead)
 {
  seg = seg->next;
  nextradius = tSegRadius[seg->id];
  if (0.0 > (lookahead = seg->lgfromstart - car->_distFromStartLine))
   lookahead = seg->lgfromstart + (track->length - car->_distFromStartLine);
  if (nextradius < radius - 10.0 && nextradius > 0.0)
   break;
 }

 double speed1 = 1000.0, speed2 = 1000.0;
 if (radius && radius < STRAIGHT_RADIUS)
 {
  int next = (nextdiv+10) % Divs;
  double thisspeed = MAX(tSpeed[thisdiv], car->_speed_x);
  speed1 = sqrt((mu * G * radius * 3.0) / (1.0 - MIN(1.0, radius * CA * mu/(mass + MAX(0.0, CARMASS-1000.0)))));
  if (tSpeed[next] > thisspeed)
   speed1 += (tSpeed[next] - thisspeed) * 4.0;
  else 
  {
   speed1 -= (thisspeed - tSpeed[next]) * 1.0;

   double lane2left = tLane[rl][nextdiv] * seg->width;
   if ((segtype == TR_LFT && nextleft < lane2left && tLane[rl][nextdiv] <= tLane[rl][thisdiv]) || 
       (segtype == TR_RGT && nextleft > lane2left && tLane[rl][nextdiv] >= tLane[rl][thisdiv]))
    speed1 -= (segtype == TR_LFT ? MAX(0.0, 7.0 - car->_trkPos.toLeft) : MAX(0.0, 7.0 - car->_trkPos.toRight));
  }

  if ((segtype == TR_LFT && nextleft <= tLane[rl][nextdiv] * car->_trkPos.seg->width && tLane[rl][nextdiv] > tLane[rl][thisdiv]) ||
      (segtype == TR_RGT && nextleft >= tLane[rl][nextdiv] * car->_trkPos.seg->width && tLane[rl][nextdiv] < tLane[rl][thisdiv]))
   speed1 = TargetSpeed;
 }

 if (nextradius && nextradius < STRAIGHT_RADIUS)
 {
  if (lookahead > (getCorrecting() || getAllowCorrecting() ? car->_speed_x / 10 : car->_speed_x / 4))
  {
   double nsq = mu * G * nextradius / (1.0 - MIN(1.0, nextradius * CA * mu/(mass + MAX(0.0, CARMASS-1000.0))));
   double fr = (mu * mass * G) + (nsq * mu * CA);

   speed2 = sqrt((lookahead / mass) * 5.0 * fr * nsq);
   int ntype = SEGTYPE(seg);
   if ((ntype == TR_LFT && car->_trkPos.toLeft < 7.0) || (ntype == TR_RGT && car->_trkPos.toRight < 7.0))
    speed2 -= (7.0 - MIN(car->_trkPos.toLeft, car->_trkPos.toRight)) * 1.5;
  }
  else
  {
   int ndiv = tSegDivStart[seg->id];
   speed2 = sqrt((mu * G * nextradius * 2.5) / (1.0 - MIN(1.0, nextradius * CA * mu/(mass + MAX(0.0, CARMASS-1000.0)))));
   if (seg->type == TR_LFT)
    speed2 -= tRSlowSpeed[ndiv];
   else if (seg->type == TR_RGT)
    speed2 -= tLSlowSpeed[ndiv];
   if (Flying[ndiv])
    speed2 = MAX(speed2/2, speed2 - Flying[ndiv]);
  }
  speed2 += (lookahead/30 * MIN(2.0, MAX(0.3, (1.0 + AvoidSpeed/10))));
 }

 if (Flying[thisdiv] || Flying[nextdiv])
  speed1 = MAX(speed1/2, speed1 - MAX(Flying[thisdiv], Flying[nextdiv]));

 if (seg->type == TR_LFT || tRInverse[rl][nextdiv] > 0.0026)
 {
  speed1 -= tRSlowSpeed[nextdiv];
  if (car->_steerCmd > 0.1 && fabs(laststeer - lastksteer) > 0.2)
   speed1 -= car->_steerCmd * 20;
 }
 else if (seg->type == TR_RGT || tRInverse[rl][nextdiv] < -0.0026)
 {
  speed1 -= tLSlowSpeed[nextdiv];
  if (car->_steerCmd < -0.1 && fabs(laststeer - lastksteer) > 0.2)
   speed1 -= fabs(car->_steerCmd) * 20;
 }

 double posmodifier = 0.0;
 double tlane = nextleft / seg->width;
 if (segtype == TR_LFT || (segtype != TR_RGT && prefer_side == TR_LFT) && tlane > tLane[LINE_MID][nextdiv] && tlane > tLane[LINE_RL][nextdiv])
  posmodifier = -(tlane - tLane[LINE_MID][nextdiv]) * 5;
 else if (segtype == TR_RGT || (segtype != TR_LFT && prefer_side == TR_RGT) && tlane < tLane[LINE_MID][nextdiv] && tlane < tLane[LINE_RL][nextdiv])
  posmodifier = -(tLane[LINE_MID][nextdiv] - tlane) * 5;
 if (AvoidSpeed < 0.0)
  posmodifier *= MIN(3.0, 1.0 - AvoidSpeed/5);
 return MIN(speed1+AvoidSpeed, MIN(speed2+AvoidSpeed, TargetSpeed - (avoiding && !(getAllowCorrecting()) ? 2.0 : 1.5))) + posmodifier;
}
#endif

// Compute fitting acceleration.
double Driver::getAccel()
{
 if (fStuck || fStuckForward)
  return AccelCmd;

 //float trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
 double MaxSpeed = 1.0;
 int avoid = ((avoiding && !(avoiding & AVOID_ALIGNED)) || getCorrecting() || (align_hold-2.0 > currentsimtime));
 if (fabs(angle) > 0.6)
  MaxSpeed = 0.8;
 if (fabs(car->_steerCmd) > 0.6 && avoid)
  MaxSpeed = 0.9 - (1.0 - fabs(car->_steerCmd)) * 0.2;

 if (0 && Learning > 0.0)
 {
  double dr = learn->getRadius(car->_trkPos.seg, (!Learning || !alone || avoid || align_hold > currentsimtime));
  if (dr > 0.0)
   TargetSpeed += dr * 0.15;
  else
   TargetSpeed += dr * 0.05;
 }

 tTrackSeg *seg = car->_trkPos.seg;
 //double lane2left = (tLane[rl][thisdiv]) * seg->width;
 //double nleft = car->_trkPos.toLeft + (car->_trkPos.toLeft-prevleft) * (segtype != TR_STR || cornerdist < cornerlimit ? 25 : 15);
//fprintf(stderr,"%s: TS=%.3f %.3f/%.3f ah=%.3f cst=%.3f",car->_name,TargetSpeed,nextleft,nleft,align_hold,currentsimtime);

 double avoidspeed = getAvoidSpeed(nextleft);
 double altspeed = MAX(getSpeed() - 0.1, avoidspeed);

 if (avoid || (align_hold+3.0 > currentsimtime && fabs(car->_wheelSlipSide(0)) > 14.0))
 {
  //int nnext = (nextdiv + (int)(car->_speed_x/12)) % Divs;
  double lane2left = (tLane[rl][nextdiv]) * seg->width;
  //double nlane2left = (tLane[rl][nnext]) * seg->width;
  //double llane2left = ((tLane[LINE_LEFT][thisdiv] + tLane[LINE_LEFT][nextdiv])/2) * seg->width;
  //double rlane2left = ((tLane[LINE_RIGHT][thisdiv] + tLane[LINE_RIGHT][nextdiv])/2) * seg->width;
  //double sideslip = (car->_wheelSlipSide(0) + car->_wheelSlipSide(1)) / 2;
  //double margin1 = 5.0, margin2 = 4.0;

  //if ((seg->type == TR_STR && cornerdist > cornerlimit) ||
  //    (seg->type == TR_LFT && nlane2left > lane2left && nlane2left >= nleft) ||
  //    (seg->type == TR_RGT && nlane2left < lane2left && nlane2left <= nleft))
  //{
  // // doing nothing at all
 // }
#if 0
  else if ((nleft > nlane2left + seg->width/margin2) || nleft > rlane2left - 0.5)
  {
   TargetSpeed = MAX(car->_speed_x - 0.3, RTargetSpeed);
//fprintf(stderr," R: %.3f (%.3f)",TargetSpeed,RTargetSpeed);
  }
  else if ((nleft < nlane2left - seg->width/margin2) || nleft < llane2left + 0.5)
  {
   TargetSpeed = MAX(car->_speed_x - 0.3, LTargetSpeed);
//fprintf(stderr," L: %.3f (%.3f)",TargetSpeed,LTargetSpeed);
  }
  if (MAX(fabs(car->_wheelSlipSide(0)), fabs(car->_wheelSlipSide(1))) > 10.0 &&
      MAX(fabs(car->_wheelSlipSide(0)), fabs(car->_wheelSlipSide(1))) > MAX(fabs(car->_wheelSlipSide(2)), fabs(car->_wheelSlipSide(3))) * 0.94)
  {
   TargetSpeed -= (MAX(fabs(car->_wheelSlipSide(0)), fabs(car->_wheelSlipSide(1)))-10.0)*0.5;
//fprintf(stderr," SL: %.3f",TargetSpeed);
  }
  else if (fabs(laststeer) > 0.7)
  {
   TargetSpeed = MIN(TargetSpeed, car->_speed_x + (1.0 - fabs(laststeer))*0.7);
//fprintf(stderr," ST: %.3f",TargetSpeed);
  }
#else

  if (fabs(car->_steerCmd - k1999steer) > 0.2 || fabs(nextleft - lane2left) > 4.0)
   TargetSpeed = MIN(TargetSpeed, altspeed);
#endif
 }
//fprintf(stderr," -> %.3f\n",TargetSpeed);
 TargetSpeed = AvoidBrake(TargetSpeed);

#if 0
 if (!brake_collision && !avoid && TargetSpeed < car->_speed_x)
 {
  TargetSpeed += (1.0 - (CARMASS + car->_fuel) / (CARMASS + car->_tank)) * 4.0;
 }
#endif

 //
 // Throttle and brake command
 //
 AccelCmd = BTAccelCmd = 0.0;
 BrakeCmd = BTBrakeCmd = KBrakeCmd = 0.0;

 double x = (10 + car->_speed_x) * (TargetSpeed - car->_speed_x) / 200;
 if (fDirt && x > 0)
  x = 1;

 if (x > 0)
 {
  AccelCmd = MIN(x, TractionHelp);
 }
 else
 {
  if (0 && (avoid || brake_avoid || brake_collision))
  {
   double absredux = MAX(0.02, ABS - MAX(fabs(angle), fabs(car->_yaw_rate)*0.25) * 0.35);
   if (absredux < 0.0)
   {
    AccelCmd = MAX(AccelCmd, -absredux);
    ABS = 0.0f;
   }
   else
    ABS = absredux;
  }
  //if (brake_avoid)
  // ABS *= brake_avoid;
  if (K1999Brakes && !brake_collision)
  {
   BrakeCmd = MIN(-10 * x, ABS);
   if (brake_collision && BrakePressure < 3200000.0)
   {
    // extra braking to account for low pressure brakes
    BrakeCmd = MIN(2.0, BrakeCmd + (1.0 - (BrakePressure/3200000.0))*0.75);
   }
   KBrakeCmd = BrakeCmd = MIN(BrakeCmd, ABS);
  }
  else
   KBrakeCmd = BrakeCmd = -10*x;

  if (brake_collision)
  {
   x = (10 + car->_speed_x) * (KTargetSpeed - car->_speed_x) / 200;

   if (x > 0)
   {
    KBrakeCmd = 0.0;
   }
   else
   {
    if (K1999Brakes && !brake_collision)
     KBrakeCmd = MIN(-10 * x, ABS);
    else
     KBrakeCmd = -10*x;
   }
  }

  //if ((avoiding || getCorrecting())
  // RELAXATION(BrakeCmd, lastbrake, MaxBrake * 0.8);//1.0f : 0.7f);

 }
 
 if (AccelCmd > 0.0)
 {
  BrakeCmd = 0.0;
  //if (AccelCmd >= 0.75 && fabs(car->_steerCmd) < 0.1)
  // AccelCmd = 1.0f;
 }

 x = (10 + car->_speed_x) * (TargetSpeed - car->_speed_x) / 200;
 if (fDirt && x > 0)
  x = 1;

 AccelCmd = MIN(AccelCmd, MaxSpeed);

 //if (fabs(car->_steerCmd) > 0.3f && racetype == RM_TYPE_RACE)
 // AccelCmd = MIN(AccelCmd, ABS * 1.6f);

 // get the BT speed limit, will decide later if we want to use it
 BTargetSpeed = getAllowedSpeed(car->_trkPos.seg, 1);

#if 0
 if (!brake_collision && !avoid && BTargetSpeed < car->_speed_x)
 {
  BTargetSpeed += (1.0 - (CARMASS + car->_fuel) / (CARMASS + car->_tank)) * 4.0;
 }
#endif

 x = (10 + car->_speed_x) * (BTargetSpeed - car->_speed_x) / 200;
 if (fDirt && x > 0)
  x = 1;

 if (x > 0)
  BTAccelCmd = MIN(x, TractionHelp);
 else
  BTAccelCmd = 0.0;

 if (brake_collision) 
  BTAccelCmd = MIN(BTAccelCmd, AccelCmd);

 if (avoid)
 {
  if (fabs(angle) > 0.4 || (car->_trkPos.seg->type == TR_LFT && angle > 0.05) || (car->_trkPos.seg->type == TR_RGT && angle < -0.05))
  { 
   BTAccelCmd = MAX(0.1, BTAccelCmd - fabs(angle)/2);
   AccelCmd = MAX(0.1, AccelCmd - fabs(angle)/2);
  }
 }

 if (fabs(car->_steerCmd) > 0.9) 
 {
  AccelCmd = MIN(AccelCmd, 0.8);
  BTAccelCmd = MIN(BTAccelCmd, 0.8);
 }
 if (!BrakeCmd && car->_speed_x < 5.0 && getSpeed() < 5.0 && !fStuck && !fStuckForward)
 {
  AccelCmd = MAX(0.4, AccelCmd);
  BTAccelCmd = MAX(0.4, BTAccelCmd);
 }

 //
 // Traction help
 //
 if (!fDirt)
 {
  double slip = 0;
  if (car->_speed_x > 0.1)
   for (int i = 4; --i >= 0;)
   {
    double s = (car->_wheelRadius(i) * car->_wheelSpinVel(i) - car->_speed_x);
    if (s > slip)
     slip = s;
   }

  if (slip > TSlipLimit)
   TractionHelp *= 1.2;
  else
  {
   if (TractionHelp < 0.1)
    TractionHelp = 0.1;
   TractionHelp *= 1.2;
   if (TractionHelp > 1.0)
    TractionHelp = 1.0;
  }
 }

 //
 // ABS
 //
#if 0
 if (K1999Brakes)
 {
  double sliplimit = SlipLimit;
  if (fabs(angle) < 0.4 && !fStuck && fabs(laststeer) > 0.3)
   sliplimit *= 1.0 + fabs(laststeer) * 1.5;
  double slip = 0;
  if (car->_speed_x > 0.1)
   for (int i = 4; --i >= 0;)
   {
    double s = (car->_wheelRadius(i) * car->_wheelSpinVel(i) - car->_speed_x);
    if (s < slip)
     slip = s;
   }

  if (slip < -sliplimit)
   ABS *= (brake_avoid || brake_collision) ? 0.9 : ABSFactor;//0.9;
  else
  {
   if (ABS < 0.1)
    ABS = 0.1;
   ABS *= (2.5 + (slip + 3.0) / 2);
   if (ABS > 2.5)
    ABS = 2.5;
  }
 } 
#endif

 if (fabs(car->_yaw_rate) > 2.0 || fabs(angle) > 1.2)
 {
  AccelCmd = BrakeCmd = BTAccelCmd = BTBrakeCmd = 0.0;
 }

 if (car->_gear > 0) {
  if (TargetSpeed > car->_speed_x) {
   return MaxSpeed;
  } else {
   double gr = car->_gearRatio[car->_gear + car->_gearOffset];
   double rm = car->_enginerpmRedLine;
   if (pit->getInPit())
    return MIN(MaxSpeed, MIN(AccelCmd, TargetSpeed/car->_wheelRadius(REAR_RGT)*gr /rm));
   else
    return MIN(MaxSpeed, MAX(AccelCmd, TargetSpeed/car->_wheelRadius(REAR_RGT)*gr /rm));
  }
 } else {
  return MaxSpeed;
 }
}


// If we get lapped reduce accelerator.
double Driver::filterOverlap(double accel)
{
 int i;

 if (!(avoiding & AVOID_LETPASS))
  return accel;

 for (i = 0; i < opponents->getNOpponents(); i++) {
  if (opponent[i].car->_state >= RM_CAR_STATE_PIT) continue;
  if ((opponent[i].getState() & OPP_LETPASS)) {
   double slowaccel = 0.85;
   if (opponent[i].getTeam() & TEAM_CONTROL)
    slowaccel = 0.7;
   BTAccelCmd = MIN(BTAccelCmd, slowaccel);
   AccelCmd = MIN(AccelCmd, slowaccel);
   return MIN(accel, slowaccel);
  }
 }
 return accel;
}


// Compute initial brake value.
double Driver::getBrake(double brake)
{
 BTBrakeCmd = brake;
 int avoid = ((avoiding && !(avoiding & AVOID_ALIGNED)) || getCorrecting());
 if (fStuck || fStuckForward || (car->_speed_x > -2.0 && car->_speed_x < 10.0) || (!avoid && !useBTSpeed))
  return brake;

 // Car drives backward?
 if (car->_speed_x < -MAX_UNSTUCK_SPEED) 
 {
  // Yes, brake.
  BTBrakeCmd = 1.0;
  return brake;
 }

 // We drive forward, normal braking.
 tTrackSeg *segptr = car->_trkPos.seg;
 double mu = segptr->surface->kFriction;
 double maxlookaheaddist = currentspeedsqr/(2.0*mu*G);
 double lookaheaddist = getDistToSegEnd();

 double allowedspeed = getAllowedSpeed(segptr, 0);
 // this increases its speed around corners
 if (myoffset == 0.0)
  allowedspeed *= 1.2;
 else
  allowedspeed *= 1.05;

 if (allowedspeed < car->_speed_x) {
  BTBrakeCmd = MIN(0.4, ((car->_speed_x-allowedspeed)/(FULL_ACCEL_MARGIN))/2);
  return brake;
 }

 segptr = segptr->next;
 while (lookaheaddist < maxlookaheaddist) 
 {
  allowedspeed = getAllowedSpeed(segptr, 0);
  double dr = learn->getRadius(segptr, avoid);
  if (myoffset == 0.0)
   allowedspeed *= 1.1;
  else
   allowedspeed *= 1.05;

  if (allowedspeed < car->_speed_x) {
   double bd = brakedist(allowedspeed, mu);

   if (bd > lookaheaddist) {
    if (segptr->radius > 200.0 && lookaheaddist < 80.0 && allowedspeed > car->_speed_x+20.0 && (segptr->next->type == TR_STR || segptr->next->radius > 200.0))
    {
     if (bd < lookaheaddist + segptr->next->length)
     {
      lookaheaddist += segptr->length;
      segptr = segptr->next;
      continue;
     }
    }
    if (segptr->type != TR_STR && (segptr->radius > 100.0 || tSegLength[tSegID[segptr->id]] < segptr->radius * 0.75))
    {
     double angle = RtTrackSideTgAngleL(&(car->_trkPos));
     angle -= car->_yaw;
     NORM_PI_PI(angle);

     if (dr > 0.0f && bd > lookaheaddist + dr/3 && fabs(angle) < 0.4 && fabs(car->_yaw_rate) < 0.7)
     {
      lookaheaddist += segptr->length;
      segptr = segptr->next;
      continue;
     }
    }

    BTBrakeCmd = 0.4;
    return brake;
   }
  }
  lookaheaddist += segptr->length;
  segptr = segptr->next;
 }
 return brake;
}


// Compute gear.
int Driver::getGear()
{
 car->_gearCmd = car->_gear;
 if (car->_gear <= 0)
  car->_gearCmd = 1;
 else
 {
  double speed = (avoiding || getCorrecting() ? MAX(car->_speed_x, getSpeed()) : car->_speed_x);
  float *tRatio = car->_gearRatio + car->_gearOffset;
  float rpm = (float)((speed + (SlipLimit/3)) * tRatio[car->_gear] / car->_wheelRadius(2));
  float down_rpm = (float)(car->_gear > 1 ? (speed + (SlipLimit/3)) * tRatio[car->_gear-1] / car->_wheelRadius(2) : rpm);
  //double up_rpm = car->_gear < 6 ? (car->_speed_x + (SlipLimit/3)) * tRatio[car->_gear+1] / car->_wheelRadius(2) : rpm;

  if (rpm > car->_enginerpmRedLine * RevsChangeUp)
   car->_gearCmd = car->_gear + 1;
     
  if (car->_gear > 1 &&
    rpm < car->_enginerpmRedLine * RevsChangeDown && 
    down_rpm < car->_enginerpmRedLine * RevsChangeDownMax)
   car->_gearCmd = car->_gear - 1;
 }
  return car->_gearCmd;
}


// Compute steer value.
double Driver::getSteer(tSituation *s)
{
 double targetAngle;
 double steercmd = 0, bsteercmd, dsteercmd = 0;
 int pitexiting = (pit->isExiting() && car->_trkPos.toLeft > 0.0 && car->_trkPos.toRight > 0.0);
 int inpit = (!pitexiting && (pit->getInPit() || pit->getLastInPit())) ? 2 : 0;
 int ctype = TR_STR;
 tTrackSeg *seg = car->_trkPos.seg;
 double nextright = seg->width - nextleft;
 //float lane2left = tLane[rl][nextdiv] * seg->width;
 //double yaw_limit = (((0.5f-CR)) * (CA)) * 5;
 //double yaw_limit = -1.121;

 //if (getCorrecting() || avoiding)
 {
  cornerdist = getDistToCorner();
//fprintf(stderr,"cdist %.3f\n",cornerdist);
  //cseg = getNextCornerSeg();
  //cornerlimit = 30.0f + ((5.55f - CW) * (100.0 - cseg->radius));
  if (cseg)
  {
   //cornerlimit = cseg->length*0.5f + ((5.55f - CW) * (100.0f - cseg->radius));
   cornerlimit = (car->_speed_x < cseg->radius ? car->_speed_x : car->_speed_x + (car->_speed_x - cseg->radius)) * 1.2;
   ctype = (segtype != TR_STR ? segtype :
            cornerdist < cornerlimit ? SEGTYPE(cseg) : TR_STR);
  }
  else
   cornerlimit = 0.0;
 }

 double lane2left = tLane[rl][thisdiv] * seg->width;
 if (avoiding && !(avoiding & (AVOID_SIDE|AVOID_ALIGNED)) && 
     ((car->_trkPos.toLeft < 0.0f && car->_trkPos.toLeft < lane2left-1.0) || 
      (car->_trkPos.toRight < 0.0f && car->_trkPos.toLeft > lane2left+1.0)))
 {
  avoid_hold = 0.0f;
  setCorrecting(1);
  avoiding = 0;
 }

 if (pit->isExiting() && !pitexiting)
 {
  setCorrecting(1);
  correct->initCorrectValues();
 }

 v2d target = getTargetPoint();
 rl = LINE_RL;
 k1999steer = K1999Steer(s);
 //k1999steer_c = K1999Steer(s, 1);
#if 0
 rl = LINE_LEFT;
 k1999steerL = K1999Steer(s);
 rl = LINE_RIGHT;
 k1999steerR = K1999Steer(s);
 rl = LINE_RL;
#endif

#if 0
 if (!inpit && (pit->isApproaching()) && !(avoiding & AVOID_SIDE))
 {
  inpit = 1;
  avoiding = 0;
 }
#endif

 double lftlimit = 0.0, rgtlimit = 0.0;
 //double tlane2left = tLane[rl][thisdiv] * seg->width;

 if (inpit && !pitexiting)
 {
  // bt steering for getting into pits.
  targetAngle = atan2(target.y - car->_pos_Y, target.x - car->_pos_X);
  double yr = car->_yaw_rate * 0.05;//0.1;
  targetAngle -= (car->_yaw + yr);
  NORM_PI_PI(targetAngle);
  steercmd = targetAngle / car->_steerLock;
  //if ((angle > 0.8 && steercmd > 0.25) ||
  //    (angle < -0.8 && steercmd < -0.25))
  // steercmd *= 0.8f;
  bsteercmd = steercmd;

#if 1
  double limitfactor = SteerLimit;//1.0 - MIN(0.7, 3.2 - MIN(3.2, CA)) * 0.08;
#if K1999AVOIDSTEER
  limitfactor -= 0.3;
#endif
  limitfactor += (1.0-limitfactor) * (0.6+(0.19*2));
  limitfactor -= ((car->_speed_x < 30 ? 0.0f : (MIN(75.0f, car->_speed_x) - 30.0f) * 0.006));
  limitfactor = MAX(limitfactor, 0.01f);
  double frntslip = (car->_wheelSlipSide(0) + car->_wheelSlipSide(1)) / 2;
  double rearslip = (car->_wheelSlipSide(2) + car->_wheelSlipSide(3)) / 2;
  if (fabs(rearslip - frntslip) > 10.0 && fabs(rearslip) > fabs(frntslip))
   limitfactor *= 1.0 + (1.0 - fabs(frntslip)/fabs(rearslip)) / 2;
  double aerofactor = 0.09 * 4;

  lftlimit = (SEGTYPE(seg) == TR_LFT && prefer_side == TR_LFT && (nextleft > 6.0f || (avoiding & AVOID_SIDE)) && !(avoiding&AVOID_LEFT) ? 0.07f+aerofactor : seg->type != TR_RGT && prefer_side == TR_LFT ? 0.07f : 0.03f) * limitfactor;
  rgtlimit = (SEGTYPE(seg) == TR_RGT && prefer_side == TR_RGT && (nextright > 6.0f || (avoiding & AVOID_SIDE)) && !(avoiding&AVOID_RIGHT) ? 0.07f+aerofactor : seg->type != TR_LFT && prefer_side == TR_RGT ? 0.07f : 0.03f) * limitfactor;
#else
  float limitfactor = MAX(0.001f, ((1.0-0.19) / ((car->_speed_x*(car->_speed_x)/10) / 12))*SteerLimit);
  float lftlimit = (SEGTYPE(seg) == TR_LFT && !(avoiding&AVOID_LEFT) ? 1.1 : (seg->type != TR_RGT ? 1.05 : 0.90)) * limitfactor;
  float rgtlimit = (SEGTYPE(seg) == TR_RGT && !(avoiding&AVOID_RIGHT) ? 1.1 : (seg->type != TR_LFT ? 1.05 : 0.90)) * limitfactor;
#endif
  steercmd = MAX(laststeer - rgtlimit*3, MIN(laststeer + lftlimit*3, steercmd));
 }

 // K1999 steering for general avoidance
 if (!inpit && (avoiding || getCorrecting() || align_hold > s->currentTime) /*&& !pit->getInPit()*/ && !fStuck)
  dsteercmd = steercmd = AvoidSteer(s);
 else
  correct_meld = 0.0;

 if (avoiding && !(avoiding & AVOID_SIDE) && fabs(k1999steer) > fabs(dsteercmd) && (car->_trkPos.toLeft < -2.0 || car->_trkPos.toLeft > car->_trkPos.seg->width+2.0))
 if (avoiding && !(avoiding & (AVOID_SIDE|AVOID_ALIGNED)) && fabs(k1999steer) > fabs(dsteercmd) && 
     ((car->_trkPos.toLeft < 0.0f && car->_trkPos.toLeft < lane2left-1.0) || 
      (car->_trkPos.toRight < 0.0f && car->_trkPos.toLeft > lane2left+1.0)))
 {
  setCorrecting(1);
  avoiding = 0;
  avoid_hold = 0.0f;
 }

 if (avoiding && car->_gear > 0 && fabs(angle) > 0.5 && fabs(steercmd) > 0.3 && car->_speed_x > 15.0 &&
     car->_trkPos.toLeft >= 0.0 && car->_trkPos.toRight >= 0.0)
 {
  // in trouble, ease off on the gas.
  AccelCmd = MIN(AccelCmd, 0.65);
  //steercmd = MIN(0.4, MAX(-0.4, steercmd));
 }

 int this_allow_correcting = getAllowCorrecting();
 if (!this_allow_correcting && avoiding && !((avoiding & AVOID_LEFT) && (avoiding & AVOID_RIGHT)))
 {
  //int ndiv = (nextdiv + (int)(car->_speed_x/7)) % Divs;
  //double nlane2left = tLane[rl][ndiv] * seg->width;
  if ((k1999steer > k1999steer_a && !(avoiding & AVOID_LEFT))
      || (k1999steer < k1999steer_a && !(avoiding & AVOID_RIGHT))) 
  {
   this_allow_correcting = 1;
  }

  if (this_allow_correcting && !getAllowCorrecting())
  {
   setAllowCorrecting(1);
  }

 }

 if (avoiding && ((avoiding & AVOID_ALIGNED) || (this_allow_correcting && lastksteer == laststeer)))
 {
  k1999steer_a = k1999steer;
 }

 if ((!avoiding && !getCorrecting() && !inpit) || (avoiding & AVOID_ALIGNED))
 {
  // rl steer - pure k1999
  steercmd = k1999steer;
 }
 else if (!inpit)
 {
  double limitfactor = SteerLimit * ((1.0 - MAX(20.0, MIN(70.0, car->_speed_x))/80)/10);
  double frntslip = (car->_wheelSlipSide(0) + car->_wheelSlipSide(1)) / 2;
  double rearslip = (car->_wheelSlipSide(2) + car->_wheelSlipSide(3)) / 2;
  if (fabs(rearslip - frntslip) > 10.0 && fabs(rearslip) > fabs(frntslip))
   limitfactor *= 1.0 + (1.0 - fabs(frntslip)/fabs(rearslip)) / 2;
  else if (fabs(car->_yaw_rate) < 0.5)
   limitfactor *= 0.75 + fabs(car->_yaw_rate)/2;

  double lftlimit = limitfactor, rgtlimit = limitfactor;
  if (segtype == TR_LFT)
   rgtlimit *= 0.7;
  else if (segtype == TR_RGT)
   lftlimit *= 0.7;

  if (currentsimtime < 5.0 && car->_speed_x < 15.0)
   steercmd = 0.0f;
  //else if ((last_flying & FLYING_FRONT) || (fabs(angle) < 0.5 && fabs(car->_yaw_rate) < 1.8 && !fStuckForward))
  else if (!getCorrecting() && !getAllowCorrecting() && !fStuckForward)
  {
   steercmd = MAX(laststeer - rgtlimit, MIN(laststeer + lftlimit, steercmd));
  }
  else if ((avoiding || getCorrecting()) && !fStuckForward && fabs(angle) < 1.2)
  {
   if (fabs(steercmd) < fabs(laststeer))
    steercmd = MAX(laststeer - rgtlimit*6, MIN(laststeer + lftlimit*6, steercmd));
   else
    steercmd = MAX(laststeer - rgtlimit*2, MIN(laststeer + lftlimit*2, steercmd));
  }

  if ((getCorrecting() && start_finished) || this_allow_correcting)
  {
   double nlane2left = tLane[rl][nextdiv] * seg->width;
   if ((last_flying & FLYING_FRONT) || pit->isExiting())
    correct->initCorrectValues();
   int slowcorrect = ((last_flying & FLYING_FRONT) ? 1 : 
     (pit->isExiting() || car->_accel_x < -5.0 || 
      (segtype == TR_STR && cornerdist < cornerlimit * 0.7) ||
      (tRInverse[LINE_RL][nextdiv] > 0.001 && nlane2left > nextleft) ||
      (tRInverse[LINE_RL][nextdiv] < -0.001 && nlane2left < nextleft)) ? 1 : 0);
   
   steercmd = correct->CorrectSteer(steercmd, k1999steer, slowcorrect, car->_speed_x, car->_accel_x);
   //steercmd = correct->CorrectSteer(steercmd, k1999steer, nlane2left, slowcorrect, 1.0 + (MaxIncFactor < MAXINCFACTOR ? ((MAXINCFACTOR-MaxIncFactor)+0.1) * (2.0 + car->_speed_x/15) : 0.1 * (2.0 + car->_speed_x/15)), car->_speed_x);
#if 0
   double old_ct = correct_timer;
   int next2 = (nextdiv + 2 + Divs) % Divs;
   double speed = MAX(35.0, MIN(getSpeed(), tMaxSpeed[next2])) - 10.0;
   double csteer = k1999steer;
   double factor = 100.0;
   if (this_allow_correcting & CORRECT_LEFT)
    csteer = k1999steerL;
   else if (this_allow_correcting & CORRECT_RIGHT)
    csteer = k1999steerR;

   if (correcting && segtype != TR_STR && seg->radius <= 60.0 && (prefer_side == seg->type || prefer_side == TR_STR) && fabs(nextleft-lane2left) > 5.0)
   {
    speed *= 2;
   }

   if ((last_flying & FLYING_FRONT) || car->_accel_x < -5.0)
    correct_timer += deltaTime;
   else if (correct_timer < currentsimtime ||
            (MIN(car->_trkPos.toLeft, car->_trkPos.toRight) > 2.0 &&
             ((avoiding & AVOID_ALIGNED) ||
              (csteer > laststeer && csteer < laststeer + (lftlimit/2)) || 
              (csteer < laststeer && csteer > laststeer - (rgtlimit/2)))))
   {
    correct_timer = currentsimtime;
   }
   else
   {
    //if (((seg->type == TR_LFT || prefer_side == TR_LFT) && csteer < steercmd && k1999steer_c > csteer) ||
    //    ((seg->type == TR_RGT || prefer_side == TR_RGT) && csteer > steercmd && k1999steer_c < csteer))
    // csteer = k1999steer_c;

    factor = MIN(CORRECT_TIMER, ((speed*(speed/10) * MIN(1.0, fabs(steercmd-csteer)*2)) / 2.0));

    factor *= MAX(correcting, this_allow_correcting);

    double redux = (speed > 40.0 ? MAX(0.01, 0.08- MIN(30.0, speed-40.0)/380) : 0.08);
    if (speed<=45.0 && segtype != TR_STR && seg->radius <= 50.0)
     redux += (55.0-seg->radius)/380;
    redux -= FWA/8;
    if (fabs(steercmd-k1999steer) < redux && fabs(k1999steer) < redux)
     factor *= fabs(steercmd-k1999steer)/(redux >= 0.08 ? redux : 0.08);

    int change = 0;

    if (fabs(k1999steer - steercmd) > fabs(k1999steer - laststeer) && fabs(laststeer) < fabs(steercmd))
    {
     if (segtype != TR_LFT && steercmd <= laststeer - lftlimit && !(side_count & AVOID_LEFT)
         && !(segtype != TR_RGT && prefer_side == TR_RGT && cornerdist < cornerlimit))
     {
      steercmd += lftlimit*0.8;
      change = 1;
     }
     else if (segtype != TR_RGT && steercmd >= laststeer + rgtlimit && !(side_count & AVOID_RIGHT)
         && !(segtype != TR_LFT && prefer_side == TR_LFT && cornerdist < cornerlimit))
     {
      steercmd -= rgtlimit*0.8;
      change = 1;
     }
    }
#if 1
    if (!change && !this_allow_correcting 
        && fabs(k1999steer) < fabs(laststeer) && fabs(laststeer) < fabs(steercmd) && fabs(laststeer - k1999steer) < fabs(steercmd - k1999steer))
    {
     // following lane path is harder steering, whereas raceline is less - move to raceline.
     steercmd = laststeer;
#if 1
     if (k1999steer < laststeer)
      steercmd = laststeer - rgtlimit/4;
     else
      steercmd = laststeer + lftlimit/4;
#endif
    }
#endif
    
    correct_timer = MAX(currentsimtime, MIN(correct_timer, MAX(currentsimtime, currentsimtime + factor)));
   }

   if (lastbrake > 0.0)
    correct_timer = MAX(currentsimtime, MIN(old_ct, correct_timer + lastbrake * 5.0));

   {
    double incr = MAX(0.0, MIN(1.0, ((CORRECT_TIMER+0.5) - (correct_timer - currentsimtime)) / CORRECT_TIMER));
    if (incr > 0.0)
    {
     double change = ((exp(incr)-1)/(exp(1.0)-1));
     if (MaxIncFactor < 2.6 && fabs(steercmd - csteer) > 0.06)
      change *= MAX(0.05, 1.0 - (2.6 - MaxIncFactor)*1.2);
     steercmd += (csteer - steercmd) * change;
    }
   }
#endif
  }
 }
 else
 {
  // in pit
  correct->setAligned(0);
  correct->setCorrecting(1);
  correct->initCorrectValues();
 }

 if (rearWheelSlip() && (avoiding || getCorrecting() || align_hold > currentsimtime))
 {
  double limit = MAX(0.1, 0.5 - fabs(car->_yaw_rate)/2);
  steercmd = MIN(limit, MAX(-limit, steercmd));
 }

 if (!pit->getInPit() && fabs(steercmd) * 4 > fabs(getSpeed()))
  steercmd *= 0.6;

#ifdef STEER_DEBUG_MSG
fprintf(stderr,"%s %d/%d: %d%c%c%c%d|%d %.3f (k%.3f a%.3f/%.3f l%.3f) ac=%.3f ct=%.3f cl=%.3f\n",car->_name,thisdiv,car->_dammage,avoiding,((avoiding&AVOID_SIDE)?'s':(avoiding&AVOID_FRONT)?'f':' '),((avoiding&AVOID_LEFT)?'l':(avoiding&AVOID_RIGHT)?'r':' '),((avoiding & AVOID_SIDE_COLL) ? 'X' : ' '),correct->getAllowCorrecting(),getCorrecting(),steercmd,k1999steer,dsteercmd,k1999steer_a,laststeer,car->_accelCmd,correct->getCorrectTimer(),correct->getCorrectLimiter());fflush(stderr);
#endif
#ifdef STATE_DEBUG_MSG
fprintf(stderr,"%s %d/%d: 2l=%.3f sp=%.3f sk=%.3f/%.3f/%.3f/%.3f re=%.3f/%.3f/%.3f/%.3f\n",car->_name,thisdiv,car->_dammage,car->_trkPos.toLeft,car->_speed_x,car->_skid[0],car->_skid[1],car->_skid[2],car->_skid[3],car->_reaction[0],car->_reaction[1],car->_reaction[2],car->_reaction[3]);fflush(stderr);
#endif
 laststeer = steercmd;
 lastksteer = k1999steer;

 return steercmd;
}


// Compute the clutch value.
double Driver::getClutch()
{
 /*if (car->_brakeCmd > 0.0)
  return car->_brakeCmd;
 else if (car->_gear > 1) {
  clutchtime = 0.0;
  return 0.0;
 } else*/ 
 if (1 || car->_gearCmd > 1)
 {
  double maxtime = MAX(0.06, 0.32 - ((double) car->_gearCmd / 65));
  if (car->_gear != car->_gearCmd)
   clutchtime = (float)maxtime;
  if (clutchtime > 0.0)
   clutchtime -= (float)(RCM_MAX_DT_ROBOTS * (0.2 + ((double) car->_gearCmd / 8.0)));
  //if (car->_brakeCmd > 0.0 && lastbrake > 0.0 && lastaccel == 0.0)
  // clutchtime = MIN(maxtime*0.75, clutchtime + MIN(car->_brakeCmd, (car->_brakeCmd+lastbrake)/2)/15);
  return 2.0 * clutchtime;
 }
 else
 {
  double drpm = car->_enginerpm - car->_enginerpmRedLine/2.0;
  double ctlimit = ClutchTime;
  if (car->_gearCmd > 1)
   ctlimit -= 0.15 + (double)car->_gearCmd/13;
  clutchtime = (float)MIN(ctlimit, clutchtime);
  if (car->_gear != car->_gearCmd)
   clutchtime = 0.0;
  double clutcht = (ctlimit - clutchtime)/ctlimit;
  if (car->_gear >= 1 && car->_accelCmd > 0.0) {
   clutchtime += (float) RCM_MAX_DT_ROBOTS;
  }

  if (car->_gearCmd == 1 || drpm > 0) {
   double speedr;
   if (car->_gearCmd >= 1) {
    // Compute corresponding speed to engine rpm.
    float omega = car->_enginerpmRedLine/car->_gearRatio[car->_gear + car->_gearOffset];
    float wr = car->_wheelRadius(2);
    speedr = (CLUTCH_SPEED + fabs(car->_speed_x))/fabs(wr*omega);
    float clutchr = (float)MAX(0.0, (1.0 - (speedr*2.0*drpm/car->_enginerpmRedLine))) * (car->_gearCmd == 1 ? 0.95f : (float)(0.7 - (double)(car->_gearCmd)/30));
    return MIN(clutcht, clutchr);
   } else {
    // For the reverse gear.
    clutchtime = 0.0;
    return 0.0;
   }
  } else {
   return 0.0;
  }
 }
}

// Compute target point for steering.
v2d Driver::getTargetPoint(double offset, double lookahead)
{
 tTrackSeg *seg = car->_trkPos.seg;
 double factor = 1.0;
 double length = getDistToSegEnd();

 // Search for the segment containing the target point.
 while (length < lookahead * factor) {
  seg = seg->next;
  length += seg->length;
 }

 length = lookahead * factor - length + seg->length;
 double fromstart = seg->lgfromstart;
 fromstart += length;

 v2d s, k;
 s.x = (float)((seg->vertex[TR_SL].x + seg->vertex[TR_SR].x)/2.0);
 s.y = (float)((seg->vertex[TR_SL].y + seg->vertex[TR_SR].y)/2.0);

 if ( seg->type == TR_STR) {
  v2d d, n;
  n.x = (seg->vertex[TR_EL].x - seg->vertex[TR_ER].x)/seg->length;
  n.y = (seg->vertex[TR_EL].y - seg->vertex[TR_ER].y)/seg->length;
  n.normalize();
  d.x = (seg->vertex[TR_EL].x - seg->vertex[TR_SL].x)/seg->length;
  d.y = (seg->vertex[TR_EL].y - seg->vertex[TR_SL].y)/seg->length;
  s = s + d*(float)length + (float)offset*n;
 } else {
  v2d c, n;
  c.x = seg->center.x;
  c.y = seg->center.y;
  double arc = length/seg->radius;
  double arcsign = (seg->type == TR_RGT) ? -1.0 : 1.0;
  arc = arc*arcsign;
  s = s.rotate(c, (float)arc);

  n = c - s;
  n.normalize();
  s = s + ((float)(arcsign*offset))*n;
 }

 return s;
}

v2d Driver::getTargetPoint()
{
 tTrackSeg *seg = car->_trkPos.seg;
 double lookahead;
 double length = getDistToSegEnd();
 double offset = getOffset();
 double nextoffset = seg->width * 0.5 - nextleft;
 double factor = 1.0;

 if (pit->getInPit()) {
  // To stop in the pit we need special lookahead values.
  if (currentspeedsqr > pit->getSpeedlimitSqr()) {
   lookahead = PIT_LOOKAHEAD + car->_speed_x*LOOKAHEAD_FACTOR;
  } else {
   lookahead = PIT_LOOKAHEAD;
  }
  align_hold = avoid_hold = 0.0;
  avoiding = 0;
 } else {
  // Usual lookahead.
  //if (correcting)
  // factor = 0.82;
  //double aero_factor = (((0.5f-CR)) * (CA)) * 2;
  double aero_factor = -0.4484;
  lookahead = LOOKAHEAD_CONST + MAX(35.0f, car->_speed_x)*(LOOKAHEAD_FACTOR - aero_factor);
  // Prevent "snap back" of lookahead on harsh braking.
  double cmplookahead = oldlookahead - car->_speed_x*RCM_MAX_DT_ROBOTS;
  if (lookahead < cmplookahead) {
   lookahead = cmplookahead;
  }

  if ((avoiding & AVOID_FRONT) && !(avoiding & AVOID_SIDE))
   offset = myoffset = (float)MAX(-seg->width * 0.5f + 0.2f, MIN(seg->width*0.5f-0.2f, offset));
 }

 oldlookahead = (float)(lookahead * factor);

 // Search for the segment containing the target point.
 while (length < lookahead * factor) {
  seg = seg->next;
  length += seg->length;
 }

 length = lookahead * factor - length + seg->length;
 double fromstart = seg->lgfromstart;
 fromstart += length;

 // Compute the target point.
 double pitoffset = pit->getPitOffset(car->_trkPos.toMiddle, fromstart, nextleft, pitpos);
 int pitting = 0;
 if (car->_trkPos.toMiddle != pitoffset || pit->isExiting())
 {
  offset = pitoffset;
  pitting = 1;
  if (!pit->isApproaching() &&
      fabs(offset) < fabs(nextoffset) &&
      fabs(nextoffset) < seg->width * 0.5f - 2.0f && 
      ((track->pits.side == TR_LFT && nextoffset > offset) ||
       (track->pits.side == TR_RGT && nextoffset < offset)))
  {
   offset = myoffset = (float)nextoffset;
  }
  else
  {
   nextoffset = myoffset = (float)offset;
  }
 }

 v2d s, k;
 s.x = (float)((seg->vertex[TR_SL].x + seg->vertex[TR_SR].x)/2.0);
 s.y = (float)((seg->vertex[TR_SL].y + seg->vertex[TR_SR].y)/2.0);
 nextpt = s;

 if (pitting || avoiding || getCorrecting())
 {
  if ( seg->type == TR_STR) {
   v2d d, n;
   n.x = (seg->vertex[TR_EL].x - seg->vertex[TR_ER].x)/seg->length;
   n.y = (seg->vertex[TR_EL].y - seg->vertex[TR_ER].y)/seg->length;
   n.normalize();
   d.x = (seg->vertex[TR_EL].x - seg->vertex[TR_SL].x)/seg->length;
   d.y = (seg->vertex[TR_EL].y - seg->vertex[TR_SL].y)/seg->length;
   s = s + d*((float)length) + (float)offset*n;
   nextpt = nextpt + d*((float)length) + (float)nextoffset*n;
  } else {
   v2d c, n;
   c.x = seg->center.x;
   c.y = seg->center.y;
   double arc = length/seg->radius;
   double arcsign = (seg->type == TR_RGT) ? -1.0 : 1.0;
   arc = arc*arcsign;
   s = s.rotate(c, (float)arc);

   n = c - s;
   n.normalize();
   nextpt = s + (float)(arcsign*nextoffset)*n;
   s = s + (float)(arcsign*offset)*n;
  }

  return s;
 }
 else
 {
  nextpt = k1999pt;
  return k1999pt;
 }
}

void Driver::updateSegAvoidance()
{
 tTrackSeg *seg = car->_trkPos.seg;

 int id = tSegID[seg->id];

 if (id != lastsegid)
 {
  lastsegid = id;
  last_damage = car->_dammage;
  segupdated = 0;
  return;
 }

 if (segupdated)
  return;

 double lane2left = tLane[rl][nextdiv] * seg->width;

 if (!pit->getInPit()
     && last_damage == car->_dammage
     && ((nextleft < -1.0 && nextleft - fabs(angle)*2 < -1.5 && nextleft < lane2left - 1.0) 
      || (nextleft > seg->width+1.0 && nextleft + fabs(angle)*2 > seg->width + 1.5 && nextleft > lane2left + 1.0)))
 {
  float lftaddition = 0.0f, rgtaddition = 0.0f;
  int stype = seg->type, i;
  int div = nextdiv;

  if (stype == TR_STR)
  {
   for (i=1; i<(int)(car->_speed_x/3); i++)
   {
    int j = nextdiv - i;
    if (j < 0) j += (Divs+1);

    if (tSegment[tDivSeg[j]]->type != TR_STR)
    {
     if ((nextleft < -1.0 && tSegment[tDivSeg[j]]->type == TR_LFT) ||
         (nextleft > seg->width + 1.0 && tSegment[tDivSeg[j]]->type == TR_RGT))
      break;
     stype = tSegment[tDivSeg[j]]->type;
     div = j;
     break;
    }
   }
  }

  if (nextleft < -1.0)
  {
   if (stype != TR_RGT)
   {
    lftaddition = 3.0;
    tLSlowSpeed[div] += 0.3;
   }
   else
   {
    lftaddition = 3.0;
    tLSlowSpeed[div]++;
   }
  }
  else
  {
   if (stype != TR_LFT)
   {
    rgtaddition = 3.0;
    tRSlowSpeed[div] += 0.3;
   }
   else
   {
    rgtaddition = 3.0;
    tRSlowSpeed[div]++;
   }
  }

  for (i=0; i<80; i++)
  {
   int j = div - i;
   if (j < 0) j += (Divs+1);

   if (tSegment[tDivSeg[j]]->type != TR_LFT)
    tAvoidLeft[j] += lftaddition;
   if (tSegment[tDivSeg[j]]->type != TR_RGT)
    tAvoidRight[j] += rgtaddition;

   tAvoidBoost[j] = MIN(0.02, tAvoidBoost[j] + 0.02);
   //tSegRadius[j] *= 0.9;
   tAvoidLeft[j] = MIN(seg->width * 0.7, tAvoidLeft[j]);
   tAvoidRight[j] = MIN(seg->width * 0.7, tAvoidRight[j]);

   lftaddition = (float)MAX(0.0, lftaddition - (double)i / 23.0);
   rgtaddition = (float)MAX(0.0, rgtaddition - (double)i / 23.0);
  }

  segupdated = 1;
 }
}

double Driver::getSlowestSpeed(double distance)
{
 int inc = 1, looknext = nextdiv;
 double speed = car->_speed_x;

 while (inc * DivLength < distance)
 {
  speed = MIN(tSpeed[looknext], speed);
  looknext = (Divs + looknext + 1) % Divs;
  inc++;
 }

 return speed;
}

double Driver::getOffset()
{
 int i;
 side_count = 0;
 tTrackSeg *seg = car->_trkPos.seg;
 double lane2left = tLane[rl][nextdiv] * track->width;
 //double lane2right = seg->width - lane2left;
 double nextright = seg->width - nextleft;
 double tlane2left = tLane[rl][thisdiv] * track->width;
 double maxea = (segtype == TR_LFT || (prefer_side == TR_LFT && cornerdist<cornerlimit) ? EdgeAllowance*2 : EdgeAllowance);
 double minea = (segtype == TR_RGT || (prefer_side == TR_RGT && cornerdist<cornerlimit) ? EdgeAllowance*2 : EdgeAllowance);
 double maxoffset = (MAX(seg->width, tlane2left + 2.0) - MAX(car->_dimension_y * 1.0, maxea));
 double minoffset = (MIN(0.0, tlane2left - 2.0) + MAX(car->_dimension_y * 1.0, minea));
 maxoffset = MAX(maxoffset, tLane[rl][thisdiv]*seg->width);
 minoffset = MIN(minoffset, tLane[rl][thisdiv]*seg->width);
 if (segtype == TR_LFT)
  minoffset += 1.0;
 else if (segtype == TR_RGT)
  maxoffset -= 1.0;
#if 1
 FindPreferredSide();

 setAllowCorrecting(0);

 int next = nextdiv;
 double avoidright = tAvoidRight[nextdiv], avoidleft = tAvoidLeft[nextdiv];
 for (i=1; i<6; i++)
 {
  next = (Divs+nextdiv+i) % Divs;
  avoidright = MAX(avoidright, tAvoidRight[next]);
  avoidleft = MAX(avoidleft, tAvoidLeft[next]);
 }
 int nextnext = (next+(int)(car->_speed_x/7)+Divs)%Divs;
 tTrackSeg *nseg = tSegment[tDivSeg[next]];
 int nsegtype = SEGTYPE(nseg);
 double nnlane2left = tLane[rl][nextnext] * track->width;
 double nlane2left = tLane[rl][next] * track->width;

 double Tleft = car->_trkPos.toLeft;// + (car->_trkPos.toLeft - prevleft)/6;
 //if (fabs(tRInverse[nextdiv]) > 0.002 || segtype != TR_STR)
 // Tleft += (nextleft - car->_trkPos.toLeft) / 4;

 if (!start_finished)
 {
  if (avoiding)
   start_finished = 1;
  else if (getCorrecting())
  {
   if (car->_trkPos.toMiddle > 0.0 && nlane2left >= car->_trkPos.toLeft && nlane2left > tlane2left)
    start_finished = 1;
   else if (car->_trkPos.toMiddle < 0.0 && nlane2left <= car->_trkPos.toLeft && nlane2left < tlane2left)
    start_finished = 1;
  }
 }

#if 0
 if (getCorrecting() || getAllowCorrecting())
 {
  if (segtype == TR_LFT && nextleft < car->_trkPos.toLeft && nextleft < lane2left)
   Tleft += (car->_trkPos.toLeft-nextleft);
  else if (segtype == TR_RGT && nextleft > car->_trkPos.toLeft && nextleft > lane2left)
   Tleft -= (nextleft-car->_trkPos.toLeft);
 }
#endif
 //double ta = (angle < 0.0 ? MIN(0.0, angle+car->_yaw_rate*0.1) : MAX(0.0, angle-car->_yaw_rate*0.1));

 double startleft = Tleft;

#endif

 double cornertype = nseg->type;
 double cornerradius = tSegRadius[tDivSeg[next]];
 tTrackSeg *tseg = nseg->next;
 while (cornertype == TR_STR)
 {
  cornertype = tseg->type;
  cornerradius = tseg->radius;
  tseg = tseg->next;
 }

 double yaw_limit = (((0.5-CR)) * (CA)) * 2;
 if (yaw_limit < 0.0) yaw_limit *= 2;
 //double incfactor = OVERTAKE_OFFSET_INC * (5.5 - MIN(2.3, MIN(50.0, car->_speed_x) * 0.004)) * (0.51+yaw_limit*0.25);
 float incfactor = (float)((MaxIncFactor - MAX(MaxIncFactor/10, MIN(fabs(car->_speed_x*0.04), (MaxIncFactor-0.5)))) / 1.4);
#if K1999AVOIDSTEER
 //incfactor *= 0.8;
 incfactor = MAX(incfactor / 5, incfactor - (FCA/5 + MAX(0.0, CR-0.48)*2));
#endif
 incfactor *= (float)(2.0 + (car->_accel_x > 0.0 ? car->_accel_x / 10 : 0.0));
 if (racetype == RM_TYPE_QUALIF)
  incfactor = (float)(MaxIncFactor - MIN(fabs(car->_speed_x)/(MaxIncFactor), (MaxIncFactor-1.0)));

 if (car->_accel_x < 0.0)
  incfactor = (float)MAX(incfactor * 0.5, incfactor + car->_accel_x / 15);
 double leftinc = MAX(incfactor, tlane2left-nnlane2left), rightinc = MAX(incfactor, nnlane2left - tlane2left);
 double avoidleftinc = leftinc - MIN(tRInverse[LINE_RL][nextdiv]*100, -tRInverse[LINE_RL][nextdiv]*50);
 double avoidrightinc = rightinc + MAX(tRInverse[LINE_RL][nextdiv]*100, fabs(tRInverse[LINE_RL][nextdiv])*50);

#if 0
 if (Tleft < minoffset)
  minoffset = Tleft;
 if (Tleft > maxoffset)
  maxoffset = Tleft;
#endif


#if 0
 if (MIN(tlane2left, nlane2left) < nextleft && (tlane2left <= nlane2left || MAX(tlane2left, nlane2left) < nextleft - 3.0))
 {
  leftinc *= 1.2;
  rightinc *= 0.8;
 }
 else if (MAX(tlane2left, nlane2left) > nextleft && (tlane2left >= nlane2left || MIN(tlane2left, nlane2left) > nextleft + 3.0))
 {
  leftinc *= 0.8;
  rightinc *= 1.2;
 }
#else
 // THIS NEEDS TESTING!
#if 0
 if (segtype == TR_LFT || (seg->type == TR_LFT && seg->radius <= 400.0))
 {
  rightinc *= MAX(1.0, (6.0 - seg->radius / 100.0)/2);
 }
 else if (segtype == TR_RGT || (seg->type == TR_RGT && seg->radius <= 400.0))
 {
  leftinc *= MAX(1.0, (6.0 - seg->radius / 100.0)/2);
 }
#endif
#endif

#if 0
 if (segtype == TR_RGT && nextleft < avoidleft)
  leftinc *= 0.4f;
 if (segtype == TR_LFT && nextright < avoidright)
  rightinc *= 0.4f;
#endif

 if (car->_yaw_rate > 0.2)
 {
  rightinc = MIN(rightinc, MAX(rightinc * 0.2, (rightinc * (1.0 - car->_yaw_rate*0.75))/2));
 }
 else if (car->_yaw_rate < -0.2)
 {
  leftinc = MIN(leftinc, MAX(leftinc * 0.2, (leftinc * (1.0 - fabs(car->_yaw_rate)*0.75))/2));
 }

 sidemargin = 1000;
 if (avoiding)
  avoiding = AVOID_HOLD;
 int front_avoid_allowed = 1;//(currentsimtime > 5.0);
 int this_allow_correcting = 0;
 double oppmin = seg->width+2.0;
 double oppmax = -2.0;

 double lowestrad = 1000.0;
 for (i=0; i<MAX(4, (int) (car->_speed_x/DivLength)); i++)
 {
  int div = (nextdiv+i) % Divs;
  tTrackSeg *lseg = tSegment[tDivSeg[div]];
  if (SEGTYPE(lseg) != TR_STR)
   lowestrad = MIN(lseg->radius, lowestrad);
 }
 if (lowestrad < 25.0 && getSpeed() > 12.0)
  front_avoid_allowed = 0;

 // process opponents beside us
 for (i = 0; i < opponents->getNOpponents(); i++) 
 {
  if (opponent[i].car->_state >= RM_CAR_STATE_PIT) continue;

  tCarElt *otherCar = opponent[i].car;
  if ((otherCar == car))
   continue;

  if (otherCar->_trkPos.toRight < minoffset - 3.0 || otherCar->_trkPos.toLeft > maxoffset+3.0)
   continue;

  if (!(opponent[i].getState() & OPP_SIDE))
   continue;

  if (car->_trkPos.toLeft < otherCar->_trkPos.toLeft)
   side_count |= AVOID_RIGHT;
  else
   side_count |= AVOID_LEFT;
  
  if (segtype != TR_STR && seg->radius <= 30.0 && (opponent[i].getState() & OPP_SIDE_FRONT))
  {
   continue;
  }

#if 0
  if (!(opponent[i].getState() & OPP_SIDE_COLL) && 
      fabs(otherCar->_trkPos.toLeft - car->_trkPos.toLeft) - (opponent[i].getWidth()/2 + getWidth()/2) > 4.0 &&
      (fabs(nextleft - nlane2left) < 3.0 || fabs(otherCar->_trkPos.toLeft - nlane2left) > fabs(nextleft - nlane2left)))
   continue;
#endif

  // adjust our boundaries for how far we can move
  int closing = opponent[i].getClosing();
  double width = opponent[i].getWidth();
  //double dspd = getSpeed() - opponent[i].getSpeed();
  double thisoppmax;
  double thisoppmin;
  if (segtype != TR_STR)
  {
   if (segtype == TR_RGT)
   {
    thisoppmax = opponent[i].getNextLeft() + width/2 + getWidth()/2 + 2.0 + (140.0-seg->radius) / (closing == CLOSING_FAST ? 10 : 40);
    thisoppmin = opponent[i].getNextLeft() - (width/2 + getWidth()/2 + 1.0 + (140.0-seg->radius) / 80);
   }
   else
   {
    thisoppmax = opponent[i].getNextLeft() + width/2 + getWidth()/2 + 1.0 + (140.0-seg->radius) / 80;
    thisoppmin = opponent[i].getNextLeft() - (width/2 + getWidth()/2 + 2.0 + (140.0-seg->radius) / (closing == CLOSING_FAST ? 10 : 40));
   }
  }
  else
  {
   thisoppmax = opponent[i].getNextLeft() + width/2 + getWidth()/2 + 1.0;
   thisoppmin = opponent[i].getNextLeft() - (width/2 + getWidth()/2 + 1.0);
  }

#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"%s: SIDE %s %d %.3f/%.3f (%.3f|%.3f %.3f|%.3f)\n",car->_name,otherCar->_name,(opponent[i].getState() & OPP_COLL), Tleft,car->_trkPos.toLeft,oppmin,thisoppmin,oppmax,thisoppmax);
#endif


  if (car->_trkPos.toLeft < otherCar->_trkPos.toLeft)
  {
   // opponent to the right of me
   if (thisoppmin > oppmin)
   {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"side bail a (%.3f > %.3f)\n",thisoppmin,oppmin);
#endif
    continue;
   }
   oppmin = MIN(thisoppmin, oppmin);
#ifdef DEBUGMSG
fprintf(stderr,"%s: RSIDE %s %.3f/%.3f (%.3f/%.3f)\n",car->_name,otherCar->_name,oppmin,oppmax,thisoppmin,thisoppmax);
#endif

#if 0
   if (!closing && (nextleft+getWidth()/2+4.0) < oppmin && nlane2left < car->_trkPos.toLeft)
    continue;
#endif

   maxoffset = MIN(maxoffset, oppmin);
   avoiding |= (AVOID_FRONT | AVOID_RIGHT);
   if ((opponent[i].getTeam() & TEAM_CONTROL))// && !(opponent[i].getState() & OPP_SIDE_FRONT))
    avoiding |= AVOID_TEAM;
   if ((opponent[i].getState() & OPP_LETPASS))
    avoiding |= AVOID_LETPASS;
   if (avoiding & AVOID_LEFT)
   {
    this_allow_correcting = 0;
   }
   //double close_ratio = 0.45;
   double ratio = ((nextleft < oppmin || segtype == TR_RGT) ? 0.6 : 1.0);
   if ((opponent[i].getState() & OPP_SIDE_FRONT))
    ratio = 0.45;
   if (ratio > 1.0)
   {
    leftinc *= ratio;
    ratio = 1.0;
   }
   //sidemargin = MAX(0.0, (sidemargin, oppmin-nextleft));
   sidemargin = MAX(0.0, MIN(sidemargin, fabs(car->_trkPos.toLeft - opponent[i].car->_trkPos.toLeft) - (getWidth()/2 + opponent[i].getWidth()/2 + 1.0)));
   //double Roffset = car->_trkPos.toLeft + getWidth()/2;
   if (segtype != TR_STR)
    sidemargin /= 2;

   //if (opponent[i].getState() & OPP_SIDE_COLL)
   // ratio *= 1.2;

   //if (sidemargin < 10.0 || seg->type == TR_RGT)
#if 1
   if ((opponent[i].getState() & OPP_SIDE_COLL) || sidemargin < 3.0)
   {
    //Tleft -= leftinc * ratio;
    if (sidemargin < 1.0 || (opponent[i].getState() & OPP_SIDE_COLL))
     avoiding |= AVOID_SIDE_COLL;
   }
#endif

   // see if we can use the raceline to help us
   if (!(avoiding & AVOID_LEFT) && 
       ((MAX(tlane2left, lane2left) <= MIN(car->_trkPos.toLeft, nextleft)
         && MAX(tlane2left, lane2left) <= oppmin-((opponent[i].getState() & OPP_SIDE_COLL) ? 8.0 : 6.0)) ||
        (MAX(tlane2left, lane2left) < oppmin - ((opponent[i].getState() & OPP_SIDE_COLL) ? 8.0 : 6.0))))
   {
    this_allow_correcting = 0;
   }
   else
   {
    this_allow_correcting = 0;
    // omigosh - new!
    //if (!(avoiding & AVOID_LEFT) && nextleft < oppmin - 2.0 && nlane2left > tlane2left)
    // Tleft += MIN(rightinc/2, nlane2left - tlane2left);
   }

   if (!(side_count & AVOID_LEFT) &&
       ((opponent[i].getState() & OPP_COLL) || sidemargin < 2.0 + fabs(tRInverse[LINE_RL][nextdiv]) * 500))
   {
    Tleft -= avoidleftinc;
   }
   else if (tlane2left > nextleft && tRInverse[LINE_RL][nextdiv] < 0.0005 && nlane2left > tlane2left && !(opponent[i].getState() & OPP_COLL) && sidemargin > 5.0)
   {
    Tleft += avoidrightinc * 0.2;
   }
  }
  else
  {
#ifdef DEBUGMSG
fprintf(stderr,"%s: LSIDE %s %.3f/%.3f (%.3f/%.3f)\n",car->_name,otherCar->_name,oppmin,oppmax,thisoppmin,thisoppmax);
#endif
   if (thisoppmax < oppmax)
   {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"side bail b (%.3f < %.3f)\n",thisoppmax,oppmax);
#endif
    continue;
   }
   oppmax = MAX(thisoppmax, oppmax);

#if 0
   if (!closing && (nextleft-(getWidth()/2+4.0)) > oppmax && nlane2left > car->_trkPos.toLeft)
    continue;
#endif

   minoffset = MAX(minoffset, oppmax);
   avoiding |= (AVOID_FRONT | AVOID_LEFT);
   if ((opponent[i].getTeam() & TEAM_CONTROL))
    avoiding |= AVOID_TEAM;
   if ((opponent[i].getState() & OPP_LETPASS))
    avoiding |= AVOID_LETPASS;
   if (avoiding & AVOID_RIGHT)
   {
    this_allow_correcting = 0;
   }
   //double close_ratio = 0.45;
   double ratio = ((nextleft > oppmax || segtype == TR_LFT) ? 0.6 : 1.0);
   if ((opponent[i].getState() & OPP_SIDE_FRONT))
    ratio = 0.45;
   if (ratio > 1.0)
   {
    rightinc *= ratio;
    ratio = 1.0;
   }
   //sidemargin = MAX(0.0, MIN(sidemargin, nextleft-oppmax));
   sidemargin = MAX(0.0, MIN(sidemargin, fabs(car->_trkPos.toLeft - opponent[i].car->_trkPos.toLeft) - (getWidth()/2 + opponent[i].getWidth()/2 + 1.0)));
   //double Loffset = car->_trkPos.toLeft - getWidth()/2;
   if (segtype != TR_STR)
    sidemargin /= 2;

   //if (opponent[i].getState() & OPP_SIDE_COLL)
   // ratio *= 1.2;

   //if (sidemargin < 10.0 || seg->type == TR_LFT)
#if 1
   if ((opponent[i].getState() & OPP_SIDE_COLL) || sidemargin < 3.0)
   {
    //Tleft += rightinc * ratio;
    if (sidemargin < 1.0 || (opponent[i].getState() & OPP_SIDE_COLL))
     avoiding |= AVOID_SIDE_COLL;
   }
#endif

   if (!(avoiding & AVOID_RIGHT) && 
       ((MIN(tlane2left, lane2left) >= MAX(car->_trkPos.toLeft, nextleft)
         && MIN(tlane2left, lane2left) >= oppmax+((opponent[i].getState() & OPP_SIDE_COLL) ? 8.0 : 6.0)) ||
        (MIN(tlane2left, lane2left) > oppmax + ((opponent[i].getState() & OPP_SIDE_COLL) ? 8.0 : 6.0))))
   {
    this_allow_correcting = 0;
   }
   else
   {
    this_allow_correcting = 0;
   }

   if (!(side_count & AVOID_RIGHT) &&
       ((opponent[i].getState() & OPP_COLL) || sidemargin < 2.0 + fabs(tRInverse[LINE_RL][nextdiv]) * 500))
   {
    Tleft += avoidrightinc;
   }
   else if (tlane2left < nextleft && tRInverse[LINE_RL][nextdiv] > -0.0005 && nlane2left < tlane2left && !(opponent[i].getState() & OPP_COLL) && sidemargin > 5.0)
   {
    Tleft -= avoidleftinc * 0.2;
   }
  }
 }

 if (minoffset > maxoffset)
  minoffset = maxoffset = (minoffset+maxoffset)/2;
 Tleft = MAX(minoffset, MIN(maxoffset, Tleft));

 if (!avoiding || avoiding == AVOID_HOLD)
 {
  // if a team-member is right in front of us and avoiding, copy them
  for (i = 0; i < opponents->getNOpponents(); i++) 
  {
   if (opponent[i].car->_state >= RM_CAR_STATE_PIT) continue;

   if (opponent[i].getTeam() != TEAM_CONTROL) continue;

   //double time_behind = opponent[i].getBrakeDistance() / car->_speed_x;
   tCarElt *ocar = opponent[i].car;

   if (opponent[i].getBrakeDistance() < -1.5) break;

   if (/*time_behind > 1.3 ||*/ opponent[i].getBrakeDistance() > car->_speed_x * 0.4)
    break;

   if (telemetry[i] == 0) break;
   if (((telemetry[i] & AVOID_RIGHT) && MIN(car->_trkPos.toLeft, nextleft) <= ocar->_trkPos.toLeft + 0.2) ||
       ((telemetry[i] & AVOID_LEFT) && MAX(car->_trkPos.toLeft, nextleft) >= ocar->_trkPos.toLeft - 0.2))
    break;

   if (telemetry[i] & AVOID_BACK)
    break;

#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"AVOID TEAM\n");
#endif
   avoiding |= (AVOID_FRONT | AVOID_TEAM);
   sidemargin = 0;

   if (telemetry[i] & AVOID_RIGHT)
   {
    avoiding |= AVOID_RIGHT;
    Tleft = MAX(Tleft - avoidleftinc, MIN(nextleft, ocar->_trkPos.toLeft));
   }
   else
   {
    avoiding |= AVOID_LEFT;
    Tleft = MIN(Tleft + avoidrightinc, MAX(nextleft, ocar->_trkPos.toLeft));
   }
   //opponent[i].setState(OPP_COLL);

   break;
  }
 }

 //double closest_dist = FLT_MAX;
 int favoiding = 0;
 //tTrackSeg *cseg = getNextCornerSeg();

 if (front_avoid_allowed)
 {
  double yr = fabs(car->_yaw_rate - laststeer/4) * (car->_accel_x < 0.0 ? MAX(1.0, fabs(car->_accel_x)/4) : 1.0) + (car->_accel_x < 0.0 ? fabs(car->_accel_x) / 10 : 0.0);
  front_avoid_allowed = (!(avoiding & (AVOID_SIDE|AVOID_FRONT)) && (old_avoiding || (fabs(laststeer) < 0.7 && yr < 1.0)));
 }

 if (front_avoid_allowed)
 {
  double catchdist = 0, mincatchdist=1000.0;
  //double mindist=-1000.0;
  Opponent *o = NULL;
  double avoidspeed = getAvoidSpeed(nextleft);
  double myspeed = MIN(getSpeed() + 2.0, (getSpeed()*2 + avoidspeed)/3);

  // now process front avoidance
  for (i = 0; i < opponents->getNOpponents(); i++) 
  {
   tCarElt *otherCar = opponent[i].car;
   if ((otherCar == car))
   {
    // can't overtake myself
    continue;
   }

   if (opponent[i].car->_state >= RM_CAR_STATE_PIT) 
   {
    // don't overtake pitted cars
    continue;
   }

   if ((opponent[i].getTeam() & TEAM_CONTROL) && 
       (car->_speed_x > otherCar->_speed_x + 8.0 ||
        (fabs(opponent[i].getAngle()) < 1.0 && 
         otherCar->_speed_x > 15.0 && 
         ((otherCar->_pos < car->_pos && otherCar->_dammage < car->_dammage + 500) ||
          (otherCar->_pos > car->_pos && otherCar->_dammage <= car->_dammage)))))
   {
    // don't overtake team members unless damaged or out of control
    continue;
   }
 
   //double distance = opponent[i].getBrakeDistance() + 2.0;
 
   if (otherCar->_trkPos.toLeft < -3.0f || otherCar->_trkPos.toRight < -3.0f)
   {
    // don't overtake if off the track
    continue;
   }
 
   if (!(opponent[i].getState() & OPP_FRONT))
   {
    if (opponent[i].getDistance() > 100.0 || opponent[i].getDistance() < 0.0)
    {
     // don't overtake if not in front
     continue;
    }
   }

   tTrackSeg *oseg = otherCar->_trkPos.seg;
   int osegtype = SEGTYPE(oseg);

#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"%s - %d %.3f %.3f\n",otherCar->_name,(opponent[i].getState()), opponent[i].getDistance(), opponent[i].getBrakeDistance());
#endif

   {
    // we must be closer or approaching faster if on a tight curve
    double osegradius = (osegtype != TR_STR ? oseg->radius : 10000.0);
    double segradius = MIN(osegradius, (segtype != TR_STR ? seg->radius : 10000.0));
    //double myspeed = MAX(getSpeed(), (getSpeed() + MIN(tSpeed[nextdiv], tSpeed[nextdiv])) / 2);
    double t_impact = MAX(0.0, MAX(opponent[i].getTimeImpact() * 0.7, opponent[i].getBrakeDistance() / (myspeed - opponent[i].getSpeed())));
    double factor = 1.0;
    if (cornerdist > cornerlimit && segtype == TR_STR)
     factor = 2.0;
    if (otherCar->_pos > car->_pos)
     factor *= 1.5;

    if (opponent[i].getTimeImpact() > 2.0 * factor && (segtype != TR_STR || car->_pos > otherCar->_pos || opponent[i].getBrakeDistance() > 4.0))
    {
     if (t_impact > (segtype == TR_STR ? 10.0 : 4.0) && 
         opponent[i].getBrakeDistance() > (segtype == TR_STR ? 7.0 : 3.5))
     {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"%s - bail a\n",otherCar->_name);
#endif
      continue;
     }
     else if (segtype != TR_STR && segradius < 10000.0)
     {
      if (t_impact > (4.0 - (1.0 - (segradius/125.0)) * 3))
      {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"%s - bail b\n",otherCar->_name);
#endif
       continue;
      }
     }
    }
   }

   {
    // don't overtake if its faster than us
    double ospeed = opponent[i].getSpeed()+otherCar->_accel_x * 0.5;
    int cancel_avoid = 0;
    double factor = 1.0;
    if (cornerdist > cornerlimit && segtype == TR_STR)
     factor = 2.6;

    if (otherCar->_pos > car->_pos && segtype != TR_STR)
     ospeed -= 1.0;

    if (segtype != TR_STR)
    {
     if (seg->type == TR_LFT)
     {
      myspeed -= (MAX(0.0, MIN(1.0, car->_trkPos.toLeft / seg->width)) / seg->radius) * 5.0;
      ospeed  -= (MAX(0.0, MIN(1.0, otherCar->_trkPos.toLeft / seg->width)) / seg->radius) * 5.0;
     }
     else
     {
      myspeed -= (MAX(0.0, MIN(1.0, 1.0 - car->_trkPos.toLeft / seg->width)) / seg->radius) * 5.0;
      ospeed  -= (MAX(0.0, MIN(1.0, 1.0 - otherCar->_trkPos.toLeft / seg->width)) / seg->radius) * 5.0;
     }
    }

    if ((opponent[i].getBrakeDistance() < 2.0*factor && ospeed > myspeed) ||
        (opponent[i].getBrakeDistance() >= 2.0*factor && ospeed > myspeed))
    {
     if (cancel_avoid)
      avoid_hold = 0;
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"%s - bail c sp=%.3f os=%.3f bd=%.3f\n",otherCar->_name,getSpeed(),ospeed,opponent[i].getBrakeDistance());
#endif
     continue;
    }

#if 0
    if (osegtype != TR_STR)
    {
     double mu = oseg->surface->kFriction;
     double radius = tSegRadius[oseg->id];
     double speed1 = sqrt((mu * G * radius * 3.0) / (1.0 - MIN(1.0, radius * CA * mu/(mass + MAX(0.0, CARMASS-1000.0)))));
     double delta = tLDelta[oseg->id] * 0.5 + tRDelta[oseg->id] * 0.5;
  
     if (delta < -0.03)
      speed1 -= MIN(speed1/3, fabs(delta) * 25);

     if (speed1 < otherCar->_speed_x + (otherCar->_pos < car->_pos ? 2.0 : 0.0))
     {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"%s - bail c2\n",otherCar->_name);
#endif
      continue;
     }
    }
#endif
   }

   // if there's a corner between us, don't overtake
   if (opponent[i].car->_trkPos.seg->type != seg->type && opponent[i].getBrakeDistance() > 5.0)
   {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"%s - bail c3\n",otherCar->_name);
#endif
    continue;
   }
   else if (opponent[i].car->_trkPos.seg != seg)
   {
    tTrackSeg *tseg = seg->next;
    while (tseg != opponent[i].car->_trkPos.seg)
    {
     if (tseg->type != seg->type && tseg->type != opponent[i].car->_trkPos.seg->type)
      break;
     tseg = tseg->next;
    }

    if (tseg != opponent[i].car->_trkPos.seg)
    {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"%s - bail c4\n",otherCar->_name);
#endif
     continue;
    }
   }

   double dspd = myspeed - otherCar->_speed_x;

   if (opponent[i].getBrakeDistance() > dspd*1.3 + 2.0 && opponent[i].getBrakeDistance() / car->_speed_x > 0.2)
   {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"%s - bail c3\n",otherCar->_name);
#endif
    continue;
   }

   if (osegtype != TR_STR && seg != oseg)
   {
    if (segtype != osegtype && opponent[i].getBrakeDistance() > oseg->radius/(otherCar->_pos < car->_pos ? 10 : 5))
    {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"%s - bail d1\n",otherCar->_name);
#endif
     continue;
    }
    else 
    {
     tTrackSeg *tseg = seg->next;
     while (tseg != oseg)
     {
      if (tseg->type != oseg->type)
       break;
      tseg = tseg->next;
     }

     if (tseg != oseg)
     {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"%s - bail d2\n",otherCar->_name);
#endif
      continue;
     }
    }
   }

   //if ((opponent[i].getState() & OPP_FRONT) && distance < 200.0 && 
   //    opponent[i].getTimeImpact() > opponent[i].getBrakeDistance() / (car->_pos > otherCar->_pos ? 4 : 2))
   {
    // don't overtake if we won't catch him before corner
    double distance = opponent[i].getBrakeDistance()+2.0;
    double ospeed = otherCar->_speed_x;
#if 0
    // if we're on the outside of the corner, adjust his speed comparatively
    if ((segtype == TR_LFT && nextleft > otherCar->_trkPos.toLeft) ||
        (segtype == TR_RGT && nextleft < otherCar->_trkPos.toLeft))
     ospeed += fabs(nextleft-otherCar->_trkPos.toLeft) * 0.75;
#endif
    
    catchdist = MIN(500.0, (myspeed*distance)/MAX(0.1, myspeed-ospeed));
    if (otherCar->_pos > car->_pos)
     catchdist /= 5;

    if (segtype == TR_STR && cornerdist*0.8 < catchdist && //car->_pos > otherCar->_pos && 
        ((cseg->type == TR_LFT && car->_trkPos.toLeft < otherCar->_trkPos.toLeft) ||
         (cseg->type == TR_RGT && car->_trkPos.toLeft > otherCar->_trkPos.toLeft)))
    {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"%s - bail d\n",otherCar->_name);
#endif
     continue;
    }

    //double radius = (seg->type != TR_STR ? seg->radius : (cornerdist < cornerlimit/2 && cseg != NULL ? cseg->radius : 10000.0));
    //double oradius = (otherCar->_trkPos.seg->type != TR_STR ? otherCar->_trkPos.seg->radius : 10000.0);
    if (catchdist < mincatchdist)// && catchdist < MIN(limit1, MIN(radius, oradius)/limit2) && catchdist >= 0.0 && distance < 80.0)
    {
     mincatchdist = catchdist;
     o = &opponent[i];
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"%s - OVERTAKE!\n",otherCar->_name);
#endif
    }
    else
    {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"%s - bail g\n",otherCar->_name);
#endif
    }
   }
  }

  if (o != NULL)
  {
   double distance = o->getBrakeDistance()+2.0;
   double speedcutoff = (segtype == TR_STR ? 0.5 : MAX(0.0, (120.0 - tSegRadius[seg->id])/40));
   frontmargin = o->getBrakeDistance();
   //double dspd = getSpeed() - o->getSpeed();//car->_speed_x - o->car->_speed_x;
   double t_impact = MIN(4.0, o->getTimeImpact());
   double Oleft = o->car->_trkPos.toLeft;
   //double ONleft = Oleft + (o->getNextLeft() - o->car->_trkPos.toLeft) * (t_impact/deltaTime);//o->getNextLeft();
   //double Nleft = car->_trkPos.toLeft + (car->_trkPos.toLeft - prevleft) * (t_impact/deltaTime);
   tTrackSeg *oseg = o->car->_trkPos.seg;
   tTrackSeg *tseg = oseg;
   if (tseg->type == TR_STR || (seg->type != TR_STR && seg->radius < tseg->radius))
    tseg = seg;
   //double wdiff = (car->_trkPos.toLeft-Oleft);
   double nwdiff = fabs((car->_trkPos.toLeft + (car->_trkPos.toLeft - prevleft) * (t_impact/deltaTime)) - 
                        (o->car->_trkPos.toLeft + (o->getNextLeft() - o->car->_trkPos.toLeft) * (t_impact/deltaTime)));
   //double ospeed = o->getSpeed();
   //int onext = (thisdiv + (int) (distance/DivLength)) % Divs;

   favoiding = 0;
   Tleft = startleft;
   double ratio = 1.0;//(nseg->type == TR_STR ? 1.5 : 0.7);
   //double avoidwidth = MAX(o->car->_dimension_y, o->getWidth()) + 0.5;

   double width = MAX(o->car->_dimension_y, o->getWidth());
   double thisoppmax;
   double thisoppmin;
   if (seg->type != TR_STR)
   {
    thisoppmax = o->getNextLeft() + width/2 + car->_dimension_y/2 + 2.0;
    thisoppmin = o->getNextLeft() - (width/2 + car->_dimension_y/2 + 2.0);
   }
   else
   {
    thisoppmax = o->getNextLeft() + width/2 + car->_dimension_y/2 + 2.0;
    thisoppmin = o->getNextLeft() - (width/2 + car->_dimension_y/2 + 2.0);
   }
 
   oppmax = MAX(oppmax, thisoppmax);
   oppmin = MIN(oppmin, thisoppmin);

   double avoidspeedR = getAvoidSpeed(thisoppmax);
   double avoidspeedL = getAvoidSpeed(thisoppmin);
   /*
   if (segtype == TR_LFT)
    rightinc *= 0.7;
   else if (segtype == TR_RGT)
    leftinc *= 0.7;
    */

#ifdef OVERTAKE_DEBUGMSG
fprintf(stderr,"%s: %s (%.3f,%.3f > %.3f (%.3f/%.3f)\n",car->_name, o->car->_name,o->getTimeImpact()*4,(distance-2)/2,fabs(nwdiff),nextleft,ONleft);
#endif

   //if (Nleft > ONleft)
   if (car->_trkPos.toLeft > o->car->_trkPos.toLeft)
   {
    // on his right
    //int allow_switch = ((prefer_side == TR_LFT && ONleft > (car->_dimension_y + avoidwidth + 6.0 + tAvoidLeft[next]) && (segtype != TR_STR || cornerdist < 100.0) && o->getTimeImpact() > 0.5) 
    if (distance < 4.0 && cseg != NULL && (segtype!=TR_STR || cornerdist<cornerlimit*0.75) 
        && seg->width-Oleft < car->_dimension_y*2 + MAX(0.0, 130.0-cseg->radius)/20)
    {
     o->setState(OPP_COLL);
    }

    if (sidemargin == 1000.0)
     sidemargin = MAX(0.0, fabs(car->_trkPos.toLeft - o->car->_trkPos.toLeft) - (getWidth()/2 + o->getWidth()/2 + 1.0));

    int prevent_switch = (side_count || (car->_speed_x - o->car->_speed_x)/4 * MAX(0.0, nwdiff) > o->getBrakeDistance()*2);//(MAX(o->getTimeImpact()*4, (distance-2)/2) < fabs(nwdiff));
    int try_allow_switch = 0, allow_switch = 0;
    {
     double onl = o->car->_trkPos.toLeft + (o->getNextLeft() - o->car->_trkPos.toLeft) * (t_impact/deltaTime*0.75);
     double mnl = car->_trkPos.toLeft + (car->_trkPos.toLeft - prevleft) * (t_impact/deltaTime*0.75);
     double margin = (segtype == TR_RGT ? 125.0 - seg->radius / 20.0 : 0.0);

     double cd = fabs(car->_trkPos.toLeft - o->car->_trkPos.toLeft);
     double mrightmove = nextleft - car->_trkPos.toLeft;
     double orightmove = o->getNextLeft() - o->car->_trkPos.toLeft;

     if ((mnl <= onl && onl > o->car->_dimension_y + car->_dimension_y + 2) || 
         (cd < 2.0 && orightmove > mrightmove * 1.2 && orightmove > 0.1) ||
         (onl + o->getWidth() > seg->width - (car->_dimension_y + 2 + margin)))// && mnl <= car->_trkPos.toLeft + 1.0))
      try_allow_switch = 1;
    }
    if (!prevent_switch)
     allow_switch = try_allow_switch;

    if (allow_switch)
    {
     sidemargin = 0;
     // switch to left side of him
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"SWITCH TO LEFT SIDE ");
#endif
#if 0
     if (ONleft < (tAvoidLeft[next] + 3.0) ||
         (t_impact >= 1.0 && fabs(nextleft+(nextleft-car->_trkPos.toLeft)*(t_impact/deltaTime) - (ONleft+(ONleft-Oleft)*((t_impact/deltaTime)/2))) > 4.0))
     {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr," - BAILING\n");
#endif
      goto frontavoid_end;
     }
#endif

     if (prefer_side == TR_LFT)
      speedcutoff = -0.5;
     else if (prefer_side == TR_RGT || seg->type == TR_RGT)
      speedcutoff *= 1.5;
     if (o->car->_pos < car->_pos && avoidspeedL < o->getSpeed()+speedcutoff)
     {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr," - BAILING\n");
#endif
      goto frontavoid_end;
     }

     /*
     if (prefer_side == TR_RGT && avoidspeed < getSpeed() - 3)
     {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr," - BAILING\n");
#endif
      goto frontavoid_end;
     }
     */

#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"\n");
#endif
     Tleft -= avoidleftinc * 1.5;
     favoiding = (AVOID_FRONT | AVOID_RIGHT | AVOID_SWITCH);
#if 0
     if (MAX(tlane2left, lane2left) < oppmin - 1.0 || MAX(tlane2left, lane2left) <= Tleft-1.0)
     {
      this_allow_correcting = 1;
     }
#endif
    }
    else
    {
     // stick to right.
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"STICK TO RIGHT SIDE, PS=%d sm=%.3f ",prevent_switch,sidemargin);
#endif
     if (try_allow_switch && fabs(nwdiff) < getWidth()/2 + o->getWidth()/2 + 1.0)
      o->setState(OPP_COLL);

     //if (nextleft > seg->width-tAvoidRight[nextnext] && car->_speed_x > tSlowSpeed[nextnext]+4.0)
#if 0
     if (((prefer_side == TR_LFT && cornerdist < cornerlimit*0.7) || (segtype == TR_LFT && tMaxSpeed[nextdiv] <= tMaxSpeed[thisdiv]))
         && nextleft > seg->width - tAvoidRight[nextnext] 
         && MAX(ospeed, getSpeed()-5) > tSpeed[nextdiv] + (o->car->_pos < car->_pos ? 2.0 : 6.0))
     {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr," - BAILING1\n");
#endif
      goto frontavoid_end;
     }
     if ((getSpeed() + tSpeed[nextnext])/2 < o->getSpeed())
     {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr," - BAILING2\n");
#endif
      goto frontavoid_end;
     }
#endif

     if (((segtype == TR_LFT || (prefer_side == TR_LFT && cornerdist < cornerlimit)) 
         && (MIN(o->car->_speed_x, getSpeed()) > avoidspeedR+1.0 && nextright < (MIN(o->car->_speed_x, (getSpeed()) - avoidspeedR) * 3))) ||
            (tRInverse[LINE_RL][nextdiv] > 0.002 && t_impact > tRInverse[LINE_RL][nextdiv] * 150))
     {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr," - BAILING3\n");
#endif
      goto frontavoid_end;
     }
     if (prefer_side == TR_RGT || prefer_side == TR_STR)
      speedcutoff = -1.5;
     else if (prefer_side == TR_LFT || seg->type == TR_LFT)
      speedcutoff *= 1.5;
     if (o->car->_pos < car->_pos && avoidspeedR < o->getSpeed()+speedcutoff)
     {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr," - BAILING4\n");
#endif
      goto frontavoid_end;
     }
     /*
     if (prefer_side == TR_LFT && avoidspeed < getSpeed() - 3)
     {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr," - BAILING\n");
#endif
      goto frontavoid_end;
     }
     */

#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"\n");
#endif

     sidemargin = MAX(0.0, nextleft-oppmax);
     if (fabs(nextleft - o->getNextLeft()) < fabs(car->_trkPos.toLeft - o->car->_trkPos.toLeft)*0.9)
      sidemargin = MAX(0.0, sidemargin - (fabs(car->_trkPos.toLeft - o->car->_trkPos.toLeft) - fabs(nextleft - o->getNextLeft()))*20);
     if (segtype != TR_STR)
      sidemargin /= 2;

     //double spdmargin = dspd * 2 - distance;
#if 0
     if (MIN(tlane2left, lane2left) > oppmax + (2.0 + spdmargin))
     {
      this_allow_correcting = 1;
     }
#endif
     if (!(side_count & AVOID_RIGHT) &&
         ((o->getState() & OPP_COLL) || sidemargin < 2.0) &&
         (1 ||
          (o->getState() & OPP_COLL) ||
          (seg->type != TR_LFT && nextleft < seg->width * 0.5) ||
          (Tleft < tlane2left - 3.0 && tlane2left < nlane2left)))
      /*
         (seg->type != TR_STR && seg->radius <= 140.0) ||
         ((Tleft < oppmax+(seg->type != TR_STR ? 5.0 : 3.0) || (prefer_side == TR_RGT && (lane2left > nextleft || nextleft < seg->width * 0.4)))
          && (1 || Tleft < tlane2left || segtype == TR_RGT || prefer_side != TR_RGT || Tleft < seg->width * 0.65)))
          */
     {
      Tleft += avoidrightinc * ratio;
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr," B1 %d sm=%.3f coll=%d Tl=%.3f\n",this_allow_correcting,sidemargin,(o->getState() & OPP_COLL),Tleft);
#endif
     }
     else if (tlane2left <= nextleft && !(o->getState() & OPP_COLL) && sidemargin > 5.0)
     {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr," B2 %d sm=%.3f coll=%d\n",this_allow_correcting,sidemargin,(o->getState() & OPP_COLL));
#endif
      Tleft -= avoidleftinc * 0.6;
     }
     // omigosh - new!
     //else if (nextleft > oppmax + 2.0 && nlane2left < tlane2left)
     // Tleft -= MIN(leftinc/2, tlane2left - nlane2left);
#if 0
     else if (nextleft > oppmax+(seg->type == TR_LFT ? 3.0 : 0.5) && tlane2left < nextleft-1.0)
     {
      Tleft -= leftinc*0.35;
#ifdef DEBUGMSG
fprintf(stderr," B2 %d\n",this_allow_correcting);
#endif
     }
#endif
     favoiding = (AVOID_FRONT | AVOID_LEFT);
    }
   }
   else
   {
    // on his left
    if (distance < 4.0 && cseg != NULL && (segtype!=TR_STR || cornerdist<cornerlimit*0.75) 
        && Oleft < car->_dimension_y*2 + MAX(0.0, 130.0-cseg->radius)/20)
    {
     o->setState(OPP_COLL);
    }

    if (sidemargin == 1000.0)
     sidemargin = MAX(0.0, fabs(car->_trkPos.toLeft - o->car->_trkPos.toLeft) - (getWidth()/2 + o->getWidth()/2 + 1.0));

    int prevent_switch = (side_count || (car->_speed_x - o->car->_speed_x)/4 * fabs(MIN(0.0, nwdiff)) > o->getBrakeDistance()*2);//(MAX(o->getTimeImpact()*4, (distance-2)/2) < fabs(nwdiff));
    int try_allow_switch = 0, allow_switch = 0;
    {
     double onl = o->car->_trkPos.toLeft + (o->getNextLeft() - o->car->_trkPos.toLeft) * (t_impact/deltaTime*0.75);
     double mnl = car->_trkPos.toLeft + (car->_trkPos.toLeft - prevleft) * (t_impact/deltaTime*0.75);
     double margin = (segtype == TR_LFT ? 125.0 - seg->radius / 20.0 : 0.0);
     double cd = fabs(car->_trkPos.toLeft - o->car->_trkPos.toLeft);
     double mleftmove = car->_trkPos.toLeft - nextleft;
     double oleftmove = o->car->_trkPos.toLeft - o->getNextLeft();

     if ((mnl >= onl && onl < seg->width - (o->car->_dimension_y + car->_dimension_y + 2)) || 
         (cd < 2.0 && oleftmove > mleftmove * 1.2 && oleftmove > 0.1) ||
         (onl - o->getWidth() < (car->_dimension_y + 2 + margin)))// && mnl >= car->_trkPos.toLeft - 1.0))
      try_allow_switch = 1;
    }
    if (!prevent_switch)
     allow_switch = try_allow_switch;

    if (allow_switch)
    {
     // switch to right side of him
     sidemargin = 0;
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"SWITCH TO RIGHT SIDE");
#endif
#if 0
     if (ONleft > seg->width - (tAvoidRight[next] + 3.0) ||
         (t_impact >= 1.0 && fabs(nextleft+(nextleft-car->_trkPos.toLeft)*(t_impact/deltaTime) - (ONleft+(ONleft-Oleft)*((t_impact/deltaTime)/2))) > 4.0))
     {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr," - BAILING\n");
#endif
      goto frontavoid_end;
     }
#endif
     if (prefer_side == TR_RGT)
      speedcutoff = -0.5;
     else if (prefer_side == TR_LFT || seg->type == TR_LFT)
      speedcutoff *= 1.5;
     if (o->car->_pos < car->_pos && avoidspeedR < o->getSpeed()+speedcutoff)
     {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr," - BAILING\n");
#endif
      goto frontavoid_end;
     }
     /*
     if (prefer_side == TR_LFT && avoidspeed < getSpeed() - 3)
     {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr," - BAILING\n");
#endif
      goto frontavoid_end;
     }
     */
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"\n");
#endif
     Tleft += avoidrightinc * 1.5;
#ifdef DEBUGMSG
fprintf(stderr," C %d %d/%d/%d/%d/%d\n",definite_allow_switch,(dspd < nwdiff - o->getBrakeDistance()/2),((prefer_side == TR_LFT && segtype != TR_LFT && lane2left > nextleft+3.0)),((nextleft-car->_trkPos.toLeft > rightinc*1.5 && nwdiff < car->_dimension_y * 2 && ONleft < seg->width - (car->_dimension_y + avoidwidth + 2.0 + tAvoidRight[next]))),((ONleft < minoffset + car->_dimension_y + avoidwidth + 2.0 + tAvoidLeft[next])),(((Oleft-ONleft)*2.0 >= car->_trkPos.toLeft-nextleft && (ONleft<Oleft-leftinc || car->_trkPos.toLeft>nextleft+rightinc || (Oleft-ONleft)+(car->_trkPos.toLeft-nextleft) > rightinc) && nwdiff < car->_dimension_y && t_impact > 0.8) && ONleft > car->_dimension_y + avoidwidth));
#endif
     favoiding = (AVOID_FRONT | AVOID_LEFT | AVOID_SWITCH);
#if 0
     if (MIN(tlane2left, lane2left) > oppmax + 1.0 || MIN(tlane2left, lane2left) >= Tleft + 1.0)
     {
      this_allow_correcting = 1;
     }
#endif
    }
    else
    {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"STICK TO LEFT SIDE");
#endif

     if (try_allow_switch && fabs(nwdiff) < getWidth()/2 + o->getWidth()/2 + 1.0)
      o->setState(OPP_COLL);

     //if (0 && nextleft < tAvoidLeft[nextnext] && car->_speed_x > tSlowSpeed[nextnext]+4.0)
#if 0
     if (((prefer_side == TR_RGT && cornerdist < cornerlimit*0.7) || (segtype == TR_RGT && tMaxSpeed[nextdiv] <= tMaxSpeed[thisdiv]))
         && nextleft < tAvoidLeft[nextnext] 
         && MAX(ospeed, getSpeed()-5) > tSpeed[nextdiv] + (o->car->_pos < car->_pos ? 2.0 : 6.0))
     {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr," - BAILING\n");
#endif
      goto frontavoid_end;
     }
     if ((getSpeed() + tSpeed[nextnext])/2 < o->getSpeed())
     {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr," - BAILING B\n");
#endif
      goto frontavoid_end;
     }
#endif
     if (((segtype == TR_RGT || (prefer_side == TR_RGT && cornerdist < cornerlimit))
         && (MIN(o->car->_speed_x, getSpeed()) > avoidspeedL+1.0 && nextleft < (MIN(o->car->_speed_x, (getSpeed()) - avoidspeedL) * 3))) ||
            (tRInverse[LINE_RL][nextdiv] < -0.002 && t_impact > fabs(tRInverse[LINE_RL][nextdiv]) * 150))
     {
      goto frontavoid_end;
     }
     if (prefer_side == TR_LFT || prefer_side == TR_STR)
      speedcutoff = -1.5;
     else if (prefer_side == TR_RGT || seg->type == TR_RGT)
      speedcutoff *= 1.5;
     if (o->car->_pos < car->_pos && avoidspeedL < o->getSpeed()+speedcutoff)
     {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr," - BAILING4\n");
#endif
      goto frontavoid_end;
     }
     /*
     if (prefer_side == TR_RGT && avoidspeed < getSpeed() - 3)
     {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr," - BAILING\n");
#endif
      goto frontavoid_end;
     }
     */

#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"\n");
#endif

     // stick to left.
     sidemargin = MAX(0.0, oppmin-nextleft);
     if (fabs(nextleft - o->getNextLeft()) < fabs(car->_trkPos.toLeft - o->car->_trkPos.toLeft)*0.9)
      sidemargin = MAX(0.0, sidemargin - (fabs(car->_trkPos.toLeft - o->car->_trkPos.toLeft) - fabs(nextleft - o->getNextLeft()))*20);
     if (segtype != TR_STR)
      sidemargin /= 2;

     //double spdmargin = dspd * 2 - distance;
     //if (MAX(tlane2left, lane2left) < oppmin - (1.0+spdmargin) || (MAX(tlane2left, lane2left) > Tleft + 4.0 && distance-dspd > 2.0 && Tleft < oppmin - (1.0+spdmargin)))
#if 0
     if (MAX(tlane2left, lane2left) < oppmin - (2.0+spdmargin))
     {
      this_allow_correcting = 1;
     }
#endif

     if (!(side_count & AVOID_LEFT) &&
         ((o->getState() & OPP_COLL) || sidemargin < 2.0) &&
         (1 ||
          (o->getState() & OPP_COLL) ||
          (seg->type != TR_RGT && nextleft > seg->width * 0.5) ||
          (Tleft > tlane2left + 3.0 && tlane2left > nlane2left)))
      /*
     if ((o->getState() & OPP_COLL) ||
         (seg->type != TR_STR && seg->radius <= 140.0) ||
         ((Tleft > oppmin-(seg->type != TR_STR ? 5.0 : 3.0) || (prefer_side == TR_LFT && (lane2left < nextleft || nextleft > seg->width * 0.6)))
          && (1 || Tleft > tlane2left || segtype == TR_LFT || prefer_side != TR_LFT || Tleft > seg->width * 0.45)))
          */
     {
      Tleft -= avoidleftinc * ratio;
#ifdef DEBUGMSG
fprintf(stderr," D1 %d sm=%.3f coll=%d\n",this_allow_correcting,sidemargin,(o->getState() & OPP_COLL));
#endif
     }
     else if (tlane2left >= nextleft && !(o->getState() & OPP_COLL) && sidemargin > 5.0)
     {
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr," D2 %d sm=%.3f coll=%d\n",this_allow_correcting,sidemargin,(o->getState() & OPP_COLL));
#endif
      Tleft += avoidrightinc * 0.6;
     }
     // omigosh - new!
     //else if (nextleft < oppmin - 2.0 && tlane2left < nlane2left)
     // Tleft += MIN(rightinc/2, nlane2left - tlane2left);
#if 0
     else if (nextleft < oppmin-(seg->type == TR_RGT ? 3.0 : 0.5) && tlane2left > nextleft+1.0)
     {
      Tleft += rightinc*0.35;
#ifdef DEBUGMSG
fprintf(stderr," D2 %d (TL=%.3f om=%.3f tl=%.3f dist=%.3f dspd=%.3f)\n",this_allow_correcting,Tleft,oppmin,MAX(tlane2left,lane2left));
#endif
     }
#endif
     favoiding = (AVOID_FRONT | AVOID_RIGHT);
    }
   }
  }
 }

frontavoid_end:
 avoiding |= favoiding;

 if (!avoiding || avoiding == AVOID_HOLD)
 {
  // look for overlappers to allow past
  for (i = 0; i < opponents->getNOpponents(); i++) 
  {
   if (opponent[i].car->_state >= RM_CAR_STATE_PIT) continue;
 
   tCarElt *otherCar = opponent[i].car;
   if ((otherCar == car))
    continue;
 
   if (otherCar->_trkPos.toLeft < -3.0f || otherCar->_trkPos.toRight < -3.0f)
    continue;
 
   if (!(opponent[i].getState() & OPP_LETPASS))
    continue;

   double dlg = opponent[i].getNextDistance();
   //double dspd = otherCar->_speed_x - car->_speed_x;

   if (dlg > 0.0)
    dlg -= track->length;
   if (dlg > -10)
   {
    if (car->_trkPos.toLeft > otherCar->_trkPos.toLeft) 
    {
     Tleft += avoidrightinc*0.7;
     avoiding |= (AVOID_BACK | AVOID_LEFT);
    } 
    else 
    {
     Tleft -= avoidleftinc*0.7;
     avoiding |= (AVOID_BACK | AVOID_RIGHT);
    }
    avoiding |= AVOID_LETPASS;
   }
  }
 }

 if (avoiding == AVOID_HOLD)
 {
  if ((old_avoiding & AVOID_SIDE) && !(old_avoiding & (AVOID_FRONT|AVOID_BACK)))
  {
   avoid_hold = 0;
  }
  if (avoid_hold < currentsimtime && (old_avoiding & AVOID_ALIGNED))
  {
   avoiding = 0;
  }
  else if (avoid_hold < currentsimtime || fabs(angle) > 1.2 || nextleft < tAvoidLeft[nextdiv] || nextright < tAvoidRight[nextdiv])
  {
   avoid_hold = avoiding = 0;
  }
  else if (SEGTYPE(seg) != TR_STR || !front_avoid_allowed)
   avoid_hold -= 0.5;
 }

 if (!fStuck && fabs(angle) < 1.0 && car->_speed_x > 25.0 && !avoiding && !getCorrecting() && 
     ((car->_trkPos.toLeft < -0.5f && car->_trkPos.toLeft < lane2left-1.5) || 
      (car->_trkPos.toRight < -0.5f && car->_trkPos.toLeft > lane2left+1.5)))
 {
  setCorrecting(1);
 }
 if (avoiding)
 {
  double av1 = 1.2 + (segtype == TR_STR ? 0.0 : (140.0 - seg->radius) / 50);
  double av2 = 1.4 + (segtype == TR_STR ? 0.0 : 0.5 + ((140.0-seg->radius) / 200));
  if ((avoiding & AVOID_SIDE))
  {
   av1 *= 1.5;
   av2 *= 1.5;
  }
#if 0
  if (((avoiding & AVOID_RIGHT) && tlane2left < MIN(seg->width - 5.0, oppmin - (segtype == TR_LFT ? av1 : av2)) 
       && lane2left < MIN(seg->width - 5.0, oppmin - (segtype == TR_LFT ? av1 : av2)) 
       && (fabs(tlane2left-nlane2left) < 1.0 || nlane2left < nextleft)) || 
      ((avoiding & AVOID_LEFT) && tlane2left > MAX(5.0, oppmax + (segtype == TR_RGT ? av1 : av2)) 
       && lane2left > MAX(5.0, oppmax + (segtype == TR_RGT ? av1 : av2)) 
       && (fabs(tlane2left-nlane2left) < 1.0 || nlane2left > nextleft)) ||
      (!(avoiding & AVOID_SIDE) && cornerdist < 50.0 && 
       ((prefer_side==TR_LFT && car->_trkPos.toRight < 6.0) || (prefer_side == TR_RGT && car->_trkPos.toLeft < 6.0))))
#else
  if (this_allow_correcting)
#endif
  {
   if ((!old_avoiding && !getCorrecting()) || (old_avoiding & AVOID_ALIGNED))
   {
    avoiding |= AVOID_ALIGNED;
    //correct_timer = 0.0;
   }
   else if ((fabs(angle) < 0.3 && fabs(laststeer) < 0.5 &&
             fabs(car->_trkPos.toLeft - lane2left) <= incfactor * 2 &&
             fabs(laststeer - lastksteer) < 0.02))
   {
    //correcting = allow_correcting = avoiding = 0;
    align_hold = currentsimtime + 5.0;
    avoiding |= AVOID_ALIGNED;
    //correct_timer = 0.0;
   }
   else 
   {
    Tleft = startleft;
    setAllowCorrecting(1);
#if 0
    if (!(avoiding & AVOID_SIDE))
     Tleft = startleft;
    else
     Tleft = MAX(startleft - leftinc*0.5, MIN(startleft + rightinc*0.5, Tleft));
#endif
#if 0
    if (lane2left > Tleft && !(avoiding & AVOID_RIGHT))
    {
     Tleft += MIN(rightinc*0.6, lane2left-Tleft);
    }
    else if (!(avoiding & AVOID_LEFT))
    {
     Tleft -= MIN(leftinc*0.6, Tleft - lane2left);
    }
#endif
   }
  }
  else if (0 && (avoiding & AVOID_FRONT) && prefer_side != TR_STR)
  {
   // keep overtaking within 60% of track width from racelane when approaching
   // or on a significant corner.
   if (Tleft < lane2left - seg->width * 0.6)
   {
    // move it off the left lane
    Tleft = startleft + avoidrightinc * 0.5;
   }
   else if (Tleft > lane2left + seg->width * 0.6)
   {
    // move it off the left lane
    Tleft = startleft - avoidleftinc * 0.5;
   }
  }
  else 
   setAllowCorrecting(0);
 }
 else if (old_avoiding || getCorrecting())
 {
  setCorrecting(1);

  if (start_finished ||
      (currentsimtime > 1.0 && 
       (//(cseg->type == TR_LFT && car->_trkPos.toMiddle <= 0.0) ||
        //(cseg->type == TR_RGT && car->_trkPos.toMiddle >= 0.0) ||
        (old_avoiding & AVOID_ALIGNED) || 
        car->_pos == 1 ||
        cornerdist < 120.0))
      || nsegtype != TR_STR)
  {
   start_finished = 1;

   if ((old_avoiding & AVOID_ALIGNED) ||
       (fabs(angle) < 0.4 && fabs(laststeer) < 0.4 &&
        fabs(car->_trkPos.toLeft - lane2left) < MAX(0.1, 1.0 - (car->_speed_x*(car->_speed_x/10))/600) &&
        fabs(nextleft - lane2left) < incfactor * 3 &&
        fabs(nextleft - car->_trkPos.toLeft) < fabs(lane2left-tlane2left)*1.3 &&
        fabs(laststeer - lastksteer) < 0.06))
#if 0
   if ((fabs(angle) < 0.3 && fabs(laststeer) < 0.3 &&
        fabs(car->_trkPos.toLeft - lane2left) <= incfactor * 2 &&
        fabs(laststeer - lastksteer) < 0.02))
#endif
   {
    setCorrecting(0);
    correct->setAligned(1);
    align_hold = currentsimtime + 5.0;
   }
   else 
   {
    if (startleft < tlane2left && startleft < car->_dimension_y*0.7)
    {
     Tleft = startleft + avoidrightinc*0.5;
    }
    else if (startleft > tlane2left && startleft > seg->width-car->_dimension_y*0.7)
    {
     Tleft = startleft - avoidleftinc*0.5;
    }
   }
  }
 }

 if (!avoiding && !getCorrecting() && align_hold > currentsimtime &&
     fabs(angle) < 0.3 && fabs(laststeer) < 0.08 && 
     fabs(car->_yaw_rate) < 0.6 && fabs(car->_trkPos.toLeft - lane2left) <= incfactor)
  align_hold = currentsimtime;

 if (fabs(angle) > 1.4)
 {
  if (old_avoiding || getCorrecting() || MIN(car->_trkPos.toLeft, car->_trkPos.toRight) < 1.5)
  {
   setCorrecting(1);
  }
  else
  {
   avoiding = 0;
   if (!getCorrecting() && !old_avoiding)
    align_hold = currentsimtime + 4.0;
  }
 }

 if (avoiding && !old_avoiding && !(avoiding & AVOID_ALIGNED))
 {
  avoid_diverge = fabs(Tleft - lastLTleft) + deltaTime * (3.0 + (car->_accel_x < 0.0 ? MAX(-4.0, car->_accel_x / 10 * fabs(car->_yaw_rate)) : 0.0));
  if (avoid_diverge <= 0.0 && !(avoiding & AVOID_SIDE_COLL))
   avoiding = 0;
 }

 if ((avoiding & !(avoiding & AVOID_ALIGNED)) || getCorrecting())
 {
  Tleft = MAX(Tleft - avoid_diverge, MIN(Tleft + avoid_diverge, Tleft));
  if (!getCorrecting() && !getAllowCorrecting())
  {
   avoid_diverge += deltaTime * (3.0 + (car->_accel_x < 0.0 ? MAX(-4.0, car->_accel_x / 2) : 0.0));
   if (car->_accel_x < 0.0 && car->_accel_x/10 * fabs(car->_yaw_rate) < -4.0)
   {
    avoiding = 0;
    if (old_avoiding)
     setCorrecting(1);
   }
   //Tleft = MAX(lastTleft - deltaTime * (8), MIN(lastTleft + deltaTime * (8), Tleft));
  }
 }
 else if (!(avoiding & AVOID_ALIGNED))
 {
  // not correcting - check to see if we're way out of raceline
  if (fabs(car->_trkPos.toLeft - tlane2left) > 2.0 - fabs(tRInverse[rl][nextdiv]) && car->_speed_x < TargetSpeed - 10.0 && (fabs(laststeer) > 0.5 || fabs(angle) > 0.5 || fabs(car->_yaw_rate) > 1.3 || car->_trkPos.toLeft < minoffset - 1.0 || car->_trkPos.toLeft > maxoffset + 1.0))
   setCorrecting(1);
 }

 if (!avoiding)
  Tleft = car->_trkPos.toLeft;

 if ((getCorrecting() || avoiding) && !(avoiding & AVOID_ALIGNED))
 {
  if (avoiding)
  {
   double factor = 0.8;
   Tleft = MAX(nextleft - avoidleftinc*factor, MIN(nextleft + avoidrightinc*factor, Tleft));
  }
  else
  {
   Tleft = MAX(startleft - leftinc, MIN(startleft + rightinc, Tleft));
   Tleft = MAX(nextleft - leftinc*3, MIN(nextleft + rightinc*3, Tleft));
  }
  if (seg->type == TR_LFT && seg->radius <= 200.0)
   minoffset = MAX(minoffset, MIN(startleft+0.5, minoffset + seg->radius/210.0 * 3));
  if (seg->type == TR_RGT && seg->radius <= 200.0)
   maxoffset = MIN(maxoffset, MAX(startleft-0.5, maxoffset + seg->radius/210.0 * 3));
  if (minoffset > maxoffset)
   minoffset = maxoffset = (minoffset+maxoffset)/2;
  Tleft = MAX(minoffset, MIN(maxoffset, Tleft));

#if 0
  double avoidmaxoffset = maxoffset, avoidminoffset = minoffset;
  if (segtype == TR_LFT && seg->radius <= 130.0)
   avoidminoffset = MIN(avoidmaxoffset, MAX(avoidminoffset, 1.5 + ((135.0-seg->radius)*(135-seg->radius))/900));
  else if (segtype == TR_RGT && seg->radius <= 130.0)
   avoidmaxoffset = MAX(avoidminoffset, MIN(avoidmaxoffset, seg->width - (1.5 + ((135.0-seg->radius)*(135-seg->radius))/900)));
  Tleft = MAX(avoidminoffset, MIN(avoidmaxoffset, Tleft));
#endif
 }
 if (avoiding && !(avoiding & AVOID_ALIGNED) && !getAllowCorrecting())
  ToLeft = Tleft;
 else
  ToLeft = 0.0;

 double lastoffset = myoffset;
 int offsetinc = (nextleft < minoffset - 1.0 || nextleft > maxoffset + 1.0 ? 15 : 3);
 myoffset = (float)(seg->width * 0.5 - Tleft);
 myoffset = (float)MIN(lastoffset + incfactor*offsetinc, MAX(lastoffset - incfactor*offsetinc, myoffset));
 lastLTleft = tLane[rl][nextdiv];
 lastTleft = Tleft;
#ifdef OVERTAKE_DEBUG_MSG
fprintf(stderr,"%s: sl=%.3f nl=%.3f Tl=%.3f ctl=%.3f ll=%.3f mo=%.3f li=%.3f ri=%.3f ii=%.3f max=%.3f min=%.3f allow=%d\n",car->_name,startleft,nextleft,Tleft,car->_trkPos.toLeft,tlane2left,myoffset,leftinc,rightinc,incfactor,maxoffset,minoffset,getAllowCorrecting());
#endif

 return myoffset;
}

int Driver::getStrategy()
{
 return strategy->getStrategy();
}

double Driver::getSideMargin()
{
 double sm = 0.5;
 //if (segtype != TR_STR || cornerdist < cornerlimit)
 // sm *= 2.0f;
 if (strategy->getStrategy() == STRATEGY_NORMAL) return sm; 
 return sm * 1.3f;
}

// Update my private data every timestep.
void Driver::update(tSituation *s)
{
 // Update global car data (shared by all instances) just once per timestep.
 if (currentsimtime != s->currentTime) {
  currentsimtime = s->currentTime;
  cardata->update();
 }

 // Update the local data rest.
 brake_collision = 0.0;
 speedangle = mycardata->getTrackangle() - atan2(car->_speed_Y, car->_speed_X);
 FLOAT_NORM_PI_PI(speedangle);
 double trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
 angle = trackangle - car->_yaw;
 NORM_PI_PI(angle);
//fprintf(stderr,"%s: angle=%.3f sa=%.3f\n",car->_name,angle,speedangle);fflush(stderr);

 nextwidth = getWidth() + (getWidth() - prevwidth);
 prevwidth = getWidth();
 nextleft = car->_trkPos.toLeft + (car->_trkPos.toLeft - prevleft);
  
 mass = (float)(CARMASS + car->_fuel);
 currentspeedsqr = car->_speed_x*car->_speed_x;
 opponents->update(s, this);
 strategy->update(car, s);
 if (!pit->getPitstop() && (car->_distFromStartLine < pit->getNPitEntry() || car->_distFromStartLine > pit->getNPitEnd())) {
  pit->setPitstop(strategy->needPitstop(car, s, opponents));
 }
 if (pit->getPitstop() && car->_pit)
 {
  pitpos = 0;
  pitstatus[carindex] = 1;
  for (int i=0; i<opponents->getNOpponents(); i++)
  {
   if (opponent[i].getTeam() != TEAM_CONTROL) continue;

   if (pitstatus[i] && car->_fuel > fuelperlap + 1 && nextleft >= 0.0 && nextleft <= track->width)
   {
    pit->setPitstop( 0 );
    pitstatus[carindex] = 0;
    break;
   }

   if (opponent[i].car->_pit->pos.seg->lgfromstart + opponent[i].car->_pit->pos.toStart ==
       car->_pit->pos.seg->lgfromstart + car->_pit->pos.toStart)
   {
    // sharing a pit
    if (opponent[i].car->_state == RM_CAR_STATE_PIT)
     pitpos = 2;  // go in behind other car
    else
     pitpos = 1;  // stop at end of pit space to leave room for car behind.
   }
   break;
  }
 }
 pit->update();
 alone = isAlone();
 last_flying = CheckFlying();

 double lane2left = car->_trkPos.seg->width * tLane[rl][nextdiv];
 if (((!avoiding && !getCorrecting()) || last_damage == car->_dammage))
 {
  int avoid = (avoiding || getCorrecting() || align_hold > currentsimtime);
  double outside = lane2left;
  if (avoid && 0)
  {
   if (segtype == TR_LFT || (segtype == TR_STR && lastcornertype == TR_LFT))
    outside = tAvoidRight[nextdiv];
   else if (segtype == TR_RGT || (segtype == TR_STR && lastcornertype == TR_RGT))
    outside = tAvoidLeft[nextdiv];
   learn->update(s, track, car, 1, avoid, outside, tRadius);
  }
 }
}


int Driver::isAlone()
{
 int i;
 if (avoiding & (AVOID_FRONT))
  return 0;

 for (i = 0; i < opponents->getNOpponents(); i++) {
  if (opponent[i].car == car) continue;
  if (opponent[i].car->_state >= RM_CAR_STATE_PIT) continue;
  if (opponent[i].getTeam() & TEAM_CONTROL) continue;
  if ((opponent[i].getState() & (OPP_COLL | OPP_SIDE | OPP_LETPASS)) ||
      ((opponent[i].getState() & OPP_FRONT) && opponent[i].getDistance() < 15.0)) {
   return 0; // Not alone.
  }
 }
 return 1; // Alone.
}

int Driver::Chased()
{
 int i;
 for (i = 0; i < opponents->getNOpponents(); i++) {
  if (opponent[i].car == car) continue;
  if (opponent[i].car->_state >= RM_CAR_STATE_PIT) continue;
  if (opponent[i].getTeam() & TEAM_CONTROL) continue;
  if (opponent[i].getDistance() < 0.0 && opponent[i].getDistance() > -100.0)
   return 1;
 }
 return 0;
}


// Check if I'm stuck.
bool Driver::isStuck()
{
 if (pit->isExiting())
  return false;

 if (fabs(angle) > MAX_UNSTUCK_ANGLE &&
   fabs(angle) < MIN_UNSTUCK_ANGLE &&
   getSpeed() < MAX_UNSTUCK_SPEED &&
   fabs(car->_trkPos.toMiddle) > MIN_UNSTUCK_DIST) 
 {
  if (stuck > MAX_UNSTUCK_COUNT && (car->_trkPos.toMiddle*angle < 0.0 || (fabs(car->_trkPos.toMiddle) > car->_trkPos.seg->width * 0.6 && fabs(car->_speed_x) < 1.0 && car->_gear > 0)) )
  {
   if ((angle > 0.4 && car->_trkPos.toLeft < 3.0 && car->_trkPos.toLeft > -4.0) ||
       (angle < -0.4 && car->_trkPos.toRight < 3.0 && car->_trkPos.toRight > -4.0))
    return false;
   return true;
  } 
  else if (fStuck && reversehold < currentsimtime && stuck > MAX_UNSTUCK_COUNT)
  {
   stuck = 0;
   return false;
  }
  else 
  {
   stuck++;
   return false;
  }
 } 
 else if (fabs(car->_trkPos.toMiddle) > car->_trkPos.seg->width * 0.6 && fabs(getSpeed()) < 1.0 && car->_gear > 0)
 {
  if (stuck > MAX_UNSTUCK_COUNT)
   return true;

  if (fStuck && reversehold < currentsimtime && stuck > MAX_UNSTUCK_COUNT)
  {
   stuck = 0;
   return false;
  }
  stuck++;
  return false;
 }
 else 
 {
  stuck = 0;
  return false;
 }
}


// Compute aerodynamic downforce coefficient CA.
void Driver::initCa()
{
 char const *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
 double rearwingarea = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGAREA, (char*) NULL, 0.0);
 double rearwingangle = GfParmGetNum(car->_carHandle, SECT_REARWING, PRM_WINGANGLE, (char*) NULL, 0.0);
 double frontwingarea = GfParmGetNum(car->_carHandle, SECT_FRNTWING, PRM_WINGAREA, (char*) NULL, 0.0);
 double frontwingangle = GfParmGetNum(car->_carHandle, SECT_FRNTWING, PRM_WINGANGLE, (char*) NULL, 0.0);
 //double frontarea = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FRNTAREA, (char*) NULL, 0.0);
 double frontclift = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FCL, (char*) NULL, 0.0);
 double rearclift = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_RCL, (char*) NULL, 0.0);
 double rearwingca = 1.23*rearwingarea*sin(rearwingangle);
 double frntwingca = 1.23*frontwingarea*sin(frontwingangle);

 double cl = frontclift + rearclift;
 double h = 0.0;
 int i;
 for (i = 0; i < 4; i++)
  h += GfParmGetNum(car->_carHandle, WheelSect[i], PRM_RIDEHEIGHT, (char*) NULL, 0.20f);
 h*= 1.5; h = h*h; h = h*h; h = 2.0 * exp(-3.0*h);
 CA = h*cl + 4.0*((frntwingca+rearwingca)/2);
 FCA = h*frontclift + 4.0*frntwingca;
 RCA = h*rearclift + 4.0*rearwingca;
//fprintf(stderr,"%s: CA=%.3f, FCA=%.3f RCA=%.3f, fclift=%.3f rclift=%.3f fwca=%.3f rwca=%.3f\n",car->_name,CA,FCA,RCA,frontclift,rearclift,frntwingca,rearwingca);fflush(stderr);
}


// Compute aerodynamic drag coefficient CW.
void Driver::initCw()
{
 char const *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
 double cx = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_CX, (char*) NULL, 0.0);
 double frontarea = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS, PRM_FRNTAREA, (char*) NULL, 0.0);
 double frontwingangle = GfParmGetNum(car->_carHandle, SECT_FRNTWING, PRM_WINGANGLE, (char*) NULL, 0.0);
 double frontwingarea = GfParmGetNum(car->_carHandle, SECT_FRNTWING, PRM_WINGAREA, (char*) NULL, 0.0);
 brakeCW = 0.645*cx*frontarea;
 //CW = 1.23 * frontarea;
 CW = (0.8f-cx) * frontarea;
 FWA = frontwingarea * sin(frontwingangle);
 //if (frontwingarea)
 // CW += FWA;
//fprintf(stderr,"CW=%.3f FWA=%.3f\n",CW,FWA);fflush(stderr);
 double h = 0.0;
 int i;
 for (i=0; i<4; i++)
 {
  h += GfParmGetNum(car->_carHandle, WheelSect[i], PRM_RIDEHEIGHT, (char*) NULL, 0.20f);
 }
 h*= 1.5; h = h*h; h = h*h; h = 2.0 * exp(-3.0*h);
 CW = (h + 4.0*CW); 
}

void Driver::initCR()
{
 CR = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_FRWEIGHTREP, (char *)NULL, 0.50);
//fprintf(stderr,"%s: CR = %.3f\n",car->_name,CR);
}


// Init the friction coefficient of the the tires.
void Driver::initTireMu()
{
 char const *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL, SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
 double tm = FLT_MAX;
 int i;

 for (i = 0; i < 4; i++) {
  tm = MIN(tm, GfParmGetNum(car->_carHandle, WheelSect[i], PRM_MU, (char*) NULL, 1.0));
 }
 TIREMU = tm;
 BrakePressure = GfParmGetNum(car->_carHandle, SECT_BRKSYST, PRM_BRKPRESS, (char *) NULL, 1.0);
}


// Reduces the brake value such that it fits the speed (more downforce -> more braking).
double Driver::filterBrakeSpeed(double brake)
{
 double tanksize = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_TANK, (char*) NULL, 100.0f);
 if (car->_fuel < tanksize / 2 && !brake_collision && !pit->getInPit())
  brake = MAX(0.0, brake - (tanksize/2 - car->_fuel) / (tanksize/2+5));

 if (!useBTSpeed || (useBTSpeed == 1 && brake > BTBrakeCmd))
 {
  useBTSpeed = 0;
  return brake;
 }

 double weight = (CARMASS + car->_fuel)*G;
 double maxForce = weight + CA*MAX_SPEED*MAX_SPEED;
 double force = weight + CA*currentspeedsqr;
 double newbrake = (BTBrakeCmd*force/maxForce);

 if (useBTSpeed == 1)
 {
  if (brake > newbrake)
   useBTSpeed = 0;
  return MAX(brake, newbrake);
 }
 else
 {
  if (brake < newbrake)
   useBTSpeed = 0;
  //else if (!newbrake)
  // car->_accelCmd = AccelCmd = (MAX(AccelCmd, BTAccelCmd));
  return MIN(brake, newbrake);
 }
}


// Brake filter for pit stop.
double Driver::filterBPit(double brake)
{
 if (pit->getPitstop() && !pit->getInPit()) {
  float dl, dw;
  RtDistToPit(car, track, &dl, &dw);
  if (dl < PIT_BRAKE_AHEAD) {
   double mu = car->_trkPos.seg->surface->kFriction*TIREMU*PIT_MU;
   if (brakedist(0.0, mu) > dl) {
    brake_avoid = 1.0f;
    useBTSpeed = 0;
    return 1.0;
   }
  }
 }

 if (pit->getInPit()) {
  useBTSpeed = 0;

  double s = pit->toSplineCoord(car->_distFromStartLine);
  // Pit entry.
  if (pit->getPitstop()) {
   double pitoffset = pit->getPitOffset(car->_trkPos.toMiddle, car->_distFromStartLine, nextleft, pitpos);
   double maxbrake = MAX(0.2, 0.8 - MAX(fabs(angle)*0.7, fabs(car->_yaw_rate)/2));
   car->_accelCmd = AccelCmd = BTAccelCmd = MIN(car->_accelCmd, MAX(0.4, MIN(0.8, car->_speed_x/60)));
   double mu = car->_trkPos.seg->surface->kFriction*TIREMU*PIT_MU;

   if (s < pit->getNPitStart()) {
    // Brake to pit speed limit.
    double dist = pit->getNPitStart() - s;
    if (dist < MAX(50.0, (car->_speed_x - pit->getSpeedlimit()) * 5) && brakedist(pit->getSpeedlimit(), mu) > dist) {
     brake_avoid = 1.0f;
     return maxbrake;
    }
   } else {
    // Hold speed limit.
    if (currentspeedsqr > pit->getSpeedlimitSqr()) {
     return pit->getSpeedLimitBrake((float)currentspeedsqr);
    }
   }
   // Brake into pit (speed limit 0.0 to stop)
   double dist = pit->getNPitLoc(pitpos) - s;
   if (pit->isTimeout((float)dist)) {
    pit->setPitstop(false);
    return (brake_collision ? brake : 0.0);
   } else {
    if (brakedist(0.0, mu) > dist) {
     brake_avoid = 1.0f;
     return maxbrake;
    } else if (s > pit->getNPitLoc(pitpos)) {
     // Stop in the pit.
     if (car->_speed_x < 5.0 && s < pit->getNPitLoc(pitpos) + 3.0 && 
         ((pitoffset > 0.0 && myoffset < pitoffset && nextleft > car->_trkPos.toLeft) || 
          (pitoffset < 0.0 && myoffset > pitoffset && nextleft < car->_trkPos.toLeft)))
     {
      car->_accelCmd = AccelCmd = BTAccelCmd = MAX(car->_accelCmd, 0.2f);
      return 0.0;
     }
     brake_avoid = 1.0f;
     return maxbrake;
    }
   }
  } else {
   // Pit exit.
   if (s < pit->getNPitEnd()) {
    // Pit speed limit.
    if (currentspeedsqr > pit->getSpeedlimitSqr()) {
     return pit->getSpeedLimitBrake((float)currentspeedsqr);
    }
   }
  }
 }

 return brake;
}

// Antilocking filter for brakes.
double Driver::filterABS(double brake)
{
 if (K1999Brakes && !brake_collision)
  return brake;
 if (!pit->getInPit())
 {
  if (!brake_collision && (getCorrecting() || align_hold > currentsimtime || pit->getInPit()))
   brake = MIN(brake, 2.5);
  else if (brake_collision)
   brake = MIN(brake, 1.8);
  else
   brake = MIN(brake, 2.0);
 }

 double minbrake = (brake_collision ? MAX(0.0, ((0.9-brake_collision)-MAX(0.0, fabs(angle)-0.3))/2) : 0.0);
 double speed = MAX(car->_speed_x, getSpeed());
 //double oldbrake = brake;
 double newbrake = brake;
 if (speed < ABS_MINSPEED) return brake;
 //if (!brake_avoid) return brake;
 int i;
 double slip = 0.0;

 double meanSpd = 0;

 for (i = 3; i < 4; i++) {
  meanSpd += car->_wheelSpinVel(i);
 }
 meanSpd /= 2.0;

 if (meanSpd > 1.0) {
  for (i = 0; i < 4; i++) {
   if (((meanSpd - car->_wheelSpinVel(i)) / meanSpd) < -0.1) {
    slip = 1.0;
   }
  }
 }
 if (slip != 0) {
  ABS *= 0.95;
  if (ABS < 0.15) {
   ABS = 0.15;
  }
 } else {
  if (ABS < 0.15) {
   ABS = 0.15;
  }
  ABS *= 1.4;
  if (ABS > 1.0) {
   ABS = 1.0;
  }
 }

 {
  double sideslip = 0;
  for (i = 0; i < 4; i++) {
   if (i < 2)
    sideslip += fabs(car->_wheelSlipSide(i)) / 2;
   else
    sideslip += fabs(car->_wheelSlipSide(i)) * 1.5;
   slip += (car->_wheelSpinVel(i) * car->_wheelRadius(i));
  }
  slip = car->_speed_x - slip/4.0;
  //slip = speed - slip/4.0;
  if (!pit->getInPit() && !avoiding && !brake_collision)
   slip += sideslip / 16.0;
  if (brake_avoid > 1.2f && fabs(angle) < 0.3f && fabs(laststeer) < 0.08f)
  {
   slip *= (1.0 - ((0.3-fabs(angle))*0.75 + (0.08-fabs(laststeer))*3));//0.75f;
   car->_steerCmd *= 0.8f;
   laststeer = car->_steerCmd;
  }
  else if (fabs(laststeer) > 0.2 || fabs(angle) > 0.3)
  {
   slip *= (1.0 + MAX(fabs(angle)*0.5f, fabs(laststeer) * 2.0f));
  }
  if (slip > ABS_SLIP) {
   brake = newbrake = MAX(0.0, brake - MIN(brake, (slip - ABS_SLIP)/ABS_RANGE));
  }
 }
 if (brake_collision && brake_collision < 1.5)
 {
  brake = MAX(newbrake, MAX((1.5-brake_collision)/5, ABS - fabs(laststeer)/4));
 }
 if (speed > car->_speed_x && currentsimtime - bc_timer < 4.0 && (!brake_collision || brake_collision > 0.8))
  brake = MAX(minbrake, brake - (speed - car->_speed_x) / 10);

 double frntslip = (car->_wheelSlipSide(0) + car->_wheelSlipSide(1)) / 2;
 double rearslip = (car->_wheelSlipSide(2) + car->_wheelSlipSide(3)) / 2;
 if (fabs(rearslip - frntslip) > 10.0 && fabs(rearslip) > fabs(frntslip))
 {
  if (brake_collision)
   brake = MAX(minbrake/2, MIN(brake, 0.5 - fabs(rearslip-frntslip) / 40));
  else
   brake = MAX(minbrake, MIN(brake, 0.5 - fabs(rearslip-frntslip) / 80));
 }

 if (!pit->getInPit())
 {
  double steer = (fabs(car->_steerCmd) > 0.75 ? fabs(car->_steerCmd) * 2.0 : 0.0);
  double yaw = MAX(steer, fabs(car->_yaw_rate - car->_steerCmd/8));
  //double yaw = MAX(fabs(car->_yaw_rate - car->_steerCmd/8), fabs(car->_yaw_rate));
  if (brake_collision && yaw > 0.8)
  {
   brake = MAX(minbrake, brake - (yaw - 0.7) * 1.5);
  }
  else if ((avoiding || getCorrecting() || align_hold > currentsimtime) && yaw > 1.3)
  {
   brake = MAX(minbrake, brake - (yaw - 1.2));
  }
  if (brake > lastbrake && (!brake_collision || brake_collision > 0.8))
  {
   RELAXATION(brake, lastbrake, 10.0);
  }
 }
 return brake;
}

int Driver::rearWheelSlip()
{
 if (fabs(car->_wheelSlipSide(2) + car->_wheelSlipSide(3)) / 2 > fabs(car->_wheelSlipSide(0) + car->_wheelSlipSide(1)) * 0.75)
 {
  int right_bad = (car->_wheelSeg(REAR_RGT) != car->_trkPos.seg &&
       (car->_wheelSeg(REAR_RGT)->surface->kFriction < car->_trkPos.seg->surface->kFriction*0.7 ||
        car->_wheelSeg(REAR_RGT)->surface->kRoughness > MAX(0.02, car->_trkPos.seg->surface->kRoughness*1.2) ||
        car->_wheelSeg(REAR_RGT)->surface->kRollRes > MAX(0.005, car->_trkPos.seg->surface->kRollRes*1.2)));

  int left_bad =  (car->_wheelSeg(REAR_LFT) != car->_trkPos.seg &&
       (car->_wheelSeg(REAR_LFT)->surface->kFriction < car->_trkPos.seg->surface->kFriction*0.7 ||
        car->_wheelSeg(REAR_LFT)->surface->kRoughness > MAX(0.02, car->_trkPos.seg->surface->kRoughness*1.2) ||
        car->_wheelSeg(REAR_LFT)->surface->kRollRes > MAX(0.005, car->_trkPos.seg->surface->kRollRes*1.2)));

  if ((left_bad && right_bad) || 
      ((avoiding || getCorrecting() || align_hold+4.0 > currentsimtime || MAX(fabs(angle), fabs(laststeer)*2) > 0.5) && (left_bad || right_bad)))
  {
   return 1;
  }
 }

 return 0;
}

// TCL filter for accelerator pedal.
double Driver::filterTCL(double accel, char slow)
{
 //if (car->_gearCmd <= -1)
 // return accel;

#if 0
  /* speed management */
 const tdble Dx = 6.0;
 double slip = 0.0;
 double accelTgt = accel = MIN((TargetSpeed+1.0 - car->_speed_x) / Dx, 1.0);
 int gear = car->_gear;
 double steer = car->_steerCmd;
 double aspect = car->_yaw_rate;
    
 /* anti-slip */
 /* assume SPOOL differential and rwd */
 if (car->_speed_x > 0) {
  slip = (car->_wheelRadius(3) * car->_wheelSpinVel(3) - car->_speed_x) / car->_speed_x;
 } else {
  slip = 0;
 }
 if ((car->_gearCmd == 1)) {
  accel = accel * exp(-fabs(steer) * 0.0) /* * exp(-fabs(aspect) * AccAngle[idx]) */ + 0.1;
 } else if (car->_gear > 1) {
  accel = accel * exp(-fabs(aspect) * 0.3); //+ 0.15;
 }
 
 
 if ((slip > 1.0) && (car->_gear > 1)) {
  accel /= 2.0;
  //lastAccel = 0.0;
 } else {
  //RELAXATION(car->_accelCmd, lastAccel[idx], 30.0);
 }
 
#else
 //if (currentsimtime < 5.0 || (fabs(angle) < 0.2 && fabs(car->_steerCmd) < 0.1))
 // return 1.0f;

 //if (fabs(tRInverse[rl][nextdiv]) > 0.0020)
 if (fabs(car->_steerCmd) > 0.02f && (avoiding || getCorrecting() || TurnDecel))
 {
  double decel = MAX(fabs(car->_steerCmd) * 1.0, fabs(tRInverse[rl][nextdiv]) * 4.0);// * (avoiding || getCorrecting() ? 1.5 : 1.0);
  if (!getCorrecting() && !avoiding)
   decel *= TurnDecel;
  //if (decel < 0.06)
  // decel = 0.0;
  accel = MIN(accel, MAX(0.55, 1.0 - decel));

  if (car->_gear <= 1 && fabs(car->_steerCmd) > 0.7)
   accel = MIN(accel, 0.5);
#ifdef DEBUGTHIS
fprintf(stderr,"filterTCL - accel=%.3f decel=%.3f ",accel,decel);
#endif
 }

 if (rearWheelSlip())
 {
  slow = 1;
  accel = MIN(0.7, MIN(accel, MIN(car->_wheelSeg(REAR_RGT)->surface->kFriction, car->_wheelSeg(REAR_LFT)->surface->kFriction)*0.65));
  car->_steerCmd = (float)MIN(0.5, MAX(-0.5, car->_steerCmd));
  BTAccelCmd = MIN(accel, BTAccelCmd);
 }

 // understeer prevention - as Mark Webber said on Top Gear, "its called throttle control"
 double skid = MAX(car->_skid[0], car->_skid[1]) - MAX(car->_skid[2], car->_skid[3]);
 accel -= skid/2;

 double steer = (fabs(car->_steerCmd) > 0.75 ? fabs(car->_steerCmd) * 2.0 : 0.0);
 double yaw = MAX(steer, fabs(car->_yaw_rate - car->_steerCmd/4));
 if (yaw > 1.3)
  slow = 1;
  //accel = MIN(accel, 0.9 - (yaw-1.3)/2);

 if (slow)
 {
  double origaccel = accel;
  accel = MIN(accel, slowslip);

  double slip = 0;
  //if (fabs(car->_speed_x) > 0.1)
  {
   slip = (this->*GET_DRIVEN_WHEEL_SPEED)() - fabs(car->_speed_x);
   //for (int i = 0; i < 4; i++) {
   // double s = (car->_wheelRadius(i) * car->_wheelSpinVel(i) - car->_speed_x);
   // slip = MAX(slip, s);
  // }
  }

  if (slip > 2.0) {
   slowslip *= 0.9;
  }
  else
  {
   slowslip = MAX(0.1, slowslip);
   slowslip *= 1.2;
   if (slowslip > 1.0)
    slowslip = 1.0;
  }
  accel = MAX(accel, MIN(origaccel, 0.1));
 }
 else
 {
  slowslip = 1.0;
  if ((nextleft < 0.0 || nextleft > car->_trkPos.seg->width) && !pit->getInPit())
  {
   laststeer = car->_steerCmd = MAX(-0.7f, MIN(0.7f, car->_steerCmd));
  }
 
  double slip = (this->*GET_DRIVEN_WHEEL_SPEED)() - fabs(car->_speed_x);
  if (slip > SlipLimit*1.2) {
   accel = accel - MIN(accel-0.1, ((slip - SlipLimit*1.2)/TCL_RANGE));
  }
 }
 if ((avoiding || getCorrecting()) && fabs(car->_yaw_rate) > 1.0)
  accel = accel * exp(-(fabs(car->_yaw_rate)-1.0) * 0.4);//0.3); //+ 0.15;
#endif
 return accel;
}


// Traction Control (TCL) setup.
void Driver::initTCLfilter()
{
 char const *traintype = GfParmGetStr(car->_carHandle, SECT_DRIVETRAIN, PRM_TYPE, VAL_TRANS_RWD);
 if (strcmp(traintype, VAL_TRANS_RWD) == 0) {
  drivetrain = DRWD;
  GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_RWD;
 } else if (strcmp(traintype, VAL_TRANS_FWD) == 0) {
  drivetrain = DFWD;
  GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_FWD;
 } else if (strcmp(traintype, VAL_TRANS_4WD) == 0) {
  drivetrain = D4WD;
  GET_DRIVEN_WHEEL_SPEED = &Driver::filterTCL_4WD;
 }
}


// TCL filter plugin for rear wheel driven cars.
double Driver::filterTCL_RWD()
{
 return (MAX(car->_wheelSpinVel(REAR_RGT), car->_wheelSpinVel(REAR_LFT))) *
   car->_wheelRadius(REAR_LFT);// / 2.0;
}


// TCL filter plugin for front wheel driven cars.
double Driver::filterTCL_FWD()
{
 return (car->_wheelSpinVel(FRNT_RGT) + car->_wheelSpinVel(FRNT_LFT)) *
   car->_wheelRadius(FRNT_LFT) / 2.0;
}


// TCL filter plugin for all wheel driven cars.
double Driver::filterTCL_4WD()
{
 return ((car->_wheelSpinVel(FRNT_RGT) + car->_wheelSpinVel(FRNT_LFT)) *
   car->_wheelRadius(FRNT_LFT) +
     (car->_wheelSpinVel(REAR_RGT) + car->_wheelSpinVel(REAR_LFT)) *
   car->_wheelRadius(REAR_LFT)) / 4.0;
}


// Hold car on the track.
double Driver::filterTrk(double accel)
{
 tTrackSeg* seg = car->_trkPos.seg;
 if ((!avoiding || !(avoiding & AVOID_ALIGNED)) && !getCorrecting())
  return accel;
 if (car->_speed_x < MAX_UNSTUCK_SPEED ||  // Too slow.
  pit->getInPit())
 {
  return accel;
 }

 double lane2left = tLane[rl][nextdiv] * seg->width;

 if (segtype == TR_STR)
 {
  double newleft = nextleft + speedangle * 10;
  double mmin = MIN(1.5, lane2left - 1.0);
  double mmax = MAX(seg->width-1.5, lane2left + 1.0);

  if (newleft < mmin)
   accel = MAX(0.0, MIN(accel, 1.0 - (mmin - newleft)/5.0));
  else if (newleft > mmax)
   accel = MAX(0.0, MIN(accel, 1.0 - (newleft - mmax)/5.0));
 }
 else if (segtype == TR_RGT)
 {
  double newleft = nextleft + (speedangle - (120.0-seg->radius)/1000.0) * 10;
  //double mmin = MIN(1.5, lane2left - 1.0);
  //double mmax = MAX(seg->width-1.5, lane2left + 1.0);

  if (newleft < MAX(2.0, lane2left + 1.0))
   accel = MAX(0.0, MIN(accel, 1.0 - (lane2left - newleft) / MAX(6.0, lane2left-1.0)));
 }
 else if (segtype == TR_LFT)
 {
  double newleft = nextleft + (speedangle + (120.0-seg->radius)/1000.0) * 10;
  double mmax = MAX(seg->width-2.0, lane2left + 1.0);

  if (newleft > mmax)
   accel = MAX(0.0, MIN(accel, 1.0 - (newleft - mmax) / MAX(6.0, (seg->width-1.0) - lane2left)));
 }
 
 if (((segtype == TR_RGT || tRInverse[rl][nextdiv] < -0.003) && car->_steerCmd < -0.2 && nextleft < 5.0 && lane2left - nextleft > 5.0) ||
     ((segtype == TR_LFT || tRInverse[rl][nextdiv] > 0.003) && car->_steerCmd > 0.2 && nextleft > seg->width - 5.0 && nextleft - lane2left > 5.0))
  accel = MIN(accel, 0.7 - (AvoidSpeed/20));

 if (seg->type == TR_STR || (segtype == TR_STR && seg->radius > 100.0)) {
  //double tm = fabs(car->_trkPos.toMiddle) + 1.0f;
  //double w = (seg->width - car->_dimension_y)/2.0 ;
  //if (tm > w) {
  // return 0.0;
  if (cornerdist < cornerlimit && 
      ((prefer_side == TR_LFT && car->_trkPos.toLeft < 5.0) ||
       (prefer_side == TR_RGT && car->_trkPos.toRight < 5.0)))
  {
   return accel/2;
  } else {
   return accel;
  }
 } else {
  double tlane2left = tLane[rl][thisdiv] * car->_trkPos.seg->width;
  if ((seg->type == TR_RGT && nextleft < car->_trkPos.toLeft && nextleft < tlane2left-1.0) ||
      (seg->type == TR_LFT && nextleft > car->_trkPos.toLeft && nextleft > tlane2left+1.0))
  {
   return accel / (1.0 + fabs(nextleft-car->_trkPos.toLeft));
  }
 }
 return accel;
}


// Compute the needed distance to brake.
double Driver::brakedist(double allowedspeed, double mu)
{
 double c = mu*G;
 double d = (CA*mu + brakeCW)/mass;
 double v1sqr = currentspeedsqr;
 double v2sqr = allowedspeed*allowedspeed;
 return -log((c + v2sqr*d)/(c + v1sqr*d))/(2.0*d);
}

