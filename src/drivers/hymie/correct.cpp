/***************************************************************************

    file                 : correct.cpp
    created              : Wed Mai 14 20:10:00 CET 2003
    copyright            : (C) 2007 Andrew Sumner
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


#include <stdio.h>
#include <math.h>
#include "track.h"

#ifndef MAX
 #define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef MIN
 #define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#include "correct.h"

Correct::Correct(tCarElt *car)
 : car(car),
 correcting(0),
 old_correcting(0),
 allow_correcting(0),
 old_allow_correcting(0),
 aligned(0),
 segtype(0),
 correct_timer(0.0),
 correct_limiter(0.0),
 simtime(0.0)
{
 //this->car = car;
 //allow_correcting = old_allow_correcting = 0;
 //simtime = 0.0;
 setCorrecting(1);
}

Correct::~Correct()
{
}

void Correct::update(double cursimtime, int aligned, int segtype)
{
 if (!correcting)
 {
  this->aligned = aligned;
 }

 this->segtype = segtype;
 simtime = cursimtime;
 old_correcting = correcting;
 old_allow_correcting = allow_correcting;
}

void Correct::initCorrectValues()
{
 correct_timer = simtime + CORRECT_TIMER;
 correct_limiter = 1000.0;
}

void Correct::setCorrecting(int newcorrect)
{
 correcting = 0;

 if (newcorrect && !aligned)
 {
  correcting = 1;
  if (!old_correcting && !old_allow_correcting)
   initCorrectValues();
 }
}

void Correct::setAllowCorrecting(int newcorrect)
{
 allow_correcting = 0;

 if (newcorrect)
 {
  allow_correcting = 1;
  if (!old_correcting && !old_allow_correcting)
   initCorrectValues();
 }
}


double Correct::CorrectSteer(double steer, double linesteer, int slowcorrect, double speed, double accel)
{
 if (correct_limiter != 1000.0)
 {
  if (steer < linesteer)
  {
   if (correct_limiter >= 0.0)
    steer = linesteer;
   else
    steer = MIN(linesteer, MAX(steer, linesteer + correct_limiter));
  }
  else
  {
   if (correct_limiter <= 0.0)
    steer = linesteer;
   else
    steer = MAX(linesteer, MIN(steer, linesteer + correct_limiter));
  }
 }

 speed = MIN(70, (speed + (speed*speed/100))/2) - accel/2;

 if (linesteer > steer)
  steer = MIN(linesteer, steer + (90.0-speed)/12000);
 else
  steer = MAX(linesteer, steer - (90.0-speed)/12000);

 correct_limiter = (steer - linesteer) * 1.1;

 return steer;
}


#if 0
double Correct::CorrectSteer(double steer, double linesteer, double line2left, int slowcorrect, double correctdelay, double speed)
{
 // apply limiter
 if (correct_limiter != 1000.0)
 {
  if (steer < linesteer)
  {
   if (correct_limiter >= 0.0)
    steer = linesteer;
   else
    steer = MIN(linesteer, MAX(steer, linesteer + correct_limiter));
  }
  else
  {
   if (correct_limiter <= 0.0)
    steer = linesteer;
   else
    steer = MAX(linesteer, MIN(steer, linesteer + correct_limiter));
  }
 }

 double diff = MIN(2.0, fabs(steer - linesteer));
 if (slowcorrect)
  correct_timer += 0.02 - (slowcorrect == 1 ? 0.007 : 0.0);
 else
  correct_timer = MIN(correct_timer, simtime + diff*(CORRECT_TIMER*correctdelay));
 double ctime1 = MAX(0.0, (correct_timer - simtime) / (CORRECT_TIMER*correctdelay));
 double ctime2 = MAX(0.0, fabs(car->_trkPos.toLeft - line2left)/5.0);//(segtype == TR_STR ? 8.0 : 15.0));
 double ctime = ctime1;
 if (ctime2 < ctime1)
  ctime = (ctime + ctime2) / 2;
 ctime *= MAX(1.0, speed/25);

 // correct steering according to how much time is left
 if (ctime == 0.0)
 {
  steer = linesteer;
 }
 else
 {
  double cspeed = (segtype == TR_STR ? 1.2 : 1.2);

  //if (!slowcorrect)
  {
   if (steer < linesteer)
    steer += MIN(linesteer - steer, 0.012);
   else if (steer > linesteer)
    steer -= MIN(steer - linesteer, 0.012);
  }

  if (steer < linesteer)
   steer = MAX(steer, linesteer - ctime*cspeed);
  else
   steer = MIN(steer, linesteer + ctime*cspeed);
 }

 correct_limiter = (steer - linesteer) * 1.1;
 return steer;
}
#endif

