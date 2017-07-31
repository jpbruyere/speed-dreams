/***************************************************************************

    file                 : opponent.cpp
    created              : Thu Apr 22 01:20:19 CET 2003
    copyright            : (C) 2007 Andrew Sumner, 2003-2004 Bernhard Wymann
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

#include "opponent.h"
#include "strategy.h"
#include "name.h"


// class variables and constants.
tTrack* Opponent::track;
const float Opponent::FRONTCOLLDIST = 200.0;   // [m] distance on the track to check other cars.
const float Opponent::BACKCOLLDIST = 70.0;    // [m] distance on the track to check other cars.
const float Opponent::LENGTH_MARGIN = 1.5;    // [m] savety margin.
const float Opponent::SIDE_MARGIN = 1.0;    // [m] savety margin.
const float Opponent::TIME_MARGIN = 4.0;    // [m] savety margin.
const float Opponent::EXACT_DIST = 10.0;    // [m] if the estimated distance is smaller, compute it more accurate
const float Opponent::LAP_BACK_TIME_PENALTY = -30.0; // [s]
const float Opponent::OVERLAP_WAIT_TIME = 3.0;   // [s] overlaptimer must reach this time before we let the opponent pass.
const float Opponent::SPEED_PASS_MARGIN = 5.0;   // [m/s] avoid overlapping opponents to stuck behind me.

static float Mag(float x, float y)
{
 return (float) sqrt(x * x + y * y);
}

Opponent::Opponent()
{
 lastspeed = nextspeed = 0.0;
 team = 0;
 prevdistance = 1000.0;
 prevtime = 0.0;
}


double Opponent::testCollision(tPosd *c0, tPosd *c1, tPosd *c2, int point_test, tPosd *cp)
{
 // new polygon overlap collision method.
 // It works by checking whether any of the polygon lines of the robot's car intersects with
 // any lines of the opponent's car.  If so, the cars are going to collide (we're looking at
 // extrapolated polygons based on estimated collision time and their speeds).
 
 int cpos[4] = { 1, 0, 2, 3 };

 tPosd pc[4];
 pc[cpos[0]].ax = c1[cpos[0]].ax ;//+ (c1[cpos[0]].ax - c0[cpos[0]].ax) * 0.3;
 pc[cpos[0]].ay = c1[cpos[0]].ay ;//+ (c1[cpos[0]].ay - c0[cpos[0]].ay) * 0.3;
 pc[cpos[1]].ax = c1[cpos[1]].ax ;//+ (c1[cpos[1]].ax - c0[cpos[1]].ax) * 0.3;
 pc[cpos[1]].ay = c1[cpos[1]].ay ;//+ (c1[cpos[1]].ay - c0[cpos[1]].ay) * 0.3;
 pc[cpos[2]].ax = c1[cpos[2]].ax;
 pc[cpos[2]].ay = c1[cpos[2]].ay;
 pc[cpos[3]].ax = c1[cpos[3]].ax;
 pc[cpos[3]].ay = c1[cpos[3]].ay;

 cp->ax = cp->ay = 1000000.0;

 // first we see if the polygons overlap
 int i, j;

#if 0
 for (j=0; j<4; j++)
 {
  tPosd *j1 = pc + cpos[j];
  tPosd *j2 = pc + cpos[(j+1) % 4];

  double div = (j1->ax == j2->ax ? 0.001 : j1->ax - j2->ax);
  double jm = (j1->ay - j2->ay) / div;

  for (i=0; i<4; i++)
  {
   tPosd *i1 = c2 + cpos[i];
   tPosd *i2 = c2 + cpos[(i+1) % 4];

   div = (i1->ax == i2->ax ? 0.001 : i1->ax - i2->ax);
   double im = (i1->ay - i2->ay) / div;

   if (im == jm)
   {
    // parallel lines
    continue;
   }

   double jc = j1->ay - (jm * j1->ax);
   double ic = i1->ay - (im * i1->ax);
   double ix = (ic - jc) / (jm - im);

   if (ix < MIN(j2->ax, j1->ax) || ix > MAX(j2->ax, j1->ax))
    continue;

   if (ix < MIN(i2->ax, i1->ax) || ix > MAX(i2->ax, i1->ax))
    continue;

   double iy = (jm * ix) + jc;

   if ((fabs(ix - c0[0].ax) + fabs(ix - c0[1].ax) + fabs(iy - c0[0].ay) + fabs(iy - c0[0].ay)) / 4
       < (fabs(cp->ax - c0[0].ax) + fabs(cp->ax - c0[1].ax) + fabs(cp->ay - c0[0].ay) + fabs(cp->ay - c0[0].ay)) / 4)
   {
    cp->ax = ix;
    cp->ay = iy;
   }

   return 1.0;
  }
 }
#else
 double collision = -100.0;

 for (j=0; j<4; j++)
 {
  tPosd *j1 = pc + cpos[j];
  tPosd *j2 = pc + cpos[(j+1) % 4];

  for (i=0; i<4; i++)
  {
   tPosd *i1 = c2 + cpos[i];
   tPosd *i2 = c2 + cpos[(i+1) % 4];

   double aM, bM, aB, bB, isX=0, isY=0;
   double lineAx1 = j1->ax;
   double lineAx2 = j2->ax;
   double lineAy1 = j1->ay;
   double lineAy2 = j2->ay;
   double lineBx1 = i1->ax;
   double lineBx2 = i2->ax;
   double lineBy1 = i1->ay;
   double lineBy2 = i2->ay;

   if ((lineAx2 - lineAx1) == 0)
   {
    isX = lineAx1;
    bM = (lineBy2 - lineBy1) / (lineBx2 - lineBx1);
    bB = lineBy2 - bM * lineBx2;
    isY = bM * isX + bB;
   }
   else if ((lineBx2 - lineBx1) == 0 )
   {
    isX = lineBx1;
    aM = (lineAy2 - lineAy1) / (lineAx2 - lineAx1);
    aB = lineAy2 - aM * lineAx2;
    isY = aM * isX + aB;
   }
   else
   {
    aM = (lineAy2 - lineAy1) / (lineAx2 - lineAx1);
    bM = (lineBy2 - lineBy1) / (lineBx2 - lineBx1);
    aB = lineAy2 - aM * lineAx2;
    bB = lineBy2 - bM * lineBx2;
    isX = MAX(((bB - aB) / (aM - bM)), 0);
    isY = aM * isX + aB;
   }

   if (isX < MIN(lineAx1, lineAx2) || isX < MIN(lineBx1, lineBx2) || isX > MAX(lineAx1, lineAx2) || isX > MAX(lineBx1, lineBx2))
    continue;
   if (isY < MIN(lineAy1, lineAy2) || isY < MIN(lineBy1, lineBy2) || isY > MAX(lineAy1, lineAy2) || isY > MAX(lineBy1, lineBy2))
    continue;

   if ((fabs(isX - c0[0].ax) + fabs(isX - c0[1].ax) + fabs(isY - c0[0].ay) + fabs(isY - c0[0].ay)) / 4
       < (fabs(cp->ax - c0[0].ax) + fabs(cp->ax - c0[1].ax) + fabs(cp->ay - c0[0].ay) + fabs(cp->ay - c0[0].ay)) / 4)
   {
    cp->ax = isX;
    cp->ay = isY;
    collision = 1.0;
   }
  }
 }

 if (collision >= 0.0)
  return collision;
#endif

 //if (!point_test)
  return -100.0;

 // polygons don't overlap - try the point test
 tPosd c[9];

 c[0].ax = c1[0].ax;
 c[0].ay = c1[0].ay;
 c[1].ax = c1[1].ax;
 c[1].ay = c1[1].ay;
 c[2].ax = (c1[0].ax + c1[1].ax) / 2;
 c[2].ax = (c1[0].ay + c1[1].ay) / 2;
 c[3].ax = c1[0].ax + (c1[0].ax - c0[0].ax) * 0.3;
 c[3].ay = c1[0].ay + (c1[0].ay - c0[0].ay) * 0.3;
 c[4].ax = c1[1].ax + (c1[1].ax - c0[1].ax) * 0.3;
 c[4].ay = c1[1].ay + (c1[1].ay - c0[1].ay) * 0.3;
 c[5].ax = (c[3].ax + c[4].ax) / 2;
 c[5].ax = (c[3].ay + c[4].ay) / 2;
 c[6].ax = c1[0].ax + (c1[0].ax - c0[0].ax) * 0.6;
 c[6].ay = c1[0].ay + (c1[0].ay - c0[0].ay) * 0.6;
 c[7].ax = c1[1].ax + (c1[1].ax - c0[1].ax) * 0.6;
 c[7].ay = c1[1].ay + (c1[1].ay - c0[1].ay) * 0.6;
 c[8].ax = (c[6].ax + c[7].ax) / 2;
 c[8].ax = (c[6].ay + c[7].ay) / 2;

 double maxcolldist = -100.0;

 for (j=0; j<9; j++)
 {
  tPosd *p = c + j;
  int inside = 1;
  double colldist = 100.0;
//fprintf(stderr,"%.1f,%.1f ",p->ax,p->ay);

  for (i=1; i<=4; i++)
  {
   tPosd *p0 = c2 + (cpos[i-1]);
   tPosd *p1 = c2 + (i == 4 ? cpos[0] : cpos[i]);

   double v0x, v0y, v1x, v1y;
   v0x = p->ax - p0->ax;
   v0y = p->ay - p0->ay;
   v1x = p1->ax - p0->ax;
   v1y = p1->ay - p0->ay;

   if (0 > (colldist = MIN(colldist, (v0x * v1y) - (v0y * v1x))))
   {
    inside = 0;
   }
  }

  if (inside)
  {
//fprintf(stderr,"\n");
   return colldist;
  }

  maxcolldist = MAX(maxcolldist, colldist);
 }
//fprintf(stderr,"\n");

 return maxcolldist;
}

void Opponent::update(tSituation *s, Driver *driver)
{
 tCarElt *mycar = driver->getCarPtr();
 tTrackSeg *seg = mycar->_trkPos.seg;
 tTrackSeg *cseg = car->_trkPos.seg;
 double speed = getSpeed();
 int i;
 nextspeed = speed + (speed - lastspeed);
 lastspeed = speed;

 if (team == 0)
 {
  if ((!strcmp(car->_teamname, mycar->_teamname)))
   team = TEAM_CONTROL;
  else
   team = TEAM_KAOS;
  prevleft = car->_trkPos.toLeft;
  prevwidth = cardata->getWidthOnTrack();

  for (i=0; i<4; i++)
  {
   corner1[i].ax = corner2[i].ax = car->_corner_x(i);
   corner1[i].ay = corner2[i].ay = car->_corner_y(i);
  }
 }

 nextleft = car->_trkPos.toLeft + (car->_trkPos.toLeft - prevleft);

 double width = cardata->getWidthOnTrack();
 nextwidth = width + (width - prevwidth);

 // Init state of opponent to ignore.
 state = OPP_IGNORE;

 double trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
 angle = trackangle - car->_yaw;
 NORM_PI_PI(angle);

 trackangle = RtTrackSideTgAngleL(&(mycar->_trkPos));
 double myangle = trackangle - mycar->_yaw;
 NORM_PI_PI(myangle);
 double anglediff = MAX(fabs(angle-myangle), MAX(fabs(angle), fabs(myangle)));

 // If the car is out of the simulation ignore it.
 if (car == mycar || (car->_state & RM_CAR_STATE_NO_SIMU)) {
  return;
 }

 // Updating distance along the middle.
 double oppToStart = cseg->lgfromstart + getDistToSegStart();
 distance = oppToStart - mycar->_distFromStartLine;
 //distance = RtGetDistFromStart(car) - RtGetDistFromStart(mycar);
 
 if (distance > track->length/2.0) {
  distance -= track->length;
 } else if (distance < -track->length/2.0) {
  distance += track->length;
 }
 
 // If the distance is small we compute it more accurate.
 if (distance < EXACT_DIST || (mycar->_trkPos.seg->type != TR_STR && distance < EXACT_DIST*5)) {
  Straight carFrontLine(
   mycar->_corner_x(FRNT_LFT),
   mycar->_corner_y(FRNT_LFT),
   mycar->_corner_x(FRNT_RGT) - mycar->_corner_x(FRNT_LFT),
   mycar->_corner_y(FRNT_RGT) - mycar->_corner_y(FRNT_LFT)
  );

  double mindist = 1000.0;
  for (i = 0; i < 4; i++) {
   v2d corner(car->_corner_x(i), car->_corner_y(i));
   double dist = carFrontLine.dist(corner);
   if (dist < mindist) {
    mindist = dist;
   }
  }

  closedistance = mindist;
  distance = MIN(distance, mindist + car->_dimension_x);
 }

 if (prevdistance != 1000.0)
  nextdistance = distance + (distance - prevdistance);
 else
  nextdistance = distance;
 prevdistance = distance;


 double SIDECOLLDIST = MAX(cardata->getLengthOnTrack(), MAX(driver->getLength(), MAX(mycar->_dimension_x, car->_dimension_x)));
 double ut = Mag(car->_speed_Y, car->_speed_X) + car->_accel_x/5;
 //double st = driver->getSlowestSpeed(distance);
 double u0 = Mag(mycar->_speed_Y, mycar->_speed_X) + mycar->_accel_x/5;
 double dwidth = driver->getWidth();
 //if (st < u0)
 // u0 = (st + u0) / 2.0;
 t_impact = 1000.0;
 s_impact = 1000.0;
 double deltatime = s->currentTime - prevtime;
 double osa = ((car->_trkPos.toLeft - prevleft)*0.8) * (t_impact/deltatime);
 double dsa = ((driver->getNextLeft() - mycar->_trkPos.toLeft)*0.8) * (t_impact/deltatime);
 double oleft = car->_trkPos.toLeft + osa*0.8;
 double dleft = mycar->_trkPos.toLeft + dsa*0.8;

 //if (MAX(mycar->_speed_x, u0) > MIN(car->_speed_x, ut))
 if (u0 > ut)
 {
  //double sidedistance = (fabs(car->_trkPos.toLeft - mycar->_trkPos.toLeft) - MAX(car->_dimension_y, mycar->_dimension_y))/3;
  //double sidedistance = (fabs(car->_trkPos.toLeft - mycar->_trkPos.toLeft) > MAX(dwidth, width) ? car->_dimension_x + 1.0 : 0.0);
  double sidedistance = MAX(0.0, fabs(car->_trkPos.toLeft - mycar->_trkPos.toLeft) - (dwidth/2 + MAX(width, nextwidth)/2));

  if ((team & TEAM_CONTROL) && driver->Alone())
   t_impact = MAX(0.0,((distance)-(car->_dimension_x+0.5))/(u0 - ut));
  else
   t_impact = MAX(0.0,((distance)-(car->_dimension_x+0.9))/(u0 - ut));

  osa = ((car->_trkPos.toLeft - prevleft)*0.8) * (t_impact/deltatime);
  dsa = ((driver->getNextLeft() - mycar->_trkPos.toLeft)*0.8) * (t_impact/deltatime);
  oleft = car->_trkPos.toLeft + osa/2;
  dleft = mycar->_trkPos.toLeft + dsa/2;

  if (sidedistance > 0.5)
  {
   if ((car->_trkPos.toLeft > mycar->_trkPos.toLeft && dsa > osa) ||
       (car->_trkPos.toLeft < mycar->_trkPos.toLeft && dsa < osa))
   {
    s_impact = MIN(1000.0, (sidedistance / ((fabs(dsa - osa) / deltatime) * 0.8)));

    double t_pass = MAX(0.0, ((distance+1.0) / (u0-ut)));
    if (t_pass < s_impact)
     s_impact = 1000.0;
   }
   else
    s_impact = 1000.0;
  }
  //if ((team & TEAM_CONTROL) && driver->Alone())
  // t_impact = MAX(0.0,((distance+sidedistance)-(car->_dimension_x+0.8))/(u0 - ut));
  //else
  // t_impact = MAX(0.0,((distance+sidedistance)-(car->_dimension_x+1.2))/(u0 - ut));
 }
 //double cardist = car->_trkPos.toMiddle - mycar->_trkPos.toMiddle;
 double ow = width + (nextwidth-width) * (t_impact / deltatime) / 2;
 double dw = dwidth + (driver->getNextWidth() - dwidth) * (t_impact / deltatime) / 2;
 if (ow < car->_dimension_y)
  ow = MIN(car->_dimension_x, car->_dimension_y + (car->_dimension_y - ow));
 else if (ow > car->_dimension_x)
  ow = MAX(car->_dimension_y, ow - (ow - car->_dimension_x));
 if (dw < mycar->_dimension_y)
  dw = MIN(mycar->_dimension_x, mycar->_dimension_y + (mycar->_dimension_y - dw));
 else if (ow > mycar->_dimension_x)
  dw = MAX(mycar->_dimension_y, dw - (dw - mycar->_dimension_x));

 double dnextleft = driver->getNextLeft();
 double dspeed = driver->getSpeed();
 double dnext2 = dnextleft;
 nextleft2 = nextleft + (nextleft - car->_trkPos.toLeft) * (t_impact > 0.0 ? MIN(150.0, ((t_impact*0.8)/deltatime)) : 0.0);

 lastLoffset = Loffset;
 lastRoffset = Roffset;
 Loffset = nextleft - width/2;
 Roffset = nextleft + width/2;
 nextLoffset = nextleft2 - width/2;
 nextRoffset = nextleft2 + width/2;

 double cardist1 = fabs(dnextleft - nextleft);
 double cardist2 = cardist1;
 tPosd ncorner[4], myncorner[4];
 tPosd *mycorner = driver->getCornerPos();
 //tPosd *mycorner2 = driver->getCornerPos2();

 if ((distance < 25.0 || t_impact < TIME_MARGIN) && distance > car->_dimension_x)
 {
  if (!driver->getAvoiding() && !driver->getCorrecting() && distance - car->_dimension_x > MAX(5.0, cardist2 * 1.8))
  {
   dnext2 = driver->getLaneLeft(car, (getSpeed() > 5.0 ? (dspeed * (distance-car->_dimension_x)/((dspeed-getSpeed())*1.5)) / 10 : 0.0), t_impact*0.7);
   cardist2 = fabs(dnext2 - nextleft);
  }
  else
  {
   if (distance - car->_dimension_x > 7.0)
   {
    cardist2 = (t_impact <= deltatime
      ? cardist1
      : (dnextleft + (dnextleft - mycar->_trkPos.toLeft) * MIN(150.0f, ((t_impact*0.8)/deltatime))) -
        (nextleft + (nextleft - car->_trkPos.toLeft) * MIN(150.0f, ((t_impact*0.8)/deltatime))));
   }
   dnext2 = dnextleft + (dnextleft - mycar->_trkPos.toLeft) * MIN(150.0, ((t_impact*0.8)/deltatime));
  }

  for (i=0; i<4; i++)
  {
   double ti = MAX(0.01, t_impact * 0.8);
   ncorner[i].ax = car->_corner_x(i) + (car->_corner_x(i) - corner1[i].ax) * (ti / deltatime);
   ncorner[i].ay = car->_corner_y(i) + (car->_corner_y(i) - corner1[i].ay) * (ti / deltatime);
   ti = MAX(0.01, t_impact);
   myncorner[i].ax = mycar->_corner_x(i) + (mycar->_corner_x(i) - mycorner[i].ax) * MAX(0.0, (ti+MAX(0.03, MIN(0.08, (dspeed-speed)/100))) / deltatime);// + 1.0 + (dspeed-speed)*1.5);
   myncorner[i].ay = mycar->_corner_y(i) + (mycar->_corner_y(i) - mycorner[i].ay) * MAX(0.0, (ti+MAX(0.03, MIN(0.08, (dspeed-speed)/100))) / deltatime);// + 1.0 + (dspeed-speed)*1.5);
  }
  if (anglediff > 0.01)
  {
   // make the other car a tad bigger so we're sure to avoid it.
   /*
   ncorner[FRNT_RGT].ax += ((ncorner[FRNT_RGT].ax - ncorner[FRNT_LFT].ax) / (car->_dimension_y)) * anglediff;
   ncorner[FRNT_RGT].ay += ((ncorner[FRNT_RGT].ay - ncorner[FRNT_LFT].ay) / (car->_dimension_y)) * anglediff;
   ncorner[FRNT_LFT].ax += ((ncorner[FRNT_LFT].ax - ncorner[FRNT_RGT].ax) / (car->_dimension_y)) * anglediff;
   ncorner[FRNT_LFT].ay += ((ncorner[FRNT_LFT].ay - ncorner[FRNT_RGT].ay) / (car->_dimension_y)) * anglediff;
   */
   ncorner[REAR_RGT].ax += ((ncorner[REAR_RGT].ax - ncorner[REAR_LFT].ax) / (car->_dimension_y)) * anglediff;
   ncorner[REAR_RGT].ay += ((ncorner[REAR_RGT].ay - ncorner[REAR_LFT].ay) / (car->_dimension_y)) * anglediff;
   ncorner[REAR_LFT].ax += ((ncorner[REAR_LFT].ax - ncorner[REAR_RGT].ax) / (car->_dimension_y)) * anglediff;
   ncorner[REAR_LFT].ay += ((ncorner[REAR_LFT].ay - ncorner[REAR_RGT].ay) / (car->_dimension_y)) * anglediff;

   /*
   ncorner[FRNT_RGT].ax += ((ncorner[FRNT_RGT].ax - ncorner[REAR_RGT].ax) / (car->_dimension_x)) * anglediff;
   ncorner[FRNT_RGT].ay += ((ncorner[FRNT_RGT].ay - ncorner[REAR_RGT].ay) / (car->_dimension_x)) * anglediff;
   ncorner[FRNT_LFT].ax += ((ncorner[FRNT_LFT].ax - ncorner[REAR_LFT].ax) / (car->_dimension_x)) * anglediff;
   ncorner[FRNT_LFT].ay += ((ncorner[FRNT_LFT].ay - ncorner[REAR_LFT].ay) / (car->_dimension_x)) * anglediff;
   */
   ncorner[REAR_RGT].ax += ((ncorner[REAR_RGT].ax - ncorner[FRNT_RGT].ax) / (car->_dimension_x)) * anglediff;
   ncorner[REAR_RGT].ay += ((ncorner[REAR_RGT].ay - ncorner[FRNT_RGT].ay) / (car->_dimension_x)) * anglediff;
   ncorner[REAR_LFT].ax += ((ncorner[REAR_LFT].ax - ncorner[FRNT_LFT].ax) / (car->_dimension_x)) * anglediff;
   ncorner[REAR_LFT].ay += ((ncorner[REAR_LFT].ay - ncorner[FRNT_LFT].ay) / (car->_dimension_x)) * anglediff;
  }
  if (fabs(angle) < 0.1)
  {
   ncorner[FRNT_RGT].ax -= ((ncorner[FRNT_RGT].ax - ncorner[FRNT_LFT].ax) / (car->_dimension_y)) * 0.5;
   ncorner[FRNT_RGT].ay -= ((ncorner[FRNT_RGT].ay - ncorner[FRNT_LFT].ay) / (car->_dimension_y)) * 0.5;
   ncorner[FRNT_LFT].ax -= ((ncorner[FRNT_LFT].ax - ncorner[FRNT_RGT].ax) / (car->_dimension_y)) * 0.5;
   ncorner[FRNT_LFT].ay -= ((ncorner[FRNT_LFT].ay - ncorner[FRNT_RGT].ay) / (car->_dimension_y)) * 0.5;
  }

  if (cseg->type == TR_LFT && cseg->radius <= 120.0)
  {
   myncorner[FRNT_LFT].ax += ((myncorner[FRNT_LFT].ax - myncorner[FRNT_RGT].ax) / (mycar->_dimension_y)) * ((1.0 - cseg->radius/125.0) * 2);
   ncorner[FRNT_LFT].ax += ((ncorner[FRNT_LFT].ax - ncorner[FRNT_RGT].ax) / (car->_dimension_y)) * ((1.0 - cseg->radius/125.0) * 2);
   ncorner[REAR_LFT].ax += ((ncorner[REAR_LFT].ax - ncorner[REAR_RGT].ax) / (car->_dimension_y)) * ((1.0 - cseg->radius/125.0) * 2);
  }
  else if (cseg->type == TR_RGT && cseg->radius <= 120.0)
  {
   myncorner[FRNT_RGT].ax += ((myncorner[FRNT_RGT].ax - myncorner[FRNT_LFT].ax) / (mycar->_dimension_y)) * ((1.0 - cseg->radius/125.0) * 2);
   ncorner[FRNT_RGT].ax += ((ncorner[FRNT_RGT].ax - ncorner[FRNT_LFT].ax) / (car->_dimension_y)) * ((1.0 - cseg->radius/125.0) * 2);
   ncorner[REAR_RGT].ax += ((ncorner[REAR_RGT].ax - ncorner[REAR_LFT].ax) / (car->_dimension_y)) * ((1.0 - cseg->radius/125.0) * 2);
  }
  {
   myncorner[FRNT_LFT].ax += ((myncorner[FRNT_LFT].ax-myncorner[REAR_LFT].ax)/mycar->_dimension_x)/2;
   myncorner[FRNT_LFT].ay += ((myncorner[FRNT_LFT].ay-myncorner[REAR_LFT].ay)/mycar->_dimension_x)/2;
   myncorner[FRNT_RGT].ax += ((myncorner[FRNT_RGT].ax-myncorner[REAR_RGT].ax)/mycar->_dimension_x)/2;
   myncorner[FRNT_RGT].ay += ((myncorner[FRNT_RGT].ay-myncorner[REAR_RGT].ay)/mycar->_dimension_x)/2;
  }
 }

 if (width > car->_dimension_y)
  cardist2 = MAX(0.0, cardist2 - (width - car->_dimension_y)/2);
 if (dwidth > mycar->_dimension_y)
 {
  dwidth += (dwidth-mycar->_dimension_y);
  cardist2 = MAX(0.0, cardist2 - (dwidth - mycar->_dimension_y)/2);
 }
 //if (car->_trkPos.seg->type != TR_STR && car->_trkPos.seg->radius <= 80.0)
 // cardist2 = MAX(0.0, cardist2 - ((90.0-car->_trkPos.seg->radius) / 10));

 sidedist = cardist2;//MIN(cardist1, cardist2);
 double cardist = fabs(sidedist) - fabs(getWidth()/2.0) - dwidth/2.0;
 brake_speed = car->_speed_x;
 brakedistance = nextdistance;

 if (0 && seg->type != TR_STR && seg->radius <= 200.0 && fabs(speed) > 10.0)
 {
  nextLoffset -= MIN((210.0-seg->radius) * ((seg->type == TR_LFT ? 0.030 - MIN(0.02, car->_trkPos.toLeft/500) : 0.018)), fabs(dspeed-(speed+car->_accel_x/2)) - t_impact) / 2;
  nextRoffset += MIN((210.0-seg->radius) * ((seg->type == TR_RGT ? 0.030 - MIN(0.02, car->_trkPos.toRight/500) : 0.018)), fabs(dspeed-(speed+car->_accel_x/2)) - t_impact) / 2;
 }

 t_impact = MAX(0.07, t_impact);

 if (nextleft < car->_trkPos.toLeft)
  Loffset -= MIN(car->_trkPos.toLeft - nextleft, fabs(car->_speed_x));
 else if (nextleft > car->_trkPos.toLeft)
  Roffset += MIN(nextleft - car->_trkPos.toLeft, fabs(car->_speed_x));

 sidegap = MIN(fabs((car->_trkPos.toLeft-(width/2)) - (mycar->_trkPos.toLeft+(dwidth/2))), fabs((car->_trkPos.toLeft+(width/2)) - (mycar->_trkPos.toLeft-(dwidth/2))));

 int closing = (sidegap < prevsidegap*0.7 ? CLOSING_FAST : sidegap < prevsidegap*0.9 ? CLOSING : 0);
 if (closing == CLOSING_FAST)// || ((car->_trkPos.seg->type != TR_STR || mycar->_trkPos.seg->type != TR_STR) && closing))
 {
  nextLoffset += MAX(fabs(dnextleft-mycar->_trkPos.toLeft), fabs(nextleft-car->_trkPos.toLeft)) * 2;//4;
  nextRoffset += MAX(fabs(dnextleft-mycar->_trkPos.toLeft), fabs(nextleft-car->_trkPos.toLeft)) * 2;//4;
 }

 // Is opponent in relevant range -BACKCOLLDIST..FRONTCOLLDIST m.
 if (distance > -BACKCOLLDIST && distance < FRONTCOLLDIST) 
 {
  double sidemargin = driver->getSideMargin();
  if (cseg->type == TR_LFT && cseg->radius <= 120.0 && dnextleft < nextleft - (width+2.0) && car->_trkPos.toLeft-width/2 > 4.0 && dspeed-speed < (130.0-cseg->radius)/10)
   sidemargin += (130.0-seg->radius) / (mycar->_trkPos.toLeft-width/2 < 5.0 ? 30.0 : 15.0);
  else if (cseg->type == TR_RGT && cseg->radius <= 120.0 && dnextleft > nextleft + (width+2.0) && car->_trkPos.toLeft-width/2 > 4.0 && dspeed-speed < (130.0-cseg->radius)/10)
   sidemargin += (130.0-cseg->radius) / (mycar->_trkPos.toRight-width/2 < 5.0 ? 30.0 : 15.0);

  // Is opponent in front and slower.
  if (((distance > SIDECOLLDIST) 
     || (distance >= 0.0 && distance <= SIDECOLLDIST && cardist < sidemargin)) 
    && nextspeed <= dspeed + 2.5f) {
   state |= OPP_FRONT;

   if ((seg->type == TR_LFT && seg->radius <= 60.0 && mycar->_trkPos.toLeft < nextleft-1.0) ||
       (seg->type == TR_RGT && seg->radius <= 60.0 && mycar->_trkPos.toLeft > nextleft+1.0))
   {
    double d = (65.0 - seg->radius) / 60.0;
    d *= d;
    SIDECOLLDIST = MIN(SIDECOLLDIST + 0.7, SIDECOLLDIST + d);
   }

   if ((team & TEAM_CONTROL) && driver->Alone() &&
       car->_dammage + 500 >= mycar->_dammage)
   {
    state |= OPP_FOLLOW;
   }

   brakedistance = distance - car->_dimension_x;

   if (brakedistance > 0.0)
    brakedistance = MAX(0.1, brakedistance - ((team & TEAM_CONTROL) && driver->Alone() ? LENGTH_MARGIN-(seg->type==TR_STR || seg->radius >= 300 ? 1.3 : 0.7) : LENGTH_MARGIN) / 2);
   
   double speedfactor = 0.0, cdist = distance;
   if (fabs(sidedist) > 1.5 && dspeed > speed && cdist-car->_dimension_x < 8.0 && fabs(mycar->_trkPos.toMiddle) < fabs(car->_trkPos.toMiddle))
    speedfactor = (dspeed-speed)/(cseg->type == TR_STR ? 3 : 5);

   if (distance <= SIDECOLLDIST)
   {
    if (cseg->radius <= 30.0 && sidedist < 10.0 &&
        ((cseg->type == TR_LFT && nextleft < mycar->_trkPos.toLeft) ||
         (cseg->type == TR_RGT && nextleft > mycar->_trkPos.toLeft)))
    {
     state |= OPP_COLL;
    }
    state |= OPP_SIDE;
   }
   //else if (distance-speedfactor <= SIDECOLLDIST && ((team & TEAM_CONTROL) || !driver->Alone()))
   // state |= (OPP_SIDE|OPP_SIDE_FRONT);

   catchdist = driver->getSpeed()*distance/(driver->getSpeed() - getSpeed());

   /*
   if (brakedistance < sidemargin && t_impact >= TIME_MARGIN)
    t_impact = TIME_MARGIN-0.1;
   else if (brakedistance > 10 * sidemargin && t_impact < -TIME_MARGIN)
    t_impact = TIME_MARGIN + 1;
    */

   double time_margin = MAX(TIME_MARGIN, (dspeed-speed) / 16);
   /*
   if (driver->getStrategy() == STRATEGY_CAREFUL)
    t_impact *= 1.4f;
   else if (driver->getStrategy() == STRATEGY_PASSIVE)
    time_margin *= 1.8f;
   if (mycar->_dammage > 2000 && driver->getStrategy() != STRATEGY_DESPERATE && nextdistance < 10.0)
    t_impact -= (double) (mycar->_dammage-2000) * 0.0006;
      */

   //double mymin = dnext2 - (dwidth/2 + fabs(dnext2-dnextleft)/4);
   //double mymax = dnext2 + dwidth/2 + fabs(dnext2-dnextleft)/4;
#if 0
   if (mycar->_trkPos.seg->type != TR_STR && mycar->_trkPos.seg->radius <= 120.0)
   {
    mymin -= (130.0-mycar->_trkPos.seg->radius) * 0.01;
    mymax += (130.0-mycar->_trkPos.seg->radius) * 0.01;
   }
#endif

   int collide = 0;

#if 0
   if (t_impact < 2.0 && brakedistance < 3.0 &&
       //sidegap < brakedistance * 3 &&
       (closing==CLOSING_FAST || !(state & OPP_SIDE) || (state & OPP_SIDE_FRONT)))
#endif
   if (s_impact == 1000.0)
    s_impact = t_impact;

   if (MAX(t_impact, s_impact) < time_margin && fabs(car->_trkPos.toLeft - mycar->_trkPos.toLeft) < 8.0)
   {
    tPosd collpoint;
    double colldist = testCollision(mycar->pub.corner, myncorner, ncorner, 1, &collpoint);
    if (colldist < 0.0)
    {
     // try again, but trace polygon from car's originating point.
     // this can pick up cases where the cars are crossing paths very quickly.
     for (i=2; i<4; i++)
     {
      myncorner[i].ax = mycar->_corner_x(i-2);
      myncorner[i].ay = mycar->_corner_y(i-2);
     }
     colldist = testCollision(mycar->pub.corner, myncorner, ncorner, 1, &collpoint);
     if (colldist < 0.0 && MAX(t_impact, s_impact) < time_margin/2)
         // && (team == TEAM_CONTROL || fabs(ut-u0) > MAX(2.0, brakedistance)))// && team == TEAM_CONTROL)
     {
      // sanity check for cars that are about to plow full speed into the back of other cars
      double margin = dwidth + 0.5; //+ 0.5 + (cseg->type == TR_STR || cseg->radius > 200.0 ? 0.0
                      //: ((cseg->type == TR_LFT && oleft > dleft) || (cseg->type == TR_RGT && oleft < dleft) 
                       //  ? (205.0 - cseg->radius)/45 : 0.0));
      double time = 0.6;
      double mti = MAX(t_impact, s_impact);
      //if (s_impact != t_impact && team != TEAM_CONTROL)
      // mti = MAX(t_impact, s_impact * 1.5);
      //if (seg->type != TR_STR && seg->radius <= 140.0)
      // time -= (150/seg->radius)/50;

      if (mti < time || (team == TEAM_CONTROL && mti < 3.0))
      {
       double addition = 0.25;//0.5;
       if (team != TEAM_CONTROL && fabs(car->_trkPos.toMiddle) > fabs(mycar->_trkPos.toMiddle))
       {
        margin *= 1.0;
       }
       //else if (team != TEAM_CONTROL && ((seg->type == TR_LFT && car->_trkPos.toLeft < mycar->_trkPos.toLeft) || (seg->type == TR_RGT && car->_trkPos.toLeft > mycar->_trkPos.toLeft)))
       // addition = -(ow/2 + dw/2) * 0.75;

       if (fabs(oleft - dleft) < ow/2 + dw/2 + addition ||
           (mti < 0.8 &&
             (((cseg->type != TR_STR && cseg->radius <= 120.0) || (seg->type != TR_STR && seg->radius <= 120.0)) &&
              ((oleft - ow/2 < margin && dleft - dw/4 < oleft + ow/2) ||
               (oleft + ow/2 > cseg->width-margin && dleft + dw/4 > oleft - ow/2)))))
       {
        colldist = 1.0;
       }
      }
#if 0
      if (!colldist && MAX(t_impact, s_impact) < 0.8)
      {
       if ((car->_trkPos.toLeft < dwidth + width*0.6 && mycar->_trkPos.toLeft < car->_trkPos.toLeft + width && driver->getNextLeft() <= mycar->_trkPos.toLeft && driver->getNextLeft() < nextleft + width) ||
               (car->_trkPos.toLeft > seg->width - (dwidth+width*0.6) && mycar->_trkPos.toLeft > car->_trkPos.toLeft - width && driver->getNextLeft() >= mycar->_trkPos.toLeft && driver->getNextLeft() > nextleft - width))
        colldist = 1.0;
      }
#endif
     }
    }
    if (colldist >= 0.0)
     collide = 1;
    else if (((seg->type == TR_LFT && seg->radius <= 200.0 && mycar->_trkPos.toLeft < nextleft && nextleft-mycar->_trkPos.toLeft < (dspeed-speed) * 4) ||
              (seg->type == TR_RGT && seg->radius <= 200.0 && mycar->_trkPos.toLeft > nextleft && mycar->_trkPos.toLeft-nextleft < (dspeed-speed) * 4)) &&
             (colldist >= 0.0 - (1.0-(seg->radius/201.0)) * 2))
     collide = 1;
#if 0
fprintf(stderr,"%s->%s A %d ti=%.3f (%.3f/%.3f-%.3f/%.3f) bd=%.3f dmg=%d\n",mycar->_name,car->_name,collide,t_impact,mycar->_speed_x,dspeed,car->_speed_x,speed,brakedistance,car->_dammage);
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#000000\"/>\n",myncorner[FRNT_LFT].ax,myncorner[FRNT_LFT].ay,myncorner[FRNT_RGT].ax,myncorner[FRNT_RGT].ay);
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#000000\"/>\n",myncorner[FRNT_RGT].ax,myncorner[FRNT_RGT].ay,myncorner[REAR_RGT].ax,myncorner[REAR_RGT].ay);
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#000000\"/>\n",myncorner[REAR_RGT].ax,myncorner[REAR_RGT].ay,myncorner[REAR_LFT].ax,myncorner[REAR_LFT].ay);
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#000000\"/>\n",myncorner[REAR_LFT].ax,myncorner[REAR_LFT].ay,myncorner[FRNT_LFT].ax,myncorner[FRNT_LFT].ay);
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#AA0000\"/>\n",ncorner[FRNT_LFT].ax,ncorner[FRNT_LFT].ay,ncorner[FRNT_RGT].ax,ncorner[FRNT_RGT].ay);
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#AA0000\"/>\n",ncorner[FRNT_RGT].ax,ncorner[FRNT_RGT].ay,ncorner[REAR_RGT].ax,ncorner[REAR_RGT].ay);
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#AA0000\"/>\n",ncorner[REAR_RGT].ax,ncorner[REAR_RGT].ay,ncorner[REAR_LFT].ax,ncorner[REAR_LFT].ay);
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#AA0000\"/>\n",ncorner[REAR_LFT].ax,ncorner[REAR_LFT].ay,ncorner[FRNT_LFT].ax,ncorner[FRNT_LFT].ay);
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#777777\"/>\n",mycar->_corner_x(FRNT_LFT),mycar->_corner_y(FRNT_LFT),mycar->_corner_x(FRNT_RGT),mycar->_corner_y(FRNT_RGT));
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#777777\"/>\n",mycar->_corner_x(FRNT_RGT),mycar->_corner_y(FRNT_RGT),mycar->_corner_x(REAR_RGT),mycar->_corner_y(REAR_RGT));
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#777777\"/>\n",mycar->_corner_x(REAR_RGT),mycar->_corner_y(REAR_RGT),mycar->_corner_x(REAR_LFT),mycar->_corner_y(REAR_LFT));
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#777777\"/>\n",mycar->_corner_x(REAR_LFT),mycar->_corner_y(REAR_LFT),mycar->_corner_x(FRNT_LFT),mycar->_corner_y(FRNT_LFT));
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#FF9999\"/>\n",car->_corner_x(FRNT_LFT),car->_corner_y(FRNT_LFT),car->_corner_x(FRNT_RGT),car->_corner_y(FRNT_RGT));
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#FF9999\"/>\n",car->_corner_x(FRNT_RGT),car->_corner_y(FRNT_RGT),car->_corner_x(REAR_RGT),car->_corner_y(REAR_RGT));
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#FF9999\"/>\n",car->_corner_x(REAR_RGT),car->_corner_y(REAR_RGT),car->_corner_x(REAR_LFT),car->_corner_y(REAR_LFT));
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#FF9999\"/>\n",car->_corner_x(REAR_LFT),car->_corner_y(REAR_LFT),car->_corner_x(FRNT_LFT),car->_corner_y(FRNT_LFT));
  fflush(stderr);
#endif
   }
#if 0
   else
   {
    collide = ((dnext2 < nextleft2 && mymax >= nextLoffset) || (dnext2 > nextleft2 && mymin <= nextRoffset));
fprintf(stderr,"%s->%s B %d ti=%.3f (%.3f/%.3f-%.3f/%.3f) bd=%.3f\n",mycar->_name,car->_name,collide,t_impact,mycar->_speed_x,dspeed,car->_speed_x,speed,brakedistance);fflush(stderr);
   }
   if (!collide && team == TEAM_CONTROL && sidegap < 5.0 && speed > 10.0 
       && fabs(angle) < 0.7 && speed > dspeed - 5.0 && !driver->getAvoiding() 
       && (brakedistance < 0.5 && t_impact < 0.5))
    collide = 1;
#endif

   if (collide && MAX(t_impact,s_impact) < time_margin && MAX(t_impact,s_impact) >= 0.0 && !(state & OPP_SIDE))
   {
//fprintf(stderr,"collide: t_impact=%.3f dist=%.3f dspd=%.3f\n",t_impact,brakedistance,dspeed-speed);fflush(stderr);
    state |= OPP_COLL;
    /*
    if (t_impact < 0.2 && brakedistance < 1.0)
    {
     state |= OPP_COLL_URGENT;
     if (t_impact <= 0.0 && brakedistance <= 0.0)
      state |= OPP_COLL_REALLYURGENT;
    }
    */
   }
   else
   {
   }
  } 
  else
  {
   //if ((seg->type == TR_LFT && seg->radius <= 200.0 && car->_trkPos.toLeft < mycar->_trkPos.toLeft-0.5) ||
   //    (seg->type == TR_RGT && seg->radius <= 200.0 && car->_trkPos.toLeft > mycar->_trkPos.toLeft+0.5))
   if ((seg->type == TR_LFT && seg->radius <= 60.0 && car->_trkPos.toLeft < mycar->_trkPos.toLeft-(width/2+3.0)) ||
       (seg->type == TR_RGT && seg->radius <= 60.0 && car->_trkPos.toLeft > mycar->_trkPos.toLeft+(width/2+3.0)))
   {
    //SIDECOLLDIST += (210.0 - seg->radius) / 100.0;
    SIDECOLLDIST += (65.0 - seg->radius) / 20.0;
   }

   // Is opponent behind and faster.
   if (distance < -SIDECOLLDIST && speed > dspeed - SPEED_PASS_MARGIN)
   {
    catchdist = dspeed*nextdistance/(nextspeed - dspeed);
    state |= OPP_BACK;
    distance -= MAX(car->_dimension_x, mycar->_dimension_x);
    distance -= LENGTH_MARGIN;
   } 
   else
   {
    // Is opponent aside.
    if (nextdistance > -SIDECOLLDIST &&
     nextdistance < SIDECOLLDIST) 
    {
     sidedist = car->_trkPos.toMiddle - mycar->_trkPos.toMiddle;
     //double nextsidedist = nextleft - dnextleft;
     state |= OPP_SIDE;


#if 0
     double mymin = dnext2 - (dwidth/2 + fabs(dnext2-dnextleft)/4 + 1.0);
     double mymax = dnext2 + dwidth/2 + fabs(dnext2-dnextleft)/4 + 1.0;
     if (mycar->_trkPos.seg->type != TR_STR && mycar->_trkPos.seg->radius <= 120.0)
     {
      mymin -= (130.0-mycar->_trkPos.seg->radius) * 0.02;
      mymax += (130.0-mycar->_trkPos.seg->radius) * 0.02;
     }

     int collide = (closing == CLOSING_FAST && ((dnext2 < nextleft2 && mymax >= nextLoffset) || (dnext2 > nextleft2 && mymin <= nextRoffset)));
     if (collide && distance > 1.0 && dspeed-speed < distance*2)
      state |= (OPP_COLL | OPP_COLL_URGENT);
     else if (0 && (team & TEAM_CONTROL) && seg->radius <= 60.0 && distance <= 1.0)
     {
      if (closing && ((dnextleft < nextleft && seg->type == TR_LFT) || (dnextleft > nextleft && seg->type == TR_RGT)))
       state |= (OPP_COLL | OPP_COLL_URGENT);
     }
#endif
    } 
    else
    {
     // Opponent is in front and faster.
     if (distance > SIDECOLLDIST && nextspeed > dspeed + 3.0f) 
     {
      state |= OPP_FRONT_FAST;
      if ((team & TEAM_CONTROL) && 
         car->_dammage + 500 >= mycar->_dammage)
      {
       state |= OPP_FOLLOW;
      }
     }
    }
   }
  }
 }

 // test for side-on collisions
#if 0
 if ((state & OPP_SIDE) || (brakedistance < 0.5))
 {
  // how far must they travel for the cars to no longer be alongside?
  double passdist = MIN(10000.0, dspeed*(distance + mycar->_dimension_x)/(dspeed - speed));
  if (dspeed < speed)
   passdist = MIN(10000.0, speed*(distance + car->_dimension_x)/(speed - dspeed));

  // and how long will that take?
  double t_pass;
  if (dspeed > speed)
   t_pass = MAX(0.0,(passdist)/(dspeed-speed));
  else
   t_pass = MAX(0.0,(passdist)/(speed-dspeed));
  t_pass = MIN(0.8, t_pass*0.8);

  // compute the cars' positions by then
  for (i=0; i<4; i++)
  {
   ncorner[i].ax = car->_corner_x(i) + (car->_corner_x(i) - corner1[i].ax) * (t_pass / deltatime);
   ncorner[i].ay = car->_corner_y(i) + (car->_corner_y(i) - corner1[i].ay) * (t_pass / deltatime);
   myncorner[i].ax = mycar->_corner_x(i) + (mycar->_corner_x(i) - mycorner[i].ax) * ((t_pass+0.01) / deltatime);
   myncorner[i].ay = mycar->_corner_y(i) + (mycar->_corner_y(i) - mycorner[i].ay) * ((t_pass+0.01) / deltatime);
  }

  if (anglediff > 0.01)
  {
   // make the other car a tad bigger so we're sure to avoid it.
   ncorner[FRNT_RGT].ax += ((ncorner[FRNT_RGT].ax - ncorner[FRNT_LFT].ax) / (car->_dimension_y*0.8)) * anglediff;
   ncorner[FRNT_RGT].ay += ((ncorner[FRNT_RGT].ay - ncorner[FRNT_LFT].ay) / (car->_dimension_y*0.8)) * anglediff;
   ncorner[FRNT_LFT].ax += ((ncorner[FRNT_LFT].ax - ncorner[FRNT_RGT].ax) / (car->_dimension_y*0.8)) * anglediff;
   ncorner[FRNT_LFT].ay += ((ncorner[FRNT_LFT].ay - ncorner[FRNT_RGT].ay) / (car->_dimension_y*0.8)) * anglediff;
   ncorner[REAR_RGT].ax += ((ncorner[REAR_RGT].ax - ncorner[REAR_LFT].ax) / (car->_dimension_y*0.8)) * anglediff;
   ncorner[REAR_RGT].ay += ((ncorner[REAR_RGT].ay - ncorner[REAR_LFT].ay) / (car->_dimension_y*0.8)) * anglediff;
   ncorner[REAR_LFT].ax += ((ncorner[REAR_LFT].ax - ncorner[REAR_RGT].ax) / (car->_dimension_y*0.8)) * anglediff;
   ncorner[REAR_LFT].ay += ((ncorner[REAR_LFT].ay - ncorner[REAR_RGT].ay) / (car->_dimension_y*0.8)) * anglediff;

   ncorner[FRNT_RGT].ax += ((ncorner[FRNT_RGT].ax - ncorner[REAR_RGT].ax) / (car->_dimension_x*0.8)) * anglediff;
   ncorner[FRNT_RGT].ay += ((ncorner[FRNT_RGT].ay - ncorner[REAR_RGT].ay) / (car->_dimension_x*0.8)) * anglediff;
   ncorner[FRNT_LFT].ax += ((ncorner[FRNT_LFT].ax - ncorner[REAR_LFT].ax) / (car->_dimension_x*0.8)) * anglediff;
   ncorner[FRNT_LFT].ay += ((ncorner[FRNT_LFT].ay - ncorner[REAR_LFT].ay) / (car->_dimension_x*0.8)) * anglediff;
   ncorner[REAR_RGT].ax += ((ncorner[REAR_RGT].ax - ncorner[FRNT_RGT].ax) / (car->_dimension_x*0.8)) * anglediff;
   ncorner[REAR_RGT].ay += ((ncorner[REAR_RGT].ay - ncorner[FRNT_RGT].ay) / (car->_dimension_x*0.8)) * anglediff;
   ncorner[REAR_LFT].ax += ((ncorner[REAR_LFT].ax - ncorner[FRNT_LFT].ax) / (car->_dimension_x*0.8)) * anglediff;
   ncorner[REAR_LFT].ay += ((ncorner[REAR_LFT].ay - ncorner[FRNT_LFT].ay) / (car->_dimension_x*0.8)) * anglediff;
  }
#if 0
  for (i=2; i<4; i++)
  {
   ncorner[i].ax = car->_corner_x(i);
   ncorner[i].ay = car->_corner_y(i);
   myncorner[i].ax = mycar->_corner_x(i);
   myncorner[i].ay = mycar->_corner_y(i);
  }
#endif

  // see if a collision will occur
  tPosd collpoint;
  double coll;
  coll = testCollision(mycar->pub.corner, myncorner, ncorner, 0, &collpoint);
  double cdist = 1000.0;
  if (coll >= 0.0)
  {
   if (!(state & OPP_SIDE_FRONT) && (state & OPP_SIDE))
    state |= OPP_SIDE_COLL;

   cdist = (fabs(collpoint.ax - mycar->_corner_x(0)) + fabs(collpoint.ax - mycar->_corner_x(1)) + fabs(collpoint.ay - mycar->_corner_y(0)) + fabs(collpoint.ay - mycar->_corner_y(1))) / 4;

   if ((car->_dimension_x - distance) < 1.5 && 
       (cdist < mycar->_speed_x*0.65 || 
        (sidedist < 5.0 
         && ((mycar->_trkPos.toLeft < 4.0 && car->_trkPos.toLeft > mycar->_trkPos.toLeft)
          || (mycar->_trkPos.toRight < 4.0 && car->_trkPos.toRight < mycar->_trkPos.toRight)))))
    state |= (OPP_COLL | OPP_COLL_URGENT);
  }
#if 0
fprintf(stderr,"%s/%s %d: side sc=%d cu=%d c=%d cdist=%.3f t_pass=%.3f\n",mycar->_name,car->_name,mycar->_dammage,(state&OPP_SIDE_COLL),(state&OPP_COLL_URGENT),(state&OPP_COLL),cdist,t_pass);fflush(stderr);
  // debug stuff in SVG format
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#000000\"/>\n",myncorner[FRNT_LFT].ax,myncorner[FRNT_LFT].ay,myncorner[FRNT_RGT].ax,myncorner[FRNT_RGT].ay);
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#000000\"/>\n",myncorner[FRNT_RGT].ax,myncorner[FRNT_RGT].ay,myncorner[REAR_RGT].ax,myncorner[REAR_RGT].ay);
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#000000\"/>\n",myncorner[REAR_RGT].ax,myncorner[REAR_RGT].ay,myncorner[REAR_LFT].ax,myncorner[REAR_LFT].ay);
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#000000\"/>\n",myncorner[REAR_LFT].ax,myncorner[REAR_LFT].ay,myncorner[FRNT_LFT].ax,myncorner[FRNT_LFT].ay);
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#888888\"/>\n",mycar->_corner_x(FRNT_LFT),mycar->_corner_y(FRNT_LFT),mycar->_corner_x(FRNT_RGT),mycar->_corner_y(FRNT_RGT));
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#888888\"/>\n",mycar->_corner_x(FRNT_RGT),mycar->_corner_y(FRNT_RGT),mycar->_corner_x(REAR_RGT),mycar->_corner_y(REAR_RGT));
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#888888\"/>\n",mycar->_corner_x(REAR_RGT),mycar->_corner_y(REAR_RGT),mycar->_corner_x(REAR_LFT),mycar->_corner_y(REAR_LFT));
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#888888\"/>\n",mycar->_corner_x(REAR_LFT),mycar->_corner_y(REAR_LFT),mycar->_corner_x(FRNT_LFT),mycar->_corner_y(FRNT_LFT));
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#AA0000\"/>\n",ncorner[FRNT_LFT].ax,ncorner[FRNT_LFT].ay,ncorner[FRNT_RGT].ax,ncorner[FRNT_RGT].ay);
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#AA0000\"/>\n",ncorner[FRNT_RGT].ax,ncorner[FRNT_RGT].ay,ncorner[REAR_RGT].ax,ncorner[REAR_RGT].ay);
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#AA0000\"/>\n",ncorner[REAR_RGT].ax,ncorner[REAR_RGT].ay,ncorner[REAR_LFT].ax,ncorner[REAR_LFT].ay);
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#AA0000\"/>\n",ncorner[REAR_LFT].ax,ncorner[REAR_LFT].ay,ncorner[FRNT_LFT].ax,ncorner[FRNT_LFT].ay);
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#FF8888\"/>\n",car->_corner_x(FRNT_LFT),car->_corner_y(FRNT_LFT),car->_corner_x(FRNT_RGT),car->_corner_y(FRNT_RGT));
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#FF8888\"/>\n",car->_corner_x(FRNT_RGT),car->_corner_y(FRNT_RGT),car->_corner_x(REAR_RGT),car->_corner_y(REAR_RGT));
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#FF8888\"/>\n",car->_corner_x(REAR_RGT),car->_corner_y(REAR_RGT),car->_corner_x(REAR_LFT),car->_corner_y(REAR_LFT));
  fprintf(stderr,"  <line fill=\"none\" x1=\"%.3f\" y1=\"%.3f\" x2=\"%.3f\" y2=\"%.3f\" stroke=\"#FF8888\"/>\n",car->_corner_x(REAR_LFT),car->_corner_y(REAR_LFT),car->_corner_x(FRNT_LFT),car->_corner_y(FRNT_LFT));
  fflush(stderr);
#endif

 }
#endif

 // Check if we should let overtake the opponent.
 updateOverlapTimer(s, mycar);
 if (overlaptimer > OVERLAP_WAIT_TIME)
 {
  if (car->_pos < mycar->_pos)
  {
   if (team != TEAM_CONTROL || mycar->_dammage > car->_dammage - 500)
    state |= OPP_LETPASS;
  }
  else if (driver->Alone() && !driver->Chased())
  {
   if (team & TEAM_CONTROL)
   {
    if (mycar->_dammage > car->_dammage + 500)
    {
     state |= OPP_LETPASS;
    }
   }
  }
 }

 prevtime = s->currentTime;

 for (i=0; i<4; i++)
 {
  corner2[i].ax = corner1[i].ax;
  corner2[i].ay = corner1[i].ay;
  corner1[i].ax = car->_corner_x(i);
  corner1[i].ay = car->_corner_y(i);
 }

 prevsidegap = sidegap;
 prevleft = car->_trkPos.toLeft;
}


// Compute the length to the start of the segment.
float Opponent::getDistToSegStart()
{
 if (car->_trkPos.seg->type == TR_STR) {
  return car->_trkPos.toStart;
 } else {
  return car->_trkPos.toStart*car->_trkPos.seg->radius;
 }
}


// Update overlaptimers of opponents.
void Opponent::updateOverlapTimer(tSituation *s, tCarElt *mycar)
{
 if ((state & (OPP_BACK|OPP_SIDE)) && (car->race.laps > mycar->race.laps 
      || ((team & TEAM_CONTROL) && mycar->_dammage > car->_dammage + 500)))
 {
  if ((getState() & (OPP_BACK | OPP_SIDE)) && distance < car->_dimension_x && distance > 30.0) {
   overlaptimer += s->deltaTime;
  } else {
   overlaptimer = 0.0;
  }
 } else if ((getState() & OPP_FRONT) && !(team & TEAM_CONTROL)) {
  overlaptimer = LAP_BACK_TIME_PENALTY;
 } else {
  if (overlaptimer > 0.0) {
   if (getState() & OPP_FRONT_FAST) {
    overlaptimer = MIN(0.0, overlaptimer);
   } else {
    overlaptimer -= s->deltaTime;
   }
  } else {
   overlaptimer += s->deltaTime;
  }
 }
}


// Initialize the list of opponents.
Opponents::Opponents(tSituation *s, Driver *driver, Cardata *c)
{
 opponent = new Opponent[s->_ncars - 1];
 int i, j = 0;
 for (i = 0; i < s->_ncars; i++) {
  if (s->cars[i] != driver->getCarPtr()) {
   opponent[j].setCarPtr(s->cars[i]);
   opponent[j].setCarDataPtr(c->findCar(s->cars[i]));
   j++;
  }
 }
 Opponent::setTrackPtr(driver->getTrackPtr());
 nopponents = s->_ncars - 1;
}


Opponents::~Opponents()
{
 delete [] opponent;
}


void Opponents::update(tSituation *s, Driver *driver)
{
 int i;
 for (i = 0; i < s->_ncars - 1; i++) {
  opponent[i].update(s, driver);
 }
}

