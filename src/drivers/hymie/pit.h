/***************************************************************************

    file                 : pit.h
    created              : Thu Mai 15 2:41:00 CET 2003
    copyright            : (C) 2003-2004 by Bernhard Wymann
    email                : berniw@bluewin.ch
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

#ifndef _PIT_H_
#define _PIT_H_

#include "driver.h"
#include "spline.h"

class Driver;

class Pit {
	public:
		Pit(tSituation *s, Driver *driver);
		~Pit();

		void setPitstop(bool pitstop);
		bool getPitstop() { return pitstop; }

		void setInPit(bool inpitlane) { this->inpitlane = inpitlane; }
		bool getInPit() { return inpitlane; }

		double getPitOffset(double offset, double fromstart, double nextleft, int which);

		bool isBetween(float fromstart);
		bool isTimeout(float distance);
		bool isApproaching();
		bool isApproaching(float fromstart);
		bool isExiting();

		float getNPitEntry() { return p0[0].x; }
		float getNPitStart() { return p0[1].x; }
		float getNPitLoc(int which) { return (which==1 ? p1[3].x : which==2 ? p2[3].x : p0[3].x); }
		float getNPitEnd() { return p0[5].x; }

		float toSplineCoord(float x);

		float getSpeedlimitSqr() { return speedlimitsqr; }
		float getSpeedlimit() { return speedlimit; }
		float getSpeedLimitBrake(float speedsqr);
		bool getLastInPit() { return lastinpit; }

		void update();

	private:
		tTrack *track;
		tCarElt *car;
		tTrackOwnPit *mypit;			// Pointer to my pit.
		tTrackPitInfo *pitinfo;			// General pit info.

		enum { NPOINTS = 7 };
		SplinePoint p0[NPOINTS];			// Spline points.
		SplinePoint p1[NPOINTS];			// Spline points.
		SplinePoint p2[NPOINTS];			// Spline points.
		Spline *spline0;					// Spline.
		Spline *spline1;					// Spline.
		Spline *spline2;					// Spline.

		bool pitstop;					// Pitstop planned.
		bool inpitlane;					// We are still in the pit lane.
		float pitapproach;
		float pitentry;					// Distance to start line of the pit entry.
		float pitend;
		float pitexit;					// Distance to the start line of the pit exit.
		float pithome;

		float speedlimitsqr;			// Pit speed limit squared.
		float speedlimit;				// Pit speed limit.
		float pitspeedlimitsqr;			// The original speedlimit squared.

		float pittimer;					// Timer for pit timeouts.
		Driver *pdriver;
		bool lastinpit;

		static const float SPEED_LIMIT_MARGIN;
};

#endif // _PIT_H_


