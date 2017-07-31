/***************************************************************************

    file                 : correct.h
    created              : Thu Mai 15 2:41:00 CET 2003
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

#ifndef _CORRECT_H_
#define _CORRECT_H_

#define CORRECT_TIMER 7.0

#include "car.h"

class Correct {
  public:
    Correct(tCarElt *car);
    ~Correct();

    void setCorrecting(int newcorrect);
    void setAllowCorrecting(int newcorrect);
    void setAligned(int aligned) { this->aligned = aligned; }
    bool getCorrecting() { return correcting; }
    bool getAllowCorrecting() { return allow_correcting; }
    bool getOldCorrecting() { return old_correcting; }
    bool getOldAllowCorrecting() { return old_allow_correcting; }
    double getCorrectTimer() { return MAX(0.0, correct_timer - simtime); }
    double getCorrectLimiter() { return correct_limiter; }
    
    void initCorrectValues();
    void update(double cursimtime, int aligned, int segtype);
    double CorrectSteer(double steer, double linesteer, int slowcorrect, double speed, double accel);
    //double CorrectSteer(double steer, double linesteer, double line2left, int slowcorrect, double correctdelay, double speed);

  private:

    tCarElt *car;
    int correcting;
    int old_correcting;
    int allow_correcting;
    int old_allow_correcting;
    int aligned;
    int segtype;

    double correct_timer;
    double correct_limiter;
    double simtime;
};

#endif // _CORRECT_H_


