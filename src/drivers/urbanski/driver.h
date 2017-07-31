#ifndef DRIVER_H
#define DRIVER_H

#include "geoutil.h"
#include "trackproc.h"

#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <tgf.h>
#include <track.h>

class Driver {
public:
	void drive(tSituation *s);
	void endRace();
	void setCar(tCarElt *car);
	void setTrack(tTrack *track, void **carParmHandle);

private:
	enum State { BRAKE, NO_THROTTLE, MAINTAIN, ACCELERATE };

	double getAccelCmd(tSituation *s);
	double getBrakeCmd(tSituation *s);
	double getBrakingDistance(double vH, double vL, double friction);
	int getGearCmd(tSituation *s);
	double getSpeedLimit(tTrackSeg *seg);
	double getSteerCmd(tSituation *s);
	Point getTargetPoint();
	Point getTargetPointForSegment(tTrackSeg *seg);
	bool isSlipping();
	bool isVisibleFromCurrentLocation(Point & p, tTrackSeg *targetSeg);
	bool shouldBrake();
	bool shouldMaintain();
	bool shouldShiftDown();
	bool shouldShiftUp();
	void updateState();

	tCarElt *car;
	State state;
	double curSpeedLimit;
	Point carPoint;
	TrackProc trackProc; //for future use

	static const double ACCEL_SPEED_MARGIN;
	static const double BRAKE_SPEED_MARGIN;
	static const double CUTTING_CORNER_MARGIN;
	static const double ENGINE_BRAKE_FACTOR;
	static const double EXCESSIVE_STEER_FACTOR;
	static const double FIRST_GEAR_SPEED;
	static const double GRAVITY;
	static const double INF_SPEED;
	static const double SHIFT_RPM_MARGIN;
	static const double SKILL_LEVEL;
	static const double SLIP_SPEED_MARGIN;
	static const double SMOOTH_STEER_FACTOR;
	static const double SPEED_LIMIT_MULTIPLIER;
	static const double STEER_LOOK_AHEAD_DIST;
};

#endif
