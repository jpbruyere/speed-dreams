#include "driver.h"

#include <cmath>

const double Driver::ACCEL_SPEED_MARGIN = 0.2;
const double Driver::BRAKE_SPEED_MARGIN = 1.0;
const double Driver::CUTTING_CORNER_MARGIN = 0.023;
const double Driver::ENGINE_BRAKE_FACTOR = 0.5;
const double Driver::EXCESSIVE_STEER_FACTOR = 0.85;
const double Driver::FIRST_GEAR_SPEED = 5.0;
const double Driver::GRAVITY = 9.81;
const double Driver::INF_SPEED = 1000.0;
const double Driver::SHIFT_RPM_MARGIN = 5.0;
const double Driver::SKILL_LEVEL = 3;
const double Driver::SLIP_SPEED_MARGIN = 2.0;
const double Driver::SMOOTH_STEER_FACTOR = 0.65;
const double Driver::SPEED_LIMIT_MULTIPLIER = 1.4;
const double Driver::STEER_LOOK_AHEAD_DIST = 15.0;

void Driver::drive(tSituation *s) {
	updateState();
	memset((void*) &car->ctrl, 0, sizeof(tCarCtrl));
	car->_steerCmd = getSteerCmd(s);
	car->_gearCmd = getGearCmd(s);
	car->_accelCmd = getAccelCmd(s);
	car->_brakeCmd = getBrakeCmd(s);
}

void Driver::endRace() {
	trackProc.shutDown();
}

void Driver::setCar(tCarElt *car) {
	this->car = car;
	this->car->_skillLevel = SKILL_LEVEL;
}

void Driver::setTrack(tTrack *track, void **carParmHandle) {
	trackProc.init(track);
	*carParmHandle = NULL;
}

double Driver::getAccelCmd(tSituation *s) {
	if (state == BRAKE || state == NO_THROTTLE)
		return 0.0;
	if (state == ACCELERATE)
		return 1.0;
	double r = car->_wheelRadius(REAR_RGT);
	double ratio = car->_gearRatio[car->_gear + car->_gearOffset];
	double maxRevs = car->_enginerpmRedLine;
	double accel = curSpeedLimit / r * ratio / maxRevs;
	if (car->_speed_x - ACCEL_SPEED_MARGIN > curSpeedLimit)
		accel *= ENGINE_BRAKE_FACTOR;
	return accel;
}

double Driver::getBrakeCmd(tSituation *s) {
	if (state == BRAKE)
		return 1.0;
	return 0.0;
}

double Driver::getBrakingDistance(double vH, double vL, double friction) {
	return (vH * vH - vL * vL) / (GRAVITY * friction);
}

int Driver::getGearCmd(tSituation *s) {
	if (car->_gear <= 0 || car->_speed_x <= FIRST_GEAR_SPEED)
		return 1;
	if (shouldShiftUp())
		return car->_gear + 1;
	if (shouldShiftDown())
		return car->_gear - 1;
	return car->_gear;
}

double Driver::getSpeedLimit(tTrackSeg *seg) {
	if (seg->type == TR_STR)
		return INF_SPEED;
	double friction = seg->surface->kFriction;
	return SPEED_LIMIT_MULTIPLIER * std::sqrt(GRAVITY * friction * seg->radius);
}

double Driver::getSteerCmd(tSituation *s) {
	Point target = getTargetPoint();
	double angle = getAngle(carPoint, target);
	angle -= car->_yaw;
	NORM_PI_PI(angle);
	double steerAngle = SMOOTH_STEER_FACTOR * angle / car->_steerLock;
	if (std::abs(steerAngle) > EXCESSIVE_STEER_FACTOR && car->_speed_x > FIRST_GEAR_SPEED)
		state = NO_THROTTLE;
	return steerAngle;
}

Point Driver::getTargetPoint() {
	Point target = getCenterOfSegmentEnd(car->_trkPos.seg->next);
	double totalDistance = getDistanceToSegmentEnd(car);
	for (tTrackSeg *seg = car->_trkPos.seg->next; totalDistance < STEER_LOOK_AHEAD_DIST; seg = seg->next) {
		Point p = getTargetPointForSegment(seg);
		if (isVisibleFromCurrentLocation(p, seg))
			target = p;
		totalDistance += seg->length;
	}
	return target;
}

Point Driver::getTargetPointForSegment(tTrackSeg *seg) {
	return getCenterOfSegmentEnd(seg);
}

bool Driver::isSlipping() {
	for (int i = 0; i < 4; i++)
		if (car->_wheelSpinVel(i) * car->_wheelRadius(i) > car->_speed_x + SLIP_SPEED_MARGIN)
			return true;
	return false;
}

bool Driver::isVisibleFromCurrentLocation(Point & p, tTrackSeg *targetSeg) {
	Vec v(carPoint, p);
	v.norm();
	for (tTrackSeg *seg = car->_trkPos.seg->next; seg != targetSeg; seg = seg->next) {
		Vec sl(carPoint, Point(seg->vertex[TR_SL])), el(carPoint, Point(seg->vertex[TR_EL]));
		Vec sr(carPoint, Point(seg->vertex[TR_SR])), er(carPoint, Point(seg->vertex[TR_ER]));
		sl.norm(); el.norm();
		sr.norm(); er.norm();
		if (sl.cross(v) > -CUTTING_CORNER_MARGIN ||
			el.cross(v) > -CUTTING_CORNER_MARGIN ||
			sr.cross(v) <  CUTTING_CORNER_MARGIN ||
			er.cross(v) <  CUTTING_CORNER_MARGIN)
			return false;
	}
	return true;
}

bool Driver::shouldBrake() {
	double speed = car->_speed_x;
	if (speed - BRAKE_SPEED_MARGIN > curSpeedLimit)
		return true;
	double totalDistance = getDistanceToSegmentEnd(car);
	double friction = car->_trkPos.seg->surface->kFriction;
	double distanceToStop = getBrakingDistance(speed, 0.0, friction);
	for (tTrackSeg *seg = car->_trkPos.seg->next; totalDistance < distanceToStop; seg = seg->next) {
		double speedLimit = getSpeedLimit(seg);
		double brakingDistance = getBrakingDistance(speed, speedLimit, seg->surface->kFriction);
		if (speed - BRAKE_SPEED_MARGIN > speedLimit && totalDistance <= brakingDistance)
			return true;
		totalDistance += seg->length;
	}
	return false;
}

bool Driver::shouldMaintain() {
	return car->_speed_x + ACCEL_SPEED_MARGIN >= curSpeedLimit;
}

bool Driver::shouldShiftDown() {
	int gear = car->_gear;
	int off = car->_gearOffset;
	double d = car->_gearRatio[gear+off-1] / car->_gearRatio[gear+off];
	bool isSafe = d * car->_enginerpm < car->_enginerpmMaxPw;
	return gear > 2 && car->_enginerpm < car->_enginerpmMaxTq && isSafe;
}

bool Driver::shouldShiftUp() {
	return car->_gear < car->_gearNb - 1 && car->_enginerpm >= car->_enginerpmRedLine - SHIFT_RPM_MARGIN;
}

void Driver::updateState() {
	curSpeedLimit = getSpeedLimit(car->_trkPos.seg);
	carPoint = Point(car->_pos_X, car->_pos_Y);
	if (shouldBrake())
		state = BRAKE;
	else if (isSlipping() && car->_gear <= 1)
		state = NO_THROTTLE;
	else if (shouldMaintain())
		state = MAINTAIN;
	else
		state = ACCELERATE;
}
