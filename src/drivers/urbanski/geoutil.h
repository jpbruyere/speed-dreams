#ifndef GEOUTIL_H
#define GEOUTIL_H

#include <cmath>

#include <car.h>
#include <track.h>

struct Point {
	double x, y;
	Point(double x = 0.0, double y = 0.0) : x(x), y(y) {}
	Point(t3Dd p) : x(p.x), y(p.y) {}
};

struct Vec {
	double x, y;
	Vec(Point a, Point b) : x(b.x - a.x), y(b.y - a.y) {}
	double cross(Vec & v) { return x * v.y - y * v.x; }
	double len() { return std::sqrt(x * x + y * y); }
	void norm() { x /= len(); y /= len(); }
};

double getAngle(Point & a, Point & b);
Point getCenterOfSegmentEnd(tTrackSeg *seg);
double getDistanceToSegmentEnd(tCarElt *car);
Point getWeightedPointAtSegmentEnd(tTrackSeg *seg, double wLeft, double wRight);

#endif
