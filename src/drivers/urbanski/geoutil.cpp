#include "geoutil.h"

double getAngle(Point & a, Point & b) {
	return atan2(b.y - a.y, b.x - a.x);
}

Point getCenterOfSegmentEnd(tTrackSeg *seg) {
	return getWeightedPointAtSegmentEnd(seg, 1, 1);
}

double getDistanceToSegmentEnd(tCarElt *car) {
	tTrackSeg *seg = car->_trkPos.seg;
	if (seg->type == TR_STR)
		return seg->length - car->_trkPos.toStart;
	else
		return (seg->arc - car->_trkPos.toStart) * seg->radius;
}

Point getWeightedPointAtSegmentEnd(tTrackSeg *seg, double wLeft, double wRight) {
	double x = (wLeft * seg->vertex[TR_EL].x + wRight * seg->vertex[TR_ER].x) / (wLeft + wRight);
	double y = (wLeft * seg->vertex[TR_EL].y + wRight * seg->vertex[TR_ER].y) / (wLeft + wRight);
	return Point(x, y);
}
