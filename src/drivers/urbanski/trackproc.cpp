#include "trackproc.h"

void TrackProc::init(tTrack *track) {
	distToNextCorner = new double[track->nseg];
	distToPrevCorner = new double[track->nseg];
	nextCornerType = new int[track->nseg];
	prevCornerType = new int[track->nseg];
	setCornerInfo(track);
}

double TrackProc::getDistToNextCorner(tTrackSeg *seg) {
	return distToNextCorner[seg->id];
}

double TrackProc::getDistToPrevCorner(tTrackSeg *seg) {
	return distToPrevCorner[seg->id];
}

int TrackProc::getNextCornerType(tTrackSeg *seg) {
	return nextCornerType[seg->id];
}

int TrackProc::getPrevCornerType(tTrackSeg *seg) {
	return prevCornerType[seg->id];
}

void TrackProc::shutDown() {
	delete[] distToNextCorner;
	delete[] distToPrevCorner;
	delete[] nextCornerType;
	delete[] prevCornerType;
}

tTrackSeg* TrackProc::nextSeg(tTrackSeg *seg, bool forwards) {
	return forwards ? seg->next : seg->prev;
}

void TrackProc::setCornerInfo(tTrack *track) {
	tTrackSeg *startSeg = track->seg;
	while (startSeg->type == TR_STR)
		startSeg = startSeg->next;
	traverseTrack(startSeg, false, distToNextCorner, nextCornerType);
	traverseTrack(startSeg, true, distToPrevCorner, prevCornerType);
}

void TrackProc::traverseTrack(tTrackSeg *startSeg, bool forwards, double *distToCorner, int *cornerType) {
	double dist = 0.0;
	int type = startSeg->type;
	for (tTrackSeg *seg = nextSeg(startSeg, forwards); seg != startSeg; seg = nextSeg(seg, forwards)) {
		if (seg->type != TR_STR) {
			type = seg->type;
			dist = 0.0;
		}
		distToCorner[seg->id] = dist;
		cornerType[seg->id] = type;
		dist += seg->length;
	}
}
