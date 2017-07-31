#ifndef TRACKPROC_H
#define TRACKPROC_H

#include <track.h>

class TrackProc {
public:
	void init(tTrack *track);
	double getDistToNextCorner(tTrackSeg *seg);
	double getDistToPrevCorner(tTrackSeg *seg);
	int getNextCornerType(tTrackSeg *seg);
	int getPrevCornerType(tTrackSeg *seg);
	void shutDown();

private:
	tTrackSeg* nextSeg(tTrackSeg *seg, bool forwards);
	void setCornerInfo(tTrack *track);
	void traverseTrack(tTrackSeg *startSeg, bool forwards, double *cornerDist, int *cornerType);

	double *distToNextCorner;
	int *nextCornerType;
	double *distToPrevCorner;
	int *prevCornerType;
};

#endif
