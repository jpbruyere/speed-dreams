/***************************************************************************

    file        : PitPath.cpp
    created     : 9 Apr 2006
    copyright   : (C) 2006 Tim Foden

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

// PitPath.cpp: implementation of the PitPath class.
//
//////////////////////////////////////////////////////////////////////

#include "PitPath.h"
#include "Utils.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

PitPath::PitPath()
{
}

PitPath::~PitPath()
{
}

void PitPath::MakePath( CarElt*	pCar, LinePath*	pBasePath, const CarModel& cm, double entryOffset, double exitOffset )
{
	LinePath::Set( *pBasePath );

	const tTrackOwnPit*		pPit = pCar->_pit;
	const tTrackPitInfo*	pPitInfo = &m_pTrack->GetTrack()->pits;

	if(	pPit != NULL )
	{
		const int	NPOINTS = 7;
		double	x[NPOINTS];
		double	y[NPOINTS];
		double	s[NPOINTS];

		// Compute pit spline points along the track.
		x[3] = pPit->pos.seg->lgfromstart + pPit->pos.toStart;
		x[2] = x[3] - pPitInfo->len;
		x[4] = x[3] + pPitInfo->len;
		x[0] = pPitInfo->pitEntry->lgfromstart + entryOffset;
		x[1] = pPitInfo->pitStart->lgfromstart;
		x[5] = x[3] + (pPitInfo->nMaxPits - pCar->index) * pPitInfo->len;
		x[6] = pPitInfo->pitExit->lgfromstart + pPitInfo->pitExit->length + exitOffset;

		m_pitEntryPos = x[0];
		m_pitStartPos = x[1];
		m_pitEndPos   = x[5];
		m_pitExitPos  = x[6];

		// Normalizing spline segments to >= 0.0.
        for( int i = 0; i < NPOINTS; i++ )
		{
			x[i] = ToSplinePos(x[i]);
			s[i] = 0.0;
        }

		// Fix broken pit exit.
		if( x[6] < x[5] )
			x[6] = x[5] + 50.0;

		// Fix point for first pit if necessary.
		if( x[1] > x[2] )
			x[1] = x[2];

		// Fix point for last pit if necessary.
		if( x[5] < x[4] )
			x[5] = x[4];


		// splice entry/exit of pit path into the base path provided.
		PtInfo	pi;
		pBasePath->GetPtInfo(m_pitEntryPos, pi);
		y[0] = pi.offs;
		s[0] = -tan(pi.oang - m_pTrack->CalcForwardAngle(m_pitEntryPos));

		pBasePath->GetPtInfo(m_pitExitPos, pi);
		y[6] = pi.offs;
		s[6] = -tan(pi.oang - m_pTrack->CalcForwardAngle(m_pitExitPos));

		float sign = (pPitInfo->side == TR_LFT) ? -1.0f : 1.0f;
		{for( int i = 1; i < NPOINTS - 1; i++ )
		{
			y[i] = fabs(pPitInfo->driversPits->pos.toMiddle) - pPitInfo->width;
			y[i] *= sign;
		}}

		y[3] = (fabs(pPitInfo->driversPits->pos.toMiddle) + 0.5) * sign;
		
		CubicSpline	spline(NPOINTS, x, y, s);

		// modify points in line path for pits...
		int		NSEG = m_pTrack->GetSize();
		int		idx0 = (m_pTrack->IndexFromPos(m_pitEntryPos) + 1) % NSEG;
		int		idx1 = m_pTrack->IndexFromPos(m_pitExitPos);
        for( int i = idx0; i != idx1; i = (i + 1) % NSEG )
		{
			double	x = ToSplinePos(m_pTrack->GetAt(i).segDist);
			double	y = spline.CalcY(x);

			m_pPath[i].offs = y;
			m_pPath[i].pt = m_pPath[i].CalcPt();
//			if( m_pPath[i].
        }

		CalcCurvaturesXY();
		CalcMaxSpeeds( cm );

		idx0 = (m_pTrack->IndexFromPos(m_pitStartPos) + NSEG - 8) % NSEG;
		idx1 = (m_pTrack->IndexFromPos(m_pitEndPos) + 1) % NSEG;
		double	spd = MN(m_pPath[idx0].spd, pPitInfo->speedLimit - 2);
		m_pPath[idx0].maxSpd = m_pPath[idx0].spd = spd;
		{for( int i = idx0; i != idx1; i = (i + 1) % NSEG )
		{
			spd = MN(m_pPath[i].spd, pPitInfo->speedLimit - 0.5);
			m_pPath[i].maxSpd = m_pPath[i].spd = spd;
		}}

		double	stopPos = pPit->pos.seg->lgfromstart + pPit->pos.toStart;
		idx0 = m_pTrack->IndexFromPos(stopPos);
		idx1 = (idx0 + 1) % NSEG;		
		m_pPath[idx0].maxSpd = m_pPath[idx0].spd = 1;
		m_pPath[idx1].maxSpd = m_pPath[idx1].spd = 1;

		m_stopIdx = idx0;

		PropagateBreaking( cm );

		idx0 = (m_pTrack->IndexFromPos(m_pitEntryPos) + 1) % NSEG;
		while( m_pPath[idx0].spd < pBasePath->GetAt(idx0).spd )
			idx0 = (idx0 + NSEG - 1) % NSEG;
		m_pitEntryPos = m_pPath[idx0].Dist();
	}
}

bool PitPath::InPitSection( double trackPos ) const
{
	trackPos = ToSplinePos(trackPos);
	double	pitExitPos = ToSplinePos(m_pitExitPos);
	return m_pitEntryPos <= trackPos && trackPos <= pitExitPos;
}

bool PitPath::CanStop( double trackPos ) const
{
	return m_pTrack->IndexFromPos(trackPos) == m_stopIdx;
}

double PitPath::ToSplinePos( double trackPos ) const
{
	if( trackPos < m_pitEntryPos )
		trackPos += m_pTrack->GetLength();
	return trackPos;
}
