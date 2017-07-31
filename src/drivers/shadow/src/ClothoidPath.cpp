/***************************************************************************

    file        : ClothoidPath.cpp
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

// ClothoidPath.cpp: implementation of the ClothoidPath class.
//
//////////////////////////////////////////////////////////////////////

#include "ClothoidPath.h"
#include "Utils.h"
#include "LinearRegression.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

ClothoidPath::ClothoidPath()
{
	m_factors.Add( 1.005 );
}

ClothoidPath::~ClothoidPath()
{
}

void ClothoidPath::ClearFactors()
{
	m_factors.RemoveAll();
}

void ClothoidPath::AddFactor( double factor )
{
	m_factors.Add( factor );
}

void ClothoidPath::SetFactors( const Array<double>& factors )
{
	m_factors = factors;
}

const Array<double>& ClothoidPath::GetFactors() const
{
	return m_factors;
}

void ClothoidPath::MakeSmoothPath( MyTrack*	pTrack,	const CarModel&	cm,	const Options& opts )
{
//	m_factor = factor;

	LinePath::Initialise( pTrack, opts.maxL, opts.maxR );

	const int	NSEG = pTrack->GetSize();

	CalcCurvaturesZ();
	int	fwdRange = 110;
	CalcFwdAbsK( fwdRange );

	const int delta = 25;
	const int n = (150 + delta - 1) / delta;

	int		step = 1;
	while( step * 4 < NSEG )
		step *= 2;

	do
	{
		step = (step + 1) / 2;
//		int n = 100 * int(sqrt(step));
		for( int i = 0; i < n; i++ )
		{
			OptimisePath( cm, step, delta, 0 );
		}
	}
	while( step > 1 );

	if( opts.bumpMod )
	{
		CalcCurvaturesZ();
		CalcFwdAbsK( fwdRange );
		AnalyseBumps( cm, false );

		step = 8;

		do
		{
			step = (step + 1) / 2;

			for( int i = 0; i < n; i++ )
			{
				OptimisePath( cm, step, delta, opts.bumpMod );
				CalcFwdAbsK( fwdRange );
				CalcMaxSpeeds( cm, step );
				PropagateBreaking( cm, step );
				PropagateAcceleration( cm, step );
			}
		}
		while( step > 1 );
	}

	CalcCurvaturesZ();
}

void ClothoidPath::AnalyseBumps( const CarModel& cm, bool dumpInfo )
{
	// here we look at the bumps on the track, and increase the buffers
	//	from the edge after the bumps.

	// get an estimate of speeds.
	CalcMaxSpeeds( cm );
	PropagateBreaking( cm );
	PropagateAcceleration( cm );

	const int		NSEG = m_pTrack->GetSize();
	const double	g = -9.81;

	double	sz = m_pPath[0].pt.z;
	double	vz = 0;
	double	pz = sz;
    //double	dt = 0.1;
    for( int count = 0; count < 2; count++ )
	{
		int		pi = NSEG - 1;

		for( int i = 0; i < NSEG; i++ )
		{
            //double	oldSz = sz;
			double	oldPz = pz;

			double	v = (m_pPath[i].accSpd + m_pPath[pi].accSpd) * 0.5;
			double	s = Utils::VecLenXY(m_pPath[i].pt - m_pPath[pi].pt);
			double	dt = s / v;

			pz = m_pPath[i].pt.z;
			sz += vz * dt + 0.5 * g * dt * dt;
			vz += g * dt;

			if( sz <= pz )
			{
				double	newVz = (pz - oldPz) / dt;
				if( vz < newVz )
					vz = newVz;
				sz = pz;
			}

			double	h = sz - pz;
			m_pPath[i].h = h;

			if( count == 1 && dumpInfo )
			{
                /*GfOut( "###  %4d  spd %3.0f k %7.4f dt %.3f pz %5.2f sz %5.2f vz %5.2f -> h %5.2f\n",
						i, m_pPath[i].accSpd * 3.6, m_pPath[i].k, dt,
                        pz, sz, vz, m_pPath[i].h );*/
			}

			pi = i;
		}
    }

    for( int count = 0; count < 3; count++ )
	{
        for( int i = 0; i < NSEG; i++ )
		{
			int	j = (i + 1) % NSEG;
			if( m_pPath[i].h < m_pPath[j].h )
				m_pPath[i].h = m_pPath[j].h;
        }
    }
}

void ClothoidPath::SmoothBetween( int step )
{
	const int	NSEG = m_pTrack->GetSize();

	// now smooth the values between steps
	PathPt*	l0 = 0;
	PathPt*	l1 = &m_pPath[((NSEG - 1) / step) * step];
	PathPt*	l2 = &m_pPath[0];
	PathPt*	l3 = &m_pPath[step];

	int		j = 2 * step;

	for( int i = 0; i < NSEG; i += step )
	{
		l0 = l1;
		l1 = l2;	// l1 represents m_pLines[i];
		l2 = l3;
		l3 = &m_pPath[j];

		j += step;
		if( j >= NSEG )
			j = 0;

		Vec3d	p0 = l0->pt;//CalcPt();
		Vec3d	p1 = l1->pt;//CalcPt();
		Vec3d	p2 = l2->pt;//CalcPt();
		Vec3d	p3 = l3->pt;//CalcPt();

		double	k1 = Utils::CalcCurvatureXY(p0, p1, p2);
		double	k2 = Utils::CalcCurvatureXY(p1, p2, p3);

		if( i + step > NSEG )
			step = NSEG - i;

		for( int k = 1; k < step; k++ )
		{
			double	t;
			PathPt&	l = m_pPath[(i + k) % NSEG];
			Utils::LineCrossesLineXY( l.Pt(), l.Norm(), p1, p2 - p1, t );
			l.offs = t;

			double	len1 = (l.CalcPt() - p1).len();
			double	len2 = (l.CalcPt() - p2).len();
			double	kappa = (k1 * len2 + k2 * len1) / (len1 + len2);

			if( kappa != 0 )
			{
				double	delta = 0.0001;
				double	deltaK = Utils::CalcCurvatureXY(
										p1, l.Pt() + l.Norm() * (t + delta), p2);
				t += delta * kappa / deltaK;
			}

			const double buf = 1.0;//1.25;
			if( t < -l.Wl() + l.lBuf + buf )
				t = -l.Wl() + l.lBuf + buf;
			else if( t > l.Wr() - l.rBuf - buf )
				t = l.Wr() - l.rBuf - buf;

			if( t < -m_maxL + l.lBuf + buf )
				t = -m_maxL + l.lBuf + buf;
			else if( t > m_maxR - l.rBuf - buf )
				t = m_maxR - l.rBuf - buf;

			l.offs = t;
			l.pt = l.CalcPt();
		}
	}
}

static double Map( double value, double fromLo, double fromHi, double toLo, double toHi )
{
	double	newValue = (value - fromLo) / (fromHi - fromLo) * (toHi - toLo) + toLo;

	if( newValue < toLo )
		newValue = toLo;
	else if( newValue > toHi )
		newValue = toHi;

	return newValue;
}

void	ClothoidPath::SetOffset( const CarModel& cm, double k, double t, PathPt* l3, const PathPt* l2, const PathPt* l4 )
{
	double	marg = cm.WIDTH / 2 + 0.02;//1.0;//1.1
	double	wl  = -MN(m_maxL, l3->Wl()) + marg;
	double	wr  =  MN(m_maxR, l3->Wr()) - marg;
	double	buf = MN(1.5, 100 * fabs(k));	// a = v*v/r;

	if( k >= 0 )// 0.00001 )
	{
		if( t < wl )
			t = wl;
		else if( t > wr - l3->rBuf - buf )
		{
			if( l3->offs > wr - l3->rBuf - buf )
				t = MN(t, l3->offs);
			else
				t = wr - l3->rBuf - buf;
			t = MN(t, wr);
		}
	}
	else //if( k < -0.00001 )
	{
		if( t > wr )
			t = wr;
		else if( t < wl + l3->lBuf + buf )
		{
			if( l3->offs < wl + l3->lBuf + buf )
				t = MX(t, l3->offs);
			else
				t = wl + l3->lBuf + buf;
			t = MX(t, wl);
		}
	}

	l3->offs = t;
	l3->pt = l3->CalcPt();
	l3->k = Utils::CalcCurvatureXY(l2->pt, l3->pt, l4->pt);
}

void ClothoidPath::OptimiseLine( const CarModel& cm, int idx, int step,	double hLimit, PathPt* l3, const PathPt* l2, const PathPt* l4 )
{
	LinearRegression	l;

	const int NSEG = m_pTrack->GetSize();

	int i = (idx + NSEG - step) % NSEG;
	while( m_pPath[i].h > hLimit )
	{
		l.Sample( m_pPath[i].pt.GetXY() );
		i = (i + NSEG - step) % NSEG;
	}

	l.Sample( m_pPath[i].pt.GetXY() );

	i = idx;
	while( m_pPath[i].h > hLimit )
	{
		l.Sample( m_pPath[i].pt.GetXY() );
		i = (i + step) % NSEG;
	}

	l.Sample( m_pPath[i].pt.GetXY() );

    GfOut( "%4d  ", idx );
	Vec2d	p, v;
	l.CalcLine( p, v );

	double	t;
	Utils::LineCrossesLine( l3->Pt().GetXY(), l3->Norm().GetXY(), p, v, t );

	SetOffset( cm, 0, t, l3, l2, l4 );
}

void	ClothoidPath::Optimise(	const CarModel&	cm,	double factor, int idx, PathPt* l3,	const PathPt* l0,
                                const PathPt*	l1,	const PathPt* l2, const PathPt*	l4,	const PathPt* l5,
                                const PathPt*	l6,	int	bumpMod )
{
	if( factor == 0 )
	{
		if( fabs(l3->fwdK) < 1 / 100 )
			factor = 1.05;
		else
//			factor = 1.003 + ((1.0 / fabs(l3->k)) - 35) * 0.00035;
//			factor = 1.003 + ((1.0 / fabs(l3->k)) - 25) * 0.0006;
//			factor = Map(fabs(l3->k), 1.0/25, 1.0/200, 1.003, 1.03);
//			factor = Map(1.0 / fabs(l3->k), 34, 100, 1.003, 1.03);// alpine 1:55.87
//			factor = Map(1.0 / fabs(l3->k), 40, 100, 1.004, 1.03);// alpine 1:55.87
//			factor = Map(1.0 / fabs(l3->k), 20, 80, 1.003, 1.03);// generally good
			factor = Map(1.0 / fabs(l3->fwdK), 20, 100, 1.003, 1.05);
//			factor = Map(fabs(l3->k), 1.0/30, 1.0/100, 1.003, 1.03);
	}

	Vec3d	p0 = l0->pt;//CalcPt();
	Vec3d	p1 = l1->pt;//CalcPt();
	Vec3d	p2 = l2->pt;//CalcPt();
	Vec3d	p3 = l3->pt;//CalcPt();
	Vec3d	p4 = l4->pt;//CalcPt();
	Vec3d	p5 = l5->pt;//CalcPt();
	Vec3d	p6 = l6->pt;//CalcPt();

	double	k1 = Utils::CalcCurvatureXY(p1, p2, p3);
	double	k2 = Utils::CalcCurvatureXY(p3, p4, p5);

	double	length1 = hypot(p3.x - p2.x, p3.y - p2.y);
	double	length2 = hypot(p4.x - p3.x, p4.y - p3.y);

    if( k1 * k2 > 0 )
	{
		double	k0 = Utils::CalcCurvatureXY(p0, p1, p2);
		double	k3 = Utils::CalcCurvatureXY(p4, p5, p6);

		if( k0 * k1 > 0 && k2 * k3 > 0 )
		{
			if( fabs(k0) < fabs(k1) && fabs(k1) * 1.02 < fabs(k2) )
			{
				k1 *= factor;
				k0 *= factor;
			}
			else if( fabs(k0) > fabs(k1) * 1.02 && fabs(k1) > fabs(k2) )
			{
				k1 *= factor;
				k0 *= factor;
			}
		}
	}
	else if( k1 * k2 < 0 )
	{
		double	k0 = Utils::CalcCurvatureXY(p0, p1, p2);
		double	k3 = Utils::CalcCurvatureXY(p4, p5, p6);

		if( k0 * k1 > 0 && k2 * k3 > 0 )
		{
			if( fabs(k1) < fabs(k2) && fabs(k1) < fabs(k3) )
			{
				k1 = (k1 * 0.25 + k2 * 0.75);
				k0 = (k0 * 0.25 + k3 * 0.75);
			}
			else if( fabs(k2) < fabs(k1) && fabs(k2) < fabs(k0) )
			{
				k2 = (k2 * 0.25 + k1 * 0.75);
				k3 = (k3 * 0.25 + k0 * 0.75);
			}
		}
	}

	double	k = (length2 * k1 + length1 * k2) / (length1 + length2);

//	double	maxSpdK = cm.CalcMaxSpdK();
	double	maxSpdK = 0.00175;//60 / (50 * 50);	// a = vv/r; r = vv/a; k = a/vv;

	if( k1 * k2 >= 0 && fabs(k1) < maxSpdK && fabs(k2) < maxSpdK )
	{
//		k = 0;//(fabs(k1) < fabs(k2) ? k1 : k2);
		k *= 0.9;
	}

	double	t = l3->offs;
    //double	oldT = t;
	Utils::LineCrossesLineXY( l3->Pt(), l3->Norm(), p2, p4 - p2, t );
//	if( l3->h < 0.1 )
	{
		double	delta = 0.0001;
		double	deltaK = Utils::CalcCurvatureXY(p2, l3->Pt() + l3->Norm() * (t + delta), p4);

		if( bumpMod == 1 )
		{
			double	f = l3->h <= 0.07 ? 1.00 :
						l3->h <= 0.10 ? 0.97 :
						l3->h <= 0.20 ? 0.90 :
						l3->h <= 0.30 ? 0.80 : 0.70;
			delta *= f;
		}

		t += delta * k / deltaK;
	}

	SetOffset( cm, k, t, l3, l2, l4 );
}

void	ClothoidPath::OptimisePath(	const CarModel&	cm,	int	step, int nIterations, int bumpMod )
{
	const int	NSEG = m_pTrack->GetSize();

	for( int j = 0; j < nIterations; j++ )
	{
		PathPt*	l0 = 0;
		PathPt*	l1 = &m_pPath[NSEG - 3 * step];
		PathPt*	l2 = &m_pPath[NSEG - 2 * step];
		PathPt*	l3 = &m_pPath[NSEG - step];
		PathPt*	l4 = &m_pPath[0];
		PathPt*	l5 = &m_pPath[step];
		PathPt*	l6 = &m_pPath[2 * step];

		// go forwards
		int		i = 3 * step;
		int		n = (NSEG + step - 1) / step;

		for( int count = 0; count < n; count++ )
		{
			l0 = l1;
			l1 = l2;
			l2 = l3;
			l3 = l4;
			l4 = l5;
			l5 = l6;
			l6 = &m_pPath[i];

			int		idx = (i + NSEG - 3 * step) % NSEG;
			int		fIndex = m_factors.GetSize() * idx / NSEG;
			double	factor = m_factors[fIndex];


			if( bumpMod == 2 && l3->h > 0.1 )
				OptimiseLine( cm, idx, step, 0.1, l3, l2, l4 );
			else
				Optimise( cm, factor, idx, l3, l0, l1, l2, l4, l5, l6, bumpMod );

			if( (i += step) >= NSEG )
				i = 0;//i -= m_nSegs;
		}
	}

	// now smooth the values between steps
	if( step > 1 )
		SmoothBetween( step );
}
