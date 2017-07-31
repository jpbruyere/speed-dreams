/***************************************************************************

    file        : CarModel.cpp
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

#include <math.h>

#include "CarModel.h"
#include "Quadratic.h"
#include "Utils.h"
//#include "Driver.h"

// The "SHADOW" logger instance.
extern GfLogger* PLogSHADOW;
#define LogSHADOW (*PLogSHADOW)

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CarModel::CarModel()
:	AERO(0),
    EMPTYMASS(0),
    MASS(0),
    LENGTH(0),
    FUEL(0),
    DAMAGE(0),
    NEEDSINLONG(false),
    USEDACCEXIT(false),
    SKILL(0),

    TYRE_MU(0),
    TYRE_MU_F(0),
    TYRE_MU_R(0),
    MU_SCALE(0),
    MIN_MU_SCALE(0),

    BRAKESCALE(0),
    BRAKEFORCE(0),
    BRAKELIMIT(0.0),

    CA(0),
    CA_FW(0),
    CA_RW(0),
    CA_GE(0),
    CA_GE_F(0),
    CA_GE_R(0),

    CD_BODY(0),
    CD_WING(0),
    CD_CX(0),

    KZ_SCALE(0),
    BUMP_FACTOR(0),
    BUMP_FACTORLEFT(0),
    BUMP_FACTORRIGHT(0),

    WIDTH(2),
    HASTYC(false),
    TYRECONDITIONFRONT(0),
    TYRECONDITIONREAR(0)
{
}

CarModel::~CarModel()
{
}

double	CarModel::CalcMaxSpeed(double k, double k1, double kz, double kFriction, double RollAngle, double TiltAngle ) const
{
    //
    //	Here we calculate the theoretical maximum speed at a point on the
    //	path.  This takes into account the curvature of the path (k), the
    //	grip on the road (mu), the downforce from the wings and the ground
    //	effect (CA), the tilt of the road (left to right slope) (sn)
    //	and the curvature of the road in z (kz).
    //
    //	There are still a few silly fudge factors to make the theory match
    //	with the reality (the car goes too slowly otherwise, aarrgh!).
    //
    double Mu;

    double Cos = cos(RollAngle)*cos(TiltAngle);
    double SinLat = sin(RollAngle);
    double SinLong = sin(TiltAngle);
    double Sin = SinLat;

    if (NEEDSINLONG)
    {
      if (SinLat < SinLong)
            Sin = SinLong;
    }

    double AbsCrv0 = MAX(0.001, fabs(k));
    double AbsCrv1 = MAX(0.001, fabs(k1));
    double AbsCrv = AbsCrv0;
    double factor = 1.0;

    if (AbsCrv < 1/200.0)
      kz *= KZ_SCALE;

    if (AbsCrv > AbsCrv1)
    {
      if (USEDACCEXIT)
        factor = 1.015;

      //AbsCrv *= oDriver->CalcCrv(AbsCrv);
    }
    else
    {
      factor = 0.985;
      //AbsCrv *= oDriver->CalcCrv(AbsCrv);
    }

    //Friction *= oDriver->CalcFriction(AbsCrv);

    double Den;

    double ScaleBump  = BUMP_FACTOR;
    if (k > 0)
      ScaleBump = BUMP_FACTORLEFT;
    else
      ScaleBump = BUMP_FACTORRIGHT;

    double MuF = kFriction * TYRE_MU_F; /* MU_SCALE;*/
    double MuR = kFriction * TYRE_MU_R; /* MU_SCALE;*/

    if (HASTYC)
    {
      double TcF = TYRECONDITIONFRONT;
      double TcR = TYRECONDITIONREAR;
      MuF = TcF * MuF;
      MuR = TcR * MuR;
      Mu = MIN(MuF, MuR); // SKILL;
      LogSHADOW.debug("TYRE MUF = %.f - TYRE MUR = %.f - MU = %.f\n", MuF, MuR, Mu);
    }
    else
    {
        Mu = MIN(MuF, MuR); // oTmpCarParam->oSkill;
        LogSHADOW.debug("MU = %.f\n", Mu);
    }

    Den = (AbsCrv - ScaleBump * kz) - (CA_FW * MuF + CA_RW * MuR
      + CA_GE_F * MuF + CA_GE_R * MuR) / MASS;

    if (Den < 0.00001)
     Den = 0.00001;

    if (AbsCrv > 0.002)
    {
        if (Sin * SGN(k) < 0)
        {
            Sin *= 8.0;
            Sin = SGN(Sin) * MIN(0.05,fabs(Sin));
        }
    }

    double Speed = factor * sqrt((Cos * G * Mu + Sin * G * SGN(k) + kz) / Den);
    /*if (oDriver->CarCharacteristic.IsValidX(Speed))
      Speed *= oDriver->CarCharacteristic.CalcOffset(Speed);*/

    //Speed = oDriver->CalcHairpin(Speed,AbsCrv);
    LogSHADOW.debug("CarModel CalcMaxSpeed = %.f\n", Speed);
    return Speed;
/*
    double	M  = MASS + FUEL;

    double	mua, muf, mur;

    if( AERO == 1 )
    {
        double	MU_F = kFriction * TYRE_MU_F;
        double	MU_R = kFriction * TYRE_MU_R;

        muf = MU_F * MU_SCALE;
        mur = MU_R * MU_SCALE;
        mua = (MU_F + MU_R) * 0.5;
    }
    else
    {
        double	MU = kFriction * TYRE_MU;

        mua   = MU * MU_SCALE;// * 0.975;
    }

    double	cs = cos(RollAngle);
    double	sn = sin(RollAngle);

    double	absK = MX(0.001, fabs(k));
    double	sgnK = SGN(k);

    double	num, den;

    if( AERO == 1 )
    {
        num = M * (cs * GRAVITY * mua + sn * G * sgnK);
        den = M * (absK - KZ_SCALE * kz) - (CA_FW * muf + CA_RW * mur + CA_GE * mua);
    }
    else
    {
//		num = M * (G * mu + sn * G * sgnK);
        num = M * (cs * G * mua + sn * G * sgnK);
//		den = M * (absK - 0.00 * kz) - CA * mu_df;
//		den = M * (absK - 0.10 * kz) - CA * mu_df;
//		den = M * (absK - 0.20 * kz) - CA * mu_df;
//		den = M * (absK - 0.25 * kz) - CA * mu_df;
//		den = M * (absK - 0.29 * kz) - CA * mu_df;
//		den = M * (absK - 0.33 * kz) - CA * mu_df;
//		den = M * (absK - 0.42 * kz) - CA * mu_df;
        den = M * (absK - KZ_SCALE * kz) - CA * mua;
    }

    if( den < 0.00001 )
        den = 0.00001;

    double	spd = sqrt(num / den);

    if( spd > 200 )
        spd = 200;

    return spd;*/
}

double	CarModel::CalcBreaking(double k0, double kz0, double k1, double kz1, double spd1, double dist, double kFriction, double RollAngle , double TiltAngle) const
{
    // when under braking we keep some grip in reserve.
    if (spd1 > 180/3.6)
      kFriction *= 0.90;
    else
      kFriction *= 0.95;

    double	cs  = cos(RollAngle);
    double  cs2 = cos(TiltAngle);
    double	sn  = sin(RollAngle);
    double  sn2 = sin(TiltAngle);

    double	K  = (0.3 * k0  + 0.9 * k1);
    double	Kz = (0.25 * kz0 + 0.75 * kz1);// * KZ_SCALE;
    if( Kz > 0 )
        Kz = 0;

    double	M  = MASS + FUEL;

    double	MU = kFriction * TYRE_MU;
    double	MU_F = MU;
    double	MU_R = MU;

    if( AERO == 1 )
    {
        MU_F = kFriction * TYRE_MU_F;
        MU_R = kFriction * TYRE_MU_R;
        MU   = (MU_F + MU_R) * 0.5;
        LogSHADOW.debug("CalcBreaking TYRE MUF = %.f - TYRE MUR = %.f - MU = %.f\n", MU_F, MU_R, MU);
    }

    if (HASTYC)
    {
      double TcF = TYRECONDITIONFRONT;
      double TcR = TYRECONDITIONREAR;
      MU_F = TcF * MU_F;
      MU_R = TcR * MU_R;
      MU = MIN(MU_F, MU_R); // SKILL;
      LogSHADOW.debug("CalcBreaking HASTYC TYRE MUF = %.f - TYRE MUR = %.f - MU = %.f\n", MU_F, MU_R, MU);
    }
    else
      MU = MIN(MU_F, MU_R); // oTmpCarParam->oSkill;

    double	CD = CD_BODY * (1.0 + DAMAGE / 10000.0) + CD_WING;

    double	Gdown = GRAVITY * cs * cs2;
    double	Glat  = fabs(sn * GRAVITY);
    double	Gtan  = - GRAVITY * sn2;

    double	v = spd1;
    double	u = v;

    for( int count = 0; count < 100; count++ )
    {
        double	avgV = (u + v) * 0.5;
        double	avgVV = avgV * avgV;

        double	Froad;

        if( AERO == 1 )
        {
            double	Fdown = M * Gdown + M * Kz * avgVV + CA_GE * avgVV;
            double	Ffrnt = CA_FW * avgVV;
            double	Frear = CA_RW * avgVV;

            Froad = Fdown * MU + Ffrnt * MU_F + Frear * MU_R; // maybe * 0.95
        }
        else
        {
            double	Fdown = M * Gdown + M * Kz * avgVV + CA * avgVV; // idem

            Froad = Fdown * MU;
        }
        double	Flat  = M * Glat;
        double	Ftan  = M * Gtan - CD * avgVV;

        double	Flatroad = fabs(M * avgVV * K - Flat);
        if( Flatroad > Froad )
            Flatroad = Froad;
        double	Ftanroad = -sqrt(Froad * Froad - Flatroad * Flatroad) + Ftan;

        double	acc = Ftanroad / M;

        acc = BRAKESCALE * Ftanroad / (MASS * ( 3 + SKILL) / 4);

        if (BRAKELIMIT)
        {
            double Radius = 1.0 / fabs(Kz);
            double factor = MIN(1.0,MAX(0.39, (Radius - 190.0) / 100.0));
            acc = MAX(acc, BRAKELIMIT * factor);
        }

        double	inner = MX(0, v * v - 2 * acc * dist );
        double	oldU = u;
        u = sqrt(inner);

        if( fabs(u - oldU) < 0.001 )
            break;
    }

    double midspd = (u + spd1)/2;

    // Check brake
    double brakedecel = BRAKESCALE * BRAKEFORCE / MASS;
    double braketargetspd = sqrt(midspd * midspd + 2 * brakedecel * dist);
    double resulttargetspd = MIN(u, braketargetspd);

    LogSHADOW.debug("CalcBreaking resulttargetspd = %.f\n", MAX(resulttargetspd, spd1));

    return MAX(resulttargetspd, spd1);
}

double	CarModel::CalcAcceleration(double k0, double kz0, double k1, double kz1, double spd0, double dist, double kFriction, double RollAngle , double TiltAngle) const
{
    double	M  = MASS + FUEL;
    double	MU = kFriction * TYRE_MU;
    double	CD = CD_BODY * (1.0 + DAMAGE / 10000.0) + CD_WING;

    // when under braking we keep some grip in reserve.
    MU *= 0.95;

    if (HASTYC)
    {
        double TcF = TYRECONDITIONFRONT;
        double TcR = TYRECONDITIONREAR;
        double MU_F = TcF * TYRE_MU_F;
        double MU_R = TcR * TYRE_MU_R;
        MU = MIN(MU_F, MU_R); // SKILL;
        LogSHADOW.debug("CalcAcceleration TYRE MUF = %.f - TYRE MUR = %.f - MU = %.f\n", MU_F, MU_R, MU);
    }

    double	cs = cos(RollAngle);
    double	sn = sin(RollAngle);
    double  sn2 = sin(TiltAngle);

    double	K  = (0.25 * k0  + 0.75 * k1);
    double	Kz = (0.25 * kz0 + 0.75 * kz1);// * KZ_SCALE;
    if( Kz > 0 )
        Kz = 0;

    double	Gdown = cs * GRAVITY;
    double	Glat  = sn * GRAVITY;
    double	Gtan  = - GRAVITY * sn2;

    double	u = spd0;
    double	v = u;

    // 30m/ss @ 0m/s
    //  3m/ss @ 60m/s
    //	1m/ss @ 75m/s
    //	0m/ss @ 85m/s
    Quadratic	accFromSpd(0.001852, -0.35, 17.7);	// approx. clkdtm
    double OldV = 0.0;
    // Power (kW) = Torque (Nm) x Speed (RPM) / 9.5488

    for( int count = 0; count < 100; count++ )
    {
        double	avgV = (u + v) * 0.5;
        double	vv = avgV * avgV;

        double	Fdown = M * Gdown + M * Kz * vv + CA * vv;
        double	Froad = Fdown * MU;
        double	Flat  = M * Glat;
        double	Ftan  = M * Gtan - CD * vv;

        double	Flatroad = fabs(M * vv * K - Flat);

        if( Flatroad > Froad )
            Flatroad = Froad;
        double	Ftanroad = sqrt(Froad * Froad - Flatroad * Flatroad) + Ftan;

        double	acc = Ftanroad / M;
        double	maxAcc = MIN(11.5, accFromSpd.CalcY(avgV));

        if( acc > maxAcc )
            acc = maxAcc;

        double	inner = MX(0, u * u + 2 * acc * dist );
        double	oldV = v;
        v = sqrt(inner);

        if( fabs(v - oldV) < 0.001 )
            break;

        OldV = v;
    }

    return v;
}

double	CarModel::CalcMaxSpdK() const
{
    const double	MAX_SPD = 112;	// ~400 kph

    return GRAVITY * TYRE_MU / (MAX_SPD * MAX_SPD);
}

double	CarModel::CalcMaxLateralF( double spd, double kFriction, double kz ) const
{
#if 0
    double	M  = MASS + FUEL;
    double	MU = kFriction * TYRE_MU;

    double	vv = spd * spd;

    double	Fdown = M * GRAVITY + /*M * Kz * vv*/ + CA * vv;
    double	Flat  = Fdown * MU;

    return Flat;
#else
    double Fdown = (MASS + FUEL) * GRAVITY + (MASS * kz + CA) * spd * spd;

    return Fdown * kFriction * TYRE_MU;
#endif
}

double CarModel::CalcMaxSpeedCrv() const
{
  const double MAX_SPD = 112; // 400 km/h
  return G * TYRE_MU / (MAX_SPD * MAX_SPD);
}

void	CarModel::CalcSimuSpeeds( double spd0, double dy, double dist, double kFriction, double& minSpd, double& maxSpd ) const
{
    // simple speed calc for use in simulation for path optimisation... the
    //	overriding pre-requisite of which is speed of calculation.
    //
    // a = v*v/r
    // max_a = M * G * MU;
    // max_spd = sqrt(max_a r) = sqrt(M * G * MU / k)

    //double	M  = MASS + FUEL;
    double	MU = kFriction * TYRE_MU;

    double	max_acc = GRAVITY * MU;
//	double	max_spd = k == 0 ? 200 : MN(200, sqrt(max_acc / k));

    //	s = ut + 0.5 att = dy
    //	a = 2(dy - ut) / tt      ... but lateral u = 0
    double	estT = dist / spd0;

    double	lat_acc = 2 * dy / (estT * estT);
    if( lat_acc > max_acc )
        lat_acc = max_acc;
    double	lin_acc = sqrt(max_acc * max_acc - lat_acc * lat_acc);

    //
    // accelerate
    //

    // acceleration is limited by engine power... and this quadratic
    //	is an estimation (poor, but hopefully good enough for our purposes).
    static const Quadratic	accFromSpd(0.001852, -0.35, 17.7);
    double	eng_acc = accFromSpd.CalcY(spd0) * kFriction;
    if( eng_acc > lin_acc )
        eng_acc = lin_acc;

    maxSpd = sqrt(spd0 * spd0 + 2 * eng_acc * dist);
//	if( maxSpd > max_spd )
//		maxSpd = max_spd;

    //
    // brake
    //

    minSpd = sqrt(spd0 * spd0 - 2 * lin_acc * dist);
}

void	CarModel::CalcSimuSpeedRanges( double spd0,	double dist, double	kFriction, double& minSpd, double& maxSpd, double& maxDY ) const
{
    // simple speed calc for use in simulation for path optimisation... the
    //	overriding pre-requisite of which is speed of calculation.
    //
    // a = v*v/r
    // max_a = M * G * MU;
    // max_spd = sqrt(max_a r) = sqrt(M * G * MU / k)

    //double	M  = MASS + FUEL;
    double	MU = kFriction * TYRE_MU;
    double	max_acc = GRAVITY * MU;

    //
    // accelerate
    //

    // acceleration is limited by engine power... and this quadratic
    //	is an estimation (poor, but hopefully good enough for our purposes).
    static const Quadratic	accFromSpd(0.001852, -0.35, 17.7);
    double	eng_acc = accFromSpd.CalcY(spd0) * kFriction;

    if( eng_acc > max_acc )
        eng_acc = max_acc;

    maxSpd = sqrt(spd0 * spd0 + 2 * eng_acc * dist);

    //
    // brake
    //

    minSpd = sqrt(spd0 * spd0 - 2 * max_acc * dist);

    //
    // turn (turning is symmetrical)
    //

    // s = ut + 1/2 att    u = 0, as we're looking along vel vector.
    // t = dist / spd0;
    double	turnT = dist / spd0;
    maxDY = 0.5 * max_acc * turnT * turnT;
}
