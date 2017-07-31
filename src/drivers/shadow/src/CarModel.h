/***************************************************************************

    file        : CarModel.h
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

#ifndef _CARMODEL_H_
#define _CARMODEL_H_

// my own planet ...
#define GRAVITY 9.81

class CarModel  
{
public:
    CarModel();
    ~CarModel();

    double	CalcMaxSpeed(double k0, double k1, double kz, double kFriction, double RollAngle , double TiltAngle) const;
    double	CalcBreaking( double k0, double kz0, double k1, double kz1, double spd1, double dist, double kFriction, double RollAngle, double TiltAngle ) const;
    double	CalcAcceleration( double k0, double kz0, double k1, double kz1, double spd0, double dist, double kFriction, double RollAngle, double TiltAngle ) const;
    double	CalcMaxSpdK() const;
    double	CalcMaxLateralF(double spd, double kFriction , double kz = 0.0) const;

	double CalcMaxSpeedCrv() const;

    void	CalcSimuSpeeds( double spd0, double dy, double dist, double kFriction, double& minSpd, double& maxSpd ) const;
    void	CalcSimuSpeedRanges( double spd0, double dist, double kFriction, double& minSpd, double& maxSpd, double& maxDY ) const;

public:
    int		AERO;           // which aero calc to use.
    double  EMPTYMASS;
    double	MASS;           // fixed mass of car.
    double  LENGTH;         // Length of car (m)
    double	FUEL;           // mass of fuel in car.
    double	DAMAGE;         // damage of this car.
    bool    NEEDSINLONG;
    bool    USEDACCEXIT;
    double  SKILL;          // skill car driver.

    double	TYRE_MU;        // mu value of tyres (min of those avail).
    double	TYRE_MU_F;      // mu value of front tyres.
    double	TYRE_MU_R;      // mu value of rear  tyres.
    double	MU_SCALE;       // scaling of MU to use for this car.
    double  MIN_MU_SCALE;   // Scaling of Min MU

    double  BRAKESCALE;     // Scaling of Brake
    double  BRAKEFORCE;     // Brake force max
	float	BRAKELIMIT;		// BrakeLimit

    double	CA;             // aerodynamic downforce constant -- total.
    double	CA_FW;          // aerodynamic downforce constant -- front wing.
    double	CA_RW;          // aerodynamic downforce constant -- rear wing.
    double	CA_GE;          // aerodynamic downforce constant -- ground effect.
    double  CA_GE_F;
    double  CA_GE_R;
    double	CD_BODY;        // aerodynamic drag constant -- car body.
    double	CD_WING;        // aerodynamic drag constant -- wings
    double  CD_CX;
    double	KZ_SCALE;       // bump sensitivity.
    double  BUMP_FACTOR;    // bump sensitivity factor.
	double	BUMP_FACTORLEFT;
	double	BUMP_FACTORRIGHT;
    double	WIDTH;          // width of car (m).
	bool	HASTYC;
	double  TYRECONDITIONFRONT;
	double	TYRECONDITIONREAR;
};

#endif
