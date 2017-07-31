/***************************************************************************

    file                 : simuv30.cpp
    created              : Sun Mar 19 00:08:04 CET 2000
    copyright            : (C) 2000 by Eric Espie
    email                : torcs@free.fr
    version              : $Id$

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include "simuv30.h"

#include "sim.h"


// The Simuv30: singleton.
Simuv30* Simuv30::_pSelf = 0;

int openGfModule(const char* pszShLibName, void* hShLibHandle)
{
	// Instanciate the (only) module instance.
	Simuv30::_pSelf = new Simuv30(pszShLibName, hShLibHandle);

	// Register it to the GfModule module manager if OK.
	if (Simuv30::_pSelf)
		GfModule::register_(Simuv30::_pSelf);

	// Report about success or error.
	return Simuv30::_pSelf ? 0 : 1;
}

int closeGfModule()
{
	// Unregister it from the GfModule module manager.
	if (Simuv30::_pSelf)
		Simuv30::unregister(Simuv30::_pSelf);
	
	// Delete the (only) module instance.
	delete Simuv30::_pSelf;
	Simuv30::_pSelf = 0;

	// Report about success or error.
	return 0;
}

Simuv30& Simuv30::self()
{
	// Pre-condition : 1 successfull openGfModule call.
	return *_pSelf;
}

Simuv30::Simuv30(const std::string& strShLibName, void* hShLibHandle)
: GfModule(strShLibName, hShLibHandle)
{
}

Simuv30::~Simuv30()
{
}

// Implementation of IPhysicsEngine.
void Simuv30::initialize(int nCars, struct Track* pTrack)
{
	::SimInit(nCars, pTrack);
}

void Simuv30::configureCar(struct CarElt* pCar)
{
	::SimConfig(pCar);
}

void Simuv30::reconfigureCar(struct CarElt* pCar)
{
	::SimReConfig(pCar);
}

void Simuv30::toggleCarTelemetry(int nCarIndex, bool bOn)
{
	::SimCarTelemetry(nCarIndex, bOn);
}

void Simuv30::updateSituation(struct Situation *pSituation, double fDeltaTime)
{
	::SimUpdate(pSituation, fDeltaTime);
}

void Simuv30::updateCar(struct Situation *pSituation, double fDeltaTime, int nCarIndex)
{
	::SimUpdateSingleCar(nCarIndex, fDeltaTime, pSituation);
}

void Simuv30::setCar(const struct DynPt& dynGCG, int nCarIndex)
{
	::UpdateSimCarTable(dynGCG, nCarIndex);
}

tDynPt* Simuv30::getCar(int nCarIndex)
{
	return ::GetSimCarTable(nCarIndex);
}

void Simuv30::shutdown()
{
	::SimShutdown();
}
