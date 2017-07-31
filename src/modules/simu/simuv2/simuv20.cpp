/***************************************************************************

    file                 : simuitf.cpp
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

#include "simuv20.h"

#include "sim.h"


// The SimuV20: singleton.
SimuV20* SimuV20::_pSelf = 0;

int openGfModule(const char* pszShLibName, void* hShLibHandle)
{
	// Instanciate the (only) module instance.
	SimuV20::_pSelf = new SimuV20(pszShLibName, hShLibHandle);

	// Register it to the GfModule module manager if OK.
	if (SimuV20::_pSelf)
		GfModule::register_(SimuV20::_pSelf);

	// Report about success or error.
	return SimuV20::_pSelf ? 0 : 1;
}

int closeGfModule()
{
	// Unregister it from the GfModule module manager.
	if (SimuV20::_pSelf)
		GfModule::unregister(SimuV20::_pSelf);
	
	// Delete the (only) module instance.
	delete SimuV20::_pSelf;
	SimuV20::_pSelf = 0;

	// Report about success or error.
	return 0;
}

SimuV20& SimuV20::self()
{
	// Pre-condition : 1 successfull openGfModule call.
	return *_pSelf;
}

SimuV20::SimuV20(const std::string& strShLibName, void* hShLibHandle)
: GfModule(strShLibName, hShLibHandle)
{
}

SimuV20::~SimuV20()
{
}

// Implementation of IPhysicsEngine.
void SimuV20::initialize(int nCars, struct Track* pTrack)
{
	::SimInit(nCars, pTrack);
}

void SimuV20::configureCar(struct CarElt* pCar)
{
	::SimConfig(pCar);
}

void SimuV20::reconfigureCar(struct CarElt* pCar)
{
	::SimReConfig(pCar);
}

void SimuV20::toggleCarTelemetry(int nCarIndex, bool bOn)
{
	::SimCarTelemetry(nCarIndex, bOn);
}

void SimuV20::updateSituation(struct Situation *pSituation, double fDeltaTime)
{
	::SimUpdate(pSituation, fDeltaTime);
}

void SimuV20::updateCar(struct Situation *pSituation, double fDeltaTime, int nCarIndex)
{
	::SimUpdateSingleCar(nCarIndex, fDeltaTime, pSituation);
}

void SimuV20::setCar(const struct DynPt& dynGCG, int nCarIndex)
{
	::UpdateSimCarTable(dynGCG, nCarIndex);
}

tDynPt* SimuV20::getCar(int nCarIndex)
{
	return ::GetSimCarTable(nCarIndex);
}

void SimuV20::shutdown()
{
	::SimShutdown();
}
