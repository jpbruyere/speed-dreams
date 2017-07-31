/***************************************************************************

    file                 : simureplay.cpp
    created              : Sun Mar 19 00:08:04 CET 2000
    copyright            : (C) 2000 by Eric Espie
    email                : torcs@free.fr
    version              : $Id: simuv4.cpp 3568 2011-05-15 15:55:24Z pouillot $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include "simureplay.h"

#include "sim.h"


// The Simureplay: singleton.
SimuReplay* SimuReplay::_pSelf = 0;

int openGfModule(const char* pszShLibName, void* hShLibHandle)
{
	// Instanciate the (only) module instance.
	SimuReplay::_pSelf = new SimuReplay(pszShLibName, hShLibHandle);

	// Register it to the GfModule module manager if OK.
	if (SimuReplay::_pSelf)
		GfModule::register_(SimuReplay::_pSelf);

	// Report about success or error.
	return SimuReplay::_pSelf ? 0 : 1;
}

int closeGfModule()
{
	// Unregister it from the GfModule module manager.
	if (SimuReplay::_pSelf)
		GfModule::unregister(SimuReplay::_pSelf);
	
	// Delete the (only) module instance.
	delete SimuReplay::_pSelf;
	SimuReplay::_pSelf = 0;

	// Report about success or error.
	return 0;
}

SimuReplay& SimuReplay::self()
{
	// Pre-condition : 1 successfull openGfModule call.
	return *_pSelf;
}

SimuReplay::SimuReplay(const std::string& strShLibName, void* hShLibHandle)
: GfModule(strShLibName, hShLibHandle)
{
}

SimuReplay::~SimuReplay()
{
}

// Implementation of IPhysicsEngine.
void SimuReplay::initialize(int nCars, struct Track* pTrack)
{
	::SimInit(nCars, pTrack);
}

void SimuReplay::configureCar(struct CarElt* pCar)
{
	::SimConfig(pCar);
}

void SimuReplay::reconfigureCar(struct CarElt* pCar)
{
	::SimReConfig(pCar);
}

void SimuReplay::toggleCarTelemetry(int nCarIndex, bool bOn)
{
	::SimCarTelemetry(nCarIndex, bOn);
}

void SimuReplay::updateSituation(struct Situation *pSituation, double fDeltaTime)
{
	::SimUpdate(pSituation, fDeltaTime);
}

void SimuReplay::updateCar(struct Situation *pSituation, double fDeltaTime, int nCarIndex)
{
	::SimUpdateSingleCar(nCarIndex, fDeltaTime, pSituation);
}

void SimuReplay::setCar(const struct DynPt& dynGCG, int nCarIndex)
{
	::UpdateSimCarTable(dynGCG, nCarIndex);
}

tDynPt* SimuReplay::getCar(int nCarIndex)
{
	return ::GetSimCarTable(nCarIndex);
}

void SimuReplay::shutdown()
{
	::SimShutdown();
}
