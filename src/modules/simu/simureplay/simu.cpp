/***************************************************************************

    file                 : simu.cpp
    created              : Sun Mar 19 00:07:53 CET 2000
    copyright            : (C) 2000 by Eric Espie
    email                : torcs@free.fr
    version              : $Id: simu.cpp 3945 2011-10-07 13:38:15Z wdbee $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include <cstdlib>
#include <cstdio>
#include <memory.h>
#include <cmath>

#include <sqlite3.h>

#include <portability.h>
#include <tgf.h>
#include <robottools.h>

#include <replay.h>

#include "sim.h"

tCar *SimCarTable = 0;
int SimTelemetry = -1;
static tTrack *PTrack = 0;
static int SimNbCars = 0;

int    replayForward;
double lastCurTime;
double replayTimeOffset;
static tReplayElt curReplayData[50];
static tReplayElt nextReplayData[50];

static int old_id;

/*
 * Check the input control from robots
 */
static void
ctrlCheck(tCar *car)
{
}

/* Initial configuration */
void
SimConfig(tCarElt *carElt)
{
    GfLogInfo("Replay SimConfig\n");

    tCar *car = &(SimCarTable[carElt->index]);

    memset(car, 0, sizeof(tCar));

    car->carElt = carElt;

    car->DynGC = carElt->_DynGC;
    car->DynGCg = carElt->pub.DynGCg;
    car->trkPos = carElt->_trkPos;
    car->ctrl   = &carElt->ctrl;
    car->params = carElt->_carHandle;

#if 0
    SimCarConfig(car);

    SimCarCollideConfig(car, PTrack);
#endif
    sgMakeCoordMat4(carElt->pub.posMat, carElt->_pos_X, carElt->_pos_Y, carElt->_pos_Z - carElt->_statGC_z,
		    (float) RAD2DEG(carElt->_yaw), (float) RAD2DEG(carElt->_roll), (float) RAD2DEG(carElt->_pitch));

    replayTimeOffset = 0;
}

/* After pit stop */
void
SimReConfig(tCarElt *carElt)
{
}


static void
RemoveCar(tCar *car, tSituation *s)
{
}

void
SimCarTelemetry(int nCarIndex, bool bOn)
{
    SimTelemetry = bOn ? nCarIndex : -1;
}

void
SimUpdate(tSituation *s, double deltaTime)
{
	tCar *car;
	tCarElt *pTgtCar;
	tReplayElt *pSrcCar, *pSrc2Car;

        char command[200];
        int result, result2;
	int reload = 0;

        if (replayDB == NULL) {
		GfLogInfo("Replay NULL\n");
		return;
	}

	// Check for reversal of time
	if (replayForward && s->currentTime < lastCurTime) {
		GfLogInfo("REPLAY: time running backward at timestamp %f\n", s->currentTime);
		reload = 1;
		replayForward = 0;
	} else if (replayForward == 0 && s->currentTime > lastCurTime) {
		GfLogInfo("REPLAY: time running forward at timestamp %f\n", s->currentTime);
		reload = 1;
		replayForward = 1;
	}

	lastCurTime = s->currentTime;

	// readback next race record as required
        for (int nCarInd = 0; nCarInd < s->_ncars; nCarInd++) {
		if (reload || 
				(replayForward && s->currentTime >= nextReplayData[nCarInd].currentTime) ||
				(replayForward == 0 && s->currentTime <= nextReplayData[nCarInd].currentTime)) {
			result = sqlite3_step(replayBlobs[nCarInd]);

			// check if the previous SELECT has returned all it's rows
			if (replayForward && (result == SQLITE_DONE || reload)) {
				// and read another chunk
				GfLogInfo("REPLAY: Chunk forward timestamp %f (%f)\n", s->currentTime, deltaTime);
				sprintf(command, "SELECT datablob FROM car%d WHERE timestamp >= %f LIMIT %d",
						nCarInd, s->currentTime, REPLAY_CHUNK);
				result2 = sqlite3_prepare_v2(replayDB, command, -1, &replayBlobs[nCarInd], 0);

				if (result2) {
					GfLogInfo("Unable to prepare car%d: %s\n", nCarInd, sqlite3_errmsg(replayDB));
				} else {
					result = sqlite3_step(replayBlobs[nCarInd]);
				}
			}
			if (replayForward == 0 && (result == SQLITE_DONE || reload)) {
				// and read another chunk
				GfLogInfo("REPLAY: Chunk backward timestamp %f (%f)\n", s->currentTime, deltaTime);
				sprintf(command, "SELECT datablob FROM car%d WHERE timestamp <= %f ORDER BY timestamp DESC LIMIT %d",
						nCarInd, s->currentTime, REPLAY_CHUNK);
				result2 = sqlite3_prepare_v2(replayDB, command, -1, &replayBlobs[nCarInd], 0);

				if (result2) {
					GfLogInfo("Unable to prepare car%d: %s\n", nCarInd, sqlite3_errmsg(replayDB));
				} else {
					result = sqlite3_step(replayBlobs[nCarInd]);
				}
			}

			if (result == SQLITE_ROW) {
				curReplayData[nCarInd] = nextReplayData[nCarInd];
				memcpy(&nextReplayData[nCarInd], sqlite3_column_blob(replayBlobs[nCarInd], 0), sizeof(tReplayElt));

				nextReplayData[nCarInd].currentTime += replayTimeOffset;
			}

			// allow for replaying data again and again
			if (result == SQLITE_DONE) replayTimeOffset = s->currentTime;

			//GfLogInfo("Replay read car%d: %f\n", nCarInd, curReplayData[nCarInd].currentTime);

			car = &(SimCarTable[nCarInd]);
			pTgtCar = car->carElt;
			pSrcCar = &curReplayData[nCarInd];

#if 1
			// Really this should only be read once at start of race
			pSrcCar->race.pit = pTgtCar->race.pit;
			pSrcCar->race.bestSplitTime = pTgtCar->race.bestSplitTime;
			pSrcCar->race.curSplitTime = pTgtCar->race.curSplitTime;

			memcpy(&pTgtCar->race, &pSrcCar->race, sizeof(tCarRaceInfo));
#endif

#if 1
			// hack to fix trkpos
			pSrcCar->pub.trkPos = pTgtCar->pub.trkPos;

			memcpy(&pTgtCar->pub, &pSrcCar->pub, sizeof(tPublicCar));
			pTgtCar->_glance = 0;
			pTgtCar->_oldglance = 0;
#endif

#if 1
			memcpy(&pTgtCar->info, &pSrcCar->info, sizeof(tInitCar));
			pTgtCar->_wrongWayTime = s->currentTime + 5.0;
#endif

#if 0
			memcpy(&pTgtCar->priv, &pSrcCar->priv, sizeof(tPrivCar));
#else
			// Selectively pick 'priv' data
			for(int i=0; i < 4; i++) {
				pTgtCar->priv.wheel[i] = pSrcCar->priv.wheel[i];
				pTgtCar->priv.wheel[i].seg = RtTrackGetSeg(&pTgtCar->_trkPos);
				pTgtCar->_skid[i] = pSrcCar->_skid[i];
				pTgtCar->priv.wheel[i].spinVel = pSrcCar->priv.wheel[i].spinVel;
			}
			pTgtCar->priv.gear = pSrcCar->priv.gear;
			pTgtCar->priv.fuel = pSrcCar->priv.fuel;
			pTgtCar->priv.enginerpm = pSrcCar->priv.enginerpm;
			pTgtCar->priv.enginerpmRedLine = pSrcCar->priv.enginerpmRedLine;
			pTgtCar->priv.dammage = pSrcCar->priv.dammage;
#endif

#if 0
			memcpy(&pTgtCar->ctrl, &pSrcCar->ctrl, sizeof(tCarCtrl));
#else
			pTgtCar->_steerCmd = pSrcCar->_steerCmd;
			pTgtCar->_brakeCmd = pSrcCar->_brakeCmd;
			pTgtCar->_clutchCmd = pSrcCar->_clutchCmd;
			pTgtCar->_gearCmd = pSrcCar->_gearCmd;
#endif

#if 0
			memcpy(&pTgtCar->pitcmd, &pSrcCar->pitcmd, sizeof(tCarPitCmd));
#endif

			// figure out track segment (allowing that they may be allocated to different memory location)
			pTgtCar->_distFromStartLine = pSrcCar->_distFromStartLine;
#if 0
			if (pTgtCar->pub.trkPos.seg->id != old_id) {
				printf("New TrkSeg %d @ %fs: DfS %f : %s = %fm", pTgtCar->pub.trkPos.seg->id, s->currentTime,
						pTgtCar->_distFromStartLine,
						pTgtCar->pub.trkPos.seg->name, pTgtCar->pub.trkPos.seg->lgfromstart);
				if (pTgtCar->pub.trkPos.seg->raceInfo & TR_LAST) printf(" LAST");
				if (pTgtCar->pub.trkPos.seg->raceInfo & TR_START) printf(" START");
				printf("\n");
				old_id = pTgtCar->pub.trkPos.seg->id;
			}
#endif
			
			if (pTgtCar->_distFromStartLine > pTgtCar->pub.trkPos.seg->next->lgfromstart) {
				// Next segment, forward direction
				pTgtCar->pub.trkPos.seg = pTgtCar->pub.trkPos.seg->next;
			} else if (pTgtCar->pub.trkPos.seg->raceInfo & TR_LAST == TR_LAST &&
					pTgtCar->pub.trkPos.seg->next->raceInfo & TR_START == TR_START &&
					pTgtCar->_distFromStartLine < pTgtCar->pub.trkPos.seg->lgfromstart/2) {
				// Crossing Start Line, forward direction
				// something weird here, sometime reports wrong lap completed
				pTgtCar->pub.trkPos.seg = pTgtCar->pub.trkPos.seg->next;
			} else if (pTgtCar->_distFromStartLine < pTgtCar->pub.trkPos.seg->lgfromstart) {
				// Previous segment, reverse direction
				pTgtCar->pub.trkPos.seg = pTgtCar->pub.trkPos.seg->prev;
			}
		}

		if ((replayForward  && s->currentTime < nextReplayData[nCarInd].currentTime) ||
				(replayForward == 0 && s->currentTime > nextReplayData[nCarInd].currentTime)) {
			// Interpolate position in between records
			double timeFrac;
			double yaw, roll, pitch;

			car = &(SimCarTable[nCarInd]);
			pTgtCar = car->carElt;
			pSrcCar = &curReplayData[nCarInd];
			pSrc2Car = &nextReplayData[nCarInd];

			if (replayForward) 
				timeFrac = (s->currentTime - curReplayData[nCarInd].currentTime) /
					(nextReplayData[nCarInd].currentTime - curReplayData[nCarInd].currentTime);
			else
				timeFrac = (curReplayData[nCarInd].currentTime - s->currentTime) /
					(curReplayData[nCarInd].currentTime - nextReplayData[nCarInd].currentTime);

			// position on track
			pTgtCar->_pos_X = pSrcCar->_pos_X + (pSrc2Car->_pos_X - pSrcCar->_pos_X) * timeFrac;
			pTgtCar->_pos_Y = pSrcCar->_pos_Y + (pSrc2Car->_pos_Y - pSrcCar->_pos_Y) * timeFrac;
			pTgtCar->_statGC_z = pSrcCar->_statGC_z + (pSrc2Car->_statGC_z - pSrcCar->_statGC_z) * timeFrac;
			pTgtCar->_pos_Z = pSrcCar->_pos_Z + (pSrc2Car->_pos_Z - pSrcCar->_pos_Z) * timeFrac - pTgtCar->_statGC_z;

			yaw = pSrc2Car->_yaw;
			roll = pSrc2Car->_roll;
			pitch = pSrc2Car->_pitch;

			// assumes that these can't change at high rate
			if (yaw < pSrcCar->_yaw - PI)
				yaw += 2 * PI;
			else if (yaw > pSrcCar->_yaw + PI)
				yaw -= 2 * PI;

			if (roll < pSrcCar->_roll - PI)
				roll += 2 * PI;
			else if (roll > pSrcCar->_roll + PI)
				roll -= 2 * PI;

			if (pitch < pSrcCar->_pitch - PI)
				pitch += 2 * PI;
			else if (pitch > pSrcCar->_pitch + PI)
				pitch -= 2 * PI;

			pTgtCar->_yaw = pSrcCar->_yaw + (yaw - pSrcCar->_yaw) * timeFrac;
			pTgtCar->_roll = pSrcCar->_roll + (roll - pSrcCar->_roll) * timeFrac;
			pTgtCar->_pitch = pSrcCar->_pitch + (pitch - pSrcCar->_pitch) * timeFrac;

			sgMakeCoordMat4(pTgtCar->pub.posMat, pTgtCar->_pos_X, pTgtCar->_pos_Y, pTgtCar->_pos_Z,
					(tdble) RAD2DEG(pTgtCar->_yaw), (tdble) RAD2DEG(pTgtCar->_roll),
					(tdble) RAD2DEG(pTgtCar->_pitch));

			// some cameras use speed to determine rain behaviour
			pTgtCar->_speed_x = pSrcCar->_speed_x + (pSrc2Car->_speed_x - pSrcCar->_speed_x) * timeFrac;
			pTgtCar->_speed_y = pSrcCar->_speed_y + (pSrc2Car->_speed_y - pSrcCar->_speed_y) * timeFrac;
			pTgtCar->_speed_z = pSrcCar->_speed_z + (pSrc2Car->_speed_z - pSrcCar->_speed_z) * timeFrac;
			pTgtCar->_speed_xy = pSrcCar->_speed_xy + (pSrc2Car->_speed_xy - pSrcCar->_speed_xy) * timeFrac;

			pTgtCar->_enginerpm = pSrcCar->_enginerpm + (pSrc2Car->_enginerpm - pSrcCar->_enginerpm) * timeFrac;

			// Audio works in world cordinates
			pTgtCar->pub.DynGCg.pos.x = pSrcCar->pub.DynGCg.pos.x + (pSrc2Car->pub.DynGCg.pos.x - pSrcCar->pub.DynGCg.pos.x) * timeFrac;
			pTgtCar->pub.DynGCg.pos.y = pSrcCar->pub.DynGCg.pos.y + (pSrc2Car->pub.DynGCg.pos.y - pSrcCar->pub.DynGCg.pos.y) * timeFrac;
			pTgtCar->pub.DynGCg.pos.z = pSrcCar->pub.DynGCg.pos.z + (pSrc2Car->pub.DynGCg.pos.z - pSrcCar->pub.DynGCg.pos.z) * timeFrac;
                                                                                                        
			pTgtCar->pub.DynGCg.vel.x = pSrcCar->pub.DynGCg.vel.x + (pSrc2Car->pub.DynGCg.vel.x - pSrcCar->pub.DynGCg.vel.x) * timeFrac;
			pTgtCar->pub.DynGCg.vel.y = pSrcCar->pub.DynGCg.vel.y + (pSrc2Car->pub.DynGCg.vel.y - pSrcCar->pub.DynGCg.vel.y) * timeFrac;
			pTgtCar->pub.DynGCg.vel.z = pSrcCar->pub.DynGCg.vel.z + (pSrc2Car->pub.DynGCg.vel.z - pSrcCar->pub.DynGCg.vel.z) * timeFrac;

			// Sound uses accelCmd to antenuate engine 
			pTgtCar->_accelCmd = pSrcCar->_accelCmd + (pSrc2Car->_accelCmd - pSrcCar->_accelCmd) * timeFrac;
		}
	}
}


void
SimInit(int nbcars, tTrack* track)
{
	tCar *car;
	tCarElt *pTgtCar;
        char command[200];
        int result;


	GfLogInfo("Replay SimInit\n");

        if (replayDB == NULL) {
		GfLogInfo("Replay Database Not Opened!!\n\n\n\n");
		return;
	}

	SimNbCars = nbcars;
	SimCarTable = (tCar*)calloc(nbcars, sizeof(tCar));
	PTrack = track;
	//SimCarCollideInit(PTrack);

	for (int nCarInd = 0; nCarInd < nbcars; nCarInd++) {
		sprintf(command, "SELECT datablob FROM car%d LIMIT %d", nCarInd, REPLAY_CHUNK);
		result = sqlite3_prepare_v2(replayDB, command, -1, &replayBlobs[nCarInd], 0);

		if (result) {
			GfLogInfo("Unable to prepare car%d: %s\n", nCarInd, sqlite3_errmsg(replayDB));
		} else {
			replayForward = 1;
			lastCurTime = -10.0;

			// read the first 2 records for each car
			result = sqlite3_step(replayBlobs[nCarInd]);
			if (result == SQLITE_ROW) {
				memcpy(&curReplayData[nCarInd], sqlite3_column_blob(replayBlobs[nCarInd], 0), sizeof(tReplayElt));
				lastCurTime = curReplayData[nCarInd].currentTime;
			}

			result = sqlite3_step(replayBlobs[nCarInd]);
			if (result == SQLITE_ROW) {
				memcpy(&nextReplayData[nCarInd], sqlite3_column_blob(replayBlobs[nCarInd], 0), sizeof(tReplayElt));
			}
#if 0
			// preload info into structure
			car = &(SimCarTable[nCarInd]);
			pTgtCar = car->carElt;
			memcpy(&pTgtCar->info, &curReplayData[nCarInd].info, sizeof(tInitCar));
#endif
		}
	}

	GfLogInfo("SimuReplay recording starts at %f\n", lastCurTime);
}

void
SimShutdown(void)
{
    tCar *car;
    int	 ncar;

    //SimCarCollideShutdown(SimNbCars);
    if (SimCarTable) {
	for (ncar = 0; ncar < SimNbCars; ncar++) {
	    car = &(SimCarTable[ncar]);
	    //SimEngineShutdown(car);
	}
	free(SimCarTable);
	SimCarTable = 0;
    }

	PTrack = 0;
}

/* Used for network games to update client physics */
void 
UpdateSimCarTable(tDynPt DynGCG,int index)
{
	tCar *pCar = SimCarTable;
	pCar[index].DynGCg = DynGCG;
}

/* Used for network games get current physics values*/
tDynPt * 
GetSimCarTable(int index)
{
	tCar *pCar = SimCarTable;
	return &pCar[index].DynGCg;
}


void
SimUpdateSingleCar(int index, double deltaTime,tSituation *s)
{
	GfLogInfo("Replay UpdateSingleCar\n");
}

