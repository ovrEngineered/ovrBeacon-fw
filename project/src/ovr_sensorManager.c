/**
 * @copyright 2016 opencxa.org
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author Christopher Armenio
 */
#include "ovr_sensorManager.h"


// ******** includes ********
#include <cxa_assert.h>

#define CXA_LOG_LEVEL		CXA_LOG_LEVEL_TRACE
#include <cxa_logger_implementation.h>


// ******** local macro definitions ********


// ******** local type definitions ********


// ******** local function prototypes ********
static void cb_battEst(cxa_batteryCapacityEstimator_t *const cbeIn, float battPcntIn, void* userVarIn);
static void cb_tempSense(cxa_tempSensor_t *const tmpSnsIn, float newTemp_degCIn, void* userVarIn);


// ********  local variable declarations *********


// ******** global function implementations ********
void ovr_sensorManager_init(ovr_sensorManager_t *const smIn, cxa_batteryCapacityEstimator_t *const battEstIn,
							cxa_tempSensor_t *const tempSnsIn)
{
	cxa_assert(smIn);
	cxa_assert(battEstIn);
	cxa_assert(tempSnsIn);

	// save our references
	smIn->battEst = battEstIn;
	smIn->tempSense = tempSnsIn;

	// set some initial values
	smIn->cb_readComp = NULL;

	// setup our logger
	cxa_logger_init(&smIn->logger, "snsMan");
}


bool ovr_sensorManager_readSensors(ovr_sensorManager_t *const smIn, ovrSensorManager_cb_readComplete_t cbIn, void* userVarIn)
{
	cxa_assert(smIn);

	// make sure we're not in the middle of a reading
	if( smIn->cb_readComp != NULL ) return false;

	// save our callback
	smIn->cb_readComp = cbIn;
	smIn->userVar = userVarIn;

	// start by reading our battery
	return cxa_batteryCapacityEstimator_getValue_withCallback(smIn->battEst, cb_battEst, (void*)smIn);
}


bool ovr_sensorManager_isReadInProgress(ovr_sensorManager_t *const smIn)
{
	cxa_assert(smIn);

	return (smIn->cb_readComp != NULL);
}


// ******** local function implementations ********
static void cb_battEst(cxa_batteryCapacityEstimator_t *const cbeIn, float battPcntIn, void* userVarIn)
{
	ovr_sensorManager_t* smIn = (ovr_sensorManager_t*)userVarIn;
	cxa_assert(smIn);

	smIn->lastBatt_pcnt100 = (uint8_t)(battPcntIn * 100.0);
	cxa_logger_debug(&smIn->logger, "battPcnt: %d", smIn->lastBatt_pcnt100);

	// now get our temperature
	cxa_tempSensor_getValue_withCallback(smIn->tempSense, cb_tempSense, (void*)smIn);
}


static void cb_tempSense(cxa_tempSensor_t *const tmpSnsIn, float newTemp_degCIn, void* userVarIn)
{
	ovr_sensorManager_t* smIn = (ovr_sensorManager_t*)userVarIn;
	cxa_assert(smIn);

	smIn->lastTemp_c = (int8_t)newTemp_degCIn;
	cxa_logger_debug(&smIn->logger, "temp_f: %d", (unsigned int)CXA_TEMPSENSE_CTOF(smIn->lastTemp_c));

	// save a local copy of our callback and uservar to avoid race conditions
	ovrSensorManager_cb_readComplete_t cb = smIn->cb_readComp;
	void* userVar = smIn->userVar;

	// clear our callbacks so the user _could_ start another read in the callback
	smIn->cb_readComp = NULL;

	// call our stored callbacks
	if( cb != NULL ) cb(smIn, smIn->lastBatt_pcnt100, smIn->lastTemp_c, userVar);
}
