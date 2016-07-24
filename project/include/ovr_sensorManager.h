/**
 * @file
 *
 * @note This object should work across all architecture-specific implementations
 *
 *
 * #### Example Usage: ####
 *
 * @code
 * @endcode
 *
 *
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
#ifndef INCLUDE_OVR_SENSORMANAGER_H_
#define INCLUDE_OVR_SENSORMANAGER_H_


// ******** includes ********
#include <stdbool.h>
#include <stdint.h>

#include <cxa_batteryCapacityEstimator.h>
#include <cxa_logger_header.h>
#include <cxa_tempSensor.h>


// ******** global macro definitions ********


// ******** global type definitions *********
/**
 * @public
 */
typedef struct ovr_sensorManager ovr_sensorManager_t;


/**
 * @public
 */
typedef void (*ovrSensorManager_cb_readComplete_t)(ovr_sensorManager_t *const smIn, uint8_t batt_pcnt100In, int8_t temp_cIn, void* userVarIn);


/**
 * @private
 */
struct ovr_sensorManager
{
	cxa_batteryCapacityEstimator_t* battEst;
	cxa_tempSensor_t* tempSense;

	uint8_t lastBatt_pcnt100;
	int8_t lastTemp_c;

	ovrSensorManager_cb_readComplete_t cb_readComp;
	void* userVar;

	cxa_logger_t logger;
};


// ******** global function prototypes ********
void ovr_sensorManager_init(ovr_sensorManager_t *const smIn, cxa_batteryCapacityEstimator_t *const battEstIn,
							cxa_tempSensor_t *const tempSnsIn);

bool ovr_sensorManager_readSensors(ovr_sensorManager_t *const smIn, ovrSensorManager_cb_readComplete_t cbIn, void* userVarIn);

bool ovr_sensorManager_isReadInProgress(ovr_sensorManager_t *const smIn);

#endif /* INCLUDE_OVR_SENSORMANAGER_H_ */
