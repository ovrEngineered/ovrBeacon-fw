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
#ifndef OVR_CHARGINGMANAGER_H_
#define OVR_CHARGINGMANAGER_H_


// ******** includes ********
#include <stdbool.h>
#include <stdint.h>

#include <cxa_array.h>
#include <cxa_gpio.h>

#include <cxa_config.h>


// ******** global macro definitions ********
#ifndef OVR_CHARGEMANAGER_MAX_NUM_LISTENERS
#define OVR_CHARGEMANAGER_MAX_NUM_LISTENERS 1
#endif


// ******** global type definitions *********
/**
 * @public
 */
typedef struct ovr_chargingManager ovr_chargingManager_t;


/**
 * @public
 */
typedef enum
{
	OVR_CHARGESTATE_DISCHARGING,
	OVR_CHARGESTATE_CHARGING,
	OVR_CHARGESTATE_CHARGED
}ovr_chargingManager_chargeState_t;


/**
 * @public
 */
typedef void (*ovrChargingManager_cb_stateChange_t)(ovr_chargingManager_t *const chrgIn, ovr_chargingManager_chargeState_t newStateIn, void* userVarIn);


/**
 * @private
 */
typedef struct
{
	ovrChargingManager_cb_stateChange_t cb_stateChange;
	void* userVar;
}ovr_chargingManager_listenerEntry_t;


/**
 * @private
 */
struct ovr_chargingManager
{
	cxa_gpio_t* gpio_charging;
	cxa_gpio_t* gpio_chargeStat;

	ovr_chargingManager_chargeState_t lastState;

	cxa_array_t listeners;
	ovr_chargingManager_listenerEntry_t listeners_raw[OVR_CHARGEMANAGER_MAX_NUM_LISTENERS];
};


// ******** global function prototypes ********
void ovr_chargingManager_init(ovr_chargingManager_t *const chrgIn, cxa_gpio_t *const gpio_chargingIn, cxa_gpio_t *const gpio_chargeStatIn);

bool ovr_chargingManager_addListener(ovr_chargingManager_t *const chrgIn, ovrChargingManager_cb_stateChange_t cbIn, void* userVarIn);

ovr_chargingManager_chargeState_t ovr_chargingManager_getCurrentState(ovr_chargingManager_t *const chrgIn);
const char* ovr_chargingManager_getCurrentState_string(ovr_chargingManager_t *const chrgIn);

#endif /* OVR_CHARGINGMANAGER_H_ */
