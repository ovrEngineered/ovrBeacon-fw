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
#include "ovr_chargingManager.h"


// ******** includes ********
#include <cxa_assert.h>
#include <cxa_runLoop.h>


// ******** local macro definitions ********


// ******** local type definitions ********


// ******** local function prototypes ********
static void cb_onRunLoopUpdate(void* userVarIn);


// ********  local variable declarations *********


// ******** global function implementations ********
void ovr_chargingManager_init(ovr_chargingManager_t *const chrgIn, cxa_gpio_t *const gpio_chargingIn, cxa_gpio_t *const gpio_chargeStatIn)
{
	cxa_assert(chrgIn);
	cxa_assert(gpio_chargingIn);
	cxa_assert(gpio_chargeStatIn);

	// save our references
	chrgIn->gpio_charging = gpio_chargingIn;
	chrgIn->gpio_chargeStat = gpio_chargeStatIn;

	// set our initial value
	chrgIn->lastState = ovr_chargingManager_getCurrentState(chrgIn);

	// setup our listener array
	cxa_array_initStd(&chrgIn->listeners, chrgIn->listeners_raw);

	// setup our stateMachine
	cxa_runLoop_addEntry(cb_onRunLoopUpdate, (void*)chrgIn);
}


bool ovr_chargingManager_addListener(ovr_chargingManager_t *const chrgIn, ovrChargingManager_cb_stateChange_t cbIn, void* userVarIn)
{
	cxa_assert(chrgIn);
	if( cbIn == NULL ) return false;

	ovr_chargingManager_listenerEntry_t newListener = {
			.cb_stateChange = cbIn,
			.userVar = userVarIn
	};
	return cxa_array_append(&chrgIn->listeners, &newListener);
}


ovr_chargingManager_chargeState_t ovr_chargingManager_getCurrentState(ovr_chargingManager_t *const chrgIn)
{
	cxa_assert(chrgIn);

	bool isCharging = cxa_gpio_getValue(chrgIn->gpio_charging);
	bool isCharged = cxa_gpio_getValue(chrgIn->gpio_chargeStat);

	return (isCharging && isCharged) ? OVR_CHARGESTATE_CHARGED :
			((isCharging && !isCharged) ? OVR_CHARGESTATE_CHARGING : OVR_CHARGESTATE_DISCHARGING);
}


const char* ovr_chargingManager_getCurrentState_string(ovr_chargingManager_t *const chrgIn)
{
	cxa_assert(chrgIn);

	const char* retVal = "<unknown>";

	ovr_chargingManager_chargeState_t currState = ovr_chargingManager_getCurrentState(chrgIn);
	switch( currState )
	{
		case OVR_CHARGESTATE_DISCHARGING:
			retVal = "discharging";
			break;

		case OVR_CHARGESTATE_CHARGING:
			retVal = "charging";
			break;

		case OVR_CHARGESTATE_CHARGED:
			retVal = "charged";
			break;
	}

	return retVal;
}


// ******** local function implementations ********
static void cb_onRunLoopUpdate(void* userVarIn)
{
	ovr_chargingManager_t* chrgIn = (ovr_chargingManager_t*)userVarIn;
	cxa_assert(chrgIn);

	// get our current state
	ovr_chargingManager_chargeState_t currState = ovr_chargingManager_getCurrentState(chrgIn);

	// compare with our previous state and notify listeners if needed
	if( currState != chrgIn->lastState )
	{
		cxa_array_iterate(&chrgIn->listeners, currListener, ovr_chargingManager_listenerEntry_t)
		{
			if( currListener == NULL ) continue;

			if( currListener->cb_stateChange != NULL ) currListener->cb_stateChange(chrgIn, currState, currListener->userVar);
		}

		chrgIn->lastState = currState;
	}
}
