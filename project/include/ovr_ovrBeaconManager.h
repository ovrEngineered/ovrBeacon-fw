/*
 * Author: Christopher Armenio
 */
#ifndef OVRBEACON_BTLEMANAGER_H_
#define OVRBEACON_BTLEMANAGER_H_


// ******** includes ********
#include <stdbool.h>
#include <cxa_batteryCapacityEstimator.h>
#include <cxa_gpio.h>
#include <cxa_tempSensor.h>
#include <cxa_ws2812String.h>


// ******** global macro definitions ********


// ******** global type definitions *********


// ******** global function prototypes ********
void ovr_ovrBeaconManager_init(cxa_ws2812String_t *const ws2812In, cxa_batteryCapacityEstimator_t *const bceIn, cxa_tempSensor_t *const tempSnsIn,
							   cxa_gpio_t *const gpio_chargingIn, cxa_gpio_t *const gpio_chargingStatIn);
bool ovr_ovrBeaconManager_isEnumerating(void);


#endif /* OVRBEACON_BTLEMANAGER_H_ */
