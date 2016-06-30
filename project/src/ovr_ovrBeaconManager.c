/*
 * Author: Christopher Armenio
 */
#include "ovr_ovrBeaconManager.h"

#include <attributes.h>
#include <blestack/blestack.h>
#include <blestack/gap.h>
#include <blestack/hw.h>
#include <blestack/dma.h>
#include <blestack/ps.h>
#include <blestack/ll.h>

#include <string.h>
#include <stdbool.h>

#include <cxa_assert.h>
#include <cxa_fixedByteBuffer.h>
#include <cxa_protocolParser.h>
#include <cxa_stateMachine.h>

#define CXA_LOG_LEVEL			CXA_LOG_LEVEL_TRACE
#include <cxa_logger_implementation.h>


// ******** local macro definitions ********
#define ENUMERATE_SUPPORT_VISUAL 		1
#define ENUMERATE_SUPPORT_AUDITORY		0
#define ENUMERATE_SUPPORT_HAPTIC		0

#define COMPANY_ID						0x5C00
#define ADVERTISEMENT_VERSION			0
#define TXPOWER_1M						1
#define TXPOWER_5M						5

#define ADVWIN_MIN_MS					750
#define ADVWIN_MAX_MS					1250

#define FADE_PERIOD_MS					2000


// ******** local type definitions ********
typedef enum
{
	STATE_IDLE,
	STATE_READ_SENSE,
	STATE_ENUMERATE
}state_t;


typedef struct
{
	bool visual;
	bool auditory;
	bool haptic;
}enumerationType_t;


typedef struct
{
	uint8_t flag_len;
	uint8_t flag_dataType;
	uint8_t flags;

	uint8_t manData_len;
	uint8_t manData_type;
	uint16_t manData_companyId;
	uint8_t manData_advVersion;
	uint8_t manData_uuid[16];
	int8_t manData_txPower_1m;
	int8_t manData_txPower_5m;
	uint8_t manData_status;
	uint8_t manData_battery_pcnt;
	int8_t manData_temp_c;
}advertisementData_t;


// ******** local function prototypes ********
static void updateAdvertData(void);
static void stateCb_readSense_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void *userVarIn);
static void stateCb_readSense_state(cxa_stateMachine_t *const smIn, void *userVarIn);
static void stateCb_readSense_leave(cxa_stateMachine_t *const smIn, int nextStateIdIn, void* userVarIn);
static void stateCb_idle_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void *userVarIn);
static void stateCb_idle_state(cxa_stateMachine_t *const smIn, void *userVarIn);
static void stateCb_enumerate_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void *userVarIn);
static void stateCb_enumerate_state(cxa_stateMachine_t *const smIn, void *userVarIn);
static void stateCb_enumerate_leave(cxa_stateMachine_t *const smIn, int nextStateIdIn, void* userVarIn);


static void cb_battEst(cxa_batteryCapacityEstimator_t *const cbeIn, float battPcntIn, void* userVarIn);
static void cb_tempSense(cxa_tempSensor_t *const tmpSnsIn, float newTemp_degCIn, void* userVarIn);


// ********  local variable declarations *********
static cxa_logger_t logger;

static cxa_stateMachine_t stateMachine;
static bool isFirstBoot = true;

static cxa_timeDiff_t td_enumerate;
static enumerationType_t currEnumType = {.auditory = false, .haptic = false, .visual = false};
static uint32_t enumerationLen_ms = 0;

static cxa_ws2812String_t* ws2812;
static cxa_batteryCapacityEstimator_t* battEst;
static cxa_tempSensor_t* tempSense;

static uint8_t lastBatt_pcnt100;
static int8_t lastTemp_c;

static uint8_t advertData_raw[29];
static cxa_fixedByteBuffer_t advertData;


// ******** global function implementations ********
void ovr_ovrBeaconManager_init(cxa_ws2812String_t *const ws2812In, cxa_batteryCapacityEstimator_t *const bceIn, cxa_tempSensor_t *const tempSnsIn)
{
	cxa_assert(ws2812In);
	cxa_assert(bceIn);
	cxa_assert(tempSnsIn);

	// save our references
	ws2812 = ws2812In;
	battEst = bceIn;
	tempSense = tempSnsIn;

	// setup our logger
	cxa_logger_init(&logger, "beaconMan");

	// setup our timeDiff
	cxa_timeDiff_init(&td_enumerate, false);

	// initialize our advertisement data buffer
	cxa_fixedByteBuffer_initStd(&advertData, advertData_raw);

	// setup our state machine
	cxa_stateMachine_init(&stateMachine, "ovrBeacon");
	cxa_stateMachine_addState(&stateMachine, STATE_READ_SENSE, "readSense", stateCb_readSense_enter, stateCb_readSense_state, stateCb_readSense_leave, NULL);
	cxa_stateMachine_addState(&stateMachine, STATE_IDLE, "idle", stateCb_idle_enter, stateCb_idle_state, NULL, NULL);
	cxa_stateMachine_addState(&stateMachine, STATE_ENUMERATE, "enumerate", stateCb_enumerate_enter, stateCb_enumerate_state, stateCb_enumerate_leave, NULL);
	cxa_stateMachine_setInitialState(&stateMachine, STATE_READ_SENSE);

	// set our advertising parameters (7= all three adv channels)
	gap_set_adv_parameters( (uint16_t)((float)ADVWIN_MIN_MS/0.625), (uint16_t)((float)ADVWIN_MAX_MS/0.625), 7);
}


bool ovr_ovrBeaconManager_isEnumerating(void)
{
	return (cxa_stateMachine_getCurrentState(&stateMachine) == STATE_ENUMERATE);
}


void ovr_ovrBeaconManager_update(void)
{
	cxa_stateMachine_update(&stateMachine);
}


// ******** local function implementations ********
static void updateAdvertData(void)
{
	cxa_fixedByteBuffer_clear(&advertData);

	cxa_fixedByteBuffer_append_uint8(&advertData, 0x02);					// field length
	cxa_fixedByteBuffer_append_uint8(&advertData, 0x01);					// flag data type
	cxa_fixedByteBuffer_append_uint8(&advertData, 0x06);					// general discoverable, no BR/EDR

	cxa_fixedByteBuffer_append_uint8(&advertData, 0x19);					// field length
	cxa_fixedByteBuffer_append_uint8(&advertData, 0xff);					// manufacturer data data type
	cxa_fixedByteBuffer_append_uint16LE(&advertData, COMPANY_ID);
	cxa_fixedByteBuffer_append_uint8(&advertData, ADVERTISEMENT_VERSION);

	uint8_t tmpUuid[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
						  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
	cxa_fixedByteBuffer_append(&advertData, tmpUuid, sizeof(tmpUuid));

	cxa_fixedByteBuffer_append_uint8(&advertData, (uint8_t)TXPOWER_1M);
	cxa_fixedByteBuffer_append_uint8(&advertData, (uint8_t)TXPOWER_5M);
	cxa_fixedByteBuffer_append_uint8(&advertData, (ovr_ovrBeaconManager_isEnumerating() << 0));
	cxa_fixedByteBuffer_append_uint8(&advertData, lastBatt_pcnt100);
	cxa_fixedByteBuffer_append_uint8(&advertData, lastTemp_c);
}


static void stateCb_readSense_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void *userVarIn)
{
	cxa_logger_info(&logger, "reading sensors");

	cxa_assert(cxa_batteryCapacityEstimator_getValue_withCallback(battEst, cb_battEst, NULL));

	// make sure we keep running
	task_send_msg(task_id_first_user, 0, 0);
}


static void stateCb_readSense_state(cxa_stateMachine_t *const smIn, void *userVarIn)
{
	cxa_batteryCapacityEstimator_update(battEst);
	cxa_tempSensor_update(tempSense);

	// make sure we keep running
	task_send_msg(task_id_first_user, 0, 0);
}


static void stateCb_readSense_leave(cxa_stateMachine_t *const smIn, int nextStateIdIn, void* userVarIn)
{
	// make sure our LEDs are off
	cxa_ws2812String_blank_now(ws2812);

	// update our advertising data
	updateAdvertData();

	// start advertising if we weren't
	if( isFirstBoot )
	{
		gap_set_mode(gap_general_discoverable, gap_undirected_connectable);
		isFirstBoot = false;
	}
	ll_set_adv_data((uint8 const*)&advertData_raw, sizeof(advertData_raw));
}


static void stateCb_idle_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void *userVarIn)
{
	cxa_logger_info(&logger, "becoming idle");

	// schedule us to run once per average advertising window (units are 32kHz ticks)
	task_send_timed(task_id_first_user, 0, 0, ((((uint32_t)ADVWIN_MAX_MS + (uint32)ADVWIN_MIN_MS) / (uint32_t)2) * (uint32_t)32));
}


static void stateCb_idle_state(cxa_stateMachine_t *const smIn, void *userVarIn)
{
	// force the transition since we won't be scheduled again
	cxa_stateMachine_transition(&stateMachine, STATE_READ_SENSE);
	cxa_stateMachine_update(&stateMachine);
}


static void stateCb_enumerate_enter(cxa_stateMachine_t *const smIn, int prevStateIdIn, void *userVarIn)
{
	cxa_logger_info(&logger, "starting enumeration - a:%d h:%d v:%d for %d secs", currEnumType.auditory, currEnumType.haptic, currEnumType.visual, enumerationLen_ms/1000);
	cxa_timeDiff_setStartTime_now(&td_enumerate);

	cxa_ws2812String_pulseColor_rgb(ws2812, FADE_PERIOD_MS, 0, 0, 255);

	// make sure we keep running
	task_send_msg(task_id_first_user, 0, 0);
}


static void stateCb_enumerate_state(cxa_stateMachine_t *const smIn, void *userVarIn)
{
	cxa_ws2812String_update(ws2812);

	// see if our enumeration is finished
	if( (enumerationLen_ms > 0) && cxa_timeDiff_isElapsed_ms(&td_enumerate, enumerationLen_ms) )
	{
		// force the transition since we won't be scheduled again
		cxa_stateMachine_transition(&stateMachine, STATE_IDLE);
		cxa_stateMachine_update(&stateMachine);
		return;
	}

	// if we made it here, make sure we keep running
	task_send_msg(task_id_first_user, 0, 0);
}


static void stateCb_enumerate_leave(cxa_stateMachine_t *const smIn, int nextStateIdIn, void* userVarIn)
{
	cxa_ws2812String_blank_now(ws2812);
}


static void cb_battEst(cxa_batteryCapacityEstimator_t *const cbeIn, float battPcntIn, void* userVarIn)
{
	lastBatt_pcnt100 = (uint8_t)(battPcntIn * 100.0);
	cxa_logger_debug(&logger, "battPcnt: %d", lastBatt_pcnt100);

	// now get our temperature
	cxa_tempSensor_getValue_withCallback(tempSense, cb_tempSense, NULL);
}


static void cb_tempSense(cxa_tempSensor_t *const tmpSnsIn, float newTemp_degCIn, void* userVarIn)
{
	lastTemp_c = (int8_t)newTemp_degCIn;
	cxa_logger_debug(&logger, "temp_f: %d", (unsigned int)CXA_TEMPSENSE_CTOF(lastTemp_c));

	// go back to idle
	cxa_stateMachine_transition(&stateMachine, STATE_IDLE);
}


// ******** BlueGiga BLE Stack Callbacks ********
void connection_complete_callback(uint8 conn)
{
	cxa_logger_info(&logger, "connect complete");
}


void connection_disconnect_callback(uint8 conn, uint8 reason)
{
	cxa_logger_info(&logger, "disconnect");

	// start advertising again
	updateAdvertData();
	gap_set_mode(gap_general_discoverable, gap_undirected_connectable);
	ll_set_adv_data((const uint8_t*)&advertData, sizeof(advertData));
}


void gatt_user_read_request_callback(uint8 connection, uint16 handle, uint16 offset, uint8 maxlen)
{
	cxa_logger_debug(&logger, "read from handle %d", handle);

	if( handle == BG_ATT_enumerate_support )
	{
		uint8_t supportVal = (ENUMERATE_SUPPORT_HAPTIC << 2) | (ENUMERATE_SUPPORT_AUDITORY << 1) | (ENUMERATE_SUPPORT_VISUAL << 0);
		gatt_user_read_rsp(connection, att_error_none, &supportVal, sizeof(supportVal));
	}
}


void gatt_attribute_value_callback(uint8 connection, uint16 handle, enum attributes_attribute_change_reason cb_type, uint16 offset, const uint8*data, uint8 len)
{
	// only handle user write requests
	if( cb_type == attributes_attribute_change_reason_write_request_user )
	{
		cxa_logger_debug(&logger, "write %d bytes to handle %d", len, handle);

		// only handle writes we care about
		if( handle == BG_ATT_enumerate_request )
		{
			// enumerate request write

			// make sure we have the right length
			if( len != 2 )
			{
				cxa_logger_warn(&logger, "invalid enumerate request length");
				gatt_user_write_rsp(connection, att_error_invalid_att_length);
				return;
			}

			// first byte is the request mask (what kind of enumeration to perform)
			enumerationType_t newEnumType = {
					.visual = (data[0] & (1 << 0)),
					.auditory = (data[0] & (1 << 1)),
					.haptic = (data[0] & (1 << 2))
			};

			// next byte is how long to enumerate for
			uint8_t enumerationLen_s = data[1];

			// make sure we support it
			if( (newEnumType.visual && !ENUMERATE_SUPPORT_VISUAL) ||
				(newEnumType.auditory && !ENUMERATE_SUPPORT_AUDITORY) ||
				(newEnumType.haptic && !ENUMERATE_SUPPORT_HAPTIC) )
			{
				cxa_logger_warn(&logger, "requested non-supported enumeration (0x%X), aborting", data[0]);
				gatt_user_write_rsp(connection, att_error_application);
				return;
			}

			// if we made it here, we're good to start (or stop) enumeration
			if( data[0] == 0 )
			{
				// simple case...stopping enumeration
				cxa_stateMachine_transition(&stateMachine, STATE_IDLE);
			}
			else
			{
				// starting enumeration
				currEnumType = newEnumType;
				enumerationLen_ms = enumerationLen_s * 1000;

				// force the transition so we get scheduled to execute
				cxa_stateMachine_transition(&stateMachine, STATE_ENUMERATE);
				cxa_stateMachine_update(&stateMachine);
			}

			// good write!
			gatt_user_write_rsp(connection, att_error_none);
		}
		else gatt_user_write_rsp(connection, att_error_att_not_found);
	}
}


// ******** below are callbacks required by BLE stack but not used ********
void gatt_attribute_status_changed(uint16 handle, uint8 flags) { }
void gatt_read_attribute_callback(uint8 connection, uint16 handle, uint8 type, uint8*data, uint8 len) { }
void gatt_procedure_complete_callback(uint8 connection, uint16 handle, uint8 ecode) { }
void gatt_timeout_callback(uint8 connection) { }
void gatt_read_multiple_callback(uint8 connection, uint8*data, uint8 len) { }
void gatt_indication_callback(uint8 connection, uint8 ind) { }
uint8 gatt_find_grouping_callback(uint8 connection, uint16 start, uint16 end, uint8*uuid, uint8 uuidlen) { return 1; }
uint8 gatt_find_information_callback(uint8 connection, uint16 handle, uint8*uuid, uint8 uuidlen) { return 1; }

void smp_passkey_request_callback(uint8 connection) { }
void smp_passkey_display_callback(uint8 connection, uint32 passkey) { }
void smp_bonding_success(uint8 connection, uint8 bond_handle) { }
void smp_bonding_timeout(uint8 connection) { }
void smp_bonding_fail(uint8 connection, errcode reason) { }

void connection_version_callback(uint8 conn, uint8 versnr, uint16 compid, uint16 subversnro) { }
void connection_features_callback(uint8 conn, uint8* features) { }
void connection_encrypted_callback(uint8 conn, uint8 reason) { }
void connection_update_callback(uint8 conn) { }

void gap_scan_response_callback(int8 rssi, uint8 packet_type, uint8*addr, uint8 random_addr, uint8 bond_handle, uint8 len, uint8*data, uint8 channel) { }
void connection_evt_status_change(uint8 conn, uint8 evt_flags) { }


uint8 sleep_can_sleep_callback()
{
    return cxa_stateMachine_getCurrentState(&stateMachine) == STATE_IDLE;
}
