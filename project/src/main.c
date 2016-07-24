/*
 * indego_firmwareEmulator.c
 *
 * Created: 9/11/2015 10:09:07 AM
 * Author: Christopher Armenio
 */
#include <blestack/blestack.h>
#include <blestack/gap.h>
#include <blestack/hw.h>

#include <intrinsics.h>
#include <string.h>

#include <cxa_assert.h>
#include <cxa_batteryCapacityEstimator.h>
#include <cxa_ble112_adcChannel.h>
#include <cxa_ble112_dmaController.h>
#include <cxa_ble112_clockController.h>
#include <cxa_ble112_gpio.h>
#include <cxa_ble112_timeBase.h>
#include <cxa_ble112_usart.h>
#include <cxa_ble112_ws2812String.h>
#include <cxa_delay.h>
#include <cxa_led_gpio.h>
#include <cxa_stringUtils.h>
#include <cxa_runLoop.h>
#include <cxa_tempSensor_adc.h>
#include <cxa_timeDiff.h>

#include <ovr_ovrBeaconManager.h>


#define CXA_LOG_LEVEL		CXA_LOG_LEVEL_TRACE
#include <cxa_logger_implementation.h>


// ******** local macro definitions ********
#define NUM_PIXELS			1


// ******** local function prototypes ********
static void sysInit(void);
static void backgroundUserTask(uint8_t context, uint8_t message);


// ******** local variable declarations ********
static cxa_timeDiff_t td_genPurp;
cxa_ble112_gpio_t led_error;
static cxa_ble112_usart_t usart_debug;

static cxa_ble112_gpio_t gpio_charging;
static cxa_ble112_gpio_t gpio_chargingStat;

static cxa_ble112_adcChannel_t adc_temp;
static cxa_tempSensor_adc_t tempSense;

static cxa_ble112_adcChannel_t adc_batt;
static cxa_batteryCapacityEstimator_t battEst;

static cxa_ble112_gpio_t gpio_ws2812;
static cxa_ble112_ws2812String_t neoPixels;
static cxa_ws2812String_pixelBuffer_t pixs[NUM_PIXELS];


const task_handler tasks[] = {
    ll_update,
    connection_update,
    smp_update,
    gap_update,
	backgroundUserTask
};


// ******** global function implementations ********
static void onInt(cxa_gpio_t *const gpioIn, cxa_gpio_interruptType_t intTypeIn, bool newValIn, void* userVarIn)
{
	cxa_gpio_toggle(&led_error.super);
}


void main(void)
{
	sysInit();

    
    ////////////////////////////////////////////////////////////////
    // TODO: MAKE SURE YOU UPDATE THE CONTENT OF THE <address> TAG
    // AND THE <license> TAG FOUND IN THE "/config/license.xml" FILE
    // THAT COMES WITH THIS TEMPLATE.
    //
    // Use the Bluegiga BLE SW Update application and the "Info"
    // button to get the target module's current MAC address and
    // 64-character license key, and copy them into the two tags in
    // this file prior to flashing any firmware built with IAR onto
    // the module; otherwise, you may erase the license key and
    // render the BLE radio inoperable until you obtain a new
    // replacement from Bluegiga support.
    //
    // The project will generate a Pre-build action compiler error
    // if you have not supplied a valid license.
    //
    // Also, if you switch out the target module during development
    // and use a new module, ensure that you update the MAC address
    // and license key at the same time. Each module requires its
    // own MAC address and license key.
    ////////////////////////////////////////////////////////////////


	static cxa_ble112_gpio_t tmpGpio;
	cxa_ble112_gpio_init_input(&tmpGpio, CXA_BLE112_GPIO_PORT_0, 3, CXA_GPIO_POLARITY_NONINVERTED);
	cxa_assert(cxa_gpio_enableInterrupt(&tmpGpio.super, CXA_GPIO_INTERRUPTTYPE_RISING_EDGE, onInt, NULL));

	// and start the main stack loop
    blestack_loop();
}


// ******** local function implementations ********
static void sysInit()
{
	// make sure we at least have a GPIO for asserts
	cxa_ble112_gpio_init_output(&led_error, CXA_BLE112_GPIO_PORT_1, 0, CXA_GPIO_POLARITY_NONINVERTED, 0);
	cxa_assert_setAssertGpio(&led_error.super);

	// should setup most of our hardware as needed
	blestack_config_init();

	// now setup our debug serial console
	cxa_ble112_usart_init_noHH(&usart_debug, CXA_BLE112_USART_1, CXA_BLE112_USART_PINCONFIG_ALT1, CXA_BLE112_USART_BAUD_115200);
	cxa_ioStream_t* ios_debug = cxa_usart_getIoStream(&usart_debug.super);
	cxa_assert_setIoStream(ios_debug);
	cxa_logger_setGlobalIoStream(ios_debug);
	cxa_ioStream_writeString(ios_debug, CXA_LINE_ENDING "<boot>" CXA_LINE_ENDING);

	// setup our timing-related systems
	cxa_ble112_timeBase_init();
	cxa_timeDiff_init(&td_genPurp, true);

	// give our system a chance to settle
	cxa_delay_ms(1000);

    // now setup our application-specific components
    blestack_init();

    cxa_ble112_gpio_init_input(&gpio_chargingStat, CXA_BLE112_GPIO_PORT_1, 1, CXA_GPIO_POLARITY_NONINVERTED);
    cxa_ble112_gpio_init_input(&gpio_charging, CXA_BLE112_GPIO_PORT_1, 4, CXA_GPIO_POLARITY_NONINVERTED);

    cxa_ble112_adcChannel_init_internalRef(&adc_batt, CXA_BLE112_ADC_CHAN_AVDD_DIV3);
    cxa_ble112_adcChannel_init_internalRef(&adc_batt, CXA_BLE112_ADC_CHAN_AIN6);
    cxa_batteryCapacityEstimator_init(&battEst, &adc_batt.super, 3.0, 3.3, 2.0);

    cxa_ble112_adcChannel_init_internalRef(&adc_temp, CXA_BLE112_ADC_CHAN_INTTEMP);
    cxa_tempSensor_adc_init_onePoint(&tempSense, &adc_temp.super, 25.0, 0.448);

    cxa_ble112_gpio_init_output(&gpio_ws2812, CXA_BLE112_GPIO_PORT_0, 0, CXA_GPIO_POLARITY_NONINVERTED, 0);
	cxa_ble112_ws2812String_init(&neoPixels, &gpio_ws2812, pixs, sizeof(pixs)/sizeof(*pixs));

    ovr_ovrBeaconManager_init(&neoPixels.super, &battEst, &tempSense.super, &gpio_charging.super, &gpio_chargingStat.super);
}


static void backgroundUserTask(uint8_t context, uint8_t message)
{
	cxa_runLoop_iterate();
}


void cxa_criticalSection_enter(void)
{
}


void cxa_criticalSection_exit(void)
{
}
