/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
//#include "app_log.h"
#include "custom_adv.h"
#include "sl_simple_timer.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "app.h"
#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_adc.h"
#include "em_gpio.h"
#include "em_letimer.h"

/*Board:  Silicon Labs EFR32BG13 Radio Board (SLWRB4104A) +
        Wireless Starter Kit Mainboard
Device: EFR32BG13P632F512GM48
PC9 - ADC0 Port 2X Channel 9 (Expansion Header Pin 10)
P7 nhaaaaaaaaaaaaaaaaaaaaaaaaaa

 */

// The advertising set handle allocated from Bluetooth stack.
//Define for IRQ from fire notify

//Cac thong so setup

#define TIME_RESCAN_NORMAL 3*1000
#define TIME_RE_ADV_NORMAL 3*1600





//
#define BSP_GPIO_LEDS
#define BSP_NO_OF_LEDS            2
#define BSP_GPIO_LED0_PORT        gpioPortF
#define BSP_GPIO_LED0_PIN         4
#define BSP_GPIO_LED1_PORT        gpioPortF
#define BSP_GPIO_LED1_PIN         5
#define BSP_GPIO_LEDARRAY_INIT    { { BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN}, { BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN } }

#define BSP_GPIO_BUTTONS
#define BSP_NO_OF_BUTTONS         2
#define BSP_GPIO_PB0_PORT         gpioPortF
#define BSP_GPIO_PB0_PIN          6
#define BSP_GPIO_PB1_PORT         gpioPortF
#define BSP_GPIO_PB1_PIN          7


#define TIME_RESCAN_NORMAL 2*1000
#define TIME_RE_ADV_NORMAL 2*1600



#define TIME_RESCAN_ALERT 5*100

#define TIME_RE_ADV_ALERT 5*160

#define TIME_RE_ADV_100ms 1*160
#define TIME_RE_ADV_1s 1*1600
#define TIME_RE_ADV_5s 5*1600

#define TIME_RESCAN_100ms 1*100
#define TIME_RESCAN_1s 1*1000
#define TIME_RESCAN_5s 5*1000

//Variable ADV
CustomAdv_t sData; // Our custom advertising data stored here
static uint32_t humidity = 0;
static int32_t temperature = 0;
static int32_t smoke = 0;

static int32_t flag_smoke = 0;
static int32_t flag_fire = 0;

static sl_simple_timer_t sensor_timer;
// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;



//For Read ADC
#define adcFreq   9600
#define REPEAT_CYCLE_WARNING   6
#define REPEAT_CYCLE_ALERT     10
//It is the cycle re ADV after have warning or alert about smoke

#define WARNING_THRESHOLD_LOW 1250
#define ALERT_THRESHOLD 1500

#define FLAG_NORMAL   0
#define FLAG_WARNING   1
#define FLAG_ALERT   2
volatile uint32_t warning_count = 0;

volatile uint32_t repeat_cycle = 0;
volatile uint32_t sample;
volatile uint32_t smoke_data;
volatile uint32_t millivolts;

static uint32_t time_reADV  = TIME_RE_ADV_NORMAL;
static int32_t time_rescan = TIME_RESCAN_NORMAL;
//Define for Packet
#define ID_DEVICE_1   0x01
#define COMPANY_ID 0x02FF
#define FLAG_PACKET_LENGTH   0x02
#define FLAG_PACKET_TYPE   0x01
#define FLAG_PACKET_VALUE   0x06
/**************************************************************************//**
 * @brief  Initialize ADC function
 *****************************************************************************/
void initADC (void)
{
  // Enable ADC0 clock
  CMU_ClockEnable(cmuClock_ADC0, true);

  // Declare init structs
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

  // Modify init structs and initialize
  init.prescale = ADC_PrescaleCalc(adcFreq, 0); // Init to max ADC clock for Series 1
  init.timebase = ADC_TimebaseCalc(0);

  initSingle.diff       = false;        // single ended
  initSingle.reference  = adcRef2V5;    // internal 2.5V reference
  initSingle.resolution = adcRes12Bit;  // 12-bit resolution
  initSingle.acqTime    = adcAcqTime8;  // set acquisition time to meet minimum requirements

  // Select ADC input. See README for corresponding EXP header pin.
  initSingle.posSel = adcPosSelAPORT2XCH9;

  ADC_Init(ADC0, &init);
  ADC_InitSingle(ADC0, &initSingle);

  // Enable ADC Single Conversion Complete interrupt
  ADC_IntEnable(ADC0, ADC_IEN_SINGLE);


  // Enable ADC interrupts
  //  NVIC_ClearPendingIRQ(ADC0_IRQn);
  //  NVIC_EnableIRQ(ADC0_IRQn);
}

/**************************************************************************//**
 * @brief  ADC Handler
 *****************************************************************************/
//void ADC0_IRQHandler(void)
//{
//  // Get ADC result
//  sample = ADC_DataSingleGet(ADC0);
//
//  // Calculate input voltage in mV
//  millivolts = (sample * 2500) / 4096;
//  app_log("millivolts: %d  ",millivolts ); app_log("sample: %d \n",sample );
//  // Start next ADC conversion
//  ADC_Start(ADC0, adcStartSingle);
//}



/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
static void sensor_timer_cb(sl_simple_timer_t *timer, void *data)
{
  (void)data;
  (void)timer;
  // send temperature measurement indication to connected client
  sl_sensor_rht_get(&humidity, &temperature);

//  app_log("\n-----------Data------------\n");
//  app_log("Advertising Interval: %d ms \n", (time_reADV / 160) *100);
//  app_log("Scan Interval: %d ms \n", time_rescan);
//  app_log("humidity: %lu - temperature: %ld\r\n", humidity/1000, temperature/1000);
  // Start next ADC conversion
  ADC_Start(ADC0, adcStartSingle);
  // Get ADC result
  sample = ADC_DataSingleGet(ADC0);
  smoke_data = sample;
  // Calculate input voltage in mV
  millivolts = (sample * 5000) / 4096;

  //




  smoke = smoke_data;
  if(millivolts > WARNING_THRESHOLD_LOW)
    {
      flag_smoke = FLAG_WARNING;
      repeat_cycle = REPEAT_CYCLE_WARNING;
    }
  if(millivolts > ALERT_THRESHOLD)
    {
      flag_smoke = FLAG_ALERT;
      repeat_cycle = REPEAT_CYCLE_ALERT;
    }

  if(repeat_cycle > 0)
    {
      repeat_cycle --;
      //After smoke data is not get over THRESHOLD 50 second will be turn of flag smoke.
    }
  else
    {
      flag_smoke = FLAG_NORMAL;
      flag_fire = FLAG_NORMAL;

      time_reADV = TIME_RE_ADV_NORMAL;
      time_rescan = TIME_RESCAN_NORMAL;
      //app_log("Reset all flag and remake time   \n");
      //app_log("-----------Reset all flag------------\n");
    }

  if (flag_fire == FLAG_ALERT)
    {
      time_rescan = TIME_RESCAN_ALERT;
      time_reADV = TIME_RE_ADV_ALERT;
    }

  if (flag_smoke == FLAG_ALERT)
    {
      time_rescan = TIME_RESCAN_ALERT;
      time_reADV = TIME_RE_ADV_ALERT;
    }

  if((temperature/1000) > 50)
    {
    flag_fire = FLAG_WARNING;
    warning_count++;
    if(warning_count > 10)
      warning_count=FLAG_ALERT;
    }else
      warning_count = 0;

//  app_log("Data for smoke   ");
//  app_log("smoke_data: %d  \n",smoke_data );
//  app_log("Voltage of smoke sensor : %d mV \n",millivolts );
//  app_log("Flag Smoke : %d  \n",flag_smoke );
//
//  app_log("Flag Fire  : %d  \n",flag_fire );

  update_adv_data(&sData, advertising_set_handle, humidity/1000, temperature/1000, smoke,flag_smoke ,flag_fire);
  //uint8_t flag_fire =  GPIO_PinModeGet(gpioPortA,3);





  if(flag_fire == FLAG_ALERT || flag_smoke == FLAG_ALERT)
    {
      sl_status_t sc;
  sc = sl_bt_advertiser_set_timing(
      advertising_set_handle,
      time_reADV, // min. adv. interval (milliseconds * 1.6)
      time_reADV, // max. adv. interval (milliseconds * 1.6)
      800,   // adv. duration 0,5s x 1.6
      0);  // max. num. adv. events
  app_assert_status(sc);
  sc = sl_simple_timer_start(&sensor_timer,
                             time_rescan,
                             sensor_timer_cb,
                             NULL,
                             true);
  app_assert_status(sc);
    }
  //Update data about smoke here
}


/**************************************************************************//**
 * Application Init for IRQ of GPIO use
 * UIF_BUTTON0 / PF6 / P34
 * UIF_BUTTON1 / PF7 / P36
 * It 2 port of 2 button use that for interupt
 *****************************************************************************/
void init_GPIO(){
  // Enable GPIO clock
  CMU_ClockEnable(cmuClock_GPIO, true);
  // Configure PB0 and PB1 as input with glitch filter enabled
  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN,
                  gpioModeInputPullFilter, 1);
  GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN,
                  gpioModeInputPullFilter, 1);
  // Configure LED0 and LED1 as output
  GPIO_PinModeSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN, gpioModePushPull,
                  0);
  GPIO_PinModeSet(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN, gpioModePushPull,
                  0);
  // Enable IRQ for even numbered GPIO pins
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  // Enable IRQ for odd numbered GPIO pins
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
  // Enable falling-edge interrupts for PB pins
  GPIO_ExtIntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN,BSP_GPIO_PB0_PIN,
                    1, 0, true);
  GPIO_ExtIntConfig(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, BSP_GPIO_PB1_PIN,
                    1, 0, true);
}
void GPIO_EVEN_IRQHandler(void)
{
  // Clear all even pin interrupt flags
  GPIO_IntClear(0x5555);
  // Toggle LED0
  GPIO_PinOutToggle(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
  flag_fire = FLAG_NORMAL;




}
void GPIO_ODD_IRQHandler(void)
{
  // Clear all odd pin interrupt flags
  GPIO_IntClear(0xAAAA);
  // Toggle LED01
  GPIO_PinOutToggle(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
  flag_fire = FLAG_ALERT;
  //app_log("--------------Fire event-----------    \n");
  repeat_cycle = REPEAT_CYCLE_ALERT;

}


SL_WEAK void app_init(void)
{
  sl_status_t sc;
  // Init temperature sensor.
  sc = sl_sensor_rht_init();
  app_assert_status(sc);
  //Chu ky doc cam bien
  sc = sl_simple_timer_start(&sensor_timer,
                             time_rescan,
                             sensor_timer_cb,
                             NULL,
                             true);
  app_assert_status(sc);
}


/**************************************************************************//**
 * Application Init for ADC use
 * P7 / PC9 / EXP10
 * Use that for Read Analog data from Smoke Sensor to warning data
 *****************************************************************************/


/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      app_assert_status(sc);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      app_assert_status(sc);

      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);


      // Set time
      // Set time advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
          advertising_set_handle,
          time_reADV, // min. adv. interval (milliseconds * 1.6)
          time_reADV, // max. adv. interval (milliseconds * 1.6)
          800,   // adv. duration
          0);  // max. num. adv. events
      app_assert_status(sc);



      //Set power
      /***************************************************************************//**
       *
       * Limit the maximum advertising TX power on an advertising set. If the value
       * goes over the global value that was set using the @ref
       * sl_bt_system_set_tx_power command, the global value will be the maximum
       * limit. The maximum TX power of legacy advertising is further constrained to
       * be less than +10 dBm. Extended advertising TX power can be +10 dBm and over
       * if Adaptive Frequency Hopping is enabled. This setting has no effect on
       * periodic advertising.
       *
       * This setting will take effect next time the legacy or extended advertising is
       * enabled.
       *
       * By default, maximum advertising TX power is limited by the global value.
       *
       * @param[in] advertising_set Advertising set handle
       * @param[in] power TX power in 0.1 dBm steps. For example, the value of 10 is 1
       *   dBm and 55 is 5.5 dBm.
       * @param[out] set_power The selected maximum advertising TX power
       * Transmit output power is set to N dBm (6dBm default)
       * @return SL_STATUS_OK if successful. Error code otherwise.
       *
       ******************************************************************************/
      uint8_t power_transmit = 30;
            sc = sl_bt_advertiser_set_tx_power(advertising_set_handle, power_transmit, NULL);
            app_assert_status(sc);

//            app_log("\n ------------Start-------------\n");
//
//      app_log("Data for set all of channel \n");
//      app_log("Power advertise at %d,%d dBm \n",power_transmit/10,power_transmit%10);
      //Set channel
      /***************************************************************************//**
       *
       * Set the primary advertising channel map on an advertising set. This setting
       * will take effect next time when the legacy or extended advertising is
       * enabled.
       *
       * @param[in] advertising_set Advertising set handle
       * @param[in] channel_map @parblock
       *   Advertising channel map which determines which of the three channels will
       *   be used for advertising. This value is given as a bitmask. Values:
       *     - <b>1:</b> Advertise on CH37
       *     - <b>2:</b> Advertise on CH38
       *     - <b>3:</b> Advertise on CH37 and CH38
       *     - <b>4:</b> Advertise on CH39
       *     - <b>5:</b> Advertise on CH37 and CH39
       *     - <b>6:</b> Advertise on CH38 and CH39
       *     - <b>7:</b> Advertise on all channels
       *
       *   Recommended value: 7
       *
       *   Default value: 7
       *   @endparblock
       *
       * @return SL_STATUS_OK if successful. Error code otherwise.
       *
       ******************************************************************************/
      sl_bt_advertiser_set_channel_map(advertising_set_handle, 7);



      fill_adv_packet(&sData, FLAG_PACKET_VALUE, COMPANY_ID, ID_DEVICE_1, humidity, temperature, smoke, flag_smoke, flag_fire, "Device-1");
      start_adv(&sData, advertising_set_handle);
      app_assert_status(sc);
     // app_log("Started advertising\r\n");

      break;

      // -------------------------------
      // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      break;

      // -------------------------------
      // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Restart advertising after client has disconnected.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);
      break;

      ///////////////////////////////////////////////////////////////////////////
      // Add additional event handlers here as your application requires!      //
      ///////////////////////////////////////////////////////////////////////////

      // -------------------------------
      // Default event handler.
    default:
      break;
  }
}
