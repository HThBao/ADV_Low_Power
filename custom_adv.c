/***************************************************************************//**
 * @file custom_adv.c
 * @brief customize advertising
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#define FLAG_PACKET_LENGTH   0x02
#define FLAG_PACKET_TYPE   0x01

#define TYPE_MANUAL   0xFF
#define LENGTH_MANUAL   14

#define HEADER1   0xAB
#define HEADER2   0xCD

#define TAILER1   0xAA
#define TAILER2   0xFF

#define LENGTH_MANUAL   14

#define FLAG_PACKET_TYPE   0x01

#include <string.h>
#include "custom_adv.h"
#include "stdio.h"

void fill_adv_packet(CustomAdv_t *pData, uint8_t flag_packet_value, uint16_t companyID, uint32_t id_adv_device, uint32_t humidity, int32_t temperature, int32_t smoke,int32_t flag_smoke, int32_t flag_fire, char *name)
{
  int n;

  pData->len_flags = FLAG_PACKET_LENGTH;
  pData->type_flags = FLAG_PACKET_TYPE;
  pData->val_flags = flag_packet_value;

  pData->len_manuf = LENGTH_MANUAL;
  // 1 byte maual + 2 bytes Company + 12 bytes data + 4 bytes Header and Tailer

  pData->type_manuf = 0xFF;
  pData->company_LO = companyID & 0xFF;
  pData->company_HI = (companyID >> 8) & 0xFF;


  pData->Header_1 = HEADER1 &  0xFF;
  pData->Header_2 = HEADER2 &  0xFF;

  //ID
  pData->id_adv_device = id_adv_device & 0xFF;
  //10
  pData->humidity = humidity & 0xFF;


  pData->temperature = temperature & 0xFF;


  pData->smoke_1 = (smoke >>8) & 0xFF;
  pData->smoke_2 = smoke & 0xFF;

  pData-> flag_smoke = flag_smoke & 0xFF;
  pData-> flag_fire = flag_fire & 0xFF;


  pData->Tailer_1 = TAILER1 &  0xFF;
  pData->Tailer_2 = TAILER2 &  0xFF;
  // Name length, excluding null terminator
  n = strlen(name);
  if (n > NAME_MAX_LENGTH) {
    // Incomplete name
    pData->type_name = 0x08;
  } else {
    pData->type_name = 0x09;
  }

  strncpy(pData->name, name, NAME_MAX_LENGTH);

  if (n > NAME_MAX_LENGTH) {
    n = NAME_MAX_LENGTH;
  }

  pData->len_name = 1 + n; // length of name element is the name string length + 1 for the AD type

  // Calculate total length of advertising data
  pData->data_size = 3 + (1 + pData->len_manuf) + (1 + pData->len_name);
}

void start_adv(CustomAdv_t *pData, uint8_t advertising_set_handle)
{
  sl_status_t sc;
  // Set custom advertising payload
  //app_log("Fail start_adv\n");
  sc = sl_bt_legacy_advertiser_set_data(advertising_set_handle, 0, pData->data_size, (const uint8_t *)pData);
  app_assert(sc == SL_STATUS_OK,
                "[E: 0x%04x] Failed to set advertising data\n",
                (int)sc);
  // Start advertising using custom data 
  sc = sl_bt_legacy_advertiser_start(advertising_set_handle, sl_bt_legacy_advertiser_connectable);
  app_assert(sc == SL_STATUS_OK,
                  "[E: 0x%04x] Failed to start advertising\n",
                  (int)sc);
}

void update_adv_data(CustomAdv_t *pData, uint8_t advertising_set_handle, uint32_t humidity, int32_t temperature, int32_t smoke, int32_t flag_smoke,int32_t flag_fire)
{
  sl_status_t sc;
  // Update the two variable fields in the custom advertising packet 

  //pData->humidity_1 = humidity;
  pData->humidity = humidity & 0xFF;


  pData->temperature = temperature & 0xFF;


  pData->smoke_1 = smoke & 0xFF;
  pData->smoke_2 = (smoke >> 8) & 0xFF;

  pData-> flag_smoke = flag_smoke & 0xF;
  pData-> flag_fire = flag_fire & 0xF;

  // Set custom advertising payload 
  sc = sl_bt_legacy_advertiser_set_data(advertising_set_handle, 0, pData->data_size, (const uint8_t *)pData);
  app_assert(sc == SL_STATUS_OK,
                  "[E: 0x%04x] Failed to set advertising data\n",
                  (int)sc);
}
