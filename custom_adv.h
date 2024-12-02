#ifndef _CUSTOM_ADV_H_
#define _CUSTOM_ADV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "sl_bt_api.h"
#include "app_assert.h"

#define NAME_MAX_LENGTH 14

typedef struct
{
  uint8_t len_flags;
  uint8_t type_flags;
  uint8_t val_flags;

  uint8_t len_manuf;
  uint8_t type_manuf;
  // First two bytes must contain the manufacturer ID (little-endian order)
  uint8_t company_LO;
  uint8_t company_HI;


  uint8_t Header_1;
  uint8_t Header_2;

  uint8_t id_adv_device;
  // The next bytes are freely configurable - using one byte for counter value and one byte for last button press
  uint8_t humidity;


  uint8_t temperature;


  uint8_t smoke_1;
  uint8_t smoke_2;

  uint8_t flag_smoke;
  uint8_t flag_fire;


  uint8_t Tailer_1;
  uint8_t Tailer_2;
  // length of the name AD element is variable, adding it last to keep things simple
  uint8_t len_name;
  uint8_t type_name;

  // NAME_MAX_LENGTH must be sized so that total length of data does not exceed 31 bytes
  char name[NAME_MAX_LENGTH];

  // These values are NOT included in the actual advertising payload, just for bookkeeping
  char dummy;        // Space for null terminator
  uint8_t data_size; // Actual length of advertising data
} CustomAdv_t;

void fill_adv_packet(CustomAdv_t *pData, uint8_t flags, uint16_t companyID,uint32_t id_adv_device, uint32_t humidity, int32_t temperature, int32_t smoke, int32_t flag_smoke, int32_t flag_fire, char *name);
void start_adv(CustomAdv_t *pData, uint8_t advertising_set_handle);
void update_adv_data(CustomAdv_t *pData, uint8_t advertising_set_handle, uint32_t humidity, int32_t temperature ,int32_t smoke,int32_t flag_smoke, int32_t flag_fire);

#ifdef __cplusplus
}
#endif

#endif
