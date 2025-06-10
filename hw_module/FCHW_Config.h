#ifndef __FCHW_CONFIG_H
#define __FCHW_CONFIG_H

#include "util.h"

#define ON 1
#define OFF 0

// #if !defined HW_BATEAIO_AT32F435 && !defined HW_MATEK_STM32H743
// #define HW_MATEK_STM32H743
// #endif

#define IMU_CNT                 2
#define BARO_CNT                1
#define MAG_CNT                 0
#define SD_CARD_ENABLE_STATE    ON
#define SDRAM_ENABLE_STATE      OFF
#define FLASH_CHIP_ENABLE_STATE OFF
#define RADIO_NUM               1

#define Storage_InfoPageSize Flash_Storage_InfoPageSize

/* get virable from .ld file defined */
extern uint32_t __rom_s;
extern uint32_t __rom_e;
extern uint32_t __boot_s;
extern uint32_t __boot_e;

#define Boot_Address_Base   ((uint32_t)(&__boot_s))
#define Boot_Section_Size   ((uint32_t)&__boot_e - (uint32_t)&__boot_s)

#define App_Address_Base    ((uint32_t)&__boot_e)
#define App_Section_Size    ((uint32_t)&__rom_e - App_Address_Base)

#define IMU_SUM             IMU_CNT
#define BARO_SUM            BARO_CNT
#define MAG_SUM             MAG_CNT
#define SDRAM_EN            SDRAM_ENABLE_STATE
#define SD_CARD             SD_CARD_ENABLE_STATE
#define FLASH_CHIP_STATE    FLASH_CHIP_ENABLE_STATE
#define RADIO_UART_NUM      RADIO_NUM

#endif
