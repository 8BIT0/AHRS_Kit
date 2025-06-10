#ifndef __HW_DEF_H
#define __HW_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Bsp_GPIO.h"
#include "Bsp_IIC.h"
#include "Bsp_DMA.h"
#include "Bsp_SPI.h"
#include "Bsp_Uart.h"
#include "debug_util.h"
#include "Dev_Led.h"
#include "../../../../FCHW_Config.h"
#include "../../common/gen_physic_def/imu_data.h"

extern const uint8_t HWVer[3];

#define LED1_PIN GPIO_PIN_3
#define LED1_PORT GPIOE

#define LED2_PIN GPIO_PIN_4
#define LED2_PORT GPIOE

#define LED3_PIN GPIO_PIN_7
#define LED3_PORT GPIOH

#define Baro_SPI_BUS NULL

/* SPI3 Reserve SPI */
#define RESERVE_SPI_CLK_PORT GPIOB
#define RESERVE_SPI_CLK_PIN GPIO_PIN_3

#define RESERVE_SPI_MISO_PORT GPIOB
#define RESERVE_SPI_MISO_PIN GPIO_PIN_4

#define RESERVE_SPI_MOSI_PORT GPIOB
#define RESERVE_SPI_MOSI_PIN GPIO_PIN_5

/* MPU6000 Pin */
#define PriIMU_SPI_BUS SPI1

#define PriIMU_CS_PORT GPIOC
#define PriIMU_CS_PIN GPIO_PIN_15

#define PriIMU_INT_PORT GPIOB
#define PriIMU_INT_PIN GPIO_PIN_2

#define PriIMU_CLK_PORT GPIOA
#define PriIMU_CLK_PIN GPIO_PIN_5

#define PriIMU_MISO_PORT GPIOA
#define PriIMU_MISO_PIN GPIO_PIN_6

#define PriIMU_MOSI_PORT GPIOD
#define PriIMU_MOSI_PIN GPIO_PIN_7

/* ICM20602 Pin */
#define SecIMU_SPI_BUS SPI4

#define SecIMU_CS_PORT GPIOC
#define SecIMU_CS_PIN GPIO_PIN_13

#define SecIMU_INT_PORT GPIOC
#define SecIMU_INT_PIN GPIO_PIN_14

#define SecIMU_CLK_PORT GPIOE
#define SecIMU_CLK_PIN GPIO_PIN_12

#define SecIMU_MISO_PORT GPIOE
#define SecIMU_MISO_PIN GPIO_PIN_13

#define SecIMU_MOSI_PORT GPIOE
#define SecIMU_MOSI_PIN GPIO_PIN_14

/* Baro IIC Pin */
#define BARO_BUS  BspIIC_Instance_I2C_2
#define IIC2_SDA_PORT GPIOB
#define IIC2_SDA_PIN GPIO_PIN_11
#define IIC2_SCK_PORT GPIOB
#define IIC2_SCK_PIN GPIO_PIN_10

extern BspIICObj_TypeDef Baro_BusCfg;

/* Serial Pin */
#define UART4_TX_PORT GPIOB
#define UART4_TX_PIN GPIO_PIN_9
#define UART4_RX_PORT GPIOB
#define UART4_RX_PIN GPIO_PIN_8

#define UART1_TX_PORT GPIOA
#define UART1_TX_PIN GPIO_PIN_9
#define UART1_RX_PORT GPIOA
#define UART1_RX_PIN GPIO_PIN_10
/* USB Detected Pin */
#define USB_DETECT_INT_PORT GPIOE
#define USB_DETECT_INT_PIN GPIO_PIN_2

#define RECEIVER_PORT UART4
#define RECEIVER_CRSF_RX_DMA Bsp_DMA_None               // Bsp_DMA_1
#define RECEIVER_CRSF_RX_DMA_STREAM Bsp_DMA_Stream_None // Bsp_DMA_Stream_4
#define RECEIVER_CRSF_TX_DMA Bsp_DMA_None               // Bsp_DMA_1
#define RECEIVER_CRSF_TX_DMA_STREAM Bsp_DMA_Stream_None // Bsp_DMA_Stream_5

#define RECEIVER_SBUS_RX_DMA Bsp_DMA_1
#define RECEIVER_SBUS_RX_DMA_STREAM Bsp_DMA_Stream_4
#define RECEIVER_SBUS_TX_DMA Bsp_DMA_1
#define RECEIVER_SBUS_TX_DMA_STREAM Bsp_DMA_Stream_5

#define CRSF_TX_PIN Uart4_TxPin
#define CRSF_RX_PIN Uart4_RxPin

#define SBUS_TX_PIN Uart4_TxPin
#define SBUS_RX_PIN Uart4_RxPin

#define CRSF_PIN_SWAP false

/* radio uart */
#define RADIO_PORT USART1

#define RADIO_TX_PIN UART1_TX_PIN
#define RADIO_RX_PIN UART1_RX_PIN

#define RADIO_TX_PIN_INIT_STATE false
#define RADIO_RX_PIN_INIT_STATE false

#define RADIO_TX_PIN_ALT GPIO_AF7_USART1
#define RADIO_RX_PIN_ALT GPIO_AF7_USART1

#define RADIO_TX_PORT UART1_TX_PORT
#define RADIO_RX_PORT UART1_RX_PORT

#define RADIO_TX_DMA Bsp_DMA_2
#define RADIO_TX_DMA_STREAM Bsp_DMA_Stream_0
#define RADIO_RX_DMA Bsp_DMA_2
#define RADIO_RX_DMA_STREAM Bsp_DMA_Stream_1

extern DebugPinObj_TypeDef Debug_PC0;
extern DebugPinObj_TypeDef Debug_PC1;
extern DebugPinObj_TypeDef Debug_PC2;
extern DebugPinObj_TypeDef Debug_PC3;
extern DebugPinObj_TypeDef Debug_PB3;
extern DebugPinObj_TypeDef Debug_PB4;
extern DebugPinObj_TypeDef Debug_PB5;
extern DebugPinObj_TypeDef Debug_PB6;
extern DebugPinObj_TypeDef Debug_PB10;

extern DevLedObj_TypeDef Led1;
extern DevLedObj_TypeDef Led2;
extern DevLedObj_TypeDef Led3;

extern BspGPIO_Obj_TypeDef USB_DctPin;
extern BspGPIO_Obj_TypeDef PriIMU_CSPin;
extern BspGPIO_Obj_TypeDef SecIMU_CSPin;
extern BspGPIO_Obj_TypeDef PriIMU_INTPin;
extern BspGPIO_Obj_TypeDef SecIMU_INTPin;
extern BspGPIO_Obj_TypeDef Uart4_TxPin;
extern BspGPIO_Obj_TypeDef Uart4_RxPin;
extern BspGPIO_Obj_TypeDef Uart1_TxPin;
extern BspGPIO_Obj_TypeDef Uart1_RxPin;

extern BspSPI_PinConfig_TypeDef PriIMU_BusPin;
extern BspSPI_PinConfig_TypeDef SecIMU_BusPin;

extern BspIIC_PinConfig_TypeDef SrvBaro_BusPin;

extern BspSPI_Config_TypeDef PriIMU_BusCfg;
extern BspSPI_Config_TypeDef SecIMU_BusCfg;

extern DebugPrintObj_TypeDef DebugPort;
#define DEBUG_TAG "[ DEBUG INFO ] "
#define DEBUG_INFO(fmt, ...) Debug_Print(&DebugPort, DEBUG_TAG, fmt, ##__VA_ARGS__)

void PriIMU_Dir_Tune(float *gyr, float *acc);
void SecIMU_Dir_Tune(float *gyr, float *acc);

#define Select_Hardware "Hardware MATEKH743"

#define Sample_Blinkly Led2
#define Noti_LED_Ptr NULL
#define BlackBox_Noti_Ptr NULL

extern SPI_HandleTypeDef Baro_Bus_Instance;
extern BspGPIO_Obj_TypeDef *p_Baro_CS;

#ifdef __cplusplus
}
#endif

#endif
