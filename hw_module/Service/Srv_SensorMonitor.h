#ifndef __SRV_SENSORMONITOR_H
#define __SRV_SENSORMONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "util.h"
#include "Bsp_Timer.h"
#include "Srv_IMUSample.h"
#include "Srv_Baro.h"

typedef SrvIMU_Range_TypeDef SrvSensorMonitor_IMURange_TypeDef;

typedef enum
{
    SrvSensorMonitor_StatisticTimer_Defualt = 0,
    SrvSensorMonitor_StatisticTimer_NoError,
    SrvSensorMonitor_StatisticTimer_Error,
} SrvSensorMonitor_StatisticTimer_State_List;

typedef enum
{
    SrvSensorMonitor_Type_IMU = 0,
    SrvSensorMonitor_Type_MAG,
    SrvSensorMonitor_Type_BARO,
    SrvSensorMonitor_Type_Flow,
    SrvSensotMonitor_Type_SUM,
} SrvSensorMonitor_Type_List;

typedef enum
{
    SrvSensorMonitor_SampleFreq_1KHz = 0,
    SrvSensorMonitor_SampleFreq_500Hz,
    SrvSensorMonitor_SampleFreq_250Hz,
    SrvSensorMonitor_SampleFreq_200Hz,
    SrvSensorMonitor_SampleFreq_100Hz,
    SrvSensorMonitor_SampleFreq_50Hz,
    SrvSensorMonitor_SampleFreq_20Hz,
    SrvSensorMonitor_SampleFreq_10Hz,
    SrvSensorMonitor_SampleFreq_5Hz,
    SrvSensorMonitor_SampleFreq_1Hz,
} SrvSensorMonitor_SampleFreq_List;

typedef union
{
    uint32_t val;
    struct
    {
        uint32_t imu  : 1;
        uint32_t mag  : 1;
        uint32_t baro : 1;

        uint32_t res  : 28;
    }bit;
} SrvSensorMonitor_GenReg_TypeDef;

typedef union
{
    uint32_t val;
    struct
    {
        uint32_t imu  : 4;
        uint32_t mag  : 4;
        uint32_t baro : 4;

        uint32_t res  : 20;
    }bit;
} SrvSensorMonitor_SampleFreqReg_TypeDef;

typedef struct
{
    uint32_t sample_cnt;
    uint32_t err_cnt;
} SrvSensorMonitor_Statistic_TypeDef;

typedef struct
{
    uint32_t imu_time;
    uint32_t mag_time;
    uint32_t baro_time;

    float acc[Axis_Sum];
    float gyr[Axis_Sum];
    float mag[Axis_Sum];
    float baro;
    float baro_temp;
} SrvSensorData_TypeDef;

/* bit field on init_state_reg set 1 represent error triggerd on */
typedef struct
{
    SrvSensorMonitor_GenReg_TypeDef init_state_reg;     /* pipe thie vriable to datahub after srv_sensormonitor init */
    SrvSensorMonitor_SampleFreqReg_TypeDef freq_reg;

    SrvSensorMonitor_Statistic_TypeDef *statistic_imu;
    SrvSensorMonitor_Statistic_TypeDef *statistic_mag;
    SrvSensorMonitor_Statistic_TypeDef *statistic_baro;

    SrvSensorMonitor_Statistic_TypeDef *statistic_list;

    SrvIMU_SampleMode_List IMU_SampleMode;
    SrvIMUData_TypeDef lst_imu_data;

    int8_t baro_err;
    SrvBaroData_TypeDef lst_baro_data;

    uint8_t mag_type;
    uint8_t mag_bus_type;
    int8_t mag_err;

    SrvSensorData_TypeDef data;
} SrvSensorMonitorObj_TypeDef;

typedef struct
{
    bool (*init)(SrvSensorMonitorObj_TypeDef *obj);
    bool (*sample_ctl)(SrvSensorMonitorObj_TypeDef *obj);
    SrvSensorData_TypeDef (*get_data)(SrvSensorMonitorObj_TypeDef obj);
} SrvSensorMonitor_TypeDef;

extern SrvSensorMonitor_TypeDef SrvSensorMonitor;

#ifdef __cplusplus
}
#endif

#endif
