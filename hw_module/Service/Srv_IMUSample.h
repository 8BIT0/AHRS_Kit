#ifndef __SRV_IMUSAMPLE_H
#define __SRV_IMUSAMPLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "Srv_OsCommon.h"
#include "imu_data.h"
#include "util.h"
#include "../FCHW_Config.h"

#define MPU_RANGE_MAX_THRESHOLD 1.2f
#define MPU_RANGE_MIN_THRESHOLD 0.9f
#define MPU_MODULE_OPR_DURATION 50 // duration time 50ms

#define IMU_Commu_TimeOut 1000
#define MPU_MODULE_INIT_RETRY 10 // init retry count 10

#define GYR_STATIC_CALIB_CYCLE 100

#define IMU_DATA_SIZE sizeof(SrvIMUData_TypeDef)

typedef union
{
    struct
    {
        uint8_t Pri_State : 2;
        uint8_t Sec_State : 2;
        uint8_t res       : 4;
    } sec;

    uint8_t val;
} SrvMpu_Reg_TypeDef;

typedef enum
{
    SrvIMU_PriModule = 1,
    SrvIMU_SecModule,
} SrvIMU_Module_Type;

typedef enum
{
    SrvIMU_PriDev_Detect_Error = -12,
    SrvIMU_SecDev_Detect_Error = -11,
    SrvIMU_SecIMU_Filter_Init_Error = -10,
    SrvIMU_PriIMU_Filter_Init_Error = -9,
    SrvIMU_PriCSPin_Init_Error = -8,
    SrvIMU_PriExtiPin_Init_Error = -7,
    SrvIMU_PriBus_Init_Error = -6,
    SrvIMU_PriDev_Init_Error = -5,
    SrvIMU_SecCSPin_Init_Error = -4,
    SrvIMU_SecExtiPin_Init_Error = -3,
    SrvIMU_SecBus_Init_Error = -2,
    SrvIMU_SecDev_Init_Error = -1,
    SrvIMU_No_Error = 0,
    SrvIMU_AllModule_Init_Error,
} SrvIMU_ErrorCode_List;

typedef enum
{
    SrvIMU_Sample_NoError = 0,
    SrvIMU_Sample_Module_UnReady,
    SrvIMU_Sample_Data_Acc_Blunt,
    SrvIMU_Sample_Data_Gyr_Blunt,
    SrvIMU_Sample_Data_Acc_OverRange,
    SrvIMU_Sample_Data_Gyr_OverRange,
    SrvIMU_Sample_Over_Angular_Accelerate,
} SrvIMU_SampleErrorCode_List;

typedef enum
{
    SrvIMU_Dev_MPU6000 = 0,
    SrvIMU_Dev_ICM20602,
    SrvIMU_Dev_ICM42688P,
    SrvIMU_Dev_ICM42605,
    SrvIMU_Dev_None,
}SrvIMU_SensorID_List;

static inline const char* SrvIMU_Get_TypeStr(SrvIMU_SensorID_List type)
{
    switch (type)
    {
        case SrvIMU_Dev_MPU6000:    return "MPU6000";
        case SrvIMU_Dev_ICM20602:   return "ICM20602";
        case SrvIMU_Dev_ICM42688P:  return "ICM42688P";
        case SrvIMU_Dev_ICM42605:   return "ICM42605";
        case SrvIMU_Dev_None:   return "None";
        default: return "Unknow type";
    }
}

typedef enum
{
    SrvIMU_Priori_Pri = UTIL_SET_BIT(0),                        /* primary IMU sample Only */
    SrvIMU_Priori_Sec = UTIL_SET_BIT(1),                        /* secondary IMU sample Only */
}SrvIMU_SampleMode_List;

typedef struct
{
    uint8_t Acc;
    uint16_t Gyr;
}SrvIMU_Range_TypeDef;

typedef struct
{
    uint32_t time_stamp;

    SrvIMU_Module_Type module;
    
    float tempera;

    float org_gyr[Axis_Sum];
    float org_acc[Axis_Sum];
} SrvIMUData_TypeDef;

typedef struct
{
    SrvIMU_ErrorCode_List (*init)(void);
    bool (*sample)(SrvIMU_SampleMode_List mode);
    bool (*get_data)(SrvIMU_Module_Type type, SrvIMUData_TypeDef *data);
} SrvIMU_TypeDef;

extern SrvIMU_TypeDef SrvIMU;

#ifdef __cplusplus
}
#endif

#endif
