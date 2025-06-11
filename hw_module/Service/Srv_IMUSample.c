/*
 * CODE: 8_B!T0
 * might need a axis remap function at the sensor init process
 */
#include "Srv_IMUSample.h"
#include "Dev_MPU6000.h"
#include "Dev_ICM20602.h"
#include "Dev_ICM426xx.h"
#include "HW_Def.h"
#include "Srv_OsCommon.h"
#include "debug_util.h"
#include "Bsp_SPI.h"
#include "error_log.h"
#include "Dev_Led.h"
#include <math.h>

/* use ENU coordinate */
/* IMU coordinate is x->forward y->right z->down */

/*
 * Angular Speed Over Speed Threshold
 * Angular Speed Per Millscond
 */
#define ANGULAR_ACCECLERATION_THRESHOLD 200 / 1.0f      /* angular speed accelerate from 0 to 100 deg/s in 1 Ms */
#define MOVING_DETECT_CONDITION_1 1.0                   /* unit deg/s */
#define MOVING_DETECT_CONDITION_2 (1000.0f / 3600.0f)   /* unit deg/s */
#define ANGULAR_SPEED_ACCURACY 1000

typedef void (*SrvIMU_SetDataReady_Callback)(void *obj);
typedef bool (*SrvIMU_GetDataReady_Callback)(void *obj);
typedef bool (*SrvIMU_Sample_Callback)(void *obj);
typedef IMUModuleScale_TypeDef (*SrvIMU_GetScale_Callback)(void *obj);
typedef IMU_Error_TypeDef (*SrvIMU_GetError_Callback)(void *obj);

typedef struct
{
    SrvIMU_SensorID_List type;
    void *obj_ptr;
    IMUData_TypeDef *OriData_ptr;

    uint8_t acc_trip;
    uint16_t gyr_trip;

    SrvIMU_GetDataReady_Callback get_drdy;
    SrvIMU_Sample_Callback sample;
    SrvIMU_GetScale_Callback get_scale;
    SrvIMU_SetDataReady_Callback set_drdy;
    SrvIMU_GetError_Callback get_error;
}SrvIMU_InuseSensorObj_TypeDef;

static uint32_t SrvIMU_Reupdate_Statistics_CNT = 0;


/* internal variable */
/* MPU6000 Instance */
static SPI_HandleTypeDef PriIMU_Bus_Instance;
/* ICM42688P Instance */
static SPI_HandleTypeDef SecIMU_Bus_Instance;

static DevMPU6000Obj_TypeDef MPU6000Obj;
static DevICM20602Obj_TypeDef ICM20602Obj;
static DevICM426xxObj_TypeDef ICM42688PObj;
static DevICM426xxObj_TypeDef ICM42605Obj;

static Error_Handler SrvMPU_Error_Handle = 0;

/*
 *   PriIMU -> MPU6000
 *   SecIMU -> ICM42688P
 */
static SrvMpu_Reg_TypeDef SrvMpu_Init_Reg;
static SrvMpu_Reg_TypeDef SrvMpu_Update_Reg;
static SrvIMUData_TypeDef IMU_Data;
static SrvIMUData_TypeDef IMU_Data_Lst;

static SrvIMUData_TypeDef PriIMU_Data;
static SrvIMU_InuseSensorObj_TypeDef InUse_PriIMU_Obj;

static SrvIMUData_TypeDef SecIMU_Data;
static SrvIMU_InuseSensorObj_TypeDef InUse_SecIMU_Obj;

/************************************************************************ Error Tree Item ************************************************************************/
static void SrvIMU_PriDev_Filter_InitError(int16_t code, uint8_t *p_arg, uint16_t size);
static void SrvIMU_SecDev_Filter_InitError(int16_t code, uint8_t *p_arg, uint16_t size);
static void SrvIMU_Dev_InitError(int16_t code, uint8_t *p_arg, uint16_t size);
static void SrvIMU_AllModule_InitError(int16_t code, uint8_t *p_arg, uint16_t size);
static void SrvIMU_PriSample_Undrdy(uint8_t *p_arg, uint16_t size);
static void SrvIMU_SecSample_Undrdy(uint8_t *p_arg, uint16_t size);

static Error_Obj_Typedef SrvIMU_ErrorList[] = {
    {
        .out = true,
        .log = false,
        .prc_callback = NULL,
        .code = SrvIMU_SecDev_Detect_Error,
        .desc = "Sec IMU None Detected\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = NULL,
        .code = SrvIMU_PriDev_Detect_Error,
        .desc = "Pri IMU None Detected\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvIMU_SecDev_Filter_InitError,
        .code = SrvIMU_SecIMU_Filter_Init_Error,
        .desc = "Sec IMU Filter Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvIMU_PriDev_Filter_InitError,
        .code = SrvIMU_PriIMU_Filter_Init_Error,
        .desc = "Pri IMU Filter Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = NULL,
        .code = SrvIMU_PriCSPin_Init_Error,
        .desc = "Pri CS Pin Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = NULL,
        .code = SrvIMU_PriExtiPin_Init_Error,
        .desc = "Pri Ext Pin Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = NULL,
        .code = SrvIMU_PriBus_Init_Error,
        .desc = "Pri Bus Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvIMU_Dev_InitError,
        .code = SrvIMU_PriDev_Init_Error,
        .desc = "Pri Dev Init Failed\r\n",
        .proc_type = Error_Proc_Immd,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = NULL,
        .code = SrvIMU_SecCSPin_Init_Error,
        .desc = "Sec CS Pin Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = NULL,
        .code = SrvIMU_SecExtiPin_Init_Error,
        .desc = "Sec Ext Pin Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = NULL,
        .code = SrvIMU_SecBus_Init_Error,
        .desc = "Sec Bus Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvIMU_Dev_InitError,
        .code = SrvIMU_SecDev_Init_Error,
        .desc = "Sec Dev Init Failed\r\n",
        .proc_type = Error_Proc_Immd,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvIMU_AllModule_InitError,
        .code = SrvIMU_AllModule_Init_Error,
        .desc = "All IMU Module Init Failed\r\n",
        .proc_type = Error_Proc_Immd,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
};
/************************************************************************ Error Tree Item ************************************************************************/

/* external function */
static SrvIMU_ErrorCode_List SrvIMU_Init(void);
static bool SrvIMU_Sample(SrvIMU_SampleMode_List mode);
static bool SrvIMU_Get_Data(SrvIMU_Module_Type type, SrvIMUData_TypeDef *data);

/* internal function */
static int8_t SrvIMU_PriIMU_Init(void);
static void SrvIMU_PriIMU_ExtiCallback(void);
static void SrvIMU_PriIMU_CS_Ctl(bool state);
static bool SrvIMU_PriIMU_BusTrans_Rec(uint8_t *Tx, uint8_t *Rx, uint16_t size);

static int8_t SrvIMU_SecIMU_Init(void);
static void SrvIMU_SecIMU_ExtiCallback(void);
static void SrvIMU_SecIMU_CS_Ctl(bool state);
static bool SrvIMU_SecIMU_BusTrans_Rec(uint8_t *Tx, uint8_t *Rx, uint16_t size);

static SrvIMU_SensorID_List SrvIMU_AutoDetect(bus_trans_callback trans, cs_ctl_callback cs_ctl);

SrvIMU_TypeDef SrvIMU = {
    .init = SrvIMU_Init,
    .sample = SrvIMU_Sample,
    .get_data = SrvIMU_Get_Data,
};

static SrvIMU_ErrorCode_List SrvIMU_Init(void)
{
    memset(&InUse_PriIMU_Obj, 0, sizeof(InUse_PriIMU_Obj));

    InUse_PriIMU_Obj.type = SrvIMU_Dev_None;

    memset(&PriIMU_Data, 0, sizeof(PriIMU_Data));
     
    /* create error log handle */
    SrvMPU_Error_Handle = ErrorLog.create("SrvIMU_Error");

    /* regist all error to the error tree */
    ErrorLog.registe(SrvMPU_Error_Handle, SrvIMU_ErrorList, sizeof(SrvIMU_ErrorList) / sizeof(SrvIMU_ErrorList[0]));

    SrvIMU_ErrorCode_List PriIMU_Init_State = SrvIMU_PriIMU_Init();

    SrvMpu_Init_Reg.val = 0;
    SrvMpu_Update_Reg.val = 0;

    SrvMpu_Init_Reg.sec.Pri_State = true;
    if (PriIMU_Init_State != SrvIMU_No_Error)
    {
        SrvMpu_Init_Reg.sec.Pri_State = false;
        ErrorLog.trigger(SrvMPU_Error_Handle, PriIMU_Init_State, (uint8_t *)&InUse_PriIMU_Obj, sizeof(InUse_PriIMU_Obj));
    }

    memset(&InUse_SecIMU_Obj, 0, sizeof(InUse_SecIMU_Obj));
    
    InUse_SecIMU_Obj.type = SrvIMU_Dev_None;

    memset(&SecIMU_Data, 0, sizeof(SecIMU_Data));

    SrvIMU_ErrorCode_List SecIMU_Init_State = SrvIMU_SecIMU_Init();
    
    SrvMpu_Init_Reg.sec.Sec_State = true;
    if (SecIMU_Init_State != SrvIMU_No_Error)
    {
        SrvMpu_Init_Reg.sec.Sec_State = false;
        ErrorLog.trigger(SrvMPU_Error_Handle, SecIMU_Init_State, (uint8_t *)&InUse_SecIMU_Obj, sizeof(InUse_SecIMU_Obj));
    }

    if (!SrvMpu_Init_Reg.sec.Pri_State && !SrvMpu_Init_Reg.sec.Sec_State)
    {
        ErrorLog.trigger(SrvMPU_Error_Handle, SrvIMU_AllModule_Init_Error, NULL, 0);
        return SrvIMU_AllModule_Init_Error;
    }
    else if (!SrvMpu_Init_Reg.sec.Pri_State && SrvMpu_Init_Reg.sec.Sec_State)
    {
        return SrvIMU_PriDev_Init_Error;
    }
    else if (SrvMpu_Init_Reg.sec.Pri_State && !SrvMpu_Init_Reg.sec.Sec_State)
    {
        return SrvIMU_SecDev_Init_Error;
    }
    
    return SrvIMU_No_Error;
}

/* init primary IMU Device */
/* consider use spi dma sample raw data */
static SrvIMU_ErrorCode_List SrvIMU_PriIMU_Init(void)
{
    /* primary IMU Pin & Bus Init */
    if (!BspGPIO.out_init(PriIMU_CSPin))
        return SrvIMU_PriCSPin_Init_Error;

    if (!BspGPIO.exti_init(PriIMU_INTPin, SrvIMU_PriIMU_ExtiCallback))
        return SrvIMU_PriExtiPin_Init_Error;

    PriIMU_BusCfg.Pin = PriIMU_BusPin;
    if (!BspSPI.init(PriIMU_BusCfg, &PriIMU_Bus_Instance))
        return SrvIMU_PriBus_Init_Error;

    switch(SrvIMU_AutoDetect(SrvIMU_PriIMU_BusTrans_Rec, SrvIMU_PriIMU_CS_Ctl))
    {
        case SrvIMU_Dev_MPU6000:
            InUse_PriIMU_Obj.type = SrvIMU_Dev_MPU6000;
            DevMPU6000.pre_init(&MPU6000Obj,
                                SrvIMU_PriIMU_CS_Ctl,
                                SrvIMU_PriIMU_BusTrans_Rec,
                                SrvOsCommon.delay_ms,
                                SrvOsCommon.get_os_ms);

            DevMPU6000.config(&MPU6000Obj,
                               MPU6000_SampleRate_1K,
                               MPU6000_Acc_16G,
                               MPU6000_Gyr_2000DPS);

            InUse_PriIMU_Obj.obj_ptr = &MPU6000Obj;
            InUse_PriIMU_Obj.OriData_ptr = &(MPU6000Obj.OriData);
            InUse_PriIMU_Obj.acc_trip = MPU6000Obj.PHY_AccTrip_Val;
            InUse_PriIMU_Obj.gyr_trip = MPU6000Obj.PHY_GyrTrip_Val;
            
            InUse_PriIMU_Obj.set_drdy = (SrvIMU_SetDataReady_Callback)(DevMPU6000.set_ready);
            InUse_PriIMU_Obj.get_drdy = (SrvIMU_GetDataReady_Callback)DevMPU6000.get_ready;
            InUse_PriIMU_Obj.get_scale = (SrvIMU_GetScale_Callback)(DevMPU6000.get_scale);
            InUse_PriIMU_Obj.sample = (SrvIMU_Sample_Callback)(DevMPU6000.sample);
            InUse_PriIMU_Obj.get_error = (SrvIMU_GetError_Callback)(DevMPU6000.get_error);

            if (!DevMPU6000.init(&MPU6000Obj))
                return SrvIMU_PriDev_Init_Error;
        break;

        case SrvIMU_Dev_ICM20602:
            InUse_PriIMU_Obj.type = SrvIMU_Dev_ICM20602;
            DevICM20602.pre_init(&ICM20602Obj,
                                 SrvIMU_PriIMU_CS_Ctl,
                                 SrvIMU_PriIMU_BusTrans_Rec,
                                 SrvOsCommon.delay_ms,
                                 SrvOsCommon.get_os_ms);

            DevICM20602.config(&ICM20602Obj,
                                ICM20602_SampleRate_1K,
                                ICM20602_Acc_16G,
                                ICM20602_Gyr_2000DPS);

            InUse_PriIMU_Obj.obj_ptr = &ICM20602Obj;
            InUse_PriIMU_Obj.OriData_ptr = &(ICM20602Obj.OriData);
            InUse_PriIMU_Obj.acc_trip = ICM20602Obj.PHY_AccTrip_Val;
            InUse_PriIMU_Obj.gyr_trip = ICM20602Obj.PHY_GyrTrip_Val;

            InUse_PriIMU_Obj.set_drdy = (SrvIMU_SetDataReady_Callback)(DevICM20602.set_ready);
            InUse_PriIMU_Obj.get_drdy = (SrvIMU_GetDataReady_Callback)DevICM20602.get_ready;
            InUse_PriIMU_Obj.get_scale = (SrvIMU_GetScale_Callback)(DevICM20602.get_scale);
            InUse_PriIMU_Obj.sample = (SrvIMU_Sample_Callback)(DevICM20602.sample);
            InUse_PriIMU_Obj.get_error = (SrvIMU_GetError_Callback)(DevICM20602.get_error);

            if (!DevICM20602.init(&ICM20602Obj))
                return SrvIMU_PriDev_Init_Error;
        break;

        case SrvIMU_Dev_ICM42688P:
            InUse_PriIMU_Obj.type = SrvIMU_Dev_ICM42688P;
            DevICM426xx.pre_init(&ICM42688PObj,
                                 SrvIMU_PriIMU_CS_Ctl,
                                 SrvIMU_PriIMU_BusTrans_Rec,
                                 SrvOsCommon.delay_ms,
                                 SrvOsCommon.get_os_ms);

            DevICM426xx.config(&ICM42688PObj,
                                ICM426xx_SampleRate_1K,
                                ICM426xx_Acc_16G,
                                ICM426xx_Gyr_2000DPS);

            InUse_PriIMU_Obj.obj_ptr = &ICM42688PObj;
            InUse_PriIMU_Obj.OriData_ptr = &(ICM42688PObj.OriData);
            InUse_PriIMU_Obj.acc_trip = ICM42688PObj.PHY_AccTrip_Val;
            InUse_PriIMU_Obj.gyr_trip = ICM42688PObj.PHY_GyrTrip_Val;

            InUse_PriIMU_Obj.set_drdy = (SrvIMU_SetDataReady_Callback)(DevICM426xx.set_ready);
            InUse_PriIMU_Obj.get_drdy = (SrvIMU_GetDataReady_Callback)(DevICM426xx.get_ready);
            InUse_PriIMU_Obj.get_scale = (SrvIMU_GetScale_Callback)(DevICM426xx.get_scale);
            InUse_PriIMU_Obj.sample = (SrvIMU_Sample_Callback)(DevICM426xx.sample);
            InUse_PriIMU_Obj.get_error = (SrvIMU_GetError_Callback)(DevICM426xx.get_error);

            if (!DevICM426xx.init(&ICM42688PObj))
                return SrvIMU_PriDev_Init_Error;
        break;

        case SrvIMU_Dev_ICM42605:
            InUse_PriIMU_Obj.type = SrvIMU_Dev_ICM42605;
            DevICM426xx.pre_init(&ICM42605Obj,
                                 SrvIMU_PriIMU_CS_Ctl,
                                 SrvIMU_PriIMU_BusTrans_Rec,
                                 SrvOsCommon.delay_ms,
                                 SrvOsCommon.get_os_ms);

            DevICM426xx.config(&ICM42605Obj,
                                ICM426xx_SampleRate_1K,
                                ICM426xx_Acc_16G,
                                ICM426xx_Gyr_2000DPS);

            InUse_PriIMU_Obj.obj_ptr = &ICM42605Obj;
            InUse_PriIMU_Obj.OriData_ptr = &(ICM42605Obj.OriData);
            InUse_PriIMU_Obj.acc_trip = ICM42605Obj.PHY_AccTrip_Val;
            InUse_PriIMU_Obj.gyr_trip = ICM42605Obj.PHY_GyrTrip_Val;

            InUse_PriIMU_Obj.set_drdy = (SrvIMU_SetDataReady_Callback)(DevICM426xx.set_ready);
            InUse_PriIMU_Obj.get_drdy = (SrvIMU_GetDataReady_Callback)(DevICM426xx.get_ready);
            InUse_PriIMU_Obj.get_scale = (SrvIMU_GetScale_Callback)(DevICM426xx.get_scale);
            InUse_PriIMU_Obj.sample = (SrvIMU_Sample_Callback)(DevICM426xx.sample);
            InUse_PriIMU_Obj.get_error = (SrvIMU_GetError_Callback)(DevICM426xx.get_error);

            if (!DevICM426xx.init(&ICM42605Obj))
                return SrvIMU_PriDev_Init_Error;
        break;

        default: return SrvIMU_PriDev_Detect_Error;
    }

    return SrvIMU_No_Error;
}

/* input true selected / false deselected */
static void SrvIMU_PriIMU_CS_Ctl(bool state)
{
    BspGPIO.write(PriIMU_CSPin, state);
}

static bool SrvIMU_PriIMU_BusTrans_Rec(uint8_t *Tx, uint8_t *Rx, uint16_t size)
{
    return BspSPI.trans_receive(&PriIMU_Bus_Instance, Tx, Rx, size, IMU_Commu_TimeOut);
}

/* init primary IMU Device */
/* consider use spi dma sample raw data */
static SrvIMU_ErrorCode_List SrvIMU_SecIMU_Init(void)
{
    /* primary IMU Pin & Bus Init */
    if (!BspGPIO.out_init(SecIMU_CSPin))
        return SrvIMU_SecCSPin_Init_Error;

    if (!BspGPIO.exti_init(SecIMU_INTPin, SrvIMU_SecIMU_ExtiCallback))
        return SrvIMU_SecExtiPin_Init_Error;

    SecIMU_BusCfg.Pin = SecIMU_BusPin;
    if (!BspSPI.init(SecIMU_BusCfg, &SecIMU_Bus_Instance))
        return SrvIMU_SecBus_Init_Error;

    switch(SrvIMU_AutoDetect(SrvIMU_SecIMU_BusTrans_Rec, SrvIMU_SecIMU_CS_Ctl))
    {
        case SrvIMU_Dev_MPU6000:
            InUse_SecIMU_Obj.type = SrvIMU_Dev_MPU6000;
            DevMPU6000.pre_init(&MPU6000Obj,
                                SrvIMU_SecIMU_CS_Ctl,
                                SrvIMU_SecIMU_BusTrans_Rec,
                                SrvOsCommon.delay_ms,
                                SrvOsCommon.get_os_ms);

            DevMPU6000.config(&MPU6000Obj,
                               MPU6000_SampleRate_1K,
                               MPU6000_Acc_16G,
                               MPU6000_Gyr_2000DPS);

            InUse_SecIMU_Obj.obj_ptr = &MPU6000Obj;
            InUse_SecIMU_Obj.OriData_ptr = &(MPU6000Obj.OriData);
            InUse_SecIMU_Obj.acc_trip = MPU6000Obj.PHY_AccTrip_Val;
            InUse_SecIMU_Obj.gyr_trip = MPU6000Obj.PHY_GyrTrip_Val;

            InUse_SecIMU_Obj.set_drdy = (SrvIMU_SetDataReady_Callback)(DevMPU6000.set_ready);
            InUse_SecIMU_Obj.get_drdy = (SrvIMU_GetDataReady_Callback)(DevMPU6000.get_ready);
            InUse_SecIMU_Obj.get_scale = (SrvIMU_GetScale_Callback)(DevMPU6000.get_scale);
            InUse_SecIMU_Obj.sample = (SrvIMU_Sample_Callback)(DevMPU6000.sample);
            InUse_SecIMU_Obj.get_error = (SrvIMU_GetError_Callback)(DevMPU6000.get_error);

            if (!DevMPU6000.init(&MPU6000Obj))
                return SrvIMU_SecDev_Init_Error;
        break;

        case SrvIMU_Dev_ICM20602:
            InUse_SecIMU_Obj.type = SrvIMU_Dev_ICM20602;
            DevICM20602.pre_init(&ICM20602Obj,
                                 SrvIMU_SecIMU_CS_Ctl,
                                 SrvIMU_SecIMU_BusTrans_Rec,
                                 SrvOsCommon.delay_ms,
                                 SrvOsCommon.get_os_ms);

            DevICM20602.config(&ICM20602Obj,
                                ICM20602_SampleRate_1K,
                                ICM20602_Acc_16G,
                                ICM20602_Gyr_2000DPS);

            InUse_SecIMU_Obj.obj_ptr = &ICM20602Obj;
            InUse_SecIMU_Obj.OriData_ptr = &(ICM20602Obj.OriData);
            InUse_SecIMU_Obj.acc_trip = ICM20602Obj.PHY_AccTrip_Val;
            InUse_SecIMU_Obj.gyr_trip = ICM20602Obj.PHY_GyrTrip_Val;

            InUse_SecIMU_Obj.set_drdy = (SrvIMU_SetDataReady_Callback)(DevICM20602.set_ready);
            InUse_SecIMU_Obj.get_drdy = (SrvIMU_GetDataReady_Callback)(DevICM20602.get_ready);
            InUse_SecIMU_Obj.get_scale = (SrvIMU_GetScale_Callback)(DevICM20602.get_scale);
            InUse_SecIMU_Obj.sample = (SrvIMU_Sample_Callback)(DevICM20602.sample);
            InUse_SecIMU_Obj.get_error = (SrvIMU_GetError_Callback)(DevICM20602.get_error);

            if (!DevICM20602.init(&ICM20602Obj))
                return SrvIMU_SecDev_Init_Error;
        break;

        case SrvIMU_Dev_ICM42688P:
            InUse_SecIMU_Obj.type = SrvIMU_Dev_ICM42688P;
            DevICM426xx.pre_init(&ICM42688PObj,
                                 SrvIMU_SecIMU_CS_Ctl,
                                 SrvIMU_SecIMU_BusTrans_Rec,
                                 SrvOsCommon.delay_ms,
                                 SrvOsCommon.get_os_ms);

            DevICM426xx.config(&ICM42688PObj,
                                ICM426xx_SampleRate_1K,
                                ICM426xx_Acc_16G,
                                ICM426xx_Gyr_2000DPS);

            InUse_SecIMU_Obj.obj_ptr = &ICM42688PObj;
            InUse_SecIMU_Obj.OriData_ptr = &(ICM42688PObj.OriData);
            InUse_SecIMU_Obj.acc_trip = ICM42688PObj.PHY_AccTrip_Val;
            InUse_SecIMU_Obj.gyr_trip = ICM42688PObj.PHY_GyrTrip_Val;

            InUse_SecIMU_Obj.set_drdy = (SrvIMU_SetDataReady_Callback)(DevICM426xx.set_ready);
            InUse_SecIMU_Obj.get_drdy = (SrvIMU_GetDataReady_Callback)(DevICM426xx.get_ready);
            InUse_SecIMU_Obj.get_scale = (SrvIMU_GetScale_Callback)(DevICM426xx.get_scale);
            InUse_SecIMU_Obj.sample = (SrvIMU_Sample_Callback)(DevICM426xx.sample);
            InUse_SecIMU_Obj.get_error = (SrvIMU_GetError_Callback)(DevICM426xx.get_error);

            if (!DevICM426xx.init(&ICM42688PObj))
                return SrvIMU_SecDev_Init_Error;
        break;

        case SrvIMU_Dev_ICM42605:
            InUse_SecIMU_Obj.type = SrvIMU_Dev_ICM42605;
            DevICM426xx.pre_init(&ICM42605Obj,
                                 SrvIMU_SecIMU_CS_Ctl,
                                 SrvIMU_SecIMU_BusTrans_Rec,
                                 SrvOsCommon.delay_ms,
                                 SrvOsCommon.get_os_ms);

            DevICM426xx.config(&ICM42605Obj,
                                ICM426xx_SampleRate_1K,
                                ICM426xx_Acc_16G,
                                ICM426xx_Gyr_2000DPS);

            InUse_SecIMU_Obj.obj_ptr = &ICM42605Obj;
            InUse_SecIMU_Obj.OriData_ptr = &(ICM42605Obj.OriData);
            InUse_SecIMU_Obj.acc_trip = ICM42605Obj.PHY_AccTrip_Val;
            InUse_SecIMU_Obj.gyr_trip = ICM42605Obj.PHY_GyrTrip_Val;

            InUse_SecIMU_Obj.set_drdy = (SrvIMU_SetDataReady_Callback)(DevICM426xx.set_ready);
            InUse_SecIMU_Obj.get_drdy = (SrvIMU_GetDataReady_Callback)(DevICM426xx.get_ready);
            InUse_SecIMU_Obj.get_scale = (SrvIMU_GetScale_Callback)(DevICM426xx.get_scale);
            InUse_SecIMU_Obj.sample = (SrvIMU_Sample_Callback)(DevICM426xx.sample);
            InUse_SecIMU_Obj.get_error = (SrvIMU_GetError_Callback)(DevICM426xx.get_error);

            if (!DevICM426xx.init(&ICM42605Obj))
                return SrvIMU_SecDev_Init_Error;
        break;

        default: return SrvIMU_SecDev_Detect_Error;
    }

    return SrvIMU_No_Error;
}

static void SrvIMU_SecIMU_CS_Ctl(bool state)
{
    BspGPIO.write(SecIMU_CSPin, state);
}

static bool SrvIMU_SecIMU_BusTrans_Rec(uint8_t *Tx, uint8_t *Rx, uint16_t size)
{
    return BspSPI.trans_receive(&SecIMU_Bus_Instance, Tx, Rx, size, IMU_Commu_TimeOut);
}

/************************************************************ Module Sample API Function *****************************************************************************/
static bool SrvIMU_Sample(SrvIMU_SampleMode_List mode)
{
    static uint32_t PriSample_Rt_Lst = 0;
    uint8_t i = Axis_X;
    bool pri_sample_state = true;
    bool sec_sample_state = true;
    static uint32_t SecSample_Rt_Lst = 0;
    bool PriSample_Enable = mode & SrvIMU_Priori_Pri;
    bool SecSample_Enable = mode & SrvIMU_Priori_Sec;

    /* don`t use error tree down below it may decrease code efficient */
    /* trigger error directly when sampling */

    if(mode > SrvIMU_Priori_Sec)
        return false;

    /* lock fus data */
    SrvMpu_Update_Reg.sec.Fus_State = true;

    /* pri imu init successed */
    if (SrvMpu_Init_Reg.sec.Pri_State & PriSample_Enable)
    {
        PriIMU_Data.module = SrvIMU_PriModule;

        /* pri imu module data ready triggered */
        if (InUse_PriIMU_Obj.get_drdy(InUse_PriIMU_Obj.obj_ptr) && InUse_PriIMU_Obj.sample(InUse_PriIMU_Obj.obj_ptr))
        {
            /* lock */
            SrvMpu_Update_Reg.sec.Pri_State = true;

            PriIMU_Data.time_stamp = InUse_PriIMU_Obj.OriData_ptr->time_stamp;

            /* check Primary IMU module Sample is correct or not */
            if (PriSample_Rt_Lst && (PriIMU_Data.time_stamp <= PriSample_Rt_Lst))
                pri_sample_state = false;

            if (pri_sample_state)
            {
                /* update pri imu data */
                PriIMU_Data.tempera = InUse_PriIMU_Obj.OriData_ptr->temp_flt;

                for (i = Axis_X; i < Axis_Sum; i++)
                {
                    PriIMU_Data.org_acc[i] = InUse_PriIMU_Obj.OriData_ptr->acc_flt[i];
                    PriIMU_Data.org_gyr[i] = InUse_PriIMU_Obj.OriData_ptr->gyr_flt[i];
                }

                PriIMU_Dir_Tune(PriIMU_Data.org_gyr, PriIMU_Data.org_acc);
                 
                /* unlock */
                SrvMpu_Update_Reg.sec.Pri_State = false;
                IMU_Data_Lst = PriIMU_Data;
            }
        }
        else
        {
            SrvIMU_PriSample_Undrdy(NULL, 0);
            pri_sample_state = false;
        }

        PriSample_Rt_Lst = PriIMU_Data.time_stamp;
    }
    else
        pri_sample_state = false;

    /* sec imu init successed */
    if (SrvMpu_Init_Reg.sec.Sec_State & SecSample_Enable)
    {
        SecIMU_Data.module = SrvIMU_SecModule;

        /* sec imu module data ready triggered */
        if (InUse_SecIMU_Obj.get_drdy(InUse_SecIMU_Obj.obj_ptr) && InUse_SecIMU_Obj.sample(InUse_SecIMU_Obj.obj_ptr))
        {
            /* lock */
            SrvMpu_Update_Reg.sec.Sec_State = true;

            SecIMU_Data.time_stamp = InUse_SecIMU_Obj.OriData_ptr->time_stamp;

            /* check Secondry IMU module Sample is correct or not */
            if (SecSample_Rt_Lst && (SecIMU_Data.time_stamp <= SecSample_Rt_Lst))
                sec_sample_state = false;

            if (sec_sample_state)
            {
                /* update sec imu data */
                SecIMU_Data.tempera = InUse_SecIMU_Obj.OriData_ptr->temp_flt;

                for (i = Axis_X; i < Axis_Sum; i++)
                {
                    SecIMU_Data.org_acc[i] = InUse_SecIMU_Obj.OriData_ptr->acc_flt[i];
                    SecIMU_Data.org_gyr[i] = InUse_SecIMU_Obj.OriData_ptr->gyr_flt[i];
                }

                SecIMU_Dir_Tune(SecIMU_Data.org_gyr, SecIMU_Data.org_acc);
                
                /* unlock */
                SrvMpu_Update_Reg.sec.Sec_State = false;
            }
        }
        else
        {
            SrvIMU_SecSample_Undrdy(NULL, 0);
            sec_sample_state = false;
        }

        SecSample_Rt_Lst = SecIMU_Data.time_stamp;
    }
    else
        sec_sample_state = false;
    
    IMU_Data = IMU_Data_Lst;
    switch(mode)
    {
        case SrvIMU_Priori_Pri:
            if(pri_sample_state)
                IMU_Data = PriIMU_Data;
        break;

        case SrvIMU_Priori_Sec:
            if(!sec_sample_state)
                IMU_Data = SecIMU_Data;
        break;

        default: return false;
    }

    /* unlock fus data */
    SrvMpu_Update_Reg.sec.Fus_State = false;
    IMU_Data_Lst = IMU_Data;

    return (pri_sample_state | sec_sample_state);
}

static bool SrvIMU_Get_Data(SrvIMU_Module_Type type, SrvIMUData_TypeDef *data)
{
    SrvIMUData_TypeDef imu_data_tmp;

    memset(&imu_data_tmp, 0, IMU_DATA_SIZE);

    if(data == NULL)
        return false;

reupdate_imu:
    if (type == SrvIMU_PriModule)
    {
        if (!SrvMpu_Update_Reg.sec.Pri_State)
        {
            memcpy(&imu_data_tmp, &PriIMU_Data, IMU_DATA_SIZE);
        }
        else
            goto reupdate_imu_statistics;
    }
    else if (type == SrvIMU_SecModule)
    {
        if (!SrvMpu_Update_Reg.sec.Sec_State)
        {
            memcpy(&imu_data_tmp, &SecIMU_Data, IMU_DATA_SIZE);
        }
        else
            goto reupdate_imu_statistics;
    }

    memcpy(data, &imu_data_tmp, sizeof(SrvIMUData_TypeDef));
    return true;

reupdate_imu_statistics:
    SrvIMU_Reupdate_Statistics_CNT ++;
    goto reupdate_imu;
}

static SrvIMU_SensorID_List SrvIMU_AutoDetect(bus_trans_callback trans, cs_ctl_callback cs_ctl)
{
    SrvOsCommon.delay_ms(20);

    if(DevMPU6000.detect(trans, cs_ctl))
        return SrvIMU_Dev_MPU6000;

    if(DevICM20602.detect(trans, cs_ctl))
        return SrvIMU_Dev_ICM20602;

    switch(DevICM426xx.detect(trans, cs_ctl))
    {
        case ICM42605: return SrvIMU_Dev_ICM42605;
        case ICM42688P: return SrvIMU_Dev_ICM42688P;
        default: break;
    }

    return SrvIMU_Dev_None;
}

/************************************************************ general function *****************************************************************************/
static char* SrvIMU_GetSensorType_Str(SrvIMU_SensorID_List type)
{
    switch(type)
    {
        case SrvIMU_Dev_MPU6000:    return "MPU6000\r\n";
        case SrvIMU_Dev_ICM20602:   return "ICM20602\r\n";
        case SrvIMU_Dev_ICM42688P:  return "ICM42688P\r\n";
        case SrvIMU_Dev_ICM42605:   return "ICM42605\r\n";
        case SrvIMU_Dev_None:       return "None\r\n";
        default:                    return "None\r\n";
    }
}

/************************************************************ DataReady Pin Exti Callback *****************************************************************************/
static void SrvIMU_PriIMU_ExtiCallback(void)
{
    if (SrvMpu_Init_Reg.sec.Pri_State)
        InUse_PriIMU_Obj.set_drdy(InUse_PriIMU_Obj.obj_ptr);
}

static void SrvIMU_SecIMU_ExtiCallback(void)
{
    if (SrvMpu_Init_Reg.sec.Sec_State)
        InUse_SecIMU_Obj.set_drdy(InUse_SecIMU_Obj.obj_ptr);
}

/*************************************************************** Error Process Callback *******************************************************************************/
static void SrvIMU_PriDev_Filter_InitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
    ErrorLog.add_desc("IMU Pri Filter Init Error\r\n");
}

static void SrvIMU_SecDev_Filter_InitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
    ErrorLog.add_desc("IMU Sec Filter Init Error\r\n");
}

static void SrvIMU_Dev_InitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
    SrvIMU_InuseSensorObj_TypeDef *InUseObj_Ptr = NULL;

    if(p_arg && size)
    {
        InUseObj_Ptr = (SrvIMU_InuseSensorObj_TypeDef *)p_arg;
        
        ErrorLog.add_desc("IMU   Type:   %s", SrvIMU_GetSensorType_Str(InUseObj_Ptr->type));
        ErrorLog.add_desc("error code:   %d\r\n", InUseObj_Ptr->get_error(InUseObj_Ptr->obj_ptr).code);
        ErrorLog.add_desc("error func:   %s\r\n", InUseObj_Ptr->get_error(InUseObj_Ptr->obj_ptr).function);
        ErrorLog.add_desc("error line:   %d\r\n", InUseObj_Ptr->get_error(InUseObj_Ptr->obj_ptr).line);
        ErrorLog.add_desc("error reg:    %02x\r\n", InUseObj_Ptr->get_error(InUseObj_Ptr->obj_ptr).tar_reg);
        ErrorLog.add_desc("error reg rx: %02x\r\n", InUseObj_Ptr->get_error(InUseObj_Ptr->obj_ptr).reg_r_val);
        ErrorLog.add_desc("error reg tx: %02x\r\n", InUseObj_Ptr->get_error(InUseObj_Ptr->obj_ptr).reg_t_val);

        ErrorLog.add_desc("\r\n");
    }
}

static void SrvIMU_AllModule_InitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
    if(p_arg && size)
        (*(uint32_t *)p_arg)++;
}

static void SrvIMU_PriSample_Undrdy(uint8_t *p_arg, uint16_t size)
{
}

static void SrvIMU_SecSample_Undrdy(uint8_t *p_arg, uint16_t size)
{
}
