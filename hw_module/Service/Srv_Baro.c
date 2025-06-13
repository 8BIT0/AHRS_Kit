/*
 * Auther: 8_B!T0
 * Baro Sensor Sample Service
 * this file still can be optimized in a big way
 * for currently only one baro sensor can be used
 */
#include "Srv_Baro.h"
#include "Srv_OsCommon.h"
#include "../FCHW_Config.h"
#include "error_log.h"
#include "bsp_iic.h"
#include "bsp_gpio.h"
#include "HW_Def.h"
#include <math.h>

#define STANDER_ATMOSPHERIC_PRESSURE (101.325f * 1000)
#define SRVBARO_SMOOTHWINDOW_SIZE 5

#define SRVBARO_MAX_SAMPLE_PERIOD 10    // unit: ms 10ms 100hz
#define SRVBARO_MIN_SAMPLE_PERIOD 100   // unit: ms 100ms 10hz

/* internal vriable */
SrvBaroObj_TypeDef SrvBaroObj = {
    .init_err = SrvBaro_Error_None,
    .sensor_data = NULL,
};

SrvBaroBusObj_TypeDef SrvBaroBus = {
    .init = false,
    .obj = NULL,
    .api = NULL,
};

/* internal function */
static bool SrvBaro_IICBus_Tx(uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint8_t len);
static bool SrvBaro_IICBus_Rx(uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint8_t len);

static float SrvBaro_PessureCnvToMeter(float pa);

/************************************************************************ Error Tree Item ************************************************************************/
static Error_Handler SrvBaro_Error_Handle = 0;

static void SrvBaro_BusInitError(int16_t code, uint8_t *p_arg, uint16_t size);

static Error_Obj_Typedef SrvBaro_ErrorList[] = {
    {
        .out = true,
        .log = false,
        .prc_callback = SrvBaro_BusInitError,
        .code = SrvBaro_Error_BadSensorObj,
        .desc = "SrvBaro Bad Sensor Object\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvBaro_BusInitError,
        .code = SrvBaro_Error_BusInit,
        .desc = "SrvBaro Bus Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvBaro_BusInitError,
        .code = SrvBaro_Error_DevInit,
        .desc = "SrvBaro Sensor Device Init Failed\r\n",
        .proc_type = Error_Proc_Immd, 
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
};
/************************************************************************ Error Tree Item ************************************************************************/

/* external function */
static uint8_t SrvBaro_Init(void);
static bool SrvBaro_Sample(void);
static bool SrvBaro_Get_Date(SrvBaroData_TypeDef *data);

SrvBaro_TypeDef SrvBaro = {
    .init = SrvBaro_Init,
    .sample = SrvBaro_Sample,
    .get_data = SrvBaro_Get_Date,
};

static bool SrvBaro_BusInit(void)
{
    SrvBaroBus.init = false;
    SrvBaroBus.obj = (void *)&Baro_BusCfg;
    SrvBaroBus.api = (void *)&BspIIC;

    ToIIC_BusObj(SrvBaroBus.obj)->handle = SrvOsCommon.malloc(I2C_HandleType_Size);
    if (ToIIC_BusObj(SrvBaroBus.obj)->handle == NULL)
    {
        SrvOsCommon.free(ToIIC_BusObj(SrvBaroBus.obj)->handle);
        return false;
    }

    ToIIC_BusObj(SrvBaroBus.obj)->PeriphClkInitStruct = SrvOsCommon.malloc(I2C_PeriphCLKInitType_Size);
    if ((ToIIC_BusObj(SrvBaroBus.obj)->PeriphClkInitStruct == NULL) || \
        !ToIIC_BusAPI(SrvBaroBus.api)->init(ToIIC_BusObj(SrvBaroBus.obj)))
    {
        SrvOsCommon.free(ToIIC_BusObj(SrvBaroBus.obj)->handle);
        SrvOsCommon.free(ToIIC_BusObj(SrvBaroBus.obj)->PeriphClkInitStruct);
        return false;
    }

    SrvBaroBus.init = true;
    return true;
}

static uint8_t SrvBaro_Init(void)
{
    /* create error log handle */
    SrvBaro_Error_Handle = ErrorLog.create("SrvBaro_Error");

    /* regist all error to the error tree */
    ErrorLog.registe(SrvBaro_Error_Handle, SrvBaro_ErrorList, sizeof(SrvBaro_ErrorList) / sizeof(SrvBaro_ErrorList[0]));

    if (!SrvBaro_BusInit())
    {
        ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_BusInit, NULL, 0);
        return SrvBaro_Error_BusInit;
    }

    SrvBaroObj.obj = SrvOsCommon.malloc(sizeof(DevDPS310Obj_TypeDef));
    SrvBaroObj.api = &DevDPS310;

    if (SrvBaroObj.obj == NULL)
    {
        SrvOsCommon.free(SrvBaroObj.obj);
        ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_BadSensorObj, NULL, 0);
        return SrvBaro_Error_BadSensorObj;
    }

    ToDPS310_OBJ(SrvBaroObj.obj)->DevAddr   = (DPS310_I2C_ADDR << 1);
    ToDPS310_OBJ(SrvBaroObj.obj)->bus_obj   = SrvBaroBus.obj;
    ToDPS310_OBJ(SrvBaroObj.obj)->bus_rx    = (DevDPS310_BusRead)ToIIC_BusAPI(SrvBaroBus.api)->read;
    ToDPS310_OBJ(SrvBaroObj.obj)->bus_tx    = (DevDPS310_BusWrite)ToIIC_BusAPI(SrvBaroBus.api)->write;
    ToDPS310_OBJ(SrvBaroObj.obj)->get_tick  = SrvOsCommon.get_os_ms;
    ToDPS310_OBJ(SrvBaroObj.obj)->bus_delay = SrvOsCommon.delay_ms;

    /* device init */
    if (!ToDPS310_API(SrvBaroObj.api)->init(ToDPS310_OBJ(SrvBaroObj.obj)))
    {
        SrvOsCommon.free(SrvBaroObj.obj);
        ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_DevInit, NULL, 0);
        return SrvBaro_Error_DevInit;
    }

    SrvBaroObj.data_size = DPS310_DataSize;
    SrvBaroObj.sensor_data = SrvOsCommon.malloc(SrvBaroObj.data_size);

    return SrvBaro_Error_None;
}

static bool SrvBaro_Sample(void)
{
    if ((SrvBaroObj.init_err != SrvBaro_Error_None) || \
        (!ToDPS310_API(SrvBaroObj.api)->sample(ToDPS310_OBJ(SrvBaroObj.obj))))
        return false;

    SrvBaroObj.sample_cnt ++;
    return true;
}

static float SrvBaro_PessureCnvToMeter(float pa)
{
    return ((1 - pow((pa / STANDER_ATMOSPHERIC_PRESSURE), 0.1903))) * 44330;
}

static bool SrvBaro_Get_Date(SrvBaroData_TypeDef *data)
{
    if ((SrvBaroObj.init_err != SrvBaro_Error_None) || \
        (data == NULL) || (SrvBaroObj.sensor_data == NULL) || \
        (SrvBaroObj.data_size == 0) || \
        !ToDPS310_API(SrvBaroObj.api)->ready(ToDPS310_OBJ(SrvBaroObj.obj)))
        return false;
        
    memset(SrvBaroObj.sensor_data, 0, DPS310_DataSize);
    *ToDPS310_DataPtr(SrvBaroObj.sensor_data) = ToDPS310_API(SrvBaroObj.api)->get_data(ToDPS310_OBJ(SrvBaroObj.obj));
    
    /* convert baro pressure to meter */
    data->time_stamp = ToDPS310_DataPtr(SrvBaroObj.sensor_data)->time_stamp;
    data->pressure_alt = SrvBaro_PessureCnvToMeter(ToDPS310_DataPtr(SrvBaroObj.sensor_data)->scaled_press);
    data->tempra = ToDPS310_DataPtr(SrvBaroObj.sensor_data)->scaled_tempra;
    data->pressure = ToDPS310_DataPtr(SrvBaroObj.sensor_data)->scaled_press;

    return true;
}

/*************************************************************** Error Process Callback *******************************************************************************/
static void SrvBaro_BusInitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
    switch(code)
    {
        case SrvBaro_Error_BadSensorObj: break;
        case SrvBaro_Error_BusInit: break;
        case SrvBaro_Error_DevInit: break;
        default: ErrorLog.add_desc("SrvBaro Triggered Unknow ErrorCode: %d\r\n", code); break;
    }
}
