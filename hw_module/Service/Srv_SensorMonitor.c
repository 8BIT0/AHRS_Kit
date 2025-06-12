#include "Srv_SensorMonitor.h"
#include "Bsp_Timer.h"
#include "Srv_OsCommon.h"
#include "debug_util.h"
#include "HW_Def.h"

/* 
 * for sensor statistic function a timer is essenial
 */
#define SrvSensorMonitor_GetSampleInterval(period) (1000 / period)

/* internal function */
static bool SrvSensorMonitor_IMU_Init(SrvSensorMonitorObj_TypeDef *obj);
static bool SrvSensorMonitor_Mag_Init(void);
static bool SrvSensorMonitor_Baro_Init(SrvSensorMonitorObj_TypeDef *obj);
static SrvIMUData_TypeDef SrvSensorMonitor_Get_IMUData(SrvSensorMonitorObj_TypeDef *obj);
static SrvBaroData_TypeDef SrvSensorMonitor_Get_BaroData(SrvSensorMonitorObj_TypeDef *obj);
static uint32_t SrvSensorMonitor_Get_MagData(SrvSensorMonitorObj_TypeDef *obj);

/* external function */
static bool SrvSensorMonitor_Init(SrvSensorMonitorObj_TypeDef *obj);
static bool SrvSensorMonitor_SampleCTL(SrvSensorMonitorObj_TypeDef *obj);
static SrvSensorData_TypeDef SrvSensorMonitor_Get_Data(SrvSensorMonitorObj_TypeDef obj);

SrvSensorMonitor_TypeDef SrvSensorMonitor = {
    .init = SrvSensorMonitor_Init,
    .sample_ctl = SrvSensorMonitor_SampleCTL,
    .get_data = SrvSensorMonitor_Get_Data,
};

static bool SrvSensorMonitor_Init(SrvSensorMonitorObj_TypeDef *obj)
{
    uint16_t list_index = 0;

    if (obj == NULL)
        return false;

    obj->statistic_list = SrvOsCommon.malloc(3 * sizeof(SrvSensorMonitor_Statistic_TypeDef));
    if (obj->statistic_list == NULL)
    {
        SrvOsCommon.free(obj->statistic_list);
        return false;
    }

    /* enabled on imu must be essential */
    obj->init_state_reg.bit.imu = false;
    if (SrvSensorMonitor_IMU_Init(obj))
    {
        if (list_index > 3)
            return false;

        obj->init_state_reg.bit.imu = true;
        obj->statistic_imu = &obj->statistic_list[list_index];
        list_index ++;
    }

    obj->init_state_reg.bit.mag = false;
    if (SrvSensorMonitor_Mag_Init())
    {
        if (list_index > 3)
            return false;

        obj->init_state_reg.bit.mag = true;
        obj->statistic_mag = &obj->statistic_list[list_index];
        list_index ++;
    }

    obj->init_state_reg.bit.baro = false;
    if (SrvSensorMonitor_Baro_Init(obj))
    {
        if (list_index > 3)
            return false;

        obj->init_state_reg.bit.baro = true;
        obj->statistic_baro = &obj->statistic_list[list_index];
        list_index ++;
    }

    return true;
}

/******************************************* IMU Section **********************************************/
static bool SrvSensorMonitor_IMU_Init(SrvSensorMonitorObj_TypeDef *obj)
{
    if ((SrvIMU.init == NULL) || (obj == NULL))
        return false;
        
    switch(SrvIMU.init())
    {
        case SrvIMU_PriDev_Init_Error: obj->IMU_SampleMode = SrvIMU_Priori_Sec; break;
        case SrvIMU_SecDev_Init_Error: obj->IMU_SampleMode = SrvIMU_Priori_Pri; break;
        case SrvIMU_No_Error: obj->IMU_SampleMode = SrvIMU_Priori_Pri; break;
        case SrvIMU_AllModule_Init_Error:
        default: return false;
    }

    return true;
}

static bool SrvSensorMonitor_IMU_SampleCTL(SrvSensorMonitorObj_TypeDef *obj)
{
    bool state = false;

    if (obj && obj->statistic_imu && \
        obj->init_state_reg.bit.imu && \
        SrvIMU.sample && SrvIMU.sample(obj->IMU_SampleMode))
    {
        // DebugPin.ctl(Debug_PB5, true);
        obj->statistic_imu->sample_cnt ++;
        state = true;
        // DebugPin.ctl(Debug_PB5, false);
    }

    return state;
}

static SrvIMUData_TypeDef SrvSensorMonitor_Get_IMUData(SrvSensorMonitorObj_TypeDef *obj)
{
    SrvIMUData_TypeDef imu_data_tmp;

    memset(&imu_data_tmp, 0, sizeof(SrvIMUData_TypeDef));

    imu_data_tmp = obj->lst_imu_data;
    if (obj && obj->init_state_reg.bit.imu && SrvIMU.get_data && SrvIMU.get_data(SrvIMU_PriModule, &imu_data_tmp))
    {
        /*
        * adjust imu axis coordinate as 
        * x -----> forward
        * y -----> right
        * z -----> down 
        **/
        obj->lst_imu_data = imu_data_tmp;
    }

    return imu_data_tmp;
}

/******************************************* Mag Section **********************************************/
/* still in developing */
static bool SrvSensorMonitor_Mag_Init(void)
{
    return false;
}

static bool SrvSensorMonitor_Mag_SampleCTL(SrvSensorMonitorObj_TypeDef *obj)
{
    if (obj && obj->init_state_reg.bit.mag)
    {
        
    }

    return false;
}

static uint32_t SrvSensorMonitor_Get_MagData(SrvSensorMonitorObj_TypeDef *obj)
{
    return 0;
}

/******************************************* Baro Section *********************************************/
static bool SrvSensorMonitor_Baro_Init(SrvSensorMonitorObj_TypeDef *obj)
{
    if (!SrvBaro.init || (obj == NULL))
        return false;
        
    obj->baro_err = SrvBaro.init();
    if (obj->baro_err == SrvBaro_Error_None)
        return true;
    
    return false;
}

static bool SrvSensorMonitor_Baro_SampleCTL(SrvSensorMonitorObj_TypeDef *obj)
{
    bool state = false;

    if (obj && obj->init_state_reg.bit.baro && SrvBaro.sample && SrvBaro.sample())
    {
        // DebugPin.ctl(Debug_PB5, true);
        obj->statistic_baro->sample_cnt ++;
        state = true;
        // DebugPin.ctl(Debug_PB5, false);
    }

    return state;
}

static SrvBaroData_TypeDef SrvSensorMonitor_Get_BaroData(SrvSensorMonitorObj_TypeDef *obj)
{
    SrvBaroData_TypeDef baro_data_tmp;

    memset(&baro_data_tmp, 0, sizeof(SrvBaroData_TypeDef));

    if (obj && obj->init_state_reg.bit.baro && SrvBaro.get_data)
    {
        if (SrvBaro.get_data(&baro_data_tmp))
        {
            obj->lst_baro_data = baro_data_tmp;
        }
        else
            baro_data_tmp = obj->lst_baro_data;
    }

    return baro_data_tmp;
}

/******************************************* Flow Section **********************************************/

/* noticed all sensor sampling and processing must finished in 1ms maximum */
static bool SrvSensorMonitor_SampleCTL(SrvSensorMonitorObj_TypeDef *obj)
{
    bool state = false;
    SrvIMUData_TypeDef imu_data;
    SrvBaroData_TypeDef baro_data;

    memset(&imu_data, 0, sizeof(SrvIMUData_TypeDef));
    memset(&baro_data, 0, sizeof(SrvBaroData_TypeDef));

    // DebugPin.ctl(Debug_PB5, true);
    
    /* imu single sampling overhead is about 60us */
    state |= SrvSensorMonitor_IMU_SampleCTL(obj);
    state |= SrvSensorMonitor_Mag_SampleCTL(obj);

    /* baro single sampling overhead is about 230us */
    state |= SrvSensorMonitor_Baro_SampleCTL(obj);

    // DebugPin.ctl(Debug_PB5, false);
    baro_data = SrvSensorMonitor_Get_BaroData(obj);
    imu_data = SrvSensorMonitor_Get_IMUData(obj);

    obj->data.imu_time = imu_data.time_stamp;
    obj->data.baro_time = baro_data.time_stamp;
    obj->data.mag_time = 0;

    /* set data */
    for (uint8_t i = Axis_X; i < Axis_Sum; i++)
    {
        obj->data.acc[i] = imu_data.org_acc[i];
        obj->data.gyr[i] = imu_data.org_gyr[i];
        obj->data.mag[i] = 0.0f;
    }

    obj->data.baro = baro_data.pressure;
    return state;
}

static SrvSensorData_TypeDef SrvSensorMonitor_Get_Data(SrvSensorMonitorObj_TypeDef obj)
{
    return obj.data;
}



