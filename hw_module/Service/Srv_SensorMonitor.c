#include "Srv_SensorMonitor.h"
#include "Bsp_Timer.h"
#include "Srv_OsCommon.h"
#include "debug_util.h"
#include "HW_Def.h"

/* 
 * for sensor statistic function a timer is essenial
 */
#define SrvSensorMonitor_GetSampleInterval(period) (1000 / period)

#define MONITOR_TAG "[ SENSOR MONITOR INFO ] "
#define MONITOR_INFO(fmt, ...) Debug_Print(&DebugPort, MONITOR_TAG, fmt, ##__VA_ARGS__)

/* internal function */
static uint32_t SrvSensorMonitor_Get_FreqVal(uint8_t freq_enum);

static bool SrvSensorMonitor_IMU_Init(SrvSensorMonitorObj_TypeDef *obj);
static bool SrvSensorMonitor_Mag_Init(void);
static bool SrvSensorMonitor_Baro_Init(SrvSensorMonitorObj_TypeDef *obj);
static SrvIMUData_TypeDef SrvSensorMonitor_Get_IMUData(SrvSensorMonitorObj_TypeDef *obj);
static SrvBaroData_TypeDef SrvSensorMonitor_Get_BaroData(SrvSensorMonitorObj_TypeDef *obj);

/* external function */
static bool SrvSensorMonitor_Init(SrvSensorMonitorObj_TypeDef *obj);
static bool SrvSensorMonitor_SampleCTL(SrvSensorMonitorObj_TypeDef *obj);

SrvSensorMonitor_TypeDef SrvSensorMonitor = {
    .init = SrvSensorMonitor_Init,
    .sample_ctl = SrvSensorMonitor_SampleCTL,
};

static bool SrvSensorMonitor_Init(SrvSensorMonitorObj_TypeDef *obj)
{
    uint8_t enable_sensor_num = 0;
    uint16_t list_index = 0;

    if (obj == NULL)
        return false;

    obj->statistic_list = SrvOsCommon.malloc(enable_sensor_num * sizeof(SrvSensorMonitor_Statistic_TypeDef));
    if (obj->statistic_list == NULL)
    {
        SrvOsCommon.free(obj->statistic_list);
        return false;
    }

    /* enabled on imu must be essential */
    obj->init_state_reg.bit.imu = false;
    if (SrvSensorMonitor_IMU_Init(obj))
    {
        if (list_index > enable_sensor_num)
            return false;

        obj->init_state_reg.bit.imu = true;
        obj->statistic_imu = &obj->statistic_list[list_index];
        obj->statistic_imu->set_period = SrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.imu);
        list_index ++;
    }

    obj->init_state_reg.bit.mag = false;
    if (SrvSensorMonitor_Mag_Init())
    {
        if (list_index > enable_sensor_num)
            return false;

        obj->init_state_reg.bit.mag = true;
        obj->statistic_mag = &obj->statistic_list[list_index];
        obj->statistic_mag->set_period = SrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.mag);
        list_index ++;
    }

    obj->init_state_reg.bit.baro = false;
    if (SrvSensorMonitor_Baro_Init(obj))
    {
        if (list_index > enable_sensor_num)
            return false;

        obj->init_state_reg.bit.baro = true;
        obj->statistic_baro = &obj->statistic_list[list_index];
        obj->statistic_baro->set_period = SrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.baro);
        list_index ++;
    }

    return true;
}

static uint32_t SrvSensorMonitor_Get_FreqVal(uint8_t freq_enum)
{
    switch(freq_enum)
    {
        case SrvSensorMonitor_SampleFreq_1KHz:  return 1000;
        case SrvSensorMonitor_SampleFreq_500Hz: return 500;
        case SrvSensorMonitor_SampleFreq_250Hz: return 250;
        case SrvSensorMonitor_SampleFreq_200Hz: return 200;
        case SrvSensorMonitor_SampleFreq_100Hz: return 100;
        case SrvSensorMonitor_SampleFreq_50Hz:  return 50;
        case SrvSensorMonitor_SampleFreq_20Hz:  return 20;
        case SrvSensorMonitor_SampleFreq_10Hz:  return 10;
        case SrvSensorMonitor_SampleFreq_5Hz:   return 5;
        case SrvSensorMonitor_SampleFreq_1Hz:   return 1;
        default: return 0;
    }

    return 0;
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
    uint32_t sample_interval_ms = 0;
    uint32_t cur_time = SrvOsCommon.get_os_ms();
    uint32_t start_tick = 0;
    uint32_t end_tick = 0;
    int32_t sample_tick = 0;

    if (obj && obj->statistic_imu && obj->init_state_reg.bit.imu && obj->statistic_imu->set_period)
    {
        sample_interval_ms = SrvSensorMonitor_GetSampleInterval(obj->statistic_imu->set_period);
        
        if ( SrvIMU.sample && \
            ((obj->statistic_imu->start_time == 0) || \
             (sample_interval_ms && \
             (cur_time >= obj->statistic_imu->nxt_sample_time))))
        {
            // DebugPin.ctl(Debug_PB5, true);
            start_tick = SrvOsCommon.get_systimer_current_tick();

            if (SrvIMU.sample(obj->IMU_SampleMode))
            { 
                end_tick = SrvOsCommon.get_systimer_current_tick();

                obj->statistic_imu->sample_cnt ++;
                obj->statistic_imu->nxt_sample_time = cur_time + sample_interval_ms;
                if (obj->statistic_imu->start_time == 0)
                    obj->statistic_imu->start_time = cur_time;

                state = true;
            }
            else
                end_tick = SrvOsCommon.get_systimer_current_tick();
            // DebugPin.ctl(Debug_PB5, false);

            if (state)
            {
                if (start_tick < end_tick)
                {
                    sample_tick = end_tick - start_tick;
                }
                else if (start_tick > end_tick)
                {
                    /* must after timer irq */
                    sample_tick = SrvOsCommon.get_systimer_period() - start_tick + end_tick;
                }

                if ((sample_tick <= obj->statistic_imu->min_sampling_overhead) || \
                   (obj->statistic_imu->min_sampling_overhead == 0))
                    obj->statistic_imu->min_sampling_overhead = sample_tick;

                if (sample_tick >= obj->statistic_imu->max_sampling_overhead)
                    obj->statistic_imu->max_sampling_overhead = sample_tick;

                obj->statistic_imu->avg_sampling_overhead += sample_tick;
                obj->statistic_imu->avg_sampling_overhead /= 2;

                obj->statistic_imu->cur_sampling_overhead = sample_tick;
            }
        }
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
    if (SrvBaro.init && obj)
    {
        obj->baro_err = SrvBaro.init();

        if (obj->baro_err == SrvBaro_Error_None)
        {
            MONITOR_INFO("Baro init accomplished\r\n");
            return true;
        }
        MONITOR_INFO("Baro init failed\r\n");
    }

    return false;
}

static bool SrvSensorMonitor_Baro_SampleCTL(SrvSensorMonitorObj_TypeDef *obj)
{
    bool state = false;
    uint32_t sample_interval_ms = 0;
    uint32_t cur_time = SrvOsCommon.get_os_ms();
    uint32_t start_tick = 0;
    uint32_t end_tick = 0;
    int32_t sample_tick = 0;

    if (obj && obj->init_state_reg.bit.baro && obj->freq_reg.bit.baro && obj->statistic_baro->set_period)
    {
        sample_interval_ms = SrvSensorMonitor_GetSampleInterval(obj->statistic_baro->set_period);
    
        if (SrvBaro.sample && \
            ((obj->statistic_baro->start_time == 0) || \
             (sample_interval_ms && \
             (cur_time >= obj->statistic_baro->nxt_sample_time))))
        {
            start_tick = SrvOsCommon.get_systimer_current_tick();

            // DebugPin.ctl(Debug_PB5, true);
            if (SrvBaro.sample())
            {
                end_tick = SrvOsCommon.get_systimer_current_tick();

                obj->statistic_baro->sample_cnt ++;
                obj->statistic_baro->nxt_sample_time = cur_time + sample_interval_ms;
                if (obj->statistic_baro->start_time == 0)
                    obj->statistic_baro->start_time = cur_time;

                state = true;
            }
            else
                end_tick = SrvOsCommon.get_systimer_current_tick();
            // DebugPin.ctl(Debug_PB5, false);
        
            if (state)
            {
                if (start_tick < end_tick)
                {
                    sample_tick = end_tick - start_tick;
                }
                else if (start_tick > end_tick)
                {
                    /* must after timer irq */
                    sample_tick = SrvOsCommon.get_systimer_period() - start_tick + end_tick;
                }

                if ((sample_tick <= obj->statistic_baro->min_sampling_overhead) || \
                    (obj->statistic_baro->min_sampling_overhead == 0))
                    obj->statistic_baro->min_sampling_overhead = sample_tick;

                if (sample_tick >= obj->statistic_baro->max_sampling_overhead)
                    obj->statistic_baro->max_sampling_overhead = sample_tick;

                obj->statistic_baro->avg_sampling_overhead += sample_tick;
                obj->statistic_baro->avg_sampling_overhead /= 2;

                obj->statistic_baro->cur_sampling_overhead = sample_tick;
            }
        }
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
/* maximum sampling rate is 1KHz currentlly */
static bool SrvSensorMonitor_SampleCTL(SrvSensorMonitorObj_TypeDef *obj)
{
    bool state = false;
    // DebugPin.ctl(Debug_PB5, true);
    
    /* imu single sampling overhead is about 60us */
    state |= SrvSensorMonitor_IMU_SampleCTL(obj);
    state |= SrvSensorMonitor_Mag_SampleCTL(obj);

    /* baro single sampling overhead is about 230us */
    state |= SrvSensorMonitor_Baro_SampleCTL(obj);

    // DebugPin.ctl(Debug_PB5, false);
    
    return state;
}




