#include "Task_Sample.h"
#include "debug_util.h"
#include "Srv_OsCommon.h"
#include "HW_Def.h"
#include "error_log.h"
#include "../FCHW_Config.h"
#include "../System/DataPipe/DataPipe.h"
#include "Srv_SensorMonitor.h"
#include "shell_port.h"

#define ToBoolStr(x) x ? "true" : "false"
#define DATAPIPE_TRANS_TIMEOUT_100Ms 100

/* internal var */
static uint32_t TaskSample_Period = 0;
static bool sample_enable = false;
static SrvSensorMonitorObj_TypeDef SensorMonitor;

/* internal function */
static void TaskInertical_Blink_Notification(uint16_t duration);

/* external function */

void TaskSample_Init(uint32_t period)
{
    SrvSensorMonitor_IMURange_TypeDef PriIMU_Range;
    SrvSensorMonitor_IMURange_TypeDef SecIMU_Range;

    memset(&PriIMU_Range, 0, sizeof(SrvSensorMonitor_IMURange_TypeDef));
    memset(&SecIMU_Range, 0, sizeof(SrvSensorMonitor_IMURange_TypeDef));
    memset(&SensorMonitor, 0, sizeof(SrvSensorMonitorObj_TypeDef));
    
    SensorMonitor.enabled_reg.bit.imu = true;
    SensorMonitor.freq_reg.bit.imu = SrvSensorMonitor_SampleFreq_1KHz;
    SensorMonitor.enabled_reg.bit.baro = true;
    SensorMonitor.freq_reg.bit.baro = SrvSensorMonitor_SampleFreq_50Hz;
    sample_enable = SrvSensorMonitor.init(&SensorMonitor);

    /* force make sensor sample task run as 1khz freq */
    TaskSample_Period = 1;
}

void TaskSample_Core(void const *arg)
{
    uint32_t sys_time = SrvOsCommon.get_os_ms();
    
    while(1)
    {
        TaskInertical_Blink_Notification(100);

        if (sample_enable)
            SrvSensorMonitor.sample_ctl(&SensorMonitor);
        
        SrvOsCommon.precise_delay(&sys_time, TaskSample_Period);
    }
}

static void TaskInertical_Blink_Notification(uint16_t duration)
{
    uint32_t Rt = 0;
    static uint32_t Lst_Rt = 0;
    static bool led_state = false;

    Rt = SrvOsCommon.get_os_ms();

    if ((Rt % duration == 0) && (Lst_Rt != Rt))
    {
        led_state = !led_state;
        Lst_Rt = Rt;
    }

    /* test code */
    // DevLED.ctl(Sample_Blinkly, led_state);
    /* test code */
}

