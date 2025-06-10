#include "Task_Sample.h"
#include "debug_util.h"
#include "Srv_OsCommon.h"
#include "HW_Def.h"
#include "error_log.h"
#include "../FCHW_Config.h"
#include "Srv_SensorMonitor.h"
#include "mavlink.h"
#include "Bsp_USB.h"

#define DATAPIPE_TRANS_TIMEOUT_100Ms 100

/* internal var */
static osSemaphoreId USBTx_Sem = NULL;
static uint32_t TaskSample_Period = 0;
static bool sample_enable = false;
static SrvSensorMonitorObj_TypeDef SensorMonitor;

/* internal function */
static void TaskInertical_Blink_Notification(uint16_t duration);
static void USBPort_Init(void);
static void USBPort_TxCplt_Callback(uint32_t obj_addr, uint8_t *p_data, uint32_t *size);

/* external function */

void TaskSample_Init(uint32_t period)
{
    SrvSensorMonitor_IMURange_TypeDef PriIMU_Range;
    SrvSensorMonitor_IMURange_TypeDef SecIMU_Range;

    memset(&PriIMU_Range, 0, sizeof(SrvSensorMonitor_IMURange_TypeDef));
    memset(&SecIMU_Range, 0, sizeof(SrvSensorMonitor_IMURange_TypeDef));
    memset(&SensorMonitor, 0, sizeof(SrvSensorMonitorObj_TypeDef));
    
    SensorMonitor.freq_reg.bit.imu = SrvSensorMonitor_SampleFreq_1KHz;
    SensorMonitor.freq_reg.bit.baro = SrvSensorMonitor_SampleFreq_50Hz;
    sample_enable = SrvSensorMonitor.init(&SensorMonitor);

    /* force make sensor sample task run as 1khz freq */
    TaskSample_Period = 1;

    /* usb port init */
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

/********************************************** port funtion ****************************************/
static void USBPort_Init(void)
{    
    osSemaphoreDef(USB_Tx);
    USBTx_Sem = osSemaphoreCreate(osSemaphore(USB_Tx), 1);

    if(BspUSB_VCP.init((uint32_t)USBTx_Sem) != BspUSB_Error_None)
        return;

    BspUSB_VCP.set_tx_cpl_callback(USBPort_TxCplt_Callback);
    BspUSB_VCP.set_rx_callback();
    BspUSB_VCP.set_connect_callback();
}

static void USBPort_TxCplt_Callback(uint32_t obj_addr, uint8_t *p_data, uint32_t *size)
{
    UNUSED(p_data);
    UNUSED(size);

    if (obj_addr == NULL)
        return;
    
    osSemaphoreId semID = (osSemaphoreId)obj_addr;
    osSemaphoreRelease(semID);
}

/************* bug *****************/
static void TaskFrameCTL_Port_Rx_Callback(uint32_t RecObj_addr, uint8_t *p_data, uint16_t size)
{
    SrvComProto_Stream_TypeDef *p_stream = NULL;
    FrameCTL_PortProtoObj_TypeDef *p_RecObj = NULL;

    /* use mavlink protocol tuning the flight parameter */
    if(p_data && size && RecObj_addr)
    {
        p_RecObj = (FrameCTL_PortProtoObj_TypeDef *)RecObj_addr;
        p_RecObj->time_stamp = SrvOsCommon.get_os_ms();

        p_stream = &USBRx_Stream;
        TaskFrameCTL_DefaultPort_Trans(p_data, size);

        if ((p_stream->size + size) <= p_stream->max_size)
        {
            memcpy(p_stream->p_buf + p_stream->size, p_data, size);
            p_stream->size += size;
        }
        else
        {
            memset(p_stream->p_buf, 0, p_stream->size);
            p_stream->size = 0;
            return;
        }
    }
}

static void TaskFrameCTL_USB_VCP_Connect_Callback(uint32_t Obj_addr, uint32_t *time_stamp)
{
    FrameCTL_PortProtoObj_TypeDef *p_Obj = NULL;

    if (Obj_addr)
    {
        p_Obj = (FrameCTL_PortProtoObj_TypeDef *)Obj_addr;

        if (p_Obj->PortObj_addr && (p_Obj->type == Port_USB))
            *time_stamp = SrvOsCommon.get_os_ms();
    }
}

