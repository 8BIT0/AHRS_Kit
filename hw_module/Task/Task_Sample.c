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
static bool port_attach = false;
static SrvSensorMonitorObj_TypeDef SensorMonitor;

/* internal function */
static void TaskInertical_Blink_Notification(uint16_t duration);
static void USBPort_Init(void);
static void USBPort_TxCplt_Callback(uint32_t obj_addr, uint8_t *p_data, uint32_t *size);
static void USBPort_Connect_Callback(uint32_t Obj_addr, uint32_t *time_stamp);

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
    USBPort_Init();
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
    BspUSB_VCP.set_rx_callback(NULL);
    BspUSB_VCP.set_connect_callback(USBPort_Connect_Callback);
}

static void USBPort_TxCplt_Callback(uint32_t obj_addr, uint8_t *p_data, uint32_t *size)
{
    UNUSED(p_data);
    UNUSED(size);

    if (obj_addr == 0)
        return;
    
    osSemaphoreId semID = (osSemaphoreId)obj_addr;
    osSemaphoreRelease(semID);
}

static bool USBPort_Trans(uint8_t *p_data, uint16_t size)
{
    bool state = false;

    /* when attach to host device then send data */
    if (USBTx_Sem && p_data && size)
    {
        state = true;
        osSemaphoreWait(USBTx_Sem, 0);
        if ((BspUSB_VCP.send(p_data, size) != BspUSB_Error_None) || \
            (osSemaphoreWait(USBTx_Sem, 100) != osOK))
            state = false;
    }

    return state;
}

static void USBPort_Connect_Callback(uint32_t Obj_addr, uint32_t *time_stamp)
{
    UNUSED(Obj_addr);
    UNUSED(time_stamp);

    if (!port_attach)
        port_attach = true;
}


//  mavlink_msg_scaled_imu_pack_chan