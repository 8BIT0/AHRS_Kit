#include "Task_Protocol.h"
#include "Srv_OsCommon.h"
#include "HW_Def.h"
#include "Srv_ComProto.h"
#include "debug_util.h"

#define PROTO_STREAM_BUF_SIZE (1024 + 128)
#define SLIENT_TIMEOUT 200              /* unit: ms 200ms */
#define TUNNING_TIMEOUT 2000            /* unit: ms 1S */

#define CLI_FUNC_BUF_SIZE 512

/* internal variable */

/* MAVLink message List */
SrvComProto_MsgInfo_TypeDef TaskProto_MAV_Raw_IMU1;
SrvComProto_MsgInfo_TypeDef TaskProto_MAV_Raw_IMU2;
SrvComProto_MsgInfo_TypeDef TaskProto_MAV_Altitude;

static bool FrameCTL_MavProto_Enable = false;
static FrameCTL_PortMonitor_TypeDef PortMonitor = {.init = false};
static uint32_t FrameCTL_Period = 0;
static __attribute__((section(".Perph_Section"))) uint8_t MavShareBuf[1024];
static uint8_t USB_RxBuf_Tmp[PROTO_STREAM_BUF_SIZE];
static uint32_t USB_VCP_Addr = 0;

static SrvComProto_Stream_TypeDef USBRx_Stream = {
    .p_buf = USB_RxBuf_Tmp,
    .size = 0,
    .max_size = sizeof(USB_RxBuf_Tmp),
};

static SrvComProto_Stream_TypeDef MavStream = {
    .p_buf = MavShareBuf,
    .size = 0,
    .max_size = sizeof(MavShareBuf),
};

/* frame section */
static void TaskFrameCTL_PortFrameOut_Process(void);
static void TaskFrameCTL_MavMsg_Trans(FrameCTL_Monitor_TypeDef *Obj, uint8_t *p_data, uint16_t size);

/* default vcp port section */
static void TaskFrameCTL_DefaultPort_Init(FrameCTL_PortMonitor_TypeDef *monitor);
static bool TaskFrameCTL_MAV_Msg_Init(void);
static void TaskFrameCTL_Port_Rx_Callback(uint32_t RecObj_addr, uint8_t *p_data, uint16_t size);
static void TaskFrameCTL_Port_TxCplt_Callback(uint32_t RecObj_addr, uint8_t *p_data, uint32_t *size);
static void TaskFrameCTL_USB_VCP_Connect_Callback(uint32_t Obj_addr, uint32_t *time_stamp);
static bool TaskFrameCTL_DefaultPort_Trans(uint8_t *p_data, uint16_t size);
static void TaskFrameCTL_ConfigureStateCheck(void);

void TaskFrameCTL_Init(uint32_t period)
{
    FrameCTL_Period = FrameCTL_MAX_Period;

    /* USB VCP as defaut port to tune parameter and frame porotcol */
    memset(&PortMonitor, 0, sizeof(PortMonitor));
    
    TaskFrameCTL_DefaultPort_Init(&PortMonitor);

    PortMonitor.init = true;
    
    /* init radio protocol*/
    FrameCTL_MavProto_Enable = TaskFrameCTL_MAV_Msg_Init();

    if (period && (period <= FrameCTL_MAX_Period))
        FrameCTL_Period = period;

    SrvOsCommon.delay_ms(50);
}

void TaskFrameCTL_Core(void const *arg)
{
    uint32_t per_time = SrvOsCommon.get_os_ms();

    while(1)
    {
        TaskFrameCTL_ConfigureStateCheck();

        /* frame protocol process */
        TaskFrameCTL_PortFrameOut_Process();

        SrvOsCommon.precise_delay(&per_time, FrameCTL_Period);
    }
}

/************************************** radio section ********************************************/
static void TaskFrameCTL_DefaultPort_Init(FrameCTL_PortMonitor_TypeDef *monitor)
{
    if (monitor == NULL)
        return;
    
    if(BspUSB_VCP.init((uint32_t)&(monitor->VCP_Port.RecObj)) != BspUSB_Error_None)
    {
        /* init default port VCP first */
        monitor->VCP_Port.init_state = false;
        return;
    }
    
    monitor->VCP_Port.init_state = true;

    /* create USB VCP Tx semaphore */
    osSemaphoreDef(DefaultPort_Tx);
    monitor->VCP_Port.p_tx_semphr = osSemaphoreCreate(osSemaphore(DefaultPort_Tx), 1);

    if(monitor->VCP_Port.p_tx_semphr == NULL)
    {
        monitor->VCP_Port.init_state = false;
        return;
    }

    BspUSB_VCP.set_tx_cpl_callback(TaskFrameCTL_Port_TxCplt_Callback);
    BspUSB_VCP.set_rx_callback(TaskFrameCTL_Port_Rx_Callback);
    BspUSB_VCP.set_connect_callback(TaskFrameCTL_USB_VCP_Connect_Callback);

    monitor->VCP_Port.RecObj.PortObj_addr = (uint32_t)&(monitor->VCP_Port);
    monitor->VCP_Port.RecObj.type = Port_USB;

    USB_VCP_Addr = (uint32_t)&(monitor->VCP_Port);
}

static bool TaskFrameCTL_DefaultPort_Trans(uint8_t *p_data, uint16_t size)
{
    bool state = false;

    /* when attach to host device then send data */
    if (PortMonitor.VCP_Port.init_state && \
        PortMonitor.vcp_connect_state && \
        PortMonitor.VCP_Port.p_tx_semphr && \
        p_data && size)
    {
        state = true;
        osSemaphoreWait(PortMonitor.VCP_Port.p_tx_semphr, 0);
        if ((BspUSB_VCP.send(p_data, size) != BspUSB_Error_None) || \
            (osSemaphoreWait(PortMonitor.VCP_Port.p_tx_semphr, FrameCTL_Port_Tx_TimeOut) != osOK))
            state = false;
    }

    return state;
}

/************************************** receive process callback section *************************/
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

static void TaskFrameCTL_Port_TxCplt_Callback(uint32_t Obj_addr, uint8_t *p_data, uint32_t *size)
{
    UNUSED(p_data);
    UNUSED(size);

    FrameCTL_PortProtoObj_TypeDef *p_Obj = NULL;
    FrameCTL_VCPPortMonitor_TypeDef *p_USBPortObj = NULL;
    osSemaphoreId semID = NULL;
    uint32_t *p_rls_err_cnt = NULL;

    if(Obj_addr)
    {
        p_Obj = (FrameCTL_PortProtoObj_TypeDef *)Obj_addr;

        if(p_Obj->PortObj_addr)
        {
            p_USBPortObj = (FrameCTL_VCPPortMonitor_TypeDef *)(p_Obj->PortObj_addr);

            if(p_USBPortObj->init_state && p_USBPortObj->p_tx_semphr)
            {
                semID = p_USBPortObj->p_tx_semphr;
                p_rls_err_cnt = &p_USBPortObj->tx_semphr_rls_err;
            }
            
            if(semID && p_rls_err_cnt && (osSemaphoreRelease(semID) != osOK))
                (*p_rls_err_cnt) ++;
        }
    }
}

/************************************** USB Only Callback section ********************************************/
/*
 * use usb sof interrupt
 * as long as usb port attach to computer or other host device
 * flight controller will receive sof interrupt
 * if flight controller continue to receive this interrupt it must be attach to computer
 */
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

/************************************** frame protocol section ********************************************/
static bool TaskFrameCTL_MAV_Msg_Init(void)
{
    SrvComProto_MavPackInfo_TypeDef PckInfo;
    /* create mavlink output message object */
    memset(&PckInfo, 0, sizeof(PckInfo));
    memset(&TaskProto_MAV_Raw_IMU1, 0, sizeof(TaskProto_MAV_Raw_IMU1));
    memset(&TaskProto_MAV_Raw_IMU2, 0, sizeof(TaskProto_MAV_Raw_IMU2));

    // period 10Ms 100Hz
    PckInfo.system_id = MAV_SysID_Drone;
    PckInfo.component_id = MAV_CompoID_Raw_IMU1;
    PckInfo.chan = 0;
    SrvComProto.mav_msg_obj_init(&TaskProto_MAV_Raw_IMU1, PckInfo, 10);
    SrvComProto.mav_msg_enable_ctl(&TaskProto_MAV_Raw_IMU1, false);
            
    // period 10Ms 100Hz
    PckInfo.system_id = MAV_SysID_Drone;
    PckInfo.component_id = MAV_CompoID_Raw_IMU2;
    PckInfo.chan = 0;
    SrvComProto.mav_msg_obj_init(&TaskProto_MAV_Raw_IMU2, PckInfo, 10);
    SrvComProto.mav_msg_enable_ctl(&TaskProto_MAV_Raw_IMU2, true);
    
    // period 100MS 10Hz
    PckInfo.system_id = MAV_SysID_Drone;
    PckInfo.component_id = MAV_CompoID_Altitude;
    PckInfo.chan = 0;
    SrvComProto.mav_msg_obj_init(&TaskProto_MAV_Altitude, PckInfo, 100);
    SrvComProto.mav_msg_enable_ctl(&TaskProto_MAV_Altitude, true);
    return true;
}

static void TaskFrameCTL_PortFrameOut_Process(void)
{
    FrameCTL_Monitor_TypeDef proto_monitor;
    void *proto_arg = (void *)&proto_monitor;

    proto_monitor.frame_type = ComFrame_MavMsg;

    /* when attach to configrator then disable radio port trans use default port trans mav data */
    /* check other port init state */
    if (PortMonitor.VCP_Port.init_state)
    {
        /* Proto mavlink message through default port */
        proto_monitor.port_type = Port_USB;
        proto_monitor.port_addr = USB_VCP_Addr;
        SrvComProto.mav_msg_stream(&TaskProto_MAV_Raw_IMU1,     &MavStream, proto_arg, (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
        SrvComProto.mav_msg_stream(&TaskProto_MAV_Raw_IMU2,     &MavStream, proto_arg, (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
        SrvComProto.mav_msg_stream(&TaskProto_MAV_Altitude,     &MavStream, proto_arg, (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
    }
}

static void TaskFrameCTL_MavMsg_Trans(FrameCTL_Monitor_TypeDef *Obj, uint8_t *p_data, uint16_t size)
{
    if(Obj && (Obj->frame_type == ComFrame_MavMsg) && Obj->port_addr && p_data && size)
        TaskFrameCTL_DefaultPort_Trans(p_data, size);
}

static void TaskFrameCTL_ConfigureStateCheck(void)
{
    uint32_t cur_time = SrvOsCommon.get_os_ms();
    PortMonitor.vcp_connect_state = false;

    /* check usb vcp attach state */
    if (BspUSB_VCP.check_connect)
        PortMonitor.vcp_connect_state = BspUSB_VCP.check_connect(cur_time, FrameCTL_Period);
}

