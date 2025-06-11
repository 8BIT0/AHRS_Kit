#include "Task_Manager.h"
#include "Task_Sample.h"
#include "debug_util.h"
#include "HW_Def.h"
#include "Dev_Led.h"
#include "Srv_OsCommon.h"
#include "cmsis_os.h"

#define TaskSample_Period_Def    1  /* unit: ms period 1ms  1000Hz */
#define TaskControl_Period_Def   5  /* unit: ms period 5ms  200Hz  */
#define TaskTelemetry_Period_def 2  /* unit: ms period 2ms  500Hz  */
#define TaslLog_Period_Def       5  /* unit: ms period 5ms  200Hz  */
#define TaslNavi_Period_Def      10 /* unit: ms period 10ms 100Hz  */
#define TaskFrameCTL_Period_Def  5  /* unit: ms period 5ms  200Hz  */

osThreadId TaskSample_Handle = NULL;
osThreadId TaskManager_Handle = NULL;

#define SYS_TAG "[ HARDWARE INFO ] "
#define SYS_INFO(fmt, ...) Debug_Print(&DebugPort, SYS_TAG, fmt, ##__VA_ARGS__)

void Task_Manager_Init(void)
{
    DevLED.init(Led1);
    DevLED.init(Led2);
    DevLED.init(Led3);

    DebugPin.init(Debug_PC0);
    DebugPin.init(Debug_PC1);
    DebugPin.init(Debug_PC2);
    DebugPin.init(Debug_PC3);
    DebugPin.init(Debug_PB3);
    DebugPin.init(Debug_PB4);
    DebugPin.init(Debug_PB5);
    DebugPin.init(Debug_PB6);
    DebugPin.init(Debug_PB10);
    /* vol ADC init */

    /* cur ADC init */
    
    SrvOsCommon.init();

    osThreadDef(ManagerTask, Task_Manager_CreateTask, osPriorityLow, 0, 1024);
    TaskManager_Handle = osThreadCreate(osThread(ManagerTask), NULL);

    osKernelStart();
}

void Task_Manager_CreateTask(void const *arg)
{
    bool init = false;

    DebugPort.free = SrvOsCommon.free;
    DebugPort.malloc = SrvOsCommon.malloc;
    Debug_Port_Init(&DebugPort);

    SYS_INFO("%s\r\n", Select_Hardware);
    SYS_INFO("Hardware Version %d.%d.%d\r\n", HWVer[0], HWVer[1], HWVer[2]);

    while(1)
    {
        if (!init)
        {
            uint32_t sys_time = SrvOsCommon.get_os_ms();
            DEBUG_INFO("Sys Start\r\n");
            DEBUG_INFO("Sys Time: %d\r\n", sys_time);

            TaskSample_Init(TaskSample_Period_Def);

            osThreadDef(SampleTask, TaskSample_Core, osPriorityRealtime, 0, 1024);
            TaskSample_Handle = osThreadCreate(osThread(SampleTask), NULL);

            init = true;
        }

        /* run system statistic in this task */
        osDelay(10);
    }
}
