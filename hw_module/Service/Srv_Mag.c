#include "Srv_mag.h"
#include "Srv_OsCommon.h"
#include "Dev_IST8310.h"
#include "Bsp_IIC.h"
#include "HW_Def.h"

SrvMagBusObj_TypeDef SrvMagBus = {
    .init = false,
    .obj  = NULL,
    .api  = NULL,
};

SrvMagObj_TypeDef SrvMagObj = {
    .api = NULL,
    .obj = NULL,
};

/* internal function */

/* external funtion */
static bool SrvMag_Init(void);

/* external variable */
SrvMag_TypeDef SrvMag = {
    .init = SrvMag_Init,
};

static bool SrvMag_Bus_Init(void)
{
    SrvMagBus.init = false;
    SrvMagBus.obj = (void *)&Mag_BusCfg;
    SrvMagBus.api = (void *)&BspIIC;

    ToIIC_BusObj(SrvMagBus.obj)->handle = SrvOsCommon.malloc(I2C_HandleType_Size);
    if (ToIIC_BusObj(SrvMagBus.obj)->handle == NULL)
    {
        SrvOsCommon.free(ToIIC_BusObj(SrvMagBus.obj)->handle);
        return false;
    }

    ToIIC_BusObj(SrvMagBus.obj)->PeriphClkInitStruct = SrvOsCommon.malloc(I2C_PeriphCLKInitType_Size);
    if ((ToIIC_BusObj(SrvMagBus.obj)->PeriphClkInitStruct == NULL) || \
        !ToIIC_BusAPI(SrvMagBus.api)->init(ToIIC_BusObj(SrvMagBus.obj)))
    {
        SrvOsCommon.free(ToIIC_BusObj(SrvMagBus.obj)->handle);
        SrvOsCommon.free(ToIIC_BusObj(SrvMagBus.obj)->PeriphClkInitStruct);
        return false;
    }

    SrvMagBus.init = true;
    return true;
}

static bool SrvMag_Init(void)
{
    /* mag module bus init */
    if (!SrvMag_Bus_Init())
        return false;

    /* mag sensor module init */
    SrvMagObj.obj = SrvOsCommon.malloc(sizeof(DevIST8310Obj_TypeDef));
    SrvMagObj.api = &DevIST8310;

    SrvMagObj.p_sensor_data = SrvOsCommon.malloc(IST8310_DataSize);
    SrvMagObj.data_size = IST8310_DataSize;

    if ((SrvMagObj.obj == NULL) || (SrvMagObj.p_sensor_data == NULL))
    {
        SrvOsCommon.free(SrvMagObj.obj);
        SrvOsCommon.free(SrvMagObj.p_sensor_data);
        return false;
    }

    ToIST8310_OBJ(SrvMagObj.obj)->bus_obj   = SrvMagBus.obj;
    ToIST8310_OBJ(SrvMagObj.obj)->bus_read  = (IST8310_Bus_Read)BspIIC.read;
    ToIST8310_OBJ(SrvMagObj.obj)->bus_write = (IST8310_Bus_Write)BspIIC.write;
    ToIST8310_OBJ(SrvMagObj.obj)->get_tick  = SrvOsCommon.get_os_ms;
    ToIST8310_OBJ(SrvMagObj.obj)->delay     = SrvOsCommon.delay_ms;

    /* device init */
    if (!ToIST8310_API(SrvMagObj.api)->init(ToIST8310_OBJ(SrvMagObj.obj)))
    {
        SrvOsCommon.free(SrvMagObj.obj);
        SrvOsCommon.free(SrvMagObj.p_sensor_data);
        return false;
    }

    return true;
}




