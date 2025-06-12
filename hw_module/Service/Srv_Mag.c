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
static bool SrvMag_Bus_Init(void);

/* external funtion */
static bool SrvMag_Init(void);
static bool SrvMag_Sample(void);
static bool SrvMag_GetData(SrvMagData_TypeDef *p_data);

/* external variable */
SrvMag_TypeDef SrvMag = {
    .init = SrvMag_Init,
    .sample = SrvMag_Sample,
    .get_data = SrvMag_GetData,
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

    SrvMagObj.init = false;

    /* mag sensor module init */
    SrvMagObj.obj = SrvOsCommon.malloc(sizeof(DevIST8310Obj_TypeDef));
    SrvMagObj.api = &DevIST8310;

    if (SrvMagObj.obj == NULL)
    {
        SrvOsCommon.free(SrvMagObj.obj);
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
        return false;
    }

    SrvMagObj.init = true;
    return true;
}

static bool SrvMag_Sample(void)
{
    MagData_TypeDef tmp;

    memset(&tmp, 0, sizeof(MagData_TypeDef));
    if (!SrvMagObj.init || (ToIST8310_API(SrvMagObj.api)->sample == NULL) || \
        !ToIST8310_API(SrvMagObj.api)->sample(ToIST8310_OBJ(SrvMagObj.obj)) || \
        !ToIST8310_API(SrvMagObj.api)->get(ToIST8310_OBJ(SrvMagObj.obj), &tmp))
        return false;

    SrvMagObj.data.time_stamp = tmp.time_stamp;
    for (uint8_t i = Mag_Axis_X; i < Mag_Axis_Sum; i++)
    {
        SrvMagObj.data.mag[i] = tmp.mag[i];
    }

    return true;
}

static bool SrvMag_GetData(SrvMagData_TypeDef *p_data)
{
    if (!SrvMagObj.init || (p_data == NULL))
        return false;

    memcpy(p_data, &SrvMagObj.data, sizeof(SrvMagData_TypeDef));
    return true;
}

