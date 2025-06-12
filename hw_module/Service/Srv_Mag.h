#ifndef __SRV_MAG_H
#define __SRV_MAG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "mag_data.h"

typedef struct
{
    bool init;
    void *api;
    void *obj;
} SrvMagBusObj_TypeDef;

typedef struct
{
    uint32_t time_stamp;
    float mag[Mag_Axis_Sum];
} SrvMagData_TypeDef;

typedef struct
{
    void *obj;
    void *api;
    uint8_t *p_sensor_data;
    uint16_t data_size;

    bool ready;
    SrvMagData_TypeDef data;
    uint32_t sample_cnt;
    uint32_t sample_err_cnt;
} SrvMagObj_TypeDef;

typedef struct
{
    bool (*init)(void);
} SrvMag_TypeDef;

extern SrvMag_TypeDef SrvMag;

#ifdef __cplusplus
}
#endif

#endif
