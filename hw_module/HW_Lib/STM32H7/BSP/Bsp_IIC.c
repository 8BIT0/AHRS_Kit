#include "Bsp_IIC.h"

#define To_IIC_Handle_Ptr(x) ((I2C_HandleTypeDef *)x)
#define To_IIC_PeriphCLKInitType(x) ((RCC_PeriphCLKInitTypeDef *)x)

/* internal vriable */
static I2C_HandleTypeDef* BspIIC_HandleList[BspIIC_Instance_I2C_Sum] = {0};

/* external function */
static bool BspIIC_Init(BspIICObj_TypeDef *obj);
static bool BspIIC_DeInit(BspIICObj_TypeDef *obj);
static bool BspIIC_Read(BspIICObj_TypeDef *obj, uint16_t dev_addr, uint16_t reg, uint8_t *p_buf, uint16_t len);
static bool BspIIC_Write(BspIICObj_TypeDef *obj, uint16_t dev_addr, uint16_t reg, uint8_t *p_buf, uint16_t len);


BspIIC_TypeDef BspIIC = {
    .init    = BspIIC_Init,
    .de_init = BspIIC_DeInit,
    .read    = BspIIC_Read,
    .write   = BspIIC_Write,
};

static bool BspIIC_Hal_Cfg(void *hdl)
{
    I2C_HandleTypeDef *hi2c = To_IIC_Handle_Ptr(hdl);

    if (hi2c == NULL)
        return false;

    hi2c->Init.Timing = 0x00702991;//0x10909CEC;
    hi2c->Init.OwnAddress1 = 0;
    hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c->Init.OwnAddress2 = 0;
    hi2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(hi2c) != HAL_OK)
        return false;

    if (HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
        return false;

    if (HAL_I2CEx_ConfigDigitalFilter(hi2c, 0) != HAL_OK)
        return false;

    return true;
}

static bool BspIIC_Init(BspIICObj_TypeDef *obj)
{
    obj->init = false;
    BspGPIO_Obj_TypeDef sda_obj_tmp;
    BspGPIO_Obj_TypeDef sck_obj_tmp;

    memset(&sda_obj_tmp, 0, sizeof(sda_obj_tmp));
    memset(&sck_obj_tmp, 0, sizeof(sck_obj_tmp));

    if ((obj == NULL) || \
        (obj->Pin->port_sck == NULL) || \
        (obj->Pin->port_sda == NULL) || \
        (obj->PeriphClkInitStruct == NULL) || \
        (obj->handle == NULL))
        return false;

    sda_obj_tmp.alternate = obj->Pin->pin_Alternate;
    sck_obj_tmp.alternate = obj->Pin->pin_Alternate;

    sda_obj_tmp.pin = obj->Pin->pin_sda;
    sck_obj_tmp.pin = obj->Pin->pin_sck;

    sda_obj_tmp.port = obj->Pin->port_sda;
    sck_obj_tmp.port = obj->Pin->port_sck;

    BspGPIO.alt_init(sda_obj_tmp, GPIO_MODE_AF_OD);
    BspGPIO.alt_init(sck_obj_tmp, GPIO_MODE_AF_OD);

    switch((uint8_t)(obj->instance_id))
    {
        case BspIIC_Instance_I2C_1:
            if (obj->init && (BspIIC_HandleList[BspIIC_Instance_I2C_1] != NULL))
                return true;

            To_IIC_PeriphCLKInitType(obj->PeriphClkInitStruct)->PeriphClockSelection = RCC_PERIPHCLK_I2C1;
            To_IIC_PeriphCLKInitType(obj->PeriphClkInitStruct)->I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
            if (HAL_RCCEx_PeriphCLKConfig(To_IIC_PeriphCLKInitType(obj->PeriphClkInitStruct)) != HAL_OK)
                return false;

            __HAL_RCC_I2C1_CLK_ENABLE();
            To_IIC_Handle_Ptr(obj->handle)->Instance = I2C1;
            if (!BspIIC_Hal_Cfg(obj->handle))
                return false;

            /* I2C2 interrupt Init */
            HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);
            HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);

            BspIIC_HandleList[BspIIC_Instance_I2C_1] = To_IIC_Handle_Ptr(obj->handle);
            obj->init = true;
            return true;

        case BspIIC_Instance_I2C_2:
            if (obj->init && (BspIIC_HandleList[BspIIC_Instance_I2C_2] != NULL))
                return true;

            To_IIC_PeriphCLKInitType(obj->PeriphClkInitStruct)->PeriphClockSelection = RCC_PERIPHCLK_I2C2;
            To_IIC_PeriphCLKInitType(obj->PeriphClkInitStruct)->I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
            if (HAL_RCCEx_PeriphCLKConfig(To_IIC_PeriphCLKInitType(obj->PeriphClkInitStruct)) != HAL_OK)
                return false;

            __HAL_RCC_I2C2_CLK_ENABLE();
            To_IIC_Handle_Ptr(obj->handle)->Instance = I2C2;
            if (!BspIIC_Hal_Cfg(obj->handle))
                return false;

            /* I2C2 interrupt Init */
            HAL_NVIC_SetPriority(I2C2_ER_IRQn, 5, 0);
            HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);

            BspIIC_HandleList[BspIIC_Instance_I2C_2] = To_IIC_Handle_Ptr(obj->handle);
            obj->init = true;
            return true;

        default: return false;
    }
}

static bool BspIIC_DeInit(BspIICObj_TypeDef *obj)
{
    if ((obj == NULL) || (!obj->init))
        return false;
            
    HAL_GPIO_DeInit(obj->Pin->port_sck, obj->Pin->pin_sck);
    HAL_GPIO_DeInit(obj->Pin->port_sda, obj->Pin->pin_sda);
    
    switch(obj->instance_id)
    {
        case BspIIC_Instance_I2C_1:
            __HAL_RCC_I2C1_CLK_DISABLE();
            HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
            break;

        case BspIIC_Instance_I2C_2:
            __HAL_RCC_I2C2_CLK_DISABLE();
            HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);
            break;

        default: return false;
    }

    obj->init = false;
    return true;
}

static bool BspIIC_Read(BspIICObj_TypeDef *obj, uint16_t dev_addr, uint16_t reg, uint8_t *p_buf, uint16_t len)
{
    if ((obj == NULL) || (p_buf == NULL) || (len == 0) || \
        (HAL_I2C_Mem_Read((I2C_HandleTypeDef *)(obj->handle), dev_addr, reg, I2C_MEMADD_SIZE_8BIT, p_buf, len, 100) != HAL_OK))
        return false;
        
    return true;
}

static bool BspIIC_Write(BspIICObj_TypeDef *obj, uint16_t dev_addr, uint16_t reg, uint8_t *p_buf, uint16_t len)
{
    if ((obj == NULL) || (p_buf == NULL) || (len == 0) || \
        (HAL_I2C_Mem_Write((I2C_HandleTypeDef *)(obj->handle), dev_addr, reg, I2C_MEMADD_SIZE_8BIT, p_buf, len, 100) != HAL_OK))
        return false;

    return true;
}

void *BspIIC_Get_HandlePtr(BspIIC_Instance_List index)
{
    if (index < BspIIC_Instance_I2C_Sum)
        return BspIIC_HandleList[index];

    return NULL;
}

