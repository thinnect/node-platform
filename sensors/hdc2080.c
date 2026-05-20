/******************************************************************************
 * @file    hdc2080.c
 * @brief   Driver for TI HDC2080 temperature/humidity sensor (Trigger-on-Demand)
 * @author  Madis Uusjärv
 * Copyright Thinnect Inc. 2020
 * @license <PROPRIETARY>
 *****************************************************************************/

#include "hdc2080.h"
#include "retargeti2c.h"

/*** Local Macros ************************************************************/

#define HDC2080_I2C_ADDR            (0x40U)

/* Register Map */
#define HDC2080_REG_TEMP_LOW        (0x00U)
#define HDC2080_REG_TEMP_HIGH       (0x01U)
#define HDC2080_REG_HUM_LOW         (0x02U)
#define HDC2080_REG_HUM_HIGH        (0x03U)
#define HDC2080_REG_CONFIG          (0x0EU)
#define HDC2080_REG_MEASUREMENT     (0x0FU)

/* Bitfields */
#define HDC2080_CONFIG_MODE_BIT     (0x01U)   /* 0 = humidity, 1 = temperature */
#define HDC2080_CONFIG_TRIGGER_BIT  (0x01U << 7)

/*** Local Function Prototypes ***********************************************/

static int32_t hdc2080_write_reg (uint8_t reg, uint8_t value);
static int32_t hdc2080_read_reg8 (uint8_t reg, uint8_t* value);
static int32_t hdc2080_read_reg16 (uint8_t reg_low, uint16_t* result);

/*** Public API **************************************************************/

int32_t hdc2080_init (void)
{
    /* No special init needed except clearing config */
    return hdc2080_write_reg(HDC2080_REG_CONFIG, 0x00U);
}

int32_t hdc2080_trigger_measurement (void)
{
    /* Trigger-on-demand: set TRIGGER bit in MEASUREMENT register */
    return hdc2080_write_reg(HDC2080_REG_MEASUREMENT, HDC2080_CONFIG_TRIGGER_BIT);
}

int32_t hdc2080_read_temperature (float* temperature_c)
{
    uint16_t raw = 0U;
    int32_t status = hdc2080_read_reg16(HDC2080_REG_TEMP_LOW, &raw);

    if (status != 0)
    {
        return status;
    }

    /* Convert raw value to Celsius (per TI datasheet) */
    *temperature_c = ((float)raw) * 165.0f / 65536.0f - 40.0f;

    return 0;
}

int32_t hdc2080_read_humidity (float* humidity_rh)
{
    uint16_t raw = 0U;
    int32_t status = hdc2080_read_reg16(HDC2080_REG_HUM_LOW, &raw);

    if (status != 0)
    {
        return status;
    }

    /* Convert raw value to %RH */
    *humidity_rh = ((float)raw) * 100.0f / 65536.0f;

    return 0;
}

int32_t hdc2080_read_temp_hum (float* temperature_c, float* humidity_rh)
{
    int32_t status = hdc2080_read_temperature(temperature_c);

    if (status != 0)
    {
        return status;
    }

    return hdc2080_read_humidity(humidity_rh);
}

/*** Local Functions *********************************************************/

static int32_t hdc2080_write_reg (uint8_t reg, uint8_t value)
{
    uint8_t wr_data = value;

    return RETARGET_I2CWrite(HDC2080_I2C_ADDR, reg, &wr_data, 1U);
}

static int32_t hdc2080_read_reg8 (uint8_t reg, uint8_t* value)
{
    uint8_t wr_data = reg; // write register aadress where we start reading
    
    return RETARGET_I2CWriteRead(HDC2080_I2C_ADDR, &wr_data, 1U, value, 1U);
}

static int32_t hdc2080_read_reg16 (uint8_t reg_low, uint16_t* value)
{
    uint8_t low = 0U;
    uint8_t high = 0U;
    int32_t status = hdc2080_read_reg8(reg_low, &low);

    if (status != 0)
    {
        return status;
    }

    status = hdc2080_read_reg8(reg_low + 1U, &high);

    if (status != 0)
    {
        return status;
    }

    *value = ((uint16_t)high << 8) | low;

    return 0;
}
