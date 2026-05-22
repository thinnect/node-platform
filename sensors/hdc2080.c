/******************************************************************************
 * @file    hdc2080.c
 * @brief   Driver for TI HDC2080 temperature/humidity sensor (Trigger-on-Demand)
 * @author  Madis Uusjärv
 * Copyright Thinnect Inc. 2026
 * @license <PROPRIETARY>
 *****************************************************************************/
#include <stdint.h>
#include <inttypes.h>

#include <math.h>
#include "retargeti2c.h"
#include "cmsis_os2_ext.h"
#include "hdc2080.h"

#include "loglevels.h"
#define __MODUUL__ "hdc2080"
#define __LOG_LEVEL__ (LOG_LEVEL_hdc2080 & BASE_LOG_LEVEL)
#include "log.h"

/*** Local Macros ************************************************************/

#define HDC2080_I2C_ADDR              (0x40U)

/* Register Map */
#define HDC2080_REG_TEMP_LOW          (0x00)
#define HDC2080_REG_TEMP_HIGH         (0x01)
#define HDC2080_REG_HUM_LOW           (0x02)
#define HDC2080_REG_HUM_HIGH          (0x03)
#define HDC2080_REG_TEMP_OFFSET       (0x08)
#define HDC2080_REG_CONFIG            (0x0E)
#define HDC2080_REG_MEASUREMENT       (0x0F)
#define HDC2080_REG_MANUFACTURER_LOW  (0xFC)
#define HDC2080_REG_MANUFACTURER_HIGH (0xFD)

/* Bitfields */
#define HDC2080_CONFIG_TRIGGER_BIT    (0x01)

/* Temperature Offset Adjustment Register (0x08)
 * Reference: HDC2080 Datasheet Table 8-24
 */
#define HDC2080_BIT_TEMP_OFFSET_MINUS_2062 (0x01U << 7)  // –20.62 °C
#define HDC2080_BIT_TEMP_OFFSET_PLUS_1032  (0x01U << 6)  // +10.32 °C
#define HDC2080_BIT_TEMP_OFFSET_PLUS_0516  (0x01U << 5)  // +5.16 °C
#define HDC2080_BIT_TEMP_OFFSET_PLUS_0258  (0x01U << 4)  // +2.58 °C
#define HDC2080_BIT_TEMP_OFFSET_PLUS_0128  (0x01U << 3)  // +1.28 °C
#define HDC2080_BIT_TEMP_OFFSET_PLUS_0064  (0x01U << 2)  // +0.64 °C
#define HDC2080_BIT_TEMP_OFFSET_PLUS_0032  (0x01U << 1)  // +0.32 °C
#define HDC2080_BIT_TEMP_OFFSET_PLUS_0016  (0x01U << 0)  // +0.16 °C

/*** Local Function Prototypes ***********************************************/

static int8_t hdc2080_i2c_read(uint8_t reg, uint8_t* p_value, uint8_t count);
static int8_t hdc2080_i2c_write_data(uint8_t reg, uint8_t value);
static int8_t hdc2080_i2c_write_addr(uint8_t addr);

/*** Public API **************************************************************/

int8_t hdc2080_init (void)
{
    /* No special init needed except clearing config */
    return hdc2080_i2c_write_data(HDC2080_REG_CONFIG, 0x00U);
}

int8_t hdc2080_trigger_measurement (void)
{
    /* Trigger-on-demand: set TRIGGER bit in MEASUREMENT register */
    return hdc2080_i2c_write_data(HDC2080_REG_MEASUREMENT, HDC2080_CONFIG_TRIGGER_BIT);
}

int8_t hdc2080_read_manufacturer_id (uint16_t* p_manufacturer_id)
{
    uint8_t low;
    uint8_t high;
    int32_t res;

    res = hdc2080_i2c_read(HDC2080_REG_MANUFACTURER_HIGH, &high, 1);
    if (res != 0)
    {
        return res;
    }

    res = hdc2080_i2c_read(HDC2080_REG_MANUFACTURER_LOW, &low, 1);
    *p_manufacturer_id = (high << 8) | low;
    return res;
}

int8_t hdc2080_read_temperature (float* temperature_c)
{
    uint16_t raw = 0U;
    int32_t status;
    
    hdc2080_trigger_measurement();
    osDelay(10); // wait for measurement
    status = hdc2080_i2c_read(HDC2080_REG_TEMP_LOW, (uint8_t*)&raw, 2);

    if (status != 0)
    {
        return status;
    }

    /* Convert raw value to Celsius (per TI datasheet) */
    *temperature_c = ((float)raw) * 165.0f / 65536.0f - 40.0f;

    return 0;
}

int8_t hdc2080_read_humidity (float* humidity_rh)
{
    uint16_t raw = 0U;
    int32_t status; 

    hdc2080_trigger_measurement();
    osDelay(10); // wait for measurement
    status = hdc2080_i2c_read(HDC2080_REG_HUM_LOW, (uint8_t*)&raw, 2);

    if (status != 0)
    {
        return status;
    }

    /* Convert raw value to %RH */
    *humidity_rh = ((float)raw) * 100.0f / 65536.0f;

    return 0;
}

int8_t hdc2080_read_temp_hum (float* temperature_c, float* humidity_rh)
{
    int32_t status = hdc2080_read_temperature(temperature_c);

    if (status != 0)
    {
        return status;
    }

    return hdc2080_read_humidity(humidity_rh);
}

int8_t hdc2080_set_temperature_offset (uint8_t offset_bits)
{
    return hdc2080_i2c_write_data(HDC2080_REG_TEMP_OFFSET, offset_bits);
}

int8_t hdc2080_get_temperature_offset (uint8_t* offset_bits)
{
    return hdc2080_i2c_read(HDC2080_REG_TEMP_OFFSET, offset_bits, 1);
}

float hdc2080_offset_bits_to_celsius(uint8_t bits)
{
    float offset;

    offset = 0.0f;

    if (bits & HDC2080_BIT_TEMP_OFFSET_MINUS_2062)
    {
        offset -= 20.62f;
    }
    if (bits & HDC2080_BIT_TEMP_OFFSET_PLUS_1032)
    {
        offset += 10.32f;
    }
    if (bits & HDC2080_BIT_TEMP_OFFSET_PLUS_0516)
    {
        offset += 5.16f;
    }
    if (bits & HDC2080_BIT_TEMP_OFFSET_PLUS_0258)
    {
        offset += 2.58f;
    }
    if (bits & HDC2080_BIT_TEMP_OFFSET_PLUS_0128)
    {
        offset += 1.28f;
    }
    if (bits & HDC2080_BIT_TEMP_OFFSET_PLUS_0064)
    {
        offset += 0.64f;
    }
    if (bits & HDC2080_BIT_TEMP_OFFSET_PLUS_0032)
    {
        offset += 0.32f;
    }
    if (bits & HDC2080_BIT_TEMP_OFFSET_PLUS_0016)
    {
        offset += 0.16f;
    }

    return offset;
}

uint8_t hdc2080_celsius_to_offset_bits(float offset_c)
{
    /* Table 8-24 values, sorted from largest magnitude to smallest */
    const float values[8] =
    {
        -20.62f,  /* bit 7 */
        +10.32f,  /* bit 6 */
        +5.16f,   /* bit 5 */
        +2.58f,   /* bit 4 */
        +1.28f,   /* bit 3 */
        +0.64f,   /* bit 2 */
        +0.32f,   /* bit 1 */
        +0.16f    /* bit 0 */
    };

    const uint8_t bits[8] =
    {
        HDC2080_BIT_TEMP_OFFSET_MINUS_2062,
        HDC2080_BIT_TEMP_OFFSET_PLUS_1032,
        HDC2080_BIT_TEMP_OFFSET_PLUS_0516,
        HDC2080_BIT_TEMP_OFFSET_PLUS_0258,
        HDC2080_BIT_TEMP_OFFSET_PLUS_0128,
        HDC2080_BIT_TEMP_OFFSET_PLUS_0064,
        HDC2080_BIT_TEMP_OFFSET_PLUS_0032,
        HDC2080_BIT_TEMP_OFFSET_PLUS_0016
    };

    float remaining;
    uint8_t mask;
    int i;

    remaining = offset_c;
    mask = 0U;

    /* Special case: if offset is very negative, only -20.62 is possible */
    if (offset_c <= -20.62f)
    {
        return HDC2080_BIT_TEMP_OFFSET_MINUS_2062;
    }

    /* Do NOT use -20.62 for positive offsets */
    if (offset_c > 0.0f)
    {
        for (i = 1; i < 8; i++)
        {
            if (remaining >= values[i])
            {
                mask |= bits[i];
                remaining -= values[i];
            }
        }

        return mask;
    }

    /* Negative offset but not below -20.62:
     * Try combining -20.62 with positive bits to approximate target.
     */
    {
        float best_error;
        uint8_t best_mask;

        best_error = 1000.0f;
        best_mask = 0U;

        /* Option 1: use only positive bits (for small negative offsets) */
        {
            float rem;
            uint8_t m;

            rem = offset_c;
            m = 0U;

            for (i = 1; i < 8; i++)
            {
                if (rem >= values[i])
                {
                    m |= bits[i];
                    rem -= values[i];
                }
            }

            if (fabsf(rem) < fabsf(best_error))
            {
                best_error = rem;
                best_mask = m;
            }
        }

        /* Option 2: use -20.62 plus positive bits */
        {
            float rem;
            uint8_t m;

            rem = offset_c + 20.62f;
            m = HDC2080_BIT_TEMP_OFFSET_MINUS_2062;

            for (i = 1; i < 8; i++)
            {
                if (rem >= values[i])
                {
                    m |= bits[i];
                    rem -= values[i];
                }
            }

            if (fabsf(rem) < fabsf(best_error))
            {
                best_error = rem;
                best_mask = m;
            }
        }

        return best_mask;
    }
}

/*** Local Functions *********************************************************/
static int8_t hdc2080_i2c_read(uint8_t reg, uint8_t* p_value, uint8_t count)
{
    int32_t status = hdc2080_i2c_write_addr(reg);  // write only register address!

    if (status != 0)
    {
        debug1("!Wr");
        return status;
    }

	return (RETARGET_I2CWriteRead(HDC2080_I2C_ADDR, NULL, 0, p_value, count));
}

static int8_t hdc2080_i2c_write_data(uint8_t reg, uint8_t value)
{
    uint8_t buf[2U];

    buf[0] = reg;
    buf[1] = value;
    
    return(RETARGET_I2CWriteRead(HDC2080_I2C_ADDR, buf, sizeof(buf), NULL, 0));
}

static int8_t hdc2080_i2c_write_addr(uint8_t addr)
{
    uint8_t wr_addr = addr;
    
    return(RETARGET_I2CWriteRead(HDC2080_I2C_ADDR, &wr_addr, 1, NULL, 0));
}
