/**
 * VEML6030 light sensor I2C driver.
 *
 * Takes integration time as an argument, adjusts gain automatically.
 *
 * Copyright Thinnect Inc. 2023
 * @license MIT
 */
#include "platform.h"

#include "platform_i2c.h"
#include "retargeti2c.h"

#include "cmsis_os2.h"

#include <inttypes.h>
#include <string.h>
#include "endianness.h"

#include "loglevels.h"
#define __MODUUL__ "veml"
#define __LOG_LEVEL__ (LOG_LEVEL_veml6030 & BASE_LOG_LEVEL)
#include "log.h"

#define VEML6030_ADDR_ALS_CONF 0
#define VEML6030_ADDR_ALS_WH   1
#define VEML6030_ADDR_ALS_WL   1
#define VEML6030_ADDR_ALS      4
#define VEML6030_ADDR_WHITE    5
#define VEML6030_ADDR_ALS_INT  6

#define VEML6030_ALS_GAIN_SHIFT  11
#define VEML6030_ALS_IT_SHIFT     6
#define VEML6030_ALS_PERS_SHIFT   4
#define VEML6030_ALS_INT_EN_SHIFT 1
#define VEML6030_ALS_SD_SHIFT     0

#define VEML6030_BASE_RESOLUTION 36 // 0.0036 lx/step

enum VEML6030Gains
{
    VEML6030_GAIN_1_8,
    VEML6030_GAIN_1_4,
    VEML6030_GAIN_X_1,
    VEML6030_GAIN_X_2,
    VEML6030_GAINS
};

static const uint8_t m_gain_setting[VEML6030_GAINS] =
{
    2, // 0b10 VEML6030_GAIN_1_8
    3, // 0b11 VEML6030_GAIN_1_4
    0, // 0b00 VEML6030_GAIN_X_1
    1, // 0b01 VEML6030_GAIN_X_2
};

static const uint8_t m_gain_multiplier[VEML6030_GAINS] =
{
    (1 << 4), // 1/8
    (1 << 3), // 1/4
    (1 << 1), // 1/1
    (1 << 0), // 2/1
};

enum VEML6030IntegrationTimes
{
    VEML6030_IT_800MS,
    VEML6030_IT_400MS,
    VEML6030_IT_200MS,
    VEML6030_IT_100MS,
    VEML6030_IT_50MS,
    VEML6030_IT_25MS,
    VEML6030_ITS,
};

static const uint8_t m_it_setting[VEML6030_ITS] =
{
    0x3, // VEML6030_IT_800MS - 0b0011
    0x2, // VEML6030_IT_400MS - 0b0010
    0x1, // VEML6030_IT_200MS - 0b0001
    0x0, // VEML6030_IT_100MS - 0b0000
    0x8, // VEML6030_IT_50MS  - 0b1000
    0xC, // VEML6030_IT_25MS  - 0b1100
};

static void * mp_i2c;
static uint8_t m_addr;


static int32_t read_als (uint8_t itm_idx)
{
    int32_t lux_times_10k = -1;
    // Start with lowest gain and increase as needed
    for (uint8_t gain_idx = 0; gain_idx < VEML6030_GAINS; gain_idx++)
    {
        uint16_t cfg = (m_gain_setting[gain_idx] << VEML6030_ALS_GAIN_SHIFT)
                     | (m_it_setting[itm_idx] << VEML6030_ALS_IT_SHIFT);
        if (0 != RETARGET_I2CWrite(m_addr, VEML6030_ADDR_ALS_CONF, (uint8_t*)&cfg, sizeof(cfg)))
        {
            err1("cfg");
            return -1;
        }

        osDelay(25 * (1 << itm_idx) + 5); // Delay for integration time + 5 milliseconds

        uint16_t val = 0;
        if (0 != RETARGET_I2CRead(m_addr, VEML6030_ADDR_ALS, (uint8_t*)&val, sizeof(val)))
        {
            err1("rd");
            return -1;
        }
        // Resolution doubles by each doubling of integration time and doubling of gain, but there is no 0.5 gain!
        lux_times_10k = ((int32_t)val) * VEML6030_BASE_RESOLUTION * (1 << itm_idx) * m_gain_multiplier[gain_idx];
        debug1("gain %d %d -> %d mlx", gain_idx, val, lux_times_10k / 10); // milli-LX
        if (val > 100)
        {
            break;
        }
        // If sampling of even lower light levels is necessary, datasheet suggests to incease integration time
        // if max gain does not give a result > 100.
    }
    return lux_times_10k / 10000;
}

static uint8_t get_itm_idx (uint16_t integration_time_ms)
{
    if (integration_time_ms <= 25)
    {
        return VEML6030_IT_25MS;
    }
    if (integration_time_ms <= 50)
    {
        return VEML6030_IT_50MS;
    }
    if (integration_time_ms <= 100)
    {
        return VEML6030_IT_100MS;
    }
    if (integration_time_ms <= 200)
    {
        return VEML6030_IT_200MS;
    }
    if (integration_time_ms <= 400)
    {
        return VEML6030_IT_400MS;
    }
    return VEML6030_IT_800MS;
}


int32_t veml6030_read (uint16_t integration_time_ms)
{
	int32_t lux = -1;

	platform_i2c_request(mp_i2c, osWaitForever); // I2C bus must first be acuired.
	RETARGET_I2CInit();
    osDelay(25);

    lux = read_als(get_itm_idx(integration_time_ms));

	RETARGET_I2CDeinit();
	platform_i2c_release(mp_i2c); // and subsequently released

	return lux;
}

void veml6030_init (void * i2c, uint8_t addr)
{
    mp_i2c = i2c;
    m_addr = addr;
}
