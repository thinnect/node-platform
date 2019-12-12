/**
 * Driver for LTC4015 from Linear Technologies
 *
 * @copyright Thinnect
 * @author Konstantin Bilozor
 * @license MIT
 */

#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <math.h>

#include "retargeti2c.h"
#include "ltc.h"

#ifdef USE_CMSIS_OS2
#include "cmsis_os2.h"
#else
#include "mtimer.h"
#endif//USE_CMSIS_OS2

#include "loglevels.h"
#define __MODUUL__ "ltc"
#define __LOG_LEVEL__ (LOG_LEVEL_ltc & BASE_LOG_LEVEL)
#include "log.h"

static void delay_ms (uint32_t period)
{
	#ifdef USE_CMSIS_OS2
	osDelay(period);
	#else
	mtimer_sleep(period);
	#endif//USE_CMSIS_OS2
}

bool ltc4015_init (void)
{
	uint8_t regv[2];
	regv[0] = 0x18;	// Enable maximum power point tracking and force measurement system to operate
	regv[1] = 0x00; // Default config
	RETARGET_I2CWrite(LTC_DEVICE_ADDR, LTC_CONFIG_REG, regv, 2);
	delay_ms(10);

	RETARGET_I2CRead(LTC_DEVICE_ADDR, LTC_MEAS_SYS_VALID_REG, regv, 2);
	delay_ms(10);

	if (LTC_TELEMETRY_READY == regv[0])
	{
		return true;
	}
	else
	{
		warn1("TLMTRY NOT READY");
		return false;
	}
}

uint16_t ltc4015_battery_read(void)
{
	uint16_t batteryMv;
	uint8_t regv[2];

	RETARGET_I2CRead(LTC_DEVICE_ADDR, LTC_VBAT_REG, regv, 2);

	// https://www.analog.com/media/en/technical-documentation/data-sheets/4015fb.pdf p.69
    batteryMv = ((regv[1] << 8) | regv[0]) * LTC_VBAT_LSB_RESO_uV * LTC_CELL_COUNT / 1000;
    return batteryMv;
}

uint16_t ltc4015_panelV_read(void)
{
	uint16_t panelMv;
	uint8_t regv[2];

	RETARGET_I2CRead(LTC_DEVICE_ADDR, LTC_INPUT_VOLT_REG, regv, 2);

	// https://www.analog.com/media/en/technical-documentation/data-sheets/4015fb.pdf p.69
  	panelMv = ((regv[1] << 8) | regv[0]) * LTC_VIN_LSB_RESO_uV / 1000;
  	return panelMv;
}

int16_t ltc4015_charge_current_read(void)
{
	int16_t ibat;
	uint8_t regv[2];

	RETARGET_I2CRead(LTC_DEVICE_ADDR, LTC_CHARGE_CURR_REG, regv, 2);

	// https://www.analog.com/media/en/technical-documentation/data-sheets/4015fb.pdf p.69
  	ibat = (((int8_t)regv[1] << 8) | regv[0]) * LTC_IBAT_LSB_RESO_nV / LTC_RSNSB_mOHM / 1000;
  	return ibat;
}



float ltc4015_bat_temp_read(void)
{
	uint8_t regv[2];
	uint16_t R_ntc;
	float steinhart;

	RETARGET_I2CRead(LTC_DEVICE_ADDR, LTC_NTC_RATIO_REG, regv, 2);
	R_ntc = (regv[1] << 8) | regv[0];

	// https://www.analog.com/media/en/technical-documentation/data-sheets/4015fb.pdf p.21
	R_ntc = (LTC_RNTC_BIAS_OHM * R_ntc) / (LTC_RTC_CALC_CONST - R_ntc);

	steinhart = R_ntc / 1000; // Convert Ohm to kOhm
    // Thermistor Example #3 from the Adafruit Learning System guide on Thermistors
    // https://learn.adafruit.com/thermistor/overview by Limor Fried, Adafruit Industries
    // MIT License - please keep attribution and consider buying parts from Adafruit
    steinhart = steinhart / RESISTANCE_THERM_NOMINAL_KOHM;   // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= BETA_COEFFICIENT;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURE_THERM_NOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                 // Invert
    steinhart -= 273.15;                         // convert to C

	return steinhart;
}
