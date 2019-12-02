/**
 * Driver for LTC4015 from Linear Technologies
 *
 * @copyright Thinnect
 * @author Konstantin Bilozor
 * @license MIT
 */
#pragma once

#define LTC_DEVICE_ADDR        0x68
#define LTC_CONFIG_REG         0x14
#define LTC_VBAT_REG           0x3A
#define LTC_INPUT_VOLT_REG     0x3B
#define LTC_CHARGE_CURR_REG    0x3D
#define LTC_MEAS_SYS_VALID_REG 0x4A
#define LTC_TELEMETRY_READY    0x01

#define LTC_CELL_COUNT       2
#define LTC_VBAT_LSB_RESO_uV 192.26
#define LTC_VIN_LSB_RESO_uV  1648
#define LTC_IBAT_LSB_RESO_nV 1465
#define LTC_RSNSB_mOHM       4

/**
 * Initialize LTC4015.
 *
 * @return Returns true if LTC4015 telemetry is ready, false otherwise.
 */
bool ltc4015_init(void);

/**
 * Read battery voltage from LTC4015
 *
 * @return Returns voltage in mV
 */
uint16_t ltc4015_battery_read(void);

/**
 * Read solar panel voltage from LTC4015
 *
 * @return Returns voltage in mV
 */
uint16_t ltc4015_panelV_read(void);

/**
 * Read battery current from LTC4015
 *
 * @return Returns current in mA
 */
int16_t ltc4015_charge_current_read(void);
