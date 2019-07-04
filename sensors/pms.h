/**
 * Driver for particle sensor PMS7003
 *
 * @copyright Thinnect
 * @author Konstantin Bilozor
 * @license MIT
 */
#pragma once

#define PMS_BUF_LEN     32
#define PMS_CMD_LEN     7
#define PMS_PM1_0_HIGH  4
#define PMS_PM1_0_LOW   5
#define PMS_PM2_5_HIGH  6
#define PMS_PM2_5_LOW   7
#define PMS_PM10_HIGH   8
#define PMS_PM10_LOW    9
#define PMS_CC_HIGH     30
#define PMS_CC_LOW      31

/**
 * Init PMS to passive mode
 */
void pms7003_init();

/**
 * Read data from PMS7003
 *
 * @param pm1_0 PM1.0 concentration unit ug/m3
 * @param pm2_5 PM2.5 concentration unit ug/m3
 * @param pm10  PM10 concentration unit ug/m3
 *
 * @return Returns true if reading is succesfull, false otherwise
 */
bool pms7003_read(uint16_t *pm1_0, uint16_t *pm2_5, uint16_t *pm10);
