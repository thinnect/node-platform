/**
 * For the retargeting of serial.
 *
 * Copyright Thinnect Inc. 2021
 * @license MIT
*/ 

#ifndef RETARGETSERIAL_H
#define RETARGETSERIAL_H

#include "rf_phy_driver.h"
#include "global_config.h"
#include "jump_function.h"
#include "rom_sym_def.h"

#include "ll_debug.h"
#include "ll_hw_drv.h"
#include "ll_sleep.h"

#include "uart.h"


/*
*   Initializes and retargets serial.
*/

int RETARGET_SerialInit(void);


#endif