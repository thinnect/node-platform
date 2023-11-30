/**
 * A supposedly common and basic SPIFFS filesystem setup procedure.
 *
 * !!! Look at the code, make sure your use-case falls under the same
 *     understanding of basic setup. !!!
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#ifndef BASIC_RTOS_FILESYSTEM_SETUP_H_
#define BASIC_RTOS_FILESYSTEM_SETUP_H_

void basic_rtos_filesystem_setup (void);
uint32_t get_flash_size (void);

#endif//BASIC_RTOS_FILESYSTEM_SETUP_H_
