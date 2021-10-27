/**
 * Platform specific functions.
 *
 * Copyright Thinnect Inc. 2021
 * @license MIT
*/ 
#include "platform.h"
#include "platform_io.h"

#include "rf_phy_driver.h"
#include "global_config.h"
#include "jump_function.h"
#include "rom_sym_def.h"

#include "ll_debug.h"
#include "ll_hw_drv.h"
#include "ll_sleep.h"

#include "uart.h"
#include "gpio.h"
#include "pwrmgr.h"
#include "string.h"
#include "clock.h"
#include "flash.h"
#include "version.h"


bool buttonstate = 0;
volatile uint8_t g_clk32K_config;

static void hal_low_power_io_init(void)
{
   //========= pull all io to gnd by default
    ioinit_cfg_t ioInit[]= {
        //TSOP6252 10 IO
       {GPIO_P02,   GPIO_FLOATING   },/*SWD*/
       {GPIO_P03,   GPIO_FLOATING   },/*SWD*/
       {GPIO_P09,   GPIO_PULL_UP    },/*UART TX*/
       {GPIO_P10,   GPIO_PULL_UP    },/*UART RX*/
       {GPIO_P11,   GPIO_PULL_DOWN  },
       {GPIO_P14,   GPIO_PULL_DOWN  },
       {GPIO_P15,   GPIO_PULL_DOWN  },
       {GPIO_P16,   GPIO_FLOATING   },
       {GPIO_P18,   GPIO_PULL_DOWN  },
       {GPIO_P20,   GPIO_PULL_DOWN  },
#if(SDK_VER_CHIP==__DEF_CHIP_QFN32__)
        //6222 23 IO
       {GPIO_P00,   GPIO_PULL_DOWN  },
       {GPIO_P01,   GPIO_PULL_DOWN  },
       {GPIO_P07,   GPIO_PULL_UP  },
       {GPIO_P17,   GPIO_FLOATING   },/*32k xtal*/
       {GPIO_P23,   GPIO_PULL_UP  },
       {GPIO_P24,   GPIO_PULL_DOWN  },
       {GPIO_P25,   GPIO_PULL_DOWN  },
       {GPIO_P26,   GPIO_PULL_DOWN  },
       {GPIO_P27,   GPIO_PULL_DOWN  },
       {GPIO_P31,   GPIO_PULL_DOWN  },
       {GPIO_P32,   GPIO_PULL_DOWN  },
       {GPIO_P33,   GPIO_PULL_DOWN  },
       {GPIO_P34,   GPIO_PULL_DOWN  },
#endif
    };
    for(uint8_t i=0;i<sizeof(ioInit)/sizeof(ioinit_cfg_t);i++)
        hal_gpio_pull_set(ioInit[i].pin,ioInit[i].type);

    DCDC_CONFIG_SETTING(0x0a); DCDC_REF_CLK_SETTING(1);
    DIG_LDO_CURRENT_SETTING(0x01);
    hal_pwrmgr_RAM_retention(RET_SRAM0);
    hal_pwrmgr_RAM_retention_set();
    hal_pwrmgr_LowCurrentLdo_enable();
}


static void hal_init(void)
{
    hal_low_power_io_init();

    clk_init(g_system_clk);

    hal_rtc_clock_config(CLK_32K_XTAL);

    SystemCoreClockUpdate();

    hal_pwrmgr_init();
    xflash_Ctx_t cfg =
    {
			  .spif_ref_clk   =   SYS_CLK_DLL_64M,
        .rd_instr       =   XFRD_FCMD_READ_DUAL
    };
    hal_spif_cache_init(cfg);
    hal_gpio_init();
}


void PLATFORM_uart(void)
{
  uart_Cfg_t cfg = {
#ifndef TEST_SYSTEM
	.tx_pin = P9,
  .rx_pin = P10,
#else
	.tx_pin = P14,
  .rx_pin = P15,
#endif
  .rts_pin = GPIO_DUMMY,
  .cts_pin = GPIO_DUMMY,
  .baudrate = 115200,
  .use_fifo = TRUE,
  .hw_fwctrl = FALSE,
  .use_tx_buf = FALSE,
  .parity     = FALSE,
  .evt_handler = NULL,
  };
  hal_uart_init(cfg, UART0);//uart init
}


int PLATFORM_Init()
{
	g_system_clk = SYS_CLK_DLL_48M; //SYS_CLK_XTAL_16M, SYS_CLK_DLL_32M, SYS_CLK_DLL_64M
  g_clk32K_config = CLK_32K_XTAL;

  drv_irq_init();
  init_config();
  hal_init();
	
	
	return true;
}
void PLATFORM_LedsSet(uint8_t leds)
{
	//LOG("Count is %d \r\n", leds);
	
	hal_gpio_write(LED_1, leds & LED_1_MASK);
	hal_gpio_write(LED_2, leds & LED_2_MASK);
	hal_gpio_write(LED_3, leds & LED_3_MASK);
}
bool PLATFORM_ButtonGet()
{
	buttonstate = hal_gpio_read(PLATFORM_BUTTON);
	//WaitMs(200);
	return (buttonstate != hal_gpio_read(PLATFORM_BUTTON));
}
