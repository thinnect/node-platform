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

#include "sys_panic.h"

#ifdef DUMP_LOG_BUF
extern uint8_t m_log_dma_buf;
#endif

extern void dbg_printf(const char *format, ...);
extern void RETARGET_SerialInit(void);
extern void SysTick_Handler(void); // 223
extern void SVC_Handler(void); // 221
extern void PendSV_Handler(void); // 222

void hard_fault_handler_c(unsigned int * hardfault_args, unsigned lr_value);
__asm void hard_fault_handler_asm(void);


/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
#define  XTAL            (16000000UL)     /* Oscillator frequency */

#define  SYSTEM_CLOCK    (6 * XTAL)

/*----------------------------------------------------------------------------
  System Core Clock Variable
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = SYSTEM_CLOCK;  /* System Core Clock Frequency */


bool buttonstate = 0;
volatile uint8_t g_clk32K_config;


static void hal_low_power_io_init (void)
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
			 {GPIO_P11,   GPIO_PULL_DOWN  },
			 {GPIO_P18,   GPIO_PULL_DOWN  },
       {GPIO_P20,   GPIO_PULL_DOWN  },
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


static void hal_init (void)
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

void PLATFORM_Init (void)
{
	g_system_clk = SYS_CLK_DLL_48M; //SYS_CLK_XTAL_16M, SYS_CLK_DLL_32M, SYS_CLK_DLL_64M
  g_clk32K_config = CLK_32K_XTAL;

	
	//Setup Jumptable entries
	JUMP_FUNCTION(HARDFAULT_HANDLER) = (uint32_t)&hard_fault_handler_asm;
	JUMP_FUNCTION(SVC_HANDLER) = (uint32_t)&SVC_Handler;
	JUMP_FUNCTION(PENDSV_HANDLER) = (uint32_t)&PendSV_Handler;
	JUMP_FUNCTION(SYSTICK_HANDLER) = (uint32_t)&SysTick_Handler;
	
	
	// Init everything else
  drv_irq_init();
  init_config();
  hal_init();
	
}
void PLATFORM_LedsSet(uint8_t leds)
{
	
	hal_gpio_write(LED_1, leds & LED_1_MASK);
	hal_gpio_write(LED_2, leds & LED_2_MASK);
	hal_gpio_write(LED_3, leds & LED_3_MASK);
}
bool PLATFORM_ButtonGet (void)
{
	buttonstate = hal_gpio_read(PLATFORM_BUTTON);
	//WaitMs(200);
	return (buttonstate != hal_gpio_read(PLATFORM_BUTTON));
}


/*----------------------------------------------------------------------------
  System Core Clock update function
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate (void)
{
	
	switch(g_system_clk)
	{
		case SYS_CLK_XTAL_16M:
			SystemCoreClock = XTAL;
			break;
		case SYS_CLK_DLL_32M:
			SystemCoreClock = 2 * XTAL;
			break;
		case SYS_CLK_DLL_48M:
			SystemCoreClock = 3 * XTAL;
			break;
		case SYS_CLK_DLL_64M:
			SystemCoreClock = 4 * XTAL;
			break;
		case SYS_CLK_DLL_96M:
			SystemCoreClock = 6 * XTAL;
			break;
		default:
			sys_panic("Clock Init");
	}
}

/*----------------------------------------------------------------------------
  System initialization function
 *----------------------------------------------------------------------------*/
void SystemInit (void)
{
  SystemCoreClock = SYSTEM_CLOCK;
}

__asm void hard_fault_handler_asm(void)
{
		MOVS r0, #4
		MOV r1, LR
		TST r0, r1
		BEQ stacking_used_MSP
		MRS R0, PSP
		B get_LR_and_branch
stacking_used_MSP
		MRS R0, MSP
get_LR_and_branch
		MOV R1, LR
		LDR R2,=__cpp(hard_fault_handler_c)
		BX R2
}

#ifdef DUMP_LOG_BUF
void dump_ldma_buf()
{
    hal_uart_send_buff(UART0, (uint8_t*)&m_log_dma_buf, 256);
}
#endif


/*******************************************************************************
 * HardFault handler in C, with stack frame location and LR value
 * extracted from the assembly wrapper as input parameters
 */
void hard_fault_handler_c (unsigned int * hardfault_args, unsigned lr_value)
{
	    RETARGET_SerialInit();
		unsigned int stacked_r0;
		unsigned int stacked_r1;
		unsigned int stacked_r2;
		unsigned int stacked_r3;
		unsigned int stacked_r12;
		unsigned int stacked_lr;
		unsigned int stacked_pc;
		unsigned int stacked_psr;
		stacked_r0 = ((unsigned long) hardfault_args[0]);
		stacked_r1 = ((unsigned long) hardfault_args[1]);
		stacked_r2 = ((unsigned long) hardfault_args[2]);
		stacked_r3 = ((unsigned long) hardfault_args[3]);
		stacked_r12 = ((unsigned long) hardfault_args[4]);
		stacked_lr = ((unsigned long) hardfault_args[5]);
		stacked_pc = ((unsigned long) hardfault_args[6]);
		stacked_psr = ((unsigned long) hardfault_args[7]);
		dbg_printf("\r\n[HardFault handler]\r\n");
		dbg_printf("R0 = %X\r\n", stacked_r0);
		dbg_printf("R1 = %X\r\n", stacked_r1);
		dbg_printf("R2 = %X\r\n", stacked_r2);
		dbg_printf("R3 = %X\r\n", stacked_r3);
		dbg_printf("R12 = %X\r\n", stacked_r12);
		dbg_printf("Stacked LR = %X\r\n", stacked_lr);
		dbg_printf("Stacked PC = %X\r\n", stacked_pc);
		dbg_printf("Stacked PSR = %X\r\n", stacked_psr);
		dbg_printf("Current LR = %X\r\n", lr_value);
#ifdef DUMP_LOG_BUF
		dbg_printf("[Dumping ldma]\r\n");
		dump_ldma_buf();
#endif
		while(1); // endless loop
}

