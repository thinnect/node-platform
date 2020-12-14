/**
 * Driver for ST7735 LCD
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */

#include "st7735.h"
#include "st7735_config.h"
#include "font8x8_basic.h"

#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

static uint8_t st7735_buffer[160 / 8][80];

static void st7735_cmd(uint8_t cmd);
static void st7735_data(uint8_t data);
static void st7735_cmd_data(uint8_t cmd, const void *data, int count);
static void st7735_set_column(uint8_t x_start, uint8_t x_end);
static void st7735_set_row(uint8_t y_start, uint8_t y_end);

static void st7735_hal_init();
static void st7735_hal_cs(uint8_t level);
static void st7735_hal_res(uint8_t level);
static void st7735_hal_dc(uint8_t level);
static void st7735_hal_spi_send(uint8_t byte);
static void st7735_hal_delay(uint32_t d);

void st7735_init()
{
	st7735_hal_cs(1);
	st7735_hal_dc(1);

	st7735_hal_res(0);
	st7735_hal_delay(500);
	st7735_hal_res(1);
	st7735_hal_delay(500);

	st7735_hal_cs(0);

	st7735_cmd(ST7735_SWRESET);
	st7735_hal_delay(500);
	st7735_cmd(ST7735_SLPOUT);
	st7735_hal_delay(500);
	st7735_cmd_data(ST7735_FRMCTR1, "\x01\x2C\x2D", 3);
	st7735_cmd_data(ST7735_FRMCTR2, "\x01\x2C\x2D", 3);
	st7735_cmd_data(ST7735_FRMCTR3, "\x01\x2C\x2D\x01\x2C\x2D", 6);
	st7735_cmd_data(ST7735_INVCTR, "\x07", 1);
	st7735_cmd_data(ST7735_PWCTR1, "\xA2\x02\x84", 3);
	st7735_cmd_data(ST7735_PWCTR2, "\xC5", 1);
	st7735_cmd_data(ST7735_PWCTR3, "\x0A\x00", 2);
	st7735_cmd_data(ST7735_PWCTR4, "\x8A\x2A", 2);
	st7735_cmd_data(ST7735_PWCTR5, "\x8A\xEE", 2);
	st7735_cmd_data(ST7735_VMCTR1, "\x0E", 1);
	st7735_cmd(ST7735_INVOFF);
	st7735_cmd_data(ST7735_MADCTL, "\x68", 1);
	st7735_cmd_data(ST7735_COLMOD, "\x05", 1);

	st7735_cmd_data(ST7735_CASET, "\x00\x00\x00\x4F", 4);
	st7735_cmd_data(ST7735_RASET, "\x00\x00\x00\x9F", 4);
	st7735_cmd(ST7735_INVON);

	st7735_cmd_data(ST7735_GMCTRP1, "\x02\x1C\x07\x12\x37\x32\x29\x2D\x29\x25\x2B\x39\x00\x01\x03\x10", 16);
	st7735_cmd_data(ST7735_GMCTRN1, "\x03\x1D\x07\x06\x2E\x2C\x29\x2D\x2E\x2E\x37\x3F\x00\x00\x02\x10", 16);
	st7735_cmd(ST7735_NORON);
	st7735_hal_delay(500);
	st7735_cmd(ST7735_DISPON);
	st7735_hal_delay(500);

	st7735_hal_cs(1);

	st7735_clear(0x0000);
	st7735_refresh(0x0000);
}

void st7735_clear(uint8_t on)
{
	int x, y;

	if (on)
		on = 0xFF;

	for (y = 0; y < 80; y++) {
		for (x = 0; x < (160 / 8); x++) {
			st7735_buffer[x][y] = on;
		}
	}
}

void st7735_refresh(uint16_t color)
{
	int x, y;
	st7735_hal_cs(0);
	st7735_set_column(1, 160);
	st7735_set_row(26, 105);
	st7735_cmd(ST7735_RAMWR);
	for (y = 0; y < 80; y++) {
		for (x = 0; x < 160; x++) {
			if (st7735_point(x, y)) {
				st7735_data((color >> 8) & 0xFF);
				st7735_data((color >> 0) & 0xFF);
			} else {
				st7735_data(0x00);
				st7735_data(0x00);
			}
		}
	}
	st7735_hal_cs(1);
}

void st7735_pixel(uint8_t x, uint8_t y, uint8_t on)
{
	if (on) {
		st7735_buffer[x / 8][y] |= (1 << (x & 0x07));
	} else {
		st7735_buffer[x / 8][y] &= ~(1 << (x & 0x07));
	}
}

uint8_t st7735_point(uint8_t x, uint8_t y)
{
	if (st7735_buffer[x / 8][y] & (1 << (x & 0x07)))
		return 1;
	return 0;
}

void st7735_putc(uint8_t x, uint8_t y, int c){
	int i, j;
	for (i = 0; i < 8; i++) {
		for (j = 0; j < 8; j++) {
			if (font8x8_basic[c][j] & (1 << i)) {
				st7735_pixel(x + (i * 2) + 0, y + (j * 2) + 0, 1);
				st7735_pixel(x + (i * 2) + 0, y + (j * 2) + 1, 1);
				st7735_pixel(x + (i * 2) + 1, y + (j * 2) + 0, 1);
				st7735_pixel(x + (i * 2) + 1, y + (j * 2) + 1, 1);
			} else {
				st7735_pixel(x + (i * 2) + 0, y + (j * 2) + 0, 0);
				st7735_pixel(x + (i * 2) + 0, y + (j * 2) + 1, 0);
				st7735_pixel(x + (i * 2) + 1, y + (j * 2) + 0, 0);
				st7735_pixel(x + (i * 2) + 1, y + (j * 2) + 1, 0);
			}
		}
	}
}

void st7735_puts(uint8_t x, uint8_t y, char *s){
        int i;
        for(i = 0; s[i]; i++){
                st7735_putc(x + (i * 16), y, s[i]);
        }
}

static void st7735_cmd(uint8_t cmd)
{
	st7735_hal_dc(0);
	st7735_hal_spi_send(cmd);
}

static void st7735_data(uint8_t data)
{
	st7735_hal_dc(1);
	st7735_hal_spi_send(data);
}

static void st7735_cmd_data(uint8_t cmd, const void *data, int count)
{
	int i;
	st7735_cmd(cmd);
	for (i = 0; i < count; i++) {
		st7735_data(((uint8_t *)data)[i]);
	}
}

static void st7735_set_column(uint8_t x_start, uint8_t x_end)
{
	st7735_cmd(ST7735_CASET);
	st7735_data(0x00);
	st7735_data(x_start);
	st7735_data(0x00);
	st7735_data(x_end);
}

static void st7735_set_row(uint8_t y_start, uint8_t y_end)
{
	st7735_cmd(ST7735_RASET);
	st7735_data(0x00);
	st7735_data(y_start);
	st7735_data(0x00);
	st7735_data(y_end);
}

static void st7735_hal_init()
{
	USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;
	init.msbf = true;
	init.enable = usartDisable;
	init.baudrate = 40000000L;

	CMU_ClockEnable(cmuClock_GPIO, true);

	GPIO_PinModeSet(ST7735_EN_PORT, ST7735_EN_PIN, gpioModePushPull, 1); // EN

	GPIO_PinModeSet(ST7735_DC_PORT, ST7735_DC_PIN, gpioModePushPull, 1); // DC
	GPIO_PinModeSet(ST7735_RST_PORT, ST7735_RST_PIN, gpioModePushPull, 1); // RST

	GPIO_PinModeSet(ST7735_CS_PORT, ST7735_CS_PIN, gpioModePushPull, 1); // CS
	GPIO_PinModeSet(ST7735_MOSI_PORT, ST7735_MOSI_PIN, gpioModePushPull, 1); // MOSI
	GPIO_PinModeSet(ST7735_SCK_PORT, ST7735_SCK_PIN, gpioModePushPull, 1); // SCK

	USART_Reset(ST7735_USART);
	CMU_ClockEnable(ST7735_USART_CMU, true);
	USART_InitSync(ST7735_USART, &init);

	GPIO->USARTROUTE[ST7735_USART_NUM].TXROUTE =
		(ST7735_MOSI_PORT << _GPIO_USART_TXROUTE_PORT_SHIFT)
		| (ST7735_MOSI_PIN << _GPIO_USART_TXROUTE_PIN_SHIFT);
	GPIO->USARTROUTE[ST7735_USART_NUM].CLKROUTE =
		(ST7735_SCK_PORT << _GPIO_USART_CLKROUTE_PORT_SHIFT)
		| (ST7735_SCK_PIN << _GPIO_USART_CLKROUTE_PIN_SHIFT);

	// Enable USART interface pins
	GPIO->USARTROUTE[ST7735_USART_NUM].ROUTEEN =
		GPIO_USART_ROUTEEN_TXPEN |  // MOSI
		GPIO_USART_ROUTEEN_CLKPEN;  // CLK

	USART_Enable(ST7735_USART, usartEnable);
}

static void st7735_hal_cs(uint8_t level)
{
	if (level) {
		GPIO_PinOutSet(ST7735_CS_PORT, ST7735_CS_PIN);
	} else {
		GPIO_PinOutClear(ST7735_CS_PORT, ST7735_CS_PIN);
	}
}

static void st7735_hal_res(uint8_t level)
{
	if (level) {
		GPIO_PinOutSet(ST7735_RST_PORT, ST7735_RST_PIN);
	} else {
		GPIO_PinOutClear(ST7735_RST_PORT, ST7735_RST_PIN);
	}
}

static void st7735_hal_dc(uint8_t level)
{
	if (level) {
		GPIO_PinOutSet(ST7735_DC_PORT, ST7735_DC_PIN);
	} else {
		GPIO_PinOutClear(ST7735_DC_PORT, ST7735_DC_PIN);
	}
}

static void st7735_hal_spi_send(uint8_t byte)
{
	USART_SpiTransfer(ST7735_USART, byte);
}

static void st7735_hal_delay(uint32_t d)
{
	volatile uint32_t i;
	for(i = 0; i < d; i++)__asm__("nop");
}

