#ifndef _ST7735_CONFIG_H_
#define _ST7735_CONFIG_H_

#define ST7735_EN_PORT   gpioPortB
#define ST7735_EN_PIN    1

#define ST7735_DC_PORT   gpioPortD
#define ST7735_DC_PIN    2

#define ST7735_RST_PORT  gpioPortC
#define ST7735_RST_PIN   3

#define ST7735_CS_PORT   gpioPortC
#define ST7735_CS_PIN    2

#define ST7735_MOSI_PORT gpioPortD
#define ST7735_MOSI_PIN  3

#define ST7735_SCK_PORT  gpioPortD
#define ST7735_SCK_PIN   4

#define ST7735_USART     USART2
#define ST7735_USART_NUM 2
#define ST7735_USART_CMU cmuClock_USART2

#endif

