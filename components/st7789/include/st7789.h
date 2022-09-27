#ifndef MAIN_ST7789_H_
#define MAIN_ST7789_H_

#include "driver/spi_master.h"
#include "fonts.h"

#define WHITE 0xFFFF
#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define MAGENTA 0xF81F
#define GREEN 0x07E0
#define CYAN 0x7FFF
#define YELLOW 0xFFE0
#define GRAY 0X8430
#define BRED 0XF81F
#define GRED 0XFFE0
#define GBLUE 0X07FF
#define BROWN 0XBC40
#define BRRED 0XFC07
#define DARKBLUE 0X01CF
#define LIGHTBLUE 0X7D7C
#define GRAYBLUE 0X5458

#define LIGHTGREEN 0X841F
#define LGRAY 0XC618
#define LGRAYBLUE 0XA651
#define LBBLUE 0X2B12

/* Control Registers and constant codes */
#define ST7789_NOP     0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID   0x04
#define ST7789_RDDST   0x09

#define ST7789_SLPIN   0x10
#define ST7789_SLPOUT  0x11
#define ST7789_PTLON   0x12
#define ST7789_NORON   0x13

#define ST7789_INVOFF  0x20
#define ST7789_INVON   0x21
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON  0x29
#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_RAMRD   0x2E

#define ST7789_PTLAR   0x30
#define ST7789_COLMOD  0x3A
#define ST7789_MADCTL  0x36


/** 
 * Memory Data Access Control Register (0x36H)
 * MAP:     D7  D6  D5  D4  D3  D2  D1  D0 
 * param:   MY  MX  MV  ML  RGB MH  -   -
 * 
 */ 

/* Page Address Order ('0': Top to Bottom, '1': the opposite) */
#define ST7789_MADCTL_MY  0x80  
/* Column Address Order ('0': Left to Right, '1': the opposite) */
#define ST7789_MADCTL_MX  0x40  
/* Page/Column Order ('0' = Normal Mode, '1' = Reverse Mode) */
#define ST7789_MADCTL_MV  0x20  
/* Line Address Order ('0' = LCD Refresh Top to Bottom, '1' = the opposite) */
#define ST7789_MADCTL_ML  0x10
/* RGB/BGR Order ('0' = RGB, '1' = BGR) */
#define ST7789_MADCTL_RGB 0x00

#define ST7789_RDID1   0xDA
#define ST7789_RDID2   0xDB
#define ST7789_RDID3   0xDC
#define ST7789_RDID4   0xDD


/* Advanced options */
/**
 * Caution: Do not operate these settings
 * You know what you are doing 
 */

#define ST7789_COLOR_MODE_16bit 0x55    //  RGB565 (16bit)
#define ST7789_COLOR_MODE_18bit 0x66    //  RGB666 (18bit)

void spi_master_init( spi_device_handle_t *handle);

bool spi_master_write_byte( spi_device_handle_t handle ,const uint8_t* Data, size_t DataLength);

bool ST7789_WriteCommand( spi_device_handle_t *handle ,uint8_t cmd);

bool ST7789_WriteData( spi_device_handle_t *handle ,uint8_t *data, size_t data_size);

 bool ST7789_WriteSmallData( spi_device_handle_t *handle ,uint8_t data);


void ST7789_SetRotation(spi_device_handle_t *handle, uint8_t m);

static void ST7789_SetAddressWindow(spi_device_handle_t *handle, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

void ST7789_Fill_Color(spi_device_handle_t *handle, uint16_t color);

void ST7789_Init( spi_device_handle_t *handle);

void ST7789_Fill(spi_device_handle_t *handle, uint16_t xSta, uint16_t ySta, uint16_t xEnd, uint16_t yEnd, uint16_t color);

void ST7789_DrawPixel(spi_device_handle_t *handle, uint16_t x, uint16_t y, uint16_t color);

void ST7789_DrawPixel_4px(spi_device_handle_t *handle, uint16_t x, uint16_t y, uint16_t color);

void ST7789_DrawLine(spi_device_handle_t *handle, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
        uint16_t color);

void ST7789_DrawRectangle(spi_device_handle_t *handle, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

void ST7789_DrawCircle(spi_device_handle_t *handle, uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);

void ST7789_DrawImage(spi_device_handle_t *handle, uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data);

void ST7789_InvertColors(spi_device_handle_t *handle, uint8_t invert);

void ST7789_WriteChar(spi_device_handle_t *handle, uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor);

void ST7789_WriteNumbers(spi_device_handle_t *handle, uint16_t x, uint16_t y, int16_t number, FontDef font, uint16_t color, uint16_t bgcolor);

void ST7789_WriteString(spi_device_handle_t *handle, uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, uint16_t bgcolor);

void ST7789_DrawFilledRectangle(spi_device_handle_t *handle, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

void ST7789_DrawTriangle(spi_device_handle_t *handle, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color);

void ST7789_DrawFilledTriangle(spi_device_handle_t *handle, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color);

void ST7789_DrawFilledCircle(spi_device_handle_t *handle, int16_t x0, int16_t y0, int16_t r, uint16_t color);

void ST7789_TearEffect(spi_device_handle_t *handle, uint8_t tear);

void ST7789_Test(spi_device_handle_t *handle);

#endif /* MAIN_ST7789_H_ */

