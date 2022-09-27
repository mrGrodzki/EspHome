#include <stdio.h>
#include "st7789.h"

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "esp_log.h"

#include "fonts.h"

#define TAG "ST7789"
#define	_DEBUG_ 0

#if 0
#ifdef CONFIG_IDF_TARGET_ESP32
#define LCD_HOST HSPI_HOST
#elif defined CONFIG_IDF_TARGET_ESP32S2
#define LCD_HOST SPI2_HOST
#elif defined CONFIG_IDF_TARGET_ESP32S3
#define LCD_HOST SPI2_HOST
#elif defined CONFIG_IDF_TARGET_ESP32C3
#define LCD_HOST SPI2_HOST
#endif
#endif

#if CONFIG_SPI2_HOST
#define HOST_ID SPI2_HOST
#elif CONFIG_SPI3_HOST
#define HOST_ID SPI3_HOST
#endif


#define	INTERVAL		400
#define WAIT	vTaskDelay(INTERVAL)

static const int SPI_Command_Mode = 0;
static const int SPI_Data_Mode = 1;
//static const int SPI_Frequency = SPI_MASTER_FREQ_20M;
//static const int SPI_Frequency = SPI_MASTER_FREQ_26M;
static const int SPI_Frequency = SPI_MASTER_FREQ_40M;
//static const int SPI_Frequency = SPI_MASTER_FREQ_80M;




spi_device_handle_t spi_one;
spi_host_device_t _spihost = SPI2_HOST;

static void spi_transfer_callback()
{

}

void spi_master_init(spi_device_handle_t *handle)
{
	esp_err_t ret;

	ESP_LOGI(TAG, "CONFIG_CS_GPIO=%d",CONFIG_CS_GPIO);
	if ( CONFIG_CS_GPIO >= 0 ) {
		//gpio_pad_select_gpio( CONFIG_CS_GPIO );
		gpio_reset_pin( CONFIG_CS_GPIO );
		gpio_set_direction( CONFIG_CS_GPIO, GPIO_MODE_OUTPUT );
		gpio_set_level( CONFIG_CS_GPIO, 0 );
	}

	ESP_LOGI(TAG, "CONFIG_DC_GPIO=%d",CONFIG_DC_GPIO);
	//gpio_pad_select_gpio( CONFIG_DC_GPIO );
	gpio_reset_pin( CONFIG_DC_GPIO );
	gpio_set_direction( CONFIG_DC_GPIO, GPIO_MODE_OUTPUT );
	gpio_set_level( CONFIG_DC_GPIO, 0 );

	ESP_LOGI(TAG, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
	if ( CONFIG_RESET_GPIO >= 0 ) {
		//gpio_pad_select_gpio( CONFIG_RESET_GPIO );
		gpio_reset_pin( CONFIG_RESET_GPIO );
		gpio_set_direction( CONFIG_RESET_GPIO, GPIO_MODE_OUTPUT );
		gpio_set_level( CONFIG_RESET_GPIO, 1 );
		vTaskDelay(250 / portTICK_PERIOD_MS);
		gpio_set_level( CONFIG_RESET_GPIO, 0 );
		vTaskDelay(250 / portTICK_PERIOD_MS);
		gpio_set_level( CONFIG_RESET_GPIO, 1 );
		vTaskDelay(250 / portTICK_PERIOD_MS);
	}

	ESP_LOGI(TAG, "CONFIG_BL_GPIO=%d",CONFIG_BL_GPIO);
	if ( CONFIG_BL_GPIO >= 0 ) {
		//gpio_pad_select_gpio(CONFIG_BL_GPIO);
		gpio_reset_pin(CONFIG_BL_GPIO);
		gpio_set_direction( CONFIG_BL_GPIO, GPIO_MODE_OUTPUT );
		gpio_set_level( CONFIG_BL_GPIO, 0 );
	}

	ESP_LOGI(TAG, "CONFIG_MOSI_GPIO=%d",CONFIG_MOSI_GPIO);
	ESP_LOGI(TAG, "CONFIG_SCLK_GPIO=%d",CONFIG_SCLK_GPIO);
	
   

	spi_bus_config_t buscfg = {
		.mosi_io_num = CONFIG_MOSI_GPIO,
		.miso_io_num = -1,
		.sclk_io_num = CONFIG_SCLK_GPIO,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 0,
		.flags = 0
	};

	ret=spi_bus_initialize(_spihost, &buscfg,SPI_DMA_CH_AUTO);
	ESP_LOGD(TAG, "spi_bus_initialize=%d",ret);
	assert(ret==ESP_OK);

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(spi_device_interface_config_t));

    devcfg.clock_speed_hz = SPI_Frequency;
	devcfg.queue_size = 7;
	devcfg.mode = 2;
	devcfg.flags = SPI_DEVICE_NO_DUMMY;

    
     memset(&spi_one, 0, sizeof(spi_device_handle_t));
    ret=spi_bus_add_device(_spihost,&devcfg,&spi_one);
    ESP_ERROR_CHECK(ret);
	
	ESP_LOGD(TAG, "spi_bus_add_device=%d",ret);

    vTaskDelay(2000 / portTICK_PERIOD_MS);


    // uint8_t zero_buf[1] = {0x00};
    // spi_transaction_t t;
  
    /* memset(&t, 0, sizeof(t));
    t.tx_buffer=&zero_buf;
    t.length=8;
    ret=spi_device_transmit(spi, &t); */
	
}

bool spi_master_write_byte( spi_device_handle_t handle, const uint8_t* Data, size_t DataLength)
{	
	spi_transaction_t SPITransaction;
	esp_err_t ret;


	if ( DataLength > 0 ) {
		memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
		SPITransaction.length = DataLength * 8;
		SPITransaction.tx_buffer = Data;
		// for(uint8_t i=0; i<DataLength * 8; i++)
		// {
		// ESP_LOGI(TAG, "data %d = %d",i, Data[i] );
		// }
#if 1
		ret = spi_device_transmit( spi_one, &SPITransaction );
#else
		ret = spi_device_polling_transmit( SPIHandle, &SPITransaction );
#endif
		assert(ret==ESP_OK); 
	}

	return true;

}



  bool ST7789_WriteCommand(spi_device_handle_t *handle, uint8_t cmd )
{
	static uint8_t data = 0;
	data = cmd;
	gpio_set_level( CONFIG_DC_GPIO, SPI_Command_Mode );
    return spi_master_write_byte(handle, &data, 1); 
                
}

/**
 * @brief Write data to ST7789 controller
 * @param buff -> pointer of data buffer
 * @param buff_size -> size of the data buffer
 * @return none
 */
 bool ST7789_WriteData(spi_device_handle_t *handle, uint8_t *data, size_t data_size)
{
	
	gpio_set_level( CONFIG_DC_GPIO, SPI_Data_Mode );

	// split data in small chunks because HAL can't send more than 64K at once
                 return spi_master_write_byte(handle, data, data_size);
	}

 bool ST7789_WriteSmallData(spi_device_handle_t *handle, uint8_t data)
{
	static uint8_t datapoint = 0;
	datapoint = data;
	gpio_set_level( CONFIG_DC_GPIO, SPI_Data_Mode );
    return spi_master_write_byte(handle, &datapoint, 1);
}      



void ST7789_SetRotation(spi_device_handle_t *handle, uint8_t m)
{
	ST7789_WriteCommand(handle, ST7789_MADCTL);	// MADCTL
	switch (m) {
	case 0:
		ST7789_WriteSmallData(handle, ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB);
		break;
	case 1:
		ST7789_WriteSmallData(handle, ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
		break;
	case 2:
		ST7789_WriteSmallData(handle, ST7789_MADCTL_RGB);
		break;
	case 3:
		ST7789_WriteSmallData(handle, ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);
		break;
	default:
		break;
	}
}

/**
 * @brief Set address of DisplayWindow
 * @param xi&yi -> coordinates of window
 * @return none
 */
static void ST7789_SetAddressWindow(spi_device_handle_t *handle, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	
	uint16_t x_start = x0 + CONFIG_OFFSETX, x_end = x1 + CONFIG_OFFSETX;
	uint16_t y_start = y0 + CONFIG_OFFSETY, y_end = y1 + CONFIG_OFFSETY;
	
	/* Column Address set */
	ST7789_WriteCommand(handle, ST7789_CASET); 
	{
		uint8_t data[] = {x_start >> 8, x_start & 0xFF, x_end >> 8, x_end & 0xFF};
		ST7789_WriteData(handle, data, sizeof(data));
	}

	/* Row Address set */
	ST7789_WriteCommand(handle, ST7789_RASET);
	{
		uint8_t data[] = {y_start >> 8, y_start & 0xFF, y_end >> 8, y_end & 0xFF};
		ST7789_WriteData(handle, data, sizeof(data));
	}
	/* Write to RAM */
	ST7789_WriteCommand(handle, ST7789_RAMWR);
	
}

/**
 * @brief Fill the DisplayWindow with single color
 * @param color -> color to Fill with
 * @return none
 */
void ST7789_Fill_Color(spi_device_handle_t *handle, uint16_t color)
{
	uint16_t i, j;
	ST7789_SetAddressWindow(handle, 0, 0, CONFIG_WIDTH - 1, CONFIG_HEIGHT - 1);
	for (i = 0; i < CONFIG_WIDTH; i++)
		for (j = 0; j < CONFIG_HEIGHT; j++) {
			uint8_t data[] = {color >> 8, color & 0xFF};
			ST7789_WriteData(handle, data, sizeof(data));
		}
}



/**
 * @brief Initialize ST7789 controller
 * @param none
 * @return none
 */
void ST7789_Init(spi_device_handle_t *handle)


{
	ST7789_WriteCommand(handle, 0x01);	
    vTaskDelay(150 / portTICK_PERIOD_MS);
    ST7789_WriteCommand(handle, ST7789_COLMOD);		//	Set color mode
    vTaskDelay(150 / portTICK_PERIOD_MS);
    ST7789_WriteSmallData(handle, ST7789_COLOR_MODE_16bit);
    vTaskDelay(150 / portTICK_PERIOD_MS);
  	ST7789_WriteCommand(handle, 0xB2);				//	Porch control
    vTaskDelay(150 / portTICK_PERIOD_MS);
	{
		uint8_t data[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};
		ST7789_WriteData(handle,data, sizeof(data));
	}
	// ST7789_SetRotation(ST7789_ROTATION);	//	MADCTL (Display Rotation)
	
	/* Internal LCD Voltage generator settings */
    ST7789_WriteCommand(handle, 0XB7);				//	Gate Control
    ST7789_WriteSmallData(handle,0x35);			//	Default value
    ST7789_WriteCommand(handle,0xBB);				//	VCOM setting
    ST7789_WriteSmallData(handle,0x19);			//	0.725v (default 0.75v for 0x20)
    ST7789_WriteCommand(handle,0xC0);				//	LCMCTRL	
    ST7789_WriteSmallData (handle,0x2C);			//	Default value
    ST7789_WriteCommand (handle,0xC2);				//	VDV and VRH command Enable
    ST7789_WriteSmallData (handle,0x01);			//	Default value
    ST7789_WriteCommand (handle,0xC3);				//	VRH set
    ST7789_WriteSmallData (handle,0x12);			//	+-4.45v (defalut +-4.1v for 0x0B)
    ST7789_WriteCommand (handle,0xC4);				//	VDV set
    ST7789_WriteSmallData (handle,0x20);			//	Default value
    ST7789_WriteCommand (handle,0xC6);				//	Frame rate control in normal mode
    ST7789_WriteSmallData (handle,0x0F);			//	Default value (60HZ)
    ST7789_WriteCommand (handle,0xD0);				//	Power control
    ST7789_WriteSmallData (handle,0xA4);			//	Default value
    ST7789_WriteSmallData (handle, 0xA1);			//	Default value
	/**************** Division line ****************/

	ST7789_WriteCommand(handle,0xE0);
	{
		uint8_t data[] = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23};
		ST7789_WriteData(handle,data, sizeof(data));
	}

    ST7789_WriteCommand(handle,0xE1);
	{
		uint8_t data[] = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23};
		ST7789_WriteData(handle,data, sizeof(data));
	}
    ST7789_WriteCommand (handle,ST7789_INVON);		//	Inversion ON
	ST7789_WriteCommand (handle,ST7789_SLPOUT);	//	Out of sleep mode
  	ST7789_WriteCommand (handle,ST7789_NORON);		//	Normal Display on
  	ST7789_WriteCommand (handle,ST7789_DISPON);	//	Main screen turned on	

	
// 	ST7789_Fill_Color(0xBDF8);
// char temp_str[20] = {0,};
// snprintf(temp_str, 20, "T:%dC*", 20);
// ST7789_WriteString(1, 1, temp_str, Font_11x18, YELLOW, 0xBDF8);

// snprintf(temp_str, 20, "H:%dm", 22);
// ST7789_WriteString(1, 40, temp_str, Font_11x18, YELLOW, 0xBDF8);

// snprintf(temp_str, 20, "P:%dmm", 734);
// ST7789_WriteString(1, 20, temp_str, Font_11x18, YELLOW, 0xBDF8);

// snprintf(temp_str, 20, "%d:%d", 13, 44 );
// ST7789_WriteString(80, 105, temp_str, Font_16x26, YELLOW, 0xBDF8);

// snprintf(temp_str, 20, "Steps:%d", 0);
// ST7789_WriteString(1, 220, temp_str, Font_11x18, YELLOW, 0xBDF8);			//	Fill with Black.

}


void ST7789_Fill(spi_device_handle_t *handle, uint16_t xSta, uint16_t ySta, uint16_t xEnd, uint16_t yEnd, uint16_t color)
{
	if ((xEnd < 0) || (xEnd >= CONFIG_WIDTH) ||
		 (yEnd < 0) || (yEnd >= CONFIG_HEIGHT))	return;
	
	uint16_t i, j;
	ST7789_SetAddressWindow(handle, xSta, ySta, xEnd, yEnd);
	for (i = ySta; i <= yEnd; i++)
		for (j = xSta; j <= xEnd; j++) {
			uint8_t data[] = {color >> 8, color & 0xFF};
			ST7789_WriteData(handle, data, sizeof(data));
		}
	
}

/**
 * @brief Draw a big Pixel at a point
 * @param x&y -> coordinate of the point
 * @param color -> color of the Pixel
 * @return none
 */

 /**
 * @brief Draw a Pixel
 * @param x&y -> coordinate to Draw
 * @param color -> color of the Pixel
 * @return none
 */
void ST7789_DrawPixel(spi_device_handle_t *handle, uint16_t x, uint16_t y, uint16_t color)
{
	if ((x < 0) || (x >= CONFIG_WIDTH) ||
		 (y < 0) || (y >= CONFIG_HEIGHT))	return;
	
	ST7789_SetAddressWindow(handle, x, y, x, y);
	uint8_t data[] = {color >> 8, color & 0xFF};
	
	ST7789_WriteData(handle, data, sizeof(data));
	
}

void ST7789_DrawPixel_4px(spi_device_handle_t *handle, uint16_t x, uint16_t y, uint16_t color)
{
	if ((x <= 0) || (x > CONFIG_WIDTH) ||
		 (y <= 0) || (y > CONFIG_HEIGHT))	return;
	
	ST7789_Fill(handle, x - 1, y - 1, x + 1, y + 1, color);
	
}

/**
 * @brief Draw a line with single color
 * @param x1&y1 -> coordinate of the start point
 * @param x2&y2 -> coordinate of the end point
 * @param color -> color of the line to Draw
 * @return none
 */
void ST7789_DrawLine(spi_device_handle_t *handle, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
        uint16_t color) {
	uint16_t swap;
    uint16_t steep = fabs(y1 - y0) > fabs(x1 - x0);
    if (steep) {
		swap = x0;
		x0 = y0;
		y0 = swap;

		swap = x1;
		x1 = y1;
		y1 = swap;
        //_swap_int16_t(x0, y0);
        //_swap_int16_t(x1, y1);
    }

    if (x0 > x1) {
		swap = x0;
		x0 = x1;
		x1 = swap;

		swap = y0;
		y0 = y1;
		y1 = swap;
        //_swap_int16_t(x0, x1);
        //_swap_int16_t(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = fabs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (; x0<=x1; x0++) {
        if (steep) {
            ST7789_DrawPixel(handle, y0, x0, color);
        } else {
            ST7789_DrawPixel(handle, x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}

/**
 * @brief Draw a Rectangle with single color
 * @param xi&yi -> 2 coordinates of 2 top points.
 * @param color -> color of the Rectangle line
 * @return none
 */
void ST7789_DrawRectangle(spi_device_handle_t *handle, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	
	ST7789_DrawLine(handle, x1, y1, x2, y1, color);
	ST7789_DrawLine(handle, x1, y1, x1, y2, color);
	ST7789_DrawLine(handle, x1, y2, x2, y2, color);
	ST7789_DrawLine(handle, x2, y1, x2, y2, color);
	
}

/** 
 * @brief Draw a circle with single color
 * @param x0&y0 -> coordinate of circle center
 * @param r -> radius of circle
 * @param color -> color of circle line
 * @return  none
 */
void ST7789_DrawCircle(spi_device_handle_t *handle, uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	
	ST7789_DrawPixel(handle, x0, y0 + r, color);
	ST7789_DrawPixel(handle, x0, y0 - r, color);
	ST7789_DrawPixel(handle, x0 + r, y0, color);
	ST7789_DrawPixel(handle, x0 - r, y0, color);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		ST7789_DrawPixel(handle, x0 + x, y0 + y, color);
		ST7789_DrawPixel(handle, x0 - x, y0 + y, color);
		ST7789_DrawPixel(handle, x0 + x, y0 - y, color);
		ST7789_DrawPixel(handle, x0 - x, y0 - y, color);

		ST7789_DrawPixel(handle, x0 + y, y0 + x, color);
		ST7789_DrawPixel(handle, x0 - y, y0 + x, color);
		ST7789_DrawPixel(handle, x0 + y, y0 - x, color);
		ST7789_DrawPixel(handle, x0 - y, y0 - x, color);
	}
	
}

/**
 * @brief Draw an Image on the screen
 * @param x&y -> start point of the Image
 * @param w&h -> width & height of the Image to Draw
 * @param data -> pointer of the Image array
 * @return none
 */
void ST7789_DrawImage(spi_device_handle_t *handle, uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data)
{
	if ((x >= CONFIG_WIDTH) || (y >= CONFIG_HEIGHT))
		return;
	if ((x + w - 1) >= CONFIG_WIDTH)
		return;
	if ((y + h - 1) >= CONFIG_HEIGHT)
		return;

	
	ST7789_SetAddressWindow(handle, x, y, x + w - 1, y + h - 1);
	ST7789_WriteData(handle, (uint8_t *)data, sizeof(uint16_t) * w * h);

}

/**
 * @brief Invert Fullscreen color
 * @param invert -> Whether to invert
 * @return none
 */
void ST7789_InvertColors(spi_device_handle_t *handle, uint8_t invert)
{
	
	ST7789_WriteCommand(handle, invert ? 0x21 /* INVON */ : 0x20 /* INVOFF */);
	
}

/** 
 * @brief Write a char
 * @param  x&y -> cursor of the start point.
 * @param ch -> char to write
 * @param font -> fontstyle of the string
 * @param color -> color of the char
 * @param bgcolor -> background color of the char
 * @return  none
*/
void ST7789_WriteChar(spi_device_handle_t *handle, uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor)
{
	uint32_t i, b, j;
	
	ST7789_SetAddressWindow(handle, x, y, x + font.width - 1, y + font.height - 1);

	for (i = 0; i < font.height; i++) {
		b = font.data[(ch - 32) * font.height + i];
		for (j = 0; j < font.width; j++) {
			if ((b << j) & 0x8000) {
				uint8_t data[] = {color >> 8, color & 0xFF};
				ST7789_WriteData(handle, data, sizeof(data));
			}
			else {
				uint8_t data[] = {bgcolor >> 8, bgcolor & 0xFF};
				ST7789_WriteData(handle, data, sizeof(data));
			}
		}
	}
	
}

void ST7789_WriteNumbers(spi_device_handle_t *handle, uint16_t x, uint16_t y, int16_t number, FontDef font, uint16_t color, uint16_t bgcolor)
{

}
/** 
 * @brief Write a string 
 * @param  x&y -> cursor of the start point.
 * @param str -> string to write
 * @param font -> fontstyle of the string
 * @param color -> color of the string
 * @param bgcolor -> background color of the string
 * @return  none
*/
void ST7789_WriteString(spi_device_handle_t *handle, uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, uint16_t bgcolor)
{
	
	while (*str) {
		if (x + font.width >= CONFIG_WIDTH) {
			x = 0;
			y += font.height;
			if (y + font.height >= CONFIG_HEIGHT) {
				break;
			}

			if (*str == ' ') {
				// skip spaces in the beginning of the new line
				str++;
				continue;
			}
		}
		ST7789_WriteChar(handle, x, y, *str, font, color, bgcolor);
		x += font.width;
		str++;
	}
	
}

/** 
 * @brief Draw a filled Rectangle with single color
 * @param  x&y -> coordinates of the starting point
 * @param w&h -> width & height of the Rectangle
 * @param color -> color of the Rectangle
 * @return  none
 */
void ST7789_DrawFilledRectangle(spi_device_handle_t *handle, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
	
	uint8_t i;

	/* Check input parameters */
	if (x >= CONFIG_WIDTH ||
		y >= CONFIG_HEIGHT) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= CONFIG_WIDTH) {
		w = CONFIG_WIDTH - x;
	}
	if ((y + h) >= CONFIG_HEIGHT) {
		h = CONFIG_HEIGHT - y;
	}

	/* Draw lines */
	for (i = 0; i <= h; i++) {
		/* Draw lines */
		ST7789_DrawLine(handle, x, y + i, x + w, y + i, color);
	}
	
}

/** 
 * @brief Draw a Triangle with single color
 * @param  xi&yi -> 3 coordinates of 3 top points.
 * @param color ->color of the lines
 * @return  none
 */
void ST7789_DrawTriangle(spi_device_handle_t *handle, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color)
{
	
	/* Draw lines */
	ST7789_DrawLine(handle, x1, y1, x2, y2, color);
	ST7789_DrawLine(handle, x2, y2, x3, y3, color);
	ST7789_DrawLine(handle, x3, y3, x1, y1, color);
	
}

/** 
 * @brief Draw a filled Triangle with single color
 * @param  xi&yi -> 3 coordinates of 3 top points.
 * @param color ->color of the triangle
 * @return  none
 */
void ST7789_DrawFilledTriangle(spi_device_handle_t *handle, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color)
{

	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
			yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
			curpixel = 0;

	deltax = fabs(x2 - x1);
	deltay = fabs(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) {
		xinc1 = 1;
		xinc2 = 1;
	}
	else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) {
		yinc1 = 1;
		yinc2 = 1;
	}
	else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay) {
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	}
	else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++) {
		ST7789_DrawLine(handle, x, y, x3, y3, color);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
}

/** 
 * @brief Draw a Filled circle with single color
 * @param x0&y0 -> coordinate of circle center
 * @param r -> radius of circle
 * @param color -> color of circle
 * @return  none
 */
void ST7789_DrawFilledCircle(spi_device_handle_t *handle, int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
	
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	ST7789_DrawPixel(handle, x0, y0 + r, color);
	ST7789_DrawPixel(handle, x0, y0 - r, color);
	ST7789_DrawPixel(handle, x0 + r, y0, color);
	ST7789_DrawPixel(handle, x0 - r, y0, color);
	ST7789_DrawLine(handle, x0 - r, y0, x0 + r, y0, color);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		ST7789_DrawLine(handle, x0 - x, y0 + y, x0 + x, y0 + y, color);
		ST7789_DrawLine(handle, x0 + x, y0 - y, x0 - x, y0 - y, color);

		ST7789_DrawLine(handle, x0 + y, y0 + x, x0 - y, y0 + x, color);
		ST7789_DrawLine(handle, x0 + y, y0 - x, x0 - y, y0 - x, color);
	}
}


/**
 * @brief Open/Close tearing effect line
 * @param tear -> Whether to tear
 * @return none
 */
void ST7789_TearEffect(spi_device_handle_t *handle, uint8_t tear)
{
	
	ST7789_WriteCommand(handle, tear ? 0x35 /* TEON */ : 0x34 /* TEOFF */);
	
}


/** 
 * @brief A Simple test function for ST7789
 * @param  none
 * @return  none
 */
void ST7789_Test(spi_device_handle_t *handle)
{
	ST7789_Fill_Color(handle, WHITE);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	//ST7789_WriteString(10, 20, "Speed Test", Font_11x18, RED, WHITE);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	ST7789_Fill_Color(handle, CYAN);
    vTaskDelay(500/ portTICK_PERIOD_MS);
	ST7789_Fill_Color(handle, RED);
    vTaskDelay(500/ portTICK_PERIOD_MS);
	ST7789_Fill_Color(handle, BLUE);
    vTaskDelay(500/ portTICK_PERIOD_MS);
	ST7789_Fill_Color(handle, GREEN);
    vTaskDelay(500/ portTICK_PERIOD_MS);
	ST7789_Fill_Color(handle, YELLOW);
    vTaskDelay(500/ portTICK_PERIOD_MS);
	ST7789_Fill_Color(handle, BROWN);
    vTaskDelay(500/ portTICK_PERIOD_MS);
	ST7789_Fill_Color(handle, DARKBLUE);
    vTaskDelay(500/ portTICK_PERIOD_MS);
	ST7789_Fill_Color(handle, MAGENTA);
    vTaskDelay(500/ portTICK_PERIOD_MS);
	ST7789_Fill_Color(handle, LIGHTGREEN);
    vTaskDelay(500/ portTICK_PERIOD_MS);
	ST7789_Fill_Color(handle, LGRAY);
    vTaskDelay(500/ portTICK_PERIOD_MS);
	ST7789_Fill_Color(handle, LBBLUE);
    vTaskDelay(500/ portTICK_PERIOD_MS);
	ST7789_Fill_Color(handle, WHITE);
	vTaskDelay(500/ portTICK_PERIOD_MS);

	ST7789_WriteString(handle, 10, 10, "Font test.", Font_16x26, GBLUE, WHITE);
	ST7789_WriteString(handle, 10, 50, "Hello Steve!", Font_7x10, RED, WHITE);
	ST7789_WriteString(handle, 10, 75, "Hello Steve!", Font_11x18, YELLOW, WHITE);
	ST7789_WriteString(handle, 10, 100, "Hello Steve!", Font_16x26, MAGENTA, WHITE);
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	ST7789_Fill_Color(handle, RED);
	ST7789_WriteString(handle, 10, 10, "Rect./Line.", Font_11x18, YELLOW, RED);
	ST7789_DrawRectangle(handle, 30, 30, 100, 100, WHITE);
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	ST7789_Fill_Color(handle, RED);
	ST7789_WriteString(handle, 10, 10, "Filled Rect.", Font_11x18, YELLOW, RED);
	ST7789_DrawFilledRectangle(handle, 30, 30, 50, 50, WHITE);
	vTaskDelay(1000 / portTICK_PERIOD_MS);


	ST7789_Fill_Color(handle, RED);
	ST7789_WriteString(handle, 10, 10, "Circle.", Font_11x18, YELLOW, RED);
	ST7789_DrawCircle(handle, 60, 60, 25, WHITE);
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	ST7789_Fill_Color(handle, RED);
	ST7789_WriteString(handle, 10, 10, "Filled Cir.", Font_11x18, YELLOW, RED);
	ST7789_DrawFilledCircle(handle, 60, 60, 25, WHITE);
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	ST7789_Fill_Color(handle, RED);
	ST7789_WriteString(handle, 10, 10, "Triangle", Font_11x18, YELLOW, RED);
	ST7789_DrawTriangle(handle, 30, 30, 30, 70, 60, 40, WHITE);
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	ST7789_Fill_Color(handle, RED);
	ST7789_WriteString(handle, 10, 10, "Filled Tri", Font_11x18, YELLOW, RED);
	ST7789_DrawFilledTriangle(handle, 30, 30, 30, 70, 60, 40, WHITE);
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	//	If FLASH cannot storage anymore datas, please delete codes below.
	ST7789_Fill_Color(handle, WHITE);
	//ST7789_DrawImage(0, 0, 128, 128, (uint16_t *)saber);
	vTaskDelay(3000/ portTICK_PERIOD_MS);
}




