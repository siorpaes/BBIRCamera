/*!
* @file Adafruit_ILI9341.cpp
*
* @mainpage Adafruit ILI9341 TFT Displays
*
* @section intro_sec Introduction
*
* This is the documentation for Adafruit's ILI9341 driver for the
* Arduino platform. 
*
* This library works with the Adafruit 2.8" Touch Shield V2 (SPI)
*    http://www.adafruit.com/products/1651
*
* Adafruit 2.4" TFT LCD with Touchscreen Breakout w/MicroSD Socket - ILI9341
*    https://www.adafruit.com/product/2478
*
* 2.8" TFT LCD with Touchscreen Breakout Board w/MicroSD Socket - ILI9341
*    https://www.adafruit.com/product/1770
*
* 2.2" 18-bit color TFT LCD display with microSD card breakout - ILI9340
*    https://www.adafruit.com/product/1770
*
* TFT FeatherWing - 2.4" 320x240 Touchscreen For All Feathers 
*    https://www.adafruit.com/product/3315
*
* These displays use SPI to communicate, 4 or 5 pins are required
* to interface (RST is optional).
*
* Adafruit invests time and resources providing this open source code,
* please support Adafruit and open-source hardware by purchasing
* products from Adafruit!
*
* @section dependencies Dependencies
*
* This library depends on <a href="https://github.com/adafruit/Adafruit_GFX">
* Adafruit_GFX</a> being present on your system. Please make sure you have
* installed the latest version before using this library.
*
* @section author Author
*
* Written by Limor "ladyada" Fried for Adafruit Industries.
*
* @section license License
*
* BSD license, all text here must be included in any redistribution.
*
*/

#include <stdint.h>
#include <limits.h>
#include <Adafruit_ILI9341.h>
#include "main.h"
#include "stm32f4xx_hal.h"

#define MADCTL_MY  0x80     ///< Bottom to top
#define MADCTL_MX  0x40     ///< Right to left
#define MADCTL_MV  0x20     ///< Reverse Mode
#define MADCTL_ML  0x10     ///< LCD refresh Bottom to top
#define MADCTL_RGB 0x00     ///< Red-Green-Blue pixel order
#define MADCTL_BGR 0x08     ///< Blue-Green-Red pixel order
#define MADCTL_MH  0x04     ///< LCD refresh right to left

/*
 * Control Pins
 * */
#define SPI_DC_HIGH()           HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET)
#define SPI_DC_LOW()            HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET)
#define SPI_CS_HIGH()           HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET)
#define SPI_CS_LOW()            HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET)




/*
 * Hardware SPI Macros
 * */
#if 0
#define HSPI_WRITE(b)            SPI_OBJECT.transfer((uint8_t)(b))    ///< Hardware SPI write 8 bits
#define HSPI_READ()              HSPI_WRITE(0)    ///< Hardware SPI read 8 bits
#define HSPI_WRITE16(s)          HSPI_WRITE((s) >> 8); HSPI_WRITE(s)  ///< Hardware SPI write 16 bits
#define HSPI_WRITE32(l)          HSPI_WRITE((l) >> 24); HSPI_WRITE((l) >> 16); HSPI_WRITE((l) >> 8); HSPI_WRITE(l)          ///< Hardware SPI write 32 bits
#define HSPI_WRITE_PIXELS(c,l)   for(uint32_t i=0; i<(l); i+=2){ HSPI_WRITE(((uint8_t*)(c))[i+1]); HSPI_WRITE(((uint8_t*)(c))[i]); }       ///< Hardware SPI write 'l' pixels 16-bits each

/*
 * Final SPI Macros
 * */
#define SPI_BEGIN()             {SPI_OBJECT.begin();}          ///< SPI initialize
#define SPI_BEGIN_TRANSACTION() {HSPI_BEGIN_TRANSACTION();}    ///< SPI begin transaction
#define SPI_END_TRANSACTION()   {HSPI_END_TRANSACTION();}      ///< SPI end transaction
#define SPI_WRITE16(s)          {HSPI_WRITE16(s);}else{SSPI_WRITE16(s);}  ///< SPI write 16 bits
#define SPI_WRITE32(l)          {HSPI_WRITE32(l);}else{SSPI_WRITE32(l);}  ///< SPI write 32 bits
#define SPI_WRITE_PIXELS(c,l)   {HSPI_WRITE_PIXELS(c,l);}else{SSPI_WRITE_PIXELS(c,l);}  ///< SPI write 'l' pixels of 16-bits each


#else

extern SPI_HandleTypeDef hspi2;

void HSPI_WRITE(uint8_t data)
{
	HAL_StatusTypeDef err;

	err = HAL_SPI_Transmit(&hspi2, &data, 1, 0xffff);
	if(err != HAL_OK)
		while(1);
}

uint8_t HSPI_READ(void)
{
	HAL_StatusTypeDef err;
	uint8_t data;
	uint8_t dummy;
	
	err = HAL_SPI_TransmitReceive(&hspi2, &dummy, &data, 1, 0xffff);
	if(err != HAL_OK)
		while(1);
	
	return data;
}

/* TODO: check endianness */
void HSPI_WRITE16(uint16_t data)
{
	HAL_StatusTypeDef err;
	uint8_t data8;

	data8 = (data >> 8);
	err = HAL_SPI_Transmit(&hspi2, &data8, 1, 0xffff);
	if(err != HAL_OK)
		while(1);
	
	data8 = data & 0xff;
	err = HAL_SPI_Transmit(&hspi2, &data8, 1, 0xffff);
	if(err != HAL_OK)
		while(1);
}

void HSPI_WRITE32(uint32_t data)
{
	HAL_StatusTypeDef err;
	uint8_t data8;
	
	data8 = data >> 24;
	err = HAL_SPI_Transmit(&hspi2, &data8, 1, 0xffff);
	if(err != HAL_OK)
		while(1);

	data8 = data >> 16;
	err = HAL_SPI_Transmit(&hspi2, &data8, 1, 0xffff);
	if(err != HAL_OK)
		while(1);

	data8 = data >> 8;
	err = HAL_SPI_Transmit(&hspi2, &data8, 1, 0xffff);
	if(err != HAL_OK)
		while(1);

	data8 = data & 0xff;
	err = HAL_SPI_Transmit(&hspi2, &data8, 1, 0xffff);
	if(err != HAL_OK)
		while(1);

}

#define HSPI_WRITE_PIXELS(c,l)   for(uint32_t i=0; i<(l); i+=2){ HSPI_WRITE(((uint8_t*)(c))[i+1]); HSPI_WRITE(((uint8_t*)(c))[i]); }       ///< Hardware SPI write 'l' pixels 16-bits each


/*
 * Final SPI Macros
 * */
#define SPI_BEGIN()             {}          ///< SPI initialize
#define SPI_BEGIN_TRANSACTION() {HSPI_BEGIN_TRANSACTION();}    ///< SPI begin transaction
#define SPI_END_TRANSACTION()   {HSPI_END_TRANSACTION();}      ///< SPI end transaction
#define SPI_WRITE16(s)          {HSPI_WRITE16(s);}  ///< SPI write 16 bits
#define SPI_WRITE32(l)          {HSPI_WRITE32(l);}  ///< SPI write 32 bits
#define SPI_WRITE_PIXELS(c,l)   {HSPI_WRITE_PIXELS(c,l);}  ///< SPI write 'l' pixels of 16-bits each

#endif



int _width = ILI9341_TFTWIDTH;
int _height = ILI9341_TFTHEIGHT;

/**************************************************************************/
/*!
    @brief  Pass 8-bit (each) R,G,B, get back 16-bit packed color
            This function converts 8-8-8 RGB data to 16-bit 5-6-5
    @param    red   Red 8 bit color
    @param    green Green 8 bit color
    @param    blue  Blue 8 bit color
    @return   Unsigned 16-bit down-sampled color in 5-6-5 format
*/
/**************************************************************************/
uint16_t color565(uint8_t red, uint8_t green, uint8_t blue) {
    return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | ((blue & 0xF8) >> 3);
}


/**************************************************************************/
/*!
    @brief   Initialize ILI9341 chip
    Connects to the ILI9341 over SPI and sends initialization procedure commands
    @param    freq  Desired SPI clock frequency
*/
/**************************************************************************/
void ILI9341Init(void)
{
	_width  = ILI9341_TFTWIDTH;
	_height = ILI9341_TFTHEIGHT;
	
	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  // Hardware SPI
	
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(200);

	startWrite();

	writeCommand(0xEF);
	spiWrite(0x03);
	spiWrite(0x80);
	spiWrite(0x02);

	writeCommand(0xCF);
	spiWrite(0x00);
	spiWrite(0XC1);
	spiWrite(0X30);

	writeCommand(0xED);
	spiWrite(0x64);
	spiWrite(0x03);
	spiWrite(0X12);
	spiWrite(0X81);

	writeCommand(0xE8);
	spiWrite(0x85);
	spiWrite(0x00);
	spiWrite(0x78);

	writeCommand(0xCB);
	spiWrite(0x39);
	spiWrite(0x2C);
	spiWrite(0x00);
	spiWrite(0x34);
	spiWrite(0x02);

	writeCommand(0xF7);
	spiWrite(0x20);

	writeCommand(0xEA);
	spiWrite(0x00);
	spiWrite(0x00);

	writeCommand(ILI9341_PWCTR1);    //Power control
	spiWrite(0x23);   //VRH[5:0]

	writeCommand(ILI9341_PWCTR2);    //Power control
	spiWrite(0x10);   //SAP[2:0];BT[3:0]

	writeCommand(ILI9341_VMCTR1);    //VCM control
	spiWrite(0x3e);
	spiWrite(0x28);

	writeCommand(ILI9341_VMCTR2);    //VCM control2
	spiWrite(0x86);  //--

	writeCommand(ILI9341_MADCTL);    // Memory Access Control
	spiWrite(0x48);

	writeCommand(ILI9341_VSCRSADD); // Vertical scroll
	SPI_WRITE16(0);                 // Zero

	writeCommand(ILI9341_PIXFMT);
	spiWrite(0x55);

	writeCommand(ILI9341_FRMCTR1);
	spiWrite(0x00);
	spiWrite(0x18);

	writeCommand(ILI9341_DFUNCTR);    // Display Function Control
	spiWrite(0x08);
	spiWrite(0x82);
	spiWrite(0x27);

	writeCommand(0xF2);    // 3Gamma Function Disable
	spiWrite(0x00);

	writeCommand(ILI9341_GAMMASET);    //Gamma curve selected
	spiWrite(0x01);

	writeCommand(ILI9341_GMCTRP1);    //Set Gamma
	spiWrite(0x0F);
	spiWrite(0x31);
	spiWrite(0x2B);
	spiWrite(0x0C);
	spiWrite(0x0E);
	spiWrite(0x08);
	spiWrite(0x4E);
	spiWrite(0xF1);
	spiWrite(0x37);
	spiWrite(0x07);
	spiWrite(0x10);
	spiWrite(0x03);
	spiWrite(0x0E);
	spiWrite(0x09);
	spiWrite(0x00);

	writeCommand(ILI9341_GMCTRN1);    //Set Gamma
	spiWrite(0x00);
	spiWrite(0x0E);
	spiWrite(0x14);
	spiWrite(0x03);
	spiWrite(0x11);
	spiWrite(0x07);
	spiWrite(0x31);
	spiWrite(0xC1);
	spiWrite(0x48);
	spiWrite(0x08);
	spiWrite(0x0F);
	spiWrite(0x0C);
	spiWrite(0x31);
	spiWrite(0x36);
	spiWrite(0x0F);

	writeCommand(ILI9341_SLPOUT);    //Exit Sleep
	//HAL_Delay(12);
	writeCommand(ILI9341_DISPON);    //Display on
	//HAL_Delay(12);
	endWrite();

}


/**************************************************************************/
/*!
    @brief   Set origin of (0,0) and orientation of TFT display
    @param   m  The index for rotation, from 0-3 inclusive
*/
/**************************************************************************/
void setRotation(uint8_t m) {
		static int rotation;
    rotation = m % 4; // can't be higher than 3
    switch (rotation) {
        case 0:
            m = (MADCTL_MX | MADCTL_BGR);
            _width  = ILI9341_TFTWIDTH;
            _height = ILI9341_TFTHEIGHT;
            break;
        case 1:
            m = (MADCTL_MV | MADCTL_BGR);
            _width  = ILI9341_TFTHEIGHT;
            _height = ILI9341_TFTWIDTH;
            break;
        case 2:
            m = (MADCTL_MY | MADCTL_BGR);
            _width  = ILI9341_TFTWIDTH;
            _height = ILI9341_TFTHEIGHT;
            break;
        case 3:
            m = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
            _width  = ILI9341_TFTHEIGHT;
            _height = ILI9341_TFTWIDTH;
            break;
    }

    startWrite();
    writeCommand(ILI9341_MADCTL);
    spiWrite(m);
    endWrite();
}

/**************************************************************************/
/*!
    @brief   Enable/Disable display color inversion
    @param   invert True to invert, False to have normal color
*/
/**************************************************************************/
void invertDisplay(int invert) {
    startWrite();
    writeCommand(invert ? ILI9341_INVON : ILI9341_INVOFF);
    endWrite();
}

/**************************************************************************/
/*!
    @brief   Scroll display memory
    @param   y How many pixels to scroll display by
*/
/**************************************************************************/
void scrollTo(uint16_t y) {
    startWrite();
    writeCommand(ILI9341_VSCRSADD);
    SPI_WRITE16(y);
    endWrite();
}

/**************************************************************************/
/*!
    @brief   Set the "address window" - the rectangle we will write to RAM with the next chunk of SPI data writes. The ILI9341 will automatically wrap the data as each row is filled
    @param   x  TFT memory 'x' origin
    @param   y  TFT memory 'y' origin
    @param   w  Width of rectangle
    @param   h  Height of rectangle
*/
/**************************************************************************/
void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    uint32_t xa = ((uint32_t)x << 16) | (x+w-1);
    uint32_t ya = ((uint32_t)y << 16) | (y+h-1);
    writeCommand(ILI9341_CASET); // Column addr set
    SPI_WRITE32(xa);
    writeCommand(ILI9341_PASET); // Row addr set
    SPI_WRITE32(ya);
    writeCommand(ILI9341_RAMWR); // write to RAM
}

/**************************************************************************/
/*!
    @brief   Blit 1 pixel of color without setting up SPI transaction
    @param   color 16-bits of 5-6-5 color data
*/
/**************************************************************************/
void pushColor(uint16_t color) {
    SPI_WRITE16(color);
}

/**************************************************************************/
/*!
    @brief   Blit 1 pixel of color without setting up SPI transaction
    @param   color 16-bits of 5-6-5 color data
*/
/**************************************************************************/
void writePixelColor(uint16_t color){
    SPI_WRITE16(color);
}

/**************************************************************************/
/*!
    @brief   Blit 'len' pixels of color without setting up SPI transaction
    @param   colors Array of 16-bit 5-6-5 color data
    @param   len Number of 16-bit pixels in colors array
*/
/**************************************************************************/
void writePixels(uint16_t * colors, uint32_t len){
    SPI_WRITE_PIXELS((uint8_t*)colors , len * 2);
}

/**************************************************************************/
/*!
    @brief   Blit 'len' pixels of a single color without setting up SPI transaction
    @param   color 16-bits of 5-6-5 color data
    @param   len Number of 16-bit pixels you want to write out with same color
*/
/**************************************************************************/
void writeColor(uint16_t color, uint32_t len){
    uint8_t hi = color >> 8, lo = color;
    for (uint32_t t=len; t; t--){
				HSPI_WRITE(hi);
				HSPI_WRITE(lo);
		}
}

/**************************************************************************/
/*!
   @brief  Draw a single pixel, DOES NOT set up SPI transaction
    @param    x  TFT X location
    @param    y  TFT Y location
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void writePixel(int16_t x, int16_t y, uint16_t color) {
    if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;
    setAddrWindow(x,y,1,1);
    writePixelColor(color);
}

/**************************************************************************/
/*!
   @brief  Fill a rectangle, DOES NOT set up SPI transaction
    @param    x  TFT X location begin
    @param    y  TFT Y location begin
    @param    w  Width of rectangle
    @param    h  Height of rectangle
    @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color){
    if((x >= _width) || (y >= _height)) return;
    int16_t x2 = x + w - 1, y2 = y + h - 1;
    if((x2 < 0) || (y2 < 0)) return;

    // Clip left/top
    if(x < 0) {
        x = 0;
        w = x2 + 1;
    }
    if(y < 0) {
        y = 0;
        h = y2 + 1;
    }

    // Clip right/bottom
    if(x2 >= _width)  w = _width  - x;
    if(y2 >= _height) h = _height - y;

    int32_t len = (int32_t)w * h;
    setAddrWindow(x, y, w, h);
    writeColor(color, len);
}


/**************************************************************************/
/*!
   @brief  Draw a vertical line, DOES NOT set up SPI transaction
    @param    x  TFT X location begin
    @param    y  TFT Y location begin
    @param    l  Length of line in pixels
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void writeFastVLine(int16_t x, int16_t y, int16_t l, uint16_t color){
    writeFillRect(x, y, 1, l, color);
}


/**************************************************************************/
/*!
   @brief  Draw a horizontal line, DOES NOT set up SPI transaction
    @param    x  TFT X location begin
    @param    y  TFT Y location begin
    @param    l  Length of line in pixels
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void writeFastHLine(int16_t x, int16_t y, int16_t l, uint16_t color){
    writeFillRect(x, y, l, 1, color);
}

/**************************************************************************/
/*!
   @brief  Draw a single pixel, includes code for SPI transaction
    @param    x  TFT X location
    @param    y  TFT Y location
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void drawPixel(int16_t x, int16_t y, uint16_t color){
    startWrite();
    writePixel(x, y, color);
    endWrite();
}

/**************************************************************************/
/*!
   @brief  Draw a vertical line, includes code for SPI transaction
    @param    x  TFT X location begin
    @param    y  TFT Y location begin
    @param    l  Length of line in pixels
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void drawFastVLine(int16_t x, int16_t y,
        int16_t l, uint16_t color) {
    startWrite();
    writeFastVLine(x, y, l, color);
    endWrite();
}

/**************************************************************************/
/*!
   @brief  Draw a horizontal line, includes code for SPI transaction
    @param    x  TFT X location begin
    @param    y  TFT Y location begin
    @param    l  Length of line in pixels
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void drawFastHLine(int16_t x, int16_t y,
        int16_t l, uint16_t color) {
    startWrite();
    writeFastHLine(x, y, l, color);
    endWrite();
}

/**************************************************************************/
/*!
   @brief  Fill a rectangle, includes code for SPI transaction
    @param    x  TFT X location begin
    @param    y  TFT Y location begin
    @param    w  Width of rectangle
    @param    h  Height of rectangle
    @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
        uint16_t color) {
    startWrite();
    writeFillRect(x,y,w,h,color);
    endWrite();
}

/**************************************************************************/
/*!
   @brief  Draw RGB rectangle of data from RAM to a location on screen
   Adapted from https://github.com/PaulStoffregen/ILI9341_t3
   by Marc MERLIN. See examples/pictureEmbed to use this.
   5/6/2017: function name and arguments have changed for compatibility
   with current GFX library and to avoid naming problems in prior
   implementation.  Formerly drawBitmap() with arguments in different order.

    @param    x  TFT X location begin
    @param    y  TFT Y location begin
    @param    pcolors Pointer to 16-bit color data
    @param    w  Width of pcolors rectangle
    @param    h  Height of pcolors rectangle
*/
/**************************************************************************/
void drawRGBBitmap(int16_t x, int16_t y,
  uint16_t *pcolors, int16_t w, int16_t h) {

    int16_t x2, y2; // Lower-right coord
    if(( x             >= _width ) ||      // Off-edge right
       ( y             >= _height) ||      // " top
       ((x2 = (x+w-1)) <  0      ) ||      // " left
       ((y2 = (y+h-1)) <  0)     ) return; // " bottom

    int16_t bx1=0, by1=0, // Clipped top-left within bitmap
            saveW=w;      // Save original bitmap width value
    if(x < 0) { // Clip left
        w  +=  x;
        bx1 = -x;
        x   =  0;
    }
    if(y < 0) { // Clip top
        h  +=  y;
        by1 = -y;
        y   =  0;
    }
    if(x2 >= _width ) w = _width  - x; // Clip right
    if(y2 >= _height) h = _height - y; // Clip bottom

    pcolors += by1 * saveW + bx1; // Offset bitmap ptr to clipped top-left
    startWrite();
    setAddrWindow(x, y, w, h); // Clipped area
    while(h--) { // For each (clipped) scanline...
      writePixels(pcolors, w); // Push one (clipped) row
      pcolors += saveW; // Advance pointer by one full (unclipped) line
    }
    endWrite();
}


/**************************************************************************/
/*!
   @brief  Read 8 bits of data from ILI9341 configuration memory. NOT from RAM!
           This is highly undocumented/supported, it's really a hack but kinda works?
    @param    command  The command register to read data from
    @param    index  The byte index into the command to read from
    @return   Unsigned 8-bit data read from ILI9341 register
*/
/**************************************************************************/
uint8_t readcommand8(uint8_t command, uint8_t index) {

    startWrite();
    writeCommand(0xD9);  // woo sekret command?
    spiWrite(0x10 + index);
    writeCommand(command);
    uint8_t r = spiRead();
    endWrite();

		return r;
}


/**************************************************************************/
/*!
   @brief  Begin SPI transaction, for software or hardware SPI
*/
/**************************************************************************/
void startWrite(void){
    SPI_CS_LOW();
}

/**************************************************************************/
/*!
   @brief  End SPI transaction, for software or hardware SPI
*/
/**************************************************************************/
void endWrite(void){
    SPI_CS_HIGH();
}

/**************************************************************************/
/*!
   @brief  Write 8-bit data to command/register (DataCommand line low).
   Does not set up SPI transaction.
   @param  cmd The command/register to transmit
*/
/**************************************************************************/
void writeCommand(uint8_t cmd){
    SPI_DC_LOW();
    spiWrite(cmd);
    SPI_DC_HIGH();
}


/**************************************************************************/
/*!
   @brief  Read 8-bit data via hardware or software SPI. Does not set up SPI transaction.
   @returns One byte of data from SPI
*/
/**************************************************************************/
uint8_t spiRead() {
	return HSPI_READ();
}

/**************************************************************************/
/*!
   @brief  Write 8-bit data via hardware or software SPI. Does not set up SPI transaction.
   @param  b Byte of data to write over SPI
*/
/**************************************************************************/
void spiWrite(uint8_t b) {
	HSPI_WRITE(b);
}

