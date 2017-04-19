/**************************************************************************************************
  Filename:       hal_lcd.c
  Revised:        $Date: 2012-09-30 16:36:36 -0700 (Sun, 30 Sep 2012) $
  Revision:       $Revision: 31658 $

  Description:    This file contains the interface to the HAL LCD Service.


  Copyright 2007-2012 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.

  Move To OLED by SSD1306
**************************************************************************************************/

/**************************************************************************************************
 *                                           INCLUDES
 **************************************************************************************************/
#include "hal_types.h"
#include "hal_lcd.h"
#include "OSAL.h"
#include "OnBoard.h"
#include "hal_assert.h"

#if defined (ZTOOL_P1) || defined (ZTOOL_P2)
  #include "DebugTrace.h"
#endif

/**************************************************************************************************
 *                                          CONSTANTS
 **************************************************************************************************/
/*
  LCD pins

  //control
  P1.3 - LCD_MODE
  P0.0 - LCD_FLASH_RESET
  //P1.2 - LCD_CS

  //spi
  P1.5 - CLK
  P1.6 - MOSI
  P1.7 - MISO
*/

/* LCD Control lines */
#define HAL_LCD_MODE_PORT 1
#define HAL_LCD_MODE_PIN  3

#define HAL_LCD_RESET_PORT 0
#define HAL_LCD_RESET_PIN  0

// #define HAL_LCD_CS_PORT 1
// #define HAL_LCD_CS_PIN  2

/* LCD SPI lines */
#define HAL_LCD_CLK_PORT 1
#define HAL_LCD_CLK_PIN  5

#define HAL_LCD_MOSI_PORT 1
#define HAL_LCD_MOSI_PIN  6

#define HAL_LCD_MISO_PORT 1
#define HAL_LCD_MISO_PIN  7

/* SPI settings */
#define HAL_SPI_CLOCK_POL_LO       0x00
#define HAL_SPI_CLOCK_PHA_0        0x00
#define HAL_SPI_TRANSFER_MSB_FIRST 0x20

/* LCD lines */
#define LCD_MAX_LINE_COUNT              8

/* Defines for HW LCD */

/* Set power save mode */
#define OSC_OFF                         0x00
#define OSC_ON                          0x01
#define POWER_SAVE_OFF                  0x00
#define POWER_SAVE_ON                   0x02
#define SET_POWER_SAVE_MODE(options)    HalLcd_HW_Control(0x0C | (options))

/* Function Set */
#define CGROM                           0x00
#define CGRAM                           0x01
#define COM_FORWARD                     0x00
#define COM_BACKWARD                    0x02
#define TWO_LINE                        0x00
#define THREE_LINE                      0x04
#define FUNCTION_SET(options)           HalLcd_HW_Control(0x10 | (options))

/* Set Display Start Line */
#define LINE1                           0x00
#define LINE2                           0x01
#define LINE3                           0x02
#define LINE4                           0x03
#define SET_DISPLAY_START_LINE(line)    HalLcd_HW_Control(0x18 | (line))

/* Bias control */
#define BIAS_1_5                        0x00
#define BIAS_1_4                        0x01
#define SET_BIAS_CTRL(bias)             HalLcd_HW_Control(0x1C | (bias))

/* Power control */
#define VOLTAGE_DIVIDER_OFF             0x00
#define VOLTAGE_DIVIDER_ON              0x01
#define CONVERTER_AND_REG_OFF           0x00
#define CONVERTER_AND_REG_ON            0x04
#define SET_POWER_CTRL(options)         HalLcd_HW_Control(0x20 | (options))

// Set display control
#define DISPLAY_CTRL_ON                 0x01
#define DISPLAY_CTRL_OFF                0x00
#define DISPLAY_CTRL_BLINK_ON           0x02
#define DISPLAY_CTRL_BLINK_OFF          0x00
#define DISPLAY_CTRL_CURSOR_ON          0x04
#define DISPLAY_CTRL_CURSOR_OFF         0x00
#define SET_DISPLAY_CTRL(options)       HalLcd_HW_Control(0x28 | (options))

/* Set DD/ CGRAM address */
#define SET_DDRAM_ADDR(charIndex)       HalLcd_HW_Control(0x80 | (charIndex))
#define SET_GCRAM_CHAR(specIndex)       HalLcd_HW_Control(0xC0 | (specIndex))

/* Set ICONRAM address */
#define CONTRAST_CTRL_REGISTER          0x10
#define SET_ICONRAM_ADDR(addr)          HalLcd_HW_Control(0x40 | (addr))

/* Set double height */
#define LINE_1_AND_2                    0x01
#define LINE_2_AND_3                    0x02
#define NORMAL_DISPLAY                  0x00
#define SET_DOUBLE_HEIGHT(options)      HalLcd_HW_Control(0x08 | (options))

/**************************************************************************************************
 *                                           MACROS
 **************************************************************************************************/

#define HAL_IO_SET(port, pin, val)        HAL_IO_SET_PREP(port, pin, val)
#define HAL_IO_SET_PREP(port, pin, val)   st( P##port##_##pin = val; )

#define HAL_CONFIG_IO_OUTPUT(port, pin, val)      HAL_CONFIG_IO_OUTPUT_PREP(port, pin, val)
#define HAL_CONFIG_IO_OUTPUT_PREP(port, pin, val) st( P##port##SEL &= ~BV(pin); \
                                                      P##port##_##pin = val; \
                                                      P##port##DIR |= BV(pin); )

#define HAL_CONFIG_IO_PERIPHERAL(port, pin)      HAL_CONFIG_IO_PERIPHERAL_PREP(port, pin)
#define HAL_CONFIG_IO_PERIPHERAL_PREP(port, pin) st( P##port##SEL |= BV(pin); )



/* SPI interface control */
#define LCD_SPI_BEGIN()     HAL_IO_SET(HAL_LCD_CS_PORT,  HAL_LCD_CS_PIN,  0); /* chip select */
#define LCD_SPI_END()                                                         \
{                                                                             \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  asm("NOP");                                                                 \
  HAL_IO_SET(HAL_LCD_CS_PORT,  HAL_LCD_CS_PIN,  1); /* chip select */         \
}
/* clear the received and transmit byte status, write tx data to buffer, wait till transmit done */
#define LCD_SPI_TX(x)                   { U1CSR &= ~(BV(2) | BV(1)); U1DBUF = x; while( !(U1CSR & BV(1)) ); }
#define LCD_SPI_WAIT_RXRDY()            { while(!(U1CSR & BV(1))); }


/* Control macros */
#define LCD_DO_WRITE()        HAL_IO_SET(HAL_LCD_MODE_PORT,  HAL_LCD_MODE_PIN,  1);   /* write DATA */
#define LCD_DO_CONTROL()      HAL_IO_SET(HAL_LCD_MODE_PORT,  HAL_LCD_MODE_PIN,  0);   /* write COMMAND */

#define LCD_ACTIVATE_RESET()  HAL_IO_SET(HAL_LCD_RESET_PORT, HAL_LCD_RESET_PIN, 0);
#define LCD_RELEASE_RESET()   HAL_IO_SET(HAL_LCD_RESET_PORT, HAL_LCD_RESET_PIN, 1);

#if (HAL_LCD == TRUE)
/************************************6*8的点阵************************************/
static const unsigned char F6x8[][6] =		
{
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,// sp
0x00, 0x00, 0x00, 0x2f, 0x00, 0x00,// !
0x00, 0x00, 0x07, 0x00, 0x07, 0x00,// "
0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14,// #
0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12,// $
0x00, 0x62, 0x64, 0x08, 0x13, 0x23,// %
0x00, 0x36, 0x49, 0x55, 0x22, 0x50,// &
0x00, 0x00, 0x05, 0x03, 0x00, 0x00,// '
0x00, 0x00, 0x1c, 0x22, 0x41, 0x00,// (
0x00, 0x00, 0x41, 0x22, 0x1c, 0x00,// )
0x00, 0x14, 0x08, 0x3E, 0x08, 0x14,// *
0x00, 0x08, 0x08, 0x3E, 0x08, 0x08,// +
0x00, 0x00, 0x00, 0xA0, 0x60, 0x00,// ,
0x00, 0x08, 0x08, 0x08, 0x08, 0x08,// -
0x00, 0x00, 0x60, 0x60, 0x00, 0x00,// .
0x00, 0x20, 0x10, 0x08, 0x04, 0x02,// /
0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E,// 0
0x00, 0x00, 0x42, 0x7F, 0x40, 0x00,// 1
0x00, 0x42, 0x61, 0x51, 0x49, 0x46,// 2
0x00, 0x21, 0x41, 0x45, 0x4B, 0x31,// 3
0x00, 0x18, 0x14, 0x12, 0x7F, 0x10,// 4
0x00, 0x27, 0x45, 0x45, 0x45, 0x39,// 5
0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30,// 6
0x00, 0x01, 0x71, 0x09, 0x05, 0x03,// 7
0x00, 0x36, 0x49, 0x49, 0x49, 0x36,// 8
0x00, 0x06, 0x49, 0x49, 0x29, 0x1E,// 9
0x00, 0x00, 0x36, 0x36, 0x00, 0x00,// :
0x00, 0x00, 0x56, 0x36, 0x00, 0x00,// ;
0x00, 0x08, 0x14, 0x22, 0x41, 0x00,// <
0x00, 0x14, 0x14, 0x14, 0x14, 0x14,// =
0x00, 0x00, 0x41, 0x22, 0x14, 0x08,// >
0x00, 0x02, 0x01, 0x51, 0x09, 0x06,// ?
0x00, 0x32, 0x49, 0x59, 0x51, 0x3E,// @
0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C,// A
0x00, 0x7F, 0x49, 0x49, 0x49, 0x36,// B
0x00, 0x3E, 0x41, 0x41, 0x41, 0x22,// C
0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C,// D
0x00, 0x7F, 0x49, 0x49, 0x49, 0x41,// E
0x00, 0x7F, 0x09, 0x09, 0x09, 0x01,// F
0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A,// G
0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F,// H
0x00, 0x00, 0x41, 0x7F, 0x41, 0x00,// I
0x00, 0x20, 0x40, 0x41, 0x3F, 0x01,// J
0x00, 0x7F, 0x08, 0x14, 0x22, 0x41,// K
0x00, 0x7F, 0x40, 0x40, 0x40, 0x40,// L
0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F,// M
0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F,// N
0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E,// O
0x00, 0x7F, 0x09, 0x09, 0x09, 0x06,// P
0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E,// Q
0x00, 0x7F, 0x09, 0x19, 0x29, 0x46,// R
0x00, 0x46, 0x49, 0x49, 0x49, 0x31,// S
0x00, 0x01, 0x01, 0x7F, 0x01, 0x01,// T
0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F,// U
0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F,// V
0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F,// W
0x00, 0x63, 0x14, 0x08, 0x14, 0x63,// X
0x00, 0x07, 0x08, 0x70, 0x08, 0x07,// Y
0x00, 0x61, 0x51, 0x49, 0x45, 0x43,// Z
0x00, 0x00, 0x7F, 0x41, 0x41, 0x00,// [
0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55,// 55
0x00, 0x00, 0x41, 0x41, 0x7F, 0x00,// ]
0x00, 0x04, 0x02, 0x01, 0x02, 0x04,// ^
0x00, 0x40, 0x40, 0x40, 0x40, 0x40,// _
0x00, 0x00, 0x01, 0x02, 0x04, 0x00,// '
0x00, 0x20, 0x54, 0x54, 0x54, 0x78,// a
0x00, 0x7F, 0x48, 0x44, 0x44, 0x38,// b
0x00, 0x38, 0x44, 0x44, 0x44, 0x20,// c
0x00, 0x38, 0x44, 0x44, 0x48, 0x7F,// d
0x00, 0x38, 0x54, 0x54, 0x54, 0x18,// e
0x00, 0x08, 0x7E, 0x09, 0x01, 0x02,// f
0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C,// g
0x00, 0x7F, 0x08, 0x04, 0x04, 0x78,// h
0x00, 0x00, 0x44, 0x7D, 0x40, 0x00,// i
0x00, 0x40, 0x80, 0x84, 0x7D, 0x00,// j
0x00, 0x7F, 0x10, 0x28, 0x44, 0x00,// k
0x00, 0x00, 0x41, 0x7F, 0x40, 0x00,// l
0x00, 0x7C, 0x04, 0x18, 0x04, 0x78,// m
0x00, 0x7C, 0x08, 0x04, 0x04, 0x78,// n
0x00, 0x38, 0x44, 0x44, 0x44, 0x38,// o
0x00, 0xFC, 0x24, 0x24, 0x24, 0x18,// p
0x00, 0x18, 0x24, 0x24, 0x18, 0xFC,// q
0x00, 0x7C, 0x08, 0x04, 0x04, 0x08,// r
0x00, 0x48, 0x54, 0x54, 0x54, 0x20,// s
0x00, 0x04, 0x3F, 0x44, 0x40, 0x20,// t
0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C,// u
0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C,// v
0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C,// w
0x00, 0x44, 0x28, 0x10, 0x28, 0x44,// x
0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C,// y
0x00, 0x44, 0x64, 0x54, 0x4C, 0x44,// z
0x14, 0x14, 0x14, 0x14, 0x14, 0x14,// horiz lines
};
/**************************************************************************************************
 *                                       LOCAL VARIABLES
 **************************************************************************************************/

static uint8 *Lcd_Line1;

/**************************************************************************************************
 *                                       FUNCTIONS - API
 **************************************************************************************************/

void HalLcd_HW_Init(void);
void HalLcd_HW_WaitUs(uint16 i);
void HalLcd_HW_ClearAllSpecChars(void);
void HalLcd_HW_Control(uint8 cmd);
void HalLcd_HW_Write(uint8 data);
void HalLcd_HW_SetContrast(uint8 value);
void OLED_Set_Pos(uint8 x, uint8 y);
void OLED_P6x8Char(uint8 x,uint8 y,char chr);
void OLED_FillLine(uint8 y,char chr);
void HalLcd_HW_WriteChar(uint8 line, uint8 col, char text);
void HalLcd_HW_WriteLine(uint8 line, const char *pText);
#endif //LCD

/**************************************************************************************************
 * @fn      HalLcdInit
 *
 * @brief   Initilize LCD Service
 *
 * @param   init - pointer to void that contains the initialized value
 *
 * @return  None
 **************************************************************************************************/
void HalLcdInit(void)
{
#if (HAL_LCD == TRUE)
  Lcd_Line1 = NULL;
  HalLcd_HW_Init();
#endif
}

/*************************************************************************************************
 *                    LCD EMULATION FUNCTIONS
 *
 * Some evaluation boards are equipped with Liquid Crystal Displays
 * (LCD) which may be used to display diagnostic information. These
 * functions provide LCD emulation, sending the diagnostic strings
 * to Z-Tool via the RS232 serial port. These functions are enabled
 * when the "LCD_SUPPORTED" compiler flag is placed in the makefile.
 *
 * Most applications update both lines (1 and 2) of the LCD whenever
 * text is posted to the device. This emulator assumes that line 1 is
 * updated first (saved locally) and the formatting and send operation
 * is triggered by receipt of line 2. Nothing will be transmitted if
 * only line 1 is updated.
 *
 *************************************************************************************************/


/**************************************************************************************************
 * @fn      HalLcdWriteString
 *
 * @brief   Write a string to the LCD
 *
 * @param   str    - pointer to the string that will be displayed
 *          option - display options
 *
 * @return  None
 **************************************************************************************************/
void HalLcdWriteString ( char *str, uint8 option)
{
#if (HAL_LCD == TRUE)

  uint8 strLen = 0;
  uint8 totalLen = 0;
  uint8 *buf;
  uint8 tmpLen;

  if ( Lcd_Line1 == NULL )
  {
    Lcd_Line1 = osal_mem_alloc( HAL_LCD_MAX_CHARS+1 );
    HalLcdWriteString( "TexasInstruments", 1 );
  }

  strLen = (uint8)osal_strlen( (char*)str );

  /* Check boundries */
  if ( strLen > HAL_LCD_MAX_CHARS )
    strLen = HAL_LCD_MAX_CHARS;

  if ( option == HAL_LCD_LINE_1 )
  {
    /* Line 1 gets saved for later */
    osal_memcpy( Lcd_Line1, str, strLen );
    Lcd_Line1[strLen] = '\0';
  }
  else
  {
    /* Line 2 triggers action */
    tmpLen = (uint8)osal_strlen( (char*)Lcd_Line1 );
    totalLen =  tmpLen + 1 + strLen + 1;
    buf = osal_mem_alloc( totalLen );
    if ( buf != NULL )
    {
      /* Concatenate strings */
      osal_memcpy( buf, Lcd_Line1, tmpLen );
      buf[tmpLen++] = ' ';
      osal_memcpy( &buf[tmpLen], str, strLen );
      buf[tmpLen+strLen] = '\0';

      /* Send it out */
#if defined (ZTOOL_P1) || defined (ZTOOL_P2)

#if defined(SERIAL_DEBUG_SUPPORTED)
      debug_str( (uint8*)buf );
#endif //LCD_SUPPORTED

#endif //ZTOOL_P1

      /* Free mem */
      osal_mem_free( buf );
    }
  }

  if (option != HAL_LCD_SERIAL_DEBUG_ONLY)
  {
    /* Display the string */
    HalLcd_HW_WriteLine (option, str);
  }

#endif //HAL_LCD

}

/**************************************************************************************************
 * @fn      HalLcdWriteValue
 *
 * @brief   Write a value to the LCD
 *
 * @param   value  - value that will be displayed
 *          radix  - 8, 10, 16
 *          option - display options
 *
 * @return  None
 **************************************************************************************************/
void HalLcdWriteValue ( uint32 value, const uint8 radix, uint8 option)
{
#if (HAL_LCD == TRUE)
  uint8 buf[HAL_LCD_MAX_BUFF];

  _ltoa( value, &buf[0], radix );
  HalLcdWriteString( (char*)buf, option );
#endif
}

/**************************************************************************************************
 * @fn      HalLcdWriteScreen
 *
 * @brief   Write a value to the LCD
 *
 * @param   line1  - string that will be displayed on line 1
 *          line2  - string that will be displayed on line 2
 *
 * @return  None
 **************************************************************************************************/
void HalLcdWriteScreen( char *line1, char *line2 )
{
#if (HAL_LCD == TRUE)
  HalLcdWriteString( line1, HAL_LCD_DEBUG_LINE_1 );
  HalLcdWriteString( line2, HAL_LCD_DEBUG_LINE_2 );
#endif
}

/**************************************************************************************************
 * @fn      HalLcdWriteStringValue
 *
 * @brief   Write a string followed by a value to the LCD
 *
 * @param   title  - Title that will be displayed before the value
 *          value  - value
 *          format - redix
 *          line   - line number
 *
 * @return  None
 **************************************************************************************************/
void HalLcdWriteStringValue( char *title, uint16 value, uint8 format, uint8 line )
{
#if (HAL_LCD == TRUE)
  uint8 tmpLen;
  uint8 buf[HAL_LCD_MAX_BUFF];
  uint32 err;

  tmpLen = (uint8)osal_strlen( (char*)title );
  osal_memcpy( buf, title, tmpLen );
  buf[tmpLen] = ' ';
  err = (uint32)(value);
  _ltoa( err, &buf[tmpLen+1], format );
  HalLcdWriteString( (char*)buf, line );		
#endif
}

/**************************************************************************************************
 * @fn      HalLcdWriteStringValue
 *
 * @brief   Write a string followed by a value to the LCD
 *
 * @param   title   - Title that will be displayed before the value
 *          value1  - value #1
 *          format1 - redix of value #1
 *          value2  - value #2
 *          format2 - redix of value #2
 *          line    - line number
 *
 * @return  None
 **************************************************************************************************/
void HalLcdWriteStringValueValue( char *title, uint16 value1, uint8 format1,
                                  uint16 value2, uint8 format2, uint8 line )
{

#if (HAL_LCD == TRUE)

  uint8 tmpLen;
  uint8 buf[HAL_LCD_MAX_BUFF];
  uint32 err;

  tmpLen = (uint8)osal_strlen( (char*)title );
  if ( tmpLen )
  {
    osal_memcpy( buf, title, tmpLen );
    buf[tmpLen++] = ' ';
  }

  err = (uint32)(value1);
  _ltoa( err, &buf[tmpLen], format1 );
  tmpLen = (uint8)osal_strlen( (char*)buf );

  buf[tmpLen++] = ',';
  buf[tmpLen++] = ' ';
  err = (uint32)(value2);
  _ltoa( err, &buf[tmpLen], format2 );

  HalLcdWriteString( (char *)buf, line );		

#endif
}

/**************************************************************************************************
 * @fn      HalLcdDisplayPercentBar
 *
 * @brief   Display percentage bar on the LCD
 *
 * @param   title   -
 *          value   -
 *
 * @return  None
 **************************************************************************************************/
void HalLcdDisplayPercentBar( char *title, uint8 value )
{
#if (HAL_LCD == TRUE)

  uint8 percent;
  uint8 leftOver;
  uint8 buf[17];
  uint32 err;
  uint8 x;

  /* Write the title: */
  HalLcdWriteString( title, HAL_LCD_LINE_1 );

  if ( value > 100 )
    value = 100;

  /* convert to blocks */
  percent = (uint8)(value / 10);
  leftOver = (uint8)(value % 10);

  /* Make window */
  osal_memcpy( buf, "[          ]  ", 15 );

  for ( x = 0; x < percent; x ++ )
  {
    buf[1+x] = '>';
  }

  if ( leftOver >= 5 )
    buf[1+x] = '+';

  err = (uint32)value;
  _ltoa( err, (uint8*)&buf[13], 10 );

  HalLcdWriteString( (char*)buf, HAL_LCD_LINE_2 );

#endif

}

#if (HAL_LCD == TRUE)
/**************************************************************************************************
 *                                    HARDWARE LCD
 **************************************************************************************************/

/**************************************************************************************************
 * @fn      halLcd_ConfigIO
 *
 * @brief   Configure IO lines needed for LCD control.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
static void halLcd_ConfigIO(void)
{
  /* GPIO configuration */
  HAL_CONFIG_IO_OUTPUT(HAL_LCD_MODE_PORT,  HAL_LCD_MODE_PIN,  1);
  HAL_CONFIG_IO_OUTPUT(HAL_LCD_RESET_PORT, HAL_LCD_RESET_PIN, 1);
  // HAL_CONFIG_IO_OUTPUT(HAL_LCD_CS_PORT,    HAL_LCD_CS_PIN,    1);
}

/**************************************************************************************************
 * @fn      halLcd_ConfigSPI
 *
 * @brief   Configure SPI lines needed for talking to LCD.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
static void halLcd_ConfigSPI(void)
{
  /* UART/SPI Peripheral configuration */

   uint8 baud_exponent;
   uint8 baud_mantissa;

  /* Set SPI on UART 1 alternative 2 */
  PERCFG |= 0x02;

  /* Configure clk, master out and master in lines */
  HAL_CONFIG_IO_PERIPHERAL(HAL_LCD_CLK_PORT,  HAL_LCD_CLK_PIN);
  HAL_CONFIG_IO_PERIPHERAL(HAL_LCD_MOSI_PORT, HAL_LCD_MOSI_PIN);
  HAL_CONFIG_IO_PERIPHERAL(HAL_LCD_MISO_PORT, HAL_LCD_MISO_PIN);


  /* Set SPI speed to 1 MHz (the values assume system clk of 32MHz)
   * Confirm on board that this results in 1MHz spi clk.
   */
  baud_exponent = 15;
  baud_mantissa =  0;

  /* Configure SPI */
  U1UCR  = 0x80;      /* Flush and goto IDLE state. 8-N-1. */
  U1CSR  = 0x00;      /* SPI mode, master. */
  U1GCR  = HAL_SPI_TRANSFER_MSB_FIRST | HAL_SPI_CLOCK_PHA_0 | HAL_SPI_CLOCK_POL_LO | baud_exponent;
  U1BAUD = baud_mantissa;
}

/**************************************************************************************************
 * @fn      HalLcd_HW_Init
 *
 * @brief   Initilize HW LCD Driver.
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalLcd_HW_Init(void)
{
  /* Initialize LCD IO lines */
  halLcd_ConfigIO();

  /* Initialize SPI */
  halLcd_ConfigSPI();

  /* Perform reset */
  LCD_ACTIVATE_RESET();
  HalLcd_HW_WaitUs(50000); // 50 ms
  LCD_RELEASE_RESET();
  HalLcd_HW_WaitUs(15); // 15 us

  /* Perform the initialization sequence */
  HalLcd_HW_Control(0xAE);//--turn off oled panel
  HalLcd_HW_Control(0x00);//---set low column address
  HalLcd_HW_Control(0x10);//---set high column address
  HalLcd_HW_Control(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
  HalLcd_HW_Control(0x81);//--set contrast control register
  HalLcd_HW_Control(0xCF); // Set SEG Output Current Brightness
  HalLcd_HW_Control(0xA1);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
  HalLcd_HW_Control(0xC8);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
  HalLcd_HW_Control(0xA6);//--set normal display
  HalLcd_HW_Control(0xA8);//--set multiplex ratio(1 to 64)
  HalLcd_HW_Control(0x3f);//--1/64 duty
  HalLcd_HW_Control(0xD3);//-set display offset Shift Mapping RAM Counter (0x00~0x3F)
  HalLcd_HW_Control(0x00);//-not offset
  HalLcd_HW_Control(0xd5);//--set display clock divide ratio/oscillator frequency
  HalLcd_HW_Control(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
  HalLcd_HW_Control(0xD9);//--set pre-charge period
  HalLcd_HW_Control(0xF1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
  HalLcd_HW_Control(0xDA);//--set com pins hardware configuration
  HalLcd_HW_Control(0x12);
  HalLcd_HW_Control(0xDB);//--set vcomh
  HalLcd_HW_Control(0x40);//Set VCOM Deselect Level
  HalLcd_HW_Control(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
  HalLcd_HW_Control(0x02);//
  HalLcd_HW_Control(0x8D);//--set Charge Pump enable/disable
  HalLcd_HW_Control(0x14);//--set(0x10) disable
  HalLcd_HW_Control(0xA4);// Disable Entire Display On (0xa4/0xa5)
  HalLcd_HW_Control(0xA6);// Disable Inverse Display On (0xa6/a7) 
  HalLcd_HW_Control(0xAF);//--turn on oled panel

  /* Clear the display */
  HalLcd_HW_Clear();
}

/**************************************************************************************************
 * @fn      HalLcd_HW_Control
 *
 * @brief   Write 1 command to the LCD
 *
 * @param   uint8 cmd - command to be written to the LCD
 *
 * @return  None
 **************************************************************************************************/
void HalLcd_HW_Control(uint8 cmd)
{
  // LCD_SPI_BEGIN();
  LCD_DO_CONTROL();
  LCD_SPI_TX(cmd);
  LCD_SPI_WAIT_RXRDY();
  // LCD_SPI_END();
}

/**************************************************************************************************
 * @fn      HalLcd_HW_Write
 *
 * @brief   Write 1 byte to the LCD
 *
 * @param   uint8 data - data to be written to the LCD
 *
 * @return  None
 **************************************************************************************************/
void HalLcd_HW_Write(uint8 data)
{
  // LCD_SPI_BEGIN();
  LCD_DO_WRITE();
  LCD_SPI_TX(data);
  LCD_SPI_WAIT_RXRDY();
  // LCD_SPI_END();
}

/**************************************************************************************************
 * @fn          HalLcd_HW_SetContrast
 *
 * @brief       Set display contrast
 *
 * @param       uint8 value - contrast value
 *
 * @return      none
 **************************************************************************************************/
void HalLcd_HW_SetContrast(uint8 value)
{
  SET_ICONRAM_ADDR(CONTRAST_CTRL_REGISTER);
  HalLcd_HW_Write(value);
}

/**************************************************************************************************
 * @fn      HalLcd_HW_Clear
 *
 * @brief   Clear the HW LCD
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalLcd_HW_Clear(void)
{
  uint8 i,n;       
  for(i=0;i<8;i++)  
  {  
    HalLcd_HW_Control(0xb0+i);    //设置页地址（0~7）
    HalLcd_HW_Control(0x00);      //设置显示位置―列低地址
    HalLcd_HW_Control(0x10);      //设置显示位置―列高地址   
    for(n=0;n<128;n++)HalLcd_HW_Write(0); 
  } //更新显示
}

/**************************************************************************************************
 * @fn      HalLcd_HW_ClearAllSpecChars
 *
 * @brief   Clear all special chars
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalLcd_HW_ClearAllSpecChars(void)
{
  uint8 n = 0;

  SET_GCRAM_CHAR(0);
  for (n = 0; n < (8 * 8); n++)
  {
    HalLcd_HW_Write(0x00);
  }
}

/* 指定显示位置 */
void OLED_Set_Pos(uint8 x, uint8 y) 
{ 
  HalLcd_HW_Control(0xb0+y);
  HalLcd_HW_Control(((x&0xf0)>>4)|0x10);
  HalLcd_HW_Control((x&0x0f)|0x01); 
}       

/***************功能描述：行填充, y为页范围0～7****************/  
void OLED_FillLine(uint8 y,char chr)  
{  
    uint8 x;      
    HalLcd_HW_Control(0xb0+y);  
    HalLcd_HW_Control(0x01);  
    HalLcd_HW_Control(0x10);   
    for(x=0;x<127;x++)  
        HalLcd_HW_Write(chr);  
}


/***************功能描述：显示6*8标准ASCII字符    显示的坐标（x,y）x为0～126，y为页范围0～7****************/  
void OLED_P6x8Char(uint8 x,uint8 y,char chr)  
{  
    uint8 i = 0;             
    if(x>126){x=0;y++;}  
    OLED_Set_Pos(x,y);      
    for(i=0;i<6;i++)       
      HalLcd_HW_Write(F6x8[chr - 32][i]);
}



/**************************************************************************************************
 * @fn      HalLcd_HW_WriteChar
 *
 * @brief   Write one char to the display
 *
 * @param   uint8 line - line number that the char will be displayed
 *          uint8 col - colum where the char will be displayed
 *
 * @return  None
 **************************************************************************************************/
void HalLcd_HW_WriteChar(uint8 line, uint8 col, char text)
{
  if (col < HAL_LCD_MAX_CHARS)
  {
    OLED_P6x8Char((col)*8,line -1,text);
  }
  else
  {
    return;
  }
}

/**************************************************************************************************
 * @fn          halLcdWriteLine
 *
 * @brief       Write one line on display
 *
 * @param       uint8 line - display line
 *              char *pText - text buffer to write
 *
 * @return      none
 **************************************************************************************************/
void HalLcd_HW_WriteLine(uint8 line, const char *pText)
{
  uint8 count;
  uint8 totalLength = (uint8)osal_strlen( (char *)pText );

  /* Write the content first */
  for (count=0; count<totalLength; count++)
  {
    HalLcd_HW_WriteChar(line, count, (*(pText++)));
  }

  /* Write blank spaces to rest of the line */
  for(count=totalLength; count<HAL_LCD_MAX_CHARS;count++)
  {
    HalLcd_HW_WriteChar(line, count, ' ');
  }
}

/**************************************************************************************************
 * @fn      HalLcd_HW_WaitUs
 *
 * @brief   wait for x us. @ 32MHz MCU clock it takes 32 "nop"s for 1 us delay.
 *
 * @param   x us. range[0-65536]
 *
 * @return  None
 **************************************************************************************************/
void HalLcd_HW_WaitUs(uint16 microSecs)
{
  while(microSecs--)
  {
    /* 32 NOPs == 1 usecs */
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    asm("nop"); asm("nop");
  }
}
#endif


/**************************************************************************************************
**************************************************************************************************/



