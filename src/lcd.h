/*
 * lcd.h
 *
 *  Created on: 10/06/2018
 *      Author: Olivier Van den Eede
 */

#ifndef LCD_H
#define LCD_H

#include "stm32f4xx_hal.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "main.h"

//#define LCD20xN 		// For 20xN LCDs
////#define LCD16xN			// For 16xN LCDs
//
//// For row start addresses
//extern const uint8_t ROW_16[];
//extern const uint8_t ROW_20[];
//
///************************************** Command register **************************************/
//#define CLEAR_DISPLAY 0x01
//
//#define RETURN_HOME 0x02
//
//#define ENTRY_MODE_SET 0x04
//#define OPT_S	0x01					// Shift entire display to right
//#define OPT_INC 0x02					// Cursor increment
//
//#define DISPLAY_ON_OFF_CONTROL 0x08
//#define OPT_D	0x04					// Turn on display
//#define OPT_C	0x02					// Turn on cursor
//#define OPT_B 	0x01					// Turn on cursor blink
//
//#define CURSOR_DISPLAY_SHIFT 0x10		// Move and shift cursor
//#define OPT_SC 0x08
//#define OPT_RL 0x04
//
//#define FUNCTION_SET 0x20
//#define OPT_DL 0x10						// Set interface data length
//#define OPT_N 0x08						// Set number of display lines
//#define OPT_F 0x04						// Set alternate font
//#define SETCGRAM_ADDR 0x040
//#define SET_DDRAM_ADDR 0x80				// Set DDRAM address
//
//
///************************************** Helper macros **************************************/
//#define DELAY(X) HAL_Delay(X)
//
//
///************************************** LCD defines **************************************/
//#define LCD_NIB 4
//#define LCD_BYTE 8
//#define LCD_DATA_REG 1
//#define LCD_COMMAND_REG 0
//
//
///************************************** LCD typedefs **************************************/
typedef char Matrix[4][20];
typedef uint8_t Cartesian[2];
typedef GPIO_TypeDef* GPIO_Port;
typedef uint16_t GPIO_Pin;

////typedef enum {
////	LCD_4_BIT_MODE,
////	LCD_8_BIT_MODE
////} Lcd_ModeTypeDef;
//
//
typedef struct {
	GPIO_Port* data_port;
	GPIO_Pin* data_pin;

	GPIO_Port rs_port;
	GPIO_Pin rs_pin;

	GPIO_Port en_port;
	GPIO_Pin en_pin;

    Cartesian cursor_pos;
    Matrix display;
} DisplayLCD;


/************************************** Public functions **************************************/
void lcd_pos_cursor(DisplayLCD* lcd,uint8_t coordinates[2]);
void lcd_blink_cursor(DisplayLCD* lcd);
void lcd_clear_display(DisplayLCD* lcd);
void lcd_write_data(DisplayLCD* lcd,char* string);
void lcd_shift_cursor(DisplayLCD* lcd,uint8_t direction);
void lcd_shift_display(DisplayLCD* lcd,uint8_t direction);
void lcd_fast_shift(DisplayLCD* lcd,uint8_t direction,uint8_t
		            times, uint16_t delay);
void lcd_display_matrix(DisplayLCD* lcd);
void lcd_return_home(DisplayLCD* lcd);
//char* lcd_convert_number(uint8_t number);
void lcd_init(DisplayLCD* lcd);
//void Lcd_int(Lcd_HandleTypeDef * lcd, int number);
//void Lcd_string(Lcd_HandleTypeDef * lcd, char * string);
//void Lcd_cursor(Lcd_HandleTypeDef * lcd, uint8_t row, uint8_t col);
DisplayLCD lcd_generate(GPIO_Port data_port[], GPIO_Pin
data_pin[], GPIO_Port rs_port, GPIO_Pin rs_pin, GPIO_Port
en_port, GPIO_Pin en_pin);
//void Lcd_define_char(Lcd_HandleTypeDef * lcd, uint8_t code, uint8_t bitmap[]);
//void Lcd_clear(Lcd_HandleTypeDef * lcd);

#endif /* LCD_H */
