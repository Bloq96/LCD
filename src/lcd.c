/*
 * lcd.c
 *
 *  Created on: 10/06/2018
 *      Author: Olivier Van den Eede
 */

#include "lcd.h"
//const uint8_t ROW_16[] = {0x00, 0x40, 0x10, 0x50};
//const uint8_t ROW_20[] = {0x00, 0x40, 0x14, 0x54};
///************************************** Static declarations **************************************/
//
//static void lcd_write_data(Lcd_HandleTypeDef * lcd, uint8_t data);
//static void lcd_write_command(Lcd_HandleTypeDef * lcd, uint8_t command);
//static void lcd_write(Lcd_HandleTypeDef * lcd, uint8_t data, uint8_t len);
//
//
///************************************** Function definitions **************************************/

DisplayLCD generate_lcd(GPIO_Port port[],GPIO_Pin pin[],
GPIO_Port rs_port,GPIO_Pin rs_pin,GPIO_Port en_port,
GPIO_Pin en_pin) {
	DisplayLCD lcd;

	lcd.en_pin = en_pin;
	lcd.en_port = en_port;

	lcd.rs_pin = rs_pin;
	lcd.rs_port = rs_port;

	lcd.data_pin = pin;
	lcd.data_port = port;

	return lcd;
}

void lcd_init(DisplayLCD* lcd)
{

	lcd_write_command(lcd, 0x33);
	lcd_write_command(lcd, 0x32);
	lcd_write_command(lcd, FUNCTION_SET | OPT_N);

	lcd_write_command(lcd, CLEAR_DISPLAY);						// Clear screen
	lcd_write_command(lcd, DISPLAY_ON_OFF_CONTROL | OPT_D);		// Lcd-on, cursor-off, no-blink
	lcd_write_command(lcd, ENTRY_MODE_SET | OPT_INC);			// Increment cursor
}
//
///**
// * Write a number on the current position
// */
//void Lcd_int(Lcd_HandleTypeDef * lcd, int number)
//{
//	char buffer[11];
//	sprintf(buffer, "%d", number);
//
//	Lcd_string(lcd, buffer);
//}
//
///**
// * Write a string on the current position
// */
//void Lcd_string(Lcd_HandleTypeDef * lcd, char * string)
//{
//	for(uint8_t i = 0; i < strlen(string); i++)
//	{
//		lcd_write_data(lcd, string[i]);
//	}
//}
//
///**
// * Set the cursor position
// */
//void Lcd_cursor(Lcd_HandleTypeDef * lcd, uint8_t row, uint8_t col)
//{
//	#ifdef LCD20xN
//	lcd_write_command(lcd, SET_DDRAM_ADDR + ROW_20[row] + col);
//	#endif
//
//	#ifdef LCD16xN
//	lcd_write_command(lcd, SET_DDRAM_ADDR + ROW_16[row] + col);
//	#endif
//}
//
///**
// * Clear the screen
// */
//void Lcd_clear(Lcd_HandleTypeDef * lcd) {
//	lcd_write_command(lcd, CLEAR_DISPLAY);
//}
//
//void Lcd_define_char(Lcd_HandleTypeDef * lcd, uint8_t code, uint8_t bitmap[]){
//	lcd_write_command(lcd, SETCGRAM_ADDR + (code << 3));
//	for(uint8_t i=0;i<8;++i){
//		lcd_write_data(lcd, bitmap[i]);
//	}
//
//}
//
//
///************************************** Static function definition **************************************/
//
///**
// * Write a byte to the command register
// */
//void lcd_write_command(Lcd_HandleTypeDef * lcd, uint8_t command)
//{
//	HAL_GPIO_WritePin(lcd->rs_port, lcd->rs_pin, LCD_COMMAND_REG);		// Write to command register
//
//	if(lcd->mode == LCD_4_BIT_MODE)
//	{
//		lcd_write(lcd, (command >> 4), LCD_NIB);
//		lcd_write(lcd, command & 0x0F, LCD_NIB);
//	}
//	else
//	{
//		lcd_write(lcd, command, LCD_BYTE);
//	}
//
//}
//
///**
// * Write a byte to the data register
// */
//void lcd_write_data(Lcd_HandleTypeDef * lcd, uint8_t data)
//{
//	HAL_GPIO_WritePin(lcd->rs_port, lcd->rs_pin, LCD_DATA_REG);			// Write to data register
//
//	if(lcd->mode == LCD_4_BIT_MODE)
//	{
//		lcd_write(lcd, data >> 4, LCD_NIB);
//		lcd_write(lcd, data & 0x0F, LCD_NIB);
//	}
//	else
//	{
//		lcd_write(lcd, data, LCD_BYTE);
//	}
//
//}
//
///**
// * Set len bits on the bus and toggle the enable line
// */
void lcd_load(DisplayLCD* lcd, uint8_t data, uint8_t time)
{
	for(uint8_t it=0;it<4;++it) {
		HAL_GPIO_WritePin(lcd->data_port[it],
		lcd->data_pin[it],(data>>(it+4))&0x01);
	}
	HAL_GPIO_WritePin(lcd->en_port,lcd->en_pin,1);
	HAL_Delay(time);
	HAL_GPIO_WritePin(lcd->en_port,lcd->en_pin,0);
	HAL_Delaay(time);
	for(uint8_t it=0;it<4;++it) {
		HAL_GPIO_WritePin(lcd->data_port[it],
		lcd->data_pin[it],(data>>it)&0x01);
	}
    HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin,1);
	HAL_Delay(time);
	HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin,0);
	HAL_Delay(time);
}
