/*
 * lcd.c
 *
 *  Created on: 10/06/2018
 *      Author: Olivier Van den Eede
 */

#include "lcd.h"
//const uint8_t ROW_16[] = {0x00, 0x40, 0x10, 0x50};
/************************************** Static declarations **************************************/

const uint8_t lcd_row_addresses[] = {0x00,0x40,0x14,0x54};
static void lcd_load(DisplayLCD* lcd, uint8_t data, uint8_t time, uint8_t mode);


///************************************** Function definitions **************************************/

DisplayLCD lcd_generate(GPIO_Port port[],GPIO_Pin pin[],
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

void lcd_init(DisplayLCD* lcd) {
    HAL_Delay(15);
	lcd_load(lcd,0x33,5,0);
	lcd_load(lcd,0x32,5,0);

	lcd_load(lcd,0x28,1,0);

	lcd_load(lcd,0x08,1,0);
	lcd_load(lcd,0x01,2,0);

	//lcd_load(lcd,0x0F,1,0);
}

void lcd_clear_display(DisplayLCD* lcd) {
	lcd_load(lcd,0x01,2,0);
}

void lcd_blink_cursor(DisplayLCD* lcd) {
	lcd_load(lcd,0x0F,1,0);
}

void lcd_shift_cursor(DisplayLCD* lcd,uint8_t direction) {
	lcd_load(lcd,0x10|(direction << 2),1,0);
}


void lcd_shift_display(DisplayLCD* lcd,uint8_t direction) {
	lcd_load(lcd,0x18|(direction << 2),1,0);
};

//char* lcd_convert_number(uint8_t number) {
//	uint8_t it = 0;
//    while(it<20&&number>0) {
//    	chars[it]=number%10;
//    	number/=10;
//    	++it;
//    }
//    return chars;
//};

void lcd_pos_cursor(DisplayLCD* lcd,uint8_t coordinates[2]) {
    lcd_load(lcd,0x80|(lcd_row_addresses[coordinates[1]]+
    coordinates[0]),1,0);
}

void lcd_write_data(DisplayLCD* lcd,char* string,uint8_t
start_pos[2]) {
	lcd_pos_cursor(lcd,start_pos);
	uint8_t pos[2];
	pos[0] = 0;
	pos[1] = start_pos[1];
	uint8_t it = start_pos[0];
	while(pos[1]<4) {
	    while(it<(20*(pos[1]-start_pos[1]+1))&&it<(
	    strlen(string)+start_pos[0])) {
		    lcd_load(lcd,string[it-start_pos[0]],1,1);
		    ++it;
	    }
	    ++pos[1];
	    if(pos[1]<4) {
		    lcd_pos_cursor(lcd,pos);
	    } else {
	    	uint8_t final_pos[2] = {it%20,(it/20+
	    	start_pos[1])%4};
	    	lcd_pos_cursor(lcd,final_pos);
	    }
	}
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
void lcd_load(DisplayLCD* lcd,uint8_t data,uint8_t time,
		      uint8_t mode)
{
	HAL_GPIO_WritePin(lcd->rs_port,lcd->rs_pin,mode);
	for(uint8_t it=0;it<4;++it) {
		HAL_GPIO_WritePin(lcd->data_port[it],
		lcd->data_pin[it],(data>>(it+4))&0x01);
	}
	HAL_GPIO_WritePin(lcd->en_port,lcd->en_pin,1);
	HAL_Delay(time);
	HAL_GPIO_WritePin(lcd->en_port,lcd->en_pin,0);
	HAL_Delay(time);
	for(uint8_t it=0;it<4;++it) {
		HAL_GPIO_WritePin(lcd->data_port[it],
		lcd->data_pin[it],(data>>it)&0x01);
	}
    HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin,1);
	HAL_Delay(time);
	HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin,0);
	HAL_Delay(time);
}
