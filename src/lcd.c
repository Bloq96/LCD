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
const uint8_t lcd_row_positions[] = {0,2,1,3};
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
	lcd->cursor_pos[0] = 0;
	lcd->cursor_pos[1] = 0;
	for(uint8_t it=0;it<4;++it) {
		for(uint8_t it2=0;it2<20;++it2) {
			lcd->display[it][it2] = ' ';
		}
	}
    HAL_Delay(15);
	lcd_load(lcd,0x33,5,0);
	lcd_load(lcd,0x32,5,0);

	lcd_load(lcd,0x28,1,0);

	lcd_load(lcd,0x08,1,0);
	lcd_load(lcd,0x01,2,0);
}

void lcd_blink_cursor(DisplayLCD* lcd) {
	lcd_load(lcd,0x0F,1,0);
}

void lcd_clear_display(DisplayLCD* lcd) {
	lcd_load(lcd,0x01,2,0);
	lcd->cursor_pos[0] = 0;
	lcd->cursor_pos[1] = 0;
	for(uint8_t it=0;it<4;++it) {
		for(uint8_t it2=0;it2<20;++it2) {
			lcd->display[it][it2] = ' ';
		}
	}
}

void lcd_return_home(DisplayLCD* lcd) {
	lcd_load(lcd,0x02,2,0);
	lcd->cursor_pos[0] = 0;
	lcd->cursor_pos[1] = 0;
}

void lcd_shift_cursor(DisplayLCD* lcd,uint8_t direction) {
	lcd_load(lcd,0x10|(direction << 2),1,0);
	if(direction==0) {
		if(lcd->cursor_pos[0]==0) {
			lcd->cursor_pos[0] = 19;
			switch(lcd->cursor_pos[1]) {
				case 0:
					lcd->cursor_pos[1] = 3;
					break;
				case 1:
					lcd->cursor_pos[1] = 2;
					break;
				case 2:
					lcd->cursor_pos[1] = 0;
					break;
				case 3:
					lcd->cursor_pos[1] = 1;
					break;
				default:
					lcd->cursor_pos[1] = 0;
			}
		} else {
		    --lcd->cursor_pos[0];
		}
	} else {
		++lcd->cursor_pos[0];
	    if(lcd->cursor_pos[0]>=20) {
		    lcd->cursor_pos[0] = 0;
		    switch(lcd->cursor_pos[1]) {
			    case 0:
			        lcd->cursor_pos[1] = 2;
			        break;
			    case 1:
				    lcd->cursor_pos[1] = 3;
				    break;
			    case 2:
				    lcd->cursor_pos[1] = 1;
			        break;
			    case 3:
				    lcd->cursor_pos[1] = 0;
				    break;
			    default:
				    lcd->cursor_pos[1] = 0;
		    }
	    }
	}
}

void lcd_fast_shift(DisplayLCD* lcd,uint8_t direction,uint8_t
		            times, uint16_t delay) {
	for(uint8_t it=0;it<times;++it) {
	    lcd_load(lcd,0x18|(direction << 2),1,0);
	    HAL_Delay(delay);
	}
	lcd_return_home(lcd);
}

void lcd_shift_display(DisplayLCD* lcd,uint8_t direction) {
    char pivot;
	switch(direction) {
	case 0:
		if(lcd->cursor_pos[0]==0) {
		    lcd->cursor_pos[0] = 19;
		} else {
			--lcd->cursor_pos[0];
		}
	    for(uint8_t it=0;it<4;++it) {
	    	pivot = lcd->display[it][0];
		    for(uint8_t it2=0;it2<19;++it2) {
		        lcd->display[it][it2] =
		        lcd->display[it][it2+1];
		    }
		    lcd->display[it][19] = pivot;
	    }
	    break;
	default:
		if(lcd->cursor_pos[0]==19) {
		    lcd->cursor_pos[0] = 0;
		} else {
			++lcd->cursor_pos[0];
		}
	    for(uint8_t it=0;it<4;++it) {
	    	pivot = lcd->display[it][19];
		    for(uint8_t it2=0;it2<19;++it2) {
		        lcd->display[it][19-it2] =
		        lcd->display[it][18-it2];
		    }
		    lcd->display[it][0] = pivot;
	    }
	}
	lcd_pos_cursor(lcd,lcd->cursor_pos);
	lcd_display_matrix(lcd);
};

void lcd_pos_cursor(DisplayLCD* lcd,Cartesian coordinates) {
    lcd_load(lcd,0x80|(lcd_row_addresses[coordinates[1]]+
    coordinates[0]),1,0);
    lcd->cursor_pos[0] = coordinates[0];
    lcd->cursor_pos[1] = coordinates[1];
}

void lcd_display_matrix(DisplayLCD* lcd) {
	Cartesian old_pos = {lcd->cursor_pos[0],
	lcd->cursor_pos[1]};
	lcd_load(lcd,0x02,2,0);
	for(uint8_t it=0;it<4;++it) {
		for(uint8_t it2=0;it2<20;++it2) {
			lcd_load(lcd,lcd->display[lcd_row_positions[it]]
			[it2],1,1);
	    }
	}
	lcd_pos_cursor(lcd,old_pos);
}

void lcd_write_data(DisplayLCD* lcd,char* string) {
	Cartesian start_pos = {lcd->cursor_pos[0],lcd->cursor_pos[1]};
	Cartesian pos = {0,start_pos[1]};
	uint8_t it = start_pos[0];
	while(pos[1]<4) {
	    while(it<(20*(pos[1]-start_pos[1]+1))&&it<(
	    strlen(string)+start_pos[0])) {
		    lcd_load(lcd,string[it-start_pos[0]],1,1);
		    lcd->display[pos[1]][it-20*(pos[1]-
		    start_pos[1])] = string[it-start_pos[0]];
		    ++it;
	    }
	    ++pos[1];
	    if(pos[1]<4) {
		    lcd_pos_cursor(lcd,pos);
	    } else {
	    	Cartesian final_pos = {it%20,(it/20+
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
