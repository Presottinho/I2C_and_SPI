/*
 * nokia5110.h
 *
 *  Created on: May 21, 2025
 *      Author: Presotto
 */
#ifndef NOKIA5110_H
#define NOKIA5110_H

#include <stdint.h>

void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t x, uint8_t y);
void LCD_WriteChar(char c);
void LCD_WriteString(char *str);

#endif
