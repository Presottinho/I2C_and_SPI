/*
 * nokia5110.c
 *
 *  Created on: May 21, 2025
 *      Author: Presotto
 */
#include "stm32f4xx_hal.h"
#include "font6x8.h"

extern SPI_HandleTypeDef hspi1;

#define LCD_DC_Pin      GPIO_PIN_2
#define LCD_DC_Port     GPIOC
#define LCD_RST_Pin     GPIO_PIN_0
#define LCD_RST_Port    GPIOC
#define LCD_CS_Pin      GPIO_PIN_1
#define LCD_CS_Port     GPIOC

void LCD_Send(uint8_t data, uint8_t isData) {
    HAL_GPIO_WritePin(LCD_DC_Port, LCD_DC_Pin, isData ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_CS_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(LCD_CS_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

void LCD_Reset() {
    HAL_GPIO_WritePin(LCD_RST_Port, LCD_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LCD_RST_Port, LCD_RST_Pin, GPIO_PIN_SET);
}

void LCD_Init() {
    LCD_Reset();

    LCD_Send(0x21, 0);
    LCD_Send(0xB0, 0);
    LCD_Send(0x04, 0);
    LCD_Send(0x14, 0);
    LCD_Send(0x20, 0);
    LCD_Send(0x0C, 0);
}

void LCD_SetCursor(uint8_t x, uint8_t y) {
    LCD_Send(0x80 | x, 0);
    LCD_Send(0x40 | y, 0);
}

void LCD_Clear() {
    LCD_SetCursor(0, 0);
    for (int i = 0; i < 504; i++) {
        LCD_Send(0x00, 1);
    }
}

void LCD_WriteChar(char c) {
	if (c < 0x20 || c > 0x7E) c = '?';

	    if (c >= 'a' && c <= 'z') {
	        c--;
	    }

	    for (int i = 0; i < 5; i++) {
	        LCD_Send(font6x8[c - 0x20][i], 1);
	    }
	    LCD_Send(0x00, 1);
}

void LCD_WriteString(char *str) {
    while (*str) {
        LCD_WriteChar(*str++);
    }
}
