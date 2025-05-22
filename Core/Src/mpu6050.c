/*
 * mpu6050.c
 *
 *  Created on: May 21, 2025
 *      Author: Presotto
 */
#include <stdio.h>

#include <mpu6050.h>
#include <nokia5110.h>
#include <main.h>

extern I2C_HandleTypeDef hi2c1;

void mpu6050_init(){

	HAL_StatusTypeDef situation = HAL_I2C_IsDeviceReady(&hi2c1, (DEVICE_ADRESS << 1) + 0, 1, 100);

	if(situation == HAL_OK){
		LCD_WriteString("O dispositivo esta pronto pra uso!");
		HAL_Delay(1000);
		LCD_Clear();
	}else{
		LCD_WriteString("O dispositivo nao esta pronto para uso. Verifique os cabos e tente novamente...");
		HAL_Delay(1000);
		LCD_Clear();
	}

	uint8_t tempData = FS_GYRO_500;
	situation = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADRESS << 1) + 0, REG_CONFIG_GYRO, 1, &tempData, 1, 100);

	if(situation == HAL_OK){
		LCD_WriteString("Configurando oGiroscopio...");
	  	HAL_Delay(1000);
	  	LCD_Clear();
	}else{
		LCD_WriteString("Falha ao configurar Giroscopio. Tente novamente...");
	  	HAL_Delay(1000);
	  	LCD_Clear();
	}

	tempData = FS_ACC_4G;
	situation = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADRESS << 1) + 0, REG_CONFIG_ACC, 1, &tempData, 1, 100);

	if(situation == HAL_OK){
		LCD_WriteString("Configurando oAcelerometro...");
	  	HAL_Delay(1000);
	  	LCD_Clear();
	}else{
		LCD_WriteString("Falha ao configurar Acelerometro. Tente novamente...");
	  	HAL_Delay(1000);
	  	LCD_Clear();
	}

	tempData = 0;
	situation = HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADRESS << 1) + 0, REG_USR_CTRL, 1, &tempData, 1, 100);

	if(situation == HAL_OK){
		LCD_WriteString("Saindo do ModoSoneca");
	  	HAL_Delay(1000);
	  	LCD_Clear();
	}else{
		LCD_WriteString("Falha ao sair do Modo Soneca. Tente novamente...");
	  	HAL_Delay(1000);
	  	LCD_Clear();
	}

}

void mpu6050_read(){

	uint8_t dataX[2];
	int16_t xAcc;
	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADRESS << 1) + 1, 59, 1, dataX, 2, 100);

	xAcc = ((int16_t)dataX[0] << 8) + dataX[1] - 650;

	uint8_t dataY[2];
	int16_t yAcc;
	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADRESS << 1) + 1, 61, 1, dataY, 2, 100);

	yAcc = ((int16_t)dataY[0] << 8) + dataY[1] - 120;

	uint8_t dataZ[2];
	int16_t zAcc;
	HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADRESS << 1) + 1, 63, 1, dataZ, 2, 100);

	zAcc = ((int16_t)dataZ[0] << 8) + dataZ[1] - 7120;

	char buffer[20];
	sprintf(buffer, "Eixo X: %d;", xAcc);
	LCD_WriteString(buffer);
	sprintf(buffer, "Eixo Y: %d;", yAcc);
	LCD_SetCursor(0, 2);
	LCD_WriteString(buffer);
	sprintf(buffer, "Eixo Z: %d;", zAcc);
	LCD_SetCursor(0, 4);
	LCD_WriteString(buffer);

}
