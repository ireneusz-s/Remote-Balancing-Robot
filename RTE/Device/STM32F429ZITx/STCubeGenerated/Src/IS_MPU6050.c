#include "IS_MPU6050.h"
#include <stdint.h>  
#include <stdlib.h> 
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "usart.h"
#include "stdbool.h"



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IS_MPU6050_Initialize(IS_MPU6050_struct* DataStruct, MPU6050_Accelerometer_t AccelerometerSensitivity, MPU6050_Gyroscope_t GyroscopeSensitivity)
{
	uint8_t temp;
	IS_MPU6050_check_connection();										//check connection to mpu6050 (compare who i am message)
	IS_MPU6050_write_reg(MPU6050_PWR_MGMT_1,0x00);		//clearerr sleep bit, wake-up device

	//Accelerometer configuration
	temp =  IS_MPU6050_read_reg(MPU6050_ACCEL_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)AccelerometerSensitivity << 3;
	IS_MPU6050_write_reg(MPU6050_ACCEL_CONFIG,temp);
	
	//Gyroscope configuration
	temp =  IS_MPU6050_read_reg(MPU6050_GYRO_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)GyroscopeSensitivity << 3;
	IS_MPU6050_write_reg(MPU6050_GYRO_CONFIG,temp);
	
	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (AccelerometerSensitivity) {
		case MPU6050_Accelerometer_2G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_2; 
			break;
		case MPU6050_Accelerometer_4G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_4; 
			break;
		case MPU6050_Accelerometer_8G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_8; 
			break;
		case MPU6050_Accelerometer_16G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_16; 
		default:
			break;
	}
	
	switch (GyroscopeSensitivity) {
		case MPU6050_Gyroscope_250s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_250; 
			break;
		case MPU6050_Gyroscope_500s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_500; 
			break;
		case MPU6050_Gyroscope_1000s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_1000; 
			break;
		case MPU6050_Gyroscope_2000s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_2000; 
		default:
			break;
	}
	
	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t IS_MPU6050_read_reg(uint8_t reg)
{
 uint8_t value = 0;
 
 HAL_I2C_Mem_Read(&MPU6050_I2C, MPU6050_I2C_ADDR_1, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);
 
 return value;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IS_MPU6050_write_reg(uint8_t reg, uint8_t value)
{
 HAL_I2C_Mem_Write(&MPU6050_I2C, MPU6050_I2C_ADDR_1, reg, 1, &value, sizeof(value), HAL_MAX_DELAY);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool IS_MPU6050_check_connection(void)
{
 //printf("Wyszukiwanie akcelerometru...\n");
 USART_WriteString("Wyszukiwanie akcelerometru...\n");
 uint8_t who_am_i_value = IS_MPU6050_read_reg(MPU6050_WHO_AM_I);
 
	if (who_am_i_value == MPU6050_I_AM) 
	{
	    USART_WriteString("Znaleziono akcelerometr MPU6050!\n");
			return true;
	} else {
			USART_WriteString("Niepoprawna odpowiedz ukladu\n");
			return false;
  }
			
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IS_MPU6050_ReadTemperature(IS_MPU6050_struct* DataStruct) {
	uint8_t data[2];
	int16_t temp;

	//read data from thermometer
	HAL_I2C_Mem_Read(&MPU6050_I2C, MPU6050_I2C_ADDR_1, MPU6050_TEMP_OUT_H, 1, data, 2, HAL_MAX_DELAY);

	//Write data to structure
	temp = (data[0] << 8 | data[1]);

	DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IS_MPU6050_ReadALL(IS_MPU6050_struct* const DataStruct)
{
	uint8_t data[14];
	int16_t temp;
	
	HAL_I2C_Mem_Read(&MPU6050_I2C, MPU6050_I2C_ADDR_1, MPU6050_ACCEL_XOUT_H, 1, data, 14, HAL_MAX_DELAY);
	
	//Accelerometer format data
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);	
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

	//Temperature format
	temp = (data[6] << 8 | data[7]);
	DataStruct->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);
	
	//Gyroscope format data
	DataStruct->Gyroscope_X = (int16_t)(data[8] << 8 | data[9]);
	DataStruct->Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
	DataStruct->Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IS_MPU6050_ReadGyroscope(IS_MPU6050_struct* const DataStruct) {
	uint8_t data[6];
	
	//Read data from gyroscope
	HAL_I2C_Mem_Read(&MPU6050_I2C, MPU6050_I2C_ADDR_1, MPU6050_GYRO_XOUT_H, 1, data, 6, HAL_MAX_DELAY);

	//Write data to structure
	DataStruct->Gyroscope_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Gyroscope_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Gyroscope_Z = (int16_t)(data[4] << 8 | data[5]);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IS_MPU6050_ReadAccelerometer(IS_MPU6050_struct* DataStruct) {
	uint8_t data[6];
	
	//Read data from gyroscope
	HAL_I2C_Mem_Read(&MPU6050_I2C, MPU6050_I2C_ADDR_1, MPU6050_ACCEL_XOUT_H, 1, data, 6, HAL_MAX_DELAY);
	
	//Write data to structure
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);
}
