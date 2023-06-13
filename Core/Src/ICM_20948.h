/*
 * ICM_20948.h
 *
 *  Created on: Jun 7, 2023
 *      Author: Pranav Ponnusamy
 */

#ifndef SRC_ICM_20948_H_
#define SRC_ICM_20948_H_

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

//Bank 0
#define ROOMTEMP_OFFSET 21

#define USER_CTRL 0x03

#define GYRO_SENSITIVITY 0X01
#define ACCEL_SENSITIVITY 1683.67
#define MAG_SENSITIVITY 1
#define TEMP_SENSITIVITY 333

#define GYRO_XOUT_H 0x33
#define GYRO_XOUT_L 0x34

#define GYRO_YOUT_H 0x35
#define GYRO_YOUT_L 0x36

#define GYRO_ZOUT_H 0x37
#define GYRO_ZOUT_L 0x38

#define ACCEL_XOUT_H 0X2D
#define ACCEL_XOUT_L 0X2E

#define ACCEL_YOUT_H 0X2F
#define ACCEL_YOUT_L 0X30

#define ACCEL_ZOUT_H 0X31
#define ACCEL_ZOUT_L 0X32

#define EXT_SLV_SENS_DATA_00 0x3B
#define EXT_SLV_SENS_DATA_01 0x3C
#define EXT_SLV_SENS_DATA_02 0x3D
#define EXT_SLV_SENS_DATA_03 0x3E
#define EXT_SLV_SENS_DATA_04 0x3F
#define EXT_SLV_SENS_DATA_05 0x40

#define TEMP_OUT_H 0x39
#define TEMP_OUT_L 0x3A

#define USER_CTRL 0x03
#define LP_CONFIG 0x05

#define REG_BANK_SEL 0x7F

//Bank 3
#define I2C_MST_ODR_CONFIG 0x00
#define I2C_MST_CTRL 0x01
#define I2C_SLV0_ADDR 0x03
#define I2C_SLV0_REG 0x04
#define I2C_SLV0_CTRL 0x05

//#define
class ICM_20948 {

	struct Readings {
		float x;
		float y;
		float z;
	};

public:
	uint8_t address;
	I2C_HandleTypeDef i2c;
	ICM_20948(I2C_HandleTypeDef handle, uint8_t addr);

	bool isDeviceReady();

	void initIMU();
	void updateIMU();

	void readFIFO(uint8_t *buf);

	float getRoll();
	float getYaw();
	float getPitch();


	int16_t getGyroX();
	int16_t getGyroY();
	int16_t getGyroZ();


	float getAccelX();
	float getAccelY();
	float getAccelZ();

	int16_t* getMag();
	float getMagX();
	int16_t getMagY();
	int16_t getMagZ();

	int16_t getTemp();


private:

	Readings gyroReading;
	Readings accelReading;
	Readings magReading;

	int16_t temperature;

	void initMag();

	HAL_StatusTypeDef updateGyro();
	HAL_StatusTypeDef updateAccel();
	HAL_StatusTypeDef updateMag();
	void updateTemp();


	HAL_StatusTypeDef selUserBank(uint8_t userbank);

	HAL_StatusTypeDef writeICMReg(int regAddress, int data, int dataAmount,
			int userbank);
	HAL_StatusTypeDef readICMReg(int regAddress, uint8_t *buf, int dataAmount);
	uint8_t readICMReg(int regAddress, int dataAmount);

	HAL_StatusTypeDef writeMagReg(uint8_t reg, uint8_t data);
	HAL_StatusTypeDef readMagReg(uint8_t reg);
	uint8_t* readMagReg(uint8_t reg, uint8_t len);

	int16_t twoComplementToDec(uint16_t val);
	uint16_t addBinary(uint8_t b1, uint8_t b2);
};

#endif /* SRC_ICM_20948_H_ */
