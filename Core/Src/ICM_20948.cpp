/*
 * ICM_20948.cpp
 *
 *  Created on: Jun 7, 2023
 *      Author: Pranav Ponnusamy
 */

#include "ICM_20948.h"

ICM_20948::ICM_20948(I2C_HandleTypeDef handle, uint8_t addr) {
	address = addr;
	i2c = handle;

	initIMU();
}

void ICM_20948::initIMU() {

	initMag();
	readMagReg(0x11);
/*
 * 1. reset device
 * 2. remove device from sleep
 * 3. set clock source
 * 4. set up low pass filters for gyro and accel
 * 5. set up sample rate divider
 * 6. Call calibration function for gyro and accel
 * 7. Chose the full scale for both gyro and accel
 */

//	writeReg(USER_CTRL, 0b00000010, 1, 0);
//	HAL_Delay(100);
//	writeReg(USER_CTRL, 0b00100000, 1, 0);
//
//	writeReg(I2C_MST_CTRL, 0b00000111, 1, 3);
//
//	writeReg(LP_CONFIG, 0x40, 1, 0);
//
//	writeReg(I2C_MST_ODR_CONFIG, 0b00000011, 1, 3);
}

/*
 * Add function for the mag init
 * 1. reset 12c master
 * 2. enable the master
 * 3. choose the clock frequency to 7
 * 4. reset the mag
 * 5. choose the continuous operation mode for the mag
 */

//HAL_StatusTypeDef ICM_20948::writeMagReg(uint_t reg, uint8_t data){
//	writeReg(I2C_SLV0_ADDR, , dataAmount);
//}

//void ICM_20948::readFIFO(uint8_t *buf) {
//	for (int x = 0; x < 128; x++) {
//		writeReg(0x72, x, 1);
//		readReg(0x72, buf[x]);
//	}
//}

void ICM_20948::initMag(){
	uint8_t temp_data = readICMReg(USER_CTRL, 1, 0);
	temp_data |= 0x02;
	writeICMReg(USER_CTRL, temp_data, 1, 0);
	HAL_Delay(100);

	temp_data = readICMReg(USER_CTRL, 1, 0);
	temp_data |= 0x20;
	writeICMReg(USER_CTRL, temp_data, 1, 0);
	HAL_Delay(10);

	temp_data = 0x07;
	writeICMReg(I2C_MST_CTRL, temp_data, 1, 3);
	HAL_Delay(10);

	temp_data = 0x40;
	writeICMReg(LP_CONFIG, temp_data, 1, 0);
	HAL_Delay(10);


	writeMagReg(CNTL3, 0x01);
	HAL_Delay(100);

	writeMagReg(CNTL2, 0x08);
}


void ICM_20948::updateIMU(){
	updateGyro();
	updateAccel();
	updateMag();
}

HAL_StatusTypeDef ICM_20948::updateGyro() {
	uint8_t temp[6];
	HAL_StatusTypeDef ret;

	ret = readICMReg(GYRO_XOUT_H, temp, 6, 0);

	if (ret == HAL_OK) {
		gyroReading.x = twoComplementToDec(
				addBinary(temp[1], temp[0])) / GYRO_SENSITIVITY;
		gyroReading.y = twoComplementToDec(
				addBinary(temp[3], temp[2])) / GYRO_SENSITIVITY;
		gyroReading.z = twoComplementToDec(
				addBinary(temp[5], temp[4])) / GYRO_SENSITIVITY;
	} else {
		gyroReading.x = -999;
		gyroReading.y = -999;
		gyroReading.z = -999;
	}

	return ret;
}

HAL_StatusTypeDef ICM_20948::updateAccel() {
	uint8_t temp[6];
	HAL_StatusTypeDef ret;

	ret = readICMReg(ACCEL_XOUT_H, temp, 6, 0);

	if (ret == HAL_OK) {
		accelReading.x = twoComplementToDec(
				addBinary(temp[1], temp[0])) / ACCEL_SENSITIVITY;
		accelReading.y = twoComplementToDec(
				addBinary(temp[3], temp[2])) / ACCEL_SENSITIVITY;
		accelReading.z = twoComplementToDec(
				addBinary(temp[5], temp[4])) / ACCEL_SENSITIVITY;
	} else {
		accelReading.x = -999;
		accelReading.y = -999;
		accelReading.z = -999;
	}

	return ret;
}

HAL_StatusTypeDef ICM_20948::updateMag(){
	return HAL_OK;
}

void ICM_20948::updateTemp() {
	uint8_t buf[2];
	HAL_StatusTypeDef ret;

	ret = readICMReg(TEMP_OUT_H, buf, 2, 0);
	int temp = twoComplementToDec(addBinary(buf[1], buf[0]));

	if (ret == HAL_OK) {
		temperature = ((temp - 21) / 400) + 21;
	} else {
		temperature = -999;
	}
}

float ICM_20948::getPitch() {
	return atan2(getAccelX(), getAccelZ()) * 360 / (2 * M_PI);
}

float ICM_20948::getRoll() {
	return atan2(getAccelY(), getAccelZ()) * 360 / (2 * M_PI);
}

//Angular velocity in each axis
float ICM_20948::getGyroX()
{
	return gyroReading.x;
}

float ICM_20948::getGyroY() {
	return gyroReading.y;
}

float ICM_20948::getGyroZ() {
	return gyroReading.z;
}

//Acceleration in each axis
float ICM_20948::getAccelX() {
	return accelReading.x;
}

float ICM_20948::getAccelY() {
	return accelReading.y;
}

float ICM_20948::getAccelZ() {
	return accelReading.z;
}

//Magnometer reading in each axis
float ICM_20948::getMagX() {
	uint8_t buf[2];
	HAL_StatusTypeDef ret;

	ret = readICMReg(EXT_SLV_SENS_DATA_00, buf, 2, 0);
	int xMag = twoComplementToDec(addBinary(buf[0], buf[1]));

	if (ret == HAL_OK) {
		return float(xMag) / MAG_SENSITIVITY;
	} else {
		return -99;
	}
}

//int16_t ICM_20948::getMagY() {
//
//}
//int16_t ICM_20948::getMagZ() {
//
//}

int16_t ICM_20948::getTemp(){
	return temperature;
}



bool ICM_20948::isDeviceReady() {
	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&i2c, address, 1, 100);
	if (ret == HAL_OK) {
		return 1;
	} else {
		return 0;
	}
}

HAL_StatusTypeDef ICM_20948::writeICMReg(int regAddress, int data,
		int dataAmount, int userbank) {

	selUserBank(userbank);

	HAL_StatusTypeDef ret;
	uint8_t temp = data;
	ret = HAL_I2C_Mem_Write(&i2c, address, regAddress, 1, &temp, dataAmount,
	HAL_MAX_DELAY);
	return ret;
}

HAL_StatusTypeDef ICM_20948::readICMReg(int regAddress, uint8_t *buf,
		int dataAmount, int userbank) {
	selUserBank(userbank);
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Read(&i2c, address, regAddress, 1, buf, dataAmount,
	HAL_MAX_DELAY);
	return ret;
}

uint8_t ICM_20948::readICMReg(int regAddress, int dataAmount, int userbank) {
	selUserBank(userbank);
	uint8_t buf[1];
	HAL_I2C_Mem_Read(&i2c, address, regAddress, 1, buf, dataAmount,
	HAL_MAX_DELAY);
	return buf[0];
}

HAL_StatusTypeDef ICM_20948::selUserBank(uint8_t userbank){
	HAL_StatusTypeDef ret;
	uint8_t temp = userbank << 4;
	ret = HAL_I2C_Mem_Write(&i2c, address, REG_BANK_SEL, 1, &temp, 1,
			HAL_MAX_DELAY);
	return ret;
}

HAL_StatusTypeDef ICM_20948::readMagReg(uint8_t reg){
	writeICMReg(I2C_SLV0_ADDR, 0x80|MagAddress, 1, 3);
	writeICMReg(I2C_SLV0_REG, reg, 1, 3);
	writeICMReg(I2C_SLV0_CTRL, 0x80|8, 1, 3);

	return HAL_OK;
}

HAL_StatusTypeDef ICM_20948::writeMagReg(uint8_t reg, uint8_t data){
	writeICMReg(I2C_SLV0_ADDR, 0x80 | MagAddress, 1, 3);
	writeICMReg(I2C_SLV0_REG, reg, 1, 3);
	writeICMReg(I2C_SLV0_DO, data, 1, 3);
	writeICMReg(I2C_SLV0_CTRL, 0x80 | 0x01, 1, 3);

	return HAL_OK;
}



uint16_t ICM_20948::addBinary(uint8_t lowByte, uint8_t highByte) {
	return ((highByte << 8) | lowByte);
}

int16_t ICM_20948::twoComplementToDec(uint16_t val) {
	if ((val & 0x8000) == 0) {
		return val;
	} else {
		val = ~(val) + 1;
		return -val;
	}
}

