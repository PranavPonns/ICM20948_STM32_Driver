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
}

void ICM_20948::initIMU() {
/*
 * 1. reset device
 * 2. remove device from sleep
 * 3. set clock source
 * 4. set up low pass filters for gyro and accel
 * 5. set up sample rate divider
 * 6. Call calibration function for gyro and accel
 * 7. Chose the full scale for both gyro and accel
 */

	writeReg(USER_CTRL, 0b00000010, 1, 0);
	HAL_Delay(100);
	writeReg(USER_CTRL, 0b00100000, 1, 0);

	writeReg(I2C_MST_CTRL, 0b00000111, 1, 3);

	writeReg(LP_CONFIG, 0x40, 1, 0);

	writeReg(I2C_MST_ODR_CONFIG, 0b00000011, 1, 3);
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

//int16_t* ICM_20948::getGyro(){
//
//	int16_t *buf = new int16_t[3];
//	uint8_t temp[6];
//
//	readReg(GYRO_XOUT_H, temp, 6);
//
//	buf[0] = twoComplementToDec(addBinary(buf[1], buf[0]));
//	buf[1] = twoComplementToDec(addBinary(buf[3], buf[2]));
//	buf[2] = twoComplementToDec(addBinary(buf[5], buf[4]));
//
//	return buf;
//}

void ICM_20948::updateGyro() {
	uint8_t temp[6];

	readReg(GYRO_XOUT_H, temp, 6);

	gyroReading.x = twoComplementToDec(addBinary(temp[1], temp[0]));
	gyroReading.y = twoComplementToDec(addBinary(temp[3], temp[2]));
	gyroReading.z = twoComplementToDec(addBinary(temp[5], temp[4]));
}

void ICM_20948::updateAccel() {
	uint8_t temp[6];

	readReg(ACCEL_XOUT_H, temp, 6);

	accelReading.x = twoComplementToDec(addBinary(temp[1], temp[0]));
	accelReading.y = twoComplementToDec(addBinary(temp[3], temp[2]));
	accelReading.z = twoComplementToDec(addBinary(temp[5], temp[4]));
}

int16_t* ICM_20948::getAccel(){

	int16_t *buf = new int16_t[3];
	uint8_t temp[6];

	readReg(ACCEL_XOUT_H, temp, 6);

	buf[0] = twoComplementToDec(addBinary(buf[1], buf[0]));
	buf[1] = twoComplementToDec(addBinary(buf[3], buf[2]));
	buf[2] = twoComplementToDec(addBinary(buf[5], buf[4]));

	return buf;
}

//float ICM_20948::getPitch() {
//	return atan2(getAccelX(), getAccelZ()) * 360 / (2 * M_PI);
//}
float ICM_20948::getPitch() {
	updateAccel();
	return atan2(accelReading.x, accelReading.z) * 360 / (2 * M_PI);
}

float ICM_20948::getRoll() {
	return atan2(getAccelY(), getAccelZ()) * 360 / (2 * M_PI);
}

int16_t ICM_20948::getGyroX()
{
	uint8_t buf[2];
	HAL_StatusTypeDef ret;

	ret = readReg(GYRO_XOUT_H, buf, 2);
	int xGyro = twoComplementToDec(addBinary(buf[1], buf[0]));

	if (ret == HAL_OK) {
		return xGyro / GYRO_SENSITIVITY;
	} else {
		return -99;
	}
}

int16_t ICM_20948::getGyroY() {
	uint8_t buf[2];
	HAL_StatusTypeDef ret;

	ret = readReg(GYRO_YOUT_H, buf, 2);
	int yGyro = twoComplementToDec(addBinary(buf[1], buf[0]));

	if (ret == HAL_OK) {
		return yGyro / GYRO_SENSITIVITY;
	} else {
		return -99;
	}
}

int16_t ICM_20948::getGyroZ() {
	uint8_t buf[2];
	HAL_StatusTypeDef ret;

	ret = readReg(GYRO_ZOUT_H, buf, 2);
	int zGyro = twoComplementToDec(addBinary(buf[1], buf[0]));

	if (ret == HAL_OK) {
		return zGyro / GYRO_SENSITIVITY;
	} else {
		return -99;
	}
}

float ICM_20948::getAccelX() {
	uint8_t buf[2];
	HAL_StatusTypeDef ret;

	ret = readReg(ACCEL_XOUT_H, buf, 2);
	int xAccel = twoComplementToDec(addBinary(buf[1], buf[0]));

	if (ret == HAL_OK) {
		return float(xAccel) / ACCEL_SENSITIVITY;
	} else {
		return -99;
	}
}

float ICM_20948::getAccelY() {
	uint8_t buf[2];
	HAL_StatusTypeDef ret;

	ret = readReg(ACCEL_YOUT_H, buf, 2);
	int yAccel = twoComplementToDec(addBinary(buf[1], buf[0]));

	if (ret == HAL_OK) {
		return float(yAccel) / ACCEL_SENSITIVITY;
	} else {
		return -99;
	}
}

float ICM_20948::getAccelZ() {
	uint8_t buf[2];
	HAL_StatusTypeDef ret;

	ret = readReg(ACCEL_ZOUT_H, buf, 2);
	int zAccel = twoComplementToDec(addBinary(buf[1], buf[0]));

	if (ret == HAL_OK) {
		return float(zAccel) / ACCEL_SENSITIVITY;
	} else {
		return -99;
	}
}

float ICM_20948::getMagX() {
	uint8_t buf[2];
	HAL_StatusTypeDef ret;

	ret = readReg(EXT_SLV_SENS_DATA_00, buf, 2);
	int xMag = twoComplementToDec(addBinary(buf[0], buf[1]));

	if (ret == HAL_OK) {
		return float(xMag) / MAG_SENSITIVITY;
	} else {
		return -99;
	}
}

int16_t ICM_20948::getMagY() {

}
int16_t ICM_20948::getMagZ() {

}

int16_t ICM_20948::getTemp() {
	uint8_t buf[2];
	HAL_StatusTypeDef ret;

	ret = readReg(TEMP_OUT_H, buf, 2);
	int temp = twoComplementToDec(addBinary(buf[1], buf[0]));

	if (ret == HAL_OK) {
		return ((temp - 21) / 400) + 21;
	} else {
		return -99;
	}
}

bool ICM_20948::isDeviceReady() {
	HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&i2c, address, 1, 100);
	if (ret == HAL_OK) {
		return 1;
	} else {
		return 0;
	}
}

HAL_StatusTypeDef ICM_20948::writeReg(int regAddress, int data,
		int dataAmount, int userbank) {

	writeReg(REG_BANK_SEL, userbank << 4, 1, 1);

	HAL_StatusTypeDef ret;
	uint8_t temp = data;
	ret = HAL_I2C_Mem_Write(&i2c, address, regAddress, 1, &temp, dataAmount,
	HAL_MAX_DELAY);
	return ret;
}

HAL_StatusTypeDef ICM_20948::readReg(int regAddress, uint8_t *buf,
		int dataAmount) {
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Read(&i2c, address, regAddress, 1, buf, dataAmount,
	HAL_MAX_DELAY);
	return ret;
}

uint8_t ICM_20948::readReg(int regAddress, int dataAmount) {
	uint8_t buf[1];
	HAL_I2C_Mem_Read(&i2c, address, regAddress, 1, buf, dataAmount,
	HAL_MAX_DELAY);
	return buf[0];
}

//Works
uint16_t ICM_20948::addBinary(uint8_t lowByte, uint8_t highByte) {
	return ((highByte << 8) | lowByte);
}

//Works
int16_t ICM_20948::twoComplementToDec(uint16_t val) {
	if ((val & 0x8000) == 0) {
		return val;
	} else {
		val = ~(val) + 1;
		return -val;
	}
}

