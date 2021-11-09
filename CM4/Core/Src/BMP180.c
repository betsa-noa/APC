#include "stm32h7xx_hal.h"
#include "math.h"

extern I2C_HandleTypeDef hi2c1;
#define BMP180_I2C &hi2c1

#define BMP180_ADDRESS 0xEE


short AC1 = 0;
short AC2 = 0;
short AC3 = 0;
unsigned short AC4 = 0;
unsigned short AC5 = 0;
unsigned short AC6 = 0;
short B1 = 0;
short B2 = 0;
short MB = 0;
short MC = 0;
short MD = 0;

long UT = 0;
long UP = 0;
long X1 = 0;
long X2 = 0;
long B5 = 0;
long B6 = 0;
long X3 = 0;
long B3 = 0;
long oss = 0;
unsigned long B4 = 0;
unsigned long B7 = 0;

long P = 0;
long T = 0;


#define atmPress 101325

void BMP180_read_calibration_data (void){
	uint8_t Calib_Data[22] = {0};
	uint16_t Calib_Start = 0;
	HAL_I2C_Mem_Read(BMP180_I2C, BMP180_ADDRESS,Calib_Start,1, Calib_Data, 22, HAL_MAX_DELAY);

	AC1 = ((Calib_Data[0] << 8) | Calib_Data[1]);
	AC2 = ((Calib_Data[2] << 8) | Calib_Data[3]);
	AC3 = ((Calib_Data[3] << 8) | Calib_Data[5]);
	AC4 = ((Calib_Data[6] << 8) | Calib_Data[7]);
	AC5 = ((Calib_Data[8] << 8) | Calib_Data[9]);
	AC6 = ((Calib_Data[10] << 8) | Calib_Data[11]);
	B1 = ((Calib_Data[12] << 8) | Calib_Data[13]);
	B2 = ((Calib_Data[14] << 8) | Calib_Data[15]);
	MB = ((Calib_Data[16] << 8) | Calib_Data[17]);
	MC = ((Calib_Data[18] << 8) | Calib_Data[19]);
	MD = ((Calib_Data[20] << 8) | Calib_Data[21]);
}


uint16_t BMP180_read_uncompensated_temperature (void){
	uint8_t Temp_Data = 0x2E;
	uint8_t Reg_Temp[2] = {0};
	HAL_I2C_Mem_Write(BMP180_I2C, BMP180_ADDRESS, 0xF4, 1, &Temp_Data, 1, 1000);
	HAL_Delay(5);
	HAL_I2C_Mem_Read(BMP180_I2C, BMP180_ADDRESS, 0xF6, 1, Reg_Temp, 2, 1000);
	return ((Reg_Temp[0]<<8) + Reg_Temp[1]);
}

float BMP180_calculate_true_temperature(void){
	UT = BMP180_read_uncompensated_temperature();
	X1 = (UT - AC6) * (AC5 / pow(2,15));
	X2 = (MC * (pow(2,11))) / (X1 + MD);
	B5 = X1 + X2;
	T = (B5 + 8) / (pow(2,4));
	return T / 10.0;
}

uint32_t BMP180_read_uncompensated_pressure (int oss){
	uint8_t Press_Data = 0x34+(oss<<6);
	uint8_t Reg_Data[3] = {0};
	HAL_I2C_Mem_Write(BMP180_I2C, BMP180_ADDRESS, 0xF4, 1, &Press_Data, 1, 1000);
	switch(oss)
	{
		case (0):
			HAL_Delay (5);
			break;
		case (1):
			HAL_Delay (8);
			break;
		case (3):
			HAL_Delay (14);
			break;
		case (4):
			HAL_Delay (26);
			break;
	}
	HAL_I2C_Mem_Read(BMP180_I2C, BMP180_ADDRESS, 0xF6, 1, Reg_Data, 3, 1000);
	return (((Reg_Data[0]<<16)+(Reg_Data[1]<<8) + Reg_Data[2]) >> (8 - oss));
}

float BMP180_calculate_true_pressure (int oss){
	UP = BMP180_read_uncompensated_pressure(oss);
	B6 = B5 - 4000;
	X1 = (B2 * (B6 * B6) / pow(2,12)) / (pow(2,11));
	X2 = AC2 * B6 / pow(2,11);
	X3 = X1 + X2;
	B3 = (((AC1 * 4 + X3)<<oss) + 2) / 4;
	X1 = AC3 * B6 / pow(2,13);
	X2 = (B1 * (B6 * B6) / pow(2,12)) / (pow(2,16));
	X3 = ((X1 + X2) + 2) / (pow(2,2));
	B4 = AC4 * (unsigned long)(X3 + 32768) / pow(2,15);
	B7 = ((unsigned long)UP - B3) * (50000 >> oss);
	if (B7 < 0x80000000)  P = (B7 * 2) / B4;
	else P = (B7 / B4) * 2;
	X1 = (P / pow(2,8)) * (P / pow(2,8));
	X1 = (X1 * 3038) / pow(2,16);
	X2 = (-7357 * P) / pow(2,16);
	P = P + (X1 + X2 + 3791) / pow(2,4);
	return P;
}

void BMP180_Start(void){
	BMP180_read_calibration_data();
}



