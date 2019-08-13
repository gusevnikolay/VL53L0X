#ifndef _TOFLIB_H_
#define _TOFLIB_H_

#include <stdint.h>
//the number of sensors on the bus
#define TOF_SENSORS_COUNT 2
//these addresses will be set to sensors. The length of the array must be equal to the number of sensors on the bus
#define TOF_ADRESSES_LIST {0x28, 0x30}


extern int tofGetModel(int *model, int *revision);
extern int tofReadDistance(uint8_t chipId); //must be form 0 to (TOF_SENSORS_COUNT-1)
extern void tofInitAll(void);

//functions that need to be implemented by the user
extern void i2cRead(uint8_t address, uint8_t reg, uint8_t *buffer, int count);
extern void i2cWrite(uint8_t address, uint8_t reg, uint8_t *buffer, int count);
extern void tofSetResetState(uint8_t sensorNumber, uint8_t resetPinState);
extern void varDelayMs(int ms);

//example
/*
void i2cRead(uint8_t address, uint8_t reg, uint8_t *buffer, int count)
{
	HAL_I2C_Mem_Read(&hi2c2, address, reg, I2C_MEMADD_SIZE_8BIT, buffer, count, 100);
}

void i2cWrite(uint8_t address, uint8_t reg, uint8_t *buffer, int count)
{
	HAL_I2C_Mem_Write(&hi2c2, address, reg, I2C_MEMADD_SIZE_8BIT, buffer, count, 100);
}

void tofSetResetState(uint8_t sensorNumber, uint8_t resetPinState)
{
	if(sensorNumber==0)HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, resetPinState>0?GPIO_PIN_SET:GPIO_PIN_RESET);
	if(sensorNumber==1)HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, resetPinState>0?GPIO_PIN_SET:GPIO_PIN_RESET);
}

void varDelayMs(int ms)
{
	HAL_Delay(ms);
}

int main(void)
{
  tofInitAll();
  while (1)
  {
		int valueOne = tofReadDistance(0);
		int valueTwo = tofReadDistance(1);
  }
}
*/

#endif
