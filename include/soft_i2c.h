#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Default RT-Spark Sensor I2C Pins: SCL = PF1, SDA = PF0
 * (If scanner finds nothing, change to GPIOB, GPIO_PIN_6 / GPIO_PIN_7) */
#define I2C_SCL_PORT    GPIOF
#define I2C_SCL_PIN     GPIO_PIN_1
#define I2C_SDA_PORT    GPIOF
#define I2C_SDA_PIN     GPIO_PIN_0

void SoftI2C_Init(void);
bool SoftI2C_Probe(uint8_t addr7);
bool SoftI2C_WriteBytes(uint8_t addr7, const uint8_t *data, uint16_t len);
bool SoftI2C_ReadBytes(uint8_t addr7, uint8_t *data, uint16_t len);
bool SoftI2C_WriteReg(uint8_t addr7, uint8_t reg, uint8_t val);
bool SoftI2C_ReadRegs(uint8_t addr7, uint8_t reg, uint8_t *buf, uint16_t len);

#endif