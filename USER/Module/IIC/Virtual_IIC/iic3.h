//
// Created by SuperChen on 2025/1/8.
//

#ifndef ENGINEERING_USER_CONTROL_IIC4_H
#define ENGINEERING_USER_CONTROL_IIC4_H

#include <stm32f411xe.h>
#include "main.h"
#include "Hardware_i2c1.h"

// SCL
#define I2C_MONI_SCL_GPIO3      IIC_SCL3_GPIO_Port
#define I2C_MONI_SCL_PIN3       IIC_SCL3_Pin
// SDA
#define I2C_MONI_SDA_GPIO3      IIC_SDA3_GPIO_Port
#define I2C_MONI_SDA_PIN3       IIC_SDA3_Pin

void IICSoft_Init_3(void);
uint8_t detect_magnet_3(void);
uint16_t get_raw_angle_3(void);

void AS5600_Init_3(AS5600_Encoder_t *encoder);
uint16_t get_angle_3(void); // 可以修改的角度
void AS5600_Set_TempZeroByReg_3(AS5600_Encoder_t *encoder, uint16_t new_zero_raw);  // 设定零点
void AS5600_Update_3(AS5600_Encoder_t *encoder);

#endif //ENGINEERING_USER_CONTROL_IIC4_H
