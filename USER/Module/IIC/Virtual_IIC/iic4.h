//
// Created by SuperChen on 2025/1/8.
//

#ifndef ENGINEERING_USER_CONTROL_IIC7_H
#define ENGINEERING_USER_CONTROL_IIC7_H

#include <stm32f411xe.h>
#include "main.h"
#include "Hardware_i2c1.h"

// SCL
#define I2C_MONI_SCL_GPIO4      IIC_SCL4_GPIO_Port
#define I2C_MONI_SCL_PIN4       IIC_SCL4_Pin
// SDA
#define I2C_MONI_SDA_GPIO4      IIC_SDA4_GPIO_Port
#define I2C_MONI_SDA_PIN4       IIC_SDA4_Pin

void IICSoft_Init_4(void);
uint8_t detect_magnet_4(void);
uint16_t get_raw_angle_4(void);

void AS5600_Init_4(AS5600_Encoder_t *encoder);
uint16_t get_angle_4(void); // 可以修改的角度
void AS5600_Set_TempZeroByReg_4(AS5600_Encoder_t *encoder, uint16_t new_zero_raw);  // 设定零点
void AS5600_Update_4(AS5600_Encoder_t *encoder);

#endif //ENGINEERING_USER_CONTROL_IIC4_H
