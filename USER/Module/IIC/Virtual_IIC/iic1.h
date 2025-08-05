//
// Created by SuperChen on 2025/1/8.
//

#ifndef ENGINEERING_USER_CONTROL_IIC2_H
#define ENGINEERING_USER_CONTROL_IIC2_H

#include <stm32f411xe.h>
#include "main.h"
#include "Hardware_i2c1.h"

// SCL
#define I2C_MONI_SCL_GPIO1      IIC_SCL1_GPIO_Port
#define I2C_MONI_SCL_PIN1       IIC_SCL1_Pin
// SDA
#define I2C_MONI_SDA_GPIO1      IIC_SDA1_GPIO_Port
#define I2C_MONI_SDA_PIN1       IIC_SDA1_Pin

void IICSoft_Init_1(void);
uint8_t detect_magnet_1(void);
uint16_t get_raw_angle_1(void); // 不可以修改的恒定角度

void AS5600_Init_1(AS5600_Encoder_t *encoder);
uint16_t get_angle_1(void); // 可以修改的角度
void AS5600_Set_TempZeroByReg_1(AS5600_Encoder_t *encoder, uint16_t new_zero_raw);  // 设定零点
void AS5600_Update_1(AS5600_Encoder_t *encoder);

#endif //ENGINEERING_USER_CONTROL_IIC2_H
