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

void Software_IIC_Init_3(void);
uint16_t get_raw_angle_3(void); // 可以修改的角度
void MT6701_Init_3(MT6701_Encoder_t *encoder);
void MT6701_SetZero_3(MT6701_Encoder_t *encoder);  // 设定零点
float get_real_angle_3(MT6701_Encoder_t *encoder);
void MT6701_Update_3(MT6701_Encoder_t *encoder);

#endif //ENGINEERING_USER_CONTROL_IIC4_H
