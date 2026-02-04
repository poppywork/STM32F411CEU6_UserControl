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

void Software_IIC_Init_4(void);
uint16_t get_raw_angle_4(void); // 可以修改的角度
void MT6701_Init_4(MT6701_Encoder_t *encoder);
void MT6701_SetZero_4(MT6701_Encoder_t *encoder);  // 设定零点
float get_real_angle_4(MT6701_Encoder_t *encoder);
void MT6701_Update_4(MT6701_Encoder_t *encoder);

#endif //ENGINEERING_USER_CONTROL_IIC4_H
