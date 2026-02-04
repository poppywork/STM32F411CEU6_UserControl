//
// Created by SuperChen on 2025/1/8.
//

#ifndef ENGINEERING_USER_CONTROL_IIC3_H
#define ENGINEERING_USER_CONTROL_IIC3_H

#include <stm32f411xe.h>
#include "main.h"
#include "Hardware_i2c1.h"

// SCL
#define I2C_MONI_SCL_GPIO2      IIC_SCL2_GPIO_Port
#define I2C_MONI_SCL_PIN2       IIC_SCL2_Pin
// SDA
#define I2C_MONI_SDA_GPIO2      IIC_SDA2_GPIO_Port
#define I2C_MONI_SDA_PIN2       IIC_SDA2_Pin

void Software_IIC_Init_2(void);
uint16_t get_raw_angle_2(void); // 不可以修改的恒定角度
void MT6701_Init_2(MT6701_Encoder_t *encoder);
void MT6701_SetZero_2(MT6701_Encoder_t *encoder);  // 设定零点
float get_real_angle_2(MT6701_Encoder_t *encoder);
void MT6701_Update_2(MT6701_Encoder_t *encoder);

#endif //ENGINEERING_USER_CONTROL_IIC3_H
