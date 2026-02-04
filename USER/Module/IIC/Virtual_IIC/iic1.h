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

static void Software_IIC_Init_1(void);
uint16_t get_raw_angle_1(void);
void MT6701_Init_1(MT6701_Encoder_t *encoder) ;
void MT6701_SetZero_1(MT6701_Encoder_t *encoder);
float get_real_angle_1(MT6701_Encoder_t *encoder);
void MT6701_Update_1(MT6701_Encoder_t *encoder);

#endif //ENGINEERING_USER_CONTROL_IIC2_H
