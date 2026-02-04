//
// Created by 24319 on 2025/4/5.
//

#ifndef ENGINEERING_USERDEFINE_VERSION2_HARDWARE_I2C1_H
#define ENGINEERING_USERDEFINE_VERSION2_HARDWARE_I2C1_H

#include "stm32f4xx_hal.h"

#include <stdint.h>
#include <stdbool.h>

#include "usart_task.h"


typedef enum
{
    MT6701_SLAVE_ADDR=0x06,     //MT6701  IIC地址
    MT6701_REG_ANGLE_H=0x03,     // 高8位
    MT6701_REG_ANGLE_L=0x04,    // 低4位
    MT6701_ZERO_H=0x32,         //高四位
    MT6701_ZERO_L=0x33,         //低八位
    //14Bit角度信息，存储在0x03[13:6]、0x04[5:0]两个寄存器中，高位在前，原始读数0~16383

} MT6701_Registers_e;

#define MT6701FILTER 0.3f // 低通滤波系数

// 定义MT6701编码器结构体
typedef struct {
    uint32_t first_raw_angle;        //14位角度原始值

    uint32_t raw_angle;        //14位角度原始值
    float real_angle;       //14位零点角度真实值
    uint32_t raw_zero;         //14位零点原始值
    uint32_t last_raw_angle;       // 上一次读取的原始角度（用于检测圈数跳变）

    // 多圈记录相关
    int32_t  diff;            // 圈数计数（正：顺时针多圈；负：逆时针多圈）
    int32_t  turns;     // 圈数计数（正：顺时针多圈；负：逆时针多圈）
    int32_t  total_angle;      // 总角度（= turns*360 + angle_deg）

    float    last_total_angle_deg;
    float    total_angle_deg;


} MT6701_Encoder_t;



uint16_t get_raw_angle_5(void);
void MT6701_Init_5(MT6701_Encoder_t *encoder);
void MT6701_SetZero_5(MT6701_Encoder_t *encoder);
float get_real_angle_5(MT6701_Encoder_t *encoder);
void MT6701_Update_5(MT6701_Encoder_t *encoder);


uint16_t get_raw_angle_6(void);
void MT6701_Init_6(MT6701_Encoder_t *encoder);
void MT6701_SetZero_6(MT6701_Encoder_t *encoder);
float get_real_angle_6(MT6701_Encoder_t *encoder);
void MT6701_Update_6(MT6701_Encoder_t *encoder);

uint16_t get_raw_angle_7(void);
void MT6701_Init_7(MT6701_Encoder_t *encoder);
void MT6701_SetZero_7(MT6701_Encoder_t *encoder);
float get_real_angle_7(MT6701_Encoder_t *encoder);
void MT6701_Update_7(MT6701_Encoder_t *encoder);


#endif //ENGINEERING_USERDEFINE_VERSION2_HARDWARE_I2C1_H
