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
/* set i2c address */
    _ams5600_Address = 0x36,
    _zmco = 0x00,
    _zpos_hi = 0x01,
    _zpos_lo = 0x02,
    _mpos_hi = 0x03,
    _mpos_lo = 0x04,
    _mang_hi = 0x05,
    _mang_lo = 0x06,
    _conf_hi = 0x07,
    _conf_lo = 0x08,
    _raw_ang_hi = 0x0c,
    _raw_ang_lo = 0x0d,
    _ang_hi = 0x0e,
    _ang_lo = 0x0f,
    _stat = 0x0b,
    _agc = 0x1a,
    _mag_hi = 0x1b,
    _mag_lo = 0x1c,
    _burn = 0xff
} AS5600_Registers_e;

#define AS5600FILTER 0.3f // 低通滤波系数

// 定义AS5600编码器结构体
typedef struct {
    // 基础角度数据
    uint16_t first_raw_angle;      // 首次读取的原始角度（初始化时记录）
    uint16_t raw_angle;        // 原始角度（寄存器直接读取值，0-4095，12位分辨率）
    uint16_t zero_position;    // 自定义零点对应的原始角度（默认0，可通过编程设置）

    uint16_t angle;            // 真实角度值（0-4095，12位分辨率，已扣除零点偏移）
    uint16_t last_angle;   // 上一次读取的原始角度（用于检测圈数跳变）

    // 多圈记录相关
    int32_t  turns;            // 圈数计数（正：顺时针多圈；负：逆时针多圈）
    int32_t    total_angle;      // 总角度（= turns*360 + angle_deg）

    float    last_total_angle_deg;
    float    total_angle_deg;  // 低通滤波

    bool     is_initialized;   // 初始化标志（判断是否已记录首次角度）
} AS5600_Encoder_t;



uint16_t get_raw_angle_5(void);
uint8_t detect_magnet_5(void);
void AS5600_Init_5(AS5600_Encoder_t *encoder);
void AS5600_Set_TempZeroByReg_5(AS5600_Encoder_t *encoder, uint16_t new_zero_raw);
uint16_t get_angle_5(void);
void AS5600_Update_5(AS5600_Encoder_t *encoder);

uint16_t get_raw_angle_6(void);
uint8_t detect_magnet_6(void);
void AS5600_Init_6(AS5600_Encoder_t *encoder);
void AS5600_Set_TempZeroByReg_6(AS5600_Encoder_t *encoder, uint16_t new_zero_raw);
uint16_t get_angle_6(void);
void AS5600_Update_6(AS5600_Encoder_t *encoder);

uint16_t get_raw_angle_7(void);
uint8_t detect_magnet_7(void);
void AS5600_Init_7(AS5600_Encoder_t *encoder);
void AS5600_Set_TempZeroByReg_7(AS5600_Encoder_t *encoder, uint16_t new_zero_raw);
uint16_t get_angle_7(void);
void AS5600_Update_7(AS5600_Encoder_t *encoder);


#endif //ENGINEERING_USERDEFINE_VERSION2_HARDWARE_I2C1_H
