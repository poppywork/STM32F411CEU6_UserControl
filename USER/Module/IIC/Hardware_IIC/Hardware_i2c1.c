#include "arm_math.h"
#include "i2c.h"
#include "Hardware_i2c1.h"
#include "tim_delay.h"

uint16_t get_raw_angle_5(void)
{
    uint16_t getRaw = 0;               // 存储组合后的原始角度值
    uint8_t Raw[2] = {0, 0};           // 存储读取到的两个字节（高8位和低8位）
    HAL_StatusTypeDef status;          // 存储IIC操作状态

    // 使用硬件IIC1读取AMS5600的原始角度寄存器（MT6701_REG_ANGLE_H为高8位地址）
    // 连续读取2个字节到Raw数组（Raw[0]为高8位，Raw[1]为低8位）
    status = HAL_I2C_Mem_Read(&hi2c1,
                              MT6701_SLAVE_ADDR << 1,  // 设备地址左移1位（HAL库要求）
                              MT6701_REG_ANGLE_H,           // 寄存器地址
                              I2C_MEMADD_SIZE_8BIT,  // 8位寄存器地址
                              Raw,                   // 接收数据缓冲区
                              2,                     // 读取字节数
                              50);                   // 超时时间(ms)

    // 检查IIC读取是否成功
    if (status == HAL_OK)
    {
        // 将高8位左移8位，与低8位组合成16位原始角度值
        getRaw = (Raw[0] << 6) | (Raw[1] >>2);
    }
    else
    {
        // 可根据需要添加错误处理，如打印调试信息
        // USART1_DebugPrintf("I2C1读取原始角度失败: %d\r\n", status);
        getRaw = 0;  // 读取失败时返回0
    }

    return getRaw;
}


void MT6701_Init_5(MT6701_Encoder_t *encoder) {
    encoder->raw_zero = 0;
    encoder->raw_angle = 0;
    encoder->real_angle = 0;
    encoder->last_raw_angle = 0;
    encoder->turns = 0;
    encoder->total_angle = 0;  // 累计值初始化为0
    encoder->diff = 0;
    encoder->total_angle_deg = 0;
    encoder->last_total_angle_deg = 0;

    // 核心修正：初始化后读当前角度，赋值给last_raw_angle，避免第一次diff跳变
    encoder->first_raw_angle = get_raw_angle_5();
    encoder->last_raw_angle = encoder->first_raw_angle; // 同步历史角度为当前角度
}

void MT6701_SetZero_5(MT6701_Encoder_t *encoder) {
    uint8_t zero_posi[2] = {0, 0};
    HAL_StatusTypeDef status;          // 存储IIC操作状态
    status = HAL_I2C_Mem_Read(&hi2c1,
                              MT6701_SLAVE_ADDR << 1,  // 设备地址左移1位（HAL库要求）
                              MT6701_ZERO_H,       // 寄存器地址
                              I2C_MEMADD_SIZE_8BIT,  // 8位寄存器地址
                              zero_posi,                   // 接收数据缓冲区
                              2,                     // 读取字节数
                              50);                   // 超时时间(ms)
    HAL_Delay(1); //延迟1ms确保生效
    if (status == HAL_OK) {
        // 拼接12位零位值：ZERO_H低4位有效 + ZERO_L全8位有效（MT6701硬件规则）
        encoder->raw_zero = (((zero_posi[0] & 0x0F) << 8) | (zero_posi[1] & 0xFF))*4;
    }
}

float get_real_angle_5(MT6701_Encoder_t *encoder){
    encoder->real_angle= (float)(encoder->raw_angle - encoder->raw_zero)*360.0f/16384.0f;
    return encoder->real_angle;
}

void MT6701_Update_5(MT6701_Encoder_t *encoder) {


    encoder->raw_angle = get_raw_angle_5(); // 读当前原始角度
    int16_t diff =encoder->raw_angle - encoder->last_raw_angle; // 计算差值

    // 处理跳变：判断方向并修正角度差
    if (diff > 8192) {
        // 逆时针跳变（0 → 4095，实际是减少了一圈）
        diff -= 16384; // 修正为负数（代表逆时针转）
    } else if (diff < -8192) {
        // 顺时针跳变（4095 → 0，实际是增加了一圈）
        diff += 16384; // 修正为正数（代表顺时针转）
    }

    // 更新总角度
    encoder->total_angle += diff;
    // 计算当前圈数（总角度除以360，取整数部分）
    encoder->turns = (int32_t)(encoder->total_angle / 16384);

    // 计算角度差对应的度数（12位分辨率：360°/16384）
    encoder->total_angle_deg = (float)(encoder->total_angle) * (360.0f / 16384.0f);
    encoder->total_angle_deg = MT6701FILTER * encoder->total_angle_deg + (1 - MT6701FILTER) * encoder->last_total_angle_deg;

    // 缓存当前原始角度，供下次比较
    encoder->last_raw_angle = encoder->raw_angle;
    encoder->last_total_angle_deg = encoder->total_angle_deg;
}


uint16_t get_raw_angle_6(void)
{
    uint16_t getRaw = 0;               // 存储组合后的原始角度值
    uint8_t Raw[2] = {0, 0};           // 存储读取到的两个字节（高8位和低8位）
    HAL_StatusTypeDef status;          // 存储IIC操作状态

    // 使用硬件IIC1读取AMS5600的原始角度寄存器（MT6701_REG_ANGLE_H为高8位地址）
    // 连续读取2个字节到Raw数组（Raw[0]为高8位，Raw[1]为低8位）
    status = HAL_I2C_Mem_Read(&hi2c3,
                              MT6701_SLAVE_ADDR << 1,  // 设备地址左移1位（HAL库要求）
                              MT6701_REG_ANGLE_H,           // 寄存器地址
                              I2C_MEMADD_SIZE_8BIT,  // 8位寄存器地址
                              Raw,                   // 接收数据缓冲区
                              2,                     // 读取字节数
                              50);                   // 超时时间(ms)

    // 检查IIC读取是否成功
    if (status == HAL_OK)
    {
        // 将高8位左移8位，与低8位组合成16位原始角度值
        getRaw = (Raw[0] << 6) | (Raw[1] >>2);
    }
    else
    {
        // 可根据需要添加错误处理，如打印调试信息
        // USART1_DebugPrintf("I2C1读取原始角度失败: %d\r\n", status);
        getRaw = 0;  // 读取失败时返回0
    }

    return getRaw;
}


void MT6701_Init_6(MT6701_Encoder_t *encoder) {
    encoder->raw_zero = 0;
    encoder->raw_angle = 0;
    encoder->real_angle = 0;
    encoder->last_raw_angle = 0;
    encoder->turns = 0;
    encoder->total_angle = 0;  // 累计值初始化为0
    encoder->diff = 0;
    encoder->total_angle_deg = 0;
    encoder->last_total_angle_deg = 0;

    // 核心修正：初始化后读当前角度，赋值给last_raw_angle，避免第一次diff跳变
    encoder->first_raw_angle = get_raw_angle_6();
    encoder->last_raw_angle = encoder->first_raw_angle; // 同步历史角度为当前角度
}

void MT6701_SetZero_6(MT6701_Encoder_t *encoder) {
    uint8_t zero_posi[2] = {0, 0};
    HAL_StatusTypeDef status;          // 存储IIC操作状态
    status = HAL_I2C_Mem_Read(&hi2c3,
                              MT6701_SLAVE_ADDR << 1,  // 设备地址左移1位（HAL库要求）
                              MT6701_ZERO_H,       // 寄存器地址
                              I2C_MEMADD_SIZE_8BIT,  // 8位寄存器地址
                              zero_posi,                   // 接收数据缓冲区
                              2,                     // 读取字节数
                              50);                   // 超时时间(ms)
    HAL_Delay(1); //延迟1ms确保生效
    if (status == HAL_OK) {
        // 拼接12位零位值：ZERO_H低4位有效 + ZERO_L全8位有效（MT6701硬件规则）
        encoder->raw_zero = (((zero_posi[0] & 0x0F) << 8) | (zero_posi[1] & 0xFF))*4;
    }
}

float get_real_angle_6(MT6701_Encoder_t *encoder){
    encoder->real_angle= (float)(encoder->raw_angle - encoder->raw_zero)*360.0f/16384.0f;
    return encoder->real_angle;
}

void MT6701_Update_6(MT6701_Encoder_t *encoder) {


    encoder->raw_angle = get_raw_angle_6(); // 读当前原始角度
    int16_t diff =encoder->raw_angle - encoder->last_raw_angle; // 计算差值

    // 处理跳变：判断方向并修正角度差
    if (diff > 8192) {
        // 逆时针跳变（0 → 4095，实际是减少了一圈）
        diff -= 16384; // 修正为负数（代表逆时针转）
    } else if (diff < -8192) {
        // 顺时针跳变（4095 → 0，实际是增加了一圈）
        diff += 16384; // 修正为正数（代表顺时针转）
    }

    // 更新总角度
    encoder->total_angle += diff;
    // 计算当前圈数（总角度除以360，取整数部分）
    encoder->turns = (encoder->total_angle / 16384);

    // 计算角度差对应的度数（12位分辨率：360°/16384）
    encoder->total_angle_deg = (float)(encoder->total_angle) * (360.0f / 16384.0f);

    // 缓存当前原始角度，供下次比较
    encoder->last_raw_angle = encoder->raw_angle;
    encoder->last_total_angle_deg = encoder->total_angle_deg;
}
