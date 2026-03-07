
#include "arm_math.h"
#include "i2c.h"
#include "Hardware_i2c1.h"
#include "tim_delay.h"

/*******************************************************
 * 函数: get_raw_angle_5
 * 输入: 无
 * 输出: 原始角度寄存器的值
 * 描述: 使用硬件IIC(hi2c1)获取磁铁位置的原始值，不受起始/结束角度或最大角度设置影响
 ******************************************************/
uint16_t get_raw_angle_5(void)
{
    uint16_t getRaw = 0;               // 存储组合后的原始角度值
    uint8_t Raw[2] = {0, 0};           // 存储读取到的两个字节（高8位和低8位）
    HAL_StatusTypeDef status;          // 存储IIC操作状态

    // 使用硬件IIC1读取AMS5600的原始角度寄存器（_raw_ang_hi为高8位地址）
    // 连续读取2个字节到Raw数组（Raw[0]为高8位，Raw[1]为低8位）
    status = HAL_I2C_Mem_Read(&hi2c1,
                              _ams5600_Address << 1,  // 设备地址左移1位（HAL库要求）
                              _raw_ang_hi,           // 寄存器地址
                              I2C_MEMADD_SIZE_8BIT,  // 8位寄存器地址
                              Raw,                   // 接收数据缓冲区
                              2,                     // 读取字节数
                              50);                   // 超时时间(ms)

    // 检查IIC读取是否成功
    if (status == HAL_OK)
    {
        // 将高8位左移8位，与低8位组合成16位原始角度值
        getRaw = (Raw[0] << 8) | Raw[1];
    }
    else
    {
        // 可根据需要添加错误处理，如打印调试信息
        // USART1_DebugPrintf("I2C1读取原始角度失败: %d\r\n", status);
        getRaw = 0;  // 读取失败时返回0
    }

    return getRaw;
}

// 用于记录该函数的调用次数
int detect_mag_5 = 0;

uint8_t detect_magnet_5(void)
{
    uint8_t magStatus;       // 存储状态寄存器的值
    uint8_t retVal = 0;      // 函数返回值，默认0（未检测到磁铁）
    HAL_StatusTypeDef status; // 存储IIC操作状态

    /* 状态寄存器位结构：0 0 MD ML MH 0 0 0
     * MD位为高：AGC最小增益溢出（磁铁过强）
     * ML位为高：AGC最大增益溢出（磁铁过弱）
     * MH位为高：检测到磁铁（正常状态）
     */

    // 通过硬件IIC2读取AMS5600的状态寄存器（_stat）值到magStatus
    status = HAL_I2C_Mem_Read(&hi2c1,
                              _ams5600_Address << 1,  // 设备地址左移1位（HAL库要求）
                              _stat,                  // 状态寄存器地址
                              I2C_MEMADD_SIZE_8BIT,   // 8位寄存器地址
                              &magStatus,             // 接收数据缓冲区
                              1,                      // 读取1个字节
                              50);                    // 超时时间(ms)

    // 每次调用该函数，计数器自增
    detect_mag_5++;

    // 检查IIC读取是否成功且MH位为高
    if (status == HAL_OK && (magStatus & 0x20))
        retVal = 1;  // 检测到磁铁，返回1

    return retVal;
}

void AS5600_Init_5(AS5600_Encoder_t *encoder) {
    // 硬件IIC初始化由HAL系统初始化完成，无需单独调用软件初始化函数
    // 此处可添加IIC状态检查（如需要）

    encoder->first_raw_angle = 0;
    encoder->raw_angle = 0;
    encoder->zero_position = 0;  // 默认零点，可通过函数修改

    encoder->angle = 0;
    encoder->last_angle = 0;

    encoder->turns = 0;
    encoder->total_angle = 0;
    encoder->is_initialized = false;

    // 读取首次角度（需确保磁铁已检测）
    if (detect_magnet_5()) {
        encoder->first_raw_angle = get_raw_angle_5();
        encoder->raw_angle = encoder->first_raw_angle;
        encoder->is_initialized = true;
    }
}

void AS5600_Set_TempZeroByReg_5(AS5600_Encoder_t *encoder, uint16_t new_zero_raw) {
    if (!encoder->is_initialized || !detect_magnet_5()) {
        return; // 未初始化或无磁铁，退出
    }

    // 1. 拆分12位原始角度为高4位和低8位
    uint8_t zpos_hi = (new_zero_raw >> 8) & 0x0F; // 高4位（仅低4位有效）
    uint8_t zpos_lo = new_zero_raw & 0xFF;        // 低8位
    HAL_StatusTypeDef status;

    // 2. 写入ZPOS寄存器（0x01和0x02）使用硬件IIC2
    // 写入高4位到_zpos_hi（0x01）
    status = HAL_I2C_Mem_Write(&hi2c1,
                               _ams5600_Address << 1,  // 设备地址左移1位
                               _zpos_hi,               // 寄存器地址
                               I2C_MEMADD_SIZE_8BIT,   // 8位寄存器地址
                               &zpos_hi,               // 发送数据
                               1,                      // 发送1个字节
                               50);                    // 超时时间(ms)
    if (status != HAL_OK) {
        USART1_DebugPrintf("ZPOS_HI写入失败\r\n");
        return;
    }

    // 写入低8位到_zpos_lo（0x02）
    status = HAL_I2C_Mem_Write(&hi2c1,
                               _ams5600_Address << 1,
                               _zpos_lo,
                               I2C_MEMADD_SIZE_8BIT,
                               &zpos_lo,
                               1,
                               50);
    if (status != HAL_OK) {
        USART1_DebugPrintf("ZPOS_LO写入失败\r\n");
        return;
    }

    // 3. 延迟确保设置生效（文档要求寄存器修改后至少1ms生效）
    HAL_Delay(4);

    // 4. 同步结构体中的零点记录
    encoder->zero_position = new_zero_raw;
}

uint16_t get_angle_5(void) {
    uint16_t angle = 0;
    uint8_t ang[2] = {0, 0};
    HAL_StatusTypeDef status;

    // 使用硬件IIC2读取ANGLE寄存器（0x0E和0x0F）
    status = HAL_I2C_Mem_Read(&hi2c1,
                              _ams5600_Address << 1,
                              _ang_hi,                // 角度高位寄存器地址
                              I2C_MEMADD_SIZE_8BIT,
                              ang,                    // 接收缓冲区（高位在前）
                              2,                      // 读取2个字节
                              50);

    if (status == HAL_OK) {
        // 组合为16位值并保留12位有效数据
        angle = ((ang[0] << 8) | ang[1]) & 0x0FFF;
    } else {
        // 可添加错误处理
        angle = 0;
    }

    return angle;
}

/**
 * @brief 更新编码器角度并统计多圈角度
 * @param encoder AS5600结构体指针
 */
void AS5600_Update_5(AS5600_Encoder_t *encoder) {
    if (!encoder->is_initialized || !detect_magnet_5()) {
        return; // 未初始化或无磁铁，不更新
    }

    encoder->angle = get_angle_5(); // 读当前原始角度
    int16_t diff = (int16_t )(encoder->angle - encoder->last_angle); // 计算差值

    // 处理跳变：判断方向并修正角度差
    if (diff > 2048) {
        // 逆时针跳变（0 → 4095，实际是减少了一圈）
        diff -= 4096; // 修正为负数（代表逆时针转）
    } else if (diff < -2048) {
        // 顺时针跳变（4095 → 0，实际是增加了一圈）
        diff += 4096; // 修正为正数（代表顺时针转）
    }

    // 更新总角度
    encoder->total_angle += diff;
    // 计算当前圈数（总角度除以360，取整数部分）
    encoder->turns = (int32_t)(encoder->total_angle / 4096);

    // 计算角度差对应的度数（12位分辨率：360°/4096）
    encoder->total_angle_deg = (float)(encoder->total_angle) * (360.0f / 4096.0f);
    encoder->total_angle_deg = AS5600FILTER * encoder->total_angle_deg + (1 - AS5600FILTER) * encoder->last_total_angle_deg;

    // 缓存当前原始角度，供下次比较
    encoder->last_angle = encoder->angle;
    encoder->last_total_angle_deg = encoder->total_angle_deg;
}





/*******************************************************
 * 函数: get_raw_angle_6
 * 输入: 无
 * 输出: 原始角度寄存器的值
 * 描述: 使用硬件IIC(hi2c2)获取磁铁位置的原始值，不受起始/结束角度或最大角度设置影响
 ******************************************************/
uint16_t get_raw_angle_6(void)
{
    uint16_t getRaw = 0;               // 存储组合后的原始角度值
    uint8_t Raw[2] = {0, 0};           // 存储读取到的两个字节（高8位和低8位）
    HAL_StatusTypeDef status;          // 存储IIC操作状态

    // 使用硬件IIC2读取AMS5600的原始角度寄存器（_raw_ang_hi为高8位地址）
    status = HAL_I2C_Mem_Read(&hi2c2,
                              _ams5600_Address << 1,  // 设备地址左移1位
                              _raw_ang_hi,           // 寄存器地址
                              I2C_MEMADD_SIZE_8BIT,  // 8位寄存器地址
                              Raw,                   // 接收数据缓冲区
                              2,                     // 读取字节数
                              50);                   // 超时时间(ms)

    if (status == HAL_OK)
    {
        getRaw = (Raw[0] << 8) | Raw[1];
    }
    else
    {
        // 可添加错误处理
        getRaw = 0;
    }

    return getRaw;
}

// 用于记录该函数的调用次数
int detect_mag_6 = 0;

uint8_t detect_magnet_6(void)
{
    uint8_t magStatus;       // 存储状态寄存器的值
    uint8_t retVal = 0;      // 函数返回值，默认0（未检测到磁铁）
    HAL_StatusTypeDef status; // 存储IIC操作状态

    /* 状态寄存器位结构：0 0 MD ML MH 0 0 0
     * MD位为高：AGC最小增益溢出（磁铁过强）
     * ML位为高：AGC最大增益溢出（磁铁过弱）
     * MH位为高：检测到磁铁（正常状态）
     */

    // 通过硬件IIC2读取AMS5600的状态寄存器（_stat）值到magStatus
    status = HAL_I2C_Mem_Read(&hi2c2,
                              _ams5600_Address << 1,  // 设备地址左移1位（HAL库要求）
                              _stat,                  // 状态寄存器地址
                              I2C_MEMADD_SIZE_8BIT,   // 8位寄存器地址
                              &magStatus,             // 接收数据缓冲区
                              1,                      // 读取1个字节
                              50);                    // 超时时间(ms)

    // 每次调用该函数，计数器自增
    detect_mag_6++;

    // 检查IIC读取是否成功且MH位为高
    if (status == HAL_OK && (magStatus & 0x20))
        retVal = 1;  // 检测到磁铁，返回1

    return retVal;
}

void AS5600_Init_6(AS5600_Encoder_t *encoder) {
    // 硬件IIC初始化由HAL系统初始化完成，无需单独调用软件初始化函数
    // 此处可添加IIC状态检查（如需要）

    encoder->first_raw_angle = 0;
    encoder->raw_angle = 0;
    encoder->zero_position = 0;  // 默认零点，可通过函数修改

    encoder->angle = 0;
    encoder->last_angle = 0;

    encoder->turns = 0;
    encoder->total_angle = 0;
    encoder->is_initialized = false;

    // 读取首次角度（需确保磁铁已检测）
    if (detect_magnet_6()) {
        encoder->first_raw_angle = get_raw_angle_6();
        encoder->raw_angle = encoder->first_raw_angle;
        encoder->is_initialized = true;
    }
}

void AS5600_Set_TempZeroByReg_6(AS5600_Encoder_t *encoder, uint16_t new_zero_raw) {
    if (!encoder->is_initialized || !detect_magnet_6()) {
        return; // 未初始化或无磁铁，退出
    }

    // 1. 拆分12位原始角度为高4位和低8位
    uint8_t zpos_hi = (new_zero_raw >> 8) & 0x0F; // 高4位（仅低4位有效）
    uint8_t zpos_lo = new_zero_raw & 0xFF;        // 低8位
    HAL_StatusTypeDef status;

    // 2. 写入ZPOS寄存器（0x01和0x02）使用硬件IIC2
    // 写入高4位到_zpos_hi（0x01）
    status = HAL_I2C_Mem_Write(&hi2c2,
                               _ams5600_Address << 1,  // 设备地址左移1位
                               _zpos_hi,               // 寄存器地址
                               I2C_MEMADD_SIZE_8BIT,   // 8位寄存器地址
                               &zpos_hi,               // 发送数据
                               1,                      // 发送1个字节
                               50);                    // 超时时间(ms)
    if (status != HAL_OK) {
        USART1_DebugPrintf("ZPOS_HI写入失败\r\n");
        return;
    }

    // 写入低8位到_zpos_lo（0x02）
    status = HAL_I2C_Mem_Write(&hi2c2,
                               _ams5600_Address << 1,
                               _zpos_lo,
                               I2C_MEMADD_SIZE_8BIT,
                               &zpos_lo,
                               1,
                               50);
    if (status != HAL_OK) {
        USART1_DebugPrintf("ZPOS_LO写入失败\r\n");
        return;
    }

    // 3. 延迟确保设置生效（文档要求寄存器修改后至少1ms生效）
    HAL_Delay(4);

    // 4. 同步结构体中的零点记录
    encoder->zero_position = new_zero_raw;
}

uint16_t get_angle_6(void) {
    uint16_t angle = 0;
    uint8_t ang[2] = {0, 0};
    HAL_StatusTypeDef status;

    // 使用硬件IIC2读取ANGLE寄存器（0x0E和0x0F）
    status = HAL_I2C_Mem_Read(&hi2c2,
                              _ams5600_Address << 1,
                              _ang_hi,                // 角度高位寄存器地址
                              I2C_MEMADD_SIZE_8BIT,
                              ang,                    // 接收缓冲区（高位在前）
                              2,                      // 读取2个字节
                              50);

    if (status == HAL_OK) {
        // 组合为16位值并保留12位有效数据
        angle = ((ang[0] << 8) | ang[1]) & 0x0FFF;
    } else {
        // 可添加错误处理
        angle = 0;
    }

    return angle;
}

/**
 * @brief 更新编码器角度并统计多圈角度
 * @param encoder AS5600结构体指针
 */
void AS5600_Update_6(AS5600_Encoder_t *encoder) {
    if (!encoder->is_initialized || !detect_magnet_6()) {
        return; // 未初始化或无磁铁，不更新
    }

    encoder->angle = get_angle_6(); // 读当前原始角度
    int16_t diff = (int16_t )(encoder->angle - encoder->last_angle); // 计算差值

    // 处理跳变：判断方向并修正角度差
    if (diff > 2048) {
        // 逆时针跳变（0 → 4095，实际是减少了一圈）
        diff -= 4096; // 修正为负数（代表逆时针转）
    } else if (diff < -2048) {
        // 顺时针跳变（4095 → 0，实际是增加了一圈）
        diff += 4096; // 修正为正数（代表顺时针转）
    }

    // 更新总角度
    encoder->total_angle += diff;
    // 计算当前圈数（总角度除以360，取整数部分）
    encoder->turns = (int32_t)(encoder->total_angle / 4096);

    // 计算角度差对应的度数（12位分辨率：360°/4096）
    encoder->total_angle_deg = (float)(encoder->total_angle) * (360.0f / 4096.0f);
    encoder->total_angle_deg = AS5600FILTER * encoder->total_angle_deg + (1 - AS5600FILTER) * encoder->last_total_angle_deg;

    // 缓存当前原始角度，供下次比较
    encoder->last_angle = encoder->angle;
    encoder->last_total_angle_deg = encoder->total_angle_deg;
}



/*******************************************************
 * 函数: get_raw_angle_7
 * 输入: 无
 * 输出: 原始角度寄存器的值
 * 描述: 使用硬件IIC(hi2c3)获取磁铁位置的原始值，不受起始/结束角度或最大角度设置影响
 ******************************************************/
uint16_t get_raw_angle_7(void)
{
    uint16_t getRaw = 0;               // 存储组合后的原始角度值
    uint8_t Raw[2] = {0, 0};           // 存储读取到的两个字节（高8位和低8位）
    HAL_StatusTypeDef status;          // 存储IIC操作状态

    // 使用硬件IIC3读取AMS5600的原始角度寄存器（_raw_ang_hi为高8位地址）
    status = HAL_I2C_Mem_Read(&hi2c3,
                              _ams5600_Address << 1,  // 设备地址左移1位
                              _raw_ang_hi,           // 寄存器地址
                              I2C_MEMADD_SIZE_8BIT,  // 8位寄存器地址
                              Raw,                   // 接收数据缓冲区
                              2,                     // 读取字节数
                              50);                   // 超时时间(ms)

    if (status == HAL_OK)
    {
        getRaw = (Raw[0] << 8) | Raw[1];
    }
    else
    {
        // 可添加错误处理
        getRaw = 0;
    }

    return getRaw;
}

// 用于记录该函数的调用次数
int detect_mag_7 = 0;

uint8_t detect_magnet_7(void)
{
    uint8_t magStatus;       // 存储状态寄存器的值
    uint8_t retVal = 0;      // 函数返回值，默认0（未检测到磁铁）
    HAL_StatusTypeDef status; // 存储IIC操作状态

    /* 状态寄存器位结构：0 0 MD ML MH 0 0 0
     * MD位为高：AGC最小增益溢出（磁铁过强）
     * ML位为高：AGC最大增益溢出（磁铁过弱）
     * MH位为高：检测到磁铁（正常状态）
     */

    // 通过硬件IIC2读取AMS5600的状态寄存器（_stat）值到magStatus
    status = HAL_I2C_Mem_Read(&hi2c3,
                              _ams5600_Address << 1,  // 设备地址左移1位（HAL库要求）
                              _stat,                  // 状态寄存器地址
                              I2C_MEMADD_SIZE_8BIT,   // 8位寄存器地址
                              &magStatus,             // 接收数据缓冲区
                              1,                      // 读取1个字节
                              50);                    // 超时时间(ms)

    // 每次调用该函数，计数器自增
    detect_mag_7++;

    // 检查IIC读取是否成功且MH位为高
    if (status == HAL_OK && (magStatus & 0x20))
        retVal = 1;  // 检测到磁铁，返回1

    return retVal;
}

void AS5600_Init_7(AS5600_Encoder_t *encoder) {
    // 硬件IIC初始化由HAL系统初始化完成，无需单独调用软件初始化函数
    // 此处可添加IIC状态检查（如需要）

    encoder->first_raw_angle = 0;
    encoder->raw_angle = 0;
    encoder->zero_position = 0;  // 默认零点，可通过函数修改

    encoder->angle = 0;
    encoder->last_angle = 0;

    encoder->turns = 0;
    encoder->total_angle = 0;
    encoder->is_initialized = false;

    // 读取首次角度（需确保磁铁已检测）
    if (detect_magnet_7()) {
        encoder->first_raw_angle = get_raw_angle_7();
        encoder->raw_angle = encoder->first_raw_angle;
        encoder->is_initialized = true;
    }
}

void AS5600_Set_TempZeroByReg_7(AS5600_Encoder_t *encoder, uint16_t new_zero_raw) {
    if (!encoder->is_initialized || !detect_magnet_7()) {
        return; // 未初始化或无磁铁，退出
    }

    // 1. 拆分12位原始角度为高4位和低8位
    uint8_t zpos_hi = (new_zero_raw >> 8) & 0x0F; // 高4位（仅低4位有效）
    uint8_t zpos_lo = new_zero_raw & 0xFF;        // 低8位
    HAL_StatusTypeDef status;

    // 2. 写入ZPOS寄存器（0x01和0x02）使用硬件IIC2
    // 写入高4位到_zpos_hi（0x01）
    status = HAL_I2C_Mem_Write(&hi2c3,
                               _ams5600_Address << 1,  // 设备地址左移1位
                               _zpos_hi,               // 寄存器地址
                               I2C_MEMADD_SIZE_8BIT,   // 8位寄存器地址
                               &zpos_hi,               // 发送数据
                               1,                      // 发送1个字节
                               50);                    // 超时时间(ms)
    if (status != HAL_OK) {
        USART1_DebugPrintf("ZPOS_HI写入失败\r\n");
        return;
    }

    // 写入低8位到_zpos_lo（0x02）
    status = HAL_I2C_Mem_Write(&hi2c3,
                               _ams5600_Address << 1,
                               _zpos_lo,
                               I2C_MEMADD_SIZE_8BIT,
                               &zpos_lo,
                               1,
                               50);
    if (status != HAL_OK) {
        USART1_DebugPrintf("ZPOS_LO写入失败\r\n");
        return;
    }

    // 3. 延迟确保设置生效（文档要求寄存器修改后至少1ms生效）
    HAL_Delay(4);

    // 4. 同步结构体中的零点记录
    encoder->zero_position = new_zero_raw;
}

uint16_t get_angle_7(void) {
    uint16_t angle = 0;
    uint8_t ang[2] = {0, 0};
    HAL_StatusTypeDef status;

    // 使用硬件IIC2读取ANGLE寄存器（0x0E和0x0F）
    status = HAL_I2C_Mem_Read(&hi2c3,
                              _ams5600_Address << 1,
                              _ang_hi,                // 角度高位寄存器地址
                              I2C_MEMADD_SIZE_8BIT,
                              ang,                    // 接收缓冲区（高位在前）
                              2,                      // 读取2个字节
                              50);

    if (status == HAL_OK) {
        // 组合为16位值并保留12位有效数据
        angle = ((ang[0] << 8) | ang[1]) & 0x0FFF;
    } else {
        // 可添加错误处理
        angle = 0;
    }

    return angle;
}

/**
 * @brief 更新编码器角度并统计多圈角度
 * @param encoder AS5600结构体指针
 */
void AS5600_Update_7(AS5600_Encoder_t *encoder) {
    if (!encoder->is_initialized || !detect_magnet_7()) {
        return; // 未初始化或无磁铁，不更新
    }

    encoder->angle = get_angle_5(); // 读当前原始角度
    int16_t diff = (int16_t )(encoder->angle - encoder->last_angle); // 计算差值

    // 处理跳变：判断方向并修正角度差
    if (diff > 2048) {
        // 逆时针跳变（0 → 4095，实际是减少了一圈）
        diff -= 4096; // 修正为负数（代表逆时针转）
    } else if (diff < -2048) {
        // 顺时针跳变（4095 → 0，实际是增加了一圈）
        diff += 4096; // 修正为正数（代表顺时针转）
    }

    // 更新总角度
    encoder->total_angle += diff;
    // 计算当前圈数（总角度除以360，取整数部分）
    encoder->turns = (int32_t)(encoder->total_angle / 4096);

    // 计算角度差对应的度数（12位分辨率：360°/4096）
    encoder->total_angle_deg = (float)(encoder->total_angle) * (360.0f / 4096.0f);
    encoder->total_angle_deg = AS5600FILTER * encoder->total_angle_deg + (1 - AS5600FILTER) * encoder->last_total_angle_deg;

    // 缓存当前原始角度，供下次比较
    encoder->last_angle = encoder->angle;
    encoder->last_total_angle_deg = encoder->total_angle_deg;
}