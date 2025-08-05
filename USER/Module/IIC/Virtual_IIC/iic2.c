//
// Created by SuperChen on 2025/1/8.
//

#include "iic2.h"
#include "tim_delay.h"

#define SCL_L      (I2C_MONI_SCL_GPIO2->BSRR = (I2C_MONI_SCL_PIN2 << 16))
#define SCL_H      (I2C_MONI_SCL_GPIO2->BSRR =  I2C_MONI_SCL_PIN2)
#define SDA_L      (I2C_MONI_SDA_GPIO2->BSRR = (I2C_MONI_SDA_PIN2 << 16))
#define SDA_H      (I2C_MONI_SDA_GPIO2->BSRR =  I2C_MONI_SDA_PIN2)
#define SDA_READ   ((I2C_MONI_SDA_GPIO2->IDR & I2C_MONI_SDA_PIN2)?1:0)       // 不用改成输入模式就能读


static void     isBusy (void);          // 检测总线是否空闲
static void     start  (void);          // 起始信号
static void     stop   (void);          // 停止信号
static void     ackYes (void);          // 发送应答信号
static void     ackNo  (void);          // 不发送应答信号
static uint8_t  waitForAck(void);       // 等待应答信号

static void     sendByte(uint8_t data);      // 发送一个字节
static uint8_t  readByte(uint8_t ack);       // 读取一个字节

void IICSoft_Init_2(void)
{
    // CubeMax完成初始化
    delayUs(2);
    SCL_L ;
    delayUs(2);
    SDA_H ;
    delayUs(2);
    SCL_H;
    SDA_H;
    delayUs(2);
}

// 判断总线是否处于空闲状态，若不是，则循环的提供SCL时钟驱动，直到从机释放SDA线
static void isBusy(void)
{
    uint32_t i=500;                  // 大约3ms
    //SDA_IN ();
    while((SDA_READ) == 0 && (--i))  // 读取sda线上的电平状态，低电平说明总线被从机控制，高电平则说明总线空闲，可以准备发送开始信号
    {
        SCL_L;
        delayUs(1);
        SCL_H ;
        delayUs(1);
    }
    if(i==0)    USART1_DebugPrintf("\r\n SOFT_IIC_2: 总线超时错误!! \r\n");   // 错误提示
}

static void start(void)
{
    isBusy ();               // 判断总线是否处于空闲状态

    SCL_L;   delayUs(1);
    SDA_H;   delayUs(1);

    SCL_H;   delayUs(1);
    SDA_L;   delayUs(1);     // 在SCL高电平其间， SDA由高向低 跳变
    SCL_L;   delayUs(1);     // 将SCL拉低,钳住SCL线,准备发送数据
}

static void stop(void)
{
    SCL_L;   delayUs(1);
    SDA_L;   delayUs(1);

    SCL_H;   delayUs(1);
    SDA_H ;  delayUs(1);     // 在SCL高电平其间， SDA由低向高 跳变
}

static void ackYes(void)
{
    SCL_L;    delayUs(1);
    SDA_L;    delayUs(1);

    SCL_H;    delayUs(10);   // 使SDA保持一个时钟的低电平
    SCL_L;    delayUs(1);

    SDA_H ;   delayUs(1);   // 很重要: 在这里释放SDA线,从机才能获取控制权
}

static void ackNo(void)
{
    SCL_L;    delayUs(1);
    SDA_H;    delayUs(1);

    SCL_H;    delayUs(10);   // 使SDA保持一个时钟的高电平
    SCL_L;    delayUs(1);

    SDA_H;    delayUs(1);   // 很重要: 在这里释放SDA线,从机才能获取控制权
}

static uint8_t waitForAck(void)
{
    uint8_t  d=0;

    SCL_L;    delayUs(1);
    SDA_H;    delayUs(1);        // 主机释放SDA线，使总线空闲，以便从机能够响应信息

    SCL_H;    delayUs(10);        // 等待10us; 按标准模式100kbit/s
    d=SDA_READ;

    SCL_L;    delayUs(1);
    SDA_H;    delayUs(1);        // 主机释放SDA线，使总线空闲，以便从机能够响应信息

    if(d)
        return 1;                 // 返回非应答信号
    else
        return 0;                 // 返回应答信号
}

static void sendByte(uint8_t data)
{
    for(uint8_t i=0; i<8; i++)
    {
        SCL_L;     delayUs(1);

        (data&0x80)? SDA_H : SDA_L;
        data<<=1;  delayUs(1);

        SCL_H;     delayUs(1);
    }

    SCL_L;    delayUs(1);
    SDA_H;    delayUs(1);      // 主机释放SDA线，使用总线空闲，以便从机能够发出响应信息，并钳信SCL线
}

static uint8_t readByte(uint8_t ack)
{
    uint8_t data=0;

    for(uint8_t i=0; i<8; i++)
    {
        SCL_H;    delayUs(1);
        data<<=1;
        if(SDA_READ)  data|=0x01;
        SCL_L;    delayUs(1);
    }

    if(ack) ackNo ();               // 1, 不应答
    else    ackYes();               // 0, 应答

    return  data;
}

static uint8_t IICSoft_ReadByte(uint8_t slave, uint8_t addr, uint8_t *buf)
{
    start ();                     // 起始信号
    sendByte (slave<<1 | 0);      // 从机地址,　写方向 , 0写1读
    if(waitForAck())
    {
        stop();
        return 1;
    }

    sendByte(addr);               // 数据地址
    if(waitForAck())
    {
        stop();
        return 1;
    }

    // 写数据和读数据最大的时序区别， 写比较简单，读数据：再发一次起始信号、有读方向的从机地址
    start();                      // 起始信号
    sendByte(slave<<1 | 1);       // 从机地址,　读方向 ，  0写1读
    if(waitForAck())
    {
        stop();
        return 1;
    }

    *buf=readByte(1);             // 读值, 并发送ackNo;
    stop();

    return 0;
}

static uint8_t IICSoft_ReadBueffer(uint8_t slave, uint8_t addr, uint8_t *buf, uint8_t len)
{
    start ();                     // 起始信号
    sendByte (slave<<1 | 0);      // 从机地址,　写方向 , 0写1读
    if(waitForAck())              // 等待ACK
    {
        stop();
        return 1;
    }

    sendByte(addr);               // 数据地址
    if(waitForAck())              // 等待ACK
    {
        stop();
        return 1;
    }

    // 写、读时序区别， 读数据：再发一次起始信号、有读方向的从机地址
    start();                      // 起始信号
    sendByte(slave<<1 | 1);       // 从机地址,　读方向 ，  0写1读
    if(waitForAck())              // 等待ACK
    {
        stop();
        return 1;
    }


    while(len)
    {
        if(len==1)  *buf=readByte(1);   // 最后一个字节，读数据后产生NACK
        else        *buf=readByte(0);   // 读数据后产生ACK

        len--;
        buf++;
    }

    stop ();                      // 产生一个停止信号
    return 0;
}

static uint8_t IICSoft_WriteByte(uint8_t slave, uint8_t addr, uint8_t data)
{
    start ();                     // 起始信号
    sendByte (slave<<1 | 0);      // 从机地址,　写方向 , 0写1读
    if(waitForAck())
    {
        stop();
        return 1;                 // 若从机地址发送失败，就返回
    }

    sendByte(addr);               // 数据地址
    if(waitForAck())
    {
        stop();
        return 1;                 // 若发送失败，就返回
    }

    sendByte(data);               // 发送数据， 写数据和读数据最大的时序区别在这一步前
    if(waitForAck())
    {
        stop ();
        return 1;                 // 数据写入失败
    }

    stop();                       // 写入完成，产生停止信号
    return 0;
}

// 往从机写多个字节数据
static uint8_t IICSoft_WriteBuffer(uint8_t slave, uint8_t addr, uint8_t *buf, uint8_t len)
{
    start ();                     // 起始信号
    sendByte (slave<<1 | 0);      // 从机地址,　写方向 , 0写1读
    if(waitForAck())
    {
        stop();
        return 1;                 // 若发送失败，就返回
    }

    sendByte(addr);               // 数据地址
    if(waitForAck())
    {
        stop();
        return 1;                 // 若发送失败，就返回
    }

    for(uint8_t i=0; i<len; i++)
    {
        sendByte(buf[i]);         // 数据
        if(waitForAck())          // 每一个字节都要等从机应答
        {
            stop ();
            return 1;             // 数据写入失败
        }
    }

    stop();                       // 写入完成，产生停止信号
    return 0;
}

/*******************************************************
 * 函数: detect_magnet_2
 * 输入: 无
 * 输出: 1表示检测到磁铁，0表示未检测到
 * 描述: 读取状态寄存器并检查MH位（磁铁检测位）
 ******************************************************/
// 用于记录该函数的调用次数
int detect_mag_2 = 0;

uint8_t detect_magnet_2(void)
{
    uint8_t magStatus;       // 存储状态寄存器的值
    uint8_t retVal = 0;      // 函数返回值，默认0（未检测到磁铁）

    /* 状态寄存器位结构：0 0 MD ML MH 0 0 0
     * MD位为高：AGC最小增益溢出（磁铁过强）
     * ML位为高：AGC最大增益溢出（磁铁过弱）
     * MH位为高：检测到磁铁（正常状态）
     */

    // 通过I2C读取AMS5600的状态寄存器（_stat）值到magStatus
    IICSoft_ReadByte(_ams5600_Address, _stat, &magStatus);

    // 每次调用该函数，计数器自增（可用于统计检测次数或调试）
    detect_mag_2++;

    // 检查状态寄存器的第5位（MH位，对应0x20）是否为1
    // 0x20 = 00100000（二进制），与运算可提取该位
    if (magStatus & 0x20)
        retVal = 1;  // 检测到磁铁，返回1

    return retVal;
}

/*******************************************************
 * 函数: get_raw_angle_2
 * 输入: 无
 * 输出: 原始角度寄存器的值
 * 描述: 获取磁铁位置的原始值，不受起始/结束角度或最大角度设置影响
 ******************************************************/
uint16_t get_raw_angle_2(void)
{
    uint16_t getRaw = 0;               // 存储组合后的原始角度值
    uint8_t Raw[2] = {0, 0};      // 存储读取到的两个字节（高8位和低8位）

    // 注：原函数中的"IICSoft_ReadBueffer2"应为笔误，正确应为"IICSoft_ReadBuffer2"
    // 通过I2C读取AMS5600的原始角度寄存器（_raw_ang_hi为高8位地址）
    // 连续读取2个字节到Raw2数组（Raw2[0]为高8位，Raw2[1]为低8位）
    IICSoft_ReadBueffer(_ams5600_Address, _raw_ang_hi, (uint8_t *)Raw, 2);

    // 将高8位左移8位，与低8位组合成16位原始角度值
    getRaw = (Raw[0] << 8 | Raw[1]);

    return getRaw;
}

void AS5600_Init_2(AS5600_Encoder_t *encoder) {
    IICSoft_Init_2();

    encoder->first_raw_angle = 0;
    encoder->raw_angle = 0;
    encoder->zero_position = 0;  // 默认零点，可通过函数修改

    encoder->angle = 0;
    encoder->last_angle = 0;

    encoder->turns = 0;
    encoder->total_angle = 0;
    encoder->is_initialized = false;

    // 读取首次角度（需确保磁铁已检测，参考detect_magnet_1函数）
    if (detect_magnet_2()) {
        encoder->first_raw_angle = get_raw_angle_2();
        encoder->raw_angle = encoder->first_raw_angle;


        encoder->is_initialized = true;
    }
}

/**
 * @brief 临时修改零点（写入ZPOS寄存器，不执行BURN，掉电失效）
 * @param encoder AS5600结构体指针
 * @param new_zero_raw 新零点对应的原始角度（0~4095，12位）
 */
void AS5600_Set_TempZeroByReg_2(AS5600_Encoder_t *encoder, uint16_t new_zero_raw) {
    if (!encoder->is_initialized || !detect_magnet_2()) {
        return; // 未初始化或无磁铁，退出
    }

    // 1. 拆分12位原始角度为高4位和低8位
    uint8_t zpos_hi = (new_zero_raw >> 8) & 0x0F; // 高4位（仅低4位有效）
    uint8_t zpos_lo = new_zero_raw & 0xFF;        // 低8位

    // 2. 写入ZPOS寄存器（0x01和0x02）
    // 写入高4位到_zpos_hi（0x01）
    if (IICSoft_WriteByte(_ams5600_Address, _zpos_hi, zpos_hi) != 0) {
        USART1_DebugPrintf("ZPOS_HI写入失败\r\n");
        return;
    }
    // 写入低8位到_zpos_lo（0x02）
    if (IICSoft_WriteByte(_ams5600_Address, _zpos_lo, zpos_lo) != 0) {
        USART1_DebugPrintf("ZPOS_LO写入失败\r\n");
        return;
    }

    // 3. 延迟1ms确保设置生效（文档要求寄存器修改后至少1ms生效）
    HAL_Delay(4);

    // 4. 同步结构体中的零点记录（可选，用于软件层一致性）
    encoder->zero_position = new_zero_raw;
}

// 正确的角度读取函数（应读取ANGLE寄存器，受ZPOS影响）
uint16_t get_angle_2(void) {
    uint16_t angle = 0;
    uint8_t ang[2] = {0, 0};
    // 读取ANGLE寄存器（0x0E和0x0F）
    IICSoft_ReadBueffer(_ams5600_Address, _ang_hi, ang, 2);
    angle = (ang[0] << 8 | ang[1]) & 0x0FFF; // 12位有效数据
    return angle;
}


/**
 * @brief 更新编码器角度并统计多圈角度
 * @param encoder AS5600结构体指针
 */
void AS5600_Update_2(AS5600_Encoder_t *encoder) {
    if (!encoder->is_initialized || !detect_magnet_2()) {
        return; // 未初始化或无磁铁，不更新
    }

    encoder->angle = get_angle_2(); // 读当前原始角度
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