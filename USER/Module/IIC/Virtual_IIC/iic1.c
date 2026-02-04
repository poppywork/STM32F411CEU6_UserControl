//
// Created by Chenyudog on 2025/11/26.
//
#include "iic1.h"
#include "tim_delay.h"

#define SCL_L   (I2C_MONI_SCL_GPIO1->BSRR = (I2C_MONI_SCL_PIN1 << 16))
#define SCL_H   (I2C_MONI_SCL_GPIO1->BSRR =  I2C_MONI_SCL_PIN1)
#define SDA_L   (I2C_MONI_SDA_GPIO1->BSRR = (I2C_MONI_SDA_PIN1 << 16))
#define SDA_H   (I2C_MONI_SDA_GPIO1->BSRR =  I2C_MONI_SDA_PIN1)
#define SDA_READ   ((I2C_MONI_SDA_GPIO1->IDR & I2C_MONI_SDA_PIN1)?1:0)


void Software_IIC_Init_1(void)
{
    SCL_H;
    SDA_H;         delayUs(5);
}

//SCL高电平 SDA高电平  ->SCL高电平 SDA低电平
static void Start(void) {
    //IIC开始前的信号
    SCL_H;        delayUs(5);
    SDA_H;        delayUs(5);
    //IIC开始的信号
    SDA_L;        delayUs(5);
    SCL_L;        delayUs(5);
}

//SCL高电平 SDA低电平  ->SCL高电平 SDA高电平
static void Stop(void) {
       //不论一开始是什么信号都拉低 防止误读
    //IIC开始前的信号
    SCL_L;        delayUs(5);
    SDA_L;        delayUs(5);

    //IIC开始的信号
    SCL_H;        delayUs(5);
    SDA_H;        delayUs(5);

}

static void slave_ackYes(void) {
    SCL_L; delayUs(5);
    SDA_L; delayUs(5);  // 主机主动拉低SDA，告诉从机继续发数据
    SCL_H; delayUs(5);
    SCL_L; delayUs(5);
    SDA_H; delayUs(5);  // 释放SDA，准备下一次读取
}

static void slave_ackNo(void) {
    SCL_L;        delayUs(5);
    // 释放SDA，由从机保持高表示NACK
    SDA_H;        delayUs(5);

    SCL_H;        delayUs(5);
    SCL_L;        delayUs(5);
}

static uint8_t master_waitForAck(void)
{
    uint8_t  d=0;

    SCL_L;        delayUs(5);  // 拉低SCL，给SDA释放准备时间（F411必需）
    SDA_H;        delayUs(5);  // 释放SDA，确保电平稳定（抗RC延迟）

    SCL_H;        delayUs(5);  // 5μs→12μs，适配F411+MT6701的慢应答（比标准多2μs冗余）
    d=SDA_READ;                 // 采样（高电平中段，最稳定）
    // 删掉多余的 delayUs(5); —— 核心修正！

    SCL_L; delayUs(5);         // 1μs→5μs，确保SCL电平稳定拉低（核心修正）

    return d ? 1 : 0; // d=1→非应答，d=0→应答（逻辑不变）
}

static void master_SendByte(uint8_t data)
{
    for(uint8_t i=0; i<8; i++)
    {
        SCL_L;         delayUs(5);

        (data&0x80)? SDA_H : SDA_L;
        data<<=1;      delayUs(5);

        SCL_H;         delayUs(5);
    }
}

static uint8_t master_readByte(uint8_t ack) {
    uint8_t data = 0;

    // 1. 读字节前：强制释放SDA（主机放弃总线控制权）
    SDA_H;
    delayUs(5); // 确保SDA完全拉高

    for(uint8_t i = 0; i < 8; i++) {
        // 步骤1：拉低SCL，让从机更新SDA数据（必须）
        SCL_L;
        delayUs(5); // 50KHz标准：低电平≥4.7μs，给从机足够时间准备数据

        // 步骤2：拉高SCL，锁定从机数据（此时SDA必须稳定）
        SCL_H;
        delayUs(5); // 数据建立时间（短延时，等SDA电平稳定）

        // 步骤3：采样SDA（核心修正：SCL高电平稳定后采样）
        data <<= 1;
        if(SDA_READ) {
            data |= 0x01;
        }
    }

    // 2. 读字节后：拉低SCL，准备发送ACK/NACK（必须）
    SCL_L;
    delayUs(5);

    // 3. 发送ACK/NACK（主机接管SDA，告诉从机是否继续）
    if(ack) {
        slave_ackNo(); // 最后1字节：NACK（高电平）
    } else {
        slave_ackYes(); // 非最后字节：ACK（低电平）
    }

    // 4. 应答后：再次拉低SCL，释放SDA，为下一字节做准备
    SCL_L;
    SDA_H;
    delayUs(5);

    return data;
}

// 修正后的 master_readByte_Complete 函数
// 修正后的 I2C 多字节读函数（语法+时序双修复）
static uint8_t master_readByte_Complete(uint8_t slave, uint8_t addr, uint8_t *buf, uint8_t len)
{
    Start();

    // 第一步：发写地址（7位地址<<1 + 0，修复语法换行问题）
    master_SendByte(slave << 1 | 0);
    if(master_waitForAck())      // 等待从机应答
    {
        Stop();
        return 1; // 应答失败，返回错误
    }
    // 第二步：发送要读取的寄存器地址（MT6701角度寄存器0x03）
    master_SendByte(addr);
    if(master_waitForAck())      // 等待从机应答
    {
        Stop();
        return 1; // 应答失败，返回错误
    }

    // 第三步：重复起始信号（切换为读模式）
    Start();

    // 第四步：发读地址（7位地址<<1 + 1）
    master_SendByte (slave << 1 | 1);
    if(master_waitForAck())      // 等待从机应答
    {
        Stop();
        return 1; // 应答失败，返回错误
    }

    // 第五步：读取多字节数据
    while (len > 0) {
        if (len == 1) {
            *buf = master_readByte(1); // 最后1字节：主机发NACK
        } else {
            *buf = master_readByte(0); // 非最后字节：主机发ACK
        }
        len--;
        buf++;
        SCL_L;
        delayUs(5); // 修复时序：增加电平稳定延时
    }

    Stop();
    return 0; // 读取成功，返回0
}

// 修正后的 master_readByte_Complete 函数
static uint8_t master_writeByte(uint8_t slave, uint8_t addr, uint8_t data)
{
    // IIC读寄存器标准流程：
    // 起始→发写地址（7b+0）→等从机应答→发寄存器地址→等从机应答→
    // 重复起始→发读地址（7b+1）→等从机应答→读数据（主机发ACK/NACK）→停止

    Start();

    // 第一步：发【写地址】（slave是7位地址左移1位的写地址，比如0x0C）
    // 错误点1修正：原代码用 slave|1（读地址）发写操作，改为直接发slave（写地址）
    master_SendByte(slave<<1 | 0);     // 发从机写地址（7位地址<<1 + 0）

    // 错误点2修正：删除手动slave_ackYes()，改为等待从机真实应答
    if(master_waitForAck())      // 等待从机应答
    {
        Stop();
        return 1;
    }
    // 第二步：发送要读取的寄存器地址（0x03）
    master_SendByte(addr);
    if(master_waitForAck())      // 等待从机应答
    {
        Stop();
        return 1;
    }



    master_SendByte(data);               // 发送数据， 写数据和读数据最大的时序区别在这一步前
    if(master_waitForAck())
    {
        Stop ();
        return 1;                 // 数据写入失败
    }

    Stop();                       // 写入完成，产生停止信号
    return 0;
}


// 读取MT6701传感器的原始角度值（14位有效数据）
// 返回值：14位原始角度值（范围0~16383，对应0~360°）
uint16_t get_raw_angle_1(void)
{
    uint8_t Raw[2] = {0, 0};           // 存储读取的2字节数据：Raw[0]=角度高8位，Raw[1]=角度低8位

    // 步骤1：通过IIC读取MT6701角度寄存器（连续读2字节）
    // MT6701_SLAVE_ADDR：传感器IIC从机写地址（7位地址<<1+0）
    // MT6701_REG_ANGLE_H：角度高8位寄存器地址
    // Raw数组：接收数据，Raw[0]=高8位，Raw[1]=低8位
    master_readByte_Complete(MT6701_SLAVE_ADDR, MT6701_REG_ANGLE_H, Raw, 2);

    // 步骤2：拼接14位原始角度值（MT6701硬件协议规则）
    // Raw[0]（高8位）左移6位 → 占14位的高8位；Raw[1]右移2位 → 取低6位的高2位，组合为14位数据
    // 注：MT6701的角度数据共14位，低2位为无效位，需舍弃


    // 返回拼接后的14位原始角度值
    return (Raw[0] << 6) | (Raw[1] >> 2);
}

// 计算单圈相对零点的实际角度（单位：度，浮点型）
// encoder：对应编码器的结构体指针，必须传入
float get_real_angle_1(MT6701_Encoder_t *encoder)
{
    // 核心公式：(当前14位原始角度 - 14位零点原始角度) × 360° / 14位量程(16384)
    // 强制转为int16_t防止无符号数相减溢出（如raw_angle < raw_zero时）
    int16_t angle_diff = (encoder->raw_angle - encoder->raw_zero);
    encoder->real_angle = (float)angle_diff * 360.0f / 16384.0f;

    // 可选：将单圈角度限制在【0~360°】或【-180~180°】，根据需求选择
    // 限制0~360°
    if (encoder->real_angle < 0.0f)
    {
        encoder->real_angle += 360.0f;
    }

    return encoder->real_angle;
}

// 初始化MT6701编码器结构体及软件IIC总线
// encoder：MT6701编码器结构体指针（存储角度、圈数、偏移等核心参数）
// 初始化MT6701编码器结构体及软件IIC总线
void MT6701_Init_1(MT6701_Encoder_t *encoder) {
    Software_IIC_Init_1();
    encoder->raw_zero = 0;
    encoder->raw_angle = 0;
    encoder->real_angle = 0;
    encoder->last_raw_angle = 0;
    encoder->turns = 0;
    encoder->total_angle = 0;  // 累计值初始化为0
    encoder->diff = 0;
    encoder->total_angle_deg = 0;
    encoder->total_angle= 0;
    encoder->last_total_angle_deg = 0;

    // 核心修正：初始化后读当前角度，赋值给last_raw_angle，避免第一次diff跳变
    encoder->first_raw_angle = get_raw_angle_1();
    encoder->last_raw_angle = encoder->first_raw_angle; // 同步历史角度为当前角度
}


// 设置MT6701原始角度零点（当前角度设为零点）
// encoder：编码器结构体指针
void MT6701_SetZero_1(MT6701_Encoder_t *encoder) {

    uint8_t zpos[2]={0,0};

    // 2. 写入ZPOS寄存器（0x01和0x02）
    if (master_readByte_Complete(MT6701_SLAVE_ADDR,MT6701_ZERO_H,zpos,2) ) {

        return;
    }

    // 4. 同步结构体中的零点记录（可选，用于软件层一致性）
    encoder->raw_zero =get_raw_angle_1();
}
 void MT6701_Update_1(MT6701_Encoder_t *encoder) {
    // 步骤1：仅读取1次原始角度（避免I2C冗余读取）
    encoder->raw_angle = get_raw_angle_1();

    // 步骤2：计算单圈相对零点的实际角度
    get_real_angle_1(encoder);
    // 优化打印：增加total_angle/圈数/总角度，方便调试累计情况
    USART1_DebugPrintf("单圈=%.2f° | 累计原始值=%d | 圈数=%d | 总角度=%.2f°\r\n",
                       encoder->real_angle, encoder->total_angle,
                       encoder->turns, encoder->total_angle_deg);

    // 步骤3：计算角度差，【核心修正】用14位标准跨圈阈值8192/-8192
    encoder->diff = (encoder->raw_angle - encoder->last_raw_angle);
    if (encoder->diff > 8192) {        // 超过量程一半，判定为正向跨圈
        encoder->diff -= 16384;
    } else if (encoder->diff < -8192) { // 低于负的量程一半，判定为反向跨圈
        encoder->diff += 16384;
    }

    // 步骤4：diff连续累加，实现多圈累计
    encoder->total_angle += encoder->diff;
    encoder->turns = (encoder->total_angle / 16384);

    // 核心补全：根据累计原始值计算浮点总角度（度），同步更新
    encoder->total_angle_deg = (float)encoder->total_angle * 360.0f / 16384.0f;

    // 步骤5：更新历史角度，为下一次计算diff做准备
    encoder->last_raw_angle = encoder->raw_angle;
    encoder->last_total_angle_deg = encoder->total_angle_deg;
}