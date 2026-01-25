/**
  ******************************************************************************
  * @file    algorithm_task.c
  * @author  Liu JiaJun(187353224@qq.com)
  * @version V1.0.0
  * @date    2025-01-10
  * @brief   机器人算法任务线程，处理复杂算法，避免在其他线程中计算造成阻塞
  ******************************************************************************
  * @attention
  *
  * 本代码遵循GPLv3开源协议，仅供学习交流使用
  * 未经许可不得用于商业用途
  *
  ******************************************************************************
  */

#include "send_task.h"
#include "cmsis_os.h"
#include "drv_dwt.h"
#include <string.h>
#include <stdio.h>
#include "crc8_crc16.h"
#include "usart.h"
#include "usart_task.h"
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
//static struct chassis_cmd_msg chassis_cmd;
//static struct chassis_fdb_msg chassis_fdb;
//static struct trans_fdb_msg trans_fdb;
//static struct ins_msg ins_data;
//
//static publisher_t *pub_chassis;
//static subscriber_t *sub_cmd,*sub_ins,*sub_trans;
//
//static void chassis_pub_init(void);
//static void chassis_sub_init(void);
//static void chassis_pub_push(void);
//static void chassis_sub_pull(void);
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
/* -------------------------------- 调试监测线程相关 --------------------------------- */
static uint32_t send_task_dwt = 0;   // 毫秒监测
static float send_task_dt = 0;       // 线程实际运行时间dt
static float send_task_delta = 0;    // 监测线程运行时间
static float send_task_start_dt = 0; // 监测线程开始时间
/* -------------------------------- 调试监测线程相关 --------------------------------- */

static uint8_t dma_tx_buffer[2][FRAME_SIZE]; // 双缓冲区
static volatile uint8_t current_buffer = 0;           // 当前缓冲区索引
uint8_t dma_busy = 0;        // DMA状态标志位
uint8_t dma_busy2 = 0; // DMA状态标志位2（可选，视具体需求而定）

static float angles[6] = {0.0f}; // 用于临时存储队列中读取的编码器值
static float encoder_values[6] = {0, 0, 0, 0, 0, 0};              // 存储6个编码器值

extern QueueHandle_t xQueue;    // FreeRTOS 队列句柄

void PackData(float *values, uint16_t data_length, RobotArmController_t *tx_data)
{
    static uint32_t frame_seq = 0; // 帧序号
    // 设置帧头
    tx_data->frame_header.sof = 0xA5;                      // 起始字节
    tx_data->frame_header.data_length = data_length;        // 数据段长度
    tx_data->frame_header.seq = frame_seq++;                // 包序号
    if (frame_seq == 0) frame_seq = 1;  // 防止包序号溢出，最多支持255个包序号

    // 计算帧头CRC8
    tx_data->frame_header.crc8 = 0;  // 初始CRC8
    append_CRC8_check_sum((uint8_t *)(&tx_data->frame_header), FRAME_HEADER_LENGTH);  // 添加CRC8校验

    // 设置命令码
    tx_data->cmd_id = ARM_CONTROLLER_CMD_ID;

    // 数据区：6个float（每个编码器值4字节）
    // 设置数据段
    for (int i = 0; i < 7; i++) {
        uint8_t *src = (uint8_t *)&values[i];
        uint8_t *dst = &tx_data->data[i * 4];
        memcpy(dst, src, sizeof(float)); // 自动处理4个字节
    }


    // 计算帧尾CRC16
    tx_data->frame_tail = 0;  // 初始CRC16
    append_CRC16_check_sum((uint8_t *)tx_data, FRAME_SIZE );  // 添加CRC16校验
}

void DMA_Send_Frame(void)
{
    if (dma_busy == 0) { // 判断DMA是否空闲
        dma_busy = 1;    // 标志位置为忙
        // 启动DMA发送
        HAL_UART_Transmit_DMA(&huart6, dma_tx_buffer[current_buffer] , FRAME_SIZE);
        // 切换到另一个缓冲区
        current_buffer = (current_buffer == 0) ? 1 : 0;
    }
}

void DMA_Send_Frame2(void)
{
    if (dma_busy2 == 0) { // 判断DMA是否空闲
        dma_busy2 = 1;    // 标志位置为忙
        // 启动DMA发送
        HAL_UART_Transmit_DMA(&huart2, dma_tx_buffer[current_buffer] , FRAME_SIZE);
        // 切换到另一个缓冲区
        current_buffer = (current_buffer == 0) ? 1 : 0;
    }
}

// DMA发送完成回调函数
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == USART6) {
//        dma_busy = 0; // 标志位置为空闲
//    }
//}

/* -------------------------------- 线程入口 ------------------------------- */
void SendTask_Entry(void const * argument)
{
/* -------------------------------- 外设初始化段落 ------------------------------- */

/* -------------------------------- 外设初始化段落 ------------------------------- */

/* -------------------------------- 线程间Topics初始化 ------------------------------- */
//    chassis_pub_init();
//    chassis_sub_init();
/* -------------------------------- 线程间Topics初始化 ------------------------------- */
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    send_task_dt = dwt_get_delta(&send_task_dwt);
    send_task_start_dt = dwt_get_time_ms();
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    for(;;)
    {
/* -------------------------------- 调试监测线程调度 --------------------------------- */
        send_task_delta = dwt_get_time_ms() - send_task_start_dt;
        send_task_start_dt = dwt_get_time_ms();
        send_task_dt = dwt_get_delta(&send_task_dwt);
/* -------------------------------- 调试监测线程调度 --------------------------------- */
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */
//        chassis_sub_pull();
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */

/* -------------------------------- 线程代码编写段落 ------------------------------- */

        //从队列中获取编码器值
        if (xQueueReceive(xQueue, angles, 0) == pdTRUE) {
            // 更新全局的 encoder_values 数组（可选）
            for (int i = 0; i < 6; i++) {
                encoder_values[i] = angles[i];
            }

            //打包数据到tx_data结构中
            RobotArmController_t tx_data = {0};  // 定义数据包结构体
            PackData(encoder_values, 30, &tx_data);  // 打包数据帧
            // 将打包后的数据写入DMA缓冲区
            memcpy(dma_tx_buffer[current_buffer], &tx_data, FRAME_SIZE);
            // 启动 DMA 发送
            DMA_Send_Frame();
           // DMA_Send_Frame2(); // 发送数据帧

           // printf("Send Data\n");
        }

/* -------------------------------- 线程代码编写段落 ------------------------------- */

/* -------------------------------- 线程发布Topics信息 ------------------------------- */
//        chassis_pub_push();
/* -------------------------------- 线程发布Topics信息 ------------------------------- */
        vTaskDelay(1);
    }
}
/* -------------------------------- 线程结束 ------------------------------- */

/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
///**
// * @brief chassis 线程中所有发布者初始化
// */
//static void chassis_pub_init(void)
//{
//    pub_chassis = pub_register("chassis_fdb",sizeof(struct chassis_fdb_msg));
//}
//
///**
// * @brief chassis 线程中所有订阅者初始化
// */
//static void chassis_sub_init(void)
//{
//    sub_cmd = sub_register("chassis_cmd", sizeof(struct chassis_cmd_msg));
//    sub_trans= sub_register("trans_fdb", sizeof(struct trans_fdb_msg));
//    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
//}
//
///**
// * @brief chassis 线程中所有发布者推送更新话题
// */
//static void chassis_pub_push(void)
//{
//    pub_push_msg(pub_chassis,&chassis_fdb);
//}
///**
// * @brief chassis 线程中所有订阅者获取更新话题
// */
//static void chassis_sub_pull(void)
//{
//    sub_get_msg(sub_cmd, &chassis_cmd);
//    sub_get_msg(sub_trans, &trans_fdb);
//    sub_get_msg(sub_ins, &ins_data);
//}
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */