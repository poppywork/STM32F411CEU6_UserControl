//
// Created by 刘嘉俊 on 25-4-21.
//

#include "robot.h"
#include "usart_task.h"
#include "drv_dwt.h"
#include "main.h"
#include "tim.h"

QueueHandle_t xQueue = NULL;
uint8_t buf[1]={0};
uint32_t count = 0;

#define QUEUE_LENGTH 10
#define QUEUE_ITEM_SIZE sizeof(float)*6

void robot_init(void)
{
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用 dwt 进行延时
//    __disable_irq();
    xQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
    if (xQueue == NULL) {
        Error_Handler();
    }

    dwt_init(); // 初始化 DWT 计时器

    HAL_TIM_Base_Start(&htim2);

    usart_rx_semaphore_init(); // 串口信号量初始化
    usart_tx_semaphore_init(); // 串口信号量初始化


    // 初始化完成,开启中断
//    __enable_irq();
}