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
#include <stdio.h>
#include <stdbool.h>
#include "algorithm_task.h"
#include "cmsis_os.h"
#include "KalmanFilterOne.h"
#include "drv_dwt.h"
#include "filter.h"
#include "arm_math.h"

#include "Hardware_i2c1.h"

#include "iic1.h"
#include "iic2.h"
#include "iic3.h"
#include "iic4.h"
#include <stdlib.h>
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
static uint32_t algorithm_task_dwt = 0;   // 毫秒监测
static float algorithm_task_dt = 0;       // 线程实际运行时间dt
static float algorithm_task_delta = 0;    // 监测线程运行时间
static float algorithm_task_start_dt = 0; // 监测线程开始时间
/* -------------------------------- 调试监测线程相关 --------------------------------- */

static MT6701_Encoder_t mt6701_encoder_1 = {0};
static MT6701_Encoder_t mt6701_encoder_2 = {0};
static MT6701_Encoder_t mt6701_encoder_3 = {0};
static MT6701_Encoder_t mt6701_encoder_4 = {0};
static MT6701_Encoder_t mt6701_encoder_5 = {0};
static MT6701_Encoder_t mt6701_encoder_6 = {0};

float angles_encoder[6] = {0};

extern QueueHandle_t xQueue;      // 队列句柄

/* -------------------------------- 线程入口 ------------------------------- */
void AlgorithmTask_Entry(void const * argument)
{
/* -------------------------------- 外设初始化段落 ------------------------------- */
    // 先读取十次原始值在进行初始化和0点校准
    for(uint8_t i = 0; i < 10; i++){
        get_raw_angle_1();
        get_raw_angle_2();
        get_raw_angle_3();
        get_raw_angle_4();
        get_raw_angle_5();
        get_raw_angle_6();
        HAL_Delay(10); // 延时1s等待MT6701初始化完成
    }

    // 初始化MT6701编码器并标定0点
    MT6701_Init_1(&mt6701_encoder_1);
    HAL_Delay(10); // 延时1s等待MT6701初始化完成
    MT6701_SetZero_1(&mt6701_encoder_1

    );
    HAL_Delay(10); // 延时1s等待MT6701初始化完成

    MT6701_Init_2(&mt6701_encoder_2);
    HAL_Delay(10); // 延时1s等待MT6701初始化完成
    MT6701_SetZero_2(&mt6701_encoder_2);
    HAL_Delay(10); // 延时1s等待MT6701初始化完成

    MT6701_Init_3(&mt6701_encoder_3);
    HAL_Delay(10); // 延时1s等待MT6701初始化完成
    MT6701_SetZero_3(&mt6701_encoder_3);
    HAL_Delay(10); // 延时1s等待MT6701初始化完成


    MT6701_Init_4(&mt6701_encoder_4);
    HAL_Delay(10); // 延时1s等待MT6701初始化完成
    MT6701_SetZero_4(&mt6701_encoder_4);
    HAL_Delay(10); // 延时1s等待MT6701初始化完成

    MT6701_Init_5(&mt6701_encoder_5);
    HAL_Delay(10); // 延时1s等待MT6701初始化完成
    MT6701_SetZero_5(&mt6701_encoder_5);
    HAL_Delay(10); // 延时1s等待MT6701初始化完成

    MT6701_Init_6(&mt6701_encoder_6);
    HAL_Delay(10); // 延时1s等待MT6701初始化完成
    MT6701_SetZero_6(&mt6701_encoder_6);
    HAL_Delay(10); // 延时1s等待MT6701初始化完成


/* -------------------------------- 外设初始化段落 ------------------------------- */

/* -------------------------------- 线程间Topics初始化 ------------------------------- */
//    chassis_pub_init();
//    chassis_sub_init();
/* -------------------------------- 线程间Topics初始化 ------------------------------- */
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    algorithm_task_dt = dwt_get_delta(&algorithm_task_dwt);
    algorithm_task_start_dt = dwt_get_time_ms();
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    for(;;)
    {
/* -------------------------------- 调试监测线程调度 --------------------------------- */
        algorithm_task_delta = dwt_get_time_ms() - algorithm_task_start_dt;
        algorithm_task_start_dt = dwt_get_time_ms();
        algorithm_task_dt = dwt_get_delta(&algorithm_task_dwt);
/* -------------------------------- 调试监测线程调度 --------------------------------- */
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */
//        chassis_sub_pull();
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */

/* -------------------------------- 线程代码编写段落 ------------------------------- */

        MT6701_Update_1(&mt6701_encoder_1);
        MT6701_Update_2(&mt6701_encoder_2);
        MT6701_Update_3(&mt6701_encoder_3);
        MT6701_Update_4(&mt6701_encoder_4);
        MT6701_Update_5(&mt6701_encoder_5);
        MT6701_Update_6(&mt6701_encoder_6);


        angles_encoder[0] = -mt6701_encoder_1.total_angle_deg;//与下位机方向电机转动方向相反
        angles_encoder[1] = mt6701_encoder_2.total_angle_deg;
        angles_encoder[2] = mt6701_encoder_3.total_angle_deg;
        angles_encoder[3] = mt6701_encoder_4.total_angle_deg;
        angles_encoder[4] = -mt6701_encoder_5.total_angle_deg;
        angles_encoder[5] = -mt6701_encoder_6.total_angle_deg;

        //判断当前状态是否稳定   50大约是1°
        if (abs(mt6701_encoder_1.diff)<50 &&
            abs(mt6701_encoder_2.diff)<50 &&
            abs(mt6701_encoder_3.diff)<50 &&
            abs(mt6701_encoder_4.diff)<50 &&
            abs(mt6701_encoder_5.diff)<50 &&
            abs(mt6701_encoder_6.diff)<50)
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
        }
        // 将编码器数据放入队列
        xQueueSend(xQueue, angles_encoder, 0);

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