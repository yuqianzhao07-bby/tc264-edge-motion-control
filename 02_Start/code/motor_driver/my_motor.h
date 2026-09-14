/*********************************************************************************************************************
* TC264 Opensourec Library (TC264 开源库) - 一个基于英飞凌 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件为双电机驱动核心功能头文件
*
* TC264 开源库 使用 GPL3.0 开源协议发布
* 您可以在遵守 GPL3.0 协议的前提下使用本库
you can use this library under the terms of the GPL3.0 protocol
*
* 本开源库的所有代码都经过逐飞科技的严格测试，但是不保证其绝对的稳定性和安全性
* 使用本库时，请自行承担风险
* 如您发现本库存在 bug 或有更好的建议，欢迎联系我们
*
* 公司网站: https://seekfree.taobao.com/
* 技术论坛: http://www.seekfree.net/
* 公司邮箱: support@seekfree.cn
********************************************************************************************************************/

#ifndef MY_MOTOR_H
#define MY_MOTOR_H

#include "zf_common_headfile.h"

// 根据引脚表：电机 PWM 11.9, 11.10, 11.11, 11.12
// 采用双H桥驱动方案，每个电机使用两个PWM通道控制正反转

// 左电机 PWM 通道 (H桥)
#define MY_MOTOR_L_PWM_A_CHANNEL    ATOM3_CH4_P11_9   // 左电机 PWM A
#define MY_MOTOR_L_PWM_B_CHANNEL    ATOM2_CH5_P11_10  // 左电机 PWM B

// 右电机 PWM 通道 (H桥)
#define MY_MOTOR_R_PWM_A_CHANNEL    ATOM3_CH6_P11_11  // 右电机 PWM A
#define MY_MOTOR_R_PWM_B_CHANNEL    ATOM2_CH7_P11_12  // 右电机 PWM B

#define MY_MOTOR_PWM_FREQ       10000
#define MY_MOTOR_LOW_DUTY       800
#define MY_MOTOR_HIGH_DUTY      10000

// 电机状态结构体
typedef struct {
    uint32 left_duty;           // 当前左轮占空比
    uint32 right_duty;          // 当前右轮占空比
    uint8  enabled;             // 电机使能状态
    uint8  direction;           // 方向 (0=前进, 1=后退)
} motor_state_t;

// 电机状态全局变量
extern motor_state_t motor_state;

void my_motor_init(void);
void my_motor_set_speed(uint32 left_duty, uint32 right_duty);
void my_motor_start(void);
void my_motor_stop(void);
void my_motor_test_left(void);
void my_motor_test_right(void);
void my_motor_set_direction(uint8 dir);
void my_motor_send_status(void);  // 串口回传电机状态

#endif
