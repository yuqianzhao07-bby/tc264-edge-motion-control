/*********************************************************************************************************************
* TC264 Opensourec Library (TC264 开源库) - 一个基于英飞凌 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件为双电机驱动核心功能实现文件
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

#include "my_motor.h"
#include "../debug_trace.h"

// 电机状态结构体实例
motor_state_t motor_state = {0, 0, 0, 0};

void my_motor_init(void)
{
    DEBUG_PRINTF("[MOTOR] 开始初始化双H桥电机驱动...\r\n");

    // 初始化四个PWM通道（双H桥驱动）
    // 注意: 使用不同ATOM模块，需要逐一初始化
    pwm_init(MY_MOTOR_L_PWM_A_CHANNEL, MY_MOTOR_PWM_FREQ, 0);
    DEBUG_PRINTF("[MOTOR] 左电机A (P11_9) 初始化完成\r\n");

    pwm_init(MY_MOTOR_L_PWM_B_CHANNEL, MY_MOTOR_PWM_FREQ, 0);
    DEBUG_PRINTF("[MOTOR] 左电机B (P11_10) 初始化完成\r\n");

    pwm_init(MY_MOTOR_R_PWM_A_CHANNEL, MY_MOTOR_PWM_FREQ, 0);
    DEBUG_PRINTF("[MOTOR] 右电机A (P11_11) 初始化完成\r\n");

    pwm_init(MY_MOTOR_R_PWM_B_CHANNEL, MY_MOTOR_PWM_FREQ, 0);
    DEBUG_PRINTF("[MOTOR] 右电机B (P11_12) 初始化完成\r\n");

    motor_state.enabled = 1;
    motor_state.direction = 0;

    DEBUG_PRINTF("[MOTOR] 初始化完成, 频率=%dHz, 占空比范围0-%d\r\n",
        MY_MOTOR_PWM_FREQ, MY_MOTOR_HIGH_DUTY);
}

// H桥电机速度设置函数
// direction=0(前进): A=duty, B=0
// direction=1(后退): A=0, B=duty
void my_motor_set_speed(uint32 left_duty, uint32 right_duty)
{
    // 限制占空比范围
    if(left_duty > MY_MOTOR_HIGH_DUTY) left_duty = MY_MOTOR_HIGH_DUTY;
    if(right_duty > MY_MOTOR_HIGH_DUTY) right_duty = MY_MOTOR_HIGH_DUTY;

    // 更新状态
    motor_state.left_duty = left_duty;
    motor_state.right_duty = right_duty;

    // 根据方向设置H桥PWM
    if(motor_state.direction == 0)  // 前进
    {
        // 先设置B通道为0，再设置A通道（避免短路）
        pwm_set_duty(MY_MOTOR_L_PWM_B_CHANNEL, 0);
        pwm_set_duty(MY_MOTOR_R_PWM_B_CHANNEL, 0);
        pwm_set_duty(MY_MOTOR_L_PWM_A_CHANNEL, left_duty);
        pwm_set_duty(MY_MOTOR_R_PWM_A_CHANNEL, right_duty);
    }
    else  // 后退
    {
        // 先设置A通道为0，再设置B通道（避免短路）
        pwm_set_duty(MY_MOTOR_L_PWM_A_CHANNEL, 0);
        pwm_set_duty(MY_MOTOR_R_PWM_A_CHANNEL, 0);
        pwm_set_duty(MY_MOTOR_L_PWM_B_CHANNEL, left_duty);
        pwm_set_duty(MY_MOTOR_R_PWM_B_CHANNEL, right_duty);
    }

    // 串口回传状态 (调试用, 正常运行时注释以消除UART阻塞)
    // my_motor_send_status();
}

void my_motor_start(void)
{
    my_motor_set_speed(MY_MOTOR_LOW_DUTY, MY_MOTOR_LOW_DUTY);
}

void my_motor_stop(void)
{
    // 停止所有四个PWM通道
    pwm_set_duty(MY_MOTOR_L_PWM_A_CHANNEL, 0);
    pwm_set_duty(MY_MOTOR_L_PWM_B_CHANNEL, 0);
    pwm_set_duty(MY_MOTOR_R_PWM_A_CHANNEL, 0);
    pwm_set_duty(MY_MOTOR_R_PWM_B_CHANNEL, 0);
    
    motor_state.left_duty = 0;
    motor_state.right_duty = 0;
    
    DEBUG_PRINTF("[MOTOR] 电机停止\r\n");
}

// 设置电机方向
// 在双H桥驱动中，方向通过切换两个PWM通道实现
void my_motor_set_direction(uint8 dir)
{
    uint32 current_left = motor_state.left_duty;
    uint32 current_right = motor_state.right_duty;
    
    motor_state.direction = dir;
    
    // 立即应用新方向（保持当前速度）
    if(dir == 0)  // 前进
    {
        pwm_set_duty(MY_MOTOR_L_PWM_A_CHANNEL, current_left);
        pwm_set_duty(MY_MOTOR_L_PWM_B_CHANNEL, 0);
        pwm_set_duty(MY_MOTOR_R_PWM_A_CHANNEL, current_right);
        pwm_set_duty(MY_MOTOR_R_PWM_B_CHANNEL, 0);
        DEBUG_PRINTF("[MOTOR] 方向: 前进\r\n");
    }
    else  // 后退
    {
        pwm_set_duty(MY_MOTOR_L_PWM_A_CHANNEL, 0);
        pwm_set_duty(MY_MOTOR_L_PWM_B_CHANNEL, current_left);
        pwm_set_duty(MY_MOTOR_R_PWM_A_CHANNEL, 0);
        pwm_set_duty(MY_MOTOR_R_PWM_B_CHANNEL, current_right);
        DEBUG_PRINTF("[MOTOR] 方向: 后退\r\n");
    }
}

// 串口回传电机状态 — 每5次调用打印一次
void my_motor_send_status(void)
{
    static uint32 cnt = 0;
    cnt++;
    if(cnt % 5 == 1)
    {
        DEBUG_PRINTF("[MOTOR] L=%lu(%d%%) R=%lu(%d%%) En=%u Dir=%s\r\n",
            motor_state.left_duty,
            (int)(motor_state.left_duty * 100 / MY_MOTOR_HIGH_DUTY),
            motor_state.right_duty,
            (int)(motor_state.right_duty * 100 / MY_MOTOR_HIGH_DUTY),
            motor_state.enabled,
            motor_state.direction == 0 ? "前进" : "后退");
    }
}

// 单独测试左侧电机
void my_motor_test_left(void)
{
    my_motor_stop();
    system_delay_ms(100);
    // H桥驱动：前进方向，A通道输出PWM，B通道接地
    pwm_set_duty(MY_MOTOR_L_PWM_A_CHANNEL, 1000);  // 10% 占空比
    pwm_set_duty(MY_MOTOR_L_PWM_B_CHANNEL, 0);
    motor_state.left_duty = 1000;
    DEBUG_PRINTF("[MOTOR] 测试左电机: PWM_A=1000(10%%) PWM_B=0\r\n");
}

// 单独测试右侧电机
void my_motor_test_right(void)
{
    my_motor_stop();
    system_delay_ms(100);
    // H桥驱动：前进方向，A通道输出PWM，B通道接地
    pwm_set_duty(MY_MOTOR_R_PWM_A_CHANNEL, 1000);  // 10% 占空比
    pwm_set_duty(MY_MOTOR_R_PWM_B_CHANNEL, 0);
    motor_state.right_duty = 1000;
    DEBUG_PRINTF("[MOTOR] 测试右电机: PWM_A=1000(10%%) PWM_B=0\r\n");
}
