#ifndef _motor_encoder_h_
#define _motor_encoder_h_

#include "zf_common_headfile.h"

// 编码器引脚定义 (根据硬件接线)
// 左编码器: TIM4 (P02_8 CH1, P00_9 CH2) - 方向计数模式
#define LEFT_ENCODER_CH1    TIM4_ENCODER_CH1_P02_8
#define LEFT_ENCODER_CH2    TIM4_ENCODER_CH2_P00_9
#define LEFT_ENCODER        TIM4_ENCODER

// 右编码器: TIM6 (P20_3 CH1, P20_0 CH2) - 方向计数模式
// 注意: 需要禁用UART3才能使用这两个引脚
#define RIGHT_ENCODER_CH1   TIM6_ENCODER_CH1_P20_3
#define RIGHT_ENCODER_CH2   TIM6_ENCODER_CH2_P20_0
#define RIGHT_ENCODER       TIM6_ENCODER

// 编码器参数 (根据实际硬件修改)
#define ENCODER_PPR         390     // 编码器每转脉冲数
#define REDUCTION_RATIO     30      // 减速比
#define WHEEL_DIAMETER      65      // 轮子直径mm
#define SAMPLE_TIME_MS      10      // 采样周期ms

// 编码器状态结构体
typedef struct {
    int32 left_count;       // 左编码器原始计数
    int32 right_count;      // 右编码器原始计数
    int32 left_speed;       // 左轮速度 (校准单位, count×50, 等效PWM占空比)
    int32 right_speed;      // 右轮速度 (校准单位, count×50)
    int32 left_distance;    // 左轮累计距离 (速度绝对值累加)
    int32 right_distance;   // 右轮累计距离 (速度绝对值累加)
    volatile uint8 updated; // ISR更新标志
} Encoder_State;

// 全局编码器状态
extern Encoder_State encoder_state;

// 调试: ISR调用计数
extern volatile uint32 g_encoder_isr_count;

// 调试: 原始编码器值
extern volatile int32 g_enc_left_raw;
extern volatile int32 g_enc_right_raw;

// 调试: TIM4原始定时器值 (未经/4处理)
extern volatile int32 g_tim4_raw_value;

// ISR中更新编码器 (由定时器中断调用)
void encoder_isr_update(void);

// 函数声明
void encoder_init(void);
void encoder_update(void);
int32 encoder_get_left_speed(void);
int32 encoder_get_right_speed(void);
void encoder_clear(void);

#endif
