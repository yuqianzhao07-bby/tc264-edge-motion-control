#include "motor_encoder.h"
#include "../debug_trace.h"
#include "IfxGpt12_IncrEnc.h"

Encoder_State encoder_state = {0};

// ISR中使用的临时计数
static volatile int32 isr_left_count = 0;
static volatile int32 isr_right_count = 0;

// 新数据标志 (ISR设置, encoder_update清除)
static volatile uint8 encoder_data_ready = 0;

// 调试: ISR调用计数
volatile uint32 g_encoder_isr_count = 0;

// 调试: 原始编码器值 (ISR增量)
volatile int32 g_enc_left_raw = 0;
volatile int32 g_enc_right_raw = 0;

// 调试: TIM4原始定时器值 (未经/4处理, 用于诊断左编码器)
volatile int32 g_tim4_raw_value = 0;

void encoder_init(void)
{
    DEBUG_PRINTF("[ENCODER] 初始化编码器...\r\n");

    // 左编码器 (TIM4: P02_8/P00_9) - 使用方向计数模式
    // TIM4正交模式实测不工作(T4RAW=0/65535), 改用方向计数模式
    encoder_dir_init(LEFT_ENCODER, LEFT_ENCODER_CH1, LEFT_ENCODER_CH2);
    DEBUG_PRINTF("[ENCODER] 左编码器: TIM4 P02_8/P00_9 (方向计数模式)\r\n");

    // 右编码器 (TIM6: P20_3/P20_0) - 方向计数模式
    encoder_dir_init(RIGHT_ENCODER, RIGHT_ENCODER_CH1, RIGHT_ENCODER_CH2);
    DEBUG_PRINTF("[ENCODER] 右编码器: TIM6 P20_3/P20_0 (方向计数模式)\r\n");

    encoder_clear();
    DEBUG_PRINTF("[ENCODER] 初始化完成\r\n");
}

// ISR中调用: 读取编码器增量并清零定时器 (10ms周期)
void encoder_isr_update(void)
{
    g_encoder_isr_count++;

    // 调试: 直接读取TIM4原始值 (在清零之前, 不经过/4处理)
    g_tim4_raw_value = (int32)IfxGpt12_T4_getTimerValue(&MODULE_GPT120);

    // 读取定时器当前值 (获取自上次清零以来的增量)
    int16 left_raw = encoder_get_count(LEFT_ENCODER);
    int16 right_raw = encoder_get_count(RIGHT_ENCODER);

    // 立即清零定时器, 确保下次读取的是新增量
    encoder_clear_count(LEFT_ENCODER);
    encoder_clear_count(RIGHT_ENCODER);

    // 左编码器方向取反 (硬件安装方向与预期相反)
    left_raw = -left_raw;

    // 累加增量
    isr_left_count += left_raw;
    isr_right_count += right_raw;

    // 调试: 显示每次ISR的增量值 (left_raw已/4)
    g_enc_left_raw = left_raw;
    g_enc_right_raw = right_raw;

    // 标记有新数据可供主循环读取
    encoder_data_ready = 1;
}

// 主循环调用: 仅在ISR有新数据时更新速度
void encoder_update(void)
{
    // 没有新数据则直接返回, 避免频繁调用时读到0
    if (!encoder_data_ready) return;

    // 原子读取ISR累加值并清除标志
    uint32 int_state = interrupt_global_disable();
    int32 left_total = isr_left_count;
    int32 right_total = isr_right_count;
    isr_left_count = 0;
    isr_right_count = 0;
    encoder_data_ready = 0;
    interrupt_global_enable(int_state);

    encoder_state.left_count = left_total;
    encoder_state.right_count = right_total;

    // 速度计算: 将10ms脉冲数映射到PWM占空比等效单位
    // 实测: 约550脉冲/10ms对应target_speed≈600
    // 校准: ×11/10 (即×1.1) → 550→605, 与目标速度同量级
    // 原始×50导致27800+ (46倍超标), 速度环完全失效
    encoder_state.left_speed = left_total * 11 / 10;
    encoder_state.right_speed = right_total * 11 / 10;

    // 累计距离
    encoder_state.left_distance += (encoder_state.left_speed > 0) ? encoder_state.left_speed : -encoder_state.left_speed;
    encoder_state.right_distance += (encoder_state.right_speed > 0) ? encoder_state.right_speed : -encoder_state.right_speed;

    encoder_state.updated = 1;
}

int32 encoder_get_left_speed(void)
{
    return encoder_state.left_speed;
}

int32 encoder_get_right_speed(void)
{
    return encoder_state.right_speed;
}

void encoder_clear(void)
{
    encoder_clear_count(LEFT_ENCODER);
    encoder_clear_count(RIGHT_ENCODER);
    isr_left_count = 0;
    isr_right_count = 0;
    encoder_data_ready = 0;
    encoder_state.left_count = 0;
    encoder_state.right_count = 0;
    encoder_state.left_speed = 0;
    encoder_state.right_speed = 0;
    encoder_state.left_distance = 0;
    encoder_state.right_distance = 0;
    encoder_state.updated = 0;
}
