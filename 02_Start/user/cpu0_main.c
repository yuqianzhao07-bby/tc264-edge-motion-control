/*********************************************************************************************************************
* TC264 Opensourec Library (TC264 开源库) - 一个基于英飞凌 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "mt9v03x_tft180_display.h"
#include "zf_device_tft180.h"
#include "image.h"
#include "track_way.h"
#include "motor_driver/my_motor.h"
#include "motor_driver/motor_encoder.h"
#include "pid_controller.h"
//#include "imu_driver/imu_driver.h"  // 暂时注释，不使用IMU
#include "wifi_image_transfer.h"
#include "component_recognition.h"  // 元器件识别模块
#include "../code/debug_trace.h"
#pragma section all "cpu0_dsram"

// JUNCTION_NAV_ENABLE 定义在 track_way.h 中, 统一管理

// 电机控制参数 (3S电池, 11.1V, 实测标定)
// 注意: DUTY_MIN需≤最低运行速度, DUTY_MAX需≥最高差速需求, 修改速度时同步检查
// 占空比 = 数值/10000 (MY_MOTOR_HIGH_DUTY=10000)
#define DUTY_MIN          100     // 最低占空比 (允许极低速)
#define DUTY_MAX          1200    // 最高占空比
#define START_SPEED       400     // 起步速度
#define BASE_SPEED        450     // 直道基准速度
#define CURVE_SPEED       420     // 弯道速度
#define RIGHT_ANGLE_SPEED 400     // 直角弯速度
#define COMPONENT_SPEED   450     // 元器件区域速度
#define START_DELAY_MS    50      // 起步延时

// 前瞄减速参数 - 重负载需要更早减速
#define LOOKAHEAD_START   15      // 更早开始前瞄(从第15行开始)
#define LOOKAHEAD_ROWS    20      // 多看几行
#define CURVE_AHEAD_THRESHOLD 8   // 更低阈值, 更早触发减速

// PID控制器实例
static PID_Controller steering_pid;   // 位置环: 横向偏差 → 转向
static PID_Controller angle_pid;      // 角度环: 航向偏差 → 转向修正
static PID_Controller speed_pid;      // 速度环: 车速偏差 → 油门修正

// ===== 在线调参: 运行时PID参数 (初始值来自pid_controller.h宏定义) =====
// 逐飞助手在线调参功能可实时修改这些变量, 无需重新编译烧录
// 通道映射 (与PID调参记录表1.3节一致):
//   ch0: 直道转向Kp    ch1: 直道转向Kd
//   ch2: 弯道转向Kp    ch3: 弯道转向Kd  (弯道=钝角弯+直角弯共用)
//   ch4: 速度环Kp      ch5: 速度环Ki
//   ch6: 角度环Kp      ch7: 角度环Kd
static int32 g_steer_kp        = STEER_KP;              // ch0
static int32 g_steer_kd        = STEER_KD;              // ch1
static int32 g_steer_kp_curve  = STEER_KP_CURVE;        // ch2 (钝角弯+直角弯共用)
static int32 g_steer_kd_curve  = STEER_KD_CURVE;        // ch3
static int32 g_angle_kp        = ANGLE_KP;              // ch6
static int32 g_angle_kd        = ANGLE_KD;              // ch7
static int32 g_angle_kp_curve  = ANGLE_KP_CURVE;        // 弯道角度Kp (ch6 × 比例)
static int32 g_angle_kd_curve  = ANGLE_KD_CURVE;        // 弯道角度Kd (ch7 × 比例)
static int32 g_speed_kp        = SPEED_KP;              // ch4
static int32 g_speed_ki        = SPEED_KI;              // ch5

// 速度状态 (开环控制: 占空比直接等于target_speed, 速度PID禁用)
// 原因: 编码器校准值与占空比量纲不匹配, 速度环导致异常
static int32 target_speed = BASE_SPEED;
static uint8 use_speed_pid = 0;  // 0=开环占空比控制, 1=速度闭环PI

// === 前瞄减速: 检测近处是否有弯道 ===
// 看图像中部(近处), 不是顶部(远处), 避免提前转弯
// 返回: 需要减去的速度值 (0=不减)
static int32 get_lookahead_speed_reduction(void)
{
    // 计算前瞄区域(图像中部LOOKAHEAD_START开始的LOOKAHEAD_ROWS行)的中线偏移
    int16 offset_sum = 0;
    int16 count = 0;
    for (int16 i = LOOKAHEAD_START; i < LOOKAHEAD_START + LOOKAHEAD_ROWS && i < image_h; i++)
    {
        offset_sum += center_line[i] - image_w / 2;
        count++;
    }
    if (count == 0) return 0;
    
    int16 avg_offset = offset_sum / count;
    int16 abs_offset = (avg_offset >= 0) ? avg_offset : -avg_offset;
    
    // 温和阶梯式减速: 只在确实近处有弯道时才减
    if (abs_offset > 25)      return 100;   // 大弯道近了: 减100
    else if (abs_offset > 18) return 50;    // 中弯道近了: 减50
    
    return 0;
}

// === 获取目标速度 ===
static int32 get_target_speed(uint8 element_type, uint8 is_continuous)
{
    int32 base_target;
    
    switch(element_type)
    {
        case ELEMENT_RIGHT_ANGLE: base_target = RIGHT_ANGLE_SPEED; break;
        case ELEMENT_T_JUNCTION: base_target = RIGHT_ANGLE_SPEED; break;  // T形路口降速
        case ELEMENT_OBTUSE:
            if (is_continuous)
                base_target = CURVE_SPEED * 9 / 10;
            else
                base_target = CURVE_SPEED;
            break;
        case ELEMENT_CROSS:       base_target = CURVE_SPEED; break;
        case ELEMENT_BREAK:       base_target = RIGHT_ANGLE_SPEED * 2 / 3; break;  // 断路: 快速减速到400, 避免冲出赛道
        default:                  base_target = BASE_SPEED; break;
    }
    
// 元器件识别到时降速
    if (g_confirmed_valid && g_confirmed_result.type != COMPONENT_NONE)
    {
        base_target = COMPONENT_SPEED;
    }
    
    // 前瞄减速: 远处有弯道迹象时提前减去一定速度
    base_target -= get_lookahead_speed_reduction();
    
    return base_target;
}

// === 逐飞助手在线调参: 检查PC端是否有参数更新 ===========
// 通道映射 (与PID调参记录表1.3节一致):
//   ch0: 直道转向Kp  ch1: 直道转向Kd  ch2: 弯道转向Kp  ch3: 弯道转向Kd
//   ch4: 速度环Kp    ch5: 速度环Ki    ch6: 角度环Kp    ch7: 角度环Kd
// 弯道参数同时用于钝角弯和直角弯 (简化调参)
static void handle_online_tuning(void)
{
    uint8 i;
    for (i = 0; i < 8; i++)
    {
        if (seekfree_assistant_parameter_update_flag[i])
        {
            int32 val = (int32)seekfree_assistant_parameter[i];
            switch (i)
            {
                case 0: g_steer_kp       = val; break;  // 直道转向Kp
                case 1: g_steer_kd       = val; break;  // 直道转向Kd
                case 2: g_steer_kp_curve = val; break;  // 弯道转向Kp (钝角弯+直角弯)
                case 3: g_steer_kd_curve = val; break;  // 弯道转向Kd
                case 4: g_speed_kp       = val; break;  // 速度环Kp
                case 5: g_speed_ki       = val; break;  // 速度环Ki
                case 6: g_angle_kp       = val; break;  // 角度环Kp
                case 7: g_angle_kd       = val; break;  // 角度环Kd
            }
            seekfree_assistant_parameter_update_flag[i] = 0;
        }
    }
}

// === 根据元素类型和回正状态调整双层转向PID参数 ===
// 位置环 + 角度环 同步调整, 保持协调
// 使用运行时变量 g_steer_kp 等, 支持逐飞助手在线调参实时修改
// 元素切换时使用3帧斜坡过渡, 避免参数跳变导致转向冲击
static void adjust_steering_pid(uint8 element_type, AlignState align_state, uint8 is_continuous)
{
    int32 pos_kp, pos_kd, ang_kp, ang_kd;
    static uint8 last_elem = ELEMENT_NONE;
    static uint8 ramp_count = 0;
    static int32 prev_pos_kp = 0, prev_pos_kd = 0, prev_ang_kp = 0, prev_ang_kd = 0;

    // 元素类型变化时启动3帧斜坡
    if (element_type != last_elem)
    {
        ramp_count = 3;
        // 保存当前的旧目标值(即上一帧已经应用的值)
        prev_pos_kp = (steering_pid.Kp > 0) ? steering_pid.Kp : g_steer_kp;
        prev_pos_kd = (steering_pid.Kd > 0) ? steering_pid.Kd : g_steer_kd;
        prev_ang_kp = (angle_pid.Kp > 0) ? angle_pid.Kp : g_angle_kp;
        prev_ang_kd = (angle_pid.Kd > 0) ? angle_pid.Kd : g_angle_kd;
        last_elem = element_type;
    }

    switch(element_type)
    {
        case ELEMENT_RIGHT_ANGLE:
            // 直角弯: 使用专用激进参数 (Kp=12, Kd=14)，确保90度转弯
            pos_kp = STEER_KP_RIGHT_ANGLE; pos_kd = STEER_KD_RIGHT_ANGLE;
            ang_kp = ANGLE_KP_RIGHT_ANGLE; ang_kd = ANGLE_KD_RIGHT_ANGLE;
            break;
        case ELEMENT_T_JUNCTION:
            // T形路口: 弯道参数
            pos_kp = g_steer_kp_curve; pos_kd = g_steer_kd_curve;
            ang_kp = g_angle_kp_curve; ang_kd = g_angle_kd_curve;
            break;
        case ELEMENT_OBTUSE:
            if (align_state == ALIGN_FORCE)
            {
                pos_kp = (int32)(g_steer_kp_curve * 1.2); pos_kd = g_steer_kd_curve;
                ang_kp = (int32)(g_angle_kp_curve * 1.2); ang_kd = g_angle_kd_curve;
            }
            else if (align_state == ALIGN_CURVE_EXIT)
            {
                pos_kp = g_steer_kp_curve; pos_kd = g_steer_kd_curve;
                ang_kp = g_angle_kp_curve; ang_kd = g_angle_kd_curve;
            }
            else if (is_continuous)
            {
                pos_kp = (int32)(g_steer_kp_curve * 1.1); pos_kd = g_steer_kd_curve;
                ang_kp = (int32)(g_angle_kp_curve * 1.1); ang_kd = g_angle_kd_curve;
            }
            else
            {
                pos_kp = (g_steer_kp + g_steer_kp_curve) / 2;
                pos_kd = (g_steer_kd + g_steer_kd_curve) / 2;
                ang_kp = (g_angle_kp + g_angle_kp_curve) / 2;
                ang_kd = (g_angle_kd + g_angle_kd_curve) / 2;
            }
            break;
        case ELEMENT_CROSS:
            // 十字: 直道参数直接通过
            pos_kp = g_steer_kp; pos_kd = g_steer_kd;
            ang_kp = g_angle_kp; ang_kd = g_angle_kd;
            break;
        case ELEMENT_BREAK:
            // 断路: 减半位置Kp, 角度环也减弱, 保守维持方向
            pos_kp = g_steer_kp / 2; pos_kd = g_steer_kd;
            ang_kp = g_angle_kp / 2;  ang_kd = g_angle_kd;
            break;
        default:  // 直道
            if (align_state == ALIGN_FORCE)
            {
                pos_kp = (int32)(g_steer_kp * 1.2); pos_kd = g_steer_kd;
                ang_kp = (int32)(g_angle_kp * 1.2);  ang_kd = g_angle_kd;
            }
            else if (align_state == ALIGN_CURVE_EXIT)
            {
                pos_kp = (int32)(g_steer_kp * 1.1); pos_kd = g_steer_kd;
                ang_kp = (int32)(g_angle_kp * 1.1);  ang_kd = g_angle_kd;
            }
            else
            {
                pos_kp = g_steer_kp; pos_kd = g_steer_kd;
                ang_kp = g_angle_kp; ang_kd = g_angle_kd;
            }
            break;
    }

    // 斜坡过渡: 如果正在过渡中, 线性插值平滑参数跳变
    if (ramp_count > 0)
    {
        ramp_count--;
        // 线性插值: prev + (target - prev) * (3-ramp_count)/3
        int32 frac = 3 - ramp_count;  // 1/3, 2/3, 3/3
        pos_kp = prev_pos_kp + (pos_kp - prev_pos_kp) * frac / 3;
        pos_kd = prev_pos_kd + (pos_kd - prev_pos_kd) * frac / 3;
        ang_kp = prev_ang_kp + (ang_kp - prev_ang_kp) * frac / 3;
        ang_kd = prev_ang_kd + (ang_kd - prev_ang_kd) * frac / 3;
    }

    pid_update_params(&steering_pid, pos_kp, 0, pos_kd);
    pid_update_params(&angle_pid, ang_kp, 0, ang_kd);
}


// **************************** 代码区域 ****************************
int core0_main(void)
{
    clock_init();
    debug_init();

    DEBUG_PRINTF("\r\n========== CPU0 启动 (WiFi SPI图传版) ==========\r\n");

    // 初始化屏幕 (横屏160px + 6x8字体 = 每行最多26字符, IP/版本/MAC不会溢出)
    tft180_set_dir(TFT180_CROSSWISE_180);
    tft180_set_font(TFT180_6X8_FONT);
    tft180_init();
    tft180_clear();
    tft180_show_string(0, 0,  "WiFi SPI v2");
    tft180_show_string(0, 16, "REDMI 10.206.18.220");

    // 初始化 WiFi SPI 图传 (所有配置在 wifi_image_transfer.h 中)
    if(wifi_image_transfer_init())
    {
        DEBUG_PRINTF("[FAIL] WiFi SPI图传初始化失败!\r\n");
        tft180_show_string(0, 48, "WIFI INIT FAIL!");
        tft180_show_string(0, 64, "Check wiring");
        while(1);
    }

    DEBUG_PRINTF("[INIT] WiFi SPI图传初始化成功\r\n");
    tft180_clear();
    tft180_show_string(0, 0,  "WiFi Transfer OK");
    tft180_show_string(0, 16, wifi_image_transfer_get_version());
    tft180_show_string(0, 32, wifi_image_transfer_get_ip());
    tft180_show_string(0, 48, wifi_image_transfer_get_mac());

    // === IMU已禁用 ===
    // IMU相关功能已注释，使用纯摄像头巡线模式
    DEBUG_PRINTF("[INIT] IMU已禁用，使用纯摄像头模式\r\n");

    // 初始化摄像头
    DEBUG_PRINTF("[INIT] 初始化摄像头...\r\n");
    uint8 init_result = mt9v03x_tft180_init();
    if(init_result != 0)
    {
        DEBUG_PRINTF("[FAIL] 摄像头初始化失败! code=%d\r\n", init_result);
        // 不清除屏幕, 保留WiFi信息 + 错误提示
        tft180_show_string(0, 64, "Camera FAIL!");
        tft180_show_string(0, 80, "Check MT9V03x");
        while(1);
    }
    DEBUG_PRINTF("[INIT] 摄像头初始化成功\r\n");
    // 注意: 图像参数已在 wifi_image_transfer_init() 中配置完成

    // 配置边界线显示 (在图传中显示巡线边界)
    #if WIFI_TRANSFER_BOUNDARY_TYPE != 0
    {
        // 使用 track_way 模块的边界线数据
        // left_border, right_border, center_line 数组在 track_way.c 中定义
        extern int16 left_border[image_h];
        extern int16 right_border[image_h];
        extern int16 center_line[image_h];

        // 创建uint8类型的边界线数组 (图传协议要求uint8类型)
        static uint8 x1_boundary[image_h], x2_boundary[image_h], x3_boundary[image_h];
        static uint8 y1_boundary[image_w], y2_boundary[image_w], y3_boundary[image_w];

        // 将int16数据转换为uint8数据 (范围检查: 0-255)
        for(int i = 0; i < image_h; i++)
        {
            int16 lx = left_border[i];
            int16 cx = center_line[i];
            int16 rx = right_border[i];
            x1_boundary[i] = (lx < 0) ? 0 : (lx > 255) ? 255 : (uint8)lx;
            x2_boundary[i] = (cx < 0) ? 0 : (cx > 255) ? 255 : (uint8)cx;
            x3_boundary[i] = (rx < 0) ? 0 : (rx > 255) ? 255 : (uint8)rx;
        }

        #if WIFI_TRANSFER_BOUNDARY_TYPE == 1
            // X边界模式: 只有横坐标，纵坐标根据图像高度得到
            wifi_image_transfer_set_boundary(X_BOUNDARY, image_h,
                                            x1_boundary, x2_boundary, x3_boundary,
                                            NULL, NULL, NULL);
            DEBUG_PRINTF("[INIT] 边界线配置: X_BOUNDARY模式, %d行\r\n", image_h);
        #elif WIFI_TRANSFER_BOUNDARY_TYPE == 2
            // Y边界模式: 只有纵坐标，横坐标根据图像宽度得到
            // 需要将边界线数据转换为Y边界格式
            for(int i = 0; i < image_w; i++)
            {
                y1_boundary[i] = (uint8)(left_border[i * image_h / image_w] & 0xFF);
                y2_boundary[i] = (uint8)(center_line[i * image_h / image_w] & 0xFF);
                y3_boundary[i] = (uint8)(right_border[i * image_h / image_w] & 0xFF);
            }
            wifi_image_transfer_set_boundary(Y_BOUNDARY, image_w,
                                            NULL, NULL, NULL,
                                            y1_boundary, y2_boundary, y3_boundary);
            DEBUG_PRINTF("[INIT] 边界线配置: Y_BOUNDARY模式, %d列\r\n", image_w);
        #elif WIFI_TRANSFER_BOUNDARY_TYPE == 3
            // XY边界模式: 完整坐标，可以显示任意位置的点
            // 需要将int16数组转换为uint8数组（图像坐标范围0-255）
            static uint8 xy_x_boundary[image_h * 3];
            static uint8 xy_y_boundary[image_h * 3];
            // 填充XY坐标数据
            for(int i = 0; i < image_h; i++)
            {
                xy_x_boundary[i * 3] = x1_boundary[i];
                xy_y_boundary[i * 3] = (uint8)(i & 0xFF);
                xy_x_boundary[i * 3 + 1] = x2_boundary[i];
                xy_y_boundary[i * 3 + 1] = (uint8)(i & 0xFF);
                xy_x_boundary[i * 3 + 2] = x3_boundary[i];
                xy_y_boundary[i * 3 + 2] = (uint8)(i & 0xFF);
            }
            wifi_image_transfer_set_boundary(XY_BOUNDARY, image_h * 3,
                                            xy_x_boundary, xy_x_boundary + image_h, xy_x_boundary + image_h * 2,
                                            xy_y_boundary, xy_y_boundary + image_h, xy_y_boundary + image_h * 2);
            DEBUG_PRINTF("[INIT] 边界线配置: XY_BOUNDARY模式, %d点\r\n", image_h * 3);
        #elif WIFI_TRANSFER_BOUNDARY_TYPE == 4
            // 无图像+边界线模式: 只发送边界线，不发送图像数据
            wifi_image_transfer_set_boundary(X_BOUNDARY, image_h,
                                            x1_boundary, x2_boundary, x3_boundary,
                                            NULL, NULL, NULL);
            DEBUG_PRINTF("[INIT] 边界线配置: 无图像+边界线模式, %d行\r\n", image_h);
        #endif
    }
    #endif

    // 初始化电机
    my_motor_init();
    DEBUG_PRINTF("[INIT] 双电机驱动初始化完成\r\n");

    // 初始化编码器
    encoder_init();
    DEBUG_PRINTF("[INIT] 编码器初始化完成\r\n");

    // 初始化PIT定时器 (10ms周期, 用于编码器采样)
    pit_ms_init(CCU60_CH1, 10);
    DEBUG_PRINTF("[INIT] PIT定时器初始化完成 (10ms编码器采样)\r\n");

    // 初始化转向PID (位置式PD, 位置环: 横向偏差)
    pid_init(&steering_pid, STEER_KP, STEER_KI, STEER_KD);
    pid_set_mode(&steering_pid, PID_MODE_POSITION);  // 位置式
    pid_set_limits(&steering_pid, STEER_OUT_MIN, STEER_OUT_MAX);
    pid_set_error_thres(&steering_pid, STEER_ERROR_THRES);
    DEBUG_PRINTF("[INIT] 位置PID: 位置式PD Kp=%d Ki=%d Kd=%d\r\n",
        STEER_KP, STEER_KI, STEER_KD);

    // 初始化角度PID (位置式PD, 角度环: 航向偏差)
    pid_init(&angle_pid, ANGLE_KP, 0, ANGLE_KD);
    pid_set_mode(&angle_pid, PID_MODE_POSITION);  // 位置式
    pid_set_limits(&angle_pid, ANGLE_OUT_MIN, ANGLE_OUT_MAX);
    pid_set_error_thres(&angle_pid, ANGLE_ERROR_THRES);
    DEBUG_PRINTF("[INIT] 角度PID: 位置式PD Kp=%d Ki=0 Kd=%d\r\n",
        ANGLE_KP, ANGLE_KD);

    // 初始化速度PID (增量式PI, 平稳)
    pid_init(&speed_pid, g_speed_kp, g_speed_ki, SPEED_KD);
    pid_set_mode(&speed_pid, PID_MODE_INCREMENT);  // 增量式
    pid_set_limits(&speed_pid, SPEED_OUT_MIN, SPEED_OUT_MAX);
    pid_set_error_thres(&speed_pid, SPEED_ERROR_THRES);
    DEBUG_PRINTF("[INIT] 速度PID: 增量式PI Kp=%d Ki=%d Kd=%d\r\n",
        (int)g_speed_kp, (int)g_speed_ki, SPEED_KD);

// 初始化元器件识别模块
    component_recognition_init();
    DEBUG_PRINTF("[INIT] 元器件识别模块初始化完成\r\n");

#if JUNCTION_NAV_ENABLE
    // 初始化路口遍历模块 (Layer 3: 右手法则)
    // ✅ 回溯已实现: JSTATE_RETURNING → 掉头→反向行驶→距离匹配→到达目标路口
    junction_init();
    DEBUG_PRINTF("[INIT] 路口遍历模块初始化完成 (右手法则: 右>直>左)\r\n");
#endif

    cpu_wait_event_ready();
    interrupt_global_enable(0);
    DEBUG_PRINTF("[INIT] 全局中断已使能\r\n");

    // 起步: 线性斜坡从 START_SPEED → BASE_SPEED (避免突变冲击)
    DEBUG_PRINTF("[INIT] 起步: 斜坡加速 %d→%d (100ms)\r\n", START_SPEED, BASE_SPEED);
    {
        int32 ramp_steps = 10;          // 分10步
        int32 ramp_step_ms = 10;        // 每步10ms, 共100ms
        int32 speed_range = START_SPEED - BASE_SPEED;
        for (int32 s = 0; s <= ramp_steps; s++)
        {
            int32 cur_speed = START_SPEED - (speed_range * s / ramp_steps);
            my_motor_set_speed((uint32)cur_speed, (uint32)cur_speed);
            system_delay_ms(ramp_step_ms);
        }
    }

    // 切换到运行速度
    my_motor_set_speed(BASE_SPEED, BASE_SPEED);
    target_speed = BASE_SPEED;
    DEBUG_PRINTF("[INIT] 进入主循环\r\n");
    DEBUG_PRINTF("========================================\r\n");

    uint32 no_frame_timeout = 0;

    while (TRUE)
    {
        g_main_loop_count++;

        // 每帧都更新编码器
        encoder_update();
#if JUNCTION_NAV_ENABLE
        // 路口距离累计 (回溯时需要距离指纹匹配)
        junction_update_distance(encoder_get_left_speed(), encoder_get_right_speed());
#endif

        // 逐飞助手在线调参: 接收PC端参数更新 + 应用
        seekfree_assistant_data_analysis();
        handle_online_tuning();

        if(mt9v03x_tft180_check_finish())
        {
            g_frame_process_count++;

            if(no_frame_timeout > 1000)
            {
                DEBUG_PRINTF("[RECOVER] 恢复采集, 帧号=%lu\r\n", g_frame_process_count);
            }
            no_frame_timeout = 0;

            // 图像压缩 + 二值化 (关中段)
            {
                uint32 int_state = interrupt_global_disable();
                compressimage();
                interrupt_global_enable(int_state);
            }
            {
                uint32 int_state = interrupt_global_disable();
                turn_to_bin();
                interrupt_global_enable(int_state);
            }

            // 首帧诊断
            if(g_frame_process_count == 1)
            {
                DEBUG_PRINTF("[DIAG] 阈值=%d 尺寸=%dx%d\r\n",
                    image_thereshold, image_w, image_h);
            }

            // 滤波
            median_filter(bin_image);
            image_filter(bin_image);

            // 巡线 + 元素检测 + 补线
            track_find_line();
            track_get_error();

#if JUNCTION_NAV_ENABLE  // 岔路口OFS状态机 (回溯未实现, 暂禁用)
            // ==========================================
            // Layer 3: 路口遍历状态机 (右手法则)
            // 四阶段: 接近检测 → 注册 → 决策 → 转弯 → 回溯
            // ==========================================
            {
                // 阶段0: 检测是否接近路口
                if (g_junction_state == JSTATE_IDLE)
                {
                    if (junction_detect_approach())
                    {
                        g_junction_state = JSTATE_APPROACH;
                    }
                }

                // 阶段1: 注册路口 (带超时保护, 防检测抖动导致卡死)
                if (g_junction_state == JSTATE_APPROACH)
                {
                    static uint8 approach_timeout = 0;
                    // 只注册 T_JUNCTION/CROSS (直角弯是纯循线元素, 不需要导航决策)
                    if (track_element_type == ELEMENT_T_JUNCTION ||
                        track_element_type == ELEMENT_CROSS)
                    {
                        junction_register(track_element_type, (uint8)track_tjunction_dir);
                        approach_timeout = 0;
                    }
                    else
                    {
                        approach_timeout++;
                        if (approach_timeout > 15)  // 15帧内仍未检测到 → 放弃, 防卡死
                        {
                            g_junction_state = JSTATE_IDLE;
                            approach_timeout = 0;
                            junction_set_cooldown();
                        }
                    }
                }

                // 阶段2: 右手定则决策
                if (g_junction_state == JSTATE_DECIDE)
                {
                    junction_decide_direction();
                    // junction_decide_direction 内部将状态设为 JSTATE_TURN 或 JSTATE_BACKTRACK
                }

                // 阶段3: 执行转弯 (转弯中覆盖中心线和偏差)
                if (g_junction_state == JSTATE_TURN)
                {
                    uint8 turning = junction_execute_turn();
                    if (turning)
                    {
                        // 转弯中: 重新计算偏差 (center_line已被修改)
                        track_get_error();
                    }
                    // 转弯完成时 junction_execute_turn 将状态设为 JSTATE_COMPLETE
                }

                // 阶段4: 回溯处理 (搜索可回溯路口)
                if (g_junction_state == JSTATE_BACKTRACK)
                {
                    int8 match_idx = junction_match_backtrack();
                    // 匹配成功 → JSTATE_RETURNING (物理返回)
                    // 匹配失败 → JSTATE_COMPLETE (内部已设置)
                    if (match_idx < 0)
                    {
                        g_junction_state = JSTATE_COMPLETE;
                    }
                    junction_set_cooldown();
                }

                // 阶段4.5: 物理回溯执行 (掉头+反向行驶回目标路口)
                if (g_junction_state == JSTATE_RETURNING)
                {
                    uint8 returning = junction_execute_return();
                    if (returning)
                    {
                        // 回溯中: 重新计算偏差 (center_line可能被U-TURN修改)
                        track_get_error();
                    }
                    // 到达目标路口时, junction_execute_return 内部将状态设为 JSTATE_DECIDE
                }

                // 断路检测: 当前路径是死胡同 → 标记并回溯
                // 只在转弯进行中(>15帧)才触发, 避免转弯刚完成时误判
                if (track_element_type == ELEMENT_BREAK &&
                    g_junction_state == JSTATE_TURN && g_junction_turn_count > 15)
                {
                    junction_mark_dead_end();
                }

                // 路口完成后恢复空闲状态 + 冷却
                if (g_junction_state == JSTATE_COMPLETE)
                {
                    g_junction_state = JSTATE_IDLE;
                    junction_set_cooldown();  // 防重复注册
                }
            }
#endif  // [基础循迹] 岔路口OFS状态机屏蔽结束

// === 元器件识别 (多位置扫描) ===
            // 在赛道线左/中/右各试一次, 取最高分
            g_component_count = 0;
            int16 scan_x[2] = {0, 0};  // 扫描窗口x坐标 (用于叠加显示)
            int16 scan_y[2] = {0, 0};  // 扫描窗口y坐标 (用于叠加显示)
            {
                ComponentResult result;
                uint8 best_type = COMPONENT_NONE;
                uint8 best_conf = 0;
                ComponentResult best_result;

                // 计算赛道线中心 + 左右边界
                int16 track_center_x = image_w / 2;
                int16 left_avg = image_w / 4;
                int16 right_avg = image_w * 3 / 4;
                {
                    int16 bi, lsum = 0, csum = 0, rsum = 0;
                    for (bi = image_h - 10; bi < image_h; bi++)
                    {
                        lsum += left_border[bi];
                        csum += center_line[bi];
                        rsum += right_border[bi];
                    }
                    left_avg = lsum / 10;
                    track_center_x = csum / 10;
                    right_avg = rsum / 10;
                }

                // 双窗口扫描: 中线 + 根据赛道方向偏移12px
                // 窗口1: 赛道线正中心 (捕捉组件主体)
                // 窗口2: 沿赛道偏离方向偏移 (捕捉组件侧面/边缘)
                scan_x[0] = track_center_x;
                scan_y[0] = image_h / 2;
                {
                    // 窗口2偏移方向: 中线偏右则继续偏右, 偏左则偏左
                    int16 center_dev = track_center_x - image_w / 2;
                    int16 offset = (center_dev >= 0) ? 12 : -12;
                    scan_x[1] = track_center_x + offset;
                    scan_y[1] = image_h / 2;
                }

                int16 si;
                for (si = 0; si < 2; si++)
                {
                    int16 sx = scan_x[si];
                    // 不需要限幅: component_recognize 内部在 188x120 坐标空间会正确限幅

                    uint8 ct = component_recognize(
                        mt9v03x_image_1[0],
                        sx,
                        image_h / 2,
                        &result);

                    if (ct != COMPONENT_NONE && result.confidence > best_conf)
                    {
                        best_type = ct;
                        best_conf = result.confidence;
                        best_result = result;
                    }
                    if (best_conf >= 70) break;
                }

                if (best_type != COMPONENT_NONE && best_conf >= 5)
                {
                    g_component_results[0] = best_result;
                    g_component_count = 1;
                }
            }

            // === 多帧确认 ===
            if (g_component_count > 0) {
                uint8 confirmed_type = component_confirm_result(&g_component_results[0]);
                if (confirmed_type != COMPONENT_NONE) {
                    g_confirmed_result = g_component_results[0];
                    g_confirmed_valid = 1;
                }
            }
            else
            {
                // 无元器件时逐渐衰减确认状态 (防永久降速)
                static uint8 no_component_count = 0;
                no_component_count++;
                if (no_component_count > 10)  // 连续10帧无元器件 → 清除
                {
                    g_confirmed_valid = 0;
                    no_component_count = 0;
                }
            }

            // ==========================================
            // 双层转向PID控制 (位置环 + 角度环)
            // ==========================================
            int16 steer_error = track_error;   // 位置偏差
            int16 angle_error = track_angle;    // 角度偏差 (航线斜率)

            // ALIGN_FORCE持久偏差叠加: 跨帧累积的强制回正量
            {
                extern int16 track_force_bias;
                steer_error += track_force_bias;
            }

            // 获取回正状态和连续弯道状态
            AlignState align_state = track_get_align_state();
            uint8 is_continuous = track_is_continuous_curve();

            // 调整PID参数 (位置环+角度环同步)
#if JUNCTION_NAV_ENABLE  // 岔路口转弯PID覆盖 (回溯未实现, 暂禁用)
            if (g_junction_state == JSTATE_TURN)
            {
                // 路口转弯中: 用直角弯激进参数
                pid_update_params(&steering_pid, STEER_KP_RIGHT_ANGLE, 0, STEER_KD_RIGHT_ANGLE);
                pid_update_params(&angle_pid, ANGLE_KP_RIGHT_ANGLE, 0, ANGLE_KD_RIGHT_ANGLE);
            }
            else
#endif  // [基础循迹]
            {
                adjust_steering_pid(track_element_type, align_state, is_continuous);
            }

            // 位置环: 位置式PD → 修正横向偏移
            int32 steer_output = pid_calculate(&steering_pid, steer_error);

            // 角度环: 位置式PD → 修正航向偏差
            int32 angle_output = pid_calculate(&angle_pid, angle_error);

            // 合并输出: 位置 + 角度 → 最终转向
            int32 total_steer = steer_output + angle_output;

            // 总转向限幅
            if (total_steer > STEER_OUT_MAX) total_steer = STEER_OUT_MAX;
            if (total_steer < STEER_OUT_MIN) total_steer = STEER_OUT_MIN;

            // === IMU已禁用，无转向辅助 ===
            int32 imu_steer_comp = 0;

            // === 速度控制 ===
#if JUNCTION_NAV_ENABLE
            // 路口转弯中: 强制最低速度
            if (g_junction_state == JSTATE_TURN)
            {
                target_speed = RIGHT_ANGLE_SPEED;
            }
            // 物理回溯掉头: 最低速度确保不掉头冲出
            else if (g_junction_state == JSTATE_RETURNING)
            {
                target_speed = RETURN_UTURN_SPEED;
            }
            else
#endif
            {
                target_speed = get_target_speed(track_element_type, is_continuous);
            }

            // === IMU已禁用，无速度辅助 ===
            int32 imu_pitch_comp = 0;      // 坡道补偿 (禁用)
            float  imu_roll_factor = 1.0f;  // 侧倾衰减因子 (禁用)

            int32 speed_output = 0;
            if (use_speed_pid)
            {
                // 同步在线调参后的速度PID参数
                pid_update_params(&speed_pid, g_speed_kp, g_speed_ki, SPEED_KD);
                int32 avg_speed = (encoder_get_left_speed() + encoder_get_right_speed()) / 2;
                int32 speed_error = target_speed - avg_speed;
                speed_output = pid_calculate(&speed_pid, speed_error);
            }

            // === 差速转向 (位置环+角度环 合并输出) ===
            int32 l_duty = target_speed + speed_output - total_steer;
            int32 r_duty = target_speed + speed_output + total_steer;

            // 坡道补偿: 左右电机同时增减
            l_duty += imu_pitch_comp;
            r_duty += imu_pitch_comp;

            // 侧倾保护: 左右电机同时乘以衰减因子
            l_duty = (int32)((float)l_duty * imu_roll_factor);
            r_duty = (int32)((float)r_duty * imu_roll_factor);

            // 限幅
            if (l_duty < DUTY_MIN) l_duty = DUTY_MIN;
            if (l_duty > DUTY_MAX) l_duty = DUTY_MAX;
            if (r_duty < DUTY_MIN) r_duty = DUTY_MIN;
            if (r_duty > DUTY_MAX) r_duty = DUTY_MAX;

            my_motor_set_speed((uint32)l_duty, (uint32)r_duty);

#if 0  // [调试] 10秒自动停止 — 演示模式下设为0禁用
            // ===== 10秒自动停止 (约500帧 @~50fps, 采集干净PID调参波形) =====
            if (g_frame_process_count >= 500)
            {
                my_motor_set_speed(0, 0);
                target_speed = 0;
                if (g_frame_process_count == 500)
                {
                    tft180_show_string(0, 80, "STOP 10s");
                    DEBUG_PRINTF("[STOP] 10秒自动停止, 帧数=%lu\r\n", g_frame_process_count);
                }
            }
#endif

            // 虚拟示波器: 每5帧发送 (降低WiFi SPI占用)
            if((g_frame_process_count % 5) == 0)
            {
                int32 l_spd = encoder_get_left_speed();
                int32 r_spd = encoder_get_right_speed();

                seekfree_assistant_oscilloscope_data.channel_num = 8;
                seekfree_assistant_oscilloscope_data.data[0] = (float)steer_error;
                seekfree_assistant_oscilloscope_data.data[1] = (float)steer_output;
                seekfree_assistant_oscilloscope_data.data[2] = (float)angle_error;
                seekfree_assistant_oscilloscope_data.data[3] = (float)angle_output;
                seekfree_assistant_oscilloscope_data.data[4] = (float)total_steer;
                seekfree_assistant_oscilloscope_data.data[5] = (float)((l_spd + r_spd) / 2);
                seekfree_assistant_oscilloscope_data.data[6] = (float)target_speed;
                seekfree_assistant_oscilloscope_data.data[7] = (float)speed_output;
                seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
            }

            // 调试输出 (每50帧打印一次, 减少UART阻塞)
            if((g_frame_process_count % 50) == 1)
            {
                const char *elem_str;
                switch(track_element_type)
                {
                    case ELEMENT_OBTUSE:      elem_str = "钝角弯"; break;
                    case ELEMENT_T_JUNCTION:  elem_str = "T形路口"; break;
                    case ELEMENT_RIGHT_ANGLE: elem_str = "直角弯"; break;
                    case ELEMENT_CROSS:       elem_str = "十字";   break;
                    case ELEMENT_BREAK:       elem_str = "断路";   break;
                    default:                  elem_str = "直道";   break;
                }
                
                const char *align_str;
                switch(align_state)
                {
                    case ALIGN_NONE:      align_str = "无";     break;
                    case ALIGN_CURVE_IN:  align_str = "弯道中"; break;
                    case ALIGN_CURVE_EXIT:align_str = "回正";   break;
                    case ALIGN_FORCE:     align_str = "强制回正"; break;
                    default:              align_str = "未知";   break;
                }
                
                int32 l_enc = encoder_get_left_speed();
                int32 r_enc = encoder_get_right_speed();
                
#if JUNCTION_NAV_ENABLE  // 岔路口状态调试输出
                // 路口状态描述
                const char *jstate_str;
                switch(g_junction_state)
                {
                    case JSTATE_IDLE:      jstate_str = "空闲"; break;
                    case JSTATE_APPROACH:  jstate_str = "接近中"; break;
                    case JSTATE_REGISTER:  jstate_str = "注册"; break;
                    case JSTATE_DECIDE:    jstate_str = "决策"; break;
                    case JSTATE_TURN:      jstate_str = "转弯"; break;
                    case JSTATE_BACKTRACK: jstate_str = "回溯"; break;
                    case JSTATE_COMPLETE:  jstate_str = "完成"; break;
                    default:               jstate_str = "未知"; break;
                }
#endif  // [基础循迹]

                DEBUG_PRINTF("[TRACK] 偏差=%d 元素=%s 强度=%d 方向=%d 回正=%s 连续=%d\r\n",
                    (int)steer_error, elem_str, (int)track_element_strength,
                    (int)track_right_angle_dir, align_str, is_continuous);
#if JUNCTION_NAV_ENABLE  // 岔路口调试输出
                DEBUG_PRINTF("[JUNC]  状态=%s 路口#=%d 选择方向=%s 转向帧=%d 距离=%ld\r\n",
                    jstate_str, (int)g_current_junction_idx,
                    (g_chosen_direction == DIR_RIGHT) ? "右" :
                    (g_chosen_direction == DIR_LEFT)  ? "左" :
                    (g_chosen_direction == DIR_STRAIGHT) ? "直" : "无",
                    (int)g_junction_turn_count, (long)g_junction_distance);
#endif  // [基础循迹]
                DEBUG_PRINTF("[PID]  位置=%ld(偏差=%d Kp=%d Kd=%d) 角度=%ld(偏差=%d Kp=%d Kd=%d) 总=%ld\r\n",
                    (long)steer_output, (int)steer_error, (int)steering_pid.Kp, (int)steering_pid.Kd,
                    (long)angle_output, (int)angle_error, (int)angle_pid.Kp, (int)angle_pid.Kd,
                    (long)total_steer);
                DEBUG_PRINTF("[MOTOR] L=%ld R=%ld 编码L=%ld R=%ld 目标=%ld\r\n",
                    (long)l_duty, (long)r_duty, (long)l_enc, (long)r_enc, (long)target_speed);

                // IMU已禁用，无姿态数据输出
            }

            // === TFT显示 ===
            tft180_show_gray_image(0, 0, bin_image[0], image_w, image_h, image_w, image_h, 128);

            // === 图传叠加: 画扫描窗口 + 识别结果 ===
            #if WIFI_TRANSFER_OVERLAY_ENABLE
            {
                extern uint8 overlay_image[image_h][image_w];
                int oy, ox;
                for (oy = 0; oy < image_h; oy++)
                    for (ox = 0; ox < image_w; ox++)
                        overlay_image[oy][ox] = bin_image[oy][ox];

                if (g_component_count > 0 && g_component_results[0].type != COMPONENT_NONE)
                {
                    ComponentResult *r = &g_component_results[0];
                    wifi_image_transfer_overlay_result(
                        overlay_image[0], image_w, image_h,
                        r->type, r->confidence, 0, 0, 0, 0);

                    int16 bx = r->pos_x - 10, by = r->pos_y - 8;
                    int16 bw2 = 20, bh2 = 16, dx, dy;
                    for (dx = bx; dx < bx + bw2; dx++)
                        if (dx >= 0 && dx < image_w) {
                            if (by >= 0 && by < image_h) overlay_image[by][dx] = 255;
                            if (by+bh2 >= 0 && by+bh2 < image_h) overlay_image[by+bh2][dx] = 255;
                        }
                    for (dy = by; dy < by + bh2; dy++)
                        if (dy >= 0 && dy < image_h) {
                            if (bx >= 0 && bx < image_w) overlay_image[dy][bx] = 255;
                            if (bx+bw2 >= 0 && bx+bw2 < image_w) overlay_image[dy][bx+bw2] = 255;
                        }
                }
                else
                {
                    for (int16 clr_i = 0; clr_i < 5; clr_i++)
                        overlay_image[clr_i][clr_i] = 128;
                }

                // 画扫描窗口角标
                for (int16 wi = 0; wi < 2; wi++)
                {
                    int16 cx = scan_x[wi], cy = scan_y[wi];
                    #define CS 6
                    uint8 wc = (wi == 0) ? 255 : 180;
                    for (int16 i = 0; i <= CS; i++) {
                        if (cx-CS >= 0 && cx-CS < image_w && cy-CS+i >= 0 && cy-CS+i < image_h) overlay_image[cy-CS+i][cx-CS] = wc;
                        if (cx-CS+i >= 0 && cx-CS+i < image_w && cy-CS >= 0 && cy-CS < image_h) overlay_image[cy-CS][cx-CS+i] = wc;
                        if (cx+CS >= 0 && cx+CS < image_w && cy-CS+i >= 0 && cy-CS+i < image_h) overlay_image[cy-CS+i][cx+CS] = wc;
                        if (cx+CS-i >= 0 && cx+CS-i < image_w && cy-CS >= 0 && cy-CS < image_h) overlay_image[cy-CS][cx+CS-i] = wc;
                        if (cx-CS >= 0 && cx-CS < image_w && cy+CS-i >= 0 && cy+CS-i < image_h) overlay_image[cy+CS-i][cx-CS] = wc;
                        if (cx-CS+i >= 0 && cx-CS+i < image_w && cy+CS >= 0 && cy+CS < image_h) overlay_image[cy+CS][cx-CS+i] = wc;
                        if (cx+CS >= 0 && cx+CS < image_w && cy+CS-i >= 0 && cy+CS-i < image_h) overlay_image[cy+CS-i][cx+CS] = wc;
                        if (cx+CS-i >= 0 && cx+CS-i < image_w && cy+CS >= 0 && cy+CS < image_h) overlay_image[cy+CS][cx+CS-i] = wc;
                    }
                    #undef CS
                }

                seekfree_assistant_camera_information_config(
                    SEEKFREE_ASSISTANT_MT9V03X,
                    overlay_image[0], image_w, image_h);

                // WiFi图传: 每3帧发一次 (减少SPI阻塞)
                if ((g_frame_process_count % 3) == 0)
                    wifi_image_transfer_send_frame();
            }
            #endif

            // 清除标志
            mt9v03x_tft180_clear_finish_flag();
        }
        else
        {
            no_frame_timeout++;

            if(no_frame_timeout == 1)
            {
                // 记录vsync
            }

            if((no_frame_timeout % 100000) == 0)
            {
                DEBUG_PRINTF("[WAIT] 超时=%lu VSYNC1=%lu DMA1=%lu\r\n",
                    no_frame_timeout, g_vsync1_count, g_dma1_count);
            }

            if(no_frame_timeout == 200000 && g_vsync1_count == 0)
            {
                DEBUG_PRINTF("\r\n!!! [ERROR] VSYNC中断从未触发!\r\n");
            }

            if(no_frame_timeout == 400000 && g_vsync1_count > 0)
            {
                DEBUG_PRINTF("\r\n!!! [ERROR] VSYNC正常但采集未完成!\r\n");
            }
        }
    }
}

#pragma section all restore

// 图传叠加缓冲区 (放普通内存, 不占 DSRAM)
// 60x100 = 6000字节, DSRAM 放不下
uint8 overlay_image[image_h][image_w];
