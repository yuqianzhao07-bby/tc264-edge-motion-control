/*********************************************************************************************************************
 * IMU660RA 姿态传感器驱动模块 - 实现文件
 * 基于逐飞科技 zf_device_imu660ra 底层驱动封装
 * 提供初始化、数据获取、姿态角计算、辅助控制等接口
 ********************************************************************************************************************/

#include "imu_driver.h"
#include "zf_device_imu660ra.h"
#include "zf_driver_pit.h"
#include "zf_driver_gpio.h"
#include "zf_common_debug.h"
#include "zf_common_clock.h"
#include <math.h>

// ==================== 全局变量 ====================

IMU_State g_imu = {0};

// 姿态角计算内部变量
static float s_pitch_acc = 0;
static float s_roll_acc  = 0;
static float s_pitch     = 0;
static float s_roll      = 0;
static float s_yaw       = 0;
static uint8 s_first_run = 1;

// PIT周期 (秒), 用于陀螺仪积分
static float s_dt = IMU_PIT_PERIOD_MS / 1000.0f;

// Z轴角速度低通滤波 (用于打滑检测和转向辅助)
static float s_gyro_z_filtered = 0;
static float s_gyro_z_last     = 0;

// 低通滤波系数 (0~1, 越小越平滑)
#define GYRO_Z_LPF_ALPHA  0.2f

// ==================== 内部函数 ====================

static void imu_calculate_acc_angle(float acc_x, float acc_y, float acc_z)
{
    float acc_yz = acc_y * acc_y + acc_z * acc_z;
    if(acc_yz > 0.0001f)
    {
        s_pitch_acc = (float)atan2(acc_x, (float)sqrt(acc_yz)) * 57.2957795f;
    }

    float acc_xz = acc_x * acc_x + acc_z * acc_z;
    if(acc_xz > 0.0001f)
    {
        s_roll_acc = (float)atan2(acc_y, (float)sqrt(acc_xz)) * 57.2957795f;
    }
}

static void imu_complementary_filter_update(float gyro_x, float gyro_y, float gyro_z,
                                             float acc_x, float acc_y, float acc_z)
{
    imu_calculate_acc_angle(acc_x, acc_y, acc_z);

    if(s_first_run)
    {
        s_pitch = s_pitch_acc;
        s_roll  = s_roll_acc;
        s_yaw   = 0.0f;
        s_gyro_z_filtered = gyro_z;
        s_gyro_z_last     = gyro_z;
        s_first_run = 0;
        return;
    }

    // 陀螺仪积分
    float pitch_gyro = s_pitch + gyro_y * s_dt;
    float roll_gyro  = s_roll  + gyro_x * s_dt;
    float yaw_gyro   = s_yaw   + gyro_z * s_dt;

    // 互补滤波
    s_pitch = IMU_COMPLEMENTARY_ALPHA * s_pitch_acc + (1.0f - IMU_COMPLEMENTARY_ALPHA) * pitch_gyro;
    s_roll  = IMU_COMPLEMENTARY_ALPHA * s_roll_acc  + (1.0f - IMU_COMPLEMENTARY_ALPHA) * roll_gyro;

    // 航向角只有陀螺仪积分
    s_yaw = yaw_gyro;
    if(s_yaw > 180.0f)  s_yaw -= 360.0f;
    if(s_yaw < -180.0f) s_yaw += 360.0f;

    // Z轴角速度低通滤波
    s_gyro_z_filtered = GYRO_Z_LPF_ALPHA * gyro_z + (1.0f - GYRO_Z_LPF_ALPHA) * s_gyro_z_filtered;
}

// ==================== 外部接口 ====================

uint8 imu_driver_init(void)
{
    uint8 result;

    g_imu.initialized = 0;
    g_imu.data_ready  = 0;

    result = imu660ra_init();
    if(result != 0)
    {
        // 不使用 zf_log, 避免输出到图传屏幕
        // 主函数中已有初始化失败的警告输出
        return 1;
    }

    pit_ms_init(IMU_PIT_CH, IMU_PIT_PERIOD_MS);

    s_pitch_acc = 0;
    s_roll_acc  = 0;
    s_pitch     = 0;
    s_roll      = 0;
    s_yaw       = 0;
    s_first_run = 1;
    s_gyro_z_filtered = 0;
    s_gyro_z_last     = 0;
    s_dt = IMU_PIT_PERIOD_MS / 1000.0f;

    g_imu.assist.steer_compensation = 0;
    g_imu.assist.speed_pitch_comp   = 0;
    g_imu.assist.speed_roll_factor  = 1.0f;
    g_imu.assist.gyro_z_spike       = 0;
    g_imu.assist.gyro_z_filtered    = 0;

    g_imu.initialized = 1;

    // 不使用 zf_log, 避免输出到图传屏幕
    // 主函数中已有初始化成功的输出
    return 0;
}

void imu_driver_isr_update(void)
{
    if(!g_imu.initialized)
        return;

    // 获取原始数据
    imu660ra_get_acc();
    imu660ra_get_gyro();

    // 保存原始数据
    g_imu.raw.acc_x  = imu660ra_acc_x;
    g_imu.raw.acc_y  = imu660ra_acc_y;
    g_imu.raw.acc_z  = imu660ra_acc_z;
    g_imu.raw.gyro_x = imu660ra_gyro_x;
    g_imu.raw.gyro_y = imu660ra_gyro_y;
    g_imu.raw.gyro_z = imu660ra_gyro_z;

    // 转换为物理量
    g_imu.physics.acc_x  = imu660ra_acc_transition(imu660ra_acc_x);
    g_imu.physics.acc_y  = imu660ra_acc_transition(imu660ra_acc_y);
    g_imu.physics.acc_z  = imu660ra_acc_transition(imu660ra_acc_z);
    g_imu.physics.gyro_x = imu660ra_gyro_transition(imu660ra_gyro_x);
    g_imu.physics.gyro_y = imu660ra_gyro_transition(imu660ra_gyro_y);
    g_imu.physics.gyro_z = imu660ra_gyro_transition(imu660ra_gyro_z);

    // 互补滤波计算姿态角
    imu_complementary_filter_update(
        g_imu.physics.gyro_x, g_imu.physics.gyro_y, g_imu.physics.gyro_z,
        g_imu.physics.acc_x,  g_imu.physics.acc_y,  g_imu.physics.acc_z
    );

    // 保存姿态角
    g_imu.attitude.pitch = s_pitch;
    g_imu.attitude.roll  = s_roll;
    g_imu.attitude.yaw   = s_yaw;

    // 保存Z轴滤波值
    g_imu.assist.gyro_z_filtered = s_gyro_z_filtered;

    // Z轴角速度突变检测 (打滑/甩尾)
    float gyro_z_delta = g_imu.physics.gyro_z - s_gyro_z_last;
    if(gyro_z_delta < 0) gyro_z_delta = -gyro_z_delta;
    g_imu.assist.gyro_z_spike = (gyro_z_delta > IMU_GYRO_Z_SPIKE_THRESHOLD) ? 1 : 0;
    s_gyro_z_last = g_imu.physics.gyro_z;

    g_imu.data_ready = 1;
}

void imu_driver_get_raw(IMU_RawData *raw)
{
    if(raw) *raw = g_imu.raw;
}

void imu_driver_get_physics(IMU_PhysicsData *physics)
{
    if(physics) *physics = g_imu.physics;
}

void imu_driver_get_attitude(IMU_AttitudeData *attitude)
{
    if(attitude) *attitude = g_imu.attitude;
}

void imu_driver_get_assist(IMU_ControlAssist *assist)
{
    if(assist) *assist = g_imu.assist;
}

float imu_driver_get_pitch(void)
{
    return g_imu.attitude.pitch;
}

float imu_driver_get_roll(void)
{
    return g_imu.attitude.roll;
}

float imu_driver_get_gyro_z(void)
{
    return g_imu.physics.gyro_z;
}

uint8 imu_driver_is_ready(void)
{
    return g_imu.initialized;
}

// ==================== 辅助控制计算函数 ====================

int32 imu_driver_calc_steer_compensation(int32 camera_steer_output)
{
    if(!g_imu.initialized)
        return 0;

    // 使用低通滤波后的Z轴角速度, 避免噪声干扰
    float gyro_z = s_gyro_z_filtered;

    // 陀螺仪转向补偿: gyro_z * Kp
    // gyro_z > 0 表示车正在右转, 应该增加右转补偿
    // gyro_z < 0 表示车正在左转, 应该增加左转补偿
    // 系数 IMU_GYRO_STEER_KP 放大100倍存储, 实际 = IMU_GYRO_STEER_KP / 100
    int32 gyro_comp = (int32)(gyro_z * (float)IMU_GYRO_STEER_KP / 100.0f * 100.0f);

    // 判断摄像头和陀螺仪方向是否一致
    // 同号: 转向一致, 陀螺仪辅助增强 (转向不足补偿)
    // 异号: 转向不一致, 可能过转, 陀螺仪抑制
    if(camera_steer_output != 0)
    {
        int32 same_dir = ((camera_steer_output > 0) && (gyro_comp > 0)) ||
                         ((camera_steer_output < 0) && (gyro_comp < 0));

        if(same_dir)
        {
            // 同向: 陀螺仪检测到正在转向但角速度偏小, 说明转向不足, 增加补偿
            // 补偿量 = 期望角速度 - 实际角速度 的映射
            // 简化处理: 直接叠加陀螺仪补偿 (但限制幅度, 避免过冲)
            int32 max_comp = (camera_steer_output > 0) ? camera_steer_output / 3 : -camera_steer_output / 3;
            if(max_comp < 0) max_comp = -max_comp;

            if(gyro_comp > 0 && gyro_comp > max_comp) gyro_comp = max_comp;
            if(gyro_comp < 0 && -gyro_comp > max_comp) gyro_comp = -max_comp;

            g_imu.assist.steer_compensation = gyro_comp;
        }
        else
        {
            // 反向: 陀螺仪检测到过转, 抑制转向
            // 补偿量与摄像头输出反向, 幅度限制为摄像头输出的20%
            int32 max_comp = (camera_steer_output > 0) ? camera_steer_output / 5 : -camera_steer_output / 5;
            if(max_comp < 0) max_comp = -max_comp;

            if(gyro_comp > 0 && gyro_comp > max_comp) gyro_comp = max_comp;
            if(gyro_comp < 0 && -gyro_comp > max_comp) gyro_comp = -max_comp;

            g_imu.assist.steer_compensation = gyro_comp;
        }
    }
    else
    {
        // 摄像头偏差为0 (直道), 但陀螺仪检测到偏转, 纠正
        g_imu.assist.steer_compensation = gyro_comp;
    }

    return g_imu.assist.steer_compensation;
}

int32 imu_driver_calc_pitch_speed_comp(void)
{
    if(!g_imu.initialized)
        return 0;

    float pitch = g_imu.attitude.pitch;

    // 俯仰角补偿: 上坡(pitch>0)加速, 下坡(pitch<0)减速
    // 系数 IMU_PITCH_SPEED_KP 放大100倍存储, 实际 = IMU_PITCH_SPEED_KP / 100
    // 补偿量 = pitch(度) * 实际系数 * 100 (转为整数占空比单位)
    int32 comp = (int32)(pitch * (float)IMU_PITCH_SPEED_KP);

    // 限制补偿幅度, 避免极端情况 (3S电池限幅缩小)
    if(comp > 150)  comp = 150;
    if(comp < -150) comp = -150;

    g_imu.assist.speed_pitch_comp = comp;
    return comp;
}

float imu_driver_calc_roll_speed_factor(void)
{
    if(!g_imu.initialized)
        return 1.0f;

    float roll = g_imu.attitude.roll;
    float abs_roll = (roll >= 0) ? roll : -roll;
    float factor = 1.0f;

    if(abs_roll > IMU_ROLL_DANGER_THRESHOLD)
    {
        // 危险: 大幅降速
        factor = IMU_ROLL_BRAKE_FACTOR;
    }
    else if(abs_roll > IMU_ROLL_WARN_THRESHOLD)
    {
        // 警告: 线性降速
        // 从1.0线性降到IMU_ROLL_BRAKE_FACTOR
        float range = IMU_ROLL_DANGER_THRESHOLD - IMU_ROLL_WARN_THRESHOLD;
        float ratio = (abs_roll - IMU_ROLL_WARN_THRESHOLD) / range;
        factor = 1.0f - ratio * (1.0f - IMU_ROLL_BRAKE_FACTOR);
    }

    g_imu.assist.speed_roll_factor = factor;
    return factor;
}

uint8 imu_driver_detect_gyro_spike(void)
{
    return g_imu.assist.gyro_z_spike;
}
