/*********************************************************************************************************************
 * IMU660RA 姿态传感器驱动模块
 * 基于逐飞科技 zf_device_imu660ra 底层驱动封装
 * 提供初始化、数据获取、姿态角计算、辅助控制等接口
 *
 * 硬件连接 (SPI模式, 默认引脚):
 *   SCL/SPC  -> P20_11
 *   SDA/DSI  -> P20_14
 *   SA0/SDO  -> P20_12
 *   CS       -> P20_13
 *   VCC      -> 3.3V
 *   GND      -> GND
 ********************************************************************************************************************/

#ifndef _IMU_DRIVER_H_
#define _IMU_DRIVER_H_

#include "zf_common_typedef.h"

// ==================== 配置参数 ====================

// IMU数据更新定时器 (使用CCU60_CH0, 5ms周期)
#define IMU_PIT_CH           CCU60_CH0
#define IMU_PIT_PERIOD_MS    5

// 加速度计量程 (默认8G)
// 可选: IMU660RA_ACC_SAMPLE_SGN_2G, IMU660RA_ACC_SAMPLE_SGN_4G,
//       IMU660RA_ACC_SAMPLE_SGN_8G, IMU660RA_ACC_SAMPLE_SGN_16G
#define IMU_ACC_RANGE        IMU660RA_ACC_SAMPLE_SGN_8G

// 陀螺仪量程 (默认2000DPS)
// 可选: IMU660RA_GYRO_SAMPLE_SGN_125DPS, IMU660RA_GYRO_SAMPLE_SGN_250DPS,
//       IMU660RA_GYRO_SAMPLE_SGN_500DPS, IMU660RA_GYRO_SAMPLE_SGN_1000DPS,
//       IMU660RA_GYRO_SAMPLE_SGN_2000DPS
#define IMU_GYRO_RANGE       IMU660RA_GYRO_SAMPLE_SGN_2000DPS

// ==================== 姿态角计算参数 ====================

// 互补滤波系数 (0~1, 越大越信任加速度计, 越小越信任陀螺仪)
// 小车运行环境振动较大, 建议取较小值 (0.02~0.05)
#define IMU_COMPLEMENTARY_ALPHA  0.03f

// ==================== IMU辅助控制参数 ====================

// --- 转向辅助 (陀螺仪Z轴角速度反馈) ---
// 当摄像头偏差与陀螺仪角速度方向不一致时, 陀螺仪补偿转向
// gyro_z_compensation = GYRO_STEER_KP * gyro_z
// gyro_z 单位: 度/s, 典型弯道角速度 50~150 度/s
#define IMU_GYRO_STEER_KP       3       // 陀螺仪转向补偿比例系数 (放大100倍存储, 实际=0.03)
                                         // 调参: 值越大陀螺仪补偿越强, 过大会振荡

// --- 坡道速度补偿 (俯仰角) ---
// 上坡时增加动力, 下坡时减少动力
// pitch_compensation = PITCH_SPEED_KP * pitch (度)
// 3S电池: 补偿系数同比缩小
#define IMU_PITCH_SPEED_KP      6       // 俯仰角速度补偿系数 (原2S: 8, ×0.73)

// --- 侧倾保护 (横滚角) ---
// 横滚角过大时降速防翻车
// 当 |roll| > ROLL_WARN_THRESHOLD 时, 速度乘以衰减因子
#define IMU_ROLL_WARN_THRESHOLD 10.0f   // 横滚角警告阈值 (度), 超过此值开始降速
#define IMU_ROLL_DANGER_THRESHOLD 20.0f // 横滚角危险阈值 (度), 超过此值大幅降速
#define IMU_ROLL_BRAKE_FACTOR    0.7f   // 危险时速度衰减因子 (0~1, 越小减速越猛)

// --- Z轴角速度异常检测 (打滑/甩尾) ---
// 当角速度突然大幅偏离预期时, 认为发生打滑
#define IMU_GYRO_Z_SPIKE_THRESHOLD 200.0f  // Z轴角速度突变阈值 (度/s)

// ==================== 数据结构 ====================

// IMU原始数据
typedef struct {
    int16 acc_x;        // 加速度计 X 轴原始值
    int16 acc_y;        // 加速度计 Y 轴原始值
    int16 acc_z;        // 加速度计 Z 轴原始值
    int16 gyro_x;       // 陀螺仪 X 轴原始值
    int16 gyro_y;       // 陀螺仪 Y 轴原始值
    int16 gyro_z;       // 陀螺仪 Z 轴原始值
} IMU_RawData;

// IMU物理量数据 (转换后的实际值)
typedef struct {
    float acc_x;        // 加速度 X (单位: g)
    float acc_y;        // 加速度 Y (单位: g)
    float acc_z;        // 加速度 Z (单位: g)
    float gyro_x;       // 角速度 X (单位: 度/s)
    float gyro_y;       // 角速度 Y (单位: 度/s)
    float gyro_z;       // 角速度 Z (单位: 度/s)
} IMU_PhysicsData;

// 姿态角数据 (通过互补滤波计算)
typedef struct {
    float pitch;        // 俯仰角 (度), 绕Y轴旋转, 抬头为正
    float roll;         // 横滚角 (度), 绕X轴旋转, 右倾为正
    float yaw;          // 航向角 (度), 绕Z轴旋转, 顺时针为正 (仅陀螺仪积分, 会漂移)
} IMU_AttitudeData;

// IMU辅助控制输出 (供主循环使用)
typedef struct {
    int32 steer_compensation;    // 转向补偿量 (正=右补偿, 负=左补偿, 已转为整数)
    int32 speed_pitch_comp;      // 坡道速度补偿 (正=上坡加速, 负=下坡减速)
    float  speed_roll_factor;    // 侧倾速度衰减因子 (0~1, 1=不衰减, <1=降速)
    uint8  gyro_z_spike;         // Z轴角速度突变标志 (1=检测到打滑/甩尾)
    float  gyro_z_filtered;      // Z轴角速度低通滤波值 (度/s, 用于平稳判断)
} IMU_ControlAssist;

// IMU模块状态
typedef struct {
    uint8 initialized;          // 初始化标志: 0=未初始化, 1=已初始化
    uint8 data_ready;           // 数据就绪标志: 1=有新数据
    IMU_RawData        raw;        // 原始数据
    IMU_PhysicsData    physics;    // 物理量数据
    IMU_AttitudeData   attitude;   // 姿态角数据
    IMU_ControlAssist  assist;     // 辅助控制输出
} IMU_State;

// ==================== 全局变量声明 ====================

extern IMU_State g_imu;

// ==================== 函数接口 ====================

/**
 * @brief  初始化IMU660RA模块
 *         包括传感器初始化、PIT定时器配置、姿态角初始化
 * @retval 0=成功, 1=失败
 */
uint8 imu_driver_init(void);

/**
 * @brief  在PIT中断中调用, 周期性获取IMU原始数据
 *         并更新姿态角 (互补滤波) 和辅助控制数据
 * @note   此函数应在 CCU60_CH0 的PIT中断服务函数中调用
 */
void imu_driver_isr_update(void);

/**
 * @brief  获取IMU原始数据
 */
void imu_driver_get_raw(IMU_RawData *raw);

/**
 * @brief  获取IMU物理量数据
 */
void imu_driver_get_physics(IMU_PhysicsData *physics);

/**
 * @brief  获取姿态角数据
 */
void imu_driver_get_attitude(IMU_AttitudeData *attitude);

/**
 * @brief  获取辅助控制数据
 */
void imu_driver_get_assist(IMU_ControlAssist *assist);

/**
 * @brief  获取俯仰角
 */
float imu_driver_get_pitch(void);

/**
 * @brief  获取横滚角
 */
float imu_driver_get_roll(void);

/**
 * @brief  获取Z轴角速度
 */
float imu_driver_get_gyro_z(void);

/**
 * @brief  检查IMU是否初始化成功
 */
uint8 imu_driver_is_ready(void);

/**
 * @brief  计算IMU转向补偿量
 *         当摄像头偏差与陀螺仪角速度方向不一致时, 陀螺仪提供额外转向补偿
 *         典型场景: 车轮打滑导致实际转向不足, 陀螺仪检测到角速度偏小, 增加补偿
 * @param  camera_steer_output: 摄像头PID计算的转向输出 (正=右转, 负=左转)
 * @retval 转向补偿量 (与camera_steer_output同号=增强转向, 反号=抑制过转)
 */
int32 imu_driver_calc_steer_compensation(int32 camera_steer_output);

/**
 * @brief  计算坡道速度补偿
 *         上坡增加动力, 下坡减少动力
 * @retval 速度补偿量 (正=加速, 负=减速)
 */
int32 imu_driver_calc_pitch_speed_comp(void);

/**
 * @brief  计算侧倾速度衰减因子
 *         横滚角过大时降速防翻车
 * @retval 速度衰减因子 (0~1, 1=不衰减)
 */
float imu_driver_calc_roll_speed_factor(void);

/**
 * @brief  检测Z轴角速度突变 (打滑/甩尾)
 * @retval 1=检测到突变, 0=正常
 */
uint8 imu_driver_detect_gyro_spike(void);

#endif // _IMU_DRIVER_H_
