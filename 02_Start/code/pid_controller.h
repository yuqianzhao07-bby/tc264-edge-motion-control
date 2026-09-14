#ifndef _pid_controller_h_
#define _pid_controller_h_

#include "zf_common_headfile.h"

// PID模式枚举
typedef enum {
    PID_MODE_POSITION = 0,  // 位置式PID (转向用, 响应快)
    PID_MODE_INCREMENT = 1  // 增量式PID (速度用, 平稳)
} PID_Mode;

// PID控制器结构体
typedef struct {
    // 模式
    PID_Mode mode;      // PID模式

    // 参数
    int32 Kp;           // 比例系数
    int32 Ki;           // 积分系数
    int32 Kd;           // 微分系数

    // 状态变量
    int32 error;         // 当前偏差
    int32 last_error;    // 上次偏差
    int32 prev_error;    // 上上次偏差
    int32 integral;      // 积分累加
    int32 last_output;   // 上次输出

    // 限幅
    int32 out_min;       // 输出最小值
    int32 out_max;       // 输出最大值
    int32 error_thres;   // 积分分离阈值 (偏差大于此值不积分)

    // 输出变化率限制 (仅增量式使用)
    int32 max_step;      // 单步最大变化量

    // 输出
    int32 output;        // PID输出

    // 调试
    uint32 call_count;   // 调用次数
} PID_Controller;

// ===== 转向PID参数 (位置式PD) =====
// 降速+增阻尼: Kp↓减少过冲, Kd↑增加阻尼防晃动
#define STEER_KP          3      // 比例系数 (降, 减少过冲)
#define STEER_KI          0      // 积分系数 (循迹不用)
#define STEER_KD          14     // 微分系数 (增, 增强阻尼防蛇形)

// 弯道参数 (直角弯/钝角弯)
#define STEER_KP_CURVE    8      // 弯道比例系数
#define STEER_KD_CURVE    15     // 弯道微分系数

// 直角弯专用参数
#define STEER_KP_RIGHT_ANGLE  10  // 直角弯比例系数
#define STEER_KD_RIGHT_ANGLE  16  // 直角弯微分系数

// ===== 角度PID参数 (位置式PD) =====
// 角度环暂时归零: 先验证位置环单独工作是否稳定, 确认震荡来源
#define ANGLE_KP          0      // 角度比例系数 (0=禁用角度环)
#define ANGLE_KD          0      // 角度微分系数
#define ANGLE_KP_CURVE    0      // 弯道角度KP
#define ANGLE_KD_CURVE    0      // 弯道角度KD
#define ANGLE_KP_RIGHT_ANGLE 0   // 直角弯角度KP
#define ANGLE_KD_RIGHT_ANGLE 0   // 直角弯角度KD
#define ANGLE_OUT_MIN     0
#define ANGLE_OUT_MAX     0
#define ANGLE_ERROR_THRES  6     // 角度积分分离阈值

// ===== 速度PID参数 (增量式PI) =====
#define SPEED_KP          30     // (原2S: 40, ×0.73)
#define SPEED_KI          2      // (原2S: 3, ×0.73)
#define SPEED_KD          6      // (原2S: 8, ×0.73)

// 输出限幅 (BASE_SPEED=750, 差速±250 → 500~1000, 2x转向力)
#define STEER_OUT_MIN     -250
#define STEER_OUT_MAX      250
#define SPEED_OUT_MIN     -400
#define SPEED_OUT_MAX      400

// 积分分离阈值
#define STEER_ERROR_THRES  8
#define SPEED_ERROR_THRES  30

// 最大单步变化量 (仅增量式使用)
#define STEER_MAX_STEP     500
#define SPEED_MAX_STEP     200

// 函数声明
void pid_init(PID_Controller *pid, int32 kp, int32 ki, int32 kd);
void pid_set_mode(PID_Controller *pid, PID_Mode mode);
void pid_reset(PID_Controller *pid);
int32 pid_calculate(PID_Controller *pid, int32 error);
void pid_set_limits(PID_Controller *pid, int32 min, int32 max);
void pid_set_step_limit(PID_Controller *pid, int32 max_step);
void pid_set_error_thres(PID_Controller *pid, int32 thres);
void pid_update_params(PID_Controller *pid, int32 kp, int32 ki, int32 kd);

#endif
