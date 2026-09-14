#ifndef _component_recognition_h_
#define _component_recognition_h_

#include "zf_common_typedef.h"
#include "image.h"

// 元器件类型 - 极简版
// 只有三极管需要特殊处理，其余全部归为NORMAL直接压过
#define COMPONENT_NONE        0   // 无元器件
#define COMPONENT_NORMAL      1   // 普通元器件(电阻/电容/电感/二极管/电流表/电池/电源/运放/接地/开关等): 直接压过去直走
#define COMPONENT_TRANSISTOR  2   // 三极管: 十字/Y形特征，需要特殊处理
#define COMPONENT_UNKNOWN     99  // 未知(当作NORMAL处理)

// 异常类型
#define ANOMALY_NONE          0
#define ANOMALY_LEFT_JUMP     1   // 左边界突跳
#define ANOMALY_RIGHT_JUMP    2   // 右边界突跳
#define ANOMALY_BLOB          3   // 独立白色区域

// 特征框大小（双框特征比对版）
#define FRAME_W 28      // 特征框宽度
#define FRAME_H 28      // 特征框高度

// 异常检测阈值
#define ANOMALY_JUMP_THRESHOLD  5    // 相邻行边界跳变>5px触发
#define ANOMALY_MIN_CONTINUOUS  3    // 连续至少3行异常才算有效

// 识别结果
typedef struct {
    uint8 type;                 // 元器件类型
    uint8 confidence;           // 置信度 0-100
    int16 pos_x, pos_y;         // 在赛道上的位置 (60x100坐标)
    int16 physical_w_mm;        // 估算实际宽度 (mm)
    int16 physical_h_mm;        // 估算实际高度 (mm)
    uint8 anomaly_type;         // 触发异常类型
    int16 anomaly_row;          // 异常行号 (60x100坐标)
} ComponentResult;

// 当前帧识别结果 (件)
extern ComponentResult g_component_results[3];
extern uint8 g_component_count;

// 多帧确认: 连续 CONFIRM_FRAMES 帧识别为同一类型才输出
#define CONFIRM_FRAMES  3

// 确认后的稳定输出 (供外部读取)
extern ComponentResult g_confirmed_result;
extern uint8 g_confirmed_valid;  // 1=有确认结果, 0=无

// 标定参数: 1px = K mm (当前 1.79mm/px)
#define MM_PER_PX_188  1.79f

// 初始化元器件识别模块
void component_recognition_init(void);

// 检测边界异常 (在track_way边界检测后调用)
// 返回: 1=检测到异常, 0=无异常
uint8 component_detect_anomaly(int16 *left_border, int16 *right_border,
                                int16 rows, uint8 *anomaly_type, int16 *anomaly_row);

// 识别元器件 (在检测到异常后调用)
// 从原始灰度图 mt9v03x_image_1 中截取窗口并识别
// 返回元器件类型
uint8 component_recognize(uint8 *gray_image_188x120,
                           int16 anomaly_x60, int16 anomaly_y60,
                           ComponentResult *result);

// 获取元器件名称字符串
const char *component_get_name(uint8 type);

// 多帧确认: 每帧调用, 连续N帧同类型才输出
// 返回: 确认后的元器件类型
uint8 component_confirm_result(ComponentResult *result);

#endif
