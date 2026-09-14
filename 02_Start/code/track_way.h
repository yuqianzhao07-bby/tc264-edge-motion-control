#ifndef _track_way_h_
#define _track_way_h_

#include "image.h"

// 元素类型定义 (电路图赛道专用)
// 顺序: 直道 → 钝角弯 → T形 → 直角弯 → 十字 → 断路
#define ELEMENT_NONE          0   // 无元素 (直道)
#define ELEMENT_OBTUSE        1   // 钝角弯
#define ELEMENT_T_JUNCTION    2   // T形路口 (右手定则入口)
#define ELEMENT_RIGHT_ANGLE   3   // 直角弯
#define ELEMENT_CROSS         4   // 十字路口
#define ELEMENT_BREAK         5   // 断路

// 左边界/右边界/中线数组
extern int16 left_border[image_h];
extern int16 right_border[image_h];
extern int16 center_line[image_h];

// 最终偏差值 (正值偏右需左转, 负值偏左需右转)
extern int16 track_error;

// 航线角度 (中线斜率, 像素/10行)
// 正值=轨道向右偏(需右转), 负值=轨道向左偏(需左转)
extern int16 track_angle;

// 元素信息 (供速度控制使用)
extern uint8 track_element_type;     // 当前元素类型
extern int16 track_element_strength; // 元素强度 (0-100)

// 直角弯方向: +1=右直角弯(需右转), -1=左直角弯(需左转)
extern int8 track_right_angle_dir;

// T形路口支路方向: +1=右侧支路, -1=左侧支路, 0=非T形
extern int8 track_tjunction_dir;

// 扫描行数 (从底部向上)
#define TRACK_SCAN_ROWS  45

// 边缘搜索范围 (用于直角弯检测)
// 关键: 只有最边缘的列才有直角弯特征，范围不能太大
// 陕科大只搜最边缘约10列
#define EDGE_SEARCH_COLS_INNER  12   // 内区: 最边缘12列, 直角弯特征区
#define EDGE_SEARCH_COLS_OUTER  25   // 外区: 最边缘25列, 钝角弯特征区
#define EDGE_SEARCH_STEP  2          // 搜索步长

// 顶部搜索范围 (用于十字/正常巡线)
#define TOP_SEARCH_ROWS   15   // 从图像顶部向下搜索的行数

// 底部搜索范围
#define BOTTOM_SEARCH_ROWS 10  // 从图像底部向上搜索的行数

// 直角弯检测阈值
#define RIGHT_ANGLE_CHANGE_MIN  3   // 内区跳变>3次(即≥4次/6列=67%)才确认直角弯, 防噪点误触发
#define OBTUSE_CHANGE_MIN       2   // 外区跳变>2次认为有钝角弯边缘

// ==================== 弯道回正相关 ====================

// 回正状态枚举
typedef enum {
    ALIGN_NONE = 0,      // 无回正需求
    ALIGN_CURVE_IN,      // 弯道中
    ALIGN_CURVE_EXIT,    // 弯道出口, 需要回正
    ALIGN_FORCE          // 强制回正模式
} AlignState;

// 回正状态
extern AlignState track_align_state;
extern uint8 track_align_count;      // 回正计数
extern int16 track_target_center;    // 目标中线位置
extern int16 track_force_bias;       // ALIGN_FORCE持久偏差 (跨帧累积, 叠加到track_error)

// 钝角弯参数
#define OBTUSE_MIN_ANGLE    20      // 最小钝角角度(度) - 降低阈值提高灵敏度
#define OBTUSE_MAX_ANGLE    150     // 最大钝角角度(度) - 增大支持更大角度
#define OBTUSE_DETECT_ROWS  15      // 钝角弯检测行数
#define ALIGN_REQUIRED_THRES  10    // 需要回正的偏差阈值 - 稍增大避免误触发
#define ALIGN_STABLE_COUNT   8      // 稳定回正所需帧数 - 稍减少加快退出

// 连续弯道参数
#define CONTINUOUS_CURVE_THRESHOLD 30  // 连续弯道阈值(帧数)
#define MAX_CURVE_ANGLE_ACCUM 180       // 最大累积转弯角度(度)

// 巡线: 找左右边界 + 元素检测 + 补线
void track_find_line(void);

// 计算偏差
void track_get_error(void);

// 补线函数: 从(x1,y1)到(x2,y2)画虚拟中线
void track_add_line(int16 x1, int16 y1, int16 x2, int16 y2);

// 检查是否需要回正
uint8 track_need_align(void);

// 获取回正状态
AlignState track_get_align_state(void);

// 检查是否连续弯道
uint8 track_is_continuous_curve(void);

// === 岔路口导航全局开关 ===
#define JUNCTION_NAV_ENABLE  0   // 1=启用路口遍历+回溯, 0=基础循迹模式

// ==================== Layer 3: 路口遍历 (右手法则) ====================

// 方向编码 (位掩码, 用于 dirs_available / dirs_tried)
#define DIR_STRAIGHT    (1 << 0)   // 0x01 直行
#define DIR_LEFT        (1 << 1)   // 0x02 左转
#define DIR_RIGHT       (1 << 2)   // 0x04 右转
#define DIR_BACK        (1 << 3)   // 0x08 掉头(回溯用)

// 右手法则决策优先级: 右 > 直 > 左 > 回溯
// 决策优先级数组 (dirs_available &~ dirs_tried 中按序选取)
#define DIR_PRIORITY_COUNT  3
extern const uint8 dir_priority[DIR_PRIORITY_COUNT];  // {DIR_RIGHT, DIR_STRAIGHT, DIR_LEFT}

// 路口记录 (距离指纹)
#define MAX_JUNCTIONS  16        // 最多记录16个路口
#define JUNCTION_DIST_TOLERANCE   80    // 段距离匹配容差 (脉冲)
#define JUNCTION_ABS_TOLERANCE   300    // 绝对距离匹配容差 (脉冲)
#define MAX_BACKTRACK   3         // 最大回溯次数

typedef struct {
    int32 seg_distance;          // 距上一个路口的段距离 (编码器脉冲)
    int32 abs_distance;          // 从起点累计的绝对距离
    int32 outbound_distance;     // 从此路口到死胡同的距离 (回溯返回时匹配用)
    uint8 dirs_available;        // 可走方向位掩码
    uint8 dirs_tried;            // 已尝试方向位掩码
    uint8 entry_dir;             // 从哪个方向进入此路口
    uint8 backtrack_count;       // 回溯次数
    uint8 element_type;          // 原始元素类型 (T_JUNCTION / CROSS)
    uint8 registered;            // 1=此槽位已占用
    uint8 turn_started;          // 1=已开始转弯
} JunctionRecord;

// 路口遍历状态机
typedef enum {
    JSTATE_IDLE = 0,        // 无路口, 正常巡线
    JSTATE_APPROACH,        // 检测到路口, 正在接近
    JSTATE_REGISTER,        // 注册路口信息
    JSTATE_DECIDE,          // 右手定则决策方向
    JSTATE_TURN,            // 执行转弯
    JSTATE_BACKTRACK,       // 搜索可回溯路口
    JSTATE_RETURNING,       // 物理返回: 掉头→反向行驶→到达目标路口
    JSTATE_COMPLETE         // 路口遍历完成
} JunctionState;

// 路口遍历全局状态
extern JunctionRecord g_junctions[MAX_JUNCTIONS];
extern uint8 g_junction_count;
extern uint8 g_current_junction_idx;
extern JunctionState g_junction_state;
extern uint8 g_chosen_direction;     // 右手定则选中的方向
extern int32 g_junction_distance;    // 当前段累计距离 (编码器脉冲)
extern int16 g_junction_turn_count;  // 转弯执行帧计数
extern uint8 g_backtrack_total;      // 全局回溯计数
extern uint8 g_exploration_complete; // 1=全部路口已探索完毕, 禁止重新进入

// 路口遍历函数
void junction_init(void);                                     // 初始化
void junction_update_distance(int32 left_enc, int32 right_enc); // 每帧更新距离
uint8 junction_detect_approach(void);                          // 检测是否接近路口
void junction_register(uint8 element_type, uint8 tjunction_side); // 注册路口
void junction_decide_direction(void);                          // 右手定则决策
uint8 junction_execute_turn(void);                             // 执行转弯 (返回1=转弯中)
uint8 junction_execute_return(void);                           // 物理回溯: 掉头+反向行驶 (返回1=执行中)
int8  junction_match_backtrack(void);                          // 距离指纹回溯匹配 (返回路口索引, -1=未匹配)
void junction_mark_dead_end(void);                             // 标记死路, 准备回溯
void junction_set_cooldown(void);                              // 设置路口冷却 (防重复注册)

// 回溯参数
#define RETURN_UTURN_FRAMES      70    // 180°掉头帧数 (~1.4s @50fps)
#define RETURN_UTURN_SPEED       500   // 掉头速度 (低于直角弯, 确保不冲出)
#define RETURN_TRACK_SPEED       600   // 反向行驶速度
#define RETURN_DIST_MATCH        60    // 返回距离匹配容差 (编码器脉冲)

// 元器件识别: 使用 component_recognition 模块 (描线法+模板匹配)
// 目前基础循迹阶段已临时禁用, 路径完全恢复后重新启用

#endif
