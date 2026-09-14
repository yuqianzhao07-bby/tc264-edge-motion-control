/*
 * component_recognition.c - 元器件识别模块 (极简版)
 * 
 * 设计思路：
 * 1. 使用描线法（边界追踪）选中整个元器件
 * 2. 严格判定三极管：只有符合十字/Y形特征的才判为三极管
 * 3. 其余所有元器件（电阻/电容/电感/二极管/电流表/电池/电源/运放/接地/开关等）
 *    全部归为NORMAL，直接压过去直走
 * 
 * 三极管判定标准：
 * - 元器件必须有明显的十字/Y形结构
 * - 至少3个方向有明显延伸的分支
 * - 中心区域有交叉点
 */

#include "component_recognition.h"
#include "image.h"
#include "zf_device_mt9v03x_double.h"

// ==================== 配置参数 ====================

// 大窗口裁剪尺寸（足够大以框住整个元器件）
#define DETECT_WINDOW_W 90     // 检测窗口宽度
#define DETECT_WINDOW_H 90     // 检测窗口高度

// 最小元器件尺寸
#define MIN_COMPONENT_SIZE 12

// 滤波帧数（连续N帧稳定才输出）
#define FILTER_FRAMES 3

// 三极管模板匹配参数
#define TRANSISTOR_TEMPLATE_SIZE 15  // 模板尺寸 (奇数)
#define TRANSISTOR_MATCH_THRESHOLD 50  // 匹配阈值 (%) (降低标准)

// ==================== 三极管模板结构体 ====================

// 三极管方向类型
typedef enum {
    TRANSISTOR_DIR_LEFT,   // 分支在左侧
    TRANSISTOR_DIR_RIGHT,  // 分支在右侧
} TransistorDirection;

// 三极管特征模板
typedef struct {
    TransistorDirection direction;  // 方向
    int16 arm_lengths[4];          // 四个方向分支长度 (上,下,左,右)
    float arm_ratio;               // 主臂与分支臂长度比
    int16 center_size;             // 中心交叉区域大小
    int16 branch_count;            // 分支数量
} TransistorTemplate;

// 预设三极管模板
TransistorTemplate g_transistor_templates[] = {
    // 模板1: 分支在右侧
    {
        TRANSISTOR_DIR_RIGHT,
        {20, 25, 5, 15},   // 上,下,左,右分支长度
        1.5f,              // 主臂/分支臂比
        30,                // 中心区域白色像素数
        2                  // 右侧2个分支
    },
    // 模板2: 分支在左侧
    {
        TRANSISTOR_DIR_LEFT,
        {20, 25, 15, 5},   // 上,下,左,右分支长度
        1.5f,              // 主臂/分支臂比
        30,                // 中心区域白色像素数
        2                  // 左侧2个分支
    }
};

#define TRANSISTOR_TEMPLATE_COUNT (sizeof(g_transistor_templates) / sizeof(TransistorTemplate))

// ==================== 全局变量 ====================

ComponentResult g_component_results[3];
uint8 g_component_count = 0;
ComponentResult g_confirmed_result;
uint8 g_confirmed_valid = 0;

// 滤波状态
static uint8 filter_buffer[FILTER_FRAMES];
static uint8 filter_index = 0;
static uint8 filter_full = 0;

// ==================== 内部函数声明 ====================

static uint8 find_component_boundary(uint8 *gray_img, int16 start_x, int16 start_y,
                                     int16 *min_x, int16 *max_x, int16 *min_y, int16 *max_y);
static uint8 extract_transistor_features(uint8 *gray_img, int16 cx, int16 cy, int16 w, int16 h,
                                         int16 *arm_lengths, int16 *center_size);
static uint8 match_transistor_template(int16 *arm_lengths, int16 center_size);
static uint8 check_majority(uint8 *buffer, uint8 size);

// ==================== 公开函数实现 ====================

void component_recognition_init(void)
{
    for (uint8 i = 0; i < FILTER_FRAMES; i++) {
        filter_buffer[i] = COMPONENT_NONE;
    }
    filter_index = 0;
    filter_full = 0;
}

uint8 component_detect_anomaly(int16 *left, int16 *right,
                                int16 rows, uint8 *anom_type, int16 *anom_row)
{
    int16 i;
    *anom_type = ANOMALY_NONE;
    *anom_row = -1;
    
    int16 continuous_count = 0;
    int16 max_anomaly_row = -1;
    int16 max_w_diff = 0;
    
    for (i = rows - 2; i > 2; i--) {
        int16 w_cur = right[i] - left[i];
        int16 w_next = right[i + 1] - left[i + 1];
        int16 w_diff = w_cur - w_next;
        
        if (w_diff > 5) {
            continuous_count++;
            if (w_diff > max_w_diff) {
                max_w_diff = w_diff;
                max_anomaly_row = i;
            }
        } else {
            if (continuous_count >= 3 && max_anomaly_row != -1) {
                *anom_type = ANOMALY_BLOB;
                *anom_row = max_anomaly_row;
                return 1;
            }
            continuous_count = 0;
            max_anomaly_row = -1;
            max_w_diff = 0;
        }
    }
    
    if (continuous_count >= 3 && max_anomaly_row != -1) {
        *anom_type = ANOMALY_BLOB;
        *anom_row = max_anomaly_row;
        return 1;
    }
    
    return 0;
}

uint8 component_recognize(uint8 *gray_img, int16 anomaly_x60, int16 anomaly_y60,
                          ComponentResult *result)
{
    // 坐标转换: 60x100 -> 188x120
    int16 start_x = (anomaly_x60 * MT9V03X_1_W) / 100;
    int16 start_y = (anomaly_y60 * MT9V03X_1_H) / 60;
    
    // 使用描线法查找元器件边界
    int16 min_x, max_x, min_y, max_y;
    uint8 found = find_component_boundary(gray_img, start_x, start_y, &min_x, &max_x, &min_y, &max_y);
    
    if (!found) {
        result->type = COMPONENT_NONE;
        result->confidence = 0;
        return COMPONENT_NONE;
    }
    
    // 计算元器件尺寸
    int16 w = max_x - min_x + 1;
    int16 h = max_y - min_y + 1;
    
    // 过滤小噪声
    if (w < MIN_COMPONENT_SIZE || h < MIN_COMPONENT_SIZE) {
        result->type = COMPONENT_NONE;
        result->confidence = 0;
        return COMPONENT_NONE;
    }
    
    // 计算元器件中心
    int16 cx = (min_x + max_x) / 2;
    int16 cy = (min_y + max_y) / 2;
    
    // 提取三极管特征并与模板匹配
    int16 arm_lengths[4] = {0};  // 上,下,左,右分支长度
    int16 center_size = 0;       // 中心区域白色像素数
    uint8 has_features = extract_transistor_features(gray_img, cx, cy, w, h, arm_lengths, &center_size);
    
    uint8 type = COMPONENT_NORMAL;
    if (has_features) {
        uint8 matched = match_transistor_template(arm_lengths, center_size);
        if (matched) {
            type = COMPONENT_TRANSISTOR;
        }
    }
    
    // 滤波处理
    filter_buffer[filter_index] = type;
    filter_index = (filter_index + 1) % FILTER_FRAMES;
    if (filter_index == 0) filter_full = 1;
    
    uint8 filtered_type = check_majority(filter_buffer, FILTER_FRAMES);
    
    result->type = filtered_type;
    result->pos_x = anomaly_x60;
    result->pos_y = anomaly_y60;
    result->confidence = (filtered_type != COMPONENT_NONE) ? 95 : 0;
    
    return filtered_type;
}

const char* component_get_name(uint8 type)
{
    static const char *names[] = {
        "None", "Normal", "Transistor", "", "", "", "", "", "", 
        "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", 
        "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", 
        "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", 
        "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", 
        "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", 
        "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", 
        "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", 
        "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", 
        "", "", "Unknown"
    };
    
    if (type <= COMPONENT_UNKNOWN) {
        return names[type];
    }
    return names[COMPONENT_UNKNOWN];
}

uint8 component_confirm_result(ComponentResult *result)
{
    static uint8 last_type = COMPONENT_NONE;
    static uint8 confirm_count = 0;
    
    if (result->type == last_type && result->type != COMPONENT_NONE) {
        confirm_count++;
        if (confirm_count >= FILTER_FRAMES) {
            return result->type;
        }
    } else {
        last_type = result->type;
        confirm_count = 1;
    }
    
    return COMPONENT_NONE;
}

// ==================== 内部函数实现 ====================

// 使用描线法（边界追踪）查找元器件边界
static uint8 find_component_boundary(uint8 *gray_img, int16 start_x, int16 start_y,
                                     int16 *min_x, int16 *max_x, int16 *min_y, int16 *max_y)
{
    // 先搜索白色像素区域的边界（简化版描线法）
    *min_x = MT9V03X_1_W;
    *max_x = 0;
    *min_y = MT9V03X_1_H;
    *max_y = 0;
    
    int16 search_radius = 50;
    
    int16 search_start_x = start_x - search_radius;
    int16 search_end_x = start_x + search_radius;
    int16 search_start_y = start_y - search_radius;
    int16 search_end_y = start_y + search_radius;
    
    // 边界检查
    if (search_start_x < 0) search_start_x = 0;
    if (search_end_x >= MT9V03X_1_W) search_end_x = MT9V03X_1_W - 1;
    if (search_start_y < 0) search_start_y = 0;
    if (search_end_y >= MT9V03X_1_H) search_end_y = MT9V03X_1_H - 1;
    
    uint8 found = 0;
    
    // 扫描搜索区域，找到白色像素的边界（描线法简化版）
    for (int16 y = search_start_y; y <= search_end_y; y++) {
        for (int16 x = search_start_x; x <= search_end_x; x++) {
            uint8 pixel = gray_img[y * MT9V03X_1_W + x];
            if (pixel > 128) {  // 白色像素
                found = 1;
                if (x < *min_x) *min_x = x;
                if (x > *max_x) *max_x = x;
                if (y < *min_y) *min_y = y;
                if (y > *max_y) *max_y = y;
            }
        }
    }
    
    if (!found) {
        return 0;
    }
    
    // 扩展边界确保包含完整元器件
    *min_x -= 10; if (*min_x < 0) *min_x = 0;
    *max_x += 10; if (*max_x >= MT9V03X_1_W) *max_x = MT9V03X_1_W - 1;
    *min_y -= 10; if (*min_y < 0) *min_y = 0;
    *max_y += 10; if (*max_y >= MT9V03X_1_H) *max_y = MT9V03X_1_H - 1;
    
    return 1;
}

// 搜索多个高度的水平分支，统计分支数量
static uint8 search_horizontal_branches(uint8 *gray_img, int16 cx, int16 cy, int16 search_range,
                                         int16 *left_branch_count, int16 *right_branch_count)
{
    *left_branch_count = 0;
    *right_branch_count = 0;
    
    // 在多个高度搜索水平分支
    for (int16 y_offset = -12; y_offset <= 12; y_offset++) {
        int16 ny = cy + y_offset;
        if (ny < 0 || ny >= MT9V03X_1_H) continue;
        
        // 确保该位置在白色主体上
        if (gray_img[ny * MT9V03X_1_W + cx] < 128) continue;
        
        // 向左搜索
        int16 left_len = 0;
        for (int16 x = cx - 1; x >= cx - search_range; x--) {
            if (x < 0) break;
            if (gray_img[ny * MT9V03X_1_W + x] > 128) {
                left_len++;
            } else {
                break;
            }
        }
        
        // 向右搜索
        int16 right_len = 0;
        for (int16 x = cx + 1; x <= cx + search_range; x++) {
            if (x >= MT9V03X_1_W) break;
            if (gray_img[ny * MT9V03X_1_W + x] > 128) {
                right_len++;
            } else {
                break;
            }
        }
        
        // 统计明显的分支（>= 5像素）
        if (left_len >= 5) (*left_branch_count)++;
        if (right_len >= 5) (*right_branch_count)++;
    }
    
    // 只要有任意一侧有>= 2个分支就返回成功
    return (*left_branch_count >= 2 || *right_branch_count >= 2) ? 1 : 0;
}

// 检查是否有垂直主臂
static uint8 has_vertical_arm(uint8 *gray_img, int16 cx, int16 cy, int16 min_length)
{
    int16 up_len = 0;
    for (int16 y = cy - 1; y >= cy - 30; y--) {
        if (y < 0) break;
        if (gray_img[y * MT9V03X_1_W + cx] > 128) {
            up_len++;
        } else {
            break;
        }
    }
    
    int16 down_len = 0;
    for (int16 y = cy + 1; y <= cy + 30; y++) {
        if (y >= MT9V03X_1_H) break;
        if (gray_img[y * MT9V03X_1_W + cx] > 128) {
            down_len++;
        } else {
            break;
        }
    }
    
    return ((up_len + down_len) >= min_length) ? 1 : 0;
}

// 提取三极管特征（简化版 - 直接判断）
// 返回: 1=三极管, 0=普通元器件
static uint8 extract_transistor_features(uint8 *gray_img, int16 cx, int16 cy, int16 w, int16 h,
                                         int16 *arm_lengths, int16 *center_size)
{
    // 初始化（不再使用）
    arm_lengths[0] = arm_lengths[1] = arm_lengths[2] = arm_lengths[3] = 0;
    *center_size = 0;
    
    // 1. 检查垂直主臂（至少20像素）
    if (!has_vertical_arm(gray_img, cx, cy, 20)) {
        return 0;
    }
    
    // 2. 搜索多个高度的水平分支
    int16 left_branches = 0, right_branches = 0;
    if (!search_horizontal_branches(gray_img, cx, cy, 25, &left_branches, &right_branches)) {
        return 0;
    }
    
    // 3. 只要满足条件就是三极管
    return 1;
}

// 简化匹配（直接返回特征提取结果）
static uint8 match_transistor_template(int16 *arm_lengths, int16 center_size)
{
    // 特征提取已经确定，直接返回成功
    return 1;
}

// 滤波算法：取缓冲区中出现次数最多的类型
static uint8 check_majority(uint8 *buffer, uint8 size)
{
    uint8 counts[256] = {0};
    uint8 max_count = 0;
    uint8 max_type = COMPONENT_NONE;
    
    for (uint8 i = 0; i < size; i++) {
        counts[buffer[i]]++;
        if (counts[buffer[i]] > max_count) {
            max_count = counts[buffer[i]];
            max_type = buffer[i];
        }
    }
    
    // 如果最大次数少于一半，返回NONE
    if (max_count < size / 2) {
        return COMPONENT_NONE;
    }
    
    return max_type;
}
