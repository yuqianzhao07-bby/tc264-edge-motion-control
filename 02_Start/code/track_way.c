#include "zf_common_headfile.h"
#include "image.h"
#include "track_way.h"

#pragma section all "cpu0_dsram"

int16 left_border[image_h];
int16 right_border[image_h];
int16 center_line[image_h];
int16 track_error = 0;
int16 track_angle = 0;

// 元素类型
uint8 track_element_type = ELEMENT_NONE;
int16 track_element_strength = 0;
int8 track_right_angle_dir = 0;     // +1=右直角, -1=左直角
int8 track_tjunction_dir  = 0;      // +1=右侧支路(T形), -1=左侧支路(T形)

// 丢线记忆
static int16 mem_lw = -1;
static int16 mem_rw = -1;
static int16 mem_mid = -1;
static int16 mem_dx = 0;
static uint8 lose_line_count = 0;

// 赛道宽度平滑 (初始12≈2×6px赛道线, 快速收敛到真实值, 避免40导致早期单边补线偏差~17px)
static int16 smooth_rw_global = 12;

// 上一帧中线变化量 (用于弯道外推)
static int16 last_curve_dx = 0;

// ==================== 弯道回正相关 ====================

AlignState track_align_state = ALIGN_NONE;
uint8 track_align_count = 0;
int16 track_target_center = image_w / 2;

// 历史元素类型 (用于检测弯道进入/退出)
static uint8 last_element_type = ELEMENT_NONE;

// 弯道累积偏差
static int16 curve_accum_error = 0;

// ALIGN_FORCE持久偏差 (跨帧累积, 供PID直接使用)
int16 track_force_bias = 0;

// 检测到弯道时的初始中线位置
static int16 curve_entry_center = image_w / 2;

// 连续弯道检测变量
static uint8 continuous_curve_count = 0;      // 连续弯道计数
static int32 accumulated_curve_angle = 0;     // 累积转弯角度
static int16 last_curve_center = image_w / 2; // 上一帧弯道中线
static uint8 is_continuous_curve = 0;         // 是否连续弯道

// ==========================================
// 补线函数: 从(x1,y1)到(x2,y2)画虚拟中线
// 借鉴陕科大 Add_Line, 用直线覆盖center_line
// ==========================================
void track_add_line(int16 x1, int16 y1, int16 x2, int16 y2)
{
    int16 i, temp;
    int16 hx;

    // 限幅
    if (x1 >= image_w - 1) x1 = image_w - 1;
    if (x1 <= 0) x1 = 0;
    if (y1 >= image_h - 1) y1 = image_h - 1;
    if (y1 <= 0) y1 = 0;
    if (x2 >= image_w - 1) x2 = image_w - 1;
    if (x2 <= 0) x2 = 0;
    if (y2 >= image_h - 1) y2 = image_h - 1;
    if (y2 <= 0) y2 = 0;

    // 确保从上往下绘制 (y1 < y2)
    if (y1 > y2)
    {
        temp = x1; x1 = x2; x2 = temp;
        temp = y1; y1 = y2; y2 = temp;
    }

    // 按斜率补线
    if (y2 != y1)
    {
        for (i = y1; i <= y2; i++)
        {
            hx = (int16)((int32)(i - y1) * (x2 - x1) / (y2 - y1) + x1);
            if (hx >= image_w) hx = image_w - 1;
            if (hx <= 0) hx = 0;
            center_line[i] = hx;
        }
    }
}

// ==========================================
// 边缘特征检测 (双区域搜索 + 中线偏移)
// 
// 核心思路:
// 1. 内区(最边缘8列): 只有直角弯才有纵向跳变 → 区分直角/钝角
// 2. 外区(最边缘25列): 直角弯和钝角弯都有跳变
// 3. 中线偏移: 用basic_line_tracking的结果判断弯道方向
// 4. 底部横向扫线: 获取底部中线坐标
// ==========================================
typedef struct {
    // 内区标志 (直角弯特征)
    uint8 inner_left_found;     // 内区左边缘找到
    uint8 inner_right_found;    // 内区右边缘找到
    int16 inner_left_start_y;   // 内区左边缘起始行
    int16 inner_right_start_y;  // 内区右边缘起始行
    
    // 外区标志 (钝角弯特征)
    uint8 outer_left_found;     // 外区左边缘找到
    uint8 outer_right_found;    // 外区右边缘找到
    
    // 顶部/底部横向扫线
    uint8 top_found;            // 顶部找到引导线
    uint8 bottom_found;         // 底部找到引导线
    int16 top_mid_x;            // 顶部中线x坐标
    int16 bottom_mid_x;         // 底部中线x坐标
    int16 bottom_mid_y;         // 底部中线y坐标
    
    // 中线偏移 (来自basic_line_tracking)
    int16 center_offset;        // 底部中线偏移(正=偏右, 负=偏左)
    int16 center_offset_top;    // 顶部中线偏移
} ElementFeature;

static void detect_element_features(ElementFeature *feat)
{
    int16 i, j;

    // 初始化
    feat->inner_left_found = 0;
    feat->inner_right_found = 0;
    feat->inner_left_start_y = 0;
    feat->inner_right_start_y = 0;
    feat->outer_left_found = 0;
    feat->outer_right_found = 0;
    feat->top_found = 0;
    feat->bottom_found = 0;
    feat->top_mid_x = image_w / 2;
    feat->bottom_mid_x = image_w / 2;
    feat->bottom_mid_y = image_h - 1;
    feat->center_offset = 0;
    feat->center_offset_top = 0;

    // === 1. 顶部横向扫线 ===
    int16 top_change_pos[4] = {0, 0, 0, 0};
    for (i = 0; i < TOP_SEARCH_ROWS; i++)
    {
        uint8 top_change = 0;
        for (j = 0; j < image_w - 2; j++)
        {
            if (bin_image[i][j] != bin_image[i][j + 1])
            {
                if (top_change < 4)
                    top_change_pos[top_change] = j;
                top_change++;
            }
        }
        if (top_change >= 2)
        {
            feat->top_found = 1;
            feat->top_mid_x = (top_change_pos[0] + top_change_pos[1]) / 2;
            break;
        }
    }

    // === 2. 底部横向扫线 ===
    int16 bottom_change_pos[4] = {0, 0, 0, 0};
    for (i = image_h - 1; i >= image_h - BOTTOM_SEARCH_ROWS && i >= 0; i--)
    {
        uint8 bottom_change = 0;
        for (j = 0; j < image_w - 2; j++)
        {
            if (bin_image[i][j] != bin_image[i][j + 1])
            {
                if (bottom_change < 4)
                    bottom_change_pos[bottom_change] = j;
                bottom_change++;
            }
        }
        if (bottom_change >= 2)
        {
            feat->bottom_found = 1;
            feat->bottom_mid_x = (bottom_change_pos[0] + bottom_change_pos[1]) / 2;
            feat->bottom_mid_y = i;
            break;
        }
    }

    // === 3. 用basic_line_tracking的center_line计算中线偏移 ===
    // 这是关键! basic_line_tracking已经算好了center_line, 直接用它
    // 底部几行的中线偏移
    int16 bottom_center_sum = 0;
    int16 bottom_count = 0;
    for (i = image_h - 1; i >= image_h - 5 && i >= 0; i--)
    {
        bottom_center_sum += center_line[i];
        bottom_count++;
    }
    if (bottom_count > 0)
        feat->center_offset = bottom_center_sum / bottom_count - image_w / 2;
    
    // 顶部几行的中线偏移
    int16 top_center_sum = 0;
    int16 top_count = 0;
    for (i = 0; i < 5 && i < image_h; i++)
    {
        top_center_sum += center_line[i];
        top_count++;
    }
    if (top_count > 0)
        feat->center_offset_top = top_center_sum / top_count - image_w / 2;

    // === 4. 内区纵向扫线 (最边缘8列) - 直角弯特征 ===
    // 只有直角弯才会在最边缘有纵向黑→白跳变
    int16 inner_left_change = 0;
    int16 inner_right_change = 0;
    
    for (j = 0; j < EDGE_SEARCH_COLS_INNER; j += EDGE_SEARCH_STEP)
    {
        for (i = 0; i < image_h - 1; i++)
        {
            if (bin_image[i][j] == 0 && bin_image[i + 1][j] == 255)
            {
                inner_left_change++;
                if (!feat->inner_left_found)
                    feat->inner_left_start_y = i;
                break;
            }
        }
    }
    
    for (j = image_w - 1; j > image_w - 1 - EDGE_SEARCH_COLS_INNER; j -= EDGE_SEARCH_STEP)
    {
        for (i = 0; i < image_h - 1; i++)
        {
            if (bin_image[i][j] == 0 && bin_image[i + 1][j] == 255)
            {
                inner_right_change++;
                if (!feat->inner_right_found)
                    feat->inner_right_start_y = i;
                break;
            }
        }
    }
    
    if (inner_left_change > RIGHT_ANGLE_CHANGE_MIN)
        feat->inner_left_found = 1;
    if (inner_right_change > RIGHT_ANGLE_CHANGE_MIN)
        feat->inner_right_found = 1;

    // === 5. 外区纵向扫线 (最边缘25列) - 钝角弯特征 ===
    int16 outer_left_change = 0;
    int16 outer_right_change = 0;
    
    for (j = 0; j < EDGE_SEARCH_COLS_OUTER; j += EDGE_SEARCH_STEP)
    {
        for (i = 0; i < image_h - 1; i++)
        {
            if (bin_image[i][j] == 0 && bin_image[i + 1][j] == 255)
            {
                outer_left_change++;
                break;
            }
        }
    }
    
    for (j = image_w - 1; j > image_w - 1 - EDGE_SEARCH_COLS_OUTER; j -= EDGE_SEARCH_STEP)
    {
        for (i = 0; i < image_h - 1; i++)
        {
            if (bin_image[i][j] == 0 && bin_image[i + 1][j] == 255)
            {
                outer_right_change++;
                break;
            }
        }
    }
    
    if (outer_left_change > OBTUSE_CHANGE_MIN)
        feat->outer_left_found = 1;
    if (outer_right_change > OBTUSE_CHANGE_MIN)
        feat->outer_right_found = 1;
}

// ==========================================
// 元素识别 + 补线决策
// 
// 判断优先级:
// 1. 直角弯: 内区有跳变 + 顶部无 → 直角弯
// 2. 十字: 内区两侧都有 + 顶部有 → 十字
// 3. 正常巡线: 顶部有 + 内区无 → 直道
// 4. 钝角弯: 外区有跳变 或 中线偏移大 → 钝角弯
// 5. 断路: 全部无 + 中线偏移小 → 断路
// ==========================================
static void identify_element_and_supplement(ElementFeature *feat)
{
    int16 abs_offset = (feat->center_offset > 0) ? feat->center_offset : -feat->center_offset;
    int16 center_ref = image_w / 2;

    // T→直角弯防抖: 上帧是T_JUNCTION, 本帧若判为同侧直角弯 → 维持T_JUNCTION
    // 原因: 车靠近路口时 top_found 可能丢失, 导致T_JUNCTION误退化为RIGHT_ANGLE
    static uint8  prev_was_tjunction = 0;
    static int8  prev_tjunction_dir = 0;
    static uint8 tjunction_hold_count = 0;

    // R→T防抖: 上帧是RIGHT_ANGLE, 本帧若判为同侧T → 维持RIGHT_ANGLE
    // 原因: 直角弯中摄像头短暂看到顶部线路, top_found闪回导致误升为T_JUNCTION
    static uint8  prev_was_rightangle = 0;
    static int8  prev_rightangle_dir = 0;
    static uint8 rightangle_hold_count = 0;

    // 决策优先级 (从高到低):
    //   直角弯(无顶部) > T形路口(单内区+有顶部) > 十字(双内区+有顶部) > 直道 > 全丢(偏移分级) > 钝角弯

    // === 1. 右直角弯: 内区右边缘有 + 内区左边缘无 + 顶部无 ===
    if (feat->inner_right_found && !feat->inner_left_found && !feat->top_found)
    {
        track_element_type = ELEMENT_RIGHT_ANGLE;
        track_right_angle_dir = 1;
        track_tjunction_dir  = 0;

        int16 bottom_target = image_w - 8;
        track_add_line(image_w - 1, feat->inner_right_start_y,
                       bottom_target, image_h - 1);

        for (int16 k = image_h - 1; k >= image_h - 6 && k >= 0; k--)
        {
            if (center_line[k] < image_w - 10)
                center_line[k] = image_w - 10;
        }

        for (int16 k = feat->inner_right_start_y; k > 0; k--)
        {
            center_line[k] = center_line[k + 1];
        }
    }
    // === 2. 左直角弯: 内区左边缘有 + 内区右边缘无 + 顶部无 ===
    else if (!feat->inner_right_found && feat->inner_left_found && !feat->top_found)
    {
        track_element_type = ELEMENT_RIGHT_ANGLE;
        track_right_angle_dir = -1;
        track_tjunction_dir  = 0;

        int16 bottom_target = 8;
        track_add_line(0, feat->inner_left_start_y,
                       bottom_target, image_h - 1);

        for (int16 k = image_h - 1; k >= image_h - 6 && k >= 0; k--)
        {
            if (center_line[k] > 10)
                center_line[k] = 10;
        }

        for (int16 k = feat->inner_left_start_y; k > 0; k--)
        {
            center_line[k] = center_line[k + 1];
        }
    }
    // === 3. T形路口(右侧支路): 内区右边缘有 + 左无 + 顶部有 ===
    //     关键特征: 单侧内区跳变 + 上方仍然有引导线 → 不是直角弯的封闭端点
    else if (feat->inner_right_found && !feat->inner_left_found && feat->top_found)
    {
        track_element_type = ELEMENT_T_JUNCTION;
        track_tjunction_dir = 1;   // 右侧有支路
        track_right_angle_dir = 0;

        // Layer 2 补线策略: 让车先直行通过路口中心
        // 实际转弯由 Layer 3 (路口遍历/右手定则) 决定
        // 连接顶部中线到底部中线 → 车沿主线直行
        track_add_line(feat->top_mid_x, 0,
                       feat->bottom_mid_x, feat->bottom_mid_y);

        // 在跳变行附近偏支路侧: 基础模式2px, 导航模式8px(引导入支路)
        int16 tj_bias = JUNCTION_NAV_ENABLE ? 8 : 2;
        for (int16 k = feat->inner_right_start_y + 3; k > feat->inner_right_start_y - 3 && k > 0; k--)
        {
            if (k < image_h)
            {
                int16 adjust = center_line[k] + tj_bias;
                if (adjust < image_w - 3)
                    center_line[k] = adjust;
            }
        }
    }
    // === 4. T形路口(左侧支路): 内区左边缘有 + 右无 + 顶部有 ===
    else if (!feat->inner_right_found && feat->inner_left_found && feat->top_found)
    {
        track_element_type = ELEMENT_T_JUNCTION;
        track_tjunction_dir = -1;  // 左侧有支路
        track_right_angle_dir = 0;

        // 直行通过路口中心
        track_add_line(feat->top_mid_x, 0,
                       feat->bottom_mid_x, feat->bottom_mid_y);

        // 在跳变行附近偏支路侧: 基础模式2px, 导航模式8px(引导入支路)
        int16 tj_bias_l = JUNCTION_NAV_ENABLE ? 8 : 2;
        for (int16 k = feat->inner_left_start_y + 3; k > feat->inner_left_start_y - 3 && k > 0; k--)
        {
            if (k < image_h)
            {
                int16 adjust = center_line[k] - tj_bias_l;
                if (adjust > 3)
                    center_line[k] = adjust;
            }
        }
    }
    // === 5. 十字: 两侧内区都有 + 顶部有 ===
    else if (feat->inner_right_found && feat->inner_left_found && feat->top_found)
    {
        track_element_type = ELEMENT_CROSS;
        track_right_angle_dir = 0;
        track_tjunction_dir  = 0;
        track_add_line(feat->top_mid_x, 0,
                       feat->bottom_mid_x, feat->bottom_mid_y);
    }
    // === 6. 正常巡线: 顶部有 + 内区无 ===
    else if (!feat->inner_right_found && !feat->inner_left_found && feat->top_found)
    {
        track_element_type = ELEMENT_NONE;
        track_right_angle_dir = 0;
        track_tjunction_dir  = 0;
        track_add_line(feat->top_mid_x, 0,
                       feat->bottom_mid_x, feat->bottom_mid_y);
    }
    // === 7. 全部标志为0: 用偏移区分弯道/断路 ===
    else if (!feat->inner_right_found && !feat->inner_left_found && !feat->top_found)
    {
        track_right_angle_dir = 0;
        track_tjunction_dir  = 0;

        if (abs_offset > 8)
        {
            if (abs_offset > 20)
            {
                // 偏移大 → 可能是直角弯(内区漏检)
                track_element_type = ELEMENT_RIGHT_ANGLE;
                track_right_angle_dir = (feat->center_offset > 0) ? 1 : -1;

                int16 bottom_target = center_ref + feat->center_offset;
                if (feat->center_offset > 0)
                {
                    if (bottom_target < image_w - 8) bottom_target = image_w - 8;
                }
                else
                {
                    if (bottom_target > 8) bottom_target = 8;
                }
                int16 top_target = center_ref + feat->center_offset * 2;
                if (top_target > image_w - 3) top_target = image_w - 3;
                if (top_target < 3) top_target = 3;
                track_add_line(top_target, 0,
                               bottom_target, image_h - 1);

                if (feat->center_offset > 0)
                {
                    for (int16 k = image_h - 1; k >= image_h - 6 && k >= 0; k--)
                        if (center_line[k] < image_w - 10) center_line[k] = image_w - 10;
                }
                else
                {
                    for (int16 k = image_h - 1; k >= image_h - 6 && k >= 0; k--)
                        if (center_line[k] > 10) center_line[k] = 10;
                }
            }
            else
            {
                track_element_type = ELEMENT_OBTUSE;

                int16 target_x = center_ref + feat->center_offset;
                if (target_x > image_w - 5) target_x = image_w - 5;
                if (target_x < 5) target_x = 5;
                track_add_line(center_line[image_h - 1], image_h - 1,
                               target_x, 0);
            }
        }
        else
        {
            track_element_type = ELEMENT_BREAK;
        }
    }
    // === 8. 其余组合 → 钝角弯(兜底) ===
    else
    {
        track_element_type = ELEMENT_OBTUSE;
        track_right_angle_dir = 0;
        track_tjunction_dir  = 0;

        int16 use_aggressive = 0;
        if (is_continuous_curve || abs_offset > 20)
            use_aggressive = 1;

        if (use_aggressive)
        {
            int16 target_x = feat->top_found ? feat->top_mid_x :
                             (center_ref + feat->center_offset);
            if (target_x < 3) target_x = 3;
            if (target_x > image_w - 3) target_x = image_w - 3;

            track_add_line(center_line[image_h - 1], image_h - 1,
                           target_x, 0);

            int16 extend_dx = feat->center_offset / (image_h > 0 ? image_h : 1);
            if (abs_offset > 25) extend_dx *= 2;

            for (int16 k = 0; k < 10 && k < image_h; k++)
            {
                int16 new_pos = center_line[k] + extend_dx;
                if (new_pos >= 3 && new_pos <= image_w - 3)
                    center_line[k] = new_pos;
            }
        }
        else
        {
            if (feat->top_found)
            {
                track_add_line(center_line[image_h - 1], image_h - 1,
                               feat->top_mid_x, 0);
            }
            else
            {
                int16 target_x = center_ref + feat->center_offset;
                if (target_x < 5) target_x = 5;
                if (target_x > image_w - 5) target_x = image_w - 5;
                track_add_line(center_line[image_h - 1], image_h - 1,
                               target_x, 0);
            }
        }
    }

    // === T→直角弯防抖覆盖 ===
    // 场景: 车靠近T形路口时top_found丢失 → 决策树判为RIGHT_ANGLE
    if (track_element_type == ELEMENT_RIGHT_ANGLE && prev_was_tjunction)
    {
        if ((track_right_angle_dir == 1 && prev_tjunction_dir == 1) ||
            (track_right_angle_dir == -1 && prev_tjunction_dir == -1))
        {
            track_element_type = ELEMENT_T_JUNCTION;
            track_tjunction_dir = prev_tjunction_dir;
            track_right_angle_dir = 0;
            tjunction_hold_count = 8;
        }
    }

    // === R→T防抖覆盖 (对称) ===
    // 场景: 直角弯中摄像头短暂看到顶部线路 → top_found闪回 → 决策树误升为T_JUNCTION
    if (track_element_type == ELEMENT_T_JUNCTION && prev_was_rightangle
        && tjunction_hold_count == 0)  // 不被T→R防抖同时激活时
    {
        if ((track_tjunction_dir == 1 && prev_rightangle_dir == 1) ||
            (track_tjunction_dir == -1 && prev_rightangle_dir == -1))
        {
            track_element_type = ELEMENT_RIGHT_ANGLE;
            track_right_angle_dir = prev_rightangle_dir;
            track_tjunction_dir = 0;
            rightangle_hold_count = 5;  // 维持5帧RIGHT_ANGLE
        }
    }

    // 更新上帧记忆 + 维持计数
    if (track_element_type == ELEMENT_T_JUNCTION)
    {
        prev_was_tjunction = 1;
        prev_tjunction_dir = track_tjunction_dir;
        if (tjunction_hold_count > 0) tjunction_hold_count--;
        // 进入T状态时清除R记忆
        prev_was_rightangle = 0;
        rightangle_hold_count = 0;
    }
    else if (tjunction_hold_count > 0 && track_element_type == ELEMENT_RIGHT_ANGLE)
    {
        track_element_type = ELEMENT_T_JUNCTION;
        track_tjunction_dir = prev_tjunction_dir;
        track_right_angle_dir = 0;
        tjunction_hold_count--;
    }
    else
    {
        prev_was_tjunction = 0;
        prev_tjunction_dir = 0;
        tjunction_hold_count = 0;
    }

    // R状态记忆更新
    if (track_element_type == ELEMENT_RIGHT_ANGLE)
    {
        prev_was_rightangle = 1;
        prev_rightangle_dir = track_right_angle_dir;
        if (rightangle_hold_count > 0) rightangle_hold_count--;
    }
    else if (rightangle_hold_count > 0 && track_element_type == ELEMENT_T_JUNCTION)
    {
        track_element_type = ELEMENT_RIGHT_ANGLE;
        track_right_angle_dir = prev_rightangle_dir;
        track_tjunction_dir = 0;
        rightangle_hold_count--;
    }
    else
    {
        prev_was_rightangle = 0;
        prev_rightangle_dir = 0;
        rightangle_hold_count = 0;
    }
}

// ==========================================
// 基础巡线: 逐行搜索左右边界 + 计算中线
// ==========================================
static void basic_line_tracking(void)
{
    int16 i, j;
    int16 smooth_rw = smooth_rw_global;
    int16 predict_center = image_w / 2;
    int16 last_mid = image_w / 2;
    int16 last_lw = image_w / 2 - 20;
    int16 last_rw = image_w / 2 + 20;
    int16 last_dx = 0;
    uint8 confidence = 0;

    // 帧间记忆: 保存上一帧底部中线位置, 用于本帧搜索起点
    static int16 prev_bottom_mid = -1;
    static int16 prev_bottom_lw = -1;
    static int16 prev_bottom_rw = -1;

    // 初始化
    for (i = 0; i < image_h; i++)
    {
        left_border[i]  = image_w / 2 - 20;
        right_border[i] = image_w / 2 + 20;
        center_line[i]  = image_w / 2;
    }

    lose_line_count = 0;

    // 如果有上一帧的底部中线记忆, 用它作为本帧第一行(last_mid)的搜索起点
    if (prev_bottom_mid >= 5 && prev_bottom_mid <= image_w - 5)
    {
        last_mid = prev_bottom_mid;
        if (prev_bottom_lw >= 0) last_lw = prev_bottom_lw;
        if (prev_bottom_rw >= 0) last_rw = prev_bottom_rw;
        // 急弯/元素切换时降低初始置信度, 扩大搜索窗口防止帧记忆中毒
        if (track_element_type == ELEMENT_RIGHT_ANGLE ||
            track_element_type == ELEMENT_T_JUNCTION)
            confidence = 1;  // 低置信度→宽搜索窗口, 适应急弯大位移
        else
            confidence = 3;  // 直道/钝角弯: 正常置信度
    }

    // 从底部向上扫描
    for (i = image_h - 1; i >= image_h - TRACK_SCAN_ROWS && i >= 0; i--)
    {
        if (i < image_h - 1)
        {
            last_mid = center_line[i + 1];
            last_lw = left_border[i + 1];
            last_rw = right_border[i + 1];
        }

        // === 预测中心 ===
        if (confidence >= 3 && mem_dx != 0)
        {
            predict_center = last_mid + mem_dx;
        }
        else if (confidence >= 1)
        {
            predict_center = last_mid + last_dx / 2;
        }
        else
        {
            predict_center = last_mid;
        }

        if (predict_center < 5) predict_center = 5;
        if (predict_center > image_w - 5) predict_center = image_w - 5;

        // === 搜索范围 ===
        int16 search_half;
        if (confidence >= 5)
            search_half = smooth_rw / 2 + 8;
        else if (confidence >= 3)
            search_half = smooth_rw / 2 + 14;
        else if (confidence >= 1)
            search_half = smooth_rw / 2 + 20;
        else
            search_half = image_w / 2 - 1;  // 低置信度: 全宽搜索

        if (search_half > image_w / 2 - 1) search_half = image_w / 2 - 1;
        if (search_half < 12) search_half = 12;

        int16 search_l = predict_center - search_half;
        int16 search_r = predict_center + search_half;
        if (search_l < 1) search_l = 1;
        if (search_r > image_w - 2) search_r = image_w - 2;

        // === 找左边界 (0→255 跳变, 即黑到白) ===
        int16 lw = -1;
        for (j = search_l; j < predict_center && j < image_w - 1; j++)
        {
            if (bin_image[i][j] == 0 && bin_image[i][j + 1] == 255)
            {
                lw = j + 1;
                break;
            }
        }
        // 回退搜索
        if (lw < 0)
        {
            for (j = predict_center - 1; j > search_l; j--)
            {
                if (bin_image[i][j] == 255 && bin_image[i][j - 1] == 0)
                {
                    lw = j;
                    break;
                }
            }
        }

        // === 找右边界 (255→0 跳变, 即白到黑) ===
        int16 rw = -1;
        for (j = search_r; j > predict_center && j > 0; j--)
        {
            if (bin_image[i][j] == 0 && bin_image[i][j - 1] == 255)
            {
                rw = j - 1;
                break;
            }
        }
        // 回退搜索
        if (rw < 0)
        {
            for (j = predict_center + 1; j < search_r; j++)
            {
                if (bin_image[i][j] == 255 && bin_image[i][j + 1] == 0)
                {
                    rw = j;
                    break;
                }
            }
        }

        // 记录边界
        left_border[i]  = (lw >= 0) ? lw : last_lw;
        right_border[i] = (rw >= 0) ? rw : last_rw;

        uint8 found = 0;
        if (lw >= 0) found |= 2;
        if (rw >= 0) found |= 1;

        // === 计算中线 ===
        if (found == 3)  // 找到两边边界
        {
            int16 w = rw - lw;
            if (w >= 4 && w <= 50)  // 赛道线~6px, 平行支路间距58px(>50), 严格排除双线误合并
            {
                smooth_rw = (smooth_rw * 4 + w) / 5;
                int16 new_mid = (lw + rw) / 2;

                // 轻滤波: 只做2帧平均, 减少延迟
                int16 smooth_dx = (new_mid - last_mid + last_dx) / 2;

                center_line[i] = last_mid + smooth_dx;
                last_dx = smooth_dx;

                mem_lw = lw;
                mem_rw = rw;
                mem_mid = center_line[i];
                mem_dx = smooth_dx;

                confidence = (confidence < 10) ? confidence + 1 : 10;
                lose_line_count = 0;
            }
            else
            {
                // 宽度异常: 可能看到平行支路或干扰, 外推保持方向
                center_line[i] = last_mid + last_dx;
                confidence = (confidence > 2) ? confidence - 2 : 0;
                lose_line_count = 0;  // 找到了边界(虽宽度异常), 重置丢线计数
            }
        }
        else if (found & 2)  // 只找到左边界
        {
            // 补右边界: 右边界 = 左边界 + 赛道宽度
            right_border[i] = lw + smooth_rw;
            if (right_border[i] > image_w - 1) right_border[i] = image_w - 1;
            center_line[i] = lw + smooth_rw / 2;
            mem_lw = lw;
            last_dx = (center_line[i] - last_mid + last_dx) / 2;
            confidence = (confidence > 0) ? confidence - 1 : 0;
            lose_line_count = 0;
        }
        else if (found & 1)  // 只找到右边界
        {
            // 补左边界: 左边界 = 右边界 - 赛道宽度
            left_border[i] = rw - smooth_rw;
            if (left_border[i] < 0) left_border[i] = 0;
            center_line[i] = rw - smooth_rw / 2;
            mem_rw = rw;
            last_dx = (center_line[i] - last_mid + last_dx) / 2;
            confidence = (confidence > 0) ? confidence - 1 : 0;
            lose_line_count = 0;
        }
        else  // 全丢
        {
            // 丢线策略: 用已知方向外推
            if (mem_mid >= 0 && lose_line_count < 30)
            {
                // 短期丢线(0-29行): 沿记忆方向慢衰减外推
                if (track_element_type == ELEMENT_RIGHT_ANGLE ||
                    (JUNCTION_NAV_ENABLE && track_element_type == ELEMENT_T_JUNCTION))
                {
                    // 直角弯/T形路口(导航启用时): 极慢衰减(5%/行), 防止高速过路口丢线
                    center_line[i] = last_mid + last_dx;
                    mem_dx = (int16)(mem_dx * 19 / 20);
                    last_dx = mem_dx;
                }
                else if (lose_line_count < 20)
                {
                    // 0-19行: 标准慢衰减
                    center_line[i] = last_mid + last_dx;
                    mem_dx = (int16)(mem_dx * 9 / 10);   // 10%/行衰减
                    last_dx = mem_dx;
                }
                else
                {
                    // 20-29行: 中等衰减, 平滑过渡避免断崖
                    center_line[i] = last_mid + last_dx;
                    mem_dx = (int16)(mem_dx * 17 / 20);  // 15%/行衰减, 介于慢/快之间
                    last_dx = mem_dx;
                }
            }
            else
            {
                // 长期丢线(≥30行): 保持最后方向, 不跳回中心
                center_line[i] = last_mid + last_dx;
                mem_dx = (int16)(mem_dx * 3 / 4);
                last_dx = mem_dx;
            }

            lose_line_count++;
            confidence = 0;
        }

        // 限幅
        if (center_line[i] > image_w - 5) center_line[i] = image_w - 5;
        if (center_line[i] < 5) center_line[i] = 5;

        last_mid = center_line[i];
    }

    // 保存全局赛道宽度
    smooth_rw_global = smooth_rw;

    // 保存最后一行的变化量
    last_curve_dx = last_dx;

    // 保存底部中线/边界供下一帧搜索起点 (帧间记忆)
    prev_bottom_mid = center_line[image_h - 1];
    prev_bottom_lw = left_border[image_h - 1];
    prev_bottom_rw = right_border[image_h - 1];
}

// ==========================================
// 弯道回正状态更新
// ==========================================
static void update_align_state(void)
{
    // 弯道退出检测的持久状态 (提升到函数作用域, 以便在状态重置时清除)
    static int16 last_bottom_center = 0;
    static uint8 align_first_time = 1;

    int16 center_ref = image_w / 2;

    // 获取当前底部中线偏差
    int16 current_error = center_line[image_h - 1] - center_ref;
    
    // ==================== 连续弯道检测 ====================
    if (track_element_type == ELEMENT_OBTUSE)
    {
        continuous_curve_count++;
        
        // 计算当前帧的转弯角度贡献
        int16 center_change = center_line[image_h - 1] - last_curve_center;
        if (abs(center_change) > 1)
        {
            // 累积转弯角度（近似值）
            if (center_change > 0)
                accumulated_curve_angle += 2;  // 每像素约2度
            else
                accumulated_curve_angle -= 2;
        }
        
        // 检测是否连续弯道
        if (continuous_curve_count > CONTINUOUS_CURVE_THRESHOLD)
        {
            is_continuous_curve = 1;
        }
        
        last_curve_center = center_line[image_h - 1];
    }
    else
    {
        // 不是弯道，逐渐衰减连续弯道标志
        if (continuous_curve_count > 0)
            continuous_curve_count--;
        
        if (continuous_curve_count == 0)
        {
            is_continuous_curve = 0;
            accumulated_curve_angle = 0;
        }
    }
    
    // ==================== 弯道状态管理 ====================
    
    // 检测弯道进入
    if (track_element_type == ELEMENT_OBTUSE && last_element_type != ELEMENT_OBTUSE)
    {
        // 进入弯道, 记录初始中线位置
        curve_entry_center = center_line[image_h - 1];
        curve_accum_error = 0;
        track_align_state = ALIGN_CURVE_IN;
        track_align_count = 0;
    }
    
    // 弯道中累积偏差
    if (track_align_state == ALIGN_CURVE_IN)
    {
        curve_accum_error += abs(current_error);
        track_align_count++;
        
        // 弯道中检测是否即将退出
        if (track_align_count > 15)  // 弯道持续一定帧数后开始检测退出
        {
            // 检查中线是否开始反向变化 (弯道出口特征)
            // 首次进入时初始化
            if (align_first_time)
            {
                last_bottom_center = center_ref;
                align_first_time = 0;
            }
            
            int16 center_diff = center_line[image_h - 1] - last_bottom_center;
            
            // 改进的退出检测：
            // 1. 偏差变小且接近中线 OR
            // 2. 检测到元素类型变为直道
            uint8 exit_condition = 0;
            
            // 条件1: 偏差足够小且变化稳定
            if (abs(current_error) < ALIGN_REQUIRED_THRES + 5 && abs(center_diff) < 4)
            {
                exit_condition = 1;
            }
            
            // 条件2: 如果还有连续弯道标志，不退出
            if (is_continuous_curve)
            {
                exit_condition = 0;
            }
            
            if (exit_condition)
            {
                track_align_state = ALIGN_CURVE_EXIT;
                track_align_count = 0;
                // 设置目标中线为图像中心
                track_target_center = center_ref;
                align_first_time = 1;  // 重置首次标志, 下次弯道重新初始化
            }
            last_bottom_center = center_line[image_h - 1];
        }
    }
    
    // ==================== 弯道出口回正 ====================
    if (track_align_state == ALIGN_CURVE_EXIT)
    {
        // 检查是否已经对齐
        if (abs(current_error) < ALIGN_REQUIRED_THRES)
        {
            track_align_count++;
            if (track_align_count >= ALIGN_STABLE_COUNT)
            {
                // 稳定对齐，退出回正模式
                track_align_state = ALIGN_NONE;
                track_align_count = 0;
                curve_accum_error = 0;
                track_force_bias = 0;  // 清除强制回正偏差
                align_first_time = 1;  // 重置, 下次弯道入口重新初始化
            }
        }
        else
        {
            // 还未对齐，继续回正
            track_align_count = 0;
            
            // 对于大偏差，强制回正 - 提高阈值避免误触发
            if (abs(current_error) > ALIGN_REQUIRED_THRES * 3)
            {
                track_align_state = ALIGN_FORCE;
            }
        }
    }
    
    // ==================== 强制回正 ====================
    if (track_align_state == ALIGN_FORCE)
    {
        // 强制修改底部若干行的中线, 引导回正
        int16 pull_strength = 6;  // 每帧拉回力度(增, 避免单帧修正被下一帧basic_line_tracking覆盖)
        int16 pull_rows = 12;     // 拉回行数(增, 扩大影响范围)

        for (int16 i = image_h - 1; i >= image_h - pull_rows && i >= 0; i--)
        {
            int16 error = center_line[i] - center_ref;
            // 渐变力度: 越靠近底部(车)力度越大, 产生真正梯度
            int16 row_weight = (image_h - i) * pull_strength / pull_rows;
            if (row_weight < 1) row_weight = 1;

            if (error > row_weight)
                center_line[i] -= row_weight;
            else if (error < -row_weight)
                center_line[i] += row_weight;
            else
                center_line[i] = center_ref;
        }

        // 持久偏差: 跨帧累积, 即使center_line被basic_line_tracking覆盖也持续作用
        // 缓慢衰减避免无限累积
        track_force_bias = track_force_bias * 7 / 8 + current_error / 4;
        if (track_force_bias > 40)  track_force_bias = 40;
        if (track_force_bias < -40) track_force_bias = -40;

        // 检查是否回正完成
        if (abs(current_error) < ALIGN_REQUIRED_THRES)
        {
            track_align_count++;
            if (track_align_count >= ALIGN_STABLE_COUNT)
            {
                track_align_state = ALIGN_NONE;
                track_align_count = 0;
                track_force_bias = 0;  // 清除持久偏差
                align_first_time = 1;  // 重置, 下次弯道入口重新初始化
            }
        }
        else
        {
            track_align_count = 0;
        }
    }
    
    // 直道状态下的自动微调
    if (track_align_state == ALIGN_NONE && track_element_type == ELEMENT_NONE)
    {
        // 检查是否需要轻微回正
        if (abs(current_error) > ALIGN_REQUIRED_THRES * 3)
        {
            // 大偏差时启动缓慢回正
            track_align_state = ALIGN_CURVE_EXIT;
            track_target_center = center_ref;
            track_align_count = 0;
        }
    }
    
    // 更新历史元素类型
    last_element_type = track_element_type;
}

// ==========================================
// 主巡线函数: 先基础巡线, 再元素检测+补线
// ==========================================
void track_find_line(void)
{
    // 第一步: 基础巡线 (逐行搜索边界+中线)
    basic_line_tracking();

    // 第二步: 边缘特征检测
    ElementFeature feat;
    detect_element_features(&feat);

    // 第三步: 元素识别 + 补线 (覆盖巡线的中线)
    identify_element_and_supplement(&feat);

    // 第四步: 计算元素强度
    if (track_element_type != ELEMENT_NONE)
    {
        int16 center_ref = image_w / 2;
        int16 max_offset = 0;
        for (int16 i = image_h - 1; i >= image_h - 16 && i >= 0; i--)
        {
            int16 offset = center_line[i] - center_ref;
            int16 abs_offset = (offset > 0) ? offset : -offset;
            if (abs_offset > max_offset) max_offset = abs_offset;
        }
        track_element_strength = max_offset * 2;
        if (track_element_strength > 100) track_element_strength = 100;
    }
    else
    {
        track_element_strength = 0;
    }
    
    // 第五步: 更新弯道回正状态
    update_align_state();

    // 第六步: 元器件识别 (由 cpu0_main.c 中的 component_recognition 模块处理)
    // 基础循迹阶段已禁用, 恢复路径遍历后再启用
}

// ==========================================
// 检查是否需要回正
// ==========================================
uint8 track_need_align(void)
{
    return (track_align_state == ALIGN_CURVE_EXIT || 
            track_align_state == ALIGN_FORCE);
}

// ==========================================
// 获取回正状态
// ==========================================
AlignState track_get_align_state(void)
{
    return track_align_state;
}

// ==========================================
// 检查是否连续弯道
// ==========================================
uint8 track_is_continuous_curve(void)
{
    return is_continuous_curve;
}

// ==========================================
// 偏差计算 (加权平均, 无惯性阻尼)
// ==========================================
void track_get_error(void)
{
    int32 error_sum = 0;
    int32 weight_sum = 0;
    int16 center_ref = image_w / 2;

    // 根据元素类型选择偏差计算行数
    int16 rows;
    if (track_element_type == ELEMENT_RIGHT_ANGLE)
        rows = 15;   // 直角弯: 多看几行, 确保转向持续输出
    else if (track_element_type == ELEMENT_T_JUNCTION)
        rows = 15;   // T形路口: 多看几行, 为Layer 3转弯准备
    else if (track_element_type == ELEMENT_OBTUSE)
        rows = 10;
    else if (track_element_type == ELEMENT_CROSS)
        rows = 10;
    else
        rows = 10;   // 直道/断路

    int16 start_row = image_h - 1;
    int16 end_row   = image_h - rows;
    if (end_row < image_h - TRACK_SCAN_ROWS)
        end_row = image_h - TRACK_SCAN_ROWS;
    if (end_row < 0) end_row = 0;

    // 加权平均 (越靠近底部权重越大: 底部行=10, 远处行=1)
    // 注: start_row=59(底部), end_row=50(直道), i从59→50递减
    //     i=59时 w=59-50+1=10(最大), i=50时 w=50-50+1=1(最小)
    for (int16 i = start_row; i >= end_row; i--)
    {
        int32 w = i - end_row + 1;
        error_sum += (center_line[i] - center_ref) * w;
        weight_sum += w;
    }

    if (weight_sum > 0)
        track_error = (int16)(error_sum / weight_sum);
    else
        track_error = 0;

    // === 航线角度计算 (供角度PID使用) ===
    // 方法: 取底部区域和中上区域的中线均值, 计算斜率
    // 正值=轨道向左偏(需左转), 负值=向右偏(需右转) — 与track_error符号一致
    {
        int32 bot_sum = 0, mid_sum = 0;
        int16 bot_count = 0, mid_count = 0;
        int16 i;

        // 底部区域: 行54-59 (最靠近车身的6行)
        for (i = image_h - 1; i >= image_h - 6 && i >= 0; i--)
        {
            bot_sum += center_line[i];
            bot_count++;
        }
        // 中上区域: 行45-50 (前方约10-15行的位置)
        for (i = image_h - 12; i >= image_h - 17 && i >= TRACK_SCAN_ROWS; i--)
        {
            mid_sum += center_line[i];
            mid_count++;
        }

        if (bot_count > 0 && mid_count > 0)
        {
            int16 bot_avg = (int16)(bot_sum / bot_count);
            int16 mid_avg = (int16)(mid_sum / mid_count);
            // angle = mid - bot → 正值=前方在左, 底部在右 → 轨道偏左 → 需左转
            track_angle = mid_avg - bot_avg;
        }
        else
        {
            track_angle = 0;
        }
    }
}

// ==========================================
// Layer 3: 路口遍历 (右手法则)
// 四阶段: 注册 → 决策 → 转弯 → 回溯
// ==========================================

// 右手法则决策优先级: 右 > 直 > 左
const uint8 dir_priority[DIR_PRIORITY_COUNT] = { DIR_RIGHT, DIR_STRAIGHT, DIR_LEFT };

// 全局路口遍历状态
JunctionRecord g_junctions[MAX_JUNCTIONS];
uint8 g_junction_count = 0;
uint8 g_current_junction_idx = 0;
JunctionState g_junction_state = JSTATE_IDLE;
uint8 g_chosen_direction = 0;
int32 g_junction_distance = 0;
int16 g_junction_turn_count = 0;
uint8 g_backtrack_total = 0;
uint8 g_exploration_complete = 0;  // 1=全部路口已探索完毕, 禁止重新进入

// 路口接近检测缓冲 (防抖)
static uint8 junction_detect_buffer = 0;
static uint8 junction_detect_count = 0;
#define JUNCTION_DETECT_FRAMES  3   // 连续3帧检测到才确认

// 路口完成后冷却计数 (防重复注册)
static int16 junction_cooldown = 0;
#define JUNCTION_COOLDOWN_FRAMES 30  // 完成后30帧内不检测新路口

// 转弯退出防抖 (文件级, 避免跨路口残留)
static uint8 turn_exit_debounce = 0;

void junction_init(void)
{
    uint8 i;
    for (i = 0; i < MAX_JUNCTIONS; i++)
    {
        g_junctions[i].registered = 0;
        g_junctions[i].dirs_available = 0;
        g_junctions[i].dirs_tried = 0;
        g_junctions[i].seg_distance = 0;
        g_junctions[i].abs_distance = 0;
        g_junctions[i].outbound_distance = 0;
        g_junctions[i].entry_dir = 0;
        g_junctions[i].backtrack_count = 0;
        g_junctions[i].element_type = 0;
        g_junctions[i].turn_started = 0;
    }
    g_junction_count = 0;
    g_current_junction_idx = 0;
    g_junction_state = JSTATE_IDLE;
    g_chosen_direction = 0;
    g_junction_distance = 0;
    g_junction_turn_count = 0;
    g_backtrack_total = 0;
    g_exploration_complete = 0;
    junction_detect_buffer = 0;
    junction_detect_count = 0;
    junction_cooldown = 0;
    turn_exit_debounce = 0;
}

// 每帧更新编码器累计距离
void junction_update_distance(int32 left_enc, int32 right_enc)
{
    int32 avg = (left_enc + right_enc) / 2;
    if (avg > 0)
        g_junction_distance += avg;
}

// 检测是否正在接近路口 (防抖滤波)
// 返回: 1=已确认接近路口, 0=未检测到
uint8 junction_detect_approach(void)
{
    // 全部探索完毕: 永不重新进入
    if (g_exploration_complete)
        return 0;

    // 冷却期内不触发 (防回溯后重复注册同一路口)
    if (junction_cooldown > 0)
    {
        junction_cooldown--;
        junction_detect_count = 0;
        junction_detect_buffer = 0;
        return 0;
    }

    // 只有 T形路口和十字才触发路口遍历
    if (track_element_type == ELEMENT_T_JUNCTION || track_element_type == ELEMENT_CROSS)
    {
        junction_detect_count++;
        if (junction_detect_count >= JUNCTION_DETECT_FRAMES)
        {
            junction_detect_buffer = 1;
        }
    }
    else
    {
        junction_detect_count = 0;
        junction_detect_buffer = 0;
    }

    return junction_detect_buffer;
}

// 设置路口冷却 (在完成/回溯后调用, 防止立即重复注册)
void junction_set_cooldown(void)
{
    junction_cooldown = JUNCTION_COOLDOWN_FRAMES;
}

// 阶段1: 注册路口
// 记录: 可走方向 + 进入方向 + 距离指纹
void junction_register(uint8 element_type, uint8 tjunction_side)
{
    if (g_junction_count >= MAX_JUNCTIONS)
    {
        g_exploration_complete = 1;  // 溢出: 标记完成, 防止无限重试
        return;
    }

    JunctionRecord *jr = &g_junctions[g_junction_count];

    // === 确定可走方向 ===
    jr->dirs_available = 0;

    if (element_type == ELEMENT_CROSS)
    {
        // 十字路口: 三个方向都可走 (直行/左/右)
        jr->dirs_available = DIR_STRAIGHT | DIR_LEFT | DIR_RIGHT;
    }
    else if (element_type == ELEMENT_T_JUNCTION)
    {
        // T形路口: 直行 + 支路方向
        jr->dirs_available = DIR_STRAIGHT;
        if (tjunction_side == 1)
            jr->dirs_available |= DIR_RIGHT;   // 右侧支路
        else if (tjunction_side == -1)
            jr->dirs_available |= DIR_LEFT;    // 左侧支路
    }
    else
    {
        // 不应到达这里
        return;
    }

    // === 排除进入方向 (不能原路返回) ===
    {
        uint8 exclude_dir = 0;
        if (g_junction_count == 0)
        {
            // 第一个路口: 车从物理后方进入, 三个方向(直/左/右)都是前方
            // "直行" ≠ "返回", 所以不排除任何方向
            exclude_dir = 0;
        }
        else
        {
            // 后续路口: 从上一路口的方向的对面进入
            // 上一路口选了right → 本路口从left方向进入 → 排除left
            if (g_chosen_direction == DIR_RIGHT)
                exclude_dir = DIR_LEFT;
            else if (g_chosen_direction == DIR_LEFT)
                exclude_dir = DIR_RIGHT;
            else if (g_chosen_direction == DIR_STRAIGHT)
                exclude_dir = DIR_STRAIGHT;
        }
        if (exclude_dir != 0)
            jr->dirs_available &= ~exclude_dir;
    }

    // === 记录距离指纹 ===
    jr->seg_distance = g_junction_distance;     // 距上一路口的段距离
    jr->abs_distance = g_junction_distance;     // 绝对距离 (累加在外部做)
    if (g_junction_count > 0)
    {
        jr->abs_distance += g_junctions[g_junction_count - 1].abs_distance;
    }

    // === 进入方向 ===
    // 首个路口: 从后方进入 (视为直行进入)
    if (g_junction_count == 0)
    {
        jr->entry_dir = DIR_STRAIGHT;
    }
    else
    {
        // 从上一路口的选择方向的反方向进入本路口
        // 上一路口选了right → 本路口从left方向进入
        if (g_chosen_direction == DIR_RIGHT)
            jr->entry_dir = DIR_LEFT;
        else if (g_chosen_direction == DIR_LEFT)
            jr->entry_dir = DIR_RIGHT;
        else
            jr->entry_dir = DIR_STRAIGHT;
    }

    // === 标记已尝试的方向 ===
    jr->dirs_tried = 0;          // 初始: 没有方向被尝试过
    jr->backtrack_count = 0;
    jr->element_type = element_type;
    jr->registered = 1;
    jr->turn_started = 0;

    // 重置段距离 (为下一个路口做准备)
    g_junction_distance = 0;

    g_current_junction_idx = g_junction_count;
    g_junction_count++;

    g_junction_state = JSTATE_DECIDE;  // 注册完毕, 进入决策阶段
}

// 阶段2: 右手定则决策
// 优先级: 右 > 直 > 左
// 如果所有方向都尝试过, 触发回溯
void junction_decide_direction(void)
{
    if (g_junction_state != JSTATE_DECIDE)
        return;

    JunctionRecord *jr = &g_junctions[g_current_junction_idx];
    uint8 available = jr->dirs_available & ~jr->dirs_tried;
    uint8 i;
    uint8 chosen = 0;

    // 按优先级依次检查
    for (i = 0; i < DIR_PRIORITY_COUNT; i++)
    {
        if (available & dir_priority[i])
        {
            chosen = dir_priority[i];
            break;
        }
    }

    if (chosen != 0)
    {
        // 找到可走方向
        g_chosen_direction = chosen;
        jr->dirs_tried |= chosen;   // 标记为已尝试
        jr->turn_started = 0;
        g_junction_turn_count = 0;
        turn_exit_debounce = 0;     // 重置转弯退出防抖
        g_junction_state = JSTATE_TURN;
    }
    else
    {
        // 所有方向都尝试过了 → 回溯
        if (jr->backtrack_count < MAX_BACKTRACK && g_current_junction_idx > 0)
        {
            g_junction_state = JSTATE_BACKTRACK;
        }
        else
        {
            // 回溯次数耗尽或这是第一个路口 → 全部探索完毕
            g_junction_state = JSTATE_COMPLETE;
            g_exploration_complete = 1;  // 防止重新进入状态机
        }
    }
}

// 阶段3: 执行转弯
// 修改 center_line 和 track_element_strength 来驱动转向
// 返回: 1=转弯进行中, 0=转弯完成
uint8 junction_execute_turn(void)
{
    if (g_junction_state != JSTATE_TURN)
        return 0;

    JunctionRecord *jr = &g_junctions[g_current_junction_idx];

    // 转弯帧计数
    g_junction_turn_count++;

    // 转弯最大帧数保护: 150帧 (约3秒 @50fps, 覆盖大路口慢速转弯)
    if (g_junction_turn_count > 150)
    {
        jr->turn_started = 0;
        g_junction_state = JSTATE_COMPLETE;
        return 0;
    }

    // 检查是否已经通过路口 (元素类型不再是T形/十字, 且持续至少5帧)
    // 使用文件级防抖计数器, 新转弯开始时由 junction_decide_direction 重置
    {
        if (g_junction_turn_count > 20 &&
            track_element_type != ELEMENT_T_JUNCTION &&
            track_element_type != ELEMENT_CROSS)
        {
            turn_exit_debounce++;
            if (turn_exit_debounce >= 5)
            {
                turn_exit_debounce = 0;
                jr->turn_started = 0;
                g_junction_state = JSTATE_COMPLETE;
                return 0;
            }
        }
        else
        {
            turn_exit_debounce = 0;
        }
    }

    // === 根据选定方向修改 center_line (渐进式, 防高速过冲) ===
    jr->turn_started = 1;
    int16 center_ref = image_w / 2;

    // 渐进因子: 前20帧从5%→100%, 之后全程100%
    // ramp范围: 0→5, 20→100, 每帧递增5%
    int32 ramp = (g_junction_turn_count < 20) ? (g_junction_turn_count * 5) : 100;
    if (ramp < 5) ramp = 5;  // 首帧至少5%以避免完全无效果

    if (g_chosen_direction == DIR_RIGHT)
    {
        // 右转: 渐进推中线到右侧
        int16 target = image_w - 12;
        for (int16 k = image_h - 1; k >= image_h - 12 && k >= 0; k--)
        {
            int16 current = center_line[k];
            int16 desired = current + (target - current) * ramp / 100;
            if (desired < image_w - 12) desired = image_w - 12;
            if (desired > image_w - 3) desired = image_w - 3;
            center_line[k] = desired;
        }
        // 上方补线: 从远处(left)渐变到边缘(right)
        int16 top_x = image_w - 5 - (100 - ramp) * 40 / 100;
        if (top_x < 3) top_x = 3;
        track_add_line(top_x, image_h / 2, image_w - 12, image_h - 1);
    }
    else if (g_chosen_direction == DIR_LEFT)
    {
        // 左转: 渐进推中线到左侧
        int16 target = 12;
        for (int16 k = image_h - 1; k >= image_h - 12 && k >= 0; k--)
        {
            int16 current = center_line[k];
            int16 desired = current + (target - current) * ramp / 100;
            if (desired > 12) desired = 12;
            if (desired < 3) desired = 3;
            center_line[k] = desired;
        }
        int16 top_x = 5 + (100 - ramp) * 40 / 100;
        if (top_x > image_w - 3) top_x = image_w - 3;
        track_add_line(top_x, image_h / 2, 12, image_h - 1);
    }
    else  // DIR_STRAIGHT
    {
        // 直行: 温和拉回图像中心
        for (int16 k = image_h - 1; k >= image_h - 8 && k >= 0; k--)
        {
            int16 error = center_line[k] - center_ref;
            center_line[k] = center_ref + error * 2 / 3;  // 拉回1/3
        }
    }

    return 1;  // 转弯进行中
}

// ==========================================
// 阶段3.5: 物理回溯 — 掉头+反向行驶回目标路口
// 三阶段: U-TURN(180°掉头) → RETURN(反向行驶) → ARRIVE(到达, 转DECIDE)
// 返回: 1=回溯执行中, 0=已到达目标路口
// ==========================================
uint8 junction_execute_return(void)
{
    if (g_junction_state != JSTATE_RETURNING)
        return 0;

    // 静态状态: 跨帧保持回溯进度
    static uint8  return_phase = 0;       // 0=U-TURN, 1=RETURN, 2=ARRIVE
    static uint16 return_frame = 0;       // 当前阶段帧计数
    static int32  return_start_dist = 0;  // 返回阶段起点距离
    static uint8  return_uturn_dir = 0;   // 掉头方向 (1=右转掉头)

    JunctionRecord *jr = &g_junctions[g_current_junction_idx];

    // ==================== 阶段0: U-TURN 180°掉头 ====================
    if (return_phase == 0)
    {
        return_frame++;

        // 第一帧: 确定掉头方向
        if (return_frame == 1)
        {
            // 默认右转掉头 (推中线到右侧边缘)
            return_uturn_dir = 1;
            return_start_dist = g_junction_distance;
        }

        // 掉头: 强制推中线到边缘, 使车持续急转
        int16 edge_target = (return_uturn_dir == 1) ? (image_w - 8) : 8;
        for (int16 k = image_h - 1; k >= image_h - 15 && k >= 0; k--)
        {
            if (return_uturn_dir == 1)
            {
                if (center_line[k] < edge_target)
                    center_line[k] = edge_target;
            }
            else
            {
                if (center_line[k] > edge_target)
                    center_line[k] = edge_target;
            }
        }
        // 上方补线, 确保整个视野都偏向掉头方向
        track_add_line(edge_target, 0, edge_target, image_h - 1);

        // 掉头完成条件: 达到目标帧数 OR 中线回正(说明已转过来看到路了)
        if (return_frame >= RETURN_UTURN_FRAMES ||
            (return_frame > 30 &&
             abs(center_line[image_h - 1] - image_w / 2) < 15))
        {
            return_phase = 1;   // 进入反向行驶阶段
            return_frame = 0;
            g_junction_distance = 0;  // 清零, 开始累积返回距离
        }

        return 1;  // U-TURN进行中
    }

    // ==================== 阶段1: 反向行驶回目标路口 ====================
    if (return_phase == 1)
    {
        return_frame++;

        // 超时保护: 200帧(~4秒)仍未到达 → 放弃
        if (return_frame > 200)
        {
            return_phase = 0;  // 重置状态
            return_frame = 0;
            g_junction_state = JSTATE_COMPLETE;  // 放弃回溯
            return 0;
        }

        // 距离匹配: 累积的返回距离 ≈ 出站距离(路口→死胡同)
        int32 dist_diff = g_junction_distance - jr->outbound_distance;
        if (dist_diff < 0) dist_diff = -dist_diff;

        if (g_junction_distance > 0 &&
            dist_diff <= RETURN_DIST_MATCH)
        {
            // 距离匹配! 已到达目标路口
            return_phase = 0;   // 重置状态(为下次回溯准备)
            return_frame = 0;

            // 重置段距离为到达值
            g_junction_distance = jr->seg_distance;

            // 进入决策阶段, 选择下一个未尝试方向
            g_junction_state = JSTATE_DECIDE;
            return 0;  // 回溯完成
        }

        // 仍在返回途中: 正常巡线 (track_find_line会处理)
        // junction_update_distance在cpu0_main中每帧调用, 自动累积距离
        return 1;  // 返回进行中
    }

    return 0;
}

// 距离指纹回溯匹配
// 在遇到断路后, 回到最近的有未尝试方向的路口
// 返回: 匹配到的路口索引, -1=未匹配
int8 junction_match_backtrack(void)
{
    if (g_backtrack_total >= MAX_BACKTRACK)
        return -1;

    // 从最近的路口向前搜索, 找有未尝试方向的路口
    int8 i;
    for (i = (int8)g_junction_count - 1; i >= 0; i--)
    {
        JunctionRecord *jr = &g_junctions[i];
        if (!jr->registered)
            continue;

        uint8 remaining = jr->dirs_available & ~jr->dirs_tried;
        if (remaining != 0 && jr->backtrack_count < MAX_BACKTRACK)
        {
            // 找到可用路口, 匹配成功
            jr->backtrack_count++;
            g_backtrack_total++;
            g_current_junction_idx = (uint8)i;
            // 进入物理回溯阶段 (掉头+反向行驶), 而非直接DECIDE
            g_junction_state = JSTATE_RETURNING;
            return i;
        }
    }

    // 没有可用路口 → 遍历完成
    g_junction_state = JSTATE_COMPLETE;
    return -1;
}

// 标记当前路径为死路
void junction_mark_dead_end(void)
{
    // 路口遍历未激活时: 不处理
    if (g_junction_state == JSTATE_IDLE || g_junction_state == JSTATE_COMPLETE)
        return;

    // 记录当前路口的已尝试方向
    JunctionRecord *jr = &g_junctions[g_current_junction_idx];

    if (g_junction_state == JSTATE_TURN && g_junction_turn_count > 20)
    {
        // 转弯中遇到死路: 标记当前方向已尝试, 记录出站距离供回溯匹配
        jr->dirs_tried |= g_chosen_direction;
        jr->outbound_distance = g_junction_distance;  // 从路口到死胡同的距离
    }
    else if (g_junction_state == JSTATE_APPROACH || g_junction_state == JSTATE_DECIDE)
    {
        // 接近/决策阶段遇死路: 标记全部已尝试, 强制回溯
        jr->dirs_tried = jr->dirs_available;
    }

    // 触发回溯
    if (g_backtrack_total < MAX_BACKTRACK)
    {
        g_junction_state = JSTATE_BACKTRACK;
    }
    else
    {
        // 回溯耗尽: 标记完成
        g_junction_state = JSTATE_COMPLETE;
    }
}

#pragma section all restore
