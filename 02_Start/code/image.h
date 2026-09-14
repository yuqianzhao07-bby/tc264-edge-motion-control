#ifndef _image_h_
#define _image_h_

#include "stdint.h"
#include "stdbool.h"
#include "zf_common_typedef.h"

// 图像高度定义
#define image_h    60
// 图像宽度定义
#define image_w    100
// 白色像素值定义
#define WHITE  255
// 黑色像素值定义
#define BLACK    0

// MT9V03X 摄像头分辨率定义
#define MT9V03X_W    188  // 摄像头宽度
#define MT9V03X_H    120  // 摄像头高度


// 大津法阈值计算函数
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row);

// 图像压缩函数（将大图裁切为小图）
void compressimage(void);

// 图像二值化函数（将灰度图转为黑白图）
void turn_to_bin(void);

// 形态学滤波函数（膨胀和腐蚀）
void image_filter(uint8(*image)[image_w]);

// 中值滤波函数（去除噪声，3x3核）
void median_filter(uint8(*image)[image_w]);


// 二值化图像数组
extern uint8 bin_image[image_h][image_w];

// 断路误差数组
extern int16 D_err[10];

// 断路计数
extern uint8 D_count;

// 显示标志位（调试用）
extern bool show_flag;

// 线标志位（是否检测到赛道线）
extern bool line_flag;

// 图像基准值（用于调整二值化阈值）
extern int8 image_base;

// 图像分割阈值（大津法计算结果）
extern uint8 image_thereshold;

#endif
