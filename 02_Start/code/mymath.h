#ifndef _MYMATH_H_
#define _MYMATH_H_

// 求绝对值函数
// value: 输入的整数
// 返回值: 绝对值
int my_abs(int16 value);

// 最小二乘法计算斜率
// begin: 起始索引
// end: 结束索引
// border: 边界数据数组指针
// 返回值: 拟合直线的斜率
float Slope_Calculate(uint8 begin, uint8 end, uint8 *border);

// 计算斜率和截距
// start: 起始索引
// end: 结束索引
// border: 边界数据数组指针
// slope_rate: 输出斜率的指针
// intercept: 输出截距的指针
void calculate_s_i(uint8 start, uint8 end, uint8 *border, float *slope_rate, float *intercept);

// 在屏幕上显示圆点标记（调试用）
// x: X坐标
// y: Y坐标
void ips_show_round(uint8 x,uint8 y);

// 计算斜率k
// y1: 第一个点的Y坐标（行数）
// y2: 第二个点的Y坐标（行数）
// border: 边界数据数组指针
// 返回值: 斜率k
float k_get(uint8 y1,uint8 y2,uint8 *border);

// 连接两点之间的直线（布雷森ham算法）
// temp: 输出数组（存储连线上的点）
// x1, x2: 两个点的行数坐标
// y1, y2: 两个点的列数坐标
void connect_point(int temp[], int x1, int x2, int y1, int y2);

// 根据两点斜率从上往下延伸
// temp: 输出数组（存储延伸后的点）
// x1, x2: 两个点的行数坐标
// y1, y2: 两个点的列数坐标
void yanshen_to_up(int temp[], int x1, int x2, int y1, int y2);

// 根据两点斜率从下往上延伸
// temp: 输出数组（存储延伸后的点）
// x1, x2: 两个点的行数坐标
// y1, y2: 两个点的列数坐标
void yanshen_to_down(int temp[], int x1, int x2, int y1, int y2);

// 滑动平均滤波函数
// buffer: 数据缓冲区指针
// size: 缓冲区大小
// new_value: 新输入的数据值
// 返回值: 滤波后的平均值
int16 moving_average_filter(int16 *buffer, uint8 size, int16 new_value);

// 连接两点生成引导线
// x0, y0: 第一个点的坐标（底部跳变点）
// x1, y1: 第二个点的坐标（直角处跳变点）
// line_out: 输出的中线数组
// h: 图像高度
// w: 图像宽度
void connect_angle_points(uint8 x0,uint8 y0,uint8 x1,uint8 y1,uint8 *line_out,uint8 h,uint8 w);

// 历史误差数组大小
#define ERR_SIZE 5

// 声明历史误差数组（用于滤波）
extern int16 history_err[ERR_SIZE];

#endif