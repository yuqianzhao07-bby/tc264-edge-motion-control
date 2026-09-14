// 头文件包含
#include "zf_common_headfile.h"  // 逐飞科技通用头文件
#include "mymath.h"               // 数学库头文件


// 求绝对值函数
// value: 输入的有符号整数
// 返回值: 输入数的绝对值
int my_abs(int16 value)
{
    // 如果值大于等于0，直接返回
    if(value>=0) return value;
    // 否则返回其相反数（负负得正）
    else return -value;
}


// 最小二乘法计算斜率函数
// 使用最小二乘法拟合边界点，得到直线的斜率
// begin: 拟合数据的起始索引（行号）
// end: 拟合数据的结束索引（行号）
// border: 边界数据数组指针，border[i]表示第i行的边界列坐标
// 返回值: 拟合直线的斜率k (y = k*x + b)
float Slope_Calculate(uint8 begin, uint8 end, uint8 *border)
{
    float xsum = 0;     // X坐标（行号）之和
    float ysum = 0;     // Y坐标（列号/边界位置）之和
    float xysum = 0;    // X*Y乘积之和
    float x2sum = 0;   // X平方之和
    int16 i = 0;
    float result = 0;    // 计算结果（斜率）
    static float resultlast;  // 上一次计算的斜率（用于除数为零时备用）

    // 遍历指定范围内的所有点，累加各项求和
    for (i = begin; i < end; i++)
    {
        xsum += i;                 // 累加行号
        ysum += border[i];         // 累加边界列坐标
        xysum += i * (border[i]);  // 累加行号×列号
        x2sum += i * i;            // 累加行号的平方
    }

    // 计算斜率公式：k = (n*Σxy - Σx*Σy) / (n*Σx² - (Σx)²)
    // 判断除数是否为零（判断直线是否垂直）
    if ((end - begin)*x2sum - xsum * xsum)
    {
        // 最小二乘法斜率计算公式
        result = ((end - begin)*xysum - xsum * ysum) / ((end - begin)*x2sum - xsum * xsum);
        resultlast = result;  // 保存本次结果
    }
    else
    {
        // 除数为零时（直线接近垂直），使用上一次的斜率值
        result = resultlast;
    }
    return result;
}


// 计算斜率和截距函数
// 使用最小二乘法同时计算边界拟合直线的斜率和截距
// start: 拟合数据的起始索引
// end: 拟合数据的结束索引
// border: 边界数据数组指针
// slope_rate: 输出参数，返回计算得到的斜率
// intercept: 输出参数，返回计算得到的截距
void calculate_s_i(uint8 start, uint8 end, uint8 *border, float *slope_rate, float *intercept)
{
    uint16 i, num = 0;      // 循环计数器和数据个数
    uint16 xsum = 0;        // X坐标求和
    uint16 ysum = 0;        // Y坐标求和
    float y_average;        // Y平均值
    float x_average;        // X平均值

    // 初始化变量
    num = 0;
    xsum = 0;
    ysum = 0;
    y_average = 0;
    x_average = 0;

    // 第一步：遍历数据，累加求和
    for (i = start; i < end; i++)
    {
        xsum += i;          // 累加行号
        ysum += border[i];  // 累加边界列坐标
        num++;              // 计数
    }

    // 第二步：计算平均值
    if (num)
    {
        x_average = (float)(xsum / num);  // X平均值
        y_average = (float)(ysum / num);  // Y平均值
    }

    // 第三步：计算斜率（调用最小二乘法函数）
    *slope_rate = Slope_Calculate(start, end, border);

    // 第四步：计算截距
    // 直线方程 y = kx + b，变形得 b = y - kx
    *intercept = y_average - (*slope_rate) * x_average;
}


// 在屏幕上显示圆点标记函数（调试用）
// 在指定坐标点显示一个红色的圆点（9点构成）
// x: 圆心X坐标
// y: 圆心Y坐标
void ips_show_round(uint8 x,uint8 y)
{
    // 在圆心周围绘制8个点，形成一个圆形的标记
    ips200_draw_point(x+3,y,RGB565_RED);      // 右
    ips200_draw_point(x+3,y+3,RGB565_RED);    // 右下
    ips200_draw_point(x,y+3,RGB565_RED);      // 下
    ips200_draw_point(x-3,y+3,RGB565_RED);   // 左下
    ips200_draw_point(x-3,y,RGB565_RED);      // 左
    ips200_draw_point(x-3,y-3,RGB565_RED);    // 左上
    ips200_draw_point(x,y-3,RGB565_RED);      // 上
    ips200_draw_point(x+3,y-3,RGB565_RED);    // 右上
}


// 计算斜率k函数
// 根据两个点的坐标计算斜率
// y1: 第一个点的行坐标
// y2: 第二个点的行坐标
// border: 边界数据数组，border[i]表示第i行的边界列坐标
// 返回值: 斜率k = (y1-y2) / (border[y1]-border[y2])
float k_get(uint8 y1,uint8 y2,uint8 *border)
{
    float k;
    // k = Δy / Δx = (y1行坐标 - y2行坐标) / (y1处边界列 - y2处边界列)
    k = 1.0 * (y1 - y2) / (border[y1] - border[y2]);
    return k;
}


// 连接两点之间的直线函数（布雷森ham算法/Bresenham's line algorithm）
// 使用增量误差法绘制两点之间的直线
// temp: 输出数组，存储直线经过的每个点的坐标
// x1, x2: 两个点的行坐标（X坐标）
// y1, y2: 两个点的列坐标（Y坐标）
void connect_point(int temp[], int x1, int x2, int y1, int y2)
{
    int dx = x2 - x1;  // X方向增量
    int dy = y2 - y1;  // Y方向增量
    int ux;            // X方向单位增量（+1或-1）
    int uy;            // Y方向单位增量（+1或-1）

    // 确定X方向增量单位
    if (dx > 0)
        ux = 1;   // X增大方向
    else
        ux = -1;  // X减小方向

    // 确定Y方向增量单位
    if (dy > 0)
        uy = 1;   // Y增大方向
    else
        uy = -1;  // Y减小方向

    int x = x1, y = y1, eps;  // 当前点和累加误差
    eps = 0;
    dx = my_abs(dx);  // 取绝对值
    dy = my_abs(dy);

    // 根据斜率绝对值选择主方向
    if (dx > dy)
    {
        // 斜率小于1，以X为主方向
        for (x = x1; x != x2; x += ux)
        {
            temp[x] = y;  // 记录当前点
            eps += dy;   // 累加误差
            if ((eps << 1) >= dx)  // 如果误差超过阈值
            {
                y += uy;      // Y方向移动
                eps -= dx;    // 减去误差
            }
        }
    }
    else
    {
        // 斜率大于1，以Y为主方向
        for (y = y1; y != y2; y += uy)
        {
            temp[x] = y;  // 记录当前点
            eps += dx;   // 累加误差
            if ((eps << 1) >= dy)  // 如果误差超过阈值
            {
                x += ux;      // X方向移动
                eps -= dy;    // 减去误差
            }
        }
    }
}


// 根据两点斜率从上往下延伸函数
// 根据已知两点的斜率，从第一个点向上延伸直线
// temp: 输出数组，存储延伸后的点坐标
// x1, x2: 两个基准点的行坐标
// y1, y2: 两个基准点的列坐标
void yanshen_to_up(int temp[], int x1, int x2, int y1, int y2)
{
    int dx = x2 - x1;  // X方向增量
    int dy = y2 - y1;  // Y方向增量
    int ux;            // X方向单位增量
    int uy;            // Y方向单位增量

    // 确定X方向增量单位
    if (dx > 0)
        ux = 1;
    else
        ux = -1;

    // 确定Y方向增量单位
    if (dy > 0)
        uy = 1;
    else
        uy = -1;

    int x = x1, y = y1, eps;  // 当前点和累加误差
    eps = 0;
    dx = my_abs(dx);
    dy = my_abs(dy);

    // 如果X方向是负的（向上延伸），交换起点和终点
    if (ux == 1)
    {
        ux = -1;
        int temper = 0;
        temper = x1;
        x1 = x2;
        x2 = temper;
        temper = y1;
        y1 = y2;
        y2 = temper;
    }

    // 从起点向上（X递减）延伸
    for (x = x1; x >= 0; x += ux)
    {
        // 边界检查，确保坐标在有效范围内
        if (y < 1) y = 1;
        if (y > 186) y = 186;
        temp[x] = y;  // 记录当前点
        eps += dy;   // 累加误差
        if ((eps << 1) >= dx)  // 误差检查
        {
            y += uy;      // Y方向移动
            eps -= dx;    // 减去误差
        }
    }
}


// 根据两点斜率从下往上延伸函数
// 根据已知两点的斜率，从第一个点向下延伸直线
// temp: 输出数组，存储延伸后的点坐标
// x1, x2: 两个基准点的行坐标
// y1, y2: 两个基准点的列坐标
void yanshen_to_down(int temp[], int x1, int x2, int y1, int y2)
{
    int dx = x2 - x1;  // X方向增量
    int dy = y2 - y1;  // Y方向增量
    int ux;            // X方向单位增量
    int uy;            // Y方向单位增量

    // 确定X方向增量单位
    if (dx > 0)
        ux = 1;
    else
        ux = -1;

    // 确定Y方向增量单位
    if (dy > 0)
        uy = 1;
    else
        uy = -1;

    int x = x1, y = y1, eps;  // 当前点和累加误差
    eps = 0;
    dx = my_abs(dx);
    dy = my_abs(dy);

    // 如果X方向是正的（向下延伸），交换起点和终点
    if (ux == -1)
    {
        ux = 1;
        int temper = 0;
        temper = x1;
        x1 = x2;
        x2 = temper;
        temper = y1;
        y1 = y2;
        y2 = temper;
    }

    // 从起点向下（X递增）延伸
    for (x = x1; x < 120; x += ux)
    {
        // 边界检查，确保坐标在有效范围内
        if (y < 1) y = 1;
        if (y > 186) y = 186;
        temp[x] = y;  // 记录当前点
        eps += dy;   // 累加误差
        if ((eps << 1) >= dx)  // 误差检查
        {
            y += uy;      // Y方向移动
            eps -= dx;    // 减去误差
        }
    }
}


// 历史误差数组（用于滑动平均滤波）
int16 history_err[ERR_SIZE]={0,0,0,0,0};


// 滑动平均滤波函数
// 使用滑动窗口的方式对输入数据进行滤波，减少噪声干扰
// buffer: 数据缓冲区指针，用于存储历史数据
// size: 缓冲区大小（滤波窗口长度）
// new_value: 新输入的数据值
// 返回值: 滤波后的平均值
int16 moving_average_filter(int16 *buffer, uint8 size, int16 new_value)
{
    // 静态变量：索引（环形缓冲区的当前位置）
    static uint8 index = 0;
    // 静态变量：历史数据的总和
    static int16 sum = 0;
    // 静态变量：当前平均值
    static int16 avg = 0;
    // 静态变量：标记是否已经初始化
    static uint8 is_initialized = 0;

    // 第一次调用时，初始化所有历史数据
    if (!is_initialized)
    {
        // 累加所有历史数据到sum
        for (uint8 i = 0; i < size; i++)
        {
            sum += buffer[i];
        }
        // 计算初始平均值
        avg = sum / size;
        // 标记已初始化
        is_initialized = 1;
    }

    // 更新数据：用新值替换最旧的值
    // 1. 从总和中减去被替换的旧值
    sum -= buffer[index];
    // 2. 将新值存入缓冲区当前位置
    buffer[index] = new_value;
    // 3. 将新值加到总和中
    sum += new_value;
    // 4. 更新索引（环形移动）
    index = (index + 1) % size;

    // 计算新的平均值
    avg = (int16)(sum / size);

    return avg;
}


// 连接两点生成引导线函数
// 使用改进的Bresenham算法，在两点之间绘制直线
// x0, y0: 第一个点的坐标（通常是图像底部的跳变点）
// x1, y1: 第二个点的坐标（通常是直角或元素处的跳变点）
// line_out: 输出的中线数组，line_out[y] = x表示第y行的中线X坐标
// h: 图像高度（用于边界检查）
// w: 图像宽度（用于边界检查）
void connect_angle_points(uint8 x0,uint8 y0,uint8 x1,uint8 y1, uint8 *line_out, uint8 h,uint8 w)
{
    // 计算X和Y方向的增量（绝对值）
    int16 dx = my_abs(x1 - x0);
    // 确定X方向的正负
    int16 sx = x0 < x1 ? 1 : -1;

    // 计算Y方向增量（注意这里是负数，因为图像Y轴向下）
    int16 dy = -my_abs(y1 - y0);
    // 确定Y方向的正负
    int16 sy = y0 < y1 ? 1 : -1;

    // 计算初始误差值
    int16 error = dx + dy;

    // 计算最大迭代次数（确保能画完整条线）
    int16 max_iter = 2 * (dx > -dy ? dx : -dy);

    // 使用for循环绘制直线
    for(int16 i = 0; i <= max_iter; i++)
    {
        // 边界检查，确保点在图像范围内
        if(y0 >= 0 && y0 < h-1 && x0 >= 0 && x0 < w-1)
        {
            // 存储直线上的点：line_out[行号] = 列坐标
            line_out[y0] = x0;
        }

        // 如果到达终点，提前退出
        if(x0 == x1 && y0 == y1) break;

        // 计算误差
        int16 e2 = 2 * error;

        // 沿X方向移动
        if(e2 >= dy)
        {
            error += dy;
            x0 += sx;
        }

        // 沿Y方向移动
        if(e2 <= dx)
        {
            error += dx;
            y0 += sy;
        }
    }
}