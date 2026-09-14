// 头文件包含
#include "zf_common_headfile.h"
#include "image.h"
#include "zf_device_mt9v03x_double.h"
#include "../code/debug_trace.h"

// 放在 CPU0 本地 DSPR0, 避免跨 SRI 总线访问 CPU1 DSPR1
#pragma section all "cpu0_dsram"

uint8 stop_otsu_flag = 0;
uint8 image_thereshold;
int8 image_base = -10;
int16 D_err[10] = {0,0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8 D_count = 0;
bool show_flag = 0;
bool line_flag = 0;


uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)
{
    uint32 HistGram[256] = {0};
    uint32 total_pixels = (uint32)col * row;
    uint8 *ptr = image;

    for (uint32 i = 0; i < total_pixels; i++)
    {
        HistGram[*ptr++]++;
    }

    uint16 MinValue = 0;
    uint16 MaxValue = 255;
    for (MinValue = 0; MinValue < 255 && HistGram[MinValue] == 0; MinValue++);
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MaxValue] == 0; MaxValue--);

    if (MaxValue == MinValue) return (uint8)MaxValue;
    if (MinValue + 1 == MaxValue) return (uint8)MinValue;

    uint32 sum1 = 0;
    uint32 Amount = 0;
    for (uint16 t = MinValue; t <= MaxValue; t++)
    {
        Amount += HistGram[t];
        sum1 += (uint32)t * HistGram[t];
    }

    uint32 sum2 = 0;
    uint32 wB = 0;
    uint64 maxVar = 0;
    uint8 Threshold = (uint8)MinValue;

    for (uint16 t = MinValue; t < MaxValue; t++)
    {
        uint32 hist_t = HistGram[t];
        wB += hist_t;
        if (wB == 0) continue;

        sum2 += (uint32)t * hist_t;
        uint32 wF = Amount - wB;
        if (wF == 0) break;

        // uint32硬件除法 (TriCore单周期), 远优于uint64软件除法
        uint32 SF = sum1 - sum2;
        uint32 mB = sum2 / wB;
        uint32 mF = SF / wF;
        uint32 diff = (mB > mF) ? (mB - mF) : (mF - mB);
        uint64 var = (uint64)wB * wF * diff * diff;

        if (var > maxVar)
        {
            maxVar = var;
            Threshold = (uint8)t;
        }
    }

    // 动态下限: 阈值不低于 MinValue+10, 避免极暗场景截断
    // 旧版硬编码45在极暗环境下会导致二值化失效
    uint8 floor = (MinValue + 10 > 45) ? (uint8)(MinValue + 10) : 45;
    return (Threshold < floor) ? floor : Threshold;
}


// 压缩后图像缓冲区
uint8 Image_use_zip[image_h][image_w];

// 从原图居中裁切60x100到Image_use_zip
// 摄像头188x120, 裁切100x60, 居中: 行(120-60)/2=30, 列(188-100)/2=44
void compressimage(void)
{
    uint8 *src = &mt9v03x_image_1[30][44];
    uint8 *dst = &Image_use_zip[0][0];

    DEBUG_PRINTF_V("[ZIP]   裁切起点(30,44) 输出%dx%d\r\n", image_w, image_h);

    for (int i = 0; i < image_h; i++)
    {
        memcpy(dst, src, image_w);
        src += MT9V03X_W;
        dst += image_w;
    }

    DEBUG_PRINTF_V("[ZIP]   压缩完成, 首行[0,1,2]=[%d,%d,%d]\r\n",
        Image_use_zip[0][0], Image_use_zip[0][1], Image_use_zip[0][2]);
}


// 二值化图像数组
uint8 bin_image[image_h][image_w];

// 二值化: Image_use_zip → bin_image
void turn_to_bin(void)
{
    DEBUG_PRINTF_V("[BIN]   大津法开始, %dx%d...\r\n", image_w, image_h);

    image_thereshold = otsuThreshold(Image_use_zip[0], image_w, image_h);

    uint8 effective = image_thereshold + image_base;
    if (effective > 254) effective = 254;
    DEBUG_PRINTF_V("[BIN]   阈值=%d 有效=%d\r\n", image_thereshold, effective);

    uint8 *src = &Image_use_zip[0][0];
    uint8 *dst = &bin_image[0][0];
    uint32 total = (uint32)image_h * image_w;

    // 4像素展开
    uint32 blocks = total >> 2;
    while (blocks--)
    {
        *dst++ = (*src++ > effective) ? 255 : 0;
        *dst++ = (*src++ > effective) ? 255 : 0;
        *dst++ = (*src++ > effective) ? 255 : 0;
        *dst++ = (*src++ > effective) ? 255 : 0;
    }
    total &= 3;
    while (total--)
    {
        *dst++ = (*src++ > effective) ? 255 : 0;
    }

    DEBUG_PRINTF_V("[BIN]   完成, %lu像素\r\n", (uint32)image_h * image_w);
}


// 形态学滤波
void image_filter(uint8(*image)[image_w])
{
    DEBUG_PRINTF_V("[FLT]   开始...\r\n");
    uint16 i, j;
    uint32 num;

    for (i = 1; i < image_h - 1; i++)
    {
        for (j = 1; j < image_w - 1; j++)
        {
            num = image[i-1][j-1] + image[i-1][j] + image[i-1][j+1]
                + image[i][j-1] + image[i][j+1]
                + image[i+1][j-1] + image[i+1][j] + image[i+1][j+1];

            if (num >= 2040 && image[i][j] == 0)
                image[i][j] = 255;
            if (num <= 255 && image[i][j] == 255)
                image[i][j] = 0;
        }
    }
    DEBUG_PRINTF_V("[FLT]   完成\r\n");
}


// 中值滤波 (指针轮转, 无memcpy)
void median_filter(uint8(*image)[image_w])
{
    DEBUG_PRINTF_V("[MED]   开始...\r\n");
    uint16 i, j;
    uint16 sum;

    uint8 *prev = image[0];
    uint8 *curr = image[1];
    uint8 *next;

    for (i = 1; i < image_h - 1; i++)
    {
        next = image[i + 1];

        for (j = 1; j < image_w - 1; j++)
        {
            sum = (uint16)prev[j-1] + prev[j] + prev[j+1]
                + curr[j-1] + curr[j] + curr[j+1]
                + next[j-1] + next[j] + next[j+1];

            image[i][j] = (sum >= 1275) ? 255 : 0;
        }

        prev = curr;
        curr = next;
    }
    DEBUG_PRINTF_V("[MED]   完成\r\n");
}

#pragma section all restore
