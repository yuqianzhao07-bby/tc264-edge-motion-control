/*********************************************************************************************************************
* TC264 Opensourec Library (TC264 开源库) - 一个基于英飞凌 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
********************************************************************************************************************/

#include "mt9v03x_tft180_display.h"
#include "zf_device_mt9v03x_double.h"
#include "zf_device_tft180.h"
#include "debug_trace.h"

uint8 mt9v03x_tft180_init(void)
{
    DEBUG_PRINTF("  [CAM] 设置TFT180显示方向...\r\n");
    tft180_set_dir(TFT180_CROSSWISE_180);    // 横屏+180度旋转, 匹配赛道方向

    DEBUG_PRINTF("  [CAM] 初始化TFT180显示屏...\r\n");
    tft180_init();
    tft180_show_string(0, 0, "mt9v03x init.");
    DEBUG_PRINTF("  [CAM] TFT180显示屏初始化完成\r\n");

    uint8 init_result = 0;
    uint16 retry_count = 0;

    DEBUG_PRINTF("  [CAM] 开始初始化MT9V03X摄像头 (最多重试10次)...\r\n");

    while(1)
    {
        DEBUG_PRINTF("  [CAM] 尝试第 %d 次初始化...\r\n", retry_count + 1);

        if(mt9v03x_double_init(mt9v03x_1))
        {
            tft180_show_string(0, 16, "mt9v03x reinit.");
            init_result = 1;
            retry_count++;

            DEBUG_PRINTF("  [CAM] 第 %d 次初始化失败, 1秒后重试\r\n", retry_count);

            // 最多重试 10 次
            if(retry_count > 10)
            {
                DEBUG_PRINTF("  [CAM] 已达最大重试次数(10次), 初始化失败!\r\n");
                break;
            }

            system_delay_ms(1000);  // 闪烁提示错误
        }
        else
        {
            init_result = 0;
            DEBUG_PRINTF("  [CAM] MT9V03X摄像头初始化成功\r\n");
            break;
        }
    }

    if(init_result == 0)
    {
        tft180_show_string(0, 16, "init success.");
        DEBUG_PRINTF("  [CAM] 摄像头+显示屏全部初始化完成\r\n");
    }

    return init_result;
}

uint8 mt9v03x_tft180_display(uint8 *image_data, uint16 width, uint16 height)
{
    if(image_data == NULL)
    {
        return 1;
    }

    tft180_displayimage03x((const uint8 *)image_data, width, height);

    return 0;
}

uint8 mt9v03x_tft180_check_finish(void)
{
    return mt9v03x_finish_flag_1;
}

void mt9v03x_tft180_clear_finish_flag(void)
{
    mt9v03x_finish_flag_1 = 0;
}

uint8 *mt9v03x_tft180_get_image(void)
{
    return mt9v03x_image_1;
}
