/*********************************************************************************************************************
* TC264 Opensourec Library (TC264 开源库) - 一个基于英飞凌 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件为 MT9V03X 摄像头与 TFT180 显示屏驱动的示例代码
*
* TC264 开源库 使用 GPL3.0 开源协议发布
* 您可以在遵守 GPL3.0 协议的前提下使用本库
you can use this library under the terms of the GPL3.0 protocol
*
* 本开源库的所有代码都经过逐飞科技的严格测试，但是不保证其绝对的稳定性和安全性
* 使用本库时，请自行承担风险
* 如您发现本库存在 bug 或有更好的建议，欢迎联系我们
*
* 公司网站: https://seekfree.taobao.com/
* 技术论坛: http://www.seekfree.net/
* 公司邮箱: support@seekfree.cn
********************************************************************************************************************/

#include "mt9v03x_tft180_display.h"

/**
 * @brief MT9V03X 摄像头与 TFT180 显示屏示例
 * @return 无
 */
void mt9v03x_tft180_example(void)
{
    // 初始化 MT9V03X 摄像头和 TFT180 显示屏
    uint8 init_result = mt9v03x_tft180_init();
    
    if(init_result != 0)
    {
        // 初始化失败，可能需要检查硬件连接
        while(1);
    }
    
    // 等待所有核心初始化完毕
    cpu_wait_event_ready();
    
    while(TRUE)
    {
        // 检查摄像头是否采集完成一帧图像
        if(mt9v03x_tft180_check_finish())
        {
            // 获取图像数据指针
            uint8 *image_data = mt9v03x_tft180_get_image();
            
            // 显示图像到 TFT180 显示屏
            // 注意：直接显示 188*120 分辨率会显示不全，建议使用 160*128
            mt9v03x_tft180_display(image_data, 160, 128);
            
            // 清除采集完成标志
            mt9v03x_tft180_clear_finish_flag();
        }
    }
}

/**
 * @brief 主函数示例
 * @return 无
 */
int core0_main_example(void)
{
    // 初始化时钟和调试串口
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    
    // 运行 MT9V03X 摄像头与 TFT180 显示屏示例
    mt9v03x_tft180_example();
    
    return 0;
}
