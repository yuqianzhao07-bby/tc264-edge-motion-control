/*********************************************************************************************************************
* TC264 Opensourec Library (TC264 开源库) - 一个基于英飞凌 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件为 MT9V03X 摄像头与 TFT180 显示屏驱动的核心功能头文件
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

#ifndef MT9V03X_TFT180_DISPLAY_H
#define MT9V03X_TFT180_DISPLAY_H

#include "zf_common_headfile.h"

// 函数声明

/**
 * @brief 初始化 MT9V03X 摄像头和 TFT180 显示屏
 * @return 初始化结果，0 表示成功，非 0 表示失败
 */
uint8 mt9v03x_tft180_init(void);

/**
 * @brief 显示摄像头图像到 TFT180 显示屏
 * @param image_data 图像数据指针
 * @param width 显示宽度
 * @param height 显示高度
 * @return 显示结果，0 表示成功，非 0 表示失败
 */
uint8 mt9v03x_tft180_display(uint8 *image_data, uint16 width, uint16 height);

/**
 * @brief 检查摄像头是否采集完成一帧图像
 * @return 采集状态，1 表示采集完成，0 表示未完成
 */
uint8 mt9v03x_tft180_check_finish(void);

/**
 * @brief 清除摄像头采集完成标志
 */
void mt9v03x_tft180_clear_finish_flag(void);

/**
 * @brief 获取摄像头图像数据指针
 * @return 图像数据指针
 */
uint8 *mt9v03x_tft180_get_image(void);

#endif // MT9V03X_TFT180_DISPLAY_H
