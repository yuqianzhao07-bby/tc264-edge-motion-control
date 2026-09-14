/*********************************************************************************************************************
* WiFi SPI 图传模块
* 封装 WiFi SPI + 逐飞助手图传协议，提供简洁的初始化和发送接口
* 参考工程: E07_09_wifi_spi_mt9v03x_demo
*********************************************************************************************************************/

#include "wifi_image_transfer.h"
#include "zf_device_wifi_spi.h"
#include "zf_device_mt9v03x_double.h"
#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"
#include "debug_trace.h"
#include "image.h"

// 连接状态
static uint8 s_wifi_connected = 0;
static uint8 s_frame_count = 0;

// 边界线配置状态
static uint8 s_boundary_configured = 0;

//-------------------------------------------------------------------------------------------------------------------
// @brief       初始化 WiFi SPI 图传
//              流程: WiFi连接 -> TCP连接 -> 逐飞助手接口初始化 -> 图像参数配置
// @return      0=成功, 1=WiFi连接失败, 2=TCP连接失败
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_image_transfer_init(void)
{
    s_wifi_connected = 0;
    s_frame_count = 0;

    DEBUG_PRINTF("[WIFI_IMG] 初始化WiFi SPI图传...\r\n");
    DEBUG_PRINTF("[WIFI_IMG]   SSID: %s\r\n", WIFI_TRANSFER_SSID);
    DEBUG_PRINTF("[WIFI_IMG]   目标: %s:%s\r\n", WIFI_TRANSFER_TARGET_IP, WIFI_TRANSFER_TARGET_PORT);

    // 1. 连接WiFi热点
    DEBUG_PRINTF("[WIFI_IMG] 正在连接WiFi热点...\r\n");
    if(wifi_spi_init(WIFI_TRANSFER_SSID, WIFI_TRANSFER_PASSWORD))
    {
        DEBUG_PRINTF("[WIFI_IMG] WiFi连接失败!\r\n");
        return 1;
    }

    DEBUG_PRINTF("[WIFI_IMG] WiFi连接成功\r\n");
    DEBUG_PRINTF("[WIFI_IMG]   固件版本: %s\r\n", wifi_spi_version);
    DEBUG_PRINTF("[WIFI_IMG]   MAC地址:  %s\r\n", wifi_spi_mac_addr);
    DEBUG_PRINTF("[WIFI_IMG]   IP地址:   %s\r\n", wifi_spi_ip_addr_port);

    // 2. 手动连接TCP服务器 (使用模块头文件中配置的IP和端口)
    DEBUG_PRINTF("[WIFI_IMG] 连接TCP服务器 %s:%s ...\r\n", WIFI_TRANSFER_TARGET_IP, WIFI_TRANSFER_TARGET_PORT);
    while(wifi_spi_socket_connect(
        "TCP",
        WIFI_TRANSFER_TARGET_IP,
        WIFI_TRANSFER_TARGET_PORT,
        WIFI_TRANSFER_LOCAL_PORT))
    {
        DEBUG_PRINTF("[WIFI_IMG] TCP连接失败, 重试...\r\n");
        system_delay_ms(100);
    }

    DEBUG_PRINTF("[WIFI_IMG] TCP连接成功\r\n");

    // 3. 初始化逐飞助手图传接口 (WiFi SPI模式)
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    DEBUG_PRINTF("[WIFI_IMG] 逐飞助手接口初始化完成 (WiFi SPI模式)\r\n");

    // 4. 配置图像参数
#if WIFI_TRANSFER_IMAGE_SOURCE == 1
    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,     // 灰度格式发送二值图 (每像素1字节, 0=黑255=白)
        bin_image[0],
        image_w,
        image_h);
    DEBUG_PRINTF("[WIFI_IMG] 图像参数配置完成 (%dx%d 二值化)\r\n", image_w, image_h);
#else
    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        mt9v03x_image_1[0],
        MT9V03X_1_W,
        MT9V03X_1_H);
    DEBUG_PRINTF("[WIFI_IMG] 图像参数配置完成 (%dx%d 灰度)\r\n", MT9V03X_1_W, MT9V03X_1_H);
#endif

    s_wifi_connected = 1;
    DEBUG_PRINTF("[WIFI_IMG] 图传初始化完成!\r\n");

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       发送当前 MT9V03X 摄像头1的帧 (简化接口)
//-------------------------------------------------------------------------------------------------------------------
void wifi_image_transfer_send_frame(void)
{
    if(!s_wifi_connected)
        return;

    s_frame_count++;
    if(s_frame_count >= WIFI_TRANSFER_SEND_INTERVAL)
    {
        s_frame_count = 0;
        seekfree_assistant_camera_send();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       发送指定图像到上位机
// @param       image       图像数据指针
// @param       width       图像宽度
// @param       height      图像高度
//-------------------------------------------------------------------------------------------------------------------
void wifi_image_transfer_send(uint8 *image, uint16 width, uint16 height)
{
    if(!s_wifi_connected)
        return;

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        image,
        width,
        height);

    seekfree_assistant_camera_send();
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       检查 WiFi 图传是否已连接
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_image_transfer_is_connected(void)
{
    return s_wifi_connected;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       获取 WiFi SPI 模块固件版本
//-------------------------------------------------------------------------------------------------------------------
const char *wifi_image_transfer_get_version(void)
{
    return wifi_spi_version;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       获取 WiFi SPI 模块 IP 地址
//-------------------------------------------------------------------------------------------------------------------
const char *wifi_image_transfer_get_ip(void)
{
    return wifi_spi_ip_addr_port;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       获取 WiFi SPI 模块 MAC 地址
//-------------------------------------------------------------------------------------------------------------------
const char *wifi_image_transfer_get_mac(void)
{
    return wifi_spi_mac_addr;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       配置边界线信息 (用于在图传中显示巡线边界)
//-------------------------------------------------------------------------------------------------------------------
void wifi_image_transfer_set_boundary(seekfree_assistant_boundary_type_enum boundary_type, uint16 dot_num,
                                      void *x1_boundary, void *x2_boundary, void *x3_boundary,
                                      void *y1_boundary, void *y2_boundary, void *y3_boundary)
{
    if(!s_wifi_connected)
        return;

    seekfree_assistant_camera_boundary_config(boundary_type, dot_num,
                                              x1_boundary, x2_boundary, x3_boundary,
                                              y1_boundary, y2_boundary, y3_boundary);

    s_boundary_configured = 1;
    DEBUG_PRINTF("[WIFI_IMG] 边界线配置完成, 类型=%d, 点数=%d\r\n", boundary_type, dot_num);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief       清除边界线配置 (不再显示边界线)
//-------------------------------------------------------------------------------------------------------------------
void wifi_image_transfer_clear_boundary(void)
{
    if(!s_wifi_connected)
        return;

    seekfree_assistant_camera_boundary_config(NO_BOUNDARY, 0, NULL, NULL, NULL, NULL, NULL, NULL);

    s_boundary_configured = 0;
    DEBUG_PRINTF("[WIFI_IMG] 边界线配置已清除\r\n");
}

// ==========================================
// 图像叠加: 在二值化图像上绘制识别结果
// ==========================================

#if WIFI_TRANSFER_OVERLAY_ENABLE

#include "component_recognition.h"

// 3x5 像素字体 (0-9, A-Z, 空格, %)
// 每个字符3列x5行, 按列存储 (MSB=顶)
static const uint8 tiny_font[][5] = {
    // 0-9
    {0x7, 0x5, 0x5, 0x5, 0x7},  // 0
    {0x2, 0x6, 0x2, 0x2, 0x7},  // 1
    {0x7, 0x1, 0x7, 0x4, 0x7},  // 2
    {0x7, 0x1, 0x7, 0x1, 0x7},  // 3
    {0x5, 0x5, 0x7, 0x1, 0x1},  // 4
    {0x7, 0x4, 0x7, 0x1, 0x7},  // 5
    {0x7, 0x4, 0x7, 0x5, 0x7},  // 6
    {0x7, 0x1, 0x1, 0x1, 0x1},  // 7
    {0x7, 0x5, 0x7, 0x5, 0x7},  // 8
    {0x7, 0x5, 0x7, 0x1, 0x7},  // 9
    // A-Z (10-35)
    {0x2, 0x5, 0x7, 0x5, 0x5},  // A
    {0x6, 0x5, 0x6, 0x5, 0x6},  // B
    {0x7, 0x4, 0x4, 0x4, 0x7},  // C
    {0x6, 0x5, 0x5, 0x5, 0x6},  // D
    {0x7, 0x4, 0x7, 0x4, 0x7},  // E
    {0x7, 0x4, 0x7, 0x4, 0x4},  // F
    {0x7, 0x4, 0x5, 0x5, 0x7},  // G
    {0x5, 0x5, 0x7, 0x5, 0x5},  // H
    {0x7, 0x2, 0x2, 0x2, 0x7},  // I
    {0x1, 0x1, 0x1, 0x5, 0x7},  // J
    {0x5, 0x5, 0x6, 0x5, 0x5},  // K
    {0x4, 0x4, 0x4, 0x4, 0x7},  // L
    {0x5, 0x7, 0x7, 0x5, 0x5},  // M
    {0x5, 0x7, 0x7, 0x7, 0x5},  // N
    {0x7, 0x5, 0x5, 0x5, 0x7},  // O
    {0x7, 0x5, 0x7, 0x4, 0x4},  // P
    {0x7, 0x5, 0x5, 0x7, 0x7},  // Q
    {0x7, 0x5, 0x7, 0x6, 0x5},  // R
    {0x7, 0x4, 0x7, 0x1, 0x7},  // S
    {0x7, 0x2, 0x2, 0x2, 0x2},  // T
    {0x5, 0x5, 0x5, 0x5, 0x7},  // U
    {0x5, 0x5, 0x5, 0x5, 0x2},  // V
    {0x5, 0x5, 0x7, 0x7, 0x5},  // W
    {0x5, 0x5, 0x2, 0x5, 0x5},  // X
    {0x5, 0x5, 0x7, 0x2, 0x2},  // Y
    {0x7, 0x1, 0x2, 0x4, 0x7},  // Z
    // special
    {0x0, 0x0, 0x0, 0x0, 0x0},  // 36: space
    {0x7, 0x1, 0x7, 0x0, 0x0},  // 37: %
};

// 在图像上画一个像素点
static inline void draw_pixel(uint8 *img, uint16 w, uint16 h, int16 x, int16 y, uint8 color)
{
    if (x >= 0 && x < w && y >= 0 && y < h)
        img[y * w + x] = color;
}

// 画矩形框
static void draw_rect(uint8 *img, uint16 w, uint16 h,
                       int16 x1, int16 y1, int16 x2, int16 y2, uint8 color)
{
    int16 i;
    for (i = x1; i <= x2; i++) { draw_pixel(img, w, h, i, y1, color); draw_pixel(img, w, h, i, y2, color); }
    for (i = y1; i <= y2; i++) { draw_pixel(img, w, h, x1, i, color); draw_pixel(img, w, h, x2, i, color); }
}

// 画一个3x5字符, 返回下一个字符的x位置
static int16 draw_char(uint8 *img, uint16 w, uint16 h, int16 x, int16 y, char c, uint8 color)
{
    int idx = -1;
    if (c >= '0' && c <= '9') idx = c - '0';
    else if (c >= 'A' && c <= 'Z') idx = c - 'A' + 10;
    else if (c >= 'a' && c <= 'z') idx = c - 'a' + 10;  // 小写当大写
    else if (c == ' ') idx = 36;
    else if (c == '%') idx = 37;

    if (idx >= 0 && idx < 38)
    {
        int16 cx, cy;
        for (cy = 0; cy < 5; cy++)
        {
            uint8 bits = tiny_font[idx][cy];
            for (cx = 0; cx < 3; cx++)
            {
                if (bits & (4 >> cx))
                    draw_pixel(img, w, h, x + cx, y + cy, color);
            }
        }
    }
    return x + 4;  // 3px char + 1px gap
}

// 画字符串
static void draw_string(uint8 *img, uint16 w, uint16 h, int16 x, int16 y, const char *s, uint8 color)
{
    while (*s)
    {
        x = draw_char(img, w, h, x, y, *s, color);
        s++;
    }
}

// 画数字 (int转字符串)
static void draw_number(uint8 *img, uint16 w, uint16 h, int16 x, int16 y, int num, uint8 color)
{
    char buf[8];
    int i = 0;
    if (num < 0) { buf[i++] = '-'; num = -num; }
    if (num == 0) { buf[i++] = '0'; }
    else
    {
        char tmp[8]; int j = 0;
        while (num > 0) { tmp[j++] = '0' + num % 10; num /= 10; }
        while (j > 0) { buf[i++] = tmp[--j]; }
    }
    buf[i] = 0;
    draw_string(img, w, h, x, y, buf, color);
}

// ==========================================
// 主函数: 在图像上叠加元器件识别结果
// ==========================================
void wifi_image_transfer_overlay_result(uint8 *image, uint16 w, uint16 h,
                                         uint8 comp_type, uint8 confidence,
                                         int16 bbox_x, int16 bbox_y,
                                         int16 bbox_w, int16 bbox_h)
{
    if (!image || comp_type == COMPONENT_NONE || comp_type == COMPONENT_UNKNOWN)
        return;

    // 1. 画边框 (围绕元器件)
    if (bbox_w > 0 && bbox_h > 0)
    {
        draw_rect(image, w, h,
                  bbox_x - 1, bbox_y - 1,
                  bbox_x + bbox_w, bbox_y + bbox_h,
                  255);
    }

    // 2. 在图像顶部画名称+置信度
    //    格式: "RES 85%" 或 "CAP 70%"
    const char *name = component_get_name(comp_type);

    // 清除顶部2行 (作为文字背景)
    int16 i, j;
    for (j = 0; j < 7; j++)  // 7行: 5px字 + 2px边距
        for (i = 0; i < w; i++)
            image[j * w + i] = 0;

    // 画名称 (左上角)
    draw_string(image, w, h, 1, 1, name, 255);

    // 画置信度 (名称右边)
    int16 name_len = 0;
    const char *p = name;
    while (*p) { name_len++; p++; }
    int16 conf_x = 1 + name_len * 4 + 2;
    draw_number(image, w, h, conf_x, 1, confidence, 255);
    draw_string(image, w, h, conf_x + (confidence >= 10 ? 8 : 4), 1, "%", 255);
}

#endif // WIFI_TRANSFER_OVERLAY_ENABLE
