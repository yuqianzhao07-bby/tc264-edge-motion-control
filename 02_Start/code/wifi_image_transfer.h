/*********************************************************************************************************************
* WiFi SPI 图传模块
* 封装 WiFi SPI + 逐飞助手图传协议，提供简洁的初始化和发送接口
* 基于 TC264 + MT9V03X 摄像头 + WiFi SPI 模块 (参考 E07_09_wifi_spi_mt9v03x_demo)
*
* 所有图传配置在此文件中集中管理，修改后重新编译即可
*********************************************************************************************************************/

#ifndef WIFI_IMAGE_TRANSFER_H
#define WIFI_IMAGE_TRANSFER_H

#include "zf_common_headfile.h"
#include "seekfree_assistant.h"

// ======================== WiFi 配置选择 ========================
// 1 = REDMI K80 Pro (手机热点)
// 2 = vivo X23 Magic color (手机热点)
#define WIFI_CONFIG_ID 1

// ======================== 配置1: REDMI K80 Pro ========================
#define WIFI1_SSID          "REDMI K80 Pro"
#define WIFI1_PASSWORD      "07793958"
#define WIFI1_TARGET_IP     "10.206.18.220"
#define WIFI1_TARGET_PORT   "8086"
#define WIFI1_LOCAL_PORT    "6666"

// ======================== 配置2: vivo X23 Magic color ========================
#define WIFI2_SSID          "vivo X23 Magic color"
#define WIFI2_PASSWORD      "88888888"
#define WIFI2_TARGET_IP     "192.168.43.223"
#define WIFI2_TARGET_PORT   "8086"
#define WIFI2_LOCAL_PORT    "6666"

// ======================== 根据选择映射到实际宏 ========================
#if WIFI_CONFIG_ID == 1
  #define WIFI_TRANSFER_SSID          WIFI1_SSID
  #define WIFI_TRANSFER_PASSWORD      WIFI1_PASSWORD
  #define WIFI_TRANSFER_TARGET_IP     WIFI1_TARGET_IP
  #define WIFI_TRANSFER_TARGET_PORT   WIFI1_TARGET_PORT
  #define WIFI_TRANSFER_LOCAL_PORT    WIFI1_LOCAL_PORT
#elif WIFI_CONFIG_ID == 2
  #define WIFI_TRANSFER_SSID          WIFI2_SSID
  #define WIFI_TRANSFER_PASSWORD      WIFI2_PASSWORD
  #define WIFI_TRANSFER_TARGET_IP     WIFI2_TARGET_IP
  #define WIFI_TRANSFER_TARGET_PORT   WIFI2_TARGET_PORT
  #define WIFI_TRANSFER_LOCAL_PORT    WIFI2_LOCAL_PORT
#else
  #error "WIFI_CONFIG_ID 必须为 1 或 2"
#endif

// ======================== 图传参数 ========================
// 图像源: 0=原始灰度图(188x120 检查曝光)  1=二值化图像(60x100 调试赛道)
#define WIFI_TRANSFER_IMAGE_SOURCE   (1)
// 发送间隔: 每N帧发送一次 (1=每帧都发, 5=每5帧发1帧, 避免阻塞电机控制)
#define WIFI_TRANSFER_SEND_INTERVAL (1)

// ======================== 图像叠加参数 ========================
// 在图传图像上叠加识别结果 (元器件名称+置信度+边框)
// 0=关闭 (纯净图像)  1=开启 (调试识别时使用)
#define WIFI_TRANSFER_OVERLAY_ENABLE (1)

// ======================== 边界线配置 ========================
// 边界线类型: 0=无边界线, 1=X边界(只有横坐标), 2=Y边界(只有纵坐标), 3=XY边界(完整坐标), 4=无图像+边界线
#define WIFI_TRANSFER_BOUNDARY_TYPE   (0)

// ================================================================
// PID 参数调参预设
//
// 使用方法: 修改 PID_TUNING_PROFILE 的值即可一键切换整套参数
//   - 这里定义的宏会在 pid_controller.h 之前生效 (#ifndef保护)
//   - 选 CUSTOM 时可自由修改下方各参数
//   - 调参完成后可将最终值回填到对应预设, 方便多场景切换
// ================================================================

// 预设选择:
//   0 = DEFAULT     出厂保守参数 (稳, 适合初调)
//   1 = BALANCED    均衡参数 (日常使用)
//   2 = AGGRESSIVE  激进参数 (高速/大弯道)
//   3 = CUSTOM      自定义 (修改下方 CUSTOM 段)
#define PID_TUNING_PROFILE  0

// ======================== 预设0: DEFAULT 保守 ========================
// 特点: 转向柔和, 阻尼大, 不容易甩尾, 适合初次上赛道验证
#if PID_TUNING_PROFILE == 0
  // --- 转向直道 ---
  #define STEER_KP             400     // 实际=4.00 (柔和)
  #define STEER_KP2            10      // 实际=0.10
  #define STEER_KD             1000    // 实际=10.00
  // --- 转向直角弯 ---
  #define STEER_KP_RIGHT_ANGLE 1000    // 实际=10.00
  #define STEER_KP2_RIGHT_ANGLE 20     // 实际=0.20
  #define STEER_KD_RIGHT_ANGLE 1200    // 实际=12.00
  // --- 陀螺仪 ---
  #define STEER_GKD            60      // 实际=0.60 (强阻尼)
  // --- 速度 ---
  #define SPEED_KP             2500    // 实际=25.00
  #define SPEED_KI             150     // 实际=1.50
  #define SPEED_KD             500     // 实际=5.00
  // --- 限幅 (保守) ---
  #define STEER_OUT_MIN        -1200
  #define STEER_OUT_MAX         1200

// ======================== 预设1: BALANCED 均衡 ========================
// 特点: 转向适中, 兼顾稳定性和响应速度, 适合日常调试
#elif PID_TUNING_PROFILE == 1
  // --- 转向直道 ---
  #define STEER_KP             500     // 实际=5.00
  #define STEER_KP2            15      // 实际=0.15
  #define STEER_KD             1100    // 实际=11.00
  // --- 转向直角弯 ---
  #define STEER_KP_RIGHT_ANGLE 1200    // 实际=12.00
  #define STEER_KP2_RIGHT_ANGLE 25     // 实际=0.25
  #define STEER_KD_RIGHT_ANGLE 1400    // 实际=14.00
  // --- 陀螺仪 ---
  #define STEER_GKD            50      // 实际=0.50
  // --- 速度 ---
  #define SPEED_KP             3000    // 实际=30.00
  #define SPEED_KI             200     // 实际=2.00
  #define SPEED_KD             600     // 实际=6.00
  // --- 限幅 ---
  #define STEER_OUT_MIN        -1600
  #define STEER_OUT_MAX         1600

// ======================== 预设2: AGGRESSIVE 激进 ========================
// 特点: 转向猛, 二次项强, 适合高速跑圈, 需配合陀螺仪防甩尾
#elif PID_TUNING_PROFILE == 2
  // --- 转向直道 ---
  #define STEER_KP             600     // 实际=6.00 (快)
  #define STEER_KP2            20      // 实际=0.20
  #define STEER_KD             1200    // 实际=12.00
  // --- 转向直角弯 ---
  #define STEER_KP_RIGHT_ANGLE 1500    // 实际=15.00 (猛)
  #define STEER_KP2_RIGHT_ANGLE 35     // 实际=0.35
  #define STEER_KD_RIGHT_ANGLE 1600    // 实际=16.00
  // --- 陀螺仪 ---
  #define STEER_GKD            40      // 实际=0.40 (稍弱以允许快转)
  // --- 速度 ---
  #define SPEED_KP             3500    // 实际=35.00
  #define SPEED_KI             300     // 实际=3.00
  #define SPEED_KD             800     // 实际=8.00
  // --- 限幅 (全量程) ---
  #define STEER_OUT_MIN        -1800
  #define STEER_OUT_MAX         1800

// ======================== 预设3: CUSTOM 自定义 ========================
// 修改下方参数, 然后设 PID_TUNING_PROFILE=3 即可生效
#elif PID_TUNING_PROFILE == 3
  // --- 转向直道 ---
  #define STEER_KP             500     // ← 修改这里
  #define STEER_KP2            15      // ← 修改这里
  #define STEER_KD             1100    // ← 修改这里
  // --- 转向直角弯 ---
  #define STEER_KP_RIGHT_ANGLE 1200    // ← 修改这里
  #define STEER_KP2_RIGHT_ANGLE 25     // ← 修改这里
  #define STEER_KD_RIGHT_ANGLE 1400    // ← 修改这里
  // --- 陀螺仪 ---
  #define STEER_GKD            50      // ← 修改这里
  // --- 速度 ---
  #define SPEED_KP             3000    // ← 修改这里
  #define SPEED_KI             200     // ← 修改这里
  #define SPEED_KD             600     // ← 修改这里
  // --- 限幅 ---
  #define STEER_OUT_MIN        -1600   // ← 修改这里
  #define STEER_OUT_MAX         1600   // ← 修改这里
#endif  // PID_TUNING_PROFILE (唯一的 #endif, 关闭整个 #if...#elif 链)

//-------------------------------------------------------------------------------------------------------------------
// @brief       初始化 WiFi SPI 图传
//              流程: WiFi连接 -> TCP连接 -> 逐飞助手接口初始化 -> 图像参数配置
// @return      0=成功, 1=WiFi连接失败, 2=TCP连接失败
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_image_transfer_init(void);

//-------------------------------------------------------------------------------------------------------------------
// @brief       发送一帧摄像头图像到上位机 (使用默认 MT9V03X 参数 188x120)
//              需在主循环中检测到 mt9v03x_finish_flag 后调用
//-------------------------------------------------------------------------------------------------------------------
void wifi_image_transfer_send_frame(void);

//-------------------------------------------------------------------------------------------------------------------
// @brief       发送指定图像到上位机
// @param       image       图像数据指针
// @param       width       图像宽度
// @param       height      图像高度
//-------------------------------------------------------------------------------------------------------------------
void wifi_image_transfer_send(uint8 *image, uint16 width, uint16 height);

//-------------------------------------------------------------------------------------------------------------------
// @brief       检查 WiFi 图传是否已连接
// @return      1=已连接, 0=未连接
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_image_transfer_is_connected(void);

//-------------------------------------------------------------------------------------------------------------------
// @brief       获取 WiFi SPI 模块固件版本 (初始化成功后可用)
//-------------------------------------------------------------------------------------------------------------------
const char *wifi_image_transfer_get_version(void);

//-------------------------------------------------------------------------------------------------------------------
// @brief       获取 WiFi SPI 模块 IP 地址 (初始化成功后可用)
//-------------------------------------------------------------------------------------------------------------------
const char *wifi_image_transfer_get_ip(void);

//-------------------------------------------------------------------------------------------------------------------
// @brief       获取 WiFi SPI 模块 MAC 地址 (初始化成功后可用)
//-------------------------------------------------------------------------------------------------------------------
const char *wifi_image_transfer_get_mac(void);

//-------------------------------------------------------------------------------------------------------------------
// @brief       配置边界线信息 (用于在图传中显示巡线边界)
// @param       boundary_type   边界线类型 (X_BOUNDARY, Y_BOUNDARY, XY_BOUNDARY, NO_BOUNDARY)
// @param       dot_num         边界点数量
// @param       x1_boundary     左边界X坐标数组 (可为NULL)
// @param       x2_boundary     中线X坐标数组 (可为NULL)
// @param       x3_boundary     右边界X坐标数组 (可为NULL)
// @param       y1_boundary     左边界Y坐标数组 (可为NULL)
// @param       y2_boundary     中线Y坐标数组 (可为NULL)
// @param       y3_boundary     右边界Y坐标数组 (可为NULL)
//-------------------------------------------------------------------------------------------------------------------
void wifi_image_transfer_set_boundary(seekfree_assistant_boundary_type_enum boundary_type, uint16 dot_num,
                                      void *x1_boundary, void *x2_boundary, void *x3_boundary,
                                      void *y1_boundary, void *y2_boundary, void *y3_boundary);

//-------------------------------------------------------------------------------------------------------------------
// @brief       清除边界线配置 (不再显示边界线)
//-------------------------------------------------------------------------------------------------------------------
void wifi_image_transfer_clear_boundary(void);

//-------------------------------------------------------------------------------------------------------------------
// @brief       在图像上叠加元器件识别结果 (名称+置信度+边框)
//              需要 WIFI_TRANSFER_OVERLAY_ENABLE=1
// @param       image       二值化图像数组 (会被原地修改)
// @param       w           图像宽度
// @param       h           图像高度
// @param       comp_type   元器件类型 (component_recognition.h 中的定义)
// @param       confidence  置信度 0-100
// @param       bbox_x      边界框左上角x (图像坐标)
// @param       bbox_y      边界框左上角y (图像坐标)
// @param       bbox_w      边界框宽度
// @param       bbox_h      边界框高度
//-------------------------------------------------------------------------------------------------------------------
void wifi_image_transfer_overlay_result(uint8 *image, uint16 w, uint16 h,
                                         uint8 comp_type, uint8 confidence,
                                         int16 bbox_x, int16 bbox_y,
                                         int16 bbox_w, int16 bbox_h);

#endif // WIFI_IMAGE_TRANSFER_H
