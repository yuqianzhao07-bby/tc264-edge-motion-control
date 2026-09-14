#ifndef _debug_trace_h_
#define _debug_trace_h_

#include "zf_common_typedef.h"
#include "zf_common_debug.h"

// 调试计数器（ISR 中递增，主循环中打印）
extern volatile uint32 g_vsync1_count;      // 摄像头1 VSYNC 中断次数
extern volatile uint32 g_vsync2_count;      // 摄像头2 VSYNC 中断次数
extern volatile uint32 g_dma1_count;        // DMA通道1 完成中断次数
extern volatile uint32 g_dma2_count;        // DMA通道2 完成中断次数
extern volatile uint32 g_frame_done_count;  // 图像采集完成帧数
extern volatile uint32 g_frame_process_count; // 主循环已处理的帧数
extern volatile uint32 g_main_loop_count;   // 主循环总迭代次数

// 调试开关：设为 1 开启详细打印，0 关闭
// 注意: 开启时调试信息会通过WiFi SPI发送, 导致串口助手显示乱码
// 建议: 调试图传时设为0, 需要串口调试时设为1
#define DEBUG_PRINTF_ENABLE  0

// 详细打印开关：设为 1 打印每帧 ZIP/BIN/FLT/MED 细节，0 仅打印帧号和阈值
// 115200bps 下每帧详细打印约 500 字符 → 43ms，严重影响流畅度
// 正常运行时建议设为 0
#define DEBUG_VERBOSE  0

#if DEBUG_PRINTF_ENABLE
  #define DEBUG_PRINTF(...) do { \
      char _dbg_buf[256]; \
      int _dbg_len = sprintf(_dbg_buf, __VA_ARGS__); \
      debug_send_buffer((uint8 *)_dbg_buf, (uint32)_dbg_len); \
  } while(0)

  #if DEBUG_VERBOSE
    #define DEBUG_PRINTF_V(...) DEBUG_PRINTF(__VA_ARGS__)
  #else
    #define DEBUG_PRINTF_V(...)  // 静默模式, 不输出
  #endif
#else
  #define DEBUG_PRINTF(...)
  #define DEBUG_PRINTF_V(...)
#endif

#endif
