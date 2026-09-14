# TC264 Edge-Perception & Real-Time Motion Control Platform

> 基于 Infineon TC264D（200 MHz）+ MT9V034 灰度相机的自主移动机器人平台：图像采集、边缘感知、路径决策与运动控制全链路在设备端实时运行，无需上位机参与。

![Platform](https://img.shields.io/badge/Platform-Infineon%20TC264D-blue)
![IDE](https://img.shields.io/badge/IDE-AURIX%20Development%20Studio-orange)
![License](https://img.shields.io/badge/License-MIT-green)

## 项目简介

这是一个完整的嵌入式视觉机器人项目，演示了如何在资源受限的 MCU 上完成从图像采集到运动控制的全链路实时处理：

- **实时循迹** —— 摄像头采集赛道图像，OTSU 二值化 + 连通域边线提取，双闭环 PID 控制转向与速度
- **直角弯通过** —— 特征判别算法处理直角弯道
- **干扰目标过滤** —— 识别路径上的干扰目标，经"窗口特征 + 多帧确认"后按直线通过
- **多岔路自动遍历** —— DFS 图搜索 + 编码器距离指纹，无需额外传感器即可完成多支路环境导航
- **WiFi 图传** —— 摄像头图像实时回传 PC 上位机，辅助调试

## 硬件配置

| 组件 | 型号/规格 | 说明 |
|------|-----------|------|
| 主控芯片 | Infineon TC264D | AURIX TriCore, 200 MHz |
| 摄像头 | MT9V034 总钻风 | 188×120 灰度，DMA 采集 |
| 电机驱动 | VNH5019 双 H 桥 | PWM 10kHz，前进/后退方向控制 |
| 编码器 | 390 PPR × 30 减速比 | 正交解码测速 |
| 显示屏 | TFT180 | 实时图像与调试信息显示 |
| WiFi 图传 | 逐飞 WiFi SPI 模块 | TCP 传输到 PC 上位机 |
| 供电 | 3S 锂电池 | 11.1V 标称电压 |

## 软件环境

- **IDE**: AURIX Development Studio（ADS v1.10.2）
- **编译器**: TASKING
- **库版本**: Seekfree TC264 开源库 V3.4.5
- **语言**: C（嵌入式）

## 快速开始

### 1. 环境搭建

1. 安装 AURIX Development Studio：https://www.infineon.com/aurixdevelopmentstudio
2. 安装 Git：https://git-scm.com/
3. 克隆本仓库：

```bash
git clone https://github.com/yuqianzhao07-bby/tc264-edge-motion-control.git
```

### 2. 导入与编译

1. 打开 ADS → `File` → `Import` → `General` → `Existing Projects into Workspace`
2. `Browse` 选择仓库根目录（含 `.project` 文件的一层）
3. `Project` → `Build Project`，确认 0 errors

### 3. 烧录与运行

1. 连接 DAP 调试器（注意排线 1 脚方向）
2. 使用工具栏烧录按钮下载固件
3. 上电后观察 TFT180 屏幕出现图像，电机进入循迹状态

详细的环境搭建踩坑记录见系列博客：[CSDN 博客](https://blog.csdn.net/2502_94289764?type=blog)

## 项目结构

```
├── 02_Start/                  # ADS 工程
│   ├── code/                  # 功能模块
│   │   ├── image.c/h          # 图像采集与预处理
│   │   ├── track_way.c/h      # 巡线与路径决策
│   │   ├── component_recognition.c/h  # 干扰目标识别
│   │   ├── pid_controller.c/h # PID 控制器
│   │   ├── motor_driver/      # 双 H 桥电机驱动 + 编码器
│   │   ├── imu_driver/        # IMU 驱动（预留，未启用）
│   │   ├── wifi_image_transfer.c/h    # WiFi 图传
│   │   └── mymath.c/h         # 数学工具
│   ├── user/                  # 主程序与中断
│   │   ├── cpu0_main.c        # CPU0 主函数
│   │   ├── cpu1_main.c        # CPU1（预留）
│   │   └── isr.c              # 中断服务（VSYNC/DMA/PIT）
│   └── libraries/             # 逐飞库引用入口
├── 03_Docs/                   # 工程文档（15 份，见下表）
├── 04_Seekfree  Library/      # 逐飞 TC264 开源库（GPL-3.0）
├── .project / .cproject       # ADS 工程配置
└── build.sh                   # 命令行编译脚本
```

## 工程文档

这个仓库最大的特色是**代码与文档同步交付**，15 份文档沉淀了完整开发过程：

| 文档 | 说明 |
|------|------|
| [项目核心架构.md](03_Docs/项目核心架构.md) | 系统架构、模块设计、算法详解 |
| [项目开发记录.md](03_Docs/项目开发记录.md) | 1600+ 行开发记录：问题/方案/设计思路 |
| [参数速查表.md](03_Docs/参数速查表.md) | 所有可调参数集中管理，调试必看 |
| [硬件接线图.md](03_Docs/硬件接线图.md) | 完整引脚接线表 + 冲突检查 |
| [踩坑错误总结.md](03_Docs/踩坑错误总结.md) | 真实踩坑记录与排查过程 |
| [PID调参记录表.md](03_Docs/PID调参记录表.md) | PID 参数调优历史 |
| [双层架构详细分析.md](03_Docs/双层架构详细分析.md) | 采集/控制双层架构拆解 |
| [架构验证报告.md](03_Docs/架构验证报告.md) | 架构设计验证与复查 |
| [代码审查报告.md](03_Docs/代码审查报告.md) | 静态审查问题清单与修复 |
| [电路元素识别.md](03_Docs/电路元素识别.md) | 干扰目标识别算法设计 |
| [T字路口逻辑详解.docx](03_Docs/T字路口逻辑详解.docx) | 路口判别算法详解 |
| [岔路口导航逻辑说明.docx](03_Docs/岔路口导航逻辑说明.docx) | DFS 导航逻辑说明 |
| [Loop_Engineering.docx](03_Docs/Loop_Engineering.docx) | 循环工程化设计 |
| [Loop复查报告_第二轮.md](03_Docs/Loop复查报告_第二轮_2026-07-08.md) | 第二轮审查修复记录 |
| [改动记录.md](03_Docs/改动记录_2026-07-08_Loop审查修复.md) | 本轮变更明细 |

## 关键参数速览

```c
// 速度参数 (cpu0_main.c)
BASE_SPEED = 750            // 直道基础速度
DUTY_MIN = 500              // 最低占空比（负载下再低转不动）
DUTY_MAX = 1800             // 最高占空比

// 转向 PID (pid_controller.h) —— 分段参数
STEER_KP = 5    STEER_KD = 11          // 直道
STEER_KP_CURVE = 10    STEER_KD_CURVE = 13     // 弯道
STEER_KP_RIGHT_ANGLE = 12    STEER_KD_RIGHT_ANGLE = 14  // 直角弯

// 速度 PID
SPEED_KP = 30    SPEED_KI = 2    SPEED_KD = 6
```

完整参数说明见 [参数速查表](03_Docs/参数速查表.md)。

## 已解决的关键问题

这些问题在 [踩坑错误总结](03_Docs/踩坑错误总结.md) 和 [项目开发记录](03_Docs/项目开发记录.md) 中有完整的"现象 → 定位 → 根因 → 修复"过程：

| 问题 | 现象 | 根因 | 修复 |
|------|------|------|------|
| PWM 跨模块同步冲突 | 左轮编码器始终为 0 | `pwm_init` 对 ATOM 模块同步更新，跨模块初始化干扰已配置通道 | 电机通道统一到同一 ATOM 模块 |
| T 字路口误判 | T 字被当作十字，补线导致冲出路径 | 十字判断条件对 T 字同样成立 | 新增顶部横向赛道检测区分 T 字与十字 |
| 弯道退出检测异常 | 第二次进弯参考值过时 | `first_time` 变量未在状态退出时重置 | 提升变量作用域并在 `ALIGN_NONE` 重置 |
| 积分饱和输出跳变 | 堵转恢复后电机猛冲 | 增量式 PID 积分器无抗饱和 | 增加 ±8000 积分限幅 |

## 开发规范

- 每次提交前确保编译通过（0 errors）
- 提交信息遵循 Conventional Commits（`feat:` / `fix:` / `docs:` / `debug:`）
- 重大更改同步更新 `03_Docs/` 内对应文档

## 许可证

- 本项目代码（`02_Start/code`、`02_Start/user`、`03_Docs`）采用 [MIT](LICENSE) 许可证
- [04_Seekfree  Library](04_Seekfree%20%20Library/) 目录下的逐飞 TC264 开源库遵循 **GPL-3.0** 许可证，版权归逐飞科技所有，使用时请保留其版权声明

## 作者

**赵雨乾** —— 通信工程在读，嵌入式方向

- 邮箱：yuqianzhao07@gmail.com
- 博客：[CSDN](https://blog.csdn.net/2502_94289764?type=blog)

## 致谢

- [逐飞科技](https://seekfree.taobao.com/)提供的 TC264 开源库
- Infineon 提供的 AURIX 开发工具链
