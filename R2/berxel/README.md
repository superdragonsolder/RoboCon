# HowLong - 深度相机楼梯检测系统

基于 Berxel 100R 深度相机的距离测量与楼梯下台阶检测系统。

## 功能概述

本项目包含三个核心功能：

### 1. 校准工具 (calibrate.py)
测量并计算深度像素到实际距离的转换因子。

### 2. 距离检测器 (distance_detector.py)
实时显示彩色视图并测量中心区域距离。

### 3. 楼梯检测器 (stairs_detector.py)
检测台阶并发送下台阶指令。

### 4. 主测量工具 (main.py)
综合距离测量与可视化工具。

## 硬件要求

- Berxel 100R 深度相机
- USB 3.0 连接
- Windows 或 Linux 系统

## 软件依赖

```
pip install pyserial numpy opencv-python
```

## 使用说明

### 第一步：校准转换因子

```bash
python calibrate.py
```

使用步骤：
1. 将相机固定在已知高度位置
2. 使用尺子测量实际距离
3. 点击画面选择测量区域
4. 按 Enter 键开始测量
5. 输入实际距离（cm）
6. 改变高度，重复测量3次
7. 程序自动计算平均转换因子

校准完成后会显示转换因子值，例如：`8050.50`

### 第二步：测量距离

```bash
# 使用默认转换因子
python distance_detector.py

# 使用校准后的转换因子
python distance_detector.py --conversion-factor 8050.50
```

功能特点：
- 显示彩色视图
- 实时测量中心区域距离
- 显示有效像素数量

### 第三步：楼梯检测

```bash
python stairs_detector.py --conversion-factor 8050.50
```

参数说明：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--step-height` | 台阶高度（cm） | 20 |
| `--tolerance` | 容差范围（cm） | 2 |
| `--confirm-frames` | 确认帧数 | 3 |
| `--serial-port` | 串口端口 | None |
| `--baudrate` | 波特率 | 9600 |
| `--conversion-factor` | 转换因子 | 7900.0 |
| `--roi-size` | 测量区域大小 | 10 |

完整示例：

```bash
python stairs_detector.py \
    --conversion-factor 8050.50 \
    --step-height 20 \
    --tolerance 2 \
    --serial-port COM3
```

### 主测量工具

```bash
python main.py
```

显示深度图、彩色图和实时距离统计。

## 检测原理

当相机垂直向下安装时：
- 正常情况下测量的是到地面的距离
- 当距离突然增加约20cm时，表示相机经过台阶边缘
- 系统检测到距离变化在 18-22cm 范围内时，发送 STEP_DOWN 指令

```
距离变化 = 现在距离 - 前一刻距离
目标范围 = 20cm ± 2cm (18-22cm)
```

## Linux 部署

```bash
# 安装依赖
sudo apt update
sudo apt install python3-pip
pip3 install pyserial numpy opencv-python

# USB 权限配置
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0501", MODE="0666"' | sudo tee /etc/udev/rules.d/99-berxel.rules
sudo udevadm control --reload-rules

# 运行
python3 stairs_detector.py --conversion-factor 8050.50
```

## 项目结构

```
howlong/
├── calibrate.py              # 校准工具
├── distance_detector.py      # 距离检测器
├── stairs_detector.py        # 楼梯检测器
├── main.py                   # 主测量工具
├── src/
│   ├── camera_manager.py     # 相机管理
│   ├── distance_measurer.py  # 距离测量
│   └── visualizer.py         # 可视化
└── BerxelSdkDriver/          # SDK驱动
```

## 常见问题

### 相机无法打开
- 检查 USB 连接
- 关闭其他使用相机的程序
- 尝试管理员权限运行

### 距离测量不准确
- 运行校准工具重新校准
- 确保转换因子设置正确

### 楼梯检测不灵敏
- 调整 `--confirm-frames` 参数
- 调整 `--tolerance` 容差范围
