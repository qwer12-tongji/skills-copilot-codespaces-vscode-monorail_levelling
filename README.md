# 单轨车辆调平系统 Monorail Vehicle Leveling System

本项目为跨座式单轨列车的车辆调平系统，通过传感器测量和算法计算，确定车辆姿态和所需的垫片调整量，以实现车辆精确调平。

This project provides a vehicle leveling system for straddle-type monorail trains. It uses sensor measurements and computational algorithms to determine vehicle attitude and required shim adjustments for precise leveling.

## 系统概述 System Overview

### 车辆配置 Vehicle Configuration

- **单轴转向架** (Single-axle Bogie): 每节车体包含2个转向架
  - 走行轮 (Running Wheels): 2个/转向架
  - 导向轮 (Guide Wheels): 4个/转向架
  - 稳定轮 (Stabilizing Wheels): 2个/转向架
  
- **调平位置** (Leveling Positions): 共16个位置
  - 水平垫片位置 (Horizontal Shims): 12个 (4导向轮 + 2稳定轮) × 2转向架
  - 垂向垫片位置 (Vertical Shims): 4个 (空气弹簧位置)

### 测量系统 Measurement System

- **称重传感器** (Load Cells): 测量各轮位载荷
- **水平激光位移传感器** (Horizontal Laser Sensors): 12个，测量车体与轨道梁横向间距
- **垂向激光位移传感器** (Vertical Laser Sensors): 4个，测量车体四角与轨道梁垂向间距

### 调平目标 Leveling Objectives

消除以下因素导致的车辆不平：
- 车体制造误差 (Manufacturing tolerances)
- 转向架重心偏差 (Bogie center of gravity deviation)
- 车体重心偏差 (Vehicle body center of gravity deviation)

## 文件结构 File Structure

```
.
├── MonorailLevelingConfig.m     # 系统配置类
├── SensorData.m                 # 传感器数据类
├── VehicleAttitude.m            # 车辆姿态类
├── MonorailLevelingSystem.m     # 主调平系统类
├── example_leveling.m           # 示例脚本
└── README.md                    # 本文档
```

## 快速开始 Quick Start

### 环境要求 Requirements

- MATLAB R2018b 或更高版本
- 无需额外工具箱

### 运行示例 Run Example

在MATLAB命令窗口中运行：

```matlab
example_leveling
```

这将：
1. 创建系统配置
2. 生成模拟传感器数据（包含重心偏差和姿态偏差）
3. 计算车辆姿态
4. 计算所需的垫片调整量
5. 显示结果并生成可视化图表
6. 保存结果到文件

## 使用说明 Usage Guide

### 基本用法 Basic Usage

```matlab
% 1. 创建配置
config = MonorailLevelingConfig();

% 2. 创建调平系统
system = MonorailLevelingSystem(config);

% 3. 准备传感器数据
sensorData = SensorData();
sensorData = sensorData.loadFromMeasurement(...
    runningWheelLoads, ...        % 走行轮载荷 [4×1]
    guideWheelLoads, ...           % 导向轮载荷 [8×1]
    stabilizingWheelLoads, ...     % 稳定轮载荷 [4×1]
    horizontalDisplacements, ...   % 水平位移 [12×1]
    verticalDisplacements);        % 垂向位移 [4×1]

% 4. 更新系统数据
system = system.updateSensorData(sensorData);

% 5. 计算车辆姿态
system = system.calculateAttitude();

% 6. 计算垫片调整量
system = system.calculateShimAdjustments();

% 7. 显示结果
system.displayResults();

% 8. 可视化
fig = system.visualizeSystem();
```

### 自定义配置 Custom Configuration

```matlab
% 创建配置并修改参数
config = MonorailLevelingConfig();

% 修改车辆尺寸
config.vehicleLength = 15000;  % mm
config.vehicleWidth = 3000;    % mm
config.bogieSpacing = 10000;   % mm

% 修改质量参数
config.vehicleBodyMass = 30000;  % kg
config.bogieMass = 2500;         % kg

% 修改公差参数
config.levelingTolerance = 0.5;   % mm
config.maxShimThickness = 15.0;   % mm
config.shimIncrement = 0.25;      % mm

% 使用自定义配置创建系统
system = MonorailLevelingSystem(config);
```

## 核心算法 Core Algorithms

### 1. 姿态计算 Attitude Calculation

系统通过传感器数据计算以下参数：

- **横摆角 (Roll)**: 从垂向激光传感器数据，使用最小二乘平面拟合
- **俯仰角 (Pitch)**: 从垂向激光传感器数据，使用最小二乘平面拟合
- **重心位置 (COG)**: 从载荷数据和位置信息计算质心
- **重心偏差**: 实际重心与理论重心的差值
- **转向架重心偏差**: 分别计算两个转向架的重心偏差

计算方法：
```
重心X = Σ(载荷i × Xi) / Σ(载荷i)
重心Y = Σ(载荷i × Yi) / Σ(载荷i)
```

### 2. 垫片调整计算 Shim Adjustment Calculation

**垂向调整** (空气弹簧位置):
- 使用垂向激光传感器数据
- 目标：使四角高度一致
- 调整量 = 测量位移 - 平均位移

**水平调整** (导向轮和稳定轮位置):
- 使用水平激光传感器数据
- 补偿横摆角影响
- 考虑重心横向偏差进行微调

**重心偏差补偿**:
- 横向偏差 > 10mm: 在偏移反方向增加垫片
- 纵向偏差 > 10mm: 调整前后垫片差异

### 3. 优化算法 Optimization

使用迭代方法优化垫片调整量：
- 最大迭代次数: 可配置 (默认100次)
- 收敛判据: 相邻迭代调整量变化 < 0.1mm

## 输出结果 Output

### 控制台输出 Console Output

```
========== 单轨车辆调平系统结果 ==========

车辆姿态 Vehicle Attitude:
  位置 Position (mm): [x, y, z]
  横摆角 Roll (deg): xxx
  俯仰角 Pitch (deg): xxx
  偏航角 Yaw (deg): xxx
  重心偏差 COG Deviation (mm): [dx, dy, dz]
  外部力矩 External Moments (N·m): [Mx, My, Mz]
  
垫片调整量 Shim Adjustments:
  水平垫片 Horizontal Shims (mm):
    位置 1-12: xxx
  垂向垫片 Vertical Shims (mm):
    位置 1-4: xxx
    
调平状态 Leveling Status: 已调平 ✓ / 未调平 ✗
```

### 可视化图表 Visualization

生成6个子图：
1. 俯视图 (Top View) - 显示车体轮廓和重心位置
2. 侧视图 (Side View) - 显示俯仰角
3. 前视图 (Front View) - 显示横摆角
4. 载荷分布 (Load Distribution) - 3D散点图
5. 垫片调整 (Shim Adjustments) - 柱状图
6. 重心位置 (Center of Gravity) - 3D图显示偏差

### 输出文件 Output Files

- `leveling_system_visualization.png` - 可视化图表
- `leveling_results.mat` - MATLAB数据文件
- `shim_adjustments.csv` - 垫片调整量CSV表格

## 技术细节 Technical Details

### 坐标系统 Coordinate System

- **原点**: 车体几何中心
- **X轴**: 车辆纵向，向前为正
- **Y轴**: 车辆横向，向右为正
- **Z轴**: 车辆垂向，向上为正

### 单位 Units

- 长度/位移: mm (毫米)
- 质量: kg (千克)
- 力/载荷: N (牛顿)
- 力矩: N·m (牛顿米)
- 角度: rad (弧度) 或 deg (度)

### 精度要求 Precision Requirements

- 姿态角度公差: ±0.1°
- 重心偏差公差: ±5 mm
- 垫片调整精度: 0.5 mm

## 扩展功能 Extensions

系统设计支持以下扩展：

1. **实时数据采集**: 可连接实际传感器获取实时数据
2. **历史数据分析**: 可记录和分析多次测量数据
3. **自动调整**: 可与执行机构连接实现自动调平
4. **报告生成**: 可生成PDF/Word调平报告

## 注意事项 Notes

1. 确保传感器数据有效（载荷 > 0，位移在合理范围内）
2. 垫片调整量受最大厚度限制
3. 建议在调整后重新测量验证
4. 对于大偏差情况，可能需要多次迭代调整

## 许可证 License

本项目仅供学习和研究使用。

## 作者 Author

单轨车辆调平系统开发团队

## 更新日志 Changelog

### Version 1.0.0 (2025-11-06)
- 初始版本发布
- 实现基本调平算法
- 支持姿态计算和垫片调整
- 包含示例和可视化功能
