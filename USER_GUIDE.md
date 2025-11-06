# 单轨车辆调平系统使用指南

## 目录

1. [系统简介](#系统简介)
2. [快速入门](#快速入门)
3. [详细使用说明](#详细使用说明)
4. [算法原理](#算法原理)
5. [常见问题](#常见问题)
6. [技术支持](#技术支持)

## 系统简介

### 系统功能

单轨车辆调平系统是一套完整的MATLAB算法工具，用于：

1. **采集和处理传感器数据**
   - 16个称重传感器（走行轮、导向轮、稳定轮）
   - 12个水平激光位移传感器
   - 4个垂向激光位移传感器

2. **计算车辆姿态参数**
   - 横摆角（Roll）
   - 俯仰角（Pitch）
   - 偏航角（Yaw）
   - 重心位置及偏差
   - 转向架重心偏差

3. **优化垫片调整方案**
   - 12个水平垫片位置（导向轮+稳定轮）
   - 4个垂向垫片位置（空气弹簧）
   - 自动计算最优调整量

4. **可视化分析**
   - 多视角车辆姿态展示
   - 载荷分布3D图
   - 垫片调整量分布
   - 重心位置偏差

### 适用场景

- 新车出厂调平
- 定期维护调平
- 重心偏差诊断
- 转向架不平衡分析
- 车辆姿态监测

## 快速入门

### 步骤1: 准备环境

要求：
- MATLAB R2018b或更高版本
- 无需额外工具箱

### 步骤2: 下载文件

确保以下文件在同一目录：
- `MonorailLevelingConfig.m`
- `SensorData.m`
- `VehicleAttitude.m`
- `MonorailLevelingSystem.m`
- `example_leveling.m`（示例脚本）

### 步骤3: 运行示例

在MATLAB命令窗口输入：

```matlab
example_leveling
```

这将运行完整的调平计算示例，包括：
- 模拟传感器数据生成
- 姿态计算
- 垫片调整计算
- 结果可视化
- 数据导出

### 步骤4: 查看结果

运行后将生成：
- 可视化图表窗口（6个子图）
- `leveling_system_visualization.png`（图表文件）
- `leveling_results.mat`（MATLAB数据文件）
- `shim_adjustments.csv`（垫片调整表）

## 详细使用说明

### 实际数据采集流程

#### 1. 准备工作

**硬件准备**：
- 在轨道梁上安装称重台
- 安装水平激光位移传感器（12个）
- 安装垂向激光位移传感器（4个）
- 连接数据采集系统

**软件准备**：
```matlab
% 创建系统配置
config = MonorailLevelingConfig();

% 根据实际车辆修改参数
config.vehicleLength = 12000;  % 车体长度(mm)
config.vehicleWidth = 2600;    % 车体宽度(mm)
config.bogieSpacing = 8000;    % 转向架间距(mm)

% 创建调平系统
system = MonorailLevelingSystem(config);
```

#### 2. 数据采集

**采集顺序**：

1. **称重数据采集**（单位：牛顿N）
   ```matlab
   % 走行轮载荷 (4个)
   runningWheelLoads = [F1; F2; F3; F4];
   
   % 导向轮载荷 (8个)
   guideWheelLoads = [G1; G2; G3; G4; G5; G6; G7; G8];
   
   % 稳定轮载荷 (4个)
   stabilizingWheelLoads = [S1; S2; S3; S4];
   ```

2. **水平位移数据采集**（单位：毫米mm）
   ```matlab
   % 对应12个水平轮位置的横向距离
   horizontalDisplacements = [H1; H2; ...; H12];
   ```

3. **垂向位移数据采集**（单位：毫米mm）
   ```matlab
   % 对应车体四角的垂向距离
   verticalDisplacements = [V1; V2; V3; V4];
   ```

#### 3. 数据加载

```matlab
% 创建传感器数据对象
sensorData = SensorData();

% 加载测量数据
sensorData = sensorData.loadFromMeasurement(...
    runningWheelLoads, ...
    guideWheelLoads, ...
    stabilizingWheelLoads, ...
    horizontalDisplacements, ...
    verticalDisplacements);

% 验证数据有效性
if ~sensorData.isValid
    error('传感器数据无效，请检查测量值');
end
```

#### 4. 姿态计算

```matlab
% 更新系统数据
system = system.updateSensorData(sensorData);

% 计算车辆姿态
system = system.calculateAttitude();

% 查看姿态信息
disp(system.attitude);
```

#### 5. 计算垫片调整

```matlab
% 计算垫片调整量
system = system.calculateShimAdjustments();

% 显示完整结果
system.displayResults();
```

#### 6. 结果分析

```matlab
% 生成可视化
fig = system.visualizeSystem();

% 检查调平状态
toleranceAngle = deg2rad(0.1);  % 角度公差0.1度
toleranceCog = 5;               % 重心偏差公差5mm

isLevel = system.attitude.checkLevelStatus(toleranceAngle, toleranceCog);

if isLevel
    fprintf('车辆已达到调平标准\n');
else
    fprintf('需要进行垫片调整\n');
end
```

### 调整操作流程

#### 1. 获取调整指令

```matlab
% 水平垫片调整 (位置1-12)
horizontalShims = system.shimAdjustments(1:12);

% 垂向垫片调整 (位置13-16)
verticalShims = system.shimAdjustments(13:16);

% 按优先级排序（调整量大的优先）
[sortedShims, indices] = sort(abs(system.shimAdjustments), 'descend');

fprintf('调整优先顺序:\n');
for i = 1:length(indices)
    pos = indices(i);
    adj = system.shimAdjustments(pos);
    if abs(adj) > 0.5  % 只显示需要调整的位置
        fprintf('位置 %2d: %+6.2f mm\n', pos, adj);
    end
end
```

#### 2. 执行调整

**水平垫片调整**（导向轮、稳定轮位置）：
- 正值：增加垫片厚度
- 负值：减少垫片厚度（或在对侧增加）
- 单位：毫米（mm）

**垂向垫片调整**（空气弹簧位置）：
- 正值：增加垫片厚度，抬高车体
- 负值：减少垫片厚度，降低车体
- 单位：毫米（mm）

#### 3. 验证调整效果

调整后重新测量：

```matlab
% 采集调整后的数据
newSensorData = SensorData();
newSensorData = newSensorData.loadFromMeasurement(...);

% 重新计算姿态
system = system.updateSensorData(newSensorData);
system = system.calculateAttitude();

% 对比调整前后
fprintf('调整效果:\n');
fprintf('横摆角: %.4f° → %.4f°\n', ...
    rad2deg(oldAttitude.roll), rad2deg(system.attitude.roll));
fprintf('俯仰角: %.4f° → %.4f°\n', ...
    rad2deg(oldAttitude.pitch), rad2deg(system.attitude.pitch));
```

### 高级功能

#### 参数自定义

```matlab
% 修改调平公差
config.levelingTolerance = 0.5;  % 更严格的公差

% 修改垫片参数
config.maxShimThickness = 15.0;  % 最大垫片厚度
config.shimIncrement = 0.25;     % 更精细的调整步长

% 修改优化参数
config.maxIterations = 50;       % 最大迭代次数
config.convergenceTolerance = 0.05;  % 收敛公差
```

#### 批量处理

```matlab
% 处理多组测量数据
numMeasurements = 5;
results = cell(numMeasurements, 1);

for i = 1:numMeasurements
    % 加载第i组数据
    sensorData = loadMeasurementData(i);
    
    % 计算
    system = system.updateSensorData(sensorData);
    system = system.calculateAttitude();
    system = system.calculateShimAdjustments();
    
    % 保存结果
    results{i} = struct(...
        'attitude', system.attitude, ...
        'shimAdjustments', system.shimAdjustments);
end
```

## 算法原理

### 姿态计算原理

#### 1. 重心位置计算

基于力矩平衡原理：

```
重心X坐标 = Σ(Fi × Xi) / Σ(Fi)
重心Y坐标 = Σ(Fi × Yi) / Σ(Fi)
重心Z坐标 = Σ(Fi × Zi) / Σ(Fi)
```

其中：
- Fi: 第i个支撑点的载荷
- Xi, Yi, Zi: 第i个支撑点的坐标

#### 2. 姿态角计算

使用最小二乘法拟合平面：

对于垂向测量点，拟合平面方程：
```
z = ax + by + c
```

求解参数后：
```
俯仰角 pitch = arctan(a / 1000)
横摆角 roll = arctan(-b / 1000)
```

#### 3. 转向架重心计算

分别计算两个转向架的重心：

```matlab
% 转向架1的传感器索引
bogie1Indices = [1, 2, 5:8, 13, 14];

% 计算转向架1重心
bogie1Weight = sum(loads(bogie1Indices));
bogie1CogX = sum(loads(bogie1Indices) .* positions(bogie1Indices, 1)) / bogie1Weight;
```

### 垫片调整算法

#### 1. 垂向调整策略

目标：使四个角的高度一致

```matlab
% 计算平均高度
avgHeight = mean(verticalDisplacements);

% 计算各点调整量
verticalShims = verticalDisplacements - avgHeight;
```

#### 2. 水平调整策略

目标：消除横向不平衡

```matlab
% 基础调整
avgLateralDisp = mean(horizontalDisplacements);
horizontalShims = horizontalDisplacements - avgLateralDisp;

% 补偿横摆角
for i = 1:12
    y_pos = horizontalPositions(i, 2);
    rollCompensation = y_pos * tan(roll);
    horizontalShims(i) = horizontalShims(i) - rollCompensation;
end
```

#### 3. 重心偏差补偿

```matlab
% 横向重心偏差补偿
if abs(cogDeviationY) > 10
    lateralAdjustment = -cogDeviationY / 10;
    % 在偏移反方向增加垫片
    for i = 1:12
        if y_pos(i) < 0
            horizontalShims(i) = horizontalShims(i) + lateralAdjustment;
        else
            horizontalShims(i) = horizontalShims(i) - lateralAdjustment;
        end
    end
end
```

### 优化方法

迭代优化流程：

```
1. 初始姿态计算
2. 计算垫片调整量
3. 应用调整（模拟）
4. 重新计算姿态
5. 检查收敛：|新调整量 - 旧调整量| < 阈值
6. 未收敛则返回步骤2
```

## 常见问题

### Q1: 传感器数据显示无效怎么办？

**检查项目**：
1. 载荷值是否为正（不能为负值）
2. 位移值是否在合理范围（-100mm ~ 100mm）
3. 数据格式是否正确（列向量）
4. 数据点数是否完整（走行轮4个，导向轮8个等）

```matlab
% 数据验证示例
if any(runningWheelLoads < 0)
    error('走行轮载荷不能为负值');
end

if length(runningWheelLoads) ~= 4
    error('走行轮载荷应为4个数据点');
end
```

### Q2: 计算出的垫片调整量超过最大值？

这表示车辆偏差较大，可能需要：

1. **分步调整**：
   ```matlab
   % 限制单次调整量
   maxSingleAdjustment = 10;  % mm
   adjustments = min(max(calculated_shims, -maxSingleAdjustment), ...
                     maxSingleAdjustment);
   ```

2. **检查是否有制造缺陷**：
   - 查看哪个位置调整量最大
   - 可能需要机械维修而非调平

3. **增加垫片厚度限制**：
   ```matlab
   config.maxShimThickness = 30.0;  % 增加到30mm
   ```

### Q3: 调整后仍未达标怎么办？

**可能原因**：

1. **测量误差**
   - 重新校准传感器
   - 多次测量取平均值
   
2. **调整不到位**
   - 检查垫片实际安装厚度
   - 确认垫片位置正确

3. **存在其他因素**
   - 温度变形
   - 基础不均匀沉降
   - 结构永久变形

**解决方案**：

```matlab
% 多次迭代调整
for iteration = 1:3
    % 测量
    sensorData = acquireData();
    
    % 计算
    system = system.updateSensorData(sensorData);
    system = system.calculateAttitude();
    system = system.calculateShimAdjustments();
    
    % 检查
    if system.attitude.checkLevelStatus(tolAngle, tolCog)
        fprintf('第%d次调整后达标\n', iteration);
        break;
    end
    
    % 应用调整
    applyShimAdjustments(system.shimAdjustments);
end
```

### Q4: 如何解读重心偏差？

**重心偏差含义**：

- **X方向（纵向）**：
  - 正值：重心偏后
  - 负值：重心偏前
  - 影响：俯仰平衡

- **Y方向（横向）**：
  - 正值：重心偏右
  - 负值：重心偏左
  - 影响：横摆平衡

- **Z方向（垂向）**：
  - 通常影响较小
  - 主要反映装载高度

**可接受范围**：
- ±50mm：正常范围
- ±100mm：需要关注
- >100mm：需要重新分配载荷或检查

### Q5: 转向架重心偏差如何处理？

```matlab
% 查看转向架偏差
disp(system.attitude.bogie1CogDeviation);
disp(system.attitude.bogie2CogDeviation);

% 如果某个转向架偏差过大（>30mm）
if norm(system.attitude.bogie1CogDeviation) > 30
    warning('转向架1重心偏差过大，建议检查转向架本身');
    % 可能需要调整转向架内部组件
end
```

## 技术支持

### 文件清单

- `MonorailLevelingConfig.m` - 配置类（167行）
- `SensorData.m` - 传感器数据类（92行）
- `VehicleAttitude.m` - 姿态类（120行）
- `MonorailLevelingSystem.m` - 主系统类（520行）
- `example_leveling.m` - 基础示例（200行）
- `example_advanced.m` - 高级示例（250行）
- `test_basic.m` - 基础测试（72行）

### 数据格式说明

所有输入数据应为列向量：

```matlab
% 正确格式
runningWheelLoads = [1000; 1100; 1050; 1080];  % 4×1列向量

% 错误格式（行向量）
runningWheelLoads = [1000, 1100, 1050, 1080];  % 会导致错误
```

### 性能参考

- 单次计算时间：< 1秒
- 优化迭代时间：< 10秒（通常3-5次迭代收敛）
- 内存占用：< 10 MB
- 适用车辆：所有单轴转向架单轨车辆

### 联系方式

如有技术问题，请提供：
1. MATLAB版本
2. 错误信息截图
3. 输入数据样本
4. 系统配置参数

---

**版本**: 1.0.0  
**更新日期**: 2025-11-06  
**语言**: MATLAB R2018b+
