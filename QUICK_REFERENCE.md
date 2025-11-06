# 快速参考卡 Quick Reference Card

## 一、快速开始 Quick Start

### 运行示例 Run Example
```matlab
% 基础示例
example_leveling

% 高级示例（复杂场景）
example_advanced

% 基础测试
test_basic
```

## 二、基本使用流程 Basic Workflow

### 步骤1: 创建配置
```matlab
config = MonorailLevelingConfig();
```

### 步骤2: 创建系统
```matlab
system = MonorailLevelingSystem(config);
```

### 步骤3: 准备数据
```matlab
sensorData = SensorData();
sensorData = sensorData.loadFromMeasurement(...
    runningWheelLoads,      % [4×1] 走行轮载荷(N)
    guideWheelLoads,        % [8×1] 导向轮载荷(N)
    stabilizingWheelLoads,  % [4×1] 稳定轮载荷(N)
    horizontalDisplacements, % [12×1] 水平位移(mm)
    verticalDisplacements);  % [4×1] 垂向位移(mm)
```

### 步骤4: 计算姿态
```matlab
system = system.updateSensorData(sensorData);
system = system.calculateAttitude();
```

### 步骤5: 计算垫片
```matlab
system = system.calculateShimAdjustments();
```

### 步骤6: 查看结果
```matlab
system.displayResults();
system.visualizeSystem();
```

## 三、数据格式 Data Format

### 载荷数据（单位：牛顿N）
```matlab
runningWheelLoads = [
    F_左前;  % 转向架1左前走行轮
    F_右前;  % 转向架1右前走行轮
    F_左后;  % 转向架2左后走行轮
    F_右后   % 转向架2右后走行轮
];

guideWheelLoads = [
    G1; G2; G3; G4;  % 转向架1的4个导向轮
    G5; G6; G7; G8   % 转向架2的4个导向轮
];

stabilizingWheelLoads = [
    S1; S2;  % 转向架1的2个稳定轮
    S3; S4   % 转向架2的2个稳定轮
];
```

### 位移数据（单位：毫米mm）
```matlab
horizontalDisplacements = [H1; H2; ...; H12];  % 12个水平位移
verticalDisplacements = [V1; V2; V3; V4];      % 4个垂向位移
```

## 四、常用参数设置 Common Parameters

### 车辆尺寸
```matlab
config.vehicleLength = 12000;   % 车体长度(mm)
config.vehicleWidth = 2600;     % 车体宽度(mm)
config.bogieSpacing = 8000;     % 转向架间距(mm)
config.wheelBase = 1800;        % 轮距(mm)
```

### 质量参数
```matlab
config.vehicleBodyMass = 25000; % 车体质量(kg)
config.bogieMass = 2000;        % 转向架质量(kg)
```

### 调平参数
```matlab
config.levelingTolerance = 1.0;      % 调平公差(mm)
config.maxShimThickness = 20.0;      % 最大垫片厚度(mm)
config.shimIncrement = 0.5;          % 垫片增量(mm)
config.maxIterations = 100;          % 最大迭代次数
config.convergenceTolerance = 0.1;   % 收敛公差(mm)
```

## 五、结果解读 Result Interpretation

### 车辆姿态 Vehicle Attitude
```matlab
% 访问姿态数据
roll = system.attitude.roll;           % 横摆角(rad)
pitch = system.attitude.pitch;         % 俯仰角(rad)
cogDev = system.attitude.cogDeviation; % 重心偏差[dx;dy;dz](mm)

% 转换为度
roll_deg = rad2deg(roll);
pitch_deg = rad2deg(pitch);
```

### 垫片调整量
```matlab
% 获取调整量
allShims = system.shimAdjustments;     % [16×1] 所有调整量

% 分离水平和垂向
horizShims = allShims(1:12);   % 水平垫片(导向轮+稳定轮)
vertShims = allShims(13:16);   % 垂向垫片(空气弹簧)

% 解读
% 正值：增加垫片
% 负值：减少垫片（或在对侧增加）
```

### 调平状态检查
```matlab
toleranceAngle = deg2rad(0.1);  % 角度公差0.1°
toleranceCog = 5;               % 重心偏差公差5mm

isLevel = system.attitude.checkLevelStatus(...
    toleranceAngle, toleranceCog);

if isLevel
    fprintf('✓ 车辆已调平\n');
else
    fprintf('✗ 需要调整\n');
end
```

## 六、输出文件 Output Files

### 运行example_leveling.m后：
- `leveling_system_visualization.png` - 可视化图表
- `leveling_results.mat` - 完整数据
- `shim_adjustments.csv` - 调整量表格

### 运行example_advanced.m后：
- `advanced_leveling_visualization.png` - 高级可视化
- `advanced_leveling_results.mat` - 高级数据
- `adjustment_instructions.csv` - 调整指导

## 七、常用命令 Common Commands

### 显示姿态信息
```matlab
disp(system.attitude);
```

### 保存结果
```matlab
save('my_results.mat', 'system');
```

### 导出调整表
```matlab
shimTable = table(...
    (1:16)', ...
    system.shimAdjustments, ...
    'VariableNames', {'Position', 'Adjustment_mm'});
writetable(shimTable, 'my_adjustments.csv');
```

### 生成可视化
```matlab
fig = system.visualizeSystem();
saveas(fig, 'my_visualization.png');
```

## 八、故障排除 Troubleshooting

### 问题：数据无效
```matlab
% 检查数据
if ~sensorData.isValid
    % 检查载荷是否为正
    % 检查位移是否在合理范围(-100~100mm)
end
```

### 问题：调整量过大
```matlab
% 查看最大调整量
maxAdj = max(abs(system.shimAdjustments));
if maxAdj > config.maxShimThickness
    warning('调整量超过限制');
    % 可能需要分步调整或检查制造问题
end
```

### 问题：未收敛
```matlab
% 增加迭代次数
config.maxIterations = 200;

% 或放宽收敛公差
config.convergenceTolerance = 0.5;
```

## 九、技术指标 Specifications

| 项目 | 规格 |
|------|------|
| 支撑点数量 | 16 (4走行+8导向+4稳定) |
| 调整位置 | 16 (12水平+4垂向) |
| 传感器数量 | 28 (16称重+12水平激光+4垂向激光) |
| 角度精度 | ±0.01° |
| 位移精度 | ±0.1 mm |
| 调平公差 | 角度±0.1°, 重心±5mm |
| 垫片步长 | 0.5 mm (可配置) |
| 计算时间 | < 1 秒 |

## 十、文档索引 Documentation Index

- **README.md** - 系统概述和快速入门
- **USER_GUIDE.md** - 详细用户指南（推荐首次阅读）
- **IMPLEMENTATION_SUMMARY.md** - 实现总结和技术细节
- **QUICK_REFERENCE.md** - 本文档（快速参考）

## 十一、支持 Support

### 运行环境
- MATLAB R2018b 或更高版本
- 无需额外工具箱

### 代码文件
- `MonorailLevelingConfig.m` - 配置类
- `SensorData.m` - 传感器数据类
- `VehicleAttitude.m` - 车辆姿态类
- `MonorailLevelingSystem.m` - 主系统类

### 示例文件
- `example_leveling.m` - 基础示例
- `example_advanced.m` - 高级示例
- `test_basic.m` - 功能测试

---

**提示**: 首次使用请运行 `example_leveling` 查看完整示例
