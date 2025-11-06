% example_leveling.m
% 单轨车辆调平系统示例脚本
% Example script for monorail vehicle leveling system
%
% 此脚本演示如何使用MonorailLevelingSystem进行车辆调平计算
% This script demonstrates how to use the MonorailLevelingSystem for vehicle leveling

clear all;
close all;
clc;

fprintf('========== 单轨车辆调平系统示例 ==========\n\n');

%% 1. 创建系统配置
fprintf('步骤 1: 创建系统配置...\n');
config = MonorailLevelingConfig();

fprintf('  车辆参数:\n');
fprintf('    车体长度: %.0f mm\n', config.vehicleLength);
fprintf('    车体宽度: %.0f mm\n', config.vehicleWidth);
fprintf('    转向架间距: %.0f mm\n', config.bogieSpacing);
fprintf('    轮距: %.0f mm\n', config.wheelBase);

%% 2. 创建调平系统
fprintf('\n步骤 2: 创建调平系统实例...\n');
system = MonorailLevelingSystem(config);

%% 3. 模拟传感器数据
fprintf('\n步骤 3: 生成模拟传感器数据...\n');

% 3.1 模拟场景：车辆有一定的重心偏差和姿态偏差
fprintf('  模拟场景: 车辆存在重心横向偏差和轻微横摆\n');

% 基础载荷 (假设总重约27吨)
totalVehicleMass = config.vehicleBodyMass + 2 * config.bogieMass;
totalWeight = totalVehicleMass * config.GRAVITY;
avgLoadPerWheel = totalWeight / 16;  % 16个支撑点

% 走行轮载荷 (4个) - 承载主要重量
runningWheelLoads = ones(4, 1) * avgLoadPerWheel * 1.5;

% 导向轮载荷 (8个) - 承载部分重量
guideWheelLoads = ones(8, 1) * avgLoadPerWheel * 0.8;

% 稳定轮载荷 (4个) - 承载较小重量
stabilizingWheelLoads = ones(4, 1) * avgLoadPerWheel * 0.5;

% 3.2 添加重心偏差效应
% 假设重心向右偏移30mm，向后偏移20mm
cogDeviationY = 30;  % mm, 向右
cogDeviationX = 20;  % mm, 向后

% 横向重心偏差导致左右载荷不均
lateralLoadAdjustment = cogDeviationY * 0.5;  % 简化模型
runningWheelLoads(1) = runningWheelLoads(1) - lateralLoadAdjustment;  % 左前
runningWheelLoads(2) = runningWheelLoads(2) + lateralLoadAdjustment;  % 右前
runningWheelLoads(3) = runningWheelLoads(3) - lateralLoadAdjustment;  % 左后
runningWheelLoads(4) = runningWheelLoads(4) + lateralLoadAdjustment;  % 右后

% 纵向重心偏差导致前后载荷不均
longitudinalLoadAdjustment = cogDeviationX * 0.3;
runningWheelLoads(1:2) = runningWheelLoads(1:2) - longitudinalLoadAdjustment;  % 前
runningWheelLoads(3:4) = runningWheelLoads(3:4) + longitudinalLoadAdjustment;  % 后

% 导向轮和稳定轮也受影响
for i = 1:4
    if mod(i, 2) == 1  % 左侧
        guideWheelLoads(i) = guideWheelLoads(i) - lateralLoadAdjustment * 0.3;
        guideWheelLoads(i+4) = guideWheelLoads(i+4) - lateralLoadAdjustment * 0.3;
        stabilizingWheelLoads(i) = stabilizingWheelLoads(i) - lateralLoadAdjustment * 0.2;
    else  % 右侧
        guideWheelLoads(i) = guideWheelLoads(i) + lateralLoadAdjustment * 0.3;
        guideWheelLoads(i+4) = guideWheelLoads(i+4) + lateralLoadAdjustment * 0.3;
        stabilizingWheelLoads(i) = stabilizingWheelLoads(i) + lateralLoadAdjustment * 0.2;
    end
end

% 3.3 水平激光位移传感器数据
% 假设有轻微的横摆角 (约0.2度)
rollAngle = deg2rad(0.2);
horizontalDisplacements = zeros(12, 1);

horizPos = config.horizontalLaserPositions;
for i = 1:12
    y_pos = horizPos(i, 2);
    % 由于横摆导致的位移
    horizontalDisplacements(i) = y_pos * tan(rollAngle) / 10;
    % 添加一些测量噪声
    horizontalDisplacements(i) = horizontalDisplacements(i) + randn() * 0.5;
end

% 3.4 垂向激光位移传感器数据
% 假设有轻微的俯仰角 (约0.15度)
pitchAngle = deg2rad(0.15);
verticalDisplacements = zeros(4, 1);

vertPos = config.verticalLaserPositions;
for i = 1:4
    x_pos = vertPos(i, 1);
    % 由于俯仰导致的位移
    verticalDisplacements(i) = x_pos * tan(pitchAngle) / 10;
    % 添加一些测量噪声
    verticalDisplacements(i) = verticalDisplacements(i) + randn() * 0.3;
end

% 3.5 创建传感器数据对象
sensorData = SensorData();
sensorData = sensorData.loadFromMeasurement(runningWheelLoads, ...
                                            guideWheelLoads, ...
                                            stabilizingWheelLoads, ...
                                            horizontalDisplacements, ...
                                            verticalDisplacements);

fprintf('  传感器数据已生成\n');
fprintf('    总载荷: %.2f N (%.2f kg)\n', sensorData.getTotalLoad(), ...
    sensorData.getTotalLoad() / config.GRAVITY);

%% 4. 更新系统传感器数据
fprintf('\n步骤 4: 更新系统传感器数据...\n');
system = system.updateSensorData(sensorData);

%% 5. 计算车辆姿态
fprintf('\n步骤 5: 计算车辆姿态...\n');
system = system.calculateAttitude();

fprintf('  姿态计算完成\n');
fprintf('    横摆角: %.4f° (输入: %.4f°)\n', ...
    rad2deg(system.attitude.roll), rad2deg(rollAngle));
fprintf('    俯仰角: %.4f° (输入: %.4f°)\n', ...
    rad2deg(system.attitude.pitch), rad2deg(pitchAngle));
fprintf('    重心Y偏差: %.2f mm (输入: %.2f mm)\n', ...
    system.attitude.cogDeviation(2), cogDeviationY);
fprintf('    重心X偏差: %.2f mm (输入: %.2f mm)\n', ...
    system.attitude.cogDeviation(1), cogDeviationX);

%% 6. 计算垫片调整量
fprintf('\n步骤 6: 计算垫片调整量...\n');
system = system.calculateShimAdjustments();

fprintf('  垫片调整量计算完成\n');
fprintf('    水平垫片调整范围: [%.2f, %.2f] mm\n', ...
    min(system.shimAdjustments(1:12)), max(system.shimAdjustments(1:12)));
fprintf('    垂向垫片调整范围: [%.2f, %.2f] mm\n', ...
    min(system.shimAdjustments(13:16)), max(system.shimAdjustments(13:16)));

%% 7. 显示详细结果
fprintf('\n步骤 7: 显示详细结果...\n');
system.displayResults();

%% 8. 可视化系统状态
fprintf('\n步骤 8: 生成可视化图表...\n');
fig = system.visualizeSystem();

fprintf('\n========== 示例完成 ==========\n');
fprintf('\n提示: 可以查看生成的图表了解系统状态\n');

%% 9. 保存结果到文件 (可选)
fprintf('\n步骤 9: 保存结果到文件...\n');

% 保存图表
saveas(fig, 'leveling_system_visualization.png');
fprintf('  图表已保存: leveling_system_visualization.png\n');

% 保存数据到MAT文件
results.config = config;
results.sensorData = sensorData;
results.attitude = system.attitude;
results.shimAdjustments = system.shimAdjustments;
save('leveling_results.mat', 'results');
fprintf('  数据已保存: leveling_results.mat\n');

% 导出调整量到CSV文件
shimData = array2table(system.shimAdjustments, ...
    'VariableNames', {'Adjustment_mm'});
shimData.Position = (1:16)';
shimData.Type = [repmat({'Horizontal'}, 12, 1); repmat({'Vertical'}, 4, 1)];
shimData = shimData(:, [2, 3, 1]);  % 重新排列列顺序
writetable(shimData, 'shim_adjustments.csv');
fprintf('  调整量已导出: shim_adjustments.csv\n');

fprintf('\n========== 所有步骤完成 ==========\n');
