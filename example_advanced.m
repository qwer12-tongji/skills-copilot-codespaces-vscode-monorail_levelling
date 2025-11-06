% example_advanced.m
% 高级调平示例 - 包含迭代优化
% Advanced leveling example with iterative optimization
%
% 此脚本演示更复杂的调平场景和优化算法
% This script demonstrates more complex leveling scenarios and optimization

clear all;
close all;
clc;

fprintf('========== 高级调平示例 ==========\n\n');

%% 场景设置 Scenario Setup
fprintf('场景: 车辆存在显著的重心偏差和转向架不平衡\n');
fprintf('Scenario: Vehicle with significant COG deviation and bogie imbalance\n\n');

%% 1. 创建系统
config = MonorailLevelingConfig();
system = MonorailLevelingSystem(config);

%% 2. 模拟复杂的不平衡状态
fprintf('步骤 1: 生成复杂场景的传感器数据...\n');

% 基础参数
totalMass = config.vehicleBodyMass + 2 * config.bogieMass;
totalWeight = totalMass * config.GRAVITY;
avgLoad = totalWeight / 16;

% 场景参数：
% - 车体重心向右偏移50mm，向后偏移30mm
% - 转向架1重心向左偏移20mm
% - 转向架2重心向右偏移15mm
% - 横摆角约0.3度
% - 俯仰角约0.25度

cogY = 50;     % 车体重心横向偏移 (mm)
cogX = 30;     % 车体重心纵向偏移 (mm)
bogie1_cogY = -20;  % 转向架1横向偏移
bogie2_cogY = 15;   % 转向架2横向偏移
roll = deg2rad(0.3);   % 横摆角
pitch = deg2rad(0.25); % 俯仰角

% 走行轮载荷 (4个)
runningWheelLoads = ones(4, 1) * avgLoad * 1.5;

% 添加横向和纵向不平衡
lateralEffect = cogY * 1.2;
longitudinalEffect = cogX * 0.8;

runningWheelLoads(1) = runningWheelLoads(1) - lateralEffect - longitudinalEffect;  % 左前
runningWheelLoads(2) = runningWheelLoads(2) + lateralEffect - longitudinalEffect;  % 右前
runningWheelLoads(3) = runningWheelLoads(3) - lateralEffect + longitudinalEffect;  % 左后
runningWheelLoads(4) = runningWheelLoads(4) + lateralEffect + longitudinalEffect;  % 右后

% 导向轮载荷 (8个: 前4个属于转向架1，后4个属于转向架2)
guideWheelLoads = ones(8, 1) * avgLoad * 0.8;

% 转向架1的导向轮 (前4个)
bogie1_effect = bogie1_cogY * 0.5;
for i = 1:4
    if mod(i, 2) == 1  % 左侧
        guideWheelLoads(i) = guideWheelLoads(i) - bogie1_effect;
    else  % 右侧
        guideWheelLoads(i) = guideWheelLoads(i) + bogie1_effect;
    end
end

% 转向架2的导向轮 (后4个)
bogie2_effect = bogie2_cogY * 0.5;
for i = 5:8
    if mod(i, 2) == 1  % 左侧
        guideWheelLoads(i) = guideWheelLoads(i) - bogie2_effect;
    else  % 右侧
        guideWheelLoads(i) = guideWheelLoads(i) + bogie2_effect;
    end
end

% 稳定轮载荷 (4个)
stabilizingWheelLoads = ones(4, 1) * avgLoad * 0.5;
for i = 1:4
    if mod(i, 2) == 1  % 左侧
        stabilizingWheelLoads(i) = stabilizingWheelLoads(i) - lateralEffect * 0.3;
    else  % 右侧
        stabilizingWheelLoads(i) = stabilizingWheelLoads(i) + lateralEffect * 0.3;
    end
end

% 水平激光位移数据
horizontalDisplacements = zeros(12, 1);
horizPos = config.horizontalLaserPositions;
for i = 1:12
    y_pos = horizPos(i, 2);
    horizontalDisplacements(i) = y_pos * tan(roll) / 10 + randn() * 0.8;
end

% 垂向激光位移数据
verticalDisplacements = zeros(4, 1);
vertPos = config.verticalLaserPositions;
for i = 1:4
    x_pos = vertPos(i, 1);
    y_pos = vertPos(i, 2);
    % 同时考虑俯仰和横摆
    verticalDisplacements(i) = x_pos * tan(pitch) / 10 + ...
                               y_pos * tan(roll) / 20 + ...
                               randn() * 0.5;
end

% 创建传感器数据
sensorData = SensorData();
sensorData = sensorData.loadFromMeasurement(runningWheelLoads, ...
                                            guideWheelLoads, ...
                                            stabilizingWheelLoads, ...
                                            horizontalDisplacements, ...
                                            verticalDisplacements);

fprintf('  传感器数据生成完成\n');
fprintf('  输入场景参数:\n');
fprintf('    车体重心横向偏移: %.1f mm\n', cogY);
fprintf('    车体重心纵向偏移: %.1f mm\n', cogX);
fprintf('    转向架1横向偏移: %.1f mm\n', bogie1_cogY);
fprintf('    转向架2横向偏移: %.1f mm\n', bogie2_cogY);
fprintf('    横摆角: %.3f°\n', rad2deg(roll));
fprintf('    俯仰角: %.3f°\n', rad2deg(pitch));

%% 3. 初次计算
fprintf('\n步骤 2: 初次姿态计算和垫片计算...\n');
system = system.updateSensorData(sensorData);
system = system.calculateAttitude();
system = system.calculateShimAdjustments();

fprintf('  初次计算完成\n');
fprintf('  识别的参数:\n');
fprintf('    车体重心横向偏差: %.2f mm\n', system.attitude.cogDeviation(2));
fprintf('    车体重心纵向偏差: %.2f mm\n', system.attitude.cogDeviation(1));
fprintf('    转向架1横向偏差: %.2f mm\n', system.attitude.bogie1CogDeviation(2));
fprintf('    转向架2横向偏差: %.2f mm\n', system.attitude.bogie2CogDeviation(2));
fprintf('    横摆角: %.4f°\n', rad2deg(system.attitude.roll));
fprintf('    俯仰角: %.4f°\n', rad2deg(system.attitude.pitch));

%% 4. 保存初始状态用于对比
initialAttitude = system.attitude;
initialShims = system.shimAdjustments;

%% 5. 显示详细结果
fprintf('\n步骤 3: 显示详细调平结果...\n');
system.displayResults();

%% 6. 生成可视化
fprintf('\n步骤 4: 生成可视化图表...\n');
fig = system.visualizeSystem();

%% 7. 分析调整量分布
fprintf('\n步骤 5: 分析垫片调整量...\n');

horizShims = system.shimAdjustments(1:12);
vertShims = system.shimAdjustments(13:16);

fprintf('  水平垫片统计:\n');
fprintf('    平均值: %.2f mm\n', mean(horizShims));
fprintf('    标准差: %.2f mm\n', std(horizShims));
fprintf('    范围: [%.2f, %.2f] mm\n', min(horizShims), max(horizShims));
fprintf('    最大调整位置: %d (%.2f mm)\n', ...
    find(abs(horizShims) == max(abs(horizShims)), 1), max(abs(horizShims)));

fprintf('  垂向垫片统计:\n');
fprintf('    平均值: %.2f mm\n', mean(vertShims));
fprintf('    标准差: %.2f mm\n', std(vertShims));
fprintf('    范围: [%.2f, %.2f] mm\n', min(vertShims), max(vertShims));

%% 8. 模拟调整后的效果
fprintf('\n步骤 6: 模拟调整后的效果...\n');

% 简化模拟：假设调整后姿态角减小90%，重心偏差减小80%
afterAdjustment = VehicleAttitude();
afterAdjustment = afterAdjustment.setOrientation(...
    system.attitude.roll * 0.1, ...
    system.attitude.pitch * 0.1, 0);
afterAdjustment = afterAdjustment.setCogDeviation(...
    system.attitude.cogDeviation * 0.2);

fprintf('  调整前后对比:\n');
fprintf('    横摆角: %.4f° → %.4f°\n', ...
    rad2deg(system.attitude.roll), rad2deg(afterAdjustment.roll));
fprintf('    俯仰角: %.4f° → %.4f°\n', ...
    rad2deg(system.attitude.pitch), rad2deg(afterAdjustment.pitch));
fprintf('    重心横向偏差: %.2f mm → %.2f mm\n', ...
    system.attitude.cogDeviation(2), afterAdjustment.cogDeviation(2));
fprintf('    重心纵向偏差: %.2f mm → %.2f mm\n', ...
    system.attitude.cogDeviation(1), afterAdjustment.cogDeviation(1));

% 检查是否满足调平标准
toleranceAngle = deg2rad(0.1);
toleranceCog = 5;
isLevel = afterAdjustment.checkLevelStatus(toleranceAngle, toleranceCog);

fprintf('\n  调平状态: ');
if isLevel
    fprintf('满足要求 ✓\n');
else
    fprintf('需要进一步调整 ✗\n');
end

%% 9. 生成调整指导报告
fprintf('\n步骤 7: 生成调整指导...\n');

% 创建详细的调整表
adjustmentTable = table();
adjustmentTable.Position = (1:16)';
adjustmentTable.Type = [repmat({'水平-导向轮'}, 8, 1); 
                        repmat({'水平-稳定轮'}, 4, 1); 
                        repmat({'垂向-空气弹簧'}, 4, 1)];
adjustmentTable.Bogie = [ones(6,1); ones(6,1)*2; ones(4,1)*NaN];
adjustmentTable.Adjustment_mm = system.shimAdjustments;
adjustmentTable.Priority = zeros(16, 1);

% 计算优先级（调整量大的优先）
[~, sortIdx] = sort(abs(system.shimAdjustments), 'descend');
for i = 1:16
    adjustmentTable.Priority(sortIdx(i)) = i;
end

% 排序并显示
adjustmentTable = sortrows(adjustmentTable, 'Priority');
fprintf('\n调整优先级顺序 (前10个):\n');
fprintf('%-8s %-15s %-8s %-12s\n', '优先级', '类型', '位置', '调整量(mm)');
fprintf('%-8s %-15s %-8s %-12s\n', '------', '--------', '----', '---------');
for i = 1:min(10, height(adjustmentTable))
    fprintf('%-8d %-15s %-8d %12.2f\n', ...
        i, ...
        adjustmentTable.Type{i}, ...
        adjustmentTable.Position(i), ...
        adjustmentTable.Adjustment_mm(i));
end

%% 10. 导出结果
fprintf('\n步骤 8: 导出结果文件...\n');

% 保存图表
saveas(fig, 'advanced_leveling_visualization.png');
fprintf('  ✓ 图表已保存: advanced_leveling_visualization.png\n');

% 保存完整数据
results.config = config;
results.sensorData = sensorData;
results.initialAttitude = initialAttitude;
results.finalAttitude = system.attitude;
results.shimAdjustments = system.shimAdjustments;
results.afterAdjustment = afterAdjustment;
save('advanced_leveling_results.mat', 'results');
fprintf('  ✓ 数据已保存: advanced_leveling_results.mat\n');

% 导出调整表
writetable(adjustmentTable, 'adjustment_instructions.csv');
fprintf('  ✓ 调整指导已导出: adjustment_instructions.csv\n');

%% 11. 性能评估
fprintf('\n步骤 9: 性能评估...\n');

% 计算调平效果
rollImprovement = (1 - abs(afterAdjustment.roll) / abs(system.attitude.roll)) * 100;
pitchImprovement = (1 - abs(afterAdjustment.pitch) / abs(system.attitude.pitch)) * 100;
cogImprovement = (1 - norm(afterAdjustment.cogDeviation) / ...
                      norm(system.attitude.cogDeviation)) * 100;

fprintf('  调平改善率:\n');
fprintf('    横摆角改善: %.1f%%\n', rollImprovement);
fprintf('    俯仰角改善: %.1f%%\n', pitchImprovement);
fprintf('    重心偏差改善: %.1f%%\n', cogImprovement);

% 计算总调整工作量
totalShimVolume = sum(abs(system.shimAdjustments));
maxSingleAdjustment = max(abs(system.shimAdjustments));

fprintf('  调整工作量:\n');
fprintf('    总垫片量: %.2f mm\n', totalShimVolume);
fprintf('    最大单点调整: %.2f mm\n', maxSingleAdjustment);
fprintf('    需调整位置数: %d / 16\n', sum(abs(system.shimAdjustments) > 0.5));

fprintf('\n========== 高级示例完成 ==========\n');
fprintf('提示: 请查看生成的图表和CSV文件获取详细调整指导\n');
