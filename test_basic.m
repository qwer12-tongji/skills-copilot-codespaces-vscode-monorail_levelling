% test_basic.m
% 基本功能测试脚本
% Basic functionality test script

fprintf('测试 1: 创建配置对象...\n');
try
    config = MonorailLevelingConfig();
    fprintf('  ✓ 配置对象创建成功\n');
    fprintf('    车辆长度: %.0f mm\n', config.vehicleLength);
    fprintf('    转向架数量: %d\n', config.NUM_BOGIES);
    fprintf('    总调整位置: %d\n', config.NUM_TOTAL_SHIMS);
catch ME
    fprintf('  ✗ 错误: %s\n', ME.message);
    rethrow(ME);
end

fprintf('\n测试 2: 创建传感器数据对象...\n');
try
    sensorData = SensorData();
    fprintf('  ✓ 传感器数据对象创建成功\n');
    
    % 测试加载数据
    runningLoads = ones(4, 1) * 5000;
    guideLoads = ones(8, 1) * 2000;
    stabLoads = ones(4, 1) * 1000;
    horizDisp = zeros(12, 1);
    vertDisp = zeros(4, 1);
    
    sensorData = sensorData.loadFromMeasurement(runningLoads, guideLoads, ...
                                                stabLoads, horizDisp, vertDisp);
    fprintf('  ✓ 数据加载成功\n');
    fprintf('    总载荷: %.2f N\n', sensorData.getTotalLoad());
    fprintf('    数据有效性: %d\n', sensorData.isValid);
catch ME
    fprintf('  ✗ 错误: %s\n', ME.message);
    rethrow(ME);
end

fprintf('\n测试 3: 创建车辆姿态对象...\n');
try
    attitude = VehicleAttitude();
    fprintf('  ✓ 车辆姿态对象创建成功\n');
    
    % 测试设置姿态
    attitude = attitude.setOrientation(0.01, 0.02, 0);
    fprintf('  ✓ 姿态设置成功\n');
    fprintf('    横摆角: %.4f rad\n', attitude.roll);
    fprintf('    俯仰角: %.4f rad\n', attitude.pitch);
catch ME
    fprintf('  ✗ 错误: %s\n', ME.message);
    rethrow(ME);
end

fprintf('\n测试 4: 创建调平系统...\n');
try
    system = MonorailLevelingSystem(config);
    fprintf('  ✓ 调平系统创建成功\n');
    
    % 更新传感器数据
    system = system.updateSensorData(sensorData);
    fprintf('  ✓ 传感器数据更新成功\n');
    
    % 计算姿态
    system = system.calculateAttitude();
    fprintf('  ✓ 姿态计算成功\n');
    fprintf('    计算的横摆角: %.6f rad\n', system.attitude.roll);
    fprintf('    计算的俯仰角: %.6f rad\n', system.attitude.pitch);
    
    % 计算垫片调整
    system = system.calculateShimAdjustments();
    fprintf('  ✓ 垫片调整计算成功\n');
    fprintf('    水平垫片数量: %d\n', config.NUM_HORIZONTAL_SHIMS_TOTAL);
    fprintf('    垂向垫片数量: %d\n', config.NUM_VERTICAL_SHIMS);
catch ME
    fprintf('  ✗ 错误: %s\n', ME.message);
    rethrow(ME);
end

fprintf('\n========== 所有基本测试通过 ==========\n');
