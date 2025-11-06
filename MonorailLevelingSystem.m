classdef MonorailLevelingSystem
    % MonorailLevelingSystem - 单轨车辆调平系统主类
    % Main class for monorail vehicle leveling system
    
    properties
        config;         % MonorailLevelingConfig 配置对象
        sensorData;     % SensorData 传感器数据
        attitude;       % VehicleAttitude 车辆姿态
        shimAdjustments; % 垫片调整量 [16x1] (前12个为水平，后4个为垂向)
    end
    
    methods
        function obj = MonorailLevelingSystem(config)
            % 构造函数
            if nargin < 1
                obj.config = MonorailLevelingConfig();
            else
                obj.config = config;
            end
            obj.sensorData = SensorData();
            obj.attitude = VehicleAttitude();
            obj.shimAdjustments = zeros(obj.config.NUM_TOTAL_SHIMS, 1);
        end
        
        function obj = updateSensorData(obj, sensorData)
            % 更新传感器数据
            obj.sensorData = sensorData;
        end
        
        function obj = calculateAttitude(obj)
            % 根据传感器数据计算车辆姿态
            % Calculate vehicle attitude from sensor data
            
            if ~obj.sensorData.isValid
                error('Invalid sensor data');
            end
            
            % 获取所有载荷和位置
            loads = obj.sensorData.getAllLoads();
            positions = obj.config.loadCellPositions;
            
            % 计算总重量
            totalWeight = sum(loads);
            totalMass = totalWeight / obj.config.GRAVITY;
            
            % 计算重心位置 (质心)
            cogX = sum(loads .* positions(:,1)) / totalWeight;
            cogY = sum(loads .* positions(:,2)) / totalWeight;
            cogZ = sum(loads .* positions(:,3)) / totalWeight;
            
            % 理论重心位置 (假设为车体中心)
            theoreticalCogX = 0;
            theoreticalCogY = 0;
            theoreticalCogZ = 0;
            
            % 计算重心偏差
            cogDeviation = [cogX - theoreticalCogX;
                           cogY - theoreticalCogY;
                           cogZ - theoreticalCogZ];
            
            % 从垂向激光传感器数据计算姿态角
            vertDisp = obj.sensorData.verticalDisplacements;
            vertPos = obj.config.verticalLaserPositions;
            
            % 使用最小二乘法拟合平面: z = ax + by + c
            % 构造设计矩阵
            A = [vertPos(:,1), vertPos(:,2), ones(4,1)];
            
            % 求解平面参数
            planeParams = A \ vertDisp;
            a = planeParams(1);
            b = planeParams(2);
            
            % 从平面参数计算姿态角
            % pitch = atan(a), roll = atan(-b)
            pitch = atan(a / 1000);  % 转换为弧度，考虑单位mm
            roll = atan(-b / 1000);
            
            % 偏航角从水平激光传感器数据估计
            horizDisp = obj.sensorData.horizontalDisplacements;
            % 简化处理，偏航角设为0或从水平位移的不对称性估计
            yaw = 0;
            
            % 计算外部力矩
            % Mx = sum(F_i * y_i), My = sum(F_i * x_i), Mz = 0 (简化)
            Mx = sum(loads .* positions(:,2)) / 1000;  % 转换为N·m
            My = sum(loads .* positions(:,1)) / 1000;
            Mz = 0;
            
            % 分析转向架重心偏差
            % 分析前后转向架的载荷分布
            bogie1Indices = [1, 2, 5:8, 13, 14];  % 转向架1的传感器索引
            bogie2Indices = [3, 4, 9:12, 15, 16];  % 转向架2的传感器索引
            
            bogie1Loads = loads(bogie1Indices);
            bogie2Loads = loads(bogie2Indices);
            bogie1Positions = positions(bogie1Indices, :);
            bogie2Positions = positions(bogie2Indices, :);
            
            % 转向架1重心
            bogie1Weight = sum(bogie1Loads);
            if bogie1Weight > 0
                bogie1CogX = sum(bogie1Loads .* bogie1Positions(:,1)) / bogie1Weight;
                bogie1CogY = sum(bogie1Loads .* bogie1Positions(:,2)) / bogie1Weight;
                bogie1CogZ = sum(bogie1Loads .* bogie1Positions(:,3)) / bogie1Weight;
                
                % 理论转向架1重心位置
                theoreticalBogie1X = -obj.config.bogieSpacing/2;
                bogie1CogDeviation = [bogie1CogX - theoreticalBogie1X;
                                     bogie1CogY;
                                     bogie1CogZ];
            else
                bogie1CogDeviation = [0; 0; 0];
            end
            
            % 转向架2重心
            bogie2Weight = sum(bogie2Loads);
            if bogie2Weight > 0
                bogie2CogX = sum(bogie2Loads .* bogie2Positions(:,1)) / bogie2Weight;
                bogie2CogY = sum(bogie2Loads .* bogie2Positions(:,2)) / bogie2Weight;
                bogie2CogZ = sum(bogie2Loads .* bogie2Positions(:,3)) / bogie2Weight;
                
                % 理论转向架2重心位置
                theoreticalBogie2X = obj.config.bogieSpacing/2;
                bogie2CogDeviation = [bogie2CogX - theoreticalBogie2X;
                                     bogie2CogY;
                                     bogie2CogZ];
            else
                bogie2CogDeviation = [0; 0; 0];
            end
            
            % 更新车辆姿态
            obj.attitude = obj.attitude.setPosition(cogX, cogY, cogZ);
            obj.attitude = obj.attitude.setOrientation(roll, pitch, yaw);
            obj.attitude = obj.attitude.setCogDeviation(cogDeviation);
            obj.attitude = obj.attitude.setExternalMoments([Mx; My; Mz]);
            obj.attitude.bogie1CogDeviation = bogie1CogDeviation;
            obj.attitude.bogie2CogDeviation = bogie2CogDeviation;
        end
        
        function obj = calculateShimAdjustments(obj)
            % 计算垫片调整量以实现车辆调平
            % Calculate shim adjustments to level the vehicle
            
            % 初始化垫片调整
            horizontalShims = zeros(obj.config.NUM_HORIZONTAL_SHIMS_TOTAL, 1);
            verticalShims = zeros(obj.config.NUM_VERTICAL_SHIMS, 1);
            
            % 目标：消除横摆角和俯仰角，以及重心偏差
            
            % 1. 垂向调整 (通过空气弹簧垫片)
            % 使用垂向激光传感器数据和姿态角
            vertDisp = obj.sensorData.verticalDisplacements;
            
            % 目标是使所有四个角的垂向位移一致
            avgVertDisp = mean(vertDisp);
            verticalShims = vertDisp - avgVertDisp;
            
            % 限制垫片厚度在允许范围内
            verticalShims = max(min(verticalShims, obj.config.maxShimThickness), ...
                              -obj.config.maxShimThickness);
            
            % 2. 水平调整 (通过导向轮和稳定轮垫片)
            % 使用水平激光传感器数据
            horizDisp = obj.sensorData.horizontalDisplacements;
            
            % 目标是使所有水平轮的横向位移一致
            avgHorizDisp = mean(horizDisp);
            horizontalShims = horizDisp - avgHorizDisp;
            
            % 考虑横摆角的影响
            horizPos = obj.config.horizontalLaserPositions;
            
            % 补偿由于横摆角引起的位移差异
            for i = 1:obj.config.NUM_HORIZONTAL_SHIMS_TOTAL
                y_pos = horizPos(i, 2);
                rollCompensation = y_pos * tan(obj.attitude.roll);
                horizontalShims(i) = horizontalShims(i) - rollCompensation;
            end
            
            % 限制垫片厚度
            horizontalShims = max(min(horizontalShims, obj.config.maxShimThickness), ...
                                -obj.config.maxShimThickness);
            
            % 3. 根据重心偏差进行细调
            % 如果重心偏向某一侧，在对侧增加垫片
            if abs(obj.attitude.cogDeviation(2)) > 10  % 横向偏差 > 10mm
                % 在重心偏移的反方向调整
                lateralAdjustment = -obj.attitude.cogDeviation(2) / 10;
                
                % 对左右两侧的垫片进行差异调整
                for i = 1:obj.config.NUM_HORIZONTAL_SHIMS_TOTAL
                    y_pos = horizPos(i, 2);
                    if y_pos < 0
                        horizontalShims(i) = horizontalShims(i) + lateralAdjustment;
                    else
                        horizontalShims(i) = horizontalShims(i) - lateralAdjustment;
                    end
                end
            end
            
            if abs(obj.attitude.cogDeviation(1)) > 10  % 纵向偏差 > 10mm
                % 在重心偏移的反方向调整垂向垫片
                longitudinalAdjustment = -obj.attitude.cogDeviation(1) / 10;
                
                airSpringPos = obj.config.airSpringPositions;
                for i = 1:obj.config.NUM_VERTICAL_SHIMS
                    x_pos = airSpringPos(i, 1);
                    if x_pos < 0
                        verticalShims(i) = verticalShims(i) + longitudinalAdjustment;
                    else
                        verticalShims(i) = verticalShims(i) - longitudinalAdjustment;
                    end
                end
            end
            
            % 合并所有垫片调整
            obj.shimAdjustments = [horizontalShims; verticalShims];
            
            % 按照增量步长四舍五入
            obj.shimAdjustments = round(obj.shimAdjustments / obj.config.shimIncrement) * ...
                                 obj.config.shimIncrement;
        end
        
        function obj = optimizeShimAdjustments(obj)
            % 优化垫片调整量 (迭代方法)
            % Optimize shim adjustments using iterative method
            
            tolerance = obj.config.convergenceTolerance;
            maxIter = obj.config.maxIterations;
            
            for iter = 1:maxIter
                % 保存当前调整量
                prevShims = obj.shimAdjustments;
                
                % 计算当前姿态
                obj = obj.calculateAttitude();
                
                % 计算新的调整量
                obj = obj.calculateShimAdjustments();
                
                % 检查收敛
                shimChange = norm(obj.shimAdjustments - prevShims);
                
                if shimChange < tolerance
                    fprintf('优化收敛于第 %d 次迭代 (变化量: %.4f mm)\n', iter, shimChange);
                    break;
                end
                
                if iter == maxIter
                    warning('达到最大迭代次数，未完全收敛');
                end
            end
        end
        
        function displayResults(obj)
            % 显示调平结果
            fprintf('\n========== 单轨车辆调平系统结果 ==========\n\n');
            
            % 显示车辆姿态
            disp(obj.attitude);
            
            % 显示垫片调整
            fprintf('\n垫片调整量 Shim Adjustments:\n');
            fprintf('  水平垫片 Horizontal Shims (mm):\n');
            for i = 1:obj.config.NUM_HORIZONTAL_SHIMS_TOTAL
                fprintf('    位置 %2d: %6.2f\n', i, obj.shimAdjustments(i));
            end
            
            fprintf('  垂向垫片 Vertical Shims (mm):\n');
            for i = 1:obj.config.NUM_VERTICAL_SHIMS
                idx = obj.config.NUM_HORIZONTAL_SHIMS_TOTAL + i;
                fprintf('    位置 %2d: %6.2f\n', i, obj.shimAdjustments(idx));
            end
            
            % 检查调平状态
            toleranceAngle = deg2rad(0.1);  % 0.1度
            toleranceCog = 5;  % 5mm
            isLevel = obj.attitude.checkLevelStatus(toleranceAngle, toleranceCog);
            
            fprintf('\n调平状态 Leveling Status: ');
            if isLevel
                fprintf('已调平 ✓\n');
            else
                fprintf('未调平 ✗\n');
            end
            
            fprintf('\n==========================================\n');
        end
        
        function fig = visualizeSystem(obj)
            % 可视化系统状态
            % Visualize system state
            
            fig = figure('Name', '单轨车辆调平系统可视化', 'Position', [100, 100, 1200, 800]);
            
            % 子图1: 车辆姿态 (俯视图)
            subplot(2, 3, 1);
            obj.plotTopView();
            title('俯视图 Top View');
            
            % 子图2: 车辆姿态 (侧视图)
            subplot(2, 3, 2);
            obj.plotSideView();
            title('侧视图 Side View');
            
            % 子图3: 车辆姿态 (前视图)
            subplot(2, 3, 3);
            obj.plotFrontView();
            title('前视图 Front View');
            
            % 子图4: 载荷分布
            subplot(2, 3, 4);
            obj.plotLoadDistribution();
            title('载荷分布 Load Distribution');
            
            % 子图5: 垫片调整
            subplot(2, 3, 5);
            obj.plotShimAdjustments();
            title('垫片调整 Shim Adjustments');
            
            % 子图6: 重心位置
            subplot(2, 3, 6);
            obj.plotCenterOfGravity();
            title('重心位置 Center of Gravity');
        end
        
        function plotTopView(obj)
            % 绘制俯视图
            hold on;
            grid on;
            axis equal;
            
            % 绘制车体轮廓
            L = obj.config.vehicleLength / 2;
            W = obj.config.vehicleWidth / 2;
            rectangle('Position', [-L, -W, 2*L, 2*W], 'EdgeColor', 'k', 'LineWidth', 2);
            
            % 绘制转向架位置
            bogiePos = [-obj.config.bogieSpacing/2; obj.config.bogieSpacing/2];
            plot(bogiePos, [0; 0], 'bs', 'MarkerSize', 10, 'LineWidth', 2);
            
            % 绘制重心
            plot(obj.attitude.x, obj.attitude.y, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
            
            xlabel('X (mm)');
            ylabel('Y (mm)');
            legend('车体', '转向架', '重心');
        end
        
        function plotSideView(obj)
            % 绘制侧视图
            hold on;
            grid on;
            axis equal;
            
            % 绘制车体轮廓
            L = obj.config.vehicleLength / 2;
            H = obj.config.vehicleHeight;
            rectangle('Position', [-L, 0, 2*L, H], 'EdgeColor', 'k', 'LineWidth', 2);
            
            % 考虑俯仰角的影响
            pitchDeg = rad2deg(obj.attitude.pitch);
            title(sprintf('俯仰角: %.3f°', pitchDeg));
            
            xlabel('X (mm)');
            ylabel('Z (mm)');
        end
        
        function plotFrontView(obj)
            % 绘制前视图
            hold on;
            grid on;
            axis equal;
            
            % 绘制车体轮廓
            W = obj.config.vehicleWidth / 2;
            H = obj.config.vehicleHeight;
            rectangle('Position', [-W, 0, 2*W, H], 'EdgeColor', 'k', 'LineWidth', 2);
            
            % 考虑横摆角的影响
            rollDeg = rad2deg(obj.attitude.roll);
            title(sprintf('横摆角: %.3f°', rollDeg));
            
            xlabel('Y (mm)');
            ylabel('Z (mm)');
        end
        
        function plotLoadDistribution(obj)
            % 绘制载荷分布
            loads = obj.sensorData.getAllLoads();
            positions = obj.config.loadCellPositions;
            
            scatter3(positions(:,1), positions(:,2), loads, 100, loads, 'filled');
            colorbar;
            xlabel('X (mm)');
            ylabel('Y (mm)');
            zlabel('载荷 Load (N)');
            view(3);
            grid on;
        end
        
        function plotShimAdjustments(obj)
            % 绘制垫片调整量
            numShims = length(obj.shimAdjustments);
            bar(1:numShims, obj.shimAdjustments);
            
            % 标记水平和垂向垫片
            hold on;
            plot([obj.config.NUM_HORIZONTAL_SHIMS_TOTAL, ...
                  obj.config.NUM_HORIZONTAL_SHIMS_TOTAL] + 0.5, ...
                 ylim, 'r--', 'LineWidth', 2);
            
            xlabel('垫片位置 Shim Position');
            ylabel('调整量 Adjustment (mm)');
            legend('垫片调整', '水平/垂向分界');
            grid on;
        end
        
        function plotCenterOfGravity(obj)
            % 绘制重心位置
            hold on;
            grid on;
            
            % 理论重心
            plot3(0, 0, 0, 'go', 'MarkerSize', 15, 'LineWidth', 2);
            
            % 实际重心
            plot3(obj.attitude.x, obj.attitude.y, obj.attitude.z, ...
                  'r*', 'MarkerSize', 15, 'LineWidth', 2);
            
            % 连线显示偏差
            plot3([0, obj.attitude.x], [0, obj.attitude.y], [0, obj.attitude.z], ...
                  'r--', 'LineWidth', 2);
            
            xlabel('X (mm)');
            ylabel('Y (mm)');
            zlabel('Z (mm)');
            legend('理论重心', '实际重心', '偏差');
            view(3);
            
            % 显示偏差数值
            deviation = norm(obj.attitude.cogDeviation);
            title(sprintf('重心偏差: %.2f mm', deviation));
        end
    end
end
