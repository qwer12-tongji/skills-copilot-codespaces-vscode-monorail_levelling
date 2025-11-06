classdef SensorData
    % SensorData - 传感器数据类
    % Sensor data class for monorail vehicle leveling system
    
    properties
        % 称重数据 Weight measurements (N)
        runningWheelLoads;      % 走行轮载荷 [4x1]
        guideWheelLoads;        % 导向轮载荷 [8x1]
        stabilizingWheelLoads;  % 稳定轮载荷 [4x1]
        
        % 激光位移传感器数据 Laser displacement measurements (mm)
        horizontalDisplacements; % 水平距离 [12x1]
        verticalDisplacements;   % 垂向距离 [4x1]
        
        % 时间戳 Timestamp
        timestamp;
        
        % 数据质量标志 Data quality flags
        isValid;
    end
    
    methods
        function obj = SensorData()
            % 构造函数
            obj.runningWheelLoads = zeros(4, 1);
            obj.guideWheelLoads = zeros(8, 1);
            obj.stabilizingWheelLoads = zeros(4, 1);
            obj.horizontalDisplacements = zeros(12, 1);
            obj.verticalDisplacements = zeros(4, 1);
            obj.timestamp = datetime('now');
            obj.isValid = true;
        end
        
        function obj = loadFromMeasurement(obj, runningLoads, guideLoads, ...
                                          stabLoads, horizDisp, vertDisp)
            % 从测量值加载数据
            % Load data from measurements
            obj.runningWheelLoads = runningLoads;
            obj.guideWheelLoads = guideLoads;
            obj.stabilizingWheelLoads = stabLoads;
            obj.horizontalDisplacements = horizDisp;
            obj.verticalDisplacements = vertDisp;
            obj.timestamp = datetime('now');
            obj.isValid = obj.validateData();
        end
        
        function valid = validateData(obj)
            % 验证数据有效性
            % Validate sensor data
            valid = true;
            
            % 检查载荷数据是否为正值
            if any(obj.runningWheelLoads < 0) || ...
               any(obj.guideWheelLoads < 0) || ...
               any(obj.stabilizingWheelLoads < 0)
                valid = false;
                warning('Negative load detected in sensor data');
            end
            
            % 检查位移数据是否在合理范围内 (-100mm to 100mm)
            if any(abs(obj.horizontalDisplacements) > 100) || ...
               any(abs(obj.verticalDisplacements) > 100)
                valid = false;
                warning('Displacement out of reasonable range');
            end
        end
        
        function totalLoad = getTotalLoad(obj)
            % 获取总载荷
            % Get total load
            totalLoad = sum(obj.runningWheelLoads) + ...
                       sum(obj.guideWheelLoads) + ...
                       sum(obj.stabilizingWheelLoads);
        end
        
        function allLoads = getAllLoads(obj)
            % 获取所有载荷数据为单个向量
            % Get all loads as a single vector
            allLoads = [obj.runningWheelLoads; 
                       obj.guideWheelLoads; 
                       obj.stabilizingWheelLoads];
        end
    end
end
