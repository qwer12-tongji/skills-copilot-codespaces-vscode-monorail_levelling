classdef MonorailLevelingConfig
    % MonorailLevelingConfig - Configuration parameters for monorail vehicle leveling system
    % 单轨车辆调平系统配置参数
    
    properties (Constant)
        % 转向架配置 Bogie Configuration
        NUM_BOGIES = 2;                    % 每节车体转向架数量
        NUM_RUNNING_WHEELS = 2;            % 每个转向架走行轮数量
        NUM_GUIDE_WHEELS = 4;              % 每个转向架导向轮数量
        NUM_STABILIZING_WHEELS = 2;        % 每个转向架稳定轮数量
        NUM_AIR_SPRINGS = 4;               % 空气弹簧数量
        
        % 调平位置数量 Adjustment Positions
        NUM_HORIZONTAL_SHIMS_PER_BOGIE = 6;  % 每个转向架水平垫片位置(4导向+2稳定)
        NUM_HORIZONTAL_SHIMS_TOTAL = 12;     % 总水平垫片位置
        NUM_VERTICAL_SHIMS = 4;              % 垂向垫片位置(空气弹簧)
        NUM_TOTAL_SHIMS = 16;                % 总调整位置
        
        % 传感器配置 Sensor Configuration
        NUM_HORIZONTAL_LASER_SENSORS = 12;   % 水平激光位移传感器数量
        NUM_VERTICAL_LASER_SENSORS = 4;      % 垂向激光位移传感器数量
        NUM_LOAD_CELLS = 14;                 % 称重传感器数量 (2走行轮+12水平轮) per bogie
        
        % 物理常数 Physical Constants
        GRAVITY = 9.81;                      % 重力加速度 (m/s^2)
    end
    
    properties
        % 车辆尺寸参数 Vehicle Dimensions (单位: mm)
        vehicleLength;          % 车体长度
        vehicleWidth;           % 车体宽度
        vehicleHeight;          % 车体高度
        bogieSpacing;           % 转向架间距
        wheelBase;              % 轮距
        
        % 位置坐标 Position Coordinates (相对车体中心, mm)
        runningWheelPositions;      % 走行轮位置 [2*NUM_BOGIES x 3]
        guideWheelPositions;        % 导向轮位置 [4*NUM_BOGIES x 3]
        stabilizingWheelPositions;  % 稳定轮位置 [2*NUM_BOGIES x 3]
        airSpringPositions;         % 空气弹簧位置 [NUM_AIR_SPRINGS x 3]
        
        % 传感器位置 Sensor Positions
        horizontalLaserPositions;   % 水平激光传感器位置
        verticalLaserPositions;     % 垂向激光传感器位置
        loadCellPositions;          % 称重传感器位置
        
        % 质量参数 Mass Parameters (kg)
        vehicleBodyMass;        % 车体质量
        bogieMass;              % 单个转向架质量
        
        % 公差参数 Tolerance Parameters (mm)
        levelingTolerance;      % 调平公差
        maxShimThickness;       % 最大垫片厚度
        shimIncrement;          % 垫片增量
        
        % 优化参数 Optimization Parameters
        maxIterations;          % 最大迭代次数
        convergenceTolerance;   % 收敛公差
    end
    
    methods
        function obj = MonorailLevelingConfig()
            % 默认构造函数 - 使用典型单轨车辆参数
            % Default constructor - use typical monorail vehicle parameters
            
            % 车辆尺寸 (mm)
            obj.vehicleLength = 12000;
            obj.vehicleWidth = 2600;
            obj.vehicleHeight = 3800;
            obj.bogieSpacing = 8000;
            obj.wheelBase = 1800;
            
            % 初始化位置坐标
            obj = obj.initializePositions();
            
            % 质量参数 (kg)
            obj.vehicleBodyMass = 25000;
            obj.bogieMass = 2000;
            
            % 公差参数 (mm)
            obj.levelingTolerance = 1.0;
            obj.maxShimThickness = 20.0;
            obj.shimIncrement = 0.5;
            
            % 优化参数
            obj.maxIterations = 100;
            obj.convergenceTolerance = 0.1;
        end
        
        function obj = initializePositions(obj)
            % 初始化各部件位置坐标 (相对车体中心)
            % Initialize component positions (relative to vehicle center)
            
            % 转向架中心位置
            bogie1_x = -obj.bogieSpacing/2;
            bogie2_x = obj.bogieSpacing/2;
            
            % 走行轮位置 (每个转向架2个, 沿宽度方向分布)
            obj.runningWheelPositions = [
                bogie1_x, -obj.wheelBase/2, 0;
                bogie1_x,  obj.wheelBase/2, 0;
                bogie2_x, -obj.wheelBase/2, 0;
                bogie2_x,  obj.wheelBase/2, 0
            ];
            
            % 导向轮位置 (每个转向架4个)
            guide_offset_x = 400;  % 导向轮纵向偏移
            guide_offset_z = -200; % 导向轮垂向偏移
            obj.guideWheelPositions = [
                bogie1_x-guide_offset_x, -obj.wheelBase/2, guide_offset_z;
                bogie1_x-guide_offset_x,  obj.wheelBase/2, guide_offset_z;
                bogie1_x+guide_offset_x, -obj.wheelBase/2, guide_offset_z;
                bogie1_x+guide_offset_x,  obj.wheelBase/2, guide_offset_z;
                bogie2_x-guide_offset_x, -obj.wheelBase/2, guide_offset_z;
                bogie2_x-guide_offset_x,  obj.wheelBase/2, guide_offset_z;
                bogie2_x+guide_offset_x, -obj.wheelBase/2, guide_offset_z;
                bogie2_x+guide_offset_x,  obj.wheelBase/2, guide_offset_z
            ];
            
            % 稳定轮位置 (每个转向架2个)
            stab_offset_z = 100;  % 稳定轮垂向偏移
            obj.stabilizingWheelPositions = [
                bogie1_x, -obj.wheelBase/2, stab_offset_z;
                bogie1_x,  obj.wheelBase/2, stab_offset_z;
                bogie2_x, -obj.wheelBase/2, stab_offset_z;
                bogie2_x,  obj.wheelBase/2, stab_offset_z
            ];
            
            % 空气弹簧位置 (车体四角)
            spring_offset_x = obj.vehicleLength/2 - 1000;
            spring_offset_y = obj.wheelBase/2;
            obj.airSpringPositions = [
                -spring_offset_x, -spring_offset_y, 0;
                -spring_offset_x,  spring_offset_y, 0;
                 spring_offset_x, -spring_offset_y, 0;
                 spring_offset_x,  spring_offset_y, 0
            ];
            
            % 水平激光传感器位置 (对应所有水平轮)
            obj.horizontalLaserPositions = [
                obj.guideWheelPositions;
                obj.stabilizingWheelPositions
            ];
            
            % 垂向激光传感器位置 (车体四角)
            corner_offset_x = obj.vehicleLength/2 - 500;
            corner_offset_y = obj.vehicleWidth/2 - 300;
            obj.verticalLaserPositions = [
                -corner_offset_x, -corner_offset_y, obj.vehicleHeight;
                -corner_offset_x,  corner_offset_y, obj.vehicleHeight;
                 corner_offset_x, -corner_offset_y, obj.vehicleHeight;
                 corner_offset_x,  corner_offset_y, obj.vehicleHeight
            ];
            
            % 称重传感器位置 (走行轮 + 所有水平轮)
            obj.loadCellPositions = [
                obj.runningWheelPositions;
                obj.guideWheelPositions;
                obj.stabilizingWheelPositions
            ];
        end
    end
end
