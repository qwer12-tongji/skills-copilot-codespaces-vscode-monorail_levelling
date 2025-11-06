classdef VehicleAttitude
    % VehicleAttitude - 车辆姿态类
    % Vehicle attitude class containing position and orientation
    
    properties
        % 位置 Position (mm)
        x;  % 纵向位置 Longitudinal
        y;  % 横向位置 Lateral
        z;  % 垂向位置 Vertical
        
        % 姿态角 Attitude angles (rad)
        roll;   % 横摆角 (绕x轴)
        pitch;  % 俯仰角 (绕y轴)
        yaw;    % 偏航角 (绕z轴)
        
        % 重心偏差 Center of Gravity deviation (mm)
        cogDeviation;  % [dx, dy, dz] 相对标称位置的偏差
        
        % 外部力矩 External moments (N·m)
        externalMoments;  % [Mx, My, Mz]
        
        % 转向架重心偏差 Bogie CG deviations
        bogie1CogDeviation;  % 转向架1重心偏差 [dx, dy, dz]
        bogie2CogDeviation;  % 转向架2重心偏差 [dx, dy, dz]
    end
    
    methods
        function obj = VehicleAttitude()
            % 构造函数 - 初始化为零
            obj.x = 0;
            obj.y = 0;
            obj.z = 0;
            obj.roll = 0;
            obj.pitch = 0;
            obj.yaw = 0;
            obj.cogDeviation = [0; 0; 0];
            obj.externalMoments = [0; 0; 0];
            obj.bogie1CogDeviation = [0; 0; 0];
            obj.bogie2CogDeviation = [0; 0; 0];
        end
        
        function obj = setPosition(obj, x, y, z)
            % 设置位置
            obj.x = x;
            obj.y = y;
            obj.z = z;
        end
        
        function obj = setOrientation(obj, roll, pitch, yaw)
            % 设置姿态角
            obj.roll = roll;
            obj.pitch = pitch;
            obj.yaw = yaw;
        end
        
        function obj = setCogDeviation(obj, deviation)
            % 设置重心偏差
            obj.cogDeviation = deviation;
        end
        
        function obj = setExternalMoments(obj, moments)
            % 设置外部力矩
            obj.externalMoments = moments;
        end
        
        function R = getRotationMatrix(obj)
            % 获取旋转矩阵 (ZYX欧拉角顺序)
            % Get rotation matrix (ZYX Euler angle sequence)
            
            % 旋转矩阵分量
            Rx = [1, 0, 0;
                  0, cos(obj.roll), -sin(obj.roll);
                  0, sin(obj.roll), cos(obj.roll)];
            
            Ry = [cos(obj.pitch), 0, sin(obj.pitch);
                  0, 1, 0;
                  -sin(obj.pitch), 0, cos(obj.pitch)];
            
            Rz = [cos(obj.yaw), -sin(obj.yaw), 0;
                  sin(obj.yaw), cos(obj.yaw), 0;
                  0, 0, 1];
            
            R = Rz * Ry * Rx;
        end
        
        function isLevel = checkLevelStatus(obj, tolerance_angle, tolerance_cog)
            % 检查车辆是否水平
            % Check if vehicle is level within tolerances
            % tolerance_angle: 角度公差 (rad)
            % tolerance_cog: 重心偏差公差 (mm)
            
            angle_ok = abs(obj.roll) < tolerance_angle && ...
                      abs(obj.pitch) < tolerance_angle;
            
            cog_ok = norm(obj.cogDeviation) < tolerance_cog;
            
            isLevel = angle_ok && cog_ok;
        end
        
        function disp(obj)
            % 显示车辆姿态信息
            fprintf('车辆姿态 Vehicle Attitude:\n');
            fprintf('  位置 Position (mm): [%.2f, %.2f, %.2f]\n', obj.x, obj.y, obj.z);
            fprintf('  横摆角 Roll (deg): %.4f\n', rad2deg(obj.roll));
            fprintf('  俯仰角 Pitch (deg): %.4f\n', rad2deg(obj.pitch));
            fprintf('  偏航角 Yaw (deg): %.4f\n', rad2deg(obj.yaw));
            fprintf('  重心偏差 COG Deviation (mm): [%.2f, %.2f, %.2f]\n', ...
                obj.cogDeviation(1), obj.cogDeviation(2), obj.cogDeviation(3));
            fprintf('  外部力矩 External Moments (N·m): [%.2f, %.2f, %.2f]\n', ...
                obj.externalMoments(1), obj.externalMoments(2), obj.externalMoments(3));
            fprintf('  转向架1重心偏差 Bogie1 COG Dev (mm): [%.2f, %.2f, %.2f]\n', ...
                obj.bogie1CogDeviation(1), obj.bogie1CogDeviation(2), obj.bogie1CogDeviation(3));
            fprintf('  转向架2重心偏差 Bogie2 COG Dev (mm): [%.2f, %.2f, %.2f]\n', ...
                obj.bogie2CogDeviation(1), obj.bogie2CogDeviation(2), obj.bogie2CogDeviation(3));
        end
    end
end
