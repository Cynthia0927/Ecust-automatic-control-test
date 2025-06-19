clc; clear;  % 清空命令窗口和工作区变量

%% === 1. 读取CSV数据 ===
data = readtable('B.csv');  % 读取CSV文件到表格结构
time = data.time;           % 提取时间列（单位：秒）
temp = data.temperature;    % 提取温度列（单位：摄氏度）

%% === 2. 系统辨识 ===
u_step = 3.5;  % 输入阶跃电压幅值（伏特）

% 计算初始温度（前100个采样点的平均值）
T0 = mean(temp(1:100));  
% 计算稳态温度（最后50个采样点的平均值）
T_inf = mean(temp(end-50:end));  

% 计算系统静态增益K = Δ温度 / Δ输入
K = (T_inf - T0) / u_step;  

% 计算达到63.2%稳态变化量的温度点
T_63 = T0 + 0.632 * (T_inf - T0);  
% 找到首次超过T_63的温度点索引
idx_T = find(temp >= T_63, 1);  
% 找到温度首次明显上升的点（超过初始值0.5℃）
idx_L = find(temp > T0 + 0.5, 1);  

% 计算时间常数T = 达到63.2%的时间 - 响应开始时间
T_val = time(idx_T) - time(idx_L);  
% 延迟时间τ = 响应开始时间
L = time(idx_L);  

% 创建一阶惯性环节传递函数 G(s) = K/(T*s+1)
G = tf(K, [T_val 1]);  
% 对纯延迟环节进行3阶Pade近似（提高模型精度）
G_delay = pade(tf(1,1,'InputDelay',L), 3);  
% 组合成完整模型：惯性环节 + 延迟环节
G_total = G * G_delay;  

% 显示辨识结果
disp('一阶惯性+延迟系统辨识结果（3阶 Pade）：');
G_total  % 输出传递函数模型

%% === 3. 模型验证与对比 ===
% 计算模型对阶跃输入的响应
[y_model, t_model] = step(G_total * u_step, time);  
% 添加初始温度偏移（模型输出从0开始）
y_model = y_model + T0;  

% 绘制对比曲线
figure;
plot(time, temp, 'b', 'DisplayName', '原始温度');  % 原始实验数据
hold on;
plot(t_model, y_model, 'r--', 'DisplayName', '模型输出');  % 模型输出
xlabel('时间 (s)'); ylabel('温度 (°C)');
title('系统辨识对比');
legend; grid on;

% 将原始数据插值到模型时间点上（用于计算拟合优度）
temp_interp = interp1(time, temp, t_model);  
% 计算残差平方和
SS_res = sum((temp_interp - y_model).^2);  
% 计算总平方和
SS_tot = sum((temp_interp - mean(temp_interp)).^2);  
% 计算R²拟合优度
R2 = 1 - SS_res / SS_tot;  
fprintf('\n拟合优度 R² = %.4f\n', R2);  % 输出拟合质量

%% === 4. 粒子群优化PID参数 ===
target_temp = 35;    % 目标控制温度
room_temp = 16.8;    % 环境温度（外部扰动）
tspan = 0:1:600;     % 仿真时间范围（0-600秒，步长1秒）

% 定义PSO目标函数（评估PID参数性能）
cost_func = @(x) pid_fitness_modified(x, G_total, target_temp, tspan, room_temp);

% 设置PID参数搜索范围 [Kp, Ki, Kd]
lb = [0 0 0];        % 参数下限
ub = [10 0.1 1000];  % 参数上限

% 配置粒子群优化选项
opts = optimoptions('particleswarm', ...
    'Display', 'iter', ...    % 显示迭代过程
    'SwarmSize', 50, ...      % 粒子数量
    'MaxIterations', 80);     % 最大迭代次数

% 运行PSO优化
[x_opt, fval] = particleswarm(cost_func, 3, lb, ub, opts);

% 提取优化后的PID参数
Kp = x_opt(1); Ki = x_opt(2); Kd = x_opt(3);
fprintf('\n最优 PID 参数: Kp=%.4f, Ki=%.5f, Kd=%.4f\n', Kp, Ki, Kd);

%% === 5. 闭环系统仿真 ===
% 创建PID控制器对象
PID = pid(Kp, Ki, Kd);  

% 构建带扰动的开环系统：u -> PID -> 加热炉模型
G_disturbed = G_total;  % 被控对象（加热炉模型）
sys_open = PID * G_disturbed;  % 开环传递函数

% 构建闭环系统（单位负反馈）
sys_cl = feedback(sys_open, 1);  

% 计算闭环系统阶跃响应（目标值减去环境温度作为输入）
[y_out, t_out] = step(sys_cl * (target_temp - room_temp), tspan);

% 添加环境温度扰动（输出叠加）
y_out = y_out + room_temp;  

% 绘制闭环响应曲线
figure;
plot(t_out, y_out, 'b', 'LineWidth', 1.5); 
hold on;
yline(target_temp, '--r', '目标温度');  % 目标温度参考线
xlabel('时间 (s)'); ylabel('温度 (°C)');
title('优化后闭环响应（含扰动16.8°C）');
legend('系统输出', '目标温度');
grid on;

%% === 6. 性能指标分析 ===
% 计算阶跃响应性能指标
S = stepinfo(y_out, t_out, target_temp);

% 输出关键性能指标
fprintf('\n=== 校正后闭环性能指标 ===\n');
fprintf('超调量 (Overshoot): %.2f%%\n', S.Overshoot);
fprintf('调节时间 (SettlingTime): %.2f 秒\n', S.SettlingTime);
fprintf('上升时间 (RiseTime): %.2f 秒\n', S.RiseTime);
fprintf('峰值时间 (PeakTime): %.2f 秒\n', S.PeakTime);
fprintf('稳态值 (Steady-State Value): %.2f °C\n', y_out(end));

%% === 辅助函数：PID参数评估函数 ===
function cost = pid_fitness_modified(x, G, target, tspan, disturb)
    % 输入参数：
    %   x: PID参数向量 [Kp, Ki, Kd]
    %   G: 被控对象传递函数
    %   target: 目标温度
    %   tspan: 仿真时间向量
    %   disturb: 环境扰动温度
    
    % 提取PID参数
    Kp = x(1); Ki = x(2); Kd = x(3);
    
    % 创建PID控制器
    PID = pid(Kp, Ki, Kd);
    
    % 构建开环系统
    sys_open = PID * G;
    
    % 构建闭环系统（单位负反馈）
    sys_cl = feedback(sys_open, 1);
    
    % 计算阶跃响应（输入为目标值与扰动的差值）
    [y, t] = step(sys_cl * (target - disturb), tspan);
    
    % 添加环境扰动温度
    y = y + disturb;
    
    % 计算性能指标
    overshoot = max(y) - target;           % 超调量（绝对温度值）
    steady_error = abs(y(end) - target);   % 稳态误差（绝对值）
    
    % 使用stepinfo获取系统响应特性
    S = stepinfo(y, t, target);
    settling_time = S.SettlingTime;        % 调节时间
    rise_time = S.RiseTime;                % 上升时间
    
    % 处理异常值（当stepinfo返回NaN或超出仿真范围时）
    if isnan(settling_time) || settling_time > t(end)
        settling_time = t(end);  % 设为最大仿真时间
    end
    if isnan(rise_time) || rise_time > t(end)
        rise_time = t(end);      % 设为最大仿真时间
    end

    % 加权目标函数（权重系数根据控制需求调整）
    % 稳态误差权重100 > 超调权重20 > 调节时间权重10 > 上升时间权重0.5
    cost = 100 * steady_error + 20 * abs(overshoot) + 0.5 * rise_time + 10 * settling_time;
end
