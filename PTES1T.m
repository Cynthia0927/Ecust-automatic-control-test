clc; clear;  % �������ں͹���������

%% === 1. ��ȡCSV���� ===
data = readtable('B.csv');  % ��ȡCSV�ļ������ṹ
time = data.time;           % ��ȡʱ���У���λ���룩
temp = data.temperature;    % ��ȡ�¶��У���λ�����϶ȣ�

%% === 2. ϵͳ��ʶ ===
u_step = 3.5;  % �����Ծ��ѹ��ֵ�����أ�

% �����ʼ�¶ȣ�ǰ100���������ƽ��ֵ��
T0 = mean(temp(1:100));  
% ������̬�¶ȣ����50���������ƽ��ֵ��
T_inf = mean(temp(end-50:end));  

% ����ϵͳ��̬����K = ���¶� / ������
K = (T_inf - T0) / u_step;  

% ����ﵽ63.2%��̬�仯�����¶ȵ�
T_63 = T0 + 0.632 * (T_inf - T0);  
% �ҵ��״γ���T_63���¶ȵ�����
idx_T = find(temp >= T_63, 1);  
% �ҵ��¶��״����������ĵ㣨������ʼֵ0.5�棩
idx_L = find(temp > T0 + 0.5, 1);  

% ����ʱ�䳣��T = �ﵽ63.2%��ʱ�� - ��Ӧ��ʼʱ��
T_val = time(idx_T) - time(idx_L);  
% �ӳ�ʱ��� = ��Ӧ��ʼʱ��
L = time(idx_L);  

% ����һ�׹��Ի��ڴ��ݺ��� G(s) = K/(T*s+1)
G = tf(K, [T_val 1]);  
% �Դ��ӳٻ��ڽ���3��Pade���ƣ����ģ�;��ȣ�
G_delay = pade(tf(1,1,'InputDelay',L), 3);  
% ��ϳ�����ģ�ͣ����Ի��� + �ӳٻ���
G_total = G * G_delay;  

% ��ʾ��ʶ���
disp('һ�׹���+�ӳ�ϵͳ��ʶ�����3�� Pade����');
G_total  % ������ݺ���ģ��

%% === 3. ģ����֤��Ա� ===
% ����ģ�ͶԽ�Ծ�������Ӧ
[y_model, t_model] = step(G_total * u_step, time);  
% ��ӳ�ʼ�¶�ƫ�ƣ�ģ�������0��ʼ��
y_model = y_model + T0;  

% ���ƶԱ�����
figure;
plot(time, temp, 'b', 'DisplayName', 'ԭʼ�¶�');  % ԭʼʵ������
hold on;
plot(t_model, y_model, 'r--', 'DisplayName', 'ģ�����');  % ģ�����
xlabel('ʱ�� (s)'); ylabel('�¶� (��C)');
title('ϵͳ��ʶ�Ա�');
legend; grid on;

% ��ԭʼ���ݲ�ֵ��ģ��ʱ����ϣ����ڼ�������Ŷȣ�
temp_interp = interp1(time, temp, t_model);  
% ����в�ƽ����
SS_res = sum((temp_interp - y_model).^2);  
% ������ƽ����
SS_tot = sum((temp_interp - mean(temp_interp)).^2);  
% ����R�0�5����Ŷ�
R2 = 1 - SS_res / SS_tot;  
fprintf('\n����Ŷ� R�0�5 = %.4f\n', R2);  % ����������

%% === 4. ����Ⱥ�Ż�PID���� ===
target_temp = 35;    % Ŀ������¶�
room_temp = 16.8;    % �����¶ȣ��ⲿ�Ŷ���
tspan = 0:1:600;     % ����ʱ�䷶Χ��0-600�룬����1�룩

% ����PSOĿ�꺯��������PID�������ܣ�
cost_func = @(x) pid_fitness_modified(x, G_total, target_temp, tspan, room_temp);

% ����PID����������Χ [Kp, Ki, Kd]
lb = [0 0 0];        % ��������
ub = [10 0.1 1000];  % ��������

% ��������Ⱥ�Ż�ѡ��
opts = optimoptions('particleswarm', ...
    'Display', 'iter', ...    % ��ʾ��������
    'SwarmSize', 50, ...      % ��������
    'MaxIterations', 80);     % ����������

% ����PSO�Ż�
[x_opt, fval] = particleswarm(cost_func, 3, lb, ub, opts);

% ��ȡ�Ż����PID����
Kp = x_opt(1); Ki = x_opt(2); Kd = x_opt(3);
fprintf('\n���� PID ����: Kp=%.4f, Ki=%.5f, Kd=%.4f\n', Kp, Ki, Kd);

%% === 5. �ջ�ϵͳ���� ===
% ����PID����������
PID = pid(Kp, Ki, Kd);  

% �������Ŷ��Ŀ���ϵͳ��u -> PID -> ����¯ģ��
G_disturbed = G_total;  % ���ض��󣨼���¯ģ�ͣ�
sys_open = PID * G_disturbed;  % �������ݺ���

% �����ջ�ϵͳ����λ��������
sys_cl = feedback(sys_open, 1);  

% ����ջ�ϵͳ��Ծ��Ӧ��Ŀ��ֵ��ȥ�����¶���Ϊ���룩
[y_out, t_out] = step(sys_cl * (target_temp - room_temp), tspan);

% ��ӻ����¶��Ŷ���������ӣ�
y_out = y_out + room_temp;  

% ���Ʊջ���Ӧ����
figure;
plot(t_out, y_out, 'b', 'LineWidth', 1.5); 
hold on;
yline(target_temp, '--r', 'Ŀ���¶�');  % Ŀ���¶Ȳο���
xlabel('ʱ�� (s)'); ylabel('�¶� (��C)');
title('�Ż���ջ���Ӧ�����Ŷ�16.8��C��');
legend('ϵͳ���', 'Ŀ���¶�');
grid on;

%% === 6. ����ָ����� ===
% �����Ծ��Ӧ����ָ��
S = stepinfo(y_out, t_out, target_temp);

% ����ؼ�����ָ��
fprintf('\n=== У����ջ�����ָ�� ===\n');
fprintf('������ (Overshoot): %.2f%%\n', S.Overshoot);
fprintf('����ʱ�� (SettlingTime): %.2f ��\n', S.SettlingTime);
fprintf('����ʱ�� (RiseTime): %.2f ��\n', S.RiseTime);
fprintf('��ֵʱ�� (PeakTime): %.2f ��\n', S.PeakTime);
fprintf('��ֵ̬ (Steady-State Value): %.2f ��C\n', y_out(end));

%% === ����������PID������������ ===
function cost = pid_fitness_modified(x, G, target, tspan, disturb)
    % ���������
    %   x: PID�������� [Kp, Ki, Kd]
    %   G: ���ض��󴫵ݺ���
    %   target: Ŀ���¶�
    %   tspan: ����ʱ������
    %   disturb: �����Ŷ��¶�
    
    % ��ȡPID����
    Kp = x(1); Ki = x(2); Kd = x(3);
    
    % ����PID������
    PID = pid(Kp, Ki, Kd);
    
    % ��������ϵͳ
    sys_open = PID * G;
    
    % �����ջ�ϵͳ����λ��������
    sys_cl = feedback(sys_open, 1);
    
    % �����Ծ��Ӧ������ΪĿ��ֵ���Ŷ��Ĳ�ֵ��
    [y, t] = step(sys_cl * (target - disturb), tspan);
    
    % ��ӻ����Ŷ��¶�
    y = y + disturb;
    
    % ��������ָ��
    overshoot = max(y) - target;           % �������������¶�ֵ��
    steady_error = abs(y(end) - target);   % ��̬������ֵ��
    
    % ʹ��stepinfo��ȡϵͳ��Ӧ����
    S = stepinfo(y, t, target);
    settling_time = S.SettlingTime;        % ����ʱ��
    rise_time = S.RiseTime;                % ����ʱ��
    
    % �����쳣ֵ����stepinfo����NaN�򳬳����淶Χʱ��
    if isnan(settling_time) || settling_time > t(end)
        settling_time = t(end);  % ��Ϊ������ʱ��
    end
    if isnan(rise_time) || rise_time > t(end)
        rise_time = t(end);      % ��Ϊ������ʱ��
    end

    % ��ȨĿ�꺯����Ȩ��ϵ�����ݿ������������
    % ��̬���Ȩ��100 > ����Ȩ��20 > ����ʱ��Ȩ��10 > ����ʱ��Ȩ��0.5
    cost = 100 * steady_error + 20 * abs(overshoot) + 0.5 * rise_time + 10 * settling_time;
end
