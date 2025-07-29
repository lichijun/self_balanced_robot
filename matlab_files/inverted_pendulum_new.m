%% 连续系统模型
M = 0.104; % 小车重（平衡小车轮子的重量52g*2：轮胎37.5g+联轴器13g+轮轴1.5g）
m = 0.650; % 摆杆重（平衡小车除轮子外的重量650g：总重量754g-两个轮子重量104g）
% m = 1.106; % 加配重后
l = 0.04; % 摆杆重心到铰链的距离（平衡小车除轮子外的部分的重心到轮轴的距离，吊绳估计4cm）
% l = 0.09; % 加配重后
I = m*l^2/3; % 摆杆惯量（平衡小车除轮子外的部分相对于轮轴的惯量，按重心估算ml^2/3）
g = 9.8;
R_w = 0.075/2; % 轮子半径
I_w = 0.00016; % 轮子+转子的惯量

p = 2*I*I_w + I*M*R_w^2 + I*R_w^2*m + 2*I_w*l^2*m + M*R_w^2*l^2*m; %denominator for the A and B matrices

A = [0      1                          0                       0;
     0      0                 (R_w^2*g*l^2*m^2)/p              0;
     0      0                          0                       1;
     0      0       (l*m*(2*I_w*g + M*R_w^2*g + R_w^2*g*m))/p  0];
B = [       0;
     (R_w*(m*l^2 + I))/p;
            0;
        (R_w*l*m)/p];
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x'; 'phi'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

poles = eig(A)

co = ctrb(sys_ss);
controllability = rank(co)

%% 连续系统lqr
Q = C'*C;
Q(1,1) = 20;
Q(3,3) = 40;
R = 1;
K = lqr(A,B,Q,R)

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'r'};
outputs = {'x'; 'phi'};

sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
r =0.2*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with LQR Control')

%% 连续系统离散化
Ts = 0.01;
sysd_ss = c2d(sys_ss, Ts);
Ad = sysd_ss.A;
Bd = sysd_ss.B;
Cd = sysd_ss.C;
Dd = sysd_ss.D;
poles = abs(pole(sysd_ss))

%% 离散系统lqr
Qd =  Cd'*Cd;
Qd(1,1) = 10;
Qd(2,2) = 0;
Qd(3,3) = 40;
Qd(4,4) = 0;
Rd = 1;
Kd = dlqr(Ad, Bd, Qd, Rd)

Adc = [(Ad-Bd*Kd)];
Bdc = [Bd];
Cdc = [Cd];
Ddc = [Dd];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'r'};
outputs = {'x'; 'phi'};

sysd_cl = ss(Adc,Bdc,Cdc,Ddc,Ts,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
r =zeros(size(t));
state_init = [0, 0, 5/180*pi, 0];
[y,t,x]=lsim(sysd_cl,r,t,state_init);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with LQR Control')

%电机pwm
ud = (-Kd * x')'; % 单轮所需力矩
motor_para = [-0.0049, 0.001587];
PWM = (ud - motor_para(1) * x(:, 2)/R_w) / motor_para(2); % x(:, 2)用单轮当前线速度更准确

%%-------------------------------------------------------------------------
%绕z轴惯量测试
M = 0.104; % 小车重（平衡小车轮子的重量52g*2：轮胎37.5g+联轴器13g+轮轴1.5g）
m = 0.650; % 摆杆重（平衡小车除轮子外的重量650g：总重量754g-两个轮子重量104g）
g = 9.8;
m0 = m+M; % 平衡小车重量0.754kg
d = 0.12;   % 转动惯量测试参数0.12m
L = 0.465;  % 转动惯量测试参数0.465m
T = 57;     % 转动惯量测试参数57s
J = m0*g*d^2 / (16*pi^2*L) * (T/50)^2; % 平衡时的惯量（倾斜会变大），小角度倾斜时认为惯量近似不变
%%-------------------------------------------------------------------------

%% 包含航向角的模型
M = 0.104; % 小车重（平衡小车轮子的重量52g*2：轮胎37.5g+联轴器13g+轮轴1.5g）
m = 0.650; % 摆杆重（平衡小车除轮子外的重量650g：总重量754g-两个轮子重量104g）
% m = 1.106; % 加配重后
l = 0.04; % 摆杆重心到铰链的距离（平衡小车除轮子外的部分的重心到轮轴的距离，吊绳估计4cm）
% l = 0.09; % 加配重后
I = m*l^2/3; % 摆杆惯量（平衡小车除轮子外的部分相对于轮轴的惯量，按重心估算ml^2/3）
g = 9.8;
Lw = 0.17;  % 轮距0.17m 

p = 2*I*I_w + I*M*R_w^2 + I*R_w^2*m + 2*I_w*l^2*m + M*R_w^2*l^2*m; %denominator for the A and B matrices

A = [0      1                          0                            0  0  0;
     0      0                 (R_w^2*g*l^2*m^2)/p                   0  0  0;
     0      0                          0                            1  0  0;
     0      0       (l*m*(2*I_w*g + M*R_w^2*g + R_w^2*g*m))/p       0  0  0;
     0      0                          0                            0  0  1;
     0      0                          0                            0  0  0];
B = [         0                                   0;
     (R_w*(m*l^2 + I))/p                          0;
              0                                   0;
        (R_w*l*m)/p                               0;
              0                                   0;
  -R_w*d/(4*J*R_w^2 + I_w*Lw^2)        R_w*d/(4*J*R_w^2 + I_w*Lw^2)];
C = [1 0 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 1 0];
D = [0 0;
     0 0;
     0 0];

states = {'x' 'x_dot' 'phi' 'phi_dot', 'theta', 'theta_dot'};
inputs = {'u1' 'u2'};
outputs = {'x'; 'phi'; 'theta'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

poles = eig(A)

co = ctrb(sys_ss);
controllability = rank(co)

%% 连续系统lqr
Q = C'*C;
Q(1,1) = 20;
Q(3,3) = 40;
R = 1;
K = lqr(A,B,Q,R)

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

states = {'x' 'x_dot' 'phi' 'phi_dot', 'theta', 'theta_dot'};
inputs = {'r'};
outputs = {'x'; 'phi'; 'theta'};

sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
num = size(t);
num = num(2);
r =zeros(num, 2);
state_init = [0, 0, 5/180*pi, 0, 0, 0];
[y,t,x]=lsim(sys_cl,r,t,state_init);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with LQR Control')

%% 连续系统离散化
Ts = 0.01;
sysd_ss = c2d(sys_ss, Ts);
Ad = sysd_ss.A;
Bd = sysd_ss.B;
Cd = sysd_ss.C;
Dd = sysd_ss.D;
poles = abs(pole(sysd_ss))

%% 离散系统lqr
Qd = C'*C;
Qd(1,1) = 20;
Qd(2,2) = 0;
Qd(3,3) = 400;
Qd(4,4) = 0;
Qd(5,5) = 20;
Qd(6,6) = 0;
Rd = 1;
Kd = dlqr(Ad, Bd, Qd, Rd)

Adc = [(Ad-Bd*Kd)];
Bdc = [Bd];
Cdc = [Cd];
Ddc = [Dd];

states = {'x' 'x_dot' 'phi' 'phi_dot', 'theta', 'theta_dot'};
inputs = {'r'};
outputs = {'x'; 'phi'; 'theta'};

sysd_cl = ss(Adc,Bdc,Cdc,Ddc,Ts,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
num = size(t);
num = num(2);
r =zeros(num, 2);
state_init = [0, 0, 5/180*pi, 0, 5/180*pi, 0];
[y,t,x]=lsim(sys_cl,r,t,state_init);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with LQR Control')

%电机pwm
ud = (-Kd * x')'; % 单轮所需力矩
motor_para = [-0.0049, 0.001587];
PWM_L = (ud(:,1) - motor_para(1) * x(:, 2)/R_w) / motor_para(2); % x(:, 2)用单轮当前线速度更准确
PWM_R = (ud(:,2) - motor_para(1) * x(:, 2)/R_w) / motor_para(2);