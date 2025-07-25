%% 连续系统模型
M = 0.104; % 小车重（平衡小车轮子的重量52g*2：轮胎37.5g+联轴器13g+轮轴1.5g）
m = 0.650; % 摆杆重（平衡小车除轮子外的重量650g：总重量754g-两个轮子重量104g）
b = 0; % 小车摩擦系数：摩擦力/速度（对于平衡小车，摩擦力不是阻力，它对应倒立摆小车的推力，因此取b=0）
I = 0.000347; % 摆杆惯量（平衡小车除轮子外的部分相对于轮轴的惯量，按重心估算ml^2/3）
g = 9.8;
l = 0.04; % 摆杆重心到铰链的距离（平衡小车除轮子外的部分的重心到轮轴的距离，吊绳估计4cm）

p = I*(M+m)+M*m*l^2; %denominator for the A and B matrices

A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
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
Q(1,1) = 5000;
Q(3,3) = 100
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
Qd(1,1) = 20;
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
r =0.2*ones(size(t));
[y,t,x]=lsim(sysd_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with LQR Control')

%反算电机pwm
ud = (-Kd * x')';
acc = ud / M / 2; % 单轮加速度
motor_para_L = [-17.41, 0.25, -0.24];
motor_para_R = [-19.93, 0.26, -0.04];
PWM_L = (acc - motor_para_L(1) * x(:, 2) - motor_para_L(3)) / motor_para_L(2); % x(:, 2)用单轮当前线速度更准确
PWM_R = (acc - motor_para_R(1) * x(:, 2) - motor_para_R(3)) / motor_para_R(2);

%%-------------------------------------------------------------------------

%% 包含航向角的模型
M = 0.104; % 小车重（平衡小车轮子的重量52g*2：轮胎37.5g+联轴器13g+轮轴1.5g）
m = 0.650; % 摆杆重（平衡小车除轮子外的重量650g：总重量754g-两个轮子重量104g）
b = 0; % 小车摩擦系数：摩擦力/速度（对于平衡小车，摩擦力不是阻力，它对应倒立摆小车的推力，因此取b=0）
I = 0.000347; % 摆杆惯量（平衡小车除轮子外的部分相对于轮轴的惯量，按重心估算ml^2/3）
g = 9.8;
l = 0.04; % 摆杆重心到铰链的距离（平衡小车除轮子外的部分的重心到轮轴的距离，吊绳估计4cm）
m0 = m+M; % 平衡小车重量0.754kg
d = 0.12;   % 转动惯量测试参数0.12m
L = 0.465;  % 转动惯量测试参数0.465m
T = 57;     % 转动惯量测试参数57s
J = m0*g*d^2 / (16*pi^2*L) * (T/50)^2; % 平衡时的惯量（倾斜会变大），小角度倾斜时认为惯量近似不变
Lw = 0.17;  % 轮距0.17m 

T = 31;
mW = 0.0375;
Rw = 0.075/2;
d = 0.075;   % 转动惯量测试参数0.12m
L = 0.17;  % 转动惯量测试参数0.465m
g = 9.8;
IW = mW*g*d^2 / (16*pi^2*L) * (T/50)^2;
Iwr = IW*(1+1/0.06);

p = I*(M+m)+M*m*l^2; %denominator for the A and B matrices

A = [0      1              0           0         0          0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0         0          0;
     0      0              0           1         0          0;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0         0          0;
     0      0              0           0         0          1;
     0      0              0           0         0          0];
B = [     0       0;
     (I+m*l^2)/p  0;
          0       0;
        m*l/p     0;
          0       0;
          0       Lw/2/J];
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
Q(1,1) = 50;
Q(3,3) = 20;
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
r =0.2*ones(num, 2);
[y,t,x]=lsim(sys_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with LQR Control')
plot(t, y(:,3))
plot(t, x(:,6))

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
Qd(3,3) = 40;
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
r =0.2*ones(num, 2);
[y,t,x]=lsim(sysd_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with LQR Control')

%反算电机pwm
ud = (-Kd * x')';
acc_L = (ud(:,1) / 2 - ud(:,2)) / M; % 左轮加速度
acc_R = (ud(:,1) / 2 + ud(:,2)) / M; % 右轮加速度
motor_para_L = [-17.41, 0.25, -0.24];
motor_para_R = [-19.93, 0.26, -0.04];
PWM_L = (acc_L - motor_para_L(1) * x(:, 2) - motor_para_L(3)) / motor_para_L(2); % x(:, 2)用单轮当前线速度更准确
PWM_R = (acc_R - motor_para_R(1) * x(:, 2) - motor_para_R(3)) / motor_para_R(2);

F_L = (ud(:,1) / 2 - ud(:,2)); % 左轮力
F_R = (ud(:,1) / 2 + ud(:,2)); % 右轮力
motor_para_L = [-17.41, 0.25] * Iwr/Rw;
motor_para_R = [-19.93, 0.26] * Iwr/Rw;
PWM_L = (F_L - motor_para_L(1) * x(:, 2)/Rw) / motor_para_L(2); % x(:, 2)用单轮当前线速度更准确
PWM_R = (F_R - motor_para_R(1) * x(:, 2)/Rw) / motor_para_R(2);