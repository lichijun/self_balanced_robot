load('motor_param_regress_data.mat')

% smoothing
dataToFit = vel_r;
num = size(dataToFit);
index = (1:num(1))';
dataPred = fittedmodel(index);
plot([dataToFit, dataPred])

% cal acc
ts = 0.01;
acc_l_pwm60_no_wheel = zeros(298,1);
for index = 3 : 300
    acc_l_pwm60_no_wheel(index - 2) = (vel_l_pwm60_no_wheel(index) - vel_l_pwm60_no_wheel(index-1)) / ts;
end

vel_r_pwm20_num298 = vel_r_pwm20(3:300);
vel_r_pwm40_num298 = vel_r_pwm40(3:300);
vel_r_pwm60_num298 = vel_r_pwm60(3:300);
vel_r_pwm80_num298 = vel_r_pwm80(3:300);

% regress
y = [acc_l_pwm20; acc_l_pwm40; acc_l_pwm60; acc_l_pwm80];
X = [vel_l_pwm20_num298, pwm20, ones(298,1);
     vel_l_pwm40_num298, pwm40, ones(298,1);
     vel_l_pwm60_num298, pwm60, ones(298,1);
     vel_l_pwm80_num298, pwm80, ones(298,1)];
regress(y, X)

y = [acc_r_pwm20; acc_r_pwm40; acc_r_pwm60; acc_r_pwm80];
X = [vel_r_pwm20_num298, pwm20, ones(298,1);
     vel_r_pwm40_num298, pwm40, ones(298,1);
     vel_r_pwm60_num298, pwm60, ones(298,1);
     vel_r_pwm80_num298, pwm80, ones(298,1)];
regress(y, X)

%%-------------------------------------------------------------------

% smoothing
dataToFit = vel_l;
num = size(dataToFit);
index = (1:num(1))';
dataPred = fittedmodel(index);
plot([dataToFit, dataPred])

% cal acc
ts = 0.01;
acc_l_pwm60_no_wheel = zeros(298,1);
for index = 3 : 300
    acc_l_pwm60_no_wheel(index - 2) = (vel_l_pwm60_no_wheel(index) - vel_l_pwm60_no_wheel(index-1)) / ts;
end

vel_l_pwm60_no_wheel_num298 = vel_l_pwm60_no_wheel(3:300);

% regress
y = [acc_l_pwm60_no_wheel];
X = [vel_l_pwm60_no_wheel_num298, pwm60];
[b1,~,~,~,stats] = regress(y, X);

y = [acc_l_pwm60];
X = [vel_l_pwm60_num298, pwm60];
[b2,~,~,~,stats] = regress(y, X);

b1./b2