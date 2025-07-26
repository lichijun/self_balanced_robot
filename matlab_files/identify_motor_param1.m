load('pwm60_with_wheel.mat')
% load('pwm60_no_wheel.mat')

Rw = 0.075/2; % 轮子半径(m)
w_l_pwm60_raw = vel_l / Rw;
w_r_pwm60_raw = vel_r / Rw;

% smoothing
num = size(w_l_pwm60_raw);
index = (1:num(1))';
w_l_pwm60_filterred = fittedmodel_l(index);
plot([w_l_pwm60_raw, w_l_pwm60_filterred])
num = size(w_r_pwm60_raw);
index = (1:num(1))';
w_r_pwm60_filterred = fittedmodel_r(index);
plot([w_r_pwm60_raw, w_r_pwm60_filterred])

% cal acc
ts = 0.01;
w_dot_l_pwm60 = zeros(298,1);
w_dot_r_pwm60 = zeros(298,1);
for index = 3 : 300
    w_dot_l_pwm60(index - 2) = (w_l_pwm60_filterred(index) - w_l_pwm60_filterred(index-1)) / ts;
    w_dot_r_pwm60(index - 2) = (w_r_pwm60_filterred(index) - w_r_pwm60_filterred(index-1)) / ts;
end

w_l_pwm60 = w_l_pwm60_filterred(3:300);
w_r_pwm60 = w_r_pwm60_filterred(3:300);

% regress
pwm60 = 60 * ones(298,1);
y = w_dot_l_pwm60;
X = [w_l_pwm60, pwm60];
b_l = regress(y, X)
y = w_dot_r_pwm60;
X = [w_r_pwm60, pwm60];
b_r = regress(y, X)

%
coef1 = -0.003933;
coef2 = 0.13038;
Iwr = (coef1 * w_l_pwm60 + coef2 * 0.6 * ones(298,1)) ./ w_dot_l_pwm60;