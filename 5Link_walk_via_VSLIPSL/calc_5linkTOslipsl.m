function [x_M, y_M, theta, r, dx_M, dy_M, dtheta, dr, time_elapsed] = calc_5linkTOslipsl(X, flag, param, slipslParams)

tic;

%% variables (absolute angles)
th1 = X(1);
th2 = X(1) + X(2);
th3 = X(1) + X(2) + X(3);
th4 = X(1) + X(2) + X(3) + X(4);
th5 = X(1) + X(2) + X(5);

dth1 = X(6);
dth2 = X(6) + X(7);
dth3 = X(6) + X(7) + X(8);
dth4 = X(6) + X(7) + X(8) + X(9);
dth5 = X(6) + X(7) + X(10);

q = [th1;th2;th3;th4;th5];
dq = [dth1;dth2;dth3;dth4;dth5];

%% Parameters

m1 = param(1);
m2 = param(2);
m3 = m2;
m4 = m1;
m5 = param(3);

l1 = param(4);
l2 = param(5);
l5 = param(6);
l3 = l2;
l4 = l1;

gravi = param(7);
I1 = param(8);
I2 = param(9);
I5 = param(10);
I3 = I1;
I4 = I2;

%% flags
f_SS = flag(1); % indicates which stance phase it is
foot = flag(2); % location of the stance foot
foot_prev = flag(3); % previous location of the stance foot

t_mode_change = flag(4); % time stamp when the mode change happens (TD or LO)

x_LO = flag(5); % lift off location of CoM when it happened

%%
% foot location is the origin
CoM = calculate_com(X, param, foot, [0;0;0;0;0]);
x_CoM = CoM(1) - foot;
y_CoM = CoM(2);

dx_CoM = CoM(3);
dy_CoM = CoM(4);

x_sw = l1*cos(th1) + l2*cos(th2) + l3*cos(th3) + l4*cos(th4);
y_sw = l1*sin(th1) + l2*sin(th2) + l3*sin(th3) + l4*sin(th4);

%%
% SLIP-SL constant params
m_M = slipslParams(1);
m_swLeg = slipslParams(2);
m_swFoot = slipslParams(3);
L_thigh = slipslParams(4);
I_swLeg = slipslParams(5);
I_swFoot = slipslParams(6);
gravi = slipslParams(7);

F_X = @(X) [
        x_CoM - (m_swFoot*(X(1) + cos(X(3))*(L_thigh + X(4))) + m_swLeg*(X(1) + (L_thigh*cos(X(3)))/2) + m_M*X(1))/(m_M + m_swLeg + m_swFoot);
        y_CoM - (m_swFoot*(X(2) + sin(X(3))*(L_thigh + X(4))) + m_swLeg*(X(2) + (L_thigh*sin(X(3)))/2) + m_M*X(2))/(m_M + m_swLeg + m_swFoot);
        x_sw - (X(1) + cos(X(3))*(L_thigh + X(4)));
        y_sw - (X(2) + sin(X(3))*(L_thigh + X(4)))];
    
X_slipsl = fsolve(F_X,[x_CoM,1.0479,4.5052,0.3708],optimoptions('fsolve','Algorithm', 'Levenberg-Marquardt'));

x_M = X_slipsl(1);
y_M = X_slipsl(2);
theta = X_slipsl(3);
r = X_slipsl(4);

% because I was lazy to change them in F_dx
theta_plus = theta;
r_plus = r;

F_dX = @(dX) [
        dx_CoM - (dX(1)*m_M + m_swLeg*(dX(1) - (L_thigh*dX(3)*sin(theta_plus))/2) + m_swFoot*(dX(1) + dX(4)*cos(theta_plus) - dX(3)*sin(theta_plus)*(L_thigh + r_plus)))/(m_M + m_swLeg + m_swFoot);
        dy_CoM - (dX(2)*m_M + m_swLeg*(dX(2) + (L_thigh*dX(3)*cos(theta_plus))/2) + m_swFoot*(dX(2) + dX(4)*sin(theta_plus) + dX(3)*cos(theta_plus)*(L_thigh + r_plus)))/(m_M + m_swLeg + m_swFoot);
        dX(1) + dX(4)*cos(theta_plus) - dX(3)*sin(theta_plus)*(L_thigh + r_plus);
        dX(2) + dX(4)*sin(theta_plus) + dX(3)*cos(theta_plus)*(L_thigh + r_plus)];
    
 dX_slipsl = fsolve(F_dX, [0.7156,0.1696,-0.6214,0.3132], optimoptions('fsolve','Algorithm', 'Levenberg-Marquardt'));
 
dx_M = dX_slipsl(1);
dy_M = dX_slipsl(2);
dtheta = dX_slipsl(3);
dr = dX_slipsl(4);

time_elapsed = toc;
end