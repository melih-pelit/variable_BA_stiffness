function [des_th, time_elapsed] = calc_desJointAngles(X_curr, des_traj, flag, param, flag_des)

% 2021.07.22 - Function to calculate desired joint angles from the desired
% CoM and swing foot positions

tic;

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
x1 = foot;

foot_prev = flag(3); % previous location of the stance foot

t_mode_change = flag(4); % time stamp when the mode change happens (TD or LO)

x_LO = flag(5); % lift off location of CoM when it happened

%%

% des_traj

x_CoM_des = des_traj(1);
y_CoM_des = des_traj(2);

dx_CoM_des = des_traj(5);
dy_CoM_des = des_traj(6);
dx_sw_ft_des = des_traj(7);
dy_sw_ft_des = des_traj(8);

% ddx_CoM_des = des_traj(9);
% ddy_CoM_des = des_traj(10);
% ddx_sw_ft_des = des_traj(11);
% ddy_sw_ft_des = des_traj(12);

if f_SS == 1
    % if the system is in single stance phase
    x_sw_ft_des = des_traj(3);
    y_sw_ft_des = des_traj(4);
else
    % if the system is in double stance phase
    if flag_des ~= 2
        x_sw_ft_des = foot_prev;
        y_sw_ft_des = 0;
        dx_sw_ft_des = 0;
        dy_sw_ft_des = 1.5; 
    else
        x_sw_ft_des = foot_prev;
        y_sw_ft_des = 0;
        dx_sw_ft_des = 0;
        dy_sw_ft_des = 0;               
    end
    
end


F_X = @(X) [
        x_CoM_des - ((m4*(x1 + (l4*cos(X(1) + X(2) + X(3) + X(4)))/2 + l2*cos(X(1) + X(2)) + l1*cos(X(1)) + l3*cos(X(1) + X(2) + X(3))) + m1*(x1 + (l1*cos(X(1)))/2) + m3*(x1 + l2*cos(X(1) + X(2)) + l1*cos(X(1)) + (l3*cos(X(1) + X(2) + X(3)))/2) + m5*(x1 + l2*cos(X(1) + X(2)) + l1*cos(X(1)) + (l5*cos(X(1) + X(2) + X(5)))/2) + m2*(x1 + (l2*cos(X(1) + X(2)))/2 + l1*cos(X(1))))/(m1 + m2 + m3 + m4 + m5));
        y_CoM_des - ((m3*(l2*sin(X(1) + X(2)) + l1*sin(X(1)) + (l3*sin(X(1) + X(2) + X(3)))/2) + m5*(l2*sin(X(1) + X(2)) + l1*sin(X(1)) + (l5*sin(X(1) + X(2) + X(5)))/2) + m2*((l2*sin(X(1) + X(2)))/2 + l1*sin(X(1))) + m4*((l4*sin(X(1) + X(2) + X(3) + X(4)))/2 + l2*sin(X(1) + X(2)) + l1*sin(X(1)) + l3*sin(X(1) + X(2) + X(3))) + (l1*m1*sin(X(1)))/2)/(m1 + m2 + m3 + m4 + m5));
        x_sw_ft_des - (x1 + l4*cos(X(1) + X(2) + X(3) + X(4)) + l2*cos(X(1) + X(2)) + l1*cos(X(1)) + l3*cos(X(1) + X(2) + X(3)));
        y_sw_ft_des - (l4*sin(X(1) + X(2) + X(3) + X(4)) + l2*sin(X(1) + X(2)) + l1*sin(X(1)) + l3*sin(X(1) + X(2) + X(3)));
        X(1) + X(2) + X(5) - pi/2];

X_des = fsolve(F_X,[X_curr(1),X_curr(2),X_curr(3),X_curr(4),X_curr(5)],optimoptions('fsolve','Algorithm', 'Levenberg-Marquardt'));

th1_des = wrapTo2Pi(X_des(1));
th2_des = wrapTo2Pi(X_des(2));
th3_des = wrapTo2Pi(X_des(3));
th4_des = wrapTo2Pi(X_des(4));
th5_des = wrapTo2Pi(X_des(5));

F_dX = @(dX) [
        dx_CoM_des - (- (dX(3)*(m4*((l4*sin(th1_des + th2_des + th3_des + th4_des))/2 + l3*sin(th1_des + th2_des + th3_des)) + (l3*m3*sin(th1_des + th2_des + th3_des))/2))/(m1 + m2 + m3 + m4 + m5) - (dX(1)*(m3*(l2*sin(th1_des + th2_des) + l1*sin(th1_des) + (l3*sin(th1_des + th2_des + th3_des))/2) + m5*(l2*sin(th1_des + th2_des) + l1*sin(th1_des) + (l5*sin(th1_des + th2_des + th5_des))/2) + m2*((l2*sin(th1_des + th2_des))/2 + l1*sin(th1_des)) + m4*((l4*sin(th1_des + th2_des + th3_des + th4_des))/2 + l2*sin(th1_des + th2_des) + l1*sin(th1_des) + l3*sin(th1_des + th2_des + th3_des)) + (l1*m1*sin(th1_des))/2))/(m1 + m2 + m3 + m4 + m5) - (dX(2)*(m4*((l4*sin(th1_des + th2_des + th3_des + th4_des))/2 + l2*sin(th1_des + th2_des) + l3*sin(th1_des + th2_des + th3_des)) + m3*(l2*sin(th1_des + th2_des) + (l3*sin(th1_des + th2_des + th3_des))/2) + m5*(l2*sin(th1_des + th2_des) + (l5*sin(th1_des + th2_des + th5_des))/2) + (l2*m2*sin(th1_des + th2_des))/2))/(m1 + m2 + m3 + m4 + m5) - (dX(4)*l4*m4*sin(th1_des + th2_des + th3_des + th4_des))/(2*(m1 + m2 + m3 + m4 + m5)) - (dX(5)*l5*m5*sin(th1_des + th2_des + th5_des))/(2*(m1 + m2 + m3 + m4 + m5)));
        dy_CoM_des - ((dX(1)*(m3*(l2*cos(th1_des + th2_des) + l1*cos(th1_des) + (l3*cos(th1_des + th2_des + th3_des))/2) + m5*(l2*cos(th1_des + th2_des) + l1*cos(th1_des) + (l5*cos(th1_des + th2_des + th5_des))/2) + m2*((l2*cos(th1_des + th2_des))/2 + l1*cos(th1_des)) + m4*((l4*cos(th1_des + th2_des + th3_des + th4_des))/2 + l2*cos(th1_des + th2_des) + l1*cos(th1_des) + l3*cos(th1_des + th2_des + th3_des)) + (l1*m1*cos(th1_des))/2))/(m1 + m2 + m3 + m4 + m5) + (dX(3)*(m4*((l4*cos(th1_des + th2_des + th3_des + th4_des))/2 + l3*cos(th1_des + th2_des + th3_des)) + (l3*m3*cos(th1_des + th2_des + th3_des))/2))/(m1 + m2 + m3 + m4 + m5) + (dX(2)*(m3*(l2*cos(th1_des + th2_des) + (l3*cos(th1_des + th2_des + th3_des))/2) + m5*(l2*cos(th1_des + th2_des) + (l5*cos(th1_des + th2_des + th5_des))/2) + m4*((l4*cos(th1_des + th2_des + th3_des + th4_des))/2 + l2*cos(th1_des + th2_des) + l3*cos(th1_des + th2_des + th3_des)) + (l2*m2*cos(th1_des + th2_des))/2))/(m1 + m2 + m3 + m4 + m5) + (dX(4)*l4*m4*cos(th1_des + th2_des + th3_des + th4_des))/(2*(m1 + m2 + m3 + m4 + m5)) + (dX(5)*l5*m5*cos(th1_des + th2_des + th5_des))/(2*(m1 + m2 + m3 + m4 + m5)));
        dx_sw_ft_des - (- dX(2)*(l4*sin(th1_des + th2_des + th3_des + th4_des) + l2*sin(th1_des + th2_des) + l3*sin(th1_des + th2_des + th3_des)) - dX(3)*(l4*sin(th1_des + th2_des + th3_des + th4_des) + l3*sin(th1_des + th2_des + th3_des)) - dX(1)*(l4*sin(th1_des + th2_des + th3_des + th4_des) + l2*sin(th1_des + th2_des) + l1*sin(th1_des) + l3*sin(th1_des + th2_des + th3_des)) - dX(4)*l4*sin(th1_des + th2_des + th3_des + th4_des));
        dy_sw_ft_des - (dX(3)*(l4*cos(th1_des + th2_des + th3_des + th4_des) + l3*cos(th1_des + th2_des + th3_des)) + dX(1)*(l4*cos(th1_des + th2_des + th3_des + th4_des) + l2*cos(th1_des + th2_des) + l1*cos(th1_des) + l3*cos(th1_des + th2_des + th3_des)) + dX(2)*(l4*cos(th1_des + th2_des + th3_des + th4_des) + l2*cos(th1_des + th2_des) + l3*cos(th1_des + th2_des + th3_des)) + dX(4)*l4*cos(th1_des + th2_des + th3_des + th4_des));
        dX(5)^2];

dX_des = fsolve(F_dX,[X_curr(6),X_curr(7),X_curr(8),X_curr(9),X_curr(10)],optimoptions('fsolve','Algorithm', 'Levenberg-Marquardt'));

des_th = [
    X_des(1); X_des(2); X_des(3); X_des(4); X_des(5); 
    dX_des(1); dX_des(2); dX_des(3); dX_des(4); dX_des(5)
    ];

time_elapsed = toc;
end