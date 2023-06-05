% 2019.01.06 Deriving the acceleration of CoM
clear 
clc

syms th1 th2 th3 th4 th5 x1
syms dth1 dth2 dth3 dth4 dth5
syms ddth1 ddth2 ddth3 ddth4 ddth5
syms l1 l2 l3 l4 l5
syms m1 m2 m3 m4 m5

q = [th1; th2; th3; th4; th5];
dq = [dth1; dth2; dth3; dth4; dth5];
ddq = [ddth1; ddth2; ddth3; ddth4; ddth5];

% x_g CoM's of respective links
x_G(1) = x1 + (l1/2)*cos(th1); 
x_G(2) = x1 + l1*cos(th1) + (l2/2)*cos(th1 + th2);
x_G(3) = x1 + l1*cos(th1) + l2*cos(th1 + th2) + (l3/2)*cos(th1 + th2 + th3);
x_G(4) = x1 + l1*cos(th1) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3) + (l4/2)*cos(th1 + th2 + th3 + th4);
x_G(5) = x1 + l1*cos(th1) + l2*cos(th1 + th2) + (l5/2)*cos(th1 + th2 + th5); % body

% z_g CoM's of respective links
z_G(1) = (l1/2)*sin(th1); 
z_G(2) = l1*sin(th1) + (l2/2)*sin(th1 + th2);
z_G(3) = l1*sin(th1) + l2*sin(th1 + th2) + (l3/2)*sin(th1 + th2 + th3);
z_G(4) = l1*sin(th1) + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3) + (l4/2)*sin(th1 + th2 + th3 + th4);
z_G(5) = l1*sin(th1) + l2*sin(th1 + th2) + (l5/2)*sin(th1 + th2 + th5);

xG = simplify((m1*x_G(1) + m2*x_G(2) + m3*x_G(3) + m4*x_G(4) + m5*x_G(5))/(m1 + m2 + m3 + m4 + m5));
zG = simplify((m1*z_G(1) + m2*z_G(2) + m3*z_G(3) + m4*z_G(4) + m5*z_G(5))/(m1 + m2 + m3 + m4 + m5));

%% CoM velocity

dxG = jacobian(xG, q)*dq;
dzG = jacobian(zG, q)*dq;

%% CoM Acceleration

ddxG = simplify(jacobian(dxG, q)*dq + jacobian(dxG, dq)*ddq);
ddzG = simplify(jacobian(dzG, q)*dq + jacobian(dzG, dq)*ddq);

%% swing foot location
x_sw = x1 + l1*cos(th1) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3) + (l4)*cos(th1 + th2 + th3 + th4);
y_sw = l1*sin(th1) + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3) + (l4)*sin(th1 + th2 + th3 + th4);

dx_sw = jacobian(x_sw, q)*dq;
dy_sw = jacobian(y_sw, q)*dq;

ddx_sw = simplify(jacobian(dx_sw, q)*dq + jacobian(dx_sw, dq)*ddq);
ddy_sw = simplify(jacobian(dy_sw, q)*dq + jacobian(dy_sw, dq)*ddq);