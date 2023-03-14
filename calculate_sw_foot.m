function [x_sw, y_sw, dx_sw, dy_sw] = calculate_sw_foot(X, param, flag)

%% variables
th1 = X(1);
th2 = X(2);
th3 = X(3);
th4 = X(4);
th5 = X(5);

dth1 = X(6);
dth2 = X(7);
dth3 = X(8);
dth4 = X(9);
dth5 = X(10);

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

%%
x_st = flag(2);
y_st = flag(7);

%%
x_sw = x_st + l1*cos(th1) + l2*cos(th1 + th2) + ...
    l3*cos(th1 + th2 + th3) + l4*cos(th1 + th2 + th3 + th4);

y_sw = y_st + l1*sin(th1) + l2*sin(th1 + th2) + ...
    l3*sin(th1 + th2 + th3) + l4*sin(th1 + th2 + th3 + th4);

dx_sw = -l1*sin(th1)*dth1 - l2*sin(th1 + th2)*(dth1 + dth2) - ...
    l3*sin(th1 + th2 + th3)*(dth1 + dth2 + dth3) - ...
    l4*sin(th1 + th2 + th3 + th4)*(dth1 + dth2);

dy_sw = l1*cos(th1)*(dth1) + l2*cos(th1 + th2)*(dth1 + dth2)  + ...
    l3*cos(th1 + th2 + th3)*(dth1 + dth2 + dth3) + ...
    l4*cos(th1 + th2 + th3 + th4)*(dth1 + dth2 + dth3 + dth4);

end