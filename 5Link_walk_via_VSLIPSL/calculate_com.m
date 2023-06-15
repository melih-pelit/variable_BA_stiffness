function CoM = calculate_com(X, param, foot, ddq)


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

ddth1 = ddq(1);
ddth2 = ddq(2);
ddth3 = ddq(3);
ddth4 = ddq(4);
ddth5 = ddq(5);

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

%% Calculation CoM position
x1 = foot; % x position of the stance foot
% x_g CoM's of respective links
x_G = zeros(5);
x_G(1) = x1 + (l1/2)*cos(th1); 
x_G(2) = x1 + l1*cos(th1) + (l2/2)*cos(th1 + th2);
x_G(3) = x1 + l1*cos(th1) + l2*cos(th1 + th2) + (l3/2)*cos(th1 + th2 + th3);
x_G(4) = x1 + l1*cos(th1) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3) + (l4/2)*cos(th1 + th2 + th3 + th4);
x_G(5) = x1 + l1*cos(th1) + l2*cos(th1 + th2) + (l5/2)*cos(th1 + th2 + th5); % body

% z_g CoM's of respective links
z_G = zeros(5);
z_G(1) = (l1/2)*sin(th1); 
z_G(2) = l1*sin(th1) + (l2/2)*sin(th1 + th2);
z_G(3) = l1*sin(th1) + l2*sin(th1 + th2) + (l3/2)*sin(th1 + th2 + th3);
z_G(4) = l1*sin(th1) + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3) + (l4/2)*sin(th1 + th2 + th3 + th4);
z_G(5) = l1*sin(th1) + l2*sin(th1 + th2) + (l5/2)*sin(th1 + th2 + th5);

xG = (m1*x_G(1) + m2*x_G(2) + m3*x_G(3) + m4*x_G(4) + m5*x_G(5))/(m1 + m2 + m3 + m4 + m5);
zG = (m1*z_G(1) + m2*z_G(2) + m3*z_G(3) + m4*z_G(4) + m5*z_G(5))/(m1 + m2 + m3 + m4 + m5);


%% Calculation of CoM velocity

% dx_G: CoM velocities in x direction
dx_G = zeros(5);
dx_G(1) = -(l1/2)*dth1*sin(th1);
dx_G(2) = -l1*dth1*sin(th1) - (l2/2)*(dth1 + dth2)*sin(th1 + th2);
dx_G(3) = -l1*dth1*sin(th1) - l2*(dth1 + dth2)*sin(th1 + th2) - (l3/2)*(dth1 + dth2 + dth3)*sin(th1 + th2 + th3);
dx_G(4) = -l1*dth1*sin(th1) - l2*(dth1 + dth2)*sin(th1 + th2) - l3*(dth1 + dth2 + dth3)*sin(th1 + th2 + th3) - (l4/2)*(dth1 + dth2 + dth3 + dth4)*sin(th1 + th2 + th3 + th4);
dx_G(5) = -l1*dth1*sin(th1) - l2*(dth1 + dth2)*sin(th1 + th2) - (l5/2)*(dth1 + dth2 + dth5)*sin(th1 + th2 + th5);

% dy_g: CoM velocities in y direction
dz_G = zeros(5);
dz_G(1) = (l1/2)*dth1*cos(th1);
dz_G(2) = l1*dth1*cos(th1) + (l2/2)*(dth1 + dth2)*cos(th1 + th2);
dz_G(3) = l1*dth1*cos(th1) + l2*(dth1 + dth2)*cos(th1 + th2) + (l3/2)*(dth1 + dth2 + dth3)*cos(th1 + th2 + th3);
dz_G(4) = l1*dth1*cos(th1) + l2*(dth1 + dth2)*cos(th1 + th2) + l3*(dth1 + dth2 + dth3)*cos(th1 + th2 + th3) + (l4/2)*(dth1 + dth2 + dth3 + dth4)*cos(th1 + th2 + th3 + th4);
dz_G(5) = l1*dth1*cos(th1) + l2*(dth1 + dth2)*cos(th1 + th2) + (l5/2)*(dth1 + dth2 + dth5)*cos(th1 + th2 + th5);

dxG = (m1*dx_G(1) + m2*dx_G(2) + m3*dx_G(3) + m4*dx_G(4) + m5*dx_G(5))/(m1 + m2 + m3 + m4 + m5);
dzG = (m1*dz_G(1) + m2*dz_G(2) + m3*dz_G(3) + m4*dz_G(4) + m5*dz_G(5))/(m1 + m2 + m3 + m4 + m5);

%% CoM Acceleration

ddxG = - dth4*((dth1*l4*m4*cos(th1 + th2 + th3 + th4))/(2*(m1 + m2 + m3 + m4 + m5)) + (dth2*l4*m4*cos(th1 + th2 + th3 + th4))/(2*(m1 + m2 + m3 + m4 + m5)) + (dth3*l4*m4*cos(th1 + th2 + th3 + th4))/(2*(m1 + m2 + m3 + m4 + m5)) + (dth4*l4*m4*cos(th1 + th2 + th3 + th4))/(2*(m1 + m2 + m3 + m4 + m5))) - dth2*((dth3*(m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l3*cos(th1 + th2 + th3)) + (l3*m3*cos(th1 + th2 + th3))/2))/(m1 + m2 + m3 + m4 + m5) + (dth1*(m3*(l2*cos(th1 + th2) + (l3*cos(th1 + th2 + th3))/2) + m5*(l2*cos(th1 + th2) + (l5*cos(th1 + th2 + th5))/2) + m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3)) + (l2*m2*cos(th1 + th2))/2))/(m1 + m2 + m3 + m4 + m5) + (dth2*(m3*(l2*cos(th1 + th2) + (l3*cos(th1 + th2 + th3))/2) + m5*(l2*cos(th1 + th2) + (l5*cos(th1 + th2 + th5))/2) + m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3)) + (l2*m2*cos(th1 + th2))/2))/(m1 + m2 + m3 + m4 + m5) + (dth4*l4*m4*cos(th1 + th2 + th3 + th4))/(2*(m1 + m2 + m3 + m4 + m5)) + (dth5*l5*m5*cos(th1 + th2 + th5))/(2*(m1 + m2 + m3 + m4 + m5))) - dth3*((dth1*(m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l3*cos(th1 + th2 + th3)) + (l3*m3*cos(th1 + th2 + th3))/2))/(m1 + m2 + m3 + m4 + m5) + (dth2*(m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l3*cos(th1 + th2 + th3)) + (l3*m3*cos(th1 + th2 + th3))/2))/(m1 + m2 + m3 + m4 + m5) + (dth3*(m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l3*cos(th1 + th2 + th3)) + (l3*m3*cos(th1 + th2 + th3))/2))/(m1 + m2 + m3 + m4 + m5) + (dth4*l4*m4*cos(th1 + th2 + th3 + th4))/(2*(m1 + m2 + m3 + m4 + m5))) - dth1*((dth1*(m3*(l2*cos(th1 + th2) + l1*cos(th1) + (l3*cos(th1 + th2 + th3))/2) + m5*(l2*cos(th1 + th2) + l1*cos(th1) + (l5*cos(th1 + th2 + th5))/2) + m2*((l2*cos(th1 + th2))/2 + l1*cos(th1)) + m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l2*cos(th1 + th2) + l1*cos(th1) + l3*cos(th1 + th2 + th3)) + (l1*m1*cos(th1))/2))/(m1 + m2 + m3 + m4 + m5) + (dth3*(m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l3*cos(th1 + th2 + th3)) + (l3*m3*cos(th1 + th2 + th3))/2))/(m1 + m2 + m3 + m4 + m5) + (dth2*(m3*(l2*cos(th1 + th2) + (l3*cos(th1 + th2 + th3))/2) + m5*(l2*cos(th1 + th2) + (l5*cos(th1 + th2 + th5))/2) + m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3)) + (l2*m2*cos(th1 + th2))/2))/(m1 + m2 + m3 + m4 + m5) + (dth4*l4*m4*cos(th1 + th2 + th3 + th4))/(2*(m1 + m2 + m3 + m4 + m5)) + (dth5*l5*m5*cos(th1 + th2 + th5))/(2*(m1 + m2 + m3 + m4 + m5))) - dth5*((dth1*l5*m5*cos(th1 + th2 + th5))/(2*(m1 + m2 + m3 + m4 + m5)) + (dth2*l5*m5*cos(th1 + th2 + th5))/(2*(m1 + m2 + m3 + m4 + m5)) + (dth5*l5*m5*cos(th1 + th2 + th5))/(2*(m1 + m2 + m3 + m4 + m5))) - (ddth3*(m4*((l4*sin(th1 + th2 + th3 + th4))/2 + l3*sin(th1 + th2 + th3)) + (l3*m3*sin(th1 + th2 + th3))/2))/(m1 + m2 + m3 + m4 + m5) - (ddth1*(m3*(l2*sin(th1 + th2) + l1*sin(th1) + (l3*sin(th1 + th2 + th3))/2) + m5*(l2*sin(th1 + th2) + l1*sin(th1) + (l5*sin(th1 + th2 + th5))/2) + m2*((l2*sin(th1 + th2))/2 + l1*sin(th1)) + m4*((l4*sin(th1 + th2 + th3 + th4))/2 + l2*sin(th1 + th2) + l1*sin(th1) + l3*sin(th1 + th2 + th3)) + (l1*m1*sin(th1))/2))/(m1 + m2 + m3 + m4 + m5) - (ddth2*(m4*((l4*sin(th1 + th2 + th3 + th4))/2 + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3)) + m3*(l2*sin(th1 + th2) + (l3*sin(th1 + th2 + th3))/2) + m5*(l2*sin(th1 + th2) + (l5*sin(th1 + th2 + th5))/2) + (l2*m2*sin(th1 + th2))/2))/(m1 + m2 + m3 + m4 + m5) - (ddth4*l4*m4*sin(th1 + th2 + th3 + th4))/(2*(m1 + m2 + m3 + m4 + m5)) - (ddth5*l5*m5*sin(th1 + th2 + th5))/(2*(m1 + m2 + m3 + m4 + m5));
ddzG = (ddth1*(m3*(l2*cos(th1 + th2) + l1*cos(th1) + (l3*cos(th1 + th2 + th3))/2) + m5*(l2*cos(th1 + th2) + l1*cos(th1) + (l5*cos(th1 + th2 + th5))/2) + m2*((l2*cos(th1 + th2))/2 + l1*cos(th1)) + m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l2*cos(th1 + th2) + l1*cos(th1) + l3*cos(th1 + th2 + th3)) + (l1*m1*cos(th1))/2))/(m1 + m2 + m3 + m4 + m5) - dth3*((dth1*(m4*((l4*sin(th1 + th2 + th3 + th4))/2 + l3*sin(th1 + th2 + th3)) + (l3*m3*sin(th1 + th2 + th3))/2))/(m1 + m2 + m3 + m4 + m5) + (dth2*(m4*((l4*sin(th1 + th2 + th3 + th4))/2 + l3*sin(th1 + th2 + th3)) + (l3*m3*sin(th1 + th2 + th3))/2))/(m1 + m2 + m3 + m4 + m5) + (dth3*(m4*((l4*sin(th1 + th2 + th3 + th4))/2 + l3*sin(th1 + th2 + th3)) + (l3*m3*sin(th1 + th2 + th3))/2))/(m1 + m2 + m3 + m4 + m5) + (dth4*l4*m4*sin(th1 + th2 + th3 + th4))/(2*(m1 + m2 + m3 + m4 + m5))) - dth1*((dth3*(m4*((l4*sin(th1 + th2 + th3 + th4))/2 + l3*sin(th1 + th2 + th3)) + (l3*m3*sin(th1 + th2 + th3))/2))/(m1 + m2 + m3 + m4 + m5) + (dth1*(m3*(l2*sin(th1 + th2) + l1*sin(th1) + (l3*sin(th1 + th2 + th3))/2) + m5*(l2*sin(th1 + th2) + l1*sin(th1) + (l5*sin(th1 + th2 + th5))/2) + m2*((l2*sin(th1 + th2))/2 + l1*sin(th1)) + m4*((l4*sin(th1 + th2 + th3 + th4))/2 + l2*sin(th1 + th2) + l1*sin(th1) + l3*sin(th1 + th2 + th3)) + (l1*m1*sin(th1))/2))/(m1 + m2 + m3 + m4 + m5) + (dth2*(m4*((l4*sin(th1 + th2 + th3 + th4))/2 + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3)) + m3*(l2*sin(th1 + th2) + (l3*sin(th1 + th2 + th3))/2) + m5*(l2*sin(th1 + th2) + (l5*sin(th1 + th2 + th5))/2) + (l2*m2*sin(th1 + th2))/2))/(m1 + m2 + m3 + m4 + m5) + (dth4*l4*m4*sin(th1 + th2 + th3 + th4))/(2*(m1 + m2 + m3 + m4 + m5)) + (dth5*l5*m5*sin(th1 + th2 + th5))/(2*(m1 + m2 + m3 + m4 + m5))) - dth4*((dth1*l4*m4*sin(th1 + th2 + th3 + th4))/(2*(m1 + m2 + m3 + m4 + m5)) + (dth2*l4*m4*sin(th1 + th2 + th3 + th4))/(2*(m1 + m2 + m3 + m4 + m5)) + (dth3*l4*m4*sin(th1 + th2 + th3 + th4))/(2*(m1 + m2 + m3 + m4 + m5)) + (dth4*l4*m4*sin(th1 + th2 + th3 + th4))/(2*(m1 + m2 + m3 + m4 + m5))) - dth2*((dth3*(m4*((l4*sin(th1 + th2 + th3 + th4))/2 + l3*sin(th1 + th2 + th3)) + (l3*m3*sin(th1 + th2 + th3))/2))/(m1 + m2 + m3 + m4 + m5) + (dth1*(m4*((l4*sin(th1 + th2 + th3 + th4))/2 + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3)) + m3*(l2*sin(th1 + th2) + (l3*sin(th1 + th2 + th3))/2) + m5*(l2*sin(th1 + th2) + (l5*sin(th1 + th2 + th5))/2) + (l2*m2*sin(th1 + th2))/2))/(m1 + m2 + m3 + m4 + m5) + (dth2*(m4*((l4*sin(th1 + th2 + th3 + th4))/2 + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3)) + m3*(l2*sin(th1 + th2) + (l3*sin(th1 + th2 + th3))/2) + m5*(l2*sin(th1 + th2) + (l5*sin(th1 + th2 + th5))/2) + (l2*m2*sin(th1 + th2))/2))/(m1 + m2 + m3 + m4 + m5) + (dth4*l4*m4*sin(th1 + th2 + th3 + th4))/(2*(m1 + m2 + m3 + m4 + m5)) + (dth5*l5*m5*sin(th1 + th2 + th5))/(2*(m1 + m2 + m3 + m4 + m5))) - dth5*((dth1*l5*m5*sin(th1 + th2 + th5))/(2*(m1 + m2 + m3 + m4 + m5)) + (dth2*l5*m5*sin(th1 + th2 + th5))/(2*(m1 + m2 + m3 + m4 + m5)) + (dth5*l5*m5*sin(th1 + th2 + th5))/(2*(m1 + m2 + m3 + m4 + m5))) + (ddth3*(m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l3*cos(th1 + th2 + th3)) + (l3*m3*cos(th1 + th2 + th3))/2))/(m1 + m2 + m3 + m4 + m5) + (ddth2*(m3*(l2*cos(th1 + th2) + (l3*cos(th1 + th2 + th3))/2) + m5*(l2*cos(th1 + th2) + (l5*cos(th1 + th2 + th5))/2) + m4*((l4*cos(th1 + th2 + th3 + th4))/2 + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3)) + (l2*m2*cos(th1 + th2))/2))/(m1 + m2 + m3 + m4 + m5) + (ddth4*l4*m4*cos(th1 + th2 + th3 + th4))/(2*(m1 + m2 + m3 + m4 + m5)) + (ddth5*l5*m5*cos(th1 + th2 + th5))/(2*(m1 + m2 + m3 + m4 + m5));


CoM = [xG, zG, dxG, dzG, ddxG, ddzG];
end