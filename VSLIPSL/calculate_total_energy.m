function total_E = calculate_total_energy(X, param)
%% defining variables

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

%% parameters

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

m = zeros(5);
m(1) = m1;
m(2) = m2;
m(3) = m3;
m(4) = m4;
m(5) = m5;

I = zeros(5);
I(1) = I1;
I(2) = I2;
I(3) = I3;
I(4) = I4;
I(5) = I5;

%% Calculations

% z_g CoM's of respective links
z_G = zeros(5);
z_G(1) = (l1/2)*sin(th1); 
z_G(2) = l1*sin(th1) + (l2/2)*sin(th1 + th2);
z_G(3) = l1*sin(th1) + l2*sin(th1 + th2) + (l3/2)*sin(th1 + th2 + th3);
z_G(4) = l1*sin(th1) + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3) + (l4/2)*sin(th1 + th2 + th3 + th4);
z_G(5) = l1*sin(th1) + l2*sin(th1 + th2) + (l5/2)*sin(th1 + th2 + th5);

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

% total rotations of links
rot = zeros(5);
rot(1) = dth1;
rot(2) = dth1 + dth2;
rot(3) = dth1 + dth2 + dth3;
rot(4) = dth1 + dth2 + dth3 + dth4;
rot(5) = dth1 + dth2 + dth5;

%% Potential Energy
E_pot = zeros(5);
E_kin = zeros(5);
E_rot = zeros(5);
for i = 1:1:5
    E_pot(i) = m(i)*gravi*z_G(i);
    E_kin(i) = 0.5*m(i)*(dx_G(i)^2 + dz_G(i)^2);
    E_rot(i) = 0.5*I(i)*rot(i)^2;
end
E = E_pot + E_kin + E_rot; % vector that stores total energy of each link
total_E = E(1) + E(2) + E(3) + E(4) + E(5);
end