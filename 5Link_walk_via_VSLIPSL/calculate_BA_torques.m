function [tau_st, tau_sw, delta_l_st, delta_l_sw] = calculate_BA_torques( ...
    X_5Link, param_5Link, varargin)
% calculates BA torques [Nm] and spring deflections [m]

%% Handle optional variable BA stiffness input
numArgs = nargin;
if numArgs == 3
    U_varStiff_BA = varargin{1};
    u_st = U_varStiff_BA(1);
    u_sw = U_varStiff_BA(2);
elseif numArgs > 3
    display("Error, too many inputs")
    return
else
    u_st = 0;
    u_sw = 0;
end


%% variables
th1 = X_5Link(1);
th2 = X_5Link(2);
th3 = X_5Link(3);
th4 = X_5Link(4);
th5 = X_5Link(5);

dth1 = X_5Link(6);
dth2 = X_5Link(7);
dth3 = X_5Link(8);
dth4 = X_5Link(9);
dth5 = X_5Link(10);

q = [th1;th2;th3;th4;th5];
dq = [dth1;dth2;dth3;dth4;dth5];

%% parameters

m1 = param_5Link(1);
m2 = param_5Link(2);
m3 = m2;
m4 = m1;
m5 = param_5Link(3);

l1 = param_5Link(4);
l2 = param_5Link(5);
l5 = param_5Link(6);
l3 = l2;
l4 = l1;

gravi = param_5Link(7);
I1 = param_5Link(8);
I2 = param_5Link(9);
I5 = param_5Link(10);
I3 = I1;
I4 = I2;

r_k = param_5Link(11);
r_h = param_5Link(12);
k_ba = param_5Link(13);
phi_h0 = param_5Link(14);
phi_k0 = param_5Link(15);

%% Biarticular Spring Force

% Calculating the knee and hip angles
% We meed wrapTo2Pi() because some angles increase to inf due to the reset
% function eg. th1 and th5

phi_h_st = mod(th5 - pi, 2*pi); % angle between torso and stance thigh
phi_k_st = mod(pi - th2, 2*pi); % angle between stance thigh and shank
phi_h_sw = mod(th5 - th3, 2*pi); % angle between torso and swing thigh
phi_k_sw = mod(th4 - pi, 2*pi); % angle between swing thigh and shank

% Calculating the elongation in the springs
delta_l_st = r_h*(phi_h_st - phi_h0) - r_k*(phi_k_st - phi_k0); % elongation of BA in stance leg [m]
delta_l_sw = r_h*(phi_h_sw - phi_h0) - r_k*(phi_k_sw - phi_k0); % elongation of BA in swing leg [m]

% torques due to BA in stance leg
tau_st = [
    0;
    -(k_ba + u_st)*delta_l_st*r_k;
    0;
    0;
    -(k_ba + u_st)*delta_l_st*r_h];

% torques due to BA in swing leg
tau_sw = [
    0;
    0;
    (k_ba + u_sw)*delta_l_sw*r_h;
    (k_ba + u_sw)*delta_l_sw*r_k;
    -(k_ba + u_sw)*delta_l_sw*r_h];

end