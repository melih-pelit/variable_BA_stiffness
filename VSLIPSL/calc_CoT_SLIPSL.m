function CoT = calc_CoT_SLIPSL(sol)

% 2020.11.11 - function to calculate Cost of Transport

L0_ss = mean(sol{1}.parameters.L0.value);
L0_ds = mean(sol{2}.parameters.L0.value);
% L0 = 1; % [m]
m_M = mean(sol{1}.parameters.m_M.value); % hip mass [kg]
m_swLeg = mean(sol{1}.parameters.m_swLeg.value); % [kg]
m_swFoot = mean(sol{1}.parameters.m_swFoot.value); % [kg]
L_thigh = mean(sol{1}.parameters.L_thigh.value); % [m]
I_swLeg = mean(sol{1}.parameters.I_swLeg.value); %
I_swFoot = mean(sol{1}.parameters.I_swFoot.value); %; %
gravi = mean(sol{1}.parameters.gravi.value); % gravitational acc

k0_ss = mean(sol{1}.parameters.k0.value); % nominal leg stiffness [N/m]
k0_ds = mean(sol{2}.parameters.k0.value); % nominal leg stiffness [N/m]

k_swFoot = mean(sol{1}.parameters.k_swFoot.value);
k_swLeg = mean(sol{1}.parameters.k_swLeg.value);
theta0 = mean(sol{1}.parameters.theta0.value);
r0 = mean(sol{1}.parameters.r0.value);
foot = mean(sol{1}.parameters.foot.value);
footPlus = mean(sol{1}.parameters.footPlus.value);

% calculating the spring forces and power in the single stance phase

x_M = sol{1}.states.x_M.value;
y_M = sol{1}.states.y_M.value;
theta = sol{1}.states.theta.value;
r = sol{1}.states.r.value;

dx_M = sol{1}.states.dx_M.value;
dy_M = sol{1}.states.dy_M.value;
dtheta = sol{1}.states.dtheta.value;
dr = sol{1}.states.dr.value;

L_stLeg = sqrt((x_M - foot).^2 + y_M.^2);
F_st_ss = k0_ss*(L0_ss - L_stLeg);
dL_stLeg_ss = (1/2)*(((x_M - foot).^2 + y_M.^2)).^(-1/2).*(2*(x_M - foot).*dx_M + 2.*y_M.*dy_M);
P_st_ss = abs(F_st_ss.*dL_stLeg_ss);

F_swLeg = k_swLeg*(theta0 - theta);
P_swLeg = abs(F_swLeg.*dtheta);

F_swFoot = k_swFoot*(r0 - r);
P_swFoot = abs(F_swFoot.*dr);

energy_ss = trapz(sol{1}.states.time.value, P_st_ss + P_swLeg + P_swFoot);

% calculating the spring forces and power in the double stance phase

x_CoM = sol{2}.states.x_CoM.value;
y_CoM = sol{2}.states.y_CoM.value;

dx_CoM = sol{2}.states.dx_CoM.value;
dy_CoM = sol{2}.states.dy_CoM.value;

% Power of stance leg P_st = F_st*dL_swLeg
L_stLeg_ds = sqrt((x_CoM - footPlus).^2 + y_CoM.^2);
F_st_ds = k0_ds*(L0_ds - L_stLeg_ds);
dL_stLeg_ds = (1/2)*(((x_CoM - footPlus).^2 + y_CoM.^2)).^(-1/2).*(2.*(x_CoM - footPlus).*dx_CoM + 2.*y_CoM.*dy_CoM);
P_st_ds = abs(F_st_ds.*dL_stLeg_ds);

L_swLeg = sqrt((x_CoM - foot).^2 + y_CoM.^2);
F_sw = k0_ds*(L0_ds - L_swLeg);
dL_swLeg = (1/2).*(((x_CoM - foot).^2 + y_CoM.^2)).^(-1/2).*(2*(x_CoM - foot).*dx_CoM + 2*y_CoM.*dy_CoM);
P_sw = abs(F_sw.*dL_swLeg);

energy_ds = trapz(sol{2}.states.time.value, P_st_ds + P_sw);

% calculating the length of the complete stride
x_CoM_ss_I = (m_swLeg*(x_M(1) + (L_thigh*cos(theta(1)))/2) + m_M*x_M(1) + m_swFoot*(x_M(1) + cos(theta(1))*(L_thigh + r(1))))/(m_M + m_swLeg + m_swFoot);
d = x_CoM(end) - x_CoM_ss_I; % length of the stride

CoT = (energy_ss + energy_ds)/((m_M + m_swLeg + m_swFoot)*gravi*d);

% figure
% plot([F_st_ss.*dL_stLeg_ss + F_swLeg.*dtheta + F_swFoot.*dr, F_st_ds.*dL_stLeg_ds + F_sw.*dL_swLeg])
% grid on

end