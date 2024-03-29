%% five_link_walking_sim
tic
clear
close all

addpath("..\misc_resources\altmany-export_fig-d8b9f4a")
addpath("..\VSLIPSL")
%% SLIP Data

% 2021.09.01: new SLIP-SL trajectory with low CoT (CoT = 0.1907)
load('SLIPSL_Data\dc_comp2021-09-01-16-06') % SLIPSLOpenOCL2021-09-01-15-36 (@C:\Matlab Workspace\SLIPSL_OpenOCL\202108111747 - SLIPSL_OpenOCL (True CoM))

dc = calc_slipsl_traj(dc); % function to calculate CoM traj. at SS phase (appends to dc)

slipsl.time_ss = dc.time_ss';
slipsl.x_ss = dc.ss_traj.x_CoM_ss;
slipsl.y_ss = dc.ss_traj.y_CoM_ss;
slipsl.dx_ss = dc.ss_traj.dx_CoM_ss;
slipsl.dy_ss = dc.ss_traj.dy_CoM_ss;

slipsl.time_ds = dc.time_ds';
slipsl.x_ds = dc.simout_ds(1,:)';
slipsl.y_ds = dc.simout_ds(2,:)';
slipsl.dx_ds = dc.simout_ds(3,:)';
slipsl.dy_ds = dc.simout_ds(4,:)';

swFootSLIPSL.x_CoM = slipsl.x_ss;
swFootSLIPSL.x = dc.ss_traj.x_sw;
swFootSLIPSL.z = dc.ss_traj.y_sw;
swFootSLIPSL.dx = dc.ss_traj.dx_sw;
swFootSLIPSL.dz = dc.ss_traj.dy_sw;

var_stiff_bound = 1; % limiting constant for var. stiffness

slipslParams = [
    dc.col_param.L0_ss; 
    dc.col_param.k0_ss; 
    dc.const_param.m_M; 
    dc.const_param.m_swLeg; 
    dc.const_param.m_swFoot; 
    dc.const_param.I_swLeg; 
    dc.const_param.I_swFoot; 
    dc.const_param.L_thigh;
    dc.col_param.k_swFoot; 
    dc.col_param.k_swLeg; 
    dc.col_param.theta0; 
    dc.col_param.r0;  
    dc.const_param.gravi;
    dc.col_param.L0_ds; 
    dc.col_param.k0_ds;
    var_stiff_bound;
    dc.col_param.foot; 
    dc.col_param.footPlus];

% 2021.09.01: new SLIP-SL trajectory with low CoT (CoT = 0.1907)
load('SLIPSL_Data\ref_star2021-09-03-14-20') % dc_comp2021-09-01-16-06

ref_star_ss = [ref_star.ss.x_M_star, ... 
    ref_star.ss.y_M_star, ref_star.ss.x_swFoot_star, ref_star.ss.y_swFoot_star, ...
    ref_star.ss.del_y_M_star, ref_star.ss.del_x_swFoot_star, ref_star.ss.del_y_swFoot_star, ...
    ref_star.ss.del2_y_M_star, ref_star.ss.del2_x_swFoot_star, ref_star.ss.del2_y_swFoot_star, ...
    ref_star.ss.x_CoM_star];

ref_star_ds = [ref_star.ds.x_CoM_star, ...
    ref_star.ds.y_CoM_star, ref_star.ds.dx_CoM_star, ...
    ref_star.ds.del_y_CoM_star, ref_star.ds.del_dx_CoM_star, ...
    ref_star.ds.del2_y_CoM_star];

%% alpha_ref
alpha_ss_ref = pi - atan2(ref_star.dc.ss.y_CoM, ref_star.dc.ss.x_CoM);
alpha_ds_ref = pi - atan2(ref_star.dc.ds.y_CoM, ref_star.dc.ds.x_CoM);

%%
sample_time = 0.001;

%% Model parameters

params.m1 = 4.75;
params.m2 = 5.25;

params.m5 = 60; % 5,1,1
params.m3 = params.m2;
params.m4 = params.m1;

params.l1 = 0.55;
params.l2 = 0.5;
params.l3 = params.l2;
params.l4 = params.l1;
params.l5 = 0.3;

params.g = 9.81;

params.I1 = params.m1*params.l1^2/12;
params.I2 = params.m2*params.l2^2/12;
params.I3 = params.I1;
params.I4 = params.I2;
params.I5 = params.m5*params.l5^2/12;

params.r = 2.0; % dimensionless lever arm ratio (found from optimizing wrt SR) r = r_h / r_k
params.k_bar_ba = 30; % [Nm] k_bar_ba = k_ba * r_k^2

params.r_k = 0.02; % [m] we choose this value and rest of the parameters are defined according to it and r, k_bar_ba
params.r_h = params.r*params.r_k; % [m]
params.k_ba = params.k_bar_ba/(params.r_k^2); % [N/m]
% params.k_ba = 0; % [N/m]
params.phi_h0 = pi; % [rad] free angle of springs at hip
params.phi_k0 = pi; % [rad] free angle of springs at knee

params.varStiff_control_reduction_ratio = 0.5; % reduce the effect of VSLISP controller by this ratio

param = [
    params.m1; 
    params.m2; 
    params.m5; 
    params.l1; 
    params.l2; 
    params.l5; 
    params.g; 
    params.I1; 
    params.I2; 
    params.I5; 
    params.r_k; 
    params.r_h; 
    params.k_ba; 
    params.phi_h0; 
    params.phi_k0; 
    params.varStiff_control_reduction_ratio];

%% generate discrete alpha

% for single stance phase
% figure()
alpha_ss_ref_discrete = zeros(length(slipsl.time_ss), 1);
for N=1:length(slipsl.time_ss)

    x_CoM_des = slipsl.x_ss(N);
    z_CoM_des = slipsl.y_ss(N);
    x_sw_des = swFootSLIPSL.x(N);
    z_sw_des = swFootSLIPSL.z(N);
    dx_CoM_des = slipsl.dx_ss(N);
    dz_CoM_des = slipsl.dy_ss(N);
    des_dx_sw = swFootSLIPSL.dx(N);
    des_dz_sw = swFootSLIPSL.dz(N);

    init_flag = [1; 0; -dc.col_param.footPlus; -slipsl.time_ss(N); slipsl.x_ss(1); slipsl.y_ss(1); alpha_ss_ref(1)];
    des_traj_alpha = [
    x_CoM_des; z_CoM_des; x_sw_des; z_sw_des;
    dx_CoM_des; dz_CoM_des; des_dx_sw; des_dz_sw];
    [des_th, ~] = calc_desJointAngles(zeros(10,1), des_traj_alpha, init_flag, param, 1);

    % draw_robot(des_th, param, x_CoM_des, z_CoM_des, x_sw_des, z_sw_des, init_flag)
    % pause

    alpha_ss_ref_discrete(N,1) = pi - (2*des_th(1) + des_th(2))/2;

end

%%
% for double stance phase
% figure()
alpha_ds_ref_discrete = zeros(length(slipsl.time_ds), 1);
for N=1:length(slipsl.time_ds)

    x_CoM_des = slipsl.x_ds(N);
    z_CoM_des = slipsl.y_ds(N);
    x_sw_des = -swFootSLIPSL.x(end);
    z_sw_des = 0;
    dx_CoM_des = slipsl.dx_ds(N);
    dz_CoM_des = slipsl.dy_ds(N);
    des_dx_sw = 0;
    des_dz_sw = 0;

    init_flag = [2; dc.col_param.footPlus; 0; -slipsl.time_ss(N); slipsl.x_ss(1); slipsl.y_ss(1); alpha_ss_ref(1)];
    des_traj_alpha = [
    x_CoM_des; z_CoM_des; x_sw_des; z_sw_des;
    dx_CoM_des; dz_CoM_des; des_dx_sw; des_dz_sw];
    [des_th, ~] = calc_desJointAngles(zeros(10,1), des_traj_alpha, init_flag, param, 2);
    % draw_robot(des_th, param, x_CoM_des, z_CoM_des, x_sw_des, z_sw_des, init_flag);
    % pause

    alpha_ds_ref_discrete(N,1) = pi - (2*des_th(1) + des_th(2))/2;

end

%% Getting the ground reaction forces equation

% slipslTraj_ss = [ref_star.dc.ss.time, ref_star.dc.ss.x_CoM', ref_star.dc.ss.y_CoM', ref_star.dc.ss.dx_CoM', ref_star.dc.ss.dy_CoM', alpha_ss_ref'];
% slipslTraj_ds = [ref_star.dc.ds.time - ref_star.dc.ds.time(1), ref_star.dc.ds.x_CoM, ref_star.dc.ds.y_CoM, ref_star.dc.ds.dx_CoM, ref_star.dc.ds.dy_CoM, alpha_ds_ref];
% 
% swFootTraj = [ref_star.dc.ss.x_CoM', ref_star.dc.ss.x_sw', ref_star.dc.ss.y_sw', ref_star.dc.ss.dx_sw', ref_star.dc.ss.dy_sw']; % swing foot reference trajectory

slipslTraj_ss = [slipsl.time_ss, slipsl.x_ss, slipsl.y_ss, slipsl.dx_ss, slipsl.dy_ss, alpha_ss_ref_discrete];
slipslTraj_ds = [slipsl.time_ds, slipsl.x_ds, slipsl.y_ds, slipsl.dx_ds, slipsl.dy_ds, alpha_ds_ref_discrete];

swFootTraj = [swFootSLIPSL.x_CoM, swFootSLIPSL.x, swFootSLIPSL.z, swFootSLIPSL.dx, swFootSLIPSL.dz]; % swing foot reference trajectory

%% Calculate Link Lengths
[~, link_lengths] = calculate_optimal_link_lengths(dc, param);
display(link_lengths)

%% Initial conditions
N = 2; % starting collocation point
% N = dc.N_ss + 1;
x_CoM_des = slipsl.x_ss(N);
z_CoM_des = slipsl.y_ss(N);
x_sw_des = swFootSLIPSL.x(N);
z_sw_des = swFootSLIPSL.z(N);
dx_CoM_des = slipsl.dx_ss(N);
dz_CoM_des = slipsl.dy_ss(N);
des_dx_sw = swFootSLIPSL.dx(N);
des_dz_sw = swFootSLIPSL.dz(N);

init_flag = [1; 0; -dc.col_param.footPlus; -slipsl.time_ss(N); slipsl.x_ss(1); slipsl.y_ss(1); alpha_ss_ref(1)];
clear CoM
des_traj_alpha = [
    x_CoM_des; z_CoM_des; x_sw_des; z_sw_des;
    dx_CoM_des; dz_CoM_des; des_dx_sw; des_dz_sw];
[des_th, decoder_time_elapsed] = calc_desJointAngles(zeros(10,1), des_traj_alpha, init_flag, param, 1);
% draw_robot(des_th, param, x_CoM_des, z_CoM_des, x_sw_des, z_sw_des, init_flag)
q0 = des_th(1:5, 1);
dq0 = des_th(6:end, 1);

Initial_state = [q0;dq0]; % For Simulink integrator with external reset
%% VSLIPSL Controller gains and ref trajectories
close
% Controller Gains
gain_VSLIPSL.K_p = 200;
gain_VSLIPSL.K_d = 40;

gain_VSLIPSL.K_p_sw = 200;
gain_VSLIPSL.K_d_sw = 40;

gain_VSLIPSL.K_p_ds = 200;
gain_VSLIPSL.K_d_ds = 40;
gain_VSLIPSL.K_v_ds = 40; % 5 in visser 2012

gains_VSLIPSL = [gain_VSLIPSL.K_p; gain_VSLIPSL.K_d; gain_VSLIPSL.K_p_sw; gain_VSLIPSL.K_d_sw; gain_VSLIPSL.K_p_ds; gain_VSLIPSL.K_d_ds; gain_VSLIPSL.K_v_ds];

%% PSO

repeat_step = 1;
pNumber = 1; % number of particles
search_step = 1; % number of times to try the particles

f_reset = 1; % choose if you have reset best particle or not
f_record = 0; % choose whether pBestMemo should be saved automatically
f_dist = [1; -100; 0]; % flag for activating or deactivating the dist forces and choosing their magnitudes [on/off; F_dist_x, F_dist_y]

Tf = 15; % simulation finish time [secs]
%%

% gains = [kPi, kDi, ...], i = [2, 3, 4, 5]

gains = [
    1000, 75, ...
    1000, 75, ...
    1000, 75, ...
    1000, 75];

% Run the simulation
sim_settings.use_variable_stiffness_BA = true;

sim_settings_bus_info = Simulink.Bus.createObject(sim_settings);
sim_settings_bus = evalin('base', sim_settings_bus_info.busName);
% set the Constant Block output datatype as "Bus: sim_settings_bus"
% Also in the model explorer inside the matlab function

display(sim_settings)

gamma = 5;

open_system('model_5LinkWalking')
sim('model_5LinkWalking')

%%
flag_print = false;
if f_dist(1) == 1
    save_name = "disturbance_on";
else
    save_name = "disturbance_off";
end

%% Stability Analysis
% figure_poincare(simout, param, flag(:,2), step_no, rt_VLO, time)

%% CoM acceleration comparison
plot_tracking_jointAngles(simout, flag, time, des_th, flag_print, save_name)

%% Plot VSLIPSL Variable stiffness commands
plot_VSLIPSL_U(time, U_varStiff_SLIPSL, flag, dc, flag_print, save_name)

%% Plot 5-Link BA stifnesses
plot_5Link_ba_stiffness(time, U_varStiff_BA, flag, params, flag_print, save_name)

%% Plot Tracking Error
% TODO
figure()
for i = 1:5
    subplot(5,1,i)
    tracking_error = abs(mod(des_th(:,i), 2*pi) ...
        - mod(simout(:,i), 2*pi));
    plot(time, tracking_error)
    ylabel("error " + num2str(i) + " [rad]")
end
%% Calculate the tracking error 
% detect state change
flag_prev = flag(1,1);
state_change_idx = [1];
for i = 2:length(flag(:,1))
    flag_cur = flag(i,1);
    if flag_cur ~= flag_prev
        state_change_idx(end + 1) = i;
    end
    flag_prev = flag_cur;
end

tracking_error = zeros(length(time), 5);
end_step = step_no(end) - 2;
for j = 1:5
    for i = 1:end_step
        % tracking_error(:,i) = abs(mod(des_th(:,i), 2*pi) - mod(simout(:,i), 2*pi));
        start_id = state_change_idx(2*i + 1);
        end_id = state_change_idx(2*i + 3);
        current_step_tracking_error(i,j) =sum( ...
            abs(mod(des_th(start_id:end_id,j), 2*pi) - mod(simout(start_id:end_id,j), 2*pi)));
    end
end

sprintf('Total tracking error is %f', sum(sum(current_step_tracking_error))/end_step)

%% Animation
f_animation = 1;
if f_animation == 1
    f_video = 0; % flag for recrding video
    f_pause = 0;
    frame_leap = 20;
    animation(f_video, simout, sample_time, param, f_pause, frame_leap, flag, step_no, force, des_traj, des_th);
%     animation(f_video, simout, sample_time, param, f_pause, frame_leap, flag, step_no, force, des_traj_alpha, des_th);
end
%% encoder-decoder elapsed time

encoder_time_elapsed = encoder_time_elapsed(~isnan(encoder_time_elapsed));
mean_encoder__time = mean(encoder_time_elapsed);
max_encoder__time = max(encoder_time_elapsed);
display(mean_encoder__time)
display(max_encoder__time)

mean_decoder_time = mean(decoder_time_elapsed);
max_decoder_time_elapsed = max(decoder_time_elapsed);
display(mean_decoder_time)
display(max_encoder__time)


%%

function dc = calc_slipsl_traj(dc)
% constant parameters
m_M = dc.const_param.m_M;
m_swLeg = dc.const_param.m_swLeg;
m_swFoot = dc.const_param.m_swFoot;
L_thigh = dc.const_param.L_thigh;
% I_swLeg = dc.const_param.I_swLeg;
% I_swFoot = dc.const_param.I_swFoot;
% gravi = dc.const_param.gravi;

x_M = dc.simout_ss(1,:)';
y_M = dc.simout_ss(2,:)';
theta = dc.simout_ss(3,:)';
r = dc.simout_ss(4,:)';

dx_M = dc.simout_ss(5,:)';
dy_M = dc.simout_ss(6,:)';
dtheta = dc.simout_ss(7,:)';
dr = dc.simout_ss(8,:)';

dc.ss_traj.x_CoM_ss = (m_swLeg*(x_M + (L_thigh*cos(theta))/2) + m_M*x_M + m_swFoot*(x_M + cos(theta).*(L_thigh + r)))/(m_M + m_swLeg + m_swFoot);
dc.ss_traj.y_CoM_ss = (m_swFoot*(y_M + sin(theta).*(L_thigh + r)) + m_swLeg*(y_M + (L_thigh*sin(theta))/2) + m_M*y_M)/(m_M + m_swLeg + m_swFoot);

dc.ss_traj.x_sw = x_M + cos(theta).*(L_thigh + r);
dc.ss_traj.y_sw = y_M + sin(theta).*(L_thigh + r);

dc.ss_traj.dx_CoM_ss = dx_M + (dr.*m_swFoot.*cos(theta))/(m_M + m_swLeg + m_swFoot) - (dtheta.*sin(theta).*((L_thigh*m_swLeg)/2 + L_thigh*m_swFoot + m_swFoot*r))/(m_M + m_swLeg + m_swFoot);
dc.ss_traj.dy_CoM_ss = dy_M + (dr.*m_swFoot.*sin(theta))/(m_M + m_swLeg + m_swFoot) + (dtheta.*cos(theta).*((L_thigh*m_swLeg)/2 + L_thigh*m_swFoot + m_swFoot*r))/(m_M + m_swLeg + m_swFoot);

dc.ss_traj.dx_sw = dx_M + dr.*cos(theta) - dtheta.*sin(theta).*(L_thigh + r);
dc.ss_traj.dy_sw = dy_M + dr.*sin(theta) + dtheta.*cos(theta).*(L_thigh + r);
end