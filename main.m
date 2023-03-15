%% five_link_walking_sim
% 2022.11.11 - Melih Pelit
% Blind underactuated walking based on optimized trajectory on rough
% terrain

clear all
close all

%% Reference OpenOCL traj

load('OpenOCLTraj\BA_landing_traj_v12022-07-06-17-15'); % loads the landing_traj variable
ocl_traj = landing_traj.ocl_traj;

%% uneven ground input

terrain_name = 'terrain data\unevenground_v3_1.mat'; % single seed
load(terrain_name)

% try catch is because I previously didn't have "seed" fields for some terrain
try
    uneven_terrain.y_g_seed;
catch
    uneven_terrain.y_g_seed= 10 * uneven_terrain.y_g(101,:); % TODO
end
uneven_terrain.y_g_curr = 0.001 * uneven_terrain.y_g_seed; % set this temporarily for BUS setting
%% Model parameters
params.m1 = 5;
params.m2 = 5;

params.m5 = 60; % 5,1,1
params.m3 = params.m2;
params.m4 = params.m1;

params.l1 = 0.48;
params.l2 = 0.48;
params.l3 = params.l2;
params.l4 = params.l1;
params.l5 = 0.48;

params.g = 9.81;
params.I1 = 0.0960;
params.I2 = 0.0960;
params.I3 = params.I1;
params.I4 = params.I2;
params.I5 = 1.1520;

% biarticular muscle parameters

% params.r = ocl_traj.biart.r; % dimensionless lever arm ratio (found from optimizing wrt SR) r = r_h / r_k
% params.k_bar_ba = ocl_traj.biart.k_bi; % [Nm] k_bar_ba = k_ba * r_k^2

params.r = 1.6; % dimensionless lever arm ratio (found from optimizing wrt SR) r = r_h / r_k
params.k_bar_ba = 200; % [Nm] k_bar_ba = k_ba * r_k^2

params.r_k = 0.02; % [m] we choose this value and rest of the parameters are defined according to it and r, k_bar_ba
params.r_h = params.r*params.r_k; % [m]
% params.k_ba = params.k_bar_ba/(params.r_k^2); % [N/m]
params.k_ba = 0; % [N/m]
params.phi_h0 = pi; % [rad] free angle of springs at hip
params.phi_k0 = pi; % [rad] free angle of springs at knee

param = [params.m1; params.m2; params.m5; params.l1; params.l2; params.l5; params.g; params.I1; params.I2; params.I5; params.r_k; params.r_h; params.k_ba; params.phi_h0; params.phi_k0];
%% 

Tf = 10;
K_p = 9700;
K_d = 220;
% K_p = 2700;
% K_d = 80;
gains = [K_p,K_d,K_p,K_d,K_p,K_d,K_p,K_d,K_p,K_d];
load_system('model_5LinkWalking_NODS')

simulation_type = 1;
switch simulation_type
    case 1
        % run a single walking simulation on terrain height k

        % select the uneven terrain difficulty, k=1 is flat terrain
        k = 1; % delta = (k-1)*0.001 m
        deltaY = 0.001;
        [simout, inputTorque, des_theta_alpha, des_com_sw_alpha, decoder_output, flag, time] = run_walking_simulation(landing_traj, uneven_terrain, params, Tf, gains, k);
        fprintf("Time(end) = " + num2str(time(end)) + "\n")
end
%% Trajectory Tracking Plots

f_print = 0;
time_start = 0;
time_end = 2;
trackingPlots( ...
    simout, inputTorque, des_theta_alpha, param, flag, time, f_print, ...
    time_start, time_end, ...
    des_com_sw_alpha, decoder_output)

%% Animation
f_animation = 0;
if f_animation == 1
    f_video = 0; % flag for recording video
    f_pause = 0;
    frame_leap = 10;
    animation(f_video, simout, param, f_pause, frame_leap, flag, uneven_terrain, time, k, deltaY)
pause
end