function [simout, inputTorque, des_theta_alpha, flag, time] = run_walking_simulation(landing_traj, uneven_terrain, params, Tf, gains, k)

%%
ocl_traj = landing_traj.ocl_traj;

alpha_ref = pi - (2*ocl_traj.simout(:,1)+ocl_traj.simout(:,2))/2;
[step_alpha_ref, step_inputs_ref] = calc_step_input_ref(ocl_traj);

% inputs_ref = ocl_traj.inputTorques;
inputs_ref = [step_alpha_ref, step_inputs_ref];
joint_angles_ref = [ocl_traj.time, ocl_traj.simout, alpha_ref];

alpha_ref_landing = pi - (2*landing_traj.simout(:,1)+landing_traj.simout(:,2))/2;
landing_ref = [landing_traj.time, landing_traj.simout, alpha_ref_landing];

%% Set the uneven_terrain BUS
uneven_terrain_bus_info = Simulink.Bus.createObject(uneven_terrain);
uneven_terrain_bus = evalin('base', uneven_terrain_bus_info.busName);

%%
sample_time = 0.001;

%%
param = [params.m1; params.m2; params.m5; params.l1; params.l2; params.l5; params.g; params.I1; params.I2; params.I5; params.r_k; params.r_h; params.k_ba; params.phi_h0; params.phi_k0];

%%
N = 3; % desired starting position of the simulation
q0 =  [ocl_traj.simout(N,1); ocl_traj.simout(N,2); ocl_traj.simout(N,3); ocl_traj.simout(N,4); ocl_traj.simout(N,5)];
dq0 = [ocl_traj.simout(N,6); ocl_traj.simout(N,7); ocl_traj.simout(N,8); ocl_traj.simout(N,9); ocl_traj.simout(N,10)];
X = [q0; dq0];

Initial_state = [q0;dq0];
init_t_mode_change = -ocl_traj.time(N); % if I start the simulation for the mid point of the SS phase, i need to set this so that controller starts from the correct spot

delta = (k - 1)*0.001;
uneven_terrain.y_g_curr = delta * uneven_terrain.y_g_seed;

x_g = uneven_terrain.track_start:uneven_terrain.dist_step_size:uneven_terrain.track_end;
y_g = uneven_terrain.y_g_curr;
y_g_curr = interp1(x_g, y_g, 0);
init_flag = [-1; 0; ocl_traj.x_sw(1); init_t_mode_change; alpha_ref(1); 0; y_g_curr]; % change the stance foot to start on the uneven ground

%%
options = simset('SrcWorkspace','current');
sim('model_5LinkWalking_NODS', [], options)
end