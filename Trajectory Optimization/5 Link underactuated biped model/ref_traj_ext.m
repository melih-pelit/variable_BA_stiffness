% 2021.01.18 Extracting the OpenOCL trajectrory as reference trajectory
% Mustafa Melih Pelit

function [ocl_traj] = ref_traj_ext(sol)

% loading the OpenOCL solution
% load('OpenOCLTraj\5LinkWalkingOpenOCL2021-01-16-17-54') % with updated v.o.
% load('OpenOCLTraj\5LinkWalkingOpenOCL2021-01-19-21-19') % with th2 >= 5
% load('OpenOCLTraj\5LinkWalkingOpenOCL2021-01-21-15-10') % with 80<torso<90

%%%%% Constant Parameters %%%%%
m1 = mean(sol{1}.parameters.m1.value); % mass of link 1 [kg]
m2 = mean(sol{1}.parameters.m2.value); % mass of link 2 [kg]
m3 = m2; % mass of link 3 [kg]
m4 = m1; % mass of link 4 [kg]
m5 = mean(sol{1}.parameters.m5.value); % mass of link 5 (trunk) [kg]

l1 = mean(sol{1}.parameters.l1.value); % length of link n [m]
l2 = mean(sol{1}.parameters.l2.value);
l3 = l2;
l4 = l1;
l5 = mean(sol{1}.parameters.l5.value);

I1 = mean(sol{1}.parameters.I1.value); % rotational inertia [kg m^2]
I2 = mean(sol{1}.parameters.I2.value);
I3 = I2;
I4 = I1;
I5 = mean(sol{1}.parameters.I5.value);

g = mean(sol{1}.parameters.g.value); % gravitational acceleration [m/s^2]

d_obs = mean(sol{1}.parameters.d_obs.value); % [m] distance of the obstacle from the stance foot
w_obs = mean(sol{1}.parameters.w_obs.value);
h_obs = mean(sol{1}.parameters.h_obs.value);

%%
simout_1 = [
    sol{1}.states.th1.value', ...
    sol{1}.states.th2.value', ...
    sol{1}.states.th3.value', ...
    sol{1}.states.th4.value', ...
    sol{1}.states.th5.value', ...
    sol{1}.states.dth1.value', ...
    sol{1}.states.dth2.value', ...
    sol{1}.states.dth3.value', ...
    sol{1}.states.dth4.value', ...
    sol{1}.states.dth5.value'];

simout_2 = [
    sol{2}.states.th1.value', ...
    sol{2}.states.th2.value', ...
    sol{2}.states.th3.value', ...
    sol{2}.states.th4.value', ...
    sol{2}.states.th5.value', ...
    sol{2}.states.dth1.value', ...
    sol{2}.states.dth2.value', ...
    sol{2}.states.dth3.value', ...
    sol{2}.states.dth4.value', ...
    sol{2}.states.dth5.value'];
% simout = [sol.states.th1.value; sol.states.th2.value; sol.states.th3.value; sol.states.th4.value; sol.states.th5.value; sol.states.dth1.value; sol.states.dth2.value; sol.states.dth3.value; sol.states.dth4.value; sol.states.dth5.value]';
% u = [sol.controls.u2.value; sol.controls.u3.value; sol.controls.u4.value; sol.controls.u5.value]';
% u_ext = [u; u(end,:)];
time_1 = sol{1}.states.time.value;
time_2 = sol{2}.states.time.value;

%% Calculation CoM position and vel.

for j = 1:length(sol)
    if j == 1
        simout = simout_1;
        time = time_1;
    elseif j == 2
        simout = simout_2;
        time = time_2;
    end

    for i=1:length(time)

        X = simout(i,:);

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

        x1 = 0; % x position of the stance foot
        % x_g CoM's of respective links
        x_G = zeros(5);
        x_G(1) = x1 + (l1/2)*cos(th1);
        x_G(2) = x1 + l1*cos(th1) + (l2/2)*cos(th1 + th2);
        x_G(3) = x1 + l1*cos(th1) + l2*cos(th1 + th2) + (l3/2)*cos(th1 + th2 + th3);
        x_G(4) = x1 + l1*cos(th1) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3) + (l4/2)*cos(th1 + th2 + th3 + th4);
        x_G(5) = x1 + l1*cos(th1) + l2*cos(th1 + th2) + (l5/2)*cos(th1 + th2 + th5); % body

        % z_g CoM's of respective links
        y_G = zeros(5);
        y_G(1) = (l1/2)*sin(th1);
        y_G(2) = l1*sin(th1) + (l2/2)*sin(th1 + th2);
        y_G(3) = l1*sin(th1) + l2*sin(th1 + th2) + (l3/2)*sin(th1 + th2 + th3);
        y_G(4) = l1*sin(th1) + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3) + (l4/2)*sin(th1 + th2 + th3 + th4);
        y_G(5) = l1*sin(th1) + l2*sin(th1 + th2) + (l5/2)*sin(th1 + th2 + th5);

        xG(i) = (m1*x_G(1) + m2*x_G(2) + m3*x_G(3) + m4*x_G(4) + m5*x_G(5))/(m1 + m2 + m3 + m4 + m5);
        yG(i) = (m1*y_G(1) + m2*y_G(2) + m3*y_G(3) + m4*y_G(4) + m5*y_G(5))/(m1 + m2 + m3 + m4 + m5);

        % Calculation of CoM velocity

        % dx_G: CoM velocities in x direction
        dx_G = zeros(5);
        dx_G(1) = -(l1/2)*dth1*sin(th1);
        dx_G(2) = -l1*dth1*sin(th1) - (l2/2)*(dth1 + dth2)*sin(th1 + th2);
        dx_G(3) = -l1*dth1*sin(th1) - l2*(dth1 + dth2)*sin(th1 + th2) - (l3/2)*(dth1 + dth2 + dth3)*sin(th1 + th2 + th3);
        dx_G(4) = -l1*dth1*sin(th1) - l2*(dth1 + dth2)*sin(th1 + th2) - l3*(dth1 + dth2 + dth3)*sin(th1 + th2 + th3) - (l4/2)*(dth1 + dth2 + dth3 + dth4)*sin(th1 + th2 + th3 + th4);
        dx_G(5) = -l1*dth1*sin(th1) - l2*(dth1 + dth2)*sin(th1 + th2) - (l5/2)*(dth1 + dth2 + dth5)*sin(th1 + th2 + th5);

        % dy_g: CoM velocities in y direction
        dy_G = zeros(5);
        dy_G(1) = (l1/2)*dth1*cos(th1);
        dy_G(2) = l1*dth1*cos(th1) + (l2/2)*(dth1 + dth2)*cos(th1 + th2);
        dy_G(3) = l1*dth1*cos(th1) + l2*(dth1 + dth2)*cos(th1 + th2) + (l3/2)*(dth1 + dth2 + dth3)*cos(th1 + th2 + th3);
        dy_G(4) = l1*dth1*cos(th1) + l2*(dth1 + dth2)*cos(th1 + th2) + l3*(dth1 + dth2 + dth3)*cos(th1 + th2 + th3) + (l4/2)*(dth1 + dth2 + dth3 + dth4)*cos(th1 + th2 + th3 + th4);
        dy_G(5) = l1*dth1*cos(th1) + l2*(dth1 + dth2)*cos(th1 + th2) + (l5/2)*(dth1 + dth2 + dth5)*cos(th1 + th2 + th5);

        dxG(i) = (m1*dx_G(1) + m2*dx_G(2) + m3*dx_G(3) + m4*dx_G(4) + m5*dx_G(5))/(m1 + m2 + m3 + m4 + m5);
        dyG(i) = (m1*dy_G(1) + m2*dy_G(2) + m3*dy_G(3) + m4*dy_G(4) + m5*dy_G(5))/(m1 + m2 + m3 + m4 + m5);

        % calculating the swing foot traj

        % calculating the torso orientation

        x_sw(i) = l1*cos(th1) + l2*cos(th1 + th2) + l3*cos(th1 + th2 + th3) + l4*cos(th1 + th2 + th3 + th4);
        y_sw(i) = l1*sin(th1) + l2*sin(th1 + th2) + l3*sin(th1 + th2 + th3) + l4*sin(th1 + th2 + th3 + th4);

        dx_sw(i) = -(l1*sin(th1)*dth1 + l2*sin(th1 + th2)*(dth1 + dth2) + l3*sin(th1 + th2 + th3)*(dth1 + dth2 + dth3) + l4*sin(th1 + th2+ th3 + th4)*(dth1 + dth2 + dth3 + dth4));
        dy_sw(i) = l1*cos(th1)*dth1 + l2*cos(th1 + th2)*(dth1 + dth2) + l3*cos(th1 + th2 + th3)*(dth1 + dth2 + dth3) + l4*cos(th1 + th2+ th3 + th4)*(dth1 + dth2 + dth3 + dth4);

    end

    if j == 1
        ocl_traj.ss.time = time';

        ocl_traj.ss.xG = xG';
        ocl_traj.ss.yG = yG';
        ocl_traj.ss.dxG = dxG';
        ocl_traj.ss.dyG = dyG';

        ocl_traj.ss.x_sw = x_sw';
        ocl_traj.ss.y_sw = y_sw';
        ocl_traj.ss.dx_sw = dx_sw';
        ocl_traj.ss.dy_sw = dy_sw';

        ocl_traj.ss.trunk = simout(:,1) + simout(:,2) + simout(:,5);
        ocl_traj.ss.dtrunk = simout(:,6) + simout(:,7) + simout(:,10);

        ocl_traj.ss.simout = simout;
    elseif j == 2
        ocl_traj.ds.time = time';

        ocl_traj.ds.xG = xG';
        ocl_traj.ds.yG = yG';
        ocl_traj.ds.dxG = dxG';
        ocl_traj.ds.dyG = dyG';

        ocl_traj.ds.x_sw = x_sw';
        ocl_traj.ds.y_sw = y_sw';
        ocl_traj.ds.dx_sw = dx_sw';
        ocl_traj.ds.dy_sw = dy_sw';

        ocl_traj.ds.trunk = simout(:,1) + simout(:,2) + simout(:,5);
        ocl_traj.ds.dtrunk = simout(:,6) + simout(:,7) + simout(:,10);

        ocl_traj.ds.simout = simout;
    end

    clear th1 th2 th3 th4 th5 dth1 dth2 dth3 dth4 dth5
    clear x_G y_G dx_G dy_G
end

clear th1 th2 th3 th4 th5 dth1 dth2 dth3 dth4 dth5
clear x_G y_G dx_G dy_G
%%

%% saving the traj
filename = sprintf('ocl_5link_traj%s.mat', datestr(now,'yyyy-mm-dd-HH-MM'));
subfolder = 'OpenOCLtraj';
save(fullfile(subfolder,filename),'ocl_traj')

end

