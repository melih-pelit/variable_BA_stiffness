function figures(sol)
%FIGURES function creates and exports the figures for 5 link underactuated
%wakling with OpenOCL

param.m1 = mean(sol.parameters.m1.value); % mass of link 1 [kg]
param.m2 = mean(sol.parameters.m2.value); % mass of link 2 [kg]
param.m3 = mean(sol.parameters.m2.value); % mass of link 3 [kg]
param.m4 = mean(sol.parameters.m1.value); % mass of link 4 [kg]
param.m5 = mean(sol.parameters.m5.value); % mass of link 5 (trunk) [kg]

param.l1 = mean(sol.parameters.l1.value); % length of link n [m]
param.l2 = mean(sol.parameters.l2.value);
param.l3 = mean(sol.parameters.l2.value);
param.l4 = mean(sol.parameters.l1.value);
param.l5 = mean(sol.parameters.l5.value);

param.I1 = mean(sol.parameters.I1.value); % rotational inertia [kg m^2]
param.I2 = mean(sol.parameters.I2.value);
param.I3 = mean(sol.parameters.I2.value);
param.I4 = mean(sol.parameters.I1.value);
param.I5 = mean(sol.parameters.I5.value);

param.g = mean(sol.parameters.g.value); % gravitational acceleration [m/s^2]

simout = [sol.states.th1.value; sol.states.th2.value; sol.states.th3.value; sol.states.th4.value; sol.states.th5.value; sol.states.dth1.value; sol.states.dth2.value; sol.states.dth3.value; sol.states.dth4.value; sol.states.dth5.value]';
u = [sol.controls.u2.value; sol.controls.u3.value; sol.controls.u4.value; sol.controls.u5.value]';
time = sol.states.time.value;

x_F.th1 = simout(end,1);
x_F.th2 = simout(end,2);
x_F.th3 = simout(end,3);
x_F.th4 = simout(end,4);
x_F.th5 = simout(end,5);

x_I.th1 = simout(1,1);
x_I.th2 = simout(1,2);
x_I.th3 = simout(1,3);
x_I.th4 = simout(1,4);
x_I.th5 = simout(1,5);

th_I = [x_I.th1; x_I.th2; x_I.th3; x_I.th4; x_I.th5];
th_new = [(x_F.th1 + x_F.th2 + x_F.th3 + x_F.th4 - pi); (-x_F.th4);(-x_F.th3);(-x_F.th2);(x_F.th5 - x_F.th3 + pi)];

u_2_ext = [u(:,1); u(end,1)];
u_3_ext = [u(:,2); u(end,2)];
u_4_ext = [u(:,3); u(end,3)];
u_5_ext = [u(:,4); u(end,4)];
u_ext = [u_2_ext, u_3_ext, u_4_ext, u_5_ext];

% plotting
lgd_font_size = 13;

figure
subplot(3,2,1)
plot(time, simout(:,1))
ylabel('$\theta_{1}$ $[rad]$', 'Interpreter','Latex', 'FontSize', lgd_font_size);
xlabel('time $[s]$', 'Interpreter','Latex', 'FontSize', lgd_font_size);
xlim([0, time(end)])

subplot(3,2,2)
plot(time, simout(:,2))
ylabel('$\theta_{2}$ $[rad]$', 'Interpreter','Latex', 'FontSize', lgd_font_size);
xlabel('time $[s]$', 'Interpreter','Latex', 'FontSize', lgd_font_size);
xlim([0, time(end)])

subplot(3,2,3)
plot(time, simout(:,3))
ylabel('$\theta_{3}$ $[rad]$', 'Interpreter','Latex', 'FontSize', lgd_font_size);
xlabel('time $[s]$', 'Interpreter','Latex', 'FontSize', lgd_font_size);
xlim([0, time(end)])

subplot(3,2,4)
plot(time, simout(:,4))
ylabel('$\theta_{4}$ $[rad]$', 'Interpreter','Latex', 'FontSize', lgd_font_size);
xlabel('time $[s]$', 'Interpreter','Latex', 'FontSize', lgd_font_size);
xlim([0, time(end)])

subplot(3,2,[5,6])
plot(time, simout(:,5))
ylabel('$\theta_{5}$ $[rad]$', 'Interpreter','Latex', 'FontSize', lgd_font_size);
xlabel('time $[s]$', 'Interpreter','Latex', 'FontSize', lgd_font_size);
xlim([0, time(end)])

% set(gcf, 'color', 'none');
% set(gca, 'color', 'none');
% export_fig theta_traj.png -m3

figure
plot(time, u_ext)
ylabel('input torques $[Nm]$', 'Interpreter','Latex', 'FontSize', lgd_font_size);
xlabel('time $[s]$', 'Interpreter','Latex', 'FontSize', lgd_font_size);
xlim([0, time(end)])
lgd = legend('$u_{2}$','$u_{3}$','$u_{4}$','$u_{5}$','Interpreter','Latex');
lgd.FontSize = lgd_font_size;

set(gcf, 'color', 'none');
set(gca, 'color', 'none');
export_fig inputs.png -m3
end

