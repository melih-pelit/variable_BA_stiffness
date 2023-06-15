function figures1(time, simout, des_traj, CoM_acc, flag, params)

f_print = 0; % set to 1 if you want to print the figure

x_start = 7;
x_end = 13;
lgd_font_size = 10;
fr_LineW = 1.5;
back_LineW = 2.5;

figure()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subplot(4,1,1)
set(gca,'FontSize', lgd_font_size)
rectangle2([10.05 0 0.1, 400] ,'FaceColor',[0.5882    0.5882    0.5882],'EdgeColor',[0.5882    0.5882    0.5882])
hold on
plot(time, CoM_acc(:,1) - flag(:,2), 'Color', [0.8500 0.3250 0.0980], 'LineWidth', back_LineW)

plot(time, des_traj(:,9) - flag(:,2), 'k--', 'LineWidth', fr_LineW)
xlim([x_start,x_end])
% ylabel('$x_{CoM}$ $[m]$', 'Interpreter','Latex', 'FontWeight','bold', 'FontSize', lgd_font_size);
ylim([-0.25,0.15])
box on

legend({'dist. reg.','$x_{CoM}$ $[m]$', '$x_{CoM}^*$ $[m]$'},'Interpreter','latex', 'FontSize', lgd_font_size) % R2018a and earlier

if f_print == 1
    set(gcf, 'color', 'none');
    set(gca, 'color', 'none');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


subplot(4,1,2)
set(gca,'FontSize', lgd_font_size)

rectangle2([10.05 0 0.1, 400] ,'FaceColor',[0.5882    0.5882    0.5882],'EdgeColor',[0.5882    0.5882    0.5882])
hold on
plot(time, CoM_acc(:,2), 'Color', [0.8500 0.3250 0.0980], 'LineWidth', back_LineW)
plot(time, des_traj(:,10), 'k--', 'LineWidth', fr_LineW)
xlim([x_start,x_end])
% ylabel('$y_{CoM}$ $[m]$', 'Interpreter','Latex', 'FontWeight','bold', 'FontSize', lgd_font_size);

legend({'dist. reg.','$y_{CoM}$ $[m]$', '$y_{CoM}^*$ $[m]$'},'Interpreter','latex', 'FontSize', lgd_font_size) % R2018a and earlier

ylim([0.945, 0.99])
box on

if f_print == 1
    set(gcf, 'color', 'none');
    set(gca,'FontSize', lgd_font_size)
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


x_sw = params.l1*cos(simout(:,1)) + params.l2*cos(simout(:,1) + simout(:,2)) + params.l3*cos(simout(:,1) + simout(:,2) + simout(:,3)) + params.l4*cos(simout(:,1) + simout(:,2) + simout(:,3) + simout(:,4)); 
subplot(4,1,3)
set(gca,'FontSize', lgd_font_size)

rectangle2([10.05 0 0.1, 400] ,'FaceColor',[0.5882    0.5882    0.5882],'EdgeColor',[0.5882    0.5882    0.5882])
hold on
plot(time, x_sw, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', back_LineW)

plot(time, des_traj(:,11) - flag(:,2), 'k--', 'LineWidth', fr_LineW)
xlim([x_start,x_end])
% ylabel('$x_{sw}$ $[m]$', 'Interpreter','Latex', 'FontWeight','bold', 'FontSize', lgd_font_size);

legend({'dist. reg.','$x_{sw}$ $[m]$', '$x_{sw}^*$ $[m]$'},'Interpreter','latex', 'FontSize', lgd_font_size) % R2018a and earlier


ylim([-0.5, 0.5])
box on

if f_print == 1
    set(gcf, 'color', 'none');
    set(gca,'FontSize', lgd_font_size)
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


y_sw = params.l1*sin(simout(:,1)) + params.l2*sin(simout(:,1) + simout(:,2)) + params.l3*sin(simout(:,1) + simout(:,2) + simout(:,3)) + params.l4*sin(simout(:,1) + simout(:,2) + simout(:,3) + simout(:,4)); 
subplot(4,1,4)
set(gca,'FontSize', lgd_font_size)

rectangle2([10.05 0 0.1, 400] ,'FaceColor',[0.5882    0.5882    0.5882],'EdgeColor',[0.5882    0.5882    0.5882])
hold on
plot(time, y_sw, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', back_LineW)

plot(time, des_traj(:,12), 'k--', 'LineWidth', fr_LineW)
xlim([x_start,x_end])
% ylabel('$y_{sw}$ $[m]$', 'Interpreter','Latex', 'FontWeight','bold', 'FontSize', lgd_font_size);
xlabel('time $[s]$', 'Interpreter','Latex', 'FontWeight','bold', 'FontSize', lgd_font_size);
ylim([0,0.15])
box on

legend({'dist. reg.','$y_{sw}$ $[m]$', '$y_{sw}^*$ $[m]$'},'Interpreter','latex', 'FontSize', lgd_font_size) % R2018a and earlier
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if f_print == 1
    set(gcf, 'color', 'none');
    set(gca, 'color', 'none');
    % print(gcf,'V-SLIPSL_refTraj_tracking.png','-dpng','-r300');
end


end