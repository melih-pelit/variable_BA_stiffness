function trackingPlots(simout, inputTorque, des_theta_alpha, param, flag, time, f_print, time_start, time_end, des_com_sw_alpha)

[CoM, X_task_calculated] = calculate_com_and_sw_foot_from_simout(simout, param, flag);

% if time(end) > 5
%     time_start = time(end) - 5;
%     time_end = time(end);
% else
%     time_start = 0;
%     time_end = time(end);
% end

% time_start = 9;
% time_end = 10;

%% Joint Angle Plots
figure

lineW_ref = 1.2;
lineW = 1.7;
marker_size = 7;

y_limit_plus_rad = 0.02;

color = [0.8500, 0.3250, 0.0980];

y_labels = ['$\theta_1$ [rad]'; '$\theta_2$ [rad]'; '$\theta_3$ [rad]'; '$\theta_4$ [rad]'; '$\theta_5$ [rad]'];

label_font_size = 14;

for i = 1:5
    subplot(5, 1, i);
    plot(time, simout(:,i), '.-', 'MarkerSize', marker_size, 'Color', color, 'LineWidth', lineW); hold on;
    plot(time, des_theta_alpha(:,i), 'k:', 'LineWidth', lineW_ref);
    xlim([time_start,time_end])
    
    index_time_start = find(time == time_start);
    index_time_end = find(time == time_end);
    
    ylim([min(simout(index_time_start:index_time_end,i)) - y_limit_plus_rad, max(simout(index_time_start:index_time_end,i)) + y_limit_plus_rad])
    ylabel(y_labels(i, :), 'Interpreter', 'latex','FontSize',label_font_size)
    
    if i ~= 5
        set(gca, 'XTickLabel', [])
    else
        xlabel('Time [sec]', 'Interpreter', 'latex','FontSize',label_font_size)
        hYLabel = get(gca,'YLabel');
        fig_size = 550;
        set(gcf,'position',[0,0,fig_size*0.9,fig_size])
        leg1 = legend('$\theta_i$', '$\theta_i^*$');
        set(leg1,'Interpreter','latex');
        set(leg1,'FontSize',label_font_size);
    end
    
    if f_print == 1
    set(gcf, 'color', 'none');
    set(gca, 'color', 'none');
        if i == 5
            print(gcf,'trajectory_tracking_plots.png','-dpng','-r300');
        end
    end
end

%% CoM and Swing Foot plots

figure

lineW_ref = 1.2;
lineW = 1.7;
marker_size = 7;

y_limit_plus_rad = 0.02;

color = [0.8500, 0.3250, 0.0980];

y_labels = {'$x_{CoM}$ [m]'; '$y_{CoM}$ [m]'; '$\dot{x}_{CoM}$ [m/s]'; '$\dot{y}_{CoM}$ [m/s]'};

label_font_size = 14;

% des_com_sw_alpha = [
%     xG_des_alpha; yG_des_alpha; x_sw_des_alpha; y_sw_des_alpha; trunk_des_alpha;
%     dxG_des_alpha; dyG_des_alpha; dx_sw_des_alpha; dy_sw_des_alpha; dtrunk_des_alpha;
%     flag_landing];

%% CoM plots
i_to_des_com_index = [1, 2, 6, 7];

for i = 1:4
    subplot(4, 1, i);
    plot(time, CoM(:,i), '.-', 'MarkerSize', marker_size, 'Color', color, 'LineWidth', lineW); hold on;
    plot(time, des_com_sw_alpha(:,i_to_des_com_index(i)), 'k:', 'LineWidth', lineW_ref);
    xlim([time_start,time_end])
    
    index_time_start = find(time == time_start);
    index_time_end = find(time == time_end);
    
    ylim([min(CoM(index_time_start:index_time_end,i)) - y_limit_plus_rad, max(CoM(index_time_start:index_time_end,i)) + y_limit_plus_rad])
    ylabel(y_labels(i, :), 'Interpreter', 'latex','FontSize',label_font_size)
    
    if i ~= 5
        set(gca, 'XTickLabel', [])
    else
        xlabel('Time [sec]', 'Interpreter', 'latex','FontSize',label_font_size)
        hYLabel = get(gca,'YLabel');
        fig_size = 550;
        set(gcf,'position',[0,0,fig_size*0.9,fig_size])
        leg1 = legend('$\theta_i$', '$\theta_i^*$');
        set(leg1,'Interpreter','latex');
        set(leg1,'FontSize',label_font_size);
    end
    
    if f_print == 1
    set(gcf, 'color', 'none');
    set(gca, 'color', 'none');
        if i == 5
            print(gcf,'trajectory_tracking_plots.png','-dpng','-r300');
        end
    end
end

%% sw plots
figure
y_labels = {'$x_{sw}$ [m]'; '$y_{sw}$ [m]'; '$\dot{x}_{sw}$ [m/s]'; '$\dot{y}_{sw}$ [m/s]'};
i_to_des_sw_index = [3, 4, 8, 9];

for i = 1:4
    subplot(4, 1, i);
    plot(time, X_task_calculated(:,4 + i), '.-', 'MarkerSize', marker_size, 'Color', color, 'LineWidth', lineW); hold on;
    plot(time, des_com_sw_alpha(:,i_to_des_sw_index(i)), 'k:', 'LineWidth', lineW_ref);
    xlim([time_start,time_end])
    
    index_time_start = find(time == time_start);
    index_time_end = find(time == time_end);
    
    ylim([min(X_task_calculated(index_time_start:index_time_end,4 + i)) - ...
        y_limit_plus_rad, max(X_task_calculated(index_time_start:index_time_end,4 + i)) + y_limit_plus_rad])
    ylabel(y_labels(i, :), 'Interpreter', 'latex','FontSize',label_font_size)
    
    if i ~= 5
        set(gca, 'XTickLabel', [])
    else
        xlabel('Time [sec]', 'Interpreter', 'latex','FontSize',label_font_size)
        hYLabel = get(gca,'YLabel');
        fig_size = 550;
        set(gcf,'position',[0,0,fig_size*0.9,fig_size])
        leg1 = legend('$\theta_i$', '$\theta_i^*$');
        set(leg1,'Interpreter','latex');
        set(leg1,'FontSize',label_font_size);
    end
    
    if f_print == 1
    set(gcf, 'color', 'none');
    set(gca, 'color', 'none');
        if i == 5
            print(gcf,'trajectory_tracking_plots.png','-dpng','-r300');
        end
    end
end
end

function [CoM, X_task_calculated] = calculate_com_and_sw_foot_from_simout(simout, param, flag)
for i = 1:length(simout)
    CoM(i, :) = calculate_com(simout(i,:), param, flag(i,:), [0; 0; 0; 0; 0]);
    [x_sw, y_sw, dx_sw, dy_sw] = calculate_sw_foot(simout(i,:), param, flag(i,:));

    X_task_calculated(i, :) = [CoM(i, 1:4), x_sw - flag(i,2), y_sw, dx_sw, dy_sw];
end
end