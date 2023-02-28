function trackingPlots(simout, inputTorque, des_theta_alpha, param, flag, time, f_print, time_start, time_end)

% if time(end) > 5
%     time_start = time(end) - 5;
%     time_end = time(end);
% else
%     time_start = 0;
%     time_end = time(end);
% end

% time_start = 9;
% time_end = 10;

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



% subplot(5,1,1);
% plot(time, simout(:,1), '.', 'MarkerSize', marker_size, 'Color', color, 'LineWidth', lineW); hold on;
% plot(time, des_theta_alpha(:,1), 'k--', 'LineWidth', lineW_ref);
% % plot(time, flag(:,1))
% % grid on;
% xlim([time_start,time_end])
% set(gca, 'XTickLabel', [])
% 
% subplot(5,1,2);
% plot(time, simout(:,2), '.', 'MarkerSize', marker_size, 'Color', color, 'LineWidth', lineW); hold on;
% plot(time, des_theta_alpha(:,2), 'k--', 'LineWidth', lineW_ref);
% % plot(time, flag(:,1))
% % grid on;
% xlim([time_start,time_end])
% set(gca, 'XTickLabel', [])
% 
% subplot(5,1,3);
% plot(time, simout(:,3), '.', 'MarkerSize', marker_size, 'Color', color, 'LineWidth', lineW); hold on;
% plot(time, des_theta_alpha(:,3), 'k--', 'LineWidth', lineW_ref);
% % plot(time, flag(:,1));
% % grid on;
% xlim([time_start,time_end])
% set(gca, 'XTickLabel', [])
% 
% subplot(5,1,4);
% plot(time, simout(:,4), '.', 'MarkerSize', marker_size, 'Color', color, 'LineWidth', lineW); hold on;
% plot(time, des_theta_alpha(:,4), 'k--', 'LineWidth', lineW_ref);
% % plot(time, flag(:,1))
% % grid on;
% xlim([time_start,time_end])
% set(gca, 'XTickLabel', [])
% 
% subplot(5,1,5);
% plot(time, simout(:,5), '.', 'MarkerSize', marker_size, 'Color', color, 'LineWidth', lineW); hold on;
% plot(time, des_theta_alpha(:,5), 'k--', 'LineWidth', lineW_ref);
% % plot(time, flag(:,1)) 
% % grid on;
% xlim([time_start,time_end])


% figure
% 
% subplot(2,6,1:2);
% plot(time, des_theta_alpha(:,6), 'LineWidth', lineW); hold on;
% plot(time, simout(:,6), 'LineWidth', lineW); title('dth_1');
% % plot(time, flag(:,1))
% grid on;
% xlim([time_start,time_end])
% 
% subplot(2,6,3:4);
% plot(time, des_theta_alpha(:,7), 'LineWidth', lineW); hold on;
% plot(time, simout(:,7), 'LineWidth', lineW); title('dth_2');
% % plot(time, flag(:,1))
% grid on;
% xlim([time_start,time_end])
% 
% subplot(2,6,5:6);
% plot(time, des_theta_alpha(:,8), 'LineWidth', lineW); hold on;
% plot(time, simout(:,8), 'LineWidth', lineW); title('dth3');
% % plot(time, flag(:,1));
% grid on;
% xlim([time_start,time_end])
% 
% subplot(2,6,8:9);
% plot(time, des_theta_alpha(:,9), 'LineWidth', lineW); hold on;
% plot(time, simout(:,9), 'LineWidth', lineW); title('dth_4');
% % plot(time, flag(:,1))
% grid on;
% xlim([time_start,time_end])
% 
% subplot(2,6,10:11);
% % plot(time, pi/2*ones(length(time),1)); hold on;
% plot(time, des_theta_alpha(:,10), 'LineWidth', lineW); hold on;
% plot(time, simout(:,10), 'LineWidth', lineW); title('dth_5');
% % plot(time, flag(:,1))
% title('th_5'); grid on;
% xlim([time_start,time_end])
% 
% figure
% plot(time, inputTorques, 'LineWidth', lineW)
% hold on
% grid on
% % plot(time, 150*flag(:,1))
% xlim([time_start,time_end])


end