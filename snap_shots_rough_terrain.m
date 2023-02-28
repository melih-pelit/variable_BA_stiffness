function snap_shots_rough_terrain(simout, param, flag, time, x_g, y_g, f_print)

f_ba_springs = 0; % turn plotting ba springs on or off

%% Determining one step
f_impact_happened = -1;
impact_indexes = [];
for i =1:length(time)
    f_impact = flag(i,1);
    if f_impact == 1 && f_impact_happened == -1
        f_impact_happened = 1;
        impact_indexes = [impact_indexes;i];
    elseif f_impact == -1 && f_impact_happened == 1
        f_impact_happened = -1;
    end
    
end

figure(3)

%% Plotting the rough terrain
plot(x_g, y_g, 'k', 'LineWidth', 2)
hold on

p.lw = 2;
gray_increment = 0.07;
%%
for i = 1:length(impact_indexes) - 10
    % Plotting the impact moments
    curr_index = impact_indexes(i);
    
    % Plotting the intermediary moments
    next_index = impact_indexes(i+1);
    gray = 0.5;
    for j = curr_index:100:next_index
        p.st_leg_color = [gray gray gray];
        p.sw_leg_color = [gray gray gray];
        p.body_color = [gray gray gray];
        
        current_X = simout(j,:);
        plot_5_links_robot(current_X, flag(j,2), flag(j,7), param, f_ba_springs, p)
        hold on
        
        gray = gray + gray_increment;
    end
    
    current_X = simout(curr_index,:);
    plot_5_links_robot(current_X, flag(curr_index,2), flag(curr_index,7), param, f_ba_springs)
    hold on
end

% canvas size
x_start = -0.5;
x_end = 2.5;

y_start = -0.2;
y_end = 1.7;

xlim([x_start, x_end])
ylim([y_start, y_end]) 
%%

nt = length(simout);
k = 1;

no_of_impacts = length(impact_indexes);
frame_start = impact_indexes(1);
frame_end = impact_indexes(12);

frame_leap = 150;

x1 = 0; %initial location of the stance foot
flag_ds = 0; % flag used to detect when ds change happens so we can add an offset to x1
m = 1;

% for i = frame_start:frame_leap:frame_end
%     
%     if m == 2
%         m = m + 1;
%         continue
%     end
%     current_X = simout(i,:);
%     plot_5_links_robot(current_X, flag(i,2), flag(i,7), param, f_ba_springs)
%     hold on
%  
%     xlim([-0.5, 3.5])
%     ylim([-0.2, 1.7])
%     com(k,:) = calculate_com(current_X, param, flag, [0;0;0;0;0]);
%     k = k + 1;
% 
% %     x1 = x1 + 0.3;
%     m = m+1;
%     
% end

% plot([-20 + x1 3 + x1],[0 0],'k', 'LineWidth', 2) % ground line
% plot(com(:,1), com(:,2),'-.', 'Color',[0.8500, 0.3250, 0.0980] , 'LineWidth', 1.5)

% % pbaspect([2.5 1 1])
% pbaspect([1.2*(3.5+0.5)/(1.7+0.2) 1 1])
% set(gcf, 'color', 'none');
% set(gca, 'color', 'none');

pbaspect([(x_end - x_start) (y_end - y_start)*0.8 1])

if f_print == 1
    set(gcf, 'color', 'none');
    set(gca, 'color', 'none');
    print(gcf,'fig_snap_shots.png','-dpng','-r300');
end

end