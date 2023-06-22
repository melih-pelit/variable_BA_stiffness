function plot_tracking_jointAngles(simout, flag, time, des_th, flag_print, varargin)

%% Calculations
% detect state change
flag_prev = flag(1,1);
state_change_idx = [];
for i = 2:length(flag(:,1))
    flag_cur = flag(i,1);
    if flag_cur ~= flag_prev
        state_change_idx(end + 1) = i;
    end
    flag_prev = flag_cur;
end

% TODO: displace state change

%% plotting
if nargin >= 7
    font_size = varargin{2};
else
    font_size = 14;
end
line_width = 1.5;

x_start = 0;
x_end = time(end);

fig = figure();
fig.Position = [100 100 800 800]; % make the figure spawn larger
for i = 1:5
    subplot(5,1,i);
    plot(time, mod(des_th(:,i),2*pi), "LineWidth", line_width); hold on;
    plot(time, mod(simout(:,i),2*pi), "LineWidth", line_width);
    ylabel(sprintf("$\\theta_%d$ [rad]", i), "Interpreter","latex", "FontSize", font_size)
    % plot(time, flag(:,1))
    grid on;
    xlim([x_start, x_end])

    if flag_print
        set(gcf, 'color', 'none');
        set(gca, 'color', 'none');
    end
end

if flag_print
    if nargin >= 6
        export_fig("figures\fig_trackingPlots_jointAngles" + varargin{1}, '-m3')
    else
        export_fig("figures\fig_trackingPlots_jointAngles", '-m3')
    end
end

end