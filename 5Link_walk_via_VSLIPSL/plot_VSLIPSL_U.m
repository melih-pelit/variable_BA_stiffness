function plot_VSLIPSL_U(time,input_varStiff, flag, dc, flag_print, varargin)

% find the flag change indexes
flag_prev = 1;
state_change_idx = [];
for i = 1:length(time)
    if flag_prev ~= flag(i,1)
        state_change_idx(end+1) = i;
        flag_prev = flag(i,1);
    end
end

t_start = 0;
t_end = time(end);

if nargin >= 7
    font_size = varargin{2};
else
    font_size = 14;
end
line_width = 1.5;

fig = figure();
fig.Position = [100 100 800 800]; % make the figure spawn larger

nominal = [
    dc.col_param.k0_ss; 
    dc.col_param.k_swLeg; 
    dc.col_param.k_swFoot; 
    dc.col_param.k0_ds; 
    dc.col_param.k0_ds];
labels = {
    "$\bar{u}_1$ [N/m]"; 
    "$\bar{u}_2$ [Nm/rad]"; 
    "$\bar{u}_3$ [N/m]"; 
    "$\bar{u}_4$ [N/m]"; 
    "$\bar{u}_5$ [N/m]"};

for i = 1:5
    subplot(5,1,i)
    % plot(time, nominal(i) + input_varStiff(:,i))
    plot(time, input_varStiff(:,i), "LineWidth", line_width)
    hold on
    % plot(time, nominal(i)*ones(length(time),1))
    grid on
    % vline(time(state_change_idx),'r')
    xlim([t_start, t_end])
    ylabel(labels(i,:), "Interpreter","latex", "FontSize", font_size)
end

sgtitle('VSLIPSL U_{var. stiff}') 

if flag_print
    if nargin >= 5
        export_fig("figures\VSLIPSL_U" + varargin{1}, '-m3')
    else
        export_fig("figures\VSLIPSL_U", '-m3')
    end
end