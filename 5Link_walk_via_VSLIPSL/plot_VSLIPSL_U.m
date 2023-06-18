function plot_VSLIPSL_U(time,input_varStiff, flag, dc)

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

figure()

nominal = [
    dc.col_param.k0_ss; 
    dc.col_param.k_swLeg; 
    dc.col_param.k_swFoot; 
    dc.col_param.k0_ds; 
    dc.col_param.k0_ds];
labels = ["k_1"; "k_2"; "k_3"; "k_4"; "k_5"];

for i = 1:5
    subplot(5,1,i)
    % plot(time, nominal(i) + input_varStiff(:,i))
    plot(time, input_varStiff(:,i))
    hold on
    % plot(time, nominal(i)*ones(length(time),1))
    grid on
    % vline(time(state_change_idx),'r')
    xlim([t_start, t_end])
    ylabel(labels(i,:))
end

sgtitle('VSLIPSL U_{var. stiff}') 