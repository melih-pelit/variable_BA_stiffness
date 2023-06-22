function plot_5Link_ba_stiffness(time, U_varStiff_BA, flag, params, flag_print, varargin)
% TODO
line_width = 1.5;
font_size = 14;

fig = figure();
fig.Position = [100 100 800 800]; % make the figure spawn larger
subplot(3,1,1:2)
plot(time, params.k_ba + squeeze(U_varStiff_BA), "LineWidth", line_width)
legend( ...
    "$k_{0, \textrm{ba}} + u_{\textrm{ba}}^{\textrm{st}}$", ...
    "$k_{0, \textrm{ba}} + u_{\textrm{ba}}^{\textrm{sw}}$", ...
    'Interpreter','latex', "FontSize", font_size)
ylabel('[N/m]')
subplot(3,1,3)
plot(time, flag(:,1), "LineWidth", line_width)
xlabel("Time [sec]", "FontSize", font_size)
ylabel("Walking Phase", "FontSize", font_size)

if flag_print
    if nargin >= 5
        export_fig("figures\5Link_ba_stiffness" + varargin{1}, '-m3')
    else
        export_fig("figures\5Link_ba_stiffness", '-m3')
    end
end
end