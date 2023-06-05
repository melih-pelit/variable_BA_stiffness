function figures2(time, inputTorque)

f_print = 0;

figure()

rectangle2([10.05 0 0.1, 400] ,'FaceColor',[0.5882    0.5882    0.5882],'EdgeColor',[0.5882    0.5882    0.5882])
hold on

plot(time, inputTorque, 'LineWidth',1.5)
xlim([7,13])
ylim([-200,200])
lgd_font_size = 12;
legend({'dist. reg.','$u_{1}$ $[Nm]$', '$u_{2}$ $[Nm]$','$u_{3}$ $[Nm]$','$u_{4}$ $[Nm]$','$u_{5}$ $[Nm]$',},'Interpreter','latex', 'FontSize', lgd_font_size) % R2018a and earlier
box on
xlabel('time $[s]$', 'Interpreter','Latex', 'FontWeight','bold', 'FontSize', lgd_font_size);


if f_print == 1
    set(gcf, 'color', 'none');
    set(gca, 'color', 'none');
    % print(gcf,'V-SLIPSL_inputs.png','-dpng','-r300');
end

end
