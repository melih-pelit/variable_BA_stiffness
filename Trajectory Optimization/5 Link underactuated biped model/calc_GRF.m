function  [ddq_GRF, GRF] = calc_GRF(t, simout, param, flag, inputTorque)
% [ddq_GRF, GRF] = calc_GRF(time, simout, param, flag, inputTorque)

ddq_GRF = zeros(length(t), 7);
GRF = zeros(length(t), 4);
for i = 1:length(t)
    [ddq_GRF_dummy, GRF_dummy] = EoM_floating(t(i), simout(i,:), param, flag(1,:), transpose(inputTorque(i,:))); 
    % please note that flags index is set to one since in the single phase OpenOCL case flag is constant
    
    ddq_GRF(i,:) = ddq_GRF_dummy;
    GRF(i,:) = GRF_dummy;
end

end