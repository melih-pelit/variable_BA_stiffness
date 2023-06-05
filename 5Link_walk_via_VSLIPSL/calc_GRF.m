function  [ddq_GRF, GRF, U_test] = calc_GRF(t, simout, param, flag, inputTorque)
% [ddq_GRF, GRF] = calc_GRF(time, simout, param, flag, inputTorque)

ddq_GRF = zeros(length(t), 7);
GRF = zeros(length(t), 4);
for i = 1:length(t)
    [ddq_GRF_dummy, GRF_dummy] = EoM_floating(t(i), simout(i,:), param, flag(i,:), transpose(inputTorque(i,:)));
    
    ddq_GRF(i,:) = ddq_GRF_dummy;
    GRF(i,:) = GRF_dummy;
    
    U_test(:,i) = ZMP_control(t(i), simout(i,:), param, flag(i,:), transpose(inputTorque(i,:)));
end

U_test = U_test';
end