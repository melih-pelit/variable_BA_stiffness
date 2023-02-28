function [step_alpha_ref, step_inputs_ref] = calc_step_input_ref(ocl_traj)
inputs_ref = ocl_traj.input_Torques;
time_steps = ocl_traj.time;

alpha_ref = pi - (2*ocl_traj.simout(:,1) + ocl_traj.simout(:,2))/2;

alpha = alpha_ref(1):0.001:alpha_ref(end);
k = 1;
for i=1:length(alpha)
    step_alpha_ref(i,:) = alpha(i);
    if alpha(i) <= alpha_ref(k+1)
        step_inputs_ref(i,:) = inputs_ref(k,:);
    else
        k=k+1;
        step_inputs_ref(i,:) = inputs_ref(k,:);
    end
end

end