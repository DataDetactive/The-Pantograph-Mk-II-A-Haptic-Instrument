
a1 = 63; a2 = 75; a3 = 75; a4 = 63; a5 = 25; t1 = 0.5; t5 = 0.6;epsilon=0.001;
% result= jacobian(a1, a2, a3, a4, a5, t1, t5);

% Compute the forward kinematics result for the original joint angles
p_original = forward_kinematics(a1, a2, a3, a4, a5, t1, t5);

% Compute the forward kinematics result for perturbed joint angles
t1_perturbed = t1 + epsilon;
t5_perturbed = t5 + epsilon;

p_perturbed = forward_kinematics(a1, a2, a3, a4, a5, t1_perturbed, t5_perturbed);

% Compute the change in end-effector position predicted by the forward kinematics solution
delta_p_expected = p_perturbed - p_original;

% Compute the change in end-effector position predicted by the Jacobian matrix
Jac = jacobian(a1, a2, a3, a4, a5, t1, t5);
delta_theta = [epsilon; epsilon]; % Perturbation vector
delta_p_actual = Jac * delta_theta;

% Compare the changes in end-effector position
difference = norm(delta_p_expected - delta_p_actual);

% Display the difference
disp(['Difference between expected and actual change in end-effector position: ' num2str(difference)]);

