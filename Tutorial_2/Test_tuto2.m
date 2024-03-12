
%4- J=df(theta)/dtheta
% We could induce a little change in the angle delta(theta)
% And compare J to delta(p)/delta(theta)= (f(theta+ delta(theta))- f(theta))/delta(theta)


%6 tau= transpose(J)*f
% Explanation: 
% Virtual work:
% A system in equilibrum have zero virtual work
% It is expressed as delta_w=sum((tau_i)transpose*delta_q)=0, where delta_q corresponds to the joint displacement
% Formula
% To represent forces at the robot's end-effector, a Cartesian force is used, f.
% The relationship tau= transpose(J)*f is established
% Why virtual environment:
% Working with working has limitations so we consider this environment
% The formula comes from the necessity to ensure zero virtual work in a multibody system, linking Cartesian forces to joint torques.

addpath('./Tutorial_1 /');
% a1 = 63; a2 = 75; a3 = 75; a4 = 63; a5 = 25; t1 = 0.5; t5 = 0.6;epsilon=rand(1)*0.01
a1 = 0.25; a2 = 0.25; a3 = 0.25; a4 = 0.25; a5 = 0.1; t1 = 0.7; t5 = 0.7;epsilon=rand(1)*0.01;
vec_epsilon = rand(1,2)*0.01;


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
% delta_theta = [epsilon; epsilon];% Perturbation vector
delta_theta = [epsilon,epsilon];
delta_p_actual = Jac * delta_theta';

% Compare the changes in end-effector position
% difference = norm(delta_p_expected - delta_p_actual);
disp("FKM(theta+delta theta)-FKM(theta)")
disp(delta_p_expected)

 disp("J*delta theta= " )
disp(delta_p_actual)

% Display the difference
% disp(['Difference between expected and actual change in end-effector position: ' num2str(difference)]);


