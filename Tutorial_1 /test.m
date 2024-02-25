% Test script for the forward kinematics function and Inverse kinematics
%6- As the FKM and the IKM are linked, the best method would be:
%- FKM: impose theta to have p
%- IKM: take the results of p obtained the FKM
%- Check: check if theta given in the IKM is close to theta imposed %in the FKM.
a1 = 63; a2 = 75; a3 = 75; a4 = 63; a5 = 25; t1 = 0.7; t5 = 2.09;
p = forward_kinematics(a1, a2, a3, a4, a5, t1, t5);
disp('Forward Kinematics:');
disp(p);

T = INV(a1, a2, a3, a4, a5, p(1), p(2));
disp('Inverse Kinematics:');
disp(T);

% Test script for plotting function
plot_linkage(a1, a2, a3, a4, a5, t1,t5)
