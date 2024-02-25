% inverse kinematics of the device based on link lengths and end-effector
% position

%3 - Robot IKM:

%theta= f⁻¹(p) 

%4- No, the equations are not correct.

%theta1 = pi- alpha1- beta1
%theta5= alpha5+ beta5 

%In those formulas, alpha1 and beta5 are not correctly defined.
%Correction:
%alpha1 = arccos((a1²- a2² + ||P1,P3||²)/(2*a*||P1,P3||))
%beta5= arccos((a4²-a3²+||P5,P3||)/(2*a4*||P5, P3||))

%(alpha5 and beta1 are correctly defined)

function t = INV(a1,a2,a3,a4,a5,x3,y3)

P13 = sqrt(x3.^2+y3.^2);
P53 = sqrt((x3+a5).^2 + y3.^2);

alphaOne = acos((a1.^2 + P13.^2 - a2.^2)./(2.*a1.*P13));
betaOne = atan2(y3,-x3);
thetaOne = pi - alphaOne - betaOne;

alphaFive = atan2(y3,x3+a5);
betaFive = acos((P53.^2 + a4.^2 - a3.^2)./(2.*P53.*a4));
thetaFive = alphaFive + betaFive;


t = [thetaOne thetaFive];