%1- Jacobian matrix of the robot

% (Using d as partial derivate)
% dp/dt= J*dtheta/dt
% It gives: J= 	df/dtheta
% Finaly, J= [[dx3/dtheta1 dx3/dtheta5][dy3/dtheta1 dy3/dtheta5]]

%2- Differentiation of the FKM

% Correction of the equations:
%  dx2/dtheta1= -a1*sin(theta1)
%  dx4/dtheta5= -a4*sin(thata5)

function J = jacobian(a1, a2, a3, a4, a5, t1, t5)
    addpath('./Tutorial_1 /');


    x1 = 0;
    y1 = 0;


    x2 = a1*cos(t1);
    y2 = a1*sin(t1);

    x4 = a4*cos(t5)-a5;
    y4 = a4*sin(t5);

    x5 = -a5;
    y5 = 0;

    P2 = [x2;y2];
    P4 = [x4;y4];

    P2Ph = (a2^2-a3^2 + norm(P4-P2)^2) / (2 * norm(P4-P2));
    Ph   = P2 + (P2Ph/norm(P2-P4)) * (P4 - P2);
    P3Ph = sqrt(a2^2 - (P2Ph)^2);

    x3 = Ph(1) + (P3Ph/norm(P2-P4)) * (y4 - y2);
    y3 = Ph(2) - (P3Ph/norm(P2-P4)) * (x4 - x2);

    P3 = [x3;y3];
    
   
  

    d = norm(P2-P4);
    b = norm(P2-Ph);
    h = norm(P3-Ph);

    del1_x2 = -a1*sin(t1);   %NOTE: THE AUTHOR FORGOT NEGATIVE SIGN IN THE PAPER
    del1_y2 = a1*cos(t1);
    del5_x4 = -a4*sin(t5);   %NOTE: THE AUTHOR FORGOT NEGATIVE SIGN IN THE PAPER
    del5_y4 = a4*cos(t5);

    del1_y4 = 0;
    del1_x4 = 0;
    del5_y2 = 0;
    del5_x2 = 0;

    del1_d = ((a4*cos(t5)-a5-a1*cos(t1))*(del1_x4-del1_x2) + (a4*sin(t5)-a1*sin(t1))*(del1_y4-del1_y2)) / d;
    del1_b = del1_d - (del1_d*(a2^2-a3^2+d^2))/(2*d^2);
    del1_h = -b*del1_b / h;

    del1_yh = del1_y2 + (del1_b*d-del1_d*b)/d^2 * (a4*sin(t5)-a1*sin(t1)) + b/d * (del1_y4 - del1_y2);
    del1_xh = del1_x2 + (del1_b*d-del1_d*b)/d^2 * (a4*cos(t5)-a5-a1*cos(t1)) + b/d * (del1_x4 - del1_x2);

    del1_y3 = del1_yh - h/d * (a4*cos(t5)-a5-a1*cos(t1)) - (del1_h*d - del1_d*h)/d^2 *(a4*sin(t5)-a1*sin(t1));
    del1_x3 = del1_xh + h/d * (a4*sin(t5)-a1*sin(t1)) + (del1_h*d - del1_d*h)/d^2 *(a4*cos(t5)-a5-a1*cos(t1));

    del5_d = ((a4*cos(t5)-a5-a1*cos(t1))*(del5_x4-del5_x2) + (a4*sin(t5)-a1*sin(t1))*(del5_y4-del5_y2)) / d;
    del5_b = del5_d - (del5_d*(a2^2-a3^2+d^2))/(2*d^2);
    del5_h = -b*del5_b / h;

    del5_yh = del5_y2 + (del5_b*d-del5_d*b)/d^2 * (a4*sin(t5)-a1*sin(t1)) + b/d * (del5_y4 - del5_y2);
    del5_xh = del5_x2 + (del5_b*d-del5_d*b)/d^2 * (a4*cos(t5)-a5-a1*cos(t1)) + b/d * (del5_x4 - del5_x2);

    del5_y3 = del5_yh - h/d * (a4*cos(t5)-a5-a1*cos(t1)) - (del5_h*d - del5_d*h)/d^2 *(a4*sin(t5)-a1*sin(t1));
    del5_x3 = del5_xh + h/d * (a4*sin(t5)-a1*sin(t1)) + (del5_h*d - del5_d*h)/d^2 *(a4*cos(t5)-a5-a1*cos(t1));

    J = [del1_x3 del5_x3; del1_y3 del5_y3];
end
