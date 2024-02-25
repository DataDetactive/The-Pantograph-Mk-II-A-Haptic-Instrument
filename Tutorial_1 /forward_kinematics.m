%Tuto 1- Kinematic models of the pantograph

%1- Robot's FKM

%p= f(theta), with
%	p=transpose([x3, y3])
%	thet= transpose([theta1, theta3])

%2- Yes, and the equations are correct

function p = forward_kinematics(a1, a2, a3, a4, a5, t1, t5)
    x1 = 0;
    y1 = 0;

    x2 = a1 * cos(t1);
    y2 = a1 * sin(t1);

    x4 = a4 * cos(t5) - a5;
    y4 = a4 * sin(t5);

    x5 = -a5;
    y5 = 0;

    P2 = [x2; y2];
    P4 = [x4; y4];

    P2Ph = (a2^2 - a3^2 + norm(P4 - P2)^2) / (2 * norm(P4 - P2));
    Ph = P2 + (P2Ph / norm(P2 - P4)) * (P4 - P2);
    P3Ph = sqrt(a2^2 - (P2Ph)^2);

    x3 = Ph(1) + (P3Ph / norm(P2 - P4)) * (y4 - y2);
    y3 = Ph(2) - (P3Ph / norm(P2 - P4)) * (x4 - x2);

    p = [x3; y3];
end
