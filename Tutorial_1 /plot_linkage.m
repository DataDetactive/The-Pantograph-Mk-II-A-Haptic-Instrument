

function plot_linkage(a1, a2, a3, a4, a5, theta1, theta5)
    
    p = forward_kinematics(a1, a2, a3, a4, a5, theta1, theta5);


    x1 = 0;
    y1 = 0;


    x2 = a1*cos(theta1);
    y2 = a1*sin(theta1);

    x4 = a4*cos(theta5)-a5;
    y4 = a4*sin(theta5);

    x5 = -a5;
    y5 = 0;
    
    x3 = p(1);
    y3 = p(2);
    plot(-[x1 x1],-[y1 y1],'o',-[x2 x2],-[y2 y2],'o',-[x3 x3],-[y3 y3],'o',-[x4 x4],-[y4 y4],'o',-[x5 x5],-[y5 y5],'o',-[x1 x2],-[x1 y2],-[x2 x3],-[y2 y3],-[x3 x4],-[y3 y4],-[x4 x5],-[y4 y5],-[x5 x1],-[y5 y1]);   
  
    axis equal;
    xlabel('X');
    ylabel('Y');
    title('Pantograph');
    grid on;
end


