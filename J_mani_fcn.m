function [J_mani] = J_mani_fcn(qi)
    
% Definición de las matrices de transformación homogénea
    di = [0.290, 0, 0, 0.302, 0, 0.072]; % Desplazamiento en z
    ai = [0, 0.270, 0.070, 0, 0, 0];     % Desplazamiento en x
    alphai = [-pi/2, 0, -pi/2, pi/2, -pi/2, 0]; % Ángulos alpha
    qi = [0.5, 0.3, -0.1, 0.2, -0.4, 0.6]; % Ángulos theta (valores de ejemplo)
    
    % Calcular las matrices de transformación para cada eslabón
    T01 = despZ(di(1)) * rotZ(qi(1)) * despX(ai(1)) * rotX(alphai(1));
    T12 = despZ(di(2)) * rotZ(qi(2)) * despX(ai(2)) * rotX(alphai(2));
    T23 = despZ(di(3)) * rotZ(qi(3)) * despX(ai(3)) * rotX(alphai(3));
    T34 = despZ(di(4)) * rotZ(qi(4)) * despX(ai(4)) * rotX(alphai(4));
    T45 = despZ(di(5)) * rotZ(qi(5)) * despX(ai(5)) * rotX(alphai(5));
    T56 = despZ(di(6)) * rotZ(qi(6)) * despX(ai(6)) * rotX(alphai(6));

    
    %T01 computed before.
    T02=(T01*T12);
    T03=(T02*T23);
    T04=(T03*T34);
    T05=(T04*T45);
    T06=(T05*T56);
    
    %joint vectors (zi)
    zL=[0 0 1]';
    z0=zL;
    %%%%% PUT YOUR CODE HERE %%%%%%
    % Complete from z1 to z5
    z1 = T01(1:3,1:3) * zL;
    z2 = T02(1:3,1:3) * zL;
    z3 = T03(1:3,1:3) * zL;
    z4 = T04(1:3,1:3) * zL;
    z5 = T05(1:3,1:3) * zL;
    
    %position vectors (pi); they are the position part of the homogeneus
    %transformation matrices (position of the origin of the reference frame)
    p0=[0 0 0]';
    %%%%% PUT YOUR CODE HERE %%%%%%
    % Complete from p1 to p6
    p1 = T01(1:3, 4);
    p2 = T02(1:3, 4);
    p3 = T03(1:3, 4);
    p4 = T04(1:3, 4);
    p5 = T05(1:3, 4);
    p6 = T06(1:3, 4);
    
    %Columns according to the algorithm (J=[JP;JO]); the left side with the three
    %first elements of each pi vector and the right side with the z_i vector
    J1=([cross(z0,(p6-p0));z0]);
    J2=([cross(z1,(p6-p1));z1]);
    J3=([cross(z2,(p6-p2));z2]);
    J4=([cross(z3,(p6-p3));z3]);
    J5=([cross(z4,(p6-p4));z4]);
    J6=([cross(z5,(p6-p5));z5]);
    
    J_mani=double(([J1 J2 J3 J4 J5 J6]));
end

