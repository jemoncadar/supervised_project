function [J_base] = J_base_fcn(phi)
    
    LB = 0;
    r = 0.165;
    a = 0.560/2;

    J_base = [
        (r/2)*cos(phi) + (r*LB/(2*a))*sin(phi), (r/2)*cos(phi) - (r*LB/(2*a))*sin(phi);
        (r/2)*sin(phi) - (r*LB/(2*a))*cos(phi), (r/2)*sin(phi) + (r*LB/(2*a))*cos(phi);
        -r/(2*a), r/(2*a)
    ];
end

