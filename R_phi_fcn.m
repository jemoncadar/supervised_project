function [R_phi] = R_phi_fcn(phi)
    c_phi = cos(phi);
    s_phi = sin(phi);
    R_phi = [
        c_phi, -s_phi, 0,     0,     0,     0;
        s_phi,  c_phi, 0,     0,     0,     0;
        0,      0,     1,     0,     0,     0;
        0,      0,     0,  c_phi, -s_phi,  0;
        0,      0,     0,  s_phi,  c_phi,  0;
        0,      0,     0,     0,     0,     1;
    ];
end

