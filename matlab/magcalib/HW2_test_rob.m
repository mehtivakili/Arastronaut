% Define symbolic variables
syms theta phi alpha psi d l_1234

% Define cosines and sines with subscripts for clarity
c_theta = cos(theta);
s_theta = sin(theta);
c_phi = cos(phi);
s_phi = sin(phi);
s_psi = sin( psi); % beta = alpha + psi
c_psi = cos( psi); % beta = alpha + psi
s_alpha = sin(alpha);
c_alpha = cos(alpha);
d_z= [0; 0; d];
l_1234_z = [0; 0; l_1234];

% Define the rotation matrices
R_y_theta = [ c_theta,    0,  s_theta;
             0,          1,       0;
            -s_theta,    0,  c_theta ];

R_z_phi = [ c_phi, -s_phi, 0;
           s_phi,  c_phi, 0;
              0,      0, 1 ];

R_y_alpha = [ c_alpha,    0,  s_alpha;
              0,      1,      0;
            -s_alpha,    0,  c_alpha ];

R_y_psi = [ c_psi,    0,  s_psi;
              0,      1,      0;
            -s_psi,    0,  c_psi ];
% Multiply the rotation matrices
% First compute R_z(phi) * R_y(beta)
A = R_y_psi * R_y_alpha;

% Then multiply by R_z(phi) again (as per the expression)
B = R_y_theta * R_z_phi;

% Finally, multiply by R_y(theta)
P_a_d =  B * A* d_z;

% Simplify the resulting matrix
P_a_d = simplify(P_a_d)

% Display the resulting matrix
%'The resulting rotation matrix P a/d is:');
%disp(P_a_d);

P_a=R_y_theta * R_z_phi * l_1234_z;
P_a = simplify(P_a)

P_d = P_a + P_a_d;
P_d = simplify(P_d);

disp('The resulting rotation matrix P d is:');
disp(P_d);
