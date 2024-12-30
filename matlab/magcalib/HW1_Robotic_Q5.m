% Clear previous variables and commands
clear all;
clc;

% Define symbolic variables
syms t a b c d k real

% Define the columns of the rotation matrix R(t)
C1 = [cos(t); sin(t); 0];
C2 = [a; (k*cos(t))/sqrt(2); (-k*sin(t))/sqrt(2)];
C3 = [b; c; d];

% Rotation matrix properties:
% 1. Columns are orthonormal: C_i^T * C_j = delta_ij
% 2. Determinant of R is 1

% Orthonormality conditions:
% C1^T * C1 = 1 (Already satisfied since cos^2(t) + sin^2(t) = 1)

% Equation 1: C2^T * C2 = 1
E1 = a^2 + ((k*cos(t))/sqrt(2))^2 + ((-k*sin(t))/sqrt(2))^2 == 1;

% Simplify Equation 1
E1 = simplify(E1);

% Equation 2: C1^T * C2 = 0
E2 = cos(t)*a + sin(t)*(k*cos(t))/sqrt(2) + 0*((-k*sin(t))/sqrt(2)) == 0;

% Simplify Equation 2
E2 = simplify(E2);

% Solve Equation 2 for a
a_sol = solve(E2, a);

% Substitute a into Equation 1
E1_sub = subs(E1, a, a_sol);

% Solve for k
k_sol = solve(E1_sub, k);

% Choose the positive root for k
k_sol = simplify(k_sol(1));

% Substitute k back into a
a_sol = simplify(subs(a_sol, k, k_sol));

% Now compute b, c, d using the cross product (since C3 = C1 x C2)
C3_cross = simplify(cross(C1, C2));

% Equate components of C3 and C3_cross
E_b = b == C3_cross(1);
E_c = c == C3_cross(2);
E_d = d == C3_cross(3);

% Substitute k and a into C3 components
b_sol = simplify(subs(E_b, [a, k], [a_sol, k_sol]));
c_sol = simplify(subs(E_c, [a, k], [a_sol, k_sol]));
d_sol = simplify(subs(E_d, [a, k], [a_sol, k_sol]));

% Display the solutions
disp('Expressions for the unknown functions:')
disp('---------------------------------------')
disp('a(t) = ')
pretty(a_sol)
disp(' ')
disp('b(t) = ')
pretty(b_sol)
disp(' ')
disp('c(t) = ')
pretty(c_sol)
disp(' ')
disp('d(t) = ')
pretty(d_sol)
disp(' ')
disp('k(t) = ')
pretty(k_sol)
