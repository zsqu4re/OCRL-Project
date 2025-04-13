% rad
syms phi1 phi2 phi3 phi4;
syms phi_1_dot phi_4_dot;
% m
l1 = 0.105;
l2 = 0.05;
l4 = l1;
l3 = l2;
L = 0.06;
% calculate the coord of B,D
xb = l1 * cos(phi1);
yb = l1 * sin(phi1);
xd = L + l4 * cos(phi4);
yd = l4 * sin(phi4);

% calculate phi2 
A0 = 2 * l2 * (xd - xb);
B0 = 2 * l2 * (yd - yb);
lBD = sqrt((xd - xb)^2 + (yd - yb)^2);
C0 = l2^2 + lBD^2 - l3^2;
phi2 = 2 * atan((B0 + sqrt(A0^2 + B0^2 - C0^2))/(A0 + C0));

xc = l1 * cos(phi1) + l2 * cos(phi2);
yc = l1 * sin(phi1) + l2 * sin(phi2);

% calculate L0 (distance between robot to land) and its angle phi0
% robot pos
L0 = sqrt(yc^2 + (xc - L/2)^2);
phi0 = asin(yc/L0);

% jacobian
% this is for determine the change of phi1 and phi4 and the feedback of L0
% and phi0
% x = f(q)
% x = [L0; phi0]
% q = [phi1; phi4]
x1 = simplify(L0);
x2 = simplify(phi0);
q = [phi1;phi4];
J = jacobian([x1;x2],q);
