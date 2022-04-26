syms x1 x2 x3 x4 u
% syms g rg L K tau
g = 9.81;
rg = .0254;
L  = .4255;
K = 1.5;
tau = .025;



dx1 = x2;
dx2 = 5*g/7*rg/L*sin(x3) - 5/7*(L/2-x1)*(rg/L)^2*x4^2*cos(x3)^2;
dx3 = x4;
dx4 = -x4/tau + K/tau*u;

dx = [dx1;dx2;dx3;dx4];
x = [x1;x2;x3;x4];

g = diff(dx,u);
f = simplify(dx - g*u);

%% Linearize about origin



A = jacobian(dx,x);
B = jacobian(dx,u);

% Evaluate at origin

A = double(subs(A,[x;u],zeros(5,1)));
B = double(subs(B,[x;u],zeros(5,1)));

%% Sontag Control
%Solve ARE 
Q = diag([500 0 50 0]);

P = icare(A,B,Q,3);

V = x.'*P*x; %CLF

%Lie derivatives
Lf = jacobian(V,x)*f;
Lg = jacobian(V,x)*g;

%Sontag controller
as = (-(Lf + sqrt(Lf^2 + Lg^4)))/Lg;
vpa(as,3)