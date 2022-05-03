syms x1 x2 x3 x4 u
syms g rg L K tau
% g = 9.81;
% rg = .0254;
% L  = .4255;
% K = 1.5;
% tau = .025;



dx1 = x2;
dx2 = 5*g/7*rg/L*sin(x3) - 5/7*(L/2-x1)*(rg/L)^2*x4^2*cos(x3)^2;
dx3 = x4;
dx4 = -x4/tau + K/tau*u;

dx = [dx1;dx2;dx3;dx4];
x = [x1;x2;x3;x4];

gx = diff(dx,u);
fx = simplify(dx - gx*u);

%% Feedforward

syms a_ball_ref
ff = subs(dx2,x4,0);
solve(ff == a_ball_ref,x3)

%% Linearize about origin

Asym = jacobian(dx,x);
Bsym = jacobian(dx,u);
