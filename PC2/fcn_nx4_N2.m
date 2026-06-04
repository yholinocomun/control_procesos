function u = fcn(r, x)
% Modelo discreto del pendulo
Ad = [ 1.0000  0.5000  -0.2763  -0.0360
       0       1.0000  -1.7995  -0.2763
       0       0        9.3349   1.5871
       0       0       54.2766   9.3349];
Bd = [ 0.1783
       0.7956
       -0.9891
       -6.4410];
% Parametros MPC
N = 2; nx = 4; nu = 1;
Q = diag([75, 1, 1, 1]); R = 5;
% Limites fisicos
u1max = 50; x1max = 50; x2max = 50; x3max = pi; x4max = 50;
% Matrices de ceros e identidad
In = eye(nx); Zn = zeros(nx); Zm = zeros(nx, nu);
% Matrices del optimizador QP
H = 2*blkdiag(R, R, Q, Q); % Hessiana
f = -2*[0; 0; Q*r; Q*r]; % Vector lineal
Ain = []; bin = []; % Desigualdad vacia
% Restricciones de igualdad dinamica
Aeq = [-Bd, Zm, In, Zn;
Zm, -Bd, -Ad, In;];
beq = [Ad*x; zeros(nx,1)]; % Lado derecho
% Cotas de caja
lb_x = [-x1max; -x2max; -x3max; -x4max];
ub_x = [ x1max;  x2max;  x3max;  x4max];
lb = [-u1max; -u1max; lb_x; lb_x]; % Limite inferior
ub = [u1max; u1max; ub_x; ub_x]; % Limite superior
% Ejecucion de quadprog
z0 = zeros(N*(nx+nu),1);
options = optimoptions('quadprog', 'Algorithm','active-set', 'Display', 'off');
z = quadprog(H,f,Ain,bin,Aeq,beq,lb,ub,z0,options);
u = z(1); % Primera accion de control
