function u = fcn(r, x)
% Modelo discreto de la planta (EJEMPLO 5x5 - reemplazar por el real)
Ad = [ 1.0000   0.5000   0        0        0
       0        1.0000   0.5000   0        0
       0        0        1.0000   0.5000   0
       0        0        0        1.0000   0.5000
       0        0        0        0        1.0000];
Bd = [ 0
       0
       0
       0
       1.0000];
% Parametros MPC
N = 1; nx = 5; nu = 1;
Q = diag([75, 1, 1, 1, 1]); R = 5;
% Limites fisicos
u1max = 50; x1max = 50; x2max = 50; x3max = 50; x4max = 50; x5max = 50;
% Matrices de ceros e identidad
In = eye(nx); Zn = zeros(nx); Zm = zeros(nx, nu);
% Matrices del optimizador QP
H = 2*blkdiag(R, Q); % Hessiana
f = -2*[0; Q*r]; % Vector lineal
Ain = []; bin = []; % Desigualdad vacia
% Restricciones de igualdad dinamica
Aeq = [-Bd, In;];
beq = [Ad*x]; % Lado derecho
% Cotas de caja
lb_x = [-x1max; -x2max; -x3max; -x4max; -x5max];
ub_x = [ x1max;  x2max;  x3max;  x4max;  x5max];
lb = [-u1max; lb_x]; % Limite inferior
ub = [u1max; ub_x]; % Limite superior
% Ejecucion de quadprog
z0 = zeros(N*(nx+nu),1);
options = optimoptions('quadprog', 'Algorithm','active-set', 'Display', 'off');
z = quadprog(H,f,Ain,bin,Aeq,beq,lb,ub,z0,options);
u = z(1); % Primera accion de control
