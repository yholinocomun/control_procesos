function u = mpc_qp_step(r, x, Ad, Bd, N, varargin)
%MPC_QP_STEP  Un paso de MPC por QP, totalmente parametrico.
%
%  Resuelve el MPC para CUALQUIER numero de estados y CUALQUIER
%  horizonte. Solo le pasas las matrices de estado (Ad, Bd) y el
%  horizonte N; el numero de estados y entradas se deduce solo.
%
%  USO BASICO:
%     u = mpc_qp_step(r, x, Ad, Bd, N);
%
%  USO CON OPCIONES (pares nombre-valor):
%     u = mpc_qp_step(r, x, Ad, Bd, N, ...
%                     'q1', 10, 'qrest', 1, 'R', 0.1, ...
%                     'umax', 100, 'xmax', 100);
%
%  ENTRADAS
%     r   : referencia del estado          (nx x 1)
%     x   : estado actual medido           (nx x 1)
%     Ad  : matriz de estados discreta     (nx x nx)
%     Bd  : matriz de entradas discreta    (nx x nu)
%     N   : horizonte de prediccion        (escalar)
%
%  OPCIONES (nombre-valor, todas con valor por defecto)
%     'q1'    peso del primer estado          (def 10)
%     'qrest' peso de los demas estados       (def 1)
%     'R'     peso del esfuerzo de control     (def 0.1)
%     'umax'  cota de la entrada |u|<=umax     (def 100)
%     'xmax'  cota de los estados |x_i|<=xmax  (def 100)
%
%  SALIDA
%     u   : primera accion de control optima  (escalar)

% ---- valores por defecto y parseo de opciones ----
opt.q1    = 10;
opt.qrest = 1;
opt.R     = 0.1;
opt.umax  = 100;
opt.xmax  = 100;
for k = 1:2:numel(varargin)
    opt.(varargin{k}) = varargin{k+1};
end

% ---- dimensiones automaticas ----
nx = size(Ad, 1);
nu = size(Bd, 2);
I  = eye(nx);

% ---- pesos automaticos segun nx ----
q_diag = opt.qrest * ones(nx, 1);
q_diag(1) = opt.q1;
Q = diag(q_diag);
R = opt.R * eye(nu);

% ---- matrices del QP: z = [U ; X] ----
H = 2 * blkdiag( kron(eye(N), R), kron(eye(N), Q) );

f = [zeros(N*nu, 1); repmat(-2 * Q * r, N, 1)];

Aeq_u = kron(eye(N), -Bd);
Aeq_x = kron(eye(N), I) - kron(diag(ones(N-1,1), -1), Ad);
Aeq   = [Aeq_u, Aeq_x];
beq   = [Ad*x; zeros((N-1)*nx, 1)];

% ---- limites automaticos ----
lb = [repmat(-opt.umax, N*nu, 1); repmat(-opt.xmax * ones(nx,1), N, 1)];
ub = [repmat( opt.umax, N*nu, 1); repmat( opt.xmax * ones(nx,1), N, 1)];

% ---- resolucion ----
z0 = zeros(N*nu + N*nx, 1);
options = optimoptions('quadprog', 'Algorithm', 'active-set', 'Display', 'off');
z = quadprog(H, f, [], [], Aeq, beq, lb, ub, z0, options);

u = z(1);
end
