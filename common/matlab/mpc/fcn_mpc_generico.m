function u = fcn(r, x)
%% ============================================================
%  Bloque MATLAB Function GENERICO para MPC (QP con quadprog)
%  Control de Procesos - UTEC
%
%  >>> SOLO necesitas editar 3 cosas: Ad, Bd y N <<<
%
%  Todo lo demas (numero de estados nx, numero de entradas nu,
%  matriz de pesos Q, restricciones y las matrices del QP) se
%  calcula AUTOMATICAMENTE a partir del tamano de Ad y Bd.
%
%  Funciona para cualquier numero de estados (2, 3, 4, 5, ...) y
%  cualquier horizonte N sin tener que reescribir el codigo.
%
%  Entradas del bloque:
%     r : referencia/consigna del estado  (vector nx x 1)
%     x : estado actual medido            (vector nx x 1)
%  Salida:
%     u : primera accion de control optima (escalar, u = z(1))
% ============================================================

% Forzar a Simulink a tratar quadprog/optimoptions como externas
coder.extrinsic('quadprog', 'optimoptions');

%% ------------------------------------------------------------
%  1) PARAMETROS A EDITAR POR EL USUARIO
%  ------------------------------------------------------------
N = 5;          % <-- HORIZONTE DE PREDICCION (1 a 10, el que quieras)

% Espacio de Estados discreto: x[k+1] = Ad x[k] + Bd u[k]
% Cambia SOLO estas dos matrices para tu planta; el resto es automatico.
Ad = [1.0000    0.1000;
      0         1.0000];
Bd = [0.0050;
      0.1000];

% Pesos del MPC (opcional ajustarlos)
q_primer_estado = 10;   % peso del 1er estado (el que sigue la referencia)
q_resto         = 1;    % peso de los demas estados
R_peso          = 0.1;  % peso del esfuerzo de control

% Restricciones (iguales para todos los estados/entradas)
umax = 100;     % |u|   <= umax
xmax = 100;     % |x_i| <= xmax  para todo estado

%% ------------------------------------------------------------
%  2) DIMENSIONES AUTOMATICAS
%  ------------------------------------------------------------
nx = size(Ad, 1);    % numero de estados  (automatico)
nu = size(Bd, 2);    % numero de entradas (automatico)
I  = eye(nx);

%% ------------------------------------------------------------
%  3) PESOS Q y R (automaticos segun nx)
%  ------------------------------------------------------------
% Q = diag([q_primer_estado, q_resto, q_resto, ...])  -> tamano nx x nx
q_diag = q_resto * ones(nx, 1);
q_diag(1) = q_primer_estado;
Q = diag(q_diag);

R = R_peso * eye(nu);

%% ------------------------------------------------------------
%  4) CONSTRUCCION DE MATRICES MPC (igual que tus codigos, generico)
%  ------------------------------------------------------------
% Vector de optimizacion: z = [U ; X] = [u_0..u_{N-1} ; x_1..x_N]
H = 2 * blkdiag( kron(eye(N), R), kron(eye(N), Q) );

f_u = zeros(N*nu, 1);
f_x = repmat(-2 * Q * r, N, 1);
f   = [f_u; f_x];

Ain = []; bin = [];

% Restricciones de igualdad: dinamica del sistema
Aeq_u = kron(eye(N), -Bd);
Aeq_x = kron(eye(N), I) - kron(diag(ones(N-1,1), -1), Ad);
Aeq   = [Aeq_u, Aeq_x];

beq = [Ad*x; zeros((N-1)*nx, 1)];

%% ------------------------------------------------------------
%  5) LIMITES (automaticos segun nx y N)
%  ------------------------------------------------------------
lb_u = repmat(-umax, N*nu, 1);
ub_u = repmat( umax, N*nu, 1);

lim_x_min = -xmax * ones(nx, 1);   % automatico para cualquier nx
lim_x_max =  xmax * ones(nx, 1);

lb = [lb_u; repmat(lim_x_min, N, 1)];
ub = [ub_u; repmat(lim_x_max, N, 1)];

%% ------------------------------------------------------------
%  6) RESOLUCION DEL QP
%  ------------------------------------------------------------
z0 = zeros(N*nu + N*nx, 1);
z  = zeros(N*nu + N*nx, 1);   % inicializacion fija para el coder

options = optimoptions('quadprog', 'Algorithm', 'active-set', 'Display', 'off');
z = quadprog(H, f, Ain, bin, Aeq, beq, lb, ub, z0, options);

u = z(1);   % primera accion de control
end
