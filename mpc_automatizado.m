function u = fcn(r, x)

% Para que quadprog funcione dentro del bloque MATLAB Function de Simulink:
% (Descomenta la línea siguiente si usas Simulink Coder para generación de código)
% coder.extrinsic('quadprog', 'optimoptions');

% ====================== PARÁMETROS A CAMBIAR ======================
N = 5;     % <-- Horizonte de Prediccion (1, 2, 3, ...)
% ===================================================================

% Espacio de Estado de la Planta (Modelo)
Ad = [     1.0000         0    0.1058    0.0051
           0    1.0000    0.0560    0.1000
           0         0    1.1759    0.1058
           0         0    3.6183    1.1759];

Bd = [     0.0060
      -0.0201
       0.1165
      -0.2009];

% ============== PESOS Y RESTRICCIONES (AUTOMÁTICAS) ===============
% Número de estados y controles (se deducen de Ad y Bd)
nx = size(Ad, 1);   % número de estados
nu = size(Bd, 2);   % número de controles

% Matriz de penalización de error (OPCIÓN 1: pesos decrecientes)
% Q = diag([10, 5, 3, 1, 0.5, ...]) para los primeros nx estados
Q_weights = 10 ./ (1:nx)';  % [10, 5, 3.33, 2.5, ...] para nx estados
Q = diag(Q_weights);

% (OPCIÓN 2: todos los estados con igual peso - descomenta si lo prefieres)
% Q = diag(ones(nx, 1) * 10);

% Peso de control
R = 0.1;

% Límites de control y estados (automáticos)
umax = 100;
xmax = ones(nx, 1) * 100;  % [100, 100, 100, 100, ...]

% ==================== CONSTRUCCIÓN AUTOMÁTICA ====================
% Matrices para restricciones
I = eye(nx);         % Identidad para los Estados
Z = zeros(nx);       % Ceros para los Estados

% Matriz Hessiana: H = 2*blkdiag( I_N⊗R , I_N⊗Q )
% N bloques de R en la diagonal + N bloques de Q en la diagonal
H = 2 * blkdiag( kron(eye(N), R), kron(eye(N), Q) );

% Vector gradiente f
f_u = zeros(N*nu, 1);            % No penalizamos el valor absoluto de u
f_x = repmat(-2 * Q * r, N, 1);  % Penalizamos la distancia de x a r en cada paso
f = [f_u; f_x];

% Matrices Vacias (no usamos restricciones de desigualdad extra)
Ain = [];
bin = [];

% Bloque que multiplica a los controles
% blkdiag(-Bd, -Bd, ..., -Bd)  (N veces)
Aeq_u = kron(eye(N), -Bd);

% Bloque que multiplica a los estados
% Es una matriz triangular con I en la diagonal y -Ad en la subdiagonal
%   I   en diagonal       ->  kron(eye(N), I)
%  -Ad  en subdiagonal    ->  -kron(diag(ones(N-1,1),-1), Ad)
Aeq_x = kron(eye(N), I) - kron(diag(ones(N-1,1), -1), Ad);

% Matriz completa de restricciones de igualdad
Aeq = [Aeq_u, Aeq_x];

% Vector del lado derecho (solo el primer paso conoce la x actual)
beq = [Ad*x; zeros((N-1)*nx, 1)];

% Límites de caja (box constraints)
lb_u = repmat(-umax, N*nu, 1);
ub_u = repmat( umax, N*nu, 1);

% Límites para los estados (repetidos N veces, uno por paso)
lim_x_min = -xmax;  % [-100; -100; -100; -100; ...]
lim_x_max =  xmax;  % [ 100;  100;  100;  100; ...]

lb = [lb_u; repmat(lim_x_min, N, 1)];
ub = [ub_u; repmat(lim_x_max, N, 1)];

% Punto inicial (warm start para el solver)
z0 = zeros(N*nu + N*nx, 1);

% Pre-declaración de z para Simulink (tamaño fijo)
z = zeros(N*nu + N*nx, 1);

% Configuración del solver quadprog
options = optimoptions('quadprog', ...
                       'Algorithm', 'active-set', ...
                       'Display', 'off', ...
                       'MaxIterations', 1000);

% Resolver problema de optimización cuadrática
z = quadprog(H, f, Ain, bin, Aeq, beq, lb, ub, z0, options);

% Ley de Control Actual (horizonte recesivo: solo el primer elemento)
u = z(1);

end
