function u = fcn(r, x)
% =========================================================================
% Controlador MPC (Model Predictive Control) con horizonte N generico
% Compatible con bloques MATLAB Function de Simulink (sin cell arrays)
%
% ENTRADAS:
%   r  - vector [2x1]: referencia deseada para los estados (setpoint)
%   x  - vector [2x1]: estado actual medido del sistema
%
% SALIDA:
%   u  - escalar: accion de control optima para el instante actual
%
% DESCRIPCION:
%   Resuelve un QP en cada instante para obtener la secuencia optima
%   z = [u_0; u_1;...; u_{N-1}; x_1; x_2;...; x_N]
%   y aplica solo u_0 (principio receding-horizon del MPC).
%   Tamano de z: nz = N*nu + N*nx = N + 2N = 3N
%
% NOTA SIMULINK:
%   Este codigo NO usa cell arrays ni blkdiag dinamico porque el analizador
%   estatico de Simulink (equivalente a MATLAB Coder) no puede inferir
%   los tamanos de salida cuando se usan {R_blocks{:}}. En su lugar,
%   todas las matrices se construyen con preallocacion + indexacion en loop,
%   lo que permite al analizador determinar los tamanos en tiempo de compilacion.
% =========================================================================

% -------------------------------------------------------------------------
% PARAMETRO: Horizonte de Prediccion
%   Cambia SOLO este valor. Todo lo demas se calcula automaticamente.
% -------------------------------------------------------------------------
N = 5; % <-- Modifica aqui: N=1, N=3, N=10, N=20, etc.

% Dimensiones del sistema
nx = 2; % Numero de estados (x1, x2)
nu = 1; % Numero de entradas de control (u)

% Tamano total del vector de decision z = [u_0..u_{N-1}; x_1..x_N]
nz = N * nu + N * nx; % = 3*N

% -------------------------------------------------------------------------
% MODELO DISCRETO: x_{k+1} = Ad*x_k + Bd*u_k
% -------------------------------------------------------------------------
Ad = [0.9909  0.0861;
     -0.1722  0.7326];

Bd = [0.0045;
      0.0861];

% -------------------------------------------------------------------------
% PESOS DE LA FUNCION DE COSTO
%   Q penaliza error de estados (mayor => sigue mejor la referencia)
%   R penaliza esfuerzo de control (mayor => senal de control mas suave)
% -------------------------------------------------------------------------
Q = diag([50, 1]); % [2x2]: peso 50 sobre x1, peso 1 sobre x2
R = 0.1;           % escalar: penalizacion sobre u

% -------------------------------------------------------------------------
% LIMITES FISICOS DEL SISTEMA
% -------------------------------------------------------------------------
umax  = 100; % -umax <= u_k <= umax
x1max = 100; % -x1max <= x1_k <= x1max
x2max = 100; % -x2max <= x2_k <= x2max

% =========================================================================
% HESSIANA H [nz x nz] — construida con indexacion directa (sin cell arrays)
%
% Estructura diagonal por bloques:
%   Posiciones 1..N         -> bloques escalares 2*R  (coste de control)
%   Posiciones N+1..N+N*nx  -> bloques 2*Q [2x2]     (coste de estados)
%
% La construccion usa preallocacion + asignacion por indice para que
% Simulink pueda inferir el tamano [nz x nz] en analisis estatico.
% =========================================================================
H = zeros(nz, nz); % Preallocacion: Simulink infiere tamano [3N x 3N]

% Bloques de control: H(k,k) = 2*R para k = 1..N
for k = 1:N
    H(k, k) = 2 * R; % Cada control u_k contribuye 2*R en la diagonal
end

% Bloques de estado: H(bloque_k, bloque_k) = 2*Q para k = 1..N
for k = 1:N
    % Indice de inicio del bloque de estado k en z (despues de los N controles)
    idx = N*nu + (k-1)*nx + 1;
    H(idx:idx+nx-1, idx:idx+nx-1) = 2 * Q; % Bloque [2x2] en la diagonal
end

% =========================================================================
% VECTOR LINEAL f [nz x 1]
%
% f_u = 0 (no penalizamos valor absoluto de u, solo su magnitud via H)
% f_x = -2*Q*r repetido N veces (atrae cada x_k hacia la referencia r)
% =========================================================================
f = zeros(nz, 1); % Preallocacion: primeros N elementos quedan en cero (f_u=0)

Qr = -2 * Q * r;  % Producto [-2*Q*r], calculado una sola vez fuera del loop [2x1]
for k = 1:N
    % Indice de inicio del bloque de estado k en f
    idx = N*nu + (k-1)*nx + 1;
    f(idx:idx+nx-1) = Qr; % Copiamos -2*Q*r en la posicion del estado k
end

% =========================================================================
% RESTRICCIONES DE IGUALDAD: Aeq [N*nx x nz]  y  beq [N*nx x 1]
%
% Codifican la dinamica: x_{k+1} = Ad*x_k + Bd*u_k para k=0..N-1
% Reescrito para el QP:
%   Fila k=1:  -Bd*u_0 +         x_1              = Ad*x   (condicion inicial)
%   Fila k>1:  -Bd*u_k - Ad*x_{k-1} + x_k         = 0
%
% Aeq se divide en dos bloques horizontales:
%   Aeq_u [N*nx x N*nu]: coeficientes de los controles u_k
%   Aeq_x [N*nx x N*nx]: coeficientes de los estados  x_k
% =========================================================================
nrows_eq = N * nx; % Numero de filas de Aeq = numero de ecuaciones de dinamica

% --- Bloque Aeq_u: diagonal de -Bd, una por cada u_k ---
Aeq_u = zeros(nrows_eq, N*nu); % Preallocacion [2N x N]
for k = 1:N
    row = (k-1)*nx + 1; % Fila de inicio del bloque k
    col = (k-1)*nu + 1; % Columna de inicio del bloque k (nu=1 => columna k)
    Aeq_u(row:row+nx-1, col:col+nu-1) = -Bd; % Coloca -Bd en la diagonal de bloques
end

% --- Bloque Aeq_x: I en la diagonal, -Ad en la subdiagonal de bloques ---
Aeq_x = zeros(nrows_eq, N*nx); % Preallocacion [2N x 2N]
for k = 1:N
    row = (k-1)*nx + 1; % Fila de inicio del bloque k
    col = (k-1)*nx + 1; % Columna de la identidad I (corresponde a x_k)

    Aeq_x(row:row+nx-1, col:col+nx-1) = eye(nx); % I en la diagonal principal

    % -Ad en la subdiagonal (conecta x_k con x_{k-1}); no aplica para k=1
    if k > 1
        col_ad = (k-2)*nx + 1; % Columna de x_{k-1}
        Aeq_x(row:row+nx-1, col_ad:col_ad+nx-1) = -Ad;
    end
end

% Ensamblado final de Aeq y beq
Aeq = [Aeq_u, Aeq_x]; % [N*nx x nz] = [2N x 3N]

% beq: primera ecuacion usa la condicion inicial Ad*x; el resto son ceros
beq = zeros(nrows_eq, 1);           % Preallocacion [2N x 1]
beq(1:nx) = Ad * x;                 % Solo el primer bloque conoce el estado inicial

% =========================================================================
% RESTRICCIONES DE DESIGUALDAD (vacias — los limites van en lb/ub)
% =========================================================================
Ain = zeros(0, nz); % Matriz vacia pero con numero de columnas conocido
bin = zeros(0, 1);  % Vector vacio compatible

% =========================================================================
% LIMITES DEL VECTOR DE DECISION: lb <= z <= ub
%
% Construidos con preallocacion + loop para compatibilidad con Simulink.
% =========================================================================
lb = zeros(nz, 1); % Preallocacion
ub = zeros(nz, 1); % Preallocacion

% Limites de control: posiciones 1..N
for k = 1:N
    lb(k) = -umax;
    ub(k) =  umax;
end

% Limites de estado: posiciones N+1..nz, agrupados en bloques de nx
lim_min = [-x1max; -x2max]; % Limites inferiores para un paso [2x1]
lim_max = [ x1max;  x2max]; % Limites superiores para un paso [2x1]
for k = 1:N
    idx = N*nu + (k-1)*nx + 1; % Inicio del bloque de estado k
    lb(idx:idx+nx-1) = lim_min;
    ub(idx:idx+nx-1) = lim_max;
end

% =========================================================================
% RESOLUCION DEL QP
%   min   0.5 * z'*H*z + f'*z
%   s.t.  Aeq*z = beq
%         lb <= z <= ub
% =========================================================================
z0 = zeros(nz, 1); % Punto de arranque del solver

options = optimoptions('quadprog', ...
    'Algorithm', 'active-set', ... % Activo: rapido para QP de tamano moderado
    'Display',   'off');           % Sin mensajes en consola

z = quadprog(H, f, Ain, bin, Aeq, beq, lb, ub, z0, options);

% -------------------------------------------------------------------------
% LEY DE CONTROL RECEDING-HORIZON: aplicar solo u_0
%   u_1..u_{N-1} se descartan; en el proximo ciclo se re-optimiza todo
%   el horizonte con el nuevo estado medido (principio fundamental del MPC).
% -------------------------------------------------------------------------
u = z(1); % u_0 es la primera componente de z
