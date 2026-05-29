function u = fcn(r, x)
% =========================================================================
% Controlador MPC (Model Predictive Control) con horizonte N generico
%
% ENTRADAS:
%   r  - vector [2x1]: referencia deseada para los estados (setpoint)
%   x  - vector [2x1]: estado actual medido del sistema
%
% SALIDA:
%   u  - escalar: accion de control optima para el instante actual
%
% DESCRIPCION:
%   Resuelve un problema de optimizacion cuadratica (QP) en cada instante
%   de muestreo para encontrar la secuencia optima de controles u_0..u_{N-1}
%   y estados x_1..x_N, aplicando solo el primer control u_0.
%   El vector de decision tiene la forma: z = [u_0; u_1;...;u_{N-1}; x_1;...;x_N]
%   Tamano total de z: N*1 (controles) + N*2 (estados) = N*3
% =========================================================================

% -------------------------------------------------------------------------
% PARAMETRO: Horizonte de Prediccion
%   Cambia solo este valor para ajustar el horizonte. Todo lo demas se
%   construye automaticamente en funcion de N.
% -------------------------------------------------------------------------
N = 5; % <-- Modifica este valor: N=1, N=10, N=20, etc.

% Numero de estados del sistema (nx) y entradas de control (nu)
nx = 2; % El sistema tiene 2 estados: x1 y x2
nu = 1; % El sistema tiene 1 entrada de control: u

% Tamano total del vector de decision z
% Primero vienen los N controles, luego los N*nx estados
nz = N * nu + N * nx; % = N + 2N = 3N

% -------------------------------------------------------------------------
% MODELO EN ESPACIO DE ESTADO DISCRETO: x_{k+1} = Ad*x_k + Bd*u_k
% Ad: matriz de transicion de estados (como evolucionan los estados solos)
% Bd: matriz de entrada (como afecta el control a los estados)
% -------------------------------------------------------------------------
Ad = [0.9909  0.0861;
     -0.1722  0.7326];

Bd = [0.0045;
      0.0861];

% -------------------------------------------------------------------------
% MATRICES DE PESO DEL LQR (definen la funcion de costo J)
% Q: penaliza el error en los estados (mayor Q => sigue mejor la referencia)
% R: penaliza el esfuerzo de control (mayor R => control mas suave)
% -------------------------------------------------------------------------
Q = diag([50, 1]); % Penaliza x1 (peso 50) y x2 (peso 1) por separado
R = 0.1;           % Penaliza el uso de la senal de control u

% -------------------------------------------------------------------------
% RESTRICCIONES (limites fisicos del sistema)
% -------------------------------------------------------------------------
umax  = 100; % Limite maximo/minimo del actuador: -umax <= u <= umax
x1max = 100; % Limite maximo/minimo para el estado x1
x2max = 100; % Limite maximo/minimo para el estado x2

% =========================================================================
% CONSTRUCCION DE LA MATRIZ HESSIANA H (funcion de costo cuadratica)
%
% La funcion de costo es: J = z'*H*z + f'*z
% H tiene bloques diagonales: R para cada u_k, Q para cada x_k
%
% blkdiag construye la diagonal por bloques automaticamente con N bloques
% de R y N bloques de Q, sin importar el valor de N.
% =========================================================================

% Bloques de R (uno por cada control u_0, u_1, ..., u_{N-1})
R_blocks = repmat({R}, 1, N); % Crea un cell array {R, R, ..., R} de N elementos

% Bloques de Q (uno por cada estado x_1, x_2, ..., x_N)
Q_blocks = repmat({Q}, 1, N); % Crea un cell array {Q, Q, ..., Q} de N elementos

% H = 2 * diag([R,...,R, Q,...,Q]) -> factor 2 por la derivada de z'Hz
H = 2 * blkdiag(R_blocks{:}, Q_blocks{:}); % Tamano: [nz x nz]

% =========================================================================
% CONSTRUCCION DEL VECTOR LINEAL f (gradiente de la funcion de costo)
%
% Parte de u: f_u = 0 porque no penalizamos el valor absoluto de u,
%             solo su magnitud via R en H.
% Parte de x: f_x = -2*Q*r repetido N veces (queremos que x_k se acerque a r)
% =========================================================================
f_u = zeros(N * nu, 1);        % Vector de ceros para los N controles [N x 1]
f_x = repmat(-2 * Q * r, N, 1); % Penaliza la distancia de x_k a r en cada paso [N*nx x 1]
f   = [f_u; f_x];               % Vector completo de costo lineal [nz x 1]

% =========================================================================
% RESTRICCIONES DE IGUALDAD: Aeq * z = beq
% Codifican la dinamica del sistema: x_{k+1} = Ad*x_k + Bd*u_k
% Reescrito como: -Bd*u_k + x_{k+1} - Ad*x_k = 0 (para k=1..N-1)
%               y -Bd*u_0 + x_1 = Ad*x0          (para k=0, condicion inicial)
%
% El vector z = [u_0;...;u_{N-1}; x_1;...;x_N]
% Por eso Aeq se divide en dos bloques:
%   Aeq_u: multiplica a los controles (columnas de u)
%   Aeq_x: multiplica a los estados   (columnas de x)
% =========================================================================

I = eye(nx);   % Identidad [nx x nx] = [2x2]
Z = zeros(nx); % Ceros    [nx x nx] = [2x2]

% --- Bloque Aeq_u: cada fila k tiene -Bd en la posicion de u_k ---
% Construimos una lista de celdas para blkdiag
Bd_blocks = repmat({-Bd}, 1, N); % {-Bd, -Bd, ..., -Bd} con N elementos
Aeq_u = blkdiag(Bd_blocks{:}); % Matriz diagonal por bloques [N*nx x N*nu]

% --- Bloque Aeq_x: representa la cadena de dinamica entre estados ---
% Fila 1: [I,  Z,  Z, ..., Z ]  => x_1 aparece sola (sin x_0 porque x_0 = x actual)
% Fila 2: [-Ad, I,  Z, ..., Z ]  => x_2 - Ad*x_1 = 0
% Fila k: [0...0, -Ad, I, 0..0]  => x_{k+1} - Ad*x_k = 0
%
% Se construye fila a fila: cada fila de bloques [nx x N*nx]
Aeq_x = zeros(N * nx, N * nx); % Preallocate para eficiencia
for k = 1:N
    row_start = (k-1)*nx + 1; % Inicio de la fila k (en bloques de nx)
    col_I     = (k-1)*nx + 1; % Columna donde va la identidad I (estado x_k)

    % Colocamos la identidad I en la diagonal (corresponde a x_k)
    Aeq_x(row_start:row_start+nx-1, col_I:col_I+nx-1) = I;

    % Colocamos -Ad una columna de bloques a la izquierda (corresponde a x_{k-1})
    % Solo aplica desde la fila 2 en adelante (fila 1 no tiene x_{k-1} = x_0)
    if k > 1
        col_Ad = (k-2)*nx + 1; % Columna donde va -Ad (estado x_{k-1})
        Aeq_x(row_start:row_start+nx-1, col_Ad:col_Ad+nx-1) = -Ad;
    end
end

% Ensamblamos Aeq combinando los bloques de u y x horizontalmente
Aeq = [Aeq_u, Aeq_x]; % Tamano: [N*nx x nz]

% --- Vector beq: lado derecho de las restricciones de igualdad ---
% La primera ecuacion es: -Bd*u_0 + x_1 = Ad*x  (condicion inicial conocida)
% Las demas ecuaciones son: x_{k+1} - Ad*x_k = 0
beq = [Ad * x; zeros((N-1)*nx, 1)]; % [N*nx x 1]

% =========================================================================
% RESTRICCIONES DE DESIGUALDAD: Ain*z <= bin  (vacias en este caso)
% Los limites se imponen directamente via lb y ub (mas eficiente para QP)
% =========================================================================
Ain = []; % Sin restricciones de desigualdad generales
bin = [];

% =========================================================================
% LIMITES (BOUNDS) DEL VECTOR DE DECISION z = [u_0;...;u_{N-1}; x_1;...;x_N]
%
% lb <= z <= ub
% Se construyen repitiendo los limites individuales N veces automaticamente
% =========================================================================

% Limites para los controles: -umax <= u_k <= umax para k=0..N-1
lb_u = repmat(-umax, N * nu, 1);  % Vector [-umax; ...; -umax] de N elementos
ub_u = repmat( umax, N * nu, 1);  % Vector [ umax; ...; umax]  de N elementos

% Limites para cada estado en cada paso: [x1min; x2min] y [x1max; x2max]
lim_x_min = [-x1max; -x2max]; % Limite inferior para un paso [nx x 1]
lim_x_max = [ x1max;  x2max]; % Limite superior para un paso [nx x 1]

% Repetimos los limites de estado N veces (uno por cada paso del horizonte)
lb_x = repmat(lim_x_min, N, 1); % [N*nx x 1]
ub_x = repmat(lim_x_max, N, 1); % [N*nx x 1]

% Concatenamos limites de control y estados
lb = [lb_u; lb_x]; % Limite inferior total [nz x 1]
ub = [ub_u; ub_x]; % Limite superior total [nz x 1]

% =========================================================================
% RESOLUCION DEL PROBLEMA DE OPTIMIZACION CUADRATICA (QP)
%   min   0.5 * z'*H*z + f'*z
%   s.t.  Aeq*z = beq
%         lb <= z <= ub
% =========================================================================

z0 = zeros(nz, 1); % Punto inicial para el solver (se puede mejorar con warm-start)

options = optimoptions('quadprog', ...
    'Algorithm', 'active-set', ... % Metodo activo: eficiente para problemas QP pequenos
    'Display',   'off');           % Sin mensajes en consola durante la simulacion

% Llamada al solver: z contiene [u_0; u_1; ...; u_{N-1}; x_1; ...; x_N]
z = quadprog(H, f, Ain, bin, Aeq, beq, lb, ub, z0, options);

% -------------------------------------------------------------------------
% LEY DE CONTROL: aplicamos SOLO el primer control u_0 (principio de MPC)
% Los controles u_1..u_{N-1} se descartan; en el proximo instante
% se re-optimiza todo el horizonte con el nuevo estado medido.
% -------------------------------------------------------------------------
u = z(1); % Primera componente del vector de decision = u_0
