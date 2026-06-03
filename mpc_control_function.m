%% FUNCIÓN DE CONTROL MPC - ANÁLISIS DETALLADO LÍNEA POR LÍNEA
% Fecha: 2024
% Descripción: Función que implementa un controlador MPC usando quadprog
%
% Entrada:
%   r: Referencia (vector 2x1) - posiciones deseadas para los 2 estados
%   x: Estado actual (vector 2x1) - valores presentes de los estados
%
% Salida:
%   u: Control a aplicar (escalar) - acción de control en el instante actual

function u = fcn(r, x)

%% ========================================================================
% LÍNEA 3: HORIZONTE DE PREDICCIÓN
% ========================================================================
% N = 5; % Horizonte de Prediccion
%
% EXPLICACIÓN:
%   - N define cuántos pasos hacia adelante predice el MPC
%   - En este caso: PREDICE 5 PASOS FUTUROS
%   - Significa: Optimiza u[k], u[k+1], u[k+2], u[k+3], u[k+4]
%   - Horizonte mayor = mejor predicción pero más cómputo
%   - Típicamente: N ∈ [3, 20] para sistemas rápidos
%
% VECTOR DE OPTIMIZACIÓN (será de tamaño 5 + 2*5 = 15):
%   z = [u[k], u[k+1], u[k+2], u[k+3], u[k+4],
%        x1[k], x2[k], x1[k+1], x2[k+1], ..., x1[k+4], x2[k+4]]
%   |____________ 5 entradas _____________| |_________ 10 estados _________|

N = 5; % Horizonte de Prediccion

%% ========================================================================
% LÍNEAS 6-10: MATRICES DEL MODELO DISCRETIZADO
% ========================================================================
% Ad = [ 1.0000 0.1903
%        0 0.9048];
%
% EXPLICACIÓN:
%   Ad es la MATRIZ DE ESTADO DISCRETA (2x2)
%   Relaciona el estado actual con el siguiente: x[k+1] = Ad*x[k] + Bd*u[k]
%
%   Ad = [1.0000  0.1903]  ← Coeficientes de acoplamiento entre estados
%        [0      0.9048]   ← Elemento 2,2 < 1 → estado 2 decae naturalmente
%
%   Interpretación física (ejemplo): Sistema mecánico masa-amortiguador
%   - Elemento (1,1) = 1: Posición se mantiene sin control
%   - Elemento (1,2) = 0.1903: Velocidad suma a posición
%   - Elemento (2,1) = 0: No hay retroalimentación directa
%   - Elemento (2,2) = 0.9048: Velocidad decae con amortiguamiento

Ad = [ 1.0000 0.1903;
       0      0.9048];

% Bd = [ 0.0097
%        0.0952];
%
% EXPLICACIÓN:
%   Bd es la MATRIZ DE ENTRADA DISCRETA (2x1)
%   Define cómo el control u afecta al estado: x[k+1] = Ad*x[k] + Bd*u[k]
%
%   Bd = [0.0097]  ← Control tiene poco efecto directo en posición
%        [0.0952]  ← Control tiene mayor efecto en velocidad (¡10 veces más!)
%
%   Interpretación: Control es una fuerza aplicada que cambia velocidad (x2)
%                   La posición (x1) cambia de forma indirecta vía velocidad

Bd = [ 0.0097;
       0.0952];

%% ========================================================================
% LÍNEAS 13-14: PESOS DE LA FUNCIÓN DE COSTO
% ========================================================================
% Q = diag([10, 1]);
%
% EXPLICACIÓN:
%   Q es la MATRIZ DE PESO DEL ERROR DE SEGUIMIENTO (2x2)
%   Q = diag([10, 1]) = [10  0]
%                       [0   1]
%
%   Penaliza cuánto se desvía cada estado de su referencia
%   - Q(1,1) = 10: ERROR en x1 (posición) tiene ALTA penalización
%   - Q(2,2) = 1:  ERROR en x2 (velocidad) tiene BAJA penalización
%
%   Efecto:
%     Si e = x - r (error), costo = e' * Q * e = 10*e1² + 1*e2²
%     → El controlador PRIORIZA llegar a x1 deseada
%     → Tolera más error en x2 (velocidad)
%
% R = 0.1;
%
% EXPLICACIÓN:
%   R es el PESO DEL ESFUERZO DE CONTROL (escalar)
%   Penaliza la magnitud del control (energía)
%
%   Costo total de control = u' * R * u = 0.1 * u²
%
%   Efectos:
%   - R pequeño (0.1): Permite controles GRANDES y RÁPIDOS
%   - R grande (ej: 10): Controles más SUAVES y LENTOS
%
%   Relación Q/R:
%   - Q/R = 10/0.1 = 100 → Control AGRESIVO (sigue referencias rápido)
%   - Q/R grande → Prioritiza tracking sobre energía
%   - Q/R pequeño → Prioritiza eficiencia sobre tracking

Q = diag([10, 1]);
R = 0.1;

%% ========================================================================
% LÍNEAS 17-19: LÍMITES DE CONTROL Y ESTADOS
% ========================================================================
% umax = 100;     % Límite máximo de control
% x1max = 100;    % Límite máximo de estado 1
% x2max = 100;    % Límite máximo de estado 2
%
% EXPLICACIÓN:
%   Estos son RESTRICCIONES CAJA (box constraints)
%   Limitan valores máximos y mínimos de variables
%
%   umax = 100: El control debe satisfacer -100 ≤ u[k] ≤ 100 para todo k
%   x1max = 100: El estado 1 debe satisfacer -100 ≤ x1[k] ≤ 100
%   x2max = 100: El estado 2 debe satisfacer -100 ≤ x2[k] ≤ 100
%
%   En sistemas reales:
%   - Control limitado: motor saturado, válvula máxima, etc.
%   - Estados limitados: posición máxima, velocidad máxima, etc.
%
%   En este ejemplo son límites grandes (100) → No son restrictivos

umax = 100;
x1max = 100;
x2max = 100;

%% ========================================================================
% LÍNEAS 22-23: MATRICES IDENTIDAD Y CEROS PARA BLOQUES
% ========================================================================
% I = eye(2);     % Identidad para los Estados
% Z = zeros(2);   % Zeros para los Estados
%
% EXPLICACIÓN:
%   Se crean matrices 2x2 que se usarán para construir matrices grandes
%   mediante "blkdiag" (block diagonal)
%
%   I = [1 0]   ← Identidad 2x2
%       [0 1]
%
%   Z = [0 0]   ← Ceros 2x2
%       [0 0]
%
%   Estas servirán para construir las restricciones de igualdad (ecuaciones dinámicas)

I = eye(2);     % Identidad para los Estados
Z = zeros(2);   % Zeros para los Estados

%% ========================================================================
% LÍNEA 26: MATRIZ HESSIANA H
% ========================================================================
% H = 2 * blkdiag(R, R, R, R, R, Q, Q, Q, Q, Q);
%
% EXPLICACIÓN (CRÍTICO):
%   H es la MATRIZ HESSIANA del problema cuadrático
%   El problema a resolver es:
%     min  z' * H * z + f' * z     (forma canónica)
%     s.a. Aeq * z = beq,  Ain * z ≤ bin,  lb ≤ z ≤ ub
%
%   blkdiag(R, R, R, R, R, Q, Q, Q, Q, Q) crea:
%
%   H = [ R   0   0  ...  0 ]   ← Bloque de controles (5 × 5)
%       [ 0   Q   0  ...  0 ]   ← Bloque de errores paso 1 (2 × 2)
%       [ 0   0   Q  ...  0 ]   ← Bloque de errores paso 2 (2 × 2)
%       [         ...        ]
%       [ 0   0   0  ...  Q ]   ← Bloque de errores paso 5 (2 × 2)
%
%   Multiplicado por 2 porque:
%   - Función de costo es: J = Σ ||u[k]||²_R + Σ ||e[k]||²_Q
%   - En forma cuadrática: J = z' * H * z (sin el 2)
%   - quadprog asume: J = ½ z' * H * z (CON el 2)
%   - Por eso multiplican por 2 para compensar el ½
%
%   Tamaño: H es 15×15 (5 controles + 10 estados = 15 variables)
%
%   ESTRUCTURA RESULTANTE (conceptual):
%   H = [0.1   0    0  ...  0 ]     ← 5×5: Penaliza controles
%       [ 0   20    0  ...  0 ]     ← 2×2: Q de paso 1 (peso error)
%       [ 0    0   20  ...  0 ]     ← 2×2: Q de paso 2
%       [ 0    0    0  ... 20 ]     ← 2×2: Q de paso 5
%
%   (Note: Q = diag([10,1]), así que bloque Q aparece 5 veces)

H = 2 * blkdiag(R, R, R, R, R, Q, Q, Q, Q, Q);
% H es 15×15 diagonal por bloques

%% ========================================================================
% LÍNEA 28: VECTOR DE REFERENCIA r
% ========================================================================
% El vector r es un vector de 2x1 (la referencia para los 2 estados)
%
% EXPLICACIÓN:
%   r = [r1]  ← Referencia para estado 1 (posición deseada)
%       [r2]  ← Referencia para estado 2 (velocidad deseada)
%
%   Es la ENTRADA a la función fcn(r, x)
%   Ejemplo: r = [10; 0] significa:
%     - Llevar x1 (posición) a 10
%     - Llevar x2 (velocidad) a 0 (estado estacionario)

%% ========================================================================
% LÍNEA 31: VECTOR GRADIENTE PARTE 1: ESFUERZO DE CONTROL
% ========================================================================
% f_u = zeros(5, 1);
%
% EXPLICACIÓN:
%   f_u son los coeficientes lineales para LOS CONTROLES (5×1)
%
%   Función de costo para controles:
%     Σ ||u[k]||²_R
%
%   En forma cuadrática:
%     z' * H * z + f' * z
%
%   Para los CONTROLES solo hay término cuadrático (u²), NO hay término lineal
%   Por eso f_u = zeros(5,1)
%
%   Nota: Si hubiera un costo lineal en controles (ej: energía), sería distinto de cero
%
%   Ejemplo: Si tuviera costo = 0.5*u² + 5*u (energía + bias)
%           Entonces f_u = [5; 5; 5; 5; 5]

f_u = zeros(5, 1);

%% ========================================================================
% LÍNEA 32: VECTOR GRADIENTE PARTE 2: ERROR DE SEGUIMIENTO
% ========================================================================
% f_x = repmat(-2 * Q * r, 5, 1);
%
% EXPLICACIÓN (CRÍTICO):
%   Función de costo para ESTADOS:
%     Σ ||x[k] - r||²_Q = Σ (x[k] - r)' * Q * (x[k] - r)
%                       = Σ (x[k]' * Q * x[k] - 2 * r' * Q * x[k] + r' * Q * r)
%
%   El término lineal es: -2 * Q * r
%   Este se repite 5 veces (uno para cada paso del horizonte)
%
%   -2 * Q * r = -2 * [10  0] * [r1]  = [-20*r1]
%                      [0   1]   [r2]    [-2*r2]
%
%   repmat(-2*Q*r, 5, 1) replica esto 5 veces:
%   f_x = [-20*r1]
%         [-2*r2 ]
%         [-20*r1]
%         [-2*r2 ]
%         [-20*r1]
%         [-2*r2 ]
%         [-20*r1]
%         [-2*r2 ]
%         [-20*r1]
%         [-2*r2 ]
%
%   Efecto: Pendiente del costo que "tira" los estados hacia r
%
%   Notar el signo NEGATIVO:
%   - f > 0: Minimizar z significa mover z en dirección negativa
%   - f < 0: Minimizar z significa mover z en dirección positiva (hacia r)

f_x = repmat(-2 * Q * r, 5, 1);

%% ========================================================================
% LÍNEA 33: VECTOR GRADIENTE COMPLETO
% ========================================================================
% f = [f_u; f_x];
%
% EXPLICACIÓN:
%   Se concatenan los gradientes:
%   f = [f_u (5×1) = costo control]
%       [f_x (10×1) = costo error estado]
%
%   Resultado: f es 15×1, mismo tamaño que z
%
%   f = [ 0;       ← u[k] no tiene costo lineal
%         0;       ← u[k+1]
%         0;       ← u[k+2]
%         0;       ← u[k+3]
%         0;       ← u[k+4]
%        -20*r1;   ← x1[k] "atrae" hacia r1
%        -2*r2;    ← x2[k] "atrae" hacia r2
%        -20*r1;   ← x1[k+1]
%        -2*r2;    ← x2[k+1]
%        ... ]

f = [f_u; f_x];

%% ========================================================================
% LÍNEAS 36-37: RESTRICCIONES DE DESIGUALDAD (VACÍAS)
% ========================================================================
% Ain = [];
% bin = [];
%
% EXPLICACIÓN:
%   Ain es la MATRIZ DE RESTRICCIONES DE DESIGUALDAD: Ain * z ≤ bin
%
%   En este código están VACÍAS porque:
%   - Los límites de control y estado se manejan con lb, ub (box constraints)
%   - No hay restricciones acopladas (ej: x1 + x2 ≤ 5)
%
%   Si fueran necesarias (ejemplo):
%     Restricción: x1[k] + x2[k] ≤ 50 para todos los pasos
%     Entonces Ain tendría filas [0 0 0 0 0 1 1 0 0 0 ...]
%
%   Aquí: Ain = [] × [], equivalentemente sin restricciones de desigualdad

Ain = [];
bin = [];

%% ========================================================================
% LÍNEAS 40-50: RESTRICCIONES DE IGUALDAD - DINÁMICAS DEL SISTEMA
% ========================================================================
% ESTO ES LO MÁS CRÍTICO DEL MPC
%
% Restricción: Aeq * z = beq
%
% Aeq = [Aeq_u, Aeq_x]
%
% donde Aeq_u relaciona controles con estados (ecuación dinámica)
%       Aeq_x relaciona estados entre pasos (continuidad)

% ========================================================================
% LÍNEA 40: BLOQUE DE CONTROL EN RESTRICCIONES
% ========================================================================
% Aeq_u = blkdiag(-Bd,-Bd,-Bd,-Bd,-Bd);
%
% EXPLICACIÓN:
%   blkdiag(-Bd, -Bd, -Bd, -Bd, -Bd) crea matriz 10×5:
%
%   Aeq_u = [-0.0097    0         0         0         0    ]  ← -Bd
%           [-0.0952    0         0         0         0    ]
%           [   0    -0.0097     0         0         0    ]  ← -Bd
%           [   0    -0.0952     0         0         0    ]
%           [   0       0     -0.0097     0         0    ]  ← -Bd
%           [   0       0     -0.0952     0         0    ]
%           [   0       0         0    -0.0097     0    ]  ← -Bd
%           [   0       0         0    -0.0952     0    ]
%           [   0       0         0        0     -0.0097]  ← -Bd
%           [   0       0         0        0     -0.0952]
%
%   Tamaño: 10×5 (10 estados a través de 5 controles)
%
%   Propósito: Multiplica cada control por -Bd
%   Cuando se combine con estados, formará: x[k+1] - Ad*x[k] - Bd*u[k] = 0

Aeq_u = blkdiag(-Bd,-Bd,-Bd,-Bd,-Bd);

%% ========================================================================
% LÍNEA 44-48: BLOQUE DE ESTADOS EN RESTRICCIONES
% ========================================================================
% Aeq_x = [I, Z, Z, Z, Z;
%         -Ad, I, Z, Z, Z;
%          Z,-Ad, I, Z, Z;
%          Z, Z,-Ad, I, Z;
%          Z, Z, Z,-Ad, I];
%
% EXPLICACIÓN (MATRIZ DE TRANSICIÓN):
%   Esta es una matriz TRIANGULAR INFERIOR ESPECIAL de tamaño 10×10
%   Conecta estados consecutivos mediante la dinámica
%
%   ESTRUCTURA VISUAL:
%   Aeq_x = [I   Z   Z   Z   Z]   ← Conecta x[k]
%           [-Ad I   Z   Z   Z]   ← Conecta x[k] con x[k+1]
%           [Z  -Ad  I   Z   Z]   ← Conecta x[k+1] con x[k+2]
%           [Z   Z  -Ad  I   Z]   ← Conecta x[k+2] con x[k+3]
%           [Z   Z   Z  -Ad  I]   ← Conecta x[k+3] con x[k+4]
%
%   SIGNIFICADO DE CADA FILA:
%   - Fila 1-2 (I): x[k] = x[k] (estado inicial, multiplicado por Aeq_u da -Bd*u)
%   - Fila 3-4 (-Ad,I): x[k+1] - Ad*x[k] = Bd*u[k]
%   - Fila 5-6 (-Ad,I): x[k+2] - Ad*x[k+1] = Bd*u[k+1]
%   - ...
%   - Fila 9-10 (-Ad,I): x[k+4] - Ad*x[k+3] = Bd*u[k+3]
%
%   Tamaño: 10×10 (10 filas para 10 estados, 10 columnas para 10 estados)

Aeq_x = [I,  Z,   Z,   Z,   Z;
        -Ad, I,   Z,   Z,   Z;
         Z, -Ad,  I,   Z,   Z;
         Z,  Z,  -Ad,  I,   Z;
         Z,  Z,   Z,  -Ad,  I];

%% ========================================================================
% LÍNEA 50: MATRIZ COMPLETA DE IGUALDAD
% ========================================================================
% Aeq = [Aeq_u, Aeq_x];
%
% EXPLICACIÓN:
%   Concatena horizontalmente los bloques de control y estado
%   Aeq = [Aeq_u (10×5) | Aeq_x (10×10)]
%
%   Resultado: Aeq es 10×15
%
%   Interpretación:
%   Aeq * z = beq significa:
%   [Aeq_u | Aeq_x] * [u1 u2 u3 u4 u5 x1_1 x2_1 ... x2_5]' = beq
%
%   Expandido (forma conceptual):
%   Row 1: 1*x1[k] + 0*x2[k]               = Ad[1,1]*x1[k-1] + Ad[1,2]*x2[k-1] + Bd[1]*u[k]
%   Row 2: 1*x2[k] + 0*x1[k]               = Ad[2,1]*x1[k-1] + Ad[2,2]*x2[k-1] + Bd[2]*u[k]
%   Row 3: x1[k+1] - Ad*x1[k] - Ad*x2[k]  = Bd[1]*u[k+1]
%   ...

Aeq = [Aeq_u, Aeq_x];

%% ========================================================================
% LÍNEA 53: VECTOR DE IGUALDAD (LADO DERECHO)
% ========================================================================
% beq = [Ad*x; zeros(2,1); zeros(2,1); zeros(2,1); zeros(2,1)];
%
% EXPLICACIÓN (CRÍTICO):
%   beq es el lado derecho de Aeq * z = beq
%   Tamaño: 10×1 (mismo que Aeq filas)
%
%   beq = [Ad*x(0)]        ← 2×1: Condición inicial (x conocida)
%         [0]              ← 2×1: Ceros para ecuación dinámica paso 1
%         [0]              ← 2×1: Ceros para ecuación dinámica paso 2
%         [0]              ← 2×1: Ceros para ecuación dinámica paso 3
%         [0]              ← 2×1: Ceros para ecuación dinámica paso 4
%
%   Ad*x es el ESTADO PREDICHO en el siguiente paso si NO aplicamos control
%   Si x = [1; 0], entonces Ad*x = [1.0000 0.1903] * [1] = [1.0000]
%                                  [0     0.9048]   [0]   [0]
%
%   Forma expandida (conceptual):
%   Fila 1-2: x[k] = Ad*x[k-1]              (estado inicial)
%   Fila 3-4: x[k+1] = Ad*x[k] + Bd*u[k]   (predicción paso 1)
%   Fila 5-6: x[k+2] = Ad*x[k+1] + Bd*u[k+1] (predicción paso 2)
%   ...
%
%   Nota: La dinámica x[k+i] - Ad*x[k+i-1] - Bd*u[k+i] = 0
%         se codifica como (-Ad)*x[k+i-1] + I*x[k+i] = Bd*u[k+i]
%         Por eso beq tiene zeros (los Bd*u están en Aeq_u)

beq = [Ad*x; zeros(2,1); zeros(2,1); zeros(2,1); zeros(2,1)];

%% ========================================================================
% LÍNEAS 56-57: LÍMITES INFERIORES Y SUPERIORES DE CONTROL
% ========================================================================
% lb_u = [-umax;-umax;-umax;-umax;-umax];
% ub_u = [ umax; umax; umax; umax; umax];
%
% EXPLICACIÓN:
%   Límites de caja para los 5 controles
%   lb_u = [-100; -100; -100; -100; -100]  ← control mínimo en cada paso
%   ub_u = [ 100;  100;  100;  100;  100]  ← control máximo en cada paso
%
%   Restricción: -100 ≤ u[k] ≤ 100 para k = 0,1,2,3,4
%
%   En quadprog: lb ≤ z ≤ ub (en nuestro caso)

lb_u = [-umax;-umax;-umax;-umax;-umax];
ub_u = [ umax; umax; umax; umax; umax];

%% ========================================================================
% LÍNEAS 60-61: LÍMITES DE ESTADOS
% ========================================================================
% lim_x_min = [-x1max;-x2max];
% lim_x_max = [ x1max; x2max];
%
% EXPLICACIÓN:
%   Límites para UN PASO del horizonte
%   lim_x_min = [-100; -100]  ← estado mínimo (ambas componentes)
%   lim_x_max = [ 100;  100]  ← estado máximo
%
%   Restricción por paso: -100 ≤ x1[k] ≤ 100
%                         -100 ≤ x2[k] ≤ 100
%
%   Se repetirán 5 veces (una para cada paso)

lim_x_min = [-x1max;-x2max];
lim_x_max = [ x1max; x2max];

%% ========================================================================
% LÍNEA 63-64: LÍMITES COMPLETOS
% ========================================================================
% lb = [lb_u; lim_x_min; lim_x_min; lim_x_min; lim_x_min; lim_x_min];
% ub = [ub_u; lim_x_max; lim_x_max; lim_x_max; lim_x_max; lim_x_max];
%
% EXPLICACIÓN:
%   Concatenan límites de controles y estados
%
%   lb (límite inferior, 15×1):
%   lb = [-100]         ← u[0] mínimo
%        [-100]         ← u[1] mínimo
%        [-100]         ← u[2] mínimo
%        [-100]         ← u[3] mínimo
%        [-100]         ← u[4] mínimo
%        [-100,-100]ᵀ   ← x[0] mínimo (repetido 5 veces)
%        [-100,-100]ᵀ   ← x[1] mínimo
%        [-100,-100]ᵀ   ← x[2] mínimo
%        [-100,-100]ᵀ   ← x[3] mínimo
%        [-100,-100]ᵀ   ← x[4] mínimo
%
%   ub (límite superior, 15×1):
%   ub = [ 100]         ← u[0] máximo
%        [ 100]         ← u[1] máximo
%        [ 100]         ← u[2] máximo
%        [ 100]         ← u[3] máximo
%        [ 100]         ← u[4] máximo
%        [ 100, 100]ᵀ   ← x[0] máximo (repetido 5 veces)
%        [ 100, 100]ᵀ   ← x[1] máximo
%        [ 100, 100]ᵀ   ← x[2] máximo
%        [ 100, 100]ᵀ   ← x[3] máximo
%        [ 100, 100]ᵀ   ← x[4] máximo

lb = [lb_u; lim_x_min; lim_x_min; lim_x_min; lim_x_min; lim_x_min];
ub = [ub_u; lim_x_max; lim_x_max; lim_x_max; lim_x_max; lim_x_max];

%% ========================================================================
% LÍNEA 67: PUNTO INICIAL DE OPTIMIZACIÓN
% ========================================================================
% z0 = zeros(15,1);
%
% EXPLICACIÓN:
%   z0 es el punto inicial para el solver de optimización cuadrática
%   z0 = [0; 0; ...; 0]  ← 15 ceros
%
%   Significado:
%   - Supone que no hay control inicial: u[k] = 0
%   - Supone que los estados son cero: x[k] = 0
%
%   Este es solo un "warm start" (inicialización)
%   El solver iterará desde aquí para encontrar la solución óptima
%
%   Nota: Buen z0 acelera convergencia, pero quadprog es robusto

z0 = zeros(15,1);

%% ========================================================================
% LÍNEA 69: OPCIONES DEL SOLVER
% ========================================================================
% options = optimoptions('quadprog', 'Algorithm','active-set', 'Display','off');
%
% EXPLICACIÓN:
%   Configura el comportamiento del solver quadprog
%
%   'Algorithm','active-set':
%   - Algoritmo activo-pasivo (eficiente para problemas medianos)
%   - Alternativas: 'interior-point-convex', 'trust-region-reflective'
%   - Activo-pasivo: rápido para restricciones caja (nuestro caso)
%
%   'Display','off':
%   - NO imprime iteraciones en pantalla
%   - Alternativa: 'iter' para ver progreso de convergencia
%
%   Típicamente:
%   options = optimoptions('quadprog', ...
%       'Algorithm', 'active-set', ...
%       'Display', 'off', ...
%       'TolFun', 1e-9, ...      % tolerancia función objetivo
%       'TolX', 1e-9, ...        % tolerancia variables
%       'MaxIterations', 10000); % iteraciones máximas

options = optimoptions('quadprog', 'Algorithm','active-set', 'Display','off');

%% ========================================================================
% LÍNEA 71: RESOLVER PROBLEMA CUADRÁTICO
% ========================================================================
% z = quadprog(H,f,Ain,bin,Aeq,beq,lb,ub,z0,options);
%
% EXPLICACIÓN (INVOCACIÓN DEL SOLVER):
%   quadprog resuelve el problema:
%
%   min      ½ * z' * H * z + f' * z
%   z
%
%   Sujeto a:
%   Ain * z ≤ bin         (restricciones de desigualdad) ← VACÍAS aquí
%   Aeq * z = beq         (restricciones de igualdad)   ← DINÁMICAS
%   lb ≤ z ≤ ub           (límites de caja)             ← CONTROLES Y ESTADOS
%
%   Parámetros de entrada:
%   H (15×15):  Matriz Hessiana (semidefinida positiva)
%   f (15×1):   Vector gradiente
%   Ain (0×15): Matriz restricciones desigualdad (vacía)
%   bin (0×1):  Vector restricciones desigualdad (vacío)
%   Aeq (10×15): Matriz restricciones igualdad (dinámicas)
%   beq (10×1): Vector restricciones igualdad (condición inicial)
%   lb (15×1):  Límites inferiores
%   ub (15×1):  Límites superiores
%   z0 (15×1):  Punto inicial (warm start)
%   options:    Opciones del solver
%
%   Salida:
%   z (15×1): Solución óptima
%     z = [u*[0], u*[1], u*[2], u*[3], u*[4],   ← Controles óptimos
%          x*[1,0], x*[2,0], x*[1,1], x*[2,1], ..., x*[2,4]]  ← Estados predichos
%
%   Tiempo de ejecución: ~5-20 ms (tiempo real típico)

z = quadprog(H,f,Ain,bin,Aeq,beq,lb,ub,z0,options);

%% ========================================================================
% LÍNEA 73: EXTRAER ACCIÓN DE CONTROL
% ========================================================================
% u = z(1);
%
% EXPLICACIÓN:
%   Se extrae el PRIMER elemento del vector solución z
%   u = z(1) es el control óptimo para el instante ACTUAL
%
%   ¿Por qué solo z(1)?
%   - El MPC predice 5 pasos: u[k], u[k+1], u[k+2], u[k+3], u[k+4]
%   - Pero solo APLICA el primero: u[k]
%   - En el siguiente paso (k+1):
%     * Mide el nuevo estado x[k+1]
%     * Resuelve el problema de nuevo
%     * Aplica z(1) que antes era z(2) → "receding horizon"
%
%   ESTRATEGIA DEL HORIZONTE RECESIVO (Receding Horizon):
%   En tiempo k:   Aplica u[k]   = z(1)
%   En tiempo k+1: Aplica u[k+1] = z(2) del paso anterior (si fuera)
%                  Pero resuelve NUEVO problema → Aplica nuevo z(1)
%   En tiempo k+2: Idem...
%
%   Ventaja: Se adapta a nuevas mediciones y perturbaciones

u = z(1);

end  % FIN DE LA FUNCIÓN

