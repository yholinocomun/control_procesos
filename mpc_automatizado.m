function u = fcn(r, x)

% Para que quadprog funcione dentro del bloque MATLAB Function de Simulink:
coder.extrinsic('quadprog', 'optimoptions');

N = 5; % Horizonte de Prediccion   <-- CAMBIA SOLO ESTE NUMERO (1,2,3,...)

% Espacio de Estado de la Planta (Modelo)
Ad = [ 1.0000 0.1903
       0      0.9048];

Bd = [ 0.0097
       0.0952];

% Ganancia de los Estados del LQR
Q = diag([10, 1]);
R = 0.1;

% Restricciones
umax = 100;
x1max = 100;
x2max = 100;

% Dimensiones del modelo (se deducen solas de Ad y Bd)
nx = size(Ad, 1);   % numero de estados   (= 2)
nu = size(Bd, 2);   % numero de controles (= 1)

% Matrices en Base a la restriccion
I = eye(nx);  % Identidad para los Estados
Z = zeros(nx); % Zeros para los Estados

% Tiene N bloques de R (esfuerzo) y luego N bloques de Q (castigo al error).
% blkdiag(R,R,...,Q,Q,...) se genera automaticamente con kron:
H = 2 * blkdiag( kron(eye(N), R), kron(eye(N), Q) );

% El vector r es un vector de 2x1 (la referencia para los 2 estados)
% f debe tener tamano (N*nu + N*nx) x 1
f_u = zeros(N*nu, 1);            % No penalizamos el valor absoluto de u
f_x = repmat(-2 * Q * r, N, 1);  % Penalizamos la distancia de x a r en cada paso
f = [f_u; f_x];

% Matrices Vacias
Ain = [];
bin = [];

% Bloque que multiplica a las u  ->  blkdiag(-Bd,-Bd,...,-Bd)  (N veces)
Aeq_u = kron(eye(N), -Bd);

% Bloque que multiplica a las x (conecta x_k con x_{k+1})
% Es una matriz con Identidades en la diagonal y -Ad debajo de ellas.
%   I  en la diagonal por bloques  ->  kron(eye(N), I)
%  -Ad en la subdiagonal           -> -kron(diag(ones(N-1,1),-1), Ad)
Aeq_x = kron(eye(N), I) - kron(diag(ones(N-1,1), -1), Ad);

Aeq = [Aeq_u, Aeq_x];

% El vector beq. Solo el primer paso conoce la 'x' actual.
beq = [Ad*x; zeros((N-1)*nx, 1)];

% Repetimos los limites N veces
lb_u = repmat(-umax, N*nu, 1);
ub_u = repmat( umax, N*nu, 1);

% Limites para los estados (repetidos para cada paso del horizonte)
lim_x_min = [-x1max; -x2max];
lim_x_max = [ x1max;  x2max];

lb = [lb_u; repmat(lim_x_min, N, 1)];
ub = [ub_u; repmat(lim_x_max, N, 1)];

% Configuracion del MPC "quadprog"
z0 = zeros(N*nu + N*nx, 1);   % (N ley de control + N*nx estados)

% Pre-inicializa z con tamano fijo (necesario por usar quadprog extrinseco)
z = zeros(N*nu + N*nx, 1);

options = optimoptions('quadprog', 'Algorithm', 'active-set', 'Display', 'off');

z = quadprog(H, f, Ain, bin, Aeq, beq, lb, ub, z0, options);

% Ley de Control Actual
u = z(1);

end
