%% ========================================================================
%  DEMO MPC AUTOMATIZADO  -  Simulación en lazo cerrado
%  ------------------------------------------------------------------------
%  Script ejecutable. Lo ÚNICO que cambias es N.
%  Construye TODAS las matrices solo (con kron) y simula la respuesta.
% =========================================================================
clear; close all; clc;

% ===================== ÚNICO PARÁMETRO A CAMBIAR =====================
N = 5;                      %  <-- CAMBIA SOLO ESTE NÚMERO
% =====================================================================

%% ---------------- MODELO, PESOS Y RESTRICCIONES ----------------------
Ad   = [1.0000 0.1903; 0 0.9048];
Bd   = [0.0097; 0.0952];
Q    = diag([10, 1]);
R    = 0.1;
umax = 100;
xmax = [100; 100];

r    = [10; 0];             % referencia (a dónde llevar los estados)
x0   = [0; 0];              % estado inicial
Tsim = 60;                  % nº de pasos de simulación

%% ---------------- CONSTRUCCIÓN AUTOMÁTICA (función local) ------------
[H, Aeq_x_part, Aeq_u_part, lb, ub] = build_static(Ad, Bd, Q, R, N, umax, xmax);
nx = size(Ad,1);  nu = size(Bd,2);
Aeq = [Aeq_u_part, Aeq_x_part];

opts = optimoptions('quadprog','Algorithm','active-set','Display','off');

%% ---------------- LAZO CERRADO --------------------------------------
x = x0;
X = zeros(nx, Tsim+1);  U = zeros(nu, Tsim);  X(:,1) = x;

for k = 1:Tsim
    % f y beq dependen de r y x actual -> se recalculan cada paso
    f   = [zeros(N*nu,1); repmat(-2*Q*r, N, 1)];
    beq = [Ad*x; zeros((N-1)*nx, 1)];

    z = quadprog(H, f, [], [], Aeq, beq, lb, ub, [], opts);

    u = z(1:nu);              % horizonte recesivo: solo el primero
    U(:,k)   = u;
    x        = Ad*x + Bd*u;   % evolución de la planta
    X(:,k+1) = x;
end

%% ---------------- MÉTRICAS DE DESEMPEÑO -----------------------------
t  = 0:Tsim;
y  = X(1,:);                 % salida = primer estado (posición)
yf = y(end);
os = (max(y) - r(1)) / r(1) * 100;   if os < 0, os = 0; end
ts = NaN;  tol = 0.02*abs(r(1));
for i = numel(y):-1:1
    if abs(y(i) - r(1)) > tol, ts = t(i); break; end
end

fprintf('========== MPC AUTOMATIZADO (N = %d) ==========\n', N);
fprintf('dim(z)              = %d   (%d controles + %d estados)\n', ...
        N*nu + N*nx, N*nu, N*nx);
fprintf('Valor final y       = %.4f\n', yf);
fprintf('Sobrepico os%%       = %.2f %%\n', os);
if ~isnan(ts), fprintf('t. estab. ts(2%%)    = %d pasos (%.1f s aprox)\n', ts, ts); end
fprintf('===============================================\n');

%% ---------------- GRÁFICOS ------------------------------------------
figure('Name', sprintf('MPC automatizado N=%d', N), 'NumberTitle','off');

subplot(2,1,1);
plot(t, y, 'b-', 'LineWidth', 2); hold on;
yline(r(1), 'r--', 'LineWidth', 1.5);
yline(r(1)*1.02, 'g:'); yline(r(1)*0.98, 'g:');
grid on; xlabel('paso k'); ylabel('y = x_1');
title(sprintf('Respuesta en lazo cerrado  (N = %d)', N));
legend('salida y', 'referencia r', 'banda \pm2%', 'Location','best');

subplot(2,1,2);
stairs(0:Tsim-1, U, 'm-', 'LineWidth', 1.5); hold on;
yline( umax, 'r--'); yline(-umax, 'r--');
grid on; xlabel('paso k'); ylabel('u');
title('Acción de control');

% =========================================================================
%  FUNCIÓN LOCAL: construye lo que NO cambia entre pasos (H, Aeq, lb, ub)
% =========================================================================
function [H, Aeq_x, Aeq_u, lb, ub] = build_static(Ad, Bd, Q, R, N, umax, xmax)
    nx = size(Ad,1);  nu = size(Bd,2);

    % Hessiana = 2*blkdiag( I_N⊗R , I_N⊗Q )
    H = 2 * blkdiag( kron(eye(N), R), kron(eye(N), Q) );

    % Igualdad: bloque de controles (−B en diagonal por bloques)
    Aeq_u = kron(eye(N), -Bd);

    % Igualdad: bloque de estados (I en diagonal, −Ad en subdiagonal)
    sub   = diag(ones(N-1,1), -1);
    Aeq_x = kron(eye(N), eye(nx)) - kron(sub, Ad);

    % Límites de caja
    lb = [repmat(-umax, N*nu, 1); repmat(-xmax, N, 1)];
    ub = [repmat( umax, N*nu, 1); repmat( xmax, N, 1)];
end
