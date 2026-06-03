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

%% ==================== MODELO Y PARÁMETROS ============================
Ad   = [1.0000 0.1903; 0 0.9048];
Bd   = [0.0097; 0.0952];
Q    = diag([10, 1]);
R    = 0.1;
umax = 100;
xmax = [100; 100];

r    = [10; 0];             % referencia
x0   = [0; 0];              % estado inicial
Tsim = 60;                  % pasos de simulación

nx = size(Ad,1);  nu = size(Bd,2);

%% ================== CONSTRUCCIÓN AUTOMÁTICA (INLINE) =================
% Matriz Hessiana = 2*blkdiag( I_N⊗R , I_N⊗Q )
H = 2 * blkdiag( kron(eye(N), R), kron(eye(N), Q) );

% Bloque de controles: -B en diagonal por bloques
Aeq_u = kron(eye(N), -Bd);

% Bloque de estados: I en diagonal, -Ad en subdiagonal
sub   = diag(ones(N-1,1), -1);
Aeq_x = kron(eye(N), eye(nx)) - kron(sub, Ad);

% Matriz de restricción de igualdad completa
Aeq = [Aeq_u, Aeq_x];

% Límites de caja
lb = [repmat(-umax, N*nu, 1); repmat(-xmax, N, 1)];
ub = [repmat( umax, N*nu, 1); repmat( xmax, N, 1)];

% Opciones del solver
opts = optimoptions('quadprog','Algorithm','active-set','Display','off');

%% ==================== LAZO CERRADO ==================================
x = x0;
X = zeros(nx, Tsim+1);
U = zeros(nu, Tsim);
X(:,1) = x;

for k = 1:Tsim
    % f y beq se recalculan cada paso (dependen de r y x actual)
    f   = [zeros(N*nu,1); repmat(-2*Q*r, N, 1)];
    beq = [Ad*x; zeros((N-1)*nx, 1)];

    % Resolver problema cuadrático
    z = quadprog(H, f, [], [], Aeq, beq, lb, ub, [], opts);

    % Horizonte recesivo: aplica solo el primer control
    u = z(1:nu);
    U(:,k)   = u;

    % Evolucionar planta
    x        = Ad*x + Bd*u;
    X(:,k+1) = x;
end

%% ==================== MÉTRICAS DE DESEMPEÑO =========================
t  = 0:Tsim;
y  = X(1,:);                 % salida = primer estado
yf = y(end);

% Sobrepico
os = max(0, (max(y) - r(1)) / r(1) * 100);

% Tiempo de estabilización (2%)
ts = NaN;  tol = 0.02*abs(r(1));
for i = numel(y):-1:1
    if abs(y(i) - r(1)) > tol
        ts = t(i);
        break;
    end
end

%% ==================== REPORTE ========================================
fprintf('\n');
fprintf('========== MPC AUTOMATIZADO (N = %d) ==========\n', N);
fprintf('dim(z)              = %d   (%d controles + %d estados)\n', ...
        N*nu + N*nx, N*nu, N*nx);
fprintf('Valor final y       = %.4f\n', yf);
fprintf('Sobrepico os%%       = %.2f %%\n', os);
if ~isnan(ts)
    fprintf('t. estab. ts(2%%)    = %d pasos\n', ts);
end
fprintf('===============================================\n\n');

%% ==================== GRÁFICOS =======================================
figure('Name', sprintf('MPC automatizado N=%d', N), 'NumberTitle','off');

subplot(2,1,1);
plot(t, y, 'b-', 'LineWidth', 2); hold on;
yline(r(1), 'r--', 'LineWidth', 1.5, 'Label', sprintf('ref r=%.0f', r(1)));
yline(r(1)*1.02, 'g:', 'LineWidth', 1);
yline(r(1)*0.98, 'g:', 'LineWidth', 1, 'Label', 'banda \pm2%');
grid on; xlabel('paso k'); ylabel('y = x_1');
title(sprintf('Respuesta en lazo cerrado  (N = %d)', N));
legend('salida y', 'Location','best');
xlim([0 Tsim]);

subplot(2,1,2);
stairs(0:Tsim-1, U, 'm-', 'LineWidth', 1.5); hold on;
yline( umax, 'r--', 'LineWidth', 1, 'Label', sprintf('límites \\pm%d', umax));
yline(-umax, 'r--', 'LineWidth', 1);
grid on; xlabel('paso k'); ylabel('u');
title('Acción de control');
xlim([0 Tsim]);

drawnow;
