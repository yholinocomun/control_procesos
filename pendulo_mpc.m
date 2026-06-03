%% CONTROL MPC DE PÉNDULO CON CARRO
% Problema: Diseñar un controlador MPC para seguir una referencia escalón unitario
% Autor: Control Automático
% Fecha: 2024

clear all; close all; clc;

%% ========================================================================
% PARÁMETROS DEL SISTEMA
% =========================================================================
fprintf('================================================================================\n');
fprintf('PARÁMETROS DEL SISTEMA\n');
fprintf('================================================================================\n\n');

M = 0.77;       % masa del carro [kg]
m = 0.089;      % masa del péndulo [kg]
g = 9.81;       % gravedad [m/s²]
L = 0.32;       % longitud del péndulo [m]
T = 0.1;        % período de muestreo [s]

fprintf('M (masa carro)     = %g kg\n', M);
fprintf('m (masa péndulo)   = %g kg\n', m);
fprintf('g (gravedad)       = %g m/s²\n', g);
fprintf('L (longitud)       = %g m\n', L);
fprintf('T (período muestreo) = %g s\n\n', T);

%% ========================================================================
% PARTE A: DISCRETIZACIÓN DEL SISTEMA
% =========================================================================
fprintf('================================================================================\n');
fprintf('PARTE A: DISCRETIZACIÓN DEL SISTEMA\n');
fprintf('================================================================================\n\n');

% Matriz de estado continua A_c: ẋ = A_c*x + B_c*u
% Estados: x1=x(posición), x2=ẋ(velocidad), x3=θ(ángulo), x4=θ̇(vel angular)
A_c = [
    0,           1,                    0,          0;
    0,           0,            -(m*g)/M,          0;
    0,           0,                    0,          1;
    0,           0,  ((M+m)*g)/(M*L),          0
];

B_c = [
    0;
    1/M;
    0;
    -1/(M*L)
];

C_c = [1, 0, 0, 0];
D_c = 0;

fprintf('Matrices continuas del sistema:\n\n');
fprintf('A_c (matriz de estado continua):\n');
disp(A_c);
fprintf('\nB_c (matriz de entrada continua):\n');
disp(B_c);
fprintf('\nC_c (matriz de salida):\n');
disp(C_c);

% DISCRETIZACIÓN: Método de Euler hacia adelante
% x[k+1] = (I + T*A_c)*x[k] + T*B_c*u[k]
% y[k] = C_c*x[k] + D_c*u[k]

A_d = eye(4) + T * A_c;
B_d = T * B_c;
C_d = C_c;
D_d = D_c;

fprintf('\n================================================================================\n');
fprintf('MATRICES DISCRETIZADAS (con T = %.3f s)\n', T);
fprintf('================================================================================\n\n');

fprintf('A_d (matriz de estado discreta):\n');
disp(A_d);
fprintf('\nB_d (matriz de entrada discreta):\n');
disp(B_d);
fprintf('\nC_d (matriz de salida discreta):\n');
disp(C_d);

% Verificar estabilidad
eigs_A = eig(A_d);
fprintf('\nValores propios de A_d: ');
fprintf('%g ', eigs_A);
fprintf('\n');
estable = all(abs(eigs_A) < 1);
fprintf('Sistema discreto es estable: %s\n\n', strrep(strrep(mat2str(estable), '0', 'No'), '1', 'Sí'));

%% ========================================================================
% PARTE B: FORMULACIÓN DEL PROBLEMA MPC
% =========================================================================
fprintf('================================================================================\n');
fprintf('PARTE B: FORMULACIÓN DEL PROBLEMA MPC\n');
fprintf('================================================================================\n\n');

N = 4;          % Horizonte de predicción
n = 4;          % número de estados
m_ctrl = 1;     % número de entradas de control

fprintf('Horizonte de predicción N = %d\n', N);
fprintf('Estados n = %d\n', n);
fprintf('Entradas m = %d\n\n', m_ctrl);

% Especificaciones de desempeño
r = 1;          % referencia escalón unitario
os_max = 0.01;  % 1% de sobrepico
ts_max = 4;     % 4 segundos para estabilizar

fprintf('Especificaciones:\n');
fprintf('  Referencia r = %g\n', r);
fprintf('  Sobrepico máximo os%% = %.1f%%\n', os_max*100);
fprintf('  Tiempo estabilización ts₂%% = %g s\n\n', ts_max);

% CONSTRUCCIÓN DE MATRICES DE PREDICCIÓN

% Matriz Φ: propaga estado actual
Phi = zeros(N, n);
A_pow = A_d;
for i = 1:N
    Phi(i,:) = C_d * A_pow;
    A_pow = A_d * A_pow;
end

% Matriz Ψ: propaga control anterior (constante)
Psi = zeros(N, 1);
A_pow = A_d;
for i = 1:N
    Psi(i,1) = C_d * A_pow * B_d;
    A_pow = A_d * A_pow;
end

% Matriz Λ: propaga cambios incrementales (triangular)
Lambda = zeros(N, N);
for i = 1:N
    for j = 1:i
        A_pow_j = A_d^(i - j + 1);
        Lambda(i, j) = C_d * A_pow_j * B_d;
    end
end

fprintf('--------------------------------------------------------------------------------\n');
fprintf('MATRICES DE PREDICCIÓN\n');
fprintf('--------------------------------------------------------------------------------\n\n');

fprintf('Φ (propaga estado actual a salidas):\n');
disp(Phi);
fprintf('\nΨ (propaga control anterior a salidas):\n');
disp(Psi);
fprintf('\nΛ (propaga cambios de control a salidas):\n');
disp(Lambda);

% PESOS DE LA FUNCIÓN DE COSTO
Q = 100 * eye(N);   % Penalización en error de seguimiento
R = 0.1 * eye(N);   % Penalización en cambio de control

fprintf('\n--------------------------------------------------------------------------------\n');
fprintf('PESOS DE LA FUNCIÓN DE COSTO\n');
fprintf('--------------------------------------------------------------------------------\n\n');

fprintf('Q (peso error de seguimiento):\n');
disp(diag(Q));
fprintf('R (peso cambio de control):\n');
disp(diag(R));

% CONSTRUCCIÓN DE PROBLEMA DE OPTIMIZACIÓN CUADRÁTICA

% Matriz Hessiana: H = 2*(Λ'*Q*Λ + R)
H = 2 * (Lambda' * Q * Lambda + R);

fprintf('\n--------------------------------------------------------------------------------\n');
fprintf('MATRIZ HESSIANA H y RESTRICCIONES\n');
fprintf('--------------------------------------------------------------------------------\n\n');

fprintf('H (matriz Hessiana):\n');
disp(H);

% RESTRICCIONES DE CONTROL
u_max = 10;     % límite máximo de fuerza [N]
u_min = -10;

% Matriz A_ineq para desigualdades acumulativas
A_ineq = zeros(2*N, N);
for i = 1:N
    % Límite superior: Σ Δu ≤ u_max - u[k-1]
    A_ineq(i, 1:i) = 1;
    % Límite inferior: -Σ Δu ≤ u_max + u[k-1]
    A_ineq(N+i, 1:i) = -1;
end

fprintf('A_ineq (matriz restricciones desigualdad):\n');
disp(A_ineq);
fprintf('Restricción: A_ineq * Δu ≤ b_ineq\n\n');

%% ========================================================================
% PARTE C: SIMULACIÓN CON CONTROLADOR MPC
% =========================================================================
fprintf('================================================================================\n');
fprintf('PARTE C: SIMULACIÓN CON CONTROLADOR MPC\n');
fprintf('================================================================================\n\n');

% Parámetros de simulación
t_sim = 10;                     % 10 segundos
n_steps = int32(t_sim / T);

% Condiciones iniciales
x = [0; 0; 0; 0];              % Estado inicial [x, ẋ, θ, θ̇]
u_prev = 0;                    % Control anterior

% Vectores de historia
y_history = zeros(n_steps, 1);
u_history = zeros(n_steps, 1);
t_history = zeros(n_steps, 1);

fprintf('Simulación: %g s con período T=%g s (%d pasos)\n', t_sim, T, n_steps);
fprintf('Estado inicial: x = [%g %g %g %g]\n\n', x(1), x(2), x(3), x(4));

fprintf('Resolviendo MPC en cada paso...\n\n');

% Opciones para fmincon
options = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...
    'Display', 'off', ...
    'TolFun', 1e-9, ...
    'TolCon', 1e-9, ...
    'MaxIterations', 100);

% SIMULACIÓN ITERATIVA
for k = 1:n_steps
    t_history(k) = (k-1) * T;

    % Construir vector gradiente con estado actual
    pred_term = Phi * x + Psi * u_prev - r * ones(N, 1);  % vector (4,1)
    f_vec = 2.0 * Lambda' * Q * pred_term;                % (4,4)*(4,1) = (4,1)
    f_vec = f_vec(:);  % asegurar vector columna

    % Restricciones de desigualdad acumulativas
    b_ineq = [u_max - u_prev * ones(N, 1);
              u_max + u_prev * ones(N, 1)];

    % Función objetivo
    obj_fun = @(delta_u) 0.5 * delta_u' * H * delta_u + f_vec' * delta_u;

    % Restricción de desigualdad
    con_fun = @(delta_u) A_ineq * delta_u - b_ineq;

    % Condición inicial para optimización
    delta_u_init = zeros(N, 1);

    % Resolver problema de optimización
    [delta_u_opt, ~, exitflag] = fmincon(obj_fun, delta_u_init, ...
        A_ineq, b_ineq, [], [], [], [], con_fun, options);

    if exitflag <= 0
        delta_u_opt = zeros(N, 1);
    end

    % Aplicar primer elemento del control óptimo
    u_optimal = u_prev + delta_u_opt(1);
    u_optimal = max(min(u_optimal, u_max), u_min);

    % Registrar
    u_history(k) = u_optimal;
    y_history(k) = C_d * x;

    % Evolucionar sistema: x[k+1] = A*x[k] + B*u[k]
    x = A_d * x + B_d * u_optimal;
    u_prev = u_optimal;

    if mod(k-1, 20) == 0
        fprintf('Paso %3d: t=%6.2f s, x=%8.3f, y=%8.3f, u=%8.3f\n', ...
            k-1, t_history(k), x(1), y_history(k), u_optimal);
    end
end

fprintf('\n================================================================================\n');
fprintf('RESULTADOS DE LA SIMULACIÓN\n');
fprintf('================================================================================\n\n');

% Calcular parámetros de desempeño
y_final = y_history(end);
y_max = max(y_history);
y_min = min(y_history);

% Sobrepico
if y_final ~= 0
    overshoot_percent = ((y_max - y_final) / abs(y_final)) * 100;
else
    overshoot_percent = 0;
end

% Tiempo de estabilización (dentro del 2%)
ts_2percent = NaN;
tolerance = 0.02 * abs(y_final);
if y_final == 0
    tolerance = 0.02;
end

for i = length(y_history):-1:1
    if abs(y_history(i) - y_final) > tolerance
        ts_2percent = t_history(i);
        break;
    end
end

fprintf('Valor final y = %.4f m\n', y_final);
fprintf('Valor máximo y = %.4f m\n', y_max);
fprintf('Sobrepico os%% = %.2f %%\n', overshoot_percent);
if ~isnan(ts_2percent)
    fprintf('Tiempo estabilización ts₂%% = %.2f s\n', ts_2percent);
else
    fprintf('Tiempo estabilización ts₂%%: No alcanzado\n');
end

fprintf('\n✓ Especificación os%% ≤ 1%%: %s\n', strrep(strrep(mat2str(overshoot_percent <= 1.0), '0', 'No'), '1', 'Sí'));
if ~isnan(ts_2percent)
    fprintf('✓ Especificación ts₂%% ≤ 4s: %s\n\n', strrep(strrep(mat2str(ts_2percent <= 4.0), '0', 'No'), '1', 'Sí'));
end

%% ========================================================================
% GRÁFICOS
% =========================================================================

figure('Name', 'MPC Control de Péndulo con Carro', 'NumberTitle', 'off');

% Gráfico 1: Seguimiento de referencia
subplot(3, 1, 1);
plot(t_history, y_history, 'b-', 'LineWidth', 2);
hold on;
plot(t_history, r*ones(size(t_history)), 'r--', 'LineWidth', 2, 'DisplayName', sprintf('Referencia r=%g', r));
fill_y_upper = r * 1.02 * ones(size(t_history));
fill_y_lower = r * 0.98 * ones(size(t_history));
fill(t_history, fill_y_upper, 'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
fill(t_history, fill_y_lower, 'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
ylabel('Posición y [m]', 'FontSize', 11);
title('PARTE C: Respuesta del Sistema con Controlador MPC (N=4)', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
alpha(0.3);
legend('Posición y(t)', sprintf('Referencia r=%g', r), 'Banda ±2%', 'Location', 'best');
xlim([0, t_sim]);

% Gráfico 2: Acción de control
subplot(3, 1, 2);
plot(t_history, u_history, 'g-', 'LineWidth', 2);
hold on;
plot(t_history, u_max*ones(size(t_history)), 'r--', 'Alpha', 0.5, 'DisplayName', sprintf('±%gN', u_max));
plot(t_history, -u_max*ones(size(t_history)), 'r--', 'Alpha', 0.5);
ylabel('Control u [N]', 'FontSize', 11);
title('Acción de Control MPC', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
alpha(0.3);
legend('Control u(t)', sprintf('Límites ±%gN', u_max));
xlim([0, t_sim]);

% Gráfico 3: Error de seguimiento
subplot(3, 1, 3);
error = y_history - r;
plot(t_history, error, 'r-', 'LineWidth', 2);
hold on;
plot(t_history, zeros(size(t_history)), 'k-', 'Alpha', 0.3);
fill_error_upper = 0.02 * ones(size(t_history));
fill_error_lower = -0.02 * ones(size(t_history));
fill(t_history, fill_error_upper, 'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
fill(t_history, fill_error_lower, 'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
xlabel('Tiempo [s]', 'FontSize', 11);
ylabel('Error e(t) [m]', 'FontSize', 11);
title('Error de Seguimiento', 'FontSize', 12, 'FontWeight', 'bold');
grid on;
alpha(0.3);
legend('Error e(t) = y(t) - r', 'Banda error ±2%', 'Location', 'best');
xlim([0, t_sim]);

% Guardar figura
savefig('resultados_mpc.fig');
print('resultados_mpc.png', '-dpng', '-r150');
fprintf('✓ Gráfico guardado en: resultados_mpc.png\n\n');

%% ========================================================================
% PARÁMETROS DE DISEÑO DEL CONTROLADOR MPC
% =========================================================================

fprintf('================================================================================\n');
fprintf('PARÁMETROS DE DISEÑO DEL CONTROLADOR MPC\n');
fprintf('================================================================================\n\n');

fprintf('z (variable de optimización): Δu = [Δu₀, Δu₁, Δu₂, Δu₃]ᵀ\n\n');

fprintf('H (Hessiana, 4×4):\n');
disp(H);

fprintf('\nf (Gradiente, depende de estado actual x(k))\n');
fprintf('Ejemplo con x₀ = [0,0,0,0] y u_prev = 0:\n');
f_init = 2 * Lambda' * Q * (Phi * [0;0;0;0] + Psi * 0 - r * ones(N, 1));
disp(f_init');

fprintf('\nA_ineq (restricciones desigualdad, 8×4):\n');
disp(A_ineq);

fprintf('\nb_ineq = [10, 10, 10, 10, 10, 10, 10, 10]ᵀ\n');

fprintf('\nA_eq: No hay restricciones de igualdad adicionales\n');
fprintf('b_eq: (dinámica incorporada en Φ, Ψ, Λ)\n\n');

fprintf('lb (límite inferior): [-∞, -∞, -∞, -∞]ᵀ\n');
fprintf('ub (límite superior): [+∞, +∞, +∞, +∞]ᵀ\n\n');

fprintf('================================================================================\n');
fprintf('SIMULACIÓN COMPLETADA\n');
fprintf('================================================================================\n');
