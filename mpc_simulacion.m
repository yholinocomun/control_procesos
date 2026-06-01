%% ======================================================================
%  SIMULACION MPC EN LAZO CERRADO (equivalente al modelo de Simulink)
%
%  Reproduce el diagrama:
%     Step (r) --> Controlador MPC fcn(r,x) --> u --> Planta --> y, x
%                         ^                                         |
%                         |_________ realimentacion de x __________|
%
%  Todo en un solo archivo .m, SIN Simulink.
% ======================================================================
clear; clc; close all;

%% ---------------------- PARAMETROS DEL SISTEMA ------------------------
Ad = [0.9909    0.0861
     -0.1722    0.7326];     % matriz de estados (discreta)
Bd = [0.0045
      0.0861];               % matriz de entrada
Cd = [1, 0];                 % salida y = x1
Dd = 0;

Q = diag([10, 1]);           % peso de estados
R = 0.1;                     % peso de control
umax  = 10;                  % limite de control
x1max = 10;                  % limite estado 1
x2max = 10;                  % limite estado 2

N = 5;                       % <-- HORIZONTE DE PREDICCION (cambia: 1,2,5,...)

%% ---------------------- PARAMETROS DE SIMULACION ----------------------
Ts    = 0.1;                 % periodo de muestreo (ZOH / Rate Transition)
Tfin  = 10;                  % tiempo final de simulacion [s]
kmax  = round(Tfin/Ts);      % numero de muestras del controlador

% Sub-muestreo para integrar la planta CONTINUA dentro de cada Ts
Nsub  = 20;                  % pasos finos por periodo (curva suave)
dt    = Ts/Nsub;             % paso de integracion continuo

% Referencia tipo escalon (bloque Step): r = 1 a partir de t>=0
r = 1;

% Condiciones iniciales
x = [0; 0];                  % estado inicial de la planta

%% --------- RECONSTRUCCION DE LA PLANTA CONTINUA (d2c, ZOH) ------------
%  En Simulink la planta es continua (1/s) y el control entra por un ZOH.
%  Invertimos la discretizacion ZOH:   Ad = e^{Ac*Ts},  Bd = Ac^{-1}(Ad-I)Bc
Ac = logm(Ad)/Ts;                 % matriz continua de estados
Bc = (Ad - eye(size(Ad,1)))\(Ac*Bd);      % matriz continua de entrada
% Discretizacion fina para integrar exacto con u constante (ZOH en dt):
Ad_f = expm(Ac*dt);
Bd_f = Ac\(Ad_f - eye(size(Ad_f)))*Bc;

% Vectores para guardar resultados (resolucion fina)
Ntot   = kmax*Nsub;
t_hist = (0:Ntot)*dt;
x_hist = zeros(2, Ntot+1);   x_hist(:,1) = x;
u_hist = zeros(1, Ntot);
y_hist = zeros(1, Ntot+1);   y_hist(1) = Cd*x;

%% ---------------------- BUCLE DE SIMULACION ---------------------------
idx = 1;
for k = 1:kmax
    % 1) El controlador MPC se actualiza UNA vez por periodo Ts (ZOH)
    %    usando su modelo DISCRETO interno (Ad, Bd) para predecir.
    u = fcn(r, x, N, Ad, Bd, Q, R, umax, x1max, x2max);

    % 2) La planta CONTINUA evoluciona durante Ts con u constante (ZOH)
    for s = 1:Nsub
        x = Ad_f*x + Bd_f*u;          % integracion exacta del tramo dt
        y = Cd*x + Dd*u;
        u_hist(idx)     = u;
        x_hist(:,idx+1) = x;
        y_hist(idx+1)   = y;
        idx = idx + 1;
    end
end

%% ---------------------- GRAFICAS --------------------------------------
figure('Name','Simulacion MPC','Color','w');

subplot(3,1,1);
plot(t_hist, y_hist, 'b', 'LineWidth', 1.5); hold on;
yline(r, 'k--', 'LineWidth', 1);
grid on; ylabel('y (salida)'); title(sprintf('Respuesta en lazo cerrado (N = %d)', N));
legend('y', 'referencia r', 'Location','best');

subplot(3,1,2);
stairs(t_hist(1:end-1), u_hist, 'r', 'LineWidth', 1.5); hold on;
yline( umax, 'k--'); yline(-umax, 'k--');
grid on; ylabel('u (control ZOH)'); legend('u','limites \pm umax','Location','best');

subplot(3,1,3);
plot(t_hist, x_hist(1,:), 'LineWidth', 1.5); hold on;
plot(t_hist, x_hist(2,:), 'LineWidth', 1.5);
grid on; ylabel('estados'); xlabel('tiempo [s]');
legend('x_1','x_2','Location','best');

%% ======================================================================
%  FUNCION DEL CONTROLADOR MPC (la misma que va en el bloque "fcn")
% ======================================================================
function u = fcn(r, x, N, Ad, Bd, Q, R, umax, x1max, x2max)

    [n, m] = size(Bd);       % n = nº estados, m = nº entradas

    % --- Matrices aumentadas de prediccion: X = Gamma*x + Phi*U ---
    Gamma = zeros(N*n, n);
    Phi   = zeros(N*n, N*m);
    for k = 1:N
        Gamma((k-1)*n+1:k*n, :) = Ad^k;
        for j = 1:k
            Phi((k-1)*n+1:k*n, (j-1)*m+1:j*m) = Ad^(k-j) * Bd;
        end
    end

    % --- Pesos aumentados ---
    Q_bar = kron(eye(N), Q);
    R_bar = kron(eye(N), R);

    % --- Costo cuadratico: J = U'*H*U + f'*U ---
    H = 2*(R_bar + Phi'*Q_bar*Phi);
    H = (H + H')/2;
    f = 2*(Phi'*Q_bar*Gamma*x);

    % --- Restricciones de entrada ---
    lb = -umax*ones(N*m, 1);
    ub =  umax*ones(N*m, 1);

    % --- Restricciones de estado: Ain*U <= bin ---
    xmin = [-x1max; -x2max];
    xmax = [ x1max;  x2max];
    Ain = zeros(2*n*N, N*m);
    bin = zeros(2*n*N, 1);
    for k = 1:N
        Phi_k   = Phi((k-1)*n+1:k*n, :);
        Gamma_k = Gamma((k-1)*n+1:k*n, :);
        x_lib   = Gamma_k*x;

        fs = (k-1)*2*n + 1;   % filas restriccion superior
        fi = fs + n;          % filas restriccion inferior

        Ain(fs:fs+n-1, :) =  Phi_k;   bin(fs:fs+n-1) =  xmax - x_lib;
        Ain(fi:fi+n-1, :) = -Phi_k;   bin(fi:fi+n-1) = -xmin + x_lib;
    end

    % --- Solver QP (igual que el original: active-set) ---
    z0 = zeros(N*m, 1);
    options = optimoptions('quadprog', 'Algorithm', 'active-set', 'Display', 'off');
    z = quadprog(H, f, Ain, bin, [], [], lb, ub, z0, options);

    u = z(1);   % se aplica solo la primera accion de control
end
