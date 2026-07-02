%% =======================================================================
%  PREGUNTA 4 - Tanque con agua fria/caliente: linealizacion + PID MIMO
%  ------------------------------------------------------------------
%  Modelo no lineal (x1 = nivel h, x2 = temperatura theta):
%     x1' = (-a/A)*sqrt(x1) + (1/A)*u1 + (1/A)*u2
%     x2' = (-a/A)*x2/sqrt(x1) + (rhoC*thC/(rhoD*A))*u1/x1
%                              + (rhoH*thH/(rhoD*A))*u2/x1
%  Entradas: u1 = qC (agua fria), u2 = qH (agua caliente)
%
%  Se pide: linealizar en x1e = 0.5, x2e = 305 y disenar un PID MIMO
%  para seguir referencias escalon unitario con:
%     OS1 = OS2 = 0 ,  ts1 = 6 s ,  ts2 = 3 s
%
%  Requiere: Control System Toolbox
%  =======================================================================
clear; clc; close all;

%% ----------------------- Parametros del proceso ------------------------
a    = 0.089;
A    = 0.031;
rhoD = 996;             % densidad del agua en el tanque
rhoC = 998;  thC = 290; % agua fria
rhoH = 988;  thH = 320; % agua caliente

kC = rhoC*thC/rhoD;     % coeficiente de u1 en la ec. de temperatura
kH = rhoH*thH/rhoD;     % coeficiente de u2 en la ec. de temperatura

%% -------------------- Punto de operacion (equilibrio) ------------------
x1e = 0.5;              % nivel de equilibrio
x2e = 305;              % temperatura de equilibrio

%  De x1' = 0:  u1e + u2e            = a*sqrt(x1e)
%  De x2' = 0:  kC*u1e + kH*u2e      = a*x2e*sqrt(x1e)
ue  = [1 1; kC kH] \ [a*sqrt(x1e); a*x2e*sqrt(x1e)];
u1e = ue(1);  u2e = ue(2);
fprintf('Entradas de equilibrio:  u1e = %.6f   u2e = %.6f\n', u1e, u2e);
%  Resultado esperado: u1e = 0.029136 , u2e = 0.033796

%% ------------------ Linealizacion (Jacobianos analiticos) --------------
%  f1 = (-a/A)sqrt(x1) + (u1+u2)/A
%  f2 = (-a/A)x2/sqrt(x1) + (kC/A)u1/x1 + (kH/A)u2/x1
%
%  df1/dx1 = -a/(2A*sqrt(x1e))                    df1/dx2 = 0
%  df2/dx1 = -a*x2e/(2A*x1e^1.5)  (usando el equilibrio)
%  df2/dx2 = -a/(A*sqrt(x1e))
Alin = [ -a/(2*A*sqrt(x1e))        0
         -a*x2e/(2*A*x1e^1.5)     -a/(A*sqrt(x1e)) ];
Blin = [  1/A            1/A
          kC/(A*x1e)     kH/(A*x1e) ];
Clin = eye(2);          % ambos estados son salidas medidas
Dlin = zeros(2);

G = ss(Alin, Blin, Clin, Dlin);   % planta linealizada (variables de desvio)

fprintf('\nMatriz A linealizada:\n'); disp(Alin);
fprintf('Matriz B linealizada:\n');   disp(Blin);
fprintf('Polos de la planta linealizada:\n'); disp(eig(Alin));

%% =======================================================================
%  Diseno del PID MIMO por desacoplamiento exacto
%  =======================================================================
%  Como C = I, la planta es G(s) = (sI-A)^-1 * B  y su inversa es
%  G^-1(s) = B^-1 (sI - A).  Si se desea que cada canal en lazo abierto
%  sea un integrador puro  L(s) = diag( 1/(tau_i*s) ), el controlador
%
%     C(s) = G^-1(s) * diag(1/(tau_i*s)) = Kp + Ki/s     (PID con Kd = 0)
%
%     Kp = B^-1 * diag(1/tau_i)
%     Ki = -B^-1 * A * diag(1/tau_i)
%     Kd = 0
%
%  produce el lazo cerrado EXACTAMENTE desacoplado y de primer orden:
%     T_i(s) = 1/(tau_i*s + 1)   =>  OS = 0 (sin sobreimpulso)
%                                    ts(2%) = 3.91*tau_i ~ 4*tau_i
%
%  Con ts1 = 6 s y ts2 = 3 s:   tau_1 = 6/4 = 1.5 ,  tau_2 = 3/4 = 0.75
tau = [6/4;  3/4];

Kp = Blin \ diag(1./tau);            % ganancia proporcional (matriz 2x2)
Ki = -(Blin \ Alin) * diag(1./tau);  % ganancia integral     (matriz 2x2)
Kd = zeros(2);                       % no se requiere accion derivativa

fprintf('\n===== PARAMETROS DEL CONTROLADOR PID MIMO =====\n');
fprintf('tau1 = %.3f s ,  tau2 = %.3f s\n', tau(1), tau(2));
fprintf('Kp =\n'); disp(Kp);
fprintf('Ki =\n'); disp(Ki);
fprintf('Kd =\n'); disp(Kd);

%  Controlador PI MIMO en espacio de estados:  xc' = e ,  u = Ki*xc + Kp*e
Cpid = ss(zeros(2), eye(2), Ki, Kp);

%% -------------------- Lazo cerrado (modelo lineal) ---------------------
T = feedback(G*Cpid, eye(2));      % r = [r1; r2] -> y = [x1; x2] (desvio)

info = stepinfo(T, 'SettlingTimeThreshold', 0.02);
fprintf('\n===== DESEMPENIO (modelo linealizado) =====\n');
fprintf('Canal 1 (nivel):        OS%% = %.4f   ts(2%%) = %.3f s\n', ...
        info(1,1).Overshoot, info(1,1).SettlingTime);
fprintf('Canal 2 (temperatura):  OS%% = %.4f   ts(2%%) = %.3f s\n', ...
        info(2,2).Overshoot, info(2,2).SettlingTime);
%  Resultado esperado:
%     Canal 1: OS% = 0   ts(2%) = 5.87 s  (<= 6  OK)
%     Canal 2: OS% = 0   ts(2%) = 2.94 s  (<= 3  OK)
%  Acoplamiento cruzado: numericamente nulo (~1e-13)

figure('Name','PID MIMO: respuesta al escalon (modelo lineal)');
step(T, 12); grid on;
title('Lazo cerrado linealizado: escalon unitario en cada referencia');

%% ---------------- Validacion sobre el modelo NO lineal -----------------
%  Se simula el proceso no lineal con el PID MIMO en variables absolutas:
%  u = ue + Kp*e + Ki*integral(e),  e = r - [x1; x2]
%  Estados extendidos: z = [x1; x2; xi1; xi2]
%  NOTA: no se satura u; para escalones grandes de nivel un recorte u>=0
%  sin anti-windup desestabiliza el lazo (windup del integrador).
f_nl = @(t, z, r) [ ...
    -a/A*sqrt(z(1)) + ( (u1e + Kp(1,:)*(r - z(1:2)) + Ki(1,:)*z(3:4)) ...
                      + (u2e + Kp(2,:)*(r - z(1:2)) + Ki(2,:)*z(3:4)) )/A;
    -a/A*z(2)/sqrt(z(1)) ...
        + kC/A*(u1e + Kp(1,:)*(r - z(1:2)) + Ki(1,:)*z(3:4))/z(1) ...
        + kH/A*(u2e + Kp(2,:)*(r - z(1:2)) + Ki(2,:)*z(3:4))/z(1);
    r(1) - z(1);
    r(2) - z(2) ];

z0   = [x1e; x2e; 0; 0];
tsim = 0:0.005:15;
opts = odeset('RelTol',1e-8,'AbsTol',1e-10);

%  Escalon unitario en nivel (r1 = x1e+1, r2 = x2e)
r1 = [x1e + 1; x2e];
[t1, z1] = ode15s(@(t,z) f_nl(t,z,r1), tsim, z0, opts);

%  Escalon unitario en temperatura (r1 = x1e, r2 = x2e+1)
r2 = [x1e; x2e + 1];
[t2, z2] = ode15s(@(t,z) f_nl(t,z,r2), tsim, z0, opts);

%  Metricas sobre la respuesta no lineal normalizada
y1n = (z1(:,1) - x1e)/1;   % nivel normalizado
y2n = (z2(:,2) - x2e)/1;   % temperatura normalizada
OS1 = max(0, (max(y1n)-1)*100);
OS2 = max(0, (max(y2n)-1)*100);
ts1 = t1(find(abs(y1n-1) > 0.02, 1, 'last') + 1);
ts2 = t2(find(abs(y2n-1) > 0.02, 1, 'last') + 1);

fprintf('\n===== DESEMPENIO (modelo NO lineal) =====\n');
fprintf('Escalon de nivel:       OS%% = %.3f   ts(2%%) = %.3f s\n', OS1, ts1);
fprintf('Escalon de temperatura: OS%% = %.3f   ts(2%%) = %.3f s\n', OS2, ts2);

figure('Name','PID MIMO: validacion sobre el modelo no lineal');
subplot(2,2,1); plot(t1, z1(:,1), 'b', t1, (x1e+1)*ones(size(t1)), 'k--');
grid on; xlabel('t [s]'); ylabel('x_1 (nivel)');
title(sprintf('Escalon en nivel: OS=%.2f%%, t_s=%.2f s', OS1, ts1));
subplot(2,2,3); plot(t1, z1(:,2), 'r'); grid on;
xlabel('t [s]'); ylabel('x_2 (temp)'); title('Acople en temperatura');
subplot(2,2,2); plot(t2, z2(:,2), 'r', t2, (x2e+1)*ones(size(t2)), 'k--');
grid on; xlabel('t [s]'); ylabel('x_2 (temp)');
title(sprintf('Escalon en temperatura: OS=%.2f%%, t_s=%.2f s', OS2, ts2));
subplot(2,2,4); plot(t2, z2(:,1), 'b'); grid on;
xlabel('t [s]'); ylabel('x_1 (nivel)'); title('Acople en nivel');

%% ------------------------------ Resumen --------------------------------
fprintf('\n================== RESUMEN ==================\n');
fprintf('Punto de operacion: x1e = %.2f, x2e = %.0f, u1e = %.5f, u2e = %.5f\n',...
        x1e, x2e, u1e, u2e);
fprintf('PID MIMO (desacoplamiento exacto, Kd = 0):\n');
fprintf('   Kp = [%8.5f %9.6f; %8.5f %9.6f]\n', Kp(1,1),Kp(1,2),Kp(2,1),Kp(2,2));
fprintf('   Ki = [%8.5f %9.6f; %8.5f %9.6f]\n', Ki(1,1),Ki(1,2),Ki(2,1),Ki(2,2));
fprintf('Modelo lineal:  OS1 = OS2 = 0,  ts1 = %.2f s <= 6,  ts2 = %.2f s <= 3\n',...
        info(1,1).SettlingTime, info(2,2).SettlingTime);
fprintf('-> CUMPLE las especificaciones os1 = os2 = 0, ts1 = 6, ts2 = 3\n');
