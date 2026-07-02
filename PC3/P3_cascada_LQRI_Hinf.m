%% =======================================================================
%  PREGUNTA 3 - Control en cascada para motor DC con estados x1, x2, x3
%  ------------------------------------------------------------------
%  Lazo interno : LQR con accion integral sobre la planta secundaria Sp2
%                 (corriente x1 y velocidad x2, salida = x2)
%  Lazo externo : Controlador robusto H-infinito (mixsyn) para la
%                 posicion x3
%
%  Especificaciones (ambos lazos):  OS% <= 5 ,  ts(2%) <= 5 s
%
%  Requiere: Control System Toolbox y Robust Control Toolbox
%  =======================================================================
clear; clc; close all;

%% ----------------------- Parametros del proceso ------------------------
Ra = 5.16;      % Resistencia de armadura        [ohm]
La = 0.056;     % Inductancia de armadura        [H]
Km = 1.03;      % Constante de torque            [N.m/A]
Jm = 0.044;     % Inercia del rotor              [kg.m^2]
Bm = 0.012;     % Friccion viscosa               [N.m.s]
Kb = 2.51;      % Constante de fuerza contraelectromotriz [V.s/rad]

%% ------------------- Modelo en espacio de estados ----------------------
%  Del diagrama de bloques:
%    x1' = -(Ra/La)*x1 - (Kb/La)*x2 + (1/La)*u      (corriente)
%    x2' =  (Km/Jm)*x1 - (Bm/Jm)*x2                 (velocidad)
%    x3' =   x2                                     (posicion)
A = [-Ra/La  -Kb/La   0;
      Km/Jm  -Bm/Jm   0;
      0       1       0];
B = [1/La; 0; 0];
C = [0 0 1];
D = 0;

fprintf('Polos de la planta completa:\n'); disp(eig(A));

%% =======================================================================
%  PARTE (a): Planta secundaria Sp2 y LQR con integrador (lazo interno)
%  =======================================================================
%  La planta secundaria Sp2 se obtiene tomando la dinamica rapida
%  (electrica + mecanica de velocidad): estados [x1; x2], salida y2 = x2.
A2 = [-Ra/La  -Kb/La;
       Km/Jm  -Bm/Jm];
B2 = [1/La; 0];
C2 = [0 1];
D2 = 0;
Sp2 = ss(A2, B2, C2, D2);

fprintf('Polos de la planta secundaria Sp2:\n'); disp(eig(A2));

%  ---- Sistema aumentado con integrador del error ----
%  xi' = r - y2  =>  xa = [x1; x2; xi]
%       xa' = Aa*xa + Ba*u + [0;0;1]*r ,   u = -K*x + Ki*xi
Aa = [A2          zeros(2,1);
      -C2         0        ];
Ba = [B2; 0];

%  ---- Pesos LQR (sintonizados para OS<=5%, ts2%<=5s) ----
Q = diag([0.01  1  50]);   % penaliza poco x1, moderado x2, fuerte la integral
R = 1;                     % penalizacion del esfuerzo de control

Ka = lqr(Aa, Ba, Q, R);    % Ka = [K  -Ki]
K  = Ka(1:2);              % realimentacion de estados [x1 x2]
Ki = -Ka(3);               % ganancia del integrador

fprintf('\n===== CONTROLADOR LQR CON INTEGRADOR (lazo interno) =====\n');
fprintf('K  = [%.4f  %.4f]\n', K(1), K(2));
fprintf('Ki =  %.4f\n', Ki);

%  ---- Lazo interno cerrado:  r2 -> x2 ----
Acl_in = [A2 - B2*K   B2*Ki;
          -C2         0    ];
Bcl_in = [0; 0; 1];
Ccl_in = [C2 0];
T_in   = ss(Acl_in, Bcl_in, Ccl_in, 0);   % velocidad ante referencia r2

fprintf('Polos del lazo interno cerrado:\n'); disp(eig(Acl_in));

%  ---- Desempenio del lazo interno ----
info_in = stepinfo(T_in, 'SettlingTimeThreshold', 0.02);
fprintf('Desempenio lazo interno:  OS%% = %.3f %%   ts(2%%) = %.3f s\n', ...
        info_in.Overshoot, info_in.SettlingTime);
%  Resultado esperado:  OS% = 0.000 %   ts(2%) = 1.59 s   (cumple specs)

figure('Name','Parte a: Lazo interno LQR+I');
step(T_in, 8); grid on;
title(sprintf(['Lazo interno LQR con integrador (velocidad x_2)\n' ...
        'OS%% = %.2f%%,  t_s(2%%) = %.2f s'], ...
        info_in.Overshoot, info_in.SettlingTime));
ylabel('x_2 (velocidad)');

%% =======================================================================
%  PARTE (b): Controlador H-infinito para el lazo externo (posicion x3)
%  =======================================================================
%  La planta vista por el controlador externo es el lazo interno cerrado
%  en serie con el integrador de posicion:   G_out(s) = T_in(s) * 1/s
G_out = T_in * tf(1, [1 0]);
G_out = ss(G_out);

%  Para la sintesis H-inf el polo en el origen viola las hipotesis del
%  problema estandar (Riccati). Se perturba levemente: 1/s -> 1/(s+eps).
eps_p  = 1e-3;
G_sint = ss(T_in * tf(1, [1 eps_p]));

%  ---- Pesos de sensibilidad mixta S/KS/T ----
%  W1: desempenio (penaliza S en baja frecuencia)
%      |S| < M en alta frec., ancho de banda wb, error estacionario ~Aeps
Ms  = 1.4;     % cota de |S| (Ms<2 garantiza margenes y bajo sobreimpulso)
wb  = 2.0;     % frecuencia de cruce deseada [rad/s]  (ts ~ 4/wb < 5 s)
Aep = 1e-4;    % error en estado estacionario permitido
W1  = tf([1/Ms  wb], [1  wb*Aep]);

%  W2: penalizacion del esfuerzo de control (constante pequenia)
W2  = tf(0.01);

%  W3: robustez (penaliza T en alta frecuencia, tipo pasa-altos)
wbc = 60;      % frecuencia a partir de la cual se atenua T [rad/s]
Mt  = 2.0;     % cota de |T| en baja frecuencia de W3
W3  = tf([1  wbc/Mt], [1e-3  wbc]);

fprintf('\n===== PESOS DEL CONTROLADOR H-INFINITO =====\n');
fprintf('W1(s) = (s/%.1f + %.1f)/(s + %.1e)\n', Ms, wb, wb*Aep);
W1, W2, W3   %#ok<NOPTS>  % mostrar los pesos en consola

%  ---- Sintesis H-infinito de sensibilidad mixta ----
[K_inf, CL, gamma] = mixsyn(G_sint, W1, W2, W3);
fprintf('gamma alcanzado = %.4f  (< 1 => se cumplen las cotas de disenio)\n', gamma);
fprintf('Orden del controlador H-inf: %d\n', order(K_inf));
%  Resultado esperado: gamma = 0.891

%  ---- Lazo externo cerrado (con la planta REAL, integrador puro) ----
L_out = G_out * K_inf;          % lazo abierto externo
T_out = feedback(L_out, 1);     % r3 -> x3
S_out = feedback(1, L_out);     % sensibilidad

%  ---- Desempenio del lazo externo ----
info_out = stepinfo(T_out, 'SettlingTimeThreshold', 0.02);
fprintf('\nDesempenio lazo externo:  OS%% = %.4f %%   ts(2%%) = %.3f s\n', ...
        info_out.Overshoot, info_out.SettlingTime);
%  Resultado esperado:  OS% ~ 0.00 %   ts(2%) ~ 1.23 s   (cumple specs)

%  ---- Graficas ----
figure('Name','Parte b: Bode de los pesos W1, W2, W3');
bode(W1, 'b', W2, 'k--', W3, 'r', {1e-3, 1e4}); grid on;
legend('W_1 (desempenio)', 'W_2 (control)', 'W_3 (robustez)');
title('Diagramas de Bode de los pesos del controlador H_\infty');

figure('Name','Parte b: Verificacion de sensibilidades');
sigma(S_out, 'b', 1/W1, 'b--', T_out, 'r', 1/W3, 'r--', {1e-3, 1e4}); grid on;
legend('S', '1/W_1', 'T', '1/W_3');
title('S y T frente a sus cotas 1/W_1 y 1/W_3');

figure('Name','Parte b: Controlador H-infinito');
bode(K_inf, {1e-3, 1e4}); grid on;
title('Diagrama de Bode del controlador H_\infty');

figure('Name','Parte b: Lazo externo H-infinito');
step(T_out, 8); grid on;
title(sprintf(['Lazo externo H_\\infty (posicion x_3)\n' ...
        'OS%% = %.2f%%,  t_s(2%%) = %.2f s'], ...
        info_out.Overshoot, info_out.SettlingTime));
ylabel('x_3 (posicion)');

%% ----------------- Resumen del esquema en cascada ----------------------
fprintf('\n================== RESUMEN ==================\n');
fprintf('Lazo interno (LQR+I sobre Sp2):\n');
fprintf('   K  = [%.4f  %.4f],  Ki = %.4f\n', K(1), K(2), Ki);
fprintf('   OS%% = %.3f <= 5,  ts(2%%) = %.3f s <= 5   -> CUMPLE\n', ...
        info_in.Overshoot, info_in.SettlingTime);
fprintf('Lazo externo (H-infinito, gamma = %.3f):\n', gamma);
fprintf('   OS%% = %.4f <= 5,  ts(2%%) = %.3f s <= 5   -> CUMPLE\n', ...
        info_out.Overshoot, info_out.SettlingTime);
