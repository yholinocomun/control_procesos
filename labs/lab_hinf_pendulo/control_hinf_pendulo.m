%% ============================================================
%  Control H-infinito de un PENDULO CON RUEDA DE REACCION
%  Control de Procesos - UTEC
%
%  Diseno de mezcla de sensibilidades (mixed-sensitivity S/KS/T)
%  para una planta INESTABLE (polo en el semiplano derecho).
%
%  Requiere: Control System Toolbox + Robust Control Toolbox
%  Funciones clave: makeweight, augw, hinfsyn, feedback, stepinfo
%
%  Estructura del archivo:
%   1) Parametros fisicos del modelo
%   2) Funcion de transferencia de la planta (entrada V -> salida theta)
%   3) Diseno H-infinito NOMINAL (con explicacion de las ponderaciones)
%   4) Verificacion: gamma, polos de lazo cerrado, margenes
%   5) Simulacion: respuesta al escalon y senal de control
%   6) Variacion de parametros (4 casos) y graficas comparativas
%   7) Discretizacion del controlador
% ============================================================

clear; clc; close all;
set(0,'DefaultAxesFontSize',12);
set(0,'DefaultLineLineWidth',1.6);

%% ============================================================
%  1) PARAMETROS DEL MODELO
% ============================================================
Kt = 25e-3;      % N.m/A        Constante de torque del motor
Ke = 25e-3;      % V/(rad/s)    Constante contra-electromotriz
Ra = 3.5;        % Ohm          Resistencia de armadura
Gr = 300;        % -            Relacion de engranajes
bf = 1e-4;       % N.m/(rad/s)  Friccion rotacional

Jp = 1.9e-3;     % kg.m^2       Inercia del pendulo
Jm = 1e-5;       % kg.m^2       Inercia del motor
Jw = 2.7e-5;     % kg.m^2       Inercia de la rueda

mp = 0.31;       % kg           Masa del pendulo
mw = 0.03;       % kg           Masa de la rueda

lg = 8e-2;       % m            Distancia al centro de masa del pendulo
rw = 3e-2;       % m            Radio de la rueda
g  = 9.81;       % m/s^2        Gravedad

%% ============================================================
%  2) PLANTA: funcion de transferencia V(s) -> theta(s)
% ============================================================
s = tf('s');

num = (Kt/Ra*Gr)*mp*lg*rw;   % numerador (ganancia electromecanica)
% Denominador: (inercia equivalente)*(inercia del pendulo)*s^2 - m*g*l
den = ((Jm/Gr^2)+Jw+mw*rw^2+mp*rw^2)*(Jp+mp*lg^2)*s^2 - mp*g*lg;

ft = minreal(num/den);       % planta nominal
Gp = ft;                     % alias para mantener la notacion del codigo base

% --- Diagnostico de la planta ----------------------------------------
fprintf('================ PLANTA ================\n');
disp(zpk(ft));
p = pole(ft);
fprintf('Polos de la planta:\n'); disp(p);
fprintf('==> La planta es INESTABLE: hay un polo en +%.1f rad/s\n', max(real(p)));
fprintf('    (proviene del termino de gravedad -mp*g*lg, signo negativo).\n');
fprintf('    CONSECUENCIA DE DISENO: el ancho de banda del lazo cerrado debe\n');
fprintf('    superar la frecuencia del polo inestable (~%.0f rad/s). Por eso\n', max(real(p)));
fprintf('    las ponderaciones se colocan con cruce ~1000-3000 rad/s (NO ~1 rad/s).\n\n');

%% ============================================================
%  3) DISENO H-INFINITO NOMINAL  (Caso 1)
% ============================================================
%  Ponderaciones de la mezcla de sensibilidades:
%
%   W1 -> pesa la SENSIBILIDAD  S = 1/(1+GK)   (error de seguimiento /
%         rechazo de perturbaciones). Se quiere |S| pequena en BAJA
%         frecuencia, por eso W1 tiene GANANCIA ALTA en baja frecuencia
%         (1/W1 acota a S por arriba).
%
%   W2 -> pesa la senal de control KS = K/(1+GK). Constante pequena para
%         limitar el esfuerzo/actuacion sin sobre-restringir.
%
%   W3 -> pesa la SENSIBILIDAD COMPLEMENTARIA  T = GK/(1+GK). Se quiere
%         |T| pequena en ALTA frecuencia (robustez ante dinamica no
%         modelada y ruido), por eso W3 tiene GANANCIA ALTA en alta
%         frecuencia.
%
%  Sintaxis: makeweight(gananciaBaja, frecCruce, gananciaAlta)
%    -> el segundo argumento (escalar) es la frecuencia de cruce a 0 dB.
%
%  *** Clave para esta planta ***: el cruce de W1 (ancho de banda exigido)
%  se coloca por ENCIMA del polo inestable (~434 rad/s).
% ---------------------------------------------------------------------

W1 = makeweight(100, 1000, 0.01);   % S: alta ganancia DC -> buen seguimiento
W2 = 0.05;                          % KS: penaliza (poco) el esfuerzo de control
W3 = makeweight(0.01, 2500, 100);   % T: alta ganancia en alta frec -> robustez

% Planta generalizada (aumentada con las ponderaciones)
P = augw(Gp, W1, W2, W3);

% Sintesis H-infinito:  1 medida (error), 1 control (voltaje)
[Sc, CL, gamma] = hinfsyn(P, 1, 1);
Gc = minreal(tf(Sc));               % controlador como funcion de transferencia

fprintf('================ CONTROLADOR NOMINAL ================\n');
fprintf('gamma (norma H-inf alcanzada) = %.4f\n', gamma);
fprintf('Estados del controlador: %d\n', order(Sc));
disp('Controlador Gc(s):'); disp(zpk(Gc));

%% ============================================================
%  4) VERIFICACION DE ESTABILIDAD Y MARGENES
% ============================================================
L  = Gp*Gc;                 % lazo abierto compensado
T  = feedback(L, 1);        % lazo cerrado r -> theta  (= sensibilidad complementaria)
Su = feedback(1, L);        % sensibilidad S
KS = feedback(Gc, Gp);      % r -> u  (senal de control = K*S)

clp = pole(T);
fprintf('\n================ LAZO CERRADO ================\n');
fprintf('Parte real maxima de los polos de LC: %.3g\n', max(real(clp)));
if all(real(clp) < 0)
    fprintf('==> LAZO CERRADO ESTABLE.\n');
else
    fprintf('==> ATENCION: lazo cerrado NO estable, ajustar ponderaciones.\n');
end

[Gm, Pm, Wcg, Wcp] = margin(L);
fprintf('Margen de ganancia = %.3g dB @ %.4g rad/s\n', 20*log10(Gm), Wcg);
fprintf('Margen de fase     = %.3g deg @ %.4g rad/s\n', Pm, Wcp);

%% ============================================================
%  5) SIMULACION DEL CASO NOMINAL
% ============================================================
%  La dinamica es MUY rapida (polo inestable a ~434 rad/s -> el lazo
%  cerrado trabaja en escala de milisegundos). Simulamos hasta ~70 ms.
tsim = linspace(0, 0.07, 6000);

figure('Name','Nominal: respuesta y control','Position',[80 80 1100 420]);
subplot(1,2,1);
step(T, tsim); grid on;
title('Respuesta al escalon (angulo del pendulo)');
xlabel('Tiempo'); ylabel('\theta (normalizado)');

subplot(1,2,2);
step(KS, tsim); grid on;
title('Senal de control u = K\cdotS');
xlabel('Tiempo'); ylabel('u [V]');

info = stepinfo(T);
fprintf('\nMetricas nominales:\n');
fprintf('  Tiempo de subida   = %.4g ms\n', info.RiseTime*1e3);
fprintf('  Sobreimpulso       = %.1f %%\n', info.Overshoot);
fprintf('  Tiempo de asent.   = %.4g ms\n', info.SettlingTime*1e3);
fprintf('  Error estacionario = %.4g\n', abs(1 - dcgain(T)));

%% ============================================================
%  6) VARIACION DE PARAMETROS  (4 casos comparados)
% ============================================================
%  Definimos cada caso por su terna de ponderaciones. Reutilizamos el
%  mismo flujo augw -> hinfsyn para todos.
%
%  Caso 1 (nominal): equilibrio seguimiento/robustez.
%  Caso 2: W1 con MAYOR ganancia DC  -> fuerza menor error estacionario
%          (pero, por la limitacion del polo inestable, AUMENTA el
%           sobreimpulso: compromiso fundamental "waterbed").
%  Caso 3: W3 que penaliza T desde MENOR frecuencia -> mas robustez,
%          lazo mas conservador (menor ancho de banda efectivo).
%  Caso 4: MAYOR ancho de banda en W1 y W3 -> respuesta mas rapida.
% ---------------------------------------------------------------------

casos(1) = struct('nombre','Caso 1 (nominal)', ...
    'W1',makeweight(100,1000,0.01), 'W2',0.05, 'W3',makeweight(0.01,2500,100));
casos(2) = struct('nombre','Caso 2 (W1 DC=500)', ...
    'W1',makeweight(500,1000,0.01), 'W2',0.05, 'W3',makeweight(0.01,2500,100));
casos(3) = struct('nombre','Caso 3 (W3 robusta, wc=1200)', ...
    'W1',makeweight(100,1000,0.01), 'W2',0.05, 'W3',makeweight(0.01,1200,100));
casos(4) = struct('nombre','Caso 4 (BW amplio wc=2000)', ...
    'W1',makeweight(100,2000,0.01), 'W2',0.05, 'W3',makeweight(0.01,4000,100));

colores = lines(numel(casos));

figStep = figure('Name','Comparacion: respuesta al escalon','Position',[80 80 1000 520]); hold on;
figU    = figure('Name','Comparacion: senal de control','Position',[80 80 1000 520]); hold on;

fprintf('\n================ COMPARACION DE CASOS ================\n');
fprintf('%-30s %8s %10s %8s %10s\n','Caso','gamma','tr[ms]','OS[%]','ess');

for k = 1:numel(casos)
    Pk = augw(Gp, casos(k).W1, casos(k).W2, casos(k).W3);
    [Sck, ~, gk] = hinfsyn(Pk, 1, 1);
    Gck = minreal(tf(Sck));

    Tk  = feedback(Gp*Gck, 1);     % r -> theta
    KSk = feedback(Gck, Gp);       % r -> u

    % Verificacion de estabilidad por caso
    if any(real(pole(Tk)) >= 0)
        fprintf('  [%s] lazo cerrado INESTABLE -> revisar ponderaciones\n', casos(k).nombre);
    end

    [yk, tk] = step(Tk, tsim);
    [uk, ~ ] = step(KSk, tsim);

    figure(figStep);
    plot(tk*1e3, yk, 'Color', colores(k,:), ...
         'DisplayName', sprintf('%s  (\\gamma=%.1f)', casos(k).nombre, gk));
    figure(figU);
    plot(tk*1e3, uk, 'Color', colores(k,:), 'DisplayName', casos(k).nombre);

    ik = stepinfo(Tk);
    fprintf('%-30s %8.2f %10.3f %8.0f %10.4f\n', ...
        casos(k).nombre, gk, ik.RiseTime*1e3, ik.Overshoot, abs(1-dcgain(Tk)));
end

figure(figStep);
yline(1,'k--','LineWidth',0.8);
grid on; box on; xlabel('Tiempo [ms]'); ylabel('\theta (normalizado)');
title('Respuesta al escalon en lazo cerrado - variacion de ponderaciones H\infty');
legend('Location','best','FontSize',9);

figure(figU);
grid on; box on; xlabel('Tiempo [ms]'); ylabel('u [V]');
title('Esfuerzo de control u = K\cdotS - comparacion de casos');
legend('Location','best','FontSize',9);

%% ============================================================
%  7) DISCRETIZACION DEL CONTROLADOR NOMINAL
% ============================================================
%  Para implementar en un microcontrolador / PLC. El periodo de muestreo
%  debe ser >> rapido que la dinamica (polo a ~434 rad/s => f ~ 70 Hz).
%  Se recomienda Ts ~ 0.5-1 ms (fs >= 10x el ancho de banda).
Ts = 5e-4;                         % 0.5 ms
Gc_d = c2d(Sc, Ts, 'tustin');      % Tustin (bilineal) preserva mejor la fase
fprintf('\n================ CONTROLADOR DISCRETO (Ts=%.3g s) ================\n', Ts);
disp(tf(Gc_d));

fprintf('\n[OK] Ejecucion completa. Revise las figuras generadas.\n');
