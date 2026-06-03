%% ============================================================
%  Control H-infinito de un PENDULO CON RUEDA DE REACCION
%  Control de Procesos - UTEC
%
%  Diseno de mezcla de sensibilidades (mixed-sensitivity S/KS/T)
%  para una planta INESTABLE (polo en el semiplano derecho).
%
%  *** NO REQUIERE Robust Control Toolbox ***
%  Solo usa Control System Toolbox (tf, ss, feedback, step, margin,
%  stepinfo, c2d, ...) + MATLAB base (schur, ordschur, eig).
%
%  Las funciones makeweight / augw / hinfsyn de la Robust Control Toolbox
%  se reimplementan al final del archivo como funciones locales:
%      local_makeweight, local_augw, local_hinfsyn
%
%  Estructura del archivo:
%   1) Parametros fisicos del modelo
%   2) Funcion de transferencia de la planta (entrada V -> salida theta)
%   3) Diseno H-infinito NOMINAL (con explicacion de las ponderaciones)
%   4) Verificacion: gamma, polos de lazo cerrado, margenes
%   5) Simulacion: respuesta al escalon y senal de control
%   6) Variacion de parametros (4 casos) y graficas comparativas
%   7) Discretizacion del controlador
%   8) Funciones locales (makeweight, augw, hinfsyn, riccati)
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
%  Sintaxis (local): local_makeweight(gananciaBaja, frecCruce, gananciaAlta)
%    -> el segundo argumento es la frecuencia de cruce a 0 dB.
%
%  *** Clave para esta planta ***: el cruce de W1 (ancho de banda exigido)
%  se coloca por ENCIMA del polo inestable (~434 rad/s).
% ---------------------------------------------------------------------

W1 = local_makeweight(100, 1000, 0.01);   % S: alta ganancia DC -> seguimiento
W2 = 0.05;                                 % KS: penaliza (poco) el control
W3 = local_makeweight(0.01, 2500, 100);    % T: alta ganancia alta frec -> robustez

% Planta generalizada (aumentada con las ponderaciones)
P = local_augw(Gp, W1, W2, W3);

% Sintesis H-infinito:  1 medida (error), 1 control (voltaje)
[Sc, gamma] = local_hinfsyn(P, 1, 1);
Gc = minreal(tf(Sc));               % controlador como funcion de transferencia

fprintf('================ CONTROLADOR NOMINAL ================\n');
fprintf('gamma (norma H-inf alcanzada) = %.4f\n', gamma);
fprintf('Estados del controlador: %d\n', order(Sc));
disp('Controlador Gc(s):'); disp(zpk(Gc));

%% ============================================================
%  4) VERIFICACION DE ESTABILIDAD Y MARGENES
% ============================================================
L  = Gp*Gc;                 % lazo abierto compensado
T  = feedback(L, 1);        % lazo cerrado r -> theta  (= sens. complementaria)
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
%  Caso 1 (nominal): equilibrio seguimiento/robustez.
%  Caso 2: W1 con MAYOR ganancia DC  -> fuerza menor error estacionario
%          (pero, por la limitacion del polo inestable, AUMENTA el
%           sobreimpulso: compromiso fundamental "waterbed").
%  Caso 3: W3 que penaliza T desde MENOR frecuencia -> mas robustez,
%          lazo mas conservador (menor ancho de banda efectivo).
%  Caso 4: MAYOR ancho de banda en W1 y W3 -> respuesta mas rapida.
% ---------------------------------------------------------------------

casos(1) = struct('nombre','Caso 1 (nominal)', ...
    'W1',local_makeweight(100,1000,0.01), 'W2',0.05, 'W3',local_makeweight(0.01,2500,100));
casos(2) = struct('nombre','Caso 2 (W1 DC=500)', ...
    'W1',local_makeweight(500,1000,0.01), 'W2',0.05, 'W3',local_makeweight(0.01,2500,100));
casos(3) = struct('nombre','Caso 3 (W3 robusta, wc=1200)', ...
    'W1',local_makeweight(100,1000,0.01), 'W2',0.05, 'W3',local_makeweight(0.01,1200,100));
casos(4) = struct('nombre','Caso 4 (BW amplio wc=2000)', ...
    'W1',local_makeweight(100,2000,0.01), 'W2',0.05, 'W3',local_makeweight(0.01,4000,100));

colores = lines(numel(casos));

figStep = figure('Name','Comparacion: respuesta al escalon','Position',[80 80 1000 520]); hold on;
figU    = figure('Name','Comparacion: senal de control','Position',[80 80 1000 520]); hold on;

fprintf('\n================ COMPARACION DE CASOS ================\n');
fprintf('%-30s %8s %10s %8s %10s\n','Caso','gamma','tr[ms]','OS[%]','ess');

for k = 1:numel(casos)
    Pk = local_augw(Gp, casos(k).W1, casos(k).W2, casos(k).W3);
    [Sck, gk] = local_hinfsyn(Pk, 1, 1);
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
hl = yline(1,'k--','LineWidth',0.8);     % referencia, fuera de la leyenda
set(hl,'HandleVisibility','off');
grid on; box on; xlabel('Tiempo [ms]'); ylabel('\theta (normalizado)');
title('Respuesta al escalon en lazo cerrado - variacion de ponderaciones H\infty');
ylim([-0.5 3.5]);
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


%% ============================================================
%  8) FUNCIONES LOCALES  (reemplazan a la Robust Control Toolbox)
% ============================================================

function W = local_makeweight(dcgain, wc, hfgain)
%LOCAL_MAKEWEIGHT  Peso de primer orden equivalente a makeweight.
%   |W(0)| = dcgain, |W(Inf)| = hfgain, |W(j*wc)| = 1 (cruce a 0 dB).
%   W(s) = (hfgain*s + wb*dcgain) / (s + wb)
%   con wb elegido para que el cruce este exactamente en wc.
    wb = wc * sqrt((1 - hfgain^2) / (dcgain^2 - 1));
    W  = tf([hfgain, wb*dcgain], [1, wb]);
end


function P = local_augw(G, W1, W2, W3)
%LOCAL_AUGW  Planta generalizada de mezcla de sensibilidades (= augw).
%   Entradas:  [w ; u]      (referencia/perturbacion ; control)
%   Salidas :  [z1; z2; z3; v]
%       z1 = W1*(w - G*u) = W1*S*w      (sensibilidad ponderada)
%       z2 = W2*u         = W2*K*S*w    (esfuerzo de control ponderado)
%       z3 = W3*G*u       = W3*T*w      (sens. complementaria ponderada)
%       v  = w - G*u      = error (lo que mide el controlador)
%
%   Estructura por bloques:
%        | W1   -W1*G |
%   P =  | 0     W2   |
%        | 0     W3*G |
%        | 1    -G    |
    G  = tf(G);  W1 = tf(W1);  W2 = tf(W2);  W3 = tf(W3);
    z  = tf(0);  one = tf(1);
    P  = [ W1,  -W1*G ;
           z,    W2   ;
           z,    W3*G ;
           one, -G    ];
    P  = minreal(ss(P));        % realizacion de estados (minima)
end


function [K, gam] = local_hinfsyn(P, nmeas, ncon)
%LOCAL_HINFSYN  Controlador central H-infinito (sub)optimo (= hinfsyn).
%   Algoritmo de las DOS ecuaciones de Riccati (Doyle-Glover-Khargonekar-
%   Francis, 1989) con busqueda por biseccion en gamma. Resuelve las
%   Riccati por descomposicion de Schur del Hamiltoniano (solo MATLAB base).
%
%   [K, gam] = local_hinfsyn(P, nmeas, ncon)
%     P     : planta generalizada (ss)
%     nmeas : numero de salidas medidas (ultimas filas de C)
%     ncon  : numero de entradas de control (ultimas columnas de B)

    P = ss(P);
    A = P.A; B = P.B; C = P.C; D = P.D;
    n  = size(A,1);  nm = nmeas;  nc = ncon;
    nw = size(B,2) - nc;          nz = size(C,1) - nm;

    B1  = B(:,1:nw);      B2  = B(:,nw+1:end);
    C1  = C(1:nz,:);      C2  = C(nz+1:end,:);
    D11 = D(1:nz,1:nw);   D12 = D(1:nz,nw+1:end);
    D21 = D(nz+1:end,1:nw);

    % Empaquetamos las matrices de la planta generalizada para pasarlas
    % a la rutina del controlador central.
    pg = struct('A',A,'B1',B1,'B2',B2,'C1',C1,'C2',C2, ...
                'D11',D11,'D12',D12,'D21',D21, ...
                'n',n,'nz',nz,'nw',nw,'nm',nm,'nc',nc);

    % --- Fase 1: biseccion para hallar el gamma minimo factible ------
    %  "Factible" = existen soluciones de Riccati validas Y el controlador
    %  resultante ESTABILIZA INTERNAMENTE la planta generalizada (se verifica
    %  con los autovalores del lazo cerrado, no solo con las condiciones de
    %  Riccati: cerca del optimo el controlador central es fragil).
    lo = 1; hi = 1e5; gmin = NaN;
    for it = 1:60
        mid = sqrt(lo*hi);
        [~, ok] = central(pg, mid);
        if ok
            hi = mid;  gmin = mid;             % factible -> bajar gamma
        else
            lo = mid;                          % infactible -> subir gamma
        end
    end
    if isnan(gmin)
        error('local_hinfsyn: no se encontro controlador factible. Ajuste W1/W2/W3.');
    end

    % --- Fase 2: relajar gamma para un controlador bien condicionado --
    %  Disenar exactamente en gmin da un controlador al borde de la
    %  singularidad (Z = inv(I - Y*X/gamma^2) casi singular). Relajamos
    %  gamma un 15 % para ganar robustez/condicionamiento numerico.
    relax = 1.15;
    gam = relax*gmin;
    [K, ok] = central(pg, gam);
    if ~ok                                     % margen extra si hiciera falta
        gam = 1.30*gmin;  [K, ok] = central(pg, gam);
    end
    if ~ok || isempty(K)
        error('local_hinfsyn: fallo al reconstruir el controlador relajado.');
    end
end


function [Kc, okf] = central(pg, g)
%CENTRAL  Controlador central H-infinito para un gamma dado (o aviso de
%   infactibilidad). Implementa las formulas de las dos Riccati (DGKF).
    Kc = [];  okf = false;
    A=pg.A; B1=pg.B1; B2=pg.B2; C1=pg.C1; C2=pg.C2;
    D11=pg.D11; D12=pg.D12; D21=pg.D21;
    n=pg.n; nz=pg.nz; nw=pg.nw; nm=pg.nm; nc=pg.nc;
    g2 = g^2;
    Dz1 = [D11, D12];     Bz = [B1, B2];

    % Ecuacion de Riccati en X
    R  = Dz1'*Dz1 - blkdiag(g2*eye(nw), zeros(nc));
    if rcond(R) < 1e-12, return; end
    Ri = inv(R);
    Sx = C1'*Dz1;         Qx0 = C1'*C1;
    Ax = A  - Bz*Ri*Sx';
    Qx = Qx0 - Sx*Ri*Sx';
    [X, okX] = ric_schur(Ax, Bz*Ri*Bz', Qx);
    if ~okX, return; end

    % Ecuacion de Riccati en Y (dual)
    Dy = [D11; D21];      Cy = [C1; C2];
    Rb = Dy*Dy' - blkdiag(g2*eye(nz), zeros(nm));
    if rcond(Rb) < 1e-12, return; end
    Rbi = inv(Rb);
    Sy = B1*Dy';          Qy0 = B1*B1';
    Ay = (A - Sy*Rbi*Cy)';
    Qy = Qy0 - Sy*Rbi*Sy';
    [Y, okY] = ric_schur(Ay, Cy'*Rbi*Cy, Qy);
    if ~okY, return; end

    % Condiciones de factibilidad H-infinito
    if min(real(eig(X))) < -1e-6 || min(real(eig(Y))) < -1e-6, return; end
    if max(abs(eig(X*Y))) >= g2, return; end     % rho(XY) < gamma^2

    % Formulas del controlador central
    R12 = D12'*D12;   R21 = D21*D21';
    F   = -(R12) \ (B2'*X + D12'*C1);
    Lg  = -(Y*C2' + B1*D21') / R21;
    Z   =  inv(eye(n) - (1/g2)*Y*X);
    Ak  = A + (1/g2)*B1*B1'*X + B2*F + Z*Lg*C2;
    Bk  = -Z*Lg;
    Ck  = F;

    % --- Verificacion de ESTABILIDAD INTERNA del lazo cerrado --------
    %  Acl = matriz de estados del lazo (planta generalizada + controlador,
    %  con Dk = 0 y D22 = 0). Si algun autovalor no es estable, este gamma
    %  NO es valido aunque las Riccati "pasen".
    Acl = [A,      B2*Ck ;
           Bk*C2,  Ak    ];
    if max(real(eig(Acl))) >= -1e-9
        return;
    end

    Kc  = ss(Ak, Bk, Ck, zeros(nc,nm));
    okf = true;
end


function [X, ok] = ric_schur(A, RBB, Q)
%RIC_SCHUR  Solucion estabilizante de  A'X + X A - X*RBB*X + Q = 0.
%   Via subespacio invariante estable del Hamiltoniano (Schur ordenado).
    n = size(A,1);
    H = [ A,   -RBB ;
         -Q,   -A'  ];
    ev = eig(H);
    if sum(real(ev) < 0) ~= n          % debe haber n autovalores estables
        X = []; ok = false; return;
    end
    [U, TT] = schur(H, 'real');
    [U, ~ ] = ordschur(U, TT, 'lhp');  % estables (Re<0) arriba a la izquierda
    U11 = U(1:n, 1:n);
    U21 = U(n+1:2*n, 1:n);
    if rcond(U11) < 1e-12
        X = []; ok = false; return;
    end
    X  = real(U21 / U11);
    ok = true;
end
