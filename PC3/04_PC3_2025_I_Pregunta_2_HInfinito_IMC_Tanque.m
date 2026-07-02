%% Practica Calificada 3 - 2025 I
%% Pregunta 2: Linealizacion, H-infinito e IMC para tanque
clear all; close all; clc;

% Item a): parametros del sistema
b = 5; g = 10; a = 1;
u_eq = 10;

% Espacio de estados no lineal
syms x u
f = u/b - (a*sqrt(2*g*x))/b;

% Punto de equilibrio
xs = solve(u_eq/b - (a*sqrt(2*g*x))/b == 0, x, 'ReturnConditions', true); %#ok<NASGU>
x_eq = 5;

% Jacobianos
Jx = jacobian(f, x);
Ju = jacobian(f, u);

% Evaluacion en el punto de equilibrio
A = double(subs(Jx, {x, u}, {x_eq, u_eq}));
B = double(subs(Ju, {x, u}, {x_eq, u_eq}));

% Espacio de estados linealizado
C = 1;
D = 0;
Sp = ss(A, B, C, D);
figure;
step(Sp)
title('PC3 2025-I P2: Planta sin control')

% Item b): controlador H-infinito
Gp = minreal(zpk(tf(Sp)));

W1 = makeweight(100, [1, 0.1], 0.01);
W2 = 0.01;
W3 = makeweight(0.01, [1, 0.1], 10);

P = augw(Gp, W1, W2, W3);
[Sc, CL, gamma] = hinfsyn(P, 1, 1); %#ok<ASGLU>

Gc = minreal(tf(Sc));
Gt = feedback(Gp*Gc, 1);
figure;
step(Gt)
title('PC3 2025-I P2: Controlador H-infinito')
stepinfo(Gt)

% Item c): controlador IMC
s = tf('s');
Gp = zpk(tf(Sp));
Gm = Gp;

Gp_inv = Gp; %#ok<NASGU> % Toda la planta es invertible
Gp_noinv = 1; %#ok<NASGU>

lambda = 0.1;
gamma = 0;
n = 2;
F = (gamma*s + 1)/((lambda*s + 1)^n);
Q = inv(Gm)*F;

Gc = minreal(feedback(Q, Gm, +1));
Gt = feedback(Gc*Gp, 1);
figure;
step(Gt)
title('PC3 2025-I P2: Controlador IMC')
stepinfo(Gt)
