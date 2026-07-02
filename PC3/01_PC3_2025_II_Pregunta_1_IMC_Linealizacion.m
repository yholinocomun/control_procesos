%% Practica Calificada 3 - 2025 II
%% Pregunta 1: Linealizacion, controlador IMC y discretizacion
% Item a)
clear all; close all; clc;

% Parametros del sistema
Rf = 10; Lf = 1; Cf = 0.1;

% Puntos de equilibrio
x1_eq = 0; x2_eq = 0; u_eq = 1;

% Espacio de estados no lineal
syms x1 x2 u
f1 = -Rf/Lf*x1 + 1/Lf*x2;
f2 = -1/Cf*x1 + 1/Cf*(u - x2)*(u - x2 - 1)*(u - x2 - 2);
f = [f1; f2];

% Jacobianos
Jx = jacobian(f, [x1 x2]); % Matriz A simbolica
Ju = jacobian(f, u);       % Matriz B simbolica

% Evaluacion en el punto de equilibrio
A = double(subs(Jx, {x1, x2, u}, {x1_eq, x2_eq, u_eq}));
B = double(subs(Ju, {x1, x2, u}, {x1_eq, x2_eq, u_eq}));

% Espacio de estados linealizado
C = [0 1];
D = 0;
Sp = ss(A, B, C, D);

% Items b) y c): diseno IMC
s = tf('s');
Gp = zpk(tf(Sp));
Gm = Gp;

% Parte invertible y no invertible
Gp_inv = Gp; %#ok<NASGU> % Toda la planta es invertible
Gp_noinv = 1; %#ok<NASGU>

% Diseno del filtro
lambda = 0.1;
gamma = 0;
n = 2;
F = (gamma*s + 1)/((lambda*s + 1)^n);
Q = inv(Gm)*F;

% Diseno del controlador IMC
Gc = minreal(feedback(Q, Gm, +1));

% Lazo cerrado y simulacion
Gt = feedback(Gc*Gp, 1);
figure;
step(Gt)
title('PC3 2025-II P1: Controlador IMC')
stepinfo(Gt)

% Items d) y e): discretizacion
Ts = 0.1;
Spd = c2d(Sp, Ts, 'zoh');
Ad = Spd.A
Bd = Spd.B
