%% Practica Calificada 3 - 2025 II
%% Pregunta 2: Sistema MIMO y controlador por desacoplamiento
clear all; close all; clc;

% Parametros del sistema
s = tf('s');
m = 5; b = 0.2;

% Item a): modelo en espacio de estados
A = [0 1 0 0;
     0 -b/m 0 0;
     0 0 0 1;
     0 0 0 -b/m];
B = [0 0;
     1/m 0;
     0 0;
     0 1/m];
C = [1 0 0 0;
     0 0 1 0];
D = [0 0;
     0 0];

Sp = ss(A, B, C, D);

% Matriz de funcion de transferencia
Gp = zpk(tf(Sp));
figure;
step(Gp)
title('PC3 2025-II P2: Planta MIMO sin control')

% Items b) y c): controlador por desacoplamiento/cancelacion de polos
tau = 100/4; % Tiempo de estabilizacion deseado
Gl = [1/(tau*s) 0;
      0 1/(tau*s)];
Gc = inv(Gp)*Gl;

% Lazo cerrado y simulacion
Gt = feedback(Gp*Gc, eye(2));
figure;
step(Gt)
title('PC3 2025-II P2: Sistema desacoplado')
stepinfo(Gt(1,1))
stepinfo(Gt(2,2))
