%% ============================================================
%  DEMO / PRUEBA del MPC generico (mpc_qp_step.m)
%  Control de Procesos - UTEC
%
%  Demuestra que el MISMO codigo resuelve el MPC para 2, 3, 4 y 5
%  estados sin cambiar nada mas que las matrices Ad, Bd y N.
%
%  Requiere Optimization Toolbox (quadprog).
% ============================================================
clear; clc;

addpath(fileparts(mfilename('fullpath')));   % asegura el path a mpc_qp_step

N = 5;   % horizonte comun para la demo

%% ---- Caso 1: 2 estados ----
Ad2 = [1 0.1; 0 1];
Bd2 = [0.005; 0.1];
probar('2 estados', Ad2, Bd2, N);

%% ---- Caso 2: 3 estados ----
Ad3 = [1 0.1 0.005; 0 1 0.1; 0 0 1];
Bd3 = [0.0005; 0.005; 0.1];
probar('3 estados', Ad3, Bd3, N);

%% ---- Caso 3: 4 estados ----
Ad4 = [1 0 0.1058 0.0051;
       0 1 0.0560 0.1000;
       0 0 1.1759 0.1058;
       0 0 3.6183 1.1759];
Bd4 = [0.0060; -0.0201; 0.1165; -0.2009];
probar('4 estados', Ad4, Bd4, N);

%% ---- Caso 4: 5 estados ----
Ad5 = [1 0.1 0 0 0;
       0 1 0.1 0 0;
       0 0 1 0.1 0;
       0 0 0 1 0.1;
       0 0 0 0 1];
Bd5 = [0.0001; 0.001; 0.01; 0.1; 0.5];
probar('5 estados', Ad5, Bd5, N);

fprintf('\n[OK] El mismo codigo resolvio el MPC para 2, 3, 4 y 5 estados.\n');

%% ------------------------------------------------------------
function probar(nombre, Ad, Bd, N)
    nx = size(Ad,1);
    r = [1; zeros(nx-1,1)];        % referencia: lleva el 1er estado a 1
    x = zeros(nx,1);               % estado inicial en reposo
    u = mpc_qp_step(r, x, Ad, Bd, N);
    fprintf('  %-12s -> nx=%d, N=%d, u(1) = %8.4f\n', nombre, nx, N, u);
end
