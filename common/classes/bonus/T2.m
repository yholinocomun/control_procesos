clc; clear; close all;

% Definición de matrices
A = [-1  0;
      0 -2];

B = [2;
     3];

C = [1 1];  % 

% Matriz de Controlabilidad
Ctr = [B A*B];

rank_ctr = rank(Ctr);
det_ctr = det(Ctr);

% Matriz de Observabilidad
Obs = [C;
       C*A];

rank_obs = rank(Obs);
det_obs = det(Obs);

% Mostrar resultados
disp('matriz de Controlabilidad:')
disp(Ctr)

fprintf('rango: %d\n', rank_ctr)
fprintf('determinante: %.4f\n\n', det_ctr)

disp('matriz de Observabilidad:')
disp(Obs)

fprintf('rango: %d\n', rank_obs)
fprintf('determinante: %.4f\n', det_obs)