%% Controlabilidad y Observabilidad - Sistema Discreto
clc; clear; close all;

%% Sistema
A = [-1  0;
      0 -2];
B = [2; 3];
C = [1  1];
D = 0;

n = size(A,1);

%% Controlabilidad
Co = ctrb(A,B);

disp('Co =');
disp(Co);
fprintf('rango(Co) = %d\n', rank(Co));
fprintf('det(Co)   = %.4f\n', det(Co));
if rank(Co) == n
    fprintf('Controlable\n\n');
else
    fprintf('No controlable\n\n');
end

%% Observabilidad
Ob = obsv(A,C);

disp('Ob =');
disp(Ob);
fprintf('rango(Ob) = %d\n', rank(Ob));
fprintf('det(Ob)   = %.4f\n', det(Ob));
if rank(Ob) == n
    fprintf('Observable\n\n');
else
    fprintf('No observable\n\n');
end

