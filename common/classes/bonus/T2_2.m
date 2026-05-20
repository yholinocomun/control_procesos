 A = [-1 0; 0 -2];
B = [2; 3];
C = [1 1];

% Controlabilidad
Co = ctrb(A,B);
disp('Rango controlabilidad:'), disp(rank(Co))

% Observabilidad
Ob = obsv(A,C);
disp('Rango observabilidad:'), disp(rank(Ob))
 A = [-1 0; 0 -2];
B = [2; 3];
C = [1 1];

% Controlabilidad
Co = ctrb(A,B);
disp('Rango controlabilidad:'), disp(rank(Co))

% Observabilidad
Ob = obsv(A,C);
disp('Rango observabilidad:'), disp(rank(Ob))