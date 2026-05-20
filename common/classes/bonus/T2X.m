% Definición de las matrices del sistema
A = [-1, 0; 0, -2];
B = [2; 3];
C = [1, 1]; % Matriz de salida supuesta

% 1. Matriz de Controlabilidad
Co = ctrb(A, B);
rango_controlabilidad = rank(Co);

% 2. Matriz de Observabilidad
Ob = obsv(A, C);
rango_observabilidad = rank(Ob);

% Mostrar resultados
fprintf('Rango de la matriz de controlabilidad: %d\n', rango_controlabilidad);
if rango_controlabilidad == size(A,1)
    disp('El sistema es COMPLETAMENTE CONTROLABLE.');
else
    disp('El sistema NO es controlable.');
end

fprintf('Rango de la matriz de observabilidad: %d\n', rango_observabilidad);
if rango_observabilidad == size(A,1)
    disp('El sistema es COMPLETAMENTE OBSERVABLE.');
else
    disp('El sistema NO es observable.');
end