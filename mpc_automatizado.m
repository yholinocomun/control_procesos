function u = fcn(r, x)
%% ========================================================================
%  CONTROLADOR MPC AUTOMATIZADO
%  ------------------------------------------------------------------------
%  Lo ÚNICO que debes cambiar es N (horizonte de predicción).
%  TODAS las matrices (H, f, Aeq, beq, lb, ub) se construyen solas
%  a partir de N usando productos de Kronecker (kron).
%
%  Entradas:
%     r : referencia [nx x 1]   (lo que quieres que sigan los estados)
%     x : estado actual [nx x 1]
%  Salida:
%     u : acción de control a aplicar en este instante (escalar)
% =========================================================================

    % ===================== ÚNICO PARÁMETRO A CAMBIAR =====================
    N = 5;                       %  <-- CAMBIA SOLO ESTE NÚMERO
    % =====================================================================

    %% ---------------- MODELO DE LA PLANTA (Espacio de Estado) ----------
    Ad = [ 1.0000  0.1903;
           0       0.9048];
    Bd = [ 0.0097;
           0.0952];

    %% ---------------- PESOS DEL COSTO Y RESTRICCIONES ------------------
    Q    = diag([10, 1]);        % castigo al error de cada estado
    R    = 0.1;                  % castigo al esfuerzo de control
    umax = 100;                  % |u|   <= umax
    xmax = [100; 100];           % |x_i| <= xmax(i)   (un límite por estado)

    %% ================= A PARTIR DE AQUÍ: TODO AUTOMÁTICO ===============
    nx = size(Ad, 1);            % nº de estados   (se deduce solo)
    nu = size(Bd, 2);            % nº de controles (se deduce solo)

    % --- Tamaños del vector de decisión z = [u(0..N-1) ; x(0..N-1)] ----
    %     dim(z) = N*nu (controles) + N*nx (estados)
    Iu = eye(N*nu);
    Ix = eye(N*nx);

    % --- Matriz Hessiana:  H = 2*blkdiag( I_N⊗R , I_N⊗Q ) --------------
    H = 2 * blkdiag( kron(eye(N), R), kron(eye(N), Q) );

    % --- Vector gradiente: controles sin término lineal; estados -2Qr --
    f = [ zeros(N*nu, 1);
          repmat(-2 * Q * r, N, 1) ];

    % --- Restricciones de IGUALDAD (dinámica)  Aeq*z = beq -------------
    %     Bloque de controles:  -B en la diagonal por bloques
    Aeq_u = kron(eye(N), -Bd);                 % (N*nx) x (N*nu)
    %     Bloque de estados:  I en la diagonal, -Ad en la subdiagonal
    sub   = diag(ones(N-1, 1), -1);            % subdiagonal de unos
    Aeq_x = kron(eye(N), eye(nx)) - kron(sub, Ad);   % (N*nx) x (N*nx)
    Aeq   = [Aeq_u, Aeq_x];
    %     Lado derecho: solo el primer bloque conoce la x actual
    beq   = [ Ad * x;
              zeros((N-1)*nx, 1) ];

    % --- Restricciones de DESIGUALDAD (no se usan aquí) ----------------
    Ain = [];
    bin = [];

    % --- Límites de caja  lb <= z <= ub --------------------------------
    lb = [ repmat(-umax, N*nu, 1);
           repmat(-xmax, N, 1) ];
    ub = [ repmat( umax, N*nu, 1);
           repmat( xmax, N, 1) ];

    % --- Punto inicial -------------------------------------------------
    z0 = zeros(N*nu + N*nx, 1);

    %% ---------------- RESOLVER EL PROBLEMA CUADRÁTICO ------------------
    options = optimoptions('quadprog', ...
                           'Algorithm', 'active-set', ...
                           'Display',   'off');

    z = quadprog(H, f, Ain, bin, Aeq, beq, lb, ub, z0, options);

    %% ---------------- LEY DE CONTROL (horizonte recesivo) -------------
    %  Aplica solo el primer control u(0) = z(1:nu)
    u = z(1:nu);
    u = u(1);     % escalar (caso de una sola entrada)
end
