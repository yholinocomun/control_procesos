function u = fcn_nx5_N2(r, x)
% ============================================================
%  MPC por QP (quadprog)  -  Planta 5x5, horizonte N = 2
%  Caso: nx = 5, N = 2   (Control de Procesos - Lab 4)
%
%  Vector de decision:  z = [ u_0; ...; u_{N-1}; x_1; ...; x_N ]
%  Entradas:  r = referencia (vector columna nx x 1)
%             x = estado actual (vector columna nx x 1)
%  Salida:    u = primera accion de control (escalar)
%
%  NOTA Simulink: dentro de un bloque "MATLAB Function" renombra esta
%  funcion a  fcn(r, x)  (o pega solo el cuerpo).
% ============================================================

% ----------------- Parametros del horizonte -----------------
N  = 2;      % horizonte de prediccion/control
nx = 5;      % numero de estados
nu = 1;       % numero de entradas

% ----------------- Modelo discreto de la planta -------------
% --- PLANTA EJEMPLO 5x5 (cadena de integradores discreta) ---
%     >>> REEMPLAZA Ad y Bd por tu modelo real 5x5 <<<
Ts = 0.5;                                  % paso de muestreo (ejemplo)
Ad = eye(nx) + diag(Ts*ones(nx-1,1), 1);   % triangular superior bidiagonal
Bd = [zeros(nx-1,1); 1];                   % entrada en el ultimo estado

% ----------------- Pesos del costo --------------------------
% --- Pesos del costo (ajustables) ---
Q = diag([75, ones(1, nx-1)]);   % se penaliza fuerte el primer estado
R = 5;                           % peso del esfuerzo de control

% ----------------- Limites ----------------------------------
% --- Limites (cotas de caja, ajustables) ---
umax = 50;                  % |u| <= 50
xmax = 50;                  % |x_i| <= xmax (ejemplo)
lb_x = -xmax*ones(nx,1);    % cotas inferiores de estado
ub_x =  xmax*ones(nx,1);    % cotas superiores de estado

% ===================== Construccion del QP ==================
In = eye(nx);
Zm = zeros(nx, nu); %#ok<NASGU>  % (referencia de bloque cero, por claridad)

% --- Hessiana H y vector lineal f ---
Hcell = cell(1, N + N);
for k = 1:N,  Hcell{k}   = R;  end     % N bloques de R (entradas)
for k = 1:N,  Hcell{N+k} = Q;  end     % N bloques de Q (estados)
H = 2*blkdiag(Hcell{:});

f = zeros(N*(nu+nx), 1);
for k = 1:N
    idx = N*nu + (k-1)*nx + (1:nx);
    f(idx) = -2*(Q*r);
end

% --- Restricciones de igualdad (dinamica)  Aeq*z = beq ---
%     x_1   = Ad*x + Bd*u_0
%     x_k+1 = Ad*x_k + Bd*u_k   (k >= 1)
Aeq = zeros(N*nx, N*(nu+nx));
beq = zeros(N*nx, 1);
for k = 1:N
    rows = (k-1)*nx + (1:nx);
    cu   = (k-1)*nu + (1:nu);                 % columna de u_{k-1}
    cx   = N*nu + (k-1)*nx + (1:nx);          % columna de x_k
    Aeq(rows, cu) = -Bd;
    Aeq(rows, cx) =  In;
    if k == 1
        beq(rows) = Ad*x;                     % condicion inicial
    else
        cxm = N*nu + (k-2)*nx + (1:nx);       % columna de x_{k-1}
        Aeq(rows, cxm) = -Ad;
    end
end

% --- Desigualdades: ninguna (todo via cotas de caja) ---
Ain = [];
bin = [];

% --- Cotas de caja  lb <= z <= ub ---
lb = [repmat(-umax, N*nu, 1); repmat(lb_x, N, 1)];
ub = [repmat( umax, N*nu, 1); repmat(ub_x, N, 1)];

% ===================== Resolver el QP =======================
z0 = zeros(N*(nu+nx), 1);
options = optimoptions('quadprog', 'Algorithm', 'active-set', 'Display', 'off');
z = quadprog(H, f, Ain, bin, Aeq, beq, lb, ub, z0, options);

u = z(1);   % primera accion de control
end
