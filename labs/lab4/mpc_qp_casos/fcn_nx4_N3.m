function u = fcn_nx4_N3(r, x)
% ============================================================
%  MPC por QP (quadprog)  -  Planta 4x4, horizonte N = 3
%  Caso: nx = 4, N = 3   (Control de Procesos - Lab 4)
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
N  = 3;      % horizonte de prediccion/control
nx = 4;      % numero de estados
nu = 1;       % numero de entradas

% ----------------- Modelo discreto de la planta -------------
% --- Modelo discreto del pendulo (DATOS REALES, nx = 4) ---
Ad = [ 1.0000   0.5000  -0.2763  -0.0360;
       0        1.0000  -1.7995  -0.2763;
       0        0        9.3349   1.5871;
       0        0       54.2766   9.3349];
Bd = [ 0.1783;
       0.7956;
      -0.9891;
      -6.4410];

% ----------------- Pesos del costo --------------------------
% --- Pesos del costo (DATOS REALES, nx = 4) ---
Q = diag([75, 1, 1, 1]);
R = 5;

% ----------------- Limites ----------------------------------
% --- Limites fisicos (DATOS REALES, nx = 4) ---
umax = 50;                          % |u| <= 50
lb_x = [-50; -50; -pi; -50];        % cotas inferiores de estado
ub_x = [ 50;  50;  pi;  50];        % cotas superiores de estado

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
