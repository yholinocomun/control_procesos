#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generador de los 25 casos de MPC-QP (quadprog).

Crea un archivo .m por cada combinacion:
    Planta nx x nx, con nx = 2,3,4,5,6
    Horizonte N = 1,2,3,4,5

=> 5 plantas x 5 horizontes = 25 archivos fcn_nx<NX>_N<N>.m

El nucleo del QP es identico en todos los casos (se construye con bucles
para cualquier nx y N). Lo unico que cambia entre archivos es:
    - nx (tamano de la planta)
    - N  (horizonte)
    - el modelo de la planta (Ad, Bd), los pesos (Q, R) y los limites.

Para nx = 4 se usan EXACTAMENTE las matrices del pendulo dadas por el usuario.
Para los demas tamanos se usa una PLANTA EJEMPLO (cadena de integradores
discreta, controlable) que debe reemplazarse por el modelo real.
"""

import os

OUT = os.path.dirname(os.path.abspath(__file__))

# ----------------------------------------------------------------------
# Bloque "modelo de la planta" por cada nx
# ----------------------------------------------------------------------
def plant_block(nx):
    if nx == 4:
        # Matrices REALES del pendulo dadas en el enunciado (nx = 4)
        return (
            "% --- Modelo discreto del pendulo (DATOS REALES, nx = 4) ---\n"
            "Ad = [ 1.0000   0.5000  -0.2763  -0.0360;\n"
            "       0        1.0000  -1.7995  -0.2763;\n"
            "       0        0        9.3349   1.5871;\n"
            "       0        0       54.2766   9.3349];\n"
            "Bd = [ 0.1783;\n"
            "       0.7956;\n"
            "      -0.9891;\n"
            "      -6.4410];\n"
        )
    else:
        # Planta EJEMPLO de tamano nx (cadena de integradores discreta).
        # >>> REEMPLAZA Ad y Bd por tu modelo real de tamano nx x nx <<<
        return (
            "% --- PLANTA EJEMPLO {nx}x{nx} (cadena de integradores discreta) ---\n"
            "%     >>> REEMPLAZA Ad y Bd por tu modelo real {nx}x{nx} <<<\n"
            "Ts = 0.5;                                  % paso de muestreo (ejemplo)\n"
            "Ad = eye(nx) + diag(Ts*ones(nx-1,1), 1);   % triangular superior bidiagonal\n"
            "Bd = [zeros(nx-1,1); 1];                   % entrada en el ultimo estado\n"
        ).format(nx=nx)


def weights_block(nx):
    if nx == 4:
        return (
            "% --- Pesos del costo (DATOS REALES, nx = 4) ---\n"
            "Q = diag([75, 1, 1, 1]);\n"
            "R = 5;\n"
        )
    else:
        return (
            "% --- Pesos del costo (ajustables) ---\n"
            "Q = diag([75, ones(1, nx-1)]);   % se penaliza fuerte el primer estado\n"
            "R = 5;                           % peso del esfuerzo de control\n"
        )


def limits_block(nx):
    if nx == 4:
        return (
            "% --- Limites fisicos (DATOS REALES, nx = 4) ---\n"
            "umax = 50;                          % |u| <= 50\n"
            "lb_x = [-50; -50; -pi; -50];        % cotas inferiores de estado\n"
            "ub_x = [ 50;  50;  pi;  50];        % cotas superiores de estado\n"
        )
    else:
        return (
            "% --- Limites (cotas de caja, ajustables) ---\n"
            "umax = 50;                  % |u| <= 50\n"
            "xmax = 50;                  % |x_i| <= xmax (ejemplo)\n"
            "lb_x = -xmax*ones(nx,1);    % cotas inferiores de estado\n"
            "ub_x =  xmax*ones(nx,1);    % cotas superiores de estado\n"
        )


# ----------------------------------------------------------------------
# Plantilla del archivo .m
# ----------------------------------------------------------------------
TEMPLATE = """function u = {fname}(r, x)
% ============================================================
%  MPC por QP (quadprog)  -  Planta {nx}x{nx}, horizonte N = {N}
%  Caso: nx = {nx}, N = {N}   (Control de Procesos - Lab 4)
%
%  Vector de decision:  z = [ u_0; ...; u_{{N-1}}; x_1; ...; x_N ]
%  Entradas:  r = referencia (vector columna nx x 1)
%             x = estado actual (vector columna nx x 1)
%  Salida:    u = primera accion de control (escalar)
%
%  NOTA Simulink: dentro de un bloque "MATLAB Function" renombra esta
%  funcion a  fcn(r, x)  (o pega solo el cuerpo).
% ============================================================

% ----------------- Parametros del horizonte -----------------
N  = {N};      % horizonte de prediccion/control
nx = {nx};      % numero de estados
nu = 1;       % numero de entradas

% ----------------- Modelo discreto de la planta -------------
{plant}
% ----------------- Pesos del costo --------------------------
{weights}
% ----------------- Limites ----------------------------------
{limits}
% ===================== Construccion del QP ==================
In = eye(nx);
Zm = zeros(nx, nu); %#ok<NASGU>  % (referencia de bloque cero, por claridad)

% --- Hessiana H y vector lineal f ---
Hcell = cell(1, N + N);
for k = 1:N,  Hcell{{k}}   = R;  end     % N bloques de R (entradas)
for k = 1:N,  Hcell{{N+k}} = Q;  end     % N bloques de Q (estados)
H = 2*blkdiag(Hcell{{:}});

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
    cu   = (k-1)*nu + (1:nu);                 % columna de u_{{k-1}}
    cx   = N*nu + (k-1)*nx + (1:nx);          % columna de x_k
    Aeq(rows, cu) = -Bd;
    Aeq(rows, cx) =  In;
    if k == 1
        beq(rows) = Ad*x;                     % condicion inicial
    else
        cxm = N*nu + (k-2)*nx + (1:nx);       % columna de x_{{k-1}}
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
"""


def main():
    count = 0
    for nx in (2, 3, 4, 5, 6):
        for N in (1, 2, 3, 4, 5):
            fname = "fcn_nx{nx}_N{N}".format(nx=nx, N=N)
            content = TEMPLATE.format(
                fname=fname, nx=nx, N=N,
                plant=plant_block(nx),
                weights=weights_block(nx),
                limits=limits_block(nx),
            )
            path = os.path.join(OUT, fname + ".m")
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(content)
            count += 1
            print("generado:", os.path.basename(path))
    print("\nTOTAL:", count, "archivos")


if __name__ == "__main__":
    main()
