#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Genera los 25 casos MPC-QP MANTENIENDO EL FORMATO ORIGINAL del usuario
(explicito: blkdiag, Aeq por bloques, u1max/x1max..., function u = fcn).

Solo varia lo necesario por cada caso:
  - nx (2,3,4,5,6): tamano de Ad/Bd/Q y numero de limites de estado.
  - N  (1..5): numero de bloques en H, f, Aeq, beq, lb, ub.

nx=4 usa las matrices REALES del pendulo del enunciado.
nx=2,3,5,6 usan una planta EJEMPLO (bidiagonal) escrita explicitamente.
"""
import os
OUT = os.path.dirname(os.path.abspath(__file__))


def fmt_num(v):
    if v == int(v):
        return "%d" % int(v)
    return "%.4f" % v


def ad_matrix(nx):
    """Devuelve las filas de Ad como lista de strings (formato literal)."""
    if nx == 4:
        return [
            "1.0000  0.5000  -0.2763  -0.0360",
            "0       1.0000  -1.7995  -0.2763",
            "0       0        9.3349   1.5871",
            "0       0       54.2766   9.3349",
        ]
    # Planta ejemplo: 1 en diagonal, 0.5 en superdiagonal
    rows = []
    for i in range(nx):
        vals = []
        for j in range(nx):
            if j == i:
                vals.append("1.0000")
            elif j == i + 1:
                vals.append("0.5000")
            else:
                vals.append("0")
        rows.append("  ".join("%-7s" % v for v in vals).rstrip())
    return rows


def bd_vector(nx):
    if nx == 4:
        return ["0.1783", "0.7956", "-0.9891", "-6.4410"]
    return ["0"] * (nx - 1) + ["1.0000"]


def q_diag(nx):
    return "[" + ", ".join(["75"] + ["1"] * (nx - 1)) + "]"


def limits_lines(nx):
    """Lineas '% Limites fisicos' con u1max y x1max..xNmax."""
    parts = ["u1max = 50;"]
    for i in range(1, nx + 1):
        if nx == 4 and i == 3:
            parts.append("x%dmax = pi;" % i)   # angulo del pendulo
        else:
            parts.append("x%dmax = 50;" % i)
    return " ".join(parts)


def aeq_block(nx, N):
    """Construye el texto de Aeq por bloques (formato original)."""
    rows = []
    for k in range(1, N + 1):
        u_tok = ["-Bd" if j == k else "Zm" for j in range(1, N + 1)]
        x_tok = []
        for j in range(1, N + 1):
            if j == k:
                x_tok.append("In")
            elif j == k - 1:
                x_tok.append("-Ad")
            else:
                x_tok.append("Zn")
        rows.append(", ".join(u_tok + x_tok))
    body = ";\n".join(rows)
    return "Aeq = [" + body + ";];"


def build(nx, N):
    ad = ad_matrix(nx)
    bd = bd_vector(nx)

    # --- comentario del modelo ---
    if nx == 4:
        model_cmt = "% Modelo discreto del pendulo"
    else:
        model_cmt = "% Modelo discreto de la planta (EJEMPLO {0}x{0} - reemplazar por el real)".format(nx)

    # --- Ad / Bd literales ---
    ad_str = "Ad = [ " + ("\n       ".join(ad)) + "];"
    bd_str = "Bd = [ " + ("\n       ".join(bd)) + "];"

    # --- H ---
    H = "H = 2*blkdiag(" + ", ".join(["R"] * N + ["Q"] * N) + "); % Hessiana"

    # --- f ---
    f = "f = -2*[" + "; ".join(["0"] * N + ["Q*r"] * N) + "]; % Vector lineal"

    # --- Aeq / beq ---
    aeq = aeq_block(nx, N)
    beq = "beq = [Ad*x; " + "; ".join(["zeros(nx,1)"] * (N - 1)) + "]; % Lado derecho" \
        if N > 1 else "beq = [Ad*x]; % Lado derecho"

    # --- cotas de caja lb_x / ub_x ---
    lbx = "lb_x = [" + "; ".join(["-x%dmax" % i for i in range(1, nx + 1)]) + "];"
    ubx = "ub_x = [" + "; ".join([" x%dmax" % i for i in range(1, nx + 1)]) + "];"
    lb = "lb = [" + "; ".join(["-u1max"] * N + ["lb_x"] * N) + "]; % Limite inferior"
    ub = "ub = [" + "; ".join(["u1max"] * N + ["ub_x"] * N) + "]; % Limite superior"

    txt = """function u = fcn(r, x)
{model_cmt}
{ad_str}
{bd_str}
% Parametros MPC
N = {N}; nx = {nx}; nu = 1;
Q = diag({qd}); R = 5;
% Limites fisicos
{lims}
% Matrices de ceros e identidad
In = eye(nx); Zn = zeros(nx); Zm = zeros(nx, nu);
% Matrices del optimizador QP
{H}
{f}
Ain = []; bin = []; % Desigualdad vacia
% Restricciones de igualdad dinamica
{aeq}
{beq}
% Cotas de caja
{lbx}
{ubx}
{lb}
{ub}
% Ejecucion de quadprog
z0 = zeros(N*(nx+nu),1);
options = optimoptions('quadprog', 'Algorithm','active-set', 'Display', 'off');
z = quadprog(H,f,Ain,bin,Aeq,beq,lb,ub,z0,options);
u = z(1); % Primera accion de control
""".format(model_cmt=model_cmt, ad_str=ad_str, bd_str=bd_str, N=N, nx=nx,
           qd=q_diag(nx), lims=limits_lines(nx), H=H, f=f, aeq=aeq, beq=beq,
           lbx=lbx, ubx=ubx, lb=lb, ub=ub)
    return txt


def main():
    n = 0
    for nx in (2, 3, 4, 5, 6):
        for N in (1, 2, 3, 4, 5):
            fname = "fcn_nx%d_N%d.m" % (nx, N)
            with open(os.path.join(OUT, fname), "w", encoding="utf-8") as fh:
                fh.write(build(nx, N))
            n += 1
            print("generado:", fname)
    print("\nTOTAL:", n)


if __name__ == "__main__":
    main()
