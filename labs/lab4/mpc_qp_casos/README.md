# MPC por QP (quadprog) — 25 casos

Generalización del controlador MPC del péndulo (Lab 4) a distintos tamaños de
planta y horizontes de predicción.

## Casos generados (5 plantas × 5 horizontes = 25)

| Planta      | nx | N = 1 | N = 2 | N = 3 | N = 4 | N = 5 |
|-------------|----|-------|-------|-------|-------|-------|
| Planta 2×2  | 2  | `fcn_nx2_N1.m` | `fcn_nx2_N2.m` | `fcn_nx2_N3.m` | `fcn_nx2_N4.m` | `fcn_nx2_N5.m` |
| Planta 3×3  | 3  | `fcn_nx3_N1.m` | `fcn_nx3_N2.m` | `fcn_nx3_N3.m` | `fcn_nx3_N4.m` | `fcn_nx3_N5.m` |
| Planta 4×4  | 4  | `fcn_nx4_N1.m` | `fcn_nx4_N2.m` | `fcn_nx4_N3.m` | `fcn_nx4_N4.m` | `fcn_nx4_N5.m` |
| Planta 5×5  | 5  | `fcn_nx5_N1.m` | `fcn_nx5_N2.m` | `fcn_nx5_N3.m` | `fcn_nx5_N4.m` | `fcn_nx5_N5.m` |
| Planta 6×6  | 6  | `fcn_nx6_N1.m` | `fcn_nx6_N2.m` | `fcn_nx6_N3.m` | `fcn_nx6_N4.m` | `fcn_nx6_N5.m` |

## Formulación

Vector de decisión:

```
z = [ u_0; u_1; ...; u_{N-1}; x_1; x_2; ...; x_N ]      (longitud N*(nu+nx))
```

Problema QP resuelto con `quadprog` (algoritmo `active-set`):

```
min_z  1/2 z' H z + f' z
s.a.   Aeq z = beq      (dinámica  x_{k+1} = Ad x_k + Bd u_k)
       lb <= z <= ub    (límites de entrada y estado)
```

- `H  = 2*blkdiag(R,...,R, Q,...,Q)`  (N bloques de R, N bloques de Q)
- `f  = -2*[0; ...; 0; Q r; ...; Q r]`  (referencia en los bloques de estado)
- `Aeq`/`beq` codifican la dinámica con la condición inicial `x_1 = Ad x + Bd u_0`.

La construcción se hace con **bucles**, de modo que es válida para cualquier
`nx` y cualquier `N`. Se verificó numéricamente que reproduce exactamente las
matrices escritas a mano del caso original (`nx = 4`, `N = 4`).

## ⚠️ Sobre las matrices de la planta

- **`nx = 4`**: usa las matrices **reales** del péndulo dadas en el enunciado
  (`Ad`, `Bd`, `Q = diag([75 1 1 1])`, `R = 5`, límites `[50 50 pi 50]`).
- **`nx = 2, 3, 5, 6`**: como no se proporcionó el modelo para esos tamaños,
  cada archivo trae una **PLANTA EJEMPLO** (cadena de integradores discreta,
  controlable) claramente marcada. **Reemplaza `Ad`, `Bd`, `Q`, `R` y los
  límites por los de tu planta real** en la sección indicada.

## Uso

- **MATLAB script**: `u = fcn_nx4_N4(r, x);` con `r`, `x` vectores columna `nx×1`.
- **Simulink (bloque MATLAB Function)**: copia el cuerpo y renómbralo a `fcn(r, x)`.

Requiere *Optimization Toolbox* (`quadprog`).

## Regenerar

```bash
python3 _generador_casos.py
```
