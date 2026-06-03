# MPC genérico (QP con quadprog)

Implementación de un **MPC parametrizado** para Control de Procesos (UTEC).
La idea: **solo introduces las matrices de estado `Ad`, `Bd` y el horizonte `N`**,
y todo lo demás —número de estados `nx`, número de entradas `nu`, la matriz de
pesos `Q`, las restricciones y todas las matrices del QP— se calcula
**automáticamente**.

Sirve para cualquier número de estados (2, 3, 4, 5, …) y cualquier horizonte
sin reescribir el código, a diferencia de los bloques originales donde había que
escribir a mano `diag([10,1,1,...])` y los límites `[-x1max; -x2max; ...]`.

## Archivos

| Archivo | Para qué sirve |
|---|---|
| `fcn_mpc_generico.m` | **Bloque MATLAB Function de Simulink auto-contenido.** Pégalo en el bloque y edita solo `Ad`, `Bd`, `N`. Recomendado para uso directo en Simulink. |
| `mpc_qp_step.m` | Función reutilizable `u = mpc_qp_step(r, x, Ad, Bd, N, ...)`. Toda la lógica del QP. Se puede llamar desde scripts o desde Simulink. |
| `fcn_mpc_modular.m` | Bloque Simulink **corto** que llama a `mpc_qp_step.m` (versión modular). |
| `demo_mpc_generico.m` | Script de prueba: resuelve el MPC para 2, 3, 4 y 5 estados con el mismo código. |

## Uso en Simulink

1. Copia el contenido de `fcn_mpc_generico.m` dentro de tu bloque
   **MATLAB Function** (entradas `r`, `x`; salida `u`).
2. Edita **solo** estas líneas al inicio:
   ```matlab
   N  = 5;                 % horizonte de predicción
   Ad = [ ... ];           % matriz de estados discreta (nx x nx)
   Bd = [ ... ];           % matriz de entradas discreta (nx x nu)
   ```
3. (Opcional) ajusta pesos/restricciones: `q_primer_estado`, `q_resto`,
   `R_peso`, `umax`, `xmax`.

El número de estados se deduce de `size(Ad,1)`, así que **no tienes que tocar
nada más**.

## Uso desde un script (MATLAB)

```matlab
addpath('common/matlab/mpc');

Ad = [1 0.1; 0 1];
Bd = [0.005; 0.1];
N  = 5;

r = [1; 0];      % referencia
x = [0; 0];      % estado actual
u = mpc_qp_step(r, x, Ad, Bd, N);

% con opciones:
u = mpc_qp_step(r, x, Ad, Bd, N, 'q1', 10, 'qrest', 1, 'R', 0.1, ...
                'umax', 100, 'xmax', 100);
```

Para una prueba rápida ejecuta `demo_mpc_generico` (necesita Optimization
Toolbox / `quadprog`).

## Qué se automatizó respecto a los bloques originales

| Antes (manual, por cada nº de estados) | Ahora (automático) |
|---|---|
| `Q = diag([10, 1, 1, 1])` | `q_diag = qrest*ones(nx,1); q_diag(1)=q1;` |
| `lim_x_min = [-x1max; -x2max; -x3max; -x4max]` | `-xmax*ones(nx,1)` |
| `lim_x_max = [ x1max;  x2max;  x3max;  x4max]` | ` xmax*ones(nx,1)` |
| Reescribir todo el bloque por cada planta | Solo cambiar `Ad`, `Bd`, `N` |
