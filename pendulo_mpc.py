"""
CONTROL MPC DE PÉNDULO CON CARRO
Problema: Diseñar un controlador MPC para seguir una referencia escalón unitario
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import block_diag
from scipy.optimize import minimize
import warnings
warnings.filterwarnings('ignore')

# ============================================================================
# PARÁMETROS DEL SISTEMA
# ============================================================================
M = 0.77      # masa del carro [kg]
m = 0.089     # masa del péndulo [kg]
g = 9.81      # gravedad [m/s²]
L = 0.32      # longitud del péndulo [m]
T = 0.1       # período de muestreo [s]

print("="*80)
print("PARÁMETROS DEL SISTEMA")
print("="*80)
print(f"M (masa carro)     = {M} kg")
print(f"m (masa péndulo)   = {m} kg")
print(f"g (gravedad)       = {g} m/s²")
print(f"L (longitud)       = {L} m")
print(f"T (período muestreo) = {T} s")
print()

# ============================================================================
# PARTE A: DISCRETIZACIÓN DEL SISTEMA
# ============================================================================
print("="*80)
print("PARTE A: DISCRETIZACIÓN DEL SISTEMA")
print("="*80)

# Ecuaciones continuas del péndulo:
# ẋ = -(mg/M)θ + (1/M)u
# θ̇ = ((M+m)g/(ML))θ - (1/(ML))u

# Seleccionamos estados:
# x₁ = x (posición del carro)
# x₂ = ẋ (velocidad del carro)
# x₃ = θ (ángulo del péndulo)
# x₄ = θ̇ (velocidad angular del péndulo)

# Matriz de estado continua A_c: ẋ = A_c*x + B_c*u
A_c = np.array([
    [0,           1,                    0,          0],
    [0,           0,            -(m*g)/M,          0],
    [0,           0,                    0,          1],
    [0,           0,  ((M+m)*g)/(M*L),          0]
])

# Matriz de entrada continua B_c
B_c = np.array([
    [0],
    [1/M],
    [0],
    [-1/(M*L)]
])

# Matriz de salida (solo nos interesa la posición x₁)
C_c = np.array([[1, 0, 0, 0]])
D_c = np.array([[0]])

print("\nMatrices continuas del sistema:")
print("\nA_c (matriz de estado continua):")
print(A_c)
print("\nB_c (matriz de entrada continua):")
print(B_c)
print("\nC_c (matriz de salida):")
print(C_c)

# DISCRETIZACIÓN: Método de Euler hacia adelante
# x[k+1] = (I + T*A_c)*x[k] + T*B_c*u[k]
# y[k] = C_c*x[k] + D_c*u[k]

A_d = np.eye(4) + T * A_c
B_d = T * B_c
C_d = C_c
D_d = D_c

print("\n" + "="*80)
print("MATRICES DISCRETIZADAS (con T = {:.3f}s)".format(T))
print("="*80)
print("\nA_d (matriz de estado discreta):")
print(A_d)
print("\nB_d (matriz de entrada discreta):")
print(B_d)
print("\nC_d (matriz de salida discreta):")
print(C_d)

# Verificar estabilidad
eigs_A = np.linalg.eigvals(A_d)
print(f"\nValores propios de A_d: {eigs_A}")
print(f"Sistema discreto es estable: {np.all(np.abs(eigs_A) < 1)}")
print()

# ============================================================================
# PARTE B: FORMULACIÓN DEL PROBLEMA MPC
# ============================================================================
print("="*80)
print("PARTE B: FORMULACIÓN DEL PROBLEMA MPC")
print("="*80)

N = 4  # Horizonte de predicción
n = 4  # número de estados
m_ctrl = 1  # número de entradas de control

print(f"\nHorizonte de predicción N = {N}")
print(f"Estados n = {n}")
print(f"Entradas m = {m_ctrl}")

# Especificaciones de desempeño:
# - Seguir referencia escalón r = 1
# - Sobrepico os% ≤ 1%
# - Tiempo estabilización ts₂% ≤ 4s

r = 1  # referencia
os_max = 0.01  # 1% de sobrepico
ts_max = 4     # 4 segundos para estabilizar

print(f"\nEspecificaciones:")
print(f"  Referencia r = {r}")
print(f"  Sobrepico máximo os% = {os_max*100}%")
print(f"  Tiempo estabilización ts₂% = {ts_max}s")

"""
================================================================================
EXPLICACIÓN A PROFUNDIDAD DEL PROBLEMA MPC
================================================================================

1. OBJETIVO DEL MPC:
   Minimizar: J = Σ(k=0 a N-1) ||y[k] - r||²_Q + ||u[k]||²_R

   Donde:
   - y[k] = salida predicha en paso k
   - r = referencia (escalón unitario = 1)
   - Q = peso en error de seguimiento (penalización por desviación)
   - R = peso en control (penalización por energía de control)

2. FORMULACIÓN SPARSE (basada en cambios de control):
   En lugar de optimizar u[k], optimizamos Δu[k] = u[k] - u[k-1]

   Variables de optimización: z = [Δu[0], Δu[1], Δu[2], Δu[3]]ᵀ

   Ventajas:
   - Menos variables (solo 4 en lugar de pasos futuros)
   - Control más suave (cambios graduales)
   - Mejor condicionamiento numérico

3. PREDICCIÓN DEL SISTEMA (Formulación matricial):

   Para obtener la salida predicha en N pasos:

   Y = Φ*x(k) + Ψ*U + Λ*Δu

   Donde:
   - Y = [y[k+1], y[k+2], ..., y[k+N]]ᵀ (salidas predichas)
   - x(k) = estado actual
   - U = [u[k-1], ..., u[k-1]]ᵀ (control anterior repetido)
   - Δu = [Δu[0], Δu[1], ..., Δu[N-1]]ᵀ (cambios de control)

   Matrices de predicción:
   - Φ: matriz que propaga estado actual a salidas
   - Ψ: matriz que propaga control constante anterior a salidas
   - Λ: matriz triangular que propaga cambios incrementales

4. CONSTRUCCIÓN DE LA FUNCIÓN CUADRÁTICA:

   J = ||Y - r*1||²_Q + ||Δu||²_R

   Expandiendo en forma matricial:
   J = (Φ*x + Ψ*u_prev + Λ*Δu - r*1)ᵀ*Q*(Φ*x + Ψ*u_prev + Λ*Δu - r*1) + Δuᵀ*R*Δu

   Reordenando en forma: J = ½*Δuᵀ*H*Δu + fᵀ*Δu + cte

   Donde:
   - H = 2*(ΛᵀQΛ + R)           [matriz Hessiana, definida positiva]
   - f = 2*ΛᵀQ*(Φ*x + Ψ*u_prev - r*1)  [vector gradiente]

5. RESTRICCIONES:

   a) Restricciones de desigualdad (límites de control):
      -u_max ≤ u[k] ≤ u_max

      Se transforman en: A_ineq*Δu ≤ b_ineq

   b) Restricciones de igualdad (dinámica del sistema):
      Las ecuaciones: x[k+1] = A*x[k] + B*u[k]

      Se transforman en: A_eq*Δu = b_eq
      (Las ecuaciones dinámicas están incorporadas en Φ, Ψ, Λ)

6. MATRIZ PREDICCIÓN Λ (Triangular inferior):

   Propaga cambios incrementales de control:

        [I    0    0    0  ]
   Λ =  [I    I    0    0  ]  ⊗ C_d*B_d  (producto de Kronecker)
        [I    I    I    0  ]
        [I    I    I    I  ]

   Esto asegura que el cambio en el paso i afecta los pasos i, i+1, ..., N

================================================================================
"""

# CONSTRUCCIÓN DE MATRICES DE PREDICCIÓN

# Matriz Φ: propaga estado actual
Phi = np.zeros((N, n))
A_pow = A_d.copy()
for i in range(N):
    Phi[i] = (C_d @ A_pow).flatten()
    A_pow = A_d @ A_pow

# Matriz Ψ: propaga control anterior (constante)
Psi = np.zeros((N, 1))
A_pow = A_d.copy()
for i in range(N):
    Psi[i, 0] = (C_d @ A_pow @ B_d).item()
    A_pow = A_d @ A_pow

# Matriz Λ: propaga cambios incrementales (triangular)
Lambda = np.zeros((N, N))
for i in range(N):
    for j in range(i+1):
        A_pow_j = np.linalg.matrix_power(A_d, i - j)
        Lambda[i, j] = (C_d @ A_pow_j @ B_d).item()

print("\n" + "-"*80)
print("MATRICES DE PREDICCIÓN")
print("-"*80)
print("\nΦ (propaga estado actual a salidas):")
print(Phi)
print("\nΨ (propaga control anterior a salidas):")
print(Psi)
print("\nΛ (propaga cambios de control a salidas):")
print(Lambda)

# PESOS DE LA FUNCIÓN DE COSTO
Q = 100 * np.eye(N)  # Penalización en error de seguimiento
R = 0.1 * np.eye(N)   # Penalización en cambio de control

print("\n" + "-"*80)
print("PESOS DE LA FUNCIÓN DE COSTO")
print("-"*80)
print(f"\nQ (peso error de seguimiento):\n{np.diag(Q)}")
print(f"\nR (peso cambio de control):\n{np.diag(R)}")

# CONSTRUCCIÓN DE PROBLEMA DE OPTIMIZACIÓN CUADRÁTICA

# Matriz Hessiana: H = 2*(ΛᵀQΛ + R)
H = 2 * (Lambda.T @ Q @ Lambda + R)

# Vector gradiente: f = 2*ΛᵀQ*(Φ*x + Ψ*u_prev - r*1)
# Esto se calcula en cada iteración con x(k) actual

# RESTRICCIONES DE CONTROL
u_max = 10  # límite máximo de fuerza [N]
u_min = -10

# Para restricción: u[k] = u[k-1] + Δu[k]
# -u_max ≤ u[k-1] + Σ(j=0 a k) Δu[j] ≤ u_max

# Matriz A_ineq para desigualdades acumulativas
A_ineq = np.zeros((2*N, N))
for i in range(N):
    # Límite superior: Σ Δu ≤ u_max - u[k-1]
    A_ineq[i, :i+1] = 1
    # Límite inferior: -Σ Δu ≤ u_max + u[k-1]
    A_ineq[N+i, :i+1] = -1

print("\n" + "-"*80)
print("MATRIZ HESSIANA H y RESTRICCIONES")
print("-"*80)
print(f"\nH (matriz Hessiana):\n{H}")
print(f"\nA_ineq (matriz restricciones desigualdad):\n{A_ineq}")
print(f"Restricción: A_ineq * Δu ≤ b_ineq")

# ============================================================================
# PARTE C: SIMULACIÓN
# ============================================================================
print("\n" + "="*80)
print("PARTE C: SIMULACIÓN CON CONTROLADOR MPC")
print("="*80)

# Parámetros de simulación
t_sim = 10  # 10 segundos
n_steps = int(t_sim / T)

# Condiciones iniciales
x = np.array([0, 0, 0, 0])  # Estado inicial [x, ẋ, θ, θ̇]
u_prev = 0  # Control anterior
y_history = np.zeros(n_steps)
u_history = np.zeros(n_steps)
t_history = np.zeros(n_steps)

print(f"\nSimulación: {t_sim}s con período T={T}s ({n_steps} pasos)")
print(f"Estado inicial: x = {x}")

# FUNCIÓN DE COSTO MPC
def mpc_cost_and_constraints(delta_u, x_current, u_prev_val, H_mat, Lambda_mat, Phi_mat,
                             Psi_mat, A_ineq_mat, r_ref, u_max_val, u_min_val):
    """
    Calcula costo y construye restricciones para el problema MPC
    """
    # Salidas predichas: Y = Φ*x + Ψ*u_prev + Λ*Δu
    Y = Phi_mat @ x_current + Psi_mat @ np.array([u_prev_val]) + Lambda_mat @ delta_u

    # Error de seguimiento
    e = Y - r_ref * np.ones((N, 1))

    # Costo cuadrático: J = eᵀ*Q*e + ΔuᵀR*Δu
    J = e.T @ Q @ e + delta_u.T @ R @ delta_u

    return float(J.flatten()[0]), Y, e

# SIMULACIÓN ITERATIVA
print("\nResolviendo MPC en cada paso...\n")

from scipy.optimize import minimize

for k in range(n_steps):
    t_history[k] = k * T

    # Construir vector gradiente con estado actual
    pred_term = Phi @ x + Psi.flatten() * u_prev - r * np.ones(N)  # vector (4,)
    f_vec = 2.0 * Lambda.T @ (Q @ pred_term)  # (4,4) @ (4,) = (4,)
    f_vec = np.asarray(f_vec).flatten()

    # Restricciones de desigualdad acumulativas
    b_ineq = np.hstack([u_max - u_prev * np.ones(N),
                        u_max + u_prev * np.ones(N)])

    # Función objetivo para scipy.optimize
    def make_objective(H_mat, f_v):
        def objective(delta_u_vec):
            return 0.5 * float(delta_u_vec @ H_mat @ delta_u_vec) + float(f_v @ delta_u_vec)
        return objective

    def make_constraint(A_m, b_v):
        def constraint_ineq(delta_u_vec):
            return b_v - A_m @ delta_u_vec
        return constraint_ineq

    # Resolver problema de optimización
    constraints = {'type': 'ineq', 'fun': make_constraint(A_ineq, b_ineq)}
    delta_u_init = np.zeros(N)

    result = minimize(make_objective(H, f_vec), delta_u_init, method='SLSQP',
                     constraints=constraints, options={'ftol': 1e-9})

    if result.success:
        delta_u_opt = result.x
    else:
        delta_u_opt = np.zeros(N)

    # Aplicar primer elemento del control óptimo
    u_optimal = u_prev + delta_u_opt[0]
    u_optimal = np.clip(u_optimal, u_min, u_max)

    # Registrar
    u_history[k] = u_optimal
    y_history[k] = (C_d @ x).item() if hasattr(C_d @ x, 'item') else (C_d @ x)[0, 0]

    # Evolucionar sistema: x[k+1] = A*x[k] + B*u[k]
    x = (A_d @ x + B_d.flatten() * u_optimal).flatten()
    u_prev = u_optimal

    if k % 20 == 0:
        print(f"Paso {k:3d}: t={t_history[k]:.2f}s, x={x[0]:.3f}, y={y_history[k]:.3f}, u={u_optimal:.3f}")

print("\n" + "="*80)
print("RESULTADOS DE LA SIMULACIÓN")
print("="*80)

# Calcular parámetros de desempeño
y_final = y_history[-1]
y_max = np.max(y_history)
y_min = np.min(y_history)

# Sobrepico
if y_final != 0:
    overshoot_percent = ((y_max - y_final) / abs(y_final)) * 100
else:
    overshoot_percent = 0

# Tiempo de estabilización (dentro del 2%)
ts_2percent = None
tolerance = 0.02 * abs(y_final) if y_final != 0 else 0.02
for i in range(len(y_history) - 1, -1, -1):
    if abs(y_history[i] - y_final) > tolerance:
        ts_2percent = t_history[i]
        break

print(f"\nValor final y = {y_final:.4f}")
print(f"Valor máximo y = {y_max:.4f}")
print(f"Sobrepico os% = {overshoot_percent:.2f}%")
print(f"Tiempo estabilización ts₂% = {ts_2percent:.2f}s" if ts_2percent else "Tiempo estabilización: No alcanzado")

print(f"\n✓ Especificación os% ≤ 1%: {overshoot_percent <= 1.0}")
if ts_2percent:
    print(f"✓ Especificación ts₂% ≤ 4s: {ts_2percent <= 4.0}")

# ============================================================================
# GRÁFICOS
# ============================================================================

fig, axes = plt.subplots(3, 1, figsize=(12, 10))

# Gráfico 1: Seguimiento de referencia
axes[0].plot(t_history, y_history, 'b-', linewidth=2, label='Posición y(t)')
axes[0].axhline(y=r, color='r', linestyle='--', linewidth=2, label=f'Referencia r={r}')
axes[0].fill_between(t_history, r*0.98, r*1.02, alpha=0.2, color='green',
                      label='Banda ±2%')
axes[0].set_ylabel('Posición y [m]', fontsize=11)
axes[0].set_title('PARTE C: Respuesta del Sistema con Controlador MPC (N=4)', fontsize=12, fontweight='bold')
axes[0].grid(True, alpha=0.3)
axes[0].legend(fontsize=10)
axes[0].set_xlim([0, t_sim])

# Gráfico 2: Acción de control
axes[1].plot(t_history, u_history, 'g-', linewidth=2)
axes[1].axhline(y=u_max, color='r', linestyle='--', alpha=0.5, label=f'±{u_max}N')
axes[1].axhline(y=-u_max, color='r', linestyle='--', alpha=0.5)
axes[1].set_ylabel('Control u [N]', fontsize=11)
axes[1].set_title('Acción de Control MPC', fontsize=12, fontweight='bold')
axes[1].grid(True, alpha=0.3)
axes[1].legend(fontsize=10)
axes[1].set_xlim([0, t_sim])

# Gráfico 3: Error de seguimiento
error = y_history - r
axes[2].plot(t_history, error, 'r-', linewidth=2, label='Error e(t) = y(t) - r')
axes[2].axhline(y=0, color='k', linestyle='-', alpha=0.3)
axes[2].fill_between(t_history, -0.02, 0.02, alpha=0.2, color='green',
                      label='Banda error ±2%')
axes[2].set_xlabel('Tiempo [s]', fontsize=11)
axes[2].set_ylabel('Error e(t) [m]', fontsize=11)
axes[2].set_title('Error de Seguimiento', fontsize=12, fontweight='bold')
axes[2].grid(True, alpha=0.3)
axes[2].legend(fontsize=10)
axes[2].set_xlim([0, t_sim])

plt.tight_layout()
plt.savefig('/home/user/control_procesos/resultados_mpc.png', dpi=150, bbox_inches='tight')
print("\n✓ Gráfico guardado en: resultados_mpc.png")
plt.show()

print("\n" + "="*80)
print("PARÁMETROS DE DISEÑO DEL CONTROLADOR MPC")
print("="*80)
print(f"\nz (variable de optimización): Δu = [Δu₀, Δu₁, Δu₂, Δu₃]ᵀ")
print(f"\nH (Hessiana, {H.shape}):\n{H}")
print(f"\nf (Gradiente, depende de estado actual x(k))")
print(f"Ejemplo con x₀ = [0,0,0,0] y u_prev = 0:")
f_init = (2 * Lambda.T @ Q @ (Phi @ np.array([0,0,0,0]) + Psi @ np.array([0]) - r * np.ones((N, 1)))).flatten()
print(f"{f_init}")
print(f"\nA_ineq (restricciones desigualdad, {A_ineq.shape}):\n{A_ineq}")
print(f"\nb_ineq = [{u_max}, {u_max}, {u_max}, {u_max}, {u_max}, {u_max}, {u_max}, {u_max}]ᵀ")
print(f"\nA_eq: No hay restricciones de igualdad adicionales")
print(f"b_eq: (dinámica incorporada en Φ, Ψ, Λ)")
print(f"\nlb (límite inferior): [-∞, -∞, -∞, -∞]ᵀ")
print(f"ub (límite superior): [+∞, +∞, +∞, +∞]ᵀ")
