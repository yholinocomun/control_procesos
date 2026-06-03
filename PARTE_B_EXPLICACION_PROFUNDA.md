# PARTE B: EXPLICACIÓN A PROFUNDIDAD - FORMULACIÓN DEL MPC

## 1. ¿QUÉ ES MPC (Model Predictive Control)?

El MPC es un controlador avanzado que:

**Predice** el comportamiento futuro del sistema durante N pasos
**Optimiza** la acción de control minimizando una función de costo
**Respeta** restricciones de actución (límites de fuerza, energía, etc.)

En nuestro caso:
- Queremos que la **posición del carro (y)** siga una **referencia escalón r = 1m**
- Queremos **sobrepico máximo os% ≤ 1%** (subida suave, sin oscilaciones bruscas)
- Queremos **tiempo de estabilización ts₂% ≤ 4s** (llegar rápido pero suave)
- La fuerza está limitada: **-10N ≤ u ≤ +10N**

---

## 2. FUNCIÓN DE COSTO (OBJETIVO DEL MPC)

### Objetivo general:
```
Minimizar: J = Σ(k=0 a N-1) ||y[k] - r||²_Q + ||Δu[k]||²_R

Donde:
  y[k]     = salida predicha en el paso k (posición del carro)
  r        = referencia (1 metro)
  Δu[k]    = cambio de control (u[k] - u[k-1])
  Q        = matriz de pesos en el error (penaliza desviación)
  R        = matriz de pesos en el control (penaliza energía)
  N        = horizonte de predicción (4 pasos)
```

### Interpretación física:

| Término | Significado | Efecto |
|---------|------------|---------|
| **\|\|y - r\|\|²_Q** | Error de seguimiento | Fuerza al carro a ir hacia r = 1 |
| **\|\|Δu\|\|²_R** | Suavidad del control | Evita cambios bruscos de fuerza |

### En forma matricial (que es lo que el computador resuelve):

```
Minimizar: J = (Φx + ΨU + ΛΔu - r·1)ᵀ Q (Φx + ΨU + ΛΔu - r·1) + ΔuᵀRΔu
```

Donde:
- **x** = estado actual [x, ẋ, θ, θ̇]
- **Δu** = vector de cambios de control [Δu₀, Δu₁, Δu₂, Δu₃]ᵀ
- **Φ, Ψ, Λ** = matrices de predicción (ver sección 4)

---

## 3. CONVERSIÓN A FORMA CUADRÁTICA ESTÁNDAR

Para que scipy.optimize pueda resolver el problema, convertimos el costo a forma:

```
J = ½·ΔuᵀH·Δu + fᵀΔu + cte
```

### Expansión matemática:

1. Desarrollamos el costo:
```
J = eᵀQe + ΔuᵀRΔu,  donde e = Φx + ΨU + ΛΔu - r·1

J = (Φx + ΨU + ΛΔu - r·1)ᵀQ(Φx + ΨU + ΛΔu - r·1) + ΔuᵀRΔu
```

2. Expandemos el cuadrado:
```
J = eᵀQe = eᵀQe
  = (Φx + ΨU + ΛΔu - r·1)ᵀQ(Φx + ΨU + ΛΔu - r·1)
```

3. Agrupamos términos en función de Δu:
```
J = ΔuᵀΛᵀQΛΔu + 2ΔuᵀΛᵀQ(Φx + ΨU - r·1) + ΔuᵀRΔu + [términos que no dependen de Δu]
```

4. Identificamos los coeficientes:
```
H = 2(ΛᵀQΛ + R)                          [Matriz Hessiana, 4×4]
f = 2ΛᵀQ(Φx + ΨU - r·1)                [Vector gradiente, 4×1]
c = (Φx + ΨU - r·1)ᵀQ(Φx + ΨU - r·1)  [Constante, no afecta optimalidad]
```

### Propiedades importantes:

| Propiedad | Significado | Importancia |
|-----------|------------|-------------|
| **H definida positiva** | H tiene todos valores propios > 0 | Garantiza único mínimo |
| **H = 2(ΛᵀQΛ + R)** | H > 0 siempre (Q,R ≥ 0) | Problema siempre tiene solución |
| **f depende de x(k)** | f cambia cada paso de tiempo | Adaptativo al estado actual |

---

## 4. MATRICES DE PREDICCIÓN (Φ, Ψ, Λ)

Estas matrices propagan cómo el estado actual y el control afectan las salidas futuras.

### 4.1 MATRIZ Φ (Propagación del estado)

```
      ╔                         ╗
      ║ C·A              ║
      ║ C·A²             ║      [y₁]   ╔ C·A      ╗  [x₀]
  Φ = ║ C·A³             ║  →   [y₂] = ║ C·A²     ║  
      ║ C·A⁴             ║      [y₃]   ║ C·A³     ║
      ╚                         ║      ╚ C·A⁴     ╝
```

**Significado**: Predice qué pasará con la salida solo si dejamos que el estado actual evolucione sin control.

**En nuestro caso**: Con condición inicial x₀ = [0,0,0,0], Φ nos da cero (sin movimiento).

**Fórmula general**:
```
Φ[i] = C·Aⁱ    para i = 1, 2, ..., N
```

### 4.2 MATRIZ Ψ (Propagación del control anterior constante)

```
Si aplicamos u[k-1] = constante durante los próximos N pasos:

      ╔ C·A·B           ╗  [u[k-1]]
  Ψ = ║ C·A²·B          ║  
      ║ C·A³·B          ║
      ║ C·A⁴·B          ║
      ╚                 ╝
```

**Significado**: Cuánto contribuye el control anterior a las salidas futuras.

**En nuestro código**: Ψ = [0.013, 0.026, 0.039, 0.054]ᵀ

Esto significa que si mantenemos u = 1N durante 4 pasos, la salida será aproximadamente [0.013, 0.026, 0.039, 0.054] metros.

**Fórmula general**:
```
Ψ[i] = C·Aⁱ·B    para i = 1, 2, ..., N
```

### 4.3 MATRIZ Λ (Propagación de cambios incrementales)

**Concepto clave**: En lugar de optimizar los controles u[0], u[1], u[2], u[3], optimizamos los **cambios** Δu[0], Δu[1], Δu[2], Δu[3], donde:
```
Δu[k] = u[k] - u[k-1]
```

Esta es la **formulación Sparse** que reduce variables de optimización.

**Matriz Λ (triangular inferior)**:
```
      ╔ C·A·B      0        0        0     ╗
  Λ = ║ C·A²·B     C·A·B    0        0     ║
      ║ C·A³·B     C·A²·B   C·A·B    0     ║
      ║ C·A⁴·B     C·A³·B   C·A²·B   C·A·B ║
      ╚                                    ╝
```

**Significado**: El cambio Δu[j] en el paso j afecta a los pasos j, j+1, ..., N

**En nuestro código**:
```python
Λ = [[0.0,      0.0,      0.0,      0.0],
     [0.013,    0.0,      0.0,      0.0],
     [0.026,    0.013,    0.0,      0.0],
     [0.039,    0.026,    0.013,    0.0]]
```

**Ejemplo práctico**:
- Si Δu[0] = 5, suma 5 a TODOS los pasos (filas)
- Si Δu[1] = 3, suma 3 a los pasos 1, 2, 3 (filas 1, 2, 3)
- Si Δu[2] = 1, suma 1 a los pasos 2, 3 (filas 2, 3)
- Si Δu[3] = 0, suma 0 a solo el paso 3 (fila 3)

**Fórmula general**:
```
Λ[i,j] = C·Aⁱ⁻ʲ·B    para j ≤ i
Λ[i,j] = 0           para j > i
```

---

## 5. RELACIÓN COMPLETA: SALIDA PREDICHA

La **salida predicha en N pasos** es:

```
Y = Φ·x(k) + Ψ·u[k-1] + Λ·Δu
```

Donde:
- **Y** = [y₁, y₂, y₃, y₄]ᵀ = vector de 4 salidas predichas
- **x(k)** = estado actual (estado inicial para las predicciones)
- **u[k-1]** = control aplicado en el paso anterior
- **Δu** = [Δu₀, Δu₁, Δu₂, Δu₃]ᵀ = cambios de control que vamos a optimizar

**Ejemplo**:
Si x(k) = [1, 0, 0, 0], u[k-1] = 2, Δu = [1, 0, 0, 0]:

```
Y = Φ·[1,0,0,0] + Ψ·2 + Λ·[1,0,0,0]
  = primera fila de Φ + 2·Ψ + primera columna de Λ
  = [1 + 2·0.013 + 0, 1 + 2·0.026 + 0.013, 1 + 2·0.039 + 0.026, 1 + 2·0.054 + 0.039]ᵀ
  = [1.026, 1.077, 1.133, 1.147]ᵀ
```

---

## 6. RESTRICCIONES DEL PROBLEMA

### 6.1 Límites de control

```
-10 ≤ u[k] ≤ 10    para todo k

u[k] = u[k-1] + Δu[k]

Por lo tanto:
-10 ≤ u[k-1] + Δu[k] ≤ 10
-10 - u[k-1] ≤ Δu[k] ≤ 10 - u[k-1]
```

**En forma matricial**:
```
A_ineq · Δu ≤ b_ineq

A_ineq (8×4):  [Acumula cambios de forma triangular]
b_ineq (8×1):  [Límites superiores e inferiores]
```

### 6.2 Matriz A_ineq

```
      ╔  1   0   0   0 ╗    [Δu₀] ≤ 10 - u[k-1]
      ║  1   1   0   0 ║    [Δu₀ + Δu₁] ≤ 10 - u[k-1]
      ║  1   1   1   0 ║    [Δu₀ + Δu₁ + Δu₂] ≤ 10 - u[k-1]
      ║  1   1   1   1 ║    [Σ Δu] ≤ 10 - u[k-1]
A_ineq = ║ -1   0   0   0 ║    [-Δu₀] ≤ 10 + u[k-1]
      ║ -1  -1   0   0 ║    [-(Δu₀ + Δu₁)] ≤ 10 + u[k-1]
      ║ -1  -1  -1   0 ║    [-(Δu₀ + Δu₁ + Δu₂)] ≤ 10 + u[k-1]
      ║ -1  -1  -1  -1 ║    [-(Σ Δu)] ≤ 10 + u[k-1]
      ╚                ╝
```

**Interpretación**:
- Primeras 4 filas: aseguran que el control acumulativo no exceda +10N
- Últimas 4 filas: aseguran que el control acumulativo no caiga por debajo de -10N

### 6.3 Restricciones de desempeño

Las restricciones de **sobrepico (os% ≤ 1%)** y **tiempo de estabilización (ts₂% ≤ 4s)** se incorporan mediante:

1. **Selección de pesos Q y R**:
   - Q grande → penaliza mucho el error → controla suave (bajo sobrepico)
   - R grande → penaliza cambios de control → respuesta más lenta
   - Iteración: ajustar Q y R para cumplir especificaciones

2. **Aumento del horizonte N**:
   - N grande → más predicción hacia adelante → mejor desempeño
   - N pequeño → cálculo rápido pero desempeño limitado

En nuestro código:
- **Q = 100·I** (penalización media en el error)
- **R = 0.1·I** (bajo énfasis en suavidad)
- **N = 4** (4 pasos de predicción)

---

## 7. ALGORITMO DE SOLUCIÓN

En cada **instante de tiempo k**:

```
1. Medir estado actual x(k) = [x, ẋ, θ, θ̇]

2. Recordar control anterior u[k-1]

3. Calcular vector gradiente:
   f = 2·ΛᵀQ(Φx(k) + Ψu[k-1] - r·1)

4. Resolver problema cuadrático:
   min      ½·Δuᵀ·H·Δu + fᵀΔu
   Δu

   Sujeto a:
   A_ineq·Δu ≤ b_ineq        [límites de control]
   
5. Extraer primer elemento:
   u[k] = u[k-1] + Δu[0]

6. Aplicar al sistema:
   x(k+1) = A_d·x(k) + B_d·u[k]
   y[k] = C_d·x(k)

7. Repetir en k+1
```

---

## 8. DIMENSIONES Y VARIABLES

### Matrices del sistema discreto:
```
A_d: 4×4    [dinámica del sistema]
B_d: 4×1    [entrada de control]
C_d: 1×4    [salida (posición del carro)]
```

### Matrices de predicción:
```
Φ: 4×4      [estado a salida]
Ψ: 4×1      [control anterior a salida]
Λ: 4×4      [cambios de control a salida]
```

### Función de costo:
```
Q: 4×4      [pesos de error]
R: 4×4      [pesos de control]
H: 4×4      [Hessiana = 2(ΛᵀQΛ + R)]
f: 4×1      [gradiente]
```

### Restricciones:
```
A_ineq: 8×4    [8 restricciones de límite acumulativo]
b_ineq: 8×1    [límites]
```

### Variables de optimización:
```
Δu: 4×1    [cambios de control a optimizar]
```

---

## 9. PROPIEDADES CLAVE DEL MPC

| Propiedad | Ventaja | Desventaja |
|-----------|---------|-----------|
| **Horizonte finito N=4** | Cálculo rápido | Comportamiento limitado |
| **Formulación Sparse** | Menos variables (4 en lugar de pasos infinitos) | Requiere que u sea continua |
| **Cambios incrementales Δu** | Control suave, evita saltos | Depende del control anterior |
| **Matriz H definida positiva** | Siempre tiene solución única | No garantiza cumplimiento de restricciones |
| **Restricciones acumulativas** | Limita energía total | Puede saturar el control |

---

## 10. AJUSTE DE PESOS PARA ESPECIFICACIONES

Para cumplir **os% ≤ 1%** y **ts₂% ≤ 4s**:

```python
# Aumentar Q → menor sobrepico (menos oscilación)
# Aumentar R → respuesta más lenta (mayor ts)
# Encontrar balance iterativamente:

Para os% muy alto:
  → Aumentar Q (ej: 100 → 200)
  → Disminuir R (ej: 0.1 → 0.01)
  → Aumentar N (ej: 4 → 6)

Para ts% muy largo:
  → Disminuir Q (ej: 100 → 50)
  → Aumentar R permite respuesta más rápida
  → Aumentar límites de control (ej: ±10 → ±20)
```

---

## RESUMEN: PARÁMETROS DE DISEÑO

| Parámetro | Valor | Tipo | Descripción |
|-----------|-------|------|-------------|
| **z** | [Δu₀, Δu₁, Δu₂, Δu₃]ᵀ | 4×1 | Variables de optimización |
| **H** | 2(ΛᵀQΛ + R) | 4×4 | Matriz Hessiana |
| **f** | 2ΛᵀQ(Φx + Ψu - r·1) | 4×1 | Vector gradiente |
| **A_ineq** | Triangular acumulativa | 8×4 | Matriz restricciones desigualdad |
| **b_ineq** | [10-u, 10-u, ...] | 8×1 | Límites desigualdad |
| **A_eq** | N/A | - | No hay restricciones igualdad |
| **b_eq** | N/A | - | (Dinámica en Φ,Ψ,Λ) |
| **lb, ub** | [-∞, +∞] | 4×1 | Límites variables |

