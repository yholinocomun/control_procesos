# ANÁLISIS VISUAL Y CONCEPTUAL DEL MPC (código extraído)

## 📋 TABLA RESUMEN: DIMENSIONES Y SIGNIFICADO

| Variable | Tamaño | Tipo | Significado |
|----------|--------|------|------------|
| **r** | 2×1 | Entrada | Referencia deseada (x1, x2) |
| **x** | 2×1 | Entrada | Estado actual (x1, x2) |
| **Ad** | 2×2 | Parámetro | Matriz dinámica discreta |
| **Bd** | 2×1 | Parámetro | Matriz entrada control |
| **N** | Escalar | Parámetro | Horizonte predicción = 5 pasos |
| **Q** | 2×2 | Parámetro | Peso error de seguimiento |
| **R** | Escalar | Parámetro | Peso esfuerzo control |
| **H** | 15×15 | Construcción | Hessiana (diagonal por bloques) |
| **f** | 15×1 | Construcción | Gradiente (lineal) |
| **Aeq** | 10×15 | Construcción | Matriz restricciones igualdad |
| **beq** | 10×1 | Construcción | Vector restricciones igualdad |
| **z** | 15×1 | Salida optim. | Solución: [u₀ u₁ u₂ u₃ u₄ x₁₀ x₂₀ ... x₂₄] |
| **u** | Escalar | Salida final | Control a aplicar = z(1) |

---

## 🔍 DESGLOSE DEL VECTOR DE OPTIMIZACIÓN z (15×1)

```
z = [z(1):5]        ← Controles futuros
    [z(6):15]       ← Estados futuros predichos

z(1)  = u[k]        Control ACTUAL (se aplica)
z(2)  = u[k+1]      Control predicho paso 1
z(3)  = u[k+2]      Control predicho paso 2
z(4)  = u[k+3]      Control predicho paso 3
z(5)  = u[k+4]      Control predicho paso 4

z(6)  = x1[k]       Estado 1 en tiempo actual
z(7)  = x2[k]       Estado 2 en tiempo actual
z(8)  = x1[k+1]     Estado 1 predicho paso 1
z(9)  = x2[k+1]     Estado 2 predicho paso 1
z(10) = x1[k+2]     Estado 1 predicho paso 2
z(11) = x2[k+2]     Estado 2 predicho paso 2
z(12) = x1[k+3]     Estado 1 predicho paso 3
z(13) = x2[k+3]     Estado 2 predicho paso 3
z(14) = x1[k+4]     Estado 1 predicho paso 4
z(15) = x2[k+4]     Estado 2 predicho paso 4
```

---

## 🎯 PROBLEMA DE OPTIMIZACIÓN EN FORMA EXPANDIDA

### Función Objetivo
```
min J = Σ(k=0 a 4) [ ||u[k]||²_R + ||x[k] - r||²_Q ]
 z

     = Σ(k=0 a 4) [ R·u[k]² + Q(1,1)·(x1[k]-r1)² + Q(2,2)·(x2[k]-r2)² ]
     
     = 0.1·(u₀² + u₁² + u₂² + u₃² + u₄²)
       + 10·[(x1₀-r1)² + (x1₁-r1)² + (x1₂-r1)² + (x1₃-r1)² + (x1₄-r1)²]
       + 1·[(x2₀-r2)² + (x2₁-r2)² + (x2₂-r2)² + (x2₃-r2)² + (x2₄-r2)²]
```

### Restricciones de Igualdad (Dinámicas)
```
x[k+1] - Ad·x[k] - Bd·u[k] = 0    para k = 0, 1, 2, 3, 4

En forma matricial:
Aeq · z = beq

donde:
Aeq = [Aeq_u | Aeq_x]
      [10×5 | 10×10]

beq = [Ad·x_actual]
      [0]
      [0]
      [0]
      [0]
```

### Restricciones de Caja
```
-100 ≤ u[k] ≤ 100      para k = 0, 1, 2, 3, 4
-100 ≤ x1[k] ≤ 100     para k = 0, 1, 2, 3, 4
-100 ≤ x2[k] ≤ 100     para k = 0, 1, 2, 3, 4

En forma matricial:
lb ≤ z ≤ ub
```

---

## 🔗 ESTRUCTURA DE MATRICES DE RESTRICCIÓN

### Matriz Aeq_u (10×5): Multiplicadores de control
```
       u₀    u₁    u₂    u₃    u₄
    ┌─────┬─────┬─────┬─────┬─────┐
x1₀ │-0.01│  0  │  0  │  0  │  0  │
x2₀ │-0.10│  0  │  0  │  0  │  0  │  Fila 1-2: Efecto de u₀ en x[k]
    ├─────┼─────┼─────┼─────┼─────┤
x1₁ │  0  │-0.01│  0  │  0  │  0  │
x2₁ │  0  │-0.10│  0  │  0  │  0  │  Fila 3-4: Efecto de u₁ en x[k+1]
    ├─────┼─────┼─────┼─────┼─────┤
x1₂ │  0  │  0  │-0.01│  0  │  0  │
x2₂ │  0  │  0  │-0.10│  0  │  0  │  Fila 5-6: Efecto de u₂ en x[k+2]
    ├─────┼─────┼─────┼─────┼─────┤
x1₃ │  0  │  0  │  0  │-0.01│  0  │
x2₃ │  0  │  0  │  0  │-0.10│  0  │  Fila 7-8: Efecto de u₃ en x[k+3]
    ├─────┼─────┼─────┼─────┼─────┤
x1₄ │  0  │  0  │  0  │  0  │-0.01│
x2₄ │  0  │  0  │  0  │  0  │-0.10│  Fila 9-10: Efecto de u₄ en x[k+4]
    └─────┴─────┴─────┴─────┴─────┘

Cada columna es -Bd = [-0.0097; -0.0952]
```

### Matriz Aeq_x (10×10): Transiciones de estado
```
       x1₀ x2₀ x1₁ x2₁ x1₂ x2₂ x1₃ x2₃ x1₄ x2₄
    ┌─────┬───────┬───────┬───────┬───────┬─────┐
    │ 1   0 │ 0   0 │ 0   0 │ 0   0 │ 0   0 │  ← I
    │ 0   1 │ 0   0 │ 0   0 │ 0   0 │ 0   0 │
    ├─────┼───────┼───────┼───────┼───────┼─────┤
    │-1.0 -0.19│ 1   0 │ 0   0 │ 0   0 │ 0   0 │  ← -Ad, I
    │ 0   -0.9 │ 0   1 │ 0   0 │ 0   0 │ 0   0 │
    ├─────┼───────┼───────┼───────┼───────┼─────┤
    │ 0   0 │-1.0 -0.19│ 1   0 │ 0   0 │ 0   0 │  ← -Ad, I
    │ 0   0 │ 0   -0.9 │ 0   1 │ 0   0 │ 0   0 │
    ├─────┼───────┼───────┼───────┼───────┼─────┤
    │ 0   0 │ 0   0 │-1.0 -0.19│ 1   0 │ 0   0 │  ← -Ad, I
    │ 0   0 │ 0   0 │ 0   -0.9 │ 0   1 │ 0   0 │
    ├─────┼───────┼───────┼───────┼───────┼─────┤
    │ 0   0 │ 0   0 │ 0   0 │-1.0 -0.19│ 1   0 │  ← -Ad, I
    │ 0   0 │ 0   0 │ 0   0 │ 0   -0.9 │ 0   1 │
    └─────┴───────┴───────┴───────┴───────┴─────┘
```

---

## 📊 CONSTRUCCIÓN DEL VECTOR GRADIENTE f

### Parte 1: Controles (f_u)
```
f_u = [0; 0; 0; 0; 0]

Razón: El costo de controles es PURO CUADRÁTICO: R·u²
       NO hay término lineal (como sería si hubiera costo de bias)
```

### Parte 2: Estados (f_x)
```
Q = [10  0]
    [0   1]

r = [r1]  (referencia)
    [r2]

-2·Q·r = -2·[10  0]·[r1]  = [-20·r1]
           [0   1] [r2]     [-2·r2]

f_x = repmat(-2·Q·r, 5, 1) = [-20·r1]
                              [-2·r2]
                              [-20·r1]
                              [-2·r2]
                              [-20·r1]
                              [-2·r2]
                              [-20·r1]
                              [-2·r2]
                              [-20·r1]
                              [-2·r2]

EJEMPLO: Si r = [10; 0]
         f_x = [-200; 0; -200; 0; -200; 0; -200; 0; -200; 0]ᵀ
```

### Vector Gradiente Completo
```
f = [f_u;     ← 5 elementos (controles, todos cero)
     f_x]     ← 10 elementos (estados, depende de r)

f = [0;      u₀
     0;      u₁
     0;      u₂
     0;      u₃
     0;      u₄
     -20·r1; x1[k]
     -2·r2;  x2[k]
     -20·r1; x1[k+1]
     -2·r2;  x2[k+1]
     -20·r1; x1[k+2]
     -2·r2;  x2[k+2]
     -20·r1; x1[k+3]
     -2·r2;  x2[k+3]
     -20·r1; x1[k+4]
     -2·r2]  x2[k+4]
```

---

## 🧮 MATRIZ HESSIANA H (15×15)

```
H = 2·blkdiag(R, R, R, R, R, Q, Q, Q, Q, Q)

     = 2·[R           0    ]
         [  R        0    ]
         [    R      0    ]
         [      R    0    ]
         [        R  0    ]
         [          Q    ]
         [            Q  ]
         [              Q]
         [                Q]
         [                  Q]

Donde R = 0.1 (escalar) y Q = [10  0]
                                [0   1]

H = [0.2  0   0   0   0  | 0   0   0  ...  0 ]
    [  0  0.2  0   0   0  | 0   0   0  ...  0 ]
    [  0   0  0.2  0   0  | 0   0   0  ...  0 ]
    [  0   0   0  0.2  0  | 0   0   0  ...  0 ]
    [  0   0   0   0  0.2 | 0   0   0  ...  0 ]  ← 5×5 bloque de control
    [────────────────────┼────────────────────]
    [  0   0   0   0   0  |20  0   0  ...  0 ]
    [  0   0   0   0   0  | 0  2   0  ...  0 ]  ← 2×2 bloque Q (paso 1)
    [  0   0   0   0   0  | 0  0  20  ...  0 ]
    [  0   0   0   0   0  | 0  0   0  ... Q ]   ← 2×2 bloques Q (pasos 2-5)
    [  0   0   0   0   0  | 0  0   0  ...  2 ]

Propiedades:
- Diagonal por bloques (estructura sparse)
- Definida positiva (todos eigenvalores > 0)
- Tamaño: 15×15
- Almacenamiento: Eficiente (muchos ceros)
```

---

## 🔄 FLUJO DE EJECUCIÓN EN RECEDING HORIZON

### Tiempo k=0
```
Entrada:
  x(t=0) = [estado actual]
  r      = [referencia]

Solver resuelve:
  z* = [u*(0), u*(1), u*(2), u*(3), u*(4),
        x*(1), x*(2), ..., x*(10)]

Acción:
  u_aplicado = z*(1) = u*(0)
  
Resultado:
  Sistema evoluciona: x(t=1) = Ad·x(t=0) + Bd·u(t=0)
```

### Tiempo k=1
```
Entrada:
  x(t=1) = [NUEVA medición del sistema real]
  r      = [referencia actualizada o igual]

Solver resuelve NUEVO PROBLEMA:
  (Nota: beq cambia porque x actual es diferente)
  z* = [u*(0), u*(1), u*(2), u*(3), u*(4),  ← NUEVOS valores
        x*(1), x*(2), ..., x*(10)]

Acción:
  u_aplicado = z*(1) = u*(0) en tiempo k=1
  
Nota importante: Lo que era z*(2) en t=0 NO se aplica
                 Se resuelve TODO el problema de nuevo
                 
Ventaja del Receding Horizon:
  - Se adapta a nuevas mediciones
  - Corrige errores de modelo
  - Rechaza perturbaciones
```

### Horizonte Visual
```
Tiempo real:   k=0         k=1         k=2         k=3        k=4
               ↓           ↓           ↓           ↓          ↓
               [x actual]  [x actual]  [x actual]  [x actual] [x actual]
               
Predicción t=0:
               [u*(0), u*(1), u*(2), u*(3), u*(4)]  ← Horizonte 5 pasos
               
               Aplica: u*(0)
               
Predicción t=1:
                        [u*(0), u*(1), u*(2), u*(3), u*(4)]  ← NUEVO horizonte
                        
                        Aplica: u*(0) (distinto del anterior)
                        
Receding Horizon:
               <-------- Horizonte recede en el tiempo -------->
```

---

## 💡 COMPARACIÓN: DISTINTOS VALORES DE PARÁMETROS

### Efecto de Q (peso del error)

| Q | Comportamiento | Ventaja | Desventaja |
|---|---|---|---|
| **Q pequeño (1)** | Respuesta lenta | Bajo esfuerzo | Tracking pobre |
| **Q medio (10)** | Respuesta balanceada | Buen balance | - |
| **Q grande (100)** | Respuesta rápida/agresiva | Tracking excelente | Alto esfuerzo |

### Efecto de R (peso del control)

| R | Comportamiento | Ventaja | Desventaja |
|---|---|---|---|
| **R pequeño (0.01)** | Control agresivo | Respuesta rápida | Alto uso energía |
| **R medio (0.1)** | Control balanceado | Eficiencia media | - |
| **R grande (1)** | Control suave | Bajo uso energía | Respuesta lenta |

### Efecto de N (horizonte)

| N | Comportamiento | Ventaja | Desventaja |
|---|---|---|---|
| **N=2** | Horizonte corto | Cálculo rápido | Visión limitada |
| **N=5** | Horizonte medio | Balance | - |
| **N=20** | Horizonte largo | Mejor predicción | Cómputo intenso |

---

## 🎁 INTERPRETACIÓN INTUITIVA

```
┌─────────────────────────────────────────────────────┐
│         PROBLEMA DEL MPC EN PALABRAS                │
├─────────────────────────────────────────────────────┤
│                                                     │
│  "Encuentra una secuencia de 5 controles           │
│   [u₀, u₁, u₂, u₃, u₄] y sus estados predichos    │
│   tal que:                                          │
│                                                     │
│   1) Los estados sigan la referencia r             │
│      (con penalización Q)                          │
│                                                     │
│   2) El esfuerzo de control sea mínimo             │
│      (con penalización R)                          │
│                                                     │
│   3) La dinámica sea respetada                      │
│      x[k+1] = Ad·x[k] + Bd·u[k]                   │
│                                                     │
│   4) Los límites sean respetados                    │
│      |u[k]| ≤ 100, |x1[k]| ≤ 100, |x2[k]| ≤ 100  │
│                                                     │
│   De todos esos controles, aplica solo el primero:│
│      u_aplicado = u₀"                             │
│                                                     │
└─────────────────────────────────────────────────────┘
```

---

## 📐 GEOMETRÍA DEL ESPACIO DE SOLUCIÓN

```
Espacio de solución z ∈ ℝ¹⁵

┌─────────────────────────────────────────┐
│  Región factible (satisface restricciones) │
│  ╱─────────────────────╲                 │
│ ╱  Punto óptimo z*      ╲                │
│ │  (mínima J)            │                │
│ │  ✓                     │                │
│ │  Gradiente = 0         │                │
│ │                        │                │
│ │ Función objetivo:      │                │
│ │ J = ½ z'Hz + f'z      │                │
│ │                        │                │
│ │ (Forma de cuenco)     │                │
│ │ (Convexo - una solución) │             │
│  ╲                      ╱                │
│   ╲──────────────────╱                  │
│                                          │
│ Restricciones límite: lb ≤ z ≤ ub      │
│ Restricciones dinámicas: Aeq·z = beq  │
└─────────────────────────────────────────┘

Notas:
- Región es CONVEXA (J es convexa + restricciones lineales)
- Solución ÚNICA garantizada
- Si inviable: no existe punto en región factible
```

