# GUÍA: MPC AUTOMÁTICO POR N Y nx

## 📋 Parámetros a Cambiar

El código se autoajusta para cualquier combinación de:

| Parámetro | Tipo | Rango | Efecto |
|-----------|------|-------|--------|
| **N** | `int` | 1, 2, 3, 4, 5, ... | Horizonte de predicción (pasos) |
| **nx** (automático) | deducido de Ad | 2, 4, 6, ... | Número de estados (se lee de `size(Ad,1)`) |
| **nu** (automático) | deducido de Bd | 1, 2, 3, ... | Número de controles (se lee de `size(Bd,2)`) |

---

## 🔄 Cambiar N (Horizonte de Predicción)

```matlab
N = 5;     % <-- CAMBIAS SOLO ESTE NÚMERO
```

**Ejemplos:**
```matlab
N = 1;   % Control muy miope (poco predictivo)
N = 3;   % Control conservador
N = 5;   % Balance típico
N = 10;  % Control altamente predictivo (más cálculo)
```

**Efecto en dimensiones:**
- Para `nx=4, nu=1, N=5`: dim(z) = 5·1 + 5·4 = 25
- Para `nx=4, nu=1, N=10`: dim(z) = 10·1 + 10·4 = 50 (más cómputo)

---

## 🏋️ Cambiar nx (Número de Estados)

El código **deduce `nx` automáticamente** de tu matriz Ad:

```matlab
Ad = [     1.0000         0    0.1058    0.0051
           0    1.0000    0.0560    0.1000
           0         0    1.1759    0.1058
           0         0    3.6183    1.1759];   % 4x4 -> nx = 4
```

**Si quieres 2 estados:**
```matlab
Ad = [1.0000  0.1903;
      0       0.9048];  % 2x2 -> nx = 2
Bd = [0.0097; 0.0952];  % 2x1 -> nu = 1
```

**Si quieres 6 estados:**
```matlab
Ad = [6x6 matrix];  % nx = 6
Bd = [6x1 matrix];  % nu = 1
% El código automáticamente:
% - Genera Q de 6x6
% - Limites de 6 estados
% - Restricciones Aeq de (N*6) x (N*1 + N*6)
```

---

## ⚖️ Opciones de Matriz Q (Pesos del Error)

El código incluye **DOS opciones** para penalizar los estados:

### **OPCIÓN 1: Pesos Decrecientes** (ACTIVA POR DEFECTO)
```matlab
Q_weights = 10 ./ (1:nx)';  % [10, 5, 3.33, 2.5, 2, ...]
Q = diag(Q_weights);
```

**Interpretación:** Los primeros estados se penalizan más.
```
nx=4 -> Q = diag([10, 5, 3.33, 2.5])

Efecto: 
  x₁ (posición): penalización 10   <- MÁS IMPORTANTE
  x₂ (velocidad): penalización 5
  x₃: penalización 3.33
  x₄: penalización 2.5              <- MENOS IMPORTANTE
```

**Cuándo usarla:** Cuando algunos estados son más críticos (ej: posición > velocidad).

---

### **OPCIÓN 2: Pesos Iguales**
```matlab
Q = diag(ones(nx, 1) * 10);
```

**Interpretación:** Todos los estados tienen igual importancia.
```
nx=4 -> Q = diag([10, 10, 10, 10])

Efecto: Penalización uniforme en todos los estados
```

**Cuándo usarla:** Cuando todos los estados son igualmente relevantes.

---

## 🔧 Cambiar entre Opciones de Q

**PASO 1:** Abre el archivo `mpc_automatizado.m`

**PASO 2:** Ubica esta sección:
```matlab
% Matriz de penalización de error (OPCIÓN 1: pesos decrecientes)
% Q = diag([10, 5, 3, 1, 0.5, ...]) para los primeros nx estados
Q_weights = 10 ./ (1:nx)';  % [10, 5, 3.33, 2.5, ...] para nx estados
Q = diag(Q_weights);

% (OPCIÓN 2: todos los estados con igual peso - descomenta si lo prefieres)
% Q = diag(ones(nx, 1) * 10);
```

**PASO 3:** Para cambiar a OPCIÓN 2, comenta/descomenta:
```matlab
% OPCIÓN 1 (COMENTADA)
% Q_weights = 10 ./ (1:nx)';
% Q = diag(Q_weights);

% OPCIÓN 2 (ACTIVA)
Q = diag(ones(nx, 1) * 10);
```

---

## 🎯 OPCIÓN 3: Personalizar Q Manualmente

Si ninguna de las dos opciones automáticas te sirve, especifica Q manualmente:

```matlab
% Ejemplo: penalizar x₁ mucho más que las demás
% x₁ (posición): 100
% x₂, x₃, x₄: 1 cada una
Q = diag([100, 1, 1, 1]);

% Ejemplo: penalizaciones personalizadas arbitrarias
Q = diag([15, 8, 4, 2]);
```

---

## 📊 Tabla: Cómo Escalas Todo Automáticamente

| Si cambias... | Afecta a... | Cálculo automático |
|---|---|---|
| **Ad** | nx (filas) | `nx = size(Ad, 1)` |
| **Bd** | nu (columnas) | `nu = size(Bd, 2)` |
| **N** | Horizonte | `dim(z) = N·nu + N·nx` |
| **xmax** | Límites estados | `repmat(xmax, N, 1)` |
| **Q** | Penalizaciones | `H = 2*blkdiag(I_N⊗R, I_N⊗Q)` |

---

## 📐 Dimensiones Resultantes

Con **N = 5**, **nx = 4**, **nu = 1**:

```
H:    (25, 25)   = (N·nu+N·nx) × (N·nu+N·nx)
f:    (25, 1)    = N·nu+N·nx
Aeq:  (20, 25)   = N·nx × (N·nu+N·nx)
beq:  (20, 1)    = N·nx
lb:   (25, 1)    = N·nu+N·nx
ub:   (25, 1)    = N·nu+N·nx

Vector z: [5 controles (u₀...u₄) + 20 estados (4×5 pasos)] = 25 elementos
```

---

## 🔌 Ejemplo Práctico: Cambiar de 2 a 4 Estados

### Caso 1: Sistema de 2 estados
```matlab
Ad = [1.0000  0.1903;
      0       0.9048];
Bd = [0.0097; 0.0952];
% Automáticamente: nx = 2, nu = 1
% Q es 2x2, límites de 2 estados
% Aeq es (N*2) x (N*1 + N*2) = (N*2) x (N*3)
```

### Caso 2: Sistema de 4 estados (TU CÓDIGO ACTUAL)
```matlab
Ad = [1.0000  0  0.1058  0.0051;
      0  1.0000  0.0560  0.1000;
      0  0  1.1759  0.1058;
      0  0  3.6183  1.1759];
Bd = [0.0060; -0.0201; 0.1165; -0.2009];
% Automáticamente: nx = 4, nu = 1
% Q es 4x4, límites de 4 estados
% Aeq es (N*4) x (N*1 + N*4) = (N*4) x (N*5)
```

**Lo único que cambias es Ad y Bd.** El resto se deduce solo.

---

## ⚡ Resumen: Qué Cambiar en Cada Caso

| Necesidad | Qué cambiar | Dónde |
|-----------|------------|-------|
| Diferente horizonte | `N = ...` | Línea 8 |
| Diferente planta | Reemplaza `Ad` y `Bd` | Líneas 16-26 |
| Diferente número de estados | Reemplaza `Ad` y `Bd` | Líneas 16-26 (deducido automático) |
| Diferentes pesos en Q | Comenta/descomenta opciones Q | Líneas 35-40 |
| Límites distintos | Cambia `umax`, `xmax` | Líneas 43-44 |

---

## ✅ Verificación Rápida

Abre MATLAB y prueba:
```matlab
>> fcn([10; 0; 5; 2], [1; 0.5; 2; -1])
ans =
   -0.1234   (ejemplo, valor depende del estado)
```

Si no hay errores de dimensión, ¡estás listo para usarlo en Simulink!

