# Indice de codigos generales por tema

La idea de esta carpeta es tener un banco de codigos reutilizables. Cuando llegue un problema nuevo, busca primero en la carpeta del tema correspondiente y adapta el ejemplo mas cercano.

| Carpeta | Tema | Codigos esperados |
| --- | --- | --- |
| `01_Fundamentos_Modelado/` | Modelado, linealizacion, controlabilidad y observabilidad | `Lineal*`, `Linealizacion*`, `CONTRABILIDAD`, `OBSERVABILIDAD` |
| `02_PID_Estabilidad/` | PID, Ziegler-Nichols, error estacionario y criterios de estabilidad | `PID*`, `ZN*`, `ERROR_ESTADO_ESTACION`, `CRITERIO_ESTABILIDAD*` |
| `03_IMC/` | Control IMC y comparativas de lazo abierto/cerrado | `IMC*`, `COMPARATIVA_IMC` |
| `04_LQR_LQI/` | LQR y LQI continuo/discreto/no lineal | `LQR*`, `LQRi*` |
| `05_LQG/` | LQG, LQGi y observadores con integral | `LQG*`, `LQGi*`, `Bonus_LQG*` |
| `06_MIMO_Multilazo/` | Control MIMO, multilazo, centralizado, descentralizado y desacoplado | `CONTROL_*`, `MultiLazo*`, `Multilazo*`, `Pares*` |
| `07_MPC/` | Control predictivo MPC | `MPC*`, `Traking_MPC` |
| `08_Policy_Search/` | Busqueda de politicas y optimizacion por simulacion | `Policy_search*`, `Restricciones_Funciones` |
| `09_Discreto_Delay/` | Discretizacion, ZOH, Tustin/Euler y retardos | `Delay*`, `ESPAC_DISCRETO`, `EULER*`, `TRANSFORMADA_ZOH` |
| `10_Ejercicios_Practicas/` | Ejercicios, practicas y soluciones especificas | `E_P*`, `P1`, `P3_*`, `P4`, `p2`, `Sol_PC*` |
| `99_Bonus_Otros/` | Material extra o aun no clasificado | `Bonus*`, `pendulo`, `untitled*` |

## Flujo recomendado para resolver problemas

1. Identifica el tema del enunciado: IMC, LQR, LQG, MIMO, MPC, PID, discretizacion, etc.
2. Abre la carpeta tematica correspondiente.
3. Usa el archivo con nombre mas cercano al problema como plantilla.
4. Copia la plantilla a una carpeta de trabajo nueva antes de modificarla.
5. Si agregas codigos nuevos, dejalos en `CODE_GENERAL/` y vuelve a ejecutar `_organizar_code_general.py`.
