# CODE_GENERAL

Esta carpeta queda preparada para guardar los codigos generales del curso por tema, de forma que sea rapido encontrar una plantilla cuando se resuelva un problema nuevo.

## Como organizar los archivos `.mlx`

1. Copia o deja tus archivos `.mlx` directamente dentro de `CODE_GENERAL/`.
2. Ejecuta desde la raiz del repositorio:

```bash
python CODE_GENERAL/_organizar_code_general.py
```

El script mueve cada archivo a una carpeta tematica segun su nombre. Tambien revisa subcarpetas como `TEORIA/` y evita sobrescribir duplicados agregando sufijos `_dupN`. Por ejemplo:

- `IMC_0.mlx` -> `03_IMC/`
- `LQR_1.mlx` -> `04_LQR_LQI/`
- `LQG_1.mlx` -> `05_LQG/`
- `MPC_0.mlx` -> `07_MPC/`
- `Policy_search_normal.mlx` -> `08_Policy_Search/`

> Nota para PowerShell: `gs` no es un comando de Git por defecto. Usa `git status` o crea un alias local si quieres abreviarlo.

## Estructura propuesta

Consulta `00_INDICE_CODIGOS.md` para ver las carpetas y el criterio de clasificacion.
