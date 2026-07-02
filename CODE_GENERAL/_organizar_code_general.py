#!/usr/bin/env python3
"""Organiza CODE_GENERAL por temas usando reglas basadas en nombres de archivo.

Uso, desde la raiz del repositorio:
    python CODE_GENERAL/_organizar_code_general.py

El script es conservador:
- Solo mueve archivos dentro de CODE_GENERAL.
- No sobreescribe archivos existentes; si hay conflicto agrega sufijo _dupN.
- Ignora README, indice, este script y archivos ocultos.
"""

from __future__ import annotations

import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent

CATEGORIES: list[tuple[str, tuple[str, ...]]] = [
    ("01_Fundamentos_Modelado", ("LINEAL", "CONTRABILIDAD", "OBSERVABILIDAD")),
    ("02_PID_Estabilidad", ("PID", "ZN", "ERROR_ESTADO_ESTACION", "CRITERIO_ESTABILIDAD")),
    ("03_IMC", ("IMC", "COMPARATIVA_IMC")),
    ("04_LQR_LQI", ("LQR", "LQRI", "LQR_", "LQRi".upper())),
    ("05_LQG", ("LQG", "LQGI", "BONUS_LQG")),
    (
        "06_MIMO_Multilazo",
        (
            "CONTROL_CENTRALIZADO",
            "CONTROL_DESACOPLADO",
            "CONTROL_DESCENTRALIZADO",
            "MULTILAZO",
            "MULTILAXO",
            "PARES_DESCENTRALIZADO",
            "MIMO",
        ),
    ),
    ("07_MPC", ("MPC", "TRAKING_MPC", "TRACKING_MPC")),
    ("08_Policy_Search", ("POLICY_SEARCH", "RESTRICCIONES_FUNCIONES")),
    ("09_Discreto_Delay", ("DELAY", "ESPAC_DISCRETO", "EULER", "TRANSFORMADA_ZOH")),
    ("10_Ejercicios_Practicas", ("E_P", "P1", "P2", "P3", "P4", "P5", "SOL_PC")),
    ("99_Bonus_Otros", ("BONUS", "PENDULO", "UNTITLED")),
]

IGNORED_NAMES = {
    "README.md",
    "00_INDICE_CODIGOS.md",
    Path(__file__).name,
}


def classify(path: Path) -> str:
    """Return destination folder for a file based on its uppercase stem."""
    name = path.stem.upper()

    # LQG must be checked before LQR-like generic rules; categories order handles it.
    for folder, tokens in CATEGORIES:
        if any(token in name for token in tokens):
            return folder
    return "99_Bonus_Otros"


def unique_destination(destination: Path) -> Path:
    """Avoid overwriting by adding _dupN before the extension."""
    if not destination.exists():
        return destination

    stem = destination.stem
    suffix = destination.suffix
    parent = destination.parent
    counter = 1
    while True:
        candidate = parent / f"{stem}_dup{counter}{suffix}"
        if not candidate.exists():
            return candidate
        counter += 1


def should_move(path: Path) -> bool:
    if path.name in IGNORED_NAMES:
        return False
    if path.name.startswith("."):
        return False
    if path.is_dir():
        return False
    return path.suffix.lower() in {".mlx", ".m", ".mat", ".slx", ".slxc", ".zip"}


def is_already_organized(path: Path) -> bool:
    """Avoid re-moving files that are already inside a category folder."""
    relative_parts = path.relative_to(ROOT).parts
    if len(relative_parts) < 2:
        return False
    category_names = {folder for folder, _ in CATEGORIES}
    return relative_parts[0] in category_names


def main() -> None:
    moved = 0
    for folder, _ in CATEGORIES:
        (ROOT / folder).mkdir(exist_ok=True)

    for path in sorted(ROOT.rglob("*")):
        if is_already_organized(path):
            continue
        if not should_move(path):
            continue
        folder = classify(path)
        destination = unique_destination(ROOT / folder / path.name)
        shutil.move(str(path), str(destination))
        print(f"{path.relative_to(ROOT)} -> {destination.relative_to(ROOT)}")
        moved += 1

    if moved == 0:
        print("No se encontraron archivos nuevos para mover en CODE_GENERAL/.")
    else:
        print(f"Organizacion completada: {moved} archivo(s) movido(s).")


if __name__ == "__main__":
    main()
