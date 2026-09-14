"""Static layering check (PROJECT.md section 3): hardware must not import the MPC side."""

from __future__ import annotations

import ast
from pathlib import Path

HW_DIR = Path(__file__).resolve().parent.parent / "src" / "aqua_bridge" / "hw"


def test_hw_modules_do_not_import_control() -> None:
    paths = sorted(HW_DIR.glob("*.py"))
    assert {p.name for p in paths} >= {"aquacomputer.py", "hidraw.py", "sources.py"}
    for path in paths:
        tree = ast.parse(path.read_text(), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module] if node.module else []
            else:
                continue
            for name in names:
                assert name is not None
                assert not name.startswith("aqua_bridge.control"), (
                    f"{path} imports {name!r}, hardware must not import control/mpc"
                )
                assert name != "aqua_bridge.control"


def test_protocol_module_is_pure() -> None:
    """hw/aquacomputer.py decodes and encodes bytes only: stdlib, no I/O modules."""
    tree = ast.parse((HW_DIR / "aquacomputer.py").read_text())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    assert imported <= {"__future__", "collections", "dataclasses", "struct"}, imported
