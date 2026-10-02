#!/usr/bin/env python3
"""Run the original docplex/CPLEX MILP model with the free PuLP/CBC solver.

Injects `model/docplex_pulp_shim.py` as the `docplex.mp.model` module (and a
stub `cplex` module) into sys.modules, then executes the unchanged model file.
All DRT_* environment variables are honored exactly as in the CPLEX path.

Usage:
    DRT_DATA_DIR=.../tdrp_tw_realistic_v1/10-25 DRT_RESULTS_DIR=... \\
    DRT_INSTANCE_TAG=... DRT_RUN_TAG=... \\
        python3 model/run_with_pulp.py
"""

from __future__ import annotations

import runpy
import sys
import types
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import docplex_pulp_shim as shim  # noqa: E402

# Fabricate the docplex.mp.model module the model imports.
docplex_pkg = types.ModuleType("docplex")
mp_pkg = types.ModuleType("docplex.mp")
model_mod = types.ModuleType("docplex.mp.model")
model_mod.Model = shim.Model
docplex_pkg.mp = mp_pkg
mp_pkg.model = model_mod
sys.modules["docplex"] = docplex_pkg
sys.modules["docplex.mp"] = mp_pkg
sys.modules["docplex.mp.model"] = model_mod

# The model does `import cplex  # noqa: F401`; stub it out (never used directly).
sys.modules["cplex"] = types.ModuleType("cplex")

runpy.run_path(str(HERE / "capped_flexible_docking_ordered_sortie_model.py"),
               run_name="__main__")
