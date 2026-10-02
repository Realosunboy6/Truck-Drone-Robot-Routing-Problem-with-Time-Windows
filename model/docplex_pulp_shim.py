"""docplex-compatible facade over PuLP/CBC.

Lets the original CPLEX/docplex MILP model
(`capped_flexible_docking_ordered_sortie_model.py`) run unchanged against the
free PuLP + bundled CBC solver. Implements exactly the docplex API surface the
model uses:

  Model(name, ignore_names, checker)
    .parameters.timelimit / .threads / .emphasis.memory / .workmem / .mip.strategy.file
    .binary_var_dict(keys, name) / .continuous_var_dict(keys, lb, ub, name)
    .continuous_var(lb, ub, name)
    .add_constraint(constr, name) / .sum(iterable) / .minimize(expr)
    .new_solution() / .add_mip_start(sol, complete_vars=False)
    .add_kpi(expr, publish_name) / .print_information()
    .export_as_lp(path, basename) / .solve(log_output) / .solve_details.status
"""

from __future__ import annotations

import os
import re

import pulp


# ---------------------------------------------------------------- expressions

class _Constr:
    __slots__ = ("expr", "sense", "rhs")

    def __init__(self, expr: "Expr", sense: str, rhs: float):
        self.expr = expr
        self.sense = sense  # 'L', 'G' or 'E'
        self.rhs = rhs


class Expr:
    """Linear expression: sum(coef[var] * var) + const, vars are pulp LpVariables."""

    __slots__ = ("coefs", "const")
    __hash__ = object.__hash__

    def __init__(self, coefs=None, const: float = 0.0):
        self.coefs = dict(coefs) if coefs else {}
        self.const = float(const)

    # -- helpers -----------------------------------------------------------
    @staticmethod
    def _as_expr(other) -> "Expr":
        if isinstance(other, Expr):
            return other
        if isinstance(other, (int, float)):
            return Expr(const=other)
        raise TypeError(f"unsupported operand type: {type(other)}")

    def _binop(self, other, op) -> "Expr":
        other = Expr._as_expr(other)
        coefs = dict(self.coefs)
        for v, c in other.coefs.items():
            coefs[v] = coefs.get(v, 0.0) + (c if op == "+" else -c)
            if abs(coefs[v]) < 1e-15:
                del coefs[v]
        const = self.const + (other.const if op == "+" else -other.const)
        return Expr(coefs, const)

    # -- arithmetic ---------------------------------------------------------
    def __add__(self, other):
        return self._binop(other, "+")

    def __radd__(self, other):
        return self._binop(other, "+")

    def __sub__(self, other):
        return self._binop(other, "-")

    def __rsub__(self, other):
        return Expr._as_expr(other)._binop(self, "-")

    def __mul__(self, other):
        if isinstance(other, Expr):
            raise TypeError("nonlinear term: expr * expr")
        s = float(other)
        return Expr({v: c * s for v, c in self.coefs.items()}, self.const * s)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __truediv__(self, other):
        if isinstance(other, Expr):
            raise TypeError("nonlinear term: expr / expr")
        return self.__mul__(1.0 / float(other))

    def __neg__(self):
        return Expr({v: -c for v, c in self.coefs.items()}, -self.const)

    # -- comparisons -> constraints ------------------------------------------
    def _cmp(self, other, sense: str) -> _Constr:
        # (self - other) sense 0
        diff = self._binop(other, "-")
        return _Constr(diff, sense, 0.0)

    def __le__(self, other):
        return self._cmp(other, "L")

    def __ge__(self, other):
        return self._cmp(other, "G")

    def __eq__(self, other):
        return self._cmp(other, "E")

    def __ne__(self, other):  # pragma: no cover - not used by the model
        raise TypeError("!= is not a linear constraint")

    # -- evaluation -----------------------------------------------------------
    def value(self) -> float:
        total = self.const
        for v, c in self.coefs.items():
            vv = v.value()
            if vv is None:
                raise ValueError("variable has no value")
            total += c * vv
        return total


class Var(Expr):
    """Decision variable; behaves as an Expr with a single coefficient."""

    __slots__ = ("lp_var", "name")

    def __init__(self, lp_var: pulp.LpVariable, name: str):
        self.lp_var = lp_var
        self.name = name
        super().__init__({lp_var: 1.0}, 0.0)

    def set_initial_value(self, value: float) -> None:
        self.lp_var.setInitialValue(float(value))

    # -- docplex-style bound accessors (used by TRUCK_ONLY mode: var.ub = 0)
    @property
    def lb(self):
        return self.lp_var.lowBound

    @lb.setter
    def lb(self, value) -> None:
        self.lp_var.lowBound = None if value is None else float(value)

    @property
    def ub(self):
        return self.lp_var.upBound

    @ub.setter
    def ub(self, value) -> None:
        self.lp_var.upBound = None if value is None else float(value)


# ------------------------------------------------------------------ solutions

class _MipStart:
    def __init__(self):
        self.pairs: list[tuple[Var, float]] = []

    def add_var_value(self, var: Var, value: float) -> None:
        self.pairs.append((var, float(value)))


class _SolutionProxy:
    """Returned by Model.solve(); only get_value() is used by the model."""

    def __init__(self, model: "Model"):
        self._model = model

    def get_value(self, obj):
        if isinstance(obj, Expr):
            val = obj.value()
        else:
            val = float(obj)
        if val is None:
            raise ValueError("no value available")
        return val


class _SolveDetails:
    def __init__(self, status: str = "not solved"):
        self.status = status


class _ParamNamespace:
    def __init__(self):
        self.__dict__["_vals"] = {}

    def __getattr__(self, name):
        vals = self.__dict__["_vals"]
        if name not in vals:
            vals[name] = _ParamNamespace()
        return vals[name]

    def __setattr__(self, name, value):
        self.__dict__["_vals"][name] = value


# --------------------------------------------------------------------- model

_NAME_RE = re.compile(r"[^A-Za-z0-9_]+")


def _safe(name: str) -> str:
    return _NAME_RE.sub("_", name)


class Model:
    _STATUS_MAP = {
        "Optimal": "optimal",
        "Feasible": "feasible",
        "Not Solved": "not solved",
        "Infeasible": "infeasible",
        "Unbounded": "unbounded",
        "Undefined": "unknown",
    }

    def __init__(self, name: str = "model", **kwargs):
        self.name = name
        self.parameters = _ParamNamespace()
        self._prob = pulp.LpProblem(_safe(name), pulp.LpMinimize)
        self._constraints: list[tuple[_Constr, str | None]] = []
        self._objective: Expr | None = None
        self._mip_starts: list[_MipStart] = []
        self._kpis: dict[str, Expr] = {}
        self._var_counter = 0
        self._all_vars: list[tuple[Var, str]] = []  # (var, "Binary"/"Continuous")
        self.solve_details: _SolveDetails | None = None
        self._built = False

    # -- variables ------------------------------------------------------------
    def _new_var(self, name: str, lb, ub, cat) -> Var:
        self._var_counter += 1
        safe = _safe(f"{name}_{self._var_counter}")
        lp_var = pulp.LpVariable(safe, lowBound=lb, upBound=ub, cat=cat)
        var = Var(lp_var, safe)
        self._all_vars.append((var, "Binary" if cat == pulp.LpBinary else "Continuous"))
        return var

    def binary_var_dict(self, keys, name: str = "x") -> dict:
        return {k: self._new_var(name, 0, 1, pulp.LpBinary) for k in keys}

    def continuous_var_dict(self, keys, lb=None, ub=None, name: str = "x") -> dict:
        lb = 0.0 if lb is None else float(lb)
        ub = None if ub is None else float(ub)
        return {k: self._new_var(name, lb, ub, pulp.LpContinuous) for k in keys}

    def continuous_var(self, lb=None, ub=None, name: str = "x") -> Var:
        lb = 0.0 if lb is None else float(lb)
        ub = None if ub is None else float(ub)
        return self._new_var(name, lb, ub, pulp.LpContinuous)

    # -- model building --------------------------------------------------------
    @staticmethod
    def sum(iterable) -> Expr:
        total = Expr()
        for term in iterable:
            total = total + term
        return total

    def add_constraint(self, constr: _Constr, name: str | None = None,
                       ctname: str | None = None) -> _Constr:
        if not isinstance(constr, _Constr):
            raise TypeError(f"add_constraint expects a comparison, got {type(constr)}")
        self._constraints.append((constr, ctname if ctname is not None else name))
        return constr

    def minimize(self, expr) -> None:
        self._objective = expr if isinstance(expr, Expr) else Expr(const=float(expr))

    def add_kpi(self, expr, publish_name: str | None = None) -> None:
        if publish_name:
            self._kpis[publish_name] = expr

    # -- warm starts -------------------------------------------------------------
    def new_solution(self) -> _MipStart:
        return _MipStart()

    def add_mip_start(self, mip_start: _MipStart, complete_vars: bool = False) -> None:
        self._mip_starts.append(mip_start)

    # -- introspection ------------------------------------------------------------
    def print_information(self) -> None:
        n_bin = sum(1 for _, k in self._all_vars if k == "Binary")
        n_cont = sum(1 for _, k in self._all_vars if k != "Binary")
        print(f"Model '{self.name}': {n_bin} binary, {n_cont} continuous variables, "
              f"{len(self._constraints)} constraints (PuLP/CBC backend).")

    def export_as_lp(self, path: str | None = None, basename: str | None = None) -> str:
        self._build()
        path = path or "."
        basename = basename or self.name
        os.makedirs(path, exist_ok=True)
        lp_path = os.path.join(path, f"{_safe(basename)}.lp")
        self._prob.writeLP(lp_path)
        return lp_path

    # -- solve ---------------------------------------------------------------------
    def _build(self) -> None:
        if self._built:
            return
        if self._objective is None:
            raise RuntimeError("no objective set")
        obj = self._objective
        pulp_obj = pulp.lpSum([c * v for v, c in obj.coefs.items()]) + obj.const
        self._prob += pulp_obj
        for i, (constr, name) in enumerate(self._constraints):
            e = constr.expr
            lhs = pulp.lpSum([c * v for v, c in e.coefs.items()]) + e.const
            cname = _safe(name) if name else f"c{i}"
            if constr.sense == "L":
                self._prob += (lhs <= constr.rhs, cname)
            elif constr.sense == "G":
                self._prob += (lhs >= constr.rhs, cname)
            else:
                self._prob += (lhs == constr.rhs, cname)
        for start in self._mip_starts:
            for var, value in start.pairs:
                var.set_initial_value(value)
        self._built = True

    def solve(self, log_output: bool = False):
        self._build()
        timelimit = None
        try:
            timelimit = float(self.parameters._vals.get("timelimit"))
        except (TypeError, ValueError):
            timelimit = None
        threads = 0
        try:
            threads = int(self.parameters._vals.get("threads", 0))
        except (TypeError, ValueError):
            threads = 0
        solver = pulp.PULP_CBC_CMD(msg=1 if log_output else 0,
                                   timeLimit=timelimit,
                                   threads=threads,
                                   # Pass the MIP start to CBC only when the model
                                   # actually registered one; otherwise PuLP would
                                   # write a bogus all-zero start file.
                                   warmStart=bool(self._mip_starts))
        self._prob.solve(solver)
        raw = pulp.LpStatus[self._prob.status]
        sol_status = self._prob.sol_status
        # PuLP upgrades "Stopped on time (with incumbent)" to LpStatusOptimal;
        # sol_status distinguishes a proven optimum from a time-limit incumbent.
        # Only trust variable values when CBC reports an actual integer solution.
        if sol_status == pulp.LpSolutionOptimal:
            status = "optimal"
        elif sol_status == pulp.LpSolutionIntegerFeasible:
            status = "feasible"
        else:
            status = self._STATUS_MAP.get(raw, raw.lower())
        self.solve_details = _SolveDetails(status)
        if sol_status in (pulp.LpSolutionOptimal, pulp.LpSolutionIntegerFeasible):
            return _SolutionProxy(self)
        return None