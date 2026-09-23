"""Small deterministic bounded least-squares allocator.

The wheel-lift controller has four actuators, so enumerating the three possible
states of each variable (lower bound, free, upper bound) costs at most 3**4 = 81
linear solves.  This is predictable, dependency-free, and directly portable to a
MATLAB Function block.
"""

from __future__ import annotations

import itertools
import math
from typing import Sequence


def _solve(matrix, rhs):
    n = len(rhs)
    if n == 0:
        return []
    augmented = [
        [float(matrix[i][j]) for j in range(n)] + [float(rhs[i])]
        for i in range(n)
    ]
    for column in range(n):
        pivot_row = max(range(column, n), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot_row][column]) < 1e-12:
            raise ValueError("singular normal equations")
        augmented[column], augmented[pivot_row] = augmented[pivot_row], augmented[column]
        pivot = augmented[column][column]
        augmented[column] = [value / pivot for value in augmented[column]]
        for row in range(n):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                value - factor * basis
                for value, basis in zip(augmented[row], augmented[column])
            ]
    return [augmented[i][-1] for i in range(n)]


def bounded_weighted_least_squares(
    matrix: Sequence[Sequence[float]],
    target: Sequence[float],
    lower: Sequence[float],
    upper: Sequence[float],
    effort_weight: float = 1e-9,
) -> list[float]:
    """Minimise ``||A*x-b||² + effort_weight*||x||²`` inside box bounds."""
    rows = [list(map(float, row)) for row in matrix]
    b = list(map(float, target))
    lo = list(map(float, lower))
    hi = list(map(float, upper))
    if not rows or len(rows) != len(b):
        raise ValueError("matrix rows must match target")
    n = len(rows[0])
    if any(len(row) != n for row in rows) or len(lo) != n or len(hi) != n:
        raise ValueError("inconsistent allocator dimensions")
    if any(a > z for a, z in zip(lo, hi)):
        raise ValueError("lower bound exceeds upper bound")

    best = None
    best_cost = math.inf
    # -1 = fixed low, 0 = free, +1 = fixed high.
    for state in itertools.product((-1, 0, 1), repeat=n):
        candidate = [0.0] * n
        fixed = [i for i, flag in enumerate(state) if flag]
        free = [i for i, flag in enumerate(state) if not flag]
        for i in fixed:
            candidate[i] = lo[i] if state[i] < 0 else hi[i]

        residual_target = [
            b[row] - sum(rows[row][i] * candidate[i] for i in fixed)
            for row in range(len(rows))
        ]
        if free:
            normal = [
                [
                    sum(rows[row][i] * rows[row][j] for row in range(len(rows)))
                    + (float(effort_weight) if i == j else 0.0)
                    for j in free
                ]
                for i in free
            ]
            rhs = [
                sum(rows[row][i] * residual_target[row] for row in range(len(rows)))
                for i in free
            ]
            try:
                solution = _solve(normal, rhs)
            except ValueError:
                continue
            for i, value in zip(free, solution):
                candidate[i] = value

        if any(value < lo[i] - 1e-9 or value > hi[i] + 1e-9
               for i, value in enumerate(candidate)):
            continue
        residual = [
            sum(row[i] * candidate[i] for i in range(n)) - wanted
            for row, wanted in zip(rows, b)
        ]
        cost = sum(value * value for value in residual) + float(effort_weight) * sum(
            value * value for value in candidate
        )
        if cost < best_cost - 1e-12:
            best_cost = cost
            best = candidate
    if best is None:
        raise ValueError("bounded least-squares problem has no finite candidate")
    return [min(hi[i], max(lo[i], value)) for i, value in enumerate(best)]

