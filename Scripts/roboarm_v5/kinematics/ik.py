"""
Numerical inverse kinematics for any serial revolute chain.

Levenberg-Marquardt on the geometric Jacobian, with Sugihara's error-scaled
damping:

    dq = (AᵀA + μI)⁻¹ Aᵀe,     μ = ½|e|² + μ₀

Far from the target μ is large and the step behaves like gradient descent
(safe through singularities); close to it μ vanishes and convergence is
Gauss-Newton fast. A step that makes the error worse is thrown away and
retried with ten times the damping, so the error never goes up.

Joint limits are enforced by clamping every step. The solver is restarted from
several seeds and the valid answer closest to where the arm is now wins, which
is what stops wrists flipping half a turn between neighbouring targets.

Modes
  "pose"      position + full orientation       (6 rows)  — 6+ joints
  "axis"      position + tool Z axis direction  (5 rows)  — 5-joint arms,
              or a 6-joint arm when you don't care about spin about the tool
  "position"  position only                      (3 rows)  — 3/4-joint arms
  "auto"      pick from the joint count

Redundant arms (more joints than constrained rows) use the leftover freedom to
drift toward mid-range, keeping away from limits.
"""

import math
from dataclasses import dataclass
from typing import List

import numpy as np

from .transforms import rotvec

ORI_WEIGHT_MM = 100.0      # 1 rad of orientation error "costs" as much as 100 mm
MU_0 = 1e-3                # damping floor
MAX_STEP_RAD = 0.35        # largest per-iteration joint change
MAX_RETRIES = 8            # damping increases per iteration before giving up
NULLSPACE_GAIN = 0.1


@dataclass
class Solution:
    ok: bool
    q: np.ndarray                         # radians, kinematic convention
    pos_err_mm: float
    ori_err_deg: float
    iterations: int
    reason: str
    mode: str = "pose"
    seeds_tried: int = 1

    @property
    def deg(self) -> List[float]:
        return [math.degrees(float(a)) for a in self.q]


def auto_mode(n):
    return "pose" if n >= 6 else "axis" if n == 5 else "position"


def _residual(mode, J, T, target):
    """
    Weighted residual e, matching Jacobian rows A, and the unweighted
    (position mm, orientation rad) errors used for the acceptance test.
    J may be None when only the error is needed.
    """
    e_p = target[:3, 3] - T[:3, 3]
    perr = float(np.linalg.norm(e_p))
    if mode == "position":
        return e_p, (None if J is None else J[:3]), perr, 0.0

    if mode == "pose":
        w = rotvec(target[:3, :3] @ T[:3, :3].T)
        ori = float(np.linalg.norm(w))
        A = None if J is None else np.vstack([J[:3], ORI_WEIGHT_MM * J[3:]])
        return np.concatenate([e_p, ORI_WEIGHT_MM * w]), A, perr, ori

    # axis: align the tool Z axis; spin about it is free, so project it out
    z, zt = T[:3, 2], target[:3, 2]
    c = np.cross(z, zt)
    s = float(np.linalg.norm(c))
    ori = math.atan2(s, float(np.dot(z, zt)))
    if s < 1e-12:
        if ori < 1e-6:
            w = np.zeros(3)
        else:                             # exactly opposed: turn about any perpendicular
            perp = np.cross(z, [1.0, 0, 0])
            if np.linalg.norm(perp) < 1e-6:
                perp = np.cross(z, [0, 1.0, 0])
            w = perp / np.linalg.norm(perp) * ori
    else:
        w = c / s * ori
    A = None
    if J is not None:
        P = np.eye(3) - np.outer(z, z)
        A = np.vstack([J[:3], ORI_WEIGHT_MM * (P @ J[3:])])
    return np.concatenate([e_p, ORI_WEIGHT_MM * w]), A, perr, ori


def _attempt(chain, target, q0, mode, pos_tol, ori_tol, max_iter, mid):
    n = chain.n
    q = np.clip(np.asarray(q0, float), chain.lo, chain.hi)
    J, T = chain.jacobian(q)
    e, A, perr, ori = _residual(mode, J, T, target)
    cost = float(e @ e)
    it = 0
    for it in range(1, max_iter + 1):
        if perr <= pos_tol and ori <= ori_tol:
            return True, q, perr, ori, it
        H = A.T @ A
        g = A.T @ e
        mu = 0.5 * cost + MU_0
        drift = None
        if n > A.shape[0]:
            # spare joints drift toward mid-range, projected so the task is untouched
            N = np.eye(n) - np.linalg.pinv(A) @ A
            drift = N @ (NULLSPACE_GAIN * (mid - q))
        for retry in range(MAX_RETRIES):
            dq = np.linalg.solve(H + mu * np.eye(n), g)
            if drift is not None and retry == 0:
                dq = dq + drift
            big = float(np.max(np.abs(dq)))
            if big > MAX_STEP_RAD:
                dq *= MAX_STEP_RAD / big
            q_new = np.clip(q + dq, chain.lo, chain.hi)
            T_new = chain.fk(q_new)
            e_new, _, perr_new, ori_new = _residual(mode, None, T_new, target)
            cost_new = float(e_new @ e_new)
            if cost_new < cost:
                break
            mu *= 10.0
        else:
            break                         # no step improves: local minimum or a limit
        q = q_new
        J, T = chain.jacobian(q)
        e, A, perr, ori = _residual(mode, J, T, target)
        cost = float(e @ e)
    ok = perr <= pos_tol and ori <= ori_tol
    return ok, q, perr, ori, it


def solve(chain, target, seed=None, mode="auto", pos_tol_mm=None, ori_tol_deg=None,
          max_iter=100, extra_seeds=10, rng_seed=0):
    """
    target   4x4 TCP pose in the world frame (only the position is used in
             "position" mode, only position + Z axis in "axis" mode).
    seed     current joint angles (rad). Defaults to mid-range.
    """
    model = chain.model
    if mode == "auto":
        mode = auto_mode(chain.n)
    if mode not in ("pose", "axis", "position"):
        raise ValueError("unknown IK mode {!r}".format(mode))
    pos_tol = model.ik_pos_tol_mm if pos_tol_mm is None else pos_tol_mm
    ori_tol = math.radians(model.ik_ori_tol_deg if ori_tol_deg is None else ori_tol_deg)
    target = np.asarray(target, float)
    mid = 0.5 * (chain.lo + chain.hi)
    here = mid if seed is None else np.clip(np.asarray(seed, float), chain.lo, chain.hi)

    seeds = [here]
    if seed is not None:
        seeds.append(mid)
    rng = np.random.default_rng(rng_seed)
    seeds += [rng.uniform(chain.lo, chain.hi) for _ in range(extra_seeds)]

    found, closest = [], None
    for k, s in enumerate(seeds):
        ok, q, perr, ori, it = _attempt(chain, target, s, mode, pos_tol, ori_tol,
                                        max_iter, mid)
        if ok:
            if k == 0:                    # converged from where the arm is: done
                return Solution(True, q, perr, math.degrees(ori), it, "ok", mode, 1)
            found.append((float(np.sum((q - here) ** 2)), q, perr, ori, it))
            if len(found) >= 3:           # enough candidates to pick the nearest
                break
        elif closest is None or perr + ORI_WEIGHT_MM * ori < closest[0]:
            closest = (perr + ORI_WEIGHT_MM * ori, q, perr, ori, it)

    if found:
        _, q, perr, ori, it = min(found, key=lambda f: f[0])
        return Solution(True, q, perr, math.degrees(ori), it, "ok", mode, k + 1)
    _, q, perr, ori, it = closest
    return Solution(False, q, perr, math.degrees(ori), it,
                    "unreachable within joint limits (best {:.1f} mm, {:.1f}°)"
                    .format(perr, math.degrees(ori)), mode, len(seeds))
