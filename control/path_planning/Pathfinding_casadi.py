"""
Figure-8 — Your exact waypoint sequence
  origin → right buoy1 → mid → left buoy2 → top → right buoy2 → mid → left buoy1 → origin

Waypoints are HARD equality constraints (small box), pinned to fixed t values.
This guarantees the ordering. Buoy exclusion is enforced along the full path.
"""

import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# SHIP PARAMETERS
# ---------------------------------------------------------------------------
@dataclass
class Params:
    m:   float = 320.0
    Iz:  float = 120.0
    xG:  float = 0.0
    Xu_dot: float = -8.0
    Yv_dot: float = -20.0
    Nr_dot: float = -8.0
    Kt:    float = 1e-4
    n_max: float = 3000.0 * np.pi / 30

    @property
    def m11(self): return self.m - self.Xu_dot
    @property
    def m22(self): return self.m - self.Yv_dot
    @property
    def m66(self): return self.Iz - self.Nr_dot
    @property
    def T_max(self): return self.Kt * self.n_max**2


# ---------------------------------------------------------------------------
# SCENARIO
# ---------------------------------------------------------------------------
BUOY1  = np.array([55.0, 45.0])
BUOY2  = np.array([105.0, 95.0])
R      = 10.0
ORIGIN = np.array([0.0, 0.0])

# Geometry helpers
dv    = BUOY2 - BUOY1
ax    = dv / np.linalg.norm(dv)          # unit vec buoy1→buoy2
perp  = np.array([-ax[1], ax[0]])        # 90° CCW  (the "left" side)
mid   = (BUOY1 + BUOY2) / 2.0
D     = R + 3.0   # standoff from buoy centre for side waypoints
Dtop  = R + 3.0   # standoff for top waypoint

# ---------------------------------------------------------------------------
# WAYPOINTS — your exact sequence
#   right buoy1  = buoy1 shifted in the +ax direction (right along the axis)
#                  and slightly away in -perp so it's clearly to the "right"
#   left  buoy1  = buoy1 shifted in the -ax direction, +perp side
#   left  buoy2  = buoy2 shifted in the -ax direction (approaching from mid)
#   right buoy2  = buoy2 shifted in the +ax direction
#   top   buoy2  = buoy2 shifted in the +perp direction (above the axis)
# ---------------------------------------------------------------------------
WP = {
    "right_b1": BUOY1 + D * ax   - D * perp,   # right of buoy1
    "mid_1":    mid,                             # crossover (first pass)
    "left_b2":  BUOY2 - D * ax   + D * perp,   # left of buoy2
    "top":      BUOY2             + Dtop * perp, # top of buoy2
    "right_b2": BUOY2 + D * ax   - D * perp,   # right of buoy2
    "mid_2":    mid + 0.5 * perp,               # crossover (second pass, tiny offset so optimizer doesn't collapse both)
    "left_b1":  BUOY1 - D * ax   + D * perp,   # left of buoy1
}

WAYPOINTS = np.array(list(WP.values()))
WP_NAMES  = list(WP.keys())

# Pin each waypoint to a fixed t value — evenly spaced inside (0, 1)
T_WP = np.linspace(0.10, 0.90, len(WAYPOINTS))
WP_TOL = 1.2   # metres — tight enough to enforce topology


# ---------------------------------------------------------------------------
# B-SPLINE
# ---------------------------------------------------------------------------
def bspline_basis(i, k, t, knots):
    if k == 0:
        return ca.if_else(ca.logic_and(t >= knots[i], t < knots[i+1]), 1.0, 0.0)
    term1, term2 = 0, 0
    d1 = float(knots[i+k]   - knots[i])
    d2 = float(knots[i+k+1] - knots[i+1])
    if d1 > 1e-12:
        term1 = (t - knots[i])         / d1 * bspline_basis(i,   k-1, t, knots)
    if d2 > 1e-12:
        term2 = (knots[i+k+1] - t)     / d2 * bspline_basis(i+1, k-1, t, knots)
    return term1 + term2

def make_knots(n, degree):
    return np.concatenate([np.zeros(degree),
                           np.linspace(0, 1, n - degree + 1),
                           np.ones(degree)])

def bspline_curve(Px, Py, t, degree=3):
    knots = make_knots(Px.shape[0], degree)
    x, y = 0, 0
    for i in range(Px.shape[0]):
        b = bspline_basis(i, degree, t, knots)
        x += b * Px[i]; y += b * Py[i]
    return x, y

def eval_curve(Px, Py, t_vals, degree=3):
    t_vals = np.clip(t_vals, 0.0, 0.999)
    xs, ys = [], []
    for ti in t_vals:
        xi, yi = bspline_curve(Px, Py, float(ti), degree)
        xs.append(float(xi)); ys.append(float(yi))
    return np.array(xs), np.array(ys)


# ---------------------------------------------------------------------------
# OPTIMIZER
# ---------------------------------------------------------------------------
def build_optimizer():
    opti = ca.Opti()
    Np   = 16
    Px   = opti.variable(Np)
    Py   = opti.variable(Np)

    # Initial guess: linear interpolation through the full waypoint sequence
    all_pts = np.vstack([ORIGIN, WAYPOINTS, ORIGIN])
    t_all   = np.linspace(0, 1, len(all_pts))
    t_cp    = np.linspace(0, 1, Np)
    opti.set_initial(Px, np.interp(t_cp, t_all, all_pts[:, 0]))
    opti.set_initial(Py, np.interp(t_cp, t_all, all_pts[:, 1]))

    # --- Waypoint constraints (hard, ordered) ---
    for i, (wx, wy) in enumerate(WAYPOINTS):
        ti     = float(T_WP[i])
        xw, yw = bspline_curve(Px, Py, ti, degree=3)
        opti.subject_to((xw - wx)**2 + (yw - wy)**2 <= WP_TOL**2)

    # --- Buoy exclusion + cost along entire path ---
    Nt    = 80
    eps   = 1e-3
    cost  = 0

    for t in np.linspace(0.02, 0.98, Nt):
        x,  y  = bspline_curve(Px, Py, t,       degree=3)
        xf, yf = bspline_curve(Px, Py, t + eps, degree=3)
        xb, yb = bspline_curve(Px, Py, t - eps, degree=3)

        xd  = (xf - xb) / (2*eps)
        yd  = (yf - yb) / (2*eps)
        xdd = (xf - 2*x + xb) / eps**2
        ydd = (yf - 2*y + yb) / eps**2

        length       = ca.sqrt(xd**2 + yd**2 + 1e-8)
        curvature_sq = (xdd**2 + ydd**2) / (length**2 + 1e-6)

        cost += 1.0  * length
        cost += 0.05 * curvature_sq

        # Hard exclusion
        opti.subject_to((x - BUOY1[0])**2 + (y - BUOY1[1])**2 >= R**2)
        opti.subject_to((x - BUOY2[0])**2 + (y - BUOY2[1])**2 >= R**2)

    # --- Boundary conditions ---
    opti.subject_to(Px[0]  == ORIGIN[0]); opti.subject_to(Py[0]  == ORIGIN[1])
    opti.subject_to(Px[-1] == ORIGIN[0]); opti.subject_to(Py[-1] == ORIGIN[1])

    # --- Smoothness ---
    for i in range(Np - 1):
        cost += 0.1 * ((Px[i+1] - Px[i])**2 + (Py[i+1] - Py[i])**2)

    opti.minimize(cost)
    opti.solver("ipopt",
        {"print_time": True},
        {
            "max_iter": 2000, "tol": 1e-4,
            "acceptable_tol": 1e-3, "acceptable_iter": 10,
            "mu_strategy": "adaptive",
            "nlp_scaling_method": "gradient-based",
            "hessian_approximation": "limited-memory",
        }
    )

    sol = opti.solve()
    return sol.value(Px), sol.value(Py)


# ---------------------------------------------------------------------------
# TOPOLOGY CHECK
# ---------------------------------------------------------------------------
def winding_number(xs, ys, cx, cy):
    angles = np.arctan2(ys - cy, xs - cx)
    return np.sum(np.diff(np.unwrap(angles))) / (2 * np.pi)


# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("Waypoints (in order):")
    for name, t, (wx, wy) in zip(WP_NAMES, T_WP, WAYPOINTS):
        print(f"  t={t:.2f}  {name:<12}  ({wx:.2f}, {wy:.2f})")

    print("\nSolving...")
    Px, Py = build_optimizer()

    t_plot = np.linspace(0, 1, 500)
    x_path, y_path = eval_curve(Px, Py, t_plot)

    w1 = winding_number(x_path, y_path, *BUOY1)
    w2 = winding_number(x_path, y_path, *BUOY2)
    print(f"\nWinding buoy1: {w1:+.2f}   winding buoy2: {w2:+.2f}")
    ok = np.sign(w1) != np.sign(w2) and abs(w1) > 0.6 and abs(w2) > 0.6
    print("✓ Figure-8 confirmed" if ok else "✗ Topology check FAILED")

    # --- Plot ---
    fig, ax = plt.subplots(figsize=(8, 8))

    ax.plot(x_path, y_path, lw=2.5, color='steelblue', label='Path', zorder=3)

    # Waypoints with labels
    for i, (name, (wx, wy)) in enumerate(zip(WP_NAMES, WAYPOINTS)):
        ax.scatter(wx, wy, c='orange', s=80, zorder=5)
        ax.annotate(f"{i+1}: {name}", (wx, wy),
                    xytext=(6, 4), textcoords='offset points', fontsize=7.5)

    # Direction arrows along the path
    for t_arrow in [0.06, 0.20, 0.35, 0.50, 0.63, 0.76, 0.88, 0.96]:
        t0, t1 = t_arrow - 0.01, t_arrow + 0.01
        xs0, ys0 = eval_curve(Px, Py, [t0])
        xs1, ys1 = eval_curve(Px, Py, [t1])
        ax.annotate("", xy=(xs1[0], ys1[0]), xytext=(xs0[0], ys0[0]),
                    arrowprops=dict(arrowstyle='->', color='steelblue', lw=1.5))

    # Buoys
    for (bx, by), col, lbl in zip([BUOY1, BUOY2],
                                   ['royalblue', 'tomato'],
                                   ['Buoy 1', 'Buoy 2']):
        ax.add_patch(plt.Circle((bx, by), R,
                                color=col, fill=True, alpha=0.12, zorder=2))
        ax.add_patch(plt.Circle((bx, by), R,
                                color=col, fill=False, lw=2, zorder=2))
        ax.scatter(bx, by, c=col, s=80, zorder=6)
        ax.annotate(lbl, (bx, by), xytext=(6, -14),
                    textcoords='offset points',
                    fontsize=10, color=col, fontweight='bold')

    ax.scatter(*ORIGIN, c='black', s=150, zorder=7, marker='*', label='Start/End')

    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)
    ax.set_title(f'Figure-8   w(buoy1)={w1:+.2f}   w(buoy2)={w2:+.2f}')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    plt.tight_layout()
    plt.savefig('figure8_ordered.png', dpi=150)
    print('Saved → figure8_ordered.png')
    plt.show()