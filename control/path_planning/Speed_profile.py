"""
Phase 2 — Time-optimal speed profile along the fixed figure-8 path
===================================================================
Given: the B-spline path from phase 1 (Px, Py control points)
Find:  v(s) — speed as a function of arc length — that minimises
       total traversal time T = ∫ ds/v(s)
       subject to the required thrust never exceeding T_max.

Azimuth thruster model
----------------------
The thruster can rotate freely 360°, so it always points in exactly
the direction the dynamics require.  This means:

  T_req(s)  = sqrt(X_req(s)^2 + Y_req(s)^2)   <= T_max
  delta(s)  = atan2(Y_req(s), X_req(s))          (derived, not optimised)
  n(s)      = sign(T) * sqrt(|T| / Kt)           (derived from T)

where X_req, Y_req come from the 3-DOF equations of motion expressed
along the path using the chain rule:

  x_dot  = v * cos(psi)       y_dot  = v * sin(psi)
  x_ddot = v_dot*cos(psi) - v*psi_dot*sin(psi)   (and similarly y_ddot)

with psi(s) and kappa(s) = dpsi/ds precomputed from the geometry.

The optimisation variable is w(s) = v(s)^2  (convex substitution).
Minimising ∫ ds/v = ∫ ds/sqrt(w) with w > 0.
The thrust constraint becomes quadratic in w and its derivative w',
making the problem a convex QP (after discretisation).

Discretisation
--------------
Arc length s is discretised into N uniform steps.
w[i] = v[i]^2 at each node.
v_dot[i] ≈ (v[i+1]-v[i-1])/(2*ds) * v[i]   (chain rule: dv/dt = dv/ds * v)
"""

import numpy as np
import casadi as ca
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.integrate import trapezoid
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# SHIP PARAMETERS  (copy from your phase 1 file)
# ---------------------------------------------------------------------------
@dataclass
class Params:
    m:   float = 320.0
    Iz:  float = 120.0
    xG:  float = 0.0

    Xu:  float = 0#-2.0
    Xuu: float = -5.0
    Yv:  float = 0#-8.0
    Yvv: float =  -3.0
    Nr:  float = 0#-4.0
    Nrr: float =  -1.0

    Xu_dot: float = 0 #-8.0
    Yv_dot: float = 0# -20.0
    Nr_dot: float = 0# -8.0

    Kt:    float = 1e-4
    n_max: float = 3000.0*np.pi/30    # rad/s

    @property
    def m11(self): return self.m - self.Xu_dot
    @property
    def m22(self): return self.m - self.Yv_dot
    @property
    def m66(self): return self.Iz - self.Nr_dot
    @property
    def T_max(self): return self.Kt * self.n_max**2


# ---------------------------------------------------------------------------
# B-SPLINE  (unchanged from phase 1)
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

def bspline_curve_np(Px, Py, t_val, degree=3):
    """Pure-numpy evaluation for post-processing."""
    knots = make_knots(len(Px), degree)
    def basis(i, k, t):
        if k == 0:
            return 1.0 if knots[i] <= t < knots[i+1] else 0.0
        t1 = (t - knots[i]) / (knots[i+k] - knots[i] + 1e-16) * basis(i, k-1, t) \
             if knots[i+k] > knots[i] else 0.0
        t2 = (knots[i+k+1] - t) / (knots[i+k+1] - knots[i+1] + 1e-16) * basis(i+1, k-1, t) \
             if knots[i+k+1] > knots[i+1] else 0.0
        return t1 + t2
    x = sum(basis(i, degree, t_val) * Px[i] for i in range(len(Px)))
    y = sum(basis(i, degree, t_val) * Py[i] for i in range(len(Py)))
    return x, y


# ---------------------------------------------------------------------------
# PRECOMPUTE PATH GEOMETRY  from phase-1 spline
# ---------------------------------------------------------------------------
def compute_path_geometry(Px, Py, N: int = 300):
    """
    Evaluate the path at N points, then compute arc-length-parameterised
    geometry: positions, heading psi(s), curvature kappa(s), and ds.

    Returns a dict with arrays all of length N (or N-1 for differences).
    """
    eps   = 1e-5
    t_raw = np.linspace(0.01, 0.99, N)

    xs, ys = [], []
    dxs, dys = [], []
    ddxs, ddys = [], []

    for ti in t_raw:
        tc = float(np.clip(ti, 0.001, 0.999))

        def pt(s):
            s = float(np.clip(s, 0.001, 0.999))
            xi, yi = bspline_curve_np(Px, Py, s)
            return xi, yi

        x0, y0   = pt(tc)
        xp, yp   = pt(tc + eps)
        xm, ym   = pt(tc - eps)
        xpp, ypp = pt(tc + 2*eps)
        xmm, ymm = pt(tc - 2*eps)

        dx  = (xp  - xm)            / (2*eps)
        dy  = (yp  - ym)            / (2*eps)
        ddx = (xp  - 2*x0 + xm)    / eps**2
        ddy = (yp  - 2*y0 + ym)    / eps**2

        xs.append(x0); ys.append(y0)
        dxs.append(dx); dys.append(dy)
        ddxs.append(ddx); ddys.append(ddy)

    xs   = np.array(xs);   ys   = np.array(ys)
    dxs  = np.array(dxs);  dys  = np.array(dys)
    ddxs = np.array(ddxs); ddys = np.array(ddys)

    # Arc-length element ds/dt (speed of the parameter)
    dsdt = np.sqrt(dxs**2 + dys**2 + 1e-12)

    # Cumulative arc length
    dt_step = t_raw[1] - t_raw[0]
    arc     = np.concatenate([[0], np.cumsum(dsdt[:-1] * dt_step)])
    S_total = arc[-1]

    # Heading psi (w.r.t. world x-axis)
    psi = np.arctan2(dys, dxs)

    # Signed curvature kappa = (x' y'' - y' x'') / |x'|^3
    # (derivative w.r.t. t; we want d psi / d s = kappa)
    kappa_t = (dxs * ddys - dys * ddxs) / (dsdt**3 + 1e-12)  # d psi / d s

    # Derivatives w.r.t. arc length s (for the dynamics)
    # dx/ds = (dx/dt) / (ds/dt)
    dxds = dxs / (dsdt + 1e-12)
    dyds = dys / (dsdt + 1e-12)

    return dict(
        t_raw=t_raw, xs=xs, ys=ys,
        psi=psi, kappa=kappa_t,
        dxds=dxds, dyds=dyds,
        arc=arc, S_total=S_total,
        dsdt=dsdt,
    )


# ---------------------------------------------------------------------------
# PHASE 2 OPTIMIZER — time-optimal speed profile
# ---------------------------------------------------------------------------
def solve_speed_profile(geo: dict, ship: Params, N_opt: int = 200,
                        v_min: float = 0.1, v_max: float = 5.0):
    """
    Optimise v(s) to minimise total traversal time.

    Substitution: w = v^2  (makes thrust constraint linear in w and w').

    Decision variables: w[i] = v[i]^2,  i = 0..N_opt-1

    Objective:  sum_i  ds / sqrt(w[i])   (trapezoidal approximation of ∫ds/v)

    Constraints per node i:
      w_min <= w[i] <= w_max
      T_req[i] <= T_max

    T_req is derived from the 3-DOF equations:
      - centripetal: F_c = m22 * w * kappa   (lateral, dominates in turns)
      - surge accel: F_s = m11 * v_dot       (along path)
      - yaw:         N   = m66 * r_dot       (from curvature change)

    We approximate v_dot[i] = (v[i+1] - v[i-1]) / (2*ds) * v[i]
    (chain rule: dv/dt = dv/ds * v, centred differences for dv/ds)
    """
    # Interpolate geometry onto N_opt uniform arc-length nodes
    arc     = geo["arc"]
    S       = geo["S_total"]
    s_nodes = np.linspace(0, S, N_opt)

    psi   = np.interp(s_nodes, arc, geo["psi"])
    kappa = np.interp(s_nodes, arc, geo["kappa"])

    # curvature derivative d kappa / d s (for yaw acceleration)
    dkappa = np.gradient(kappa, s_nodes)

    ds   = s_nodes[1] - s_nodes[0]
    p    = ship

    opti = ca.Opti()
    w    = opti.variable(N_opt)    # w[i] = v[i]^2

    w_min = v_min**2
    w_max = v_max**2

    cost = 0
    T_req_list = []

    for i in range(N_opt):
        # Boundary handling for finite differences
        if i == 0:
            dwds = (w[1] - w[0]) / ds
        elif i == N_opt - 1:
            dwds = (w[N_opt-1] - w[N_opt-2]) / ds
        else:
            dwds = (w[i+1] - w[i-1]) / (2*ds)

        v_i    = ca.sqrt(w[i] + 1e-8)
        v_dot_i = dwds / (2 * v_i + 1e-8)   # dv/dt = (dw/ds)/(2v) * v ... = dw/ds/2

        # 3-DOF required forces (in world frame, then projected)
        # surge direction: (cos psi, sin psi)
        # sway  direction: (-sin psi, cos psi)
        #
        # World-frame accelerations:
        #   x_ddot = v_dot*cos(psi) - v^2*kappa*sin(psi)
        #   y_ddot = v_dot*sin(psi) + v^2*kappa*cos(psi)
        #
        # Body-frame:
        #   u = v (surge speed along path, sway ≈ 0 for path-tracking)
        #   u_dot = v_dot
        #   v_body ≈ 0  (path-tracking assumption)
        #   r = v * kappa          (yaw rate)
        #   r_dot = v_dot*kappa + v^2 * dkappa/ds

        kap_i  = float(kappa[i])
        dkap_i = float(dkappa[i])

        u_i    = v_i
        u_dot  = v_dot_i
        r_i    = v_i * kap_i
        r_dot_i = v_dot_i * kap_i + w[i] * dkap_i

        # Required surge/sway forces (body frame)
        X_req = p.m11 * u_dot - p.m22 * 0 * r_i  + p.Xu*u_i + p.Xuu*u_i*ca.fabs(u_i)
        Y_req = p.m22 * 0     + p.m11 * u_i * r_i + p.Yv*0   + p.Yvv*0

        # Required moment (yaw)
        N_req = p.m66 * r_dot_i + p.Nr * r_i + p.Nrr * r_i * ca.fabs(r_i)

        # Azimuth thruster: T produces (X_req, Y_req) directly.
        # Moment N_req must also be satisfied; for a single thruster at stern
        # offset x_T from CoG, N = x_T * Y_req. We include it in T magnitude.
        # Here we treat the net required force magnitude:
        T_req = ca.sqrt(X_req**2 + Y_req**2 + 1e-4)
        T_req_list.append(T_req)

        # Objective: ds / v = ds / sqrt(w)
        cost += ds / v_i

        # Constraints
        opti.subject_to(w[i] >= w_min)
        opti.subject_to(w[i] <= w_max)
        opti.subject_to(T_req <= ship.T_max)

    # Start and end at rest (optional — set to v_min if you want non-zero)
    opti.subject_to(w[0]        == v_min**2)
    opti.subject_to(w[N_opt-1]  == v_min**2)

    opti.minimize(cost)
    opti.solver("ipopt",
        {"print_time": True},
        {
            "max_iter":              2000,
            "tol":                   1e-5,
            "acceptable_tol":        1e-4,
            "acceptable_iter":       15,
            "mu_strategy":           "adaptive",
            "nlp_scaling_method":    "gradient-based",
            "hessian_approximation": "limited-memory",
        }
    )

    sol  = opti.solve()
    w_sol = sol.value(w)
    v_sol = np.sqrt(np.maximum(w_sol, 0))

    # Recover derived quantities
    dw    = np.gradient(w_sol, s_nodes)
    vdot  = dw / (2 * v_sol + 1e-8)

    T_req_sol = np.zeros(N_opt)
    delta_sol = np.zeros(N_opt)
    n_sol     = np.zeros(N_opt)

    for i in range(N_opt):
        kap_i  = kappa[i]
        dkap_i = dkappa[i]

        u_dot  = vdot[i]
        r_i    = v_sol[i] * kap_i
        r_dot  = vdot[i] * kap_i + v_sol[i]**2 * dkap_i

        X_req = (p.m11 * u_dot
                 + p.Xu  * v_sol[i]
                 + p.Xuu * v_sol[i] * abs(v_sol[i]))
        Y_req = (p.m11 * v_sol[i] * r_i
                 + p.Yv  * 0
                 + p.Yvv * 0)

        T_req_sol[i] = np.sqrt(X_req**2 + Y_req**2)
        delta_sol[i] = np.degrees(np.arctan2(Y_req, X_req))

        # n from T = Kt * n * |n|  =>  |n| = sqrt(T/Kt), sign follows T
        T_i = T_req_sol[i]
        n_sol[i] = np.sign(T_i) * np.sqrt(abs(T_i) / (p.Kt + 1e-16))

    T_total = trapezoid(1.0 / (v_sol + 1e-8), s_nodes)

    print("\n=== PHASE 2 RESULTS ===")
    print(f"  Total traversal time : {T_total:.1f} s")
    print(f"  Peak speed           : {v_sol.max():.2f} m/s")
    print(f"  Mean speed           : {v_sol.mean():.2f} m/s")
    print(f"  Peak thrust          : {T_req_sol.max():.1f} N  (limit {ship.T_max:.0f} N)")
    print(f"  Peak |n|             : {np.abs(n_sol).max():.0f} rpm  "
          f"(limit {ship.n_max*30/np.pi:.0f} rpm)")
    print(f"  Arc length           : {s_nodes[-1]:.1f} m")
    print("========================\n")

    return dict(
        s=s_nodes, v=v_sol, w=w_sol,
        T_req=T_req_sol, delta=delta_sol, n=n_sol,
        T_total=T_total,
        psi=psi, kappa=kappa,
    )


# ---------------------------------------------------------------------------
# PLOTS
# ---------------------------------------------------------------------------
def plot_results(geo, result, ship, Px, Py):
    fig = plt.figure(figsize=(16, 10))
    gs  = gridspec.GridSpec(3, 3, figure=fig, hspace=0.45, wspace=0.35)

    T_max   = ship.T_max
    n_max_rpm = ship.n_max * 30 / np.pi
    s       = result["s"]
    v       = result["v"]
    T_req   = result["T_req"]
    n       = result["n"]
    delta   = result["delta"]
    kappa   = result["kappa"]

    # --- Track coloured by speed ---
    ax0 = fig.add_subplot(gs[:, 0])
    sc  = ax0.scatter(geo["xs"], geo["ys"],
                      c=np.interp(geo["arc"], s, v),
                      cmap="plasma", s=6, zorder=3)
    plt.colorbar(sc, ax=ax0, label="v [m/s]")

    from matplotlib.patches import Circle
    BUOY1 = np.array([50.0, 50.0]); BUOY2 = np.array([100.0, 100.0]); R = 10.0
    for (bx, by), col, lbl in zip([BUOY1, BUOY2],
                                   ['royalblue', 'tomato'],
                                   ['Buoy 1', 'Buoy 2']):
        ax0.add_patch(Circle((bx, by), R, color=col, fill=True, alpha=0.12))
        ax0.add_patch(Circle((bx, by), R, color=col, fill=False, lw=2))
        ax0.scatter(bx, by, c=col, s=60, zorder=5)
        ax0.annotate(lbl, (bx, by), xytext=(5, -14),
                     textcoords='offset points', fontsize=8, color=col)
    ax0.scatter(0, 0, c='black', s=100, marker='*', zorder=6)
    ax0.set_aspect('equal'); ax0.grid(True, alpha=0.3)
    ax0.set_title(f"Track — coloured by speed\nT_total = {result['T_total']:.1f} s")
    ax0.set_xlabel("x [m]"); ax0.set_ylabel("y [m]")

    # --- Speed profile ---
    ax1 = fig.add_subplot(gs[0, 1])
    ax1.plot(s, v, color='purple', lw=2)
    ax1.axhline(v.max(), color='purple', ls=':', lw=1, alpha=0.5)
    ax1.fill_between(s, v, alpha=0.15, color='purple')
    ax1.set_xlabel("arc length s [m]"); ax1.set_ylabel("v [m/s]")
    ax1.set_title("Speed profile v(s)"); ax1.grid(True, alpha=0.3)

    # --- Thrust ---
    ax2 = fig.add_subplot(gs[1, 1])
    ax2.plot(s, T_req, color='tomato', lw=2, label="|T| required")
    ax2.axhline(T_max, color='tomato', ls='--', lw=1.5, label=f"T_max = {T_max:.0f} N")
    ax2.fill_between(s, T_req, T_max,
                     where=(T_req >= T_max * 0.98),
                     alpha=0.25, color='red', label="At limit")
    ax2.set_xlabel("arc length s [m]"); ax2.set_ylabel("T [N]")
    ax2.set_title("Required thrust T(s)"); ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    # --- RPM ---
    ax3 = fig.add_subplot(gs[2, 1])
    n_rpm = n * 30 / np.pi
    ax3.plot(s, n_rpm, color='steelblue', lw=2)
    ax3.axhline( n_max_rpm, color='steelblue', ls='--', lw=1.5,
                label=f"±n_max = {n_max_rpm:.0f} rpm")
    ax3.axhline(-n_max_rpm, color='steelblue', ls='--', lw=1.5)
    ax3.fill_between(s, n_rpm, 0, alpha=0.12, color='steelblue')
    ax3.set_xlabel("arc length s [m]"); ax3.set_ylabel("n [rpm]")
    ax3.set_title("Motor RPM n(s)"); ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)

    # --- Thruster angle ---
    ax4 = fig.add_subplot(gs[0, 2])
    ax4.plot(s, delta, color='teal', lw=2)
    ax4.set_xlabel("arc length s [m]"); ax4.set_ylabel("δ [deg]")
    ax4.set_title("Thruster angle δ(s)"); ax4.grid(True, alpha=0.3)

    # --- Curvature ---
    ax5 = fig.add_subplot(gs[1, 2])
    ax5.plot(s, kappa, color='olive', lw=2)
    ax5.axhline(0, color='gray', lw=0.8)
    ax5.set_xlabel("arc length s [m]"); ax5.set_ylabel("κ [1/m]")
    ax5.set_title("Path curvature κ(s)"); ax5.grid(True, alpha=0.3)

    # --- Summary table ---
    ax6 = fig.add_subplot(gs[2, 2])
    ax6.axis('off')
    rows = [
        ["Total time",   f"{result['T_total']:.1f} s"],
        ["Arc length",   f"{s[-1]:.1f} m"],
        ["Peak speed",   f"{v.max():.2f} m/s"],
        ["Mean speed",   f"{v.mean():.2f} m/s"],
        ["Peak thrust",  f"{T_req.max():.1f} N"],
        ["T_max",        f"{T_max:.0f} N"],
        ["Peak |n|",     f"{abs(n_rpm).max():.0f} rpm"],
        ["n_max",        f"{n_max_rpm:.0f} rpm"],
        ["Thrust margin",f"{(1 - T_req.max()/T_max)*100:.1f}%"],
    ]
    tbl = ax6.table(cellText=rows,
                    colLabels=["Quantity", "Value"],
                    loc="center", cellLoc="left")
    tbl.auto_set_font_size(False); tbl.set_fontsize(9)
    tbl.scale(1, 1.4)
    ax6.set_title("Summary", pad=12)

    fig.suptitle("Phase 2 — Time-optimal speed profile (azimuth thruster)", fontsize=13)
    plt.savefig("phase2_speed_profile.png", dpi=150, bbox_inches='tight')
    print("Saved → phase2_speed_profile.png")
    plt.show()


# ---------------------------------------------------------------------------
# MAIN  — plug in your phase-1 control points here
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    # -----------------------------------------------------------------------
    # PASTE YOUR PHASE-1 CONTROL POINTS HERE
    # These are the Px, Py arrays returned by build_optimizer() in phase 1.
    # -----------------------------------------------------------------------
    # Example placeholder (replace with your actual solution):
    BUOY1  = np.array([50.0, 50.0])
    BUOY2  = np.array([100.0, 100.0])
    R      = 10.0
    ORIGIN = np.array([0.0, 0.0])
    dv     = BUOY2 - BUOY1
    ax_hat = dv / np.linalg.norm(dv)
    perp   = np.array([-ax_hat[1], ax_hat[0]])
    mid    = (BUOY1 + BUOY2) / 2.0
    D      = R + 3.0

    WAYPOINTS = np.array([
        BUOY1 + D * ax_hat - D * perp,
        mid,
        BUOY2 - D * ax_hat + D * perp,
        BUOY2              + D * perp,
        BUOY2 + D * ax_hat - D * perp,
        mid + 0.5 * perp,
        BUOY1 - D * ax_hat + D * perp,
    ])
    all_pts = np.vstack([ORIGIN, WAYPOINTS, ORIGIN])
    t_all   = np.linspace(0, 1, len(all_pts))
    t_cp    = np.linspace(0, 1, 16)
    Px = np.interp(t_cp, t_all, all_pts[:, 0])
    Py = np.interp(t_cp, t_all, all_pts[:, 1])
    # -----------------------------------------------------------------------
    # If you already have Px, Py from phase 1, just set them directly above
    # and remove the placeholder block.
    # -----------------------------------------------------------------------

    ship = Params()
    print(f"T_max = {ship.T_max:.1f} N   n_max = {ship.n_max*30/np.pi:.0f} rpm")

    print("\nComputing path geometry...")
    geo = compute_path_geometry(Px, Py, N=400)
    print(f"Arc length: {geo['S_total']:.2f} m")
    print(f"Max curvature: {np.abs(geo['kappa']).max():.4f} 1/m")

    print("\nSolving time-optimal speed profile...")
    result = solve_speed_profile(geo, ship, N_opt=200, v_min=0.05, v_max=5.0)

    print("Plotting...")
    plot_results(geo, result, ship, Px, Py)
