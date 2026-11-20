"""
Headless analysis of the robot. Run:

    python analyze.py                 full report + plots in ./report/
    python analyze.py --shell         same with the 3D-printed shell on
    python analyze.py --tune          search for robust gains (takes ~2-4 min)

What it does
  1. mass budget, COM, motor curves, open-loop instability
  2. LQR gains (theory) vs the tuned firmware gains
  3. scenario tests: stand still, push recovery limit, drive, turn,
     battery runtime, sensitivity to backlash / loop rate / mounting error
"""
import argparse
import os
import random
import statistics as st
from dataclasses import replace, asdict
from math import pi

from params import make_params
from sim import Sim
from controller import Gains
from lqr import lqr_gains

def tuned_gains(**kw):
    return replace(Gains(), **kw)


# ---------------------------------------------------------------- scenarios
def scenario_stand(p, g, seconds=8.0):
    s = Sim(p=p, gains=g, log_seconds=seconds + 1)
    s.advance(seconds)
    th = list(s.log["th"])[-int(4 * 200):]
    pw = list(s.log["pwmL"])[-int(4 * 200):]
    return dict(fallen=s.fallen, th_std=st.pstdev(th), pwm_std=st.pstdev(pw),
                drift=abs(s.plant.x[0]), sim=s)


def scenario_push(p, g, impulse, settle=1.5, after=2.5):
    s = Sim(p=p, gains=g, log_seconds=settle + after + 1)
    s.advance(settle)
    s.push(impulse, 0.05, height=p.l + 0.04)
    s.advance(after)
    return not s.fallen, s


def max_push(p, g, lo=0.0, hi=0.4, iters=9):
    """Largest forward impulse (N s) at the head the robot survives."""
    ok_lo = lo
    for _ in range(iters):
        mid = 0.5 * (ok_lo + hi)
        ok, _ = scenario_push(p, g, mid)
        if ok:
            ok_lo = mid
        else:
            hi = mid
    return ok_lo


def scenario_drive(p, g, v=0.3, seconds=3.0):
    s = Sim(p=p, gains=g, log_seconds=12)
    s.advance(1.0)
    s.v_cmd = v
    s.advance(seconds)
    vs = list(s.log["v"])[-200:]
    s.v_cmd = 0.0
    s.advance(3.0)
    stop_x = s.plant.x[0]
    return dict(fallen=s.fallen, v_mean=st.mean(vs), v_std=st.pstdev(vs), sim=s,
                overshoot=stop_x)


def scenario_turn(p, g, rate=90.0, seconds=4.0):
    s = Sim(p=p, gains=g, log_seconds=10)
    s.advance(1.0)
    s.yaw_cmd = rate
    s.advance(seconds)
    from physics import IPSID
    return dict(fallen=s.fallen, yaw_rate=s.plant.x[IPSID] * 180 / pi, sim=s)


def cost(p, g):
    c = 0.0
    a = scenario_stand(p, g, 6.0)
    if a["fallen"]:
        return 1e6
    c += 20 * a["th_std"] + 0.05 * a["pwm_std"] + 2 * a["drift"]
    for imp, pen in ((0.025, 40), (0.035, 25), (0.045, 10)):
        ok, s = scenario_push(p, g, imp)
        if not ok:
            c += pen
    d = scenario_drive(p, g, 0.3, 2.5)
    if d["fallen"]:
        return 1e5 + c
    c += 30 * abs(d["v_mean"] - 0.3) + 10 * d["v_std"]
    t = scenario_turn(p, g, 90.0, 2.5)
    if t["fallen"]:
        return 1e5 + c
    c += 0.05 * abs(t["yaw_rate"] - 90)
    return c


def tune(shell=False, iters=60, seed=3):
    rng = random.Random(seed)
    p = make_params(shell=shell)
    best = tuned_gains()
    best_c = cost(p, best)
    print(f"start cost {best_c:.2f}  {asdict(best)}")
    keys = dict(kp=(10, 90), kd=(0.15, 2.0), kp_v=(2, 40), ki_v=(0.3, 20), deadband=(0, 12))
    for i in range(iters):
        cand = {}
        for k, (lo, hi) in keys.items():
            v = getattr(best, k) * (1 + rng.gauss(0, 0.25))
            cand[k] = round(min(hi, max(lo, v)), 2)
        g = replace(best, **cand)
        c = cost(p, g)
        if c < best_c:
            best, best_c = g, c
            print(f"[{i:3d}] cost {c:.2f}  " + "  ".join(f"{k}={getattr(g, k)}" for k in keys))
    return best


# ---------------------------------------------------------------- report
def motor_table(p):
    mo = p.motor
    Rt = mo.R + p.power.drv_rds_on
    rows = []
    for vb in (8.4, 7.4, 6.6):
        for cap in (1.0, 0.8):
            V = vb * cap
            w_nl = (V - 0) / (mo.Ke * mo.gear)        # ignore friction for the table
            I_st = V / Rt
            T_st = mo.eta * mo.gear * mo.Kt * I_st - mo.T_coulomb_out
            rows.append((vb, cap, w_nl * 60 / 2 / pi, w_nl * p.r, T_st / 0.0980665, I_st))
    return rows


def plot_runs(runs, path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
    for label, s in runs:
        L = s.log
        t = list(L["t"])
        ax[0].plot(t, L["th"], label=f"{label} true")
        ax[0].plot(t, L["est"], "--", lw=0.8, label=f"{label} estimate")
        ax[1].plot(t, L["pwmL"], lw=0.8, label=label)
        ax[2].plot(t, L["v"], lw=0.9, label=label)
        ax[3].plot(t, L["x"], lw=0.9, label=label)
    ax[0].set_ylabel("pitch [deg]")
    ax[1].set_ylabel("PWM L [counts]")
    ax[2].set_ylabel("speed [m/s]")
    ax[3].set_ylabel("position [m]")
    ax[3].set_xlabel("time [s]")
    for a in ax:
        a.grid(alpha=0.3)
        a.legend(fontsize=7, loc="upper right")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def report(shell=False, outdir="report"):
    os.makedirs(outdir, exist_ok=True)
    p = make_params(shell=shell)
    g = tuned_gains()
    out = []
    P = out.append
    P(f"=== ESP32 balancing robot -- {'WITH' if shell else 'without'} shell ===\n")
    P(p.summary())
    P("\n-- Motor on your battery (GA12-N20 6 V 300 rpm, 34 mm wheel) --")
    P(" Vbat  PWM cap  no-load rpm  top speed m/s  stall kg.cm  stall A")
    for vb, cap, rpm, vmax, T, I in motor_table(p):
        P(f" {vb:4.1f}   {cap*100:4.0f}%     {rpm:6.0f}        {vmax:5.2f}        {T:5.2f}     {I:4.2f}")
    K, casc, poles, ol = lqr_gains(p)
    P(f"\n-- Open loop: unstable pole {max(ol.real):.1f} 1/s "
      f"(falls with time constant {1000/max(ol.real):.0f} ms)")
    P("-- LQR (ideal, no backlash/noise) cascade gains: " +
      ", ".join(f"{k}={v:.2f}" for k, v in casc.items()))
    P("-- Firmware gains used (tuned in this sim with backlash, noise, delays): " +
      ", ".join(f"{k}={getattr(g, k)}" for k in ("kp", "kd", "ki", "kp_v", "ki_v", "kp_t", "ki_t", "deadband")))

    a = scenario_stand(p, g)
    P(f"\n[stand 8 s]   fallen={a['fallen']}  wobble {a['th_std']:.2f} deg rms  "
      f"PWM jitter {a['pwm_std']:.1f}  drift {a['drift']*100:.1f} cm  "
      f"peak traction use {a['sim'].max_traction*100:.0f}% of grip")
    mp = max_push(p, g)
    P(f"[push]        survives a head push up to {mp:.3f} N s "
      f"(~ {mp/p.total_mass:.2f} m/s kick to the whole robot)")
    _, s_push = scenario_push(p, g, mp * 0.8)
    d = scenario_drive(p, g, 0.3)
    P(f"[drive 0.3]   fallen={d['fallen']}  reached {d['v_mean']:.2f} m/s (+/- {d['v_std']:.2f})")
    d2 = scenario_drive(p, g, 0.5)
    P(f"[drive 0.5]   fallen={d2['fallen']}  reached {d2['v_mean']:.2f} m/s")
    tr = scenario_turn(p, g)
    P(f"[turn 90/s]   fallen={tr['fallen']}  actual yaw rate {tr['yaw_rate']:.0f} deg/s")

    # battery runtime (standing) from average current
    s = a["sim"]
    ib = st.mean(list(s.log["ibat"])[-800:])
    P(f"[battery]     standing draw {ib*1000:.0f} mA -> ~{p.power.capacity_mAh*0.8/ (ib*1000)/1:.1f} h "
      f"to 80 % depth of discharge")

    # sensitivity
    P("\n-- Sensitivity (stand 8 s with tuned gains) --")
    tests = [
        ("backlash 0 deg", {"motor.backlash_deg": 0.0}),
        ("backlash 3 deg", {"motor.backlash_deg": 3.0}),
        ("loop 100 Hz", {"timing.control_hz": 100.0}),
        ("loop 500 Hz", {"timing.control_hz": 500.0}),
        ("MPU tilted 3 deg", {"sensor.mount_error_deg": 3.0}),
        ("MPU on top deck (z=60mm)", {"sensor.mpu_pos": (0.0, 0.0, 0.060)}),
        ("+50 g payload on top", {"extra_payload_g": 50.0}),
        ("battery at 6.8 V (soc 15%)", {"power.soc0": 0.15}),
    ]
    for label, ov in tests:
        pp = make_params(shell=shell, **ov)
        r_ = scenario_stand(pp, g)
        P(f"  {label:28s} fallen={str(r_['fallen']):5s} wobble {r_['th_std']:.2f} deg  "
          f"PWM jitter {r_['pwm_std']:5.1f}  drift {r_['drift']*100:5.1f} cm")

    text = "\n".join(out)
    print(text)
    with open(os.path.join(outdir, f"report{'_shell' if shell else ''}.txt"), "w") as f:
        f.write(text + "\n")
    tag = "_shell" if shell else ""
    plot_runs([("stand", a["sim"])], os.path.join(outdir, f"stand{tag}.png"), "Standing still (release at 3 deg)")
    plot_runs([("push", s_push)], os.path.join(outdir, f"push{tag}.png"), f"Push of {mp*0.8:.3f} N s at t=1.5 s")
    plot_runs([("0.3 m/s", d["sim"])], os.path.join(outdir, f"drive{tag}.png"), "Drive 0.3 m/s for 3 s, then stop")
    print(f"\nplots + text written to ./{outdir}/")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--shell", action="store_true")
    ap.add_argument("--tune", action="store_true")
    ap.add_argument("--iters", type=int, default=60)
    a = ap.parse_args()
    if a.tune:
        best = tune(a.shell, a.iters)
        print("\nBEST:", {k: getattr(best, k) for k in ("kp", "kd", "kp_v", "ki_v", "deadband")})
    else:
        report(a.shell)
