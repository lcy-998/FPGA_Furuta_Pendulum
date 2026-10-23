"""Identify an upright linear Furuta model from identification CSV data."""
import argparse, json
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are

FIELDS = ["time_s", "theta", "dtheta", "alpha", "dalpha", "pwm_applied", "pwm_exc"]

# LQR 权重设置位置：状态顺序 [theta, dtheta, alpha, dalpha]，输入为 PWM。
# 提高某项 Q 对角线元素会更强地抑制对应状态；提高 R 会减少 PWM 动作。
LQR_Q = np.diag([250.0, 8.0, 20.0, 2.0])
LQR_R = np.array([[1.0]])

def fit(csv_path: Path, out: Path, sample_period: float | None):
    df = pd.read_csv(csv_path)
    # Backward compatibility: old exports lacked PRBS and/or a usable time axis.
    if "pwm_applied" not in df and "pwm_duty" in df:
        df = df.rename(columns={"pwm_duty": "pwm_applied"})
    if "pwm_exc" not in df:
        df["pwm_exc"] = 0.0
    if "time_s" not in df:
        df["time_s"] = np.arange(len(df)) * (sample_period or 0.01)
    missing = set(FIELDS) - set(df.columns)
    if missing: raise ValueError(f"CSV 缺少列: {sorted(missing)}")
    df = df[FIELDS].dropna().sort_values("time_s")
    dt = np.diff(df.time_s.to_numpy())
    median_dt = float(np.median(dt))
    if not 0.005 <= median_dt <= 0.02:
        median_dt = sample_period or 0.01
        print(f"警告：旧 CSV 的时间戳间隔为 {float(np.median(dt)):.6f}s，按 {median_dt:.3f}s 重建时间轴")
        df["time_s"] = np.arange(len(df)) * median_dt
        dt = np.diff(df.time_s.to_numpy())
    good = np.r_[True, np.isclose(dt, median_dt, rtol=0.2)]
    df = df.loc[good]
    df = df[(df.theta.abs() < 0.2) & (df.pwm_applied.abs() < 950)]
    if len(df) < 200: raise ValueError("有效直立小扰动样本不足 200 条")
    x = df[["theta","dtheta","alpha","dalpha"]].to_numpy()
    u = df[["pwm_applied"]].to_numpy()
    z = df[["pwm_exc"]].to_numpy()
    X0, X1, U, Z = x[:-1], x[1:], u[:-1], z[:-1]
    # Ridge OLS initial estimate.  PRBS is independent and reported for diagnostics.
    phi = np.c_[X0, U, np.ones(len(X0))]
    ridge = 1e-6 * np.eye(phi.shape[1])
    coef = np.linalg.solve(phi.T @ phi + ridge, phi.T @ X1)
    A, B, c = coef[:4].T, coef[4:5].T, coef[5]
    pred = X0 @ A.T + U @ B.T + c
    rmse = np.sqrt(np.mean((X1 - pred) ** 2, axis=0))
    eig = np.linalg.eigvals(A)
    P = solve_discrete_are(A, B, LQR_Q, LQR_R)
    K = np.linalg.solve(B.T @ P @ B + LQR_R, B.T @ P @ A)
    K_q10 = np.rint(K * 1024).astype(int)
    out.mkdir(parents=True, exist_ok=True)
    result = {"dt_s": median_dt, "samples": int(len(X0)), "A": A.tolist(), "B": B.tolist(), "c": c.tolist(), "rmse": rmse.tolist(), "poles": [[float(v.real),float(v.imag)] for v in eig], "prbs_rms": float(np.sqrt(np.mean(Z**2))), "Q": LQR_Q.tolist(), "R": LQR_R.tolist(), "K": K.tolist(), "K_q10": K_q10.tolist()}
    (out / "identified_model.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    pd.DataFrame(A).to_csv(out / "A.csv", index=False); pd.DataFrame(B).to_csv(out / "B.csv", index=False); pd.DataFrame(K).to_csv(out / "K.csv", index=False)
    fig, axes = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
    for i, name in enumerate(["theta","dtheta","alpha","dalpha"]):
        axes[i].plot(X1[:,i], label="measured"); axes[i].plot(pred[:,i], label="one-step prediction", alpha=.7); axes[i].set_ylabel(name); axes[i].legend()
    axes[-1].set_xlabel("sample"); fig.tight_layout(); fig.savefig(out / "validation.png", dpi=150)
    print(json.dumps(result, indent=2))

if __name__ == "__main__":
    p=argparse.ArgumentParser(); p.add_argument("csv", type=Path); p.add_argument("--out", type=Path, default=Path("identification_result")); p.add_argument("--sample-period", type=float); a=p.parse_args(); fit(a.csv,a.out,a.sample_period)
