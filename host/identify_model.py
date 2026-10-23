"""Identify an upright linear Furuta model from identification CSV data."""
import argparse, json
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

FIELDS = ["time_s", "theta", "dtheta", "alpha", "dalpha", "pwm_applied", "pwm_exc"]

def fit(csv_path: Path, out: Path):
    df = pd.read_csv(csv_path)
    missing = set(FIELDS) - set(df.columns)
    if missing: raise ValueError(f"CSV 缺少列: {sorted(missing)}")
    df = df[FIELDS].dropna().sort_values("time_s")
    dt = np.diff(df.time_s.to_numpy())
    median_dt = float(np.median(dt))
    if not 0.005 <= median_dt <= 0.02: raise ValueError(f"采样周期异常: {median_dt:.6f}s")
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
    out.mkdir(parents=True, exist_ok=True)
    result = {"dt_s": median_dt, "samples": int(len(X0)), "A": A.tolist(), "B": B.tolist(), "c": c.tolist(), "rmse": rmse.tolist(), "poles": [[float(v.real),float(v.imag)] for v in eig], "prbs_rms": float(np.sqrt(np.mean(Z**2)))}
    (out / "identified_model.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    pd.DataFrame(A).to_csv(out / "A.csv", index=False); pd.DataFrame(B).to_csv(out / "B.csv", index=False)
    fig, axes = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
    for i, name in enumerate(["theta","dtheta","alpha","dalpha"]):
        axes[i].plot(X1[:,i], label="measured"); axes[i].plot(pred[:,i], label="one-step prediction", alpha=.7); axes[i].set_ylabel(name); axes[i].legend()
    axes[-1].set_xlabel("sample"); fig.tight_layout(); fig.savefig(out / "validation.png", dpi=150)
    print(json.dumps(result, indent=2))

if __name__ == "__main__":
    p=argparse.ArgumentParser(); p.add_argument("csv", type=Path); p.add_argument("--out", type=Path, default=Path("identification_result")); a=p.parse_args(); fit(a.csv,a.out)
