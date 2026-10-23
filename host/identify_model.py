"""Identify an upright linear Furuta model from identification CSV data."""
import argparse, json
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are
from scipy.signal import savgol_filter

FIELDS = ["time_s", "theta", "dtheta", "alpha", "alpha_ref", "dalpha", "pwm_applied", "pwm_exc"]

# LQR 权重设置位置：状态顺序 [theta, dtheta, alpha, dalpha]，输入为 PWM。
# 提高某项 Q 对角线元素会更强地抑制对应状态；提高 R 会减少 PWM 动作。
LQR_Q = np.diag([20.0, 8.0, 8.0, 2.0])
LQR_R = np.array([[1.0]])

def fit(csv_path: Path, out: Path, sample_period: float | None):
    df = pd.read_csv(csv_path)
    # Backward compatibility: old exports lacked PRBS and/or a usable time axis.
    if "pwm_applied" not in df and "pwm_duty" in df:
        df = df.rename(columns={"pwm_duty": "pwm_applied"})
    if "pwm_exc" not in df:
        df["pwm_exc"] = 0.0
    if "alpha_ref" not in df:
        df["alpha_ref"] = 0.0
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
    lag = 12
    valid = (df.theta.abs() < 0.2) & (df.pwm_applied.abs() < 950)
    continuous = np.r_[True, np.isclose(np.diff(df.time_s), median_dt, rtol=0.2)]
    segment = ((~valid) | (~continuous)).cumsum()
    phis=[]; targets=[]; instruments_all=[]; used=0
    for _, part in df[valid].groupby(segment[valid.to_numpy()]):
        if len(part) < 200: continue
        theta=part.theta.to_numpy(); alpha=(part.alpha-part.alpha_ref).to_numpy()
        dtheta=savgol_filter(theta, 21, 3, deriv=1, delta=median_dt)
        dalpha=savgol_filter(alpha, 21, 3, deriv=1, delta=median_dt)
        x=np.c_[theta,dtheta,alpha,dalpha]; u=part.pwm_applied.to_numpy(); z=part.pwm_exc.to_numpy()-part.pwm_exc.mean()
        phis.append(np.c_[x[lag:-1],u[lag:-1],np.ones(len(x)-lag-1)])
        targets.append(x[lag+1:])
        instruments_all.append(np.c_[np.ones(len(x)-lag-1),*[z[lag-i:-1-i] for i in range(lag)]])
        used += len(x)-lag-1
    if not phis: raise ValueError("没有长度至少 200 点的连续有效直立片段")
    phi=np.vstack(phis); X1=np.vstack(targets); instruments=np.vstack(instruments_all)
    normal = instruments.T @ phi
    condition = np.linalg.cond(normal)
    if np.linalg.matrix_rank(normal) < phi.shape[1] or condition > 1e4:
        raise ValueError(f"PRBS 激励秩不足或病态（condition={condition:.2e}）；请采集至少 60 秒、±20 PWM 的连续有效辨识数据")
    coef = np.linalg.pinv(normal) @ instruments.T @ X1
    A, B, c = coef[:4].T, coef[4:5].T, coef[5]
    pred = phi[:,:4] @ A.T + phi[:,4:5] @ B.T + c
    rmse = np.sqrt(np.mean((X1 - pred) ** 2, axis=0))
    eig = np.linalg.eigvals(A)
    P = solve_discrete_are(A, B, LQR_Q, LQR_R)
    K = np.linalg.solve(B.T @ P @ B + LQR_R, B.T @ P @ A)
    K_q10 = np.rint(K * 1024).astype(int)
    if np.any(np.abs(K_q10) > 131071): raise ValueError("识别结果产生超出 FPGA 18 位 Q10 范围的增益，拒绝输出 K")
    out.mkdir(parents=True, exist_ok=True)
    result = {"dt_s": median_dt, "samples": int(used), "method": "closed_loop_iv_prbs_lag12_segmented", "iv_condition": float(condition), "A": A.tolist(), "B": B.tolist(), "c": c.tolist(), "rmse": rmse.tolist(), "poles": [[float(v.real),float(v.imag)] for v in eig], "Q": LQR_Q.tolist(), "R": LQR_R.tolist(), "K": K.tolist(), "K_q10": K_q10.tolist()}
    (out / "identified_model.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    pd.DataFrame(A).to_csv(out / "A.csv", index=False); pd.DataFrame(B).to_csv(out / "B.csv", index=False); pd.DataFrame(K).to_csv(out / "K.csv", index=False)
    fig, axes = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
    for i, name in enumerate(["theta","dtheta","alpha","dalpha"]):
        axes[i].plot(X1[:,i], label="measured"); axes[i].plot(pred[:,i], label="one-step prediction", alpha=.7); axes[i].set_ylabel(name); axes[i].legend()
    axes[-1].set_xlabel("sample"); fig.tight_layout(); fig.savefig(out / "validation.png", dpi=150)
    print(json.dumps(result, indent=2))

if __name__ == "__main__":
    p=argparse.ArgumentParser(); p.add_argument("csv", type=Path); p.add_argument("--out", type=Path, default=Path("identification_result")); p.add_argument("--sample-period", type=float); a=p.parse_args(); fit(a.csv,a.out,a.sample_period)
