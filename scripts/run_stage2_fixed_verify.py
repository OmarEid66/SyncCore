"""
Stage 2 Bit-Accurate Fixed-Point Verification Runner
(1-to-1 bit-exact integer mirror of matlab/fixed/*.m).

Compares Stage 2 Fixed-Point vs Stage 1 Floating-Point across all 7 required metrics:
  1. Interpolation error (open-loop & closed-loop)
  2. Gardner TED error
  3. PI controller output error
  4. Fractional-delay (mu) error
  5. Symbol output error (Q1.15 vs float64)
  6. Acquisition-time difference
  7. RMS timing jitter difference
Also sweeps W_mu in {8, 10, 12} bits and verifies 8-clock RTL pipeline delay.
"""
import os
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_stage1_float_verify as fl


def fx_round_shift(x, sh):
    if sh <= 0:
        return x << (-sh)
    return (x + (1 << (sh - 1))) >> sh


def fx_sat(x, lo, hi):
    if isinstance(x, np.ndarray):
        return np.clip(x, lo, hi)
    return lo if x < lo else (hi if x > hi else x)


def fx_div6(x):
    t1 = x + (x << 2)       # * 5
    t2 = t1 + (t1 << 4)     # * 85
    t3 = t2 + (t2 << 8)     # * 21845
    return fx_round_shift(t3, 17)


def farrow_cubic_fixed(xm1_q11, x0_q11, x1_q11, x2_q11, mu_fx, w_mu=12):
    sq_min, sq_max = -131072, 131071
    q15_min, q15_max = -32768, 32767

    xm1 = xm1_q11 << 4
    x0  = x0_q11  << 4
    x1  = x1_q11  << 4
    x2  = x2_q11  << 4

    c0 = fx_sat(x0, sq_min, sq_max)
    c1 = fx_sat(x1 - (x0 >> 1) - fx_div6((xm1 << 1) + x2), sq_min, sq_max)
    c2 = fx_sat(((xm1 + x1) >> 1) - x0, sq_min, sq_max)
    c3 = fx_sat(fx_div6(x2 - xm1) + ((x0 - x1) >> 1), sq_min, sq_max)

    h2    = fx_sat(c2 + fx_round_shift(c3 * mu_fx, w_mu), sq_min, sq_max)
    h1    = fx_sat(c1 + fx_round_shift(h2 * mu_fx, w_mu), sq_min, sq_max)
    y_raw = fx_sat(c0 + fx_round_shift(h1 * mu_fx, w_mu), sq_min, sq_max)
    return fx_sat(y_raw, q15_min, q15_max)


def gardner_ted_fixed(i_curr, q_curr, i_prev, q_prev, i_mid, q_mid):
    di = i_curr - i_prev
    dq = q_curr - q_prev
    sum_iq = di * i_mid + dq * q_mid
    return fx_sat(fx_round_shift(sum_iq, 15), -131072, 131071)


def pi_controller_fixed(e_sq3_15, integ_prev, kp4_fx, ki_fx):
    integ_next = fx_sat(integ_prev + e_sq3_15, -8388608, 8388607)
    u_p = fx_sat(fx_round_shift(e_sq3_15 * kp4_fx, 9), -1048576, 1048576)
    u_i = fx_sat(fx_round_shift(integ_next * ki_fx, 14), -1048576, 1048576)
    return u_p, u_i, integ_next


def str_fixed_model(rx_i_q11, rx_q_q11, in_valid, kp4_fx, ki_fx, w_mu=12, init_eta_q22=2097152, loop_delay_clks=0):
    W_ETA = 22
    ETA_MOD = 1 << W_ETA
    W_NOM = 1 << (W_ETA - 1)
    W_MIN = 1 << (W_ETA - 2)
    W_MAX = 3 << (W_ETA - 2)
    MU_MAX = (1 << w_mu) - 1
    MU_SHIFT = W_ETA - w_mu

    n = len(rx_i_q11)
    out_i, out_q, sym_idx, sym_mu = [], [], [], []
    ted_log, up_log, ui_log, integ_log = [], [], [], []
    eta_trace = np.zeros(n, dtype=np.int64)

    sr_i = [0, 0, 0, 0]
    sr_q = [0, 0, 0, 0]
    valid_count = 0

    eta = int(init_eta_q22)
    is_sym = False
    has_mid = False
    has_prev = False
    mid_i = mid_q = prev_i = prev_q = 0
    pi_integ = 0
    u_i_curr = 0
    kick_reg = 0
    kick_pending = False

    pipe_p = [0] * loop_delay_clks
    pipe_i = [0] * loop_delay_clks
    pipe_v = [False] * loop_delay_clks

    for idx in range(n):
        if not in_valid[idx]:
            eta_trace[idx] = eta
            continue

        in_i = fx_sat(int(rx_i_q11[idx]), -2048, 2047)
        in_q = fx_sat(int(rx_q_q11[idx]), -2048, 2047)
        sr_i = [in_i, sr_i[0], sr_i[1], sr_i[2]]
        sr_q = [in_q, sr_q[0], sr_q[1], sr_q[2]]
        valid_count += 1

        if valid_count < 4:
            eta_trace[idx] = eta
            continue

        if loop_delay_clks > 0:
            if pipe_v[-1]:
                kick_reg = pipe_p[-1]
                kick_pending = True
                u_i_curr = pipe_i[-1]
            pipe_p = [0] + pipe_p[:-1]
            pipe_i = [u_i_curr] + pipe_i[:-1]
            pipe_v = [False] + pipe_v[:-1]

        w_freq = fx_sat(W_NOM + u_i_curr, W_MIN, W_MAX)
        if is_sym and kick_pending:
            kick = kick_reg
            kick_pending = False
        else:
            kick = 0

        eta_k = eta - kick
        eta_sub = eta_k - w_freq
        eta_trace[idx] = eta

        if eta_sub < 0:
            eta_2x = eta_k << 1
            mu_fx = fx_sat(fx_round_shift(eta_2x, MU_SHIFT), 0, MU_MAX)
            eta = fx_sat(eta_sub + ETA_MOD, 0, ETA_MOD - 1)

            y_i = farrow_cubic_fixed(sr_i[3], sr_i[2], sr_i[1], sr_i[0], mu_fx, w_mu)
            y_q = farrow_cubic_fixed(sr_q[3], sr_q[2], sr_q[1], sr_q[0], mu_fx, w_mu)

            if not is_sym:
                mid_i, mid_q = y_i, y_q
                has_mid = True
                is_sym = True
            else:
                out_i.append(y_i)
                out_q.append(y_q)
                sym_idx.append(idx)
                sym_mu.append(mu_fx)

                if has_prev and has_mid:
                    e_fx = gardner_ted_fixed(y_i, y_q, prev_i, prev_q, mid_i, mid_q)
                    u_p_fx, u_i_fx, pi_integ = pi_controller_fixed(e_fx, pi_integ, kp4_fx, ki_fx)
                    if loop_delay_clks > 0:
                        pipe_p[0] = u_p_fx
                        pipe_i[0] = u_i_fx
                        pipe_v[0] = True
                    else:
                        kick_reg = u_p_fx
                        kick_pending = True
                        u_i_curr = u_i_fx
                else:
                    e_fx = 0
                    u_p_fx = 0
                    u_i_fx = u_i_curr

                ted_log.append(e_fx)
                up_log.append(u_p_fx)
                ui_log.append(u_i_fx)
                integ_log.append(pi_integ)
                prev_i, prev_q = y_i, y_q
                has_prev = True
                is_sym = False
        else:
            eta = fx_sat(eta_sub, 0, ETA_MOD - 1)

    oi = np.array(out_i, dtype=np.int64)
    oq = np.array(out_q, dtype=np.int64)
    sm = np.array(sym_mu, dtype=np.int64)
    te = np.array(ted_log, dtype=np.int64)
    up = np.array(up_log, dtype=np.int64)
    ui = np.array(ui_log, dtype=np.int64)
    return {
        "out_sym_i_q15": oi,
        "out_sym_q_q15": oq,
        "out_sym_float": (oi + 1j * oq) / 32768.0,
        "sym_sample_idx": np.array(sym_idx, dtype=np.float64),
        "sym_mu_fx": sm,
        "sym_mu_float": sm / float(1 << w_mu),
        "ted_err_q15": te,
        "ted_err_float": te / 32768.0,
        "u_p_q22": up,
        "u_i_q22": ui,
        "pi_out_float": (up / 4.0 + ui) / 4194304.0,
        "pi_integ_q15": np.array(integ_log, dtype=np.int64),
        "eta_trace_q22": eta_trace,
    }


def compute_metrics(sig, sym_idx, sym_mu, out_sym):
    eps_clk = sig["clk_ppm"] * 1e-6
    ns = sig["sps"]
    l_mf = sig["mf_delay_samp"]
    t_eff = (((sym_idx - 2.0) + sym_mu) - l_mf) / ns * (1.0 + eps_clk) + sig["timing_offset_sym"]
    k_ideal = np.round(t_eff).astype(np.int64)
    tau_err = t_eff - k_ideal
    valid_int = (k_ideal >= sig["rrc_span"]) & (k_ideal <= sig["num_symbols"] - sig["rrc_span"] - 1)
    tau_v = tau_err[valid_int]
    sym_v = out_sym[valid_int]
    ref_v = sig["tx_symbols"][k_ideal[valid_int]]

    acq = len(tau_v)
    for k in range(len(tau_v) - 20):
        if np.all(np.abs(tau_v[k:k + 20]) <= 0.05):
            acq = k + 1
            break
    ss = min(300, len(tau_v) // 2)
    return {
        "tau_err_ts": tau_v,
        "acq_syms": acq,
        "mean_err_ts": float(np.mean(tau_v[ss:])),
        "rms_jitter_ts": float(np.sqrt(np.mean(tau_v[ss:] ** 2))),
        "evm_pct": float(100.0 * np.sqrt(np.mean(np.abs(sym_v[ss:] - ref_v[ss:]) ** 2) / np.mean(np.abs(ref_v[ss:]) ** 2))),
    }


def main():
    out_dir = os.path.join("matlab", "plots", "stage2")
    os.makedirs(out_dir, exist_ok=True)

    alpha = 0.35
    ns = 4
    sig_scale = 0.5
    kd_scaled = 1.0697 * (sig_scale ** 2)
    lp = fl.design_pi_gains(0.0080, 0.707, kd_scaled, 2.0, 0)
    kp_fl, ki_fl = lp["kp"], lp["ki"]
    kp4_fx = int(round(ns * kp_fl * 65536))
    ki_fx = int(round(ki_fl * 2097152))

    print("=" * 80)
    print("  STAGE 2: BIT-ACCURATE FIXED-POINT MODEL & STAGE 1 VS STAGE 2 AUDIT")
    print("=" * 80)
    print(f"[1] Loop Gains (sig_scale={sig_scale}, Bn*Ts=0.0080, zeta=0.707):")
    print(f"    4*Kp float = {ns*kp_fl:.6f} -> kp4_fx = {kp4_fx:5d} (Q1.16 = {kp4_fx/65536:.6f}, rel err = {100*abs(kp4_fx/65536 - ns*kp_fl)/(ns*kp_fl):.4f}%)")
    print(f"    Ki   float = {ki_fl:.6f} -> ki_fx  = {ki_fx:5d} (2^-21 = {ki_fx/2097152:.6f}, rel err = {100*abs(ki_fx/2097152 - ki_fl)/ki_fl:.4f}%)\n")

    # Open-loop Q1.11 headroom & Farrow sweep over W_mu = 8, 10, 12
    sig_ol = fl.generate_str_signal("16QAM", 1000, ns, alpha, 12, 15.0, 0.35, 0.0, sig_scale, 77)
    rxi = sig_ol["rx_mf"].real
    rxq = sig_ol["rx_mf"].imag
    rxi_q11 = np.clip(np.rint(rxi * 2048.0).astype(np.int64), -2048, 2047)
    rxq_q11 = np.clip(np.rint(rxq * 2048.0).astype(np.int64), -2048, 2047)
    clip_pct = 100.0 * np.mean((np.abs(rxi) >= 1.0) | (np.abs(rxq) >= 1.0))
    print(f"[2] Input Q1.11 Headroom (16-QAM, SNR=15 dB, sig_scale={sig_scale}):")
    print(f"    Max |I/Q| = {max(np.max(np.abs(rxi)), np.max(np.abs(rxq))):.4f} | Clipping rate = {clip_pct:.4f}%\n")

    idx_t = np.arange(sig_ol["mf_delay_samp"] + 20, len(rxi) - sig_ol["mf_delay_samp"] - 20)
    mu_test_fl = 0.3731
    print("[3] Open-Loop Cubic Farrow Interpolation Error vs W_mu (16-QAM, Q1.11 -> Q1.15):")
    for w_mu in [8, 10, 12]:
        mu_test_fx = int(round(mu_test_fl * (1 << w_mu)))
        yi_fl_q = fl.farrow_cubic_float(rxi_q11[idx_t-1]/2048.0, rxi_q11[idx_t]/2048.0, rxi_q11[idx_t+1]/2048.0, rxi_q11[idx_t+2]/2048.0, mu_test_fl)
        yi_fx = np.array([farrow_cubic_fixed(int(rxi_q11[i-1]), int(rxi_q11[i]), int(rxi_q11[i+1]), int(rxi_q11[i+2]), mu_test_fx, w_mu) for i in idx_t]) / 32768.0
        yi_fl_full = fl.farrow_cubic_float(rxi[idx_t-1], rxi[idx_t], rxi[idx_t+1], rxi[idx_t+2], mu_test_fl)
        ea = yi_fx - yi_fl_q
        et = yi_fx - yi_fl_full
        sqnr = 20.0 * np.log10(np.sqrt(np.mean(yi_fl_full**2)) / np.sqrt(np.mean(et**2)))
        print(f"    W_mu={w_mu:2d} bits: Farrow arith RMS={np.sqrt(np.mean(ea**2)):.3e} (max={np.max(np.abs(ea)):.3e}) | Total RMS={np.sqrt(np.mean(et**2)):.3e} (SQNR={sqnr:.2f} dB)")

    # Closed-loop W_mu comparison (8, 10, 12) on 16-QAM 15 dB
    print("\n[4] Closed-Loop Fractional Delay Precision Sweep (16-QAM, SNR=15 dB, tau0=+0.35 Ts):")
    sig_w = fl.generate_str_signal("16QAM", 2500, ns, alpha, 12, 15.0, 0.35, 0.0, sig_scale, 43)
    skip = 48
    rxt = sig_w["rx_mf"][skip:]
    iv = np.ones(len(rxt), dtype=bool)
    ii_q11 = np.clip(np.rint(rxt.real * 2048.0).astype(np.int64), -2048, 2047)
    iq_q11 = np.clip(np.rint(rxt.imag * 2048.0).astype(np.int64), -2048, 2047)
    for w_mu in [8, 10, 12]:
        rfx = str_fixed_model(ii_q11, iq_q11, iv, kp4_fx, ki_fx, w_mu=w_mu)
        rfx["sym_sample_idx"] += skip
        mfx = compute_metrics(sig_w, rfx["sym_sample_idx"], rfx["sym_mu_float"], rfx["out_sym_float"])
        print(f"    W_mu={w_mu:2d} bits: Acq={mfx['acq_syms']:3d} sym | Mean={mfx['mean_err_ts']:+.5f} Ts | RMS={mfx['rms_jitter_ts']:.5f} Ts | EVM={mfx['evm_pct']:.2f}%")

    # Full Closed-Loop Float vs Fixed Comparison (W_mu = 12)
    w_mu_sel = 12
    cases = [
        ("QPSK  15dB   0ppm", dict(mod_type="QPSK",  num_symbols=2500, snr_db=15.0, timing_offset_sym=0.35,  clk_ppm=0.0,    sig_scale=sig_scale, seed=42), 0),
        ("16QAM 15dB   0ppm", dict(mod_type="16QAM", num_symbols=2500, snr_db=15.0, timing_offset_sym=0.35,  clk_ppm=0.0,    sig_scale=sig_scale, seed=43), 0),
        ("QPSK  15dB +200ppm",dict(mod_type="QPSK",  num_symbols=2500, snr_db=15.0, timing_offset_sym=0.35,  clk_ppm=200.0,  sig_scale=sig_scale, seed=44), 0),
        ("16QAM 15dB -200ppm",dict(mod_type="16QAM", num_symbols=2500, snr_db=15.0, timing_offset_sym=-0.35, clk_ppm=-200.0, sig_scale=sig_scale, seed=45), 0),
        ("QPSK  Inf    0ppm", dict(mod_type="QPSK",  num_symbols=2500, snr_db=np.inf, timing_offset_sym=0.35, clk_ppm=0.0,   sig_scale=sig_scale, seed=46), 0),
        ("16QAM Inf    0ppm", dict(mod_type="16QAM", num_symbols=2500, snr_db=np.inf, timing_offset_sym=0.35, clk_ppm=0.0,   sig_scale=sig_scale, seed=47), 0),
        ("QPSK  15dB D=8clk", dict(mod_type="QPSK",  num_symbols=2500, snr_db=15.0, timing_offset_sym=0.35,  clk_ppm=0.0,    sig_scale=sig_scale, seed=42), 8),
        ("16QAM 15dB D=8clk", dict(mod_type="16QAM", num_symbols=2500, snr_db=15.0, timing_offset_sym=0.35,  clk_ppm=0.0,    sig_scale=sig_scale, seed=43), 8),
    ]

    print(f"\n[5] Closed-Loop Stage 1 (Float) vs Stage 2 (Fixed, W_mu={w_mu_sel}) Comparison:")
    print(f"    {'Test Case':18s} | {'Fl/Fx Acq':10s} | {'Fl/Fx RMS (Ts)':17s} | {'Sym RMS':9s} | {'TED RMS':9s} | {'PI RMS':9s} | {'mu RMS':9s}")
    print("    " + "-" * 100)

    saved = {}
    for lbl, kw, dclk in cases:
        sig = fl.generate_str_signal(**kw)
        rx_t = sig["rx_mf"][skip:]
        iv = np.ones(len(rx_t), dtype=bool)
        r_fl = fl.str_float_model(rx_t.real, rx_t.imag, iv, kp_fl, ki_fl, sps=ns, exact_mu=True, loop_delay_clks=dclk)
        r_fl["sym_sample_idx"] += skip
        m_fl = compute_metrics(sig, r_fl["sym_sample_idx"], r_fl["sym_mu"], r_fl["out_sym"])

        i_q11 = np.clip(np.rint(rx_t.real * 2048.0).astype(np.int64), -2048, 2047)
        q_q11 = np.clip(np.rint(rx_t.imag * 2048.0).astype(np.int64), -2048, 2047)
        r_fx = str_fixed_model(i_q11, q_q11, iv, kp4_fx, ki_fx, w_mu=w_mu_sel, loop_delay_clks=dclk)
        r_fx["sym_sample_idx"] += skip
        m_fx = compute_metrics(sig, r_fx["sym_sample_idx"], r_fx["sym_mu_float"], r_fx["out_sym_float"])

        L = min(len(r_fl["out_sym"]), len(r_fx["out_sym_float"]))
        ss = min(300, L // 2)
        sym_err = np.sqrt(np.mean(np.abs(r_fx["out_sym_float"][ss:L] - r_fl["out_sym"][ss:L]) ** 2))
        ted_err = np.sqrt(np.mean((r_fx["ted_err_float"][ss:L] - r_fl["ted_err"][ss:L]) ** 2))
        pi_err = np.sqrt(np.mean((r_fx["pi_out_float"][ss:L] - r_fl["pi_out"][ss:L]) ** 2))
        dmu = r_fx["sym_mu_float"][ss:L] - r_fl["sym_mu"][ss:L]
        dmu = dmu - np.round(dmu)
        mu_err = np.sqrt(np.mean(dmu ** 2))

        print(f"    {lbl:18s} | {m_fl['acq_syms']:4d} / {m_fx['acq_syms']:3d} | {m_fl['rms_jitter_ts']:.5f} / {m_fx['rms_jitter_ts']:.5f} | {sym_err:.3e} | {ted_err:.3e} | {pi_err:.3e} | {mu_err:.3e}")
        saved[lbl] = (sig, r_fl, r_fx, m_fl, m_fx)

    # Check noiseless EVM (fixed-point)
    print(f"\n[6] Fixed-Point Noiseless EVM (Spec <= 2.5%):")
    print(f"    QPSK  Fixed Noiseless EVM = {saved['QPSK  Inf    0ppm'][4]['evm_pct']:.2f}% (Float = {saved['QPSK  Inf    0ppm'][3]['evm_pct']:.2f}%)")
    print(f"    16QAM Fixed Noiseless EVM = {saved['16QAM Inf    0ppm'][4]['evm_pct']:.2f}% (Float = {saved['16QAM Inf    0ppm'][3]['evm_pct']:.2f}%)")

    # ---- Generate Stage 2 Comparison Plots ----
    c1 = saved["QPSK  15dB   0ppm"]
    c2 = saved["16QAM 15dB   0ppm"]
    c3 = saved["QPSK  15dB +200ppm"]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9.5, 4.8), sharex=True)
    ax1.plot(c1[3]["tau_err_ts"][:600], "b-", lw=1.2, label=f"QPSK Float (RMS={c1[3]['rms_jitter_ts']:.4f} Ts)")
    ax1.plot(c1[4]["tau_err_ts"][:600], "r--", lw=1.0, label=f"QPSK Fixed (RMS={c1[4]['rms_jitter_ts']:.4f} Ts)")
    ax1.set_ylabel("Timing Error (Ts)"); ax1.grid(True, alpha=0.4); ax1.legend(loc="best")
    ax1.set_title(r"Stage 2 Float vs Fixed Timing Error Comparison (SNR=15 dB, $W_\mu=12$)")
    ax2.plot(c2[3]["tau_err_ts"][:600], "b-", lw=1.2, label=f"16-QAM Float (RMS={c2[3]['rms_jitter_ts']:.4f} Ts)")
    ax2.plot(c2[4]["tau_err_ts"][:600], "r--", lw=1.0, label=f"16-QAM Fixed (RMS={c2[4]['rms_jitter_ts']:.4f} Ts)")
    ax2.set_xlabel("Symbol Index k"); ax2.set_ylabel("Timing Error (Ts)"); ax2.grid(True, alpha=0.4); ax2.legend(loc="best")
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, "01_float_vs_fixed_timing_error.png"), dpi=150); plt.close(fig)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9.5, 4.8))
    L3 = min(len(c3[1]["sym_mu"]), len(c3[2]["sym_mu_float"]))
    ax1.plot(c3[1]["sym_mu"][:L3], "b-", lw=1.2, label="Float mu")
    ax1.plot(c3[2]["sym_mu_float"][:L3], "r--", lw=1.0, label="Fixed mu (UQ0.12)")
    ax1.set_ylabel("mu (samples)"); ax1.grid(True, alpha=0.4); ax1.legend(loc="best")
    ax1.set_title("Stage 2 Float vs Fixed Fractional Delay (+200 ppm) & Complex Symbol Error")
    L1 = min(len(c1[1]["out_sym"]), len(c1[2]["out_sym_float"]))
    ax2.plot(np.abs(c1[2]["out_sym_float"][:L1] - c1[1]["out_sym"][:L1]), "k-", lw=0.8, label="|y_fixed - y_float| (QPSK, Q1.15 vs double)")
    ax2.set_xlabel("Symbol Index k"); ax2.set_ylabel("Magnitude Error"); ax2.grid(True, alpha=0.4); ax2.legend(loc="best")
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, "02_float_vs_fixed_mu_and_sym_err.png"), dpi=150); plt.close(fig)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.8, 4.7))
    sq = c1[2]["out_sym_float"][300:-20]
    ax1.plot(sq.real, sq.imag, ".", color="#1f64b4", ms=3.5, alpha=0.6)
    iq = sig_scale * np.array([1+1j, 1-1j, -1+1j, -1-1j]) / np.sqrt(2.0)
    ax1.plot(iq.real, iq.imag, "rx", ms=10, mew=2.0)
    ax1.set_xlim(-0.8, 0.8); ax1.set_ylim(-0.8, 0.8); ax1.set_aspect("equal"); ax1.grid(True, alpha=0.4)
    ax1.set_xlabel("I (Q1.15)"); ax1.set_ylabel("Q (Q1.15)")
    ax1.set_title(f"Fixed-Point QPSK (SNR=15 dB)\nRMS Jitter = {c1[4]['rms_jitter_ts']:.4f} Ts")
    sm = c2[2]["out_sym_float"][300:-20]
    ax2.plot(sm.real, sm.imag, ".", color="#c03030", ms=3.5, alpha=0.6)
    lv = sig_scale * np.array([-3, -1, 1, 3]) / np.sqrt(10.0)
    gi, gq = np.meshgrid(lv, lv)
    ax2.plot(gi.ravel(), gq.ravel(), "kx", ms=8, mew=1.8)
    ax2.set_xlim(-0.8, 0.8); ax2.set_ylim(-0.8, 0.8); ax2.set_aspect("equal"); ax2.grid(True, alpha=0.4)
    ax2.set_xlabel("I (Q1.15)"); ax2.set_ylabel("Q (Q1.15)")
    ax2.set_title(f"Fixed-Point 16-QAM (SNR=15 dB)\nRMS Jitter = {c2[4]['rms_jitter_ts']:.4f} Ts")
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, "03_fixed_constellations.png"), dpi=150); plt.close(fig)

    print(f"\nStage 2 plots saved to: {os.path.abspath(out_dir)}")


if __name__ == "__main__":
    main()
