"""
Stage 1 Floating-Point STR Verification Runner (1-to-1 IEEE-754 float64 mirror of matlab/float/*.m).
Runs all mathematical validations using the bias-free Rice/Gardner NCO and optimal PI gains
(Bn*Ts = 0.0080, zeta = 0.707), and generates all 9 mandatory Stage 1 plots.
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def rrc_pulse(u, alpha=0.35):
    u = np.asarray(u, dtype=np.float64)
    p = np.zeros_like(u, dtype=np.float64)
    eps_tol = 1e-10
    idx0 = np.abs(u) < eps_tol
    if np.any(idx0):
        p[idx0] = 1.0 - alpha + (4.0 * alpha / np.pi)
    if alpha > 0:
        u_sing = 1.0 / (4.0 * alpha)
        idx_s = (~idx0) & (np.abs(np.abs(u) - u_sing) < eps_tol)
        if np.any(idx_s):
            p[idx_s] = (alpha / np.sqrt(2.0)) * (
                (1.0 + 2.0 / np.pi) * np.sin(np.pi / (4.0 * alpha))
                + (1.0 - 2.0 / np.pi) * np.cos(np.pi / (4.0 * alpha))
            )
    else:
        idx_s = np.zeros_like(idx0, dtype=bool)
    idx_g = (~idx0) & (~idx_s)
    if np.any(idx_g):
        ug = u[idx_g]
        num = np.sin(np.pi * ug * (1.0 - alpha)) + 4.0 * alpha * ug * np.cos(np.pi * ug * (1.0 + alpha))
        den = np.pi * ug * (1.0 - (4.0 * alpha * ug) ** 2)
        p[idx_g] = num / den
    return p


def generate_str_signal(
    mod_type="QPSK",
    num_symbols=2500,
    sps=4,
    alpha=0.35,
    rrc_span=12,
    snr_db=15.0,
    timing_offset_sym=0.35,
    clk_ppm=0.0,
    sig_scale=1.0,
    seed=42,
):
    rng = np.random.default_rng(seed)
    half_span = rrc_span // 2

    if mod_type.upper() == "QPSK":
        bits_i = 2 * rng.integers(0, 2, size=num_symbols) - 1
        bits_q = 2 * rng.integers(0, 2, size=num_symbols) - 1
        tx_sym = (bits_i + 1j * bits_q) / np.sqrt(2.0)
    elif mod_type.upper() == "16QAM":
        levels = np.array([-3.0, -1.0, 1.0, 3.0])
        tx_sym = (rng.choice(levels, size=num_symbols) + 1j * rng.choice(levels, size=num_symbols)) / np.sqrt(10.0)
    else:
        raise ValueError(f"Unsupported mod_type: {mod_type}")

    mf_delay_samp = (rrc_span * sps) // 2
    m_grid = np.arange(-mf_delay_samp, mf_delay_samp + 1, dtype=np.float64) / sps
    h_raw = rrc_pulse(m_grid, alpha)
    norm_f = np.sqrt(np.sum(h_raw ** 2))
    h_rrc = h_raw / norm_f

    eps_clk = clk_ppm * 1e-6
    n_samp = num_symbols * sps
    n_vec = np.arange(n_samp, dtype=np.float64)
    t_adc = (n_vec / sps) * (1.0 + eps_clk) + timing_offset_sym

    pad = half_span + 2
    tx_pad = np.concatenate([np.zeros(pad, dtype=np.complex128), tx_sym, np.zeros(pad, dtype=np.complex128)])
    k_center = np.floor(t_adc).astype(np.int64)
    k_offs = np.arange(-half_span, half_span + 2, dtype=np.int64)

    rx_clean = np.zeros(n_samp, dtype=np.complex128)
    for ko in k_offs:
        k_idx = k_center + ko
        u_val = t_adc - k_idx
        in_span = np.abs(u_val) <= (half_span + 1e-9)
        pulse_val = np.zeros(n_samp, dtype=np.float64)
        pulse_val[in_span] = rrc_pulse(u_val[in_span], alpha) / norm_f
        arr_idx = k_idx + pad
        valid_k = (arr_idx >= 0) & (arr_idx < len(tx_pad))
        sym_val = np.zeros(n_samp, dtype=np.complex128)
        sym_val[valid_k] = tx_pad[arr_idx[valid_k]]
        rx_clean += sym_val * pulse_val

    if np.isfinite(snr_db):
        n0 = 10.0 ** (-snr_db / 10.0)
        sigma = np.sqrt(n0 / 2.0)
        noise = sigma * (rng.standard_normal(n_samp) + 1j * rng.standard_normal(n_samp))
        rx_adc = rx_clean + noise
    else:
        rx_adc = rx_clean

    rx_adc = sig_scale * rx_adc
    tx_sym_scaled = sig_scale * tx_sym
    rx_mf = np.convolve(rx_adc, h_rrc)[:n_samp]

    return {
        "rx_mf": rx_mf,
        "rx_adc": rx_adc,
        "tx_symbols": tx_sym_scaled,
        "h_rrc": h_rrc,
        "mf_delay_samp": mf_delay_samp,
        "sps": sps,
        "rrc_span": rrc_span,
        "num_symbols": num_symbols,
        "timing_offset_sym": timing_offset_sym,
        "clk_ppm": clk_ppm,
    }


def farrow_cubic_float(xm1, x0, x1, x2, mu):
    c0 = x0
    c1 = (-1.0 / 3.0) * xm1 - 0.5 * x0 + x1 - (1.0 / 6.0) * x2
    c2 = 0.5 * xm1 - x0 + 0.5 * x1
    c3 = (-1.0 / 6.0) * xm1 + 0.5 * x0 - 0.5 * x1 + (1.0 / 6.0) * x2
    h2 = c2 + mu * c3
    h1 = c1 + mu * h2
    return c0 + mu * h1


def gardner_ted_float(i_curr, q_curr, i_prev, q_prev, i_mid, q_mid):
    return (i_curr - i_prev) * i_mid + (q_curr - q_prev) * q_mid


def pi_controller_float(e, integ_prev, kp, ki):
    integ_next = integ_prev + e
    u = kp * e + ki * integ_next
    return u, integ_next


def str_float_model(rx_i, rx_q, in_valid, kp, ki, sps=4, exact_mu=True, init_eta=0.5, loop_delay_clks=0, u_clamp=0.25):
    n = len(rx_i)
    w_nom = 2.0 / sps

    out_sym_i, out_sym_q, sym_sample_idx, sym_mu = [], [], [], []
    ted_err, pi_out, pi_integ_log = [], [], []
    eta_trace = np.zeros(n, dtype=np.float64)
    w_trace = np.zeros(n, dtype=np.float64)

    sr_i = np.zeros(4, dtype=np.float64)
    sr_q = np.zeros(4, dtype=np.float64)
    valid_count = 0

    eta = float(init_eta)
    is_symbol_strobe = False
    has_mid = False
    has_prev_sym = False
    mid_i = mid_q = prev_sym_i = prev_sym_q = 0.0
    pi_integ = 0.0
    u_i_curr = 0.0
    kick_after_mid = 0.0

    if loop_delay_clks > 0:
        pipe_p = np.zeros(loop_delay_clks, dtype=np.float64)
        pipe_i = np.zeros(loop_delay_clks, dtype=np.float64)
        pipe_v = np.zeros(loop_delay_clks, dtype=bool)

    for idx in range(n):
        if not in_valid[idx]:
            eta_trace[idx] = eta
            w_trace[idx] = w_nom + u_i_curr
            continue

        sr_i[3] = sr_i[2]; sr_i[2] = sr_i[1]; sr_i[1] = sr_i[0]; sr_i[0] = rx_i[idx]
        sr_q[3] = sr_q[2]; sr_q[2] = sr_q[1]; sr_q[1] = sr_q[0]; sr_q[0] = rx_q[idx]
        valid_count += 1

        if valid_count < 4:
            eta_trace[idx] = eta
            w_trace[idx] = w_nom
            continue

        if loop_delay_clks > 0:
            if pipe_v[-1]:
                kick_after_mid = pipe_p[-1]
                u_i_curr = pipe_i[-1]
            pipe_p[1:] = pipe_p[:-1]; pipe_p[0] = 0.0
            pipe_i[1:] = pipe_i[:-1]; pipe_i[0] = u_i_curr
            pipe_v[1:] = pipe_v[:-1]; pipe_v[0] = False

        w_freq = min(max(w_nom + u_i_curr, 0.25), 0.75)
        kick = 0.0
        if is_symbol_strobe and (kick_after_mid != 0.0):
            kick = kick_after_mid
            kick_after_mid = 0.0

        eta_k = eta - kick
        eta_sub = eta_k - w_freq

        eta_trace[idx] = eta
        w_trace[idx] = w_freq

        if eta_sub < 0.0:
            mu = (eta_k / w_freq) if exact_mu else (2.0 * eta_k)
            mu = min(max(mu, 0.0), 1.0 - 1e-12)
            eta = eta_sub + 1.0

            y_i = farrow_cubic_float(sr_i[3], sr_i[2], sr_i[1], sr_i[0], mu)
            y_q = farrow_cubic_float(sr_q[3], sr_q[2], sr_q[1], sr_q[0], mu)

            if not is_symbol_strobe:
                mid_i = y_i
                mid_q = y_q
                has_mid = True
                is_symbol_strobe = True
            else:
                out_sym_i.append(y_i)
                out_sym_q.append(y_q)
                sym_sample_idx.append(idx)
                sym_mu.append(mu)

                if has_prev_sym and has_mid:
                    e = gardner_ted_float(y_i, y_q, prev_sym_i, prev_sym_q, mid_i, mid_q)
                    u_sym, pi_integ = pi_controller_float(e, pi_integ, kp, ki)
                    u_p = min(max(sps * kp * e, -u_clamp), u_clamp)
                    u_i = min(max(ki * pi_integ, -u_clamp), u_clamp)
                    if loop_delay_clks > 0:
                        pipe_p[0] = u_p; pipe_i[0] = u_i; pipe_v[0] = True
                    else:
                        kick_after_mid = u_p
                        u_i_curr = u_i
                else:
                    e = 0.0
                    u_sym = 0.0

                ted_err.append(e)
                pi_out.append(u_sym)
                pi_integ_log.append(pi_integ)
                prev_sym_i = y_i
                prev_sym_q = y_q
                has_prev_sym = True
                is_symbol_strobe = False
        else:
            eta = eta_sub

    out_i = np.array(out_sym_i)
    out_q = np.array(out_sym_q)
    return {
        "out_sym_i": out_i,
        "out_sym_q": out_q,
        "out_sym": out_i + 1j * out_q,
        "sym_sample_idx": np.array(sym_sample_idx, dtype=np.float64),
        "sym_mu": np.array(sym_mu, dtype=np.float64),
        "ted_err": np.array(ted_err, dtype=np.float64),
        "pi_out": np.array(pi_out, dtype=np.float64),
        "pi_integ": np.array(pi_integ_log, dtype=np.float64),
        "eta_trace": eta_trace,
        "w_trace": w_trace,
    }


def validate_farrow_float(alpha=0.35):
    ns = 4
    mu_grid = np.linspace(0.0, 1.0 - 1e-4, 200)

    def rc_pulse(u):
        u = np.asarray(u, dtype=np.float64)
        out = np.zeros_like(u)
        sing = np.abs(np.abs(2 * alpha * u) - 1.0) < 1e-10
        out[sing] = (np.pi / 4.0) * np.sinc(1.0 / (2.0 * alpha))
        g = ~sing
        ug = u[g]
        out[g] = np.sinc(ug) * np.cos(np.pi * alpha * ug) / (1.0 - (2 * alpha * ug) ** 2)
        return out

    m_base = np.arange(-20, 21)
    err_rc = np.zeros((len(m_base), len(mu_grid)))
    for mi, m in enumerate(m_base):
        xm1 = rc_pulse((m - 1) / ns)
        x0 = rc_pulse(m / ns)
        x1 = rc_pulse((m + 1) / ns)
        x2 = rc_pulse((m + 2) / ns)
        yf = farrow_cubic_float(xm1, x0, x1, x2, mu_grid)
        ye = rc_pulse((m + mu_grid) / ns)
        err_rc[mi, :] = yf - ye

    max_err_rc = np.max(np.abs(err_rc))
    rms_err_rc = np.sqrt(np.mean(err_rc ** 2))
    sqnr_rc = 20.0 * np.log10(np.sqrt(np.mean(rc_pulse(m_base / ns) ** 2)) / rms_err_rc)

    sig0 = generate_str_signal("QPSK", 500, ns, alpha, 12, np.inf, 0.0, 0.0, 1.0, 77)
    mu_test = 0.37
    sig_ref = generate_str_signal("QPSK", 500, ns, alpha, 12, np.inf, mu_test / ns, 0.0, 1.0, 77)
    idx_r = np.arange(sig0["mf_delay_samp"] + 40, len(sig0["rx_mf"]) - sig0["mf_delay_samp"] - 40)
    x = sig0["rx_mf"]
    yi = farrow_cubic_float(x[idx_r - 1].real, x[idx_r].real, x[idx_r + 1].real, x[idx_r + 2].real, mu_test)
    yq = farrow_cubic_float(x[idx_r - 1].imag, x[idx_r].imag, x[idx_r + 1].imag, x[idx_r + 2].imag, mu_test)
    diff = (yi + 1j * yq) - sig_ref["rx_mf"][idx_r]
    sig_rms = np.sqrt(np.mean(np.abs(diff) ** 2))
    sig_max = np.max(np.abs(diff))
    sig_snr = 10.0 * np.log10(np.mean(np.abs(sig_ref["rx_mf"][idx_r]) ** 2) / np.mean(np.abs(diff) ** 2))
    return max_err_rc, rms_err_rc, sqnr_rc, sig_rms, sig_max, sig_snr


def compute_ted_scurve(mod_type="QPSK", alpha=0.35, num_symbols=4000, seed=101):
    tau_grid = np.linspace(-0.5, 0.5, 41)
    scurve = np.zeros_like(tau_grid)
    ns = 4
    span = 12
    for i, tau in enumerate(tau_grid):
        sig = generate_str_signal(mod_type, num_symbols, ns, alpha, span, np.inf, tau, 0.0, 1.0, seed)
        rx = sig["rx_mf"]
        d = sig["mf_delay_samp"]
        k_vec = np.arange(span + 2, num_symbols - span - 2)
        ic = d + k_vec * ns
        ip = d + (k_vec - 1) * ns
        im = d + k_vec * ns - ns // 2
        e = gardner_ted_float(rx[ic].real, rx[ic].imag, rx[ip].real, rx[ip].imag, rx[im].real, rx[im].imag)
        scurve[i] = np.mean(e)
    i0 = np.argmin(np.abs(tau_grid))
    kd = (scurve[i0 + 1] - scurve[i0 - 1]) / (tau_grid[i0 + 1] - tau_grid[i0 - 1])
    return tau_grid, scurve, kd


def design_pi_gains(bn_ts, zeta, kd, k0=2.0, d_sym=0):
    wn_ts = (2.0 * bn_ts) / (zeta + 1.0 / (4.0 * zeta))
    if zeta < 1.0:
        wd_ts = wn_ts * np.sqrt(1.0 - zeta ** 2)
        p_prod = np.exp(-2.0 * zeta * wn_ts)
        p_sum = 2.0 * np.exp(-zeta * wn_ts) * np.cos(wd_ts)
    else:
        r1 = np.exp((-zeta + np.sqrt(zeta ** 2 - 1.0)) * wn_ts)
        r2 = np.exp((-zeta - np.sqrt(zeta ** 2 - 1.0)) * wn_ts)
        p_prod = r1 * r2
        p_sum = r1 + r2
    a_val = 1.0 - p_prod
    b_val = 1.0 + p_prod - p_sum
    kp = a_val / (kd * k0)
    ki = b_val / (kd * k0)

    w = np.logspace(-4, np.log10(np.pi), 4000)
    z = np.exp(1j * w)
    l_jw = kd * k0 * (kp + ki / (1.0 - z ** -1)) * (z ** -(1 + d_sym)) / (1.0 - z ** -1)
    idx_gc = np.argmin(np.abs(np.abs(l_jw) - 1.0))
    pm_deg = ((180.0 + np.rad2deg(np.angle(l_jw[idx_gc])) + 180.0) % 360.0) - 180.0

    d_int = int(round(d_sym))
    char_poly = np.zeros(max(3, 3 + d_int))
    char_poly[0] = 1.0
    char_poly[1] = -2.0
    char_poly[2] = 1.0
    char_poly[1 + d_int] += kd * k0 * (kp + ki)
    char_poly[2 + d_int] -= kd * k0 * kp
    poles = np.roots(char_poly)
    max_p = float(np.max(np.abs(poles)))
    return {
        "wn_ts": wn_ts, "kp": kp, "ki": ki,
        "max_pole": max_p, "stable": max_p < 1.0, "pm_deg": pm_deg,
    }


def eval_case(sig_kw, kp, ki, exact_mu=True, loop_delay_clks=0):
    sig = generate_str_signal(**sig_kw)
    skip = sig["rrc_span"] * sig["sps"]
    rx_trim = sig["rx_mf"][skip:]
    in_valid = np.ones(len(rx_trim), dtype=bool)
    res = str_float_model(rx_trim.real, rx_trim.imag, in_valid, kp, ki, sps=sig["sps"],
                          exact_mu=exact_mu, loop_delay_clks=loop_delay_clks)
    res["sym_sample_idx"] = res["sym_sample_idx"] + skip

    eps_clk = sig["clk_ppm"] * 1e-6
    ns = sig["sps"]
    l_mf = sig["mf_delay_samp"]
    t_eff = (((res["sym_sample_idx"] - 2.0) + res["sym_mu"]) - l_mf) / ns * (1.0 + eps_clk) + sig["timing_offset_sym"]
    k_ideal = np.round(t_eff).astype(np.int64)
    tau_err = t_eff - k_ideal

    valid_int = (k_ideal >= sig["rrc_span"]) & (k_ideal <= sig["num_symbols"] - sig["rrc_span"] - 1)
    tau_v = tau_err[valid_int]
    sym_v = res["out_sym"][valid_int]
    ref_v = sig["tx_symbols"][k_ideal[valid_int]]

    acq = len(tau_v)
    win = 20
    for k in range(len(tau_v) - win):
        if np.all(np.abs(tau_v[k:k + win]) <= 0.05):
            acq = k + 1
            break

    ss_start = min(300, len(tau_v) // 2)
    tau_ss = tau_v[ss_start:]
    sym_ss = sym_v[ss_start:]
    ref_ss = ref_v[ss_start:]

    metrics = {
        "tau_err_ts": tau_v,
        "acq_syms": acq,
        "mean_err_ts": float(np.mean(tau_ss)),
        "rms_jitter_ts": float(np.sqrt(np.mean(tau_ss ** 2))),
        "std_jitter_ts": float(np.std(tau_ss)),
        "evm_pct": float(100.0 * np.sqrt(np.mean(np.abs(sym_ss - ref_ss) ** 2) / np.mean(np.abs(ref_ss) ** 2))),
    }
    return sig, res, metrics


def main():
    out_dir = os.path.join("matlab", "plots", "stage1")
    os.makedirs(out_dir, exist_ok=True)
    alpha = 0.35

    print("=" * 70)
    print("  STAGE 1: FLOATING-POINT GOLDEN MODEL VALIDATION (VERIFIED NCO)")
    print("=" * 70)

    f_res = validate_farrow_float(alpha)
    print(f"[1] Cubic Farrow (alpha={alpha}, Ns=4):")
    print(f"    RC pulse: max|err|={f_res[0]:.3e}, RMS={f_res[1]:.3e}, SQNR={f_res[2]:.2f} dB")
    print(f"    QPSK sig: RMS={f_res[3]:.3e}, max={f_res[4]:.3e}, SNR={f_res[5]:.2f} dB\n")

    tau_grid, sc_q, kd_q = compute_ted_scurve("QPSK", alpha, 4000, 101)
    _, sc_m, kd_m = compute_ted_scurve("16QAM", alpha, 4000, 102)
    kd = 0.5 * (kd_q + kd_m)
    print(f"[2] Gardner TED Gain (Es=1, alpha={alpha}):")
    print(f"    Kd(QPSK)={kd_q:.4f}, Kd(16QAM)={kd_m:.4f}, Kd(mean)={kd:.4f}\n")

    bn_sel = 0.0080
    zeta_sel = 0.707
    lp0 = design_pi_gains(bn_sel, zeta_sel, kd, 2.0, 0)
    lp2 = design_pi_gains(bn_sel, zeta_sel, kd, 2.0, 2)
    kp, ki = lp0["kp"], lp0["ki"]
    print(f"[3] Optimal PI Loop Filter: Bn*Ts={bn_sel}, zeta={zeta_sel}, wn*Ts={lp0['wn_ts']:.5f}")
    print(f"    Kp={kp:.6f}, Ki={ki:.6f}")
    print(f"    D=0 sym: max|p|={lp0['max_pole']:.5f}, PM={lp0['pm_deg']:.1f} deg, stable={lp0['stable']}")
    print(f"    D=2 sym: max|p|={lp2['max_pole']:.5f}, PM={lp2['pm_deg']:.1f} deg, stable={lp2['stable']}\n")

    base_q = dict(mod_type="QPSK", num_symbols=2500, sps=4, alpha=alpha, rrc_span=12, snr_db=15.0, timing_offset_sym=0.35, clk_ppm=0.0, seed=42)
    base_m = dict(mod_type="16QAM", num_symbols=2500, sps=4, alpha=alpha, rrc_span=12, snr_db=15.0, timing_offset_sym=0.35, clk_ppm=0.0, seed=43)
    base_ppm = dict(base_q, clk_ppm=200.0, seed=44)

    sig_q, res_q, m_q = eval_case(base_q, kp, ki, exact_mu=True)
    sig_m, res_m, m_m = eval_case(base_m, kp, ki, exact_mu=True)
    sig_p, res_p, m_p = eval_case(base_ppm, kp, ki, exact_mu=True)
    _, _, m_q_approx = eval_case(base_q, kp, ki, exact_mu=False)
    _, _, m_q_d8 = eval_case(base_q, kp, ki, exact_mu=False, loop_delay_clks=8)
    _, _, m_m_d8 = eval_case(base_m, kp, ki, exact_mu=False, loop_delay_clks=8)
    _, _, m_q_clean = eval_case(dict(base_q, snr_db=np.inf), kp, ki, exact_mu=True)
    _, _, m_m_clean = eval_case(dict(base_m, snr_db=np.inf), kp, ki, exact_mu=True)

    print("[4] Reference Performance Summary (tau0 = +0.35 Ts):")
    for lbl, m in [
        ("QPSK  15dB  0ppm (exact mu)", m_q),
        ("16QAM 15dB  0ppm (exact mu)", m_m),
        ("QPSK  15dB +200ppm", m_p),
        ("QPSK  15dB  0ppm (mu=2*eta)", m_q_approx),
        ("QPSK  15dB  0ppm (mu=2*eta, D=8clk)", m_q_d8),
        ("16QAM 15dB  0ppm (mu=2*eta, D=8clk)", m_m_d8),
        ("QPSK  Inf   0ppm (noiseless)", m_q_clean),
        ("16QAM Inf   0ppm (noiseless)", m_m_clean),
    ]:
        print(f"    {lbl:36s}: Acq={m['acq_syms']:3d} sym | Mean={m['mean_err_ts']:+.5f} Ts | RMS={m['rms_jitter_ts']:.5f} Ts | EVM={m['evm_pct']:.2f}%")

    # SNR sweep
    snr_list = [0, 5, 10, 12, 15, 18, 20]
    rms_snr_q, rms_snr_m = [], []
    print("\n[5] SNR Sensitivity Sweep (tau0 = +0.35 Ts):")
    print(f"    {'SNR(dB)':>7} | {'QPSK Acq':>8} {'QPSK RMS':>10} | {'16QAM Acq':>9} {'16QAM RMS':>10}")
    for i, sdb in enumerate(snr_list):
        _, _, mq = eval_case(dict(base_q, snr_db=float(sdb), seed=200 + i), kp, ki)
        _, _, mm = eval_case(dict(base_m, snr_db=float(sdb), seed=300 + i), kp, ki)
        rms_snr_q.append(mq["rms_jitter_ts"])
        rms_snr_m.append(mm["rms_jitter_ts"])
        print(f"    {sdb:7d} | {mq['acq_syms']:8d} {mq['rms_jitter_ts']:10.5f} | {mm['acq_syms']:9d} {mm['rms_jitter_ts']:10.5f}")

    # Clock mismatch sweep
    ppm_list = [-500, -200, -100, 0, 100, 200, 500]
    rms_ppm_q = []
    print("\n[6] Sampling Clock Mismatch Sweep (QPSK, SNR=15 dB, tau0=+0.35 Ts):")
    print(f"    {'Offset(ppm)':>11} | {'Acq(sym)':>8} {'Mean(Ts)':>10} {'RMS(Ts)':>10}")
    for i, ppm in enumerate(ppm_list):
        _, _, mp = eval_case(dict(base_q, clk_ppm=float(ppm), seed=400 + i), kp, ki)
        rms_ppm_q.append(mp["rms_jitter_ts"])
        print(f"    {ppm:11d} | {mp['acq_syms']:8d} {mp['mean_err_ts']:+10.5f} {mp['rms_jitter_ts']:10.5f}")

    # Initial offset sweep
    tau_tests = [-0.45, -0.25, 0.0, 0.25, 0.45]
    tau_traces = []
    print("\n[7] Initial Fractional Timing Offset Sweep (QPSK, SNR=15 dB):")
    for i, t0 in enumerate(tau_tests):
        _, _, mt = eval_case(dict(base_q, timing_offset_sym=t0, seed=500 + i), kp, ki)
        tau_traces.append((t0, mt))
        print(f"    tau0 = {t0:+.2f} Ts: Acq = {mt['acq_syms']:3d} sym | Mean = {mt['mean_err_ts']:+.5f} Ts | RMS = {mt['rms_jitter_ts']:.5f} Ts")

    # ---- Generate all 9 mandatory Stage 1 plots ----
    # Plot 1: Received vs Recovered Signal
    fig, ax = plt.subplots(figsize=(9, 4))
    k_win = np.arange(350, 390)
    n_start = int(res_q["sym_sample_idx"][k_win[0]]) - 2
    n_end = int(res_q["sym_sample_idx"][k_win[-1]]) + 2
    t_rx = np.arange(n_start, n_end + 1)
    ax.plot(t_rx, sig_q["rx_mf"][t_rx].real, "b.-", lw=0.9, ms=3, label="Received 4x-oversampled MF (I)")
    t_sym = res_q["sym_sample_idx"][k_win] - 2.0 + res_q["sym_mu"][k_win]
    ax.stem(t_sym, res_q["out_sym_i"][k_win], linefmt="r-", markerfmt="ro", basefmt=" ", label="Recovered STR Symbols (I)")
    ax.set_xlabel("Input Sample Index n"); ax.set_ylabel("Amplitude")
    ax.set_title("Plot 1: Received 4x-Oversampled vs Recovered Symbol Signal (QPSK, SNR=15 dB)")
    ax.grid(True, alpha=0.4); ax.legend(loc="best")
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, "01_received_vs_recovered.png"), dpi=150); plt.close(fig)

    # Plot 2: Timing Error Evolution
    fig, ax = plt.subplots(figsize=(9, 4))
    e_tr = res_q["ted_err"]
    ax.plot(e_tr, color="#3366cc", alpha=0.6, lw=0.7, label="Gardner TED e[k] (QPSK)")
    e_ma = np.convolve(e_tr, np.ones(25) / 25.0, mode="same")
    ax.plot(e_ma, "r-", lw=1.6, label="25-symbol Moving Avg")
    ax.set_xlabel("Symbol Index k"); ax.set_ylabel("TED Error e[k]")
    ax.set_title(r"Plot 2: Gardner Timing Error Detector Evolution (SNR=15 dB, $\tau_0=+0.35\,T_s$)")
    ax.grid(True, alpha=0.4); ax.legend(loc="best")
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, "02_timing_error_evolution.png"), dpi=150); plt.close(fig)

    # Plot 3: Fractional Delay Evolution (mu)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 4.5), sharex=True)
    ax1.plot(res_q["sym_mu"], "b-", lw=1.0, label=r"QPSK, 0 ppm, $\tau_0=+0.35\,T_s$")
    ax1.set_ylabel(r"$\mu$ (samples)"); ax1.grid(True, alpha=0.4); ax1.legend(loc="best")
    ax1.set_title(r"Plot 3: Fractional Delay $\mu$ Evolution (0 ppm vs +200 ppm Clock Offset)")
    ax2.plot(res_p["sym_mu"], "m-", lw=1.0, label="QPSK, +200 ppm (continuous wrap tracking)")
    ax2.set_xlabel("Symbol Index k"); ax2.set_ylabel(r"$\mu$ (samples)"); ax2.grid(True, alpha=0.4); ax2.legend(loc="best")
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, "03_fractional_delay_evolution.png"), dpi=150); plt.close(fig)

    # Plot 4: Timing Phase Evolution (eta and PI u)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 4.8))
    ax1.plot(np.arange(200), res_q["eta_trace"][:200], "b.-", lw=0.9, ms=3, label=r"Mod-1 NCO Phase $\eta[n]$ (first 200 samples)")
    ax1.set_xlabel("Input Sample Index n"); ax1.set_ylabel(r"$\eta[n]$"); ax1.grid(True, alpha=0.4); ax1.legend(loc="best")
    ax1.set_title(r"Plot 4: Timing Phase Accumulator $\eta[n]$ & PI Loop Filter Output $u[k]$")
    ax2.plot(res_q["pi_out"], "b-", lw=1.0, label="u[k] (0 ppm)")
    ax2.plot(res_p["pi_out"], "r-", lw=1.0, label="u[k] (+200 ppm)")
    ax2.set_xlabel("Symbol Index k"); ax2.set_ylabel("PI Output u[k]"); ax2.grid(True, alpha=0.4); ax2.legend(loc="best")
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, "04_timing_phase_evolution.png"), dpi=150); plt.close(fig)

    # Plot 5: Acquisition Behavior
    fig, ax = plt.subplots(figsize=(9, 4.2))
    for t0, mt in tau_traces:
        n_plot = min(300, len(mt["tau_err_ts"]))
        ax.plot(mt["tau_err_ts"][:n_plot], lw=1.3, label=rf"$\tau_0={t0:+.2f}\,T_s$ (acq={mt['acq_syms']} sym)")
    ax.axhline(0.02, color="k", ls="--", lw=1.0, label=r"$\pm 0.02\,T_s$")
    ax.axhline(-0.02, color="k", ls="--", lw=1.0)
    ax.axvline(150, color="r", ls=":", lw=1.6, label="150-sym Spec Limit")
    ax.set_xlabel("Symbol Index k"); ax.set_ylabel(r"True Timing Error $\tau_{err}[k]$ ($T_s$)")
    ax.set_title("Plot 5: Acquisition Behavior Across Initial Fractional Timing Offsets (SNR=15 dB)")
    ax.grid(True, alpha=0.4); ax.legend(loc="best", fontsize=8.5)
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, "05_acquisition_behavior.png"), dpi=150); plt.close(fig)

    # Plot 6: Residual Timing Error
    fig, ax = plt.subplots(figsize=(9, 4.2))
    k_q = np.arange(300, len(m_q["tau_err_ts"]))
    k_m = np.arange(300, len(m_m["tau_err_ts"]))
    ax.plot(k_q, m_q["tau_err_ts"][300:], "b-", lw=0.8, alpha=0.8, label=rf"QPSK (RMS={m_q['rms_jitter_ts']:.4f} $T_s$)")
    ax.plot(k_m, m_m["tau_err_ts"][300:], "r-", lw=0.8, alpha=0.75, label=rf"16-QAM (RMS={m_m['rms_jitter_ts']:.4f} $T_s$)")
    ax.axhline(0.02, color="k", ls="--", lw=1.1, label=r"Spec $\pm 0.02\,T_s$")
    ax.axhline(-0.02, color="k", ls="--", lw=1.1)
    ax.set_xlabel("Symbol Index k"); ax.set_ylabel(r"Residual Timing Error ($T_s$)")
    ax.set_title("Plot 6: Steady-State Residual Timing Error (SNR=15 dB)")
    ax.grid(True, alpha=0.4); ax.legend(loc="best")
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, "06_residual_timing_error.png"), dpi=150); plt.close(fig)

    # Plot 7: RMS Timing Jitter (vs SNR & vs Clock Mismatch)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.6, 4.0))
    ax1.plot(snr_list, rms_snr_q, "bo-", lw=1.5, label="QPSK")
    ax1.plot(snr_list, rms_snr_m, "rs-", lw=1.5, label="16-QAM")
    ax1.axhline(0.02, color="k", ls="--", lw=1.2, label=r"Spec $0.02\,T_s$ @ 15 dB")
    ax1.set_xlabel(r"SNR ($E_s/N_0$, dB)"); ax1.set_ylabel(r"RMS Timing Jitter ($T_s$)")
    ax1.set_title("Plot 7a: RMS Timing Jitter vs SNR"); ax1.grid(True, alpha=0.4); ax1.legend(loc="upper right")
    ax2.plot(ppm_list, rms_ppm_q, "mo-", lw=1.5, label="QPSK @ 15 dB")
    ax2.axhline(0.02, color="k", ls="--", lw=1.2, label=r"Spec $0.02\,T_s$")
    ax2.set_xlabel("Sampling Clock Mismatch (ppm)"); ax2.set_ylabel(r"RMS Timing Jitter ($T_s$)")
    ax2.set_title("Plot 7b: RMS Jitter vs Clock Mismatch"); ax2.grid(True, alpha=0.4); ax2.legend(loc="best")
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, "07_rms_timing_jitter.png"), dpi=150); plt.close(fig)

    # Plot 8: QPSK Constellation
    fig, ax = plt.subplots(figsize=(6.0, 5.6))
    sym_q = res_q["out_sym"][300:-20]
    ax.plot(sym_q.real, sym_q.imag, ".", color="#1f64b4", ms=4, alpha=0.6, label="Recovered Symbols")
    ideal_q = np.array([1+1j, 1-1j, -1+1j, -1-1j]) / np.sqrt(2.0)
    ax.plot(ideal_q.real, ideal_q.imag, "rx", ms=11, mew=2.2, label="Ideal QPSK")
    ax.set_xlim(-1.5, 1.5); ax.set_ylim(-1.5, 1.5); ax.set_aspect("equal")
    ax.set_xlabel("In-Phase (I)"); ax.set_ylabel("Quadrature (Q)")
    ax.set_title(rf"Plot 8: Recovered QPSK Constellation" + "\n" + rf"(SNR=15 dB, RMS Jitter={m_q['rms_jitter_ts']:.4f} $T_s$)")
    ax.grid(True, alpha=0.4); ax.legend(loc="upper right")
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, "08_constellation_qpsk.png"), dpi=150); plt.close(fig)

    # Plot 9: 16-QAM Constellation
    fig, ax = plt.subplots(figsize=(6.0, 5.6))
    sym_m = res_m["out_sym"][300:-20]
    ax.plot(sym_m.real, sym_m.imag, ".", color="#c03030", ms=4, alpha=0.6, label="Recovered Symbols")
    lv = np.array([-3, -1, 1, 3]) / np.sqrt(10.0)
    gi, gq = np.meshgrid(lv, lv)
    ax.plot(gi.ravel(), gq.ravel(), "kx", ms=9, mew=1.8, label="Ideal 16-QAM")
    ax.set_xlim(-1.5, 1.5); ax.set_ylim(-1.5, 1.5); ax.set_aspect("equal")
    ax.set_xlabel("In-Phase (I)"); ax.set_ylabel("Quadrature (Q)")
    ax.set_title(rf"Plot 9: Recovered 16-QAM Constellation" + "\n" + rf"(SNR=15 dB, RMS Jitter={m_m['rms_jitter_ts']:.4f} $T_s$)")
    ax.grid(True, alpha=0.4); ax.legend(loc="upper right")
    fig.tight_layout(); fig.savefig(os.path.join(out_dir, "09_constellation_16qam.png"), dpi=150); plt.close(fig)

    print(f"\nAll 9 plots generated in: {os.path.abspath(out_dir)}")


if __name__ == "__main__":
    main()
