"""
Stage 5 Golden Vector Generator (1-to-1 cycle-accurate mirror of str_fixed_cycle_accurate.m).
Generates stimulus & bit-exact cycle-by-cycle expected RTL output vectors for ALL 14
mandatory verification test cases in Section 8, plus metadata for post-simulation
acquisition/jitter/latency/EVM reporting.
"""
import json
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_stage1_float_verify as fl
import run_stage2_fixed_verify as fx

KP4_FX = 10344
KI_FX = 883


def str_fixed_cycle_accurate(rst_n_vec, in_valid_vec, in_i_vec, in_q_vec, kp4_fx=KP4_FX, ki_fx=KI_FX):
    """Exact clock-by-clock mirror of rtl/str_top.sv (Stages S0..S15)."""
    ETA_INIT = 2097152
    W_NOM = 2097152
    W_MIN = 1048576
    W_MAX = 3145728
    ETA_MOD = 4194304

    n = len(in_i_vec)
    out_i = np.zeros(n, dtype=np.int64)
    out_q = np.zeros(n, dtype=np.int64)
    out_v = np.zeros(n, dtype=np.int64)
    sym_mu_at_out = []
    sym_src_idx_at_out = []

    sr_i = [0, 0, 0, 0]; sr_q = [0, 0, 0, 0]; fill_cnt = 0; buf_valid = False
    eta_reg = ETA_INIT; is_symbol_strobe = False
    w_freq_reg = W_NOM; kick_reg = 0; w_plus_kick_reg = W_NOM; kick_pending = False
    eta_k_s1 = 0; strobe_s1 = False; is_sym_s1 = False
    mu_s2 = 0; strobe_s2 = False; is_sym_s2 = False

    sr_i_s1 = [0, 0, 0, 0]; sr_q_s1 = [0, 0, 0, 0]
    sr_i_s2 = [0, 0, 0, 0]; sr_q_s2 = [0, 0, 0, 0]
    idx_s0 = -1; idx_s1 = -1; idx_s2 = -1
    f_yi_pipe = [0] * 6; f_yq_pipe = [0] * 6
    f_val_pipe = [False] * 6; f_sym_pipe = [False] * 6
    f_mu_pipe = [0] * 6; f_idx_pipe = [-1] * 6

    oy_i = 0; oy_q = 0; oy_v = False
    mid_i = 0; mid_q = 0; prev_i = 0; prev_q = 0; has_mid = False; has_prev = False
    ted_di = 0; ted_dq = 0; ted_mi = 0; ted_mq = 0; ted_in_v = False
    prod_i_s10 = 0; prod_q_s10 = 0; val_s10 = False
    err_s11 = 0; val_s11 = False
    integ_s12 = 0; p_prod_s12 = 0; val_s12 = False
    i_prod_s13 = 0; u_p_s13 = 0; val_s13 = False
    u_p_s14 = 0; u_i_s14 = 0; val_s14 = False
    sample_seq = -1

    for c in range(n):
        if not rst_n_vec[c]:
            sr_i = [0, 0, 0, 0]; sr_q = [0, 0, 0, 0]; fill_cnt = 0; buf_valid = False
            eta_reg = ETA_INIT; is_symbol_strobe = False
            w_freq_reg = W_NOM; kick_reg = 0; w_plus_kick_reg = W_NOM; kick_pending = False
            eta_k_s1 = 0; strobe_s1 = False; is_sym_s1 = False
            mu_s2 = 0; strobe_s2 = False; is_sym_s2 = False
            sr_i_s1 = [0, 0, 0, 0]; sr_q_s1 = [0, 0, 0, 0]
            sr_i_s2 = [0, 0, 0, 0]; sr_q_s2 = [0, 0, 0, 0]
            idx_s0 = -1; idx_s1 = -1; idx_s2 = -1
            f_yi_pipe = [0] * 6; f_yq_pipe = [0] * 6
            f_val_pipe = [False] * 6; f_sym_pipe = [False] * 6
            f_mu_pipe = [0] * 6; f_idx_pipe = [-1] * 6
            oy_i = 0; oy_q = 0; oy_v = False
            mid_i = 0; mid_q = 0; prev_i = 0; prev_q = 0; has_mid = False; has_prev = False
            ted_di = 0; ted_dq = 0; ted_mi = 0; ted_mq = 0; ted_in_v = False
            prod_i_s10 = 0; prod_q_s10 = 0; val_s10 = False
            err_s11 = 0; val_s11 = False
            integ_s12 = 0; p_prod_s12 = 0; val_s12 = False
            i_prod_s13 = 0; u_p_s13 = 0; val_s13 = False
            u_p_s14 = 0; u_i_s14 = 0; val_s14 = False
        else:
            # S15 feedback latch
            n_w_freq = w_freq_reg; n_kick = kick_reg
            n_w_pk = w_plus_kick_reg; n_kp = kick_pending
            use_kick = buf_valid and is_symbol_strobe and kick_pending
            if val_s14:
                wc = fx.fx_sat(W_NOM + u_i_s14, W_MIN, W_MAX)
                n_w_freq = wc
                n_kick = u_p_s14
                n_w_pk = wc + u_p_s14
                n_kp = True
            elif use_kick:
                n_kp = False

            # S13 -> S14
            n_up14 = u_p_s14; n_ui14 = u_i_s14; n_v14 = val_s13
            if val_s13:
                n_up14 = u_p_s13
                n_ui14 = fx.fx_sat(fx.fx_round_shift(i_prod_s13, 14), -1048576, 1048576)

            # S12 -> S13
            n_ip13 = i_prod_s13; n_up13 = u_p_s13; n_v13 = val_s12
            if val_s12:
                n_ip13 = integ_s12 * ki_fx
                n_up13 = fx.fx_sat(fx.fx_round_shift(p_prod_s12, 9), -1048576, 1048576)

            # S11 -> S12
            n_int12 = integ_s12; n_pp12 = p_prod_s12; n_v12 = val_s11
            if val_s11:
                n_int12 = fx.fx_sat(integ_s12 + err_s11, -8388608, 8388607)
                n_pp12 = err_s11 * kp4_fx

            # S10 -> S11
            n_e11 = fx.fx_sat(fx.fx_round_shift(prod_i_s10 + prod_q_s10, 15), -131072, 131071)
            n_v11 = val_s10

            # S9 -> S10
            n_pi10 = ted_di * ted_mi
            n_pq10 = ted_dq * ted_mq
            n_v10 = ted_in_v

            # S8 -> S9
            fy_i = f_yi_pipe[5]; fy_q = f_yq_pipe[5]
            fv = f_val_pipe[5]; fs = f_sym_pipe[5]
            n_oyi = oy_i; n_oyq = oy_q; n_oyv = False
            if fv and fs:
                n_oyi = fy_i; n_oyq = fy_q; n_oyv = True
                sym_mu_at_out.append(f_mu_pipe[5])
                sym_src_idx_at_out.append(f_idx_pipe[5])

            n_mi = mid_i; n_mq = mid_q; n_hm = has_mid
            n_pi = prev_i; n_pq = prev_q; n_hp = has_prev
            n_tdi = ted_di; n_tdq = ted_dq; n_tmi = ted_mi; n_tmq = ted_mq; n_tv = False
            if fv:
                if not fs:
                    n_mi = fy_i; n_mq = fy_q; n_hm = True
                else:
                    n_pi = fy_i; n_pq = fy_q; n_hp = True
                    if has_prev and has_mid:
                        n_tdi = fy_i - prev_i
                        n_tdq = fy_q - prev_q
                        n_tmi = mid_i; n_tmq = mid_q; n_tv = True

            # S2 -> S3..S8
            yi2 = fx.farrow_cubic_fixed(sr_i_s2[3], sr_i_s2[2], sr_i_s2[1], sr_i_s2[0], mu_s2, 12)
            yq2 = fx.farrow_cubic_fixed(sr_q_s2[3], sr_q_s2[2], sr_q_s2[1], sr_q_s2[0], mu_s2, 12)
            n_fyi = [yi2] + f_yi_pipe[:5]
            n_fyq = [yq2] + f_yq_pipe[:5]
            n_fv  = [strobe_s2] + f_val_pipe[:5]
            n_fs  = [is_sym_s2] + f_sym_pipe[:5]
            n_fmu = [mu_s2] + f_mu_pipe[:5]
            n_fidx = [idx_s2] + f_idx_pipe[:5]

            # S1 -> S2
            n_sri2 = list(sr_i_s1); n_srq2 = list(sr_q_s1); n_idx2 = idx_s1
            n_mu2 = fx.fx_sat(fx.fx_round_shift(eta_k_s1 << 1, 10), 0, 4095)
            n_st2 = strobe_s1; n_is2 = is_sym_s1

            # S0 -> S1
            n_sri1 = list(sr_i); n_srq1 = list(sr_q); n_idx1 = idx_s0
            n_eta = eta_reg; n_iss = is_symbol_strobe
            n_ek1 = eta_k_s1; n_st1 = False; n_is1 = False
            if buf_valid:
                step_eff = w_plus_kick_reg if use_kick else w_freq_reg
                kick_eff = kick_reg if use_kick else 0
                eta_sub = eta_reg - step_eff
                eta_k = eta_reg - kick_eff
                if eta_sub < 0:
                    n_eta = fx.fx_sat(eta_sub + ETA_MOD, 0, ETA_MOD - 1)
                    n_st1 = True
                    n_iss = not is_symbol_strobe
                else:
                    n_eta = fx.fx_sat(eta_sub, 0, ETA_MOD - 1)
                    n_st1 = False
                n_ek1 = eta_k
                n_is1 = is_symbol_strobe

            # Input -> S0
            n_sri = list(sr_i); n_srq = list(sr_q); n_fc = fill_cnt; n_bv = False; n_idx0 = idx_s0
            if in_valid_vec[c]:
                sample_seq += 1
                n_idx0 = sample_seq
                n_sri = [fx.fx_sat(int(in_i_vec[c]), -2048, 2047)] + sr_i[:3]
                n_srq = [fx.fx_sat(int(in_q_vec[c]), -2048, 2047)] + sr_q[:3]
                if fill_cnt < 3:
                    n_fc = fill_cnt + 1; n_bv = False
                else:
                    n_bv = True

            # Commit
            sr_i, sr_q, fill_cnt, buf_valid, idx_s0 = n_sri, n_srq, n_fc, n_bv, n_idx0
            eta_reg, is_symbol_strobe = n_eta, n_iss
            w_freq_reg, kick_reg, w_plus_kick_reg, kick_pending = n_w_freq, n_kick, n_w_pk, n_kp
            eta_k_s1, strobe_s1, is_sym_s1 = n_ek1, n_st1, n_is1
            mu_s2, strobe_s2, is_sym_s2 = n_mu2, n_st2, n_is2
            sr_i_s1, sr_q_s1, idx_s1 = n_sri1, n_srq1, n_idx1
            sr_i_s2, sr_q_s2, idx_s2 = n_sri2, n_srq2, n_idx2
            f_yi_pipe, f_yq_pipe, f_val_pipe, f_sym_pipe = n_fyi, n_fyq, n_fv, n_fs
            f_mu_pipe, f_idx_pipe = n_fmu, n_fidx
            oy_i, oy_q, oy_v = n_oyi, n_oyq, n_oyv
            mid_i, mid_q, has_mid = n_mi, n_mq, n_hm
            prev_i, prev_q, has_prev = n_pi, n_pq, n_hp
            ted_di, ted_dq, ted_mi, ted_mq, ted_in_v = n_tdi, n_tdq, n_tmi, n_tmq, n_tv
            prod_i_s10, prod_q_s10, val_s10 = n_pi10, n_pq10, n_v10
            err_s11, val_s11 = n_e11, n_v11
            integ_s12, p_prod_s12, val_s12 = n_int12, n_pp12, n_v12
            i_prod_s13, u_p_s13, val_s13 = n_ip13, n_up13, n_v13
            u_p_s14, u_i_s14, val_s14 = n_up14, n_ui14, n_v14

        out_i[c] = oy_i
        out_q[c] = oy_q
        out_v[c] = 1 if oy_v else 0

    return out_i, out_q, out_v, np.array(sym_mu_at_out, dtype=np.int64), np.array(sym_src_idx_at_out, dtype=np.int64)


def quant_rx(sig):
    skip = sig["rrc_span"] * sig["sps"]
    rx = sig["rx_mf"][skip:]
    ii = np.clip(np.rint(rx.real * 2048.0).astype(np.int64), -2048, 2047)
    iq = np.clip(np.rint(rx.imag * 2048.0).astype(np.int64), -2048, 2047)
    return ii, iq, skip


def build_test_case(name, sig_kw, gap_mode="none", reset_mid=False, burst2_kw=None):
    sig = fl.generate_str_signal(**sig_kw)
    ii, iq, skip = quant_rx(sig)
    rng = np.random.default_rng(sig_kw["seed"] + 999)

    rst_list, val_list, i_list, q_list = [], [], [], []
    # 5 reset cycles at start
    for _ in range(5):
        rst_list.append(0); val_list.append(0); i_list.append(0); q_list.append(0)

    def push_stream(arr_i, arr_q, insert_gaps, do_mid_reset):
        for k in range(len(arr_i)):
            if do_mid_reset and k == len(arr_i) // 2:
                for _ in range(6):
                    rst_list.append(0); val_list.append(0); i_list.append(0); q_list.append(0)
            if insert_gaps == "random" and rng.random() < 0.30:
                n_gap = int(rng.integers(1, 5))
                for _ in range(n_gap):
                    rst_list.append(1); val_list.append(0); i_list.append(0); q_list.append(0)
            rst_list.append(1); val_list.append(1); i_list.append(int(arr_i[k])); q_list.append(int(arr_q[k]))

    push_stream(ii, iq, gap_mode, reset_mid)

    if burst2_kw is not None:
        # Inter-burst gap of 40 idle cycles (valid = 0, no reset) then 2nd burst
        for _ in range(40):
            rst_list.append(1); val_list.append(0); i_list.append(0); q_list.append(0)
        sig2 = fl.generate_str_signal(**burst2_kw)
        ii2, iq2, _ = quant_rx(sig2)
        push_stream(ii2, iq2, "none", False)

    # 20 trailing flush cycles so final pipeline symbols exit cleanly
    for _ in range(20):
        rst_list.append(1); val_list.append(0); i_list.append(0); q_list.append(0)

    rst_a = np.array(rst_list, dtype=np.int64)
    val_a = np.array(val_list, dtype=np.int64)
    in_i_a = np.array(i_list, dtype=np.int64)
    in_q_a = np.array(q_list, dtype=np.int64)

    exp_i, exp_q, exp_v, sym_mu, sym_src = str_fixed_cycle_accurate(rst_a, val_a, in_i_a, in_q_a)

    # Compute algorithm-level metrics for single-burst continuous/gapped tests (no mid-reset)
    metrics = {}
    if (not reset_mid) and (burst2_kw is None):
        sym_out_c = (exp_i[exp_v == 1] + 1j * exp_q[exp_v == 1]) / 32768.0
        m = fx.compute_metrics(sig, sym_src + skip, sym_mu / 4096.0, sym_out_c)
        metrics = {
            "acq_syms": int(m["acq_syms"]),
            "mean_err_ts": round(m["mean_err_ts"], 6),
            "rms_jitter_ts": round(m["rms_jitter_ts"], 6),
            "evm_pct": round(m["evm_pct"], 3),
        }

    return {
        "name": name,
        "rst_n": rst_a, "in_valid": val_a, "in_i": in_i_a, "in_q": in_q_a,
        "exp_v": exp_v, "exp_i": exp_i, "exp_q": exp_q,
        "num_cycles": int(len(rst_a)),
        "num_out_syms": int(np.sum(exp_v)),
        "metrics": metrics,
    }


def main():
    vec_dir = os.path.join("tb", "vectors")
    os.makedirs(vec_dir, exist_ok=True)

    # All 14 mandatory Stage 5 verification scenarios (Section 8)
    specs = [
        ("t01_zero_offset_qpsk",   dict(mod_type="QPSK",  num_symbols=1200, snr_db=15.0, timing_offset_sym=0.00,  clk_ppm=0.0,    sig_scale=0.5, seed=101), "none",   False, None),
        ("t02_pos_offset_qpsk",    dict(mod_type="QPSK",  num_symbols=1500, snr_db=15.0, timing_offset_sym=+0.35, clk_ppm=0.0,    sig_scale=0.5, seed=102), "none",   False, None),
        ("t03_neg_offset_qpsk",    dict(mod_type="QPSK",  num_symbols=1500, snr_db=15.0, timing_offset_sym=-0.35, clk_ppm=0.0,    sig_scale=0.5, seed=103), "none",   False, None),
        ("t04_max_offset_qpsk",    dict(mod_type="QPSK",  num_symbols=1500, snr_db=15.0, timing_offset_sym=+0.45, clk_ppm=0.0,    sig_scale=0.5, seed=104), "none",   False, None),
        ("t05_qpsk_15db_stream",   dict(mod_type="QPSK",  num_symbols=1500, snr_db=15.0, timing_offset_sym=+0.25, clk_ppm=0.0,    sig_scale=0.5, seed=105), "none",   False, None),
        ("t06_16qam_15db_stream",  dict(mod_type="16QAM", num_symbols=1500, snr_db=15.0, timing_offset_sym=+0.35, clk_ppm=0.0,    sig_scale=0.5, seed=106), "none",   False, None),
        ("t07_snr_0db_qpsk",       dict(mod_type="QPSK",  num_symbols=1000, snr_db=0.0,  timing_offset_sym=+0.25, clk_ppm=0.0,    sig_scale=0.5, seed=107), "none",   False, None),
        ("t08_snr_10db_16qam",     dict(mod_type="16QAM", num_symbols=1200, snr_db=10.0, timing_offset_sym=-0.25, clk_ppm=0.0,    sig_scale=0.5, seed=108), "none",   False, None),
        ("t09_snr_20db_16qam",     dict(mod_type="16QAM", num_symbols=1500, snr_db=20.0, timing_offset_sym=+0.30, clk_ppm=0.0,    sig_scale=0.5, seed=109), "none",   False, None),
        ("t10_valid_gaps_qpsk",    dict(mod_type="QPSK",  num_symbols=1200, snr_db=15.0, timing_offset_sym=+0.35, clk_ppm=0.0,    sig_scale=0.5, seed=110), "random", False, None),
        ("t11_clk_ppm_pos200",     dict(mod_type="QPSK",  num_symbols=1500, snr_db=15.0, timing_offset_sym=+0.35, clk_ppm=+200.0, sig_scale=0.5, seed=111), "none",   False, None),
        ("t12_clk_ppm_neg500",     dict(mod_type="16QAM", num_symbols=1500, snr_db=15.0, timing_offset_sym=-0.30, clk_ppm=-500.0, sig_scale=0.5, seed=112), "none",   False, None),
        ("t13_mid_stream_reset",   dict(mod_type="QPSK",  num_symbols=1000, snr_db=15.0, timing_offset_sym=+0.35, clk_ppm=0.0,    sig_scale=0.5, seed=113), "none",   True,  None),
        ("t14_consecutive_bursts", dict(mod_type="QPSK",  num_symbols=800,  snr_db=15.0, timing_offset_sym=+0.35, clk_ppm=0.0,    sig_scale=0.5, seed=114), "none",   False,
         dict(mod_type="16QAM", num_symbols=800, snr_db=15.0, timing_offset_sym=+0.35, clk_ppm=0.0, sig_scale=0.5, seed=214)),
    ]

    manifest = []
    print("=" * 86)
    print("  STAGE 5: GENERATING 14 BIT-ACCURATE GOLDEN TEST VECTOR SUITES")
    print("=" * 86)
    for name, skw, gmode, rmid, b2 in specs:
        tc = build_test_case(name, skw, gmode, rmid, b2)
        vec_path = os.path.join(vec_dir, f"{name}.txt")
        with open(vec_path, "w") as f:
            for c in range(tc["num_cycles"]):
                f.write(f"{tc['rst_n'][c]} {tc['in_valid'][c]} {tc['in_i'][c]} {tc['in_q'][c]} {tc['exp_v'][c]} {tc['exp_i'][c]} {tc['exp_q'][c]}\n")
        manifest.append({
            "name": name, "file": f"tb/vectors/{name}.txt",
            "num_cycles": tc["num_cycles"], "num_out_syms": tc["num_out_syms"],
            "metrics": tc["metrics"],
        })
        m = tc["metrics"]
        m_str = f"Acq={m['acq_syms']:3d} sym | RMS={m['rms_jitter_ts']:.5f} Ts | Mean={m['mean_err_ts']:+.5f} Ts" if m else "(reset/multi-burst structural test)"
        print(f"  {name:24s} | {tc['num_cycles']:5d} clks | {tc['num_out_syms']:4d} syms | {m_str}")

    with open(os.path.join(vec_dir, "manifest.json"), "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"\nSaved 14 test suites + manifest to {os.path.abspath(vec_dir)}")


if __name__ == "__main__":
    main()
