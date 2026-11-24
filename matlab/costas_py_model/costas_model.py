#!/usr/bin/env python3
"""Bit-accurate symbol-level model of the Costas loop RTL (rtl/*.v).

Same fixed-point arithmetic as the RTL.  The pipeline delay of the RTL is
modelled as LOOP_DELAY symbols (3 when strobes are 4 clocks apart).
Writes rtl/sine_quarter.mem  ->  python3 costas_model.py --mem
"""
import sys, math
import numpy as np

Q15 = 32767
A_QPSK = 11585                 # 0.7071 in Q2.14
TH_QAM = 20724                 # 2/sqrt(10) in Q1.15
DN = {(0, 0): (25905, 25905),  # (|I| outer?, |Q| outer?) -> (dnI, dnQ) Q2.14
      (0, 1): (5181, 15543),
      (1, 0): (15543, 5181),
      (1, 1): (8635, 8635)}
KP_SCALE, KP_SHIFT = 652, 6
KI_SCALE, KI_SHIFT = 10, 12
I_LIM = 1 << 30                # +/-0.25 R_sym
ACQ_GAIN = 255
T_LOCK, T_UNLOCK, LOCK_CNT, M_INIT = 6500, 10000, 32, 0x7FFF
LOOP_DELAY = 3


def sine_rom():
    return [int(round(Q15 * math.sin(2 * math.pi * (k + 0.5) / 1024))) for k in range(256)]


ROM = sine_rom()


def sin_lut(p):
    q, idx = (p >> 8) & 3, p & 255
    v = ROM[255 - idx] if (q & 1) else ROM[idx]
    return -v if (q & 2) else v


def sat16(v):
    return max(-32767, min(32767, v))


def rnd_shift(v, s):
    return (v + (1 << (s - 1))) >> s


def run(x, mod, g_track, n_sym, loop_delay=LOOP_DELAY):
    theta = 0; I = 0; m = M_INIT; cnt = 0; lock = 0
    pend = [0] * loop_delay
    ys, es, locks, freqs = [], [], [], []
    for n in range(n_sym):
        xi, xq = int(x[n].real), int(x[n].imag)
        xi, xq = max(xi, -Q15), max(xq, -Q15)
        p = (theta >> 22) & 1023
        s, c = sin_lut(p), sin_lut((p + 256) & 1023)
        yi = sat16(rnd_shift(xi * c + xq * s, 15))
        yq = sat16(rnd_shift(xq * c - xi * s, 15))
        if mod == 0:
            dni = A_QPSK if yi >= 0 else -A_QPSK
            dnq = A_QPSK if yq >= 0 else -A_QPSK
        else:
            oi, oq = int(abs(yi) > TH_QAM), int(abs(yq) > TH_QAM)
            a, b = DN[(oi, oq)]
            dni = a if yi >= 0 else -a
            dnq = b if yq >= 0 else -b
        e_dd = sat16(rnd_shift(yq * dni - yi * dnq, 14))
        e = e_dd
        ae = min(abs(e), Q15)
        m = m + ((ae - m) >> 5)
        if lock:
            if m > T_UNLOCK:
                lock = 0; cnt = 0
        else:
            if m < T_LOCK:
                cnt += 1
                if cnt >= LOCK_CNT:
                    lock = 1
            else:
                cnt = 0
        g = g_track if lock else ACQ_GAIN
        vp = rnd_shift(e * (g * KP_SCALE), KP_SHIFT)
        di = rnd_shift(e * (g * g * KI_SCALE), KI_SHIFT)
        I = max(-I_LIM, min(I_LIM, I + di))
        inc = I + vp
        pend.append(inc)
        theta = (theta + pend.pop(0)) & 0xFFFFFFFF
        ys.append((yi, yq)); es.append(e); locks.append(lock); freqs.append(I)
    return dict(y=np.array(ys), e=np.array(es), lock=np.array(locks), I=np.array(freqs))


def make_stim(mod, n, cfo, ph0_deg, snr_db, seed=1):
    rng = np.random.default_rng(seed)
    if mod == 0:
        pts = (rng.choice([-1, 1], n) + 1j * rng.choice([-1, 1], n)) / np.sqrt(2)
    else:
        pts = (rng.choice([-3, -1, 1, 3], n) + 1j * rng.choice([-3, -1, 1, 3], n)) / np.sqrt(10)
    nvar = 10 ** (-snr_db / 10)
    noise = np.sqrt(nvar / 2) * (rng.standard_normal(n) + 1j * rng.standard_normal(n))
    k = np.arange(n)
    rx = pts * np.exp(1j * (2 * np.pi * cfo * k + np.deg2rad(ph0_deg))) + noise
    return pts, np.round(rx * Q15 * 0.999)


def metrics(pts, tr, cfo, tail=1000):
    y = (tr['y'][:, 0] + 1j * tr['y'][:, 1]) / Q15
    best = None
    for k in range(4):
        r = y[-tail:] * np.exp(-1j * k * np.pi / 2)
        ev = np.sqrt(np.mean(np.abs(r - pts[-tail:]) ** 2) / np.mean(np.abs(pts[-tail:]) ** 2))
        if best is None or ev < best[0]:
            best = (ev, k)
    lk = np.nonzero(tr['lock'])[0]
    lock_t = int(lk[0]) if len(lk) else None
    fest = tr['I'][-tail:].mean() / 2 ** 32
    return dict(evm_pct=100 * best[0], cfo_err=abs(fest - cfo), lock_sym=lock_t)


if __name__ == "__main__":
    if "--mem" in sys.argv:
        with open("../rtl/sine_quarter.mem", "w") as f:
            f.write("// 256-entry quarter-wave sine, Q1.15, sin(2*pi*(k+0.5)/1024)\n")
            for v in ROM:
                f.write("%04x\n" % v)
        print("wrote ../rtl/sine_quarter.mem")
        sys.exit()
    print("mod  g    cfo     ph0  snr | EVM%   cfo_err   lock@")
    for mod in (0, 1):
        for g in (106, 157, 255):
            for cfo in (0.0, 0.005, -0.01, 0.02):
                for ph in (0, 100, 200):
                    for snr in (15, 20):
                        pts, rx = make_stim(mod, 3000, cfo, ph, snr, seed=ph + 7)
                        tr = run(rx, mod, g, 3000)
                        r = metrics(pts, tr, cfo)
                        print(f"{mod}  {g:3d} {cfo:+.3f} {ph:4d} {snr:3d} | {r['evm_pct']:5.1f} {r['cfo_err']:.5f}  {str(r['lock_sym']):>5}")
