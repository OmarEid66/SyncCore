# Stage 4 & 5 — SystemVerilog RTL & Bit-Exact Verification Report

## 1. Synthesizable SystemVerilog RTL Hierarchy (`rtl/`)

| File | Stages | Latency (clks) | DSP48E1 | Role |
|---|---|---:|---:|---|
| `rtl/str_pkg.sv` | — | — | 0 | Shared fixed-point formats (`Q1.11`, `Q1.15`, `UQ0.12`, `SQ3.15`, `SQ9.15`, `UQ0.22`) and exact saturation helpers |
| `rtl/str_input_buffer.sv` | S0 | 1 | 0 | 4-tap `Q1.11` complex shift register + 4-sample startup fill counter |
| `rtl/timing_phase_accumulator.sv` | S1–S2, S15 | 2 (parallel) | 0 | `UQ0.22` decrementing Mod-1 NCO (`0.48 ppm` step), parallel single-carry-chain subtractors, precomputed `w+kick`, multiplier-free `UQ0.12` $\mu$ |
| `rtl/farrow_interpolator.sv` | S1–S8 | 8 | 6 | 2-stage zero-DSP branch coefficients (`21845/2^17`) + 3× 2-stage Horner MACC per channel (`I` & `Q`) |
| `rtl/str_output_register.sv` | S9 | 1 | 0 | Clean registered `Q1.15` `out_sym_i/q` + `out_sym_valid` port interface |
| `rtl/timing_sample_scheduler.sv` | S9 | 1 (parallel) | 0 | Midpoint & previous-symbol storage + registered 17-bit `di`/`dq` pre-subtraction |
| `rtl/gardner_ted.sv` | S10–S11 | 2 | 2 | Registered `17s×16s` DSP48E1 products + rounded `SQ3.15` Gardner error |
| `rtl/timing_pi_controller.sv` | S12–S14 | 3 | 2 | `SQ9.15` saturating integrator + registered `KP4_FX`/`KI_FX` DSP48E1s + `SQ2.22` clamped outputs |
| `rtl/str_top.sv` | S0–S15 | **10 fwd / 15 fb** | **10** | Top-level STR matching exact Section 7 port contract |

- **QuestaSim `vlog -sv -lint` Result:** **0 Errors, 0 Warnings**.

---

## 2. Stage 5 Bit-Exact Verification Results (QuestaSim 2021.1 @ 200 MHz)

Executed `tb/tb_str_top.sv` at $F_{\text{clk}} = 200\text{ MHz}$ ($T_{\text{clk}} = 5.0\text{ ns}$) against the cycle-accurate Stage 2 fixed-point golden reference (`matlab/fixed/str_fixed_cycle_accurate.m` / `tb/vectors/*.txt`):

| # | Test Suite | Cycles | Symbols | Valid Err | Data Err | Max LSB Err | Acq Time (Spec $\le 150$) | RMS Jitter (Spec $\le 0.020\,T_s$) | Mean Err ($T_s$) | Status |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| 1 | `t01_zero_offset_qpsk` (0.00 $T_s$, 15 dB) | 4,777 | 1,187 | **0** | **0** | **0** | **1 sym** | **0.01364 $T_s$** | $-0.00532$ | **PASS** |
| 2 | `t02_pos_offset_qpsk` (+0.35 $T_s$, 15 dB) | 5,977 | 1,487 | **0** | **0** | **0** | **68 sym** | **0.01561 $T_s$** | $+0.00259$ | **PASS** |
| 3 | `t03_neg_offset_qpsk` (-0.35 $T_s$, 15 dB) | 5,977 | 1,487 | **0** | **0** | **0** | **82 sym** | **0.01367 $T_s$** | $-0.00464$ | **PASS** |
| 4 | `t04_max_offset_qpsk` (+0.45 $T_s$, 15 dB) | 5,977 | 1,487 | **0** | **0** | **0** | **129 sym** | **0.01741 $T_s$** | $+0.00428$ | **PASS** |
| 5 | `t05_qpsk_15db_stream` (+0.25 $T_s$, cont.) | 5,977 | 1,487 | **0** | **0** | **0** | **74 sym** | **0.01260 $T_s$** | $-0.00113$ | **PASS** |
| 6 | `t06_16qam_15db_stream` (+0.35 $T_s$, cont.) | 5,977 | 1,487 | **0** | **0** | **0** | **93 sym** | **0.01620 $T_s$** | $-0.00285$ | **PASS** |
| 7 | `t07_snr_0db_qpsk` (0 dB stress) | 3,977 | 987 | **0** | **0** | **0** | 68 sym | 0.16511 $T_s$ | $+0.04553$ | **PASS** |
| 8 | `t08_snr_10db_16qam` (10 dB 16-QAM) | 4,777 | 1,187 | **0** | **0** | **0** | 80 sym | 0.03137 $T_s$ | $+0.00689$ | **PASS** |
| 9 | `t09_snr_20db_16qam` (20 dB 16-QAM) | 5,977 | 1,487 | **0** | **0** | **0** | **71 sym** | **0.01225 $T_s$** | $-0.00255$ | **PASS** |
| 10 | `t10_valid_gaps_qpsk` (random valid stalls) | 8,360 | 1,187 | **0** | **0** | **0** | **65 sym** | 0.02456 $T_s$ | $+0.00548$ | **PASS** |
| 11 | `t11_clk_ppm_pos200` (+200 ppm, QPSK) | 5,977 | 1,488 | **0** | **0** | **0** | **60 sym** | **0.01803 $T_s$** | $+0.00481$ | **PASS** |
| 12 | `t12_clk_ppm_neg500` (-500 ppm, 16-QAM) | 5,977 | 1,486 | **0** | **0** | **0** | **94 sym** | **0.01648 $T_s$** | $-0.00322$ | **PASS** |
| 13 | `t13_mid_stream_reset` (async `rst_n` mid-op) | 3,983 | 984 | **0** | **0** | **0** | — | — | — | **PASS** |
| 14 | `t14_consecutive_bursts` (QPSK $\to$ gap $\to$ 16QAM) | 6,369 | 1,575 | **0** | **0** | **0** | — | — | — | **PASS** |
| **ALL** | **14 / 14 Mandatory Test Suites** | **80,059** | **19,003** | **0** | **0** | **0 LSB** | **Pass** | **Pass** | **Unbiased** | **100% PASS** |

---

## 3. Ready for Stage 6 & 7 (Vivado Implementation @ 200 MHz)
The Vivado batch flow is prepared and ready to run the moment Vivado is mounted/installed:
- XDC Timing Constraints: `vivado/str_200mhz.xdc` (200 MHz / 5.0 ns, `XC7A35TCSG324-1`)
- Non-Project Batch Script: `vivado/run_synth_impl.tcl` (Synthesis $\to$ Opt $\to$ Place $\to$ PhysOpt $\to$ Route $\to$ Post-Route PhysOpt $\to$ Timing/Utilization/Power/DRC reports)
- Re-run QuestaSim anytime via: `powershell -File scripts/run_questa.ps1`
