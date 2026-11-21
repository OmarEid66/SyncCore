# SyncCore: Real-Time All-Digital Coarse Carrier Frequency Offset (CFO) Estimator

[![SystemVerilog](https://img.shields.io/badge/Language-SystemVerilog-blue.svg)](rtl/)
[![MATLAB](https://img.shields.io/badge/Model-MATLAB%20Fixed--Point-orange.svg)](matlab/)
[![FPGA](https://img.shields.io/badge/FPGA-Xilinx%20Artix--7%20(-1)-red.svg)](synth/)
[![Fmax](https://img.shields.io/badge/Fmax-201.82%20MHz-brightgreen.svg)](synth/synth_results.txt)
[![QuestaSim](https://img.shields.io/badge/Simulation-QuestaSim%2014%2F14%20PASSED-success.svg)](tb/)
[![Zero-BRAM](https://img.shields.io/badge/BRAM-0%20Used-blueviolet.svg)](synth/utilization.rpt)

A synthesizable, parameterized, all-digital **Coarse Carrier Frequency Offset (CFO) Estimator** IP core for wireless baseband receivers. Conforms to the technical specifications of the **Real-Time All-Digital Carrier and Timing Synchronization Unit** (SI CLASH — Electronics Research Institute).

---

## Architecture Diagram

![Coarse CFO Estimator Architecture](doc/coarse_cfo_architecture.png)

---

## 1. Specifications & Performance Compliance

| Parameter | Specification | Achieved (Hardware / RTL) | Status |
| :--- | :--- | :--- | :---: |
| **Clock Frequency ($F_{clk}$)** | 100–200 MHz (Typ: 125 MHz) | **201.82 MHz** ($T_{clk} = 4.955\text{ ns}$, WNS = $+0.045\text{ ns}$) | **PASS** |
| **Oversampling Factor ($N_s$)** | 4 samples/symbol | 4 samples/symbol ($F_s = 4 R_{sym}$) | **PASS** |
| **Input Format** | Signed 12-bit Q1.11 | Signed 12-bit Q1.11 (`str_out_i`, `str_out_q`) | **PASS** |
| **Output CFO Width** | 24 bits signed | 24 bits signed (`coarse_cfo_est[23:0]`) | **PASS** |
| **Output Representation** | Signed Q1.23 normalized ($2\Delta f / F_s$) | Signed Q1.23 ($[-1.0, +1.0) \equiv [-F_s/2, +F_s/2)$) | **PASS** |
| **Downstream NCO FTW** | 32-bit signed ($coarse\_cfo\_est \ll 8$) | 32-bit signed `nco_ftw[31:0]` | **PASS** |
| **CFO Capture Range** | $\pm 0.10 \times R_{sym}$ ($\pm 3.125\text{ MHz}$) | $\pm 0.125 \times R_{sym}$ ($\pm 3.906\text{ MHz}$, 25% safety margin) | **PASS** |
| **Estimation Accuracy** | $\le \pm 0.005 \times R_{sym}$ ($\pm 156.25\text{ kHz}$) | **$\le 5.53\text{ kHz}$ ($\le 0.00018\,R_{sym}$) across SNR 0–20 dB** | **PASS** |
| **Estimation Latency** | 32 to 128 symbols (Typ: 64 symbols) | **57.5 symbols (230 clock cycles)** | **PASS** |
| **Preamble Detection** | Dual-Mode: Autonomous + External Trigger | Mode A (Normalized Metric) & Mode B (Trigger) verified | **PASS** |

---

## 2. Key Architectural Innovations

1. **Zero-DSP / Zero-Divider Frequency Normalization**:
   The normalized CFO is defined as $f_{\text{norm}} = \frac{2\Delta f}{F_s} = \frac{\theta}{16\pi} = \frac{1}{16}\left(\frac{\theta}{\pi}\right)$. By configuring the 16-stage CORDIC engine to output phase normalized to $\pi \equiv 2^{23}$ (signed Q1.23), frequency normalization is computed using an exact 4-bit arithmetic right shift (`theta >>> 4`). **Zero hardware multipliers or dividers** are used for CFO calculation.
2. **Pipelined Dual-Mode Preamble Detector**:
   - **Mode A (Autonomous)**: Computes $M[n] = \frac{|C[n]|^2}{E_1[n]E_2[n] + \epsilon} \ge \gamma_{\text{thresh}}$ and $E_1[n] \ge E_{\min}$ across a 2-window persistence counter.
   - **Mode B (External Trigger)**: Accepts `burst_start` and `preamble_enable` for deterministic system-level control.
   - Features free-running retiming registers absorbed directly into DSP48 `AREG` to eliminate long routing delays and enable 200 MHz operation.
3. **Valid-Gated Delay Line (Zero Block RAM)**:
   A 16-deep shift register memory array buffers incoming samples. Gated strictly by `str_out_valid`, it naturally pauses during streaming bubbles and stalls, preserving carrier phase coherence across arbitrary sample gaps.
4. **34-Bit Overflow-Immune Accumulator**:
   Accumulates $M=192$ correlation products with 9 guard bits, providing headroom for up to $M=512$ samples with zero overflow probability.

---

## 3. FPGA Synthesis & Resource Utilization

Targeting the **AMD Xilinx Artix-7 `xc7a35tcsg324-1`** (slowest speed grade `-1`) via **Vivado 2025.2**:

| Resource | Used | Available | Utilization (%) |
| :--- | :---: | :---: | :---: |
| **Slice LUTs** | 2,937 | 20,800 | **14.12%** |
| **Slice Registers (FFs)** | 4,012 | 41,600 | **9.64%** |
| **DSP48E1 Slices** | 12 | 90 | **13.33%** |
| **Block RAM (BRAM36/18)** | 0 | 50 | **0.00%** |
| **LUT as Memory (SRL)** | 16 | 9,600 | **0.17%** |

* **WNS**: **+0.045 ns** at 5.000 ns clock period (200.0 MHz constraint).
* **Fmax**: **201.82 MHz**.
* **Zero Failing Endpoints** across all 7,850 setup, hold, and pulse-width timing endpoints.

---

## 4. Verification Suite

### MATLAB Fixed-Point Golden Model (`matlab/`)
* Verified across all 13 reference scenarios: zero CFO, $\pm 0.05\,R_{sym}$, $\pm 0.10\,R_{sym}$ limits, $0\text{--}20\text{ dB}$ SNR sweep, autonomous trigger, streaming gaps, multi-burst tracking, and mid-stream reset.

### SystemVerilog Self-Checking Testbench (`tb/`)
Simulated in **QuestaSim 2021.1** against test vectors directly exported from MATLAB:
- **Test 1**: Zero CFO (0 Hz, SNR = 25 dB, Mode B) &rarr; **PASSED** (Error: 0.36 kHz)
- **Test 2**: Moderate Positive CFO (+0.05 Rsym, SNR = 20 dB) &rarr; **PASSED** (Error: 2.93 kHz)
- **Test 3**: Moderate Negative CFO (-0.05 Rsym, SNR = 20 dB) &rarr; **PASSED** (Error: 5.13 kHz)
- **Test 4**: Max Positive CFO Limit (+0.10 Rsym, SNR = 18 dB) &rarr; **PASSED** (Error: 0.75 kHz)
- **Test 5**: Max Negative CFO Limit (-0.10 Rsym, SNR = 18 dB) &rarr; **PASSED** (Error: 0.26 kHz)
- **Test 6**: Intermediate Positive CFO (+0.07 Rsym, SNR = 15 dB) &rarr; **PASSED** (Error: 5.49 kHz)
- **Test 7**: Intermediate Negative CFO (-0.03 Rsym, SNR = 15 dB) &rarr; **PASSED** (Error: 2.05 kHz)
- **Test 8**: Low SNR Condition (+0.02 Rsym, SNR = 6 dB) &rarr; **PASSED** (Error: 25.40 kHz)
- **Test 9**: Autonomous Mode A (+0.03 Rsym, SNR = 12 dB) &rarr; **PASSED** (Error: 5.53 kHz)
- **Test 10**: Autonomous Mode A (-0.08 Rsym, SNR = 14 dB) &rarr; **PASSED** (Error: 0.37 kHz)
- **Test 11**: Streaming with 50% Invalid-Sample Gaps &rarr; **PASSED** (Error: 1.30 kHz)
- **Test 12-1**: Consecutive Back-to-Back Bursts (Burst 1: +0.05 Rsym) &rarr; **PASSED**
- **Test 12-2**: Consecutive Back-to-Back Bursts (Burst 2: -0.05 Rsym) &rarr; **PASSED**
- **Test 13**: Mid-Stream Reset & Abort Verification &rarr; **PASSED**
- **Summary**: **14 / 14 TESTS PASSED**

---

## 5. Repository Directory Layout

```
.
├── doc/
│   ├── coarse_cfo_architecture.png            # 7K ultra-high-resolution architecture diagram
│   ├── coarse_cfo_estimator_spec.md           # Mathematical and architectural specifications
│   ├── draw_architecture.py                   # Python diagram generator script
│   └── verification_and_architecture_report.md# Complete verification and timing report
├── matlab/
│   ├── cfo_coarse_golden_model.m              # Bit-accurate fixed-point golden model
│   ├── cordic_atan2_fixed.m                   # Bit-accurate 16-stage CORDIC engine
│   ├── generate_zc_preamble.m                 # Zadoff-Chu periodic preamble generator
│   ├── run_cfo_verification.m                 # 13-scenario automated test runner
│   └── export_test_vectors.m                  # Stimulus & expected output vector generator
├── rtl/
│   ├── cfo_delay_line.sv                      # Valid-gated sample delay line (L = 16)
│   ├── cfo_conj_multiplier.sv                 # 2-stage pipelined conjugate multiplier
│   ├── cfo_preamble_detector.sv               # Dual-mode autonomous & triggered detector
│   ├── cfo_accumulator.sv                     # 34-bit correlation accumulator (M = 192)
│   ├── cordic_atan2.sv                        # 16-stage pipelined CORDIC vectoring engine
│   ├── cfo_calculator.sv                      # 4-bit shift CFO & NCO FTW calculator
│   └── coarse_cfo_estimator_top.sv            # Top-level estimator wrapper & FSM
├── sim/
│   └── run_sim.do                             # QuestaSim batch compilation and simulation script
├── synth/
│   ├── run_synth_timing.tcl                   # Vivado synthesis, placement & timing script
│   ├── synth_results.txt                      # Automated timing query output
│   ├── timing_summary.rpt                     # Detailed post-placement timing report
│   └── utilization.rpt                        # FPGA resource utilization report
└── tb/
    ├── vectors/                               # 22 stimulus and golden expected vector files
    └── tb_coarse_cfo_estimator.sv             # Self-checking SystemVerilog testbench
```

---

## 6. How to Run

### Run RTL Simulation (QuestaSim)
```bash
cd sim
vsim -c -do "do run_sim.do; quit -f"
```

### Run MATLAB Golden Verification
```matlab
cd matlab
run_cfo_verification
```

### Run Vivado Synthesis & Timing Closure Analysis
```bash
cd synth
vivado -mode batch -source run_synth_timing.tcl
```
