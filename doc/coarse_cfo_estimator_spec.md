# Coarse CFO Estimator — Detailed Technical Specification & Architecture Design

## 1. Executive Summary & Parameters

| Parameter | Value | Description |
| :--- | :--- | :--- |
| **System Clock ($F_{clk}$)** | 125 MHz (typ), 100–200 MHz range | System master clock |
| **Oversampling Factor ($N_s$)** | 4 samples/symbol | $F_s = 4 R_{sym}$ |
| **Symbol Rate ($R_{sym}$)** | 31.25 MSps (typ) | $R_{sym} = F_{clk} / 4$ |
| **Sample Rate ($F_s$)** | 125 MSamples/s (typ) | Timing-corrected samples from STR |
| **Input Format** | 12-bit signed Q1.11 | `str_out_i[11:0]`, `str_out_q[11:0]` |
| **CFO Capture Range** | $\pm 0.10 \times R_{sym}$ | $\pm 3.125\text{ MHz}$ at $F_s = 125\text{ MHz}$ |
| **Estimation Accuracy** | $\le \pm 0.005\,R_{sym}$ | $\le \pm 156.25\text{ kHz}$ over SNR 0–20 dB |
| **Algorithm** | Preamble-Assisted Conjugate Auto-Correlation | $P[n] = r[n] r^*[n-L]$, $R = \sum_{n=0}^{M-1} P[n]$ |
| **Preamble Repetition Length ($L$)** | 16 samples (4 symbols) | Power-of-2 ($2^4$) delay |
| **Accumulation Length ($M$)** | Parameterized (Default: 192 samples = 48 symbols) | Configurable |
| **Estimation Latency ($L_{est}$)** | 57 symbols typ (228 clock cycles) | Requirement: 32–128 symbols |
| **Phase Estimator** | 16-stage Pipelined CORDIC Vectoring | $\theta = \operatorname{atan2}(R_Q, R_I)$ |
| **CFO Output Format** | Signed 24-bit Q1.23 | $f_{\text{norm}} = 2\Delta f / F_s \in [-1, 1)$ |
| **NCO Interface** | 32-bit Frequency Tuning Word | `nco_ftw = $signed(coarse_cfo_est) <<< 8;` |
| **Preamble Detection Modes** | Dual-Mode: Autonomous + External Trigger | Mode A: Normalized metric, Mode B: `burst_start`, `preamble_enable` |

---

## 2. Mathematical Algorithm & Derivation

### 2.1 Received Signal Model
In the presence of carrier frequency offset $\Delta f$, the received sample stream at the output of the STR block is:
$$r[n] = s[n] \cdot e^{j \left(2\pi \Delta f \frac{n}{F_s} + \phi_0\right)} + w[n]$$
where $s[n]$ is the periodic training preamble satisfying $s[n] = s[n-L]$ for delay $L = 16$ samples, and $w[n]$ is complex AWGN.

### 2.2 Conjugate Auto-Correlation
Taking the product of $r[n]$ and the complex conjugate of the delayed sample $r[n-L]$:
$$P[n] = r[n] \cdot r^*[n-L] = |s[n]|^2 \cdot e^{j 2\pi \Delta f \frac{L}{F_s}} + \text{noise terms}$$

Decomposing into real and imaginary Cartesian components:
$$P_I[n] = I[n] I[n-L] + Q[n] Q[n-L]$$
$$P_Q[n] = Q[n] I[n-L] - I[n] Q[n-L]$$

### 2.3 Correlation Accumulation
Summing over $M$ valid samples:
$$R = R_I + j R_Q = \sum_{n=0}^{M-1} P[n]$$
$$R_I = \sum_{n=0}^{M-1} P_I[n], \quad R_Q = \sum_{n=0}^{M-1} P_Q[n]$$

The expected value of $R$ is:
$$\mathbb{E}[R] = M \cdot \sigma_s^2 \cdot e^{j \theta}, \quad \text{where } \theta = 2\pi \Delta f \frac{L}{F_s}$$

### 2.4 CFO Unambiguous Range
For unambiguous phase detection with $\operatorname{atan2}(R_Q, R_I) \in (-\pi, +\pi]$:
$$|\theta| = 2\pi |\Delta f| \frac{L}{F_s} < \pi \implies |\Delta f| < \frac{F_s}{2L}$$

With $L = 16$ and $F_s = 4 R_{sym}$:
$$|\Delta f_{\max}| = \frac{4 R_{sym}}{32} = 0.125 R_{sym} = 12.5\% R_{sym}$$

Since the project specification requires $|\Delta f| \le 0.10 R_{sym}$:
$$|\theta_{\max}| = 2\pi \frac{0.10 R_{sym} \times 16}{4 R_{sym}} = 0.8\pi = 144^\circ < 180^\circ$$
This guarantees an unambiguous estimate with a 25% safety margin.

### 2.5 CFO Normalization & Hardware Simplification
The normalized CFO is defined as:
$$f_{\text{norm}} = \frac{2\Delta f}{F_s}$$

From $\theta = 2\pi \Delta f \frac{L}{F_s}$:
$$\frac{2\Delta f}{F_s} = \frac{\theta}{\pi L} \implies f_{\text{norm}} = \frac{\theta}{16\pi}$$

When the CORDIC vectoring engine computes the phase normalized to $\pi$ (i.e., $\theta_{\text{norm}} = \theta / \pi \in [-1, 1)$ in signed Q1.23 format):
$$\boxed{f_{\text{norm}} = \frac{\theta_{\text{norm}}}{16} = \theta_{\text{norm}} \gg 4}$$

**Critical Hardware Advantage:** The CFO calculation requires **no hardware multiplier or divider**; it is performed with an exact 4-bit arithmetic right shift with sign extension!

---

## 3. Top-Level Module Interface

```systemverilog
module coarse_cfo_estimator #(
    parameter int DATA_WIDTH     = 12,       // Input I/Q bit-width (Q1.11)
    parameter int DELAY_L        = 16,       // Preamble repetition delay (samples)
    parameter int ACCUM_M        = 192,      // Default accumulation length (samples)
    parameter int CORDIC_STAGES  = 16,       // CORDIC pipeline depth
    parameter int CFO_WIDTH      = 24        // Output CFO width (Q1.23)
)(
    input  logic                     clk,
    input  logic                     rst_n,

    // Streaming I/Q Interface from STR
    input  logic signed [DATA_WIDTH-1:0] str_out_i,
    input  logic signed [DATA_WIDTH-1:0] str_out_q,
    input  logic                         str_out_valid,

    // External Trigger & Control
    input  logic                     burst_start,      // Resets and arms detector
    input  logic                     preamble_enable,  // External forced estimation trigger
    input  logic                     auto_mode_en,     // 1: Autonomous, 0: External Trigger

    // Threshold Configurations
    input  logic [31:0]              det_metric_thresh, // Metric threshold gamma
    input  logic [23:0]              energy_min_thresh, // Minimum energy threshold

    // Status & Observation (DFT)
    output logic [31:0]              det_metric_out,
    output logic                     preamble_detected,
    output logic                     accum_busy,

    // CFO Output Interface
    output logic signed [CFO_WIDTH-1:0] coarse_cfo_est,
    output logic                        coarse_cfo_valid
);
```

---

## 4. Latency & Timing Budget

At typical operating frequency $F_{clk} = 125\text{ MHz}$ ($T_{clk} = 8\text{ ns}$):

| Processing Stage | Latency (Clock Cycles) | Equivalent Symbols | Cumulative Latency |
| :--- | :---: | :---: | :---: |
| **Input Register & Alignment** | 1 | 0.25 | 0.25 symbols |
| **Delay Line ($L = 16$)** | 16 | 4.0 | 4.25 symbols |
| **Conjugate Multiplier Pipeline** | 2 | 0.5 | 4.75 symbols |
| **Correlation Accumulator ($M = 192$)**| 192 | 48.0 | 52.75 symbols |
| **Accumulator Latch & CORDIC Input** | 1 | 0.25 | 53.0 symbols |
| **16-Stage Pipelined CORDIC** | 16 | 4.0 | 57.0 symbols |
| **CFO Shift Calculator & Output Reg** | 2 | 0.5 | **57.5 symbols (230 cycles)** |

The total estimation latency of **57.5 symbols (230 cycles)** comfortably satisfies the **32 to 128 symbols** requirement and matches the typical 64-symbol specification.
