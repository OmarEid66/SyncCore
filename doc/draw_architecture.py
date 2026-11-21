"""
Script: draw_architecture.py
Description: Generates an ultra-high-resolution, publication-grade architectural
             block diagram of the Coarse Carrier Frequency Offset (CFO) Estimator.
Outputs:
  - d:/SynchCore/doc/coarse_cfo_architecture.png
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyBboxPatch, ArrowStyle

def create_diagram():
    fig = plt.figure(figsize=(24, 13.5), dpi=300)
    ax = fig.add_subplot(111)
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.axis('off')

    # Color Palette - Professional Engineering Theme
    c_bg         = '#0B1120'  # Deep Slate Navy
    c_card       = '#1E293B'  # Card Background
    c_border     = '#334155'  # Border
    c_text_main  = '#F8FAFC'  # White-ish
    c_text_sub   = '#94A3B8'  # Slate Gray
    c_accent_bl  = '#38BDF8'  # Sky Blue (Data I/Q)
    c_accent_gr  = '#34D399'  # Emerald Green (Control / Valid)
    c_accent_or  = '#FB923C'  # Amber Orange (Multiplier / Energy)
    c_accent_pu  = '#A78BFA'  # Purple (CORDIC)
    c_accent_cy  = '#2DD4BF'  # Teal (Output / CFO)
    c_accent_rd  = '#F87171'  # Red / Warning / Abort

    fig.patch.set_facecolor(c_bg)
    ax.set_facecolor(c_bg)

    # -------------------------------------------------------------------------
    # Helper Functions
    # -------------------------------------------------------------------------
    def draw_box(x, y, w, h, title, subtitle, tag, color_accent, details=[]):
        # Card shadow
        shadow = FancyBboxPatch((x+0.3, y-0.4), w, h, boxstyle="round,pad=0.5,rounding_size=1.2",
                                facecolor='#030712', edgecolor='none', alpha=0.5, zorder=1)
        ax.add_patch(shadow)
        
        # Main Box
        box = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.5,rounding_size=1.2",
                             facecolor=c_card, edgecolor=color_accent, linewidth=2.0, zorder=2)
        ax.add_patch(box)
        
        # Header banner
        header_h = 2.2
        header_box = FancyBboxPatch((x, y + h - header_h), w, header_h, boxstyle="round,pad=0.2,rounding_size=0.8",
                                    facecolor=color_accent, edgecolor='none', alpha=0.25, zorder=3)
        ax.add_patch(header_box)

        # Title & Subtitle
        ax.text(x + 0.8, y + h - 1.2, title, fontsize=12, fontweight='bold', color=c_text_main, zorder=4)
        ax.text(x + w - 0.8, y + h - 1.2, tag, fontsize=9, fontweight='bold', color=color_accent,
                ha='right', va='center', zorder=4,
                bbox=dict(boxstyle='round,pad=0.2', facecolor='#0F172A', edgecolor=color_accent, lw=1))
        
        ax.text(x + 0.8, y + h - 3.2, subtitle, fontsize=9.5, fontweight='semibold', color=color_accent, zorder=4)

        # Body details
        curr_y = y + h - 5.0
        for item in details:
            ax.text(x + 0.8, curr_y, item, fontsize=8.5, color=c_text_sub, zorder=4)
            curr_y -= 1.6

    def draw_bus(x1, y1, x2, y2, label="", label_pos=0.5, color=c_accent_bl, width=2.0, dashed=False, dy_label=1.0):
        ls = '--' if dashed else '-'
        ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                    arrowprops=dict(arrowstyle="-|>", color=color, lw=width, ls=ls,
                                    mutation_scale=15), zorder=5)
        if label:
            lx = x1 + (x2 - x1) * label_pos
            ly = y1 + (y2 - y1) * label_pos + dy_label
            ax.text(lx, ly, label, fontsize=8.5, fontweight='bold', color=color,
                    ha='center', va='center', zorder=6,
                    bbox=dict(boxstyle='round,pad=0.2', facecolor='#0F172A', edgecolor=color, lw=0.8, alpha=0.9))

    def draw_bus_corner(pts, label="", label_idx=1, color=c_accent_bl, width=2.0, dashed=False, dy_label=1.0):
        ls = '--' if dashed else '-'
        for i in range(len(pts) - 1):
            is_last = (i == len(pts) - 2)
            arr = "-|>" if is_last else "-"
            ax.annotate('', xy=pts[i+1], xytext=pts[i],
                        arrowprops=dict(arrowstyle=arr, color=color, lw=width, ls=ls,
                                        mutation_scale=15), zorder=5)
            if label and i == label_idx:
                p1, p2 = pts[i], pts[i+1]
                lx = 0.5 * (p1[0] + p2[0])
                ly = 0.5 * (p1[1] + p2[1]) + dy_label
                ax.text(lx, ly, label, fontsize=8.5, fontweight='bold', color=color,
                        ha='center', va='center', zorder=6,
                        bbox=dict(boxstyle='round,pad=0.2', facecolor='#0F172A', edgecolor=color, lw=0.8, alpha=0.9))

    # -------------------------------------------------------------------------
    # Top Header & Title Banner
    # -------------------------------------------------------------------------
    title_box = FancyBboxPatch((2, 90), 96, 8, boxstyle="round,pad=0.5,rounding_size=1.0",
                               facecolor='#1E293B', edgecolor='#38BDF8', linewidth=1.5, zorder=2)
    ax.add_patch(title_box)
    ax.text(4, 95.2, "REAL-TIME ALL-DIGITAL COARSE CFO ESTIMATOR ARCHITECTURE",
            fontsize=18, fontweight='bold', color='#FFFFFF', zorder=3)
    ax.text(4, 92.2, "Preamble-Assisted Conjugate Auto-Correlation with 16-Stage Pipelined CORDIC | Artix-7 xc7a35t-1 @ 200 MHz Timing Closure",
            fontsize=10.5, color='#94A3B8', zorder=3)
    
    # Status Badges on Title Bar
    badges = [
        ("Fmax = 201.8 MHz", "#34D399", 74),
        ("WNS = +0.045 ns", "#38BDF8", 82),
        ("DSPs = 12 / 90", "#A78BFA", 90),
        ("BRAM = 0", "#FB923C", 96)
    ]
    for b_text, b_col, b_x in badges:
        ax.text(b_x, 94.0, b_text, fontsize=9.5, fontweight='bold', color=b_col, ha='center',
                bbox=dict(boxstyle='round,pad=0.4', facecolor='#0F172A', edgecolor=b_col, lw=1.2), zorder=4)

    # -------------------------------------------------------------------------
    # 1. INPUT INTERFACE (Left Column)
    # -------------------------------------------------------------------------
    draw_box(2.5, 57, 13.5, 27, "INPUT INTERFACE", "Symbol Timing Recovery (STR)", "4 Samples/Sym", c_accent_bl, [
        "- Clock: 125 MHz typ (200 MHz max)",
        "- In I: str_out_i [11:0] (Q1.11 signed)",
        "- In Q: str_out_q [11:0] (Q1.11 signed)",
        "- In Valid: str_out_valid strobe",
        "- Preamble: ZC Periodic (L = 16)",
        "- Sample Rate: Fs = 4 * Rsym",
        "- Symbol Rate: Rsym = 31.25 MSps",
        "- Zero BRAM overhead"
    ])

    # -------------------------------------------------------------------------
    # 2. DELAY LINE (Upper Mid-Left)
    # -------------------------------------------------------------------------
    draw_box(21, 67, 16.5, 17, "cfo_delay_line", "Valid-Gated Delay (L = 16)", "L = 16 Taps", c_accent_bl, [
        "- 16-Stage Shift Array r[n-16]",
        "- Clock-gated by str_out_valid",
        "- Preserves alignment over gaps",
        "- Shift Array: shift_reg_i/q [0:15]",
        "- Output: del_i [11:0], del_q [11:0]",
        "- FPGA: 100% Slice Registers (0 BRAM)"
    ])

    # -------------------------------------------------------------------------
    # 3. PREAMBLE DETECTOR (Lower Mid-Left)
    # -------------------------------------------------------------------------
    draw_box(21, 23, 16.5, 38, "cfo_preamble_detector", "Dual-Mode Detector Engine", "8 DSP48E1", c_accent_gr, [
        "- Mode A: Autonomous Normalized Metric:",
        "    M[n] = |C[n]|^2 / (E1[n]*E2[n] + eps)",
        "- Mode B: External Trigger (burst_start)",
        "- Input Retiming FFs (AREG absorbed):",
        "    Breaks critical delay-line -> DSP path",
        "- 4-Stage Metric Evaluator Pipeline:",
        "    S1: Squares C_I^2, C_Q^2 & E1*E2",
        "    S2: Add |C|^2 & Truncate Energy",
        "    S3: Threshold Mult ((E1*E2)*gamma)",
        "    S4: Threshold Comparator >= thresh",
        "- Consecutive Persistence: N = 2 wins",
        "- Output: preamble_detected pulse"
    ])

    # -------------------------------------------------------------------------
    # 4. CONJUGATE MULTIPLIER (Center Top)
    # -------------------------------------------------------------------------
    draw_box(42, 67, 16.5, 17, "cfo_conj_multiplier", "2-Stage Complex Conjugate", "4 DSP48E1", c_accent_or, [
        "- P[n] = r[n] * conj(r[n-16])",
        "- P_I = I_n*I_d + Q_n*Q_d  (25b signed)",
        "- P_Q = Q_n*I_d - I_n*Q_d  (25b signed)",
        "- S1: 4x DSP 12x12 Mults (AREG, BREG)",
        "- S2: Pipelined Add/Sub (PREG)",
        "- Headroom: 24b -> 25b prevents overflow",
        "- Timing: Closes > 200 MHz cleanly"
    ])

    # -------------------------------------------------------------------------
    # 5. CORRELATION ACCUMULATOR (Center Bottom)
    # -------------------------------------------------------------------------
    draw_box(42, 23, 16.5, 38, "cfo_accumulator", "34-bit Correlation Accumulator", "M = 192 Samples", c_accent_or, [
        "- Accumulates M = 192 samples (48 syms)",
        "- Real: R_I = sum(P_I[n]) [33:0]",
        "- Imag: R_Q = sum(P_Q[n]) [33:0]",
        "- Bit Growth: log2(192) = 8 bits",
        "- Accumulator Width: 25b + 9b = 34 bits",
        "- Zero overflow up to M = 512 samples",
        "- Start: triggered by det_pulse",
        "- Abort: clean return on burst_start",
        "- Gated by prod_valid streaming flag",
        "- Done: asserts accum_complete pulse"
    ])

    # -------------------------------------------------------------------------
    # 6. CORDIC ENGINE (Mid-Right)
    # -------------------------------------------------------------------------
    draw_box(63, 45, 16.5, 39, "cordic_atan2", "16-Stage Vectoring Engine", "W = 36 bits", c_accent_pu, [
        "- Computes theta = atan2(R_Q, R_I)",
        "- Full 4-Quadrant Pre-Rotation:",
        "    Maps all angles into [-pi, +pi)",
        "- 16 Pipelined Shift-Add Iterations:",
        "    x[i+1] = x[i] - d_i * (y[i] >> i)",
        "    y[i+1] = y[i] + d_i * (x[i] >> i)",
        "    z[i+1] = z[i] - d_i * atan(2^-i)",
        "- Datapath Width W = 36 bits",
        "- Growth Factor K = 1.647 absorbed",
        "- Output: theta_out [23:0] (Q1.23)",
        "- Angle scaling: pi = 2^23",
        "- Latency: 16 clock cycles"
    ])

    # -------------------------------------------------------------------------
    # 7. CFO CALCULATOR (Far Right Top)
    # -------------------------------------------------------------------------
    draw_box(84, 55, 14, 29, "cfo_calculator", "Zero-DSP Right Shift Calc", "0 DSPs (Shift)", c_accent_cy, [
        "- Math Derivation:",
        "    f_norm = theta / (16 * pi)",
        "    f_norm = (theta / pi) >>> 4",
        "- Implementation:",
        "    coarse_cfo_est = theta >>> 4",
        "- Output Format: Signed Q1.23",
        "- Downstream NCO FTW:",
        "    nco_ftw = coarse_cfo_est <<< 8",
        "- 32-bit Phase Accumulator FTW",
        "- Latency: 1 cycle registered",
        "- Zero Logic Multipliers!"
    ])

    # -------------------------------------------------------------------------
    # 8. OUTPUT INTERFACE (Far Right Bottom)
    # -------------------------------------------------------------------------
    draw_box(84, 23, 14, 27, "OUTPUT INTERFACE", "Downstream Carrier Correction", "Synthesized", c_accent_cy, [
        "- coarse_cfo_est [23:0] (Q1.23)",
        "- coarse_cfo_valid (1-cycle strobe)",
        "- nco_ftw [31:0] (Signed 32-bit)",
        "- Capture: +/- 0.10 * Rsym",
        "- Accuracy: <= +/- 0.005 * Rsym",
        "- Measured Err: < 2.53 kHz @ 15dB",
        "- Typical Latency: 57.5 symbols",
        "- Status: Ready for Derotator"
    ])

    # -------------------------------------------------------------------------
    # 9. CONTROL FSM & SUPERVISOR (Center Bottom Bar)
    # -------------------------------------------------------------------------
    fsm_box = FancyBboxPatch((2.5, 3.5), 95.5, 14, boxstyle="round,pad=0.5,rounding_size=1.0",
                             facecolor='#1E293B', edgecolor='#F87171', linewidth=1.5, zorder=2)
    ax.add_patch(fsm_box)
    ax.text(4.5, 14.5, "TOP-LEVEL CONTROL FINITE STATE MACHINE (FSM) & DFT INTERFACE",
            fontsize=12, fontweight='bold', color='#FFFFFF', zorder=3)
    ax.text(4.5, 12.5, "Central Supervisory Controller: Orchestrates Reset, Search, Accumulation, Vectoring, Multi-Burst & Abort Recovery",
            fontsize=9.5, color='#94A3B8', zorder=3)

    # FSM State Bubbles
    states = [
        ("STATE_IDLE\n(Wait Reset)", 12, "#64748B"),
        ("STATE_SEARCHING\n(Preamble Hunt)", 30, "#38BDF8"),
        ("STATE_ACCUM\n(M = 192 samples)", 50, "#FB923C"),
        ("STATE_CORDIC\n(16 Iterations)", 70, "#A78BFA"),
        ("STATE_DONE\n(Latch & Export)", 88, "#34D399")
    ]
    for s_name, s_x, s_col in states:
        s_circ = FancyBboxPatch((s_x - 6, 5.0), 12, 5.5, boxstyle="round,pad=0.3,rounding_size=0.8",
                                facecolor='#0F172A', edgecolor=s_col, lw=1.5, zorder=4)
        ax.add_patch(s_circ)
        ax.text(s_x, 7.8, s_name, fontsize=8.5, fontweight='bold', color=s_col, ha='center', va='center', zorder=5)

    # FSM Transition Arrows
    for idx in range(len(states)-1):
        x_from = states[idx][1] + 6.2
        x_to   = states[idx+1][1] - 6.2
        ax.annotate('', xy=(x_to, 7.8), xytext=(x_from, 7.8),
                    arrowprops=dict(arrowstyle="-|>", color='#E2E8F0', lw=1.5, mutation_scale=12), zorder=6)
    
    # Reset / Abort back loop
    ax.annotate('', xy=(30, 10.7), xytext=(88, 10.7),
                arrowprops=dict(arrowstyle="-|>", color='#F87171', lw=1.2, ls='--',
                                connectionstyle="arc3,rad=-0.12", mutation_scale=12), zorder=6)
    ax.text(59, 12.0, "burst_start (Mid-stream Abort & Dynamic Re-Arm)", fontsize=8, color='#F87171',
            ha='center', va='center', zorder=7, bbox=dict(boxstyle='round,pad=0.2', facecolor='#0F172A', edgecolor='#F87171', lw=0.8))

    # -------------------------------------------------------------------------
    # SIGNAL ROUTING & INTERCONNECT BUSES
    # -------------------------------------------------------------------------

    # 1. Input str_out_i/q -> Delay Line
    draw_bus_corner([(16.5, 75.5), (18.5, 75.5), (18.5, 75.5), (20.5, 75.5)],
                    label="r[n] [11:0]", label_idx=1, color=c_accent_bl, width=2.5, dy_label=1.2)

    # 2. Input str_out_i/q -> Conjugate Multiplier (Split top branch)
    draw_bus_corner([(18.5, 75.5), (18.5, 87), (39, 87), (39, 78), (41.5, 78)],
                    label="Current r[n] [11:0]", label_idx=2, color=c_accent_bl, width=2.0, dy_label=1.2)

    # 3. Input str_out_i/q -> Preamble Detector
    draw_bus_corner([(18.5, 75.5), (18.5, 42), (20.5, 42)],
                    label="r[n] [11:0]", label_idx=1, color=c_accent_bl, width=2.0, dy_label=-1.4)

    # 4. Delay Line -> Conjugate Multiplier
    draw_bus(38.0, 72.0, 41.5, 72.0, label="r[n-16] [11:0]", color=c_accent_bl, width=2.5, dy_label=1.2)

    # 5. Delay Line -> Preamble Detector
    draw_bus_corner([(37.5, 70), (39.5, 70), (39.5, 52), (38.0, 52)],
                    label="del_i/q [11:0]", label_idx=1, color=c_accent_bl, width=2.0, dy_label=1.2)

    # 6. Conjugate Multiplier -> Accumulator
    draw_bus(50.25, 66.5, 50.25, 61.5, label="prod_i/q [24:0]", color=c_accent_or, width=2.5, dy_label=0.0)

    # 7. Multiplier -> Detector (prod_valid & prod_i/q)
    draw_bus_corner([(42.0, 70), (40.5, 70), (40.5, 36), (38.0, 36)],
                    label="prod_vld", label_idx=1, color=c_accent_gr, width=1.8, dy_label=1.2)

    # 8. Detector -> FSM / Accumulator (preamble_detected / det_pulse)
    draw_bus_corner([(38.0, 30), (40.0, 30), (40.0, 30), (41.5, 30)],
                    label="det_pulse", label_idx=1, color=c_accent_gr, width=2.5, dy_label=1.2)

    # 9. Accumulator -> CORDIC Engine
    draw_bus_corner([(59.0, 40), (61.0, 40), (61.0, 60), (62.5, 60)],
                    label="accum_out_i/q [33:0]", label_idx=1, color=c_accent_or, width=2.5, dy_label=1.2)

    # 10. Accumulator Complete -> CORDIC (valid_in)
    draw_bus_corner([(59.0, 48), (61.0, 48), (61.0, 52), (62.5, 52)],
                    label="accum_complete", label_idx=1, color=c_accent_gr, width=1.8, dy_label=1.2)

    # 11. CORDIC -> CFO Calculator
    draw_bus_corner([(80.0, 66), (82.0, 66), (82.0, 72), (83.5, 72)],
                    label="theta_out [23:0]", label_idx=1, color=c_accent_pu, width=2.5, dy_label=1.2)
    draw_bus_corner([(80.0, 60), (82.0, 60), (82.0, 64), (83.5, 64)],
                    label="cordic_valid", label_idx=1, color=c_accent_gr, width=1.8, dy_label=1.2)

    # 12. CFO Calculator -> Output Interface
    draw_bus_corner([(91.0, 54.5), (91.0, 52.5)],
                    label="coarse_cfo_est [23:0] & nco_ftw [31:0]", label_idx=0, color=c_accent_cy, width=2.5, dy_label=0.0)

    # Save Figure
    out_path = "d:/SynchCore/doc/coarse_cfo_architecture.png"
    plt.tight_layout()
    plt.savefig(out_path, dpi=300, facecolor=fig.get_facecolor(), edgecolor='none', bbox_inches='tight')
    plt.close()
    print(f"Architecture diagram successfully saved to: {out_path}")

if __name__ == "__main__":
    create_diagram()
