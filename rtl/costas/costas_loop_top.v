// Costas loop (fine CFO + phase tracking) - top level
//
//   x -> [1 input_interface] -> [4 complex_rotator] -> [9 output_interface] -> y
//                                    ^      |
//          [3 sincos_gen] <- [2 nco_phase_acc]     v
//                                    ^        [5 decision_slicer]
//                                    |             v
//          [8 nco_freq_ctrl] <- [7 loop_filter] <- [6 phase_error_detector]
//                                    ^                  |
//                               gain select <- [lock_detector]
//
// One x_valid strobe per symbol.  Loop delay is 3 symbols when strobes are 4
// clocks apart (symbol rate = clk/4).  Forward latency x_valid -> out_valid = 4 clocks.
module costas_loop_top #(
    parameter [7:0]  ACQ_GAIN = 8'd255,          // gain while unlocked (Bn ~ 0.05)
    parameter [15:0] T_LOCK   = 16'd6500,
    parameter [15:0] T_UNLOCK = 16'd10000,
    parameter        ROM_FILE = "sine_quarter.mem"
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               loop_clear,        // start of burst: clears NCO, integrator, lock
    // data in (from STR / coarse CFO)
    input  wire signed [15:0] x_i,
    input  wire signed [15:0] x_q,
    input  wire               x_valid,
    // config
    input  wire        [1:0]  mod_type,          // 00 = QPSK, 01 = 16-QAM
    input  wire        [7:0]  costas_loop_gain,  // tracking gain g (106..255 <-> Bn 0.02..0.05)
    // DFT force controls (tie to 0 in normal operation)
    input  wire               force_en,
    input  wire signed [31:0] force_freq,
    input  wire               freeze,            // open loop
    input  wire               bypass,            // derotator pass-through
    // optional phase ambiguity fix (tie to 0 if unused)
    input  wire        [31:0] phase_adj,
    input  wire               adj_valid,
    // data out
    output wire signed [15:0] y_i,
    output wire signed [15:0] y_q,
    output wire               y_valid,
    output wire signed [31:0] fine_cfo_est,
    output wire               lock_detect,
    // DFT observation taps
    output wire signed [15:0] dft_phase_err,
    output wire        [31:0] dft_nco_phase,
    output wire signed [31:0] dft_nco_freq,
    output wire        [15:0] dft_lock_metric
);
    // 1 input interface
    wire signed [15:0] xr_i, xr_q;
    wire               xr_valid;
    input_interface u_in (.clk(clk), .rst_n(rst_n), .x_i(x_i), .x_q(x_q), .x_valid(x_valid),
                          .xr_i(xr_i), .xr_q(xr_q), .xr_valid(xr_valid));

    // 2 NCO phase accumulator, 3 sine/cosine
    wire [31:0]        phase_inc, phase;
    wire               inc_valid;
    wire signed [15:0] cos_w, sin_w;
    nco_phase_acc u_nco (.clk(clk), .rst_n(rst_n), .clear(loop_clear),
                         .phase_inc(phase_inc), .inc_valid(inc_valid),
                         .phase_adj(phase_adj), .adj_valid(adj_valid),
                         .bypass(bypass), .phase_out(phase));
    sincos_gen #(.ROM_FILE(ROM_FILE)) u_sc (.clk(clk), .phase(phase), .cos_o(cos_w), .sin_o(sin_w));

    // 4 complex rotator
    wire signed [15:0] ry_i, ry_q;
    wire               ry_valid;
    complex_rotator u_rot (.clk(clk), .rst_n(rst_n), .xi(xr_i), .xq(xr_q), .in_valid(xr_valid),
                           .cos_i(cos_w), .sin_i(sin_w),
                           .yi(ry_i), .yq(ry_q), .out_valid(ry_valid));

    // 5 decision / slicer
    wire signed [15:0] sl_yi, sl_yq, dn_i, dn_q;
    wire               sl_valid;
    decision_slicer u_sl (.clk(clk), .rst_n(rst_n), .yi(ry_i), .yq(ry_q), .y_valid(ry_valid),
                          .mod_type(mod_type),
                          .yi_d(sl_yi), .yq_d(sl_yq), .dn_i(dn_i), .dn_q(dn_q), .out_valid(sl_valid));

    // 6 phase error detector
    wire signed [15:0] e;
    wire               e_valid;
    phase_error_detector u_ped (.clk(clk), .rst_n(rst_n), .yi(sl_yi), .yq(sl_yq),
                                .dn_i(dn_i), .dn_q(dn_q), .in_valid(sl_valid),
                                .e(e), .e_valid(e_valid));

    // lock detector + gain select (acquire at ACQ_GAIN, track at costas_loop_gain)
    wire        lock;
    wire [15:0] lock_metric;
    lock_detector #(.T_LOCK(T_LOCK), .T_UNLOCK(T_UNLOCK)) u_lk
        (.clk(clk), .rst_n(rst_n), .clear(loop_clear), .e(e), .e_valid(e_valid),
         .lock(lock), .metric(lock_metric));

    // Register gain select to break combinational timing path into loop filter multipliers
    reg [7:0] g_sel_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            g_sel_r <= ACQ_GAIN;
        else if (loop_clear)
            g_sel_r <= ACQ_GAIN;
        else
            g_sel_r <= lock ? costas_loop_gain : ACQ_GAIN;
    end

    // 7 loop filter
    wire signed [31:0] integ, vp;
    wire               lf_valid;
    loop_filter u_lf (.clk(clk), .rst_n(rst_n), .clear(loop_clear), .e(e), .e_valid(e_valid),
                      .gain(g_sel_r), .freeze(freeze),
                      .integ(integ), .vp(vp), .out_valid(lf_valid));

    // 8 NCO frequency control
    wire signed [31:0] fine_cfo_w;
    nco_freq_ctrl u_fc (.clk(clk), .rst_n(rst_n), .integ(integ), .vp(vp), .in_valid(lf_valid),
                        .force_en(force_en), .force_freq(force_freq),
                        .phase_inc(phase_inc), .inc_valid(inc_valid), .fine_cfo_est(fine_cfo_w));

    // 9 output interface
    output_interface u_out (.clk(clk), .rst_n(rst_n), .y_i(ry_i), .y_q(ry_q), .y_valid(ry_valid),
                            .lock_i(lock), .fine_cfo_i(fine_cfo_w),
                            .out_i(y_i), .out_q(y_q), .out_valid(y_valid),
                            .lock_detect(lock_detect), .fine_cfo_est(fine_cfo_est));

    assign dft_phase_err   = e;
    assign dft_nco_phase   = phase;
    assign dft_nco_freq    = integ;
    assign dft_lock_metric = lock_metric;
endmodule
