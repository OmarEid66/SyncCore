// Self-checking testbench for costas_loop_top.
// Generates a QPSK / 16-QAM stream (one symbol every 4 clocks) with a residual
// CFO, a start phase and AWGN, then reports lock time, residual CFO and EVM.
// Run:  iverilog -g2005 -o sim ../rtl/*.v tb_costas_loop.v && vvp sim
// Override with defines, e.g.:  iverilog -DMOD_V=1 -DCFO_V=0.0 -DSNR_V=15.0 ...
`timescale 1ns/1ps
`ifndef MOD_V
 `define MOD_V 0
`endif
`ifndef CFO_V
 `define CFO_V 0.005
`endif
`ifndef PH_V
 `define PH_V 60.0
`endif
`ifndef SNR_V
 `define SNR_V 20.0
`endif
`ifndef GAIN_V
 `define GAIN_V 157
`endif
module tb_costas_loop;
    parameter integer MOD_TYPE = `MOD_V;        // 0 = QPSK, 1 = 16-QAM
    parameter real    CFO      = `CFO_V;    // residual offset, cycles per symbol (x R_sym)
    parameter real    PH0_DEG  = `PH_V;     // starting phase
    parameter real    SNR_DB   = `SNR_V;     // Es/N0 at the Costas input
    parameter integer GAIN     = `GAIN_V;      // tracking gain (Bn ~ 0.03)
    parameter integer N_SYM    = 3000;
    parameter integer TAIL     = 1000;     // EVM window (last TAIL symbols)

    localparam real PI = 3.14159265358979;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #4 clk = ~clk;                  // 125 MHz

    reg  signed [15:0] x_i, x_q;
    reg                x_valid;
    wire signed [15:0] y_i, y_q;
    wire               y_valid, lock_detect;
    wire signed [31:0] fine_cfo_est;
    wire signed [15:0] dft_phase_err;
    wire        [31:0] dft_nco_phase;
    wire signed [31:0] dft_nco_freq;
    wire        [15:0] dft_lock_metric;

    costas_loop_top dut (
        .clk(clk), .rst_n(rst_n), .loop_clear(1'b0),
        .x_i(x_i), .x_q(x_q), .x_valid(x_valid),
        .mod_type(MOD_TYPE[1:0]), .costas_loop_gain(GAIN[7:0]),
        .force_en(1'b0), .force_freq(32'sd0), .freeze(1'b0), .bypass(1'b0),
        .phase_adj(32'd0), .adj_valid(1'b0),
        .y_i(y_i), .y_q(y_q), .y_valid(y_valid),
        .fine_cfo_est(fine_cfo_est), .lock_detect(lock_detect),
        .dft_phase_err(dft_phase_err), .dft_nco_phase(dft_nco_phase),
        .dft_nco_freq(dft_nco_freq), .dft_lock_metric(dft_lock_metric)
    );

    // ---------------- stimulus ----------------
    integer seed = 12345;
    real pts_i [0:N_SYM-1];
    real pts_q [0:N_SYM-1];

    function real urand;
        input dummy;
        reg [31:0] r;
        begin
            r = $random(seed);
            urand = r / 4294967296.0;
        end
    endfunction

    function real gauss;
        input dummy;
        integer k;
        real s;
        begin
            s = 0.0;
            for (k = 0; k < 12; k = k + 1) s = s + urand(0);
            gauss = s - 6.0;
        end
    endfunction

    function real level;                  // random constellation level per axis
        input dummy;
        integer r;
        begin
            if (MOD_TYPE == 0) begin
                level = (urand(0) < 0.5) ? -0.70710678 : 0.70710678;
            end else begin
                r = $rtoi(urand(0) * 4.0);
                if (r > 3) r = 3;
                case (r)
                    0: level = -3.0 / 3.16227766;
                    1: level = -1.0 / 3.16227766;
                    2: level =  1.0 / 3.16227766;
                    default: level = 3.0 / 3.16227766;
                endcase
            end
        end
    endfunction

    function signed [15:0] to_q15;
        input real v;
        integer t;
        begin
            t = $rtoi(v * 32767.0 * 0.999 + ((v >= 0.0) ? 0.5 : -0.5));
            if (t >  32767) t =  32767;
            if (t < -32767) t = -32767;
            to_q15 = t[15:0];
        end
    endfunction

    integer n, c;
    real sigma, ph, di, dq, ri, rq;

    task tick;
        begin
            @(posedge clk);
            #1;
        end
    endtask

    initial begin
        x_i = 0; x_q = 0; x_valid = 1'b0;
        sigma = $sqrt($pow(10.0, -SNR_DB / 10.0) / 2.0);
        repeat (10) tick;
        rst_n = 1'b1;
        repeat (10) tick;
        for (n = 0; n < N_SYM; n = n + 1) begin
            di = level(0); dq = level(0);
            pts_i[n] = di; pts_q[n] = dq;
            ph = 2.0 * PI * CFO * n + PH0_DEG * PI / 180.0;
            ri = di * $cos(ph) - dq * $sin(ph) + sigma * gauss(0);
            rq = di * $sin(ph) + dq * $cos(ph) + sigma * gauss(0);
            x_i = to_q15(ri); x_q = to_q15(rq); x_valid = 1'b1;
            tick;
            x_valid = 1'b0;
            for (c = 0; c < 3; c = c + 1) tick;   // 4 clocks per symbol
        end
        repeat (40) tick;
        report;
        $finish;
    end

    // ---------------- capture + metrics ----------------
    integer k_out = 0;
    integer lock_sym = -1;
    real err0 = 0.0, err1 = 0.0, err2 = 0.0, err3 = 0.0, pwr = 0.0;
    real yi_r, yq_r;

    always @(posedge clk) begin
        if (y_valid) begin
            if (lock_detect && lock_sym < 0) lock_sym = k_out;
            if (k_out >= N_SYM - TAIL && k_out < N_SYM) begin
                yi_r = y_i / 32768.0;
                yq_r = y_q / 32768.0;
                err0 = err0 + (yi_r - pts_i[k_out])**2 + (yq_r - pts_q[k_out])**2;
                err1 = err1 + ( yq_r - pts_i[k_out])**2 + (-yi_r - pts_q[k_out])**2;
                err2 = err2 + (-yi_r - pts_i[k_out])**2 + (-yq_r - pts_q[k_out])**2;
                err3 = err3 + (-yq_r - pts_i[k_out])**2 + ( yi_r - pts_q[k_out])**2;
                pwr  = pwr + pts_i[k_out]**2 + pts_q[k_out]**2;
            end
            k_out = k_out + 1;
        end
    end

    task report;
        real best, evm, floor_evm, cfo_est, cfo_err, lim;
        reg  ok;
        begin
            best = err0;
            if (err1 < best) best = err1;
            if (err2 < best) best = err2;
            if (err3 < best) best = err3;
            evm       = 100.0 * $sqrt(best / pwr);
            floor_evm = 100.0 * $pow(10.0, -SNR_DB / 20.0);   // pure channel noise
            cfo_est   = fine_cfo_est / 4294967296.0;
            cfo_err   = (cfo_est > CFO) ? (cfo_est - CFO) : (CFO - cfo_est);
            lim       = (MOD_TYPE == 0) ? 1.5 : 5.0;
            ok = (lock_sym >= 0) && (lock_sym <= 200) && (cfo_err < 0.01) && (evm - floor_evm < lim);
            $display("---- Costas loop test: MOD=%0d CFO=%f PH0=%f SNR=%f gain=%0d ----",
                     MOD_TYPE, CFO, PH0_DEG, SNR_DB, GAIN);
            $display("symbols out : %0d of %0d", k_out, N_SYM);
            $display("lock symbol : %0d  (limit 200)", lock_sym);
            $display("fine CFO    : est %f  true %f  err %f R_sym  (limit 0.01)", cfo_est, CFO, cfo_err);
            $display("EVM (last %0d): %f %%  (channel noise floor %f %%, excess %f pp)",
                     TAIL, evm, floor_evm, evm - floor_evm);
            if (ok) $display("RESULT: PASS"); else $display("RESULT: FAIL");
        end
    endtask
endmodule
