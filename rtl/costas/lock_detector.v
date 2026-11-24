// Lock detector (supports lock_detect output and gain switching)
// m <= m + (|e| - m)/32  (low-pass of |e|).  Lock after m < T_LOCK for
// LOCK_CNT consecutive symbols; unlock when m > T_UNLOCK (hysteresis).
// Thresholds are Q1.15 "radians" (6500 = 0.198 rad, 10000 = 0.305 rad) and must
// be tuned from your own noise sweep; they assume Es/N0 >= 15 dB at this input.
module lock_detector #(
    parameter [15:0] T_LOCK   = 16'd6500,
    parameter [15:0] T_UNLOCK = 16'd10000,
    parameter [5:0]  LOCK_CNT = 6'd32,
    parameter [15:0] M_INIT   = 16'h7FFF
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               clear,
    input  wire signed [15:0] e,
    input  wire               e_valid,
    output reg                lock,
    output reg         [15:0] metric
);
    reg [5:0] cnt;
    reg [15:0] abs_e_r;
    reg        e_v1;

    // Stage 1: Pipeline absolute value calculation to break 13-level critical path
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            abs_e_r <= 16'd0;
            e_v1    <= 1'b0;
        end else if (clear) begin
            abs_e_r <= 16'd0;
            e_v1    <= 1'b0;
        end else begin
            e_v1 <= e_valid;
            if (e_valid) begin
                abs_e_r <= e[15] ? (~e + 16'd1) : e;
            end
        end
    end

    // Stage 2: Filter update & lock threshold comparison
    wire signed [17:0] diff   = $signed({2'b00, abs_e_r}) - $signed({2'b00, metric});
    wire signed [17:0] step   = diff >>> 5;
    wire signed [17:0] m_next = $signed({2'b00, metric}) + step;
    wire        [15:0] m_n16  = m_next[15:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            metric <= M_INIT; cnt <= 6'd0; lock <= 1'b0;
        end else if (clear) begin
            metric <= M_INIT; cnt <= 6'd0; lock <= 1'b0;
        end else if (e_v1) begin
            metric <= m_n16;
            if (lock) begin
                if (m_n16 > T_UNLOCK) begin
                    lock <= 1'b0;
                    cnt  <= 6'd0;
                end
            end else begin
                if (m_n16 < T_LOCK) begin
                    cnt <= cnt + 6'd1;
                    if ((cnt + 6'd1) >= LOCK_CNT) lock <= 1'b1;
                end else begin
                    cnt <= 6'd0;
                end
            end
        end
    end
endmodule
