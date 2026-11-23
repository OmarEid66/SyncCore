# ==============================================================================
# Timing Constraints for High-Performance Symbol Timing Recovery (STR)
# Target Device: AMD/Xilinx Artix-7 XC7A35TCSG324-1
# Target Clock : 200.0 MHz (Period = 5.000 ns)
# ==============================================================================

create_clock -name sys_clk -period 5.000 -waveform {0.000 2.500} [get_ports clk]

# OOC clock-source hint: tells Vivado which BUFG to use for clock skew estimation
# so hold analysis in out-of-context mode is realistic (see WARNING Timing 38-242)
set_property HD.CLK_SRC BUFGCTRL_X0Y0 [get_ports clk]

# Clock uncertainty (realistic Artix-7 MMCM/PLL jitter + margin)
set_clock_uncertainty 0.150 [get_clocks sys_clk]

# Out-of-context / subsystem-level I/O timing budgets (0.8 ns external allowance)
set_input_delay  -clock sys_clk -max 0.800 [get_ports {in_sample_i[*] in_sample_q[*] in_sample_valid}]
set_input_delay  -clock sys_clk -min 0.200 [get_ports {in_sample_i[*] in_sample_q[*] in_sample_valid}]
set_output_delay -clock sys_clk -max 0.800 [get_ports {out_sym_i[*] out_sym_q[*] out_sym_valid}]
set_output_delay -clock sys_clk -min 0.000 [get_ports {out_sym_i[*] out_sym_q[*] out_sym_valid}]

# Asynchronous active-low reset input
set_false_path -from [get_ports rst_n]
