# ==============================================================================
# Report Timing at 125 MHz (Checkpoint 7) and 160 MHz on Placed & Routed Design
# ==============================================================================
open_checkpoint vivado/reports/post_route.dcp

# --- 1. Evaluate at 125 MHz (8.000 ns) - Checkpoint 7 Requirement ---
create_clock -name sys_clk -period 8.000 -waveform {0.000 4.000} [get_ports clk]
set_input_delay  -clock sys_clk -max 1.200 [get_ports {in_sample_i[*] in_sample_q[*] in_sample_valid}]
set_input_delay  -clock sys_clk -min 1.500 [get_ports {in_sample_i[*] in_sample_q[*] in_sample_valid}]
set_output_delay -clock sys_clk -max 1.200 [get_ports {out_sym_i[*] out_sym_q[*] out_sym_valid}]
set_output_delay -clock sys_clk -min 0.000 [get_ports {out_sym_i[*] out_sym_q[*] out_sym_valid}]

report_timing_summary -delay_type min_max -max_paths 10 -file vivado/reports/timing_125mhz_summary.rpt

set wns_125 [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
set whs_125 [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -hold]]
puts "================================================================"
puts [format "  125 MHz (Checkpoint 7): WNS = %.3f ns | WHS = %.3f ns" $wns_125 $whs_125]
puts "================================================================"

# --- 2. Evaluate at 160 MHz (6.250 ns) ---
create_clock -name sys_clk -period 6.250 -waveform {0.000 3.125} [get_ports clk]
report_timing_summary -delay_type min_max -max_paths 10 -file vivado/reports/timing_160mhz_summary.rpt

set wns_160 [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
set whs_160 [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -hold]]
puts "================================================================"
puts [format "  160 MHz (High-Perf)    : WNS = %.3f ns | WHS = %.3f ns" $wns_160 $whs_160]
puts "================================================================"

exit
