set PART "xc7a35tcsg324-1"

# Read all RTL files
read_verilog -sv ../rtl/cfo_delay_line.sv
read_verilog -sv ../rtl/cfo_conj_multiplier.sv
read_verilog -sv ../rtl/cfo_accumulator.sv
read_verilog -sv ../rtl/cfo_preamble_detector.sv
read_verilog -sv ../rtl/cordic_atan2.sv
read_verilog -sv ../rtl/cfo_calculator.sv
read_verilog -sv ../rtl/coarse_cfo_estimator_top.sv
read_verilog -sv ../rtl/coarse_derotator.sv
read_verilog -sv ../rtl/carrier_sync_top.sv

read_verilog ../rtl/costas/input_interface.v
read_verilog ../rtl/costas/sincos_gen.v
read_verilog ../rtl/costas/complex_rotator.v
read_verilog ../rtl/costas/nco_phase_acc.v
read_verilog ../rtl/costas/decision_slicer.v
read_verilog ../rtl/costas/phase_error_detector.v
read_verilog ../rtl/costas/lock_detector.v
read_verilog ../rtl/costas/loop_filter.v
read_verilog ../rtl/costas/nco_freq_ctrl.v
read_verilog ../rtl/costas/output_interface.v
read_verilog ../rtl/costas/costas_loop_top.v

# Synthesize design out-of-context
synth_design -top carrier_sync_top -part $PART -mode out_of_context

# Constrain clock to 125 MHz (8.0 ns period)
create_clock -period 8.000 -name clk [get_ports clk]

# Run logic optimization and physical placement for realistic routed timing
opt_design
place_design

# Generate reports
report_timing_summary -delay_type min_max -max_paths 10 -report_unconstrained -check_timing_verbose -file carrier_sync_timing_summary.rpt
report_utilization -file carrier_sync_utilization.rpt

# Query timing slack
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]

set fid [open "carrier_sync_synth_results.txt" "w"]
puts $fid "FPGA Part: $PART"
puts $fid "Clock Constraint Period: 8.000 ns (125.0 MHz)"
puts $fid "WNS: $wns ns"

if {$wns >= 0} {
    set fmax [expr {1000.0 / (8.000 - $wns)}]
    puts $fid "TIMING_MET: YES"
    puts $fid "FMAX_MHZ: $fmax"
} else {
    set min_period [expr {8.000 - $wns}]
    set fmax [expr {1000.0 / $min_period}]
    puts $fid "TIMING_MET: NO"
    puts $fid "FMAX_MHZ: $fmax"
}
close $fid

puts "========================================================================"
puts "Vivado Synthesis & Timing Closure Analysis Completed"
puts "========================================================================"
exit
