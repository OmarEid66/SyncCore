set PART "xc7a35tcsg324-1"

# 1. Read STR package and RTL
read_verilog -sv ../rtl/str_pkg.sv
read_verilog -sv ../rtl/str_input_buffer.sv
read_verilog -sv ../rtl/timing_phase_accumulator.sv
read_verilog -sv ../rtl/farrow_interpolator.sv
read_verilog -sv ../rtl/str_output_register.sv
read_verilog -sv ../rtl/timing_sample_scheduler.sv
read_verilog -sv ../rtl/gardner_ted.sv
read_verilog -sv ../rtl/timing_pi_controller.sv
read_verilog -sv ../rtl/str_top.sv

# 2. Read Coarse CFO estimator RTL
read_verilog -sv ../rtl/cfo_delay_line.sv
read_verilog -sv ../rtl/cfo_conj_multiplier.sv
read_verilog -sv ../rtl/cfo_accumulator.sv
read_verilog -sv ../rtl/cfo_preamble_detector.sv
read_verilog -sv ../rtl/cordic_atan2.sv
read_verilog -sv ../rtl/cfo_calculator.sv
read_verilog -sv ../rtl/coarse_cfo_estimator_top.sv

# 3. Read Costas Loop and Shared Math RTL
read_verilog ../rtl/costas/sincos_gen.v
read_verilog ../rtl/costas/complex_rotator.v
read_verilog ../rtl/costas/input_interface.v
read_verilog ../rtl/costas/nco_phase_acc.v
read_verilog ../rtl/costas/decision_slicer.v
read_verilog ../rtl/costas/phase_error_detector.v
read_verilog ../rtl/costas/lock_detector.v
read_verilog ../rtl/costas/loop_filter.v
read_verilog ../rtl/costas/nco_freq_ctrl.v
read_verilog ../rtl/costas/output_interface.v
read_verilog ../rtl/costas/costas_loop_top.v

# 4. Read Coarse Derotator and Integrated Top
read_verilog -sv ../rtl/coarse_derotator.sv
read_verilog -sv ../rtl/sync_subsystem_top.sv

# 5. Synthesize design out-of-context
synth_design -top sync_subsystem_top -part $PART -mode out_of_context

# 6. Constrain clock to 125 MHz (8.0 ns period)
create_clock -period 8.000 -name clk [get_ports clk]

# 7. Run logic optimization and physical placement for realistic routed timing
opt_design
place_design

# 8. Generate reports
report_timing_summary -delay_type min_max -max_paths 10 -report_unconstrained -check_timing_verbose -file full_sync_timing_summary.rpt
report_utilization -file full_sync_utilization.rpt

# 9. Query timing slack and output summary
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]

set fid [open "full_sync_synth_results.txt" "w"]
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
puts "Vivado Full Subsystem Synthesis & Timing Closure Analysis Completed"
puts "========================================================================"
exit
