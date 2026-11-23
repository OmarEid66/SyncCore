# ==============================================================================
# Vivado Non-Project Batch Synthesis & Implementation Script (Stage 6 & 7)
# Target: AMD/Xilinx Artix-7 XC7A35TCSG324-1 @ 200 MHz (5.0 ns)
#
# Run from workspace root:
#   vivado -mode batch -source vivado/run_synth_impl.tcl
# ==============================================================================

set part_name "xc7a35tcsg324-1"
set top_name  "str_top"
set out_dir   "vivado/reports"
file mkdir $out_dir

puts "================================================================"
puts "  STAGE 6: VIVADO SYNTHESIS & IMPLEMENTATION ($part_name @ 200 MHz)"
puts "================================================================"

# Read SystemVerilog RTL in dependency order
read_verilog -sv [list \
  rtl/str_pkg.sv \
  rtl/str_input_buffer.sv \
  rtl/timing_phase_accumulator.sv \
  rtl/farrow_interpolator.sv \
  rtl/timing_sample_scheduler.sv \
  rtl/gardner_ted.sv \
  rtl/timing_pi_controller.sv \
  rtl/str_output_register.sv \
  rtl/str_top.sv \
]

read_xdc vivado/str_200mhz.xdc

# 1. Synthesis (out-of-context mode so I/O buffers do not distort internal subsystem Fmax)
synth_design -top $top_name -part $part_name -mode out_of_context -directive default
write_checkpoint -force $out_dir/post_synth.dcp
report_timing_summary -file $out_dir/post_synth_timing.rpt
report_utilization    -file $out_dir/post_synth_utilization.rpt

# 2. Implementation (Opt -> Place -> PhysOpt -> Route -> Post-Route PhysOpt)
opt_design -directive Explore
place_design -directive Explore
phys_opt_design -directive AggressiveExplore
route_design -directive Explore
phys_opt_design -directive AggressiveExplore
phys_opt_design -hold_fix
write_checkpoint -force $out_dir/post_route.dcp

# 3. Generate Mandatory Stage 6 & 7 Reports
report_timing_summary -delay_type min_max -report_unconstrained -check_timing_verbose -max_paths 10 -input_pins -file $out_dir/post_route_timing_summary.rpt
report_timing -setup -max_paths 10 -sort_by group -input_pins -file $out_dir/post_route_critical_paths.rpt
report_utilization -hierarchical -file $out_dir/post_route_utilization.rpt
report_power -file $out_dir/post_route_power.rpt
report_drc -file $out_dir/post_route_drc.rpt

# Print WNS / TNS summary directly to stdout
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
set whs [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -hold]]
puts "================================================================"
puts "  POST-ROUTE TIMING RESULT ($part_name @ 200 MHz / 5.000 ns):"
puts "    Worst Negative Setup Slack (WNS) : $wns ns"
puts "    Worst Hold Slack           (WHS) : $whs ns"
if {$wns >= 0.0} {
  set fmax [expr {1000.0 / (5.000 - $wns)}]
  puts [format "    Timing Closure : PASSED! Estimated Fmax = %.2f MHz" $fmax]
} else {
  puts "    Timing Closure : FAILED (WNS < 0) - analyze post_route_critical_paths.rpt"
}
puts "================================================================"
