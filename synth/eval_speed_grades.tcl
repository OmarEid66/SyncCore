foreach PART {"xc7a35tcsg324-1" "xc7a35tcsg324-2" "xc7a35tcsg324-3"} {
    puts "=========================================================="
    puts "Evaluating Fmax for Part: $PART"
    puts "=========================================================="
    reset_timing
    set_property part $PART [current_project]
    create_clock -period 4.000 -name clk [get_ports clk]
    set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
    set min_period [expr {4.000 - $wns}]
    set fmax [expr {1000.0 / $min_period}]
    puts "--> Part: $PART | WNS (at 250 MHz): $wns ns | Fmax: $fmax MHz"
}
exit
