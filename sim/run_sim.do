vlib work
vlog -sv ../rtl/cfo_delay_line.sv
vlog -sv ../rtl/cfo_conj_multiplier.sv
vlog -sv ../rtl/cfo_accumulator.sv
vlog -sv ../rtl/cfo_preamble_detector.sv
vlog -sv ../rtl/cordic_atan2.sv
vlog -sv ../rtl/cfo_calculator.sv
vlog -sv ../rtl/coarse_cfo_estimator_top.sv
vlog -sv ../tb/tb_coarse_cfo_estimator.sv
vsim -c -t 1ps -voptargs="+acc" work.tb_coarse_cfo_estimator
run -all
exit
