# ==============================================================================
# QuestaSim Script: run_full_sim.do
# Full Synchronization Subsystem (STR + Coarse CFO + Derotator + Costas Loop)
# ==============================================================================
vlib work
vlog -sv ../rtl/str_pkg.sv \
         ../rtl/str_input_buffer.sv \
         ../rtl/timing_phase_accumulator.sv \
         ../rtl/farrow_interpolator.sv \
         ../rtl/str_output_register.sv \
         ../rtl/timing_sample_scheduler.sv \
         ../rtl/gardner_ted.sv \
         ../rtl/timing_pi_controller.sv \
         ../rtl/str_top.sv \
         ../rtl/cfo_delay_line.sv \
         ../rtl/cfo_conj_multiplier.sv \
         ../rtl/cfo_accumulator.sv \
         ../rtl/cfo_preamble_detector.sv \
         ../rtl/cordic_atan2.sv \
         ../rtl/cfo_calculator.sv \
         ../rtl/coarse_cfo_estimator_top.sv \
         ../rtl/coarse_derotator.sv \
         ../rtl/costas/*.v \
         ../rtl/sync_subsystem_top.sv \
         ../tb/tb_sync_subsystem_top.sv

vsim -c -t 1ps work.tb_sync_subsystem_top -do "run -all; quit -f"
