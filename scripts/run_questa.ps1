# Run Stage 5 Bit-Exact RTL Verification in QuestaSim 2021.1
$ErrorActionPreference = "Stop"
$vlib = "C:\questasim64_2021.1\win64\vlib.exe"
$vlog = "C:\questasim64_2021.1\win64\vlog.exe"
$vsim = "C:\questasim64_2021.1\win64\vsim.exe"

if (Test-Path "sim_work") { Remove-Item -Recurse -Force "sim_work" }
& $vlib sim_work
& $vlog -sv -work sim_work -lint `
  rtl/str_pkg.sv `
  rtl/str_input_buffer.sv `
  rtl/timing_phase_accumulator.sv `
  rtl/farrow_interpolator.sv `
  rtl/timing_sample_scheduler.sv `
  rtl/gardner_ted.sv `
  rtl/timing_pi_controller.sv `
  rtl/str_output_register.sv `
  rtl/str_top.sv `
  tb/tb_str_top.sv

& $vsim -c -work sim_work tb_str_top -do "run -all; quit -f"
