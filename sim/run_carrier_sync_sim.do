# QuestaSim Simulation Script for Integrated Carrier Synchronization (carrier_sync_top)
vlib work
vlog -sv ../rtl/*.sv ../rtl/costas/*.v ../tb/tb_carrier_sync_top.sv
vsim -c -t 1ps work.tb_carrier_sync_top
run -all
exit
