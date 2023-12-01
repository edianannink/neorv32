onerror {quit -code 1}
source "/home/edian/git/neorv32-iv-experiments/logic/neorv32/sim/vunit_out/test_output/neorv32.neorv32_tb.all_c3cfea2df17f700e2ad73ed9f0ea1f547c1f07ce/modelsim/common.do"
set failed [vunit_load]
if {$failed} {quit -code 1}
set failed [vunit_run]
if {$failed} {quit -code 1}
quit -code 0
