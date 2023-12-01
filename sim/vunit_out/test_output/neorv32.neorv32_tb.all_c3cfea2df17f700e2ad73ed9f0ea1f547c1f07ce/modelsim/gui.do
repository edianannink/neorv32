source "/home/edian/git/neorv32-iv-experiments/logic/neorv32/sim/vunit_out/test_output/neorv32.neorv32_tb.all_c3cfea2df17f700e2ad73ed9f0ea1f547c1f07ce/modelsim/common.do"
proc vunit_user_init {} {
    return 0
}
if {![vunit_load]} {
  vunit_user_init
  vunit_help
}
