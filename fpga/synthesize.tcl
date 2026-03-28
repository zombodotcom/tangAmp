# synthesize.tcl — Full chain synthesis for Tang Nano 20K
set_device -name GW2AR-18C GW2AR-LV18QN88C8/I7

add_file tangamp_selftest.v
add_file ../triode_engine.v
add_file ../wdf_triode_wdf.v
add_file ../tone_stack_iir.v
add_file ../cabinet_fir.v
add_file ../power_amp_stage.v
add_file ../ip_lut.hex
add_file ../dip_dvgk_lut.hex
add_file ../dip_dvpk_lut.hex
add_file ../cab_ir.hex

add_file tangnano20k.cst

set_option -top_module tangamp_selftest
set_option -verilog_std sysv2017
set_option -use_sspi_as_gpio 1
set_option -use_mspi_as_gpio 1
set_option -use_ready_as_gpio 1
set_option -use_done_as_gpio 1
set_option -rw_check_on_ram 1

run all
