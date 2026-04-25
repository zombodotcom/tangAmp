catch {close_project}
set_device -name GW2AR-18C GW2AR-LV18QN88C8/I7
add_file ../rtl/mclk_check.v
add_file ../rtl/clk_audio_pll.v
add_file tangnano20k.cst
add_file blink.sdc
set_option -top_module mclk_check
set_option -verilog_std sysv2017
set_option -use_sspi_as_gpio 1
set_option -use_mspi_as_gpio 1
set_option -use_ready_as_gpio 1
set_option -use_done_as_gpio 1
run all
