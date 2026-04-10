catch {close_project}
set_device -name GW2AR-18C GW2AR-LV18QN88C8/I7
add_file ../rtl/blink.v
add_file blink.cst
add_file blink.sdc
set_option -top_module blink
set_option -use_sspi_as_gpio 1
set_option -use_mspi_as_gpio 1
set_option -use_ready_as_gpio 1
set_option -use_done_as_gpio 1
run all
