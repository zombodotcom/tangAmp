catch {close_project}
set_device -name GW2AR-18C GW2AR-LV18QN88C8/I7
add_file ../rtl/cabinet_test.v
add_file ../rtl/clk_audio_gen.v
add_file ../rtl/clk_audio_pll.v
add_file ../rtl/i2s_rx.v
add_file ../rtl/i2s_tx.v
add_file ../rtl/cabinet_fir.v
add_file ../rtl/sd_spi_reader.v
add_file ../rtl/sd_ir_loader.v
add_file ../data/cab_ir.hex
add_file tangnano20k.cst
add_file blink.sdc
set_option -top_module cabinet_test
set_option -verilog_std sysv2017
set_option -use_sspi_as_gpio 1
set_option -use_mspi_as_gpio 1
set_option -use_ready_as_gpio 1
set_option -use_done_as_gpio 1
set_option -rw_check_on_ram 1
set_option -include_path ..
run all
