catch {close_project}
set_device -name GW2AR-18C GW2AR-LV18QN88C8/I7
add_file ../rtl/tangamp_top_lite.v
add_file ../rtl/clk_audio_gen.v
add_file ../rtl/i2s_rx.v
add_file ../rtl/i2s_tx.v
add_file ../rtl/oversample_2x.v
add_file ../rtl/triode_engine.v
add_file ../rtl/wdf_triode_wdf.v
add_file ../rtl/tone_stack_iir.v
add_file ../rtl/output_transformer.v
add_file ../rtl/nfb_register.v
add_file ../rtl/cabinet_fir.v
add_file ../rtl/noise_gate.v
add_file ../data/ip_lut.hex
add_file ../data/dip_dvgk_lut.hex
add_file ../data/cab_ir.hex
add_file ../data/ig_lut.hex
add_file ../data/dig_lut.hex
add_file tangnano20k.cst
add_file blink.sdc
set_option -top_module tangamp_top_lite
set_option -verilog_std sysv2017
set_option -use_sspi_as_gpio 1
set_option -use_mspi_as_gpio 1
set_option -use_ready_as_gpio 1
set_option -use_done_as_gpio 1
set_option -rw_check_on_ram 1
set_option -include_path ..
run all
