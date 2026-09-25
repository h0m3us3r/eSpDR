# Constraints on synthesized cells: clock-domain crossings and the FT600 WR
# path. Sourced after synth_design.

# ---- asynchronous inputs and static outputs ----
set_false_path -from [get_ports rst_n] -to [get_cells {reset_sync_reg[0]}]
set_false_path -from [get_ports uart_rx] -to [get_cells {control/receiver/rx_sync_reg[0]}]
set_false_path -to [get_ports {uart_tx ft_rd ft_oe}]

# ---- single-bit synchronizers ----
set_false_path -to [get_pins -hier -filter {NAME =~ reference_clock/enable_sync_reg*/CLR || NAME =~ reference_clock/output_enable_reg/CLR}]
set_false_path -to [get_pins {reference_clock/enable_sync_reg[0]/D reference_sync_reg[0]/D}]
# Phase target and status buses change only while the link is idle and are
# read after they settle.
set_false_path -to [get_pins -hier -filter {NAME =~ clocks/target_meta_reg*/D || NAME =~ clocks/status_meta_reg*/D}]
set_false_path -to [get_pins {link_input/run_sync_reg[0]/D link_input/clear_sync_reg[0]/D}]
set_false_path -to [get_cells {calibrated_sync_reg[0] stream/clear_sync_reg[0] stream/ft_reset_reg[0]}]

# ---- Gray-pointer FIFOs: each pointer bus settles within one source clock ----
proc constrain_fifo {fifo write_clock read_clock write_period read_period} {
    set_max_delay $write_period -datapath_only -from $write_clock -to [get_pins -hier -filter "NAME =~ ${fifo}/wr_gray_sync1_reg*/D"]
    set_max_delay $read_period -datapath_only -from $read_clock -to [get_pins -hier -filter "NAME =~ ${fifo}/rd_gray_sync1_reg*/D"]
}
set sample_clock [get_clocks -of_objects [get_pins {link_input/sampled_reg[0]/C}]]
set process_clock [get_clocks -of_objects [get_pins {reset_sync_reg[0]/C}]]
set ui_clock [get_clocks -of_objects [get_pins stream/ring/pending_reg/C]]
set ft_control [get_clocks -of_objects [get_pins stream/tx/sampled_txe_reg/C]]
set ft_work [get_clocks -of_objects [get_pins {stream/tx/beat_reg[0]/C}]]
set ft_write [get_clocks -of_objects [get_pins stream/tx/wr_pad_reg/C]]
if {[llength $ui_clock] != 1 || abs([get_property PERIOD $ui_clock] - 12.0) > 0.001} {
    error "unexpected DDR user-interface clock"
}
constrain_fifo link_input/sample_fifo $sample_clock $process_clock 4.167 8.333
constrain_fifo stream/ingress $process_clock $ui_clock 8.333 12.0
constrain_fifo stream/output_queue $ui_clock $ft_work 12.0 10.0

# ---- live status: snapshots of counters from other domains (snapshot.v) ----
# The copy in `held` is stable from well before the destination takes it
# until well after, so its path only needs to settle within a destination
# clock; the handshake bits are two-flop synchronizers.
proc constrain_snapshot {cell destination_period} {
    set_false_path -to [get_pins -hier -filter "NAME =~ ${cell}/request_meta_reg/D || NAME =~ ${cell}/acknowledge_meta_reg/D"]
    set_max_delay $destination_period -datapath_only \
        -from [get_cells -hier -filter "NAME =~ ${cell}/held_reg* && IS_SEQUENTIAL == 1"] \
        -to [get_cells -hier -filter "NAME =~ ${cell}/value_reg* && IS_SEQUENTIAL == 1"]
}
constrain_snapshot link_input/overflow_status 8.333
constrain_snapshot stream/ring_status 8.333
constrain_snapshot stream/usb_status 8.333

# ---- FT600 TXE ----
# TXE is sampled 1.625 ns after the FT edge following the one that launched
# it, i.e. one period later than the default relationship.
set_multicycle_path 2 -setup -end -from [get_ports ft_txe] -to $ft_control
set_multicycle_path 0 -hold -end -from [get_ports ft_txe] -to $ft_control
# The data-pad clock enable and WR also use the preceding FT edge's TXE.
set_multicycle_path 2 -setup -end -from [get_ports ft_txe] -to [get_pins stream/data_buffer/CE]
set_multicycle_path 0 -hold -end -from [get_ports ft_txe] -to [get_pins stream/data_buffer/CE]
set_multicycle_path 2 -setup -end -from [get_ports ft_txe] -to $ft_write
set_multicycle_path 0 -hold -end -from [get_ports ft_txe] -to $ft_write

# The data-clock gate's CE is timed against TXE, which comes from the far side
# of the die; of the buffer sites the MMCM can drive, this one is best for
# both the CE and the data pads.
set_property LOC BUFGCTRL_X0Y17 [get_cells stream/data_buffer]
# WR's next-state LUT sits beside WR's IOB.
set_property LOC SLICE_X65Y64 [get_cells stream/tx/wr_next_lut]
set_property BEL B6LUT [get_cells stream/tx/wr_next_lut]
