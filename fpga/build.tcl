# Vivado non-project build: vivado -mode batch -source build.tcl -tclargs OUTPUT_DIR
# Produces OUTPUT_DIR/iqstream.bit and timing/utilization reports. Fails on
# DRC errors or any negative setup/hold slack.

set here [file dirname [file normalize [info script]]]
set out [file normalize [lindex $argv 0]]
set part xc7a35tftg256-1
file mkdir $out
cd $out

exec sh $here/protocol_vh.sh $out/protocol.vh {*}[glob $here/../protocol/*.h]

# The DDR3 controller is generated from its saved configuration.
create_project -force iqstream [file join $out project] -part $part
create_ip -name mig_7series -vendor xilinx.com -library ip -version 4.2 -module_name mig_7series_0
set_property CONFIG.XML_INPUT_FILE [file join $here ip mig.prj] [get_ips mig_7series_0]
generate_target all [get_ips mig_7series_0]
set_property generate_synth_checkpoint false [get_files *.xci]
set_param general.maxThreads 8

set_property include_dirs [list $out] [current_fileset]
read_verilog -sv [glob [file join $here rtl *.v]]
read_xdc [file join $here constraints pins.xdc]
read_xdc [file join $here constraints timing.xdc]

synth_design -top iqstream_top -part $part
source [file join $here constraints implementation.tcl]

opt_design
place_design -directive Explore
phys_opt_design -directive AggressiveExplore
route_design -directive Explore
phys_opt_design -directive AggressiveExplore
route_design -directive Explore
phys_opt_design -aggressive_hold_fix
route_design -directive Explore
# TXE crosses the die to WR's next-state LUT (implementation.tcl), and the
# general routing leaves that route longer than it needs to be: route its
# nets again for least delay, with everything else in place.
set txe_nets [get_nets {ft_txe stream/tx/ft_txe_IBUF stream/tx/wr_next}]
route_design -unroute -nets $txe_nets
route_design -nets $txe_nets -delay

report_timing_summary -report_unconstrained -delay_type min_max -file timing.rpt
report_utilization -file utilization.rpt
report_utilization -hierarchical -hierarchical_depth 3 -file utilization_hierarchy.rpt
report_drc -file drc.rpt
report_cdc -details -file cdc.rpt
write_checkpoint -force routed.dcp

if {[llength [get_drc_violations -filter {SEVERITY == Error}]]} {
    error "DRC errors, see drc.rpt"
}
if {[llength [get_timing_paths -delay_type max -slack_lesser_than 0]] ||
    [llength [get_timing_paths -delay_type min -slack_lesser_than 0]]} {
    error "timing not met, see timing.rpt"
}
write_bitstream -force iqstream.bit
