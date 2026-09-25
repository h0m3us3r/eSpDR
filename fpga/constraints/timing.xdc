# Clocks and FT600 interface timing.

create_clock -name oscillator -period 10.000 [get_ports clk100]

# FT600 100 MHz clock, 245 mode. Times are relative to the FT clock edge at
# the FPGA pin.
create_clock -name ft_reference -period 10.000 [get_ports ft_clk]
# TXE: the FT600 datasheet's output valid window (setup 3.0, hold 3.5 ns)
# with 0.2 ns for the board.
set_input_delay -clock ft_reference -max 7.2 [get_ports ft_txe]
set_input_delay -clock ft_reference -min 3.3 [get_ports ft_txe]
# WR and data: measured on the Au/Ft stack by sweeping each launch phase.
# WR may change 5.5..9.0 ns after the FT edge (failures began near 9.6 ns;
# 5.5 ns is as early as the sweep reached).
set_output_delay -clock ft_reference -max 1.000 [get_ports {ft_wr ft_be[*]}]
set_output_delay -clock ft_reference -min -5.500 [get_ports {ft_wr ft_be[*]}]
# Data may change 4.0..8.0 ns after the FT edge (passed about 3.6..9.0 ns).
set_output_delay -clock ft_reference -max 2.000 [get_ports {ft_data[*]}]
set_output_delay -clock ft_reference -min -4.000 [get_ports {ft_data[*]}]
