set_property PACKAGE_PIN H12 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]
create_clock -period 10.000 -name clk [get_ports clk]

set_property PACKAGE_PIN D18 [get_ports resetn]
set_property IOSTANDARD LVCMOS33 [get_ports resetn]

set_property PACKAGE_PIN D20 [get_ports {led[0]}]
set_property PACKAGE_PIN D19 [get_ports {led[1]}]
set_property PACKAGE_PIN E19 [get_ports {led[2]}]
set_property PACKAGE_PIN J18 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[*]}]

set_property PACKAGE_PIN H18 [get_ports uart_rx]
set_property PACKAGE_PIN J19 [get_ports uart_tx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_rx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_tx]