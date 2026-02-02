# gpio_soc_kria_kv260
project currently under progress having problems in bitstream

System On Chip(SOC) desing to target the Xilinx Kria KV260 board. The core of the system is picorv32, a RISC-V CPU,with custom GPIO and a simple UART peripheral.It also contains a simple RAM unit nothing too fancy.


/rtl : source files for soc,picorv32 core and peripheral wrappers

/constraints : .xdc define pin mapping 



Open Vivado and source the project creation script or add the files in the /rtl directory.

Constraints: Ensure the uart_tx and clk pins are mapped in your .xdc file:

Clock: H12 (100MHz).

UART TX: B11 (Standard UART-to-USB bridge).
Supports a Native Memory Interface (simple valid-ready handshake) or can be adapted to standard buses like AXI4-Lite or Wishbone.

Run Synthesis, Implementation, and Generate Bitstream.

## Implementation
<img width="619" height="544" alt="Screenshot 2026-02-02 174810" src="https://github.com/user-attachments/assets/0b134add-d1ad-45c4-8207-787f818b8e14" />

## RTL Sysnthesis

<img width="1082" height="330" alt="Screenshot 2026-02-02 173515" src="https://github.com/user-attachments/assets/1a0416cd-4d52-425c-ba76-a6d51994285e" />
<img width="1391" height="265" alt="Screenshot 2026-02-02 173246" src="https://github.com/user-attachments/assets/1fe9230d-1e1c-4be6-8e24-bb466d4eabf8" />
<img width="571" height="297" alt="Screenshot 2026-02-02 172832" src="https://github.com/user-attachments/assets/567b0c1c-3ced-47ed-8edf-e3ba2a33e4fe" />

## BITSTREAM PENDING(IN PROGRESS)
