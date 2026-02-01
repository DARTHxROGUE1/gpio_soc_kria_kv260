module top_kv260 (
    input  wire clk_100mhz,   // clock (from board)
    input  wire reset_btn,     // active-high pushbutton

    output wire uart_tx,
    input  wire uart_rx
);

    wire clk;
    wire resetn;

    assign clk    = clk_100mhz;
    assign resetn = ~reset_btn;

    soc_picorv32 soc (
        .clk     (clk),
        .resetn  (resetn),
        .uart_tx (uart_tx),
        .uart_rx (uart_rx)
    );

endmodule
