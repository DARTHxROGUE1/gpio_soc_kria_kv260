module soc_picorv32 (
    input  wire clk,
    input  wire resetn,

    output wire uart_tx,
    input  wire uart_rx
);

    // PicoRV32 memory interface
    wire        mem_valid;
    wire        mem_ready;
    wire [31:0] mem_addr;
    wire [31:0] mem_wdata;
    wire [3:0]  mem_wstrb;
    wire [31:0] mem_rdata;

    // Instantiate CPU
    picorv32 #(
        .ENABLE_MUL(1),
        .ENABLE_DIV(1),
        .ENABLE_COMPRESSED_ISA(1),
        .PROGADDR_RESET(32'h0000_0000)
    ) cpu (
        .clk        (clk),
        .resetn     (resetn),
        .mem_valid  (mem_valid),
        .mem_ready  (mem_ready),
        .mem_addr   (mem_addr),
        .mem_wdata  (mem_wdata),
        .mem_wstrb  (mem_wstrb),
        .mem_rdata  (mem_rdata)
    );

    // Address decode
    wire is_bram = (mem_addr[31:16] == 16'h0000);
    wire is_uart = (mem_addr[31:16] == 16'h1000);

    // BRAM signals
    wire        bram_ready;
    wire [31:0] bram_rdata;

    // uart signals
    wire        uart_ready;
    wire [31:0] uart_rdata;

    assign mem_ready = is_bram ? bram_ready :
        is_uart ? uart_ready :
        1'b0;

    assign mem_rdata = is_bram ? bram_rdata :
        is_uart ? uart_rdata :
        32'b0;


    bram_mem #(
        .MEM_WORDS(16384) // 64 KB
    ) ram (
        .clk        (clk),
        .resetn     (resetn),
        .mem_valid  (mem_valid && is_bram),
        .mem_ready  (bram_ready),
        .mem_addr   (mem_addr),
        .mem_wdata  (mem_wdata),
        .mem_wstrb  (mem_wstrb),
        .mem_rdata  (bram_rdata)
    );

    //uart 
    simpleuart uart (
        .clk         (clk),
        .resetn      (resetn),
        .ser_tx      (uart_tx),
        .ser_rx      (uart_rx),

        .reg_div_we  (mem_valid && is_uart && mem_wstrb != 0 && mem_addr[3:0] == 4'h0),
        .reg_div_di  (mem_wdata),

        .reg_dat_we  (mem_valid && is_uart && mem_wstrb != 0 && mem_addr[3:0] == 4'h4),
        .reg_dat_di  (mem_wdata),
        .reg_dat_do  (uart_rdata),

        .reg_dat_wait(uart_ready)
    );

endmodule
