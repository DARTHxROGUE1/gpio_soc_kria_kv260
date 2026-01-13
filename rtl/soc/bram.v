module bram_mem #(
    parameter MEM_WORDS = 16384
)(
    input  wire        clk,
    input  wire        resetn,
    input  wire        mem_valid,
    output reg         mem_ready,
    input  wire [31:0] mem_addr,
    input  wire [31:0] mem_wdata,
    input  wire [3:0]  mem_wstrb,
    output reg  [31:0] mem_rdata
);

    reg [31:0] mem [0:MEM_WORDS-1];
    wire [31:0] word_addr = mem_addr >> 2;

    initial begin
        $readmemh("firmware.hex", mem);
    end

    always @(posedge clk) begin
        mem_ready <= 1'b0;

        if (mem_valid) begin
            mem_ready <= 1'b1;

            if (mem_wstrb[0]) mem[word_addr][7:0]   <= mem_wdata[7:0];
            if (mem_wstrb[1]) mem[word_addr][15:8]  <= mem_wdata[15:8];
            if (mem_wstrb[2]) mem[word_addr][23:16] <= mem_wdata[23:16];
            if (mem_wstrb[3]) mem[word_addr][31:24] <= mem_wdata[31:24];

            mem_rdata <= mem[word_addr];
        end
    end

endmodule
