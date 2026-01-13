module picorv32_soc_kv260 (
    input wire clk,
    input wire resetn,
    output wire [3:0] led,
    input wire uart_rx,
    output wire uart_tx
);

    wire mem_valid;
    wire mem_instr;
    wire mem_ready;
    wire [31:0] mem_addr;
    wire [31:0] mem_wdata;
    wire [3:0] mem_wstrb;
    wire [31:0] mem_rdata;

    wire [31:0] ram_rdata;
    wire [31:0] uart_rdata;
    wire [31:0] gpio_rdata;
    
    wire ram_ready;
    wire uart_ready;
    wire gpio_ready;

    wire ram_select = (mem_addr[31:16] == 16'h0000);
    wire uart_select = (mem_addr[31:16] == 16'h0200);
    wire gpio_select = (mem_addr[31:16] == 16'h0300);

    assign mem_ready = ram_ready | uart_ready | gpio_ready;
    assign mem_rdata = ram_select ? ram_rdata :
                       uart_select ? uart_rdata :
                       gpio_select ? gpio_rdata : 32'h0;

    picorv32 #(
        .ENABLE_COUNTERS(1),
        .ENABLE_REGS_16_31(1),
        .ENABLE_REGS_DUALPORT(1),
        .LATCHED_MEM_RDATA(0),
        .TWO_STAGE_SHIFT(1),
        .BARREL_SHIFTER(0),
        .TWO_CYCLE_COMPARE(0),
        .TWO_CYCLE_ALU(0),
        .COMPRESSED_ISA(0),
        .CATCH_MISALIGN(1),
        .CATCH_ILLINSN(1),
        .ENABLE_PCPI(0),
        .ENABLE_MUL(0),
        .ENABLE_FAST_MUL(0),
        .ENABLE_DIV(0),
        .ENABLE_IRQ(0),
        .ENABLE_IRQ_QREGS(0),
        .ENABLE_IRQ_TIMER(0),
        .ENABLE_TRACE(0),
        .REGS_INIT_ZERO(0),
        .MASKED_IRQ(32'h0),
        .LATCHED_IRQ(32'hffffffff),
        .PROGADDR_RESET(32'h0),
        .PROGADDR_IRQ(32'h0010),
        .STACKADDR(32'hffff)
    ) cpu (
        .clk(clk),
        .resetn(resetn),
        .mem_valid(mem_valid),
        .mem_instr(mem_instr),
        .mem_ready(mem_ready),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb),
        .mem_rdata(mem_rdata),
        .mem_la_read(),
        .mem_la_write(),
        .mem_la_addr(),
        .mem_la_wdata(),
        .mem_la_wstrb(),
        .pcpi_valid(),
        .pcpi_insn(),
        .pcpi_rs1(),
        .pcpi_rs2(),
        .pcpi_wr(1'b0),
        .pcpi_rd(32'h0),
        .pcpi_wait(1'b0),
        .pcpi_ready(1'b0),
        .irq(32'h0),
        .eoi(),
        .trace_valid(),
        .trace_data()
    );

    memory #(
        .WORDS(4096)
    ) ram (
        .clk(clk),
        .resetn(resetn),
        .mem_valid(mem_valid && ram_select),
        .mem_addr(mem_addr[15:2]),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb),
        .mem_rdata(ram_rdata),
        .mem_ready(ram_ready)
    );

    uart_peripheral uart (
        .clk(clk),
        .resetn(resetn),
        .mem_valid(mem_valid && uart_select),
        .mem_addr(mem_addr[3:2]),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb),
        .mem_rdata(uart_rdata),
        .mem_ready(uart_ready),
        .uart_rx(uart_rx),
        .uart_tx(uart_tx)
    );

    gpio_peripheral gpio (
        .clk(clk),
        .resetn(resetn),
        .mem_valid(mem_valid && gpio_select),
        .mem_addr(mem_addr[3:2]),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb),
        .mem_rdata(gpio_rdata),
        .mem_ready(gpio_ready),
        .gpio_out(led)
    );

endmodule

module memory #(
    parameter WORDS = 4096
) (
    input wire clk,
    input wire resetn,
    input wire mem_valid,
    input wire [13:0] mem_addr,
    input wire [31:0] mem_wdata,
    input wire [3:0] mem_wstrb,
    output reg [31:0] mem_rdata,
    output reg mem_ready
);

    reg [31:0] mem [0:WORDS-1];
    
    initial begin
        $readmemh("firmware.hex", mem);
    end

    always @(posedge clk) begin
        mem_ready <= 0;
        if (!resetn) begin
            mem_ready <= 0;
        end else if (mem_valid && !mem_ready) begin
            if (mem_wstrb[0]) mem[mem_addr][ 7: 0] <= mem_wdata[ 7: 0];
            if (mem_wstrb[1]) mem[mem_addr][15: 8] <= mem_wdata[15: 8];
            if (mem_wstrb[2]) mem[mem_addr][23:16] <= mem_wdata[23:16];
            if (mem_wstrb[3]) mem[mem_addr][31:24] <= mem_wdata[31:24];
            mem_rdata <= mem[mem_addr];
            mem_ready <= 1;
        end
    end

endmodule

module uart_peripheral (
    input wire clk,
    input wire resetn,
    input wire mem_valid,
    input wire [1:0] mem_addr,
    input wire [31:0] mem_wdata,
    input wire [3:0] mem_wstrb,
    output reg [31:0] mem_rdata,
    output reg mem_ready,
    input wire uart_rx,
    output wire uart_tx
);

    reg [31:0] cfg_divider;
    reg [3:0] recv_state;
    reg [31:0] recv_divcnt;
    reg [7:0] recv_pattern;
    reg [7:0] recv_buf_data;
    reg recv_buf_valid;
    reg [9:0] send_pattern;
    reg [3:0] send_bitcnt;
    reg [31:0] send_divcnt;
    reg send_dummy;

    assign uart_tx = send_pattern[0];

    always @(posedge clk) begin
        mem_ready <= 0;
        if (!resetn) begin
            cfg_divider <= 434;
            mem_ready <= 0;
        end else if (mem_valid && !mem_ready) begin
            case (mem_addr)
                2'b00: begin
                    mem_rdata <= cfg_divider;
                    if (mem_wstrb[0]) cfg_divider <= mem_wdata;
                end
                2'b01: begin
                    mem_rdata <= recv_buf_valid ? {24'h0, recv_buf_data} : 32'hFFFFFFFF;
                    recv_buf_valid <= 0;
                end
                2'b10: begin
                    mem_rdata <= {31'h0, (send_bitcnt != 0 || send_dummy)};
                    if (mem_wstrb[0] && send_bitcnt == 0 && !send_dummy) begin
                        send_pattern <= {1'b1, mem_wdata[7:0], 1'b0};
                        send_bitcnt <= 10;
                        send_divcnt <= 0;
                    end
                end
                default: mem_rdata <= 32'h0;
            endcase
            mem_ready <= 1;
        end
    end

    always @(posedge clk) begin
        if (!resetn) begin
            recv_state <= 0;
            recv_divcnt <= 0;
            recv_pattern <= 0;
            recv_buf_valid <= 0;
            recv_buf_data <= 0;
        end else begin
            case (recv_state)
                4'd0: begin
                    if (!uart_rx) begin
                        recv_state <= 1;
                        recv_divcnt <= 0;
                    end
                end
                4'd1: begin
                    recv_divcnt <= recv_divcnt + 1;
                    if (recv_divcnt >= (cfg_divider >> 1)) begin
                        recv_state <= 2;
                        recv_divcnt <= 0;
                    end
                end
                4'd10: begin
                    recv_divcnt <= recv_divcnt + 1;
                    if (recv_divcnt >= cfg_divider) begin
                        recv_buf_data <= recv_pattern;
                        recv_buf_valid <= 1;
                        recv_state <= 0;
                    end
                end
                default: begin
                    recv_divcnt <= recv_divcnt + 1;
                    if (recv_divcnt >= cfg_divider) begin
                        recv_pattern <= {uart_rx, recv_pattern[7:1]};
                        recv_state <= recv_state + 1;
                        recv_divcnt <= 0;
                    end
                end
            endcase
        end
    end

    always @(posedge clk) begin
        if (!resetn) begin
            send_pattern <= 10'h3FF;
            send_bitcnt <= 0;
            send_divcnt <= 0;
            send_dummy <= 1;
        end else begin
            if (send_bitcnt != 0) begin
                send_divcnt <= send_divcnt + 1;
            end
            if (send_dummy && send_bitcnt == 0) begin
                send_pattern <= 10'h3FF;
                send_bitcnt <= 15;
                send_divcnt <= 0;
                send_dummy <= 0;
            end else if (send_bitcnt != 0 && send_divcnt >= cfg_divider) begin
                send_pattern <= {1'b1, send_pattern[9:1]};
                send_bitcnt <= send_bitcnt - 1;
                send_divcnt <= 0;
            end
        end
    end

endmodule

module gpio_peripheral (
    input wire clk,
    input wire resetn,
    input wire mem_valid,
    input wire [1:0] mem_addr,
    input wire [31:0] mem_wdata,
    input wire [3:0] mem_wstrb,
    output reg [31:0] mem_rdata,
    output reg mem_ready,
    output reg [3:0] gpio_out
);

    always @(posedge clk) begin
        mem_ready <= 0;
        if (!resetn) begin
            gpio_out <= 4'h0;
            mem_ready <= 0;
        end else if (mem_valid && !mem_ready) begin
            case (mem_addr)
                2'b00: begin
                    mem_rdata <= {28'h0, gpio_out};
                    if (mem_wstrb[0]) gpio_out <= mem_wdata[3:0];
                end
                default: mem_rdata <= 32'h0;
            endcase
            mem_ready <= 1;
        end
    end

endmodule

module picorv32 #(
    parameter [0:0] ENABLE_COUNTERS = 1,
    parameter [0:0] ENABLE_REGS_16_31 = 1,
    parameter [0:0] ENABLE_REGS_DUALPORT = 1,
    parameter [0:0] LATCHED_MEM_RDATA = 0,
    parameter [0:0] TWO_STAGE_SHIFT = 1,
    parameter [0:0] BARREL_SHIFTER = 0,
    parameter [0:0] TWO_CYCLE_COMPARE = 0,
    parameter [0:0] TWO_CYCLE_ALU = 0,
    parameter [0:0] COMPRESSED_ISA = 0,
    parameter [0:0] CATCH_MISALIGN = 1,
    parameter [0:0] CATCH_ILLINSN = 1,
    parameter [0:0] ENABLE_PCPI = 0,
    parameter [0:0] ENABLE_MUL = 0,
    parameter [0:0] ENABLE_FAST_MUL = 0,
    parameter [0:0] ENABLE_DIV = 0,
    parameter [0:0] ENABLE_IRQ = 0,
    parameter [0:0] ENABLE_IRQ_QREGS = 0,
    parameter [0:0] ENABLE_IRQ_TIMER = 0,
    parameter [0:0] ENABLE_TRACE = 0,
    parameter [0:0] REGS_INIT_ZERO = 0,
    parameter [31:0] MASKED_IRQ = 32'h0,
    parameter [31:0] LATCHED_IRQ = 32'hffffffff,
    parameter [31:0] PROGADDR_RESET = 32'h0,
    parameter [31:0] PROGADDR_IRQ = 32'h0010,
    parameter [31:0] STACKADDR = 32'hffff
) (
    input clk, resetn,
    output reg mem_valid,
    output reg mem_instr,
    input mem_ready,
    output reg [31:0] mem_addr,
    output reg [31:0] mem_wdata,
    output reg [3:0] mem_wstrb,
    input [31:0] mem_rdata,
    output mem_la_read,
    output mem_la_write,
    output [31:0] mem_la_addr,
    output [31:0] mem_la_wdata,
    output [3:0] mem_la_wstrb,
    output pcpi_valid,
    output [31:0] pcpi_insn,
    output [31:0] pcpi_rs1,
    output [31:0] pcpi_rs2,
    input pcpi_wr,
    input [31:0] pcpi_rd,
    input pcpi_wait,
    input pcpi_ready,
    input [31:0] irq,
    output [31:0] eoi,
    output trace_valid,
    output [35:0] trace_data
);

    assign mem_la_read = 0;
    assign mem_la_write = 0;
    assign mem_la_addr = 0;
    assign mem_la_wdata = 0;
    assign mem_la_wstrb = 0;
    assign pcpi_valid = 0;
    assign pcpi_insn = 0;
    assign pcpi_rs1 = 0;
    assign pcpi_rs2 = 0;
    assign eoi = 0;
    assign trace_valid = 0;
    assign trace_data = 0;

    reg [31:0] pc;
    reg [31:0] regs [0:31];
    reg [2:0] state;
    reg [31:0] insn;
    reg [31:0] rs1_data, rs2_data;

    localparam STATE_FETCH = 0;
    localparam STATE_DECODE = 1;
    localparam STATE_EXECUTE = 2;
    localparam STATE_MEMWAIT = 3;

    always @(posedge clk) begin
        if (!resetn) begin
            pc <= PROGADDR_RESET;
            state <= STATE_FETCH;
            mem_valid <= 0;
            mem_instr <= 0;
            mem_wstrb <= 0;
        end else begin
            case (state)
                STATE_FETCH: begin
                    mem_valid <= 1;
                    mem_instr <= 1;
                    mem_addr <= pc;
                    mem_wstrb <= 0;
                    if (mem_ready) begin
                        insn <= mem_rdata;
                        mem_valid <= 0;
                        state <= STATE_DECODE;
                    end
                end
                STATE_DECODE: begin
                    rs1_data <= regs[insn[19:15]];
                    rs2_data <= regs[insn[24:20]];
                    state <= STATE_EXECUTE;
                end
                STATE_EXECUTE: begin
                    pc <= pc + 4;
                    state <= STATE_FETCH;
                end
            endcase
        end
    end

endmodule