module uart #(parameter integer DEFAULT_DIV = 434) (
    input wire clk, resetn, ser_rx, reg_div_we, reg_data_we, reg_data_re,
    input wire [31:0] reg_div_di, reg_data_di,
    output wire ser_tx, reg_data_wait,
    output wire [31:0] reg_div_do, reg_data_do
);
    reg [31:0] cfg_divider, recv_divcnt, send_divcnt;
    reg [3:0] recv_state, send_bitcnt;
    reg [7:0] recv_pattern, recv_buf_data;
    reg [9:0] send_pattern;
    reg recv_buf_valid, send_dummy;
    
    assign reg_div_do = cfg_divider;
    assign reg_data_wait = reg_data_we && (send_bitcnt || send_dummy);
    assign reg_data_do = recv_buf_valid ? {24'h0, recv_buf_data} : 32'hFFFFFFFF;
    assign ser_tx = send_pattern[0];

    always @(posedge clk) begin //recier state machine 
        if (!resetn) cfg_divider <= DEFAULT_DIV;
        else if (reg_div_we) cfg_divider <= reg_div_di;
    end

    always @(posedge clk) begin
        if (!resetn) begin
            recv_state <= 0; recv_divcnt <= 0; recv_pattern <= 0;
            recv_buf_valid <= 0; recv_buf_data <= 0;
        
    end else begin
            
    if (reg_data_re) recv_buf_valid <= 0;
        case (recv_state)
        4'd0: if (!ser_rx) begin recv_state <= 1; recv_divcnt <= 0;//idle
         end
         //start (wait for half bit period)
        4'd1: begin
        recv_divcnt <= recv_divcnt + 1;
           
 if (recv_divcnt >= (cfg_divider >> 1)) begin
            recv_state <= 2; recv_divcnt <= 0;
        end end
    // stop bit set valid if data is ready    
    4'd10: begin
        recv_divcnt <= recv_divcnt + 1;
                    
    if (recv_divcnt >= cfg_divider) begin
        recv_buf_data <= recv_pattern; recv_buf_valid <= 1; recv_state <= 0;
 end end
        
    default: begin
    recv_divcnt <= recv_divcnt + 1;
 if (recv_divcnt >= cfg_divider) begin
        recv_pattern <= {ser_rx, recv_pattern[7:1]};
        recv_state <= recv_state + 1; recv_divcnt <= 0;
                   
end end endcase end end


//transmitter sm
always @(posedge clk) begin
    if (!resetn) begin
    send_pattern <= 10'h3FF; send_bitcnt <= 0; //all 1's = idle 
    send_divcnt <= 0; send_dummy <= 1;
        
end else begin
            
    if (reg_div_we) send_dummy <= 1;
    if (send_bitcnt) send_divcnt <= send_divcnt + 1;//increment timing cnt
    // send dummy pattern of 15 bits after divider change
    else begin
    if (send_dummy && !send_bitcnt) begin
        send_pattern <= 10'h3FF; send_bitcnt <= 15;
        send_divcnt <= 0; send_dummy <= 0;
            
        end else if (reg_data_we && !send_bitcnt) begin
                send_pattern <= {1'b1, reg_data_di[7:0], 1'b0};
                send_bitcnt <= 10; send_divcnt <= 0;
            end else if (send_bitcnt && send_divcnt >= cfg_divider) begin
                send_pattern <= {1'b1, send_pattern[9:1]};
                send_bitcnt <= send_bitcnt - 1; send_divcnt <= 0;
            end
        end
    end
endmodule