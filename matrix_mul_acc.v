`timescale 1ns / 1ps

module matrix_mul_acc (
    input  wire          clk,
    input  wire          rst_n,
    
    // --- 控制信号 ---
    input  wire          i_row_start,
    input  wire          i_is_first_col,
    input  wire          i_is_last_col,
    
    // --- Matrix A 输入 (流) ---
    input  wire [22:0]   i_a_data,
    input  wire          i_a_valid,
    
    // --- Vector s1 输入 (来自 RAM) ---
    output wire [7:0]    o_s1_addr,
    input  wire [23:0]   i_s1_data,
    
    // --- 结果输出 ---
    output reg           o_res_valid,
    output reg  [23:0]   o_res_data,
    output wire          o_res_last,
    output reg           o_busy
);

    // =========================================================================
    // 参数定义 (与 ntt_core 保持一致)
    // =========================================================================
    localparam [23:0] Q = 24'd8380417;
    localparam [25:0] MU = 26'd33587228; // Barrett Constant for Q

    localparam S_IDLE      = 3'd0;
    localparam S_CALC      = 3'd1;
    localparam S_DUMP_INIT = 3'd2;
    localparam S_DUMP      = 3'd3;
    localparam S_DONE      = 3'd4;
    
    reg [2:0] state;

    // =========================================================================
    // 内部信号
    // =========================================================================
    reg [8:0] cnt;
    
    // FIFO
    wire        fifo_empty;
    wire        fifo_rd_en;
    wire [22:0] fifo_a_out;
    
    // 乘法器 (DSP + Barrett)
    (* use_dsp = "yes" *) reg [47:0] prod_reg; // 23bit * 24bit -> 47bit
    wire [23:0] mul_res;
    
    // 加法器
    wire [23:0] add_res;
    
    // 累加器 RAM
    reg  [7:0]  acc_addr_rd;
    reg  [7:0]  acc_addr_wr;
    reg         acc_we;
    reg  [23:0] acc_wdata;
    wire [23:0] acc_rdata;
    
    // 流水线控制
    reg         pipe_valid_s1; 
    reg         pipe_valid_mult;
    reg         pipe_is_first_col; 
    reg [7:0]   pipe_addr_stage1;
    reg [7:0]   pipe_addr_stage2;

    // =========================================================================
    // 1. 输入 FIFO
    // =========================================================================
    simple_fifo #( .WIDTH(23), .DEPTH(16) ) u_a_fifo (
        .clk(clk), .rst_n(rst_n),
        .we(i_a_valid), .din(i_a_data),
        .re(fifo_rd_en), .dout(fifo_a_out),
        .empty(fifo_empty), .full()
    );

    // =========================================================================
    // 2. 主控制逻辑
    // =========================================================================
    assign fifo_rd_en = (state == S_CALC) && (!fifo_empty);
    assign o_s1_addr = cnt[7:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            cnt <= 0;
            o_busy <= 0;
            pipe_valid_s1 <= 0;
            pipe_valid_mult <= 0;
            pipe_is_first_col <= 0;
            acc_we <= 0;
        end else begin
            acc_we <= 0;
            
            case (state)
                S_IDLE: begin
                    o_busy <= 0;
                    if (i_row_start) begin
                        state <= S_CALC;
                        cnt <= 0;
                        o_busy <= 1;
                    end
                end

                S_CALC: begin
                    // --- Stage 0: Request ---
                    if (!fifo_empty) begin
                        pipe_valid_s1 <= 1;
                        pipe_is_first_col <= i_is_first_col;
                        pipe_addr_stage1 <= cnt[7:0];
                        
                        if (cnt == 255) cnt <= 0;
                        else cnt <= cnt + 1;
                    end else begin
                        pipe_valid_s1 <= 0;
                    end

                    // --- Stage 1: Multiply Input Latch ---
                    if (pipe_valid_s1) begin
                        // 准备乘法输入: A * s1
                        prod_reg <= fifo_a_out * i_s1_data; 
                        
                        pipe_valid_mult <= 1;
                        pipe_addr_stage2 <= pipe_addr_stage1;
                        acc_addr_rd <= pipe_addr_stage1; 
                    end else begin
                        pipe_valid_mult <= 0;
                    end

                    // --- Stage 2: Add & Write Back ---
                    // Barrett 约减后的结果在 mul_res 中 (组合逻辑/内部流水)
                    if (pipe_valid_mult) begin
                        acc_we <= 1;
                        acc_addr_wr <= pipe_addr_stage2;
                        
                        if (pipe_is_first_col) begin
                            acc_wdata <= mul_res; // 第一列直接覆盖
                        end else begin
                            acc_wdata <= add_res; // 后续列累加
                        end
                        
                        // 检查是否是一行的最后一个数据
                        if (pipe_addr_stage2 == 255 && i_is_last_col) begin
                            state <= S_DUMP_INIT;
                        end
                    end
                end

                S_DUMP_INIT: begin
                    cnt <= 0;
                    acc_we <= 0;
                    acc_addr_rd <= 0;
                    state <= S_DUMP;
                end

                S_DUMP: begin
                    if (cnt < 256) begin
                        o_res_valid <= (cnt > 0); 
                        o_res_data  <= acc_rdata;
                        if (cnt < 255) acc_addr_rd <= cnt[7:0] + 1;
                        cnt <= cnt + 1;
                    end else begin
                        o_res_valid <= 1;
                        o_res_data  <= acc_rdata;
                        state <= S_DONE;
                    end
                end
                
                S_DONE: begin
                    o_res_valid <= 0;
                    o_busy <= 0;
                    state <= S_IDLE;
                end
            endcase
        end
    end
    
    assign o_res_last = (state == S_DONE);

    // =========================================================================
    // 3. 计算单元实例化 (FIXED)
    // =========================================================================
    
    // [FIX] 使用 Barrett_reduce 替代 Montgomery_mul，与 ntt_core 一致
    Barrett_reduce #( .WIDTH(24) ) u_barrett (
        .clk(clk),
        .prod(prod_reg),
        .q(Q),
        .mu(MU),
        .res(mul_res) // 乘法结果 A*s1 mod Q
    );
    
    // [FIX] 修正 mod_add 接口
    mod_add #( .WIDTH(24) ) u_add (
        .a(acc_rdata),
        .b(mul_res),
        .q(Q),         // 补充缺少 Q 输入
        .res(add_res)  // 修正端口名 c -> res
    );

    // =========================================================================
    // 4. 累加器 RAM
    // =========================================================================
    tdpram_24x256_internal u_acc_ram (
        .clk(clk),
        .we_a(1'b0), .addr_a(acc_addr_rd), .din_a(24'd0), .dout_a(acc_rdata),
        .we_b(acc_we), .addr_b(acc_addr_wr), .din_b(acc_wdata), .dout_b()
    );

endmodule

// Helper modules (FIFO & RAM) kept same as before...
module simple_fifo #(parameter WIDTH=23, DEPTH=16) (
    input wire clk, rst_n, we, input wire [WIDTH-1:0] din,
    input wire re, output wire [WIDTH-1:0] dout, output wire empty, output wire full
);
    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [4:0] wptr, rptr;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin wptr<=0; rptr<=0; end
        else begin
            if (we && !full) begin mem[wptr[3:0]]<=din; wptr<=wptr+1; end
            if (re && !empty) rptr<=rptr+1;
        end
    end
    assign dout = mem[rptr[3:0]];
    assign empty = (wptr == rptr);
    assign full  = (wptr[3:0] == rptr[3:0]) && (wptr[4] != rptr[4]);
endmodule

module tdpram_24x256_internal (
    input wire clk,
    input wire we_a, input wire [7:0] addr_a, input wire [23:0] din_a, output reg [23:0] dout_a,
    input wire we_b, input wire [7:0] addr_b, input wire [23:0] din_b, output reg [23:0] dout_b
);
    reg [23:0] ram [0:255];
    integer i;
    initial begin for(i=0; i<256; i=i+1) ram[i] = 0; end
    always @(posedge clk) begin if (we_a) ram[addr_a] <= din_a; dout_a <= ram[addr_a]; end
    always @(posedge clk) begin if (we_b) ram[addr_b] <= din_b; dout_b <= ram[addr_b]; end
endmodule