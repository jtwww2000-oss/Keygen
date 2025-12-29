`timescale 1ns / 1ps

module intt_core #(
    parameter WIDTH = 24
)(
    input  wire          clk,
    input  wire          rst_n,
    input  wire          start,
    output reg           done,
    
    // --- 适配 ram_24x256 IP 核的双端口接口 ---
    // Port A
    output reg  [7:0]    ram_addr_a,
    output reg           ram_we_a,
    output reg  [WIDTH-1:0] ram_wdata_a,
    input  wire [WIDTH-1:0] ram_rdata_a,
    
    // Port B
    output reg  [7:0]    ram_addr_b,
    output reg           ram_we_b,
    output reg  [WIDTH-1:0] ram_wdata_b,
    input  wire [WIDTH-1:0] ram_rdata_b,
    
    // --- DEBUG PORTS ---
    output wire [WIDTH-1:0] dbg_gk,
    output wire [WIDTH-1:0] dbg_g1,
    output wire [WIDTH-1:0] dbg_val_u,
    output wire [WIDTH-1:0] dbg_val_v,
    output wire [WIDTH-1:0] dbg_prod_y,
    output wire             dbg_butterfly_done
);

    // --- 参数定义 ---
    localparam [WIDTH-1:0] Q = 24'd8380417;
    localparam [25:0]      MU = 26'd33587228;
    localparam [2:0]       MUL_LATENCY = 3'd5;
    localparam [WIDTH-1:0] N_INV = 24'd8347681;
    localparam [WIDTH-1:0] G_INV = 24'd731434;

    // --- 状态机定义 ---
    localparam S_IDLE           = 6'd0;
    
    // Bit Reversal (双端口并行优化)
    localparam S_BR_CHECK       = 6'd1;
    localparam S_BR_READ_AB     = 6'd2; // 并行读
    localparam S_BR_WAIT_AB     = 6'd3; // 等待读
    localparam S_BR_WRITE_AB    = 6'd4; // 并行写 (交换)
    
    // Loops
    localparam S_STAGE_INIT     = 6'd5;
    localparam S_STAGE_INIT_WAIT= 6'd6;
    localparam S_LOOP_I         = 6'd7;
    localparam S_LOOP_J         = 6'd8;
    
    // Butterfly (Cooley-Tukey 结构, 双端口优化)
    localparam S_READ_UV        = 6'd9;  // 并行读 U, V
    localparam S_READ_UV_WAIT   = 6'd10; // 等待数据
    localparam S_CALC_MUL_REQ   = 6'd11; // 启动乘法 (同时更新 gk)
    localparam S_CALC_MUL_WAIT  = 6'd12; // 等待乘法结果
    localparam S_CALC_ADD       = 6'd13; // 计算加减
    localparam S_WRITE_UV       = 6'd14; // 并行写回
    
    // Post-Process (Port A 单口操作)
    localparam S_POST_READ      = 6'd15;
    localparam S_POST_READ_WAIT = 6'd16;
    localparam S_POST_MUL_DAT   = 6'd17;
    localparam S_POST_WAIT_DAT  = 6'd18;
    localparam S_POST_MUL_G     = 6'd19;
    localparam S_POST_WAIT_G    = 6'd20;
    
    localparam S_DONE           = 6'd21;

    reg [5:0] state;
    reg [2:0] wait_cnt;

    // --- 内部变量 ---
    reg [8:0] cnt;
    reg [7:0] idx_u, idx_rev;
    
    reg [8:0] mid, i, j;
    reg [WIDTH-1:0] g1, gk, gk_next;
    reg [WIDTH-1:0] val_u, val_v, prod_y, res_u, res_v;
    reg [WIDTH-1:0] post_factor, post_cal_data;

    // --- 模加减模块 ---
    // 注意：根据你的代码逻辑，mod_add/sub 是在乘法之后计算的 (CT结构)
    wire [WIDTH-1:0] mod_add_out, mod_sub_out;
    mod_add #( .WIDTH(WIDTH) ) u_mod_add ( .a(val_u), .b(prod_y), .q(Q), .res(mod_add_out) );
    mod_sub #( .WIDTH(WIDTH) ) u_mod_sub ( .a(val_u), .b(prod_y), .q(Q), .res(mod_sub_out) );

    // --- ROM ---
    wire [WIDTH-1:0] g1_rom_out;
    g1_intt_rom u_g1_rom ( .clk(clk), .mid(mid), .g1(g1_rom_out) );

    // --- 乘法器 1 (主数据乘法) ---
    reg  [WIDTH-1:0] mul_op1, mul_op2;
    wire [WIDTH-1:0] mul_res;
    (* use_dsp = "yes" *) reg [47:0] prod_reg;
    always @(posedge clk) prod_reg <= mul_op1 * mul_op2;
    Barrett_reduce #( .WIDTH(WIDTH) ) u_barrett_main ( 
        .clk(clk), .prod(prod_reg), .q(Q), .mu(MU), .res(mul_res) 
    );

    // --- 乘法器 2 (参数更新乘法: gk更新 / post_factor更新) ---
    reg  [WIDTH-1:0] mul_gk_op1, mul_gk_op2;
    wire [WIDTH-1:0] mul_gk_res;
    (* use_dsp = "yes" *) reg [47:0] prod_reg_gk;
    always @(posedge clk) prod_reg_gk <= mul_gk_op1 * mul_gk_op2;
    Barrett_reduce #( .WIDTH(WIDTH) ) u_barrett_gk ( 
        .clk(clk), .prod(prod_reg_gk), .q(Q), .mu(MU), .res(mul_gk_res) 
    );

    // --- 辅助函数 ---
    function [7:0] bit_reverse(input [7:0] in);
        integer k;
        begin for (k = 0; k < 8; k = k + 1) bit_reverse[k] = in[7-k]; end
    endfunction

    // --- 调试信号 ---
    assign dbg_gk = gk;
    assign dbg_g1 = g1;
    assign dbg_val_u = val_u;
    assign dbg_val_v = val_v;
    assign dbg_prod_y = prod_y;
    assign dbg_butterfly_done = (state == S_WRITE_UV);

    // --- 主逻辑 ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; 
            done <= 0; 
            ram_we_a <= 0; ram_we_b <= 0;
            cnt <= 0; mid <= 1; post_factor <= 0;
            mul_op1 <= 0; mul_op2 <= 0;
            mul_gk_op1 <= 0; mul_gk_op2 <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0; ram_we_a <= 0; ram_we_b <= 0;
                    if (start) begin cnt <= 0; state <= S_BR_CHECK; end
                end

                // ==========================================
                // 1. Bit Reversal (双端口并行优化)
                // ==========================================
                S_BR_CHECK: begin
                    ram_we_a <= 0; ram_we_b <= 0;
                    if (cnt > 255) begin 
                        mid <= 1; 
                        state <= S_STAGE_INIT; 
                    end else begin
                        idx_u = cnt[7:0]; 
                        idx_rev = bit_reverse(idx_u);
                        if (idx_u < idx_rev) state <= S_BR_READ_AB;
                        else begin cnt <= cnt + 1; state <= S_BR_CHECK; end
                    end
                end
                S_BR_READ_AB: begin
                    // 同时读 A, B
                    ram_addr_a <= idx_u;
                    ram_addr_b <= idx_rev;
                    state <= S_BR_WAIT_AB;
                end
                S_BR_WAIT_AB: state <= S_BR_WRITE_AB; // 等待读延迟
                S_BR_WRITE_AB: begin
                    // 交叉写回
                    ram_we_a <= 1; ram_addr_a <= idx_u;   ram_wdata_a <= ram_rdata_b;
                    ram_we_b <= 1; ram_addr_b <= idx_rev; ram_wdata_b <= ram_rdata_a;
                    cnt <= cnt + 1;
                    state <= S_BR_CHECK;
                end

                // ==========================================
                // 2. Loops (保持原代码结构: mid递增)
                // ==========================================
                S_STAGE_INIT: begin
                    ram_we_a <= 0; ram_we_b <= 0;
                    if (mid < 256) state <= S_STAGE_INIT_WAIT;
                    else begin 
                        cnt <= 0; 
                        post_factor <= N_INV; 
                        state <= S_POST_READ; 
                    end
                end
                S_STAGE_INIT_WAIT: begin 
                    g1 <= g1_rom_out; 
                    i <= 0; 
                    state <= S_LOOP_I; 
                end
                S_LOOP_I: begin
                    if (i < 256) begin 
                        gk <= 24'd1; 
                        j <= 0; 
                        state <= S_LOOP_J; 
                    end else begin 
                        mid <= mid << 1; 
                        state <= S_STAGE_INIT; 
                    end
                end
                S_LOOP_J: begin
                    if (j < mid) state <= S_READ_UV;
                    else begin 
                        i <= i + (mid << 1); 
                        state <= S_LOOP_I; 
                    end
                end

                // ==========================================
                // 3. Butterfly (双端口并行优化)
                // 逻辑: V*gk -> Add/Sub -> Update gk
                // ==========================================
                S_READ_UV: begin
                    // 并行读 U, V
                    ram_addr_a <= i + j;
                    ram_addr_b <= i + j + mid;
                    state <= S_READ_UV_WAIT;
                end
                S_READ_UV_WAIT: state <= S_CALC_MUL_REQ;
                
                S_CALC_MUL_REQ: begin
                    // 锁存读到的数据
                    val_u <= ram_rdata_a;
                    val_v <= ram_rdata_b;
                    
                    // 启动乘法 1: prod_y = V * gk
                    mul_op1 <= ram_rdata_b; 
                    mul_op2 <= gk; 
                    
                    // 启动乘法 2: gk_next = gk * g1 (更新旋转因子)
                    mul_gk_op1 <= gk; 
                    mul_gk_op2 <= g1;
                    
                    wait_cnt <= MUL_LATENCY; 
                    state <= S_CALC_MUL_WAIT;
                end
                
                S_CALC_MUL_WAIT: begin
                    if (wait_cnt == 0) begin 
                        prod_y <= mul_res;      // 得到 v * gk
                        gk_next <= mul_gk_res;  // 得到新的 gk
                        state <= S_CALC_ADD; 
                    end else wait_cnt <= wait_cnt - 1;
                end
                
                S_CALC_ADD: begin
                    // 计算加减结果 (组合逻辑)
                    res_u <= mod_add_out; 
                    res_v <= mod_sub_out; 
                    state <= S_WRITE_UV;
                end
                
                S_WRITE_UV: begin
                    // 并行写回
                    ram_we_a <= 1; ram_addr_a <= i + j;       ram_wdata_a <= res_u;
                    ram_we_b <= 1; ram_addr_b <= i + j + mid; ram_wdata_b <= res_v;
                    
                    // 更新状态
                    gk <= gk_next; 
                    j <= j + 1; 
                    state <= S_LOOP_J;
                end

                // ==========================================
                // 4. Post-Process (保持原逻辑: data*factor, factor*G_INV)
                // ==========================================
                S_POST_READ: begin 
                    ram_we_a <= 0; ram_we_b <= 0;
                    ram_addr_a <= cnt[7:0]; 
                    state <= S_POST_READ_WAIT; 
                end
                S_POST_READ_WAIT: state <= S_POST_MUL_DAT;
                
                S_POST_MUL_DAT: begin
                    // 启动乘法 1: data * post_factor
                    mul_op1 <= ram_rdata_a; 
                    mul_op2 <= post_factor;
                    
                    // 启动乘法 2: post_factor * G_INV (更新因子)
                    mul_gk_op1 <= post_factor; 
                    mul_gk_op2 <= G_INV;
                    
                    wait_cnt <= MUL_LATENCY; 
                    state <= S_POST_WAIT_DAT;
                end
                
                S_POST_WAIT_DAT: begin
                    if (wait_cnt == 0) begin
                        post_cal_data <= mul_res;   // 计算结果
                        post_factor <= mul_gk_res;  // 更新因子
                        state <= S_POST_MUL_G;
                    end else wait_cnt <= wait_cnt - 1;
                end
                
                S_POST_MUL_G: begin 
                    ram_we_a <= 1; 
                    ram_addr_a <= cnt[7:0]; 
                    ram_wdata_a <= post_cal_data; 
                    state <= S_POST_WAIT_G; 
                end
                
                S_POST_WAIT_G: begin
                    ram_we_a <= 0; 
                    cnt <= cnt + 1;
                    if (cnt == 255) begin 
                        cnt <= 0; 
                        state <= S_DONE; 
                    end else state <= S_POST_READ;
                end

                S_DONE: begin 
                    done <= 1; 
                    if (start == 0) state <= S_IDLE; 
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule