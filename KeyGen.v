`timescale 1ns / 1ps

module KeyGen (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          start,
    input  wire [255:0]  seed,
    input  wire [2:0]    security_level, // 2, 3, or 5
    
    output reg           done,
    output reg           busy,
    
    // --- Output A (Stream) ---
    output wire [22:0]   A_data,
    output wire          A_valid,
    output reg  [2:0]    A_row_idx,
    output reg  [2:0]    A_col_idx,
    
    // --- Output s1/s2 (Raw Stream) ---
    output wire [3:0]    s_raw_data,
    output wire          s_raw_valid,
    output reg  [3:0]    s_poly_idx,
    output wire          s_is_s2,     
    
    // --- Output s1_post/s2_post ---
    output wire [23:0]   s_post_data,
    output reg           s_post_valid,
    output reg  [3:0]    s_post_poly_idx,
    output wire          s_post_is_s2,

    // --- Output A_S1 Result ---
    output wire          o_AS1_valid,
    output wire [23:0]   o_AS1_data,
    output wire [7:0]    o_AS1_idx,

    // --- Debug: Intermediate t (Before Power2Round) ---
    output reg           o_dbg_intt_valid,
    output wire [23:0]   o_dbg_intt_data,
    output wire [7:0]    o_dbg_intt_idx,

    // --- [NEW] Final Result: t1 and t0 (After Power2Round) ---
    output wire          o_p2r_valid,
    output wire [23:0]   o_t1_data,
    output wire [23:0]   o_t0_data,
    output reg  [7:0]    o_p2r_idx
);

    // =========================================================================
    // Parameter & Configuration
    // =========================================================================
    localparam [23:0] Q = 24'd8380417;

    reg [3:0] K_PARAM;
    reg [3:0] L_PARAM;
    reg [3:0] ETA_VAL; 

    always @(*) begin
        case (security_level)
            3'd3: begin // Level 3
                K_PARAM = 6;
                L_PARAM = 5;
                ETA_VAL = 4;
            end
            3'd5: begin // Level 5
                K_PARAM = 8;
                L_PARAM = 7;
                ETA_VAL = 2;
            end
            default: begin // Level 2
                K_PARAM = 4;
                L_PARAM = 4;
                ETA_VAL = 2;
            end
        endcase
    end

    // =========================================================================
    // FSM States
    // =========================================================================
    localparam S_IDLE           = 6'd0;
    localparam S_EXPAND_SEED    = 6'd1;
    localparam S_WAIT_EXPAND    = 6'd2;
    
    // s1/s2 Generation States
    localparam S_S_INIT         = 6'd3;
    localparam S_S_START        = 6'd4;
    localparam S_S_WAIT_ACK     = 6'd5;
    localparam S_S_FILL_RAM     = 6'd6;
    localparam S_S_NTT_START    = 6'd7;
    localparam S_S_NTT_RUN      = 6'd8;
    localparam S_S_DUMP_INIT    = 6'd9;
    localparam S_S_DUMP_RUN     = 6'd10;
    localparam S_S_DUMP_WAIT    = 6'd11;
    localparam S_S_NEXT         = 6'd12;
    
    // Matrix A Generation & Multiplication
    localparam S_A_INIT         = 6'd13;
    localparam S_A_START        = 6'd14;
    localparam S_A_WAIT_ACK     = 6'd15;
    localparam S_A_RUN          = 6'd16;
    localparam S_A_WAIT_RESULT  = 6'd17; 
    
    // INTT States
    localparam S_A_INTT_START   = 6'd18; 
    localparam S_A_INTT_RUN     = 6'd19; 
    localparam S_A_ROW_DONE     = 6'd20;
    
    // Final Calculation State
    localparam S_A_INTT_CHECK   = 6'd21; 

    localparam S_DONE           = 6'd31;

    reg [5:0] state;

    // =========================================================================
    // Internal Signals
    // =========================================================================
    
    reg  shake_seed_start;
    wire shake_seed_busy;
    wire shake_seed_valid;
    wire [1023:0] shake_seed_out;
    
    reg [255:0] rho;
    reg [511:0] rho_prime;
    reg [255:0] key_K; 

    reg  rejsam_a_start;
    wire rejsam_a_done;
    wire rejsam_a_valid;
    wire [22:0] rejsam_a_data;
    
    reg  rejsam_s_start;
    wire rejsam_s_done;
    wire rejsam_s_valid;
    wire [3:0]  rejsam_s_data;

    // --- NTT/INTT Signals ---
    reg  ntt_start;
    wire ntt_done;
    reg  intt_start;
    wire intt_done;
    
    wire [7:0]  ntt_ram_addr_a, ntt_ram_addr_b;
    wire        ntt_ram_we_a, ntt_ram_we_b;
    wire [23:0] ntt_ram_wdata_a, ntt_ram_wdata_b;
    wire [23:0] ntt_ram_rdata_a, ntt_ram_rdata_b;

    wire [7:0]  intt_ram_addr_a, intt_ram_addr_b;
    wire        intt_ram_we_a, intt_ram_we_b;
    wire [23:0] intt_ram_wdata_a, intt_ram_wdata_b;
    wire [23:0] intt_ram_rdata_a, intt_ram_rdata_b;
    
    // RAM Interface Signals
    reg  [7:0]  top_ram_addr;
    reg         top_ram_we;
    reg  [23:0] top_ram_wdata;
    
    reg  [7:0]  ram_addr_a, ram_addr_b;
    reg         ram_we_a, ram_we_b;
    reg  [23:0] ram_wdata_a, ram_wdata_b;
    wire [23:0] ram_rdata_a, ram_rdata_b;

    reg [8:0] s_sample_cnt;
    reg [8:0] dump_cnt;
    reg ram_read_req;

    reg [7:0] m_cnt;
    reg [7:0] s_dump_addr_reg;
    reg [8:0] as1_out_cnt; 
    reg [8:0] intt_chk_cnt; 

    // s1 Storage Signals
    wire [7:0]  s1_ram_addr;
    wire [3:0]  s1_ram_sel;
    wire [23:0] s1_ram_rdata;
    
    // Accumulator RAM Signals
    wire [7:0]  acc_raddr;
    wire [7:0]  acc_waddr;
    wire        acc_we;
    wire [23:0] acc_wdata;
    wire [23:0] acc_rdata;

    // =========================================================================
    // Module Instantiations
    // =========================================================================

    SHAKE256 #( .RATE(1088), .CAPACITY(512), .OUTPUT_LEN_BYTES(128), .ABSORB_LEN(256) ) u_shake_expand (
        .clk(clk), .rst_n(rst_n),
        .i_start(shake_seed_start), .i_seed(seed),
        .o_busy(shake_seed_busy), .i_squeeze_req(1'b0),
        .o_squeeze_valid(shake_seed_valid), .o_squeeze_data(shake_seed_out)
    );

    Rejsam_a u_rejsam_a (
        .clk(clk), .rst_n(rst_n),
        .i_start(rejsam_a_start), .i_rho(rho),
        .i_row({5'd0, A_row_idx}), .i_column({5'd0, A_col_idx}),
        .o_coeff_valid(rejsam_a_valid), .o_coeff_data(rejsam_a_data),
        .o_done(rejsam_a_done)
    );

    Rejsam_s u_rejsam_s (
        .clk(clk), .rst_n(rst_n),
        .i_start(rejsam_s_start), .i_security_level(security_level),
        .i_rho_prime(rho_prime), .i_row({12'd0, s_poly_idx}), 
        .o_coeff_valid(rejsam_s_valid), .o_coeff_data(rejsam_s_data),
        .o_done(rejsam_s_done)
    );

    ntt_core #( .WIDTH(24) ) u_ntt (
        .clk(clk), .rst_n(rst_n),
        .start(ntt_start), .done(ntt_done),
        .ram_addr_a(ntt_ram_addr_a), .ram_we_a(ntt_ram_we_a), .ram_wdata_a(ntt_ram_wdata_a), .ram_rdata_a(ntt_ram_rdata_a),
        .ram_addr_b(ntt_ram_addr_b), .ram_we_b(ntt_ram_we_b), .ram_wdata_b(ntt_ram_wdata_b), .ram_rdata_b(ntt_ram_rdata_b),
        .dbg_gk(), .dbg_g1(), .dbg_val_u(), .dbg_val_v(), .dbg_prod_y(), .dbg_butterfly_done()
    );

    intt_core #( .WIDTH(24) ) u_intt (
        .clk(clk), .rst_n(rst_n),
        .start(intt_start), .done(intt_done),
        .ram_addr_a(intt_ram_addr_a), .ram_we_a(intt_ram_we_a), .ram_wdata_a(intt_ram_wdata_a), .ram_rdata_a(intt_ram_rdata_a),
        .ram_addr_b(intt_ram_addr_b), .ram_we_b(intt_ram_we_b), .ram_wdata_b(intt_ram_wdata_b), .ram_rdata_b(intt_ram_rdata_b),
        .dbg_gk(), .dbg_g1(), .dbg_val_u(), .dbg_val_v(), .dbg_prod_y(), .dbg_butterfly_done()
    );

    // Working RAM
    tdpram_24x256 u_ram (
        .clk(clk),
        .we_a(ram_we_a), .addr_a(ram_addr_a), .din_a(ram_wdata_a), .dout_a(ram_rdata_a),
        .we_b(ram_we_b), .addr_b(ram_addr_b), .din_b(ram_wdata_b), .dout_b(ram_rdata_b)
    );

    assign ntt_ram_rdata_a  = ram_rdata_a;
    assign ntt_ram_rdata_b  = ram_rdata_b;
    assign intt_ram_rdata_a = ram_rdata_a;
    assign intt_ram_rdata_b = ram_rdata_b;

    // --- s1/s2 Storage (Expanded to 16 RAMs) ---
    wire [23:0] s1_bank_dout [0:15];
    wire [7:0]  s1_bank_addr;
    wire        s1_bank_we [0:15];
    wire [23:0] s1_bank_din;

    assign s1_bank_addr = (state == S_A_RUN || state == S_A_WAIT_RESULT) ? s1_ram_addr : 
                          ((state == S_S_DUMP_RUN || state == S_S_DUMP_WAIT) ? s_dump_addr_reg : 
                           (state == S_A_INTT_CHECK ? top_ram_addr : 8'd0));
    
    assign s1_bank_din  = s_post_data;

    genvar k;
    generate
        for (k = 0; k < 16; k = k + 1) begin : gen_s1_ram
            assign s1_bank_we[k] = (state == S_S_DUMP_RUN || state == S_S_DUMP_WAIT) &&
                                   s_post_valid && (s_post_poly_idx == k); 
            
            tdpram_24x256 u_s1_ram_inst (
                .clk(clk),
                .we_a(s1_bank_we[k]), .addr_a(s1_bank_addr), .din_a(s1_bank_din), .dout_a(),
                .we_b(1'b0),          .addr_b(s1_bank_addr), .din_b(24'd0),   .dout_b(s1_bank_dout[k])
            );
        end
    endgenerate

    assign s1_ram_rdata = s1_bank_dout[s1_ram_sel];

    // --- Accumulator RAM ---
    tdpram_24x256 u_acc_ram (
        .clk(clk),
        .we_a(1'b0),    .addr_a(acc_raddr), .din_a(24'd0), .dout_a(acc_rdata),
        .we_b(acc_we),  .addr_b(acc_waddr), .din_b(acc_wdata), .dout_b()
    );

    // --- Matrix Vector Mul Core ---
    MatrixVecMul_Core #( .WIDTH(24) ) u_mat_mul (
        .clk(clk), .rst_n(rst_n),
        .i_A_valid(rejsam_a_valid),
        .i_A_data ({1'b0, rejsam_a_data}), 
        .i_m_idx  (m_cnt),
        .i_j_idx  ({1'b0, A_col_idx}),
        .i_l_param(L_PARAM),
        .o_s1_addr(s1_ram_addr),
        .o_s1_poly_idx(s1_ram_sel),
        .i_s1_rdata(s1_ram_rdata),
        .o_acc_addr(acc_raddr),
        .o_acc_waddr(acc_waddr),
        .o_acc_we(acc_we),
        .o_acc_wdata(acc_wdata),
        .i_acc_rdata(acc_rdata),
        .o_res_valid(o_AS1_valid),
        .o_res_data (o_AS1_data),
        .o_res_m_idx(o_AS1_idx)
    );

    // =========================================================================
    // Logic: t Calculation & Power2Round
    // =========================================================================
    reg [23:0] s_calc_val;
    always @(*) begin
        if ({20'd0, rejsam_s_data} <= {20'd0, ETA_VAL}) begin
            s_calc_val = {20'd0, ETA_VAL} - {20'd0, rejsam_s_data};
        end else begin
            s_calc_val = {20'd0, ETA_VAL} + Q - {20'd0, rejsam_s_data};
        end
    end

    // --- Calculate t = INTT(A*s1) + s2 ---
    wire [23:0] s2_for_t;
    assign s2_for_t = s1_bank_dout[L_PARAM + A_row_idx];

    wire [23:0] t_val;
    mod_add #( .WIDTH(24) ) u_add_t (
        .a(ram_rdata_a), .b(s2_for_t), .q(Q), .res(t_val)
    );
    
    // --- [NEW] Power2Round Instantiation ---
    Power2Round #( .WIDTH(24) ) u_power2round (
        .clk(clk),
        .rst_n(rst_n),
        .i_valid(o_dbg_intt_valid), // Input valid when intermediate t is valid
        .i_data(t_val),
        .o_valid(o_p2r_valid),
        .o_t1(o_t1_data),
        .o_t0(o_t0_data)
    );

    // =========================================================================
    // RAM Mux Logic
    // =========================================================================
    always @(*) begin
        if (state == S_S_NTT_RUN) begin
            ram_addr_a  = ntt_ram_addr_a; ram_we_a = ntt_ram_we_a; ram_wdata_a = ntt_ram_wdata_a;
            ram_addr_b  = ntt_ram_addr_b; ram_we_b = ntt_ram_we_b; ram_wdata_b = ntt_ram_wdata_b;
        end else if (state == S_A_INTT_RUN) begin
            ram_addr_a  = intt_ram_addr_a; ram_we_a = intt_ram_we_a; ram_wdata_a = intt_ram_wdata_a;
            ram_addr_b  = intt_ram_addr_b; ram_we_b = intt_ram_we_b; ram_wdata_b = intt_ram_wdata_b;
        end else if (state == S_A_RUN || state == S_A_WAIT_RESULT) begin
            ram_addr_a  = o_AS1_idx; ram_we_a = o_AS1_valid; ram_wdata_a = o_AS1_data;
            ram_addr_b  = 8'd0; ram_we_b = 1'b0; ram_wdata_b = 24'd0;
        end else if (state == S_A_INTT_CHECK) begin
            ram_addr_a  = top_ram_addr; ram_we_a = 1'b0; ram_wdata_a = 24'd0;
            ram_addr_b  = 8'd0; ram_we_b = 1'b0; ram_wdata_b = 24'd0;
        end else begin
            ram_addr_a  = top_ram_addr; ram_we_a = top_ram_we; ram_wdata_a = top_ram_wdata;
            ram_addr_b  = 8'd0; ram_we_b = 1'b0; ram_wdata_b = 24'd0;
        end
    end

    // =========================================================================
    // Output Assignments
    // =========================================================================
    assign A_data  = rejsam_a_data;
    assign A_valid = rejsam_a_valid;
    
    assign s_raw_data  = rejsam_s_data;
    assign s_raw_valid = rejsam_s_valid;
    assign s_is_s2     = (s_poly_idx >= L_PARAM);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) s_post_valid <= 0;
        else s_post_valid <= ram_read_req; 
    end
    
    assign s_post_data     = ram_rdata_a;
    assign s_post_is_s2    = (s_post_poly_idx >= L_PARAM);

    // Debug: Intermediate t
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) o_dbg_intt_valid <= 0;
        else o_dbg_intt_valid <= (state == S_A_INTT_CHECK) && ram_read_req;
    end
    assign o_dbg_intt_data = t_val; 
    assign o_dbg_intt_idx  = (top_ram_addr == 0) ? 8'd255 : (top_ram_addr - 1'b1);
    
    // [NEW] Output index alignment for Power2Round (delay by 1 cycle)
    always @(posedge clk) begin
        o_p2r_idx <= o_dbg_intt_idx;
    end

    // =========================================================================
    // Main FSM
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            done <= 0; busy <= 0;
            shake_seed_start <= 0;
            rejsam_a_start <= 0; rejsam_s_start <= 0;
            ntt_start <= 0; intt_start <= 0;
            rho <= 0; rho_prime <= 0; key_K <= 0;
            A_row_idx <= 0; A_col_idx <= 0;
            s_poly_idx <= 0; s_post_poly_idx <= 0;
            top_ram_addr <= 0; top_ram_we <= 0; top_ram_wdata <= 0;
            s_sample_cnt <= 0; dump_cnt <= 0; ram_read_req <= 0;
            m_cnt <= 0; s_dump_addr_reg <= 0; 
            as1_out_cnt <= 0;
            intt_chk_cnt <= 0;
        end else begin
            shake_seed_start <= 0;
            rejsam_a_start <= 0; rejsam_s_start <= 0;
            ntt_start <= 0; intt_start <= 0;
            top_ram_we <= 0; ram_read_req <= 0;

            case (state)
                S_IDLE: begin
                    done <= 0;
                    if (start) begin busy <= 1; state <= S_EXPAND_SEED; end
                    else busy <= 0;
                end

                S_EXPAND_SEED: begin shake_seed_start <= 1; state <= S_WAIT_EXPAND; end
                S_WAIT_EXPAND: if (shake_seed_valid) begin rho <= shake_seed_out[255:0]; rho_prime <= shake_seed_out[767:256]; key_K <= shake_seed_out[1023:768]; state <= S_S_INIT; end

                S_S_INIT: begin s_poly_idx <= 0; state <= S_S_START; end
                S_S_START: begin rejsam_s_start <= 1; s_sample_cnt <= 0; state <= S_S_WAIT_ACK; end
                S_S_WAIT_ACK: if (!rejsam_s_done) state <= S_S_FILL_RAM;
                
                S_S_FILL_RAM: begin
                    if (rejsam_s_valid) begin 
                        top_ram_we <= 1; top_ram_addr <= s_sample_cnt[7:0]; 
                        top_ram_wdata <= s_calc_val; s_sample_cnt <= s_sample_cnt + 1; 
                    end
                    if (rejsam_s_done) begin
                        if (s_is_s2) state <= S_S_DUMP_INIT;
                        else state <= S_S_NTT_START;
                    end
                end

                S_S_NTT_START: begin ntt_start <= 1; state <= S_S_NTT_RUN; end
                S_S_NTT_RUN: if (ntt_done) state <= S_S_DUMP_INIT;

                S_S_DUMP_INIT: begin
                    s_post_poly_idx <= s_poly_idx;
                    dump_cnt <= 0;
                    top_ram_we <= 0; top_ram_addr <= 0; 
                    ram_read_req <= 1; 
                    s_dump_addr_reg <= 0;
                    state <= S_S_DUMP_RUN;
                end

                S_S_DUMP_RUN: begin
                    if (dump_cnt < 255) begin
                        top_ram_addr <= dump_cnt[7:0] + 1;
                        ram_read_req <= 1;
                        dump_cnt <= dump_cnt + 1;
                    end else begin
                        ram_read_req <= 0;
                        dump_cnt <= 0;
                        state <= S_S_DUMP_WAIT;
                    end
                    if (s_post_valid) s_dump_addr_reg <= s_dump_addr_reg + 1;
                end
                
                S_S_DUMP_WAIT: begin
                    if (s_post_valid) s_dump_addr_reg <= s_dump_addr_reg + 1;
                    else state <= S_S_NEXT;
                end

                S_S_NEXT: begin
                    if (s_poly_idx < (L_PARAM + K_PARAM - 1)) begin
                        s_poly_idx <= s_poly_idx + 1; state <= S_S_START;
                    end else state <= S_A_INIT;
                end

                // --- Matrix & INTT Logic ---
                S_A_INIT: begin A_row_idx <= 0; A_col_idx <= 0; state <= S_A_START; end
                S_A_START: begin 
                    rejsam_a_start <= 1; m_cnt <= 0; 
                    if (A_col_idx == 0) as1_out_cnt <= 0;
                    state <= S_A_WAIT_ACK; 
                end
                
                S_A_WAIT_ACK: if (!rejsam_a_done) state <= S_A_RUN;

                S_A_RUN: begin
                    if (rejsam_a_valid) m_cnt <= m_cnt + 1;
                    if (o_AS1_valid) as1_out_cnt <= as1_out_cnt + 1;

                    if (rejsam_a_done) begin
                        if (A_col_idx < L_PARAM - 1) begin
                            A_col_idx <= A_col_idx + 1; state <= S_A_START;
                        end else state <= S_A_WAIT_RESULT;
                    end
                end

                S_A_WAIT_RESULT: begin
                    if (o_AS1_valid) as1_out_cnt <= as1_out_cnt + 1;
                    if (as1_out_cnt >= 256 || (o_AS1_valid && as1_out_cnt == 255)) begin
                        state <= S_A_INTT_START;
                    end
                end

                S_A_INTT_START: begin intt_start <= 1; state <= S_A_INTT_RUN; end
                
                S_A_INTT_RUN: begin
                    if (intt_done) begin
                        intt_chk_cnt <= 0;
                        top_ram_addr <= 0; 
                        ram_read_req <= 1; 
                        state <= S_A_INTT_CHECK;
                    end
                end

                S_A_INTT_CHECK: begin
                    if (intt_chk_cnt < 255) begin
                        top_ram_addr <= intt_chk_cnt[7:0] + 1;
                        ram_read_req <= 1;
                        intt_chk_cnt <= intt_chk_cnt + 1;
                    end else begin
                        ram_read_req <= 0;
                        top_ram_addr <= 0; // Fix for last index
                        if (intt_chk_cnt == 256) begin
                            state <= S_A_ROW_DONE;
                            intt_chk_cnt <= 0;
                        end else begin
                            intt_chk_cnt <= intt_chk_cnt + 1;
                        end
                    end
                end

                S_A_ROW_DONE: begin
                    A_col_idx <= 0;
                    if (A_row_idx < K_PARAM - 1) begin
                        A_row_idx <= A_row_idx + 1; state <= S_A_START;
                    end else state <= S_DONE;
                end

                S_DONE: begin done <= 1; busy <= 0; state <= S_IDLE; end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule