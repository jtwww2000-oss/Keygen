`timescale 1ns / 1ps

module pk_packer (
    input  wire          clk,
    input  wire          rst_n,
    
    // --- 配置 ---
    input  wire          start_pack, // 当 rho 准备好时触发
    
    // --- 输入数据 ---
    input  wire [255:0]  rho,
    input  wire          t1_valid,
    input  wire [9:0]    t1_data,
    
    // --- 输出大位宽公钥 ---
    // Max size for Dilithium5: 256 (rho) + 8*256*10 (t1) = 20736 bits
    output reg [20735:0] o_pk_bus,  
    output reg           o_pk_done    // 当所有数据打包完成时拉高
);

    // 指针，指示当前写入的位置
    // 最大位置 20736，需要 15 bits (2^15 = 32768)
    reg [14:0] write_ptr;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_pk_bus   <= 0;
            write_ptr  <= 0;
            o_pk_done  <= 0;
        end else begin
            // 启动时加载 rho 到低 256 位
            if (start_pack) begin
                o_pk_bus[255:0] <= rho;
                write_ptr       <= 256;
                o_pk_done       <= 0;
                // 清除高位旧数据 (可选)
                o_pk_bus[20735:256] <= 0;
            end
            
            // 当 t1 有效时，拼接到当前指针位置
            if (t1_valid) begin
                // Verilog 的动态位切片语法: [base +: width]
                // 将 10-bit t1 写入当前 write_ptr 开始的 10 位
                o_pk_bus[write_ptr +: 10] <= t1_data;
                
                // 指针步进
                write_ptr <= write_ptr + 10;
            end
            
            // 判断结束条件 (这里简单用外部复位或下一轮 start 来清除，
            // 或者你可以根据 write_ptr 的值来判断 done)
            // Dilithium2: 256 + 4*256*10 = 10496
            // Dilithium3: 256 + 6*256*10 = 15616
            // Dilithium5: 256 + 8*256*10 = 20736
            // 这里我们做一个简单的逻辑：如果在很长一段时间没有 valid 信号，或者 KeyGen 完成了，
            // 上层模块会控制流程。本模块仅负责"累加"。
            // 为了方便调试，我们可以检测特定的 write_ptr 值来置 done，
            // 但为了通用性，建议由 KeyGen 的 done 信号直接驱动外部观察。
        end
    end

endmodule