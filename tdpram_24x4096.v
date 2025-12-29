
// =============================================================================
// Helper: 24x4096 RAM
// =============================================================================
module tdpram_24x4096 (
    input wire clk,
    input wire we_a, input wire [11:0] addr_a, input wire [23:0] din_a, output reg [23:0] dout_a,
    input wire we_b, input wire [11:0] addr_b, input wire [23:0] din_b, output reg [23:0] dout_b
);
    reg [23:0] ram [0:4095];
    always @(posedge clk) begin
        if (we_a) ram[addr_a] <= din_a;
        dout_a <= ram[addr_a];
    end
    always @(posedge clk) begin
        if (we_b) ram[addr_b] <= din_b;
        dout_b <= ram[addr_b];
    end
endmodule