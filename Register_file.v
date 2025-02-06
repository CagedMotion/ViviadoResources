`timescale 1 ns/1 ps

module register_file(
    input  wire        clk,    // Clock signal
    input  wire        rst,    // Synchronous reset
    input  wire        we,     // Write enable signal
    input  wire [2:0]  waddr,  // 3-bit write address (0 to 7, but only 0-3 are writeable)
    input  wire [9:0]  wdata,  // 10-bit data to write
    input  wire [2:0]  raddr1, // 3-bit read address for read port 1 (can be 0-7)
    input  wire [2:0]  raddr2, // 3-bit read address for read port 2 (can be 0-7)
    output wire [9:0]  rdata1, // 10-bit output from read port 1
    output wire [9:0]  rdata2  // 10-bit output from read port 2
);

    // Declare an array of 8 registers, each 10 bits wide.
    reg [9:0] registers [7:0];
    integer i;
    
    // Synchronous reset and write operation.
    // On reset, all registers are cleared.
    // On a write, only registers with an address from 0 to 3 are updated.
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 8; i = i + 1)
                registers[i] <= 10'b0;
        end else if (we) begin
            // Only allow writes to registers 0-3 (the writeable subset)
            if (waddr < 3'd4)
                registers[waddr] <= wdata;
        end
    end

    // Asynchronous (combinational) reads.
    assign rdata1 = registers[raddr1];
    assign rdata2 = registers[raddr2];

endmodule
