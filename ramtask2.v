`timescale 1ns / 1ps

module ramtask2(
    output wire [9:0] rdata,   // 10-bit data output
    input  wire       clk,     // Clock signal
    input  wire       we,      // Write enable
    input  wire [9:0] address, // 10-bit address input
    input  wire [9:0] wdata    // 10-bit write data
);

    // Memory array: 1024 entries of 10 bits each.
    reg [9:0] ram[1023:0];
    
    initial begin
        ram[0] = 10'b0000001010; //address location for where the numbers are located.
        ram[1] = 10'b1001000000;
        ram[2] = 10'b0101001011;
        ram[10] = 10'b0000000101; // value 5
        ram[11] = 10'b1111111101; // value -3
        ram[12] = 10'b0000000000; 
     end

    // Asynchronous read: rdata immediately reflects the memory content at "address"
    assign rdata = ram[address];

    // Synchronous write: On the rising edge, if "we" is asserted,
    // write "wdata" into the memory at the given "address".
    always @(posedge clk) begin
        if (we)
            ram[address] <= wdata;
    end
endmodule
