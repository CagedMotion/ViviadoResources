`timescale 1ns / 1ps

module ramtask2(
    output wire [9:0] rdata,   // 10-bit data outpu
    input  wire       clk,     // Clock signal     
    input  wire       we,      // Write enable     
    input  wire [9:0] address, // 10-bit address in
    input  wire [9:0] wdata    // 10-bit write data
);

    // Memory array: 1024 entries of 10 bits each.
    reg [9:0] ram[1023:0];
    
    initial begin
    
    
        ram[0] = 10'b0000001010; //address location for where the numbers are located.
        ram[1] = 10'b1001000000;
        ram[2] = 10'b0101001011;
        ram[10] = 10'b0000000101; // value 5
        ram[11] = 10'b0000001010; // value 10
//        ram[10] = 10'b1111111011; // value -5
//        ram[11] = 10'b1111111101; // value -3
        ram[12] = 10'b0000000000; 
        
        ram[50] = 10'b0000001001;
        ram[51] = 10'b0000000001;
        ram[52] = 10'b0000000001;
        ram[53] = 10'b0000000001;
        ram[54] = 10'b0000000001;
        ram[55] = 10'b0000000001;
        ram[60] = 10'b0000000001;
        ram[66] = 10'b0000000001;
        ram[67] = 10'b0000000001;
        ram[69] = 10'b0000000001;
        ram[70] = 10'b0000000001;
        ram[71] = 10'b0000000001;
        ram[82] = 10'b0000000001;
        ram[83] = 10'b0000000010;
        ram[84] = 10'b0000000000;
        ram[85] = 10'b0000000100;
        ram[94] = 10'b0000000001;
        ram[95] = 10'b0000000001;
        ram[148] = 10'b0000010000;
        ram[149] = 10'b0000010001;
        ram[150] = 10'b0000001000;
        ram[151] = 10'b0000001100;
        ram[222] = 10'b0000100000;
        ram[223] = 10'b0000001000;
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