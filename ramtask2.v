`timescale 1ns / 1ps

module ramtask2(
    output wire [19:0] rdata,   // 10-bit data output
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
        ram[11] = 10'b0000000011; // value 3
//        ram[10] = 10'b1111111011; // value -5
//        ram[11] = 10'b1111111101; // value -3
        ram[12] = 10'b0000000000; 
        
        ram[50] = 10'b1010001000;
        ram[51] = 10'b1000001011;
        ram[52] = 10'b0001111001;
        ram[53] = 10'b0001101001;
        ram[54] = 10'b0101010110;
        ram[55] = 10'b0110110011;
     end

    wire [8:0] temp;
    assign temp = address [9:1];
    // Asynchronous read: rdata immediately reflects the memory content at "address"
    assign rdata = {ram[{temp[8:0], 1'b1}], ram[{temp[8:0], 1'b0}]};

    // Synchronous write: On the rising edge, if "we" is asserted,
    // write "wdata" into the memory at the given "address".
    always @(posedge clk) begin
        if (we)
            ram[address] <= wdata;
    end
endmodule
