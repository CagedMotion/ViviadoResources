`timescale 1ns / 1ps

module ramtask3(
    output wire [9:0] rdata,   // 10-bit data output
    input  wire       clk,     // Clock signal
    input  wire       we,      // Write enable
    input  wire [9:0] address, // 10-bit address input
    input  wire [9:0] wdata    // 10-bit write data
);

    // Memory array: 1024 entries of 10 bits each.
    reg [9:0] ram[1023:0];
    
    initial begin
        ram[0] = 10'b1001000000; // ram location 576 is the start of the nul 
        ram[1] = 10'b1001000000; 
        //ram[2] = 10'b1000010000;//ARRAY 1 ADDRESS
        ram[2] = 10'b0000000000;
        ram[3] = 10'b0001010111;
        ram[4] = 10'b0001100001;
        ram[5] = 10'b0001100110;
        ram[6] = 10'b0001100110;
        ram[7] = 10'b0001101100;
        ram[8] = 10'b0001100101;
        ram[9] = 10'b0001110011;
        ram[10] = 10'b0001000001;
        ram[11] = 10'b0001101110;
        ram[12] = 10'b0001100100;
        ram[13] = 10'b0001010000;
        ram[14] = 10'b0001100001;
        ram[15] = 10'b0001101110;
        ram[16] = 10'b0001100011;
        ram[17] = 10'b0001100001;
        ram[18] = 10'b0001101011;
        ram[19] = 10'b0001100101;
        ram[20] = 10'b0001110011;
        ram[21] = 10'b0000000000;
        ram[22] = 10'b1001000000;
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