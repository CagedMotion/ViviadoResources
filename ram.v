`timescale 1ns / 1ps

module ram(
    output wire [9:0] rdata,   // 10-bit data output
    input  wire       clk,     // Clock signal
    input  wire       we,      // Write enable
    input  wire [9:0] address, // 10-bit address input
    input  wire [9:0] wdata    // 10-bit write data
);

    // Memory array: 1024 entries of 10 bits each.
    reg [9:0] ram[1023:0];

    // Asynchronous read: rdata immediately reflects the memory content at "address"
    assign rdata = ram[address];

    // Synchronous write: On the rising edge, if "we" is asserted,
    // write "wdata" into the memory at the given "address".
    always @(posedge clk) begin
        if (we)
            ram[address] <= wdata;
    end

endmodule

module tb_ram();
    wire [9:0] rdata;
    reg clk, we;
    reg [9:0] address;
    reg [9:0] wdata;
    
    ram dut(.rdata(rdata), .clk(clk), .we(we), .address(address), .wdata(wdata));
    
    parameter PERIOD = 10;
    initial clk = 1'b0;
    always #(PERIOD/2) clk = ~clk;
    
    initial begin
        // Basic write operations
        wdata = 10'h01;
        address = 10'd0;
        we = 1'b1;
        #PERIOD;

        wdata = 10'h02;
        address = 10'd1;     
        #PERIOD;

        wdata = 10'h03;
        address = 10'd2;     
        #PERIOD;

        // Read operations for addresses 0, 1, 2
        address = 10'd0;
        we = 1'b0;
        #PERIOD;

        address = 10'd1;
        #PERIOD;

        address = 10'd2;
        #PERIOD;

        // Additional write/read cycle
        wdata = 10'h04;
        address = 10'd3;
        we = 1'b1;
        #PERIOD;

        address = 10'd1;
        we = 1'b0;
        #PERIOD;

        // New test: write and read at the end address (1023)
        wdata = 10'h3FF; // Maximum 10-bit value
        address = 10'd1023;
        we = 1'b1;
        #PERIOD;

        we = 1'b0;
        address = 10'd1023;
        #PERIOD;
    end
endmodule