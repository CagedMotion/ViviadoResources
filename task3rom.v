`timescale 1ns / 1ps

module task3rom(
    output reg [9:0] read_data,
    input wire clk,
    input wire [9:0] address
    );
    reg [9:0] memory[1023:0];
    
    initial begin
        memory[0] <= 10'b0000000001;
        memory[1] <= 10'b0011110001;
        memory[2] <= 10'b1010101010;
    end
    
    always @(posedge clk) begin
        read_data <= memory[address];
    end
endmodule

module tbtask3rom();
    wire [9:0] read_data;
    reg clk;
    reg [9:0] address;
    
    task1rom SUT(.read_data(read_data),.clk(clk),.address(address));
    
    parameter PERIOD = 10;
    initial clk = 1'b0;
    always #(PERIOD/2) clk = ~clk;
    
    initial begin
        address = 10'b0000000000; #PERIOD;
        address = 10'b0000000001; #PERIOD;
        address = 10'b0000000010; #PERIOD;
    end
    
endmodule