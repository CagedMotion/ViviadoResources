`timescale 1ns / 1ps

module task3rom(
    output reg [9:0] read_data,
    input wire clk,
    input wire [9:0] address
    );
    reg [9:0] memory[1023:0];
    
    initial begin
        memory[0] <= 10'b0000000000;
        memory[0] <= 10'b1100010000; // load s0, 0(t0);
        memory[1] <= 10'b1100011001; // load s1, 1(t0);
        memory[2] <= 10'b1101011100; // load s3, 0(s2);
        memory[3] <= 10'b0001010101; // add s2, s2, 1
        memory[4] <= 10'b1101001000;
        memory[5] <= 10'b1111101000;
        memory[6] <= 10'b0111010001;
        memory[7] <= 10'b1101101100;
        memory[8] <= 10'b0111111101;
        memory[9] <= 10'b1010110110;
        memory[10] <= 10'b1001111010;
        memory[11] <= 10'b0010000010;
    end
    
    always @(*) begin
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