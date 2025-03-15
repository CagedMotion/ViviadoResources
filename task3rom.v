`timescale 1ns / 1ps

module task3rom(
    output reg [9:0] read_data,
    input wire clk,
    input wire [9:0] address
    );
    reg [9:0] memory[1023:0];
    
    initial begin
        memory[0] <= 10'b1100010000; // load s0, 0(t0), 0; | this is assuming everything is at zero already and pulling the ram location zero for the first array address.
        memory[1] <= 10'b1100011001; // load s1, 1(t0), 0; | the system is a zero in the beginning and loading the array address 2 to store values in. 
        memory[2] <= 10'b1101110100; // load s2, 0(s3), 1;
        memory[3] <= 10'b0001010101; // sub s2, s2, s2, 1;
        memory[4] <= 10'b1101001000; // load t1, 0(s0), 0;
        memory[5] <= 10'b1111101000; // store t1, 0(s1), 0;
        memory[6] <= 10'b0111010001; // addi s0, s0, 1, 0;
        memory[7] <= 10'b1101101100; // load t3, 0(s3), 1;
        memory[8] <= 10'b0111111101; // addi s3, s3, 1, 1;
        memory[9] <= 10'b1010110110; // beq t3, s2, label, 1; offset of 2 for the branch.
        memory[10] <= 10'b1001111010; // jump label way into the ram to x's currently.
        memory[11] <= 10'b0010000010; // halt.
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