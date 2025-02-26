`timescale 1ns / 1ps

module task1rom(
    output reg [9:0] read_data,
    input wire clk,
    input wire [9:0] address
    );
    reg [9:0] memory[1023:0];
    
    initial begin
        //store the value 5 in t0
        memory[0] <= 10'b0110000001; //addi t0,t0,1     0000000001
        memory[1] <= 10'b0010000001; //sll t0,t0        0000000010
        memory[2] <= 10'b0010000001; //sll t0,t0        0000000100
        memory[3] <= 10'b0110000001; //addi t0,t0,1     0000000101
        //store the value 4 in t1
        memory[4] <= 10'b0110101001; //addi t1,t1,1     0000000001
        memory[5] <= 10'b0010101001; //sll t1,t1        0000000010
        memory[6] <= 10'b0010101001; //sll t1,t1        0000000100
        //add t0(5) to t1(4) and store the value in t0(9)
        memory[7] <= 10'b0000100000; //add t0,t1        0000001001
//        memory[1000] <= 10'b0000000101; //5
//        memory[1001] <= 10'b0000000100; //4
    end
    
    always @(posedge clk) begin
        read_data <= memory[address];
    end
endmodule

module tbtask1rom();
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