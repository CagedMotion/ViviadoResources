`timescale 1ns / 1ps

module task3rom(
    output reg [9:0] read_data,
    input wire clk,
    input wire [9:0] address
    );
    reg [9:0] memory[1023:0];
    
    initial begin
        memory[1] <= 10'b1100110100; // load s2, 0(t3), 1; | this is the first array address we load to get the address to start pulling values in.
        memory[2] <= 10'b1100111101; // load s3, 1(t3), 1; | this is the second array address from loading and storing the values from the first array.
        memory[3] <= 10'b0000000000;
        memory[4] <= 10'b1101000100; // load t2, 0(s2), 1; : loop | start of the loop which is loading the first value from array 1.
        memory[5] <= 10'b0000101101; // sub t3, t3, t3, 1; | making sure there is zero in the register we will compare it is complete.
        memory[6] <= 10'b0010000011; // nop
        memory[7] <= 10'b1111100100; // store t2, 0(s3), 1; | storing the second value in array 2.
        memory[8] <= 10'b0111010101; // addi s2, s2, 1, 1;
        memory[9] <= 10'b0111111101; // addi s3, s3, 1, 1;                   
        memory[10] <= 10'b1010100110; // beq t3, t2, complete mem[14], 1; offset of 2 for the branch.
        memory[11] <= 10'b1000000100; // jump loop mem[5] | back to loop start in rom.
        memory[12] <= 10'b0010000010; // halt : complete

        
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