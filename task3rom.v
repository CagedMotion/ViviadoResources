`timescale 1ns / 1ps

module task3rom(
    output reg [9:0] read_data,
    input wire clk,
    input wire [9:0] address
    );
    reg [9:0] memory[1023:0];
    
    initial begin
        memory[0] <= 10'b1100010000; // load s0, 0(t0), 0; | this is the first array address we load to get the address to start pulling values in.
        memory[1] <= 10'b1100011001; // load s1, 1(t0), 0; | this is the second array address from loading and storing the values from the first array.
        memory[2] <= 10'b0111010101; // addi s2, s2, 1, 1; | meant for the comparison to to know whether or not is is completed. 
        memory[3] <= 10'b0111010101; // addi s2, s2, 1, 1; | just so I can get the address and see the value I want.
        memory[4] <= 10'b0111010101; // addi s2, s2, 1, 1;
        memory[5] <= 10'b1101010100; // load s2, 0(s2), 0; | this is where we put the counter for how many string values need to be moved.
        memory[6] <= 10'b0000101101; // sub t3, t3, t3, 1; | making sure there is zero in the register we will compare it is complete.
        memory[7] <= 10'b1101001000; // load t1, 0(s0), 0; : loop | start of the loop which is loading the first value from array 1.
        memory[8] <= 10'b1111101000; // store t1, 0(s1), 0; | storing the second value in array 2.
        memory[9] <= 10'b0111010001; // addi s0, s0, 1, 0;
        memory[10] <= 10'b0111111001; // addi s1, s1, 1, 0;
        memory[11] <= 10'b0111010111; // addi s2, s2, -1, 1;                        
        memory[12] <= 10'b1011001110; // beq t3, s2, complete mem[13], 1; offset of 2 for the branch.
        memory[13] <= 10'b1000000111; // jump loop mem[7] | back to loop start in rom.
        memory[14] <= 10'b0010000010; // halt : complete
        
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