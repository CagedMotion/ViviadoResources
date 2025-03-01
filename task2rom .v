`timescale 1ns / 1ps

module task2rom(
    output reg [9:0] read_data,
    input wire clk,
    input wire [9:0] address
    );
    reg [9:0] memory[1023:0];
    
    initial begin
        memory[0] <= 10'b0000000001; //sub t0, t0, t0, 0
        memory[1] <= 10'b1101010000; //load s0, 0(s0), 0 
        memory[2] <= 10'b1101011001; //load s1, 0(s0), 0
        memory[3] <= 10'b0000111010; //slt t1, s1, t0, 0
        memory[4] <= 10'b1010100010; //beq t1, t0, notneg 2 offset
        memory[5] <= 10'b1000000111; //jump isneg 2 offest
        memory[6] <= 10'b1000001011; // jump mul_start 4 offeset : notneg
        memory[7] <= 10'b1100110000; // load t1, 0(s0) : isneg
        memory[8] <= 10'b0000100001; // sub t1, t0, t1
        memory[9] <= 10'b0001100001; // store t1, 0(s0)
        memory[10] <= 10'b1110110000; //sub s1, t0, s1
        memory[11] <= 10'b0000000001; //sub t0, t0, t0 : mult_start
        memory[12] <= 10'b1101110000; //load t1, 0(s0)
        memory[13] <= 10'b1011100010; //beq s1, t0, look_end : check_value
        memory[14] <= 10'b1000010000; //jump mult_loop
        memory[15] <= 10'b1000010011; //jump endmultiply : look_end
        memory[16] <= 10'b0110100000; // add t1, t0, t1 : mult_loop
        memory[17] <= 10'b0111111011; // addi s1, s1, -1
        memory[18] <= 10'b1000001101; // jump check_value
        memory[19] <= 10'b0110101010; // addi t1, t1, -2 : endmultiply
        memory[20] <= 10'b0110101010; // addi t1, t1, -2
        memory[21] <= 10'b1110110011; // store s3, 2(s0)
        memory[22] <= 10'b0010000010; // halt
        
    end
    
    always @(*) begin
        read_data <= memory[address];
    end
endmodule

module tbtask2rom();
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