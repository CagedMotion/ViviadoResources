`timescale 1ns / 1ps

module task1rom(
    output reg [9:0] read_data,
    input wire clk,
    input wire [9:0] address
    );
    reg [9:0] memory[1023:0];
    
    initial begin
        //loop1:
        memory[0] <= 10'b0000101001; //sub t1, t1, t1, 0;
        memory[0] <= 10'b0000010010; //slt t1, t0, t1, 0;
        memory[0] <= 10'b1010110011; //beq t1, s1, done2, 0;
        memory[0] <= 10'b0110010110; //addi s2, t2, -1, 1;
        memory[0] <= 10'b1000000111; //jump loop2;
        
        memory[0] <= 10'b1000101110; //jump done;
        
        //loop2:
        memory[0] <= 10'b0001111101; //sub s3, s3, s3, 1;
        memory[0] <= 10'b1111101100; //store t3, 0(s3), 1;
        memory[0] <= 10'b0001011110; //slt t3, s2, s3, 1;
        memory[0] <= 10'b0100111110; //bne t3, s3, increment, 1;
        memory[0] <= 10'b1000001111; //jump continue1;
        
        //increment
        memory[0] <= 10'b1101101100; //load t3, 0(s3), 1;
        memory[0] <= 10'b0110000001; //addi t0, t0, 1, 0;
        memory[0] <= 10'b0110000101; //addi t2, t2, 1, 1;
        memory[0] <= 10'b1000000001; //jump loop1;
        
        //continue1
        memory[0] <= 10'b1101101100; //load t3, 0(s3), 1;
        memory[0] <= 10'b1111100101; //store t2, 1(s3), 1;
        memory[0] <= 10'b1111110110; //store s2, 2(s3), 1;
        memory[0] <= 10'b0001001100; //add t3, s2, t3, 1;
        memory[0] <= 10'b1100110100; //load t2, 0(t3), 1;
        memory[0] <= 10'b0000110101; //sub t3, t3, s2, 1;
        memory[0] <= 10'b0111010101; //addi s2, s2, 1, 1; 
        memory[0] <= 10'b0001001100; //add t3, s2, t3, 1;
        memory[0] <= 10'b1100110100; //load s2, 0(t3), 1;
        memory[0] <= 10'b0000110101; //sub t3, t3, s2, 1;
        memory[0] <= 10'b0000010110; //slt s2, s2, t2, 1;
        memory[0] <= 10'b1011011110; //beq s2, s3, jinccrement, 1;
        memory[0] <= 10'b1000011111; //jump continue2;
        
        //jincrement:
        memory[0] <= 10'b1101100101; //load t2, 1(s3), 1;
        memory[0] <= 10'b1101110110; //load s2, 2(s3), 1;
        memory[0] <= 10'b1000001011; //jump increment;
        
        //continue2:
        memory[0] <= 10'b1101110110; //load s2, 2(s3), 1;
        memory[0] <= 10'b0111010101; //addi s2, s2, 1, 1;
        memory[0] <= 10'b0001001100; //add t3, s2, t3, 1;
        memory[0] <= 10'b1100111100; //load s3, s2(t3) 1;
        memory[0] <= 10'b0000110101; //sub t3, t3, s2, 1;
        memory[0] <= 10'b0001001100; //add t3, s2, t3, 1;
        memory[0] <= 10'b1110100100; //store t2, s2(t3), 1;
        memory[0] <= 10'b0000110101; //sub t3, t3, s2, 1;
        memory[0] <= 10'b0111010110; //addi s2, s2, -1, 1;
        memory[0] <= 10'b0001001100; //add t3, s2, t3, 1;
        memory[0] <= 10'b1110111100; //store, s3, s2(t3), 1;
        memory[0] <= 10'b0000110101; //sub t3, t3, s2, 1;
        memory[0] <= 10'b0111010110; //addi s2, s2, -1, 1;
        memory[0] <= 10'b1101100101; //load t2, 1(s3), 1;
        memory[0] <= 10'b1000000110; //jump loop2;
        
        //done:
        memory[0] <= 10'b0010000010; //halt
    end
    
    always @(*) begin
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