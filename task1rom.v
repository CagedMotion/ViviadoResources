`timescale 1ns / 1ps

module task1rom(
    output reg [9:0] read_data,
    input wire clk,
    input wire [9:0] address
    );
    reg [9:0] memory[1023:0];
    
    initial begin
        
        memory[0] <= 10'b1101110000; // load s0, 0(s1), 0
        memory[1] <= 10'b1101101101; // load t3, 1(s3), 0
        
        // loop1:
        memory[2] <= 10'b0000101001; // sub t1, t1, t1, 0
        memory[3] <= 10'b0000001010; // slt t1, t0, t1, 0
        memory[4] <= 10'b1010110011; // beq t1, s1, done2, 0
        memory[5] <= 10'b0110010110; // addi s2, t2, -1, 1
        memory[6] <= 10'b1000001000; // jump loop2
        
        // done2:
        memory[7] <= 10'b1000101110; // jump done
        
        // loop2:
        memory[8] <= 10'b0001111101; // sub s3, s3, s3, 1
        memory[9] <= 10'b1111101100; // store t3, 0(s3), 1
        memory[10] <= 10'b0000101101; // sub t3, t3, t3, 1
        memory[11] <= 10'b0001011110; // slt s3, s2, s3, 1
        memory[12] <= 10'b0100111110; // bne t3, s3, increment, 1
        memory[13] <= 10'b1000010010; // jump continue1
        
        // increment:
        memory[14] <= 10'b1101101100; // load t3, 0(s3), 1
        memory[15] <= 10'b0110000001; // addi t0, t0, 1, 0
        memory[16] <= 10'b0110000101; // addi t2, t2, 1, 1
        memory[17] <= 10'b1000000011; // jump loop1
        
        // continue1:
        memory[18] <= 10'b1101101100; // load t3, 0(s3), 1
        memory[19] <= 10'b1111100101; // store t2, 1(s3), 1
        memory[20] <= 10'b1111110110; // store s2, 2(s3), 1
        memory[21] <= 10'b0001001100; // add t3, s2, t3, 1
        memory[22] <= 10'b1100100100; // load t2, 0(t3), 1
        
        memory[23] <= 10'b0001010111; // nand s2, s2, s2, 1
        memory[24] <= 10'b0111010101; // addi s2, s2, 1, 1
        memory[25] <= 10'b0001001100; // add t3, s2, t3, 1
        
        memory[26] <= 10'b0111010101; // addi s2, s2, 1, 1
        memory[27] <= 10'b0001001100; // add t3, s2, t3, 1
        memory[28] <= 10'b1100110100; // load s2, 0(t3), 1
        
        memory[29] <= 10'b0001010111; // nand s2, s2, s2, 1
        memory[30] <= 10'b0111010101; // addi s2, s2, 1, 1
        memory[31] <= 10'b0001001100; // add t3, s2, t3, 1
        
        memory[32] <= 10'b0000010110; // slt s2, s2, t2, 1
        memory[33] <= 10'b1011011110; // beq s2, s3, jincrement, 1
        memory[34] <= 10'b1000100010; // jump continue2
        
        // jincrement:
        memory[35] <= 10'b1101100101; // load t2, 1(s3), 1
        memory[36] <= 10'b1101110110; // load s2, 2(s3), 1
        memory[37] <= 10'b1000001110; // jump increment
        
        // continue2:
        memory[38] <= 10'b1101110110; // load s2, 2(s3), 1
        memory[39] <= 10'b0111010101; // addi s2, s2, 1, 1
        memory[40] <= 10'b0001001100; // add t3, s2, t3, 1
        memory[41] <= 10'b1100111100; // load s3, s2(t3), 1
        
        memory[42] <= 10'b0001010111; // nand s2, s2, s2, 1
        memory[43] <= 10'b0111010101; // addi s2, s2, 1, 1
        memory[44] <= 10'b0001001100; // add t3, s2, t3, 1
        
        memory[45] <= 10'b0001001100; // add t3, s2, t3, 1
        memory[46] <= 10'b1110100100; // store t2, s2(t3), 1
        
        memory[47] <= 10'b0001010111; // nand s2, s2, s2, 1
        memory[48] <= 10'b0111010101; // addi s2, s2, 1, 1
        memory[49] <= 10'b0001001100; // add t3, s2, t3, 1
        
        memory[50] <= 10'b0111010110; // addi s2, s2, -1, 1
        memory[51] <= 10'b0001001100; // add t3, s2, t3, 1
        memory[52] <= 10'b1110111100; // store s3, s2(t3), 1
        
        memory[53] <= 10'b0001010111; // nand s2, s2, s2, 1
        memory[54] <= 10'b0111010101; // addi s2, s2, 1, 1
        memory[55] <= 10'b0001001100; // add t3, s2, t3, 1
        
        memory[56] <= 10'b0111010110; // addi s2, s2, -1, 1
        memory[57] <= 10'b1101100101; // load t2, 1(s3), 1
        memory[58] <= 10'b1000001001; // jump loop2
        
        // done:
        memory[59] <= 10'b0010000010; // halt
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