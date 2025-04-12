`timescale 1ns / 1ps

module task1rom(
    output reg [9:0] read_data,
    input wire clk,
    input wire [9:0] address
    );
    reg [9:0] memory[1023:0];
    
    initial begin
        // s0 is amount of elements in the array.
        // t3 is the array address for the elements.
    
        // initializing values.
        memory[0] <= 10'b0;
        memory[1] <= 10'b1101110000; // load s0, 0(t0), 0  // load the amount of passes needed i index
        memory[2] <= 10'b1101101101; // load t3, 1(s3), 0  // load the array address for the first value
        
        //outer loop 
        // loop1:
        memory[3] <= 10'b0000101001; // sub t1, t1, t1, 0  // subtracting t1 to make it zero for the set less than.
        memory[4] <= 10'b1010110011; // beq t1, s0, done2, 0  // check whether we are at zero.
        memory[5] <= 10'b0111010011; // addi s0, s0, -1, 1    // just subtracting the index early on so it does not have to be done elsewhere.
        memory[6] <= 10'b1101111010; // load s1, 2(t0), 0  // loading the j index for the inner loop.
        memory[7] <= 10'b1000001001; // jump loop2
    
        // done2:
        memory[8] <= 10'b1000011111; // jump done
        
        //inner loop
        // loop2:
        memory[9] <= 10'b0100111010; // beq t1, s1, decrement, 0
        memory[10] <= 10'b1000010010; // jump continue1
    
        // decrement: when it the inner loop is done come here and subtracts the orginal value 
        memory[11] <= 10'b1101101100; // load s1, 2(t0), 0
        memory[12] <= 10'b0110000001; // addi s1, s1, -1, 0
        memory[13] <= 10'b0000000000; // store s1, 2(t0), 0
        memory[14] <= 10'b0000000000; // load t3, 1(s3), 1
        memory[15] <= 10'b0000000000; // addi t3, t3, 1, 1
        memory[16] <= 10'b0000000000; // store t3, 1(s3), 1
        memory[17] <= 10'b1000000011; // jump loop1
    
        // continue1: this is where the compare and swap happen
        memory[18] <= 10'b1111100101; // load s2, 0(t3), 1
        memory[19] <= 10'b1111110110; // load t2, 1(t3), 1
        memory[20] <= 10'b0000010110; // slt s2, s2, t2, 1
        memory[21] <= 10'b1011011110; // beq s2, s3, jincrement, 1
        memory[22] <= 10'b1000011001; // jump continue2
    
        // jincrement:
        memory[23] <= 10'b1101110110; // addi t3, t3, 1, 1
        memory[24] <= 10'b1000010010; // jump continue1
    
        // continue2:
        memory[25] <= 10'b0000000000; // load s2, 0(t3), 1
        memory[26] <= 10'b1101110110; // store s2, 1(t3), 1
        memory[27] <= 10'b0111010101; // store t2, 0(t3), 1
        memory[28] <= 10'b0001001100; // addi s1, s1, -1, 0
        memory[39] <= 10'b0000000000; // addi t3, t3, 1, 1
        memory[30] <= 10'b1000001001; // jump loop2
    
        // done:
        memory[31] <= 10'b0010000010; // halt
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