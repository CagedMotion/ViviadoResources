`timescale 1ns / 1ps

module task2rom(
    output reg [9:0] read_data,
    input wire [9:0] address
    );
    reg [9:0] memory[1023:0];
            // for problem f=x*y-4
    initial begin
        memory[0] <= 10'b0000000000;                       
        memory[1] <= 10'b0000000001; //sub t0, t0, t0, 0 | subtracting t0 to be only zero in that register.
        memory[2] <= 10'b1101010000; //load s0, 0(s0), 0 | this is loading the value from ram in the ram location zero.
        memory[3] <= 10'b1101011001; //load s1, 1(s0), 0 | this is loading the ram location from s0 of the value Y in the one offset.
        memory[4] <= 10'b0001101010; //slt t1, s1, t1, 0 | this is checking to see if the value is negative or positive as t1 is zero then write t1 value in t1.
        memory[5] <= 10'b1010100010; //beq t1, t0, notneg, 0 | 2 offset | checking if t0 and t1 are equal then adding the 2 offset to the program counter.
        memory[6] <= 10'b1000000111; //jump isneg mem[7] | if not equal it does this jump and just increments program counter from branch 1 then jumps to rom location 7.
        memory[7] <= 10'b1000001011; // jump mul_start mem[11] : notneg | this is if equal makes the jump to rom location 11 to start the code.
        memory[8] <= 10'b1101001000; // load t1, 0(s0), 0 : isneg | loading the x value in ram address location and having that in register t1. starting the inverse process to make them postive values.
        memory[9] <= 10'b0000001001; // sub t1, t0, t1, 0 
        memory[10] <= 10'b0000011001; // sub s1, t0, s1, 0
        memory[11] <= 10'b1110110000; //store s1, 0(s0), 0
        memory[12] <= 10'b0000000001; //sub t0, t0, t0, 0 : mult_start | starting the process making sure t0 is zero in that register.
        memory[13] <= 10'b1101001000; //load t1, 0(s0), 0   | loading the x value into register t1 from ram address location.
        memory[14] <= 10'b1101010000;  //load s0, 0(s0), 0  | loading x value again into register s0 to add to each other then be over written again.
        memory[15] <= 10'b0111111011; // addi s1, s1, -1, 0 | this makes sure the value actually iterates the proper times instead of one extra.
        memory[16] <= 10'b1011100010; //beq s1, t0, look_end, 0 : check_value
        memory[17] <= 10'b1000010010; //jump mult_loop mem[18]
        memory[18] <= 10'b1000010101; //jump endmultiply mem[21] : look_end
        memory[19] <= 10'b0001001000; // add t1, s0, t1, 0 : mult_loop
        memory[20] <= 10'b0111111011; // addi s1, s1, -1, 0
        memory[21] <= 10'b1000001111; // jump check_value
        memory[22] <= 10'b0110101010; // addi t1, t1, -2, 0 : endmultiply
        memory[23] <= 10'b0110101010; // addi t1, t1, -2, 0
        memory[24] <= 10'b0001010001; // sub s0, s0, s0, 0
        memory[25] <= 10'b1101010000; // load s0, 0(s0), 0  | this is cause we cleared s0 during the loop operation to do multiple additions properly.
        memory[26] <= 10'b1111001010; // store t1, 2(s0), 0
        memory[27] <= 10'b0010000010; // halt
          
 //memory[1] <= 10'b0000000000; // just adding as like a nop to wait till the register is set.
//          memory[0] <= 10'b1101010000; //load s0, 0(s0), 0 
//          memory[1] <= 10'b0000110010; // slt t1, s0, t1, 0
//          memory[2] <= 10'b0000110000; // add itself
//          memory[13] <= 10'b0000000000; //add t1, t1, t1, 0   |
//          memory[3] <= 10'b0010000010; //halt
        
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