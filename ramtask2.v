`timescale 1ns / 1ps

module ramtask2(
    inout wire [19:0] data,   // 10-bit data output
    output reg mem_ready,
    input  wire       clk,     // Clock signal
    input  wire       we,      // Write enable
    input  wire [9:0] address // 10-bit address input
);

    // Memory array: 1024 entries of 10 bits each.
    reg [9:0] ram[1023:0];
    
    initial begin
        mem_ready = 1'b1;
        ram[0] = 10'b0000001010; //address location for where the numbers are located.
        ram[1] = 10'b1001000000;
        ram[2] = 10'b0101001011;
        ram[10] = 10'b0000000101; // value 5
        ram[11] = 10'b0000000011; // value 3
//        ram[10] = 10'b1111111011; // value -5
//        ram[11] = 10'b1111111101; // value -3
        ram[12] = 10'b0000000000; 
        
        ram[50] = 10'b0000001001;
        ram[51] = 10'b0000000001;
        ram[52] = 10'b0000000001;
        ram[53] = 10'b0000000001;
        ram[54] = 10'b0000000001;
        ram[55] = 10'b0000000001;
        ram[60] = 10'b0000000001;
        ram[66] = 10'b0000000001;
        ram[67] = 10'b0000000001;
        ram[69] = 10'b0000000001;
        ram[70] = 10'b0000000001;
        ram[71] = 10'b0000000001;
        ram[82] = 10'b0000000001;
        ram[83] = 10'b0000000010;
        ram[84] = 10'b0000000000;
        ram[85] = 10'b0000000100;
        ram[94] = 10'b0000000001;
        ram[95] = 10'b0000000001;
     end

    wire [8:0] temp;
    assign temp = address [9:1];
    
    assign data = (we == 1'b1) ?
        20'bzzzzzzzzzz_zzzzzzzzzz :
        {ram[{temp[8:0], 1'b1}], ram[{temp[8:0], 1'b0}]};
        
    
//    assign rdata = ram[address];
    // Synchronous write: On the rising edge, if "we" is asserted,
    // write "wdata" into the memory at the given "address".
    
    
        reg state, next_state;
    always @() begin
        next_state = 1'b0;
        case (state)
            1'b0: begin
                if (mem_ready==1'b1) begin
                   mem_ready=1'b0;
                   next_state = 1'b1;
                end
            end
            1'b1: begin
                next_state = 1'b0;
                mem_ready=1'b1;
            end
        endcase 
        
    end
    
    
    
    
    always @(posedge clk) begin
        state = next_state;
        if (we) begin
            if (mem_ready == 1'b0) begin
                mem_ready = 1'b1;
                if (address[0]==1'b1) begin
                    ram[address] <= data[19:10];
                end else begin
                    ram[address] <= data[9:0];
                end
            end else begin
                mem_ready = 1'b0;
            end
        end
    end
endmodule
