`timescale 1ns / 1ps

module ramtask2_for_cache(
    inout wire [19:0] data,   // 10-bit data output
    input  wire       clk,     // Clock signal
    input  wire       we,      // Write enable
    input  wire [9:0] address, // 10-bit address input
    input wire mem_req,
    output reg mem_ready
);

    // Memory array: 1024 entries of 10 bits each.
    reg [9:0] ram[1023:0];
    
    reg [1:0] state, next_state;
        
    parameter IDLE    = 1'b0; // Memory is idle and ready. mem_ready is high.
    parameter WAIT    = 1'b1; // First delay cycle: mem_ready is low.
    
    integer i;
    initial begin
        state = IDLE;
        next_state = IDLE;
        mem_ready = 1'b1;
    
    
        ram[0] = 10'b0000001010; //address location for where the numbers are located.
        ram[1] = 10'b1001000000;
        ram[2] = 10'b0101001011;
        ram[3] = 10'b0000000000;
        ram[4] = 10'b0000000000;
        ram[5] = 10'b0000000000;
        ram[6] = 10'b0000000000;
        ram[7] = 10'b0000000000;
        ram[8] = 10'b0000000000;
        ram[9] = 10'b0000000000;
        ram[10] = 10'b0000000101; // value 5
        ram[11] = 10'b0000001010; // value 10
//        ram[10] = 10'b1111111011; // value -5
//        ram[11] = 10'b1111111101; // value -3
        ram[12] = 10'b0000000000;
        
        for(i=13; i<1024; i=i+1)
            ram[i] = 10'b0;
        
        
//        ram[50] = 10'b0000001001;
//        ram[51] = 10'b0000000001;
//        ram[52] = 10'b0000000001;
//        ram[53] = 10'b0000000001;
//        ram[54] = 10'b0000000001;
//        ram[55] = 10'b0000000001;
//        ram[60] = 10'b0000000001;
//        ram[66] = 10'b0000000001;
//        ram[67] = 10'b0000000001;
//        ram[69] = 10'b0000000001;
//        ram[70] = 10'b0000000001;
//        ram[71] = 10'b0000000001;
//        ram[82] = 10'b0000000001;
//        ram[83] = 10'b0000000010;
//        ram[84] = 10'b0000000000;
//        ram[85] = 10'b0000000100;
//        ram[94] = 10'b0000000001;
//        ram[95] = 10'b0000000001;
//        ram[148] = 10'b0000010000;
//        ram[149] = 10'b0000010001;
//        ram[150] = 10'b0000001000;
//        ram[151] = 10'b0000001100;
//        ram[222] = 10'b0000100000;
//        ram[223] = 10'b0000001000;
     end

    
    reg [19:0] read_data;
    
     wire [8:0] temp;
    assign temp = address [9:1];    
    
    assign data = (we == 1'b1) ?
        20'bzzzzzzzzzz_zzzzzzzzzz :
        {ram[{temp[8:0], 1'b1}], ram[{temp[8:0], 1'b0}]};
    
    // state machine combinational logic always.
     always @(*) begin
        case (state)
            IDLE: begin
                if (mem_req)
                    next_state = WAIT;
                else
                    next_state = IDLE;
            end
            WAIT: begin
                next_state = IDLE;
            end
            default: next_state = IDLE;
        endcase
    end
    
    //sequential logic for the ram state machine.
    always @(posedge clk) begin
        state <= next_state;
        case (state)
            IDLE: begin
                // Only latch the new operation if a request is active.
                if (mem_req) begin
//                    latched_we <= we;
//                    latched_address <= address;
//                    if (we) begin
//                        latched_write_data <= data; // Latch the write data
//                    end
                    mem_ready <= 1'b0;
                end else begin
                    mem_ready <= 1'b1;
                end
            end
            WAIT: begin
                // During the delay cycle, maintain mem_ready low.
                if (we) begin
                    // Write operation:
                    // If the latched address's bit 0 is 0, write to the lower half; if 1, to the upper half.
                    ram[{address[9:1], 1'b0}] <= data[9:0];
                    ram[{address[9:1], 1'b1}] <= data[19:10];
                end else begin
                    // Read operation:
                    // Combine both 10-bit halves from the selected block.
                    read_data <= { ram[{address[9:1], 1'b1}],
                                   ram[{address[9:1], 1'b0}] };
                end
                mem_ready <= 1'b1;
            end
            default: begin
                mem_ready <= 1'b1;
            end
        endcase
    end
endmodule