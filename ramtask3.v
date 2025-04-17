`timescale 1ns / 1ps

module ramtask3_for_cache(
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
        ram[0] = 10'b0000000110; // ram location 5 is the start of the nul 
        ram[1] = 10'b0000110010; // location 50
        ram[2] = 10'b0000000000;
        ram[4] = 10'b0000000000;
        ram[3] = 10'b0000010100; 
        ram[5] = 10'b0000000000;
        ram[6] = 10'b0001010111; //W
        ram[7] = 10'b0001100001; //A
        ram[8] = 10'b0001100110; //F
        ram[9] = 10'b0001100110; //F
        ram[10] = 10'b0001101100; //L
        ram[11] = 10'b0001100101; //E
        ram[12] = 10'b0001110011; //s
        ram[13] = 10'b0001000001; //A
        ram[14] = 10'b0001101110; //N
        ram[15] = 10'b0001100100; //D
        ram[16] = 10'b0001010000; //P
        ram[17] = 10'b0001100001; //A
        ram[18] = 10'b0001101110; //N
        ram[19] = 10'b0001100011; //C
        ram[20] = 10'b0001100001; //A
        ram[21] = 10'b0001101011; //K
        ram[22] = 10'b0001100101; //E
        ram[23] = 10'b0001110011; //S
        ram[24] = 10'b0000000000; 
        ram[25] = 10'b1001000000;
        
        for(i=26; i<1024; i=i+1)
            ram[i] = 10'b0;
        
        
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