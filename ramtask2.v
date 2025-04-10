`timescale 1ns / 1ps

module ramtask2(
    inout wire [19:0] data,   // 20-bit data output
    output reg mem_ready,
    input  wire       clk,     // Clock signal
    input  wire       we,      // Write enable
    input  wire mem_req,
    input  wire [9:0] address // 10-bit address input
);

    // Memory array: 1024 entries of 10 bits each.
    reg [9:0] ram[1023:0];
    
    initial begin
        mem_ready = 1'b1;
        state = IDLE;
        next_state = IDLE;
        
        
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
    
//    assign rdata = ram[address];
    // Synchronous write: On the rising edge, if "we" is asserted,
    // write "wdata" into the memory at the given "address".
    
    // We'll use three states: IDLE, WAIT, and EXECUTE.
    parameter IDLE    = 2'b00; // Memory is idle and ready. mem_ready is high.
    parameter WAIT    = 2'b01; // First delay cycle: mem_ready is low.
    parameter EXECUTE = 2'b10; // Second cycle: perform the operation, then set mem_ready high.
    reg [1:0] state, next_state;
    
    reg [19:0] read_data;
    
    reg latched_we;
    reg [9:0] latched_address;
    reg [19:0] latched_write_data;  // Only used if it's a write operation
    
    assign data = ((state == EXECUTE) && (we == 1'b1)) ?
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
                next_state = EXECUTE;
            end
            EXECUTE: begin
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
                    latched_we <= we;
                    latched_address <= address;
                    if (we)
                        latched_write_data <= data; // Latch the write data
                    mem_ready <= 1'b0; // Not ready as we are starting the operation
                end else begin
                    mem_ready <= 1'b1;
                end
            end
            WAIT: begin
                // During the delay cycle, maintain mem_ready low.
                mem_ready <= 1'b0;
            end
            EXECUTE: begin
                // In EXECUTE, perform the actual read or write.
                if (latched_we) begin
                    // Write operation:
                    // If the latched address's bit 0 is 0, write to the lower half; if 1, to the upper half.
                    if (latched_address[0] == 1'b0)
                        ram[{latched_address[9:1], 1'b0}] <= latched_write_data[9:0];
                    else
                        ram[{latched_address[9:1], 1'b1}] <= latched_write_data[19:10];
                end else begin
                    // Read operation:
                    // Combine both 10-bit halves from the selected block.
                    read_data <= { ram[{latched_address[9:1], 1'b1}],
                                   ram[{latched_address[9:1], 1'b0}] };
                end
                // After execution, signal that RAM is ready for a new operation.
                mem_ready <= 1'b1;
            end
            default: begin
                mem_ready <= 1'b1;
            end
        endcase
    end

endmodule
