`timescale 1ns / 1ps

module ramtask1_for_cache(
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
        
    parameter IDLE    = 2'b00; // Memory is idle and ready. mem_ready is high.
    parameter WAIT    = 2'b01; // First delay cycle: mem_ready is low.
    parameter EXECUTE = 2'b10; // Second cycle: perform the operation, then set mem_ready high.
    
    initial begin
        state = IDLE;
        next_state = IDLE;
        mem_ready = 1'b1;
    
        ram[0] = 10'b0000000101; //0000010100;
        ram[1] = 10'b0000001010;
        ram[2] = 10'b0000000100;
        ram[10] = 10'b0000010100;  //20
        ram[11] = 10'b0000010011;  //19
        ram[12] = 10'b0000010010;  //18
        ram[13] = 10'b0001010010;  //82
        ram[14] = 10'b0000000110;  //6
//        ram[0] = 10'b0000000100;
//        ram[1] = 10'b1001000000; 
//        //ram[2] = 10'b1000010000;//ARRAY 1 ADDRESS
//        ram[2] = 10'b0000000000;
//        ram[3] = 10'b0001010111;
//        ram[4] = 10'b0001100001;
//        ram[5] = 10'b0001100110;
//        ram[6] = 10'b0001100110;
//        ram[7] = 10'b0001101100;
//        ram[8] = 10'b0001100101;
//        ram[9] = 10'b0001110011;
//        ram[10] = 10'b0001000001;
//        ram[11] = 10'b0001101110;
//        ram[12] = 10'b0001100100;
//        ram[13] = 10'b0001010000;
//        ram[14] = 10'b0001100001;
//        ram[15] = 10'b0001101110;
//        ram[16] = 10'b0001100011;
//        ram[17] = 10'b0001100001;
//        ram[18] = 10'b0001101011;
//        ram[19] = 10'b0001100101;
//        ram[20] = 10'b0001110011;
//        ram[21] = 10'b0000000000;
//        ram[22] = 10'b1001000000;
     end

    
    reg [19:0] read_data;
    
    reg latched_we;
    reg [9:0] latched_address;
    reg [19:0] latched_write_data;  // Only used if it's a write operation
    
    wire [8:0] temp;
    assign temp = address [9:1];    
    
    assign data = ((state == IDLE) && (we == 1'b1)) ?
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
                    if (we) begin
                        latched_write_data <= data; // Latch the write data
                    end
                    mem_ready <= 1'b0;
                end else begin
                    mem_ready <= 1'b1;
                end
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
            end
            WAIT: begin
                // During the delay cycle, maintain mem_ready low.
                mem_ready <= 1'b0;
            end
            EXECUTE: begin
                // In EXECUTE, perform the actual read or write.
                // After execution, signal that RAM is ready for a new operation.
                mem_ready <= 1'b1;
            end
            default: begin
                mem_ready <= 1'b1;
            end
        endcase
    end

endmodule

module tb_ram_for_cache();
    wire [9:0] rdata;
    reg clk, we;
    reg [9:0] address;
    reg [9:0] wdata;
    
    ram dut(.rdata(rdata), .clk(clk), .we(we), .address(address), .wdata(wdata));
    
    parameter PERIOD = 10;
    initial clk = 1'b0;
    always #(PERIOD/2) clk = ~clk;
    
    initial begin
        // Basic write operations
        wdata = 10'h01;
        address = 10'd0;
        we = 1'b1;
        #PERIOD;

        wdata = 10'h02;
        address = 10'd75;     
        #PERIOD;

        wdata = 10'h03;
        address = 10'd60;     
        #PERIOD;

        // Read operations for addresses 0, 1, 2
        address = 10'd0;
        we = 1'b0;
        #PERIOD;

        address = 10'd1;
        #PERIOD;

        address = 10'd2;
        #PERIOD;

        // Additional write/read cycle
        wdata = 10'h04;
        address = 10'd25;
        we = 1'b1;
        #PERIOD;

        address = 10'd50;
        we = 1'b0;
        #PERIOD;

        // New test: write and read at the end address (1023)
        wdata = 10'h3FF; // Maximum 10-bit value
        address = 10'd1023;
        we = 1'b1;
        #PERIOD;

        we = 1'b0;
        address = 10'd1023;
        #PERIOD;
    end
endmodule