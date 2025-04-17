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
    
    integer i;
    initial begin
        state = IDLE;
        next_state = IDLE;
        mem_ready = 1'b1;
    
        ram[0] = 10'b0000010110; // value 22;
        ram[1] = 10'b0000001010;
        ram[2] = 10'b0000010101; // value 21;
        ram[10] = 10'b0000010101;  //21
        ram[11] = 10'b0000010100;  //20
        ram[12] = 10'b0000010011;  //19
        ram[13] = 10'b0000010010;  //18
        ram[14] = 10'b0000010001;  //17
        ram[15] = 10'b0000010000;  //16
        ram[16] = 10'b0000001111;  //15
        ram[17] = 10'b0000001110;  //14
        ram[18] = 10'b0000001101;  //13
        ram[19] = 10'b0000001100;  //12
        ram[20] = 10'b0000001011;  //11
        ram[21] = 10'b0000001010;  //10
        ram[22] = 10'b0000001001;  //9
        ram[23] = 10'b0000001000;  //8
        ram[24] = 10'b0000000111;  //7
        ram[25] = 10'b0000000110;  //6
        ram[26] = 10'b0000000101;  //5
        ram[27] = 10'b0000000100;  //4
        ram[28] = 10'b0000000011;  //3
        ram[29] = 10'b0000000010;  //2
        ram[30] = 10'b0000000001;  //1

     for(i=31; i<1024; i=i+1)
            ram[i] = 10'b0;
     for(i=3; i<10; i=i+1)
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