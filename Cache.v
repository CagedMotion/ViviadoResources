`timescale 1ns/1ps

module Cache(
    input clk,
    input rst,
    input CPU_RW,
    input [19:0] data_in,
    input [9:0] cpu_address,
    input [19:0] mem_data_out,
    output reg [19:0] data_out,
    output reg cache_ready,
    output reg [9:0] mem_addr,
    output reg [19:0] mem_data_in,
    output reg mem_rw
    );
    
    reg valid [0:15];
    reg dirty [0:15];
    reg [4:0] cache_tag [0:15];
    reg [19:0] cache_data [0:15];
    
    
    parameter IDLE            = 2'b00;
    parameter compare         = 2'b01;
    parameter writeback_alloc = 2'b10;

    reg [1:0] current_state;
    reg [1:0] next_state;
    
    reg mem_ready;
    reg [1:0] mem_phase;  // Tracks steps in MEM_ACCESS (0=writeback, 1=read, 2=update)
    
    
    reg start_writeback;
    reg start_alloc;
    reg update_cache;
    reg set_dirty;
    reg req_valid;
    reg hit;
    reg is_write;
    
    integer i;
    
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            current_state <= IDLE;
            for(i=0; i<16; i=i+1) begin
                valid[i] <= 1'b0;
                dirty[i] <= 1'b0;
                cache_tag[i] <= 5'b00000;
                cache_data[i] <= 20'd0;
            end
         end else begin
            current_state <= next_state;

            if (current_state == writeback_alloc && mem_ready)
                mem_phase <= mem_phase + 1;
            else if (current_state != writeback_alloc)
                mem_phase <= 2'b00;
         end
    end
    
    // Combinational logic for next state and outputs
    always @(*) begin
        // Default signal values
        next_state = current_state;
        cache_ready = 1'b0;
        start_writeback = 1'b0;
        start_alloc = 1'b0;
        update_cache = 1'b0;
        set_dirty = 1'b0;

        case (current_state)
            IDLE: begin
                cache_ready = 1'b1;
                if (req_valid)
                    next_state = compare;
            end

            compare: begin
                if (hit) begin
                    update_cache = 1'b1;
                    if (is_write)
                        set_dirty = 1'b1;
                        next_state = IDLE;
                end else begin
                    next_state = writeback_alloc;
                end
            end

            writeback_alloc: begin
                if (dirty[i] && mem_phase == 2'b00) begin
                    start_writeback = 1'b1;
                end else if ((!dirty[i] && mem_phase == 2'b00) || (dirty[i] && mem_phase == 2'b01)) begin
                    start_alloc = 1'b1;
                end else if ((dirty[i] && mem_phase == 2'b10) || (!dirty[i] && mem_phase == 2'b01)) begin
                    update_cache = 1'b1;
                    if (is_write)
                        set_dirty = 1'b1;
                    next_state = IDLE;
                end
            end

            default: next_state = IDLE;
        endcase
    end 
    
endmodule