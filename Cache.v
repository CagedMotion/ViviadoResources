`timescale 1ns/1ps

module Cache(
    input clk,
    input rst,
    input CPU_RW,
    input [19:0] data_in,
    input [9:0] cpu_address,
    output reg [19:0] data_out
    
    
    );
    reg [2:0] present_state, next_state;
    reg valid, tag, dirty;
    
    
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            for(i=0; i<16; i=i+1) begin
            end
        end
    end
    
    always @(posedge clk) begin
        present_state = next_state;
    end
    
    always @(present_state,next_state) begin
        case(present_state) 
            2'b00: begin
                if (cpu_address) begin
                    next_state <= 2'b01;
                end
                else begin
                    next_state <= 2'b00;
                end
            end
            
            2'b01: begin
                if ((tag == cpu_address[9:5]) & (valid == 1'b1)) begin
                    next_state <= 2'b00;
                end
                else begin
                    valid <= 1'b0;
                    next_state <= 2'b10;
                end
            end
            
            2'b10: begin
                
            end
            
            2'b11: begin
                next_state <= 2'b00;
            end
        endcase
    end
    
endmodule

module cache_fsm (
    input wire clk,
    input wire reset,

    // Request interface
    input wire req_valid,
    input wire is_write,
    input wire hit,
    input wire dirty,
    input wire mem_ready,

    // Control outputs
    output reg cache_ready,
    output reg start_writeback,
    output reg start_alloc,
    output reg update_cache,
    output reg set_dirty
);

    // FSM states
    parameter IDLE       = 2'b00;
    parameter TAG_CHECK  = 2'b01;
    parameter MEM_ACCESS = 2'b10;

    reg [1:0] current_state;
    reg [1:0] next_state;

    reg [1:0] mem_phase;  // Tracks steps in MEM_ACCESS (0=writeback, 1=read, 2=update)

    // Sequential logic for state and memory phase
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_state <= IDLE;
            mem_phase <= 2'b00;
        end else begin
            current_state <= next_state;

            if (current_state == MEM_ACCESS && mem_ready)
                mem_phase <= mem_phase + 1;
            else if (current_state != MEM_ACCESS)
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
                    next_state = TAG_CHECK;
            end

            TAG_CHECK: begin
                if (hit) begin
                    update_cache = 1'b1;
                    if (is_write)
                        set_dirty = 1'b1;
                        next_state = IDLE;
                end else begin
                    next_state = MEM_ACCESS;
                end
            end

            MEM_ACCESS: begin
                if (dirty && mem_phase == 2'b00) begin
                    start_writeback = 1'b1;
                end else if ((!dirty && mem_phase == 2'b00) || (dirty && mem_phase == 2'b01)) begin
                    start_alloc = 1'b1;
                end else if ((dirty && mem_phase == 2'b10) || (!dirty && mem_phase == 2'b01)) begin
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


endmodule
