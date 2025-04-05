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