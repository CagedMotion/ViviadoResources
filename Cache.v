`timescale 1ns/1ps

module Cache(
    input clk,
    input rst,
    input CPU_RW,
    input [19:0] data_in,
    output reg [19:0] data_out
    
    
    );
    
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            for(i=0; i<16; i=i+1) begin
    
    
endmodule