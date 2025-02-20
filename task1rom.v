`timescale 1ns / 1ps

module task1rom(
    output reg [9:0] read_data,
    input wire clk,
    input wire [9:0] address
    );
    reg [9:0] memory[1023:0];
    
    initial begin
        memory[0] <= 10'b0000000001;
    end
    
    integer i;
    always @(address) begin
        for(i=0;i<1024;i=i+1) begin
            if (i == address) begin
                read_data <= memory[i];
            end
            else begin
                read_data <= 10'b0;
            end
        end
    end
endmodule