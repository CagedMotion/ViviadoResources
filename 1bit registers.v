`timescale 1 ns/1 ps
module register_1bit (
    output reg dout,
    input wire clk, reset, en,
    input wire din );
    
    always @(posedge clk) begin
        if (reset == 1'b1) begin
            dout = 1'b0;
        end
        else if (en == 1'b1) begin
            dout = din;
        end
    end
endmodule