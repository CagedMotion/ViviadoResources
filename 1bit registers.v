`timescale 1 ns/1 ps
module register_1bit (
    output reg dout,
    input wire clk, reset, en,
    input wire din );
    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            dout <= 1'b0;
        end
        else if (en == 1'b1) begin
            dout <= din;
        end
    end
endmodule

module tb_1reg();
    wire dout;
    reg clk, reset, en;
    reg din;

    register_1bit SUT(dout, clk, reset, en, din);

    parameter PERIOD = 10;
    initial clk = 1'b0;
    always #(PERIOD/2) clk = ~clk;
    
    initial begin
        reset = 1'b1; #PERIOD;
        reset = 1'b0; din = 1'b1; en = 1'b1; #PERIOD; #PERIOD;
    end
endmodule
