`timescale 1 ns/1 ps
module register_s1(
    output wire [9:0] dout,
    input wire clk, reset, en,
    input wire [9:0] din );
    
    wire [9:0] internal_dout;
    
    register_10bit reg_inst(.clk(clk), .reset(reset), .en(en), .dout(internal_dout), .din(din));
    
    assign dout = internal_dout;
endmodule