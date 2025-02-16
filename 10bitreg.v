`timescale 1 ns/1 ps
module register_10bit(
    output wire [9:0] dout,
    input wire clk, reset, en,
    input wire [9:0] din );
    
    wire [9:0] w1;    
    register_1bit REG1(.clk(clk), .reset(reset), .en(en), .dout(w1[0]), .din(din[0])),
                  REG2(.clk(clk), .reset(reset), .en(en), .dout(w1[1]), .din(din[1])),
                  REG3(.clk(clk), .reset(reset), .en(en), .dout(w1[2]), .din(din[2])),
                  REG4(.clk(clk), .reset(reset), .en(en), .dout(w1[3]), .din(din[3])),
                  REG5(.clk(clk), .reset(reset), .en(en), .dout(w1[4]), .din(din[4])),
                  REG6(.clk(clk), .reset(reset), .en(en), .dout(w1[5]), .din(din[5])),
                  REG7(.clk(clk), .reset(reset), .en(en), .dout(w1[6]), .din(din[6])),
                  REG8(.clk(clk), .reset(reset), .en(en), .dout(w1[7]), .din(din[7])),
                  REG9(.clk(clk), .reset(reset), .en(en), .dout(w1[8]), .din(din[8])),
                  REG10(.clk(clk), .reset(reset), .en(en), .dout(w1[9]), .din(din[9]));
    
    assign dout = w1;
endmodule