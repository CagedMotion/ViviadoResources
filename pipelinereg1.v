`timescale 1 ns/1 ps
module pipeline_register1(
    output wire [43:0] dout,
    input wire clk, reset, en,
    input wire [43:0] din );
    
    wire [43:0] w1;    
    register_10bit REG1(.clk(clk), .reset(reset), .en(en), .dout(w1[43:34]), .din(din[43:34])),
                   REG2(.clk(clk), .reset(reset), .en(en), .dout(w1[33:24]), .din(din[33:24])),
                   REG3(.clk(clk), .reset(reset), .en(en), .dout(w1[23:14]), .din(din[23:14])),
                   REG4(.clk(clk), .reset(reset), .en(en), .dout(w1[13:4]), .din(din[13:4]));
                  
    register_1bit REG5(.clk(clk), .reset(reset), .en(en), .dout(w1[3]), .din(din[3])),
                  REG6(.clk(clk), .reset(reset), .en(en), .dout(w1[2]), .din(din[2])),
                  REG7(.clk(clk), .reset(reset), .en(en), .dout(w1[1]), .din(din[1])),
                  REG8(.clk(clk), .reset(reset), .en(en), .dout(w1[0]), .din(din[0]));

    
    assign dout = w1;
endmodule
