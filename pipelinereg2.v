`timescale 1 ns/1 ps
module pipeline_register2(
    output wire [22:0] dout,
    input wire clk, reset, en,
    input wire [22:0] din );
    
    wire [22:0] w1;    
    register_10bit REG1(.clk(clk), .reset(reset), .en(en), .dout(w1[22:13]), .din(din[22:13])),
                   REG2(.clk(clk), .reset(reset), .en(en), .dout(w1[12:3]), .din(din[22:13]));
                  
    register_1bit REG3(.clk(clk), .reset(reset), .en(en), .dout(w1[2]), .din(din[2])),
                  REG4(.clk(clk), .reset(reset), .en(en), .dout(w1[1]), .din(din[1])),
                  REG5(.clk(clk), .reset(reset), .en(en), .dout(w1[0]), .din(din[0]));

    
    assign dout = w1;
endmodule