module Exe_Mem_WB_reg(
    output wire [23:0] dout,
    input wire clk, reset, en,
    input wire [23:0] din 
    
    );

    wire [23:0] w1;    
    register_10bit REG1(.clk(clk), .reset(reset), .en(en), .dout(w1[23:14]), .din(din[23:14])),
                   REG2(.clk(clk), .reset(reset), .en(en), .dout(w1[13:4]), .din(din[13:4]));

    register_1bit REG3(.clk(clk), .reset(reset), .en(en), .dout(w1[3]), .din(din[3])),
                  REG4(.clk(clk), .reset(reset), .en(en), .dout(w1[2]), .din(din[2])),
                  REG5(.clk(clk), .reset(reset), .en(en), .dout(w1[1]), .din(din[1])),
                  REG6(.clk(clk), .reset(reset), .en(en), .dout(w1[0]), .din(din[0]));


    assign dout = w1;


endmodule