module Exe_Mem_WB_reg(
    input wire clk, reset, en,
    input wire gp_reg_wb_in,
    input wire [9:0] ram_rdata_in,
    output wire [9:0] ram_rdata_out, 
    output wire gp_reg_wb_out
    );

    wire [23:0] w1;    
    register_10bit REG1(.clk(clk), .reset(reset), .en(en), .dout(ram_rdata_out), .din(ram_rdata_in));

    register_1bit REG3(.clk(clk), .reset(reset), .en(en), .dout(gp_reg_wb_out), .din(gp_reg_wb_in));


endmodule