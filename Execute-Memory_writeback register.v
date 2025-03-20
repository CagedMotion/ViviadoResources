module Exe_Mem_WB_reg(
    input wire clk, reset,
    input wire gp_reg_wb_in,
    input wire [9:0] ram_rdata_in, alu_result_in,
    input wire [2:0] gp_rdata2_address_in,
    output wire [9:0] ram_rdata_out, alu_result_out,
    output wire [2:0] gp_rdata2_address_out,
    output wire gp_reg_wb_out
    );

    wire en;
    assign en = 1'b1;
    
    register_10bit REG1(.clk(clk), .reset(reset), .en(en), .dout(ram_rdata_out), .din(ram_rdata_in)),
                   REG2(.clk(clk), .reset(reset), .en(en), .dout(alu_result_out), .din(alu_result_in));

    register_1bit REG3(.clk(clk), .reset(reset), .en(en), .dout(gp_reg_wb_out), .din(gp_reg_wb_in)),
                  REG4(.clk(clk), .reset(reset), .en(en), .dout(gp_rdata2_address_out[2]), .din(gp_rdata2_address_in[2])),
                  REG5(.clk(clk), .reset(reset), .en(en), .dout(gp_rdata2_address_out[1]), .din(gp_rdata2_address_in[1])),
                  REG6(.clk(clk), .reset(reset), .en(en), .dout(gp_rdata2_address_out[0]), .din(gp_rdata2_address_in[0]));
                  


endmodule