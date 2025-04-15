`timescale 1ns/1ps

module Exe_Mem_WB_reg(
    input  wire clk, reset,
    input  wire en,
    // Latch the ALU result (10 bits)
    input  wire [9:0] alu_result_in,
    output wire [9:0] alu_result_out,
    // Latch the memory read data (10 bits)
    input  wire [9:0] ram_rdata_in,
    output wire [9:0] ram_rdata_out,
    // latch the memory store data(10 bits)
    input  wire [9:0] ram_stdata_in,
    output wire [9:0] ram_stdata_out,
    // Latch the register-file write enable (1 bit)
    input  wire       gp_reg_wb_in,
    output wire       gp_reg_wb_out,
    // Latch the memory read enable signal (1 bit) to indicate a load
    input  wire       mem_re_in,
    output wire       mem_re_out,
    //latching the gp_rdata2_address 3 bits
    input wire  [2:0] gp_rdata2_address_in,
    output wire [2:0] gp_rdata2_address_out
    // Stall Signal
);
    
    // Latch the ALU result.
    register_10bit REG1(.clk(~clk), .reset(reset), .en(en), .din(alu_result_in), .dout(alu_result_out));
    
    // Latch the memory read data.
    register_10bit REG2(.clk(~clk), .reset(reset), .en(en), .din(ram_rdata_in), .dout(ram_rdata_out)
    );
    
    // Latch the memory store data.
    register_10bit REG10(.clk(~clk), .reset(reset), .en(en), .din(ram_stdata_in), .dout(ram_stdata_out)
    );
    // Latch the register-file write enable.
    register_1bit REG3(.clk(~clk), .reset(reset), .en(en), .din(gp_reg_wb_in), .dout(gp_reg_wb_out));
    
    // Latch the memory read enable.
    register_1bit REG4(.clk(~clk), .reset(reset),  .en(en), .din(mem_re_in), .dout(mem_re_out));
    
    // rt_field address
    register_1bit REG7(.clk(~clk), .reset(reset), .en(en), .din(gp_rdata2_address_in[2]), .dout(gp_rdata2_address_out[2])),
                  REG8(.clk(~clk), .reset(reset), .en(en), .din(gp_rdata2_address_in[1]), .dout(gp_rdata2_address_out[1])),
                  REG9(.clk(~clk), .reset(reset), .en(en), .din(gp_rdata2_address_in[0]), .dout(gp_rdata2_address_out[0]));


endmodule
