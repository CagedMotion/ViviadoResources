module fd_EX_Mem_reg(
    input wire clk, reset, en,
    input wire [2:0] gp_rdata1_address_in, gp_rdata2_address_in, 
    input wire [9:0] aluA_in, aluB_in,
    input wire [2:0] alu_ctrl_in,
    input wire gp_reg_wb_in,
    output wire [2:0] gp_rdata1_address_out, gp_rdata2_address_out,
    output wire [9:0] aluA_out, aluB_out, 
    output wire [2:0] alu_ctrl_out,
    output wire gp_reg_wb_out
    );
    
    register_10bit REG1(.clk(clk), .reset(reset), .en(en), .dout(aluA_out), .din(aluA_in)),
                   REG2(.clk(clk), .reset(reset), .en(en), .dout(aluB_out), .din(aluB_in));

    register_1bit REG3(.clk(clk), .reset(reset), .en(en), .dout(gp_rdata1_address_out[2]), .din(gp_rdata1_address_in[2])),
                  REG4(.clk(clk), .reset(reset), .en(en), .dout(gp_rdata1_address_out[1]), .din(gp_rdata1_address_in[1])),
                  REG5(.clk(clk), .reset(reset), .en(en), .dout(gp_rdata1_address_out[0]), .din(gp_rdata1_address_in[0])),
                  REG6(.clk(clk), .reset(reset), .en(en), .dout(gp_rdata2_address_out[2]), .din(gp_rdata2_address_in[2]));
                  REG7(.clk(clk), .reset(reset), .en(en), .dout(gp_rdata1_address_out[1]), .din(gp_rdata2_address_in[1]));
                  REG8(.clk(clk), .reset(reset), .en(en), .dout(gp_rdata1_address_out[0]), .din(gp_rdata2_address_in[0]));
                  REG9(.clk(clk), .reset(reset), .en(en), .dout(alu_ctrl_out[2]), .din(alu_ctrl_in[2]));
                  REG10(.clk(clk), .reset(reset), .en(en), .dout(alu_ctrl_out[1]), .din(alu_ctrl_in[1]));
                  REG12(.clk(clk), .reset(reset), .en(en), .dout(alu_ctrl_out[0]), .din(alu_ctrl_in[0]));
                  REG13(.clk(clk), .reset(reset), .en(en), .dout(gp_reg_wb_out), .din(gp_reg_wb_in));

endmodule