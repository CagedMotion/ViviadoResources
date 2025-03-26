`timescale 1ns/1ps

module fd_EX_Mem_reg(
    input  wire clk, reset,
    input  wire stall,  // NEW: Stall signal from control logic.

    // Address fields for register file operands (already 3-bit)
    input  wire [2:0] gp_rdata1_address_in, gp_rdata2_address_in,
     
    // ALU operands from FD stage (10-bit)
    input  wire [9:0] aluA_in, aluB_in,
    
    // ALU control (3-bit)
    input  wire [2:0] alu_ctrl_in,
    
    // Register file write enable from FD stage
    input  wire       gp_reg_wb_in,
    
    // Additional FD stage signals needed in EM stage:
    input  wire       mem_we_in,   // Memory write enable (STORE)
    input  wire       mem_re_in,   // Memory read enable  (LOAD)
    input  wire [9:0] store_data_in,  // Data to be stored (for STORE instructions)
    
    // Outputs to the EM stage
    output wire [2:0] gp_rdata1_address_out, gp_rdata2_address_out,
    output wire [9:0] aluA_out, aluB_out, 
    output wire [2:0] alu_ctrl_out,
    output wire       gp_reg_wb_out,
    output wire       mem_we_out,
    output wire       mem_re_out,
    output wire [9:0] store_data_out
);
    // Use the inverse of stall as the enable signal.
    wire en;
    assign en = ~stall;
    
    // Latch ALU operands (10 bits)
    register_10bit REG1(.clk(clk), .reset(reset), .en(en), .din(aluA_in), .dout(aluA_out)),
                   REG2(.clk(clk), .reset(reset), .en(en), .din(aluB_in), .dout(aluB_out)),
                   // Latch store data (10 bits)
                   REG3(.clk(clk), .reset(reset), .en(en), .din(store_data_in), .dout(store_data_out));
                   
    // Latch gp_rdata1_address (3 bits)
    register_1bit REG4(.clk(clk), .reset(reset), .en(en), .din(gp_rdata1_address_in[2]), .dout(gp_rdata1_address_out[2])),
                  REG5(.clk(clk), .reset(reset), .en(en), .din(gp_rdata1_address_in[1]), .dout(gp_rdata1_address_out[1])),
                  REG6(.clk(clk), .reset(reset), .en(en), .din(gp_rdata1_address_in[0]), .dout(gp_rdata1_address_out[0]));
    
    // Latch gp_rdata2_address (3 bits) - used as the destination register info
    register_1bit REG7(.clk(clk), .reset(reset), .en(en), .din(gp_rdata2_address_in[2]), .dout(gp_rdata2_address_out[2])),
                  REG8(.clk(clk), .reset(reset), .en(en), .din(gp_rdata2_address_in[1]), .dout(gp_rdata2_address_out[1])),
                  REG9(.clk(clk), .reset(reset), .en(en), .din(gp_rdata2_address_in[0]), .dout(gp_rdata2_address_out[0]));
    
    // Latch ALU control (3 bits)
    register_1bit REG10(.clk(clk), .reset(reset), .en(en), .din(alu_ctrl_in[2]), .dout(alu_ctrl_out[2])),
                  REG11(.clk(clk), .reset(reset), .en(en), .din(alu_ctrl_in[1]), .dout(alu_ctrl_out[1])),
                  REG12(.clk(clk), .reset(reset), .en(en), .din(alu_ctrl_in[0]), .dout(alu_ctrl_out[0]));
    
    // Latch register write enable (1 bit)
    register_1bit REG13(.clk(clk), .reset(reset), .en(en), .din(gp_reg_wb_in), .dout(gp_reg_wb_out));
    
    // Latch memory write enable (1 bit)
    register_1bit REG_mem_we(.clk(clk), .reset(reset), .en(en), .din(mem_we_in), .dout(mem_we_out));
    
    // Latch memory read enable (1 bit)
    register_1bit REG_mem_re(.clk(clk), .reset(reset), .en(en), .din(mem_re_in), .dout(mem_re_out));
    
endmodule

