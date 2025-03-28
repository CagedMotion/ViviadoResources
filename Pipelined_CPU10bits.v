`timescale 1ns/1ps

module pipeline_CPU10bits(
    input  wire clk,
    input  wire rst,
    output wire cpu_halted  // Indicates that the CPU has halted.
);

    //----------------------------------------------------------
    // Stage 1: Fetch + Decode (FD)
    //----------------------------------------------------------
    // PC & Fetch Unit Signals
    wire [9:0] pc;       // PC from fetch_unit
    wire [9:0] instr;    // Instruction from ROM

    // Branch/Jump signals for fetch unit (set by FD decode)
    reg        branch_sig;
    reg        jump_sig;
    reg [9:0]  branch_target;
    reg [9:0]  jump_target;
    
    // Halt signal
    reg halted_reg;
    assign cpu_halted = halted_reg;

    // Instantiate updated Fetch Unit
    fetch_unit FU_inst (
        .clk(clk),
        .reset(rst),
        .halted(halted_reg),
        .branch(branch_sig),
        .jump(jump_sig),
        .branch_addr(branch_target),
        .jump_target(jump_target),
        .pc_out(pc)
    );

    // Instruction Memory (ROM)
    task2rom ROM_inst (
        .address(pc),
        .read_data(instr)
    );

    // Decode the instruction fields according to ISA design.
    wire [2:0] opcode   = instr[9:7];
    wire [1:0] rs_field = instr[6:5];
    wire [1:0] rt_field = instr[4:3];
    wire       bank_sel = instr[2];
    wire [1:0] fimm     = instr[1:0];
    wire [6:0] jmp_addr = instr[6:0];
    

    // Derive destination register as 3 bits: {bank_sel, rt_field}
    wire [2:0] dest_reg_fd = {bank_sel, rt_field};

    // Register File
    wire [9:0] fd_rdata1, fd_rdata2;
    wire [9:0] wb_wdata;
    wire [1:0] wb_waddr;
    wire       wb_we;
    register_file RF_inst (
        .clk(clk),
        .rst(rst),
        .we(wb_we),
        .bank_sel(wb_dest[2]),
        .waddr(wb_waddr),   // lower 2 bits
        .wdata(wb_wdata),
        .raddr1(rs_field),
        .raddr2(rt_field),
        .rdata1(fd_rdata1),
        .rdata2(fd_rdata2)
    );

    // Signals for FD stage (to be latched into FD->EM pipeline register)
    reg  [9:0] alu_inA, alu_inB;
    reg  [2:0] fd_alu_ctrl;
    reg        fd_reg_we;
    reg        fd_mem_we;
    reg        fd_mem_re;
    reg        fd_bank_sel;
    reg  [9:0] fd_store_data; // Data to be stored for STORE instructions

    // FD Stage Decoding (same as before)
    always @(*) begin
        // Defaults
        fd_alu_ctrl   = 3'b000;
        fd_reg_we     = 1'b0;
        fd_mem_we     = 1'b0;
        fd_mem_re     = 1'b0;
        fd_bank_sel   = bank_sel;
        fd_store_data = 10'd0;
        
        // Default branch/jump signals for fetch unit.
        branch_sig    = 1'b0;
        jump_sig      = 1'b0;
        branch_target = 10'd0;
        jump_target   = 10'd0;
        
        // Default ALU operands from register file.
        alu_inA = fd_rdata1;
        alu_inB = fd_rdata2;
        
        case (opcode)
            // 000: R-type arithmetic: ADD, SUB, SLT, NAND.
            3'b000: begin
                case (fimm)
                    2'b00: fd_alu_ctrl = 3'b000; // ADD
                    2'b01: fd_alu_ctrl = 3'b001; // SUB
                    2'b10: fd_alu_ctrl = 3'b010; // SLT
                    2'b11: fd_alu_ctrl = 3'b011; // NAND
                endcase
                fd_reg_we = 1'b1;
            end

            // 001: SHIFT or HALT.
            3'b001: begin
                case (fimm)
                    2'b00: fd_alu_ctrl = 3'b100; // SLR
                    2'b01: fd_alu_ctrl = 3'b101; // SLL
                    2'b10: fd_alu_ctrl = 3'b110; // HALT
                    default: fd_alu_ctrl = 3'b000;
                endcase
                if (fimm != 2'b10)
                    fd_reg_we = 1'b1;
            end

            // 010: BNE: Branch if not equal.
            3'b010: begin
                if (fd_rdata1 != fd_rdata2) begin
                    branch_sig    = 1'b1;
                    branch_target = pc + zero_extend_imm(fimm);
                end
            end

            // 011: ADDI: Immediate addition.
            3'b011: begin
                fd_alu_ctrl = 3'b000;  // ADD
                alu_inB = sign_extend_imm(fimm);
                fd_reg_we = 1'b1;
            end

            // 100: JUMP.
            3'b100: begin
                jump_sig    = 1'b1;
                jump_target = sign_extend_jmp(jmp_addr);
            end

            // 101: BEQ: Branch if equal.
            3'b101: begin
                if (fd_rdata1 == fd_rdata2) begin
                    branch_sig    = 1'b1;
                    branch_target = pc + zero_extend_imm(fimm);
                end
            end

            // 110: LOAD.
            3'b110: begin
                fd_alu_ctrl = 3'b000;  // Effective address = base + offset
                alu_inB = zero_extend_imm(fimm);
                fd_reg_we   = 1'b1;    // Write loaded data in WB stage
                fd_mem_re   = 1'b1;    // Memory read
            end

            // 111: STORE.
            3'b111: begin
                fd_alu_ctrl   = 3'b000;  // Effective address
                alu_inB = zero_extend_imm(fimm);
                fd_mem_we     = 1'b1;    // Memory write
                fd_store_data = fd_rdata2;  // Data to store is from rt
            end

            default: begin
                fd_reg_we = 1'b0;
            end
        endcase
    end

    //----------------------------------------------------------
    // FD->EM Pipeline Register
    //----------------------------------------------------------
    // Compute the source addresses from FD stage for hazard detection:
    wire [2:0] fd_srcA_addr = {bank_sel, rs_field};
    wire [2:0] fd_srcB_addr = dest_reg_fd; // Note: for simplicity, we use the same as dest_reg_fd

    wire [9:0] em_operandA, em_operandB;
    wire [2:0] em_alu_ctrl, gp_rdata1_address_out, gp_rdata2_address_out;
    wire       em_reg_we;
    wire       em_mem_we;
    wire       em_mem_re;
    wire [9:0] em_store_data;
    wire [2:0] wb_dest;
    wire [2:0] alt_rdata2;
    
    assign alt_rdata2 = (opcode == 3'b100) ? wb_dest : fd_srcB_addr;
    
    fd_EX_Mem_reg FD_EM_reg (
        .clk(clk),
        .reset(rst),
        .gp_rdata1_address_in(fd_srcA_addr),
        .gp_rdata1_address_out(gp_rdata1_address_out),
        .gp_rdata2_address_in(alt_rdata2),
        .gp_rdata2_address_out(gp_rdata2_address_out),
        .aluA_in(alu_inA),
        .aluA_out(em_operandA),
        .aluB_in(alu_inB),
        .aluB_out(em_operandB),
        .alu_ctrl_in(fd_alu_ctrl),
        .alu_ctrl_out(em_alu_ctrl),
        .gp_reg_wb_in(fd_reg_we),
        .gp_reg_wb_out(em_reg_we),
        .mem_we_in(fd_mem_we),
        .mem_we_out(em_mem_we),
        .mem_re_in(fd_mem_re),
        .mem_re_out(em_mem_re),
        .store_data_in(fd_store_data),
        .store_data_out(em_store_data)
    );
    
    //----------------------------------------------------------
    // Forwarding Unit
    //----------------------------------------------------------
    // Here we use em_mem_re as the indicator that the previous instruction is a load.
    wire [9:0] wb_alu_result;
    wire [9:0] wb_mem_rdata;
    wire       wb_reg_we;
    wire       wb_mem_re;
    wire       forwardA, forwardB;

    forwarding_unit fw_unit (
        .exmem_wb_wr(wb_reg_we),
        .ex_dest_reg(wb_dest),
        .id_dest_reg(gp_rdata2_address_out),
        .id_src_reg(gp_rdata1_address_out),
        .forwardA(forwardA),
        .forwardB(forwardB)
    );
    
    reg [9:0] final_wdata;
    // Mux the EM stage ALU inputs to handle forwarding.
    wire [9:0] alu_operandA = (forwardA) ? final_wdata : em_operandA;
    wire [9:0] alu_operandB = (forwardB) ? final_wdata : em_operandB;

    //----------------------------------------------------------
    // Stage 2: Execute + Memory (EM)
    //----------------------------------------------------------
    wire [9:0] alu_result;
    wire       alu_halt;

    ALU ALU_inst (
        .A(alu_operandA),
        .B(alu_operandB),
        .alu_ctrl(em_alu_ctrl),
        .result(alu_result),
        .halt(alu_halt)
    );

    // Data Memory
    wire [9:0] mem_rdata;
    wire [9:0] mem_addr  = alu_result;
    wire [9:0] mem_wdata = (em_mem_we) ? em_store_data : 10'd0;

    ramtask2 RAM_inst (
        .clk(clk),
        .we(em_mem_we),
        .address(mem_addr),
        .wdata(mem_wdata),
        .rdata(mem_rdata)
    );

    //----------------------------------------------------------
    // EM->WB Pipeline Register
    //----------------------------------------------------------
    Exe_Mem_WB_reg EM_WB_reg (
        .clk(clk), 
        .reset(rst), 
        .alu_result_in(alu_result), 
        .alu_result_out(wb_alu_result),
        .ram_rdata_in(mem_rdata), 
        .ram_rdata_out(wb_mem_rdata),
        .gp_reg_wb_in(em_reg_we),
        .gp_reg_wb_out(wb_reg_we),
        .mem_re_in(em_mem_re),
        .mem_re_out(wb_mem_re),
        .gp_rdata2_address_in(gp_rdata2_address_out), 
        .gp_rdata2_address_out(wb_dest)
    );

    //----------------------------------------------------------
    // Stage 3: Writeback (WB)
    //----------------------------------------------------------
 
    always @(wb_mem_re, wb_mem_rdata, wb_alu_result) begin
        if (wb_mem_re)
            final_wdata <= wb_mem_rdata;   // load
        else
            final_wdata <= wb_alu_result;    // ALU result
    end

    // Drive register file write signals
    assign wb_wdata = final_wdata;
    assign wb_waddr = wb_dest[1:0];  // lower 2 bits
    assign wb_we = wb_reg_we;

    //----------------------------------------------------------
    // Halt Logic
    //----------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst)
            halted_reg <= 1'b0;
        else if (alu_halt)
            halted_reg <= 1'b1;
    end

    //----------------------------------------------------------
    // Helper Functions for Immediate Extension
    //----------------------------------------------------------
    function [9:0] zero_extend_imm;
        input [1:0] imm;
        begin
            zero_extend_imm = {8'b0, imm};
        end
    endfunction

    function [9:0] sign_extend_imm;
        input [1:0] imm;
        begin
            sign_extend_imm = {{8{imm[1]}}, imm};
        end
    endfunction

    function [9:0] sign_extend_jmp;
        input [6:0] jmp;
        begin
            sign_extend_jmp = {{3{jmp[6]}}, jmp};
        end
    endfunction

endmodule

module tb_pipeline_cpu10bits;
    reg clk;
    reg rst;
    //wire halted;
    
    // Instantiate the CPU10bits top module.
    pipeline_CPU10bits dut (
        .clk(clk),
        .rst(rst),
        .cpu_halted()  // Connect to a monitor if desired.
    );
    
    parameter PERIOD = 10;
    initial clk = 1'b1;
    always #(PERIOD/2) clk = ~clk;
    
    initial begin
        //halted = 1;
        rst = 1;
        #PERIOD;
        rst = 0;
        //halted = 0;
        
        // Optionally, drive any test stimulus here.
        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
//        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
//        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
//        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
//        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
//        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
//        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
//        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
//        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
//        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
//        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
//        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
//        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
//        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
//        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
//        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
        
        $finish;
    end
endmodule