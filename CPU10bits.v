`timescale 1ns/1ps

module CPU10bits(
    input  wire clk,
    input  wire rst,
    output wire cpu_halted  // Indicates that the CPU has halted.
);

    //-------------------------------------------------------------------------
    // 1) PC & Fetch Unit Signals
    //-------------------------------------------------------------------------
    wire [9:0] pc;  // PC output from fetch unit
    reg        branch_sig;
    reg        jump_sig;
    reg [9:0]  branch_target;
    reg [9:0]  jump_target;
    
    // Halt signal for the fetch unit
    reg halted_reg;
    reg next_halt;
    assign cpu_halted = halted_reg;

    //-------------------------------------------------------------------------
    // 2) Instantiate the Fetch Unit
    //    The fetch unit uses branch, jump, branch_target, and jump_target
    //    to compute the next PC. When none of these are asserted, it simply
    //    increments PC by one.
    //-------------------------------------------------------------------------
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

    //-------------------------------------------------------------------------
    // 3) Instruction Memory (ROM)
    //    Use the PC from the fetch unit to index into ROM.
    //-------------------------------------------------------------------------
    wire [9:0] instr;
//    task1rom ROM_inst (
//        .address(pc),
//        .clk(clk),           // clk not strictly required if ROM is asynchronous
//        .read_data(instr)
//    );

    task2rom ROM_inst (
        .address(pc),
        .clk(clk),           // clk not strictly required if ROM is asynchronous
        .read_data(instr)
    );

//    task3rom ROM_inst (
//        .address(pc),
//        .clk(clk),           // clk not strictly required if ROM is asynchronous
//        .read_data(instr)
//    );

    //-------------------------------------------------------------------------
    // 4) Data Memory (Asynchronous Read, Synchronous Write)
    //-------------------------------------------------------------------------
    reg  [9:0] ram_addr;
    reg  [9:0] ram_wdata;
    reg        ram_we;
    wire [9:0] ram_rdata;
//    ramtask1 RAM_inst (
//        .clk(clk),
//        .we(ram_we),
//        .address(ram_addr),
//        .wdata(ram_wdata),
//        .rdata(ram_rdata)
//    );
    ramtask2 RAM_inst (
        .clk(clk),
        .we(ram_we),
        .address(ram_addr),
        .wdata(ram_wdata),
        .rdata(ram_rdata)
    );
//    ramtask3 RAM_inst (
//        .clk(clk),
//        .we(ram_we),
//        .address(ram_addr),
//        .wdata(ram_wdata),
//        .rdata(ram_rdata)
//    );
    //-------------------------------------------------------------------------
    // 6) Instruction Decode
    //    According to your ISA:
    //      [9:7] opcode, [6:5] rs, [4:3] rt, [2] bank_sel, [1:0] func/imm.
    //    (Swap fields if needed to match your actual machine code.)
    //-------------------------------------------------------------------------
    wire [2:0] opcode   = instr[9:7];
    wire [1:0] rs_field = instr[6:5];
    wire [1:0] rt_field = instr[4:3];
    wire       bank_sel = instr[2];
    wire [1:0] fimm     = instr[1:0];
    wire [6:0] jmp_addr = instr[6:0];

    //-------------------------------------------------------------------------
    // 5) Register File (Async Read, Sync Write)
    //-------------------------------------------------------------------------
    reg  [1:0] gp_reg_waddr;
    reg  [9:0] gp_reg_wdata;
    reg        gp_reg_we;
    reg        gp_reg_bank_sel;
    wire [9:0] gp_rdata1, gp_rdata2;
    register_file RF_inst (
        .clk(clk),
        .rst(rst),
        .we(gp_reg_we),
        .bank_sel(gp_reg_bank_sel),
        .waddr(gp_reg_waddr),
        .wdata(gp_reg_wdata),
        .raddr1(rs_field),
        .raddr2(rt_field),
        .rdata1(gp_rdata1),
        .rdata2(gp_rdata2)
    );

    //-------------------------------------------------------------------------
    // 7) ALU Instantiation
    //-------------------------------------------------------------------------
    reg  [9:0] alu_inA, alu_inB;
    wire [9:0] alu_result;
    wire       alu_halt;
    reg  [2:0] alu_ctrl;
    ALU ALU_inst (
        .A(alu_inA),
        .B(alu_inB),
        .alu_ctrl(alu_ctrl),
        .result(alu_result),
        .halt(alu_halt)
    );

    //-------------------------------------------------------------------------
    // 8) Sequential State: Halt Signal (PC update is handled in fetch_unit)
    //-------------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            halted_reg <= 1'b0;
        end else begin
            if (alu_halt)
                halted_reg <= 1'b1;
        end
    end

    //-------------------------------------------------------------------------
    // 9) COMBINATIONAL Logic: Decoding, ALU, and Control Signal Generation
    //     * NOTE: We now remove any direct computation of next_pc.
    //     * Instead, we drive branch_sig, jump_sig, branch_target, and jump_target.
    //-------------------------------------------------------------------------
    always @(*) begin
        // Default control signals
        branch_sig       = 1'b0;
        jump_sig         = 1'b0;
        branch_target    = 10'd0;
        jump_target      = 10'd0;
        
        // Default register file signals
        gp_reg_we      = 1'b0;
        gp_reg_wdata   = 10'd0;
        gp_reg_waddr   = rt_field;      // By convention, destination is rt
        gp_reg_bank_sel = bank_sel;
        
        // Default RAM signals
        ram_we         = 1'b0;
        ram_addr       = 10'd0;
        ram_wdata      = 10'd0;
        
        // Default ALU signals
        alu_inA        = gp_rdata1;     // Read base from rs
        alu_inB        = gp_rdata2;     // Read second operand from rt
        alu_ctrl       = 3'b000;

        case (opcode)
            // 000: R-type arithmetic: ADD, SUB, SLT, NAND.
            3'b000: begin
                case (fimm)
                    2'b00: alu_ctrl = 3'b000; // ADD
                    2'b01: alu_ctrl = 3'b001; // SUB
                    2'b10: alu_ctrl = 3'b010; // SLT
                    2'b11: alu_ctrl = 3'b011; // NAND
                endcase
                gp_reg_we    = 1'b1;
                gp_reg_wdata = alu_result;
            end

            // 001: SHIFT or HALT.
            3'b001: begin
                case (fimm)
                    2'b00: alu_ctrl = 3'b100; // SLR
                    2'b01: alu_ctrl = 3'b101; // SLL
                    2'b10: alu_ctrl = 3'b110; // HALT
                    default: alu_ctrl = 3'b000;
                endcase
                if (fimm == 2'b10)
                    ; // Let alu_halt drive halt
                else begin
                    gp_reg_we    = 1'b1;
                    gp_reg_wdata = alu_result;
                end
            end

            // 010: BNE: Branch if not equal.
            3'b010: begin
                if (alu_inA != alu_inB) begin
                    branch_sig    = 1'b1;
                    branch_target = pc + zero_extend_imm(fimm);  // Branch relative to current PC
                end
            end

            // 011: ADDI: Immediate addition.
            3'b011: begin
                alu_ctrl  = 3'b000;  // ADD
                alu_inB   = zero_extend_imm(fimm);
                gp_reg_we    = 1'b1;
                gp_reg_wdata = alu_result;
            end

            // 100: JUMP.
            3'b100: begin
                jump_sig    = 1'b1;
                jump_target = sign_extend_jmp(jmp_addr);
            end

            // 101: BEQ: Branch if equal.
            3'b101: begin
                if (alu_inA == alu_inB) begin
                    branch_sig    = 1'b1;
                    branch_target = pc + zero_extend_imm(fimm);
                end
            end

            // 110: LOAD.
            3'b110: begin
                alu_ctrl = 3'b000;            // Compute effective address: base + offset.
                alu_inB  = sign_extend_imm(fimm);
                // Use ALU result as RAM address.
                ram_addr = alu_result;
                // Asynchronous read from RAM is valid in the same cycle.
                gp_reg_we    = 1'b1;
                gp_reg_wdata = ram_rdata;
            end

            // 111: STORE.
            3'b111: begin
                alu_ctrl = 3'b000;            // Compute effective address.
                alu_inB  = sign_extend_imm(fimm);
                ram_addr = alu_result;
                // Write the data from the rt register.
                ram_wdata = gp_rdata2;
                ram_we    = 1'b1;
            end
        endcase

        // If the ALU asserts halt, we also set next_halt.
        if (alu_halt)
            next_halt = 1'b1;
    end

    //-----------------------------------
    // Helper Functions for Immediate Extension
    //-----------------------------------
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


// Testbench for CPU10bits
module tb_cpu10bits;
    reg clk;
    reg rst;
    //wire halted;
    
    // Instantiate the CPU10bits top module.
    CPU10bits dut (
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
//        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
//        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
//        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
//        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
//        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
        
        $finish;
    end
endmodule
