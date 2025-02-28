`timescale 1ns/1ps

module CPU10bits(
    input  wire clk,
    input  wire rst,
    output wire cpu_halted  // Indicates that the CPU has halted.
);

    // PC & Fetch Unit
    wire [9:0] pc_val;
    reg         branch_sig;
    reg         jump_sig;
    reg  [9:0]  branch_target;
    reg  [9:0]  jump_target;
    
    // Latch for halted state.
    reg halted_reg;
    assign cpu_halted = halted_reg;
    
    // Instantiate the fetch unit (from Fetch_unit.v)
    fetch_unit FU_inst (
        .clk         (clk),
        .reset       (rst),
        .branch      (branch_sig),
        .jump        (jump_sig),
        .halted      (halted_reg),  // When true, the PC is frozen.
        .branch_addr (branch_target),
        .jump_target (jump_target),
        .pc_out      (pc_val)
    );
    
    // ROM (Task1 ROM from task1rom.v)
    wire [9:0] instr;
    task1rom ROM_inst (
        .address   (pc_val),
        .clk       (clk),
        .read_data (instr)
    );
    
    // Instruction Decode
    // Instruction format: [9:7] opcode, [6:5] rs, [4:3] rt, [2] bank_sel, [1:0] func/imm.
    wire [2:0] opcode    = instr[9:7];
    wire [1:0] rs_field  = instr[6:5];
    wire [1:0] rt_field  = instr[4:3];
    wire       bank_sel  = instr[2];    // This drives the GP register file.
    wire [1:0] fimm      = instr[1:0];   // For R-type functions or I-type immediate.
    wire [6:0] jmp_addr  = instr[6:0];   // For J-type instructions.
    
    // General-Purpose Register File (from Register_file.v)
    // The register file uses a separate 1-bit bank_sel along with 2-bit read/write addresses.
    wire [9:0] gp_rdata1;
    wire [9:0] gp_rdata2;
    reg  [1:0] gp_waddr;
    reg  [9:0] gp_wdata;
    reg        gp_we;

    
    register_file RF_inst (
        .clk       (clk),
        .rst       (rst),
        .bank_sel  (bank_sel),
        .raddr1    (rs_field),
        .raddr2    (rt_field),
        .waddr     (gp_waddr),
        .wdata     (gp_wdata),
        .we        (gp_we),
        .rdata1    (gp_rdata1),
        .rdata2    (gp_rdata2)
    );
    
//    // Special-Purpose Registers
//    // Zero Register (always outputs zero) - from zero_register.v.
//    wire [9:0] zero_val;
//    register_zero ZREG_inst (
//        .clk   (clk),
//        .reset (rst),
//        .en    (1'b1),      // Always enabled.
//        .din   (10'd0),     // Always 0.
//        .dout  (zero_val)
//    );
    
    // Stack Pointer Register (from Register_sp.v).
    // It is initialized to 10'h3FF and updated by push/pop operations.
//    reg        sp_we;
//    reg  [9:0] sp_din;
//    wire [9:0] sp_val;
//    register_sp SP_inst (
//        .clk   (clk),
//        .reset (rst),
//        .en    (sp_we),
//        .din   (sp_din),
//        .dout  (sp_val)
//    );
    
    // Simple update for the stack pointer.
    // (Replace the conditions with your actual push/pop control.)
//    always @(posedge clk or posedge rst) begin
//        if (rst) begin
//            sp_we  <= 1'b1;
//            sp_din <= 10'h3FF;  // Initialize SP to top of RAM.
//        end else begin
//            sp_we  <= 1'b0;
//            sp_din <= sp_val;
//            // Example conditions (currently disabled):
//            if (/* push condition */ 1'b0) begin
//                sp_we  <= 1'b1;
//                sp_din <= sp_val - 10'd1;
//            end else if (/* pop condition */ 1'b0) begin
//                sp_we  <= 1'b1;
//                sp_din <= sp_val + 10'd1;
//            end
//        end
//    end
    
    // ALU
    // The ALU takes two 10-bit inputs and a 3-bit control signal.
    // Its halt output is used to freeze the CPU.
    reg  [9:0] alu_inA;
    reg  [9:0] alu_inB;
    reg  [2:0] alu_ctrl;
    wire [9:0] alu_result;
    wire       alu_halt;
    
    ALU ALU_inst (
        .A         (alu_inA),
        .B         (alu_inB),
        .alu_ctrl  (alu_ctrl),
        .result    (alu_result),
        .halt      (alu_halt)
    );
    
    // RAM (from ram.v)
    // Implements a 1024×10-bit memory.
    reg        ram_we;
    reg  [9:0] ram_addr;
    reg  [9:0] ram_wdata;
    wire [9:0] ram_rdata;
    
    ram RAM_inst (
        .clk     (clk),
        .we      (ram_we),
        .address (ram_addr),
        .wdata   (ram_wdata),
        .rdata   (ram_rdata)
    );
    
    // Control / Datapath Logic
    // We decode the instruction (from the ROM) and drive the datapath.
    // Once the ALU outputs a halt, we latch the halted state.
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            branch_sig      <= 1'b0;
            jump_sig        <= 1'b0;
            branch_target   <= 10'd0;
            jump_target     <= 10'd0;
            halted_reg      <= 1'b0;
            
            gp_we           <= 1'b0;
            alu_ctrl        <= 3'b000;
            alu_inA         <= 10'd0;
            alu_inB         <= 10'd0;
            gp_wdata        <= 10'd0;
            gp_waddr        <= 2'b00;
            
            ram_we          <= 1'b0;
            ram_addr        <= 10'd0;
            ram_wdata       <= 10'd0;
        end else begin
            // Default assignments each cycle:
            branch_sig      <= 1'b0;
            jump_sig        <= 1'b0;
            branch_target   <= pc_val + 10'd1;
            jump_target     <= pc_val;
            gp_we           <= 1'b0;
            alu_ctrl        <= 3'b000;
            alu_inA         <= 10'd0;
            alu_inB         <= 10'd0;
            gp_wdata        <= 10'd0;
            ram_we          <= 1'b0;
            ram_addr        <= 10'd0;
            ram_wdata       <= 10'd0;
            gp_waddr        <= gp_waddr;
            
            if (halted_reg) begin
                // CPU remains halted.
            end else begin
                case (opcode)
                    // 000: R-type arithmetic: ADD, SUB, SLT, NAND.
                    3'b000: begin
                        alu_inA     <= gp_rdata1;
                        alu_inB     <= gp_rdata2;
                        case (fimm)
                            2'b00: alu_ctrl <= 3'b000; // ADD
                            2'b01: alu_ctrl <= 3'b001; // SUB
                            2'b10: alu_ctrl <= 3'b010; // SLT
                            2'b11: alu_ctrl <= 3'b011; // NAND
                        endcase
                        gp_we       <= 1'b1;
                        gp_wdata    <= alu_result;
                        gp_waddr  <= rt_field;
                    end
                    // 001: R-type shift or HALT.
                    // func/imm: 00 = SLR, 01 = SLL, 10 = HALT.
                    3'b001: begin
                        alu_inA  <= gp_rdata1;
                        case (fimm)
                          2'b00: begin
                                alu_ctrl <= 3'b100; // SLR
                                gp_we    <= 1'b1;
                                gp_wdata <= alu_result;
                                gp_waddr <= rt_field;
                            end
                            2'b01: begin
                                alu_ctrl <= 3'b101; // SLL
                                gp_we    <= 1'b1;
                                gp_wdata <= alu_result;
                                gp_waddr <= rt_field;
                            end
                            2'b10: begin
                                alu_ctrl <= 3'b110; // HALT
                                // No register write.
                            end
                        endcase
                    end
                    // 010: BNE - branch if gp_rdata1 != gp_rdata2.
                    3'b010: begin
                        if (gp_rdata1 != gp_rdata2)
                            branch_target <= pc_val + sign_extend_imm(fimm);
                        branch_sig <= 1'b1;
                    end
                    // 011: ADDI - add immediate to gp_rdata1.
                    3'b011: begin
                        alu_inA   <= gp_rdata1;
                        alu_inB   <= sign_extend_imm(fimm);
                        alu_ctrl  <= 3'b000; // ADD.
                        gp_we     <= 1'b1;
                        gp_wdata  <= alu_result;
                        gp_waddr  <= rt_field;
                    end
                    // 100: JUMP.
                    3'b100: begin
                        jump_target <= sign_extend_jmp(jmp_addr);
                        jump_sig    <= 1'b1;
                    end
                    // 101: BEQ - branch if gp_rdata1 == gp_rdata2.
                    3'b101: begin
                        if (gp_rdata1 == gp_rdata2)
                            branch_target <= pc_val + zero_extend_imm(fimm);
                        branch_sig <= 1'b1;
                    end
                    // 110: LOAD.
                    3'b110: begin
                        alu_inA  <= gp_rdata1;
                        alu_inB   <= sign_extend_imm(fimm);
                        alu_ctrl  <= 3'b000; // Compute effective address.
                        ram_addr  <= alu_result;
                        gp_we     <= 1'b1;
                        gp_wdata  <= ram_rdata;
                        gp_waddr  <= rt_field;
                    end
                    // 111: STORE.
                    3'b111: begin
                        alu_inA  <= gp_rdata1;
                        alu_inB   <= sign_extend_imm(fimm);
                        alu_ctrl  <= 3'b000; // Compute effective address.
                        ram_addr  <= alu_result;
                        ram_we    <= 1'b1;
                        ram_wdata <= gp_rdata2;
                    end
                    default: begin
                        alu_ctrl <= 3'b000;
                        alu_inB  <= 10'd0;
                    end
                endcase
                // Latch halt state if ALU signals halt.
                if (alu_halt)
                    halted_reg <= 1'b1;
            end
        end
    end

    // Immediate Extension Functions
    function [9:0] sign_extend_imm;
        input [1:0] imm;
        begin
            sign_extend_imm = {{8{imm[1]}}, imm};
        end
    endfunction

    function [9:0] zero_extend_imm;
        input [1:0] imm;
        begin
            zero_extend_imm = {8'd0, imm};
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
    wire halted;
    
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
        #PERIOD;
        //halted = 0;
        
        // Optionally, drive any test stimulus here.
        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;//#PERIOD;
//        #PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;#PERIOD;
//        #PERIOD;
        
        $finish;
    end
endmodule
