`timescale 1ns/1ps

module cache_pipeline_CPU10bits(
    input  wire clk,
    input  wire rst,
    output wire cpu_halted,  // Indicates that the CPU has halted.
    output wire instr_valid
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
    
    //stall signal wire
    wire stall_global;
    
    reg instr_valid_r;
    always @(posedge clk or posedge rst) begin
        if (rst)
            instr_valid_r <= 1'b0;
        else
            instr_valid_r <= !stall_global;  
    end
    assign instr_valid = instr_valid_r & ~halted_reg;
    
    // Instantiate updated Fetch Unit
    fetch_unit FU_inst (
        .clk(clk),
        .reset(rst),
        .stalled(stall_global),
        .halted(halted_reg),
        .branch(branch_sig),
        .jump(jump_sig),
        .branch_addr(branch_target),
        .jump_target(jump_target),
        .pc_out(pc)
    );
    // Instruction Memory (ROM)
//    task1rom ROM_inst (
//        .address(pc),
//        .read_data(instr)
//    );

//    task2rom ROM_inst (
//        .address(pc),
//        .read_data(instr)
//    );
    
    task3rom ROM_inst (
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
    
    
    // Single hazard unit instance:
    wire load_use_hz;     // generate this from your load-use detection logic
    wire branch_hz;       // generate this from your branch-pending logic
    assign load_use_hz = 1'b0;
    assign branch_hz = 1'b0;
    
    hazard_unit hu (
      .cache_ready     (cache_ready),
      .load_use_hazard (load_use_hz),
      .branch_hazard   (branch_hz),
      .stall           (stall_global)
    );

    
    // Signals for FD stage (to be latched into FD->EM pipeline register)
    reg  [9:0] alu_inA, alu_inB;
    reg  [2:0] fd_alu_ctrl;
    reg        fd_reg_we;
    reg        fd_mem_we;
    reg        fd_mem_re;
    reg  [9:0] fd_store_data; // Data to be stored for STORE instructions

    // FD Stage Decoding (same as before)
    always @(*) begin
        // Defaults
        fd_alu_ctrl   = 3'b000;
        fd_reg_we     = 1'b0;
        fd_mem_we     = 1'b0;
        fd_mem_re     = 1'b0;
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
                    2'b11: fd_alu_ctrl = 3'b111; // nop
                endcase
                if (fimm != (2'b10 || 2'b11))
                    fd_reg_we = 1'b1;
            end

            // 010: BNE: Branch if not equal.
            3'b010: begin
                fd_alu_ctrl = 3'b111;
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
                fd_alu_ctrl = 3'b111;
                jump_sig    = 1'b1;
                jump_target = sign_extend_jmp(jmp_addr);
            end

            // 101: BEQ: Branch if equal.
            3'b101: begin
                fd_alu_ctrl = 3'b111;
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
        .en(!stall_global),
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

    //-------------------------------------------------------------------------
    // Instead of connecting the ALU output directly to a RAM instance,
    // insert the Cache here between the EM->WB pipeline register and the RAM.
    //-------------------------------------------------------------------------
    // Signals on the RAM side of the Cache (20-bit bidirectional bus).
    wire [19:0] ram_data_bus;
    wire [9:0]  cache_mem_addr;
    wire        cache_mem_rw;
    wire        cache_mem_req;
    wire        mem_ready_from_RAM;
    
    //----------------------------------------------------------
    // Bidirectional CPU Data Bus with Tri-State Control
    //----------------------------------------------------------
    // Declare the CPU data bus as an inout signal.
    wire [9:0] cpu_data_bus;
    
    // Tri-State Buffer:
    //   When em_mem_we is high (store operation), drive the bus with em_store_data.
    //   When em_mem_we is low (load operation), the bus is high impedance and the cache drives the bus.
    assign cpu_data_bus = (em_mem_we) ? em_store_data : 10'bz;
    
    Cache Cache_inst (
         .clk(clk),
         .rst(rst),
         .CPU_RW(em_mem_we),        // Use mem_we as store indicator (1: write, 0: read)
         .cpu_address(alu_result),   // Effective address from the ALU
         .mem_ready(mem_ready_from_RAM),
         .mem_data_ram_bus(ram_data_bus),
         .cpu_data_bus(cpu_data_bus),  // Data output for load operations
         .cache_ready(cache_ready),            // (Optional: can be used for stall control)
         .mem_addr(cache_mem_addr),
         .mem_rw(cache_mem_rw),
         .mem_req(cache_mem_req)
    );
    
    //ram instantiation.
//        ramtask1_for_cache RAM_inst (
//        .clk(clk),
//        .we(cache_mem_rw),       // Write enable as driven by the Cache.
//        .address(cache_mem_addr),
//        .data(ram_data_bus),
//        .mem_ready(mem_ready_from_RAM),
//        .mem_req(cache_mem_req)
//    );

//    ramtask2_for_cache RAM_inst (
//        .clk(clk),
//        .we(cache_mem_rw),       // Write enable as driven by the Cache.
//        .address(cache_mem_addr),
//        .data(ram_data_bus),
//        .mem_ready(mem_ready_from_RAM),
//        .mem_req(cache_mem_req)
//    );
    
        ramtask3_for_cache RAM_inst (
        .clk(clk),
        .we(cache_mem_rw),       // Write enable as driven by the Cache.
        .address(cache_mem_addr),
        .data(ram_data_bus),
        .mem_ready(mem_ready_from_RAM),
        .mem_req(cache_mem_req)
    );
    
    //----------------------------------------------------------
    // EM->WB Pipeline Register
    //----------------------------------------------------------
    Exe_Mem_WB_reg EM_WB_reg (
        .clk(clk), 
        .reset(rst),
        .en(!stall_global), 
        .alu_result_in(alu_result), 
        .alu_result_out(wb_alu_result),
        .ram_rdata_in(cpu_data_bus),    // Use data coming from the Cache
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

module tb_cache_pipeline_cpu10bits;
    //------------------------------------------------------------------------
    // Parameters
    //------------------------------------------------------------------------
    parameter PERIOD       = 10;       // ns (→ 100 MHz)
    
    //------------------------------------------------------------------------
    // DUT I/Os
    //------------------------------------------------------------------------
    reg        clk = 1'b1;
    reg        rst = 1'b1;
    wire       halted;

    //------------------------------------------------------------------------
    // Performance counters
    //------------------------------------------------------------------------
    integer    cycle_count   = 0;
    real       start_time_ns;
    real       stop_time_ns;
    // (Optional) if your CPU exports an "instr_valid" pulse, you can count instructions:
    integer instr_count = 0;
    wire    instr_valid;  // hook this up to your core's retire-valid

    //------------------------------------------------------------------------
    // Clock gen
    //------------------------------------------------------------------------
    always #(PERIOD/2) clk = ~clk;

    //------------------------------------------------------------------------
    // Instantiate DUT
    //------------------------------------------------------------------------
    cache_pipeline_CPU10bits dut (
        .clk       (clk),
        .rst       (rst),
        .cpu_halted(halted),
        .instr_valid(instr_valid)
    );

    //------------------------------------------------------------------------
    // Cycle counter
    //------------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst && !halted) begin
            cycle_count = cycle_count + 1;
        end
        // Optional instruction counting:
        if (instr_valid) instr_count = instr_count + 1;
    end

    //------------------------------------------------------------------------
    // Reset release & timestamp the start
    //------------------------------------------------------------------------
    initial begin
        #PERIOD;        // hold reset for 1 cycle
        rst = 1'b0;
        @(negedge rst); // wait for reset deassert
        start_time_ns = $realtime;
    end

    //------------------------------------------------------------------------
    // Wait for halt, then report
    //------------------------------------------------------------------------
    initial begin
        wait (halted);
        stop_time_ns = $realtime;
        $display("\n=== CPU PERFORMANCE ===");
        $display("  Cycles executed     : %0d", cycle_count);
        $display("  Simulated time      : %0.3f ns", stop_time_ns - start_time_ns);
        $display("  Wall-clock @100 MHz : %0.3f µs", (stop_time_ns - start_time_ns)/1e3);
        // if you counted instrs:
        $display("  Instructions        : %0d", instr_count);
        $display("  CPI                 : %0.2f", cycle_count / (instr_count*1.0));
        $finish;
    end
endmodule

