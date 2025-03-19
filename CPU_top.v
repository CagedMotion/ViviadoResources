`timescale 1 ns/1 ps

module cpu_top(
    input  wire       clk,         // Global clock signal
    input  wire       reset,       // Global reset signal
//    input  wire       branch,      // Branch signal for the fetch unit
//    input  wire [9:0] branch_addr, // Branch address if branch is taken
    output wire [9:0] alu_result,  // ALU result (for observation)
    output wire       alu_halt,    // ALU halt flag (for observation)
//    output wire [9:0] pc_out,       // Current PC value from the fetch unit
    output wire [9:0] rom_out    //rom out for reg files
);

    // -----------------------------------------------------
    // Fetch Unit: Produces the PC value (program counter)
    // -----------------------------------------------------
    wire [9:0] pc;
    reg branch,jump;
    reg [9:0] branch_addr,jump_target;

    fetch_unit_with_reg fetch_unit (
        .clk(clk),
        .reset(reset),
        .halt(alu_halt),
        .branch(branch),
        .branch_addr(branch_addr),
        .pc_out(pc),
        .jump(jump),
        .jump_target(jump_target)
    );
    
     // rom wires
    
    // Instantiate ROM inside the fetch unit.
    task1rom ROM_task1 (
        .read_data(rom_out),
        .clk(clk),
        .address(pc)
    );
    
    // Expose the PC value
//    assign pc_out = pc;

    // -----------------------------------------------------
    // Register File and ALU (for example purposes)
    // In a full design, the PC would drive an instruction
    // memory and then an instruction decoder, which in turn
    // sets up the register file read/write addresses.
    // Here we use fixed addresses just to demonstrate connections.
    // -----------------------------------------------------
    
    // Wires connecting the register file and ALU
    wire [9:0] rdata1, rdata2;
    reg bank_sel;
    wire [9:0] alu_out;
    
    // ALU control signal; here, we force an ADD operation.
    reg [2:0] alu_ctrl;
//    assign alu_ctrl = 3'b000;  // 3'b000 corresponds to ADD

    // For demonstration, we use fixed addresses for the register file.
    // In a complete design, these would come from an instruction decoder.
    reg [2:0] raddr1, raddr2, waddr;
//    assign raddr1 = 3'b001;  // e.g., R[rs]
//    assign raddr2 = 3'b010;  // e.g., R[rt]
//    assign waddr  = 3'b011;  // e.g., destination register

    // Write-back: Write the ALU result back into the register file.
    reg we;
//    assign we    = 1'b1;     // Always enabled for demonstration
    reg [9:0]  wdata;
//    assign wdata = alu_out;  // Write ALU output

   
    // Instantiate the register file
    register_file RF (
        .clk(clk),
        .rst(reset),
        .we(we),
        .waddr(waddr),
        .wdata(wdata),
        .raddr1(raddr1),
        .raddr2(raddr2),
        .rdata1(rdata1),
        .rdata2(rdata2),
        .bank_sel(bank_sel)
    );
    
    reg [9:0]aluin_a, aluin_b;
    
    // Instantiate the ALU
    ALU alu_inst (
        .A(aluin_a),       // Operand A from register file
        .B(aluin_b),       // Operand B from register file
        .alu_ctrl(alu_ctrl),
        .result(alu_out),
        .halt(alu_halt)
    );
    
    // Output the ALU result for observation
    assign alu_result = alu_out;

    wire [4:0] instr;
    assign instr = {rom_out[9:7], rom_out[1:0]};
    
    always @(posedge clk) begin
        case (instr[4:2])
            3'b000 : begin
                case (instr[1:0])
                    2'b00 : begin           //add
                        branch = 10'b0;
                        jump = 10'b0;
                        bank_sel = rom_out[2];
                        raddr1 = rom_out[6:5];
                        raddr2 = rom_out[4:3];
                        aluin_a = rdata1;
                        aluin_b = rdata2;
                        alu_ctrl = 3'b000;
                        waddr = rom_out[4:3];
                        wdata = alu_out;
                        we = 1'b1;
                    end
                    2'b01 : begin           //sub
                        branch = 10'b0;
                        jump = 10'b0;
                        bank_sel = rom_out[2];
                        raddr1 = rom_out[6:5];
                        raddr2 = rom_out[4:3];
                        aluin_a = rdata1;
                        aluin_b = rdata2;
                        alu_ctrl = 3'b001;
                        waddr = rom_out[4:3];
                        wdata = alu_out;
                        we = 1'b1;
                    end
                    2'b10 : begin           //slt
                        branch = 10'b0;
                        jump = 10'b0;
                        bank_sel = rom_out[2];
                        raddr1 = rom_out[6:5];
                        raddr2 = rom_out[4:3];
                        aluin_a = rdata1;
                        aluin_b = rdata2;
                        alu_ctrl = 3'b010;
                        waddr = rom_out[4:3];
                        wdata = alu_out;
                        we = 1'b1;
                    end
                    2'b11 : begin           //nand
                        branch = 10'b0;
                        jump = 10'b0;
                        bank_sel = rom_out[2];
                        raddr1 = rom_out[6:5];
                        raddr2 = rom_out[4:3];
                        aluin_a = rdata1;
                        aluin_b = rdata2;
                        alu_ctrl = 3'b011;
                        waddr = rom_out[4:3];
                        wdata = alu_out;
                        we = 1'b1;
                    end
                endcase
            end
            3'b001 : begin
                case (instr[1:0])
                    2'b00 : begin           //slr
                        branch = 10'b0;
                        jump = 10'b0;
                        bank_sel = rom_out[2];
                        raddr1 = rom_out[6:5];
                        raddr2 = rom_out[4:3];
                        aluin_a = rdata1;
                        aluin_b = rdata2;
                        alu_ctrl = 3'b100;
                        waddr = rom_out[4:3];
                        wdata = alu_out;
                        we = 1'b1;
                    end
                    2'b01 : begin           //sll
                        branch = 10'b0;
                        jump = 10'b0;
                        bank_sel = rom_out[2];
                        raddr1 = rom_out[6:5];
                        raddr2 = rom_out[4:3];
                        aluin_a = rdata1;
                        aluin_b = rdata2;
                        alu_ctrl = 3'b101;
                        waddr = rom_out[4:3];
                        wdata = alu_out;
                        we = 1'b1;
                    end
                    2'b10 : begin           //halt
                        branch = 10'b0;
                        jump = 10'b0;
                        bank_sel = rom_out[2];
                        raddr1 = rom_out[6:5];
                        raddr2 = rom_out[4:3];
                        aluin_a = rdata1;
                        aluin_b = rdata2;
                        alu_ctrl = 3'b110;
                        waddr = rom_out[4:3];
                        wdata = alu_out;
                        we = 1'b0;
                    end
                    2'b11 : begin           //n/a
                        branch = 10'b0;
                        jump = 10'b0;
                        bank_sel = rom_out[2];
                        raddr1 = rom_out[6:5];
                        raddr2 = rom_out[4:3];
                        aluin_a = rdata1;
                        aluin_b = rdata2;
                        alu_ctrl = 3'b110;
                        waddr = rom_out[4:3];
                        wdata = alu_out;
                        we = 1'b0;
                    end
                endcase
            end
            3'b010 : begin                  //bne
                bank_sel = rom_out[2];
                raddr1 = rom_out[6:5];
                raddr2 = rom_out[4:3];
                aluin_a = rdata1;
                aluin_b = rdata2;
                alu_ctrl = 3'b111;
                we = 1'b0;
                if (alu_out == 0) begin
                    branch_addr = {8'b0,rom_out[1:0]};
                    jump_target = 10'b0;
                    branch = 10'd1;
                    jump = 10'd0;
                end
                else begin
                    branch_addr = 10'b0;
                    jump_target = 10'b0;
                    branch = 10'd0;
                    jump = 10'd0;
                end
            end
            3'b011 : begin                  //addi
                branch = 10'b0;
                jump = 10'b0;
                bank_sel = rom_out[2];
                raddr1 = rom_out[6:5];
                raddr2 = rom_out[4:3];
                aluin_a = rdata1;
                aluin_b = rom_out[1:0];
                alu_ctrl = 3'b001;
                waddr = rom_out[4:3];
                wdata = alu_out;
                we = 1'b1;
            end
            3'b100 : begin                  //jump
//                jump_target = pc + signex;                         ======ERROR======
                branch = 10'b0;
                jump = 10'b1;
                bank_sel = rom_out[2];
                raddr1 = rom_out[6:5];
                raddr2 = rom_out[4:3];
                aluin_a = rdata1;
                aluin_b = rom_out[1:0];
                alu_ctrl = 3'b001;
                waddr = rom_out[4:3];
                wdata = alu_out;
                we = 1'b0;
            end
        endcase
    end
        
endmodule
