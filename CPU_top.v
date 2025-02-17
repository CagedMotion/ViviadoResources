`timescale 1 ns/1 ps

module cpu_top(
    input  wire       clk,         // Global clock signal
    input  wire       reset,       // Global reset signal
    input  wire       branch,      // Branch signal for the fetch unit
    input  wire [9:0] branch_addr, // Branch address if branch is taken
    output wire [9:0] alu_result,  // ALU result (for observation)
    output wire       alu_halt,    // ALU halt flag (for observation)
    output wire [9:0] pc_out       // Current PC value from the fetch unit
);

    // -----------------------------------------------------
    // Fetch Unit: Produces the PC value (program counter)
    // -----------------------------------------------------
    wire [9:0] pc;
    fetch_unit_with_reg fetch_unit (
        .clk(clk),
        .reset(reset),
        .branch(branch),
        .branch_addr(branch_addr),
        .pc_out(pc)
    );
    // Expose the PC value
    assign pc_out = pc;

    // -----------------------------------------------------
    // Register File and ALU (for example purposes)
    // In a full design, the PC would drive an instruction
    // memory and then an instruction decoder, which in turn
    // sets up the register file read/write addresses.
    // Here we use fixed addresses just to demonstrate connections.
    // -----------------------------------------------------
    
    // Wires connecting the register file and ALU
    wire [9:0] rdata1, rdata2;
    wire [9:0] alu_out;
    
    // ALU control signal; here, we force an ADD operation.
    wire [2:0] alu_ctrl;
    assign alu_ctrl = 3'b000;  // 3'b000 corresponds to ADD

    // For demonstration, we use fixed addresses for the register file.
    // In a complete design, these would come from an instruction decoder.
    wire [2:0] raddr1, raddr2, waddr;
    assign raddr1 = 3'b001;  // e.g., R[rs]
    assign raddr2 = 3'b010;  // e.g., R[rt]
    assign waddr  = 3'b011;  // e.g., destination register

    // Write-back: Write the ALU result back into the register file.
    wire        we;
    assign we    = 1'b1;     // Always enabled for demonstration
    wire [9:0]  wdata;
    assign wdata = alu_out;  // Write ALU output

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
        .rdata2(rdata2)
    );
    
    // Instantiate the ALU
    ALU alu_inst (
        .A(rdata1),       // Operand A from register file
        .B(rdata2),       // Operand B from register file
        .alu_ctrl(alu_ctrl),
        .result(alu_out),
        .halt(alu_halt)
    );
    
    // Output the ALU result for observation
    assign alu_result = alu_out;

endmodule
