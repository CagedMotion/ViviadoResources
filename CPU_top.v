`timescale 1 ns/1 ps

module cpu_top(
    input  wire       clk,        // Global clock signal
    input  wire       reset,      // Global reset signal
    output wire [9:0] alu_result, // ALU result (for observation)
    output wire       alu_halt    // ALU halt flag (for observation)
);
    // Register file read data (operands)
    wire [9:0] rdata1, rdata2;
    // ALU output
    wire [9:0] alu_out;
 
    // Control signals for the ALU
    // For example, here we force an ADD operation.
    wire [2:0] alu_ctrl;
    assign alu_ctrl = 3'b000;  // 3'b000 corresponds to ADD
    
    // Register file read addresses.
    // In a full design, these would come from an instruction decoder.
    // For this example, we simply choose two registers.
    wire [2:0] raddr1, raddr2;
    assign raddr1 = 3'b001;  // Read from register 1 (source operand, R[rs])
    assign raddr2 = 3'b010;  // Read from register 2 (source operand, R[rt])
    
    // Write-back control for the register file.
    // For this example, we write the ALU result into register 3.
    wire        we;
    wire [2:0]  waddr;
    wire [9:0]  wdata;
    
    assign waddr = 3'b011;   // Destination register (R[rt] if performing R[rt] = R[rt] + R[rs])
    assign we    = 1'b1;     // For demonstration, write is always enabled
    assign wdata = alu_out;  // Write the ALU result back to the register file
    
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
    
    ALU alu_inst (
        .A(rdata1),       // Connect read port 1 from register file to ALU operand A
        .B(rdata2),       // Connect read port 2 from register file to ALU operand B
        .alu_ctrl(alu_ctrl),
        .result(alu_out), // ALU computed result
        .halt(alu_halt)
    );
    // outputs
    assign alu_result = alu_out;

endmodule
