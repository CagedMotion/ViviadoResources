`timescale 1 ns/1 ps
module ALU(
    input  wire [9:0] A,         // 10-bit operand A (e.g. from R[rs])
    input  wire [9:0] B,         // 10-bit operand B (e.g. from R[rt] or sign-extended imm)
    input  wire [2:0] alu_ctrl,  // ALU control signal (selects operation)
    output wire [9:0] result,    // 10-bit result output (goes to R[rd] or R[rt])
    output wire       halt       // Halt flag (for HALT instruction)
);


    // Internal wires for addition and subtraction
    wire [9:0] add_result;
    //wire       add_cout = 1'b0;
    
    // For subtraction, compute A - B = A + (~B) + 1
    wire [9:0] B_inverted;
    genvar j;
    generate
        for (j = 0; j < 10; j = j + 1) begin : invert_B_loop
            not u_not(B_inverted[j], B[j]);
        end
    endgenerate
    
    wire [9:0] sub_result;
    //wire       sub_cout = 1'b0;
    
    //wire unused_add_cout, unused_sub_cout;
    
    // 10-bit addition for ADD (A + B) with carry in = 0
    ripple_carry_adder_10 adder_inst (
        .A   (A),
        .B   (B),
        .cin (1'b0),
        .sum (add_result),
        .cout()
    );
    
    // 10-bit subtraction for SUB (A - B) implemented as A + (~B) + 1
    ripple_carry_adder_10 subtractor_inst (
        .A   (A),
        .B   (B_inverted),
        .cin (1'b1),
        .sum (sub_result),
        .cout()
    );
    

    
    // NAND Operation (bitwise NAND of A and B)
    wire [9:0] nand_result;
    generate
        for (j = 0; j < 10; j = j + 1) begin : nand_loop
            nand u_nand(nand_result[j], A[j], B[j]);
        end
    endgenerate
    
    // Shift Operations
    wire [9:0] slr_result;  // Shift Right Logical: A >> 1
    wire [9:0] sll_result;  // Shift Left Logical: A << 1
    assign slr_result = A >> 1;
    assign sll_result = A << 1;

    // SLT Operation (Set Less Than)
    // Compare A and B by using the subtraction result.
    // If A - B is negative (i.e. MSB of sub_result is 1), then set result = 1;
    // otherwise, result = 0 (in 10 bits: either 10'b0000000001 or 10'b0).

    wire [9:0] slt_result;
    assign slt_result = sub_result[9] ? 10'b0000000001 : 10'b0000000000;
    
    
    wire [9:0] equal_result,temp_equal;
    assign temp_equal = A - B;
    assign equal_result = ~(10'b0 | temp_equal);

    
    // ALU Output Multiplexer
    reg [9:0] alu_out;
    reg       halt_flag;
    
    // The ALU control coding is defined as:
    // 3'b000: ADD       → result = A + B
    // 3'b001: SUB       → result = A - B
    // 3'b010: SLT       → result = (A < B) ? 1 : 0
    // 3'b011: NAND      → result = ~(A & B)
    // 3'b100: SLR       → result = A >> 1
    // 3'b101: SLL       → result = A << 1
    // 3'b110: HALT      → set halt flag, result = 0
    // 3'b111: EQ        → result = ((A-B) nor 0)
    // default:        result = 0
    always @(*) begin
        // Default values
        halt_flag = 1'b0;
        case (alu_ctrl)
            3'b000: alu_out = add_result;   // ADD
            3'b001: alu_out = sub_result;   // SUB
            3'b010: alu_out = slt_result;   // SLT
            3'b011: alu_out = nand_result;  // NAND
            3'b100: alu_out = slr_result;   // SLR (shift right logical)
            3'b101: alu_out = sll_result;   // SLL (shift left logical)
            3'b110: begin
                alu_out   = 10'b0;          // For HALT, drive result low
                halt_flag = 1'b1;           // and set the halt flag.
            end
            3'b111: alu_out = equal_result;
            default: alu_out = 10'b0;
        endcase
    end
    
    assign result = alu_out;
    assign halt   = halt_flag;

endmodule

module tb_ALU;
    // Inputs
    reg [9:0] A, B;
    reg [2:0] alu_ctrl;
    // Outputs
    wire [9:0] result;
    wire halt;
    
    // Instantiate the ALU module (make sure the module name matches your design)
    ALU dut (
         .A(A),
         .B(B),
         .alu_ctrl(alu_ctrl),
         .result(result),
         .halt(halt)
    );
    parameter PERIOD = 10;
    initial begin
        // Test ADD: alu_ctrl = 3'b000 → result = A + B
        A = 10'd15;
        B = 10'd10;
        alu_ctrl = 3'b000;
        #PERIOD;
        $display("ADD: A=%d, B=%d, result=%d, halt=%b", A, B, result, halt);
        
        // Test SUB: alu_ctrl = 3'b001 → result = A - B
        A = 10'd20;
        B = 10'd8;
        alu_ctrl = 3'b001;
        #PERIOD;
        $display("SUB: A=%d, B=%d, result=%d, halt=%b", A, B, result, halt);
        
        // Test SLT: alu_ctrl = 3'b010 → result = (A < B) ? 1 : 0
        A = 10'd5;
        B = 10'd10;
        alu_ctrl = 3'b010;
        #PERIOD;
        $display("SLT: A=%d, B=%d, result=%d, halt=%b", A, B, result, halt);
        
        // Test NAND: alu_ctrl = 3'b011 → result = ~(A & B)
        A = 10'b1010101010;
        B = 10'b1100110011;
        alu_ctrl = 3'b011;
        #PERIOD;
        $display("NAND: A=%b, B=%b, result=%b, halt=%b", A, B, result, halt);
        
        // Test SLR: alu_ctrl = 3'b100 → result = A >> 1
        A = 10'b1000000001;
        // B is don’t-care for shift operations
        alu_ctrl = 3'b100;
        #PERIOD;
        $display("SLR: A=%b, result=%b, halt=%b", A, result, halt);
        
        // Test SLL: alu_ctrl = 3'b101 → result = A << 1
        A = 10'b0000011111;
        alu_ctrl = 3'b101;
        #PERIOD;
        $display("SLL: A=%b, result=%b, halt=%b", A, result, halt);
        
        // Test HALT: alu_ctrl = 3'b110 → result = 0, halt flag = 1
        A = 10'd0;
        B = 10'd0;
        alu_ctrl = 3'b110;
        #PERIOD;
        $display("HALT: A=%d, B=%d, result=%d, halt=%b", A, B, result, halt);
        
        $finish;
    end
endmodule
