`timescale 1 ns/1 ps
module ripple_carry_adder_10(
    input  wire [9:0] A,    // 10-bit operand A
    input  wire [9:0] B,    // 10-bit operand B
    input  wire       cin,  // initial carry in
    output wire [9:0] sum,  // 10-bit sum output
    output wire       cout  // final carry out
);
    wire [8:0] carry;  // internal carry wires

    // Bit 0
    full_adder fa0 (
        .a   (A[0]),
        .b   (B[0]),
        .cin (cin),
        .sum (sum[0]),
        .cout(carry[0])
    );
    
    // Bits 1 to 8
    genvar i;
    generate
        for (i = 1; i < 10; i = i + 1) begin : adder_bits
            // For the last bit, the carry out will be assigned to cout.
            if (i < 9) begin
                full_adder fa (
                    .a   (A[i]),
                    .b   (B[i]),
                    .cin (carry[i-1]),
                    .sum (sum[i]),
                    .cout(carry[i])
                );
            end else begin
                full_adder fa (
                    .a   (A[i]),
                    .b   (B[i]),
                    .cin (carry[i-1]),
                    .sum (sum[i]),
                    .cout(cout)
                );
            end
        end
    endgenerate
endmodule