`timescale 1 ns/1 ps
module full_adder(
    input  wire a,    // one bit of A
    input  wire b,    // one bit of B
    input  wire cin,  // carry in
    output wire sum,  // sum bit
    output wire cout  // carry out
);
    wire axorb;
    // Sum = a XOR b XOR cin
    xor (axorb, a, b);
    xor (sum, axorb, cin);
    
    // Carry-out = (a & b) | ((a XOR b) & cin)
    wire a_and_b, axorb_and_cin;
    and (a_and_b, a, b);
    and (axorb_and_cin, axorb, cin);
    or  (cout, a_and_b, axorb_and_cin);
endmodule