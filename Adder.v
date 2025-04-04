
module Adder_8bit(
    input [7:0] inA,
    input [7:0] inB,
    output [7:0] total
    );
    wire [7:0] carry;
    reg carry_out;
    
    // Full adder logic for each bit
    assign {carry[0], total[0]} = inA[0] + inB[0];
    assign {carry[1], total[1]} = inA[1] + inB[1] + carry[0];
    assign {carry[2], total[2]} = inA[2] + inB[2] + carry[1];
    assign {carry[3], total[3]} = inA[3] + inB[3] + carry[2];
    assign {carry[4], total[4]} = inA[4] + inB[4] + carry[3];
    assign {carry[5], total[5]} = inA[5] + inB[5] + carry[4];
    assign {carry[6], total[6]} = inA[6] + inB[6] + carry[5];
    assign {carry_out, total[7]} = inA[7] + inB[7] + carry[6];
    
endmodule