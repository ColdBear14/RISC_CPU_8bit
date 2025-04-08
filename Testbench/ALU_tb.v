`timescale 1ns / 1ps

module ALU_tb;

    // Inputs
    reg [7:0] inA;
    reg [7:0] inB;
    reg [2:0] alu_op;

    // Outputs
    wire [7:0] alu_out;
    wire SKZ_cmp;

    // Instantiate the ALU
    ALU uut (
        .inA(inA),
        .inB(inB),
        .alu_op(alu_op),
        .alu_out(alu_out),
        .SKZ_cmp(SKZ_cmp)
    );

    initial begin
        $display("===== ALU TEST START =====");

        // Test STO (opcode: 110)
        alu_op = 3'b110; inA = 8'hAB; inB = 8'hFF;
        #5 $display("STO:  alu_out = %h (Expected: AB), SKZ = %b", alu_out, SKZ_cmp);

        // Test LDA (opcode: 101)
        alu_op = 3'b101; inA = 8'h00; inB = 8'h3C;
        #5 $display("LDA:  alu_out = %h (Expected: 3C), SKZ = %b", alu_out, SKZ_cmp);

        // Test AND (opcode: 011)
        alu_op = 3'b011; inA = 8'hF0; inB = 8'h0F;
        #5 $display("AND:  alu_out = %h (Expected: 00), SKZ = %b", alu_out, SKZ_cmp);

        // Test XOR (opcode: 100)
        alu_op = 3'b100; inA = 8'hA5; inB = 8'h5A;
        #5 $display("XOR:  alu_out = %h (Expected: FF), SKZ = %b", alu_out, SKZ_cmp);

        // Test ADD (opcode: 010) using Adder_8bit
        alu_op = 3'b010; inA = 8'h0F; inB = 8'h01;
        #5 $display("ADD:  alu_out = %h (Expected: 10), SKZ = %b", alu_out, SKZ_cmp);

        // Test SKZ_cmp for zero result
        alu_op = 3'b011; inA = 8'h00; inB = 8'h00;
        #5 $display("AND Zero: alu_out = %h, SKZ = %b (Expected: 00, 1)", alu_out, SKZ_cmp);

        // Test SKZ_cmp for non-zero result
        alu_op = 3'b011; inA = 8'hF0; inB = 8'hFF;
        #5 $display("AND NonZero: alu_out = %h, SKZ = %b (Expected: F0, 0)", alu_out, SKZ_cmp);

        $display("===== ALU TEST END =====");
        #10 $finish;
    end

endmodule
