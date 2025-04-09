module ALU(
    input [7:0] inA,
    input [7:0] inB,
    input [2:0] alu_op,
    output [7:0] alu_out,
    output SKZ_cmp
    );
    reg [7:0] alu_result;
    reg SKZ_source;
    wire [7:0] adder_result;
    
    Adder ad8(inA, inB, adder_result);
    
    always @(*)
    begin
        case(alu_op)
        3'b110: // STO 
            alu_result = inA;
        3'b101: // LDA
            alu_result = inB;
        3'b011: // AND
            alu_result = inA & inB;
        3'b100: // XOR
            alu_result = inA ^ inB;
        3'b010: // ADD
            alu_result = adder_result;
        default:
            alu_result = inA;
        endcase
        
        case (alu_op)
            3'b000, 3'b001, 3'b110, 3'b111: // NOR all bits
                SKZ_source = ~(inA[7] | inA[6] | inA[5] | inA[4] |
                               inA[3] | inA[2] | inA[1] | inA[0]);
            default: // NOR all bits
                SKZ_source = ~(alu_result[7] | alu_result[6] | alu_result[5] | alu_result[4] |
                               alu_result[3] | alu_result[2] | alu_result[1] | alu_result[0]);
                
        endcase
    end

    assign alu_out = alu_result;
    assign SKZ_cmp = SKZ_source;
endmodule
