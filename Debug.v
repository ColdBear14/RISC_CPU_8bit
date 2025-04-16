`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:
// Design Name: 
// Module Name:
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module CPU(
    input clock,
    input Load,
    input reset,
    input [7:0] data_in,
    output [7:0] Instruction,
    output [7:0] Acc,
    output [7:0] Mem,
    output [4:0] Program_counter
    );

    reg [4:0] PC;
    wire [4:0] addr_wire;
    wire [4:0] Address;
    wire [7:0] Accumulator;
    wire [7:0] Data_out;
    wire En_mem_wire;
    wire En_acc_wire;
    wire En_cpu_wire;
    wire En_run_wire;
    wire [2:0] Opcode_wire;
    wire SKZ_cmp;
    wire [4:0] pc_jmp;
    wire [2:0] ALU_OP;
    wire [7:0] ALU_result;

    assign Instruction = {Opcode_wire, addr_wire};
    assign Acc = Accumulator;
    assign Mem = Data_out;


    Instruction_Memory im(
        .clock(clock),
        .reset(reset),
        .mem_ins(data_in),
        .Opcode_out(Opcode_wire),
        .Address_out(addr_wire)
        );
   
    Program_Counter pc(
        .clock(clock),
        .reset(reset),
        .addr(addr_wire),
        .Opcode(Opcode_wire),
        .SKZ_cmp(SKZ_cmp),
        .Load_in(Load),
        .En_cpu_in(En_cpu_wire),
        .Program_counter(Program_counter),
        .Address(Address)
       );
   
    Controller c(
        .clock(clock),
        .reset(reset),
        .Load_in(Load),
        .Opcode(Opcode_wire),
        .En_acc(En_acc_wire),
        .En_mem(En_mem_wire),
        .En_cpu(En_cpu_wire),
        .ALU_OP(ALU_OP)
        );
    
    Accumulator acc(
       .clock(clock),
       .reset(reset),
       .en_acc_in(En_acc_wire),
       .alu_result(ALU_result),
       .acc_out(Accumulator)
       );
   
    Data_Memory dm(
       .clock(clock),
       .reset(reset),
       .Data_in(ALU_result),
       .En(En_mem_wire),
       .Address(Address),
       .Data_out(Data_out)
       );

    ALU alu(
        .inA(Accumulator),
       .inB(Data_out),
       .alu_op(ALU_OP),
       .alu_out(ALU_result),
       .SKZ_cmp(SKZ_cmp)
       );
endmodule

module Accumulator(
    input clock,
    input reset,
    input en_acc_in,
    input [7:0] alu_result,
    output reg [7:0] acc_out
);
    
    always @(posedge clock or posedge reset) begin
        if (reset)
            acc_out <= 8'd0;
        else if (en_acc_in)
            acc_out <= alu_result;
    end
endmodule

module Adder_8bit(
    input [7:0] inA,
    input [7:0] inB,
    output [7:0] total
    );
    wire [7:0] carry;
    wire carry_out;
    
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

module Controller(
    input clock,
    input reset,
    input Load_in,
    input [2:0] Opcode,
    output En_acc,
    output En_mem,
    output En_cpu,
    output reg [2:0] ALU_OP
    );
    reg En_run;
    reg En_write_reg;
    reg En_write_mem;
    reg Load_prev;
    wire write_reg_next;
    wire write_mem_next;

    assign write_mem_next = (Opcode == 3'b110);
    assign write_reg_next = (Opcode == 3'b010 || Opcode == 3'b011 || Opcode == 3'b100 || Opcode == 3'b101);

    assign En_acc = ~Load_in && En_write_reg && En_run;
    assign En_mem = ~Load_in && En_write_mem && En_run;
    assign En_cpu = (~Load_in && |Opcode) || (Load_prev && ~Load_in);

        
    always @(posedge clock or posedge reset) begin
        if (reset) begin
            En_write_reg <= 1'b0;
            En_write_mem <= 1'b0;
            ALU_OP <= 3'b000;
            Load_prev <= 1'b0;
        end else if (En_cpu) begin
            En_write_reg <= write_reg_next;
            En_write_mem <= write_mem_next;
            ALU_OP <= Opcode;
            Load_prev <= Load_in;
        end
    end

    always @(posedge clock, posedge reset) begin
        if(reset) En_run <= 1'b0;
        else En_run <= En_cpu;
   end



endmodule

module Data_Memory(
    input clock,
    input reset,
    input [7:0] Data_in,
    input En,
    input [4:0] Address,
    output [7:0] Data_out
);
    reg [7:0] memory [31:0]; // Memory array with 32 elements, each 8 bits wide
    integer i;

    // Assign the output to the memory at the given address
    assign Data_out = memory[Address];

    // Memory update logic
    always @(posedge clock or posedge reset) begin
        if (reset) begin
            // Reset all memory elements
            for (i = 0; i < 32; i = i + 1) begin
                if (i == 8'h1B) memory[i] <= 8'hFF; // Special reset value for address 0x1B
                else if (i == 8'h1C) memory[i] <= 8'hAA; // Special reset value for address 0x1C
                else memory[i] <= 8'h00; // Default reset value
            end
        end else if (En) begin
            // Write data to the specified address
            memory[Address] <= Data_in;
        end
    end
endmodule

module Instruction_Memory(
    input clock,
    input reset,
    input [7:0] mem_ins,
    output reg [2:0] Opcode_out,
    output reg [4:0] Address_out
    );
    reg [2:0] opcode_next;
    reg [4:0] address_next;

    always @(*) begin
        opcode_next = mem_ins[7:5];
        address_next = mem_ins[4:0];
    end

    always @(posedge clock or posedge reset) begin
        if (reset) begin
            Opcode_out <= 3'b0;      
            Address_out <= 5'b0;    
        end else begin
            Opcode_out <= opcode_next;
            Address_out <= address_next;
        end
    end
endmodule

module Program_Counter(
    input clock,
    input reset,
    input [4:0] addr,
    input [2:0] Opcode,
    input SKZ_cmp,
    input Load_in,
    input En_cpu_in,
    output reg [4:0] Program_counter,
    output reg [4:0] Address
);

    reg [4:0] pc_next;

    always @(*) begin
        // Calculate the next PC value based on the Opcode and conditions
        if (Opcode == 3'b111) begin
            pc_next = addr; // JMP instruction
        end else if (Opcode == 3'b001 && SKZ_cmp) begin
            pc_next = Program_counter + 5'd2; // SKZ instruction
        end else if (Opcode == 3'b000) begin
            pc_next = Program_counter; // HLT instruction
        end else begin
            pc_next = Program_counter + 5'd1; // Default increment
        end
    end

    always @(posedge clock or posedge reset) begin
        if (reset) begin
            Program_counter <= 5'd0; // Reset PC to 0
            Address <= 5'd0;         // Reset Address to 0
        end else if (Load_in) begin
            Program_counter <= 5'd0; // Reset PC during load
        end else if (En_cpu_in) begin
            Program_counter <= pc_next; // Update PC with the calculated value
            Address <= addr;            // Update Address
        end
    end

endmodule




