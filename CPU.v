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
        .En_run(En_run_wire),
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






