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