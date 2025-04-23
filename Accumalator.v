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