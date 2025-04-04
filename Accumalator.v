module Accumalator(
    input clk,
    input reset,
    input en_acc_in,
    input [7:0] alu_result,
    output [7:0] acc_out
    );
    reg [7:0] Accumulator;

    always @(posedge clk, posedge reset) begin
        if(reset) Accumulator <= 8'd0;
        else if(en_acc) Accumulator <= result;
   end

    assign alu_result = Accumulator;
    
endmodule