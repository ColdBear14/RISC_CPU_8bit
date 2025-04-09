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