`timescale 1ns/1ps

module Accumulator_tb;

    // Testbench signals
    reg clk;
    reg reset;
    reg en_acc_in;
    reg [7:0] alu_result;
    wire [7:0] acc_out;

    // Instantiate the Accumulator module
    Accumulator uut (
        .clk(clk),
        .reset(reset),
        .en_acc_in(en_acc_in),
        .alu_result(alu_result),
        .acc_out(acc_out)
    );

    // Clock generator: 10ns period
    always #5 clk = ~clk;

    initial begin
        // Initialize signals
        clk = 0;
        reset = 1;
        en_acc_in = 0;
        alu_result = 8'h00;

        // Hold reset for a bit
        #10;
        reset = 0;

        // Test 1: Load 8'h55 into accumulator
        alu_result = 8'h55;
        en_acc_in = 1;
        #10; // One clock cycle

        // Test 2: Disable enable, change alu_result, acc_out should NOT change
        en_acc_in = 0;
        alu_result = 8'hAA;
        #10;

        // Test 3: Enable again, new value should be stored
        en_acc_in = 1;
        alu_result = 8'hF0;
        #10;

        // Test 4: Apply reset
        reset = 1;
        #10;
        reset = 0;
        en_acc_in = 0;
        alu_result = 8'hFF;

        // Test 5: Enable again after reset
        en_acc_in = 1;
        #10;

        // End simulation
        $finish;
    end

    // Monitor output
    initial begin
        $monitor("Time: %0t | reset=%b | en_acc_in=%b | alu_result=%h | acc_out=%h", 
                  $time, reset, en_acc_in, alu_result, acc_out);
    end

endmodule
