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
module Data_Memory_tb;

    // Inputs
    reg clock;
    reg reset;
    reg [7:0] Data_in;
    reg En;
    reg [4:0] Address;

    // Output
    wire [7:0] Data_out;

    // Instantiate the Unit Under Test (UUT)
    Data_Memory uut (
        .clock(clock),
        .reset(reset),
        .Data_in(Data_in),
        .En(En),
        .Address(Address),
        .Data_out(Data_out)
    );

    // Clock generator: 10ns period
    always #5 clock = ~clock;

    initial begin
        // Initialize
        clock = 0;
        reset = 0;
        Data_in = 8'h00;
        En = 0;
        Address = 5'h00;

        // Apply reset
        #2 reset = 1;
        #10 reset = 0;

        // Wait for memory to initialize
        #5;

        // === Verify RESET values ===
        Address = 5'h1A;  // DATA_1
        #5 $display("DATA_1 (0x1A) = %h (Expected: 00)", Data_out);

        Address = 5'h1B;  // DATA_2
        #5 $display("DATA_2 (0x1B) = %h (Expected: FF)", Data_out);

        Address = 5'h1C;  // TEMP
        #5 $display("TEMP   (0x1C) = %h (Expected: AA)", Data_out);

        // === Test Writing ===
        Address = 5'h1C;  // TEMP
        Data_in = 8'h5A;
        En = 1;
        #10 En = 0;

        // === Verify Write ===
        Address = 5'h1C;
        #5 $display("TEMP (0x1C) after write = %h (Expected: 5A)", Data_out);

        // Finish simulation
        #20 $finish;
    end

endmodule
