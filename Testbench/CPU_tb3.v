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
module CPU_tb_3(

    );
    //Input
    reg clock;
    reg Load;
    reg reset;
    reg [7:0] data_in;
    
    //Output
    wire [7:0] Instruction;
    wire [7:0] Acc;
    wire [7:0] Mem;
    wire [4:0] Program_counter;
    
    CPU uut (
        .clock(clock),
        .Load(Load),
        .reset(reset),
        .data_in(data_in),
        .Instruction(Instruction),
        .Acc(Acc),
        .Mem(Mem),
        .Program_counter(Program_counter)
    );
    

    
    initial begin
        clock = 0;
        reset = 1;
        Load = 1;
        data_in = 8'b0;
        
        // Reset the CPU
        #10 reset = 0;
        
        Load = 0;
        
        // Load instructions into instruction memory
        #10 data_in = 8'b11100011; // @00: JMP LOOP
        #10 data_in = 8'b00000000; // @01: Unused
        #10 data_in = 8'b00000000; // @02: Unused
        #10 data_in = 8'b10111011; // @03: LDA FN2
        #10 data_in = 8'b11011100; // @04: STO TEMP
        #10 data_in = 8'b01011010; // @05: ADD FN1
        #10 data_in = 8'b11011011; // @06: STO FN2
        #10 data_in = 8'b10111100; // @07: LDA TEMP
        #10 data_in = 8'b11011010; // @08: STO FN1
        #10 data_in = 8'b10011101; // @09: XOR LIMIT
        #10 data_in = 8'b00100000; // @0A: SKZ
        #10 data_in = 8'b11100011; // @0B: JMP LOOP
        #10 data_in = 8'b00000000; // @0C: HLT
        #10 data_in = 8'b10111111; // @0D: LDA ONE
        #10 data_in = 8'b11011010; // @0E: STO FN1
        #10 data_in = 8'b10111110; // @0F: LDA ZERO
        #10 data_in = 8'b11011011; // @10: STO FN2
        #10 data_in = 8'b11100011; // @11: JMP LOOP

        // Load data into memory
        #10 data_in = 8'b00000001; // @1A: FN1 = 1
        #10 data_in = 8'b00000000; // @1B: FN2 = 0
        #10 data_in = 8'b00000000; // @1C: TEMP = 0
        #10 data_in = 8'b10010000; // @1D: LIMIT = 144 (decimal)
        #10 data_in = 8'b00000000; // @1E: ZERO = 0
        #10 data_in = 8'b00000001; // @1F: ONE = 1

        Load = 0;

        // Run the simulation
        #200
        
        $finish;
    end
    
    always #5 clock = ~clock;

endmodule