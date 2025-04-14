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
module CPU_tb_2(

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
        Load = 0;
        data_in = 8'b0;
        
        // Reset the CPU
        #10 reset = 0;
        
        Load = 1;
        
        // Load instructions into instruction memory
        #10 data_in = 8'b10111011; // @00: LDA DATA_2
        #10 data_in = 8'b01111100; // @01: AND DATA_3
        #10 data_in = 8'b10011011; // @02: XOR DATA_2
        #10 data_in = 8'b00100000; // @03: SKZ
        #10 data_in = 8'b00000000; // @04: HLT
        #10 data_in = 8'b01011010; // @05: ADD DATA_1
        #10 data_in = 8'b00100000; // @06: SKZ
        #10 data_in = 8'b11101001; // @07: JMP ADD_OK
        #10 data_in = 8'b00000000; // @08: HLT
        #10 data_in = 8'b10011100; // @09: XOR DATA_3
        #10 data_in = 8'b01011010; // @0A: ADD DATA_1
        #10 data_in = 8'b11011101; // @0B: STO TEMP
        #10 data_in = 8'b10111010; // @0C: LDA DATA_1
        #10 data_in = 8'b01011101; // @0D: ADD TEMP
        #10 data_in = 8'b00100000; // @0E: SKZ
        #10 data_in = 8'b00000000; // @0F: HLT
        #10 data_in = 8'b00000000; // @10: HLT
        #10 data_in = 8'b11100000; // @11: JMP BEGIN
        #10 data_in = 8'b00000001; // @1A: DATA_1
        #10 data_in = 8'b10101010; // @1B: DATA_2
        #10 data_in = 8'b11111111; // @1C: DATA_3
        #10 data_in = 8'b00000000; // @1D: TEMP

        Load = 0;

        // Run the simulation
        #200
        
        $finish;
    end
    
    always #5 clock = ~clock;

endmodule