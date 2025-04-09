module CPU_tb_1(

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
        #10 data_in = 8'b11111110; // @00: JMP TST_JMP
        #10 data_in = 8'b00000000; // @01: HLT
        #10 data_in = 8'b00000000; // @02: HLT
        #10 data_in = 8'b10111010; // @03: LDA DATA_1
        #10 data_in = 8'b00100000; // @04: SKZ
        #10 data_in = 8'b00000000; // @05: HLT
        #10 data_in = 8'b10111011; // @06: LDA DATA_2
        #10 data_in = 8'b00100000; // @07: SKZ
        #10 data_in = 8'b11101010; // @08: JMP SKZ_OK
        #10 data_in = 8'b00000000; // @09: HLT
        #10 data_in = 8'b11011100; // @0A: STO TEMP
        #10 data_in = 8'b10111010; // @0B: LDA DATA_1
        #10 data_in = 8'b11011100; // @0C: STO TEMP
        #10 data_in = 8'b10111100; // @0D: LDA TEMP
        #10 data_in = 8'b00100000; // @0E: SKZ
        #10 data_in = 8'b00000000; // @0F: HLT
        #10 data_in = 8'b10011011; // @10: XOR DATA_2
        #10 data_in = 8'b00100000; // @11: SKZ
        #10 data_in = 8'b11110100; // @12: JMP XOR_OK
        #10 data_in = 8'b00000000; // @13: HLT
        #10 data_in = 8'b10011011; // @14: XOR DATA_2
        #10 data_in = 8'b00100000; // @15: SKZ
        #10 data_in = 8'b00000000; // @16: HLT
        #10 data_in = 8'b00000000; // @17: HLT
        #10 data_in = 8'b11100000; // @18: JMP BEGIN
        #10 data_in = 8'b00000000; // @19: HLT
        #10 data_in = 8'b00000000; // @1A: DATA_1
        #10 data_in = 8'b11111111; // @1B: DATA_2
        #10 data_in = 8'b10101010; // @1C: TEMP
        #10 data_in = 8'b00000000; // @1D: Unused
        #10 data_in = 8'b11100011; // @1E: TST_JMP
        #10 data_in = 8'b00000000; // @1F: HLT

        Load = 0;

        // Run the simulation
        #200
        
        $finish;
    end
    
    always #5 clock = ~clock;

endmodule