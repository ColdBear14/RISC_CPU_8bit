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
    input Clk,
    input Load,
    input RX,
    input Reset,
    output FE,
    output [7:0] Instruction,
    output [7:0] Acc,
    output [7:0] Mem,
    output [4:0] Program_counter
    );
    parameter Baudrate = 10415;
    wire [7:0] Ins;
    reg [4:0] PC;
    reg En_run;
    wire [4:0] addr;
    reg [4:0] Address;
    reg [7:0] Accumulator;
    wire [7:0] Data_out;
    wire En_mem;
    wire En_acc;
    wire En_cpu;
    wire [2:0] Opcode;
    wire jmp_or_not;
    wire SKZ_cmp;
    wire [4:0] pc_jmp;
    wire En_write_reg;
    wire En_write_mem;
    wire [2:0] ALU_OP;
    wire tmp_e;
    wire tmp_f;
    wire [7:0] result;
    // use uart to receive instruction data
    assign Instruction = {Opcode, addr};
    assign Acc = Accumulator;
    assign Mem = Data_out;
     UART #(.Baudrate(Baudrate)) a(
       .Clk(Clk),
       .RX(RX),
       .Load(Load),
       .PC(Program_counter),
       .data_out(Ins),
       .FE(FE)
   );
    //every time Clk rise edge, opcode and address change according to Program_counter
   Instruction_Memory b(
       .Clk(Clk),
       .Reset(Reset),
       .mem_ins(Ins),
       .Opcode(Opcode),
       .Address(addr)
       );
   
   assign pc_jmp = (Opcode == 3'b111) ? addr : 
                    (Opcode == 3'b001 && SKZ_cmp)? (PC + 5'd2) : 
                    (Opcode == 3'b000) ? PC : 
                    (PC + 5'd1);
   //Choose what pc next in case Load
    assign Program_counter = ~Load ? pc_jmp : 5'd0;   
   //When HLT or Load then disable
   assign En_cpu = ~Load && | Opcode;
   
   //Program counter and Address
   always @(posedge Clk, posedge Reset) begin
        if(Reset) begin
            PC <= 5'd0;
            Address <= 5'd0;
        end
        else if(En_cpu) begin
            PC <= Program_counter;
            Address <= addr; 
        end
   end
   
   //En_run
   always @(posedge Clk, posedge Reset) begin
        if(Reset) En_run <= 1'b0;
        else En_run <= En_cpu;
   end
   
   //Control unit
   Control_Unit c(
       .Clk(Clk),
       .Reset(Reset),
       .En(En_cpu),
       .Opcode(Opcode),
       .En_write_reg(En_write_reg),
       .En_write_mem(En_write_mem),
       .ALU_OP(ALU_OP)
       );
       
   //When HLT disable Accumulator and Data_Memory after one cycle
   assign En_acc = ~Load && En_write_reg && En_run;
   assign En_mem = ~Load && En_write_mem && En_run;
   
   //Accumulator
   always @(posedge Clk, posedge Reset) begin
        if(Reset) Accumulator <= 8'd0;
        else if(En_acc) Accumulator <= result;
   end
   
   //Data memory
   Data_Memory d(
       .Clk(Clk),
       .Reset(Reset),
       .Data_in(result),
       .En(En_mem),
       .Address(Address),
       .Data_out(Data_out)
       );
   //ALU
   
   ALU e(
       .inA(Accumulator),
       .inB(Data_out),
       .ALU_OP(ALU_OP),
       .result(result),
       .SKZ_cmp(SKZ_cmp)
       );
endmodule

module ALU(
    input [7:0] inA,
    input [7:0] inB,
    input [2:0] ALU_OP,
    output [7:0] result,
    output SKZ_cmp
    );
    reg [7:0] ALU_result;
    reg SKZ_source;
    wire [7:0] adder_result;
    
    assign result = ALU_result;
    assign SKZ_cmp = SKZ_source;
    ADDER full_adder(inA, inB, adder_result);
    
    always @(*)
    begin
        case(ALU_OP)
        3'b110: // STO 
            ALU_result = inA;
        3'b101: // LDA
            ALU_result = inB;
        3'b011: // AND
            ALU_result = inA & inB;
        3'b100: // XOR
            ALU_result = inA ^ inB;
        3'b010: // ADD
            ALU_result = adder_result;
        default:
            ALU_result = inA;
        endcase
        
        case (ALU_OP)
            3'b000, 3'b001, 3'b110, 3'b111: // NOR all bits
                SKZ_source = ~(inA[7] | inA[6] | inA[5] | inA[4] |
                               inA[3] | inA[2] | inA[1] | inA[0]);
            default: // NOR all bits
                SKZ_source = ~(ALU_result[7] | ALU_result[6] | ALU_result[5] | ALU_result[4] |
                               ALU_result[3] | ALU_result[2] | ALU_result[1] | ALU_result[0]);
                
        endcase
        
    end
    
endmodule

module Control_Unit(
    input Clk,
    input Reset,
    input En,
    input [2:0] Opcode,
    output reg En_write_reg,
    output reg En_write_mem,
    output reg [2:0] ALU_OP
    );
    
    wire write_reg_next;
    wire write_mem_next;
    assign write_mem_next = (Opcode == 3'b110);
    assign write_reg_next = (Opcode == 3'b010 || Opcode == 3'b011 || Opcode == 3'b100 || Opcode == 3'b101);
        
    always @(posedge Clk or posedge Reset) begin
        if (Reset) begin
            En_write_reg <= 1'b0;
            En_write_mem <= 1'b0;
            ALU_OP <= 3'b000;
        end else if (En) begin
            En_write_reg <= write_reg_next;
            En_write_mem <= write_mem_next;
            ALU_OP <= Opcode;
        end
    end
endmodule

module data_mem(
    input Clk,
    input Reset,
    input  [7:0] Reset_value,
    input En,
    input [7:0] data_in,
    output [7:0] data_out
    );
    reg [7:0] data;
    assign data_out = data;
    always @(posedge Clk, posedge Reset) begin
        if(Reset) data <= Reset_value;
        else if(En) data <= data_in;
    end
endmodule

module Data_Memory(
    input Clk,
    input Reset,
    input [7:0] Data_in,
    input En,
    input [4:0] Address,
    output [7:0] Data_out
);
    wire [7:0] memory [31:0];
    
    assign Data_out = memory[Address];
    generate
        genvar i;
        for(i = 0; i < 32; i = i + 1) begin: dataGen
            data_mem u(
                .Clk(Clk),
                .Reset(Reset),
                .Reset_value((i == 8'h1B) ? 8'hFF : (i == 8'h1C ? 8'hAA: 8'h00)), 
                .En(Address == i && En),
                .data_in(Data_in),
                .data_out(memory[i])
                );
        end
    endgenerate
    
endmodule

module fifo(
    input CPU_Clk,
    input Reset,
    input [7:0] data_in,
    input WR,
    input RD,
    input [4:0] PC,
    output [7:0] data_out,
    output full,
    output empty
    );
    parameter LENGTH = 32;
    reg [$clog2(LENGTH) - 1 : 0] wr_ptr;
    reg [$clog2(LENGTH) - 1 : 0] wr_ptr_next;
    wire [7:0] memory [LENGTH - 1:0];
    assign full = &wr_ptr;
    assign empty = |wr_ptr;
    assign data_out = memory[PC];
    always @(*) begin
        wr_ptr_next = wr_ptr + 1'b1;
    end
    generate
        genvar i;
        for(i = 0; i < 32; i = i + 1) begin: insGen
            data_mem u(
                .Clk(CPU_Clk),
                .Reset(Reset),
                .En(wr_ptr == i && WR && !full),
                .data_in(data_in),
                .data_out(memory[i])
                );
        end
    endgenerate
    always @(posedge CPU_Clk, posedge Reset) begin
        if(Reset) begin
            wr_ptr <= 5'd0;
        end else if(WR && !full) begin
            wr_ptr <= wr_ptr_next;
        end
    end
endmodule

module ADDER(
    input [7:0] inA,
    input [7:0] inB,
    output [7:0] total
    );
    Adder_8bit ADD(inA, inB, 1'b0, total, );
endmodule

module Adder_8bit(input [7:0] A, B, input Cin, output [7:0] S, output Cout);
  wire Cout1;
  Adder_4bit ADD1 (A[3:0], B[3:0], Cin, S[3:0], Cout1);
  Adder_4bit ADD2 (A[7:4], B[7:4], Cout1, S[7:4], Cout);
endmodule

module Adder_4bit(input [3:0] A, B, input C0, output [3:0] S, output C4);
  wire C1, C2, C3;
  Full_adder FA0 (S[0], C1, A[0], B[0], C0);
  Full_adder FA1 (S[1], C2, A[1], B[1], C1);
  Full_adder FA2 (S[2], C3, A[2], B[2], C2);
  Full_adder FA3 (S[3], C4, A[3], B[3], C3);
endmodule

module Full_adder(output S, C1, input A, B, C0);
  wire w1, w2, w3;

  Half_Adder HA1 (w1, w2, A, B);
  Half_Adder HA2 (S, w3, w1, C0);

  // Directly use the OR operation for C1
  assign C1 = w2 | w3;
endmodule

module Half_Adder(output S, C, input A, B);
  xor G1 (S, A, B);
  and G2 (C, A, B);
endmodule

module Instruction_Memory(
    input Clk,
    input Reset,
    input [7:0] mem_ins,
    output reg [2:0] Opcode,
    output reg [4:0] Address
    );
    reg [2:0] opcode_next;
    reg [4:0] address_next;

    always @(*) begin
        opcode_next = mem_ins[7:5];
        address_next = mem_ins[4:0];
    end

    always @(posedge Clk or posedge Reset) begin
        if (Reset) begin
            Opcode <= 3'b0;      
            Address <= 5'b0;    
        end else begin
            Opcode <= opcode_next;
            Address <= address_next;
        end
    end
endmodule

module posedge_detection(
    input CPU_Clk,
    input signal_in,
    output reg signal_out
    );
    reg signal_in_next;
    always @(*) begin
        signal_out = signal_in && ~signal_in_next;
    end
    always @(posedge CPU_Clk) begin
        signal_in_next <= signal_in;
    end
endmodule

module UART(
    input Clk,
    input RX,
    input Load,
    input [4:0] PC,
    //output reg [255:0] memory_ins,
    output [7:0] data_out,
    output FE
    );
    parameter Baudrate = 2603;
    wire    [12:0] count_clk_next;
    reg     [12:0] count_clk;
    wire    Reset;
    reg     [3:0] i;
    wire    [3:0] i_next;
    reg     [9:0] buffer;
    reg    [9:0] buffer_next;
    wire    tmp_a;
    wire    En_1;
    wire    En_2;
    reg     [4:0] address;
    wire    [4:0] address_next;
//    reg     [7:0] memory_ins[31:0];
//    reg     [7:0] memory_ins_next [31:0];
    
    posedge_detection a(
        .CPU_Clk(Clk),
        .signal_in(Load),
        .signal_out(Reset)
    );

    assign FE = (i == 4'd9) & ~RX;
//assign FE = En_2;
    //count_clk        
    assign count_clk_next = (count_clk == Baudrate - 1'b1) ? 13'd0 : (count_clk + 13'd1);
    always @(posedge Clk, posedge Reset) begin
        if(Reset) count_clk <= 13'd0;
        else count_clk <= count_clk_next;
    end
    
    //buffer and i
    assign i_next = (i == 4'd9) ? 4'd0 : ((RX & i == 4'd0) ? 4'd0 : (i + 4'd1));
    assign En_1 = Load & (count_clk == Baudrate - 1'b1);
    
    always @(*) begin
        buffer_next = buffer;
        buffer_next[i] = RX;
    end
    always @(posedge Clk, posedge Reset) begin
        if(Reset) begin
            buffer <= 10'd0;
            i <= 4'd0;
        end else if(En_1) begin
            buffer <= buffer_next;
            i <= i_next;
        end
    end
    
    //Insmem and address
    assign En_2 = RX & (i == 4'd9) & (count_clk == Baudrate - 1'b1);
        fifo #(.LENGTH(32)) b(
        .CPU_Clk(Clk),
        .Reset(Reset),
        .data_in(buffer[8:1]),
        .WR(En_2),
        .PC(PC),
        .data_out(data_out),
        .full(),
        .empty()
    );
//    assign data_out = buffer[8:1];
endmodule

module UART_tb;

    // Testbench Signals
    reg Clk;
    reg RX;
    reg Load;
    wire [7:0] data_out;
    wire FE;
    wire [12:0]in;
    // Instantiate UART module
    UART #(.Baudrate(24)) uut (
        .Clk(Clk),
        .RX(RX),
        .Load(Load),
        .data_out(data_out),
        .FE(FE),
        .index(in)
    );

    // Clock generation (250MHz clock -> 4ns period)
    initial begin
        Clk = 0;
        forever #2 Clk = ~Clk; // 4ns clock period
    end

    // Task to send a byte via UART
    task send_uart_byte;
        input [7:0] data;
        integer i;
        begin
            // Start bit (low)
            RX = 0;
            #(24 * 4); // Baudrate duration
            
            // Data bits (LSB first)
            for (i = 0; i < 8; i = i + 1) begin
                RX = data[i];
                #(24 * 4); // Baudrate duration
            end
            
            // Stop bit (high)
            RX = 1;
            #(24 * 4); // Baudrate duration
        end
    endtask

    // Test Sequence
    initial begin
        // Initialize signals
        RX = 1; // Idle state (UART line high)
        Load = 0;

        // Wait for system to stabilize
        #12;

        // Apply Load signal (reset)
        Load = 1;

        // Wait for system ready
        #4;

        // Send valid bytes
        send_uart_byte(8'h55); // Send 0x55 (binary: 01010101)
        send_uart_byte(8'hA3); // Send 0xA3 (binary: 10100011)
        send_uart_byte(8'hFF); // Send 0xFF (binary: 11111111)
        send_uart_byte(8'h00); // Send 0x00 (binary: 00000000)

        // Wait for processing
        #200000;

        // Display results
        $display("Memory contents: %h", data_out);
        $display("Frame Error (FE): %b", FE);

        // End simulation
        $finish;
    end

    // Monitor signals
    initial begin
        $monitor("Time = %0dns | RX = %b | Load = %b | FE = %b | data_out = %h",
                 $time, RX, Load, FE, data_out);
    end

endmodule

module CPU_tb;
    // Testbench Signals
    reg Clk;
    reg RX;
    reg Load;
    reg Reset;
    wire FE;
    wire [7:0] Instruction;
    wire [7:0] Acc;
    wire [7:0] Mem;
    wire [4:0] Program_counter;
    // Instantiate CPU module
CPU #(.Baudrate(24)) uut(
    .Clk(Clk),
    .Load(Load),
    .RX(RX),
    .Reset(Reset),
    .FE(FE),
    .Instruction(Instruction),
    .Acc(Acc),
    .Mem(Mem),
    .Program_counter(Program_counter)
    );
    // Clock generation (250MHz clock -> 4ns period)
    initial begin
        Clk = 0;
        forever #2 Clk = ~Clk; // 4ns clock period
    end

    // Task to send a byte via UART
    task send_uart_byte;
        input [7:0] data;
        integer i;
        begin
            // Start bit (low)
            RX = 0;
            #(24 * 4); // Baudrate duration
            
            // Data bits (LSB first)
            for (i = 0; i < 8; i = i + 1) begin
                RX = data[i];
                #(24 * 4); // Baudrate duration
            end
            
            // Stop bit (high)
            RX = 1;
            #(24 * 4); // Baudrate duration
        end
    endtask

    // Test Sequence
    initial begin
        // Initialize signals
        Reset = 1;
        RX = 1; // Idle state (UART line high)
        Load = 0;

        // Wait for system to stabilize
        #12;

        // Apply Load signal (reset)
        Load = 1;
        Reset = 0;
        // Wait for system ready
        #4;

        // Send valid bytes
    send_uart_byte(8'b111_11110);     //  00   BEGIN:   JMP TST_JMP
    send_uart_byte(8'b000_00000);     //  01            HLT        
    send_uart_byte(8'b000_00000);     //  02            HLT       
    send_uart_byte(8'b101_11010);     //  03   JMP_OK:  LDA DATA_1
    send_uart_byte(8'b001_00000);     //  04            SKZ
    send_uart_byte(8'b000_00000);     //  05            HLT        
    send_uart_byte(8'b101_11011);     //  06            LDA DATA_2
    send_uart_byte(8'b001_00000);     //  07            SKZ
    send_uart_byte(8'b111_01010);     //  08            JMP SKZ_OK
    send_uart_byte(8'b000_00000);     //  09            HLT        
    send_uart_byte(8'b110_11100);     //  0A   SKZ_OK:  STO TEMP   
    send_uart_byte(8'b101_11010);     //  0B            LDA DATA_1
    send_uart_byte(8'b110_11100);     //  0C            STO TEMP   
    send_uart_byte(8'b101_11100);     //  0D            LDA TEMP
    send_uart_byte(8'b001_00000);     //  0E            SKZ        
    send_uart_byte(8'b000_00000);     //  0F            HLT        
    send_uart_byte(8'b100_11011);     //  10            XOR DATA_2
    send_uart_byte(8'b001_00000);     //  11            SKZ        
    send_uart_byte(8'b111_10100);     //  12            JMP XOR_OK
    send_uart_byte(8'b000_00000);     //  13            HLT        
    send_uart_byte(8'b100_11011);     //  14   XOR_OK:  XOR DATA_2
    send_uart_byte(8'b001_00000);     //  15            SKZ
    send_uart_byte(8'b000_00000);     //  16            HLT        
    send_uart_byte(8'b000_00000);     //  17   END:     HLT        
    send_uart_byte(8'b111_00000);     //  18            JMP BEGIN  
    send_uart_byte(8'b000_00000);     //  19            HLT
    send_uart_byte(8'b000_00000);     //  1A            HLT
    send_uart_byte(8'b000_00000);     //  1B            HLT
    send_uart_byte(8'b000_00000);     //  1C            HLT
    send_uart_byte(8'b000_00000);     //  1D            HLT
    send_uart_byte(8'b111_00011);     //  1E   TST_JMP: JMP JMP_OK
    send_uart_byte(8'b000_00000);     //  1F            HLT
        
        #4 Load = 0;
        // Wait for processing
        #200;
        // Display results
        $display("Instruction contents: %h", Instruction);
        $display("Accumulator contents: %h", Acc);
        $display("Memory[address] contents: %h", Mem);
        $display("Frame Error (FE): %b", FE);

        // End simulation
        $finish;
    end

    // Monitor signals
    initial begin
        $monitor("Time = %0dns | RX = %b | Load = %b | Instruction = %b | FE = %b | Accumulator = %h | Memory[address] = %h",
                 $time, RX, Load, Instruction, FE, Acc, Mem);
    end

endmodule

