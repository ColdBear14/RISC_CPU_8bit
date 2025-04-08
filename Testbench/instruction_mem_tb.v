`timescale 1ns/1ps

module Instruction_Memory_tb;
	reg clock;
	reg reset;
	reg [7:0] mem_ins;
	wire [2:0] Opcode_out;
	wire [4:0] Address_out;
	
	Instruction_Memory dut (
		.clock(clock),
		.reset(reset),
		.mem_ins(mem_ins),
		.Opcode_out(Opcode_out),
		.Address_out(Address_out)
	);
	
	always #5 clock = ~clock;
	
	reg [7:0] instruction_mem [0:31];
	
	integer i;
	
	initial begin
		clock = 0;
		reset = 0;
		mem_ins = 8'b0;
		
		instruction_mem[8'h00] = 8'b111_11110; // JMP TST_JMP
        instruction_mem[8'h01] = 8'b000_00000; // HLT
        instruction_mem[8'h02] = 8'b000_00000; // HLT
        instruction_mem[8'h03] = 8'b101_11010; // LDA DATA_1
        instruction_mem[8'h04] = 8'b001_00000; // SKZ
        instruction_mem[8'h05] = 8'b000_00000; // HLT
        instruction_mem[8'h06] = 8'b101_11011; // LDA DATA_2
        instruction_mem[8'h07] = 8'b001_00000; // SKZ
        instruction_mem[8'h08] = 8'b111_01010; // JMP SKZ_OK
        instruction_mem[8'h09] = 8'b000_00000; // HLT
        instruction_mem[8'h0A] = 8'b110_11100; // STO TEMP
        instruction_mem[8'h0B] = 8'b101_11010; // LDA DATA_1
        instruction_mem[8'h0C] = 8'b110_11100; // STO TEMP
        instruction_mem[8'h0D] = 8'b101_11100; // LDA TEMP
        instruction_mem[8'h0E] = 8'b001_00000; // SKZ
        instruction_mem[8'h0F] = 8'b000_00000; // HLT
        instruction_mem[8'h10] = 8'b100_11011; // XOR DATA_2
        instruction_mem[8'h11] = 8'b001_00000; // SKZ
        instruction_mem[8'h12] = 8'b111_10100; // JMP XOR_OK
        instruction_mem[8'h13] = 8'b000_00000; // HLT
        instruction_mem[8'h14] = 8'b100_11011; // XOR DATA_2
        instruction_mem[8'h15] = 8'b001_00000; // SKZ
        instruction_mem[8'h16] = 8'b000_00000; // HLT
        instruction_mem[8'h17] = 8'b000_00000; // HLT
        instruction_mem[8'h18] = 8'b111_00000; // JMP BEGIN
        instruction_mem[8'h1A] = 8'b00000000;  // DATA_1
        instruction_mem[8'h1B] = 8'b11111111;  // DATA_2
        instruction_mem[8'h1C] = 8'b10101010;  // TEMP
        instruction_mem[8'h1E] = 8'b111_00011; // TST_JMP: JMP JMP_OK
        instruction_mem[8'h1F] = 8'b000_00000; // HLT
		
		#10;
		reset = 0;
		
		for (i = 0; i <= 8'h1F; i = i + 1) begin
			mem_ins = instruction_mem[i];
			$display("Time: %0t | Addr: %02h | mem_ins: %b | Opcode: %b | Address: %b", $time, i, mem_ins, Opcode_out, Address_out);
		end
		
		$finish
	end
endmodule