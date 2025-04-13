module Program_Counter(
    input clock,
    input reset,
    input [4:0] addr,
    input [2:0] Opcode,
    input SKZ_cmp,
    input Load_in,
    input En_cpu_in,
    output reg [4:0] Program_counter,
    output reg [4:0] Address
);

    reg [4:0] pc_next; // Temporary register for the next PC value

    always @(*) begin
        // Calculate the next PC value based on the Opcode and conditions
        if (Opcode == 3'b111) begin
            pc_next = addr; // JMP instruction
        end else if (Opcode == 3'b001 && SKZ_cmp) begin
            pc_next = Program_counter + 5'd2; // SKZ instruction
        end else if (Opcode == 3'b000) begin
            pc_next = Program_counter; // HLT instruction
        end else begin
            pc_next = Program_counter + 5'd1; // Default increment
        end
    end

    always @(posedge clock or posedge reset) begin
        if (reset) begin
            Program_counter <= 5'd0; // Reset PC to 0
            Address <= 5'd0;         // Reset Address to 0
        end else if (Load_in) begin
            Program_counter <= 5'd0; // Reset PC during load
        end else if (En_cpu_in) begin
            Program_counter <= pc_next; // Update PC with the calculated value
            Address <= addr;            // Update Address
        end
    end

endmodule