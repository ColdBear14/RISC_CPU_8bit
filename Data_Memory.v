module Data_Memory(
    input clock,
    input reset,
    input [7:0] Data_in,
    input En,
    input [4:0] Address,
    output [7:0] Data_out
);
    reg [7:0] memory [31:0]; // Memory array with 32 elements, each 8 bits wide

    // Assign the output to the memory at the given address
    assign Data_out = memory[Address];

    // Memory update logic
    always @(posedge clock or posedge reset) begin
        integer i;
        if (reset) begin
            // Reset all memory elements
            for (i = 0; i < 32; i = i + 1) begin
                if (i == 8'h1B) memory[i] <= 8'hFF; // Special reset value for address 0x1B
                else if (i == 8'h1C) memory[i] <= 8'hAA; // Special reset value for address 0x1C
                else memory[i] <= 8'h00; // Default reset value
            end
        end else if (En) begin
            // Write data to the specified address
            memory[Address] <= Data_in;
        end
    end
endmodule