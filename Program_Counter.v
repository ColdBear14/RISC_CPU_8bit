module Program_Counter(
    input clock,
    input reset,
    input [4:0] addr,
    input [2:0] Opcode,
    input   SKZ_cmp,
    input   Load_in,
    input   En_cpu_in,
    output [4:0] Program_counter,
    output reg [4:0] Address
    );
    
    wire [7:0] pc_jmp;
    reg [4:0] PC;

    assign pc_jmp = (Opcode == 3'b111) ? addr : 
                    (Opcode == 3'b001 && SKZ_cmp)? (PC + 5'd2) : 
                    (Opcode == 3'b000) ? PC : 
                    (PC + 5'd1);
    //Choose what pc next in case Load
    assign Program_counter = ~Load_in ? pc_jmp : 5'd0;   
    

    always @(posedge clock, posedge reset) begin
        if(reset) begin
            PC <= 5'd0;
            Address <= 5'd0; 
        end
        else if(En_cpu_in) begin 
            PC <= Program_counter;
            Address <= addr;
        end
    end

endmodule