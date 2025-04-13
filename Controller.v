module Controller(
    input clock,
    input reset,
    input Load_in,
    input [2:0] Opcode,
    output En_acc,
    output En_mem,
    output En_cpu,
    output reg [2:0] ALU_OP
    );
    reg En_run;
    reg En_write_reg;
    reg En_write_mem;
    wire write_reg_next;
    wire write_mem_next;

    assign write_mem_next = (Opcode == 3'b110);
    assign write_reg_next = (Opcode == 3'b010 || Opcode == 3'b011 || Opcode == 3'b100 || Opcode == 3'b101);

    assign En_acc = ~Load_in && En_write_reg && En_run;
    assign En_mem = ~Load_in && En_write_mem && En_run;
    assign En_cpu = ~Load_in && | Opcode;

        
    always @(posedge clock or posedge reset) begin
        if (reset) begin
            En_write_reg <= 1'b0;
            En_write_mem <= 1'b0;
            ALU_OP <= 3'b000;
        end else if (En_cpu) begin
            En_write_reg <= write_reg_next;
            En_write_mem <= write_mem_next;
            ALU_OP <= Opcode;
        end
    end

    always @(posedge clock, posedge reset) begin
        if(reset) En_run <= 1'b0;
        else En_run <= En_cpu;
   end



endmodule