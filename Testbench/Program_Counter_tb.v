module Program_counter_tb(

    );
        // Inputs
    reg clock;
    reg reset;
    reg [4:0] addr;
    reg [2:0] Opcode;
    reg SKZ_cmp;
    reg Load_in;
    reg En_cpu_in;

    // Outputs
    wire [4:0] Program_counter;
    wire [4:0] Address;

    // Instantiate the Unit Under Test (UUT)
    Program_counter uut (
        .clock(clock), 
        .reset(reset), 
        .addr(addr), 
        .Opcode(Opcode), 
        .SKZ_cmp(SKZ_cmp), 
        .Load_in(Load_in), 
        .En_cpu_in(En_cpu_in), 
        .Program_counter(Program_counter), 
        .Address(Address)
    );

    initial begin
        // Initialize Inputs
        clock = 0;
        reset = 0;
        addr = 5'd0;
        Opcode = 3'b000;
        SKZ_cmp = 0;
        Load_in = 0;
        En_cpu_in = 0;

        // Wait for global reset
        #20;
        
        // Test reset functionality
        reset = 1;
        #10;
        reset = 0;
        #10;

        // Test normal operation
        En_cpu_in = 1;
        addr = 5'd10;
        Opcode = 3'b111; // Jump to address
        #10;
        
        Opcode = 3'b001; // Skip if zero
        SKZ_cmp = 1;
        #10;
        
        Opcode = 3'b000; // No operation
        #10;
        
        // Test Load functionality
        Load_in = 1;
        #10;
        Load_in = 0;
        #10;

        // Test with different address
        addr = 5'd15;
        Opcode = 3'b111; // Jump to address
        #10;

        // Finish simulation
        $finish;
    end
    
    // Clock generation
    always #5 clock = ~clock;
endmodule