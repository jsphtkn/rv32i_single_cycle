`timescale 1ns/1ps
module tb_rv32i;

    reg clk;
    reg rst;

    wire [31:0] data_o;
    //wire update_o;
    wire [31:0] pc_o;
    wire [31:0] instr_o;
    wire [4:0] reg_addr_o;
    wire [31:0] reg_data_o;
    wire [31:0] mem_addr_o;
    wire [31:0] mem_data_o;
    wire mem_wrt_o;
    wire mem_read_o;
    wire UNRECOGNISED_OPCODE;

    // Instantiate your CPU
    rv32i_top cpu0(
        .clk_i(clk),
        .rst_ni(rst),
        .data_o(data_o),
        //.update_o(update_o),
        .pc_o(pc_o),
        .instr_o(instr_o),
        .reg_addr_o(reg_addr_o),
        .reg_data_o(reg_data_o),
        .mem_addr_o(mem_addr_o),
        .mem_data_o(mem_data_o),
        .mem_wrt_o(mem_wrt_o),
        .mem_read_o(mem_read_o),
        .UNRECOGNISED_OPCODE(UNRECOGNISED_OPCODE)
    );

    // Clock generation
    initial clk = 0;
    always #5 clk = ~clk;  // 100MHz

    // Test sequence
    initial begin
        rst = 0;
        #10;
        rst = 1;

        // Run for some cycles
        repeat (20) @(posedge clk);

        $display("Final PC = %h", pc_o);
        $display("Final x1 = %h, x2 = %h, x3 = %h, x4 = %h", cpu0.dp0.rf0.mem[1], cpu0.dp0.rf0.mem[2], cpu0.dp0.rf0.mem[3], cpu0.dp0.rf0.mem[4]);
        $display("Memory at addr 1 = %h", cpu0.dp0.data_mem0.mem[1]);

        $stop;
    end

endmodule
