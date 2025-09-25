module rv32i_top #(
    parameter XLEN = 32,
    parameter DMemInitFile = "dmem.mem", 
    parameter IMemInitFile = "imem.mem",
    parameter PCInitAdress = 0
    )(
    input               clk_i,
    input               rst_ni,
    // input [XLEN-1:0]    addr_i,
    output [31:0]       data_o,
    //output              update_o,
    output [XLEN-1:0]   pc_o,
    output [31:0]       instr_o,
    output [XLEN-1:0]   reg_addr_o,
    output [31:0]       reg_data_o,
    output [XLEN-1:0]   mem_addr_o,
    output [31:0]       mem_data_o,
    output              mem_wrt_o,
    output              mem_read_o,
    output UNRECOGNISED_OPCODE
    );
    
    wire w_zero, w_RegWE, w_MemWE, w_ResultSrc, w_ALUSrc;
    wire [6:0] w_op, w_func7;
    wire [3:0] w_ALUControl;
    wire [2:0] w_func3, w_ImmSrc;
    wire [1:0] w_PCSrc, w_WriteBackSrc;
    wire [31:0] w_ALUResult;
    
    datapath #(
        .DEPTH(XLEN), 
        .INIT(PCInitAdress),
        .IMEM(IMemInitFile),
        .DMEM(DMemInitFile)
        ) dp0(
        .clk_i(clk_i),         
        .rst_ni(rst_ni),        
        .RegWE_i(w_RegWE),       
        .PCSrc_i(w_PCSrc),       
        .ImmSrc_i(w_ImmSrc),      
        .ALUSrc_i(w_ALUSrc),      
        .ALUControl_i(w_ALUControl),  
        .MemWE_i(w_MemWE),       
        .ResultSrc_i(w_ResultSrc),   
        .WriteBackSrc_i(w_WriteBackSrc),
        .zero_o(w_zero),        
        .ALUresult_o(w_ALUResult),   
        .op_o(w_op),          
        .func3_o(w_func3),       
        .func7_o(w_func7),
        .data_o(data_o),     
        //.update_o(update_o),   
        .pc_o(pc_o),       
        .instr_o(instr_o),    
        .reg_addr_o(reg_addr_o), 
        .reg_data_o(reg_data_o), 
        .mem_addr_o(mem_addr_o), 
        .mem_data_o(mem_data_o), 
        .mem_wrt_o(mem_wrt_o),  
        .mem_read_o(mem_read_o)        
        );
    
    control c0(
        .zero_i(w_zero),            
        .op_i(w_op),              
        .func3_i(w_func3),           
        .func7_i(w_func7),           
        .ALUresult_i(w_ALUResult),       
        .RegWE_o(w_RegWE),           
        .PCSrc_o(w_PCSrc),           
        .ImmSrc_o(w_ImmSrc),          
        .ALUSrc_o(w_ALUSrc),          
        .ALUControl_o(w_ALUControl),      
        .MemWE_o(w_MemWE),           
        .ResultSrc_o(w_ResultSrc),       
        .WriteBackSrc_o(w_WriteBackSrc),    
        .UNRECOGNISED_OPCODE(UNRECOGNISED_OPCODE) 
        );
    
endmodule

module control(
    input               zero_i,
    input [6:0]         op_i,
    input [2:0]         func3_i,
    input [6:0]         func7_i,
    input [31:0]        ALUresult_i,
    output reg          RegWE_o,
    output reg [1:0]    PCSrc_o,            // 00 -> PC =  PC + 4, 01 -> PC = PC + Imm, 10 -> PC = rs1 + Imm, 11 -> Trap
    output reg [2:0]    ImmSrc_o,           // 000=I, 001=S, 010=B, 011=U, 100=J, 101=Shift
    output reg          ALUSrc_o,
    output reg [3:0]    ALUControl_o,
    output reg          MemWE_o,
    output reg          ResultSrc_o,
    output reg [1:0]    WriteBackSrc_o,
    output reg          UNRECOGNISED_OPCODE
    
    );

    always @(*) begin
        // defaults
        RegWE_o <= 1'b0;
        PCSrc_o <= 2'b00;
        ImmSrc_o <= 3'b000;
        ALUSrc_o <= 1'b0;
        ALUControl_o <= 4'b1111;
        MemWE_o <= 1'b0;
        ResultSrc_o <= 1'b0;
        WriteBackSrc_o <= 2'b00;
        UNRECOGNISED_OPCODE <= 1'b0;
        case(op_i)
            
// ------------------- ARITHMETIC -----------------------------------------------            
            7'b011_0011: begin          // R Type Aritmetics
                UNRECOGNISED_OPCODE <= 1'b0;
                ALUSrc_o <= 1'b0;       // Use rs1 and rs2
                RegWE_o <= 1'b1;        // WE for rd 
                PCSrc_o <= 2'b00;       // PC + 4 
                MemWE_o <= 1'b0;        // No Load or Store
                ResultSrc_o <= 1'b0;    // Result is ALU output 
                WriteBackSrc_o <= 2'b00; // WB Result into Rd 
                ImmSrc_o <= 3'b000;
                case({func3_i, func7_i})
                    {3'b000, 7'b000_0000}: ALUControl_o <= 4'b0000; // ADD
                    {3'b000, 7'b010_0000}: ALUControl_o <= 4'b0001; // SUB
                    {3'b111, 7'b000_0000}: ALUControl_o <= 4'b0010; // AND
                    {3'b110, 7'b000_0000}: ALUControl_o <= 4'b0011; // OR
                    {3'b100, 7'b000_0000}: ALUControl_o <= 4'b0100; // XOR
                    {3'b001, 7'b000_0000}: ALUControl_o <= 4'b0101; // SLL
                    {3'b101, 7'b000_0000}: ALUControl_o <= 4'b0110; // SRL
                    {3'b101, 7'b010_0000}: ALUControl_o <= 4'b0111; // SRA
                    {3'b010, 7'b000_0000}: ALUControl_o <= 4'b1000; // SLT
                    {3'b011, 7'b000_0000}: ALUControl_o <= 4'b1001; // SLTU
                    default: ALUControl_o <= 4'b1111;               // ALU_i == ALU_o
                endcase
            end
            7'b001_0011: begin          // I Type Aritmetics
                UNRECOGNISED_OPCODE <= 1'b0;
                ALUSrc_o <= 1'b1;       // Use rs1 and Imm
                RegWE_o <= 1'b1;        // WE for rd 
                PCSrc_o <= 2'b00;       // PC + 4 
                MemWE_o <= 1'b0;        // No Load or Store
                ResultSrc_o <= 1'b0;    // Result is ALU output 
                WriteBackSrc_o <= 2'b00; // WB Result into Rd 
                case(func3_i)
                    3'b000: begin                       // ADDI 
                        ImmSrc_o <= 3'b000;             // I Type Imm Extend
                        ALUControl_o <= 4'b0000;        // ADD
                    end
                    3'b111: begin                       // ANDI
                        ImmSrc_o <= 3'b000;             // I Type Imm Extend
                        ALUControl_o <= 4'b0010;        // AND
                    end
                    3'b110: begin                       // ORI 
                        ImmSrc_o <= 3'b000;             // I Type Imm Extend
                        ALUControl_o <= 4'b0011;        // OR
                    end
                    3'b100: begin                       // XORI 
                        ImmSrc_o <= 3'b000;             // I Type Imm Extend
                        ALUControl_o <= 4'b0100;        // XOR
                    end
                    3'b001: begin                       // SLLI 
                        if(func7_i == 7'b000_0000) begin
                            ImmSrc_o <= 3'b101;         // SHIFT Type Imm Extend
                            ALUControl_o <= 4'b0101;    // SLL
                        end    
                    end
                    3'b101: begin                        
                        if(func7_i == 7'b000_0000) begin           // SRLI
                            ImmSrc_o <= 3'b101;         // SHIFT Type Imm Extend
                            ALUControl_o <= 4'b0110;    // SRL
                        end else if (func7_i == 7'b010_0000) begin // SRAI
                            ImmSrc_o <= 3'b101;         // SHIFT Type Imm Extend
                            ALUControl_o <= 4'b0111;    // SRA
                        end    
                    end
                    3'b010: begin                       // SLTI
                        ImmSrc_o <= 3'b000;             // I Type Imm Extend
                        ALUControl_o <= 4'b1000;        // SLT
                    end
                    3'b011: begin                       // SLTIU
                        ImmSrc_o <= 3'b000;             // I Type Imm Extend
                        ALUControl_o <= 4'b1001;        // SLTU
                    end                     
                endcase
            end
            
// ------------------- MEMORY -----------------------------------------------         
        7'b000_0011: begin                              // LOADs
            UNRECOGNISED_OPCODE <= 1'b0;
            PCSrc_o <= 2'b00;                           // PC_Next = PC + 4
            case(func3_i)                               // The difference between the Loads and Stores are done in the dmem unit
                3'b000: begin                           // LB
                   ImmSrc_o <= 3'b000;                  // I Type Imm Extend 
                   ALUSrc_o <= 1'b1;                    // Use rs1 and Imm
                   ALUControl_o <= 4'b0000;             // ADD
                   MemWE_o <= 1'b0;                     // Mem Read at adress: rs1 + Imm
                   ResultSrc_o <= 1'b1;                 // Result is MEM output 
                   WriteBackSrc_o <= 2'b00;              // WB Result into Rd
                   RegWE_o <= 1'b1;                     // WE for rd
                end
                3'b100: begin                           // LBU 
                   ImmSrc_o <= 3'b000;                  // I Type Imm Extend 
                   ALUSrc_o <= 1'b1;                    // Use rs1 and Imm
                   ALUControl_o <= 4'b0000;             // ADD
                   MemWE_o <= 1'b0;                     // Mem Read at adress: rs1 + Imm
                   ResultSrc_o <= 1'b1;                 // Result is MEM output 
                   WriteBackSrc_o <= 2'b00;              // WB Result into Rd
                   RegWE_o <= 1'b1;                     // WE for rd
                end
                3'b001: begin                           // LH
                   ImmSrc_o <= 3'b000;                  // I Type Imm Extend 
                   ALUSrc_o <= 1'b1;                    // Use rs1 and Imm
                   ALUControl_o <= 4'b0000;             // ADD
                   MemWE_o <= 1'b0;                     // Mem Read at adress: rs1 + Imm
                   ResultSrc_o <= 1'b1;                 // Result is MEM output 
                   WriteBackSrc_o <= 2'b00;              // WB Result into Rd
                   RegWE_o <= 1'b1;                     // WE for rd
                end
                3'b101: begin                           // LHU 
                   ImmSrc_o <= 3'b000;                  // I Type Imm Extend 
                   ALUSrc_o <= 1'b1;                    // Use rs1 and Imm
                   ALUControl_o <= 4'b0000;             // ADD
                   MemWE_o <= 1'b0;                     // Mem Read at adress: rs1 + Imm
                   ResultSrc_o <= 1'b1;                 // Result is MEM output 
                   WriteBackSrc_o <= 2'b00;              // WB Result into Rd
                   RegWE_o <= 1'b1;                     // WE for rd
                end
                3'b010: begin                           // LW 
                   ImmSrc_o <= 3'b000;                  // I Type Imm Extend 
                   ALUSrc_o <= 1'b1;                    // Use rs1 and Imm
                   ALUControl_o <= 4'b0000;             // ADD
                   MemWE_o <= 1'b0;                     // Mem Read at adress: rs1 + Imm
                   ResultSrc_o <= 1'b1;                 // Result is MEM output 
                   WriteBackSrc_o <= 2'b00;              // WB Result into Rd
                   RegWE_o <= 1'b1;                     // WE for rd
                end
            endcase
        end
        7'b010_0011: begin                              // STORES
            UNRECOGNISED_OPCODE <= 1'b0;
            PCSrc_o <= 2'b00;                           // PC_Next = PC + 4
            RegWE_o <= 1'b0;
            WriteBackSrc_o <= 2'b00;                     // WB Result into Rd
            ResultSrc_o <= 1'b1;                        // Result is MEM output DONT_CARE
            case(func3_i)
                3'b000: begin                           // SB
                    ImmSrc_o <= 3'b001;                 // S Type Imm Extend
                    ALUSrc_o <= 1'b1;                   // Use rs1 and Imm
                    ALUControl_o <= 4'b0000;            // ADD
                    MemWE_o <= 1'b1;                    // Mem Write RD2 at adress: rs1 + Imm
                end
                3'b001: begin                           // SH
                    ImmSrc_o <= 3'b001;                 // S Type Imm Extend
                    ALUSrc_o <= 1'b1;                   // Use rs1 and Imm
                    ALUControl_o <= 4'b0000;            // ADD
                    MemWE_o <= 1'b1;                    // Mem Write RD2 at adress: rs1 + Imm
                end
                3'b010: begin                           // SW
                    ImmSrc_o <= 3'b001;                 // S Type Imm Extend
                    ALUSrc_o <= 1'b1;                   // Use rs1 and Imm
                    ALUControl_o <= 4'b0000;            // ADD
                    MemWE_o <= 1'b1;                    // Mem Write RD2 at adress: rs1 + Imm
                end
            endcase
        end
        
// ------------------- CONTROL -----------------------------------------------         
        7'b110_0011: begin                              // B Type Controls
            UNRECOGNISED_OPCODE <= 1'b0;
            RegWE_o <= 1'b0;
            WriteBackSrc_o <= 2'b00;                     // WB Result into Rd
            MemWE_o <= 1'b0;                            // Mem Read DONT_CARE
            ResultSrc_o <= 1'b1;                        // Result is MEM output DONT_CARE
            case(func3_i)
                3'b000: begin                           // BEQ
                    ALUSrc_o <= 1'b0;                   // Use rs1 and rs2
                    ImmSrc_o <= 3'b010;                 // B Type Imm Extend
                    ALUControl_o <= 4'b0001;            // SUB
                    if(zero_i) PCSrc_o <= 2'b01;        // PC + Imm
                    else       PCSrc_o <= 2'b00;        // PC + 4
                end
                3'b001: begin                           // BNE
                    ALUSrc_o <= 1'b0;                   // Use rs1 and rs2
                    ImmSrc_o <= 3'b010;                 // B Type Imm Extend
                    ALUControl_o <= 4'b0001;            // SUB
                    if(zero_i) PCSrc_o <= 2'b00;        // PC + 4
                    else       PCSrc_o <= 2'b01;        // PC + Imm
                end
                3'b101: begin                           // BGE
                    ALUSrc_o <= 1'b0;                   // Use rs1 and rs2
                    ImmSrc_o <= 3'b010;                 // B Type Imm Extend
                    ALUControl_o <= 4'b1000;            // SLT
                    if(ALUresult_i == 32'h0000_0000) PCSrc_o <= 2'b01;        // PC + Imm
                    else                             PCSrc_o <= 2'b00;        // PC + 4
                end
                3'b111: begin                           // BGEU
                    ALUSrc_o <= 1'b0;                   // Use rs1 and rs2
                    ImmSrc_o <= 3'b010;                 // B Type Imm Extend
                    ALUControl_o <= 4'b1001;            // SLTU
                    if(ALUresult_i == 32'h0000_0000) PCSrc_o <= 2'b01;        // PC + Imm
                    else                             PCSrc_o <= 2'b00;        // PC + 4
                end
                3'b100: begin                           // BLT
                    ALUSrc_o <= 1'b0;                   // Use rs1 and rs2
                    ImmSrc_o <= 3'b010;                 // B Type Imm Extend
                    ALUControl_o <= 4'b1000;            // SLT
                    if(ALUresult_i == 32'h0000_0001) PCSrc_o <= 2'b01;        // PC + Imm
                    else                             PCSrc_o <= 2'b00;        // PC + 4
                end
                3'b110: begin                           // BLTU
                    ALUSrc_o <= 1'b0;                   // Use rs1 and rs2
                    ImmSrc_o <= 3'b010;                 // B Type Imm Extend
                    ALUControl_o <= 4'b1001;            // SLTU
                    if(ALUresult_i == 32'h0000_0001) PCSrc_o <= 2'b01;        // PC + Imm
                    else                             PCSrc_o <= 2'b00;        // PC + 4
                end
            endcase 
        end
        
        7'b110_1111: begin                              // JAL
            UNRECOGNISED_OPCODE <= 1'b0;
            WriteBackSrc_o <= 2'b10;                    // Rd = PC + 4    
            RegWE_o <= 1'b1;                            // WE for rd  
            ImmSrc_o <= 3'b100;                         // J Type Imm Extend
            PCSrc_o <= 2'b01;                           // PC_Next = PC + Imm
            // --------------------------------------------------------------------------------
            ALUControl_o <= 4'b0000;                    // ADD DONT_CARE
            MemWE_o <= 1'b0;                            // MEM Read DONT_CARE
            ResultSrc_o <= 1'b0;                        // Result is ALU output DONT_CARE
            ALUSrc_o <= 1'b0;                           // Use rs1 and rs2    
        end
        7'b110_0111: begin
            UNRECOGNISED_OPCODE <= 1'b0;
            MemWE_o <= 1'b0;                            // MEM Read DONT_CARE
            ResultSrc_o <= 1'b0;                        // Result is ALU output DONT_CARE
            if(func3_i == 3'b000) begin                 // JALR
                WriteBackSrc_o <= 2'b10;                // Rd = PC + 4    
                RegWE_o <= 1'b1;                        // WE for rd  
                ImmSrc_o <= 3'b000;                     // I Type Imm Extend
                ALUSrc_o <= 1'b1;                       // Use rs1 and Imm
                ALUControl_o <= 4'b0000;                // ADD
                PCSrc_o <= 2'b10;                       // PC_Next = PC + result
            end
        end
        
// ------------------- OTHER -----------------------------------------------  
        7'b001_0111: begin                              // AUIPC
            UNRECOGNISED_OPCODE <= 1'b0;
            WriteBackSrc_o <= 2'b01;                    // Rd = PC + Imm
            RegWE_o <= 1'b1;                            // WE for rd
            ImmSrc_o <= 3'b011;                         // U Type Imm Extend
            PCSrc_o <= 2'b00;                           // PC + 4
            // ----------------------------------------------------------------------------
            ALUSrc_o <= 1'b0;                           // Use rs1 and rs2 DONT_CARE
            ALUControl_o <= 4'b0000;                    // ADD DONT_CARE
            MemWE_o <= 1'b0;                            // MEM Read DONT_CARE
            ResultSrc_o <= 1'b0;                        // Result is ALU output DONT_CARE
        end
        7'b011_0111: begin                              // LUI
            UNRECOGNISED_OPCODE <= 1'b0;
            WriteBackSrc_o <= 2'b00;                    // Rd = result
            RegWE_o <= 1'b1;                            // WE for rd
            ImmSrc_o <= 3'b011;                         // U Type Imm Extend
            ALUSrc_o <= 1'b1;                           // Use rs1 and Imm
            ALUControl_o <= 4'b1111;                    // ALUResult = SrcB for LUI
            PCSrc_o <= 2'b00;                           // PC + 4 
            // ----------------------------------------------------------------------------
            MemWE_o <= 1'b0;                            // MEM Read DONT_CARE
            ResultSrc_o <= 1'b0;                        // Result is ALU output DONT_CARE
        end 
        endcase        
    end
endmodule

module datapath #(
    parameter DEPTH = 32,
    parameter INIT = 0,
    parameter IMEM = "imem.mem",
    parameter DMEM = "dmem.mem"    
    )(
    input           clk_i,
    input           rst_ni,
    input           RegWE_i,
    input [1:0]     PCSrc_i,
    input [2:0]     ImmSrc_i,
    input           ALUSrc_i,
    input [3:0]     ALUControl_i,
    input           MemWE_i,
    input           ResultSrc_i,
    input [1:0]     WriteBackSrc_i,
    output          zero_o, 
    output [31:0]   ALUresult_o,      
    output [6:0]    op_o,   
    output [2:0]    func3_o,
    output [6:0]    func7_o,
    
    // Debug signals
    output [32-1:0]      data_o,
    //output               update_o,
    output [DEPTH-1:0]   pc_o,
    output [31:0]        instr_o,
    output [5-1:0]       reg_addr_o,
    output [32-1:0]      reg_data_o,
    output [32-1:0]      mem_addr_o,
    output [32-1:0]      mem_data_o,
    output               mem_wrt_o,
    output               mem_read_o  
    );

wire [DEPTH-1:0] w_pc, w_pc4, w_pcimm, w_pcn;
wire [31:0] w_ins, w_ImmExt, w_result, w_result_1, w_rd1, w_rd2, w_rd2_srcb, w_alu_result, w_rd_o;



// Control inputs
assign op_o = w_ins[6:0];
assign func3_o = w_ins[14:12];
assign func7_o = w_ins[31:25]; 

// Debug outputs
assign data_o = w_result;
assign ALUresult_o = w_alu_result;
assign pc_o = w_pc;
assign instr_o = w_ins;
assign reg_addr_o = w_ins[19:15];
assign reg_data_o = w_rd1;
assign mem_addr_o = w_alu_result;
assign mem_data_o = w_rd_o;
assign mem_wrt_o = MemWE_i;
assign mem_read_o = ~MemWE_i;

flopr #(.DEPTH(DEPTH), .INIT(INIT)) pc0(
    .clk_i(clk_i),             
    .rst_ni(rst_ni),            
    .pc_next(w_pcn),
    .pc(w_pc)
    );

imem #(.DEPTH(DEPTH), .IMEM(IMEM)) imem0(
    .a(w_pc),
    .rd(w_ins)
    );
    
adder #(.DEPTH(DEPTH)) b_adder(
    .a_i(w_pc),
    .b_i(w_ImmExt),
    .c_o(w_pcimm)
    );
    
adder #(.DEPTH(DEPTH)) pc4_adder(
    .a_i(w_pc),
    .b_i({{(DEPTH-3){1'b0}}, 3'b100}),
    .c_o(w_pc4)
    );
        
rfile rf0(
    .clk_i(clk_i),           
    .rst_ni(rst_ni),          
    .we_i(RegWE_i),            
    .a1_i(w_ins[19:15]), 
    .a2_i(w_ins[24:20]), 
    .a3_i(w_ins[11:7]),
    .wd3_i(w_result_1),           
    .rd1_o(w_rd1), 
    .rd2_o(w_rd2)     
    );   

extend ImmExt0(
    .ImmSrc_i(ImmSrc_i),
    .Imm_Ins_i(w_ins),
    .ImmExt_o(w_ImmExt)
    );

mux4 pc_mux0(.a0_i(w_pc4), .a1_i(w_pcimm), .a2_i(w_alu_result), .a3_i(32'b0),  .sel_i(PCSrc_i), .c_o(w_pcn));
mux2 alu_mux0(.a_i(w_rd2), .b_i(w_ImmExt), .sel_i(ALUSrc_i), .c_o(w_rd2_srcb));
mux2 result_mux0(.a_i(w_alu_result), .b_i(w_rd_o), .sel_i(ResultSrc_i), .c_o(w_result));
mux4 wb_mux0(.a0_i(w_result), .a1_i(w_pcimm), .a2_i(w_pc4), .a3_i(32'b0), .sel_i(WriteBackSrc_i), .c_o(w_result_1));

alu alu0(.a(w_rd1), .b(w_rd2_srcb), .aluCtrl(ALUControl_i), .result(w_alu_result), .zero(zero_o));

dmem #(.DEPTH(DEPTH), .DMEM(DMEM)) data_mem0(.clk_i(clk_i), .rst_ni(rst_ni), .we_i(MemWE_i), .addr_i(w_alu_result), .wd_i(w_rd2), .funct3(w_ins[14:12]), .rd_o(w_rd_o));          

endmodule



module imem #(parameter DEPTH = 32, parameter IMEM = "imem.mem")(
    input   [DEPTH-1:0] a,
    output  [31:0] rd
    );
    
    reg [31:0] mem [0:DEPTH - 1];
    initial $readmemh(IMEM, mem);
    assign rd = mem[a[DEPTH-1:2]];               // word aligned
endmodule

module dmem #(
    parameter DEPTH = 1024,          // number of 32-bit words
    parameter DMEM  = "dmem.mem"     // optional init file (hex words)
) (
    input           clk_i,
    input           rst_ni,          // unused for synthesis here (kept for interface)
    input           we_i,
    input  [31:0]   addr_i,          // byte address
    input  [31:0]   wd_i,            // write data (word-aligned value)
    input  [2:0]    funct3,          // load/store type
    output [31:0]   rd_o
);

    // address width to index DEPTH words
    localparam AW = (DEPTH > 1) ? $clog2(DEPTH) : 1;

    reg [31:0] mem [0:DEPTH-1];

    initial begin
        if (DMEM != "") $readmemh(DMEM, mem);
    end

    // word index and offsets
    wire [AW-1:0] waddr      = addr_i[AW+1:2]; // word index (addr >> 2) limited to DEPTH
    wire [1:0]    byte_off   = addr_i[1:0];     // offset inside the 32-bit word (0..3)
    integer i;

    // --- Write (synchronous) ---
    // RISC-V little-endian semantics:
    //  - SB: store wd_i[7:0] to selected byte
    //  - SH: store wd_i[15:0] to bytes {1:0} or {3:2} depending on byte_off (0 or 2)
    //  - SW: store whole word
    always @(posedge clk_i) begin
        if(!rst_ni) begin
            for(i=0; i<DEPTH; i = i + 1) begin
                mem[i] <= 32'h0000_0000;
            end
            
        end else if (we_i) begin
            case (funct3)
                3'b000: begin // SB
                    case (byte_off)
                        2'b00: mem[waddr][ 7:0]   <= wd_i[7:0];
                        2'b01: mem[waddr][15:8]   <= wd_i[7:0];
                        2'b10: mem[waddr][23:16]  <= wd_i[7:0];
                        2'b11: mem[waddr][31:24]  <= wd_i[7:0];
                    endcase
                end

                3'b001: begin // SH (halfword). Spec requires halfword-aligned (addr[0]==0).
                    if (byte_off == 2'b00) begin
                        mem[waddr][15:0] <= wd_i[15:0]; // bytes 0..1
                    end else begin
                        mem[waddr][31:16] <= wd_i[15:0]; // bytes 2..3
                    end
                end

                3'b010: begin // SW
                    mem[waddr] <= wd_i;
                end

                default: begin
                    // unsupported store type: do nothing
                end
            endcase
        end
    end

    // --- Read (combinational) ---
    wire [31:0] word = mem[waddr];

    reg [31:0] load_ext;
    always @(*) begin
        case (funct3)
            // LB : sign-extend selected byte
            3'b000: begin
                case (byte_off)
                    2'b00: load_ext = {{24{word[7]}},  word[7:0]};
                    2'b01: load_ext = {{24{word[15]}}, word[15:8]};
                    2'b10: load_ext = {{24{word[23]}}, word[23:16]};
                    2'b11: load_ext = {{24{word[31]}}, word[31:24]};
                endcase
            end

            // LH : sign-extend selected halfword (addr[0]==0 expected)
            3'b001: begin
                if (byte_off == 2'b00)
                    load_ext = {{16{word[15]}}, word[15:0]};
                else
                    load_ext = {{16{word[31]}}, word[31:16]};
            end

            // LW : full word
            3'b010: load_ext = word;

            // LBU : zero-extend selected byte
            3'b100: begin
                case (byte_off)
                    2'b00: load_ext = {24'b0, word[7:0]};
                    2'b01: load_ext = {24'b0, word[15:8]};
                    2'b10: load_ext = {24'b0, word[23:16]};
                    2'b11: load_ext = {24'b0, word[31:24]};
                endcase
            end

            // LHU : zero-extend selected halfword
            3'b101: begin
                if (byte_off == 2'b00)
                    load_ext = {16'b0, word[15:0]};
                else
                    load_ext = {16'b0, word[31:16]};
            end

            default: load_ext = 32'b0;
        endcase
    end

    assign rd_o = load_ext;

endmodule



// Note that in greater designs where an OS communication with CSR unit is required
// the Register File needs a initial mem file to define each register in spesific
module rfile(
    input           clk_i,
    input           rst_ni,
    input           we_i,
    input   [4:0]   a1_i, a2_i, a3_i,
    input   [31:0]  wd3_i,
    output  [31:0]  rd1_o, rd2_o
    );
    
    reg [31:0] mem [0:31];
    integer i;
    
    always @(posedge clk_i) begin
        if(!rst_ni) begin
            for(i=0; i<32; i= i+1) begin
                mem[i] <= 32'h0000_0000;
            end
        end else begin
           if(we_i && (a3_i != 0)) begin
               mem[a3_i] <= wd3_i;  
           end     
        end
    end
    
   assign rd1_o = (a1_i != 0) ?  mem[a1_i] : 32'h0000_0000;
   assign rd2_o = (a2_i != 0) ?  mem[a2_i] : 32'h0000_0000;
endmodule


module alu(
    input [31:0] a,
    input [31:0] b,
    input [3:0] aluCtrl,
    output reg [31:0] result,
    output zero
    );
     
    always @(*) begin
        case(aluCtrl)
            4'b0000: result <= a + b;        // ADD
            4'b0001: result <= a - b;        // SUB
            4'b0010: result <= a & b;        // AND
            4'b0011: result <= a | b;        // OR
            4'b0100: result <= a ^ b;        // XOR
            4'b0101: result <= a << b[4:0];  // SLL
            4'b0110: result <= a >> b[4:0];  // SLR
            4'b0111: result <= $signed(a) >>> b[4:0];                        // SRA
            4'b1000: result <= ($signed(a) < $signed(b)) ? 32'h0000_0001 : 32'h0000_0000; // SLT
            4'b1001: result <= (a < b)              ? 32'h0000_0001 : 32'h0000_0000; // SLTU


            
            4'b1111: result <= b;                                            // Do Nothing --> LUI 
            default: result <= 32'h0000_0000;
        endcase
    end
    
    assign zero = (result == 32'h0000_0000) ? 1'b1 : 1'b0;
endmodule

module mux2(
    input [31:0] a_i,b_i,
    input sel_i,
    output [31:0] c_o
    );

assign c_o = sel_i ? b_i : a_i;

endmodule

module mux4(
    input [31:0] a0_i, a1_i, a2_i, a3_i,
    input [1:0] sel_i,
    output reg [31:0] c_o
    );

    always @(*) begin
        case(sel_i)
            2'b00: c_o <= a0_i;
            2'b01: c_o <= a1_i;
            2'b10: c_o <= a2_i;
            2'b11: c_o <= a3_i;
        endcase
    end

endmodule

module extend(
    input  [31:0] Imm_Ins_i,
    input  [2:0]  ImmSrc_i,    // 000=I, 001=S, 010=B, 011=U, 100=J, 101=Shift
    output reg [31:0] ImmExt_o
);

    always @(*) begin
        case (ImmSrc_i)
            3'b000: // I-type
                ImmExt_o = {{20{Imm_Ins_i[31]}}, Imm_Ins_i[31:20]};
            3'b001: // S-type
                ImmExt_o = {{20{Imm_Ins_i[31]}}, Imm_Ins_i[31:25], Imm_Ins_i[11:7]};
            3'b010: // B-type
                ImmExt_o = {{19{Imm_Ins_i[31]}}, Imm_Ins_i[31], Imm_Ins_i[7], Imm_Ins_i[30:25], Imm_Ins_i[11:8], 1'b0};
            3'b011: // U-type (LUI/AUIPC)
                ImmExt_o = {Imm_Ins_i[31:12], 12'b0};
            3'b100: // J-type
                ImmExt_o = {{11{Imm_Ins_i[31]}}, Imm_Ins_i[31], Imm_Ins_i[19:12], Imm_Ins_i[20], Imm_Ins_i[30:21], 1'b0};
            3'b101: // Shift immediate (SLLI, SRLI, SRAI)
                ImmExt_o = {27'b0, Imm_Ins_i[24:20]};
            default:
                ImmExt_o = 32'b0;
        endcase
    end
endmodule

module flopr  #(
    parameter DEPTH = 32,
    parameter INIT = 0
    )(
    input clk_i,
    input rst_ni,
    input [DEPTH-1:0] pc_next,
    output reg [DEPTH-1:0] pc
    );
    
    initial pc <= INIT;
    always @(posedge clk_i) begin
        pc <= rst_ni ? {pc_next[DEPTH-1:1], 1'b0} : {DEPTH{1'b0}};
    end
endmodule

module adder #(parameter DEPTH = 32)(
    input [DEPTH-1:0] a_i,
    input [DEPTH-1:0] b_i,
    output [DEPTH-1:0] c_o
    );

    assign c_o = a_i + b_i;
    
endmodule
