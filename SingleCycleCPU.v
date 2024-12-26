// Template for Northwestern - CompEng 361 - Lab3
// Groupname:
// NetIDs:

// Some useful defines...please add your own
`define WORD_WIDTH 32
`define NUM_REGS 32
`define OPCODE_COMPUTE    7'b0110011
`define OPCODE_BRANCH     7'b1100011
`define OPCODE_LOAD       7'b0000011
`define OPCODE_STORE      7'b0100011 
`define AUX_FUNC_ADD  7'b0000000
`define AUX_FUNC_SUB  7'b0100000
`define FUNC_SUB      3'b000
`define SIZE_BYTE  2'b00
`define SIZE_HWORD 2'b01
`define SIZE_WORD  2'b10

`define OPCODE_IMMEDIATE  7'b0010011
`define OPCODE_JAL        7'b1101111
`define OPCODE_JALR       7'b1100111
`define OPCODE_LUI        7'b0110111
`define OPCODE_AUIPC      7'b0010111

// Basic instructions
`define ADD_OPCODE 3'b000
`define SUB_OPCODE 3'b000
`define SLL_OPCODE 3'b001
`define SLT_OPCODE 3'b010
`define SLTU_OPCODE 3'b011
`define XOR_OPCODE 3'b100
`define SRL_OPCODE 3'b101
`define SRA_OPCODE 3'b101
`define OR_OPCODE 3'b110
`define AND_OPCODE 3'b111

// Multiply extension
`define MUL_OPCODE  3'b000
`define MULH_OPCODE   3'b001
`define MULHSU_OPCODE 3'b010
`define MULHU_OPCODE  3'b011
`define DIV_OPCODE  3'b100
`define DIVU_OPCODE 3'b101
`define REM_OPCODE  3'b110
`define REMU_OPCODE 3'b111

// Immediate computation
`define ADDI_OPCODE  3'b000
`define SLTI_OPCODE  3'b010
`define SLTIU_OPCODE 3'b011
`define XORI_OPCODE  3'b100
`define ORI_OPCODE   3'b110
`define ANDI_OPCODE  3'b111
`define SLLI_OPCODE  3'b001
`define SRLI_OPCODE 3'b101
`define SRAI_OPCODE 3'b101

// Load instruction opcodes
`define LB_OPCODE  3'b000
`define LH_OPCODE  3'b001
`define LW_OPCODE  3'b010
`define LBU_OPCODE 3'b100
`define LHU_OPCODE 3'b101

// Store instruction opcodes
`define SB_OPCODE  3'b000
`define SH_OPCODE  3'b001
`define SW_OPCODE  3'b010

// Branch instruction opcodes
`define BEQ_OPCODE   3'b000
`define BNE_OPCODE   3'b001
`define BLT_OPCODE   3'b100
`define BGE_OPCODE   3'b101
`define BLTU_OPCODE  3'b110
`define BGEU_OPCODE  3'b111


module SingleCycleCPU(halt, clk, rst);
   output halt;
   input clk, rst;

   wire [`WORD_WIDTH-1:0] PC, InstWord;
   wire [`WORD_WIDTH-1:0] DataAddr, StoreData, DataWord;
   wire [1:0]  MemSize;
   wire        MemWrEn;
   
   wire [4:0]  Rsrc1, Rsrc2, Rdst;
   wire [`WORD_WIDTH-1:0] Rdata1, Rdata2, RWrdata;
   wire        RWrEn;

   wire [`WORD_WIDTH-1:0] NPC, PC_Plus_4;
   wire [6:0]  opcode;

   wire [6:0]  funct7;
   wire [2:0]  funct3;

   wire invalid_op;

   assign halt = invalid_op || mem_alignment_error;
   assign invalid_op = !(
        (opcode == `OPCODE_COMPUTE && 
            (funct7 == 7'b0000000 || funct7 == 7'b0100000 || funct7 == 7'b0000001)) ||
        (opcode == `OPCODE_BRANCH) ||
        (opcode == `OPCODE_LOAD) ||
        (opcode == `OPCODE_STORE) ||
        (opcode == `OPCODE_IMMEDIATE) ||
        (opcode == `OPCODE_JAL) ||
        (opcode == `OPCODE_JALR) ||
        (opcode == `OPCODE_LUI) ||
        (opcode == `OPCODE_AUIPC)
    );

    wire [31:0] effective_addr = (opcode == `OPCODE_LOAD) ? (Rdata1 + imm_I) :
                                 (opcode == `OPCODE_STORE) ? (Rdata1 + imm_S) :
                                 32'b0;

    assign mem_alignment_error = 
        ((opcode == `OPCODE_LOAD || opcode == `OPCODE_STORE) &&
        (
            ((funct3 == `LH_OPCODE || funct3 == `LHU_OPCODE || funct3 == `SH_OPCODE) && (effective_addr[0] != 1'b0)) ||
            ((funct3 == `LW_OPCODE || funct3 == `SW_OPCODE) && (effective_addr[1:0] != 2'b00))
        ));

   // System State 
   Mem   MEM(.InstAddr(PC), .InstOut(InstWord), 
            .DataAddr(DataAddr), .DataSize(MemSize), .DataIn(StoreData), .DataOut(DataWord), .WE(MemWrEn), .CLK(clk));

   RegFile RF(.AddrA(Rsrc1), .DataOutA(Rdata1), 
         .AddrB(Rsrc2), .DataOutB(Rdata2), 
         .AddrW(Rdst), .DataInW(RWrdata), .WenW(RWrEn), .CLK(clk));

   Reg PC_REG(.Din(NPC), .Qout(PC), .WE(1'b1), .CLK(clk), .RST(rst));

   // Instruction Decode
   assign opcode = InstWord[6:0];   
   assign Rdst = InstWord[11:7]; 
   assign Rsrc1 = InstWord[19:15]; 
   assign Rsrc2 = InstWord[24:20];
   assign funct3 = InstWord[14:12];  // R-Type, I-Type, S-Type
   assign funct7 = InstWord[31:25];  // R-Type

   // Decode immediate values
   wire [31:0] imm_I = {{20{InstWord[31]}}, InstWord[31:20]};
   wire [31:0] imm_S = {{20{InstWord[31]}}, InstWord[31:25], InstWord[11:7]};
   wire [31:0] imm_B = {{19{InstWord[31]}}, InstWord[31], InstWord[7], InstWord[30:25], InstWord[11:8], 1'b0};
   wire [31:0] imm_U = {InstWord[31:12], 12'b0};
   wire [31:0] imm_J = {{11{InstWord[31]}}, InstWord[31], InstWord[19:12], InstWord[20], InstWord[30:21], 1'b0};

   // only memory write enabled if opcode is to store
   assign MemWrEn = (opcode == `OPCODE_STORE) ? 1'b1 : 1'b0;

   // lots of opcodes for register write enable
   assign RWrEn = (opcode == `OPCODE_COMPUTE) || 
               (opcode == `OPCODE_IMMEDIATE) || 
               (opcode == `OPCODE_LOAD) || 
               (opcode == `OPCODE_JAL) ||
               (opcode == `OPCODE_JALR) ||
               (opcode == `OPCODE_LUI) ||
               (opcode == `OPCODE_AUIPC);

   // Every execution unit
   wire [31:0] eu_out;
   wire [31:0] ieu_out;
   wire [31:0] branch_out;
   wire [31:0] load_out;
   wire [31:0] load_DataAddr;
   wire [1:0] load_MemSize;
   wire [31:0] store_DataAddr;
   wire [31:0] store_StoreData;
   wire [1:0] store_MemSize;
   wire [31:0] lui_out;
   wire [31:0] auipc_out;
   wire [31:0] jal_out;
   wire [31:0] jalr_out;

   ExecutionUnit EU(.out(eu_out), .opA(Rdata1), .opB(Rdata2), .func(funct3), .auxFunc(funct7));
   ImmediateExecutionUnit IEU(.out(ieu_out), .opA(Rdata1), .imm(imm_I), .func(funct3), .auxFunc(funct7));
   
   BranchUnit BU(.out(branch_out), .opA(Rdata1), .opB(Rdata2), .pc(PC), .imm(imm_B[11:0]), .func(funct3), .auxFunc(opcode));
   LoadUnit LU(.out(load_out), .DataAddr(load_DataAddr), .MemSize(load_MemSize), .base(Rdata1), .offset(imm_I[11:0]), .func(funct3), .DataWord(DataWord));
   StoreUnit SU(.DataAddr(store_DataAddr), .StoreData(store_StoreData), .MemSize(store_MemSize), .MemWrEn(store_enable), .data(Rdata2), .base(Rdata1), .offset(imm_S[11:0]), .func(funct3));

   LuiUnit LUI(.out(lui_out), .imm(InstWord[31:12]), .pc(PC));
   AuipcUnit AUIPC(.out(auipc_out), .imm(InstWord[31:12]), .pc(PC));
   JalUnit JAL(.out(jal_out), .offset(imm_J[20:0]), .pc(PC));
   JalrUnit JALR(.out(jalr_out), .base(Rdata1), .offset(imm_I[11:0]), .pc(PC));


   // only memory write enabled if opcode is to store
   assign MemWrEn = (opcode == `OPCODE_STORE) ? 1'b1 : 1'b0;

   // lots of opcodes for register write enable
   assign RWrEn = (opcode == `OPCODE_COMPUTE) || 
               (opcode == `OPCODE_IMMEDIATE) || 
               (opcode == `OPCODE_LOAD) || 
               (opcode == `OPCODE_JAL) ||
               (opcode == `OPCODE_JALR) ||
               (opcode == `OPCODE_LUI) ||
               (opcode == `OPCODE_AUIPC);

   // assignment of DataAddr
   assign DataAddr = (opcode == `OPCODE_LOAD) ? load_DataAddr :
                  (opcode == `OPCODE_STORE) ? store_DataAddr : 
                  32'b0;

   // assignment of MemSize
   assign MemSize = (opcode == `OPCODE_LOAD) ? load_MemSize :
                  (opcode == `OPCODE_STORE) ? store_MemSize : 
                  2'b00;

   // asignment of StoreData
   assign StoreData = (opcode == `OPCODE_STORE) ? store_StoreData : 32'b0;

   // assignment of data to write to register
   assign RWrdata = (opcode == `OPCODE_COMPUTE) ? eu_out :
                 (opcode == `OPCODE_IMMEDIATE) ? ieu_out :
                 (opcode == `OPCODE_LOAD) ? load_out :
                 (opcode == `OPCODE_LUI) ? lui_out :
                 (opcode == `OPCODE_AUIPC) ? auipc_out :
                 (opcode == `OPCODE_JAL) ? (PC + 4) :
                 (opcode == `OPCODE_JALR) ? (PC + 4) : 
                 32'b0;

   // Fetch Address Datapath
   assign PC_Plus_4 = PC + 4;
   assign NPC = (opcode == `OPCODE_BRANCH) ? branch_out :
             (opcode == `OPCODE_JAL) ? jal_out :
             (opcode == `OPCODE_JALR) ? jalr_out :
             PC_Plus_4;
   
endmodule // SingleCycleCPU

module ExecutionUnit(
           output [31:0] out,
           input [31:0]  opA,
           input [31:0]  opB,
           input [2:0]   func,
           input [6:0]   auxFunc);

   // Basic instructions
   wire [31:0] add_output = opA + opB;
   wire [31:0] sub_output = opA - opB;
   wire [31:0] sll_output = opA << opB[4:0];
   wire [31:0] slt_output = {31'b0, ($signed(opA) < $signed(opB)) ? 1'b1 : 1'b0};
   wire [31:0] sltu_output = {31'b0, (opA < opB) ? 1'b1 : 1'b0};
   wire [31:0] xor_output = opA ^ opB;
   wire [31:0] srl_output = opA >> opB[4:0];
   wire [31:0] sra_output = $signed(opA) >>> opB[4:0];
   wire [31:0] or_output = opA | opB;
   wire [31:0] and_output = opA & opB;

   // Multiply extension
   wire [31:0] mul_output = opA * opB;
   wire [63:0] mulh_temp = ($signed(opA) * $signed(opB));
   wire [31:0] mulh_output =  mulh_temp [63:32];
   wire [63:0] mulhsu_temp = ($signed(opA) * opB);
   wire [31:0] mulhsu_output =  mulhsu_temp [63:32];
   wire [63:0] mulhu_temp = (opA * opB);
   wire [31:0] mulhu_output =  mulhu_temp [63:32];
   wire [31:0] div_output = (opB != 0) ? $signed(opA) / $signed(opB) : 32'b0;
   wire [31:0] divu_output = (opB != 0) ? opA / opB : 32'b0;
   wire [31:0] rem_output = (opB != 0) ? $signed(opA) % $signed(opB) : 32'b0;
   wire [31:0] remu_output = (opB != 0) ? opA % opB : 32'b0;

   assign out = (func == `SUB_OPCODE && auxFunc == 7'b0100000) ? sub_output :
             (func == `ADD_OPCODE && auxFunc == 7'b0000000) ? add_output :
             (func == `SLL_OPCODE && auxFunc == 7'b0000000) ? sll_output :
             (func == `SLT_OPCODE && auxFunc == 7'b0000000) ? slt_output :
             (func == `SLTU_OPCODE && auxFunc == 7'b0000000) ? sltu_output :
             (func == `XOR_OPCODE && auxFunc == 7'b0000000) ? xor_output :
             (func == `SRA_OPCODE && auxFunc == 7'b0100000) ? sra_output :
             (func == `SRL_OPCODE && auxFunc == 7'b0000000) ? srl_output :
             (func == `OR_OPCODE && auxFunc == 7'b0000000) ? or_output :
             (func == `AND_OPCODE && auxFunc == 7'b0000000) ? and_output :
             (func == `MUL_OPCODE && auxFunc == 7'b0000001) ? mul_output :
             (func == `DIV_OPCODE && auxFunc == 7'b0000001) ? div_output :
             (func == `DIVU_OPCODE && auxFunc == 7'b0000001) ? divu_output :
             (func == `REM_OPCODE && auxFunc == 7'b0000001) ? rem_output :
             (func == `REMU_OPCODE && auxFunc == 7'b0000001) ? remu_output :
             (func == `MULH_OPCODE && auxFunc == 7'b0000001) ? mulh_output :
             (func == `MULHSU_OPCODE && auxFunc == 7'b0000001) ? mulhsu_output :
             (func == `MULHU_OPCODE && auxFunc == 7'b0000001) ? mulhu_output :
             32'b0;

endmodule

module ImmediateExecutionUnit(
           output [31:0] out,
           input [31:0]  opA,
           input [31:0]  imm,
           input [2:0]   func,
           input [6:0]   auxFunc);

   wire [31:0] addi_output  = opA + imm;
   wire [31:0] slti_output  = {31'b0, ($signed(opA) < $signed(imm)) ? 1'b1 : 1'b0};
   wire [31:0] sltiu_output = {31'b0, (opA < imm) ? 1'b1 : 1'b0};
   wire [31:0] xori_output  = opA ^ imm;
   wire [31:0] ori_output   = opA | imm;
   wire [31:0] andi_output  = opA & imm;
   wire [31:0] slli_output = opA << imm[4:0];
   wire [31:0] srli_output = opA >> imm[4:0];
   wire [31:0] srai_output = $signed(opA) >>> imm[4:0];

   assign out = (func == `ADDI_OPCODE) ? addi_output :
                (func == `SLTI_OPCODE) ? slti_output :
                (func == `SLTIU_OPCODE) ? sltiu_output :
                (func == `XORI_OPCODE) ? xori_output :
                (func == `ORI_OPCODE) ? ori_output :
                (func == `ANDI_OPCODE) ? andi_output :
                (func == `SLLI_OPCODE) ? slli_output :
                (func == `SRLI_OPCODE && auxFunc == 7'b0000000) ? srli_output : 
                (func == `SRAI_OPCODE && auxFunc == 7'b0100000) ? srai_output : 
                32'b0;

endmodule 


module BranchUnit(
           output [31:0] out,
           input [31:0]  opA,
           input [31:0]  opB,
           input [31:0]  pc,
           input [11:0]  imm,
           input [2:0]   func,
           input [6:0]   auxFunc);
   
   wire [31:0] extended_imm = {{20{imm[11]}}, imm};
   wire [31:0] branch_offset = extended_imm;

   wire condition_met = (func == `BEQ_OPCODE) ? (opA == opB) :
                     (func == `BNE_OPCODE) ? (opA != opB) :
                     (func == `BLT_OPCODE) ? ($signed(opA) < $signed(opB)) :
                     (func == `BGE_OPCODE) ? ($signed(opA) >= $signed(opB)) :
                     (func == `BLTU_OPCODE) ? (opA < opB) :
                     (func == `BGEU_OPCODE) ? (opA >= opB) :
                     1'b0;

   assign out = (condition_met) ? (pc + branch_offset) : (pc + 4);

endmodule

module LoadUnit(
    output [31:0] out,
    output [31:0] DataAddr,
    output [1:0] MemSize,
    input [31:0] base,
    input [11:0] offset,
    input [2:0] func,
    input [31:0] DataWord
);
    wire [31:0] address = base + {{20{offset[11]}}, offset};
   
    assign DataAddr = address;
    assign MemSize = (func == `LB_OPCODE || func == `LBU_OPCODE) ? `SIZE_BYTE :
                     (func == `LH_OPCODE || func == `LHU_OPCODE) ? `SIZE_HWORD :
                    `SIZE_WORD;

    // Output 
    assign out = (func == `LB_OPCODE) ? {{24{DataWord[7]}}, DataWord[7:0]} :  // sign-extend w
                 (func == `LBU_OPCODE) ? {24'b0, DataWord[7:0]} :             // zero-extend w
                 (func == `LH_OPCODE) ? {{16{DataWord[15]}}, DataWord[15:0]} : // sign-extend hw
                 (func == `LHU_OPCODE) ? {16'b0, DataWord[15:0]} :            // zero-extend hw
                 DataWord;  // Load word
endmodule

module StoreUnit(
    output [31:0] DataAddr,
    output [31:0] StoreData,
    output [1:0] MemSize,
    output MemWrEn,
    input [31:0] data,
    input [31:0] base,
    input [11:0] offset,
    input [2:0] func
);
    wire [31:0] address = base + {{20{offset[11]}}, offset};
   
    assign DataAddr = address;
    assign MemSize = (func == `SB_OPCODE) ? `SIZE_BYTE :
                     (func == `SH_OPCODE) ? `SIZE_HWORD :
                     `SIZE_WORD;
    assign MemWrEn = 1'b1; // Enable write
    assign StoreData = (func == `SB_OPCODE) ? {24'b0, data[7:0]} :   // byte data
                       (func == `SH_OPCODE) ? {16'b0, data[15:0]} :  // hw data
                       data;  // w
endmodule

module LuiUnit(
    output [31:0] out,
    input [19:0]  imm,
    input [31:0]  pc
);
    assign out = {imm, 12'b0};  
endmodule

module AuipcUnit(
    output [31:0] out,
    input [19:0]  imm,
    input [31:0]  pc
);
    assign out = pc + {imm, 12'b0};  // Add upper imm to pc
endmodule

module JalUnit(
    output [31:0] out,
    input [20:0]  offset,
    input [31:0]  pc
);
    wire [31:0] target = pc + {11'b0, offset};
    assign out = target;
endmodule

module JalrUnit(
    output [31:0] out,
    input [31:0]  base,
    input [11:0]  offset,
    input [31:0]  pc
);
    wire [31:0] target = base + {20'b0, offset};
    assign out = {target[31:1], 1'b0};  // Zero lsb
endmodule


