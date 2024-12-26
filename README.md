# SingleCycleCPU
# Single-Cycle RISC-V CPU
# Overview
This project implements a single-cycle RISC-V CPU based on the RV32IM instruction set architecture. The CPU is designed for the Northwestern - CompEng 361 - Lab3 course.

The design focuses on simplicity, with all instructions executed in a single clock cycle. This implementation includes modules for decoding instructions, performing arithmetic/logical operations, memory access, and control flow.

# Team Information
# Group Name:

# Features

# Single-Cycle Operation:

All instructions are executed within one clock cycle.
No pipeline; simplicity is prioritized.
Instruction Set Support:

Arithmetic and logical operations.

Load and store instructions.

Immediate operations.

Branch and jump instructions.

Multiply and divide extensions.

# Error Detection:


Detects invalid opcodes.
Identifies memory misalignment errors for load/store operations.
Modules for Modularity:

Dedicated modules for different functionalities such as arithmetic, control flow, and memory operations.
Key Modules
SingleCycleCPU: The main CPU module integrating all components.
ExecutionUnit: Handles arithmetic, logical, and multiply/divide operations.
ImmediateExecutionUnit: Processes immediate values for I-Type instructions.
BranchUnit: Evaluates branch conditions and calculates branch targets.
LoadUnit: Manages memory load operations with size-specific data alignment.
StoreUnit: Handles memory store operations with proper data alignment.
LuiUnit and AuipcUnit: Implements LUI and AUIPC instructions.
JalUnit and JalrUnit: Calculates jump addresses for JAL and JALR instructions.
Implementation Details
Instruction Decode:

Decodes opcodes, source/destination registers, function fields, and immediate values.
Supports R-Type, I-Type, S-Type, B-Type, U-Type, and J-Type instructions.
Memory and Register Management:

Fully implemented memory and register files.
Address alignment for load/store operations.
Arithmetic and Logical Operations:

ADD, SUB, AND, OR, XOR, SLL, SRL, and SRA.
Comparison instructions: SLT, SLTU.
Multiply and divide extensions: MUL, DIV, REM, etc.

# Control Flow:

Supports conditional branches (BEQ, BNE, etc.).
Jump instructions (JAL, JALR).


# Testing
The CPU is tested for:
Arithmetic operations.
Load/store instructions with different alignments.
Control flow instructions (branching, jumping).
Edge cases like invalid opcodes and misaligned memory access.
Challenges
Balancing the simplicity of a single-cycle design with the complexity of RISC-V instructions.
Managing memory alignment and ensuring accurate data handling.

# Future Work
Implement hazard detection and forwarding for a pipelined version.
Add support for additional RISC-V extensions.
Optimize for performance and clock cycle time.
