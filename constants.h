//  Timing constants
`define clock_period    20
`define clk_to_q_min	0.1
`define clk_to_q_max	0.2
`define setup		0.2
`define hold		0.1
//
`define alu_delay	1.5
`define mux2_delay	0.2
`define mux4_delay	0.3
`define mux8_delay      0.4
`define nor32_delay     0.3
//
`define mem_data_delay  1.5
`define mem_addr_delay  0.5
`define rf_data_delay   0.7
`define rf_addr_delay   0.5

// Opcodes 
// R-format, FUNC
`define R_FORMAT  6'b0
`define SLL 6'b000000
`define SRL 6'b000010
`define SLLV 6'b000100
`define SRLV 6'b000110
`define ADD 6'b100000            
`define SUB 6'b100010
`define AND 6'b100100
`define OR  6'b100101
`define NOR 6'b100111
`define XOR 6'b100110
`define SLT 6'b101010

// I-format, OPCODE
`define ADDI 6'b001000
`define LW  6'b100011 
`define SW  6'b101011 
`define BEQ  6'b000100 
`define BNE  6'b000101
`define J    6'b000010
`define NOP  32'b0000_0000_0000_0000_0000_0000_0000_0000

// ALU Opcodes
`define ALU_AND  4'b0000 // 0
`define ALU_OR   4'b0001 // 1
`define ALU_ADD  4'b0010 // 2
`define ALU_XOR  4'b0011 // 3
`define ALU_SUB  4'b0110 // 6
`define ALU_SLT  4'b0111 // 7
`define ALU_SRL  4'b1000 // 8
`define ALU_SRLV 4'b1001 // 9
`define ALU_NOR  4'b1100 // 12
`define ALU_SLL  4'b1110 // 14
`define ALU_SLLV 4'b1111 // 15