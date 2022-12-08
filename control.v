`timescale 1ns/1ps
`include "constants.h"


/************** Main control in ID pipe stage  *************/
module control_main(RegDst, Branch, MemRead, MemWrite, MemToReg, ALUSrc, RegWrite, ALUcntrl, Jump, opcode);
    output reg       RegDst;
    output reg       MemRead;
    output reg       MemWrite;  
    output reg       MemToReg;  
    output reg       ALUSrc;  
    output reg       RegWrite;  
    output reg       Jump;
    output reg [1:0] Branch;  
    output reg [1:0] ALUcntrl;  
    input      [5:0] opcode;

    always @(*) begin
        case (opcode)
            `R_FORMAT: begin
                RegWrite = 1'b1;
                RegDst   = 1'b1;
                ALUSrc   = 1'b0;
                Branch   = 2'b00;
                MemRead  = 1'b0;
                MemWrite = 1'b0;
                MemToReg = 1'b0;
                ALUcntrl = 2'b10; 
                Jump     = 1'b0;
            end
            `LW: begin
                RegWrite = 1'b1;
                RegDst   = 1'b0;
                ALUSrc   = 1'b1;
                Branch   = 2'b00;
                MemRead  = 1'b1;
                MemWrite = 1'b0;
                MemToReg = 1'b1;
                ALUcntrl = 2'b00; 
                Jump     = 1'b0;
            end
            `SW: begin
                RegWrite = 1'b0;
                RegDst   = 1'bX;
                ALUSrc   = 1'b1;
                Branch   = 2'b00;
                MemRead  = 1'b0;
                MemWrite = 1'b1;
                MemToReg = 1'bX;
                ALUcntrl = 2'b00; 
                Jump     = 1'b0;
            end
            `BEQ: begin
                RegWrite = 1'b0;
                RegDst   = 1'bX;
                ALUSrc   = 1'b0;
                Branch   = 2'b01;
                MemRead  = 1'b0;
                MemWrite = 1'b0;
                MemToReg = 1'bX;
                ALUcntrl = 2'b01; 
                Jump     = 1'b0;
            end 
            `BNE: begin
                RegWrite = 1'b0;
                RegDst   = 1'bX;
                ALUSrc   = 1'b0;
                Branch   = 2'b10;
                MemRead  = 1'b0;
                MemWrite = 1'b0;
                MemToReg = 1'bX;
                ALUcntrl = 2'b01;
                Jump     = 1'b0;
            end
            `J: begin
                RegWrite = 1'b0;
                RegDst   = 1'bX;
                ALUSrc   = 1'bX;
                Branch   = 2'bXX;
                MemRead  = 1'b0;
                MemWrite = 1'b0;
                MemToReg = 1'bX;
                ALUcntrl = 2'bXX;
                Jump     = 1'b1;
            end
            `ADDI: begin
                RegWrite = 1'b1;
                RegDst   = 1'b0;
                ALUSrc   = 1'b1;
                Branch   = 2'b00;
                MemRead  = 1'b0;
                MemWrite = 1'b0;
                MemToReg = 1'b0;
                ALUcntrl = 2'b00; 
                Jump     = 1'b0;
            end        
            default: begin
                RegWrite = 1'b0;
                RegDst   = 1'b0;
                ALUSrc   = 1'b0;
                Branch   = 2'b00;
                MemRead  = 1'b0;
                MemWrite = 1'b0;
                MemToReg = 1'b0;
                ALUcntrl = 2'b00;
                Jump     = 1'b0;
            end
        endcase
    end
endmodule

/**************** Module for Bypass Detection in EX pipe stage goes here  *********/
// TO FILL IN: Module details 
module forward_unit(EXMEM_rd, EXMEM_RegWrite, MEMWB_rd, MEMWB_RegWrite, IDEX_rt, IDEX_rs, FwA, FwB);
    input      [4:0] EXMEM_rd, MEMWB_rd, IDEX_rt, IDEX_rs;
    input            EXMEM_RegWrite, MEMWB_RegWrite;
    output reg [1:0] FwA,FwB;

    always @(*) begin
        FwA = 2'b00;
        FwB = 2'b00;

        if (EXMEM_RegWrite && EXMEM_rd != 5'b0) begin
            if (EXMEM_rd == IDEX_rs)
                FwA = 2'b10;
            if (EXMEM_rd == IDEX_rt)
                FwB = 2'b10;
        end

        if (MEMWB_RegWrite && MEMWB_rd != 5'b0) begin
            if (MEMWB_rd == IDEX_rs && (EXMEM_rd != IDEX_rs || ~EXMEM_RegWrite))
                FwA = 2'b01;
            if (MEMWB_rd == IDEX_rt && (EXMEM_rd != IDEX_rt || ~EXMEM_RegWrite))
                FwB = 2'b01;
        end
    end
endmodule          
                       

/**************** Module for Stall Detection in ID pipe stage goes here  *********/
// TO FILL IN: Module details 
module hazard_unit(IDEX_MemRead, IDEX_rt, IFID_rs, IFID_rt, bubble_idex, IFID_Write, PCWrite);
    input [4:0] IFID_rs, IFID_rt, IDEX_rt;
    input       IDEX_MemRead;
    output      PCWrite, bubble_idex, IFID_Write;

    assign PCWrite = (IDEX_MemRead && (IDEX_rt == IFID_rs || IDEX_rt == IFID_rt)) ? 1'b0 : 1'b1;
    assign IFID_Write = PCWrite;
    assign bubble_idex = ~PCWrite;
endmodule

                       
/************** control for ALU control in EX pipe stage  *************/
module control_alu(ALUOp, ALUcntrl, func);            
    output reg [3:0] ALUOp;
    input      [1:0] ALUcntrl;
    input      [5:0] func;

    always @(ALUcntrl, func) begin
        case (ALUcntrl)
            2'b00: ALUOp = `ALU_ADD;
            2'b01: ALUOp = `ALU_SUB;
            2'b10: begin
                case (func)
                    `AND:  ALUOp = `ALU_AND;
                    `OR:   ALUOp = `ALU_OR;
                    `ADD:  ALUOp = `ALU_ADD;
                    `NOR:  ALUOp = `ALU_NOR;
                    `XOR:  ALUOp = `ALU_XOR;
                    `SUB:  ALUOp = `ALU_SUB;
                    `SLT:  ALUOp = `ALU_SLT;
                    `SLL:  ALUOp = `ALU_SLL;
                    `SLLV: ALUOp = `ALU_SLLV;
                    `SRL:  ALUOp = `ALU_SRL;
                    `SRLV: ALUOp = `ALU_SRLV;
                endcase
            end
            default: ALUOp = 4'hX;
        endcase
    end
endmodule
