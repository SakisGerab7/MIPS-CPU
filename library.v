`include "constants.h"
`timescale 1ns/1ps

// Small ALU. Inputs: inA, inB. Output: out. 
// Operations: bitwise and (op = 0)
//             bitwise or  (op = 1)
//             addition    (op = 2)
//             subtraction (op = 6)
//             slt         (op = 7)
//             nor         (op = 12)
//             sllv        (op = 14)
//             sll         (op = 15)
module ALU #(parameter N = 32) (out, zero, inA, inB, op, shamt);
    output reg [N-1:0] out;
    output             zero;
    input      [N-1:0] inA, inB;
    input      [3:0]   op;
    input      [4:0]   shamt;


    always @(inA, inB, op) begin
        case (op)
            `ALU_AND:  out =  inA &  inB; 
            `ALU_OR:   out =  inA |  inB; 
            `ALU_ADD:  out =  inA +  inB;
            `ALU_XOR:  out =  inA ^  inB;
            `ALU_SUB:  out =  inA + ~inB + 1; 
            `ALU_SLT:  out =  inA <  inB;
            `ALU_SRL:  out =  inB >> shamt;
            `ALU_SRLV: out =  inB >> inA;
            `ALU_NOR:  out = ~inA & ~inB;
            `ALU_SLL:  out =  inB << shamt;
            `ALU_SLLV: out =  inB << inA;
            default:   out = 'hX;
        endcase
    end

    assign zero = (out == 'b0);
endmodule


// Memory (active 1024 words, from 10 address lsbs).
// Read : disable wen, enable ren, address addr, data dout
// Write: enable wen, disable ren, address addr, data din.
module Memory (clock, reset, ren, wen, addr, din, dout);
    input         clock, reset, ren, wen;
    input  [31:0] addr, din;
    output [31:0] dout;

    reg    [31:0] data[0:4095];

    always @(ren, wen) begin // It does not correspond to hardware. Just for error detection
        if (ren & wen) begin
            $display("\nMemory ERROR (time %0d): ren and wen both active!\n", $time);
        end
    end

    always @(posedge ren, posedge wen) begin // It does not correspond to hardware. Just for error detection
        if (addr[31:10] != 0) begin
            $display("\nMemory WARNING (time %0d): address msbs are not zero\n", $time);
        end
    end  

    // Read
    assign dout = (reset && ~wen && ren) ? data[addr[9:0]] : 32'bx;  
    
    // Write
    always @(negedge clock) begin
        if (reset && wen && ~ren) begin
            data[addr[9:0]] = din;
        end
    end
endmodule


// Register File. Read ports: address raA, data rdA
//                            address raB, data rdB
//                Write port: address wa, data wd, enable wen.
module RegFile (clock, reset, raA, raB, wa, wen, wd, rdA, rdB);
    input          clock, reset;
    input   [4:0]  raA, raB, wa;
    input          wen;
    input   [31:0] wd;
    output  [31:0] rdA, rdB;

    reg     [31:0] data[0:31];
    integer        i;

    always @(negedge clock, negedge reset) begin
        if (~reset) begin
            for (i = 0; i < 32; i = i + 1) begin
                data[i] <= i;
            end
        end
        else begin
            if (wen) begin
                data[wa] <= wd;
            end
        end
    end

    assign rdA = data[raA];
    assign rdB = data[raB];
endmodule

