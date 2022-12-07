/***********************************************************************************************/
/*********************************  MIPS 5-stage pipeline implementation ***********************/
/***********************************************************************************************/
`timescale 1ns/1ps

module cpu(clock, reset);
    input clock,  reset;

    // PC
    reg [31:0] PC; 
    // IFID
    reg [31:0] IFID_PCplus4, IFID_instr;
    // IDEX
    reg [31:0] IDEX_signExtend, IDEX_PCplus4;
    reg [4:0]  IDEX_instr_rt, IDEX_instr_rs, IDEX_instr_rd;                            
    reg [1:0]  IDEX_ALUcntrl, IDEX_Branch;
    reg        IDEX_RegDst, IDEX_ALUSrc, IDEX_MemRead, IDEX_MemWrite, IDEX_MemToReg, IDEX_RegWrite;
    // EXMEM           
    reg [31:0] EXMEM_ALUOut, EXMEM_PCBranch, EXMEM_MemWriteData;
    reg [4:0]  EXMEM_RegWriteAddr, EXMEM_instr_rd; 
    reg [1:0]  EXMEM_Branch;
    reg        EXMEM_Zero, EXMEM_MemRead, EXMEM_MemWrite, EXMEM_RegWrite, EXMEM_MemToReg;
    // MEMWB
    reg [31:0] MEMWB_DMemOut;
    reg [4:0]  MEMWB_RegWriteAddr, MEMWB_instr_rd; 
    reg [31:0] MEMWB_ALUOut;
    reg        MEMWB_MemToReg, MEMWB_RegWrite;               

    wire [31:0] instr, MemWriteData, ALUInA, ALUInB, ALUOut, rdA, rdB, signExtend, DMemOut, wRegData, PCBranch, PCJump;
    wire        Zero, RegDst, MemRead, MemWrite, MemToReg, ALUSrc, RegWrite, Jump, PCSrc;
    wire [1:0]  Branch, ALUcntrl, FwA, FwB;
    wire [5:0]  opcode, func;
    wire [4:0]  instr_rs, instr_rt, instr_rd, RegWriteAddr, shamt;
    wire [3:0]  ALUOp;
    wire [15:0] imm;
    wire        PCWrite, bubble_idex, IFID_Write;
    wire        IF_Flush, ID_Flush, EX_Flush;


    /***************** Instruction Fetch Unit (IF)  ****************/
    always @(posedge clock, negedge reset) begin 
        if (~reset)     
            PC <= -1;     
        else if (PC == -1)
            PC <= 0;
        else if (PCWrite == 1'b1) begin
            if (Jump)
                PC <= PCJump;
            else if (PCSrc)
                PC <= EXMEM_PCBranch;
            else
                PC <= PC + 4;
        end
    end
    
    // IFID pipeline register
    always @(posedge clock, negedge reset) begin 
        if (~reset || IF_Flush) begin
            IFID_PCplus4 <= 32'b0;    
            IFID_instr   <= 32'b0;
        end 
        else if (IFID_Write) begin
            IFID_PCplus4 <= PC + 32'd4;
            IFID_instr   <= instr;
        end
    end
    
    // TO FILL IN: Instantiate the Instruction Memory here 
    Memory cpu_IMem(clock, reset, 1'b1, 1'b0, PC >> 2, 32'b0, instr); 
    

    /***************** Instruction Decode Unit (ID)  ****************/
    assign opcode     = IFID_instr[31:26];
    assign instr_rs   = IFID_instr[25:21];
    assign instr_rt   = IFID_instr[20:16];
    assign instr_rd   = IFID_instr[15:11];
    assign imm        = IFID_instr[15:0];
    assign signExtend = { { 16{ imm[15] } }, imm };

    assign PCJump   = { IFID_PCplus4[31:28], (IFID_instr[25:0] << 2) };

    // Register file
    RegFile cpu_regs(clock, reset, instr_rs, instr_rt, MEMWB_RegWriteAddr, MEMWB_RegWrite, wRegData, rdA, rdB);

    // IDEX pipeline register
    always @(posedge clock, negedge reset) begin 
        if (~reset) begin
            IDEX_PCplus4    <= 32'b0;
            IDEX_signExtend <= 32'b0;
            IDEX_instr_rd   <= 5'b0;
            IDEX_instr_rs   <= 5'b0;
            IDEX_instr_rt   <= 5'b0;
            IDEX_RegDst     <= 1'b0;
            IDEX_ALUcntrl   <= 2'b0;
            IDEX_ALUSrc     <= 1'b0;
            IDEX_Branch     <= 2'b0;
            IDEX_MemRead    <= 1'b0;
            IDEX_MemWrite   <= 1'b0;
            IDEX_MemToReg   <= 1'b0;
            IDEX_RegWrite   <= 1'b0;
        end 
        else begin
            IDEX_PCplus4    <= IFID_PCplus4;
            IDEX_signExtend <= signExtend;
            IDEX_instr_rd   <= instr_rd;
            IDEX_instr_rs   <= instr_rs;
            IDEX_instr_rt   <= instr_rt;
            if (ID_Flush) begin
                IDEX_RegDst   <= 1'b0;
                IDEX_ALUcntrl <= 2'b0;
                IDEX_ALUSrc   <= 1'b0;
                IDEX_Branch   <= 2'b0;
                IDEX_MemRead  <= 1'b0;
                IDEX_MemWrite <= 1'b0;
                IDEX_MemToReg <= 1'b0;
                IDEX_RegWrite <= 1'b0;
            end
            else begin
                IDEX_RegDst   <= RegDst;
                IDEX_ALUcntrl <= ALUcntrl;
                IDEX_ALUSrc   <= ALUSrc;
                IDEX_Branch   <= Branch;
                IDEX_MemRead  <= MemRead;
                IDEX_MemWrite <= MemWrite;
                IDEX_MemToReg <= MemToReg;                  
                IDEX_RegWrite <= RegWrite;
            end
        end
    end

    // Main Control Unit 
    control_main control_main(RegDst, Branch, MemRead, MemWrite, MemToReg, ALUSrc, RegWrite, ALUcntrl, Jump, opcode);
                    
    // TO FILL IN: Instantiation of Control Unit that generates stalls
    hazard_unit stall_detection(IDEX_MemRead, IDEX_instr_rt, instr_rs, instr_rt, bubble_idex, IFID_Write, PCWrite);

                            
    /***************** Execution Unit (EX)  ****************/   
    assign ALUInA = (FwA == 2'b10) ? EXMEM_ALUOut :
                    (FwA == 2'b01) ? wRegData : rdA;
    assign MemWriteData = (FwB == 2'b10) ? EXMEM_ALUOut :
                          (FwB == 2'b01) ? wRegData : rdB;
    assign ALUInB = (IDEX_ALUSrc == 1'b1) ? IDEX_signExtend : MemWriteData;
    assign shamt = IDEX_signExtend[10:6];
    assign func = IDEX_signExtend[5:0];
    assign PCBranch = IDEX_PCplus4 + (IDEX_signExtend << 2);

    //  ALU
    ALU #(32) cpu_alu(ALUOut, Zero, ALUInA, ALUInB, ALUOp, shamt);

    assign RegWriteAddr = (IDEX_RegDst == 1'b0) ? IDEX_instr_rt : IDEX_instr_rd;

    // EXMEM pipeline register
    always @(posedge clock, negedge reset) begin 
        if (~reset) begin
            EXMEM_PCBranch     <= 32'b0;
            EXMEM_ALUOut       <= 32'b0;    
            EXMEM_RegWriteAddr <= 5'b0;
            EXMEM_MemWriteData <= 32'b0;
            EXMEM_Zero         <= 1'b0;
            EXMEM_Branch       <= 2'b0;
            EXMEM_MemRead      <= 1'b0;
            EXMEM_MemWrite     <= 1'b0;
            EXMEM_MemToReg     <= 1'b0;                  
            EXMEM_RegWrite     <= 1'b0;
        end 
        else begin
            EXMEM_PCBranch     <= PCBranch;
            EXMEM_ALUOut       <= ALUOut;    
            EXMEM_RegWriteAddr <= RegWriteAddr;
            EXMEM_MemWriteData <= MemWriteData;
            EXMEM_Zero         <= Zero;
            if (EX_Flush) begin
                EXMEM_Branch   <= 2'b0;
                EXMEM_MemRead  <= 1'b0;
                EXMEM_MemWrite <= 1'b0;
                EXMEM_MemToReg <= 1'b0;
                EXMEM_RegWrite <= 1'b0;
            end
            else begin
                EXMEM_Branch   <= IDEX_Branch;
                EXMEM_MemRead  <= IDEX_MemRead;
                EXMEM_MemWrite <= IDEX_MemWrite;
                EXMEM_MemToReg <= IDEX_MemToReg;
                EXMEM_RegWrite <= IDEX_RegWrite;
            end
        end
    end
    
    // ALU control
    control_alu control_alu(ALUOp, IDEX_ALUcntrl, func);
    
    // TO FILL IN: Instantiation of control logic for Forwarding goes here
    forward_unit bypass_detection(EXMEM_RegWriteAddr, EXMEM_RegWrite, MEMWB_RegWriteAddr, MEMWB_RegWrite, IDEX_instr_rt, IDEX_instr_rs, FwA, FwB);
    
    
    /***************** Memory Unit (MEM)  ****************/  
    assign PCSrc = (EXMEM_Branch[0] & EXMEM_Zero) | (EXMEM_Branch[1] & ~EXMEM_Zero);
    assign IF_Flush = PCSrc | Jump;
    assign ID_Flush = bubble_idex | PCSrc;
    assign EX_Flush = PCSrc;

    // Data memory 1KB
    // Instantiate the Data Memory here 
    Memory cpu_DMem(clock, reset, EXMEM_MemRead, EXMEM_MemWrite, EXMEM_ALUOut, EXMEM_MemWriteData, DMemOut);

    // MEMWB pipeline register
    always @(posedge clock or negedge reset) begin 
        if (~reset) begin
            MEMWB_DMemOut      <= 32'b0;    
            MEMWB_ALUOut       <= 32'b0;
            MEMWB_RegWriteAddr <= 5'b0;
            MEMWB_MemToReg     <= 1'b0;                  
            MEMWB_RegWrite     <= 1'b0;
        end 
        else begin
            MEMWB_DMemOut      <= DMemOut;
            MEMWB_ALUOut       <= EXMEM_ALUOut;
            MEMWB_RegWriteAddr <= EXMEM_RegWriteAddr;
            MEMWB_MemToReg     <= EXMEM_MemToReg;                  
            MEMWB_RegWrite     <= EXMEM_RegWrite;
        end
    end


    /***************** WriteBack Unit (WB)  ****************/  

    // TO FILL IN: Write Back logic 
    assign wRegData = (MEMWB_MemToReg == 1'b1) ? MEMWB_DMemOut : MEMWB_ALUOut;
endmodule