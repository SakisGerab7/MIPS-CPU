.text
.globl main            		# label "main" must be global
main:

li $1, 1
li $2, 2
li $3, 3
li $4, 4
li $5, 5
li $6, 6
li $7, 7
li $8, 8
li $9, 9
li $10, 0x10010000
li $11, 11
li $12, 12
li $13, 13
li $14, 10
li $15, 15
li $16, 16
li $17, 17
li $18, 18
li $19, 19
li $20, 20
li $22, 22
li $22, 22
li $23, 23
li $24, 20
li $25, 25
li $26, 26
li $27, 27
li $28, 28
li $29, 29
li $30, 30
li $31, 31


  # Initialize each register to have the value : reg[i] = i;
	
    add $t0, $t0, $s0   # $t0 = $8 = 24 (Decimal)
    sw $ra, 8($t2)      # Mem[$t2+8] = 31
    lw $t7, 8($t2)      # $t7 = $15 = Mem[$t2+8] = 31
    sub $t1, $t1, $a0   # $t1 = $9 = 5
    or $t6, $t7, $t5    # $t6 = $14 = 31
    and $s3, $s0, $s2   # $s3 = $19 = 16
L1:
    lw $t9, 8($t2)      # $t6 = Mem[$t2+8] = 31,  $t6 = $14 = Mem[$t2+8] = 28
    sw $gp, 8($t2)      # Mem[$t2+8] = 28, Mem[$t2+8] = 28
    sll $s0, $t5, 1     # $s0 = $16 = 26,  $s0 = $16 = 28
    lw $v0, 8($t2)      # $v0 = $2 = 28, $v0 = $2 = 28 
    beq $v0, $s0, L2    # $2, $16. RAW stall, First pass NOT TAKEN, SECOND PASS TAKEN
    addi $t5, $t5, 1    # $t5 = $13 = 14
    and $a0, $v0, $t5   # $a0 = $4 = 12
    or $a0, $a0, $t3    # bypass from ALU. $a0 = $4 = 15
    add $t1, $a0, $v0   # bypass from ALU. $t1 = $9 = 43
    slt $sp, $a0, $t1   # $sp = $29 = 1
    lw $v1, 8($t2)      # $v1 = $3 = Mem[$t2+8] = 28
    addi $t4, $v1, -1020   # $t4 = $12 = -992
    add $t4, $t4, $t4      # $t4 = $t4 = -1984
    sll $s4, $v0, 12       # $s4 = $20 = 114688
    sllv $s6, $s4, $sp     # $s6 = $22 = 229376
    j L1                   # jump only once
L2: 
    add $t5, $t5, $t5      # $t5 = $13 = 28
    xor $t0, $t0, $t1      # $t0 = $8 = 51
    addi $t4, $t3, 2       # $t4 = $12 = 13
    or  $t6, $t5, $t4      # $t6 = $14 = 29