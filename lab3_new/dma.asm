// Liron Cohen 207481268
// Yuval Mor 209011543

// Initialization of the parameters
asm_cmd(ADD, 3, 1, 0, 50); // 0: R3 = 50 SOURCE
asm_cmd(ADD, 4, 1, 0, 60); // 1: R4 = 60 DESTINATION
asm_cmd(ADD, 5, 1, 0, 50); // 2: R5 = 50 LENGTH

// Calling the copy operation
asm_cmd(CPY, 4, 3, 5, 0); // 3: START COPY 

// Main program code to run in parallel
asm_cmd(ADD, 2, 1, 0, 50); // 4: R2 = 50
asm_cmd(ADD, 5, 1, 0, 55); // 5: R5 = 55 
asm_cmd(ADD, 3, 0, 0, 0); // 6: R3 = 0
asm_cmd(LD,  4, 0, 2, 0); // 7: R4 = MEM[R2]
asm_cmd(ADD, 3, 3, 4, 0); // 8: R3+= R4
asm_cmd(ADD, 2, 2, 1, 1); // 9: R2++
asm_cmd(JLT, 0, 2, 5, 7); // 10: if R2 < R5 jump to line 7

// Polling and checking for copy completion
asm_cmd(POL, 2, 0, 0, 0); // 11: R[2] = DMA_S->REMAIN (number of bytes left to copy)
asm_cmd(JNE, 0, 2, 0, 11); // 12: if R[2] != 0 jump to 11

// Copy validation
asm_cmd(ADD, 2, 1, 0, 1); // 13: R2 = 1 COPY CORRECT
asm_cmd(ADD, 3, 1, 0, 50); // 14: R3 = 50 SOURCE
asm_cmd(ADD, 4, 1, 0, 60); // 15: R4 = 60 DESTINATION
asm_cmd(ADD, 5, 1, 0, 100); // 16: R5 = 100  SOURCE + LENGTH
asm_cmd(LD, 6, 0, 3, 0); // 17: R6 = MEM[R3] 
asm_cmd(LD, 7, 0, 4, 0); // 18: R7 = MEM[R4]
asm_cmd(JEQ, 0, 6, 7, 22); // 19: if (R[6] == R[7]) jump to 22 
asm_cmd(ADD, 2, 0, 0, 0); // 20: R2 = 0 COPY WRONG
asm_cmd(JEQ, 0, 0, 0, 25); // 21: jump to 25 HALT
asm_cmd(ADD, 3, 3, 1, 1); // 22: R3++
asm_cmd(ADD, 4, 4, 1, 1); // 23: R4++
asm_cmd(JLT, 0, 3, 5, 17); // 24: if R3 < R5 jump to line 17
asm_cmd(HLT, 0, 0, 0, 0); // 25: HALT

