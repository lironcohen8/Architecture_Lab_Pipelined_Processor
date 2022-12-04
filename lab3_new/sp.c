#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>

#include "llsim.h"

#define sp_printf(a...)						\
	do {							\
		llsim_printf("sp: clock %d: ", llsim->clock);	\
		llsim_printf(a);				\
	} while (0)

int nr_simulated_instructions = 0;
FILE *inst_trace_fp = NULL, *cycle_trace_fp = NULL;

typedef struct sp_registers_s {
	// 6 32 bit registers (r[0], r[1] don't exist)
	int r[8];

	// 32 bit cycle counter
	int cycle_counter;

	// fetch0
	int fetch0_active; // 1 bit
	int fetch0_pc; // 16 bits

	// fetch1
	int fetch1_active; // 1 bit
	int fetch1_pc; // 16 bits

	// dec0
	int dec0_active; // 1 bit
	int dec0_pc; // 16 bits
	int dec0_inst; // 32 bits

	// dec1
	int dec1_active; // 1 bit
	int dec1_pc; // 16 bits
	int dec1_inst; // 32 bits
	int dec1_opcode; // 5 bits
	int dec1_src0; // 3 bits
	int dec1_src1; // 3 bits
	int dec1_dst; // 3 bits
	int dec1_immediate; // 32 bits

	// exec0
	int exec0_active; // 1 bit
	int exec0_pc; // 16 bits
	int exec0_inst; // 32 bits
	int exec0_opcode; // 5 bits
	int exec0_src0; // 3 bits
	int exec0_src1; // 3 bits
	int exec0_dst; // 3 bits
	int exec0_immediate; // 32 bits
	int exec0_alu0; // 32 bits
	int exec0_alu1; // 32 bits

	// exec1
	int exec1_active; // 1 bit
	int exec1_pc; // 16 bits
	int exec1_inst; // 32 bits
	int exec1_opcode; // 5 bits
	int exec1_src0; // 3 bits
	int exec1_src1; // 3 bits
	int exec1_dst; // 3 bits
	int exec1_immediate; // 32 bits
	int exec1_alu0; // 32 bits
	int exec1_alu1; // 32 bits
	int exec1_aluout;
} sp_registers_t;

/*
 * Master structure
 */
typedef struct sp_s {
	// local srams
#define SP_SRAM_HEIGHT	64 * 1024
	llsim_memory_t *srami, *sramd;

	unsigned int memory_image[SP_SRAM_HEIGHT];
	int memory_image_size;

	int start;

	sp_registers_t *spro, *sprn;
} sp_t;

static void sp_reset(sp_t *sp)
{
	sp_registers_t *sprn = sp->sprn;

	memset(sprn, 0, sizeof(*sprn));
}

/*
 * opcodes
 */
#define ADD 0
#define SUB 1
#define LSF 2
#define RSF 3
#define AND 4
#define OR  5
#define XOR 6
#define LHI 7
#define LD 8
#define ST 9
#define JLT 16
#define JLE 17
#define JEQ 18
#define JNE 19
#define JIN 20
#define HLT 24

#define OPCODE_MASK 0x3E000000
#define OPCODE_SHIFT 0x19
#define DST_MASK 0x01C00000
#define DST_SHIFT 0x16
#define SRC0_MASK 0x00380000
#define SRC0_SHIFT 0x13
#define SRC1_MASK 0x00070000
#define SRC1_SHIFT 0x10
#define ALU1_SHIFT 0x10
#define IMM_MASK 0x0000FFFF
#define LOWER_16_BITS_MASK 0x0000FFFF
#define SIGN_EXT_MASK 0x00008000
#define SIGN_EXT 0xFFFF0000

static char opcode_name[32][4] = {"ADD", "SUB", "LSF", "RSF", "AND", "OR", "XOR", "LHI",
				 "LD", "ST", "U", "U", "U", "U", "U", "U",
				 "JLT", "JLE", "JEQ", "JNE", "JIN", "U", "U", "U",
				 "HLT", "U", "U", "U", "U", "U", "U", "U"};

static int inst_cnt = 0;

static void dump_sram(sp_t *sp, char *name, llsim_memory_t *sram)
{
	FILE *fp;
	int i;

	fp = fopen(name, "w");
	if (fp == NULL) {
                printf("couldn't open file %s\n", name);
                exit(1);
	}
	for (i = 0; i < SP_SRAM_HEIGHT; i++)
		fprintf(fp, "%08x\n", llsim_mem_extract(sram, i, 31, 0));
	fclose(fp);
}

static void sp_ctl(sp_t *sp)
{
	sp_registers_t *spro = sp->spro;
	sp_registers_t *sprn = sp->sprn;
	int i;

	fprintf(cycle_trace_fp, "cycle %d\n", spro->cycle_counter);
	fprintf(cycle_trace_fp, "cycle_counter %08x\n", spro->cycle_counter);
	for (i = 2; i <= 7; i++)
		fprintf(cycle_trace_fp, "r%d %08x\n", i, spro->r[i]);

	fprintf(cycle_trace_fp, "fetch0_active %08x\n", spro->fetch0_active);
	fprintf(cycle_trace_fp, "fetch0_pc %08x\n", spro->fetch0_pc);

	fprintf(cycle_trace_fp, "fetch1_active %08x\n", spro->fetch1_active);
	fprintf(cycle_trace_fp, "fetch1_pc %08x\n", spro->fetch1_pc);

	fprintf(cycle_trace_fp, "dec0_active %08x\n", spro->dec0_active);
	fprintf(cycle_trace_fp, "dec0_pc %08x\n", spro->dec0_pc);
	fprintf(cycle_trace_fp, "dec0_inst %08x\n", spro->dec0_inst); // 32 bits

	fprintf(cycle_trace_fp, "dec1_active %08x\n", spro->dec1_active);
	fprintf(cycle_trace_fp, "dec1_pc %08x\n", spro->dec1_pc); // 16 bits
	fprintf(cycle_trace_fp, "dec1_inst %08x\n", spro->dec1_inst); // 32 bits
	fprintf(cycle_trace_fp, "dec1_opcode %08x\n", spro->dec1_opcode); // 5 bits
	fprintf(cycle_trace_fp, "dec1_src0 %08x\n", spro->dec1_src0); // 3 bits
	fprintf(cycle_trace_fp, "dec1_src1 %08x\n", spro->dec1_src1); // 3 bits
	fprintf(cycle_trace_fp, "dec1_dst %08x\n", spro->dec1_dst); // 3 bits
	fprintf(cycle_trace_fp, "dec1_immediate %08x\n", spro->dec1_immediate); // 32 bits

	fprintf(cycle_trace_fp, "exec0_active %08x\n", spro->exec0_active);
	fprintf(cycle_trace_fp, "exec0_pc %08x\n", spro->exec0_pc); // 16 bits
	fprintf(cycle_trace_fp, "exec0_inst %08x\n", spro->exec0_inst); // 32 bits
	fprintf(cycle_trace_fp, "exec0_opcode %08x\n", spro->exec0_opcode); // 5 bits
	fprintf(cycle_trace_fp, "exec0_src0 %08x\n", spro->exec0_src0); // 3 bits
	fprintf(cycle_trace_fp, "exec0_src1 %08x\n", spro->exec0_src1); // 3 bits
	fprintf(cycle_trace_fp, "exec0_dst %08x\n", spro->exec0_dst); // 3 bits
	fprintf(cycle_trace_fp, "exec0_immediate %08x\n", spro->exec0_immediate); // 32 bits
	fprintf(cycle_trace_fp, "exec0_alu0 %08x\n", spro->exec0_alu0); // 32 bits
	fprintf(cycle_trace_fp, "exec0_alu1 %08x\n", spro->exec0_alu1); // 32 bits

	fprintf(cycle_trace_fp, "exec1_active %08x\n", spro->exec1_active);
	fprintf(cycle_trace_fp, "exec1_pc %08x\n", spro->exec1_pc); // 16 bits
	fprintf(cycle_trace_fp, "exec1_inst %08x\n", spro->exec1_inst); // 32 bits
	fprintf(cycle_trace_fp, "exec1_opcode %08x\n", spro->exec1_opcode); // 5 bits
	fprintf(cycle_trace_fp, "exec1_src0 %08x\n", spro->exec1_src0); // 3 bits
	fprintf(cycle_trace_fp, "exec1_src1 %08x\n", spro->exec1_src1); // 3 bits
	fprintf(cycle_trace_fp, "exec1_dst %08x\n", spro->exec1_dst); // 3 bits
	fprintf(cycle_trace_fp, "exec1_immediate %08x\n", spro->exec1_immediate); // 32 bits
	fprintf(cycle_trace_fp, "exec1_alu0 %08x\n", spro->exec1_alu0); // 32 bits
	fprintf(cycle_trace_fp, "exec1_alu1 %08x\n", spro->exec1_alu1); // 32 bits
	fprintf(cycle_trace_fp, "exec1_aluout %08x\n", spro->exec1_aluout);

	fprintf(cycle_trace_fp, "\n\n\n"); // changes were made to align with example outputs

	sp_printf("cycle_counter %08x\n", spro->cycle_counter);
	sp_printf("r2 %08x, r3 %08x\n", spro->r[2], spro->r[3]);
	sp_printf("r4 %08x, r5 %08x, r6 %08x, r7 %08x\n", spro->r[4], spro->r[5], spro->r[6], spro->r[7]);
	sp_printf("fetch0_active %d, fetch1_active %d, dec0_active %d, dec1_active %d, exec0_active %d, exec1_active %d\n",
		  spro->fetch0_active, spro->fetch1_active, spro->dec0_active, spro->dec1_active, spro->exec0_active, spro->exec1_active);
	sp_printf("fetch0_pc %d, fetch1_pc %d, dec0_pc %d, dec1_pc %d, exec0_pc %d, exec1_pc %d\n",
		  spro->fetch0_pc, spro->fetch1_pc, spro->dec0_pc, spro->dec1_pc, spro->exec0_pc, spro->exec1_pc);

	sprn->cycle_counter = spro->cycle_counter + 1;

	if (sp->start)
		sprn->fetch0_active = 1;

	// fetch0
	sprn->fetch1_active = 0;
	if (spro->fetch0_active) { // reading current instruction from memory
		llsim_mem_read(sp->srami, spro->fetch0_pc); // fetching the current instruction from SRAMI
		sprn->fetch0_pc = (spro->fetch0_pc + 1) & 65535; // updating to next pc
        sprn->fetch1_pc = spro->fetch0_pc; // moving pc value in pipeline
        sprn->fetch1_active = 1; // activating next stage
	}

	// fetch1
	if (spro->fetch1_active) { // sampling memoty output to the instruction register
		sprn->dec0_pc = spro->fetch1_pc; // setting pc for next stage
		sprn->dec0_inst = llsim_mem_extract_dataout(sp->srami, 31, 0);
		sprn->dec0_active = 1;
	}
	else { // fetch1 is not active
		sprn->dec0_active = 0;
	}
	
	// dec0
	if (spro->dec0_active) { // decoding instruction 
		sprn->dec1_opcode = (spro->dec0_inst & OPCODE_MASK) >> OPCODE_SHIFT;
    	sprn->dec1_dst = (spro->dec0_inst & DST_MASK) >> DST_SHIFT;
		sprn->dec1_src0 = (spro->dec0_inst & SRC0_MASK) >> SRC0_SHIFT;
		sprn->dec1_src1 = (spro->dec0_inst & SRC1_MASK) >> SRC1_SHIFT;
		sprn->dec1_immediate = (spro->dec0_inst) & IMM_MASK;
		if ((spro->dec0_inst & SIGN_EXT_MASK) != 0) { // need sign extension with msb 1
                sprn->dec1_immediate = sprn->dec1_immediate + (SIGN_EXT);
        }

		sprn->dec1_inst = spro->dec0_inst;
        sprn->dec1_pc = spro->dec0_pc;
		sprn->dec1_active = 1;
	}
	else { // dec0 is not active
		sprn->dec1_active = 0;
	}

	// dec1
	if (spro->dec1_active) { // preparing ALU operands
		if (spro->dec1_opcode == LHI) {
            sprn->exec0_alu0 = (spro->r[spro->dec1_dst]) & LOWER_16_BITS_MASK;
            sprn->exec0_alu1 = spro->dec1_immediate;
        }
		else {
			// alu0
			if (spro->dec1_src0 == 0) {
				sprn->exec0_alu0 = 0;
			}

			else if (spro->dec1_src0 == 1) {
				sprn->exec0_alu0 = spro->dec1_immediate;
			}

			else if (spro->exec1_active && spro->dec1_src0 == spro->exec1_dst &&
			(spro->exec1_opcode == ADD || spro->exec1_opcode == SUB || spro->exec1_opcode == LSF ||
			spro->exec1_opcode == RSF || spro->exec1_opcode == AND || spro->exec1_opcode == OR ||
			spro->exec1_opcode == XOR || spro->exec1_opcode == LHI)) { // read after write bypass (ALU)
				sprn->exec0_alu0 = spro->exec1_aluout;
			}

			else if (spro->exec1_active && spro->exec1_opcode == LD && spro->exec1_dst == spro->dec1_src0) { // read after write bypass (MEM)
				sprn->exec0_alu0 = llsim_mem_extract_dataout(sp->sramd, 31, 0);
			}
			
			else if (spro->exec1_active && spro->exec1_aluout == 1 && spro->dec1_src0 == 7 &&
			(spro->exec1_opcode == JLT || spro->exec1_opcode == JLE || spro->exec1_opcode == JEQ ||
			spro->exec1_opcode == JNE || spro->exec1_opcode == JIN)) { // branch is taken, need to flush value of r[7]
				sprn->exec0_alu0 = spro->exec1_pc;
			}

			else {
				sprn->exec0_alu0 = spro->r[spro->dec1_src0];
			}

			// alu1
			if (spro->dec1_src1 == 0) {
				sprn->exec0_alu1 = 0;
			}

			else if (spro->dec1_src1 == 1) {
				sprn->exec0_alu1 = spro->dec1_immediate;
			}

			else if (spro->exec1_active && spro->dec1_src1 == spro->exec1_dst &&
			(spro->exec1_opcode == ADD || spro->exec1_opcode == SUB || spro->exec1_opcode == LSF ||
			spro->exec1_opcode == RSF || spro->exec1_opcode == AND || spro->exec1_opcode == OR ||
			spro->exec1_opcode == XOR || spro->exec1_opcode == LHI)) { // read after write bypass (ALU)
				sprn->exec0_alu1 = spro->exec1_aluout;
			}

			else if (spro->exec1_active && spro->exec1_opcode == LD && spro->exec1_dst == spro->dec1_src1) { // read after write bypass (MEM)
				sprn->exec0_alu1 = llsim_mem_extract_dataout(sp->sramd, 31, 0);
			}
			
			else if (spro->exec1_active && spro->exec1_aluout == 1 && spro->dec1_src1 == 7 &&
			(spro->exec1_opcode == JLT || spro->exec1_opcode == JLE || spro->exec1_opcode == JEQ ||
			spro->exec1_opcode == JNE || spro->exec1_opcode == JIN)) { // branch is taken, need to flush value of r[7]
				sprn->exec0_alu1 = spro->exec1_pc;
			}

			else {
				sprn->exec0_alu1 = spro->r[spro->dec1_src1];
			}
		}

		// moving instruction values in pipeline
		sprn->exec0_pc = spro->dec1_pc;
        sprn->exec0_inst = spro->dec1_inst;
        sprn->exec0_opcode = spro->dec1_opcode;
        sprn->exec0_dst = spro->dec1_dst;
        sprn->exec0_src0 = spro->dec1_src0;
        sprn->exec0_src1 = spro->dec1_src1;
        sprn->exec0_immediate = spro->dec1_immediate;

		sprn->exec0_active = 1;
	}
	else { // dec1 is not active
		sprn->exec0_active = 0;
	}

	// exec0
	if (spro->exec0_active) { // executing ALU and LD operations
		switch (spro->exec0_opcode) {
			case ADD:
				sprn->exec1_aluout = spro->exec0_alu0 + spro->exec0_alu1;
				break;
			case SUB:
				sprn->exec1_aluout = spro->exec0_alu0 - spro->exec0_alu1;
				break;
			case LSF:
				sprn->exec1_aluout = spro->exec0_alu0 << spro->exec0_alu1;
				break;
			case RSF:
				sprn->exec1_aluout = spro->exec0_alu0 >> spro->exec0_alu1;
				break;	
			case AND:
				sprn->exec1_aluout = spro->exec0_alu0 & spro->exec0_alu1;
				break;
			case OR:
				sprn->exec1_aluout = spro->exec0_alu0 | spro->exec0_alu1;
				break;
			case XOR:
				sprn->exec1_aluout = spro->exec0_alu0 & spro->exec0_alu1;
				break;
			case LHI:
				sprn->exec1_aluout = (spro->exec0_alu1 << ALU1_SHIFT) + (spro->exec0_alu0 & LOWER_16_BITS_MASK);
				break;	
			case LD:
				llsim_mem_read(sp->sramd, spro->exec0_alu1);
				break;
			case JLT:
				sprn->exec1_aluout = (spro->exec0_alu0 < spro->exec0_alu1) ? 1 : 0;
				break;
			case JLE:
				sprn->exec1_aluout = (spro->exec0_alu0 <= spro->exec0_alu1) ? 1 : 0;
				break;	
			case JEQ:
				sprn->exec1_aluout = (spro->exec0_alu0 == spro->exec0_alu1) ? 1 : 0;
				break;
			case JNE:
				sprn->exec1_aluout = (spro->exec0_alu0 != spro->exec0_alu1) ? 1 : 0;
				break;
			case JIN:
				sprn->exec1_aluout = 1;
				break;
			case HLT:
				break;
		}

		// moving instruction values in pipeline
		sprn->exec1_pc = spro->exec0_pc;
        sprn->exec1_inst = spro->exec0_inst;
        sprn->exec1_opcode = spro->exec0_opcode;
        sprn->exec1_dst = spro->exec0_dst;
        sprn->exec1_src0 = spro->exec0_src0;
        sprn->exec1_src1 = spro->exec0_src1;
        sprn->exec1_immediate = spro->exec0_immediate;
		sprn->exec1_alu0 = spro->exec0_alu0;
		sprn->exec1_alu1 = spro->exec0_alu1;

		sprn->exec1_active = 1;
	}
	else { // exec0 is not active
		sprn->exec1_active = 0;
	}

	// exec1
	if (spro->exec1_active) { // writing back ALU and memory (ST)
		fprintf(inst_trace_fp,"--- instruction %i (%04x) @ PC %i (%04x) -----------------------------------------------------------\n", inst_cnt, inst_cnt, spro->exec1_pc, spro->exec1_pc);
        fprintf(inst_trace_fp,"pc = %04d, inst = %08x, opcode = %i (%s), dst = %i, src0 = %i, src1 = %i, immediate = %08x\n", spro->exec1_pc, spro->exec1_inst, spro->exec1_opcode, opcode_name[spro->exec1_opcode],
        spro->exec1_dst, spro->exec1_src0, spro->exec1_src1, sbs(spro->exec1_inst, 15, 0));
        fprintf(inst_trace_fp,"r[0] = 00000000 r[1] = %08x r[2] = %08x r[3] = %08x \n",spro->exec1_immediate, spro->r[2], spro->r[3]);
        fprintf(inst_trace_fp,"r[4] = %08x r[5] = %08x r[6] = %08x r[7] = %08x \n\n", spro->r[4], spro->r[5], spro->r[6], spro->r[7]);

		inst_cnt = inst_cnt + 1;

		if (spro->exec1_opcode == ADD) {
            fprintf(inst_trace_fp, ">>>> EXEC: R[%i] = %i %s %i <<<<\n\n", spro->exec1_dst, spro->exec1_alu0, opcode_name[spro->exec1_opcode], spro->exec1_alu1);
            sprn->r[spro->exec1_dst] = spro->exec1_aluout;
        }
        else if (spro->exec1_opcode == SUB) {
            fprintf(inst_trace_fp, ">>>> EXEC: R[%i] = %i %s %i <<<<\n\n", spro->exec1_dst, spro->exec1_alu0, opcode_name[spro->exec1_opcode], spro->exec1_alu1);
            sprn->r[spro->exec1_dst] = spro->exec1_aluout;
        }
        else if (spro->exec1_opcode == LSF) {
            fprintf(inst_trace_fp, ">>>> EXEC: R[%i] = %i %s %i <<<<\n\n", spro->exec1_dst, spro->exec1_alu0, opcode_name[spro->exec1_opcode], spro->exec1_alu1);
            sprn->r[spro->exec1_dst] = spro->exec1_aluout;
        }
        else if (spro->exec1_opcode == RSF) {
            fprintf(inst_trace_fp, ">>>> EXEC: R[%i] = %i %s %i <<<<\n\n", spro->exec1_dst, spro->exec1_alu0, opcode_name[spro->exec1_opcode], spro->exec1_alu1);
            sprn->r[spro->exec1_dst] = spro->exec1_aluout;
        }
        else if (spro->exec1_opcode == AND) {
            fprintf(inst_trace_fp, ">>>> EXEC: R[%i] = %i %s %i <<<<\n\n", spro->exec1_dst, spro->exec1_alu0, opcode_name[spro->exec1_opcode], spro->exec1_alu1);
            sprn->r[spro->exec1_dst] = spro->exec1_aluout;
        }
        else if (spro->exec1_opcode == OR) {
            fprintf(inst_trace_fp, ">>>> EXEC: R[%i] = %i %s %i <<<<\n\n", spro->exec1_dst, spro->exec1_alu0, opcode_name[spro->exec1_opcode], spro->exec1_alu1);
            sprn->r[spro->exec1_dst] = spro->exec1_aluout;
        }
        else if (spro->exec1_opcode == XOR) {
            fprintf(inst_trace_fp, ">>>> EXEC: R[%i] = %i %s %i <<<<\n\n", spro->exec1_dst, spro->exec1_alu0, opcode_name[spro->exec1_opcode], spro->exec1_alu1);
            sprn->r[spro->exec1_dst] = spro->exec1_aluout;
        }
        else if (spro->exec1_opcode == LHI) {
            fprintf(inst_trace_fp, ">>>> EXEC: R[%i] = %i %s %i <<<<\n\n", spro->exec1_dst, spro->exec1_alu0, opcode_name[spro->exec1_opcode], spro->exec1_alu1);
            sprn->r[spro->exec1_dst] = spro->exec1_aluout;
        }
        else if (spro->exec1_opcode == LD) {
            int loaded_mem = llsim_mem_extract_dataout(sp->sramd,31,0);
            fprintf(inst_trace_fp,">>>> EXEC: R[%i] = MEM[%i] = %08x <<<<\n\n", spro->exec1_dst, spro->exec1_alu1, loaded_mem);
            sprn->r[spro->exec1_dst] = loaded_mem;
        }
        else if (spro->exec1_opcode == ST) {
            fprintf(inst_trace_fp,">>>> EXEC: MEM[%i] = R[%i] = %08x <<<<\n\n", (spro->exec1_src1 == 1)? spro->exec1_immediate : spro->r[spro->exec1_src1], spro->exec1_src0, spro->r[spro->exec1_src0]);
            llsim_mem_set_datain(sp->sramd,spro->exec1_alu0,31,0);
			llsim_mem_write(sp->sramd,spro->exec1_alu1);
        }
        else if (spro->exec1_opcode == JLT) {
            if (spro->exec1_aluout == 1) {
                fprintf(inst_trace_fp,">>>> EXEC: %s %i, %i, %i <<<<\n\n", opcode_name[spro->exec1_opcode], spro->r[spro->exec1_src0], spro->r[spro->exec1_src1], spro->exec1_immediate);
                sprn->r[7] = spro->exec1_pc;
                sprn->fetch0_pc = spro->exec1_immediate;
            }
            else {
                fprintf(inst_trace_fp,">>>> EXEC: %s %i, %i, %i <<<<\n\n", opcode_name[spro->exec1_opcode], spro->r[spro->exec1_src0], spro->r[spro->exec1_src1], spro->exec1_pc+1);
            }
        }
        else if (spro->exec1_opcode == JLE) {
            if (spro->exec1_aluout == 1) {
                fprintf(inst_trace_fp,">>>> EXEC: %s %i, %i, %i <<<<\n\n", opcode_name[spro->exec1_opcode], spro->r[spro->exec1_src0], spro->r[spro->exec1_src1], spro->exec1_immediate);
                sprn->r[7] = spro->exec1_pc;
                sprn->fetch0_pc = spro->exec1_immediate;
            }
            else {
                fprintf(inst_trace_fp,">>>> EXEC: %s %i, %i, %i <<<<\n\n", opcode_name[spro->exec1_opcode], spro->r[spro->exec1_src0], spro->r[spro->exec1_src1], spro->exec1_pc+1);
            }
        }
        else if (spro->exec1_opcode == JEQ) {
            if (spro->exec1_aluout == 1) {
                fprintf(inst_trace_fp,">>>> EXEC: %s %i, %i, %i <<<<\n\n", opcode_name[spro->exec1_opcode], spro->r[spro->exec1_src0], spro->r[spro->exec1_src1], spro->exec1_immediate);
                sprn->r[7] = spro->exec1_pc;
                sprn->fetch0_pc = spro->exec1_immediate;
            }
            else {
                fprintf(inst_trace_fp,">>>> EXEC: %s %i, %i, %i <<<<\n\n", opcode_name[spro->exec1_opcode], spro->r[spro->exec1_src0], spro->r[spro->exec1_src1], spro->exec1_pc+1);
            }
        }
        else if (spro->exec1_opcode == JNE) {
            if (spro->exec1_aluout == 1) {
                fprintf(inst_trace_fp,">>>> EXEC: %s %i, %i, %i <<<<\n\n", opcode_name[spro->exec1_opcode], spro->r[spro->exec1_src0], spro->r[spro->exec1_src1], spro->exec1_immediate);
                sprn->r[7] = spro->exec1_pc;
                sprn->fetch0_pc = spro->exec1_immediate;
            }
            else {
                fprintf(inst_trace_fp,">>>> EXEC: %s %i, %i, %i <<<<\n\n", opcode_name[spro->exec1_opcode], spro->r[spro->exec1_src0], spro->r[spro->exec1_src1], spro->exec1_pc+1);
            }
        }
        else if (spro->exec1_opcode == JIN) {
            if (spro->exec1_aluout == 1) {
                fprintf(inst_trace_fp,">>>> EXEC: %s %i, %i, %i <<<<\n\n", opcode_name[spro->exec1_opcode], spro->r[spro->exec1_src0], spro->r[spro->exec1_src1], spro->exec1_immediate);
                sprn->r[7] = spro->exec1_pc;
                sprn->fetch0_pc = spro->exec1_immediate;
            }
            else {
                fprintf(inst_trace_fp,">>>> EXEC: %s %i, %i, %i <<<<\n\n", opcode_name[spro->exec1_opcode], spro->r[spro->exec1_src0], spro->r[spro->exec1_src1], spro->exec1_pc+1);
            }
        }
		else if (spro->exec1_opcode == HLT) {
			llsim_stop();
			fprintf(inst_trace_fp, ">>>> EXEC: HALT at PC %04x<<<<\n", spro->exec1_pc);
            fprintf(inst_trace_fp, "sim finished at pc %i, %i instructions", spro->exec1_pc, inst_cnt);
			dump_sram(sp, "srami_out.txt", sp->srami);
			dump_sram(sp, "sramd_out.txt", sp->sramd);
			sp->start = 0;
		}
	}
}

static void sp_run(llsim_unit_t *unit)
{
	sp_t *sp = (sp_t *) unit->private;
	//	sp_registers_t *spro = sp->spro;
	//	sp_registers_t *sprn = sp->sprn;

	//	llsim_printf("-------------------------\n");

	if (llsim->reset) {
		sp_reset(sp);
		return;
	}

	sp->srami->read = 0;
	sp->srami->write = 0;
	sp->sramd->read = 0;
	sp->sramd->write = 0;

	sp_ctl(sp);
}

static void sp_generate_sram_memory_image(sp_t *sp, char *program_name)
{
        FILE *fp;
        int addr, i;

        fp = fopen(program_name, "r");
        if (fp == NULL) {
                printf("couldn't open file %s\n", program_name);
                exit(1);
        }
        addr = 0;
        while (addr < SP_SRAM_HEIGHT) {
                fscanf(fp, "%08x\n", &sp->memory_image[addr]);
                //              printf("addr %x: %08x\n", addr, sp->memory_image[addr]);
                addr++;
                if (feof(fp))
                        break;
        }
	sp->memory_image_size = addr;

        fprintf(inst_trace_fp, "program %s loaded, %d lines\n\n", program_name, addr);

	for (i = 0; i < sp->memory_image_size; i++) {
		llsim_mem_inject(sp->srami, i, sp->memory_image[i], 31, 0);
		llsim_mem_inject(sp->sramd, i, sp->memory_image[i], 31, 0);
	}
}

void sp_init(char *program_name)
{
	llsim_unit_t *llsim_sp_unit;
	llsim_unit_registers_t *llsim_ur;
	sp_t *sp;

	llsim_printf("initializing sp unit\n");

	inst_trace_fp = fopen("inst_trace.txt", "w");
	if (inst_trace_fp == NULL) {
		printf("couldn't open file inst_trace.txt\n");
		exit(1);
	}

	cycle_trace_fp = fopen("cycle_trace.txt", "w");
	if (cycle_trace_fp == NULL) {
		printf("couldn't open file cycle_trace.txt\n");
		exit(1);
	}

	llsim_sp_unit = llsim_register_unit("sp", sp_run);
	llsim_ur = llsim_allocate_registers(llsim_sp_unit, "sp_registers", sizeof(sp_registers_t));
	sp = llsim_malloc(sizeof(sp_t));
	llsim_sp_unit->private = sp;
	sp->spro = llsim_ur->old;
	sp->sprn = llsim_ur->new;

	sp->srami = llsim_allocate_memory(llsim_sp_unit, "srami", 32, SP_SRAM_HEIGHT, 0);
	sp->sramd = llsim_allocate_memory(llsim_sp_unit, "sramd", 32, SP_SRAM_HEIGHT, 0);
	sp_generate_sram_memory_image(sp, program_name);

	sp->start = 1;
	
	// c2v_translate_end
}
