/*
 * Liron Cohen 207481268
 * Yuval Mor 209011543
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <stdbool.h>

#include "llsim.h"

#define sp_printf(a...)                               \
	do                                                \
	{                                                 \
		llsim_printf("sp: clock %d: ", llsim->clock); \
		llsim_printf(a);                              \
	} while (0)

int nr_simulated_instructions = 0;
FILE *inst_trace_fp = NULL, *cycle_trace_fp = NULL;

// dma states
#define DMA_STATE_IDLE 0
#define DMA_STATE_WAIT 1
#define DMA_STATE_ACTIVE 2

bool is_dma_done = false;
bool is_dma_active = false;

typedef struct sp_registers_s
{
	// 6 32 bit registers (r[0], r[1] don't exist)
	int r[8];

	// 32 bit cycle counter
	int cycle_counter;

	// fetch0
	int fetch0_active; // 1 bit
	int fetch0_pc;	   // 16 bits

	// fetch1
	int fetch1_active; // 1 bit
	int fetch1_pc;	   // 16 bits

	// dec0
	int dec0_active; // 1 bit
	int dec0_pc;	 // 16 bits
	int dec0_inst;	 // 32 bits

	// dec1
	int dec1_active;	// 1 bit
	int dec1_pc;		// 16 bits
	int dec1_inst;		// 32 bits
	int dec1_opcode;	// 5 bits
	int dec1_src0;		// 3 bits
	int dec1_src1;		// 3 bits
	int dec1_dst;		// 3 bits
	int dec1_immediate; // 32 bits

	// exec0
	int exec0_active;	 // 1 bit
	int exec0_pc;		 // 16 bits
	int exec0_inst;		 // 32 bits
	int exec0_opcode;	 // 5 bits
	int exec0_src0;		 // 3 bits
	int exec0_src1;		 // 3 bits
	int exec0_dst;		 // 3 bits
	int exec0_immediate; // 32 bits
	int exec0_alu0;		 // 32 bits
	int exec0_alu1;		 // 32 bits

	// exec1
	int exec1_active;	 // 1 bit
	int exec1_pc;		 // 16 bits
	int exec1_inst;		 // 32 bits
	int exec1_opcode;	 // 5 bits
	int exec1_src0;		 // 3 bits
	int exec1_src1;		 // 3 bits
	int exec1_dst;		 // 3 bits
	int exec1_immediate; // 32 bits
	int exec1_alu0;		 // 32 bits
	int exec1_alu1;		 // 32 bits
	int exec1_aluout;

	// dma
	int dma_source;
	int dma_destination;
	int dma_length;
	int dma_state;
	int dma_remain;
	bool is_dma_busy;

} sp_registers_t;

/*
 * Master structure
 */
typedef struct sp_s
{
	// local srams
#define SP_SRAM_HEIGHT 64 * 1024
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
#define OR 5
#define XOR 6
#define LHI 7
#define LD 8
#define ST 9
#define JLT 16
#define JLE 17
#define JEQ 18
#define JNE 19
#define JIN 20
#define CPY 21
#define POL 22

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

#define branch_hist_SIZE 10
#define PREDICT_STRONG_NT 0
#define PREDICT_WEAK_NT 1
#define PREDICT_WEAK_T 2
#define PREDICT_STRONG_T 3

static char opcode_name[32][4] = {"ADD", "SUB", "LSF", "RSF", "AND", "OR", "XOR", "LHI",
								  "LD", "ST", "U", "U", "U", "U", "U", "U",
								  "JLT", "JLE", "JEQ", "JNE", "JIN", "CPY", "POL", "U",
								  "HLT", "U", "U", "U", "U", "U", "U", "U"};

static int inst_cnt = 0;
static int branch_hist[branch_hist_SIZE];

static void dump_sram(sp_t *sp, char *name, llsim_memory_t *sram)
{
	FILE *fp;
	int i;

	fp = fopen(name, "w");
	if (fp == NULL)
	{
		printf("couldn't open file %s\n", name);
		exit(1);
	}
	for (i = 0; i < SP_SRAM_HEIGHT; i++)
		fprintf(fp, "%08x\n", llsim_mem_extract(sram, i, 31, 0));
	fclose(fp);
}

/* This methods checks if the opcode represents a branch operation */
static bool is_branch_operation(int opcode)
{

	return opcode == JLT || opcode == JLE || opcode == JEQ || opcode == JNE || opcode == JIN;
}

/* This methods checks the branch history and if the branch should be taken, flushed the pipeline */
static void handle_branch_prediction(sp_registers_t *spro, sp_registers_t *sprn) {
	int pc = spro->dec0_pc;
	if (branch_hist[pc % branch_hist_SIZE] > PREDICT_WEAK_NT)
	{ // branch is taken, we need to flush the pipeline
		sprn->fetch0_pc = pc;
		sprn->dec0_active = 0;
		sprn->fetch1_active = 0;
		sprn->fetch0_active = 1;
	}
}

static void handle_dma(sp_t *sp, int is_mem_busy)
{
	if (sp->spro->dma_state == DMA_STATE_IDLE)
	{
		if (is_dma_active && !is_mem_busy)
		{
			sp->sprn->dma_state = DMA_STATE_WAIT;
			sp->sprn->is_dma_busy = 1;
		}
		else
			sp->sprn->dma_state = DMA_STATE_IDLE;
	}
	else if (sp->spro->dma_state == DMA_STATE_WAIT)
	{
		llsim_mem_read(sp->sramd, sp->spro->dma_source);
		sp->sprn->dma_state = DMA_STATE_ACTIVE;
	}
	else if (sp->spro->dma_state == DMA_STATE_ACTIVE)
	{
		int dataout = llsim_mem_extract_dataout(sp->sramd, 31, 0);
		llsim_mem_set_datain(sp->sramd, dataout, 31, 0);
		llsim_mem_write(sp->sramd, sp->spro->dma_destination);

		sp->sprn->dma_remain = sp->spro->dma_remain - 1;
		sp->sprn->dma_destination = sp->spro->dma_destination + 1;
		sp->sprn->dma_source = sp->spro->dma_source + 1;

		if (sp->spro->dma_remain == 1)
		{
			sp->sprn->is_dma_busy = 0;
			sp->sprn->dma_state = DMA_STATE_IDLE;
			is_dma_active = 0;
		}
		else
		{
			if (is_mem_busy)
			{
				sp->sprn->dma_state = DMA_STATE_IDLE;
			}
			else
			{
				sp->sprn->dma_state = DMA_STATE_WAIT;
			}
		}
	}
}

void handle_exec0_dma(sp_registers_t *sprn, sp_registers_t *spro)
{
	if (spro->exec0_opcode == CPY && spro->is_dma_busy == false &&
		(spro->exec1_opcode != CPY || spro->exec1_active == 0))
	{
		// read after write
		if (spro->exec1_active && spro->exec1_dst == spro->exec0_src0 &&
			(spro->exec1_opcode == ADD || spro->exec1_opcode == SUB || spro->exec1_opcode == AND ||
			 spro->exec1_opcode == OR || spro->exec1_opcode == XOR || spro->exec1_opcode == LSF ||
			 spro->exec1_opcode == RSF || spro->exec1_opcode == LHI ||
			 spro->exec1_opcode == CPY || spro->exec1_opcode == POL))
		{
			sprn->dma_source = spro->exec1_aluout;
		}
		else
		{
			sprn->dma_source = spro->r[spro->exec0_src0];
		}

		// read after write
		if (spro->exec1_active && spro->exec1_dst == spro->exec0_src1 &&
			(spro->exec1_opcode == ADD || spro->exec1_opcode == SUB || spro->exec1_opcode == AND ||
			 spro->exec1_opcode == OR || spro->exec1_opcode == XOR || spro->exec1_opcode == LSF ||
			 spro->exec1_opcode == RSF || spro->exec1_opcode == LHI ||
			 spro->exec1_opcode == CPY || spro->exec1_opcode == POL))
		{
			sprn->dma_remain = spro->exec1_aluout;
		}
		else
		{
			sprn->dma_remain = spro->r[spro->exec0_src1];
		}

		sprn->dma_destination = spro->r[spro->exec0_dst];
	}
}

/* This method handles load and store at the same cycle by adding stalls where needed */
static void handle_load_after_store(sp_registers_t *spro, sp_registers_t *sprn) {
	// stalling previous and next instructions
	sprn->fetch1_active = 0;
	sprn->dec1_active = 0;

	// fetch1 should return to inst in fetch0
	sprn->fetch0_active = spro->fetch1_active;
	sprn->fetch0_pc = spro->fetch1_pc;

	// doing the current stage again
	sprn->dec0_pc = spro->dec0_pc;
	sprn->dec0_inst = spro->dec0_inst;
	sprn->dec0_active = spro->dec0_active;
}

/* This method updates the branch history according to branch resolution */
static void update_branch_history(sp_registers_t *spro, sp_registers_t *sprn, bool is_branch_taken) {
	int pc = spro->exec1_pc;
	if (is_branch_taken) { // if branch is taken
		sprn->r[7] = pc;
		switch (branch_hist[pc % branch_hist_SIZE]) {
			case(PREDICT_STRONG_NT):
				branch_hist[pc % branch_hist_SIZE] = PREDICT_WEAK_NT;
				break;
			case(PREDICT_WEAK_NT):
				branch_hist[pc % branch_hist_SIZE] = PREDICT_WEAK_T;
				break;
			case(PREDICT_WEAK_T):
				branch_hist[pc % branch_hist_SIZE] = PREDICT_STRONG_T;
				break;
			case(PREDICT_STRONG_T):
				branch_hist[pc % branch_hist_SIZE] = PREDICT_STRONG_T;
				break;
		}
	}
	else { // if branch is not taken
		switch (branch_hist[pc % branch_hist_SIZE]) {
			case(PREDICT_STRONG_NT):
				branch_hist[pc % branch_hist_SIZE] = PREDICT_STRONG_NT;
				break;
			case(PREDICT_WEAK_NT):
				branch_hist[pc % branch_hist_SIZE] = PREDICT_STRONG_NT;
				break;
			case(PREDICT_WEAK_T):
				branch_hist[pc % branch_hist_SIZE] = PREDICT_WEAK_NT;
				break;
			case(PREDICT_STRONG_T):
				branch_hist[pc % branch_hist_SIZE] = PREDICT_WEAK_T;
				break;
		}
	}
}

/* This method checks if pipeline contains the pc of the instruction 
   That should be executed after branch. If not, needs to flush */
static bool check_if_flush_is_needed(sp_registers_t* spro, int next_pc) {
	// next instruction should has the pc after branch is taken
	// if it's not, we need to flush
	if (spro->fetch0_active == 1 && spro->fetch0_active != next_pc)
	{
		return true;
	}
	else if (spro->fetch1_active == 1 && spro->fetch1_pc != next_pc)
	{
		return true;
	}
	else if (spro->dec0_active == 1 && spro->dec0_active != next_pc)
	{
		return true;
	}
	else if (spro->dec1_active == 1 && spro->dec1_pc != next_pc)
	{
		return true;
	}
	else if (spro->fetch0_active == 1 && spro->fetch0_pc != next_pc)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/* This method decides the value of exec0_alu0 while taking into account
   Bypasses and branch taken */
static void decide_exec0_alu0_value(sp_t *sp, sp_registers_t *spro, sp_registers_t *sprn) {
	if (spro->dec1_src0 == 0) { // r0
		sprn->exec0_alu0 = 0;
	}

	else if (spro->dec1_src0 == 1) { // imm
		sprn->exec0_alu0 = spro->dec1_immediate;
	}

	else if (spro->exec1_active && spro->dec1_src0 == spro->exec1_dst &&
			 (spro->exec1_opcode == ADD || spro->exec1_opcode == SUB || spro->exec1_opcode == LSF ||
			  spro->exec1_opcode == RSF || spro->exec1_opcode == AND || spro->exec1_opcode == OR ||
			  spro->exec1_opcode == XOR || spro->exec1_opcode == LHI || spro->exec1_opcode == POL || spro->exec1_opcode == CPY))
	{ // read after write bypass (ALU)
		sprn->exec0_alu0 = spro->exec1_aluout;
	}

	else if (spro->exec1_active && spro->exec1_opcode == LD && spro->exec1_dst == spro->dec1_src0)
	{ // read after write bypass (MEM)
		sprn->exec0_alu0 = llsim_mem_extract_dataout(sp->sramd, 31, 0);
	}

	else if (spro->exec1_active && spro->exec1_aluout == 1 && spro->dec1_src0 == 7 &&
			 (spro->exec1_opcode == JLT || spro->exec1_opcode == JLE || spro->exec1_opcode == JEQ ||
			  spro->exec1_opcode == JNE || spro->exec1_opcode == JIN))
	{ // branch is taken, need to flush value of r[7]
		sprn->exec0_alu0 = spro->exec1_pc;
	}

	else { // no hazards
		sprn->exec0_alu0 = spro->r[spro->dec1_src0];
	}
}

/* This method decides the value of exec0_alu1 while taking into account
   Bypasses and branch taken */
static void decide_exec0_alu1_value(sp_t *sp, sp_registers_t *spro, sp_registers_t *sprn) {
	if (spro->dec1_src1 == 0) { // r0
		sprn->exec0_alu1 = 0;
	}

	else if (spro->dec1_src1 == 1) { // imm
		sprn->exec0_alu1 = spro->dec1_immediate;
	}

	else if (spro->exec1_active && spro->dec1_src1 == spro->exec1_dst &&
			 (spro->exec1_opcode == ADD || spro->exec1_opcode == SUB || spro->exec1_opcode == LSF ||
			  spro->exec1_opcode == RSF || spro->exec1_opcode == AND || spro->exec1_opcode == OR ||
			  spro->exec1_opcode == XOR || spro->exec1_opcode == LHI || spro->exec1_opcode == POL || spro->exec1_opcode == CPY))
	{ // read after write bypass (ALU)
		sprn->exec0_alu1 = spro->exec1_aluout;
	}

	else if (spro->exec1_active && spro->exec1_opcode == LD && spro->exec1_dst == spro->dec1_src1)
	{ // read after write bypass (MEM)
		sprn->exec0_alu1 = llsim_mem_extract_dataout(sp->sramd, 31, 0);
	}

	else if (spro->exec1_active && spro->exec1_aluout == 1 && spro->dec1_src1 == 7 &&
			 (spro->exec1_opcode == JLT || spro->exec1_opcode == JLE || spro->exec1_opcode == JEQ ||
			  spro->exec1_opcode == JNE || spro->exec1_opcode == JIN))
	{ // branch is taken, need to flush value of r[7]
		sprn->exec0_alu1 = spro->exec1_pc;
	}

	else { // no hazards
		sprn->exec0_alu1 = spro->r[spro->dec1_src1];
	}
}

/* This method decides the value of exec1_aluout according to the opcode */
static int decide_exec1_aluout_value(sp_t *sp, sp_registers_t *spro, sp_registers_t *sprn, int alu0, int alu1) {
	switch (spro->exec0_opcode) {
	case ADD:
		return alu0 + alu1;
	case SUB:
		return alu0 - alu1;
	case LSF:
		return alu0 << alu1;
	case RSF:
		return alu0 >> alu1;
	case AND:
		return alu0 & alu1;
	case OR:
		return alu0 | alu1;
	case XOR:
		return alu0 ^ alu1;
	case LHI:
		return (alu1 << ALU1_SHIFT) + (alu0 & LOWER_16_BITS_MASK);
	case LD:
		llsim_mem_read(sp->sramd, alu1);
		return 0;
	case JLT:
		return (alu0 < alu1) ? 1 : 0;
	case JLE:
		return (alu0 <= alu1) ? 1 : 0;
	case JEQ:
		return (alu0 == alu1) ? 1 : 0;
	case JNE:
		return (alu0 != alu1) ? 1 : 0;
	case JIN:
		return 1;
	case HLT:
		return 0;
	case POL:
		return spro->dma_remain;
	}
	return 0;
}

/* This method decides the value of exec1_alu0 while taking into account
   Bypasses and branch taken */
static void decide_exec1_alu0_value(sp_t *sp, sp_registers_t *spro, sp_registers_t *sprn, int *alu0) {
	if (spro->exec0_src0 != 0 && spro->exec0_src0 != 1) { // not r0 or imm
		if (spro->exec1_active && spro->exec1_dst == spro->exec0_src0 &&
			(spro->exec1_opcode == ADD || spro->exec1_opcode == SUB || spro->exec1_opcode == AND ||
			 spro->exec1_opcode == OR || spro->exec1_opcode == XOR || spro->exec1_opcode == LSF ||
			 spro->exec1_opcode == RSF || spro->exec1_opcode == LHI || spro->exec1_opcode == POL || spro->exec1_opcode == CPY))
		{ // read after write bypass (ALU)
			*alu0 = spro->exec1_aluout;
		}

		else if (spro->exec1_active && spro->exec1_opcode == LD && spro->exec0_src0 == spro->exec1_dst)
		{ // read after write bypass (MEM)
			*alu0 = llsim_mem_extract_dataout(sp->sramd, 31, 0);
		}

		else if (spro->exec1_active && spro->exec0_src0 == 7 &&
				 (spro->exec1_opcode == JLT || spro->exec1_opcode == JLE || spro->exec1_opcode == JEQ ||
				  spro->exec1_opcode == JNE || spro->exec1_opcode == JIN))
		{ // branch is taken, need to flush value of r[7]
			*alu0 = spro->exec1_pc;
		}
	}
}

/* This method decides the value of exec1_alu1 while taking into account
   Bypasses and branch taken */
static void decide_exec1_alu1_value(sp_t *sp, sp_registers_t *spro, sp_registers_t *sprn, int* alu1) {
	if (spro->exec0_src1 != 0 && spro->exec0_src1 != 1) { // not r0 or imm
		if (spro->exec1_active && spro->exec1_dst == spro->exec0_src1 &&
			(spro->exec1_opcode == ADD || spro->exec1_opcode == SUB || spro->exec1_opcode == AND ||
			 spro->exec1_opcode == OR || spro->exec1_opcode == XOR || spro->exec1_opcode == LSF ||
			 spro->exec1_opcode == RSF || spro->exec1_opcode == LHI || spro->exec1_opcode == POL || spro->exec1_opcode == CPY))
		{ // read after write bypass (ALU)
			*alu1 = spro->exec1_aluout;
		}

		else if (spro->exec1_active && spro->exec1_opcode == LD && spro->exec0_src1 == spro->exec1_dst)
		{ // read after write bypass (MEM)
			*alu1 = llsim_mem_extract_dataout(sp->sramd, 31, 0);
		}

		else if (spro->exec1_active && spro->exec0_src1 == 7 &&
				 (spro->exec1_opcode == JLT || spro->exec1_opcode == JLE || spro->exec1_opcode == JEQ ||
				  spro->exec1_opcode == JNE || spro->exec1_opcode == JIN))
		{ // branch is taken, need to flush value of r[7]
			*alu1 = spro->exec1_pc;
		}
	}
}

/* This method prints the instructions trace file */
static void trace_inst_to_file(sp_t *sp, sp_registers_t *spro, sp_registers_t *sprn) {
	fprintf(inst_trace_fp,"--- instruction %i (%04x) @ PC %i (%04x) -----------------------------------------------------------\n", inst_cnt, inst_cnt, spro->exec1_pc, spro->exec1_pc);
	fprintf(inst_trace_fp,"pc = %04d, inst = %08x, opcode = %i (%s), dst = %i, src0 = %i, src1 = %i, immediate = %08x\n", spro->exec1_pc, spro->exec1_inst, spro->exec1_opcode, opcode_name[spro->exec1_opcode],
	spro->exec1_dst, spro->exec1_src0, spro->exec1_src1, sbs(spro->exec1_inst, 15, 0));
	fprintf(inst_trace_fp,"r[0] = 00000000 r[1] = %08x r[2] = %08x r[3] = %08x \n",spro->exec1_immediate, spro->r[2], spro->r[3]);
	fprintf(inst_trace_fp,"r[4] = %08x r[5] = %08x r[6] = %08x r[7] = %08x \n\n", spro->r[4], spro->r[5], spro->r[6], spro->r[7]);
	
	if (spro->exec1_opcode == ADD) {
		fprintf(inst_trace_fp, ">>>> EXEC: R[%i] = %i %s %i <<<<\n\n", spro->exec1_dst, spro->exec1_alu0, opcode_name[spro->exec1_opcode], spro->exec1_alu1);
		sprn->r[spro->exec1_dst] = spro->exec1_aluout;
	}
	else if (spro->exec1_opcode == SUB)
	{
		fprintf(inst_trace_fp, ">>>> EXEC: R[%i] = %i %s %i <<<<\n\n", spro->exec1_dst, spro->exec1_alu0, opcode_name[spro->exec1_opcode], spro->exec1_alu1);
		sprn->r[spro->exec1_dst] = spro->exec1_aluout;
	}
	else if (spro->exec1_opcode == LSF)
	{
		fprintf(inst_trace_fp, ">>>> EXEC: R[%i] = %i %s %i <<<<\n\n", spro->exec1_dst, spro->exec1_alu0, opcode_name[spro->exec1_opcode], spro->exec1_alu1);
		sprn->r[spro->exec1_dst] = spro->exec1_aluout;
	}
	else if (spro->exec1_opcode == RSF)
	{
		fprintf(inst_trace_fp, ">>>> EXEC: R[%i] = %i %s %i <<<<\n\n", spro->exec1_dst, spro->exec1_alu0, opcode_name[spro->exec1_opcode], spro->exec1_alu1);
		sprn->r[spro->exec1_dst] = spro->exec1_aluout;
	}
	else if (spro->exec1_opcode == AND)
	{
		fprintf(inst_trace_fp, ">>>> EXEC: R[%i] = %i %s %i <<<<\n\n", spro->exec1_dst, spro->exec1_alu0, opcode_name[spro->exec1_opcode], spro->exec1_alu1);
		sprn->r[spro->exec1_dst] = spro->exec1_aluout;
	}
	else if (spro->exec1_opcode == OR)
	{
		fprintf(inst_trace_fp, ">>>> EXEC: R[%i] = %i %s %i <<<<\n\n", spro->exec1_dst, spro->exec1_alu0, opcode_name[spro->exec1_opcode], spro->exec1_alu1);
		sprn->r[spro->exec1_dst] = spro->exec1_aluout;
	}
	else if (spro->exec1_opcode == XOR)
	{
		fprintf(inst_trace_fp, ">>>> EXEC: R[%i] = %i %s %i <<<<\n\n", spro->exec1_dst, spro->exec1_alu0, opcode_name[spro->exec1_opcode], spro->exec1_alu1);
		sprn->r[spro->exec1_dst] = spro->exec1_aluout;
	}
	else if (spro->exec1_opcode == LHI)
	{
		fprintf(inst_trace_fp, ">>>> EXEC: R[%i] = %i %s %i <<<<\n\n", spro->exec1_dst, spro->exec1_alu0, opcode_name[spro->exec1_opcode], spro->exec1_alu1);
		sprn->r[spro->exec1_dst] = spro->exec1_aluout;
	}
	else if (spro->exec1_opcode == LD)
	{
		int loaded_mem = llsim_mem_extract_dataout(sp->sramd, 31, 0);
		fprintf(inst_trace_fp, ">>>> EXEC: R[%i] = MEM[%i] = %08x <<<<\n\n", spro->exec1_dst, spro->exec1_alu1, loaded_mem);
		sprn->r[spro->exec1_dst] = loaded_mem;
	}
	else if (spro->exec1_opcode == ST)
	{
		fprintf(inst_trace_fp, ">>>> EXEC: MEM[%i] = R[%i] = %08x <<<<\n\n", (spro->exec1_src1 == 1) ? spro->exec1_immediate : spro->r[spro->exec1_src1], spro->exec1_src0, spro->r[spro->exec1_src0]);
	}
	else if (spro->exec1_opcode == JLT)
	{
		if (spro->exec1_aluout == 1)
		{
			fprintf(inst_trace_fp, ">>>> EXEC: %s %i, %i, %i <<<<\n\n", opcode_name[spro->exec1_opcode], spro->r[spro->exec1_src0], spro->r[spro->exec1_src1], spro->exec1_immediate);
			sprn->r[7] = spro->exec1_pc;
			sprn->fetch0_pc = spro->exec1_immediate;
		}
		else
		{
			fprintf(inst_trace_fp, ">>>> EXEC: %s %i, %i, %i <<<<\n\n", opcode_name[spro->exec1_opcode], spro->r[spro->exec1_src0], spro->r[spro->exec1_src1], spro->exec1_pc + 1);
		}
	}
	else if (spro->exec1_opcode == JLE)
	{
		if (spro->exec1_aluout == 1)
		{
			fprintf(inst_trace_fp, ">>>> EXEC: %s %i, %i, %i <<<<\n\n", opcode_name[spro->exec1_opcode], spro->r[spro->exec1_src0], spro->r[spro->exec1_src1], spro->exec1_immediate);
			sprn->r[7] = spro->exec1_pc;
			sprn->fetch0_pc = spro->exec1_immediate;
		}
		else
		{
			fprintf(inst_trace_fp, ">>>> EXEC: %s %i, %i, %i <<<<\n\n", opcode_name[spro->exec1_opcode], spro->r[spro->exec1_src0], spro->r[spro->exec1_src1], spro->exec1_pc + 1);
		}
	}
	else if (spro->exec1_opcode == JEQ)
	{
		if (spro->exec1_aluout == 1)
		{
			fprintf(inst_trace_fp, ">>>> EXEC: %s %i, %i, %i <<<<\n\n", opcode_name[spro->exec1_opcode], spro->r[spro->exec1_src0], spro->r[spro->exec1_src1], spro->exec1_immediate);
			sprn->r[7] = spro->exec1_pc;
			sprn->fetch0_pc = spro->exec1_immediate;
		}
		else
		{
			fprintf(inst_trace_fp, ">>>> EXEC: %s %i, %i, %i <<<<\n\n", opcode_name[spro->exec1_opcode], spro->r[spro->exec1_src0], spro->r[spro->exec1_src1], spro->exec1_pc + 1);
		}
	}
	else if (spro->exec1_opcode == JNE)
	{
		if (spro->exec1_aluout == 1)
		{
			fprintf(inst_trace_fp, ">>>> EXEC: %s %i, %i, %i <<<<\n\n", opcode_name[spro->exec1_opcode], spro->r[spro->exec1_src0], spro->r[spro->exec1_src1], spro->exec1_immediate);
			sprn->r[7] = spro->exec1_pc;
			sprn->fetch0_pc = spro->exec1_immediate;
		}
		else
		{
			fprintf(inst_trace_fp, ">>>> EXEC: %s %i, %i, %i <<<<\n\n", opcode_name[spro->exec1_opcode], spro->r[spro->exec1_src0], spro->r[spro->exec1_src1], spro->exec1_pc + 1);
		}
	}
	else if (spro->exec1_opcode == JIN)
	{
		if (spro->exec1_aluout == 1)
		{
			fprintf(inst_trace_fp, ">>>> EXEC: %s %i, %i, %i <<<<\n\n", opcode_name[spro->exec1_opcode], spro->r[spro->exec1_src0], spro->r[spro->exec1_src1], spro->exec1_immediate);
			sprn->r[7] = spro->exec1_pc;
			sprn->fetch0_pc = spro->exec1_immediate;
		}
		else
		{
			fprintf(inst_trace_fp, ">>>> EXEC: %s %i, %i, %i <<<<\n\n", opcode_name[spro->exec1_opcode], spro->r[spro->exec1_src0], spro->r[spro->exec1_src1], spro->exec1_pc + 1);
		}
	}
	else if (spro->exec1_opcode == HLT)
	{
		fprintf(inst_trace_fp, ">>>> EXEC: HALT at PC %04x<<<<\n", spro->exec1_pc);
	}
	else if (spro->exec1_opcode == POL)
	{
		fprintf(inst_trace_fp, ">>>> EXEC: POL - Remaining copy: %i <<<<\n\n", spro->dma_remain);
	}
	else if (spro->exec1_opcode == CPY)
	{
		fprintf(inst_trace_fp, ">>>> EXEC: CPY - Source address: %i, Destination address: %i, length: %i <<<<\n\n", spro->dma_source, spro->dma_destination, spro->dma_length);
	}
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
	fprintf(cycle_trace_fp, "dec1_pc %08x\n", spro->dec1_pc);				// 16 bits
	fprintf(cycle_trace_fp, "dec1_inst %08x\n", spro->dec1_inst);			// 32 bits
	fprintf(cycle_trace_fp, "dec1_opcode %08x\n", spro->dec1_opcode);		// 5 bits
	fprintf(cycle_trace_fp, "dec1_src0 %08x\n", spro->dec1_src0);			// 3 bits
	fprintf(cycle_trace_fp, "dec1_src1 %08x\n", spro->dec1_src1);			// 3 bits
	fprintf(cycle_trace_fp, "dec1_dst %08x\n", spro->dec1_dst);				// 3 bits
	fprintf(cycle_trace_fp, "dec1_immediate %08x\n", spro->dec1_immediate); // 32 bits

	fprintf(cycle_trace_fp, "exec0_active %08x\n", spro->exec0_active);
	fprintf(cycle_trace_fp, "exec0_pc %08x\n", spro->exec0_pc);				  // 16 bits
	fprintf(cycle_trace_fp, "exec0_inst %08x\n", spro->exec0_inst);			  // 32 bits
	fprintf(cycle_trace_fp, "exec0_opcode %08x\n", spro->exec0_opcode);		  // 5 bits
	fprintf(cycle_trace_fp, "exec0_src0 %08x\n", spro->exec0_src0);			  // 3 bits
	fprintf(cycle_trace_fp, "exec0_src1 %08x\n", spro->exec0_src1);			  // 3 bits
	fprintf(cycle_trace_fp, "exec0_dst %08x\n", spro->exec0_dst);			  // 3 bits
	fprintf(cycle_trace_fp, "exec0_immediate %08x\n", spro->exec0_immediate); // 32 bits
	fprintf(cycle_trace_fp, "exec0_alu0 %08x\n", spro->exec0_alu0);			  // 32 bits
	fprintf(cycle_trace_fp, "exec0_alu1 %08x\n", spro->exec0_alu1);			  // 32 bits

	fprintf(cycle_trace_fp, "exec1_active %08x\n", spro->exec1_active);
	fprintf(cycle_trace_fp, "exec1_pc %08x\n", spro->exec1_pc);				  // 16 bits
	fprintf(cycle_trace_fp, "exec1_inst %08x\n", spro->exec1_inst);			  // 32 bits
	fprintf(cycle_trace_fp, "exec1_opcode %08x\n", spro->exec1_opcode);		  // 5 bits
	fprintf(cycle_trace_fp, "exec1_src0 %08x\n", spro->exec1_src0);			  // 3 bits
	fprintf(cycle_trace_fp, "exec1_src1 %08x\n", spro->exec1_src1);			  // 3 bits
	fprintf(cycle_trace_fp, "exec1_dst %08x\n", spro->exec1_dst);			  // 3 bits
	fprintf(cycle_trace_fp, "exec1_immediate %08x\n", spro->exec1_immediate); // 32 bits
	fprintf(cycle_trace_fp, "exec1_alu0 %08x\n", spro->exec1_alu0);			  // 32 bits
	fprintf(cycle_trace_fp, "exec1_alu1 %08x\n", spro->exec1_alu1);			  // 32 bits
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
	if (spro->fetch0_active)
	{ // reading current instruction from memory
		if (!is_dma_done)
		{
			llsim_mem_read(sp->srami, spro->fetch0_pc);					  // fetching the current instruction from SRAMI
			sprn->fetch0_pc = (spro->fetch0_pc + 1) & LOWER_16_BITS_MASK; // updating to next pc
			sprn->fetch1_pc = spro->fetch0_pc;							  // moving pc value in pipeline
		}
		sprn->fetch1_active = 1; // activating next stage
	}

	// fetch1
	if (spro->fetch1_active)
	{ // sampling memoty output to the instruction register
		if (!is_dma_done)
		{
			sprn->dec0_pc = spro->fetch1_pc; // setting pc for next stage
			sprn->dec0_inst = llsim_mem_extract_dataout(sp->srami, 31, 0);
		}
		sprn->dec0_active = 1;
	}
	else
	{ // fetch1 is not active
		sprn->dec0_active = 0;
	}

	// dec0
	if (spro->dec0_active)
	{ // decoding instruction
		if (!is_dma_done)
		{
			int opcode = (spro->dec0_inst & OPCODE_MASK) >> OPCODE_SHIFT;
			if (opcode == JLT || opcode == JLE || opcode == JEQ || opcode == JNE)
			{ // branch prediction
				handle_branch_prediction(spro, sprn);
			}

			if (opcode == LD && spro->dec1_opcode == ST && spro->dec1_active)
			{ // load after store, RAW hazard
				handle_load_after_store(spro, sprn);
			}
			else
			{
				sprn->dec1_opcode = (spro->dec0_inst & OPCODE_MASK) >> OPCODE_SHIFT;
				sprn->dec1_dst = (spro->dec0_inst & DST_MASK) >> DST_SHIFT;
				sprn->dec1_src0 = (spro->dec0_inst & SRC0_MASK) >> SRC0_SHIFT;
				sprn->dec1_src1 = (spro->dec0_inst & SRC1_MASK) >> SRC1_SHIFT;
				sprn->dec1_immediate = (spro->dec0_inst) & IMM_MASK;
				if ((spro->dec0_inst & SIGN_EXT_MASK) != 0)
				{ // need sign extension with msb 1
					sprn->dec1_immediate = sprn->dec1_immediate + (SIGN_EXT);
				}

				sprn->dec1_inst = spro->dec0_inst;
				sprn->dec1_pc = spro->dec0_pc;
				sprn->dec1_active = 1;
			}
		}
		else
		{
			sprn->dec1_active = 1;
		}
	}
	else
	{ // dec0 is not active
		sprn->dec1_active = 0;
	}

	// dec1
	if (spro->dec1_active)
	{ // preparing ALU operands
		if (!is_dma_done)
		{
			decide_exec0_alu0_value(sp, spro, sprn);
			decide_exec0_alu1_value(sp, spro, sprn);

			if (spro->dec1_opcode == LHI)
			{
				sprn->exec0_alu1 = spro->dec1_immediate;
			}

			// moving instruction values in pipeline
			sprn->exec0_pc = spro->dec1_pc;
			sprn->exec0_inst = spro->dec1_inst;
			sprn->exec0_opcode = spro->dec1_opcode;
			sprn->exec0_dst = spro->dec1_dst;
			sprn->exec0_src0 = spro->dec1_src0;
			sprn->exec0_src1 = spro->dec1_src1;
			sprn->exec0_immediate = spro->dec1_immediate;
		}
		sprn->exec0_active = 1;
	}
	else
	{ // dec1 is not active
		sprn->exec0_active = 0;
	}

	// exec0
	if (spro->exec0_active)
	{ // executing ALU and LD operations
		int alu0 = spro->exec0_alu0;
		decide_exec1_alu0_value(sp, spro, sprn, &alu0);
		int alu1 = spro->exec0_alu1;
		decide_exec1_alu1_value(sp, spro, sprn, &alu1);
		if (spro->exec0_opcode != CPY)
		{
			int aluout = decide_exec1_aluout_value(sp, spro, sprn, alu0, alu1);
			sprn->exec1_aluout = aluout;
		}
		handle_exec0_dma(sprn, spro);
		// moving instruction values in pipeline
		sprn->exec1_pc = spro->exec0_pc;
		sprn->exec1_inst = spro->exec0_inst;
		sprn->exec1_opcode = spro->exec0_opcode;
		sprn->exec1_dst = spro->exec0_dst;
		sprn->exec1_src0 = spro->exec0_src0;
		sprn->exec1_src1 = spro->exec0_src1;
		sprn->exec1_immediate = spro->exec0_immediate;
		sprn->exec1_alu0 = alu0;
		sprn->exec1_alu1 = alu1;

		sprn->exec1_active = 1;
	}
	else
	{ // exec0 is not active
		sprn->exec1_active = 0;
	}

	// exec1
	if (spro->exec1_active)
	{ // writing back
		trace_inst_to_file(sp, spro, sprn);

		inst_cnt = inst_cnt + 1;

		if (spro->exec1_opcode == HLT || is_dma_done)
		{
			if (spro->dma_remain > 0)
			{
				is_dma_done = true;
			}
			else
			{
				is_dma_done = false;
				llsim_stop();
				fprintf(inst_trace_fp, "sim finished at pc %i, %i instructions", spro->exec1_pc, inst_cnt);
				dump_sram(sp, "srami_out.txt", sp->srami);
				dump_sram(sp, "sramd_out.txt", sp->sramd);
				sp->start = 0;
			}
		}

		else if (spro->exec1_opcode == ST)
		{ // executing ST
			llsim_mem_set_datain(sp->sramd, spro->exec1_alu0, 31, 0);
			llsim_mem_write(sp->sramd, spro->exec1_alu1);
		}

		else if (spro->exec1_opcode == LD) // executing LD
		{
			if (spro->exec1_dst != 0 && spro->exec1_dst != 1)
				sprn->r[spro->exec1_dst] = llsim_mem_extract_dataout(sp->sramd, 31, 0);
		}

		else if (is_branch_operation(spro->exec1_opcode))
		{ // checks if branch is taken and updates the next pc
			bool is_branch_taken = false;
			int next_pc;

			if (spro->exec1_opcode == JIN) { // always taken
                next_pc = spro->exec1_alu0 & LOWER_16_BITS_MASK;
                is_branch_taken = true;
            }
            else // a different branch opcode, taken depends on aluout
            {
                if (spro->exec1_aluout) {
                    next_pc = spro->exec1_immediate & LOWER_16_BITS_MASK;
                    is_branch_taken = true;
                }
                else {
                    next_pc = (spro->exec1_pc + 1) & LOWER_16_BITS_MASK;
				}
			}

            // Updating the branch history according to the prediciton state machine
			update_branch_history(spro, sprn, is_branch_taken);

			bool is_flush_needed = check_if_flush_is_needed(spro, next_pc);
			if (is_flush_needed)
			{ // flushing
				sprn->fetch0_active = 1;
				sprn->dec0_active = 0;
				sprn->exec0_active = 0;
				sprn->fetch1_active = 0;
				sprn->dec1_active = 0;
				sprn->exec1_active = 0;
				sprn->fetch0_pc = next_pc;
			}
		}

		else if (spro->exec1_dst != 0 && spro->exec1_dst != 1)
		{ // WB to register
			sprn->r[spro->exec1_dst] = spro->exec1_aluout;
		}
	}

	if (spro->exec1_opcode == CPY)
	{
		is_dma_active = true;
	}

	if (!is_dma_done)
	{
		int is_mem_busy = 1;
		if (sprn->dec1_opcode != LD && sprn->dec1_opcode != ST &&
			sprn->exec0_opcode != LD && sprn->exec0_opcode != ST &&
			sprn->exec1_opcode != LD && sprn->exec1_opcode != ST)
		{
			is_mem_busy = 0;
		}
		handle_dma(sp, is_mem_busy);
	}
	else
	{
		handle_dma(sp, 0);
	}
}

static void sp_run(llsim_unit_t *unit)
{
	sp_t *sp = (sp_t *)unit->private;
	//	sp_registers_t *spro = sp->spro;
	//	sp_registers_t *sprn = sp->sprn;

	//	llsim_printf("-------------------------\n");

	if (llsim->reset)
	{
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
	if (fp == NULL)
	{
		printf("couldn't open file %s\n", program_name);
		exit(1);
	}
	addr = 0;
	while (addr < SP_SRAM_HEIGHT)
	{
		fscanf(fp, "%08x\n", &sp->memory_image[addr]);
		//              printf("addr %x: %08x\n", addr, sp->memory_image[addr]);
		addr++;
		if (feof(fp))
			break;
	}
	sp->memory_image_size = addr;

	fprintf(inst_trace_fp, "program %s loaded, %d lines\n\n", program_name, addr);

	for (i = 0; i < sp->memory_image_size; i++)
	{
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
	if (inst_trace_fp == NULL)
	{
		printf("couldn't open file inst_trace.txt\n");
		exit(1);
	}

	cycle_trace_fp = fopen("cycle_trace.txt", "w");
	if (cycle_trace_fp == NULL)
	{
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
