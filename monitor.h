#ifndef EMUMON_H
#define EMUMON_H

// Breakpoint types
enum brkpt_nums {
  BRK_EMPTY,
  BRK_WRITE,
  BRK_INST
};

// monitor.c
void set_breakpoint(int addr, int type);
int is_breakpoint(int addr, int type);
int parse_addr(char *addr, int *issym);
void monitor_init(void);
int monitor(int addr);

// external functions
extern void init_term(void);
extern void reset_term(void);
// extern void attach_sigalrm(void);
// extern void detach_sigalrm(void);
extern void print_regs(FILE *fh);

#endif
