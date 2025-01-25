#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/select.h>
#include <signal.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <m68k.h>
#include <m68kcpu.h>
#include <arpa/inet.h>
#include "ide.h"
#include "duart.h"
#include "mapfile.h"
#include "monitor.h"

/* Emulator for the Rosco r2 m68k SBC:
 * https://store.rosco-m68k.com/products/rosco-m68k-classic-v2-full-kit
 */

/* 1MB RAM at 0x00000, I/O at 0xf00000 */
/* DUART from 0x00f00000 to 0x00f0001f */

#define RAM_SIZE (1<<20)

#define DUART_START	0x00f00000
#define DUART_END	0x00f0001f

/* Executables get loaded at this address by the ROM. */
/* The kernel will relocate itself to a lower address. */
#define DEFAULT_ADDRESS 0x40000

static uint8_t ram[RAM_SIZE];
/* IDE controller */
static struct ide_controller *ide;
/* 68681 */
static struct duart *duart;

static int trace = 0;

/* If 1, we hit a write breakpoint */
static unsigned write_brkpt = 0;

#define TRACE_MEM	1
#define TRACE_CPU	2
#define TRACE_DUART	4

uint8_t fc;

/* Read/write macros */
#define READ_BYTE(BASE, ADDR) (BASE)[ADDR]
#define READ_WORD(BASE, ADDR) (((BASE)[ADDR]<<8) | \
			(BASE)[(ADDR)+1])
#define READ_LONG(BASE, ADDR) (((BASE)[ADDR]<<24) | \
			((BASE)[(ADDR)+1]<<16) | \
			((BASE)[(ADDR)+2]<<8) | \
			(BASE)[(ADDR)+3])

#define WRITE_BYTE(BASE, ADDR, VAL) (BASE)[ADDR] = (VAL)&0xff
#define WRITE_WORD(BASE, ADDR, VAL) (BASE)[ADDR] = ((VAL)>>8) & 0xff; \
			(BASE)[(ADDR)+1] = (VAL)&0xff
#define WRITE_LONG(BASE, ADDR, VAL) (BASE)[ADDR] = ((VAL)>>24) & 0xff; \
			(BASE)[(ADDR)+1] = ((VAL)>>16)&0xff; \
			(BASE)[(ADDR)+2] = ((VAL)>>8)&0xff; \
			(BASE)[(ADDR)+3] = (VAL)&0xff


unsigned int check_chario(void)
{
	fd_set i, o;
	struct timeval tv;
	unsigned int r = 0;

	FD_ZERO(&i);
	FD_SET(0, &i);
	FD_ZERO(&o);
	FD_SET(1, &o);
	tv.tv_sec = 0;
	tv.tv_usec = 0;

	if (select(2, &i, &o, NULL, &tv) == -1) {
		perror("select");
		exit(1);
	}
	if (FD_ISSET(0, &i))
		r |= 1;
	if (FD_ISSET(1, &o))
		r |= 2;
	return r;
}

unsigned int next_char(void)
{
	char c;
	if (read(0, &c, 1) != 1) {
		printf("(tty read without ready byte)\n");
		return 0xFF;
	}
	return c;
}

static unsigned int irq_pending;

void recalc_interrupts(void)
{
	int i;

	/* Duart on IPL1 */
	if (duart_irq_pending(duart))
		irq_pending |= (1 << 2);
	else
		irq_pending &= ~(1 << 2);

	if (irq_pending) {
		for (i = 7; i >= 0; i--) {
			if (irq_pending & (1 << i)) {
				m68k_set_irq(i);
				return;
			}
		}
	} else
		m68k_set_irq(0);
}

int cpu_irq_ack(int level)
{
	if (!(irq_pending & (1 << level)))
		return M68K_INT_ACK_SPURIOUS;
	if (level == 2)
		return duart_vector(duart);
	return M68K_INT_ACK_SPURIOUS;
}


/* TODO v2 board added an RTC */

static unsigned int do_io_readb(unsigned int address)
{
	/* SPI is not modelled */
	if (address >= 0xFFD000 && address <= 0xFFDFFF)
		return 0xFF;
	/* ATA CF */
	/* FIXME: FFE010-01F sets CS1 */
	if (address >= 0xFFE000 && address <= 0xFFEFFF)
		return ide_read8(ide, (address & 31) >> 1);
	/* DUART */
	if (address >= DUART_START && address <= DUART_END)
	  return duart_read(duart, address >> 1);
	return 0x00;
}

static void do_io_writeb(unsigned int address, unsigned int value)
{
	if (address == 0xFFFFFF) {
		printf("<%c>", value);
		return;
	}
	/* SPI is not modelled */
	if (address >= 0xFFD000 && address <= 0xFFDFFF)
		return;
	/* ATA CF */
	if (address >= 0xFFE000 && address <= 0xFFEFFF) {
		ide_write8(ide, (address & 31) >> 1, value);
		return;
	}
	/* DUART */
	if (address >= DUART_START && address <= DUART_END)
		duart_write(duart, address >> 1, value);
}

/* Read data from RAM or a device */
unsigned int do_cpu_read_byte(unsigned int address)
{
	address &= 0xFFFFFF;
	if (address < sizeof(ram))
		return ram[address];
	return do_io_readb(address);
}

unsigned int cpu_read_byte(unsigned int address)
{
	unsigned int v = do_cpu_read_byte(address);
	if (trace & TRACE_MEM)
		fprintf(stderr, "RB %06X -> %02X\n", address, v);
	return v;
}


unsigned int do_cpu_read_word(unsigned int address)
{
	address &= 0xFFFFFF;

	if (address < sizeof(ram) - 1)
		return READ_WORD(ram, address);
	else if (address >= 0xFFE000 && address <= 0xFFEFFF)
		return ide_read16(ide, (address & 31) >> 1);
	return (do_cpu_read_byte(address) << 8) | do_cpu_read_byte(address + 1);
}

unsigned int cpu_read_word(unsigned int address)
{
	unsigned int v = do_cpu_read_word(address);
	if (trace & TRACE_MEM)
		fprintf(stderr, "RW %06X -> %04X\n", address, v);
	return v;
}

unsigned int cpu_read_word_dasm(unsigned int address)
{
	if (address < 0xFF7FFF)
		return cpu_read_word(address);
	else
		return 0xFFFF;
}

unsigned int cpu_read_long(unsigned int address)
{
	return (cpu_read_word(address) << 16) | cpu_read_word(address + 2);
}

unsigned int cpu_read_long_dasm(unsigned int address)
{
	return (cpu_read_word_dasm(address) << 16) | cpu_read_word_dasm(address + 2);
}

void cpu_write_byte(unsigned int address, unsigned int value)
{
	address &= 0xFFFFFF;

	if (trace & TRACE_MEM)
		fprintf(stderr, "WB %06X <- %02X\n", address, value);

	if (address < sizeof(ram))
		ram[address] = value;
	else
		do_io_writeb(address, (value & 0xFF));
}

void cpu_write_word(unsigned int address, unsigned int value)
{
	address &= 0xFFFFFF;

	if (trace & TRACE_MEM)
		fprintf(stderr, "WW %06X <- %04X\n", address, value);

	if (address < sizeof(ram) - 1) {
		WRITE_WORD(ram, address, value);
	} else if (address >= 0xFFE000 && address <= 0xFFEFFF)
		ide_write16(ide, (address & 31) >> 1, value);
	else {
		/* Corner cases */
		cpu_write_byte(address, value >> 8);
		cpu_write_byte(address + 1, value & 0xFF);
	}
}

void cpu_write_long(unsigned int address, unsigned int value)
{
	address &= 0xFFFFFF;

	cpu_write_word(address, value >> 16);
	cpu_write_word(address + 2, value & 0xFFFF);
}

void cpu_write_pd(unsigned int address, unsigned int value)
{
	address &= 0xFFFFFF;

	cpu_write_word(address + 2, value & 0xFFFF);
	cpu_write_word(address, value >> 16);
}

void cpu_instr_callback(void)
{
	if (trace & TRACE_CPU) {
		char buf[128];
		unsigned int pc = m68k_get_reg(NULL, M68K_REG_PC);
		m68k_disassemble(buf, pc, M68K_CPU_TYPE_68000);
		fprintf(stderr, ">%06X %s\n", pc, buf);
	}
}

static void device_init(void)
{
	irq_pending = 0;
	ide_reset_begin(ide);
	duart_reset(duart);
	duart_set_input(duart, 1);
}

static struct termios saved_term, term;

void reset_term(void)
{
	tcsetattr(0, 0, &saved_term);
}

static void cleanup(int sig)
{
	reset_term();
	exit(1);
}

static void exit_cleanup(void)
{
	reset_term();
}

void init_term(void)
{
	if (tcgetattr(0, &term) == 0) {
		saved_term = term;
		atexit(exit_cleanup);
		signal(SIGINT, SIG_IGN);
		signal(SIGQUIT, cleanup);
		signal(SIGTSTP, SIG_IGN);
		term.c_lflag &= ~ICANON;
		term.c_iflag &= ~(ICRNL | IGNCR);
		term.c_cc[VMIN] = 1;
		term.c_cc[VTIME] = 0;
		term.c_cc[VINTR] = 0;
		term.c_cc[VSUSP] = 0;
		term.c_cc[VEOF] = 0;
		term.c_lflag &= ~(ECHO | ECHOE | ECHOK);
		tcsetattr(0, 0, &term);
	}

}

/* The following is used by the monitor */

// Given a filehandle, print the register
// values to the filehandle
void print_regs(FILE * fh) {
  fprintf(fh, "D0-D7: %08X %08X %08X %08X %08X %08X %08X %08X\n",
          m68ki_cpu.dar[0], m68ki_cpu.dar[1], m68ki_cpu.dar[2],
          m68ki_cpu.dar[3], m68ki_cpu.dar[4], m68ki_cpu.dar[5],
          m68ki_cpu.dar[6], m68ki_cpu.dar[7]);
  fprintf(fh, "A0-A7: %08X %08X %08X %08X %08X %08X %08X %08X\n",
          m68ki_cpu.dar[8], m68ki_cpu.dar[9], m68ki_cpu.dar[10],
          m68ki_cpu.dar[11], m68ki_cpu.dar[12], m68ki_cpu.dar[13],
          m68ki_cpu.dar[14], m68ki_cpu.dar[15]);
  fprintf(fh, "PC:    %08X  VBR:    %08X                                ",
          REG_PC, REG_VBR);
  fprintf(fh, "USP: %08X\n", REG_USP);
  fprintf(fh, "SFC:        %03X  DFC:         %03X\n",
          REG_SFC, REG_DFC);
  fprintf(fh, "Status: mode %c, int %d, %c%c%c%c\n",
          (FLAG_S) ? 'S' : 'U',
          FLAG_INT_MASK,
          (FLAG_N) ? 'N' : ' ',
          (FLAG_Z) ? 'Z' : ' ', (FLAG_V) ? 'V' : ' ', (FLAG_C) ? 'C' : ' ');
  fprintf(fh, "\n");
}

/* End of code used by the monitor */

#if 0
static void take_a_nap(void)
{
	struct timespec t;
	t.tv_sec = 0;
	t.tv_nsec = 100000;
	if (nanosleep(&t, NULL))
		perror("nanosleep");
}
#endif

void cpu_pulse_reset(void)
{
	device_init();
}

void cpu_set_fc(int fc)
{
}

void usage(void)
{
	fprintf(stderr, "rosco-r2 [-0][-1][-2][-e][-m][-M mapfile][-i idepath][-d debug] fuzix.bin\n");
	exit(1);
}

int main(int argc, char *argv[])
{
	int fd, cnt;
	int pc;
	int i, brkcnt = 0;
	int duart_cnt=0;
	int breakpoint;
	int cputype = M68K_CPU_TYPE_68000;
	int opt;
	uint8_t *ptr;
	const char *diskname = "rosco-r2.ide";
	int start_in_monitor = 0;
	char **brkstr;                /* Array of breakpoint strings */

  	// Create an array to hold any breakpoint string pointers
  	brkstr = (char **) malloc(argc * sizeof(char *));
  	if (brkstr == NULL) {
    		perror("brkstr malloc");
		exit(1);
	}

	while((opt = getopt(argc, argv, "mb:M:d:i:")) != -1) {
		switch(opt) {
   		case 'm':
      			start_in_monitor = 1;
      			break;
    		case 'b':
      			/* Cache the pointer for now */
      			brkstr[brkcnt++] = optarg;
      			break;
   		case 'M':
      			read_mapfile(optarg);
      			break;
		case 'd':
			trace = atoi(optarg);
			break;
		case 'i':
			diskname = optarg;
			break;
		default:
			usage();
		}
	}

	init_term();

        if (optind == argc)
                usage();

        /* Fill RAM with 0xA7 */
        memset(ram, 0xA7, sizeof(ram));

  	/* Initialise the monitor */
  	monitor_init();

  	/* Now that we might have a map file, */
  	/* parse any breakpoint strings and set them */
  	for (i = 0; i < brkcnt; i++) {
    	    breakpoint = parse_addr(brkstr[i], NULL);
    	    if (breakpoint != -1)
            	set_breakpoint(breakpoint, BRK_INST);
  	}

        /* Load fuzix.img at the DEFAULT_ADDRESS */
        fd = open(argv[optind], O_RDONLY);
        if (fd == -1) {
                perror(argv[optind]);
                exit(1);
        }
        ptr= &ram[DEFAULT_ADDRESS];
        while ((cnt=read(fd, ptr, 4096))>0)
                ptr += cnt;
        close(fd);

        /* We start directly in the executable */
        /* without running any ROM code */
        uint32_t be_start = htobe32(DEFAULT_ADDRESS);
        uint32_t be_stkptr = htobe32(RAM_SIZE);
        memcpy(ram, (void *) &be_stkptr, 4);
        memcpy(&(ram[4]), (void *) &be_start, 4);

	fd = open(diskname, O_RDWR);
	if (fd == -1) {
		perror(diskname);
		exit(1);
	}
	ide = ide_allocate("hd0");
	if (ide == NULL)
		exit(1);
	if (ide_attach(ide, 0, fd))
		exit(1);

	duart = duart_create();
	if (trace & TRACE_DUART)
		duart_trace(duart, 1);

	m68k_init();
	m68k_set_cpu_type(cputype);
	m68k_pulse_reset();

	/* Init devices */
	device_init();

  	/* Start in the monitor if needed */
  	if (start_in_monitor) {
    	    pc = monitor(m68ki_cpu.pc);
    	    /* Change the start address if the monitor says so */
    	    if (pc != -1)
      	    m68ki_cpu.pc = pc;
  	}

	while (1) {
		pc = m68ki_cpu.pc;

    	    	// If the PC is a breakpoint, or we hit a write
    	    	// breakpoint, fall into the monitor
    	    	if (write_brkpt == 1 || is_breakpoint(pc, BRK_INST)) {
      		    write_brkpt = 0;
      		    pc = monitor(pc);

      		    // If we have a new PC from the monitor, set it
      		    if (pc != -1)
        	    	m68ki_cpu.pc = pc;
    	        }
		m68k_execute(1);
		/* Do a tick every 1000 instructions */
		duart_cnt++;
		if (duart_cnt == 1000) {
			duart_tick(duart);
			duart_cnt=0;
		}
	}
}
