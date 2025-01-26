// Available log levels
#ifndef LOGLEVEL_H
#define LOGLEVEL_H

#define LOG_MEMACCESS	0x01	// Memory accesses
#define LOG_BUSERROR	0x02	// Invalid memory accesses
#define LOG_INSTDECODE	0x04	// Instruction disassembly
#define LOG_REGDUMP	0x08	// Register dump
#define LOG_ILLINST	0x10	// Illegal instruction handler
#define LOG_INTACK	0x20	// Interrupt acknowledge handler
#define LOG_SDCARD	0x40	// SD Card operations
#define LOG_IOACCESS	0x80	// Hardware I/O operations
#define LOG_CH375	0x100	// CH375 operations
#define LOG_CH375_DATA	0x200	// CH375 data movements

#endif
