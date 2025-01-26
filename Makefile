CFLAGS = -Wall -pedantic -g3 -Werror

all: rosco-r2

m68k/lib68k.a:
	$(MAKE) --directory m68k

rosco-r2: rosco-r2.c ide.c duart.c mapfile.c monitor.c bel_sdcard.c \
		m68k/lib68k.a
	cc -g3 -o rosco-r2 -Im68k rosco-r2.c ide.c duart.c \
		mapfile.c monitor.c bel_sdcard.c m68k/lib68k.a -lreadline

clean:
	rm -f rosco-r2 *.o
	$(MAKE) --directory m68k clean
