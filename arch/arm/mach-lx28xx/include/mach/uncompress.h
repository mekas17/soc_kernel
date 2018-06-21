/*
 * linux/include/asm-arm/arch-arkdmp/uncompress.h
 */


#define SERIAL_BASE (0x60070000)


#define SERIAL_DR		*(volatile unsigned char *)(SERIAL_BASE)

#define SERIAL_UARTFR0	*(volatile unsigned char *)(SERIAL_BASE+0x18)

static __inline__ void putc(char c)
{
	while( (SERIAL_UARTFR0 & 0x20) );
	if (c == '\n')					  // Replace line feed with '\r'
	{
		SERIAL_DR = '\r';
		while( (SERIAL_UARTFR0 & 0x20) );
	}
	SERIAL_DR = c;
	
}





static inline void flush(void)
{
}


/*
 * This does not append a newline
 */
 #if 0
static void putstr(const char *s)
{
	while (*s) {
		putc(*s);
		if (*s == '\n')
			putc('\r');
		s++;
	}
}
#endif

#ifdef DEBUG
static void putn(unsigned long z)
{
	int i;
	char x;

	putc('0');
	putc('x');
	for (i=0;i<8;i++) {
		x='0'+((z>>((7-i)*4))&0xf);
		if (x>'9') x=x-'0'+'A'-10;
		putc(x);
	}
}

static void putr()
{
	putc('\n');
	putc('\r');
}
#endif

/*
 * nothing to do
 */
#define arch_decomp_setup()
#define arch_decomp_wdog()
