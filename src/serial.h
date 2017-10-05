#define	SSR_TDRE	(0x80)
#define	SSR_RDRF	(0x40)
#define	SSR_OER		(0x20)
#define	SSR_FER		(0x10)
#define	SSR_PER		(0x08)
#define	SSR_TEND	(0x04)
#define	SSR_MPBR	(0x02)
#define	SSR_MPBT	(0x01)
#define	SSR_ERR		(SSR_OER | SSR_FER | SSR_PER)

void put1byte(char c)
{
	while ( SCI1.SCSSR.BIT.TDRE == 0 ) ;
	SCI1.SCSSR.BIT.TDRE = 0;
	SCI1.SCTDR = c;
}

int get1byte(void)
{
	int ch;

	while ( ( SCI1.SCSSR.BYTE & 0x78) == 0 );

	while ( ( ( ch = SCI1.SCSSR.BYTE ) & 0x78 ) == 0 ) ;
	if ((ch & SSR_RDRF) == 0) {
		ch = -((ch & SSR_ERR) >> 3);			/* error */
		SCI1.SCSSR.BYTE = 0x80;
	} else {
		ch = SCI1.SCRDR & 0xff;
		SCI1.SCSSR.BIT.RDRF = 0;
	}
	return (ch);
}

int checkRx(void)
{
	int ch;
	
	ch = SCI1.SCSSR.BYTE;
	if ( ch & 0x38 )
		SCI1.SCSSR.BYTE = 0x80;	/* reset error.	*/
	if ((ch & 0x40) != 0)
		return 1;
	else
		return 0;
}

void putnbyte(char *buf,int len)
 {//nbyte
	int c;
   
    for(c = 0; c < len; c++){
		put1byte(buf[c]);
	}           
}

int myprintf(const char *fmt, ...)
{//可変長引数・・・データ値が違っても使える
	static char buffer[100];
	int len;
	
	va_list ap;//
	va_start(ap, fmt);//omajinai 
	
	len = vsprintf(buffer, fmt, ap);//データの長さを識別
	putnbyte(buffer, len);
	va_end(ap);
	return len;
}
