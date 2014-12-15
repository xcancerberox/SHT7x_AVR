#ifndef __TEST_CODE__
#define __TEST_CODE__

#define bool	_Bool
#define true	1
#define false	0
#define Nop()	asm volatile("nop")

#define SetBit(Byte,Bit)	 (Byte |= (1<<Bit))
#define	ClearBit(Byte,Bit)	 (Byte &= (~(1<<Bit)))
#define IsBitSet(Byte,Bit)	 ( (Byte & (1<<Bit)) ? true : false )

#define max(a,b)	( a>b ? a : b)
#define min(a,b)	( a>b ? a : b)

#define concat(a,b)		a ## b
#define def_port_reg(name)	concat(PORT,name)
#define def_pin_reg(name)	concat(PIN,name)
#define def_ddr_reg(name)	concat(DDR,name)

typedef enum retval_t{
    RV_SUCCESS = 0,
    RV_ERROR = 1,
    RV_NOTDEF = 2
} retval_t;

#endif
