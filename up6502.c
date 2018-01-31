#include "up6502.h"	
#include "debug.h"
#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#define PROGMEM
#endif
typedef union _taddress taddress;

union _taddress {
	struct {
		unsigned char l;
		unsigned char h;
	} b;
	unsigned short w;
};


#define ADDRL PORTC
#define ADDRL_DIR DDRC
#define ADDRH PORTA
#define ADDRH_DIR DDRA
#define DATA PORTB
#define DATA_DIR DDRB


#define PIN_RDY PIND2
#define FLAG_RW 0x20
#define PIN_IRQ PIND3
#define PIN_NMI PIND4

#define UART_DATA       ((unsigned short)0x8800)
#define UART_STATUS     ((unsigned short)0x8801)
#define UART_CMD        ((unsigned short)0x8802)
#define UART_CTRL       ((unsigned short)0x8803)

#define	EXTRAM_ADDR	((unsigned short)0x1000)
#define INT_ADDR	((unsigned short)0xFFF8)
#define INT_ABORTL	((unsigned short)0xFFF8)
#define INT_ABORTH	((unsigned short)0xFFF9)
#define INT_NMIL	((unsigned short)0xFFFA)
#define INT_NMIH	((unsigned short)0xFFFB)
#define INT_RESETL	((unsigned short)0xFFFC)
#define INT_RESETH	((unsigned short)0xFFFD)
#define INT_IRQL	((unsigned short)0xFFFE)
#define INT_IRQH	((unsigned short)0xFFFF)

taddress fetch_address;

volatile unsigned char reg_a;
volatile unsigned char reg_x;
volatile unsigned char reg_y;
volatile unsigned char reg_p;
volatile unsigned char reg_s;
volatile taddress pc;

MemReadCallback _read_callback;
MemWriteCallback _write_callback;

#ifndef _TEST_
void wait_addr_ready(){
    //while (!PIN_RDY);
}
#endif

unsigned char irq_status;

#define 	IRQ_NMI		0x01
#define 	IRQ_IRQ		0x02

uint8_t _log;

void up6502_toggle_log(){
	_log = !_log;
}

void update_z_n(unsigned char v){
  if (v)         reg_p &= ~FLAG_Z; else reg_p |= FLAG_Z;
  if (v & 0x80)  reg_p |= FLAG_N;  else reg_p &= ~FLAG_N;
}
#define update_c(c)  if (c)         reg_p |= FLAG_C;  else reg_p &= ~FLAG_C;

const char ADDR_DEBUG_FMT[] PROGMEM = "%04x %02x %c%c-%c%c%c%c%c %02x\n";
const char MSG_UNK_OPCODE[] PROGMEM = "*** UNKNOWN OPCODE at %04x:%02x ***\n";

unsigned char fetch_addr(taddress _address){
	uint8_t value, res;
	res = _read_callback(_address.w,&value);
	
	if (!res){
		return value;
	} 
	return 0x0;
}

void store_at(unsigned char value, taddress _address){
	_write_callback(_address.w,value);
}

unsigned char fetch_a(){
	return reg_a;
}
void store_a(unsigned char value){
	reg_a = value;
}
void store_lastdir_a(unsigned char value){
	reg_a = value;
}
unsigned char fetch_imm(){
    unsigned char v;

    v = fetch_addr(pc);
    pc.w++;

    return v;
}

/** ZP ADDRESSING **/

unsigned char fetch_z(){
    fetch_address.b.h = 0;
    fetch_address.b.l = fetch_imm();
    return fetch_addr(fetch_address);
}
void store_z(unsigned char value){
    fetch_address.b.h = 0;
    fetch_address.b.l = fetch_imm();
    store_at(value,fetch_address);
}
void store_lastdir_z(unsigned char value){
    store_at(value,fetch_address);
}
/** ZP + X ADDRESSING **/

unsigned char fetch_z_x(){
    fetch_address.b.h = 0;
    fetch_address.b.l = (fetch_imm() + reg_x) & 0xFF;
    return fetch_addr(fetch_address);
}

void store_z_x(unsigned char value){
    fetch_address.b.h = 0;
    fetch_address.b.l = (fetch_imm() + reg_x) & 0xFF;
    store_at(value,fetch_address);
}
void store_lastdir_z_x(unsigned char value){
  store_at(value,fetch_address);
}

/** ZP + Y ADDRESSING **/

unsigned char fetch_z_y(){
    fetch_address.b.h = 0;
    fetch_address.b.l = (fetch_imm() + reg_y) & 0xFF;
    return fetch_addr(fetch_address);
}

void store_z_y(unsigned char value){
    fetch_address.b.h = 0;
    fetch_address.b.l = (fetch_imm() + reg_y) & 0xFF;
    store_at(value,fetch_address);
}
void store_lastdir_z_y(unsigned char value){
  store_at(value,fetch_address);
}

/** ABS ADDRESSING **/

unsigned char fetch_abs(){
    fetch_address.b.l = fetch_imm();
    fetch_address.b.h = fetch_imm();
    return fetch_addr(fetch_address);
}

void store_abs(unsigned char value){
    fetch_address.b.l = fetch_imm();
    fetch_address.b.h = fetch_imm();
    store_at(value,fetch_address);
}

void store_lastdir_abs(unsigned char value){
  store_at(value,fetch_address);
}
/** ABS + X ADDRESSING **/
unsigned char fetch_abs_x(){
    fetch_address.b.l = fetch_imm();
    fetch_address.b.h = fetch_imm();
    fetch_address.w += reg_x;
    return fetch_addr(fetch_address);
}

void store_abs_x(unsigned char value){
    fetch_address.b.l = fetch_imm();
    fetch_address.b.h = fetch_imm();
    fetch_address.w += reg_x;
    store_at(value,fetch_address);
}
void store_lastdir_abs_x(unsigned char value){
  store_at(value,fetch_address);
}
/** ABS + Y ADDRESSING **/

unsigned char fetch_abs_y(){
    fetch_address.b.l = fetch_imm();
    fetch_address.b.h = fetch_imm();
    fetch_address.w += reg_y;
    return fetch_addr(fetch_address);
}
void store_abs_y(unsigned char value){
    fetch_address.b.l = fetch_imm();
    fetch_address.b.h = fetch_imm();
    fetch_address.w += reg_y;
    store_at(value,fetch_address);
}
void store_lastdir_abs_y(unsigned char value){
  store_at(value,fetch_address);
}

/** IND Y ADDRESSING **/

unsigned char fetch_ind_y(){
    taddress address;
    taddress ind;
    address.b.h = 0;
    address.b.l = fetch_imm();
    ind.b.l = fetch_addr(address);
    address.w++;
    ind.b.h = fetch_addr(address);
    ind.w+= reg_y;
    fetch_address = ind;
    return fetch_addr(fetch_address);
}

void store_ind_y(unsigned char value){
    taddress address;
    taddress ind;
    address.b.h = 0;
    address.b.l = fetch_imm();
    ind.b.l = fetch_addr(address);
    address.w++;
    ind.b.h = fetch_addr(address);
    ind.w+=reg_y;
    fetch_address = ind;
    store_at(value,fetch_address);
}


unsigned char fetch_x_ind(){
    taddress address;
    taddress ind;
    address.b.h = 0;
    address.b.l = (fetch_imm() + reg_x) & 0xFF;
    ind.b.l = fetch_addr(address);
    address.w++;
    ind.b.h = fetch_addr(address);
    fetch_address = ind;
    return fetch_addr(fetch_address);
}

void store_x_ind(unsigned char value){
    taddress address;
    taddress ind;
    address.b.h = 0;
    address.b.l = (fetch_imm() + reg_x) & 0xFF;
    ind.b.l = fetch_addr(address);
    address.w++;
    ind.b.h = fetch_addr(address);
    fetch_address = ind;
    store_at(value,fetch_address);
}

void store_lastdir_x_ind(unsigned char value){
    store_at(value,fetch_address);
}


void do_push(unsigned char value){
    taddress _s;
    _s.w = (unsigned short)(0x100|reg_s);
    store_at(value,_s);
    reg_s--;
}

unsigned char do_pop(){
    taddress _s;
    reg_s++;
    _s.w = (unsigned short)(0x100|reg_s);
    return fetch_addr(_s);
}

void handle_interrupt(unsigned short vector,const taddress _pc, unsigned char _p){
    taddress address;

		taddress retaddr = _pc;
		//retaddr.w++;
    do_push(retaddr.b.h);
    do_push(retaddr.b.l);
    do_push(_p);
    address.w = vector;
    pc.b.l = fetch_addr(address);
    address.w++;
    pc.b.h = fetch_addr(address);
}

void op_rti(){
    taddress _pc;
    reg_p = do_pop();
    _pc.b.l = do_pop();
    _pc.b.h = do_pop();
    pc = _pc;
}

void op_unknown(){}

void op_brk(){
    handle_interrupt(0xFFFE,pc,reg_p|FLAG_B);
}

void op_jsr_abs(){
    taddress _pc,current_pc;
    _pc.b.l = fetch_imm();
    _pc.b.h = fetch_imm();
    current_pc = pc;
    current_pc.w-=1;
    do_push(current_pc.b.h);
    do_push(current_pc.b.l);
    pc = _pc;
}

void op_rts(){
    taddress _pc;
    _pc.b.l = do_pop();
    _pc.b.h = do_pop();
    _pc.w++;
    pc = _pc;
}


void op_jmp_abs(){
    taddress _pc;
    _pc.b.l = fetch_imm();
    _pc.b.h = fetch_imm();
    pc = _pc;
}

void op_jmp_ind(){
    taddress address;
    taddress _pc;

    address.b.l = fetch_imm();
    address.b.h = fetch_imm();
    _pc.b.l = fetch_addr(address);  
    address.w++;
    _pc.b.h = fetch_addr(address);

    pc = _pc;
}

void op_nop(){
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
    asm volatile ("nop");
}

#define do_op_ora(mode)\
void op_ora_##mode(){\
    reg_a |= fetch_##mode();\
    update_z_n(reg_a);\
}
do_op_ora(imm)
do_op_ora(ind_y)
do_op_ora(x_ind)
do_op_ora(z)
do_op_ora(abs)
do_op_ora(abs_x)
do_op_ora(abs_y)
do_op_ora(z_x)
#define do_op_and(mode)\
void op_and_##mode(){\
    reg_a &= fetch_##mode();\
    update_z_n(reg_a);\
}
do_op_and(x_ind)
do_op_and(ind_y)
do_op_and(imm)
do_op_and(abs)
do_op_and(z)
do_op_and(abs_y)
do_op_and(abs_x)
do_op_and(z_x)
#define do_op_eor(mode)\
void op_eor_##mode(){\
    reg_a ^= fetch_##mode();\
    update_z_n(reg_a);\
}
do_op_eor(x_ind)
do_op_eor(ind_y)
do_op_eor(imm)
do_op_eor(abs)
do_op_eor(z)
do_op_eor(abs_y)
do_op_eor(abs_x)
do_op_eor(z_x)

unsigned char bcd(unsigned char v){
  return (v & 0xF) + (v >> 4) * 10;
}
unsigned char tobin(unsigned char v){
	return ((v / 10) << 4) + (v % 10);
}
#define do_op_adc(mode)\
void op_adc_##mode(){\
    unsigned char r = fetch_##mode();\
    unsigned short v;\
    if (reg_p & FLAG_D){\
      v = bcd(reg_a) + bcd(r);\
      if (reg_p & FLAG_C) v++;\
      update_c(v > 99);\
      v = tobin(v> 99 ? v - 100 : v);\
    } else {\
      v = reg_a + r;\
      if (reg_p & FLAG_C) v++;\
      update_c(v > 0xFF);\
    }\
    if ((r & 0x80) != (v & 0x80)) reg_p |= FLAG_V; else reg_p &= ~FLAG_V;\
    reg_a = (unsigned char)(v & 0xFF);\
    update_z_n(reg_a);\
}
do_op_adc(x_ind)
do_op_adc(z)
do_op_adc(imm)
do_op_adc(abs)
do_op_adc(ind_y)
do_op_adc(z_x)
do_op_adc(abs_y)
do_op_adc(abs_x)
#define do_op_cmp(mode)\
void op_cmp_##mode(){\
    unsigned char m = fetch_##mode();\
    unsigned char v = reg_a - m;\
    update_z_n(v);\
    update_c(reg_a >= m);\
}
do_op_cmp(imm)
do_op_cmp(abs)
do_op_cmp(abs_x)
do_op_cmp(abs_y)
do_op_cmp(ind_y)
do_op_cmp(z_x)
do_op_cmp(z)
do_op_cmp(x_ind)

#define do_op_cpr(r,mode)\
void op_cp##r##_##mode(){\
    unsigned char m = fetch_##mode();\
    unsigned char v = reg_##r - m;\
    update_z_n(v);\
    update_c(reg_##r >=m);\
}

do_op_cpr(y,imm)
do_op_cpr(y,z)
do_op_cpr(y,abs)
do_op_cpr(x,imm)
do_op_cpr(x,z)
do_op_cpr(x,abs)

#define do_op_lsr(mode)\
void op_lsr_##mode(){\
    unsigned char v = fetch_##mode();\
    unsigned char cv = (v & 1);\
    v >>=1;\
    store_lastdir_##mode(v);\
    if (v == 0) reg_p |= FLAG_Z; else reg_p &= ~FLAG_Z;\
    reg_p &= ~FLAG_N;\
    update_c(cv);\
}

do_op_lsr(z)
do_op_lsr(a)
do_op_lsr(z_x)
do_op_lsr(abs_x)
do_op_lsr(abs)

#define do_op_rol(mode)\
void op_rol_##mode(){\
    unsigned char v = fetch_##mode();\
    unsigned char c = v & 0x80;\
    v <<=1;\
    if (reg_p & FLAG_C) v|=1;\
    store_lastdir_##mode(v);\
    update_z_n(v);\
    update_c(c);\
}
do_op_rol(z)
do_op_rol(a)
do_op_rol(abs_x)
do_op_rol(abs)
do_op_rol(z_x)

#define do_op_ror(mode)\
void op_ror_##mode(){\
    unsigned char v = fetch_##mode();\
    unsigned char c = (v & 1);\
    v >>=1;\
    if (reg_p & FLAG_C) v|=0x80;\
    store_lastdir_##mode(v);\
    update_z_n(v);\
    update_c(c);\
}

do_op_ror(z)
do_op_ror(a)
do_op_ror(abs)
do_op_ror(z_x)
do_op_ror(abs_x)

void op_plp(){
    reg_p = do_pop();
}
void op_pha(){
    do_push(reg_a);
}
void op_php(){
    do_push(reg_p);
}
void op_pla(){
    reg_a = do_pop();
    update_z_n(reg_a);
}

#define do_op_st(r,mode)\
void op_st##r##_##mode(){\
    store_##mode(reg_##r);\
}

do_op_st(a,z)
do_op_st(a,abs)
do_op_st(a,x_ind)
do_op_st(a,ind_y)
do_op_st(a,abs_x)
do_op_st(a,abs_y)
do_op_st(a,z_x)

do_op_st(x,z)
do_op_st(x,abs)
do_op_st(x,z_y)

do_op_st(y,z)
do_op_st(y,abs)
do_op_st(y,z_x)

#define do_op_asl(mode)\
void op_asl_##mode(){\
    unsigned char r = fetch_##mode();\
    unsigned char c = r & 0x80;\
    unsigned char v = (r << 1) & 0xFE;\
    store_lastdir_##mode(v);\
    update_z_n(v);\
    update_c(c);\
}
do_op_asl(a)
do_op_asl(z)
do_op_asl(z_x)
do_op_asl(abs)
do_op_asl(abs_x)

void op_cli(){   reg_p &= ~FLAG_I;}
void op_clc(){   reg_p &= ~FLAG_C;}
void op_cld(){   reg_p &= ~FLAG_D;}
void op_clv(){   reg_p &= ~FLAG_V;}
void op_sei(){   reg_p |= FLAG_I;}
void op_sec(){   reg_p |= FLAG_C;}
void op_sed(){   reg_p |= FLAG_D;}

#define do_op_bit(mode)\
void op_bit_##mode(){\
    unsigned char c = reg_a & fetch_##mode();\
    if (c & 0x80)  reg_p |= FLAG_N; else reg_p &= ~FLAG_N;\
    if (c & 0x40)  reg_p |= FLAG_V; else reg_p &= ~FLAG_V;\
    if (!c)        reg_p |= FLAG_Z; else reg_p &= ~FLAG_Z;\
}
do_op_bit(z)
do_op_bit(abs)

#define do_branch(name,cond)\
void op_##name##_rel(){\
    char offset = fetch_imm();\
    if (cond){\
        pc.w+=offset;\
    }\
}
do_branch(bpl,!(reg_p & FLAG_N))
do_branch(bmi,(reg_p & FLAG_N))
do_branch(bvc,!(reg_p & FLAG_V))
do_branch(bvs,(reg_p & FLAG_V))
do_branch(beq,(reg_p & FLAG_Z))
do_branch(bne,!(reg_p & FLAG_Z))
do_branch(bcs,(reg_p & FLAG_C))
do_branch(bcc,!(reg_p & FLAG_C))


#define do_op_t(s,d)\
void op_t##s##d(){\
    reg_##d = reg_##s;\
    update_z_n(reg_##d);\
}

void op_txs(){
  reg_s = reg_x;
  update_z_n(reg_s);
}

do_op_t(s,x)
do_op_t(a,x)
do_op_t(x,a)
do_op_t(a,y)
do_op_t(y,a)

// 		DEBUG("OP LD %s %02x\n",#r,r);

#define do_op_ld(r,mode)\
void op_ld##r##_##mode(){\
    reg_##r = fetch_##mode();\
    update_z_n(reg_##r);\
}
do_op_ld(a,imm)
do_op_ld(a,x_ind)
do_op_ld(a,z)
do_op_ld(a,abs)
do_op_ld(a,abs_x)
do_op_ld(a,abs_y)
do_op_ld(a,ind_y)
do_op_ld(a,z_x)

do_op_ld(y,imm)
do_op_ld(y,z)
do_op_ld(y,abs)
do_op_ld(y,z_x)
do_op_ld(y,abs_x)

do_op_ld(x,imm)
do_op_ld(x,z)
do_op_ld(x,abs)
do_op_ld(x,z_y)
do_op_ld(x,abs_y)

#define do_op_inc(mode)\
void op_inc_##mode(){\
    unsigned char v = fetch_##mode() +1;\
    store_lastdir_##mode(v);\
    update_z_n(v);\
}

do_op_inc(z)
do_op_inc(abs)
do_op_inc(z_x)
do_op_inc(abs_x)

#define do_op_inc_r(r)\
void op_in##r(){\
    reg_##r++;\
    update_z_n(reg_##r);\
}

do_op_inc_r(y)
do_op_inc_r(x)

#define do_op_dec_r(r)\
void op_de##r(){\
    reg_##r--;\
    update_z_n(reg_##r);\
}
do_op_dec_r(x)
do_op_dec_r(y)

#define do_op_dec(mode)\
void op_dec_##mode(){\
    unsigned char v = fetch_##mode() -1;\
    store_lastdir_##mode(v);\
    update_z_n(v);\
}

do_op_dec(z)
do_op_dec(abs)
do_op_dec(z_x)
do_op_dec(abs_x)

#define do_op_sbc(mode)\
void op_sbc_##mode(){\
    unsigned char c = fetch_##mode();\
    short v;\
    if (reg_p & FLAG_D){\
      v = bcd(reg_a) - bcd(c);\
      if (!(reg_p & FLAG_C)) v--;\
      if (v > 99 || v < 0) reg_p |= FLAG_V; else reg_p &= ~FLAG_V;\
      update_c(v >= 0);\
      v = tobin(v> 99 ? v - 100 : v);\
    } else {\
      v = reg_a - c;\
      if (!(reg_p & FLAG_C)) v--;\
      if (v > 127 || v < -128) reg_p |= FLAG_V; else reg_p &= ~FLAG_V;\
			update_c(v >= 0);\
    }\
    update_z_n((unsigned char)(v & 0xFF));\
    reg_a = (unsigned char)(v & 0xFF);\
}
do_op_sbc(x_ind)
do_op_sbc(z)
do_op_sbc(abs)
do_op_sbc(ind_y)
do_op_sbc(z_x)
do_op_sbc(abs_x)
do_op_sbc(abs_y)
do_op_sbc(imm)

void op_nop();


typedef void (*Operator)(void);

const Operator OPERATORS[] PROGMEM = {
    // 0x00
    op_brk,         op_ora_x_ind,   op_unknown, op_unknown, op_unknown,     op_ora_z,       op_asl_z,       op_unknown,
    op_php,         op_ora_imm,     op_asl_a,   op_unknown, op_unknown,     op_ora_abs,     op_asl_abs,     op_unknown,
    // 0x10
    op_bpl_rel,     op_ora_ind_y,   op_unknown, op_unknown, op_unknown,     op_ora_z_x,     op_asl_z_x,     op_unknown,
    op_clc,         op_ora_abs_y,   op_unknown, op_unknown, op_unknown,     op_ora_abs_x,   op_asl_abs_x,   op_unknown,
     // 0x20
    op_jsr_abs,     op_and_x_ind,   op_unknown, op_unknown, op_bit_z,       op_and_z,       op_rol_z,       op_unknown,
    op_plp,         op_and_imm,     op_rol_a,   op_unknown, op_bit_abs,     op_and_abs,     op_rol_abs,     op_unknown,
    // 0x30
    op_bmi_rel,     op_and_ind_y,   op_unknown, op_unknown, op_unknown,     op_and_z_x,     op_rol_z_x,     op_unknown,
    op_sec,         op_and_abs_y,   op_unknown, op_unknown, op_unknown,     op_and_abs_x,   op_rol_abs_x,   op_unknown,
    // 0x40
    op_rti,         op_eor_x_ind,   op_unknown, op_unknown, op_unknown,     op_eor_z,       op_lsr_z,       op_unknown,
    op_pha,         op_eor_imm,     op_lsr_a,   op_unknown, op_jmp_abs,     op_eor_abs,     op_lsr_abs,     op_unknown,
    // 0x50
    op_bvc_rel,     op_eor_ind_y,   op_unknown, op_unknown, op_unknown,     op_eor_z_x,     op_lsr_z_x,     op_unknown,
    op_cli,         op_eor_abs_y,   op_unknown, op_unknown, op_unknown,     op_eor_abs_x,   op_lsr_abs_x,   op_unknown,
    // 0x60
    op_rts,         op_adc_x_ind,   op_unknown, op_unknown, op_unknown,     op_adc_z,       op_ror_z,       op_unknown,
    op_pla,         op_adc_imm,     op_ror_a,   op_unknown, op_jmp_ind,     op_adc_abs,     op_ror_abs,     op_unknown,
    // 0x70
    op_bvs_rel,     op_adc_ind_y,   op_unknown, op_unknown, op_unknown,     op_adc_z_x,     op_ror_z_x,     op_unknown,
    op_sei,         op_adc_abs_y,   op_unknown, op_unknown, op_unknown,     op_adc_abs_x,   op_ror_abs_x,   op_unknown,
    // 0x80
    op_unknown,     op_sta_x_ind,   op_unknown, op_unknown, op_sty_z,       op_sta_z,       op_stx_z,       op_unknown,
    op_dey,         op_unknown,     op_txa,     op_unknown, op_sty_abs, 	op_sta_abs,     op_stx_abs,     op_unknown,
    // 0x90
    op_bcc_rel,     op_sta_ind_y,   op_unknown, op_unknown, op_sty_z_x,     op_sta_z_x,     op_stx_z_y,     op_unknown,
    op_tya,         op_sta_abs_y,   op_txs,     op_unknown, op_unknown,     op_sta_abs_x,   op_unknown,     op_unknown,
    // 0xA0
    op_ldy_imm,     op_lda_x_ind,   op_ldx_imm, op_unknown, op_ldy_z,       op_lda_z,       op_ldx_z,       op_unknown,
    op_tay,         op_lda_imm,     op_tax,     op_unknown, op_ldy_abs,     op_lda_abs,     op_ldx_abs,     op_unknown,
    //0xB0,
    op_bcs_rel,     op_lda_ind_y,   op_unknown, op_unknown, op_ldy_z_x,     op_lda_z_x,     op_ldx_z_y,     op_unknown,
    op_clv,         op_lda_abs_y,   op_tsx,     op_unknown, op_ldy_abs_x,   op_lda_abs_x,   op_ldx_abs_y,   op_unknown,
    //0xC0
    op_cpy_imm,     op_cmp_x_ind,   op_unknown, op_unknown, op_cpy_z,       op_cmp_z,       op_dec_z,       op_unknown,
    op_iny,         op_cmp_imm,     op_dex,     op_unknown, op_cpy_abs,     op_cmp_abs,     op_dec_abs,     op_unknown,
    // 0xD0
    op_bne_rel,     op_cmp_ind_y,   op_unknown, op_unknown, op_unknown,     op_cmp_z_x,     op_dec_z_x,     op_unknown,
    op_cld,         op_cmp_abs_y,   op_unknown, op_unknown, op_unknown,     op_cmp_abs_x,   op_dec_abs_x,   op_unknown,
    // 0xE0
    op_cpx_imm,     op_sbc_x_ind,   op_unknown, op_unknown, op_cpx_z,       op_sbc_z,       op_inc_z,       op_unknown,
    op_inx,         op_sbc_imm,     op_nop,     op_unknown, op_cpx_abs,     op_sbc_abs,     op_inc_abs,     op_unknown,
    // 0xF0
    op_beq_rel,     op_sbc_ind_y,   op_unknown, op_unknown, op_unknown,     op_sbc_z_x,     op_inc_z_x,     op_unknown,
    op_sed,         op_sbc_abs_y,   op_unknown, op_unknown, op_unknown,     op_sbc_abs_x,   op_inc_abs_x,   op_unknown
};


void start(){
  taddress address;

  address.w = 0xFFFC;
  pc.b.l = fetch_addr(address);
  address.w++;
  pc.b.h = fetch_addr(address);        
}

void up6502_reset(){
 	reg_a = reg_x = reg_y = reg_p = 0;
 	reg_s = 0xFF;
	start();
}

char irq(){
	char result = !(reg_p & FLAG_I) && (irq_status & IRQ_IRQ);
	irq_status &= ~IRQ_IRQ;
	return result;
}
char nmi(){
	char result = irq_status & IRQ_NMI;
	irq_status &= ~IRQ_NMI;
	return result;
}

char log_addr(uint16_t addr){
	return 0;//(addr >= 0xF97E && addr <= 0xF99B);
}

void up6502_loop(){
  unsigned char opcode;
  uint16_t current_pc;
  Operator operator;
	
	if (nmi()){
		unsigned short _p = reg_p;
		handle_interrupt(0xFFFA,pc,_p);
	} else if (irq()){
		unsigned short _p = reg_p;
		handle_interrupt(0xFFFE,pc,_p);
	}
	current_pc = pc.w;
	opcode = fetch_imm();
	
	operator = (Operator)pgm_read_word(&OPERATORS[opcode]);
	
	if (operator == op_unknown){
		DEBUGP(MSG_UNK_OPCODE,current_pc,opcode);
		up6502_reset();
	} else {
		operator();
	}

	if (_log || log_addr(current_pc)){
		DEBUGP(ADDR_DEBUG_FMT,
			current_pc,
			opcode,
			(reg_p & FLAG_N ? 'N' : '-'),
			(reg_p & FLAG_V ? 'V' : '-'),
			(reg_p & FLAG_B ? 'B' : '-'),
			(reg_p & FLAG_D ? 'D' : '-'),
			(reg_p & FLAG_I ? 'I' : '-'),
			(reg_p & FLAG_Z ? 'Z' : '-'),
			(reg_p & FLAG_C ? 'C' : '-'),
			reg_a);
	}

}

void up6502_setup(MemReadCallback read_callback, MemWriteCallback write_callback){
	//dump = 0;
	irq_status = 0;
	_read_callback = read_callback;
	_write_callback = write_callback;
	
	up6502_reset();

}
void up6502_irq(){
	irq_status |= IRQ_IRQ;
}

uint8_t up6502_get_a(){
	return reg_a;
}

uint8_t up6502_get_x(){
	return reg_x;
}

uint8_t up6502_get_y(){
	return reg_y;
}

uint8_t up6502_get_p(){
	return reg_p;
}

uint8_t up6502_get_s(){
	return reg_s;
}
uint16_t up6502_get_pc(){
	return pc.w;
}

void up6502_set_a(uint8_t _a){
	reg_a = _a;
}

void up6502_set_x(uint8_t _x){
	reg_x = _x;
}

void up6502_set_y(uint8_t _y){
	reg_y = _y;
}

void up6502_set_p(uint8_t _p){
	reg_p = _p;
}

void up6502_set_s(uint8_t _s){
	reg_s = _s;
}

void up6502_set_pc(uint16_t _pc){
	pc.w = _pc;
}

void up6502_set_flags(uint8_t flags){
	reg_p |= flags;
}
void up6502_clear_flags(uint8_t flags){
	reg_p &= ~flags;
}
