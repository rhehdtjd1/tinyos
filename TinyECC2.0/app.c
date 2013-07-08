#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 151 "/usr/lib/gcc-lib/msp430/3.2.3/include/stddef.h" 3
typedef int ptrdiff_t;
#line 213
typedef unsigned int size_t;
#line 325
typedef int wchar_t;
# 8 "/usr/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
}  ;
#line 14
struct __nesc_attr_one_nok {
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
}  ;
# 38 "/usr/msp430/include/sys/inttypes.h" 3
typedef signed char int8_t;
typedef unsigned char uint8_t;

typedef int int16_t;
typedef unsigned int uint16_t;

typedef long int32_t;
typedef unsigned long uint32_t;

typedef long long int64_t;
typedef unsigned long long uint64_t;




typedef int16_t intptr_t;
typedef uint16_t uintptr_t;
# 235 "/usr/lib/ncc/nesc_nx.h"
static __inline uint8_t __nesc_ntoh_uint8(const void * source)  ;




static __inline uint8_t __nesc_hton_uint8(void * target, uint8_t value)  ;
#line 269
static __inline uint16_t __nesc_hton_uint16(void * target, uint16_t value)  ;
#line 385
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 41 "/usr/msp430/include/sys/types.h" 3
typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;
typedef unsigned short ushort;
typedef unsigned int uint;

typedef uint8_t u_int8_t;
typedef uint16_t u_int16_t;
typedef uint32_t u_int32_t;
typedef uint64_t u_int64_t;

typedef u_int64_t u_quad_t;
typedef int64_t quad_t;
typedef quad_t *qaddr_t;

typedef char *caddr_t;
typedef const char *c_caddr_t;
typedef volatile char *v_caddr_t;
typedef u_int32_t fixpt_t;
typedef u_int32_t gid_t;
typedef u_int32_t in_addr_t;
typedef u_int16_t in_port_t;
typedef u_int32_t ino_t;
typedef long key_t;
typedef u_int16_t mode_t;
typedef u_int16_t nlink_t;
typedef quad_t rlim_t;
typedef int32_t segsz_t;
typedef int32_t swblk_t;
typedef int32_t ufs_daddr_t;
typedef int32_t ufs_time_t;
typedef u_int32_t uid_t;
# 40 "/usr/msp430/include/string.h" 3
extern void *memcpy(void *arg_0x40280200, const void *arg_0x40280398, size_t arg_0x40280530);

extern void *memset(void *arg_0x40283220, int arg_0x40283378, size_t arg_0x40283510);
#line 63
extern void *memset(void *arg_0x40291118, int arg_0x40291270, size_t arg_0x40291408);
# 59 "/usr/msp430/include/stdlib.h" 3
#line 56
typedef struct __nesc_unnamed4242 {
  int quot;
  int rem;
} div_t;







#line 64
typedef struct __nesc_unnamed4243 {
  long quot;
  long rem;
} ldiv_t;
# 122 "/usr/msp430/include/sys/config.h" 3
typedef long int __int32_t;
typedef unsigned long int __uint32_t;
# 12 "/usr/msp430/include/sys/_types.h" 3
typedef long _off_t;
typedef long _ssize_t;
# 28 "/usr/msp430/include/sys/reent.h" 3
typedef __uint32_t __ULong;


struct _glue {

  struct _glue *_next;
  int _niobs;
  struct __sFILE *_iobs;
};

struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _atexit {
  struct _atexit *_next;
  int _ind;
  void (*_fns[32])(void );
};








struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 116
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;


  void *_cookie;

  int (*_read)(void *_cookie, char *_buf, int _n);
  int (*_write)(void *_cookie, const char *_buf, int _n);

  _fpos_t (*_seek)(void *_cookie, _fpos_t _offset, int _whence);
  int (*_close)(void *_cookie);


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;

  struct _reent *_data;
};
#line 174
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};









struct _reent {


  int _errno;




  struct __sFILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  const char *_current_locale;

  int __sdidinit;

  void (*__cleanup)(struct _reent *arg_0x402ae510);


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0x402b2b88);




  struct _glue __sglue;
  struct __sFILE __sf[3];
};
#line 273
struct _reent;
# 18 "/usr/msp430/include/math.h" 3
union __dmath {

  __uint32_t i[2];
  double d;
};




union __dmath;
#line 208
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 261
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 20 "/opt/tinyos-2.0.2/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4247 {
#line 21
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;
uint16_t TOS_NODE_ID = 1;





struct __nesc_attr_atmostonce {
};
#line 31
struct __nesc_attr_atleastonce {
};
#line 32
struct __nesc_attr_exactlyonce {
};
# 34 "/opt/tinyos-2.0.2/tos/types/TinyError.h"
enum __nesc_unnamed4248 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2);
# 39 "/usr/msp430/include/msp430/iostructures.h" 3
#line 27
typedef union port {
  volatile unsigned char reg_p;
  volatile struct __nesc_unnamed4249 {
    unsigned char __p0 : 1, 
    __p1 : 1, 
    __p2 : 1, 
    __p3 : 1, 
    __p4 : 1, 
    __p5 : 1, 
    __p6 : 1, 
    __p7 : 1;
  } __pin;
} __attribute((packed))  ioregister_t;
#line 108
struct port_full_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t ifg;
  ioregister_t ies;
  ioregister_t ie;
  ioregister_t sel;
};









struct port_simple_t {
  ioregister_t in;
  ioregister_t out;
  ioregister_t dir;
  ioregister_t sel;
};




struct port_full_t;



struct port_full_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;



struct port_simple_t;
# 116 "/usr/msp430/include/msp430/gpio.h" 3
volatile unsigned char P1OUT __asm ("0x0021");

volatile unsigned char P1DIR __asm ("0x0022");





volatile unsigned char P1IE __asm ("0x0025");

volatile unsigned char P1SEL __asm ("0x0026");










volatile unsigned char P2OUT __asm ("0x0029");

volatile unsigned char P2DIR __asm ("0x002A");





volatile unsigned char P2IE __asm ("0x002D");

volatile unsigned char P2SEL __asm ("0x002E");










volatile unsigned char P3OUT __asm ("0x0019");

volatile unsigned char P3DIR __asm ("0x001A");

volatile unsigned char P3SEL __asm ("0x001B");










volatile unsigned char P4OUT __asm ("0x001D");

volatile unsigned char P4DIR __asm ("0x001E");

volatile unsigned char P4SEL __asm ("0x001F");










volatile unsigned char P5OUT __asm ("0x0031");

volatile unsigned char P5DIR __asm ("0x0032");

volatile unsigned char P5SEL __asm ("0x0033");










volatile unsigned char P6OUT __asm ("0x0035");

volatile unsigned char P6DIR __asm ("0x0036");

volatile unsigned char P6SEL __asm ("0x0037");
# 94 "/usr/msp430/include/msp430/usart.h" 3
volatile unsigned char U0TCTL __asm ("0x0071");
#line 275
volatile unsigned char U1CTL __asm ("0x0078");

volatile unsigned char U1TCTL __asm ("0x0079");



volatile unsigned char U1MCTL __asm ("0x007B");

volatile unsigned char U1BR0 __asm ("0x007C");

volatile unsigned char U1BR1 __asm ("0x007D");

volatile unsigned char U1RXBUF __asm ("0x007E");
# 27 "/usr/msp430/include/msp430/timera.h" 3
volatile unsigned int TA0CTL __asm ("0x0160");

volatile unsigned int TA0R __asm ("0x0170");


volatile unsigned int TA0CCTL0 __asm ("0x0162");

volatile unsigned int TA0CCTL1 __asm ("0x0164");
#line 70
volatile unsigned int TA0CCTL2 __asm ("0x0166");
#line 127
#line 118
typedef struct __nesc_unnamed4250 {
  volatile unsigned 
  taifg : 1, 
  taie : 1, 
  taclr : 1, 
  dummy : 1, 
  tamc : 2, 
  taid : 2, 
  tassel : 2;
} __attribute((packed))  tactl_t;
#line 143
#line 129
typedef struct __nesc_unnamed4251 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  dummy : 1, 
  scci : 1, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tacctl_t;


struct timera_t {
  tactl_t ctl;
  tacctl_t cctl0;
  tacctl_t cctl1;
  tacctl_t cctl2;
  volatile unsigned dummy[4];
  volatile unsigned tar;
  volatile unsigned taccr0;
  volatile unsigned taccr1;
  volatile unsigned taccr2;
};



struct timera_t;
# 26 "/usr/msp430/include/msp430/timerb.h" 3
volatile unsigned int TBR __asm ("0x0190");


volatile unsigned int TBCCTL0 __asm ("0x0182");





volatile unsigned int TBCCR0 __asm ("0x0192");
#line 76
#line 64
typedef struct __nesc_unnamed4252 {
  volatile unsigned 
  tbifg : 1, 
  tbie : 1, 
  tbclr : 1, 
  dummy1 : 1, 
  tbmc : 2, 
  tbid : 2, 
  tbssel : 2, 
  dummy2 : 1, 
  tbcntl : 2, 
  tbclgrp : 2;
} __attribute((packed))  tbctl_t;
#line 91
#line 78
typedef struct __nesc_unnamed4253 {
  volatile unsigned 
  ccifg : 1, 
  cov : 1, 
  out : 1, 
  cci : 1, 
  ccie : 1, 
  outmod : 3, 
  cap : 1, 
  clld : 2, 
  scs : 1, 
  ccis : 2, 
  cm : 2;
} __attribute((packed))  tbcctl_t;


struct timerb_t {
  tbctl_t ctl;
  tbcctl_t cctl0;
  tbcctl_t cctl1;
  tbcctl_t cctl2;

  tbcctl_t cctl3;
  tbcctl_t cctl4;
  tbcctl_t cctl5;
  tbcctl_t cctl6;



  volatile unsigned tbr;
  volatile unsigned tbccr0;
  volatile unsigned tbccr1;
  volatile unsigned tbccr2;

  volatile unsigned tbccr3;
  volatile unsigned tbccr4;
  volatile unsigned tbccr5;
  volatile unsigned tbccr6;
};





struct timerb_t;
# 20 "/usr/msp430/include/msp430/basic_clock.h" 3
volatile unsigned char DCOCTL __asm ("0x0056");

volatile unsigned char BCSCTL1 __asm ("0x0057");

volatile unsigned char BCSCTL2 __asm ("0x0058");
# 18 "/usr/msp430/include/msp430/adc12.h" 3
volatile unsigned int ADC12CTL0 __asm ("0x01A0");

volatile unsigned int ADC12CTL1 __asm ("0x01A2");
#line 42
#line 30
typedef struct __nesc_unnamed4254 {
  volatile unsigned 
  adc12sc : 1, 
  enc : 1, 
  adc12tovie : 1, 
  adc12ovie : 1, 
  adc12on : 1, 
  refon : 1, 
  r2_5v : 1, 
  msc : 1, 
  sht0 : 4, 
  sht1 : 4;
} __attribute((packed))  adc12ctl0_t;
#line 54
#line 44
typedef struct __nesc_unnamed4255 {
  volatile unsigned 
  adc12busy : 1, 
  conseq : 2, 
  adc12ssel : 2, 
  adc12div : 3, 
  issh : 1, 
  shp : 1, 
  shs : 2, 
  cstartadd : 4;
} __attribute((packed))  adc12ctl1_t;
#line 74
#line 56
typedef struct __nesc_unnamed4256 {
  volatile unsigned 
  bit0 : 1, 
  bit1 : 1, 
  bit2 : 1, 
  bit3 : 1, 
  bit4 : 1, 
  bit5 : 1, 
  bit6 : 1, 
  bit7 : 1, 
  bit8 : 1, 
  bit9 : 1, 
  bit10 : 1, 
  bit11 : 1, 
  bit12 : 1, 
  bit13 : 1, 
  bit14 : 1, 
  bit15 : 1;
} __attribute((packed))  adc12xflg_t;


struct adc12_t {
  adc12ctl0_t ctl0;
  adc12ctl1_t ctl1;
  adc12xflg_t ifg;
  adc12xflg_t ie;
  adc12xflg_t iv;
};




struct adc12_t;
# 83 "/usr/msp430/include/msp430x16x.h" 3
volatile unsigned char ME1 __asm ("0x0004");





volatile unsigned char ME2 __asm ("0x0005");
# 177 "/opt/tinyos-2.0.2/tos/chips/msp430/msp430hardware.h"
typedef uint8_t mcu_power_t  ;
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2);


enum __nesc_unnamed4257 {
  MSP430_POWER_ACTIVE = 0, 
  MSP430_POWER_LPM0 = 1, 
  MSP430_POWER_LPM1 = 2, 
  MSP430_POWER_LPM2 = 3, 
  MSP430_POWER_LPM3 = 4, 
  MSP430_POWER_LPM4 = 5
};

static inline void __nesc_disable_interrupt(void );





static inline void __nesc_enable_interrupt(void );




typedef bool __nesc_atomic_t;
__nesc_atomic_t __nesc_atomic_start(void );
void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts);






__nesc_atomic_t __nesc_atomic_start(void )  ;







void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)  ;
# 33 "/opt/tinyos-2.0.2/tos/platforms/telosb/hardware.h"
static inline void TOSH_SET_SIMO0_PIN();
#line 33
static inline void TOSH_CLR_SIMO0_PIN();
#line 33
static inline void TOSH_MAKE_SIMO0_OUTPUT();
static inline void TOSH_SET_UCLK0_PIN();
#line 34
static inline void TOSH_CLR_UCLK0_PIN();
#line 34
static inline void TOSH_MAKE_UCLK0_OUTPUT();
#line 76
enum __nesc_unnamed4258 {

  TOSH_HUMIDITY_ADDR = 5, 
  TOSH_HUMIDTEMP_ADDR = 3, 
  TOSH_HUMIDITY_RESET = 0x1E
};



static inline void TOSH_SET_FLASH_CS_PIN();
#line 85
static inline void TOSH_CLR_FLASH_CS_PIN();
#line 85
static inline void TOSH_MAKE_FLASH_CS_OUTPUT();
static inline void TOSH_SET_FLASH_HOLD_PIN();
#line 86
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT();
# 33 "result.h"
struct public_key_msg {

  uint8_t len;
  uint8_t x[24];
  uint8_t y[24];
  uint8_t id;
} __attribute((packed)) ;
typedef struct public_key_msg public_key_msg;

struct private_key_msg {

  uint8_t len;
  uint8_t d[24];
  uint8_t id;
} __attribute((packed)) ;
typedef struct private_key_msg private_key_msg;

struct time_msg {

  uint32_t t;
  uint8_t type;
  uint8_t pass;
  uint8_t id;
} __attribute((packed)) ;
typedef struct time_msg time_msg;

struct packet_msg {

  uint8_t c_len;
  uint8_t content[52];
  uint8_t r_len;
  uint8_t r[24];
  uint8_t s[24];
} __attribute((packed)) ;
typedef struct packet_msg packet_msg;

struct ecdh_key_msg {

  uint8_t len;
  uint8_t k[24];
  uint8_t id;
} __attribute((packed)) ;
typedef struct ecdh_key_msg ecdh_key_msg;

struct uint8_msg {

  uint8_t len;
  uint8_t content[101];
} __attribute((packed)) ;
typedef struct uint8_msg uint8_msg;

enum __nesc_unnamed4259 {
  AM_PUBLIC_KEY_MSG = 11, 
  AM_PRIVATE_KEY_MSG = 12, 
  AM_TIME_MSG = 13, 
  AM_PACKET_MSG = 14, 
  AM_ECDH_KEY_MSG = 15, 
  AM_UINT8_MSG = 16
};
# 138 "NN.h"
typedef uint16_t NN_DIGIT;
typedef uint32_t NN_DOUBLE_DIGIT;


typedef uint8_t NN_UINT;
typedef uint16_t NN_UINT2;
#line 209
struct Barrett {
  NN_DIGIT mu[2 * (160 / 16 + 1) + 1];
  NN_UINT mu_len;
  NN_UINT km;
};
typedef struct Barrett Barrett;
# 71 "ECC.h"
struct Curve {


  NN_DIGIT a[160 / 16 + 1];
  NN_DIGIT b[160 / 16 + 1];


  bool a_minus3;


  bool a_zero;

  bool a_one;
};
typedef struct Curve Curve;

struct Point {


  NN_DIGIT x[160 / 16 + 1];
  NN_DIGIT y[160 / 16 + 1];
};
typedef struct Point Point;

struct ZCoordinate {

  NN_DIGIT z[160 / 16 + 1];
};
typedef struct ZCoordinate ZCoordinate;


struct Params {


  NN_DIGIT p[160 / 16 + 1];


  NN_DIGIT omega[160 / 16 + 1];


  Curve E;


  Point G;


  NN_DIGIT r[160 / 16 + 1];
};



typedef struct Params Params;


struct TPParams {


  NN_DIGIT p[160 / 16 + 1];


  Curve E;


  NN_DIGIT m[160 / 16 + 1];


  NN_DIGIT c[160 / 16 + 1];


  Point P;
};
typedef struct TPParams TPParams;


struct PointSlope {
  bool dbl;
  Point P;
  NN_DIGIT slope[160 / 16 + 1];
  struct PointSlope *next;
};
typedef struct PointSlope PointSlope;
# 38 "sha1.h"
enum __nesc_unnamed4260 {

  shaSuccess = 0, 
  shaNull, 
  shaInputTooLong, 
  shaStateError
};
#line 65
#line 52
typedef struct SHA1Context {

  uint32_t Intermediate_Hash[20 / 4];

  uint32_t Length_Low;
  uint32_t Length_High;


  uint16_t Message_Block_Index;
  uint8_t Message_Block[64];

  int Computed;
  int Corrupted;
} SHA1Context;
# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.h"
enum __nesc_unnamed4261 {
  MSP430TIMER_CM_NONE = 0, 
  MSP430TIMER_CM_RISING = 1, 
  MSP430TIMER_CM_FALLING = 2, 
  MSP430TIMER_CM_BOTH = 3, 

  MSP430TIMER_STOP_MODE = 0, 
  MSP430TIMER_UP_MODE = 1, 
  MSP430TIMER_CONTINUOUS_MODE = 2, 
  MSP430TIMER_UPDOWN_MODE = 3, 

  MSP430TIMER_TACLK = 0, 
  MSP430TIMER_TBCLK = 0, 
  MSP430TIMER_ACLK = 1, 
  MSP430TIMER_SMCLK = 2, 
  MSP430TIMER_INCLK = 3, 

  MSP430TIMER_CLOCKDIV_1 = 0, 
  MSP430TIMER_CLOCKDIV_2 = 1, 
  MSP430TIMER_CLOCKDIV_4 = 2, 
  MSP430TIMER_CLOCKDIV_8 = 3
};
#line 64
#line 51
typedef struct __nesc_unnamed4262 {

  int ccifg : 1;
  int cov : 1;
  int out : 1;
  int cci : 1;
  int ccie : 1;
  int outmod : 3;
  int cap : 1;
  int clld : 2;
  int scs : 1;
  int ccis : 2;
  int cm : 2;
} msp430_compare_control_t;
#line 76
#line 66
typedef struct __nesc_unnamed4263 {

  int taifg : 1;
  int taie : 1;
  int taclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tassel : 2;
  int _unused1 : 6;
} msp430_timer_a_control_t;
#line 91
#line 78
typedef struct __nesc_unnamed4264 {

  int tbifg : 1;
  int tbie : 1;
  int tbclr : 1;
  int _unused0 : 1;
  int mc : 2;
  int id : 2;
  int tbssel : 2;
  int _unused1 : 1;
  int cntl : 2;
  int tbclgrp : 2;
  int _unused2 : 1;
} msp430_timer_b_control_t;
# 29 "/opt/tinyos-2.0.2/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4265 {
#line 29
  int notUsed;
} 
#line 29
TMilli;
typedef struct __nesc_unnamed4266 {
#line 30
  int notUsed;
} 
#line 30
T32khz;
typedef struct __nesc_unnamed4267 {
#line 31
  int notUsed;
} 
#line 31
TMicro;
# 30 "/opt/tinyos-2.0.2/tos/types/Leds.h"
enum __nesc_unnamed4268 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 39 "/opt/tinyos-2.0.2/tos/chips/cc2420/CC2420.h"
typedef uint8_t cc2420_status_t;
#line 59
#line 45
typedef nx_struct cc2420_header_t {
  nxle_uint8_t length;
  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;






  nxle_uint8_t type;
} __attribute__((packed)) cc2420_header_t;





#line 64
typedef nx_struct cc2420_footer_t {
} __attribute__((packed)) cc2420_footer_t;
#line 86
#line 71
typedef nx_struct cc2420_metadata_t {
  nx_uint8_t tx_power;
  nx_uint8_t rssi;
  nx_uint8_t lqi;
  nx_bool crc;
  nx_bool ack;
  nx_uint16_t time;
  nx_uint16_t rxInterval;
} __attribute__((packed)) 






cc2420_metadata_t;





#line 89
typedef nx_struct cc2420_packet_t {
  cc2420_header_t packet;
  nx_uint8_t data[];
} __attribute__((packed)) cc2420_packet_t;
#line 123
enum __nesc_unnamed4269 {

  MAC_HEADER_SIZE = sizeof(cc2420_header_t ) - 1, 

  MAC_FOOTER_SIZE = sizeof(uint16_t ), 

  MAC_PACKET_SIZE = MAC_HEADER_SIZE + 102 + MAC_FOOTER_SIZE
};

enum cc2420_enums {
  CC2420_TIME_ACK_TURNAROUND = 7, 
  CC2420_TIME_VREN = 20, 
  CC2420_TIME_SYMBOL = 2, 
  CC2420_BACKOFF_PERIOD = 20 / CC2420_TIME_SYMBOL, 
  CC2420_MIN_BACKOFF = 20 / CC2420_TIME_SYMBOL, 
  CC2420_ACK_WAIT_DELAY = 256
};

enum cc2420_status_enums {
  CC2420_STATUS_RSSI_VALID = 1 << 1, 
  CC2420_STATUS_LOCK = 1 << 2, 
  CC2420_STATUS_TX_ACTIVE = 1 << 3, 
  CC2420_STATUS_ENC_BUSY = 1 << 4, 
  CC2420_STATUS_TX_UNDERFLOW = 1 << 5, 
  CC2420_STATUS_XOSC16M_STABLE = 1 << 6
};

enum cc2420_config_reg_enums {
  CC2420_SNOP = 0x00, 
  CC2420_SXOSCON = 0x01, 
  CC2420_STXCAL = 0x02, 
  CC2420_SRXON = 0x03, 
  CC2420_STXON = 0x04, 
  CC2420_STXONCCA = 0x05, 
  CC2420_SRFOFF = 0x06, 
  CC2420_SXOSCOFF = 0x07, 
  CC2420_SFLUSHRX = 0x08, 
  CC2420_SFLUSHTX = 0x09, 
  CC2420_SACK = 0x0a, 
  CC2420_SACKPEND = 0x0b, 
  CC2420_SRXDEC = 0x0c, 
  CC2420_STXENC = 0x0d, 
  CC2420_SAES = 0x0e, 
  CC2420_MAIN = 0x10, 
  CC2420_MDMCTRL0 = 0x11, 
  CC2420_MDMCTRL1 = 0x12, 
  CC2420_RSSI = 0x13, 
  CC2420_SYNCWORD = 0x14, 
  CC2420_TXCTRL = 0x15, 
  CC2420_RXCTRL0 = 0x16, 
  CC2420_RXCTRL1 = 0x17, 
  CC2420_FSCTRL = 0x18, 
  CC2420_SECCTRL0 = 0x19, 
  CC2420_SECCTRL1 = 0x1a, 
  CC2420_BATTMON = 0x1b, 
  CC2420_IOCFG0 = 0x1c, 
  CC2420_IOCFG1 = 0x1d, 
  CC2420_MANFIDL = 0x1e, 
  CC2420_MANFIDH = 0x1f, 
  CC2420_FSMTC = 0x20, 
  CC2420_MANAND = 0x21, 
  CC2420_MANOR = 0x22, 
  CC2420_AGCCTRL = 0x23, 
  CC2420_AGCTST0 = 0x24, 
  CC2420_AGCTST1 = 0x25, 
  CC2420_AGCTST2 = 0x26, 
  CC2420_FSTST0 = 0x27, 
  CC2420_FSTST1 = 0x28, 
  CC2420_FSTST2 = 0x29, 
  CC2420_FSTST3 = 0x2a, 
  CC2420_RXBPFTST = 0x2b, 
  CC2420_FMSTATE = 0x2c, 
  CC2420_ADCTST = 0x2d, 
  CC2420_DACTST = 0x2e, 
  CC2420_TOPTST = 0x2f, 
  CC2420_TXFIFO = 0x3e, 
  CC2420_RXFIFO = 0x3f
};

enum cc2420_ram_addr_enums {
  CC2420_RAM_TXFIFO = 0x000, 
  CC2420_RAM_RXFIFO = 0x080, 
  CC2420_RAM_KEY0 = 0x100, 
  CC2420_RAM_RXNONCE = 0x110, 
  CC2420_RAM_SABUF = 0x120, 
  CC2420_RAM_KEY1 = 0x130, 
  CC2420_RAM_TXNONCE = 0x140, 
  CC2420_RAM_CBCSTATE = 0x150, 
  CC2420_RAM_IEEEADR = 0x160, 
  CC2420_RAM_PANID = 0x168, 
  CC2420_RAM_SHORTADR = 0x16a
};

enum cc2420_nonce_enums {
  CC2420_NONCE_BLOCK_COUNTER = 0, 
  CC2420_NONCE_KEY_SEQ_COUNTER = 2, 
  CC2420_NONCE_FRAME_COUNTER = 3, 
  CC2420_NONCE_SOURCE_ADDRESS = 7, 
  CC2420_NONCE_FLAGS = 15
};

enum cc2420_main_enums {
  CC2420_MAIN_RESETn = 15, 
  CC2420_MAIN_ENC_RESETn = 14, 
  CC2420_MAIN_DEMOD_RESETn = 13, 
  CC2420_MAIN_MOD_RESETn = 12, 
  CC2420_MAIN_FS_RESETn = 11, 
  CC2420_MAIN_XOSC16M_BYPASS = 0
};

enum cc2420_mdmctrl0_enums {
  CC2420_MDMCTRL0_RESERVED_FRAME_MODE = 13, 
  CC2420_MDMCTRL0_PAN_COORDINATOR = 12, 
  CC2420_MDMCTRL0_ADR_DECODE = 11, 
  CC2420_MDMCTRL0_CCA_HYST = 8, 
  CC2420_MDMCTRL0_CCA_MOD = 6, 
  CC2420_MDMCTRL0_AUTOCRC = 5, 
  CC2420_MDMCTRL0_AUTOACK = 4, 
  CC2420_MDMCTRL0_PREAMBLE_LENGTH = 0
};

enum cc2420_mdmctrl1_enums {
  CC2420_MDMCTRL1_CORR_THR = 6, 
  CC2420_MDMCTRL1_DEMOD_AVG_MODE = 5, 
  CC2420_MDMCTRL1_MODULATION_MODE = 4, 
  CC2420_MDMCTRL1_TX_MODE = 2, 
  CC2420_MDMCTRL1_RX_MODE = 0
};

enum cc2420_rssi_enums {
  CC2420_RSSI_CCA_THR = 8, 
  CC2420_RSSI_RSSI_VAL = 0
};

enum cc2420_syncword_enums {
  CC2420_SYNCWORD_SYNCWORD = 0
};

enum cc2420_txctrl_enums {
  CC2420_TXCTRL_TXMIXBUF_CUR = 14, 
  CC2420_TXCTRL_TX_TURNAROUND = 13, 
  CC2420_TXCTRL_TXMIX_CAP_ARRAY = 11, 
  CC2420_TXCTRL_TXMIX_CURRENT = 9, 
  CC2420_TXCTRL_PA_CURRENT = 6, 
  CC2420_TXCTRL_RESERVED = 5, 
  CC2420_TXCTRL_PA_LEVEL = 0
};

enum cc2420_rxctrl0_enums {
  CC2420_RXCTRL0_RXMIXBUF_CUR = 12, 
  CC2420_RXCTRL0_HIGH_LNA_GAIN = 10, 
  CC2420_RXCTRL0_MED_LNA_GAIN = 8, 
  CC2420_RXCTRL0_LOW_LNA_GAIN = 6, 
  CC2420_RXCTRL0_HIGH_LNA_CURRENT = 4, 
  CC2420_RXCTRL0_MED_LNA_CURRENT = 2, 
  CC2420_RXCTRL0_LOW_LNA_CURRENT = 0
};

enum cc2420_rxctrl1_enums {
  CC2420_RXCTRL1_RXBPF_LOCUR = 13, 
  CC2420_RXCTRL1_RXBPF_MIDCUR = 12, 
  CC2420_RXCTRL1_LOW_LOWGAIN = 11, 
  CC2420_RXCTRL1_MED_LOWGAIN = 10, 
  CC2420_RXCTRL1_HIGH_HGM = 9, 
  CC2420_RXCTRL1_MED_HGM = 8, 
  CC2420_RXCTRL1_LNA_CAP_ARRAY = 6, 
  CC2420_RXCTRL1_RXMIX_TAIL = 4, 
  CC2420_RXCTRL1_RXMIX_VCM = 2, 
  CC2420_RXCTRL1_RXMIX_CURRENT = 0
};

enum cc2420_rsctrl_enums {
  CC2420_FSCTRL_LOCK_THR = 14, 
  CC2420_FSCTRL_CAL_DONE = 13, 
  CC2420_FSCTRL_CAL_RUNNING = 12, 
  CC2420_FSCTRL_LOCK_LENGTH = 11, 
  CC2420_FSCTRL_LOCK_STATUS = 10, 
  CC2420_FSCTRL_FREQ = 0
};

enum cc2420_secctrl0_enums {
  CC2420_SECCTRL0_RXFIFO_PROTECTION = 9, 
  CC2420_SECCTRL0_SEC_CBC_HEAD = 8, 
  CC2420_SECCTRL0_SEC_SAKEYSEL = 7, 
  CC2420_SECCTRL0_SEC_TXKEYSEL = 6, 
  CC2420_SECCTRL0_SEC_RXKEYSEL = 5, 
  CC2420_SECCTRL0_SEC_M = 2, 
  CC2420_SECCTRL0_SEC_MODE = 0
};

enum cc2420_secctrl1_enums {
  CC2420_SECCTRL1_SEC_TXL = 8, 
  CC2420_SECCTRL1_SEC_RXL = 0
};

enum cc2420_battmon_enums {
  CC2420_BATTMON_BATT_OK = 6, 
  CC2420_BATTMON_BATTMON_EN = 5, 
  CC2420_BATTMON_BATTMON_VOLTAGE = 0
};

enum cc2420_iocfg0_enums {
  CC2420_IOCFG0_BCN_ACCEPT = 11, 
  CC2420_IOCFG0_FIFO_POLARITY = 10, 
  CC2420_IOCFG0_FIFOP_POLARITY = 9, 
  CC2420_IOCFG0_SFD_POLARITY = 8, 
  CC2420_IOCFG0_CCA_POLARITY = 7, 
  CC2420_IOCFG0_FIFOP_THR = 0
};

enum cc2420_iocfg1_enums {
  CC2420_IOCFG1_HSSD_SRC = 10, 
  CC2420_IOCFG1_SFDMUX = 5, 
  CC2420_IOCFG1_CCAMUX = 0
};

enum cc2420_manfidl_enums {
  CC2420_MANFIDL_PARTNUM = 12, 
  CC2420_MANFIDL_MANFID = 0
};

enum cc2420_manfidh_enums {
  CC2420_MANFIDH_VERSION = 12, 
  CC2420_MANFIDH_PARTNUM = 0
};

enum cc2420_fsmtc_enums {
  CC2420_FSMTC_TC_RXCHAIN2RX = 13, 
  CC2420_FSMTC_TC_SWITCH2TX = 10, 
  CC2420_FSMTC_TC_PAON2TX = 6, 
  CC2420_FSMTC_TC_TXEND2SWITCH = 3, 
  CC2420_FSMTC_TC_TXEND2PAOFF = 0
};

enum cc2420_sfdmux_enums {
  CC2420_SFDMUX_SFD = 0, 
  CC2420_SFDMUX_XOSC16M_STABLE = 24
};
# 6 "/opt/tinyos-2.0.2/tos/types/AM.h"
typedef nx_uint8_t nx_am_id_t;
typedef nx_uint8_t nx_am_group_t;
typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;
typedef uint8_t am_group_t;
typedef uint16_t am_addr_t;

enum __nesc_unnamed4270 {
  AM_BROADCAST_ADDR = 0xffff
};









enum __nesc_unnamed4271 {
  TOS_AM_GROUP = 0x22, 
  TOS_AM_ADDRESS = 1
};
# 72 "/opt/tinyos-2.0.2/tos/lib/serial/Serial.h"
typedef uint8_t uart_id_t;



enum __nesc_unnamed4272 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4273 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4274 {
  SERIAL_PROTO_ACK = 67, 
  SERIAL_PROTO_PACKET_ACK = 68, 
  SERIAL_PROTO_PACKET_NOACK = 69, 
  SERIAL_PROTO_PACKET_UNKNOWN = 255
};
#line 110
#line 98
typedef struct radio_stats {
  uint8_t version;
  uint8_t flags;
  uint8_t reserved;
  uint8_t platform;
  uint16_t MTU;
  uint16_t radio_crc_fail;
  uint16_t radio_queue_drops;
  uint16_t serial_crc_fail;
  uint16_t serial_tx_fail;
  uint16_t serial_short_packets;
  uint16_t serial_proto_drops;
} radio_stats_t;







#line 112
typedef nx_struct serial_header {
  nx_am_addr_t dest;
  nx_am_addr_t src;
  nx_uint8_t length;
  nx_am_group_t group;
  nx_am_id_t type;
} __attribute__((packed)) serial_header_t;




#line 120
typedef nx_struct serial_packet {
  serial_header_t header;
  nx_uint8_t data[];
} __attribute__((packed)) serial_packet_t;
# 48 "/opt/tinyos-2.0.2/tos/platforms/telosa/platform_message.h"
#line 45
typedef union message_header {
  cc2420_header_t cc2420;
  serial_header_t serial;
} message_header_t;



#line 50
typedef union TOSRadioFooter {
  cc2420_footer_t cc2420;
} message_footer_t;



#line 54
typedef union TOSRadioMetadata {
  cc2420_metadata_t cc2420;
} message_metadata_t;
# 19 "/opt/tinyos-2.0.2/tos/types/message.h"
#line 14
typedef nx_struct message_t {
  nx_uint8_t header[sizeof(message_header_t )];
  nx_uint8_t data[102];
  nx_uint8_t footer[sizeof(message_footer_t )];
  nx_uint8_t metadata[sizeof(message_metadata_t )];
} __attribute__((packed)) message_t;
# 80 "/opt/tinyos-2.0.2/tos/system/crc.h"
static uint16_t crcByte(uint16_t crc, uint8_t b);
# 56 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/msp430usart.h"
#line 48
typedef enum __nesc_unnamed4275 {

  USART_NONE = 0, 
  USART_UART = 1, 
  USART_UART_TX = 2, 
  USART_UART_RX = 3, 
  USART_SPI = 4, 
  USART_I2C = 5
} msp430_usartmode_t;










#line 58
typedef struct __nesc_unnamed4276 {
  unsigned int swrst : 1;
  unsigned int mm : 1;
  unsigned int sync : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
} __attribute((packed))  msp430_uctl_t;









#line 69
typedef struct __nesc_unnamed4277 {
  unsigned int txept : 1;
  unsigned int stc : 1;
  unsigned int txwake : 1;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
} __attribute((packed))  msp430_utctl_t;










#line 79
typedef struct __nesc_unnamed4278 {
  unsigned int rxerr : 1;
  unsigned int rxwake : 1;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int brk : 1;
  unsigned int oe : 1;
  unsigned int pe : 1;
  unsigned int fe : 1;
} __attribute((packed))  msp430_urctl_t;
#line 116
#line 99
typedef struct __nesc_unnamed4279 {
  unsigned int ubr : 16;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int  : 3;

  unsigned int  : 1;
  unsigned int stc : 1;
  unsigned int  : 2;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int ckph : 1;
  unsigned int  : 0;
} msp430_spi_config_t;





#line 118
typedef struct __nesc_unnamed4280 {
  uint16_t ubr;
  uint8_t uctl;
  uint8_t utctl;
} msp430_spi_registers_t;




#line 124
typedef union __nesc_unnamed4281 {
  msp430_spi_config_t spiConfig;
  msp430_spi_registers_t spiRegisters;
} msp430_spi_union_config_t;
#line 169
#line 150
typedef enum __nesc_unnamed4282 {

  UBR_32KHZ_1200 = 0x001B, UMCTL_32KHZ_1200 = 0x94, 
  UBR_32KHZ_1800 = 0x0012, UMCTL_32KHZ_1800 = 0x84, 
  UBR_32KHZ_2400 = 0x000D, UMCTL_32KHZ_2400 = 0x6D, 
  UBR_32KHZ_4800 = 0x0006, UMCTL_32KHZ_4800 = 0x77, 
  UBR_32KHZ_9600 = 0x0003, UMCTL_32KHZ_9600 = 0x29, 

  UBR_1MHZ_1200 = 0x0369, UMCTL_1MHZ_1200 = 0x7B, 
  UBR_1MHZ_1800 = 0x0246, UMCTL_1MHZ_1800 = 0x55, 
  UBR_1MHZ_2400 = 0x01B4, UMCTL_1MHZ_2400 = 0xDF, 
  UBR_1MHZ_4800 = 0x00DA, UMCTL_1MHZ_4800 = 0xAA, 
  UBR_1MHZ_9600 = 0x006D, UMCTL_1MHZ_9600 = 0x44, 
  UBR_1MHZ_19200 = 0x0036, UMCTL_1MHZ_19200 = 0xB5, 
  UBR_1MHZ_38400 = 0x001B, UMCTL_1MHZ_38400 = 0x94, 
  UBR_1MHZ_57600 = 0x0012, UMCTL_1MHZ_57600 = 0x84, 
  UBR_1MHZ_76800 = 0x000D, UMCTL_1MHZ_76800 = 0x6D, 
  UBR_1MHZ_115200 = 0x0009, UMCTL_1MHZ_115200 = 0x10, 
  UBR_1MHZ_230400 = 0x0004, UMCTL_1MHZ_230400 = 0x55
} msp430_uart_rate_t;
#line 200
#line 171
typedef struct __nesc_unnamed4283 {
  unsigned int ubr : 16;

  unsigned int umctl : 8;

  unsigned int  : 1;
  unsigned int mm : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int clen : 1;
  unsigned int spb : 1;
  unsigned int pev : 1;
  unsigned int pena : 1;
  unsigned int  : 0;

  unsigned int  : 3;
  unsigned int urxse : 1;
  unsigned int ssel : 2;
  unsigned int ckpl : 1;
  unsigned int  : 1;

  unsigned int  : 2;
  unsigned int urxwie : 1;
  unsigned int urxeie : 1;
  unsigned int  : 4;
  unsigned int  : 0;

  unsigned int utxe : 1;
  unsigned int urxe : 1;
} msp430_uart_config_t;








#line 202
typedef struct __nesc_unnamed4284 {
  uint16_t ubr;
  uint8_t umctl;
  uint8_t uctl;
  uint8_t utctl;
  uint8_t urctl;
  uint8_t ume;
} msp430_uart_registers_t;




#line 211
typedef union __nesc_unnamed4285 {
  msp430_uart_config_t uartConfig;
  msp430_uart_registers_t uartRegisters;
} msp430_uart_union_config_t;

msp430_uart_union_config_t msp430_uart_default_config = { 
{ 
.utxe = 1, 
.urxe = 1, 
.ubr = UBR_1MHZ_57600, 
.umctl = UMCTL_1MHZ_57600, 
.ssel = 0x02, 
.pena = 0, 
.pev = 0, 
.spb = 0, 
.clen = 1, 
.listen = 0, 
.mm = 0, 
.ckpl = 0, 
.urxse = 0, 
.urxeie = 1, 
.urxwie = 0, 
.utxe = 1, 
.urxe = 1 } };
#line 248
#line 240
typedef struct __nesc_unnamed4286 {
  unsigned int i2cstt : 1;
  unsigned int i2cstp : 1;
  unsigned int i2cstb : 1;
  unsigned int i2cctrx : 1;
  unsigned int i2cssel : 2;
  unsigned int i2ccrm : 1;
  unsigned int i2cword : 1;
} __attribute((packed))  msp430_i2ctctl_t;
#line 276
#line 253
typedef struct __nesc_unnamed4287 {
  unsigned int  : 1;
  unsigned int mst : 1;
  unsigned int  : 1;
  unsigned int listen : 1;
  unsigned int xa : 1;
  unsigned int  : 1;
  unsigned int txdmaen : 1;
  unsigned int rxdmaen : 1;

  unsigned int  : 4;
  unsigned int i2cssel : 2;
  unsigned int i2crm : 1;
  unsigned int i2cword : 1;

  unsigned int i2cpsc : 8;

  unsigned int i2csclh : 8;

  unsigned int i2cscll : 8;

  unsigned int i2coa : 10;
  unsigned int  : 6;
} msp430_i2c_config_t;








#line 278
typedef struct __nesc_unnamed4288 {
  uint8_t uctl;
  uint8_t i2ctctl;
  uint8_t i2cpsc;
  uint8_t i2csclh;
  uint8_t i2cscll;
  uint16_t i2coa;
} msp430_i2c_registers_t;




#line 287
typedef union __nesc_unnamed4289 {
  msp430_i2c_config_t i2cConfig;
  msp430_i2c_registers_t i2cRegisters;
} msp430_i2c_union_config_t;
# 33 "/opt/tinyos-2.0.2/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
typedef TMilli testECDSAM$LocalTime$precision_tag;
typedef TMilli testECDSAM$myTimer$precision_tag;
enum /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Timer*/Msp430Timer32khzC$0$__nesc_unnamed4290 {
  Msp430Timer32khzC$0$ALARM_ID = 0U
};
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$frequency_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$frequency_tag /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$size_type;
typedef T32khz /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$frequency_tag;
typedef /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$frequency_tag /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$precision_tag;
typedef uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$size_type;
typedef TMilli /*CounterMilli32C.Transform*/TransformCounterC$0$to_precision_tag;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type;
typedef T32khz /*CounterMilli32C.Transform*/TransformCounterC$0$from_precision_tag;
typedef uint16_t /*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type;
typedef uint32_t /*CounterMilli32C.Transform*/TransformCounterC$0$upper_count_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC$0$from_precision_tag /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$size_type;
typedef /*CounterMilli32C.Transform*/TransformCounterC$0$to_precision_tag /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$precision_tag;
typedef /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$size_type;
typedef TMilli /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type;
typedef T32khz /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_precision_tag;
typedef uint16_t /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_precision_tag /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$precision_tag;
typedef /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$size_type;
typedef TMilli /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type;
typedef /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$precision_tag /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$precision_tag;
typedef TMilli /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$precision_tag;
typedef /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$precision_tag /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$precision_tag;
typedef TMilli /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$LocalTime$precision_tag;
typedef /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$precision_tag /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$precision_tag;
typedef uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$size_type;
enum /*PlatformSerialC.UartC*/Msp430Uart1C$0$__nesc_unnamed4291 {
  Msp430Uart1C$0$CLIENT_ID = 0U
};
typedef T32khz /*Msp430Uart1P.UartP*/Msp430UartP$0$Counter$precision_tag;
typedef uint16_t /*Msp430Uart1P.UartP*/Msp430UartP$0$Counter$size_type;
enum /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C$0$__nesc_unnamed4292 {
  Msp430Usart1C$0$CLIENT_ID = 0U
};
# 51 "/opt/tinyos-2.0.2/tos/interfaces/Init.nc"
static error_t PlatformP$Init$init(void );
#line 51
static error_t MotePlatformC$Init$init(void );
# 35 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP$Msp430ClockInit$defaultInitClocks(void );
#line 32
static void Msp430ClockP$Msp430ClockInit$default$initTimerB(void );



static void Msp430ClockP$Msp430ClockInit$defaultInitTimerA(void );
#line 31
static void Msp430ClockP$Msp430ClockInit$default$initTimerA(void );





static void Msp430ClockP$Msp430ClockInit$defaultInitTimerB(void );
#line 34
static void Msp430ClockP$Msp430ClockInit$defaultSetupDcoCalibrate(void );
#line 29
static void Msp430ClockP$Msp430ClockInit$default$setupDcoCalibrate(void );
static void Msp430ClockP$Msp430ClockInit$default$initClocks(void );
# 51 "/opt/tinyos-2.0.2/tos/interfaces/Init.nc"
static error_t Msp430ClockP$Init$init(void );
# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX0$fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Overflow$fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX1$fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$default$fired(
# 40 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x405d6c08);
# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX0$fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Overflow$fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX1$fired(void );
#line 28
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$default$fired(
# 40 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x405d6c08);
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get(void );
static bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$isOverflowPending(void );
# 33 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$default$captured(uint16_t time);
# 31 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$getControl(void );
# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Event$fired(void );
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$default$fired(void );
# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Timer$overflow(void );
# 33 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$default$captured(uint16_t time);
# 31 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$getControl(void );
# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Event$fired(void );
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$default$fired(void );
# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Timer$overflow(void );
# 33 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$default$captured(uint16_t time);
# 31 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Control$getControl(void );
# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Event$fired(void );
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$default$fired(void );
# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Timer$overflow(void );
# 33 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$default$captured(uint16_t time);
# 31 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$getControl(void );







static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$enableEvents(void );
#line 36
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$setControlAsCompare(void );



static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$disableEvents(void );
#line 33
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$clearPendingInterrupt(void );
# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Event$fired(void );
# 30 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEvent(uint16_t time);

static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEventFromNow(uint16_t delta);
# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$overflow(void );
# 33 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$default$captured(uint16_t time);
# 31 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$getControl(void );
# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Event$fired(void );
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$default$fired(void );
# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Timer$overflow(void );
# 33 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$default$captured(uint16_t time);
# 31 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$getControl(void );
# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Event$fired(void );
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$default$fired(void );
# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$overflow(void );
# 33 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$default$captured(uint16_t time);
# 31 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Control$getControl(void );
# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Event$fired(void );
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$default$fired(void );
# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Timer$overflow(void );
# 33 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$default$captured(uint16_t time);
# 31 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Control$getControl(void );
# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Event$fired(void );
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$default$fired(void );
# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Timer$overflow(void );
# 33 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$default$captured(uint16_t time);
# 31 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Control$getControl(void );
# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Event$fired(void );
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$default$fired(void );
# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Timer$overflow(void );
# 33 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$getEvent(void );
#line 75
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$default$captured(uint16_t time);
# 31 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Control$getControl(void );
# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Event$fired(void );
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$default$fired(void );
# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Timer$overflow(void );
# 56 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP$TaskBasic$postTask(
# 45 "/opt/tinyos-2.0.2/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x4053c5c0);
# 64 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP$TaskBasic$default$runTask(
# 45 "/opt/tinyos-2.0.2/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x4053c5c0);
# 46 "/opt/tinyos-2.0.2/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP$Scheduler$init(void );
#line 61
static void SchedulerBasicP$Scheduler$taskLoop(void );
#line 54
static bool SchedulerBasicP$Scheduler$runNextTask(void );
# 54 "/opt/tinyos-2.0.2/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC$McuPowerOverride$default$lowestState(void );
# 59 "/opt/tinyos-2.0.2/tos/interfaces/McuSleep.nc"
static void McuSleepC$McuSleep$sleep(void );
# 99 "/opt/tinyos-2.0.2/tos/interfaces/AMSend.nc"
static void testECDSAM$PubKeyMsg$sendDone(message_t *msg, error_t error);
# 49 "/opt/tinyos-2.0.2/tos/interfaces/Boot.nc"
static void testECDSAM$Boot$booted(void );
# 92 "/opt/tinyos-2.0.2/tos/interfaces/SplitControl.nc"
static void testECDSAM$SerialControl$startDone(error_t error);
#line 117
static void testECDSAM$SerialControl$stopDone(error_t error);
# 72 "/opt/tinyos-2.0.2/tos/lib/timer/Timer.nc"
static void testECDSAM$myTimer$fired(void );
# 99 "/opt/tinyos-2.0.2/tos/interfaces/AMSend.nc"
static void testECDSAM$PacketMsg$sendDone(message_t *msg, error_t error);
#line 99
static void testECDSAM$PriKeyMsg$sendDone(message_t *msg, error_t error);
#line 99
static void testECDSAM$TimeMsg$sendDone(message_t *msg, error_t error);
# 51 "/opt/tinyos-2.0.2/tos/interfaces/Init.nc"
static error_t LedsP$Init$init(void );
# 56 "/opt/tinyos-2.0.2/tos/interfaces/Leds.nc"
static void LedsP$Leds$led0Toggle(void );
# 85 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP$22$IO$selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP$22$IO$selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP$23$IO$selectIOFunc(void );
#line 78
static void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP$23$IO$selectModuleFunc(void );






static void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP$33$IO$selectIOFunc(void );
#line 85
static void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP$34$IO$selectIOFunc(void );
#line 85
static void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP$35$IO$selectIOFunc(void );
#line 44
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$toggle(void );
#line 71
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$set(void );
#line 71
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$set(void );
#line 71
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$makeOutput(void );
#line 34
static void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$set(void );
# 31 "/opt/tinyos-2.0.2/tos/interfaces/GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$toggle(void );



static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$makeOutput(void );
#line 29
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$set(void );





static void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$makeOutput(void );
#line 29
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$set(void );





static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$makeOutput(void );
#line 29
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$set(void );
# 41 "/opt/tinyos-2.0.2/tos/interfaces/Random.nc"
static uint16_t RandomLfsrP$Random$rand16(void );
# 51 "/opt/tinyos-2.0.2/tos/interfaces/Init.nc"
static error_t RandomLfsrP$Init$init(void );
# 65 "NN.nc"
static NN_DIGIT NNM$NN$LShift(NN_DIGIT *a, NN_DIGIT *b, NN_UINT c, NN_UINT digits);
#line 81
static void NNM$NN$ModSmall(NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits);
#line 69
static NN_DIGIT NNM$NN$AddDigitMult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT c, NN_DIGIT *d, NN_UINT digits);









static void NNM$NN$ModBarrett(NN_DIGIT *a, NN_DIGIT *b, NN_UINT bDigits, NN_DIGIT *c, NN_UINT cDigits);
#line 123
static unsigned int NNM$NN$Digits(NN_DIGIT *a, NN_UINT digits);
#line 85
static void NNM$NN$ModSub(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits);
#line 113
static int NNM$NN$Cmp(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits);
#line 75
static void NNM$NN$ModBarrettInit(NN_DIGIT *c, NN_UINT cDigits, Barrett *pbuf);
#line 49
static void NNM$NN$AssignDigit(NN_DIGIT *a, NN_DIGIT b, NN_UINT digits);
#line 77
static void NNM$NN$BarrettSetBuf(Barrett *pbuf);
#line 89
static void NNM$NN$ModMult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits);



static void NNM$NN$ModMultOpt(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_DIGIT *omega, NN_UINT digits);
#line 47
static void NNM$NN$Assign(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits);
#line 41
static void NNM$NN$Decode(NN_DIGIT *a, NN_UINT digits, unsigned char *b, NN_UINT len);

static void NNM$NN$Encode(unsigned char *a, NN_UINT digits, NN_DIGIT *b, NN_UINT len);
#line 73
static void NNM$NN$Mod(NN_DIGIT *a, NN_DIGIT *b, NN_UINT bDigits, NN_DIGIT *c, NN_UINT cDigits);
#line 57
static NN_DIGIT NNM$NN$Add(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits);
#line 83
static void NNM$NN$ModAdd(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits);
#line 117
static int NNM$NN$One(NN_DIGIT *a, NN_UINT digits);

static unsigned int NNM$NN$Bits(NN_DIGIT *a, NN_UINT digits);





static int NNM$NN$Equal(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits);
#line 99
static void NNM$NN$ModSqrOpt(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *d, NN_DIGIT *omega, NN_UINT digits);
#line 51
static void NNM$NN$AssignZero(NN_DIGIT *a, NN_UINT digits);
#line 67
static NN_DIGIT NNM$NN$RShift(NN_DIGIT *a, NN_DIGIT *b, NN_UINT c, NN_UINT digits);
#line 103
static void NNM$NN$ModInv(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits);
#line 115
static int NNM$NN$Zero(NN_DIGIT *a, NN_UINT digits);
# 100 "ECC.nc"
static Params *ECCM$ECC$get_param(void );
#line 40
static void ECCM$ECC$gen_private_key(NN_DIGIT *PrivateKey);
#line 38
static void ECCM$ECC$init(void );
#line 69
static void ECCM$ECC$add_proj(Point *P0, NN_DIGIT *Z0, Point *P1, NN_DIGIT *Z1, Point *P2, NN_DIGIT *Z2);
#line 54
static void ECCM$ECC$BarrettSetBuf(void );
#line 72
static void ECCM$ECC$dbl_proj(Point *P0, NN_DIGIT *Z0, Point *P1, NN_DIGIT *Z1);
#line 88
static void ECCM$ECC$win_mul_base(Point *P0, NN_DIGIT *n);
#line 58
static void ECCM$ECC$add(Point *P0, Point *P1, Point *P2);
#line 42
static void ECCM$ECC$gen_public_key(Point *PublicKey, NN_DIGIT *PrivateKey);
#line 80
static void ECCM$ECC$win_precompute(Point *baseP, Point *pointArray);
# 37 "CurveParam.nc"
static void secp160r1$CurveParam$get_param(Params *para);

static NN_UINT secp160r1$CurveParam$omega_mul(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *omega, NN_UINT digits);
# 41 "ECDSA.nc"
static void ECDSAM$ECDSA$sign(uint8_t *msg, uint8_t len, NN_DIGIT *r, NN_DIGIT *s, NN_DIGIT *d);
#line 38
static error_t ECDSAM$ECDSA$init(Point *pKey);





static uint8_t ECDSAM$ECDSA$verify(uint8_t *msg, uint8_t len, NN_DIGIT *r, NN_DIGIT *s, Point *Q);
# 37 "SHA1.nc"
static int SHA1M$SHA1$reset(SHA1Context *context);
static int SHA1M$SHA1$update(SHA1Context *context, const uint8_t *message_array, uint32_t length);
static int SHA1M$SHA1$digest(SHA1Context *context, uint8_t Message_Digest[20]);
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$fired(void );
# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$overflow(void );
# 92 "/opt/tinyos-2.0.2/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$startAt(/*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$size_type t0, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$stop(void );
# 51 "/opt/tinyos-2.0.2/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Init$init(void );
# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$overflow(void );
# 53 "/opt/tinyos-2.0.2/tos/lib/timer/Counter.nc"
static /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$size_type /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$get(void );






static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$overflow(void );
#line 53
static /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$size_type /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$get(void );
# 98 "/opt/tinyos-2.0.2/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type dt);
#line 105
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$fired(void );
# 71 "/opt/tinyos-2.0.2/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$overflow(void );
# 64 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask(void );
# 67 "/opt/tinyos-2.0.2/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired(void );
# 125 "/opt/tinyos-2.0.2/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow(void );
#line 118
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop(void );
# 64 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$runTask(void );
# 72 "/opt/tinyos-2.0.2/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired(void );
#line 72
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$default$fired(
# 37 "/opt/tinyos-2.0.2/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x40c93e08);
# 62 "/opt/tinyos-2.0.2/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(
# 37 "/opt/tinyos-2.0.2/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x40c93e08, 
# 62 "/opt/tinyos-2.0.2/tos/lib/timer/Timer.nc"
uint32_t dt);
# 50 "/opt/tinyos-2.0.2/tos/lib/timer/LocalTime.nc"
static uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$LocalTime$get(void );
# 71 "/opt/tinyos-2.0.2/tos/lib/timer/Counter.nc"
static void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow(void );
# 89 "/opt/tinyos-2.0.2/tos/interfaces/Send.nc"
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubSend$sendDone(message_t *msg, error_t error);
# 67 "/opt/tinyos-2.0.2/tos/interfaces/Receive.nc"
static message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubReceive$receive(message_t *msg, void *payload, uint8_t len);
# 69 "/opt/tinyos-2.0.2/tos/interfaces/AMSend.nc"
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMSend$send(
# 36 "/opt/tinyos-2.0.2/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x40ccd888, 
# 69 "/opt/tinyos-2.0.2/tos/interfaces/AMSend.nc"
am_addr_t addr, message_t *msg, uint8_t len);
#line 99
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMSend$default$sendDone(
# 36 "/opt/tinyos-2.0.2/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x40ccd888, 
# 99 "/opt/tinyos-2.0.2/tos/interfaces/AMSend.nc"
message_t *msg, error_t error);
# 67 "/opt/tinyos-2.0.2/tos/interfaces/Receive.nc"
static message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Receive$default$receive(
# 37 "/opt/tinyos-2.0.2/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x40cc8298, 
# 67 "/opt/tinyos-2.0.2/tos/interfaces/Receive.nc"
message_t *msg, void *payload, uint8_t len);
# 136 "/opt/tinyos-2.0.2/tos/interfaces/AMPacket.nc"
static am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$type(message_t *amsg);
# 83 "/opt/tinyos-2.0.2/tos/interfaces/SplitControl.nc"
static error_t SerialP$SplitControl$start(void );
# 64 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
static void SerialP$stopDoneTask$runTask(void );
#line 64
static void SerialP$RunTx$runTask(void );
# 51 "/opt/tinyos-2.0.2/tos/interfaces/Init.nc"
static error_t SerialP$Init$init(void );
# 43 "/opt/tinyos-2.0.2/tos/lib/serial/SerialFlush.nc"
static void SerialP$SerialFlush$flushDone(void );
#line 38
static void SerialP$SerialFlush$default$flush(void );
# 64 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
static void SerialP$startDoneTask$runTask(void );
# 83 "/opt/tinyos-2.0.2/tos/lib/serial/SerialFrameComm.nc"
static void SerialP$SerialFrameComm$dataReceived(uint8_t data);





static void SerialP$SerialFrameComm$putDone(void );
#line 74
static void SerialP$SerialFrameComm$delimiterReceived(void );
# 64 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
static void SerialP$defaultSerialFlushTask$runTask(void );
# 60 "/opt/tinyos-2.0.2/tos/lib/serial/SendBytePacket.nc"
static error_t SerialP$SendBytePacket$completeSend(void );
#line 51
static error_t SerialP$SendBytePacket$startSend(uint8_t first_byte);
# 64 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTask$runTask(void );
# 64 "/opt/tinyos-2.0.2/tos/interfaces/Send.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$send(
# 40 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40d8d160, 
# 64 "/opt/tinyos-2.0.2/tos/interfaces/Send.nc"
message_t *msg, uint8_t len);
#line 89
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$default$sendDone(
# 40 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40d8d160, 
# 89 "/opt/tinyos-2.0.2/tos/interfaces/Send.nc"
message_t *msg, error_t error);
# 64 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$signalSendDone$runTask(void );
# 67 "/opt/tinyos-2.0.2/tos/interfaces/Receive.nc"
static message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Receive$default$receive(
# 39 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40d90918, 
# 67 "/opt/tinyos-2.0.2/tos/interfaces/Receive.nc"
message_t *msg, void *payload, uint8_t len);
# 31 "/opt/tinyos-2.0.2/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$upperLength(
# 43 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40d8dc60, 
# 31 "/opt/tinyos-2.0.2/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$offset(
# 43 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40d8dc60);
# 23 "/opt/tinyos-2.0.2/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$dataLinkLength(
# 43 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40d8dc60, 
# 23 "/opt/tinyos-2.0.2/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t upperLen);
# 70 "/opt/tinyos-2.0.2/tos/lib/serial/SendBytePacket.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$nextByte(void );









static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$sendCompleted(error_t error);
# 51 "/opt/tinyos-2.0.2/tos/lib/serial/ReceiveBytePacket.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$startPacket(void );






static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$byteReceived(uint8_t data);










static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$endPacket(error_t result);
# 79 "/opt/tinyos-2.0.2/tos/interfaces/UartStream.nc"
static void HdlcTranslateC$UartStream$receivedByte(uint8_t byte);
#line 99
static void HdlcTranslateC$UartStream$receiveDone(uint8_t *buf, uint16_t len, error_t error);
#line 57
static void HdlcTranslateC$UartStream$sendDone(uint8_t *buf, uint16_t len, error_t error);
# 45 "/opt/tinyos-2.0.2/tos/lib/serial/SerialFrameComm.nc"
static error_t HdlcTranslateC$SerialFrameComm$putDelimiter(void );
#line 68
static void HdlcTranslateC$SerialFrameComm$resetReceive(void );
#line 54
static error_t HdlcTranslateC$SerialFrameComm$putData(uint8_t data);
# 55 "/opt/tinyos-2.0.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP$0$ResourceConfigure$unconfigure(
# 43 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40e27e98);
# 49 "/opt/tinyos-2.0.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP$0$ResourceConfigure$configure(
# 43 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40e27e98);
# 39 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartConfigure.nc"
static msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP$0$Msp430UartConfigure$default$getConfig(
# 49 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40e23010);
# 48 "/opt/tinyos-2.0.2/tos/interfaces/UartStream.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP$0$UartStream$send(uint8_t *buf, uint16_t len);
# 71 "/opt/tinyos-2.0.2/tos/lib/timer/Counter.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP$0$Counter$overflow(void );
# 110 "/opt/tinyos-2.0.2/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartResource$default$release(
# 48 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40e24540);
# 87 "/opt/tinyos-2.0.2/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartResource$default$immediateRequest(
# 48 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40e24540);
# 92 "/opt/tinyos-2.0.2/tos/interfaces/Resource.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartResource$granted(
# 48 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40e24540);
# 110 "/opt/tinyos-2.0.2/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP$0$Resource$release(
# 42 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40e27450);
# 87 "/opt/tinyos-2.0.2/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP$0$Resource$immediateRequest(
# 42 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40e27450);
# 92 "/opt/tinyos-2.0.2/tos/interfaces/Resource.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP$0$Resource$default$granted(
# 42 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40e27450);
# 54 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartInterrupts$rxDone(uint8_t data);
#line 49
static void /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartInterrupts$txDone(void );
# 143 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void HplMsp430Usart1P$Usart$enableUartRx(void );
#line 123
static void HplMsp430Usart1P$Usart$enableUart(void );
#line 97
static void HplMsp430Usart1P$Usart$resetUsart(bool reset);
#line 179
static void HplMsp430Usart1P$Usart$disableIntr(void );
#line 90
static void HplMsp430Usart1P$Usart$setUmctl(uint8_t umctl);
#line 133
static void HplMsp430Usart1P$Usart$enableUartTx(void );
#line 148
static void HplMsp430Usart1P$Usart$disableUartRx(void );
#line 182
static void HplMsp430Usart1P$Usart$enableIntr(void );
#line 207
static void HplMsp430Usart1P$Usart$clrIntr(void );
#line 80
static void HplMsp430Usart1P$Usart$setUbr(uint16_t ubr);
#line 224
static void HplMsp430Usart1P$Usart$tx(uint8_t data);
#line 128
static void HplMsp430Usart1P$Usart$disableUart(void );
#line 174
static void HplMsp430Usart1P$Usart$setModeUart(msp430_uart_union_config_t *config);
#line 158
static void HplMsp430Usart1P$Usart$disableSpi(void );
#line 138
static void HplMsp430Usart1P$Usart$disableUartTx(void );
# 74 "/opt/tinyos-2.0.2/tos/interfaces/AsyncStdControl.nc"
static error_t HplMsp430Usart1P$AsyncStdControl$start(void );









static error_t HplMsp430Usart1P$AsyncStdControl$stop(void );
# 54 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$Interrupts$default$rxDone(
# 39 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40eb5bd0, 
# 54 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$Interrupts$default$txDone(
# 39 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40eb5bd0);
# 54 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$RawInterrupts$rxDone(uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$RawInterrupts$txDone(void );
# 51 "/opt/tinyos-2.0.2/tos/interfaces/Init.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$Init$init(void );
# 43 "/opt/tinyos-2.0.2/tos/interfaces/ResourceQueue.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$FcfsQueue$isEmpty(void );
#line 60
static resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$FcfsQueue$dequeue(void );
# 51 "/opt/tinyos-2.0.2/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceRequested$default$immediateRequested(
# 55 "/opt/tinyos-2.0.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ef9a38);
# 55 "/opt/tinyos-2.0.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceConfigure$default$unconfigure(
# 60 "/opt/tinyos-2.0.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ef8df0);
# 49 "/opt/tinyos-2.0.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceConfigure$default$configure(
# 60 "/opt/tinyos-2.0.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ef8df0);
# 56 "/opt/tinyos-2.0.2/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceDefaultOwner$release(void );
# 110 "/opt/tinyos-2.0.2/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$Resource$release(
# 54 "/opt/tinyos-2.0.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ef9010);
# 87 "/opt/tinyos-2.0.2/tos/interfaces/Resource.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$Resource$immediateRequest(
# 54 "/opt/tinyos-2.0.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ef9010);
# 92 "/opt/tinyos-2.0.2/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$Resource$default$granted(
# 54 "/opt/tinyos-2.0.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ef9010);
# 80 "/opt/tinyos-2.0.2/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ArbiterInfo$inUse(void );







static uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ArbiterInfo$userId(void );
# 64 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$grantedTask$runTask(void );
# 52 "/opt/tinyos-2.0.2/tos/lib/power/PowerDownCleanup.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$PowerDownCleanup$default$cleanup(void );
# 46 "/opt/tinyos-2.0.2/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$ResourceDefaultOwner$granted(void );
#line 81
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$ResourceDefaultOwner$immediateRequested(void );
# 39 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartConfigure.nc"
static msp430_uart_union_config_t *TelosSerialP$Msp430UartConfigure$getConfig(void );
# 92 "/opt/tinyos-2.0.2/tos/interfaces/Resource.nc"
static void TelosSerialP$Resource$granted(void );
# 74 "/opt/tinyos-2.0.2/tos/interfaces/StdControl.nc"
static error_t TelosSerialP$StdControl$start(void );









static error_t TelosSerialP$StdControl$stop(void );
# 31 "/opt/tinyos-2.0.2/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t SerialPacketInfoActiveMessageP$Info$upperLength(message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t SerialPacketInfoActiveMessageP$Info$offset(void );







static uint8_t SerialPacketInfoActiveMessageP$Info$dataLinkLength(message_t *msg, uint8_t upperLen);
# 51 "/opt/tinyos-2.0.2/tos/interfaces/Init.nc"
static error_t PlatformP$MoteInit$init(void );
#line 51
static error_t PlatformP$MoteClockInit$init(void );
#line 51
static error_t PlatformP$LedsInit$init(void );
# 10 "/opt/tinyos-2.0.2/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP$Init$init(void );
# 6 "/opt/tinyos-2.0.2/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC$uwait(uint16_t u);




static __inline void MotePlatformC$TOSH_wait(void );




static void MotePlatformC$TOSH_FLASH_M25P_DP_bit(bool set);










static inline void MotePlatformC$TOSH_FLASH_M25P_DP(void );
#line 56
static inline error_t MotePlatformC$Init$init(void );
# 32 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430ClockInit.nc"
static void Msp430ClockP$Msp430ClockInit$initTimerB(void );
#line 31
static void Msp430ClockP$Msp430ClockInit$initTimerA(void );
#line 29
static void Msp430ClockP$Msp430ClockInit$setupDcoCalibrate(void );
static void Msp430ClockP$Msp430ClockInit$initClocks(void );
# 39 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430ClockP.nc"
static volatile uint8_t Msp430ClockP$IE1 __asm ("0x0000");
static volatile uint16_t Msp430ClockP$TA0CTL __asm ("0x0160");
static volatile uint16_t Msp430ClockP$TA0IV __asm ("0x012E");
static volatile uint16_t Msp430ClockP$TBCTL __asm ("0x0180");
static volatile uint16_t Msp430ClockP$TBIV __asm ("0x011E");

enum Msp430ClockP$__nesc_unnamed4293 {

  Msp430ClockP$ACLK_CALIB_PERIOD = 8, 
  Msp430ClockP$TARGET_DCO_DELTA = 4096 / 32 * Msp430ClockP$ACLK_CALIB_PERIOD
};


static inline void Msp430ClockP$Msp430ClockInit$defaultSetupDcoCalibrate(void );
#line 64
static inline void Msp430ClockP$Msp430ClockInit$defaultInitClocks(void );
#line 85
static inline void Msp430ClockP$Msp430ClockInit$defaultInitTimerA(void );
#line 100
static inline void Msp430ClockP$Msp430ClockInit$defaultInitTimerB(void );
#line 115
static inline void Msp430ClockP$Msp430ClockInit$default$setupDcoCalibrate(void );




static inline void Msp430ClockP$Msp430ClockInit$default$initClocks(void );




static inline void Msp430ClockP$Msp430ClockInit$default$initTimerA(void );




static inline void Msp430ClockP$Msp430ClockInit$default$initTimerB(void );





static inline void Msp430ClockP$startTimerA(void );
#line 148
static inline void Msp430ClockP$startTimerB(void );
#line 160
static void Msp430ClockP$set_dco_calib(int calib);





static inline uint16_t Msp430ClockP$test_calib_busywait_delta(int calib);
#line 189
static inline void Msp430ClockP$busyCalibrateDco(void );
#line 214
static inline error_t Msp430ClockP$Init$init(void );
# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$fired(
# 40 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x405d6c08);
# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$overflow(void );
# 115 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX0$fired(void );




static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX1$fired(void );





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Overflow$fired(void );








static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$default$fired(uint8_t n);
# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$fired(
# 40 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerP.nc"
uint8_t arg_0x405d6c08);
# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$overflow(void );
# 51 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get(void );
#line 70
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$isOverflowPending(void );
#line 115
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX0$fired(void );




static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX1$fired(void );





static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Overflow$fired(void );








static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$default$fired(uint8_t n);
# 75 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$captured(uint16_t time);
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$fired(void );
# 44 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t;


static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$int2CC(uint16_t x);
#line 74
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Timer$overflow(void );
# 75 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$captured(uint16_t time);
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$fired(void );
# 44 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t;


static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$int2CC(uint16_t x);
#line 74
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Timer$overflow(void );
# 75 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$captured(uint16_t time);
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$fired(void );
# 44 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t;


static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$int2CC(uint16_t x);
#line 74
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$getEvent(void );
#line 169
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Timer$overflow(void );
# 75 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$captured(uint16_t time);
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$fired(void );
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$get(void );
# 44 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t;

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t x);
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$int2CC(uint16_t x);

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$compareControl(void );
#line 74
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$getControl(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$clearPendingInterrupt(void );









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$setControlAsCompare(void );
#line 119
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$enableEvents(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$disableEvents(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$getEvent(void );




static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEvent(uint16_t x);









static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEventFromNow(uint16_t x);
#line 169
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$default$captured(uint16_t n);







static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$overflow(void );
# 75 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$captured(uint16_t time);
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$fired(void );
# 44 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t;


static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$int2CC(uint16_t x);
#line 74
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Timer$overflow(void );
# 75 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$captured(uint16_t time);
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$fired(void );
# 44 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t;


static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$int2CC(uint16_t x);
#line 74
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$overflow(void );
# 75 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$captured(uint16_t time);
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$fired(void );
# 44 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t;


static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$int2CC(uint16_t x);
#line 74
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Timer$overflow(void );
# 75 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$captured(uint16_t time);
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$fired(void );
# 44 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t;


static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$int2CC(uint16_t x);
#line 74
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Timer$overflow(void );
# 75 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$captured(uint16_t time);
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$fired(void );
# 44 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t;


static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$int2CC(uint16_t x);
#line 74
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Timer$overflow(void );
# 75 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$captured(uint16_t time);
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$fired(void );
# 44 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
typedef msp430_compare_control_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t;


static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$int2CC(uint16_t x);
#line 74
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Control$getControl(void );
#line 139
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$getEvent(void );
#line 169
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Event$fired(void );







static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$default$captured(uint16_t n);



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$default$fired(void );



static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Timer$overflow(void );
# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void Msp430TimerCommonP$VectorTimerB1$fired(void );
#line 28
static void Msp430TimerCommonP$VectorTimerA0$fired(void );
#line 28
static void Msp430TimerCommonP$VectorTimerA1$fired(void );
#line 28
static void Msp430TimerCommonP$VectorTimerB0$fired(void );
# 11 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
void sig_TIMERA0_VECTOR(void )  __attribute((wakeup)) __attribute((interrupt(12))) ;
void sig_TIMERA1_VECTOR(void )  __attribute((wakeup)) __attribute((interrupt(10))) ;
void sig_TIMERB0_VECTOR(void )  __attribute((wakeup)) __attribute((interrupt(26))) ;
void sig_TIMERB1_VECTOR(void )  __attribute((wakeup)) __attribute((interrupt(24))) ;
# 51 "/opt/tinyos-2.0.2/tos/interfaces/Init.nc"
static error_t RealMainP$SoftwareInit$init(void );
# 49 "/opt/tinyos-2.0.2/tos/interfaces/Boot.nc"
static void RealMainP$Boot$booted(void );
# 51 "/opt/tinyos-2.0.2/tos/interfaces/Init.nc"
static error_t RealMainP$PlatformInit$init(void );
# 46 "/opt/tinyos-2.0.2/tos/interfaces/Scheduler.nc"
static void RealMainP$Scheduler$init(void );
#line 61
static void RealMainP$Scheduler$taskLoop(void );
#line 54
static bool RealMainP$Scheduler$runNextTask(void );
# 52 "/opt/tinyos-2.0.2/tos/system/RealMainP.nc"
int main(void )   ;
# 64 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP$TaskBasic$runTask(
# 45 "/opt/tinyos-2.0.2/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x4053c5c0);
# 59 "/opt/tinyos-2.0.2/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP$McuSleep$sleep(void );
# 50 "/opt/tinyos-2.0.2/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP$__nesc_unnamed4294 {

  SchedulerBasicP$NUM_TASKS = 9U, 
  SchedulerBasicP$NO_TASK = 255
};

volatile uint8_t SchedulerBasicP$m_head;
volatile uint8_t SchedulerBasicP$m_tail;
volatile uint8_t SchedulerBasicP$m_next[SchedulerBasicP$NUM_TASKS];








static __inline uint8_t SchedulerBasicP$popTask(void );
#line 86
static inline bool SchedulerBasicP$isWaiting(uint8_t id);




static inline bool SchedulerBasicP$pushTask(uint8_t id);
#line 113
static inline void SchedulerBasicP$Scheduler$init(void );









static bool SchedulerBasicP$Scheduler$runNextTask(void );
#line 138
static inline void SchedulerBasicP$Scheduler$taskLoop(void );
#line 159
static error_t SchedulerBasicP$TaskBasic$postTask(uint8_t id);




static void SchedulerBasicP$TaskBasic$default$runTask(uint8_t id);
# 54 "/opt/tinyos-2.0.2/tos/interfaces/McuPowerOverride.nc"
static mcu_power_t McuSleepC$McuPowerOverride$lowestState(void );
# 51 "/opt/tinyos-2.0.2/tos/chips/msp430/McuSleepC.nc"
bool McuSleepC$dirty = TRUE;
mcu_power_t McuSleepC$powerState = MSP430_POWER_ACTIVE;




const uint16_t McuSleepC$msp430PowerBits[MSP430_POWER_LPM4 + 1] = { 
0, 
0x0010, 
0x0040 + 0x0010, 
0x0080 + 0x0010, 
0x0080 + 0x0040 + 0x0010, 
0x0080 + 0x0040 + 0x0020 + 0x0010 };


static inline mcu_power_t McuSleepC$getPowerState(void );
#line 104
static inline void McuSleepC$computePowerState(void );




static inline void McuSleepC$McuSleep$sleep(void );
#line 124
static inline mcu_power_t McuSleepC$McuPowerOverride$default$lowestState(void );
# 69 "/opt/tinyos-2.0.2/tos/interfaces/AMSend.nc"
static error_t testECDSAM$PubKeyMsg$send(am_addr_t addr, message_t *msg, uint8_t len);
# 83 "/opt/tinyos-2.0.2/tos/interfaces/SplitControl.nc"
static error_t testECDSAM$SerialControl$start(void );
# 50 "/opt/tinyos-2.0.2/tos/lib/timer/LocalTime.nc"
static uint32_t testECDSAM$LocalTime$get(void );
# 40 "ECC.nc"
static void testECDSAM$ECC$gen_private_key(NN_DIGIT *PrivateKey);
#line 38
static void testECDSAM$ECC$init(void );



static void testECDSAM$ECC$gen_public_key(Point *PublicKey, NN_DIGIT *PrivateKey);
# 41 "/opt/tinyos-2.0.2/tos/interfaces/Random.nc"
static uint16_t testECDSAM$Random$rand16(void );
# 62 "/opt/tinyos-2.0.2/tos/lib/timer/Timer.nc"
static void testECDSAM$myTimer$startOneShot(uint32_t dt);
# 43 "NN.nc"
static void testECDSAM$NN$Encode(unsigned char *a, NN_UINT digits, NN_DIGIT *b, NN_UINT len);
# 56 "/opt/tinyos-2.0.2/tos/interfaces/Leds.nc"
static void testECDSAM$Leds$led0Toggle(void );
# 41 "ECDSA.nc"
static void testECDSAM$ECDSA$sign(uint8_t *msg, uint8_t len, NN_DIGIT *r, NN_DIGIT *s, NN_DIGIT *d);
#line 38
static error_t testECDSAM$ECDSA$init(Point *pKey);





static uint8_t testECDSAM$ECDSA$verify(uint8_t *msg, uint8_t len, NN_DIGIT *r, NN_DIGIT *s, Point *Q);
# 69 "/opt/tinyos-2.0.2/tos/interfaces/AMSend.nc"
static error_t testECDSAM$PacketMsg$send(am_addr_t addr, message_t *msg, uint8_t len);
#line 69
static error_t testECDSAM$PriKeyMsg$send(am_addr_t addr, message_t *msg, uint8_t len);
#line 69
static error_t testECDSAM$TimeMsg$send(am_addr_t addr, message_t *msg, uint8_t len);
# 28 "testECDSAM.nc"
message_t testECDSAM$report;
Point testECDSAM$PublicKey;
NN_DIGIT testECDSAM$PrivateKey[160 / 16 + 1];
uint8_t testECDSAM$message[52];
NN_DIGIT testECDSAM$r[160 / 16 + 1];
NN_DIGIT testECDSAM$s[160 / 16 + 1];
uint8_t testECDSAM$type;
uint32_t testECDSAM$t;
uint8_t testECDSAM$pass;
uint16_t testECDSAM$round_index;

static void testECDSAM$init_data(void );
static inline void testECDSAM$gen_PrivateKey(void );
static inline void testECDSAM$ecc_init(void );
static inline void testECDSAM$gen_PublicKey(void );
static inline void testECDSAM$ecdsa_init(void );
static inline void testECDSAM$sign(void );
static inline void testECDSAM$verify(void );


static void testECDSAM$init_data(void );
#line 85
static inline void testECDSAM$gen_PrivateKey(void );
#line 150
static inline void testECDSAM$ecc_init(void );
#line 171
static inline void testECDSAM$gen_PublicKey(void );
#line 194
static inline void testECDSAM$ecdsa_init(void );
#line 216
static inline void testECDSAM$sign(void );
#line 239
static inline void testECDSAM$verify(void );
#line 262
static inline void testECDSAM$Boot$booted(void );



static inline void testECDSAM$SerialControl$startDone(error_t e);



static inline void testECDSAM$SerialControl$stopDone(error_t e);


static inline void testECDSAM$myTimer$fired(void );




static inline void testECDSAM$PubKeyMsg$sendDone(message_t *sent, error_t error);










static inline void testECDSAM$PriKeyMsg$sendDone(message_t *sent, error_t success);



static inline void testECDSAM$PacketMsg$sendDone(message_t *sent, error_t success);










static inline void testECDSAM$TimeMsg$sendDone(message_t *sent, error_t success);
# 31 "/opt/tinyos-2.0.2/tos/interfaces/GeneralIO.nc"
static void LedsP$Led0$toggle(void );



static void LedsP$Led0$makeOutput(void );
#line 29
static void LedsP$Led0$set(void );





static void LedsP$Led1$makeOutput(void );
#line 29
static void LedsP$Led1$set(void );





static void LedsP$Led2$makeOutput(void );
#line 29
static void LedsP$Led2$set(void );
# 45 "/opt/tinyos-2.0.2/tos/system/LedsP.nc"
static inline error_t LedsP$Init$init(void );
#line 73
static inline void LedsP$Leds$led0Toggle(void );
# 54 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP$22$IO$selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP$22$IO$selectIOFunc(void );
#line 54
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP$23$IO$selectModuleFunc(void );

static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP$23$IO$selectIOFunc(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP$33$IO$selectIOFunc(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP$34$IO$selectIOFunc(void );
#line 56
static inline void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP$35$IO$selectIOFunc(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$set(void );

static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$toggle(void );




static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$set(void );






static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$makeOutput(void );
#line 45
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$set(void );






static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$makeOutput(void );
# 44 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$toggle(void );
#line 71
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$makeOutput(void );
#line 34
static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$set(void );
# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$set(void );

static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$toggle(void );



static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$makeOutput(void );
# 71 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$makeOutput(void );
#line 34
static void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$set(void );
# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$set(void );





static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$makeOutput(void );
# 71 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$makeOutput(void );
#line 34
static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$set(void );
# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$set(void );





static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$makeOutput(void );
# 57 "/opt/tinyos-2.0.2/tos/system/RandomLfsrP.nc"
uint16_t RandomLfsrP$shiftReg;
uint16_t RandomLfsrP$initSeed;
uint16_t RandomLfsrP$mask;


static inline error_t RandomLfsrP$Init$init(void );









static uint16_t RandomLfsrP$Random$rand16(void );
# 39 "CurveParam.nc"
static NN_UINT NNM$CurveParam$omega_mul(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *omega, NN_UINT digits);
# 86 "NNM.nc"
static inline void NNM$NN_DigitDiv(NN_DIGIT *a, NN_DIGIT b[2], NN_DIGIT c);
static void NNM$NN_Decode(NN_DIGIT *a, NN_UINT digits, unsigned char *b, NN_UINT len);
static void NNM$NN_Encode(unsigned char *a, NN_UINT len, NN_DIGIT *b, NN_UINT digits);
static inline void NNM$NN_Assign(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits);
static void NNM$NN_AssignZero(NN_DIGIT *a, NN_UINT digits);

static NN_DIGIT NNM$NN_Add(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits);
static NN_DIGIT NNM$NN_Sub(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits);
static void NNM$NN_Mult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits);
static NN_DIGIT NNM$NN_LShift(NN_DIGIT *a, NN_DIGIT *b, NN_UINT c, NN_UINT digits);
static NN_DIGIT NNM$NN_RShift(NN_DIGIT *a, NN_DIGIT *b, NN_UINT c, NN_UINT digits);
static inline void NNM$NN_Div(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT cDigits, NN_DIGIT *d, NN_UINT dDigits);
static inline void NNM$NN_Mod(NN_DIGIT *a, NN_DIGIT *b, NN_UINT bDigits, NN_DIGIT *c, NN_UINT cDigits);
static void NNM$NN_ModMult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits);



static int NNM$NN_Cmp(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits);
static int NNM$NN_Zero(NN_DIGIT *a, NN_UINT digits);
static unsigned int NNM$NN_Bits(NN_DIGIT *a, NN_UINT digits);
static unsigned int NNM$NN_Digits(NN_DIGIT *a, NN_UINT digits);
inline static NN_DIGIT NNM$NN_AddDigitMult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT c, NN_DIGIT *d, NN_UINT digits);
inline static NN_DIGIT NNM$NN_SubDigitMult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT c, NN_DIGIT *d, NN_UINT digits);
static unsigned int NNM$NN_DigitBits(NN_DIGIT a);

Barrett *NNM$pBarrett;
#line 164
static inline void NNM$NN_DigitDiv(NN_DIGIT *a, NN_DIGIT b[2], NN_DIGIT c);
#line 180
static void NNM$NN_Decode(NN_DIGIT *a, NN_UINT digits, unsigned char *b, NN_UINT len);
#line 204
static void NNM$NN_Encode(unsigned char *a, NN_UINT len, NN_DIGIT *b, NN_UINT digits);
#line 224
static inline void NNM$NN_Assign(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits);








static void NNM$NN_AssignZero(NN_DIGIT *a, NN_UINT digits);
#line 263
static NN_DIGIT NNM$NN_Add(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits) __attribute((noinline)) ;
#line 357
static NN_DIGIT NNM$NN_Sub(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits) __attribute((noinline)) ;
#line 454
static void NNM$NN_Mult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits) __attribute((noinline)) ;
#line 1758
static void NNM$NN_Sqr(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits) __attribute((noinline)) ;
#line 2740
static NN_DIGIT NNM$NN_LShift(NN_DIGIT *a, NN_DIGIT *b, NN_UINT c, NN_UINT digits);
#line 2766
static NN_DIGIT NNM$NN_RShift(NN_DIGIT *a, NN_DIGIT *b, NN_UINT c, NN_UINT digits);
#line 2795
static inline void NNM$NN_Div(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT cDigits, NN_DIGIT *d, NN_UINT dDigits);
#line 2846
static inline void NNM$NN_Mod(NN_DIGIT *a, NN_DIGIT *b, NN_UINT bDigits, NN_DIGIT *c, NN_UINT cDigits);
#line 2860
static void NNM$NN_ModMult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits);
#line 3022
static void NNM$NN_ModDivOpt(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits);
#line 3108
static int NNM$NN_Cmp(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits);
#line 3127
static int NNM$NN_Zero(NN_DIGIT *a, NN_UINT digits);
#line 3142
static unsigned int NNM$NN_Bits(NN_DIGIT *a, NN_UINT digits);
#line 3154
static unsigned int NNM$NN_Digits(NN_DIGIT *a, NN_UINT digits);
#line 3169
inline static NN_DIGIT NNM$NN_AddDigitMult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT c, NN_DIGIT *d, NN_UINT digits);
#line 3267
inline static NN_DIGIT NNM$NN_SubDigitMult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT c, NN_DIGIT *d, NN_UINT digits);
#line 3364
static unsigned int NNM$NN_DigitBits(NN_DIGIT a);
#line 3376
static inline void NNM$NN$Decode(NN_DIGIT *a, NN_UINT digits, unsigned char *b, NN_UINT len);








static inline void NNM$NN$Encode(unsigned char *a, NN_UINT digits, NN_DIGIT *b, NN_UINT len);






static inline void NNM$NN$Assign(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits);




static inline void NNM$NN$AssignDigit(NN_DIGIT *a, NN_DIGIT b, NN_UINT digits);





static inline void NNM$NN$AssignZero(NN_DIGIT *a, NN_UINT digits);
#line 3415
static inline NN_DIGIT NNM$NN$Add(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits);
#line 3439
static inline NN_DIGIT NNM$NN$LShift(NN_DIGIT *a, NN_DIGIT *b, NN_UINT c, NN_UINT digits);





static inline NN_DIGIT NNM$NN$RShift(NN_DIGIT *a, NN_DIGIT *b, NN_UINT c, NN_UINT digits);




static inline NN_DIGIT NNM$NN$AddDigitMult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT c, NN_DIGIT *d, NN_UINT digits);






static inline void NNM$NN$Mod(NN_DIGIT *a, NN_DIGIT *b, NN_UINT bDigits, NN_DIGIT *c, NN_UINT cDigits);







static void NNM$NN$ModBarrettInit(NN_DIGIT *c, NN_UINT cDigits, Barrett *pbuf);
#line 3495
static inline void NNM$NN$BarrettSetBuf(Barrett *pbuf);








static void NNM$NN$ModBarrett(NN_DIGIT *a, NN_DIGIT *b, NN_UINT bDigits, NN_DIGIT *c, NN_UINT cDigits);
#line 3535
static inline void NNM$NN$ModSmall(NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits);








static void NNM$NN$ModAdd(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits);
#line 3562
static void NNM$NN$ModSub(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits);
#line 3581
static inline void NNM$NN$ModMult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits);
#line 3607
static void NNM$NN$ModMultOpt(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_DIGIT *omega, NN_UINT digits);
#line 3683
static void NNM$NN$ModSqrOpt(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *d, NN_DIGIT *omega, NN_UINT digits);
#line 3731
static inline void NNM$NN$ModInv(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits);
#line 3879
static inline int NNM$NN$Cmp(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits);





static inline int NNM$NN$Zero(NN_DIGIT *a, NN_UINT digits);





static inline int NNM$NN$One(NN_DIGIT *a, NN_UINT digits);
#line 3905
static inline unsigned int NNM$NN$Bits(NN_DIGIT *a, NN_UINT digits);
#line 3917
static inline unsigned int NNM$NN$Digits(NN_DIGIT *a, NN_UINT digits);
#line 3929
static inline int NNM$NN$Equal(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits);
# 37 "CurveParam.nc"
static void ECCM$CurveParam$get_param(Params *para);
# 41 "/opt/tinyos-2.0.2/tos/interfaces/Random.nc"
static uint16_t ECCM$Random$rand16(void );
# 65 "NN.nc"
static NN_DIGIT ECCM$NN$LShift(NN_DIGIT *a, NN_DIGIT *b, NN_UINT c, NN_UINT digits);
#line 81
static void ECCM$NN$ModSmall(NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits);
#line 123
static unsigned int ECCM$NN$Digits(NN_DIGIT *a, NN_UINT digits);
#line 85
static void ECCM$NN$ModSub(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits);
#line 75
static void ECCM$NN$ModBarrettInit(NN_DIGIT *c, NN_UINT cDigits, Barrett *pbuf);
#line 49
static void ECCM$NN$AssignDigit(NN_DIGIT *a, NN_DIGIT b, NN_UINT digits);
#line 77
static void ECCM$NN$BarrettSetBuf(Barrett *pbuf);
#line 93
static void ECCM$NN$ModMultOpt(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_DIGIT *omega, NN_UINT digits);
#line 47
static void ECCM$NN$Assign(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits);









static NN_DIGIT ECCM$NN$Add(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits);
#line 83
static void ECCM$NN$ModAdd(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits);
#line 119
static unsigned int ECCM$NN$Bits(NN_DIGIT *a, NN_UINT digits);





static int ECCM$NN$Equal(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits);
#line 99
static void ECCM$NN$ModSqrOpt(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *d, NN_DIGIT *omega, NN_UINT digits);
#line 51
static void ECCM$NN$AssignZero(NN_DIGIT *a, NN_UINT digits);
#line 67
static NN_DIGIT ECCM$NN$RShift(NN_DIGIT *a, NN_DIGIT *b, NN_UINT c, NN_UINT digits);
#line 103
static void ECCM$NN$ModInv(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits);
#line 115
static int ECCM$NN$Zero(NN_DIGIT *a, NN_UINT digits);
# 42 "ECCM.nc"
Params ECCM$param;


Barrett ECCM$Bbuf;





Point ECCM$pBaseArray[(1 << 4) - 1];

NN_DIGIT ECCM$mask[16 / 4];



static inline void ECCM$c_add_projective(Point *P0, NN_DIGIT *Z0, Point *P1, NN_DIGIT *Z1, Point *P2, NN_DIGIT *Z2);
static void ECCM$c_add_mix(Point *P0, NN_DIGIT *Z0, Point *P1, NN_DIGIT *Z1, Point *P2);
static void ECCM$c_dbl_projective(Point *P0, NN_DIGIT *Z0, Point *P1, NN_DIGIT *Z1);









static void ECCM$p_clear(Point *P0);






static void ECCM$p_copy(Point *P0, Point *P1);
#line 94
static inline bool ECCM$p_equal(Point *P1, Point *P2);








static bool ECCM$Z_is_one(NN_DIGIT *Z);
#line 117
static void ECCM$c_add(Point *P0, Point *P1, Point *P2);
#line 148
static void ECCM$c_add_mix(Point *P0, NN_DIGIT *Z0, Point *P1, NN_DIGIT *Z1, Point *P2);
#line 226
static inline void ECCM$c_add_projective(Point *P0, NN_DIGIT *Z0, Point *P1, NN_DIGIT *Z1, Point *P2, NN_DIGIT *Z2);
#line 358
static void ECCM$c_dbl_projective(Point *P0, NN_DIGIT *Z0, Point *P1, NN_DIGIT *Z1);
#line 455
static inline void ECCM$c_m_dbl_projective(Point *P0, NN_DIGIT *Z0, uint8_t m);
#line 586
static void ECCM$ECC$init(void );
#line 607
static inline void ECCM$ECC$BarrettSetBuf(void );
#line 699
static inline void ECCM$ECC$add(Point *P0, Point *P1, Point *P2);
#line 781
static inline void ECCM$ECC$win_precompute(Point *baseP, Point *pointArray);
#line 804
static void ECCM$win_mul(Point *P0, NN_DIGIT *n, Point *pointArray);
#line 898
static inline void ECCM$ECC$win_mul_base(Point *P0, NN_DIGIT *n);
#line 977
static inline Params *ECCM$ECC$get_param(void );




static inline void ECCM$ECC$add_proj(Point *P0, NN_DIGIT *Z0, Point *P1, NN_DIGIT *Z1, Point *P2, NN_DIGIT *Z2);



static inline void ECCM$ECC$dbl_proj(Point *P0, NN_DIGIT *Z0, Point *P1, NN_DIGIT *Z1);







static void ECCM$ECC$gen_private_key(NN_DIGIT *PrivateKey);
#line 1034
static inline void ECCM$ECC$gen_public_key(Point *PublicKey, NN_DIGIT *PrivateKey);
# 69 "NN.nc"
static NN_DIGIT secp160r1$NN$AddDigitMult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT c, NN_DIGIT *d, NN_UINT digits);
#line 47
static void secp160r1$NN$Assign(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits);
# 41 "secp160r1.nc"
static inline void secp160r1$CurveParam$get_param(Params *para);
#line 337
static NN_UINT secp160r1$CurveParam$omega_mul(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *omega, NN_UINT digits) __attribute((noinline)) ;
# 37 "SHA1.nc"
static int ECDSAM$SHA1$reset(SHA1Context *context);
static int ECDSAM$SHA1$update(SHA1Context *context, const uint8_t *message_array, uint32_t length);
static int ECDSAM$SHA1$digest(SHA1Context *context, uint8_t Message_Digest[20]);
# 100 "ECC.nc"
static Params *ECDSAM$ECC$get_param(void );
#line 40
static void ECDSAM$ECC$gen_private_key(NN_DIGIT *PrivateKey);
#line 38
static void ECDSAM$ECC$init(void );
#line 69
static void ECDSAM$ECC$add_proj(Point *P0, NN_DIGIT *Z0, Point *P1, NN_DIGIT *Z1, Point *P2, NN_DIGIT *Z2);
#line 54
static void ECDSAM$ECC$BarrettSetBuf(void );
#line 72
static void ECDSAM$ECC$dbl_proj(Point *P0, NN_DIGIT *Z0, Point *P1, NN_DIGIT *Z1);
#line 88
static void ECDSAM$ECC$win_mul_base(Point *P0, NN_DIGIT *n);
#line 58
static void ECDSAM$ECC$add(Point *P0, Point *P1, Point *P2);
# 81 "NN.nc"
static void ECDSAM$NN$ModSmall(NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits);
#line 123
static unsigned int ECDSAM$NN$Digits(NN_DIGIT *a, NN_UINT digits);
#line 113
static int ECDSAM$NN$Cmp(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits);
#line 75
static void ECDSAM$NN$ModBarrettInit(NN_DIGIT *c, NN_UINT cDigits, Barrett *pbuf);

static void ECDSAM$NN$BarrettSetBuf(Barrett *pbuf);
#line 89
static void ECDSAM$NN$ModMult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits);



static void ECDSAM$NN$ModMultOpt(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_DIGIT *omega, NN_UINT digits);
#line 47
static void ECDSAM$NN$Assign(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits);
#line 41
static void ECDSAM$NN$Decode(NN_DIGIT *a, NN_UINT digits, unsigned char *b, NN_UINT len);
#line 73
static void ECDSAM$NN$Mod(NN_DIGIT *a, NN_DIGIT *b, NN_UINT bDigits, NN_DIGIT *c, NN_UINT cDigits);









static void ECDSAM$NN$ModAdd(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits);
#line 117
static int ECDSAM$NN$One(NN_DIGIT *a, NN_UINT digits);

static unsigned int ECDSAM$NN$Bits(NN_DIGIT *a, NN_UINT digits);
#line 51
static void ECDSAM$NN$AssignZero(NN_DIGIT *a, NN_UINT digits);
#line 103
static void ECDSAM$NN$ModInv(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits);
#line 115
static int ECDSAM$NN$Zero(NN_DIGIT *a, NN_UINT digits);
# 65 "ECDSAM.nc"
Params *ECDSAM$param;


Barrett ECDSAM$Bbuf;



Point ECDSAM$pqBaseArray[(1 << 2 * 2) - 1];
NN_DIGIT ECDSAM$s_mask[16 / 2];
#line 150
static inline void ECDSAM$shamir_init(Point *pKey, Point *pointArray);
#line 197
static inline void ECDSAM$shamir(Point *P0, NN_DIGIT *u1, NN_DIGIT *u2);
#line 289
static error_t ECDSAM$ECDSA$init(Point *pKey) __attribute((noinline)) ;
#line 309
static void ECDSAM$ECDSA$sign(uint8_t *msg, uint8_t len, NN_DIGIT *r, NN_DIGIT *s, NN_DIGIT *d) __attribute((noinline)) ;
#line 386
static uint8_t ECDSAM$ECDSA$verify(uint8_t *msg, uint8_t len, NN_DIGIT *r, NN_DIGIT *s, Point *Q) __attribute((noinline)) ;
# 56 "SHA1M.nc"
static inline void SHA1M$SHA1PadMessage(SHA1Context *arg_0x40ba5158);
static void SHA1M$SHA1ProcessMessageBlock(SHA1Context *arg_0x40ba56e8);
#line 74
static int SHA1M$SHA1$reset(SHA1Context *context);
#line 115
static int SHA1M$SHA1$digest(SHA1Context *context, uint8_t Message_Digest[20]);
#line 167
static int SHA1M$SHA1$update(SHA1Context *context, const uint8_t *message_array, uint32_t length);
#line 230
static void SHA1M$SHA1ProcessMessageBlock(SHA1Context *context);
#line 386
static inline void SHA1M$SHA1PadMessage(SHA1Context *context);
# 30 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEvent(uint16_t time);

static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEventFromNow(uint16_t delta);
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$get(void );
# 67 "/opt/tinyos-2.0.2/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$fired(void );
# 39 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$enableEvents(void );
#line 36
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$setControlAsCompare(void );



static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents(void );
#line 33
static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$clearPendingInterrupt(void );
# 42 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Init$init(void );
#line 54
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$stop(void );




static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$fired(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$startAt(uint16_t t0, uint16_t dt);
#line 103
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$overflow(void );
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$get(void );
static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$isOverflowPending(void );
# 71 "/opt/tinyos-2.0.2/tos/lib/timer/Counter.nc"
static void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$overflow(void );
# 38 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$get(void );




static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$isOverflowPending(void );









static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$overflow(void );
# 53 "/opt/tinyos-2.0.2/tos/lib/timer/Counter.nc"
static /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$size_type /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$get(void );






static bool /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$isOverflowPending(void );










static void /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$overflow(void );
# 56 "/opt/tinyos-2.0.2/tos/lib/timer/TransformCounterC.nc"
/*CounterMilli32C.Transform*/TransformCounterC$0$upper_count_type /*CounterMilli32C.Transform*/TransformCounterC$0$m_upper;

enum /*CounterMilli32C.Transform*/TransformCounterC$0$__nesc_unnamed4295 {

  TransformCounterC$0$LOW_SHIFT_RIGHT = 5, 
  TransformCounterC$0$HIGH_SHIFT_LEFT = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type ) - /*CounterMilli32C.Transform*/TransformCounterC$0$LOW_SHIFT_RIGHT, 
  TransformCounterC$0$NUM_UPPER_BITS = 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type ) - 8 * sizeof(/*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type ) + 5, 



  TransformCounterC$0$OVERFLOW_MASK = /*CounterMilli32C.Transform*/TransformCounterC$0$NUM_UPPER_BITS ? ((/*CounterMilli32C.Transform*/TransformCounterC$0$upper_count_type )2 << (/*CounterMilli32C.Transform*/TransformCounterC$0$NUM_UPPER_BITS - 1)) - 1 : 0
};

static /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$get(void );
#line 122
static inline void /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$overflow(void );
# 67 "/opt/tinyos-2.0.2/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$fired(void );
#line 92
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type dt);
#line 62
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$stop(void );
# 53 "/opt/tinyos-2.0.2/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$get(void );
# 66 "/opt/tinyos-2.0.2/tos/lib/timer/TransformAlarmC.nc"
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0;
/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt;

enum /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$__nesc_unnamed4296 {

  TransformAlarmC$0$MAX_DELAY_LOG2 = 8 * sizeof(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_size_type ) - 1 - 5, 
  TransformAlarmC$0$MAX_DELAY = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type )1 << /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$MAX_DELAY_LOG2
};

static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getNow(void );




static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getAlarm(void );










static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$stop(void );




static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$set_alarm(void );
#line 136
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type dt);
#line 151
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$fired(void );
#line 166
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$overflow(void );
# 56 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$postTask(void );
# 98 "/opt/tinyos-2.0.2/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type dt);
#line 105
static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$stop(void );
# 72 "/opt/tinyos-2.0.2/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$fired(void );
# 63 "/opt/tinyos-2.0.2/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$__nesc_unnamed4297 {
#line 63
  AlarmToTimerC$0$fired = 0U
};
#line 63
typedef int /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$__nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired];
#line 44
uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_dt;
bool /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(uint32_t t0, uint32_t dt, bool oneshot);
#line 60
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop(void );


static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask(void );






static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired(void );
#line 82
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt);


static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow(void );
# 56 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$postTask(void );
# 125 "/opt/tinyos-2.0.2/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(void );
#line 118
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$stop(void );




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$fired(
# 37 "/opt/tinyos-2.0.2/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x40c93e08);
#line 60
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_unnamed4298 {
#line 60
  VirtualizeTimerC$0$updateFromTimer = 1U
};
#line 60
typedef int /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer];
#line 42
enum /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_unnamed4299 {

  VirtualizeTimerC$0$NUM_TIMERS = 1U, 
  VirtualizeTimerC$0$END_OF_LIST = 255
};








#line 48
typedef struct /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$__nesc_unnamed4300 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t;

/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$NUM_TIMERS];




static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$fireTimers(uint32_t now);
#line 88
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$runTask(void );
#line 127
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired(void );




static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);
#line 147
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(uint8_t num, uint32_t dt);
#line 192
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$default$fired(uint8_t num);
# 53 "/opt/tinyos-2.0.2/tos/lib/timer/Counter.nc"
static /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$size_type /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$get(void );
# 42 "/opt/tinyos-2.0.2/tos/lib/timer/CounterToLocalTimeC.nc"
static inline uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$LocalTime$get(void );




static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow(void );
# 64 "/opt/tinyos-2.0.2/tos/interfaces/Send.nc"
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubSend$send(message_t *msg, uint8_t len);
# 99 "/opt/tinyos-2.0.2/tos/interfaces/AMSend.nc"
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMSend$sendDone(
# 36 "/opt/tinyos-2.0.2/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x40ccd888, 
# 99 "/opt/tinyos-2.0.2/tos/interfaces/AMSend.nc"
message_t *msg, error_t error);
# 67 "/opt/tinyos-2.0.2/tos/interfaces/Receive.nc"
static message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Receive$receive(
# 37 "/opt/tinyos-2.0.2/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x40cc8298, 
# 67 "/opt/tinyos-2.0.2/tos/interfaces/Receive.nc"
message_t *msg, void *payload, uint8_t len);
# 49 "/opt/tinyos-2.0.2/tos/lib/serial/SerialActiveMessageP.nc"
static inline serial_header_t */*SerialActiveMessageC.AM*/SerialActiveMessageP$0$getHeader(message_t *msg);



static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMSend$send(am_id_t id, am_addr_t dest, 
message_t *msg, 
uint8_t len);
#line 81
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubSend$sendDone(message_t *msg, error_t result);



static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMSend$default$sendDone(uint8_t id, message_t *msg, error_t result);



static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Receive$default$receive(uint8_t id, message_t *msg, void *payload, uint8_t len);
#line 102
static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubReceive$receive(message_t *msg, void *payload, uint8_t len);
#line 158
static am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$type(message_t *amsg);
# 92 "/opt/tinyos-2.0.2/tos/interfaces/SplitControl.nc"
static void SerialP$SplitControl$startDone(error_t error);
#line 117
static void SerialP$SplitControl$stopDone(error_t error);
# 56 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
static error_t SerialP$stopDoneTask$postTask(void );
# 74 "/opt/tinyos-2.0.2/tos/interfaces/StdControl.nc"
static error_t SerialP$SerialControl$start(void );









static error_t SerialP$SerialControl$stop(void );
# 56 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
static error_t SerialP$RunTx$postTask(void );
# 38 "/opt/tinyos-2.0.2/tos/lib/serial/SerialFlush.nc"
static void SerialP$SerialFlush$flush(void );
# 56 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
static error_t SerialP$startDoneTask$postTask(void );
# 45 "/opt/tinyos-2.0.2/tos/lib/serial/SerialFrameComm.nc"
static error_t SerialP$SerialFrameComm$putDelimiter(void );
#line 68
static void SerialP$SerialFrameComm$resetReceive(void );
#line 54
static error_t SerialP$SerialFrameComm$putData(uint8_t data);
# 56 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
static error_t SerialP$defaultSerialFlushTask$postTask(void );
# 70 "/opt/tinyos-2.0.2/tos/lib/serial/SendBytePacket.nc"
static uint8_t SerialP$SendBytePacket$nextByte(void );









static void SerialP$SendBytePacket$sendCompleted(error_t error);
# 51 "/opt/tinyos-2.0.2/tos/lib/serial/ReceiveBytePacket.nc"
static error_t SerialP$ReceiveBytePacket$startPacket(void );






static void SerialP$ReceiveBytePacket$byteReceived(uint8_t data);










static void SerialP$ReceiveBytePacket$endPacket(error_t result);
# 189 "/opt/tinyos-2.0.2/tos/lib/serial/SerialP.nc"
enum SerialP$__nesc_unnamed4301 {
#line 189
  SerialP$RunTx = 2U
};
#line 189
typedef int SerialP$__nesc_sillytask_RunTx[SerialP$RunTx];
#line 320
enum SerialP$__nesc_unnamed4302 {
#line 320
  SerialP$startDoneTask = 3U
};
#line 320
typedef int SerialP$__nesc_sillytask_startDoneTask[SerialP$startDoneTask];





enum SerialP$__nesc_unnamed4303 {
#line 326
  SerialP$stopDoneTask = 4U
};
#line 326
typedef int SerialP$__nesc_sillytask_stopDoneTask[SerialP$stopDoneTask];








enum SerialP$__nesc_unnamed4304 {
#line 335
  SerialP$defaultSerialFlushTask = 5U
};
#line 335
typedef int SerialP$__nesc_sillytask_defaultSerialFlushTask[SerialP$defaultSerialFlushTask];
#line 79
enum SerialP$__nesc_unnamed4305 {
  SerialP$RX_DATA_BUFFER_SIZE = 2, 
  SerialP$TX_DATA_BUFFER_SIZE = 4, 
  SerialP$SERIAL_MTU = 255, 
  SerialP$SERIAL_VERSION = 1, 
  SerialP$ACK_QUEUE_SIZE = 5
};

enum SerialP$__nesc_unnamed4306 {
  SerialP$RXSTATE_NOSYNC, 
  SerialP$RXSTATE_PROTO, 
  SerialP$RXSTATE_TOKEN, 
  SerialP$RXSTATE_INFO, 
  SerialP$RXSTATE_INACTIVE
};

enum SerialP$__nesc_unnamed4307 {
  SerialP$TXSTATE_IDLE, 
  SerialP$TXSTATE_PROTO, 
  SerialP$TXSTATE_SEQNO, 
  SerialP$TXSTATE_INFO, 
  SerialP$TXSTATE_FCS1, 
  SerialP$TXSTATE_FCS2, 
  SerialP$TXSTATE_ENDFLAG, 
  SerialP$TXSTATE_ENDWAIT, 
  SerialP$TXSTATE_FINISH, 
  SerialP$TXSTATE_ERROR, 
  SerialP$TXSTATE_INACTIVE
};





#line 109
typedef enum SerialP$__nesc_unnamed4308 {
  SerialP$BUFFER_AVAILABLE, 
  SerialP$BUFFER_FILLING, 
  SerialP$BUFFER_COMPLETE
} SerialP$tx_data_buffer_states_t;

enum SerialP$__nesc_unnamed4309 {
  SerialP$TX_ACK_INDEX = 0, 
  SerialP$TX_DATA_INDEX = 1, 
  SerialP$TX_BUFFER_COUNT = 2
};






#line 122
typedef struct SerialP$__nesc_unnamed4310 {
  uint8_t writePtr;
  uint8_t readPtr;
  uint8_t buf[SerialP$RX_DATA_BUFFER_SIZE + 1];
} SerialP$rx_buf_t;




#line 128
typedef struct SerialP$__nesc_unnamed4311 {
  uint8_t state;
  uint8_t buf;
} SerialP$tx_buf_t;





#line 133
typedef struct SerialP$__nesc_unnamed4312 {
  uint8_t writePtr;
  uint8_t readPtr;
  uint8_t buf[SerialP$ACK_QUEUE_SIZE + 1];
} SerialP$ack_queue_t;



SerialP$rx_buf_t SerialP$rxBuf;
SerialP$tx_buf_t SerialP$txBuf[SerialP$TX_BUFFER_COUNT];



uint8_t SerialP$rxState;
uint8_t SerialP$rxByteCnt;
uint8_t SerialP$rxProto;
uint8_t SerialP$rxSeqno;
uint16_t SerialP$rxCRC;



uint8_t SerialP$txState;
uint8_t SerialP$txByteCnt;
uint8_t SerialP$txProto;
uint8_t SerialP$txSeqno;
uint16_t SerialP$txCRC;
uint8_t SerialP$txPending;
uint8_t SerialP$txIndex;


SerialP$ack_queue_t SerialP$ackQ;

bool SerialP$offPending = FALSE;



static __inline void SerialP$txInit(void );
static __inline void SerialP$rxInit(void );
static __inline void SerialP$ackInit(void );

static __inline bool SerialP$ack_queue_is_full(void );
static __inline bool SerialP$ack_queue_is_empty(void );
static __inline void SerialP$ack_queue_push(uint8_t token);
static __inline uint8_t SerialP$ack_queue_top(void );
static inline uint8_t SerialP$ack_queue_pop(void );




static __inline void SerialP$rx_buffer_push(uint8_t data);
static __inline uint8_t SerialP$rx_buffer_top(void );
static __inline uint8_t SerialP$rx_buffer_pop(void );
static __inline uint16_t SerialP$rx_current_crc(void );

static void SerialP$rx_state_machine(bool isDelimeter, uint8_t data);
static void SerialP$MaybeScheduleTx(void );




static __inline void SerialP$txInit(void );
#line 205
static __inline void SerialP$rxInit(void );








static __inline void SerialP$ackInit(void );



static inline error_t SerialP$Init$init(void );
#line 232
static __inline bool SerialP$ack_queue_is_full(void );









static __inline bool SerialP$ack_queue_is_empty(void );





static __inline void SerialP$ack_queue_push(uint8_t token);









static __inline uint8_t SerialP$ack_queue_top(void );









static inline uint8_t SerialP$ack_queue_pop(void );
#line 295
static __inline void SerialP$rx_buffer_push(uint8_t data);



static __inline uint8_t SerialP$rx_buffer_top(void );



static __inline uint8_t SerialP$rx_buffer_pop(void );





static __inline uint16_t SerialP$rx_current_crc(void );










static inline void SerialP$startDoneTask$runTask(void );





static inline void SerialP$stopDoneTask$runTask(void );



static inline void SerialP$SerialFlush$flushDone(void );




static inline void SerialP$defaultSerialFlushTask$runTask(void );


static inline void SerialP$SerialFlush$default$flush(void );



static inline error_t SerialP$SplitControl$start(void );




static void SerialP$testOff(void );
#line 384
static inline void SerialP$SerialFrameComm$delimiterReceived(void );


static inline void SerialP$SerialFrameComm$dataReceived(uint8_t data);



static inline bool SerialP$valid_rx_proto(uint8_t proto);










static void SerialP$rx_state_machine(bool isDelimeter, uint8_t data);
#line 502
static void SerialP$MaybeScheduleTx(void );










static inline error_t SerialP$SendBytePacket$completeSend(void );








static inline error_t SerialP$SendBytePacket$startSend(uint8_t b);
#line 539
static inline void SerialP$RunTx$runTask(void );
#line 642
static inline void SerialP$SerialFrameComm$putDone(void );
# 56 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTask$postTask(void );
# 89 "/opt/tinyos-2.0.2/tos/interfaces/Send.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$sendDone(
# 40 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40d8d160, 
# 89 "/opt/tinyos-2.0.2/tos/interfaces/Send.nc"
message_t *msg, error_t error);
# 56 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$signalSendDone$postTask(void );
# 67 "/opt/tinyos-2.0.2/tos/interfaces/Receive.nc"
static message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Receive$receive(
# 39 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40d90918, 
# 67 "/opt/tinyos-2.0.2/tos/interfaces/Receive.nc"
message_t *msg, void *payload, uint8_t len);
# 31 "/opt/tinyos-2.0.2/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$upperLength(
# 43 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40d8dc60, 
# 31 "/opt/tinyos-2.0.2/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$offset(
# 43 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40d8dc60);
# 23 "/opt/tinyos-2.0.2/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$dataLinkLength(
# 43 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x40d8dc60, 
# 23 "/opt/tinyos-2.0.2/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t upperLen);
# 60 "/opt/tinyos-2.0.2/tos/lib/serial/SendBytePacket.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$completeSend(void );
#line 51
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$startSend(uint8_t first_byte);
# 152 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$__nesc_unnamed4313 {
#line 152
  SerialDispatcherP$0$signalSendDone = 6U
};
#line 152
typedef int /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$__nesc_sillytask_signalSendDone[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$signalSendDone];
#line 269
enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$__nesc_unnamed4314 {
#line 269
  SerialDispatcherP$0$receiveTask = 7U
};
#line 269
typedef int /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$__nesc_sillytask_receiveTask[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTask];
#line 55
#line 51
typedef enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$__nesc_unnamed4315 {
  SerialDispatcherP$0$SEND_STATE_IDLE = 0, 
  SerialDispatcherP$0$SEND_STATE_BEGIN = 1, 
  SerialDispatcherP$0$SEND_STATE_DATA = 2
} /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$send_state_t;

enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$__nesc_unnamed4316 {
  SerialDispatcherP$0$RECV_STATE_IDLE = 0, 
  SerialDispatcherP$0$RECV_STATE_BEGIN = 1, 
  SerialDispatcherP$0$RECV_STATE_DATA = 2
};






#line 63
typedef struct /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$__nesc_unnamed4317 {
  uint8_t which : 1;
  uint8_t bufZeroLocked : 1;
  uint8_t bufOneLocked : 1;
  uint8_t state : 2;
} /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recv_state_t;



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recv_state_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState = { 0, 0, 0, /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$RECV_STATE_IDLE };
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvType = TOS_SERIAL_UNKNOWN_ID;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvIndex = 0;


message_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$messages[2];
message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$messagePtrs[2] = { &/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$messages[0], &/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$messages[1] };




uint8_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveBuffer = (uint8_t *)&/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$messages[0];

uint8_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendBuffer = (void *)0;
/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$send_state_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SEND_STATE_IDLE;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendLen = 0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendIndex = 0;
error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendError = SUCCESS;
bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendCancelled = FALSE;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendId = 0;


uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskPending = FALSE;
uart_id_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskType = 0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskWhich;
message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskBuf = (void *)0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskSize = 0;

static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$send(uint8_t id, message_t *msg, uint8_t len);
#line 152
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$signalSendDone$runTask(void );
#line 172
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$nextByte(void );
#line 188
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$sendCompleted(error_t error);




static inline bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$isCurrentBufferLocked(void );



static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$lockCurrentBuffer(void );








static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$unlockBuffer(uint8_t which);








static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveBufferSwap(void );




static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$startPacket(void );
#line 238
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$byteReceived(uint8_t b);
#line 269
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTask$runTask(void );
#line 290
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$endPacket(error_t result);
#line 349
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$offset(uart_id_t id);


static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$dataLinkLength(uart_id_t id, message_t *msg, 
uint8_t upperLen);


static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$upperLength(uart_id_t id, message_t *msg, 
uint8_t dataLinkLen);




static inline message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Receive$default$receive(uart_id_t idxxx, message_t *msg, 
void *payload, 
uint8_t len);


static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$default$sendDone(uart_id_t idxxx, message_t *msg, error_t error);
# 48 "/opt/tinyos-2.0.2/tos/interfaces/UartStream.nc"
static error_t HdlcTranslateC$UartStream$send(uint8_t *buf, uint16_t len);
# 83 "/opt/tinyos-2.0.2/tos/lib/serial/SerialFrameComm.nc"
static void HdlcTranslateC$SerialFrameComm$dataReceived(uint8_t data);





static void HdlcTranslateC$SerialFrameComm$putDone(void );
#line 74
static void HdlcTranslateC$SerialFrameComm$delimiterReceived(void );
# 47 "/opt/tinyos-2.0.2/tos/lib/serial/HdlcTranslateC.nc"
#line 44
typedef struct HdlcTranslateC$__nesc_unnamed4318 {
  uint8_t sendEscape : 1;
  uint8_t receiveEscape : 1;
} HdlcTranslateC$HdlcState;


HdlcTranslateC$HdlcState HdlcTranslateC$state = { 0, 0 };
uint8_t HdlcTranslateC$txTemp;
uint8_t HdlcTranslateC$m_data;


static inline void HdlcTranslateC$SerialFrameComm$resetReceive(void );





static inline void HdlcTranslateC$UartStream$receivedByte(uint8_t data);
#line 86
static error_t HdlcTranslateC$SerialFrameComm$putDelimiter(void );





static error_t HdlcTranslateC$SerialFrameComm$putData(uint8_t data);
#line 104
static inline void HdlcTranslateC$UartStream$sendDone(uint8_t *buf, uint16_t len, 
error_t error);










static inline void HdlcTranslateC$UartStream$receiveDone(uint8_t *buf, uint16_t len, error_t error);
# 39 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartConfigure.nc"
static msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP$0$Msp430UartConfigure$getConfig(
# 49 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40e23010);
# 97 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP$0$Usart$resetUsart(bool reset);
#line 179
static void /*Msp430Uart1P.UartP*/Msp430UartP$0$Usart$disableIntr(void );


static void /*Msp430Uart1P.UartP*/Msp430UartP$0$Usart$enableIntr(void );
#line 224
static void /*Msp430Uart1P.UartP*/Msp430UartP$0$Usart$tx(uint8_t data);
#line 128
static void /*Msp430Uart1P.UartP*/Msp430UartP$0$Usart$disableUart(void );
#line 174
static void /*Msp430Uart1P.UartP*/Msp430UartP$0$Usart$setModeUart(msp430_uart_union_config_t *config);
# 79 "/opt/tinyos-2.0.2/tos/interfaces/UartStream.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP$0$UartStream$receivedByte(uint8_t byte);
#line 99
static void /*Msp430Uart1P.UartP*/Msp430UartP$0$UartStream$receiveDone(uint8_t *buf, uint16_t len, error_t error);
#line 57
static void /*Msp430Uart1P.UartP*/Msp430UartP$0$UartStream$sendDone(uint8_t *buf, uint16_t len, error_t error);
# 110 "/opt/tinyos-2.0.2/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartResource$release(
# 48 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40e24540);
# 87 "/opt/tinyos-2.0.2/tos/interfaces/Resource.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartResource$immediateRequest(
# 48 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40e24540);
# 92 "/opt/tinyos-2.0.2/tos/interfaces/Resource.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP$0$Resource$granted(
# 42 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
uint8_t arg_0x40e27450);
#line 59
uint8_t */*Msp430Uart1P.UartP*/Msp430UartP$0$m_tx_buf;
#line 59
uint8_t */*Msp430Uart1P.UartP*/Msp430UartP$0$m_rx_buf;
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP$0$m_tx_len;
#line 60
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP$0$m_rx_len;
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP$0$m_tx_pos;
#line 61
uint16_t /*Msp430Uart1P.UartP*/Msp430UartP$0$m_rx_pos;
uint8_t /*Msp430Uart1P.UartP*/Msp430UartP$0$m_byte_time;

static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP$0$Resource$immediateRequest(uint8_t id);
#line 76
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP$0$Resource$release(uint8_t id);





static void /*Msp430Uart1P.UartP*/Msp430UartP$0$ResourceConfigure$configure(uint8_t id);






static inline void /*Msp430Uart1P.UartP*/Msp430UartP$0$ResourceConfigure$unconfigure(uint8_t id);






static inline void /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartResource$granted(uint8_t id);
#line 123
static inline void /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartInterrupts$rxDone(uint8_t data);
#line 137
static error_t /*Msp430Uart1P.UartP*/Msp430UartP$0$UartStream$send(uint8_t *buf, uint16_t len);
#line 149
static inline void /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartInterrupts$txDone(void );
#line 186
static inline void /*Msp430Uart1P.UartP*/Msp430UartP$0$Counter$overflow(void );



static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartResource$default$immediateRequest(uint8_t id);
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartResource$default$release(uint8_t id);
static inline msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP$0$Msp430UartConfigure$default$getConfig(uint8_t id);



static inline void /*Msp430Uart1P.UartP*/Msp430UartP$0$Resource$default$granted(uint8_t id);
# 85 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart1P$UCLK$selectIOFunc(void );
# 54 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void HplMsp430Usart1P$Interrupts$rxDone(uint8_t data);
#line 49
static void HplMsp430Usart1P$Interrupts$txDone(void );
# 85 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
static void HplMsp430Usart1P$URXD$selectIOFunc(void );
#line 78
static void HplMsp430Usart1P$URXD$selectModuleFunc(void );






static void HplMsp430Usart1P$UTXD$selectIOFunc(void );
#line 78
static void HplMsp430Usart1P$UTXD$selectModuleFunc(void );






static void HplMsp430Usart1P$SOMI$selectIOFunc(void );
#line 85
static void HplMsp430Usart1P$SIMO$selectIOFunc(void );
# 87 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static volatile uint8_t HplMsp430Usart1P$IE2 __asm ("0x0001");
static volatile uint8_t HplMsp430Usart1P$ME2 __asm ("0x0005");
static volatile uint8_t HplMsp430Usart1P$IFG2 __asm ("0x0003");
static volatile uint8_t HplMsp430Usart1P$U1TCTL __asm ("0x0079");
static volatile uint8_t HplMsp430Usart1P$U1RCTL __asm ("0x007A");
static volatile uint8_t HplMsp430Usart1P$U1TXBUF __asm ("0x007F");



void sig_UART1RX_VECTOR(void )  __attribute((wakeup)) __attribute((interrupt(6))) ;




void sig_UART1TX_VECTOR(void )  __attribute((wakeup)) __attribute((interrupt(4))) ;



static inline error_t HplMsp430Usart1P$AsyncStdControl$start(void );



static inline error_t HplMsp430Usart1P$AsyncStdControl$stop(void );
#line 140
static inline void HplMsp430Usart1P$Usart$setUbr(uint16_t control);










static inline void HplMsp430Usart1P$Usart$setUmctl(uint8_t control);







static inline void HplMsp430Usart1P$Usart$resetUsart(bool reset);
#line 203
static inline void HplMsp430Usart1P$Usart$enableUart(void );







static void HplMsp430Usart1P$Usart$disableUart(void );








static inline void HplMsp430Usart1P$Usart$enableUartTx(void );




static inline void HplMsp430Usart1P$Usart$disableUartTx(void );





static inline void HplMsp430Usart1P$Usart$enableUartRx(void );




static inline void HplMsp430Usart1P$Usart$disableUartRx(void );
#line 251
static void HplMsp430Usart1P$Usart$disableSpi(void );
#line 283
static inline void HplMsp430Usart1P$configUart(msp430_uart_union_config_t *config);









static inline void HplMsp430Usart1P$Usart$setModeUart(msp430_uart_union_config_t *config);
#line 347
static inline void HplMsp430Usart1P$Usart$clrIntr(void );
#line 359
static inline void HplMsp430Usart1P$Usart$disableIntr(void );
#line 377
static inline void HplMsp430Usart1P$Usart$enableIntr(void );






static inline void HplMsp430Usart1P$Usart$tx(uint8_t data);
# 80 "/opt/tinyos-2.0.2/tos/interfaces/ArbiterInfo.nc"
static bool /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$ArbiterInfo$inUse(void );







static uint8_t /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$ArbiterInfo$userId(void );
# 54 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$Interrupts$rxDone(
# 39 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40eb5bd0, 
# 54 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
uint8_t data);
#line 49
static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$Interrupts$txDone(
# 39 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
uint8_t arg_0x40eb5bd0);









static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$RawInterrupts$txDone(void );




static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$RawInterrupts$rxDone(uint8_t data);









static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$Interrupts$default$txDone(uint8_t id);
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$Interrupts$default$rxDone(uint8_t id, uint8_t data);
# 39 "/opt/tinyos-2.0.2/tos/system/FcfsResourceQueueC.nc"
enum /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$__nesc_unnamed4319 {
#line 39
  FcfsResourceQueueC$0$NO_ENTRY = 0xFF
};
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$resQ[1U];
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$qHead = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$NO_ENTRY;
uint8_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$qTail = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$NO_ENTRY;

static inline error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$Init$init(void );




static inline bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$FcfsQueue$isEmpty(void );







static inline resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$FcfsQueue$dequeue(void );
# 51 "/opt/tinyos-2.0.2/tos/interfaces/ResourceRequested.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceRequested$immediateRequested(
# 55 "/opt/tinyos-2.0.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ef9a38);
# 55 "/opt/tinyos-2.0.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceConfigure$unconfigure(
# 60 "/opt/tinyos-2.0.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ef8df0);
# 49 "/opt/tinyos-2.0.2/tos/interfaces/ResourceConfigure.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceConfigure$configure(
# 60 "/opt/tinyos-2.0.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ef8df0);
# 43 "/opt/tinyos-2.0.2/tos/interfaces/ResourceQueue.nc"
static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$Queue$isEmpty(void );
#line 60
static resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$Queue$dequeue(void );
# 46 "/opt/tinyos-2.0.2/tos/interfaces/ResourceDefaultOwner.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceDefaultOwner$granted(void );
#line 81
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceDefaultOwner$immediateRequested(void );
# 92 "/opt/tinyos-2.0.2/tos/interfaces/Resource.nc"
static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$Resource$granted(
# 54 "/opt/tinyos-2.0.2/tos/system/ArbiterP.nc"
uint8_t arg_0x40ef9010);
# 56 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$grantedTask$postTask(void );
# 74 "/opt/tinyos-2.0.2/tos/system/ArbiterP.nc"
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$__nesc_unnamed4320 {
#line 74
  ArbiterP$0$grantedTask = 8U
};
#line 74
typedef int /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$__nesc_sillytask_grantedTask[/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$grantedTask];
#line 67
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$__nesc_unnamed4321 {
#line 67
  ArbiterP$0$RES_CONTROLLED, ArbiterP$0$RES_GRANTING, ArbiterP$0$RES_IMM_GRANTING, ArbiterP$0$RES_BUSY
};
#line 68
enum /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$__nesc_unnamed4322 {
#line 68
  ArbiterP$0$default_owner_id = 1U
};
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$RES_CONTROLLED;
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$default_owner_id;
uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$reqResId;
#line 89
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$Resource$immediateRequest(uint8_t id);
#line 107
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$Resource$release(uint8_t id);
#line 126
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceDefaultOwner$release(void );
#line 146
static inline bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ArbiterInfo$inUse(void );








static inline uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ArbiterInfo$userId(void );
#line 173
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$grantedTask$runTask(void );
#line 185
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$Resource$default$granted(uint8_t id);



static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceRequested$default$immediateRequested(uint8_t id);









static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceConfigure$default$configure(uint8_t id);

static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceConfigure$default$unconfigure(uint8_t id);
# 52 "/opt/tinyos-2.0.2/tos/lib/power/PowerDownCleanup.nc"
static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$PowerDownCleanup$cleanup(void );
# 56 "/opt/tinyos-2.0.2/tos/interfaces/ResourceDefaultOwner.nc"
static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$ResourceDefaultOwner$release(void );
# 74 "/opt/tinyos-2.0.2/tos/interfaces/AsyncStdControl.nc"
static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$AsyncStdControl$start(void );









static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$AsyncStdControl$stop(void );
# 64 "/opt/tinyos-2.0.2/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$ResourceDefaultOwner$immediateRequested(void );




static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$ResourceDefaultOwner$granted(void );




static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$PowerDownCleanup$default$cleanup(void );
# 110 "/opt/tinyos-2.0.2/tos/interfaces/Resource.nc"
static error_t TelosSerialP$Resource$release(void );
#line 87
static error_t TelosSerialP$Resource$immediateRequest(void );
# 8 "/opt/tinyos-2.0.2/tos/platforms/telosa/TelosSerialP.nc"
msp430_uart_union_config_t TelosSerialP$msp430_uart_telos_config = { { .ubr = UBR_1MHZ_115200, .umctl = UMCTL_1MHZ_115200, .ssel = 0x02, .pena = 0, .pev = 0, .spb = 0, .clen = 1, .listen = 0, .mm = 0, .ckpl = 0, .urxse = 0, .urxeie = 1, .urxwie = 0, .utxe = 1, .urxe = 1 } };

static inline error_t TelosSerialP$StdControl$start(void );


static inline error_t TelosSerialP$StdControl$stop(void );



static inline void TelosSerialP$Resource$granted(void );

static inline msp430_uart_union_config_t *TelosSerialP$Msp430UartConfigure$getConfig(void );
# 40 "/opt/tinyos-2.0.2/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP$Info$offset(void );


static inline uint8_t SerialPacketInfoActiveMessageP$Info$dataLinkLength(message_t *msg, uint8_t upperLen);


static inline uint8_t SerialPacketInfoActiveMessageP$Info$upperLength(message_t *msg, uint8_t dataLinkLen);
# 196 "/opt/tinyos-2.0.2/tos/chips/msp430/msp430hardware.h"
static inline void __nesc_enable_interrupt(void )
{
   __asm volatile ("eint");}

# 185 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Timer$overflow(void )
{
}

# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$overflow(void ){
#line 37
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Timer$overflow();
#line 37
}
#line 37
# 126 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Overflow$fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Timer$overflow();
}





static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$default$fired(uint8_t n)
{
}

# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$fired(uint8_t arg_0x405d6c08){
#line 28
  switch (arg_0x405d6c08) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Event$fired();
#line 28
      break;
#line 28
    case 1:
#line 28
      /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Event$fired();
#line 28
      break;
#line 28
    case 2:
#line 28
      /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Event$fired();
#line 28
      break;
#line 28
    case 5:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Overflow$fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$default$fired(arg_0x405d6c08);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 115 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX0$fired(void )
{
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$fired(0);
}

# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP$VectorTimerA0$fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX0$fired();
#line 28
}
#line 28
# 47 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$__nesc_unnamed4323 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$cc_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$int2CC(* (volatile uint16_t *)354U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$default$captured(time);
#line 75
}
#line 75
# 139 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$getEvent(void )
{
  return * (volatile uint16_t *)370U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$default$fired(void )
{
}

# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$default$fired();
#line 34
}
#line 34
# 47 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$__nesc_unnamed4324 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$cc_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$int2CC(* (volatile uint16_t *)356U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$default$captured(time);
#line 75
}
#line 75
# 139 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$getEvent(void )
{
  return * (volatile uint16_t *)372U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$default$fired(void )
{
}

# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$default$fired();
#line 34
}
#line 34
# 47 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$__nesc_unnamed4325 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$cc_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$int2CC(* (volatile uint16_t *)358U);
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$default$captured(time);
#line 75
}
#line 75
# 139 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$getEvent(void )
{
  return * (volatile uint16_t *)374U;
}

#line 181
static inline void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$default$fired(void )
{
}

# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$default$fired();
#line 34
}
#line 34
# 120 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX1$fired(void )
{
  uint8_t n = * (volatile uint16_t *)302U;

#line 123
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$Event$fired(n >> 1);
}

# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP$VectorTimerA1$fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerA*/Msp430TimerP$0$VectorTimerX1$fired();
#line 28
}
#line 28
# 115 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX0$fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$fired(0);
}

# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP$VectorTimerB0$fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX0$fired();
#line 28
}
#line 28
# 185 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Timer$overflow(void )
{
}

#line 185
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$overflow(void )
{
}

# 103 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$overflow(void )
{
}

# 47 "/opt/tinyos-2.0.2/tos/lib/timer/CounterToLocalTimeC.nc"
static inline void /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow(void )
{
}

# 166 "/opt/tinyos-2.0.2/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$overflow(void )
{
}

# 71 "/opt/tinyos-2.0.2/tos/lib/timer/Counter.nc"
inline static void /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$overflow(void ){
#line 71
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$overflow();
#line 71
  /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$overflow();
#line 71
}
#line 71
# 122 "/opt/tinyos-2.0.2/tos/lib/timer/TransformCounterC.nc"
static inline void /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$overflow(void )
{
  /* atomic removed: atomic calls only */
  {
    /*CounterMilli32C.Transform*/TransformCounterC$0$m_upper++;
    if ((/*CounterMilli32C.Transform*/TransformCounterC$0$m_upper & /*CounterMilli32C.Transform*/TransformCounterC$0$OVERFLOW_MASK) == 0) {
      /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$overflow();
      }
  }
}

# 186 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP$0$Counter$overflow(void )
#line 186
{
}

# 71 "/opt/tinyos-2.0.2/tos/lib/timer/Counter.nc"
inline static void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$overflow(void ){
#line 71
  /*Msp430Uart1P.UartP*/Msp430UartP$0$Counter$overflow();
#line 71
  /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$overflow();
#line 71
}
#line 71
# 53 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline void /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$overflow(void )
{
  /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$overflow();
}

# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$overflow(void ){
#line 37
  /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$overflow();
#line 37
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Timer$overflow();
#line 37
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Timer$overflow();
#line 37
}
#line 37
# 126 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Overflow$fired(void )
{
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$overflow();
}

# 56 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 70 "/opt/tinyos-2.0.2/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired(void )
{
#line 71
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$postTask();
}

# 67 "/opt/tinyos-2.0.2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$fired(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$fired();
#line 67
}
#line 67
# 151 "/opt/tinyos-2.0.2/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$fired(void )
{
  /* atomic removed: atomic calls only */
  {
    if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt == 0) 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$fired();
      }
    else 
      {
        /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$set_alarm();
      }
  }
}

# 67 "/opt/tinyos-2.0.2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$fired(void ){
#line 67
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$fired();
#line 67
}
#line 67
# 124 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$disableEvents(void )
{
  * (volatile uint16_t *)386U &= ~0x0010;
}

# 40 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents(void ){
#line 40
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$disableEvents();
#line 40
}
#line 40
# 59 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$fired(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$fired();
}

# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$fired(void ){
#line 34
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$getEvent(void )
{
  return * (volatile uint16_t *)402U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$default$captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$__nesc_unnamed4326 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$int2CC(* (volatile uint16_t *)386U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$captured(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$fired();
    }
}

# 86 "/opt/tinyos-2.0.2/tos/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP$isWaiting(uint8_t id)
{
  return SchedulerBasicP$m_next[id] != SchedulerBasicP$NO_TASK || SchedulerBasicP$m_tail == id;
}

static inline bool SchedulerBasicP$pushTask(uint8_t id)
{
  if (!SchedulerBasicP$isWaiting(id)) 
    {
      if (SchedulerBasicP$m_head == SchedulerBasicP$NO_TASK) 
        {
          SchedulerBasicP$m_head = id;
          SchedulerBasicP$m_tail = id;
        }
      else 
        {
          SchedulerBasicP$m_next[SchedulerBasicP$m_tail] = id;
          SchedulerBasicP$m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$get(void ){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 38 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline uint16_t /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$get(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$get();
}

# 53 "/opt/tinyos-2.0.2/tos/lib/timer/Counter.nc"
inline static /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$size_type /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$get(void ){
#line 53
  unsigned int result;
#line 53

#line 53
  result = /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$get();
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 70 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline bool /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$isOverflowPending(void )
{
  return * (volatile uint16_t *)384U & 1U;
}

# 35 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$isOverflowPending(void ){
#line 35
  unsigned char result;
#line 35

#line 35
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$isOverflowPending();
#line 35

#line 35
  return result;
#line 35
}
#line 35
# 43 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430CounterC.nc"
static inline bool /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$isOverflowPending(void )
{
  return /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Msp430Timer$isOverflowPending();
}

# 60 "/opt/tinyos-2.0.2/tos/lib/timer/Counter.nc"
inline static bool /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$isOverflowPending(void ){
#line 60
  unsigned char result;
#line 60

#line 60
  result = /*Msp430Counter32khzC.Counter*/Msp430CounterC$0$Counter$isOverflowPending();
#line 60

#line 60
  return result;
#line 60
}
#line 60
# 119 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$enableEvents(void )
{
  * (volatile uint16_t *)386U |= 0x0010;
}

# 39 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$enableEvents(void ){
#line 39
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$enableEvents();
#line 39
}
#line 39
# 84 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$clearPendingInterrupt(void )
{
  * (volatile uint16_t *)386U &= ~0x0001;
}

# 33 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$clearPendingInterrupt(void ){
#line 33
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$clearPendingInterrupt();
#line 33
}
#line 33
# 144 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEvent(uint16_t x)
{
  * (volatile uint16_t *)402U = x;
}

# 30 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEvent(uint16_t time){
#line 30
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEvent(time);
#line 30
}
#line 30
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$get(void ){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 154 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEventFromNow(uint16_t x)
{
  * (volatile uint16_t *)402U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Timer$get() + x;
}

# 32 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEventFromNow(uint16_t delta){
#line 32
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Compare$setEventFromNow(delta);
#line 32
}
#line 32
# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Timer.nc"
inline static uint16_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$get(void ){
#line 34
  unsigned int result;
#line 34

#line 34
  result = /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get();
#line 34

#line 34
  return result;
#line 34
}
#line 34
# 70 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$startAt(uint16_t t0, uint16_t dt)
{
  /* atomic removed: atomic calls only */
  {
    uint16_t now = /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Timer$get();
    uint16_t elapsed = now - t0;

#line 76
    if (elapsed >= dt) 
      {
        /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEventFromNow(2);
      }
    else 
      {
        uint16_t remaining = dt - elapsed;

#line 83
        if (remaining <= 2) {
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEventFromNow(2);
          }
        else {
#line 86
          /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430Compare$setEvent(now + remaining);
          }
      }
#line 88
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$clearPendingInterrupt();
    /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$enableEvents();
  }
}

# 92 "/opt/tinyos-2.0.2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$startAt(t0, dt);
#line 92
}
#line 92
# 181 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$default$fired(void )
{
}

# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$getEvent(void )
{
  return * (volatile uint16_t *)404U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$default$captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$__nesc_unnamed4327 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$cc_t /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$int2CC(* (volatile uint16_t *)388U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$captured(/*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Compare$fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$default$fired(void )
{
}

# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$getEvent(void )
{
  return * (volatile uint16_t *)406U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$default$captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$__nesc_unnamed4328 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$cc_t /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$int2CC(* (volatile uint16_t *)390U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$captured(/*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Compare$fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$default$fired(void )
{
}

# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$getEvent(void )
{
  return * (volatile uint16_t *)408U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$default$captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$__nesc_unnamed4329 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$cc_t /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$int2CC(* (volatile uint16_t *)392U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$captured(/*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Compare$fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$default$fired(void )
{
}

# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$getEvent(void )
{
  return * (volatile uint16_t *)410U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$default$captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$__nesc_unnamed4330 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$cc_t /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$int2CC(* (volatile uint16_t *)394U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$captured(/*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Compare$fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$default$fired(void )
{
}

# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$getEvent(void )
{
  return * (volatile uint16_t *)412U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$default$captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$__nesc_unnamed4331 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$cc_t /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$int2CC(* (volatile uint16_t *)396U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$captured(/*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Compare$fired();
    }
}




static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$default$fired(void )
{
}

# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Compare.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$fired(void ){
#line 34
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$default$fired();
#line 34
}
#line 34
# 139 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$getEvent(void )
{
  return * (volatile uint16_t *)414U;
}

#line 177
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$default$captured(uint16_t n)
{
}

# 75 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430Capture.nc"
inline static void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$captured(uint16_t time){
#line 75
  /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$default$captured(time);
#line 75
}
#line 75
# 47 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$int2CC(uint16_t x)
#line 47
{
#line 47
  union /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$__nesc_unnamed4332 {
#line 47
    uint16_t f;
#line 47
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t t;
  } 
#line 47
  c = { .f = x };

#line 47
  return c.t;
}

#line 74
static inline /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$cc_t /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Control$getControl(void )
{
  return /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$int2CC(* (volatile uint16_t *)398U);
}

#line 169
static inline void /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$captured(/*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Compare$fired();
    }
}

# 120 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static inline void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX1$fired(void )
{
  uint8_t n = * (volatile uint16_t *)286U;

#line 123
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$fired(n >> 1);
}

# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
inline static void Msp430TimerCommonP$VectorTimerB1$fired(void ){
#line 28
  /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$VectorTimerX1$fired();
#line 28
}
#line 28
# 113 "/opt/tinyos-2.0.2/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP$Scheduler$init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP$m_next, SchedulerBasicP$NO_TASK, sizeof SchedulerBasicP$m_next);
    SchedulerBasicP$m_head = SchedulerBasicP$NO_TASK;
    SchedulerBasicP$m_tail = SchedulerBasicP$NO_TASK;
  }
}

# 46 "/opt/tinyos-2.0.2/tos/interfaces/Scheduler.nc"
inline static void RealMainP$Scheduler$init(void ){
#line 46
  SchedulerBasicP$Scheduler$init();
#line 46
}
#line 46
# 45 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$set(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t *)49U |= 0x01 << 6;
}

# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$set(void ){
#line 34
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$set();
}

# 29 "/opt/tinyos-2.0.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led2$set(void ){
#line 29
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$set();
#line 29
}
#line 29
# 45 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$set(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t *)49U |= 0x01 << 5;
}

# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$set(void ){
#line 34
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$set();
}

# 29 "/opt/tinyos-2.0.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led1$set(void ){
#line 29
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$set();
#line 29
}
#line 29
# 45 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$set(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 45
  * (volatile uint8_t *)49U |= 0x01 << 4;
}

# 34 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$set(void ){
#line 34
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$set();
#line 34
}
#line 34
# 37 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$set(void )
#line 37
{
#line 37
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$set();
}

# 29 "/opt/tinyos-2.0.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led0$set(void ){
#line 29
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$set();
#line 29
}
#line 29
# 52 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t *)50U |= 0x01 << 6;
}

# 71 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P56*/HplMsp430GeneralIOP$38$IO$makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$HplGeneralIO$makeOutput();
}

# 35 "/opt/tinyos-2.0.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led2$makeOutput(void ){
#line 35
  /*PlatformLedsC.Led2Impl*/Msp430GpioC$2$GeneralIO$makeOutput();
#line 35
}
#line 35
# 52 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t *)50U |= 0x01 << 5;
}

# 71 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P55*/HplMsp430GeneralIOP$37$IO$makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$HplGeneralIO$makeOutput();
}

# 35 "/opt/tinyos-2.0.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led1$makeOutput(void ){
#line 35
  /*PlatformLedsC.Led1Impl*/Msp430GpioC$1$GeneralIO$makeOutput();
#line 35
}
#line 35
# 52 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$makeOutput(void )
#line 52
{
  /* atomic removed: atomic calls only */
#line 52
  * (volatile uint8_t *)50U |= 0x01 << 4;
}

# 71 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$makeOutput(void ){
#line 71
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$makeOutput();
#line 71
}
#line 71
# 43 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$makeOutput(void )
#line 43
{
#line 43
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$makeOutput();
}

# 35 "/opt/tinyos-2.0.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led0$makeOutput(void ){
#line 35
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$makeOutput();
#line 35
}
#line 35
# 45 "/opt/tinyos-2.0.2/tos/system/LedsP.nc"
static inline error_t LedsP$Init$init(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 46
  {
    ;
    LedsP$Led0$makeOutput();
    LedsP$Led1$makeOutput();
    LedsP$Led2$makeOutput();
    LedsP$Led0$set();
    LedsP$Led1$set();
    LedsP$Led2$set();
  }
  return SUCCESS;
}

# 51 "/opt/tinyos-2.0.2/tos/interfaces/Init.nc"
inline static error_t PlatformP$LedsInit$init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = LedsP$Init$init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 33 "/opt/tinyos-2.0.2/tos/platforms/telosb/hardware.h"
static inline void TOSH_SET_SIMO0_PIN()
#line 33
{
#line 33
  static volatile uint8_t r __asm ("0x0019");

#line 33
  r |= 1 << 1;
}

#line 34
static inline void TOSH_SET_UCLK0_PIN()
#line 34
{
#line 34
  static volatile uint8_t r __asm ("0x0019");

#line 34
  r |= 1 << 3;
}

#line 85
static inline void TOSH_SET_FLASH_CS_PIN()
#line 85
{
#line 85
  static volatile uint8_t r __asm ("0x001D");

#line 85
  r |= 1 << 4;
}

#line 34
static inline void TOSH_CLR_UCLK0_PIN()
#line 34
{
#line 34
  static volatile uint8_t r __asm ("0x0019");

#line 34
  r &= ~(1 << 3);
}

#line 85
static inline void TOSH_CLR_FLASH_CS_PIN()
#line 85
{
#line 85
  static volatile uint8_t r __asm ("0x001D");

#line 85
  r &= ~(1 << 4);
}

# 11 "/opt/tinyos-2.0.2/tos/platforms/telosb/MotePlatformC.nc"
static __inline void MotePlatformC$TOSH_wait(void )
#line 11
{
   __asm volatile ("nop"); __asm volatile ("nop");}

# 86 "/opt/tinyos-2.0.2/tos/platforms/telosb/hardware.h"
static inline void TOSH_SET_FLASH_HOLD_PIN()
#line 86
{
#line 86
  static volatile uint8_t r __asm ("0x001D");

#line 86
  r |= 1 << 7;
}

#line 85
static inline void TOSH_MAKE_FLASH_CS_OUTPUT()
#line 85
{
#line 85
  static volatile uint8_t r __asm ("0x001E");

#line 85
  r |= 1 << 4;
}

#line 86
static inline void TOSH_MAKE_FLASH_HOLD_OUTPUT()
#line 86
{
#line 86
  static volatile uint8_t r __asm ("0x001E");

#line 86
  r |= 1 << 7;
}

#line 34
static inline void TOSH_MAKE_UCLK0_OUTPUT()
#line 34
{
#line 34
  static volatile uint8_t r __asm ("0x001A");

#line 34
  r |= 1 << 3;
}

#line 33
static inline void TOSH_MAKE_SIMO0_OUTPUT()
#line 33
{
#line 33
  static volatile uint8_t r __asm ("0x001A");

#line 33
  r |= 1 << 1;
}

# 27 "/opt/tinyos-2.0.2/tos/platforms/telosb/MotePlatformC.nc"
static inline void MotePlatformC$TOSH_FLASH_M25P_DP(void )
#line 27
{

  TOSH_MAKE_SIMO0_OUTPUT();
  TOSH_MAKE_UCLK0_OUTPUT();
  TOSH_MAKE_FLASH_HOLD_OUTPUT();
  TOSH_MAKE_FLASH_CS_OUTPUT();
  TOSH_SET_FLASH_HOLD_PIN();
  TOSH_SET_FLASH_CS_PIN();

  MotePlatformC$TOSH_wait();


  TOSH_CLR_FLASH_CS_PIN();
  TOSH_CLR_UCLK0_PIN();

  MotePlatformC$TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(TRUE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(FALSE);
  MotePlatformC$TOSH_FLASH_M25P_DP_bit(TRUE);

  TOSH_SET_FLASH_CS_PIN();
  TOSH_SET_UCLK0_PIN();
  TOSH_SET_SIMO0_PIN();
}

#line 6
static __inline void MotePlatformC$uwait(uint16_t u)
#line 6
{
  uint16_t t0 = TA0R;

#line 8
  while (TA0R - t0 <= u) ;
}

#line 56
static inline error_t MotePlatformC$Init$init(void )
#line 56
{
  /* atomic removed: atomic calls only */

  {
    P1SEL = 0;
    P2SEL = 0;
    P3SEL = 0;
    P4SEL = 0;
    P5SEL = 0;
    P6SEL = 0;

    P1DIR = 0xe0;
    P1OUT = 0x00;

    P2DIR = 0x7b;
    P2OUT = 0x30;

    P3DIR = 0xf1;
    P3OUT = 0x00;

    P4DIR = 0xfd;
    P4OUT = 0xdd;

    P5DIR = 0xff;
    P5OUT = 0xff;

    P6DIR = 0xff;
    P6OUT = 0x00;

    P1IE = 0;
    P2IE = 0;






    MotePlatformC$uwait(1024 * 10);

    MotePlatformC$TOSH_FLASH_M25P_DP();
  }

  return SUCCESS;
}

# 51 "/opt/tinyos-2.0.2/tos/interfaces/Init.nc"
inline static error_t PlatformP$MoteInit$init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = MotePlatformC$Init$init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 148 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP$startTimerB(void )
{

  Msp430ClockP$TBCTL = 0x0020 | (Msp430ClockP$TBCTL & ~(0x0020 | 0x0010));
}

#line 136
static inline void Msp430ClockP$startTimerA(void )
{

  Msp430ClockP$TA0CTL = 0x0020 | (Msp430ClockP$TA0CTL & ~(0x0020 | 0x0010));
}

#line 100
static inline void Msp430ClockP$Msp430ClockInit$defaultInitTimerB(void )
{
  TBR = 0;









  Msp430ClockP$TBCTL = 0x0100 | 0x0002;
}

#line 130
static inline void Msp430ClockP$Msp430ClockInit$default$initTimerB(void )
{
  Msp430ClockP$Msp430ClockInit$defaultInitTimerB();
}

# 32 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP$Msp430ClockInit$initTimerB(void ){
#line 32
  Msp430ClockP$Msp430ClockInit$default$initTimerB();
#line 32
}
#line 32
# 85 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP$Msp430ClockInit$defaultInitTimerA(void )
{
  TA0R = 0;









  Msp430ClockP$TA0CTL = 0x0200 | 0x0002;
}

#line 125
static inline void Msp430ClockP$Msp430ClockInit$default$initTimerA(void )
{
  Msp430ClockP$Msp430ClockInit$defaultInitTimerA();
}

# 31 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP$Msp430ClockInit$initTimerA(void ){
#line 31
  Msp430ClockP$Msp430ClockInit$default$initTimerA();
#line 31
}
#line 31
# 64 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline void Msp430ClockP$Msp430ClockInit$defaultInitClocks(void )
{





  BCSCTL1 = 0x80 | (BCSCTL1 & ((0x04 | 0x02) | 0x01));







  BCSCTL2 = 0x04;


  Msp430ClockP$IE1 &= ~(1 << 1);
}

#line 120
static inline void Msp430ClockP$Msp430ClockInit$default$initClocks(void )
{
  Msp430ClockP$Msp430ClockInit$defaultInitClocks();
}

# 30 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP$Msp430ClockInit$initClocks(void ){
#line 30
  Msp430ClockP$Msp430ClockInit$default$initClocks();
#line 30
}
#line 30
# 166 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline uint16_t Msp430ClockP$test_calib_busywait_delta(int calib)
{
  int8_t aclk_count = 2;
  uint16_t dco_prev = 0;
  uint16_t dco_curr = 0;

  Msp430ClockP$set_dco_calib(calib);

  while (aclk_count-- > 0) 
    {
      TBCCR0 = TBR + Msp430ClockP$ACLK_CALIB_PERIOD;
      TBCCTL0 &= ~0x0001;
      while ((TBCCTL0 & 0x0001) == 0) ;
      dco_prev = dco_curr;
      dco_curr = TA0R;
    }

  return dco_curr - dco_prev;
}




static inline void Msp430ClockP$busyCalibrateDco(void )
{

  int calib;
  int step;






  for (calib = 0, step = 0x800; step != 0; step >>= 1) 
    {

      if (Msp430ClockP$test_calib_busywait_delta(calib | step) <= Msp430ClockP$TARGET_DCO_DELTA) {
        calib |= step;
        }
    }

  if ((calib & 0x0e0) == 0x0e0) {
    calib &= ~0x01f;
    }
  Msp430ClockP$set_dco_calib(calib);
}

#line 52
static inline void Msp430ClockP$Msp430ClockInit$defaultSetupDcoCalibrate(void )
{



  Msp430ClockP$TA0CTL = 0x0200 | 0x0020;
  Msp430ClockP$TBCTL = 0x0100 | 0x0020;
  BCSCTL1 = 0x80 | 0x04;
  BCSCTL2 = 0;
  TBCCTL0 = 0x4000;
}

#line 115
static inline void Msp430ClockP$Msp430ClockInit$default$setupDcoCalibrate(void )
{
  Msp430ClockP$Msp430ClockInit$defaultSetupDcoCalibrate();
}

# 29 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430ClockInit.nc"
inline static void Msp430ClockP$Msp430ClockInit$setupDcoCalibrate(void ){
#line 29
  Msp430ClockP$Msp430ClockInit$default$setupDcoCalibrate();
#line 29
}
#line 29
# 214 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430ClockP.nc"
static inline error_t Msp430ClockP$Init$init(void )
{

  Msp430ClockP$TA0CTL = 0x0004;
  Msp430ClockP$TA0IV = 0;
  Msp430ClockP$TBCTL = 0x0004;
  Msp430ClockP$TBIV = 0;
  /* atomic removed: atomic calls only */

  {
    Msp430ClockP$Msp430ClockInit$setupDcoCalibrate();
    Msp430ClockP$busyCalibrateDco();
    Msp430ClockP$Msp430ClockInit$initClocks();
    Msp430ClockP$Msp430ClockInit$initTimerA();
    Msp430ClockP$Msp430ClockInit$initTimerB();
    Msp430ClockP$startTimerA();
    Msp430ClockP$startTimerB();
  }

  return SUCCESS;
}

# 51 "/opt/tinyos-2.0.2/tos/interfaces/Init.nc"
inline static error_t PlatformP$MoteClockInit$init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = Msp430ClockP$Init$init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 10 "/opt/tinyos-2.0.2/tos/platforms/telosa/PlatformP.nc"
static inline error_t PlatformP$Init$init(void )
#line 10
{
  PlatformP$MoteClockInit$init();
  PlatformP$MoteInit$init();
  PlatformP$LedsInit$init();
  return SUCCESS;
}

# 51 "/opt/tinyos-2.0.2/tos/interfaces/Init.nc"
inline static error_t RealMainP$PlatformInit$init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = PlatformP$Init$init();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 33 "/opt/tinyos-2.0.2/tos/platforms/telosb/hardware.h"
static inline void TOSH_CLR_SIMO0_PIN()
#line 33
{
#line 33
  static volatile uint8_t r __asm ("0x0019");

#line 33
  r &= ~(1 << 1);
}

# 54 "/opt/tinyos-2.0.2/tos/interfaces/Scheduler.nc"
inline static bool RealMainP$Scheduler$runNextTask(void ){
#line 54
  unsigned char result;
#line 54

#line 54
  result = SchedulerBasicP$Scheduler$runNextTask();
#line 54

#line 54
  return result;
#line 54
}
#line 54
# 17 "/opt/tinyos-2.0.2/tos/platforms/telosa/TelosSerialP.nc"
static inline void TelosSerialP$Resource$granted(void )
#line 17
{
}

# 196 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP$0$Resource$default$granted(uint8_t id)
#line 196
{
}

# 92 "/opt/tinyos-2.0.2/tos/interfaces/Resource.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP$0$Resource$granted(uint8_t arg_0x40e27450){
#line 92
  switch (arg_0x40e27450) {
#line 92
    case /*PlatformSerialC.UartC*/Msp430Uart1C$0$CLIENT_ID:
#line 92
      TelosSerialP$Resource$granted();
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430Uart1P.UartP*/Msp430UartP$0$Resource$default$granted(arg_0x40e27450);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 96 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartResource$granted(uint8_t id)
#line 96
{
  /*Msp430Uart1P.UartP*/Msp430UartP$0$Resource$granted(id);
}

# 185 "/opt/tinyos-2.0.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$Resource$default$granted(uint8_t id)
#line 185
{
}

# 92 "/opt/tinyos-2.0.2/tos/interfaces/Resource.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$Resource$granted(uint8_t arg_0x40ef9010){
#line 92
  switch (arg_0x40ef9010) {
#line 92
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C$0$CLIENT_ID:
#line 92
      /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartResource$granted(/*PlatformSerialC.UartC*/Msp430Uart1C$0$CLIENT_ID);
#line 92
      break;
#line 92
    default:
#line 92
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$Resource$default$granted(arg_0x40ef9010);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 199 "/opt/tinyos-2.0.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceConfigure$default$configure(uint8_t id)
#line 199
{
}

# 49 "/opt/tinyos-2.0.2/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceConfigure$configure(uint8_t arg_0x40ef8df0){
#line 49
  switch (arg_0x40ef8df0) {
#line 49
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C$0$CLIENT_ID:
#line 49
      /*Msp430Uart1P.UartP*/Msp430UartP$0$ResourceConfigure$configure(/*PlatformSerialC.UartC*/Msp430Uart1C$0$CLIENT_ID);
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceConfigure$default$configure(arg_0x40ef8df0);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 173 "/opt/tinyos-2.0.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$grantedTask$runTask(void )
#line 173
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 174
    {
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$reqResId;
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$RES_BUSY;
    }
#line 177
    __nesc_atomic_end(__nesc_atomic); }
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceConfigure$configure(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$resId);
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$Resource$granted(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$resId);
}

# 19 "/opt/tinyos-2.0.2/tos/platforms/telosa/TelosSerialP.nc"
static inline msp430_uart_union_config_t *TelosSerialP$Msp430UartConfigure$getConfig(void )
#line 19
{
  return &TelosSerialP$msp430_uart_telos_config;
}

# 192 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP$0$Msp430UartConfigure$default$getConfig(uint8_t id)
#line 192
{
  return &msp430_uart_default_config;
}

# 39 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartConfigure.nc"
inline static msp430_uart_union_config_t */*Msp430Uart1P.UartP*/Msp430UartP$0$Msp430UartConfigure$getConfig(uint8_t arg_0x40e23010){
#line 39
  union __nesc_unnamed4285 *result;
#line 39

#line 39
  switch (arg_0x40e23010) {
#line 39
    case /*PlatformSerialC.UartC*/Msp430Uart1C$0$CLIENT_ID:
#line 39
      result = TelosSerialP$Msp430UartConfigure$getConfig();
#line 39
      break;
#line 39
    default:
#line 39
      result = /*Msp430Uart1P.UartP*/Msp430UartP$0$Msp430UartConfigure$default$getConfig(arg_0x40e23010);
#line 39
      break;
#line 39
    }
#line 39

#line 39
  return result;
#line 39
}
#line 39
# 359 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P$Usart$disableIntr(void )
#line 359
{
  HplMsp430Usart1P$IE2 &= ~((1 << 5) | (1 << 4));
}

#line 347
static inline void HplMsp430Usart1P$Usart$clrIntr(void )
#line 347
{
  HplMsp430Usart1P$IFG2 &= ~((1 << 5) | (1 << 4));
}

#line 159
static inline void HplMsp430Usart1P$Usart$resetUsart(bool reset)
#line 159
{
  if (reset) {
    U1CTL = 0x01;
    }
  else {
#line 163
    U1CTL &= ~0x01;
    }
}

# 54 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP$22$IO$selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)27U |= 0x01 << 6;
}

# 78 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P$UTXD$selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP$22$IO$selectModuleFunc();
#line 78
}
#line 78
# 220 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P$Usart$enableUartTx(void )
#line 220
{
  HplMsp430Usart1P$UTXD$selectModuleFunc();
  HplMsp430Usart1P$ME2 |= 1 << 5;
}

# 56 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP$23$IO$selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)27U &= ~(0x01 << 7);
}

# 85 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P$URXD$selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP$23$IO$selectIOFunc();
#line 85
}
#line 85
# 236 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P$Usart$disableUartRx(void )
#line 236
{
  HplMsp430Usart1P$ME2 &= ~(1 << 4);
  HplMsp430Usart1P$URXD$selectIOFunc();
}

# 54 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP$23$IO$selectModuleFunc(void )
#line 54
{
  /* atomic removed: atomic calls only */
#line 54
  * (volatile uint8_t *)27U |= 0x01 << 7;
}

# 78 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P$URXD$selectModuleFunc(void ){
#line 78
  /*HplMsp430GeneralIOC.P37*/HplMsp430GeneralIOP$23$IO$selectModuleFunc();
#line 78
}
#line 78
# 231 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P$Usart$enableUartRx(void )
#line 231
{
  HplMsp430Usart1P$URXD$selectModuleFunc();
  HplMsp430Usart1P$ME2 |= 1 << 4;
}

# 56 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP$22$IO$selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)27U &= ~(0x01 << 6);
}

# 85 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P$UTXD$selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P36*/HplMsp430GeneralIOP$22$IO$selectIOFunc();
#line 85
}
#line 85
# 225 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P$Usart$disableUartTx(void )
#line 225
{
  HplMsp430Usart1P$ME2 &= ~(1 << 5);
  HplMsp430Usart1P$UTXD$selectIOFunc();
}

#line 203
static inline void HplMsp430Usart1P$Usart$enableUart(void )
#line 203
{
  /* atomic removed: atomic calls only */
#line 204
  {
    HplMsp430Usart1P$UTXD$selectModuleFunc();
    HplMsp430Usart1P$URXD$selectModuleFunc();
  }
  HplMsp430Usart1P$ME2 |= (1 << 5) | (1 << 4);
}

#line 151
static inline void HplMsp430Usart1P$Usart$setUmctl(uint8_t control)
#line 151
{
  U1MCTL = control;
}

#line 140
static inline void HplMsp430Usart1P$Usart$setUbr(uint16_t control)
#line 140
{
  /* atomic removed: atomic calls only */
#line 141
  {
    U1BR0 = control & 0x00FF;
    U1BR1 = (control >> 8) & 0x00FF;
  }
}

#line 283
static inline void HplMsp430Usart1P$configUart(msp430_uart_union_config_t *config)
#line 283
{

  U1CTL = (config->uartRegisters.uctl & ~0x04) | 0x01;
  HplMsp430Usart1P$U1TCTL = config->uartRegisters.utctl;
  HplMsp430Usart1P$U1RCTL = config->uartRegisters.urctl;

  HplMsp430Usart1P$Usart$setUbr(config->uartRegisters.ubr);
  HplMsp430Usart1P$Usart$setUmctl(config->uartRegisters.umctl);
}

static inline void HplMsp430Usart1P$Usart$setModeUart(msp430_uart_union_config_t *config)
#line 293
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 295
    {
      HplMsp430Usart1P$Usart$resetUsart(TRUE);
      HplMsp430Usart1P$Usart$disableSpi();
      HplMsp430Usart1P$configUart(config);
      if (config->uartConfig.utxe == 1 && config->uartConfig.urxe == 1) {
          HplMsp430Usart1P$Usart$enableUart();
        }
      else {
#line 301
        if (config->uartConfig.utxe == 0 && config->uartConfig.urxe == 1) {
            HplMsp430Usart1P$Usart$disableUartTx();
            HplMsp430Usart1P$Usart$enableUartRx();
          }
        else {
#line 304
          if (config->uartConfig.utxe == 1 && config->uartConfig.urxe == 0) {
              HplMsp430Usart1P$Usart$disableUartRx();
              HplMsp430Usart1P$Usart$enableUartTx();
            }
          else 
#line 307
            {
              HplMsp430Usart1P$Usart$disableUart();
            }
          }
        }
#line 310
      HplMsp430Usart1P$Usart$resetUsart(FALSE);
      HplMsp430Usart1P$Usart$clrIntr();
      HplMsp430Usart1P$Usart$disableIntr();
    }
#line 313
    __nesc_atomic_end(__nesc_atomic); }

  return;
}

# 174 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP$0$Usart$setModeUart(msp430_uart_union_config_t *config){
#line 174
  HplMsp430Usart1P$Usart$setModeUart(config);
#line 174
}
#line 174
# 56 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP$33$IO$selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)51U &= ~(0x01 << 1);
}

# 85 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P$SIMO$selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P51*/HplMsp430GeneralIOP$33$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP$34$IO$selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)51U &= ~(0x01 << 2);
}

# 85 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P$SOMI$selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P52*/HplMsp430GeneralIOP$34$IO$selectIOFunc();
#line 85
}
#line 85
# 56 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP$35$IO$selectIOFunc(void )
#line 56
{
  /* atomic removed: atomic calls only */
#line 56
  * (volatile uint8_t *)51U &= ~(0x01 << 3);
}

# 85 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void HplMsp430Usart1P$UCLK$selectIOFunc(void ){
#line 85
  /*HplMsp430GeneralIOC.P53*/HplMsp430GeneralIOP$35$IO$selectIOFunc();
#line 85
}
#line 85
# 377 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P$Usart$enableIntr(void )
#line 377
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 378
    {
      HplMsp430Usart1P$IFG2 &= ~((1 << 5) | (1 << 4));
      HplMsp430Usart1P$IE2 |= (1 << 5) | (1 << 4);
    }
#line 381
    __nesc_atomic_end(__nesc_atomic); }
}

# 182 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP$0$Usart$enableIntr(void ){
#line 182
  HplMsp430Usart1P$Usart$enableIntr();
#line 182
}
#line 182
# 47 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIOP.nc"
static inline void /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$toggle(void )
#line 47
{
#line 47
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 47
    * (volatile uint8_t *)49U ^= 0x01 << 4;
#line 47
    __nesc_atomic_end(__nesc_atomic); }
}

# 44 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/HplMsp430GeneralIO.nc"
inline static void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$toggle(void ){
#line 44
  /*HplMsp430GeneralIOC.P54*/HplMsp430GeneralIOP$36$IO$toggle();
#line 44
}
#line 44
# 39 "/opt/tinyos-2.0.2/tos/chips/msp430/pins/Msp430GpioC.nc"
static inline void /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$toggle(void )
#line 39
{
#line 39
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$HplGeneralIO$toggle();
}

# 31 "/opt/tinyos-2.0.2/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led0$toggle(void ){
#line 31
  /*PlatformLedsC.Led0Impl*/Msp430GpioC$0$GeneralIO$toggle();
#line 31
}
#line 31
# 73 "/opt/tinyos-2.0.2/tos/system/LedsP.nc"
static inline void LedsP$Leds$led0Toggle(void )
#line 73
{
  LedsP$Led0$toggle();
  ;
#line 75
  ;
}

# 56 "/opt/tinyos-2.0.2/tos/interfaces/Leds.nc"
inline static void testECDSAM$Leds$led0Toggle(void ){
#line 56
  LedsP$Leds$led0Toggle();
#line 56
}
#line 56
# 69 "/opt/tinyos-2.0.2/tos/interfaces/AMSend.nc"
inline static error_t testECDSAM$TimeMsg$send(am_addr_t addr, message_t *msg, uint8_t len){
#line 69
  unsigned char result;
#line 69

#line 69
  result = /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMSend$send(AM_TIME_MSG, addr, msg, len);
#line 69

#line 69
  return result;
#line 69
}
#line 69
# 53 "/opt/tinyos-2.0.2/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$size_type /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$get(void ){
#line 53
  unsigned long result;
#line 53

#line 53
  result = /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$get();
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 42 "/opt/tinyos-2.0.2/tos/lib/timer/CounterToLocalTimeC.nc"
static inline uint32_t /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$LocalTime$get(void )
{
  return /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$Counter$get();
}

# 50 "/opt/tinyos-2.0.2/tos/lib/timer/LocalTime.nc"
inline static uint32_t testECDSAM$LocalTime$get(void ){
#line 50
  unsigned long result;
#line 50

#line 50
  result = /*HilTimerMilliC.CounterToLocalTimeC*/CounterToLocalTimeC$0$LocalTime$get();
#line 50

#line 50
  return result;
#line 50
}
#line 50
# 44 "ECDSA.nc"
inline static uint8_t testECDSAM$ECDSA$verify(uint8_t *msg, uint8_t len, NN_DIGIT *r, NN_DIGIT *s, Point *Q){
#line 44
  unsigned char result;
#line 44

#line 44
  result = ECDSAM$ECDSA$verify(msg, len, r, s, Q);
#line 44

#line 44
  return result;
#line 44
}
#line 44
# 239 "testECDSAM.nc"
static inline void testECDSAM$verify(void )
#line 239
{
  uint32_t time_a;
#line 240
  uint32_t time_b;
  time_msg *pTime;

  testECDSAM$type = 4;

  time_a = testECDSAM$LocalTime$get();

  testECDSAM$pass = testECDSAM$ECDSA$verify(testECDSAM$message, 52, testECDSAM$r, testECDSAM$s, &testECDSAM$PublicKey);

  time_b = testECDSAM$LocalTime$get();

  testECDSAM$t = time_b - time_a;

  pTime = (time_msg *)testECDSAM$report.data;
  pTime->type = 4;
  pTime->t = testECDSAM$t;
  pTime->pass = testECDSAM$pass;
  testECDSAM$TimeMsg$send(1, &testECDSAM$report, sizeof(time_msg ));
  if (testECDSAM$pass == 1) {
    testECDSAM$Leds$led0Toggle();
    }
}

# 69 "/opt/tinyos-2.0.2/tos/interfaces/AMSend.nc"
inline static error_t testECDSAM$PacketMsg$send(am_addr_t addr, message_t *msg, uint8_t len){
#line 69
  unsigned char result;
#line 69

#line 69
  result = /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMSend$send(AM_PACKET_MSG, addr, msg, len);
#line 69

#line 69
  return result;
#line 69
}
#line 69
# 3385 "NNM.nc"
static inline void NNM$NN$Encode(unsigned char *a, NN_UINT digits, NN_DIGIT *b, NN_UINT len)

{
  NNM$NN_Encode(a, digits, b, len);
}

# 43 "NN.nc"
inline static void testECDSAM$NN$Encode(unsigned char *a, NN_UINT digits, NN_DIGIT *b, NN_UINT len){
#line 43
  NNM$NN$Encode(a, digits, b, len);
#line 43
}
#line 43
# 41 "ECDSA.nc"
inline static void testECDSAM$ECDSA$sign(uint8_t *msg, uint8_t len, NN_DIGIT *r, NN_DIGIT *s, NN_DIGIT *d){
#line 41
  ECDSAM$ECDSA$sign(msg, len, r, s, d);
#line 41
}
#line 41
# 216 "testECDSAM.nc"
static inline void testECDSAM$sign(void )
#line 216
{
  uint32_t time_a;
#line 217
  uint32_t time_b;
  packet_msg *pPacket;

  testECDSAM$type = 3;

  time_a = testECDSAM$LocalTime$get();

  testECDSAM$ECDSA$sign(testECDSAM$message, 52, testECDSAM$r, testECDSAM$s, testECDSAM$PrivateKey);
#line 224
  ;

  time_b = testECDSAM$LocalTime$get();

  testECDSAM$t = time_b - time_a;

  pPacket = (packet_msg *)testECDSAM$report.data;
  pPacket->c_len = 52;
  memcpy(pPacket->content, testECDSAM$message, 52);
  pPacket->r_len = 160 / 16 * (16 / 8);
  testECDSAM$NN$Encode(pPacket->r, 160 / 16 * (16 / 8), testECDSAM$r, 160 / 16);
  testECDSAM$NN$Encode(pPacket->s, 160 / 16 * (16 / 8), testECDSAM$s, 160 / 16);
  testECDSAM$PacketMsg$send(1, &testECDSAM$report, sizeof(packet_msg ));
}

# 38 "ECDSA.nc"
inline static error_t testECDSAM$ECDSA$init(Point *pKey){
#line 38
  unsigned char result;
#line 38

#line 38
  result = ECDSAM$ECDSA$init(pKey);
#line 38

#line 38
  return result;
#line 38
}
#line 38
# 194 "testECDSAM.nc"
static inline void testECDSAM$ecdsa_init(void )
#line 194
{
  uint32_t time_a;
#line 195
  uint32_t time_b;
  time_msg *pTime;

  testECDSAM$type = 2;

  time_a = testECDSAM$LocalTime$get();

  testECDSAM$ECDSA$init(&testECDSAM$PublicKey);

  time_b = testECDSAM$LocalTime$get();

  testECDSAM$t = time_b - time_a;

  pTime = (time_msg *)testECDSAM$report.data;
  pTime->type = 2;
  pTime->t = testECDSAM$t;
  pTime->pass = 0;
  testECDSAM$TimeMsg$send(1, &testECDSAM$report, sizeof(time_msg ));
}

# 69 "/opt/tinyos-2.0.2/tos/interfaces/AMSend.nc"
inline static error_t testECDSAM$PubKeyMsg$send(am_addr_t addr, message_t *msg, uint8_t len){
#line 69
  unsigned char result;
#line 69

#line 69
  result = /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMSend$send(AM_PUBLIC_KEY_MSG, addr, msg, len);
#line 69

#line 69
  return result;
#line 69
}
#line 69
# 1034 "ECCM.nc"
static inline void ECCM$ECC$gen_public_key(Point *PublicKey, NN_DIGIT *PrivateKey)
#line 1034
{



  ECCM$win_mul(PublicKey, PrivateKey, ECCM$pBaseArray);
}

# 42 "ECC.nc"
inline static void testECDSAM$ECC$gen_public_key(Point *PublicKey, NN_DIGIT *PrivateKey){
#line 42
  ECCM$ECC$gen_public_key(PublicKey, PrivateKey);
#line 42
}
#line 42
# 171 "testECDSAM.nc"
static inline void testECDSAM$gen_PublicKey(void )
#line 171
{
  uint32_t time_a;
#line 172
  uint32_t time_b;
  public_key_msg *pPublicKey;

  testECDSAM$type = 1;

  time_a = testECDSAM$LocalTime$get();


  testECDSAM$ECC$gen_public_key(&testECDSAM$PublicKey, testECDSAM$PrivateKey);

  time_b = testECDSAM$LocalTime$get();

  testECDSAM$t = time_b - time_a;

  pPublicKey = (public_key_msg *)testECDSAM$report.data;
  pPublicKey->len = 160 / 16 * (16 / 8);
  testECDSAM$NN$Encode(pPublicKey->x, 160 / 16 * (16 / 8), testECDSAM$PublicKey.x, 160 / 16);
  testECDSAM$NN$Encode(pPublicKey->y, 160 / 16 * (16 / 8), testECDSAM$PublicKey.y, 160 / 16);
  testECDSAM$PubKeyMsg$send(1, &testECDSAM$report, sizeof(public_key_msg ));
}

#line 304
static inline void testECDSAM$TimeMsg$sendDone(message_t *sent, error_t success)
#line 304
{

  if (testECDSAM$type == 0) {

      testECDSAM$gen_PublicKey();
    }
  else {
#line 309
    if (testECDSAM$type == 1) {

        testECDSAM$ecdsa_init();
      }
    else {
#line 312
      if (testECDSAM$type == 2) {

          testECDSAM$sign();
        }
      else {
#line 315
        if (testECDSAM$type == 3) {

            testECDSAM$verify();
          }
        else {
#line 318
          if (testECDSAM$type == 4) {
              if (testECDSAM$round_index < 10) {
                  testECDSAM$init_data();
                  testECDSAM$round_index++;
                }
            }
          }
        }
      }
    }
}

#line 293
static inline void testECDSAM$PacketMsg$sendDone(message_t *sent, error_t success)
#line 293
{
  time_msg *pTime;

  testECDSAM$type = 3;
  pTime = (time_msg *)testECDSAM$report.data;
  pTime->type = 3;
  pTime->t = testECDSAM$t;
  pTime->pass = 0;
  testECDSAM$TimeMsg$send(1, &testECDSAM$report, sizeof(time_msg ));
}

# 38 "ECC.nc"
inline static void testECDSAM$ECC$init(void ){
#line 38
  ECCM$ECC$init();
#line 38
}
#line 38
# 150 "testECDSAM.nc"
static inline void testECDSAM$ecc_init(void )
#line 150
{
  uint32_t time_a;
#line 151
  uint32_t time_b;
  time_msg *pTime;

  testECDSAM$type = 0;

  time_a = testECDSAM$LocalTime$get();

  testECDSAM$ECC$init();

  time_b = testECDSAM$LocalTime$get();

  testECDSAM$t = time_b - time_a;

  pTime = (time_msg *)testECDSAM$report.data;
  pTime->type = 0;
  pTime->t = testECDSAM$t;
  pTime->pass = 0;
  testECDSAM$TimeMsg$send(1, &testECDSAM$report, sizeof(time_msg ));
}

#line 289
static inline void testECDSAM$PriKeyMsg$sendDone(message_t *sent, error_t success)
#line 289
{
  testECDSAM$ecc_init();
}

#line 278
static inline void testECDSAM$PubKeyMsg$sendDone(message_t *sent, error_t error)
#line 278
{
  time_msg *pTime;

  testECDSAM$type = 1;
  pTime = (time_msg *)testECDSAM$report.data;
  pTime->type = 1;
  pTime->t = testECDSAM$t;
  pTime->pass = 0;
  testECDSAM$TimeMsg$send(1, &testECDSAM$report, sizeof(time_msg ));
}

# 85 "/opt/tinyos-2.0.2/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMSend$default$sendDone(uint8_t id, message_t *msg, error_t result)
#line 85
{
  return;
}

# 99 "/opt/tinyos-2.0.2/tos/interfaces/AMSend.nc"
inline static void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMSend$sendDone(am_id_t arg_0x40ccd888, message_t *msg, error_t error){
#line 99
  switch (arg_0x40ccd888) {
#line 99
    case AM_PUBLIC_KEY_MSG:
#line 99
      testECDSAM$PubKeyMsg$sendDone(msg, error);
#line 99
      break;
#line 99
    case AM_PRIVATE_KEY_MSG:
#line 99
      testECDSAM$PriKeyMsg$sendDone(msg, error);
#line 99
      break;
#line 99
    case AM_TIME_MSG:
#line 99
      testECDSAM$TimeMsg$sendDone(msg, error);
#line 99
      break;
#line 99
    case AM_PACKET_MSG:
#line 99
      testECDSAM$PacketMsg$sendDone(msg, error);
#line 99
      break;
#line 99
    default:
#line 99
      /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMSend$default$sendDone(arg_0x40ccd888, msg, error);
#line 99
      break;
#line 99
    }
#line 99
}
#line 99
# 81 "/opt/tinyos-2.0.2/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubSend$sendDone(message_t *msg, error_t result)
#line 81
{
  /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMSend$sendDone(/*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$type(msg), msg, result);
}

# 367 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$default$sendDone(uart_id_t idxxx, message_t *msg, error_t error)
#line 367
{
  return;
}

# 89 "/opt/tinyos-2.0.2/tos/interfaces/Send.nc"
inline static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$sendDone(uart_id_t arg_0x40d8d160, message_t *msg, error_t error){
#line 89
  switch (arg_0x40d8d160) {
#line 89
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 89
      /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubSend$sendDone(msg, error);
#line 89
      break;
#line 89
    default:
#line 89
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$default$sendDone(arg_0x40d8d160, msg, error);
#line 89
      break;
#line 89
    }
#line 89
}
#line 89
# 152 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$signalSendDone$runTask(void )
#line 152
{
  error_t error;

  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SEND_STATE_IDLE;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 156
    error = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendError;
#line 156
    __nesc_atomic_end(__nesc_atomic); }

  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendCancelled) {
#line 158
    error = ECANCEL;
    }
#line 159
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$sendDone(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendId, (message_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendBuffer, error);
}

# 49 "/opt/tinyos-2.0.2/tos/lib/serial/SerialActiveMessageP.nc"
static inline serial_header_t */*SerialActiveMessageC.AM*/SerialActiveMessageP$0$getHeader(message_t *msg)
#line 49
{
  return (serial_header_t *)(msg->data - sizeof(serial_header_t ));
}

# 522 "/opt/tinyos-2.0.2/tos/lib/serial/SerialP.nc"
static inline error_t SerialP$SendBytePacket$startSend(uint8_t b)
#line 522
{
  bool not_busy = FALSE;

#line 524
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 524
    {
      if (SerialP$txBuf[SerialP$TX_DATA_INDEX].state == SerialP$BUFFER_AVAILABLE) {
          SerialP$txBuf[SerialP$TX_DATA_INDEX].state = SerialP$BUFFER_FILLING;
          SerialP$txBuf[SerialP$TX_DATA_INDEX].buf = b;
          not_busy = TRUE;
        }
    }
#line 530
    __nesc_atomic_end(__nesc_atomic); }
  if (not_busy) {
      SerialP$MaybeScheduleTx();
      return SUCCESS;
    }
  return EBUSY;
}

# 51 "/opt/tinyos-2.0.2/tos/lib/serial/SendBytePacket.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$startSend(uint8_t first_byte){
#line 51
  unsigned char result;
#line 51

#line 51
  result = SerialP$SendBytePacket$startSend(first_byte);
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 43 "/opt/tinyos-2.0.2/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP$Info$dataLinkLength(message_t *msg, uint8_t upperLen)
#line 43
{
  return upperLen + sizeof(serial_header_t );
}

# 352 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$dataLinkLength(uart_id_t id, message_t *msg, 
uint8_t upperLen)
#line 353
{
  return 0;
}

# 23 "/opt/tinyos-2.0.2/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$dataLinkLength(uart_id_t arg_0x40d8dc60, message_t *msg, uint8_t upperLen){
#line 23
  unsigned char result;
#line 23

#line 23
  switch (arg_0x40d8dc60) {
#line 23
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 23
      result = SerialPacketInfoActiveMessageP$Info$dataLinkLength(msg, upperLen);
#line 23
      break;
#line 23
    default:
#line 23
      result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$dataLinkLength(arg_0x40d8dc60, msg, upperLen);
#line 23
      break;
#line 23
    }
#line 23

#line 23
  return result;
#line 23
}
#line 23
# 40 "/opt/tinyos-2.0.2/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP$Info$offset(void )
#line 40
{
  return (uint8_t )(sizeof(message_header_t ) - sizeof(serial_header_t ));
}

# 349 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$offset(uart_id_t id)
#line 349
{
  return 0;
}

# 15 "/opt/tinyos-2.0.2/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$offset(uart_id_t arg_0x40d8dc60){
#line 15
  unsigned char result;
#line 15

#line 15
  switch (arg_0x40d8dc60) {
#line 15
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 15
      result = SerialPacketInfoActiveMessageP$Info$offset();
#line 15
      break;
#line 15
    default:
#line 15
      result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$offset(arg_0x40d8dc60);
#line 15
      break;
#line 15
    }
#line 15

#line 15
  return result;
#line 15
}
#line 15
# 100 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$send(uint8_t id, message_t *msg, uint8_t len)
#line 100
{
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendState != /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SEND_STATE_IDLE) {
      return EBUSY;
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 105
    {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendIndex = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$offset(id);
      if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendIndex > sizeof(message_header_t )) {
          {
            unsigned char __nesc_temp = 
#line 108
            ESIZE;

            {
#line 108
              __nesc_atomic_end(__nesc_atomic); 
#line 108
              return __nesc_temp;
            }
          }
        }
#line 111
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendError = SUCCESS;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendBuffer = (uint8_t *)msg;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SEND_STATE_DATA;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendId = id;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendCancelled = FALSE;






      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendLen = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$dataLinkLength(id, msg, len) + /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendIndex;
    }
#line 123
    __nesc_atomic_end(__nesc_atomic); }
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$startSend(id) == SUCCESS) {
      return SUCCESS;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SEND_STATE_IDLE;
      return FAIL;
    }
}

# 64 "/opt/tinyos-2.0.2/tos/interfaces/Send.nc"
inline static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubSend$send(message_t *msg, uint8_t len){
#line 64
  unsigned char result;
#line 64

#line 64
  result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$send(TOS_SERIAL_ACTIVE_MESSAGE_ID, msg, len);
#line 64

#line 64
  return result;
#line 64
}
#line 64
# 56 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP$RunTx$postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(SerialP$RunTx);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 269 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_hton_uint16(void * target, uint16_t value)
#line 269
{
  uint8_t *base = target;

#line 271
  base[1] = value;
  base[0] = value >> 8;
  return value;
}

#line 240
static __inline  uint8_t __nesc_hton_uint8(void * target, uint8_t value)
#line 240
{
  uint8_t *base = target;

#line 242
  base[0] = value;
  return value;
}

# 41 "secp160r1.nc"
static inline void secp160r1$CurveParam$get_param(Params *para)
{
#line 197
  para->p[9] = 0xFFFF;
  para->p[8] = 0xFFFF;
  para->p[7] = 0xFFFF;
  para->p[6] = 0xFFFF;
  para->p[5] = 0xFFFF;
  para->p[4] = 0xFFFF;
  para->p[3] = 0xFFFF;
  para->p[2] = 0xFFFF;
  para->p[1] = 0x7FFF;
  para->p[0] = 0xFFFF;

  para->omega[0] = 0x0001;
  para->omega[1] = 0x8000;



  para->E.a[9] = 0xFFFF;
  para->E.a[8] = 0xFFFF;
  para->E.a[7] = 0xFFFF;
  para->E.a[6] = 0xFFFF;
  para->E.a[5] = 0xFFFF;
  para->E.a[4] = 0xFFFF;
  para->E.a[3] = 0xFFFF;
  para->E.a[2] = 0xFFFF;
  para->E.a[1] = 0x7FFF;
  para->E.a[0] = 0xFFFC;

  para->E.a_minus3 = TRUE;
  para->E.a_zero = FALSE;


  para->E.b[9] = 0x1C97;
  para->E.b[8] = 0xBEFC;
  para->E.b[7] = 0x54BD;
  para->E.b[6] = 0x7A8B;
  para->E.b[5] = 0x65AC;
  para->E.b[4] = 0xF89F;
  para->E.b[3] = 0x81D4;
  para->E.b[2] = 0xD4AD;
  para->E.b[1] = 0xC565;
  para->E.b[0] = 0xFA45;


  para->G.x[9] = 0x4A96;
  para->G.x[8] = 0xB568;
  para->G.x[7] = 0x8EF5;
  para->G.x[6] = 0x7328;
  para->G.x[5] = 0x4664;
  para->G.x[4] = 0x6989;
  para->G.x[3] = 0x68C3;
  para->G.x[2] = 0x8BB9;
  para->G.x[1] = 0x13CB;
  para->G.x[0] = 0xFC82;

  para->G.y[9] = 0x23A6;
  para->G.y[8] = 0x2855;
  para->G.y[7] = 0x3168;
  para->G.y[6] = 0x947D;
  para->G.y[5] = 0x59DC;
  para->G.y[4] = 0xC912;
  para->G.y[3] = 0x0423;
  para->G.y[2] = 0x5137;
  para->G.y[1] = 0x7AC5;
  para->G.y[0] = 0xFB32;


  para->r[10] = 0x0001;
  para->r[9] = 0x0000;
  para->r[8] = 0x0000;
  para->r[7] = 0x0000;
  para->r[6] = 0x0000;
  para->r[5] = 0x0001;
  para->r[4] = 0xF4C8;
  para->r[3] = 0xF927;
  para->r[2] = 0xAED3;
  para->r[1] = 0xCA75;
  para->r[0] = 0x2257;
}

# 37 "CurveParam.nc"
inline static void ECCM$CurveParam$get_param(Params *para){
#line 37
  secp160r1$CurveParam$get_param(para);
#line 37
}
#line 37
# 75 "NN.nc"
inline static void ECCM$NN$ModBarrettInit(NN_DIGIT *c, NN_UINT cDigits, Barrett *pbuf){
#line 75
  NNM$NN$ModBarrettInit(c, cDigits, pbuf);
#line 75
}
#line 75
# 3267 "NNM.nc"
inline static NN_DIGIT NNM$NN_SubDigitMult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT c, NN_DIGIT *d, NN_UINT digits)
{
#line 3294
  NN_DIGIT borrow;
  unsigned int i;

  if (c == 0) {
    return 0;
    }
#line 3299
  borrow = 0;
  for (i = 0; i < digits; i++) {
       __asm volatile ("sub %1, %2 \n\t"
      "mov %2, %0 \n\t"
      "mov #0, %1 \n\t"
      "rlc %1 \n\t"
      "xor #0x0001, %1 \n\t"
      "mov %3, &0x130 \n\t"
      "mov %4, &0x138 \n\t"
      "sub &0x13A, %0 \n\t"
      "xor #0x0001, r2 \n\t"
      "addc &0x13C, %1 \n\t" : 
      "+r"(a[i]), "+r"(borrow) : 
      "r"(b[i]), "r"(c), "r"(d[i]));}


  return borrow;
}

#line 164
static inline void NNM$NN_DigitDiv(NN_DIGIT *a, NN_DIGIT b[2], NN_DIGIT c)
{
  NN_DOUBLE_DIGIT t;

  t = ((NN_DOUBLE_DIGIT )b[1] << 16) ^ (NN_DOUBLE_DIGIT )b[0];

  *a = t / c;
}

#line 2795
static inline void NNM$NN_Div(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT cDigits, NN_DIGIT *d, NN_UINT dDigits)
{
  NN_DIGIT ai;
#line 2797
  NN_DIGIT cc[2 * (160 / 16 + 1) + 2];
#line 2797
  NN_DIGIT dd[160 / 16 + 1 + 1];
#line 2797
  NN_DIGIT t;

  int i;
  int ddDigits;
#line 2800
  int shift;

  ddDigits = NNM$NN_Digits(d, dDigits);
  if (ddDigits == 0) {
    return;
    }


  shift = 16 - NNM$NN_DigitBits(d[ddDigits - 1]);
  NNM$NN_AssignZero(cc, ddDigits);
  cc[cDigits] = NNM$NN_LShift(cc, c, shift, cDigits);
  NNM$NN_LShift(dd, d, shift, ddDigits);
  t = dd[ddDigits - 1];

  if (a != (void *)0) {
    NNM$NN_AssignZero(a, cDigits);
    }
  for (i = cDigits - ddDigits; i >= 0; i--) {


      if (t == 0xffff) {
        ai = cc[i + ddDigits];
        }
      else {
#line 2823
        NNM$NN_DigitDiv(&ai, &cc[i + ddDigits - 1], t + 1);
        }
#line 2824
      cc[i + ddDigits] -= NNM$NN_SubDigitMult(&cc[i], &cc[i], ai, dd, ddDigits);



      while (cc[i + ddDigits] || NNM$NN_Cmp(&cc[i], dd, ddDigits) >= 0) {
          ai++;
          cc[i + ddDigits] -= NNM$NN_Sub(&cc[i], &cc[i], dd, ddDigits);
        }
      if (a != (void *)0) {
        a[i] = ai;
        }
    }

  NNM$NN_AssignZero(b, dDigits);
  NNM$NN_RShift(b, cc, shift, ddDigits);
}

#line 224
static inline void NNM$NN_Assign(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits)
{
  memcpy(a, b, digits * (16 / 8));
}

#line 3392
static inline void NNM$NN$Assign(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits)
{
  NNM$NN_Assign(a, b, digits);
}

# 47 "NN.nc"
inline static void ECCM$NN$Assign(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits){
#line 47
  NNM$NN$Assign(a, b, digits);
#line 47
}
#line 47
# 781 "ECCM.nc"
static inline void ECCM$ECC$win_precompute(Point *baseP, Point *pointArray)
{
  uint8_t i;

  ECCM$NN$Assign(pointArray[0].x, baseP->x, 160 / 16 + 1);
  ECCM$NN$Assign(pointArray[0].y, baseP->y, 160 / 16 + 1);

  for (i = 1; i < (1 << 4) - 1; i++) {



      ECCM$c_add(&pointArray[i], &pointArray[i - 1], baseP);
    }


  for (i = 0; i < 16 / 4; i++) 
    ECCM$mask[i] = 0xf << 4 * i;
}

# 3397 "NNM.nc"
static inline void NNM$NN$AssignDigit(NN_DIGIT *a, NN_DIGIT b, NN_UINT digits)
{
  NNM$NN_AssignZero(a, digits);
  a[0] = b;
}

# 49 "NN.nc"
inline static void ECCM$NN$AssignDigit(NN_DIGIT *a, NN_DIGIT b, NN_UINT digits){
#line 49
  NNM$NN$AssignDigit(a, b, digits);
#line 49
}
#line 49
# 39 "CurveParam.nc"
inline static NN_UINT NNM$CurveParam$omega_mul(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *omega, NN_UINT digits){
#line 39
  unsigned char result;
#line 39

#line 39
  result = secp160r1$CurveParam$omega_mul(a, b, omega, digits);
#line 39

#line 39
  return result;
#line 39
}
#line 39
# 47 "NN.nc"
inline static void secp160r1$NN$Assign(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits){
#line 47
  NNM$NN$Assign(a, b, digits);
#line 47
}
#line 47
# 3169 "NNM.nc"
inline static NN_DIGIT NNM$NN_AddDigitMult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT c, NN_DIGIT *d, NN_UINT digits)
{
#line 3195
  NN_DIGIT carry;
  unsigned int i;

#line 3197
  if (c == 0) {
    return 0;
    }
#line 3199
  carry = 0;
  for (i = 0; i < digits; i++) {
       __asm volatile ("add %1, %2 \n\t"
      "mov %2, %0 \n\t"
      "mov #0, %1 \n\t"
      "rlc %1 \n\t"
      "mov %3, &0x130 \n\t"
      "mov %4, &0x138 \n\t"
      "add &0x13a, %0 \n\t"
      "addc &0x13c, %1 \n\t" : 
      "+r"(a[i]), "+r"(carry) : 
      "r"(b[i]), "r"(c), "r"(d[i]));}


  return carry;
}

#line 3450
static inline NN_DIGIT NNM$NN$AddDigitMult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT c, NN_DIGIT *d, NN_UINT digits)
{
  return NNM$NN_AddDigitMult(a, b, c, d, digits);
}

# 69 "NN.nc"
inline static NN_DIGIT secp160r1$NN$AddDigitMult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT c, NN_DIGIT *d, NN_UINT digits){
#line 69
  unsigned int result;
#line 69

#line 69
  result = NNM$NN$AddDigitMult(a, b, c, d, digits);
#line 69

#line 69
  return result;
#line 69
}
#line 69
# 3731 "NNM.nc"
static inline void NNM$NN$ModInv(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits)
{



  NN_DIGIT y[160 / 16 + 1];

#line 3737
  {
#line 3737
    NNM$NN_AssignZero(y, digits);
#line 3737
    y[0] = 1;
  }
#line 3737
  ;
  NNM$NN_ModDivOpt(a, y, b, c, digits);
}

# 103 "NN.nc"
inline static void ECCM$NN$ModInv(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits){
#line 103
  NNM$NN$ModInv(a, b, c, digits);
#line 103
}
#line 103
# 3917 "NNM.nc"
static inline unsigned int NNM$NN$Digits(NN_DIGIT *a, NN_UINT digits)
{
  return NNM$NN_Digits(a, digits);
}

# 123 "NN.nc"
inline static unsigned int ECCM$NN$Digits(NN_DIGIT *a, NN_UINT digits){
#line 123
  unsigned int result;
#line 123

#line 123
  result = NNM$NN$Digits(a, digits);
#line 123

#line 123
  return result;
#line 123
}
#line 123
# 3445 "NNM.nc"
static inline NN_DIGIT NNM$NN$RShift(NN_DIGIT *a, NN_DIGIT *b, NN_UINT c, NN_UINT digits)
{
  return NNM$NN_RShift(a, b, c, digits);
}

# 67 "NN.nc"
inline static NN_DIGIT ECCM$NN$RShift(NN_DIGIT *a, NN_DIGIT *b, NN_UINT c, NN_UINT digits){
#line 67
  unsigned int result;
#line 67

#line 67
  result = NNM$NN$RShift(a, b, c, digits);
#line 67

#line 67
  return result;
#line 67
}
#line 67
# 3415 "NNM.nc"
static inline NN_DIGIT NNM$NN$Add(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits)
{
  return NNM$NN_Add(a, b, c, digits);
}

# 57 "NN.nc"
inline static NN_DIGIT ECCM$NN$Add(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits){
#line 57
  unsigned int result;
#line 57

#line 57
  result = NNM$NN$Add(a, b, c, digits);
#line 57

#line 57
  return result;
#line 57
}
#line 57
#line 85
inline static void ECCM$NN$ModSub(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits){
#line 85
  NNM$NN$ModSub(a, b, c, d, digits);
#line 85
}
#line 85








inline static void ECCM$NN$ModMultOpt(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_DIGIT *omega, NN_UINT digits){
#line 93
  NNM$NN$ModMultOpt(a, b, c, d, omega, digits);
#line 93
}
#line 93
# 3535 "NNM.nc"
static inline void NNM$NN$ModSmall(NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits)
{
  while (NNM$NN_Cmp(b, c, digits) > 0) 
    NNM$NN_Sub(b, b, c, digits);
}

# 81 "NN.nc"
inline static void ECCM$NN$ModSmall(NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits){
#line 81
  NNM$NN$ModSmall(b, c, digits);
#line 81
}
#line 81
# 3439 "NNM.nc"
static inline NN_DIGIT NNM$NN$LShift(NN_DIGIT *a, NN_DIGIT *b, NN_UINT c, NN_UINT digits)
{
  return NNM$NN_LShift(a, b, c, digits);
}

# 65 "NN.nc"
inline static NN_DIGIT ECCM$NN$LShift(NN_DIGIT *a, NN_DIGIT *b, NN_UINT c, NN_UINT digits){
#line 65
  unsigned int result;
#line 65

#line 65
  result = NNM$NN$LShift(a, b, c, digits);
#line 65

#line 65
  return result;
#line 65
}
#line 65
#line 99
inline static void ECCM$NN$ModSqrOpt(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *d, NN_DIGIT *omega, NN_UINT digits){
#line 99
  NNM$NN$ModSqrOpt(a, b, d, omega, digits);
#line 99
}
#line 99
#line 83
inline static void ECCM$NN$ModAdd(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits){
#line 83
  NNM$NN$ModAdd(a, b, c, d, digits);
#line 83
}
#line 83
# 3885 "NNM.nc"
static inline int NNM$NN$Zero(NN_DIGIT *a, NN_UINT digits)
{
  return NNM$NN_Zero(a, digits);
}

# 115 "NN.nc"
inline static int ECCM$NN$Zero(NN_DIGIT *a, NN_UINT digits){
#line 115
  int result;
#line 115

#line 115
  result = NNM$NN$Zero(a, digits);
#line 115

#line 115
  return result;
#line 115
}
#line 115
# 455 "ECCM.nc"
static inline void ECCM$c_m_dbl_projective(Point *P0, NN_DIGIT *Z0, uint8_t m)
#line 455
{
  uint8_t i;
  NN_DIGIT W[160 / 16 + 1];
  NN_DIGIT A[160 / 16 + 1];
  NN_DIGIT B[160 / 16 + 1];
  NN_DIGIT t1[160 / 16 + 1];
  NN_DIGIT y2[160 / 16 + 1];

  if (ECCM$NN$Zero(Z0, 160 / 16 + 1)) {
      return;
    }


  ECCM$NN$LShift(P0->y, P0->y, 1, 160 / 16 + 1);
  ECCM$NN$ModSmall(P0->y, ECCM$param.p, 160 / 16 + 1);

  ECCM$NN$ModSqrOpt(W, Z0, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
  ECCM$NN$ModSqrOpt(W, W, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);

  for (i = 0; i < m; i++) {
      if (ECCM$param.E.a_minus3) {

          ECCM$NN$ModSqrOpt(A, P0->x, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
          ECCM$NN$ModSub(A, A, W, ECCM$param.p, 160 / 16 + 1);
          ECCM$NN$LShift(t1, A, 1, 160 / 16 + 1);
          ECCM$NN$ModSmall(t1, ECCM$param.p, 160 / 16 + 1);
          ECCM$NN$ModAdd(A, A, t1, ECCM$param.p, 160 / 16 + 1);
        }
      else {
#line 482
        if (ECCM$param.E.a_zero) {

            ECCM$NN$ModSqrOpt(t1, P0->x, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
            ECCM$NN$LShift(A, t1, 1, 160 / 16 + 1);
            ECCM$NN$ModSmall(A, ECCM$param.p, 160 / 16 + 1);
            ECCM$NN$ModAdd(A, A, t1, ECCM$param.p, 160 / 16 + 1);
          }
        else 
#line 488
          {

            ECCM$NN$ModSqrOpt(t1, P0->x, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
            ECCM$NN$LShift(A, t1, 1, 160 / 16 + 1);
            ECCM$NN$ModSmall(A, ECCM$param.p, 160 / 16 + 1);
            ECCM$NN$ModAdd(A, A, t1, ECCM$param.p, 160 / 16 + 1);
            ECCM$NN$ModMultOpt(t1, ECCM$param.E.a, W, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
            ECCM$NN$ModAdd(A, A, t1, ECCM$param.p, 160 / 16 + 1);
          }
        }
      ECCM$NN$ModSqrOpt(y2, P0->y, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
      ECCM$NN$ModMultOpt(B, P0->x, y2, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);

      ECCM$NN$ModSqrOpt(P0->x, A, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
      ECCM$NN$LShift(t1, B, 1, 160 / 16 + 1);
      ECCM$NN$ModSmall(t1, ECCM$param.p, 160 / 16 + 1);
      ECCM$NN$ModSub(P0->x, P0->x, t1, ECCM$param.p, 160 / 16 + 1);

      ECCM$NN$ModMultOpt(Z0, Z0, P0->y, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
      ECCM$NN$ModSqrOpt(y2, y2, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
      if (i < m - 1) {

          ECCM$NN$ModMultOpt(W, W, y2, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
        }

      ECCM$NN$LShift(A, A, 1, 160 / 16 + 1);
      ECCM$NN$ModSmall(A, ECCM$param.p, 160 / 16 + 1);
      ECCM$NN$ModSub(B, B, P0->x, ECCM$param.p, 160 / 16 + 1);
      ECCM$NN$ModMultOpt(A, A, B, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
      ECCM$NN$ModSub(P0->y, A, y2, ECCM$param.p, 160 / 16 + 1);
    }
  if (P0->y[0] % 2 == 1) {
    ECCM$NN$Add(P0->y, P0->y, ECCM$param.p, 160 / 16 + 1);
    }
#line 521
  ECCM$NN$RShift(P0->y, P0->y, 1, 160 / 16 + 1);
}

# 38 "ECC.nc"
inline static void ECDSAM$ECC$init(void ){
#line 38
  ECCM$ECC$init();
#line 38
}
#line 38
# 977 "ECCM.nc"
static inline Params *ECCM$ECC$get_param(void )
#line 977
{
  return &ECCM$param;
}

# 100 "ECC.nc"
inline static Params *ECDSAM$ECC$get_param(void ){
#line 100
  struct Params *result;
#line 100

#line 100
  result = ECCM$ECC$get_param();
#line 100

#line 100
  return result;
#line 100
}
#line 100
# 699 "ECCM.nc"
static inline void ECCM$ECC$add(Point *P0, Point *P1, Point *P2)
{



  ECCM$c_add(P0, P1, P2);
}

# 58 "ECC.nc"
inline static void ECDSAM$ECC$add(Point *P0, Point *P1, Point *P2){
#line 58
  ECCM$ECC$add(P0, P1, P2);
#line 58
}
#line 58
# 47 "NN.nc"
inline static void ECDSAM$NN$Assign(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits){
#line 47
  NNM$NN$Assign(a, b, digits);
#line 47
}
#line 47
# 150 "ECDSAM.nc"
static inline void ECDSAM$shamir_init(Point *pKey, Point *pointArray)
{
  uint16_t i;
  uint16_t j;







  ECDSAM$NN$Assign(pointArray[0].x, pKey->x, 160 / 16 + 1);
  ECDSAM$NN$Assign(pointArray[0].y, pKey->y, 160 / 16 + 1);

  for (i = 1; i < (1 << 2) - 1; i++) {
      ECDSAM$ECC$add(&pointArray[i], &pointArray[i - 1], pKey);
    }
#line 179
  for (i = 0; i < (1 << 2) - 1; i++) {
      for (j = 0; j < (1 << 2) - 1 + 1; j++) {
          if (i == 0 && j == 0) {
              ECDSAM$NN$Assign(pointArray[(1 << 2) - 1].x, ECDSAM$param->G.x, 160 / 16 + 1);
              ECDSAM$NN$Assign(pointArray[(1 << 2) - 1].y, ECDSAM$param->G.y, 160 / 16 + 1);
            }
          else {
#line 184
            if (j == 0) {
                ECDSAM$ECC$add(&pointArray[(1 << 2) - 1 + ((1 << 2) - 1 + 1) * i], &pointArray[(1 << 2) - 1 + ((1 << 2) - 1 + 1) * (i - 1)], & ECDSAM$param->G);
              }
            else 
#line 186
              {
                ECDSAM$ECC$add(&pointArray[(1 << 2) - 1 + ((1 << 2) - 1 + 1) * i + j], &pointArray[(1 << 2) - 1 + ((1 << 2) - 1 + 1) * i], &pointArray[j - 1]);
              }
            }
        }
    }
  for (i = 0; i < 16 / 2; i++) 
    ECDSAM$s_mask[i] = 0x03 << 2 * i;
}

# 75 "NN.nc"
inline static void ECDSAM$NN$ModBarrettInit(NN_DIGIT *c, NN_UINT cDigits, Barrett *pbuf){
#line 75
  NNM$NN$ModBarrettInit(c, cDigits, pbuf);
#line 75
}
#line 75
# 3495 "NNM.nc"
static inline void NNM$NN$BarrettSetBuf(Barrett *pbuf)
#line 3495
{
  NNM$pBarrett = pbuf;
}

# 77 "NN.nc"
inline static void ECDSAM$NN$BarrettSetBuf(Barrett *pbuf){
#line 77
  NNM$NN$BarrettSetBuf(pbuf);
#line 77
}
#line 77
# 40 "ECC.nc"
inline static void ECDSAM$ECC$gen_private_key(NN_DIGIT *PrivateKey){
#line 40
  ECCM$ECC$gen_private_key(PrivateKey);
#line 40
}
#line 40
# 3905 "NNM.nc"
static inline unsigned int NNM$NN$Bits(NN_DIGIT *a, NN_UINT digits)
{
  return NNM$NN_Bits(a, digits);
}

# 119 "NN.nc"
inline static unsigned int ECCM$NN$Bits(NN_DIGIT *a, NN_UINT digits){
#line 119
  unsigned int result;
#line 119

#line 119
  result = NNM$NN$Bits(a, digits);
#line 119

#line 119
  return result;
#line 119
}
#line 119
# 41 "/opt/tinyos-2.0.2/tos/interfaces/Random.nc"
inline static uint16_t ECCM$Random$rand16(void ){
#line 41
  unsigned int result;
#line 41

#line 41
  result = RandomLfsrP$Random$rand16();
#line 41

#line 41
  return result;
#line 41
}
#line 41
# 115 "NN.nc"
inline static int ECDSAM$NN$Zero(NN_DIGIT *a, NN_UINT digits){
#line 115
  int result;
#line 115

#line 115
  result = NNM$NN$Zero(a, digits);
#line 115

#line 115
  return result;
#line 115
}
#line 115
#line 77
inline static void ECCM$NN$BarrettSetBuf(Barrett *pbuf){
#line 77
  NNM$NN$BarrettSetBuf(pbuf);
#line 77
}
#line 77
# 607 "ECCM.nc"
static inline void ECCM$ECC$BarrettSetBuf(void )
#line 607
{
  ECCM$NN$BarrettSetBuf(&ECCM$Bbuf);
}

# 54 "ECC.nc"
inline static void ECDSAM$ECC$BarrettSetBuf(void ){
#line 54
  ECCM$ECC$BarrettSetBuf();
#line 54
}
#line 54
# 898 "ECCM.nc"
static inline void ECCM$ECC$win_mul_base(Point *P0, NN_DIGIT *n)
{
  ECCM$win_mul(P0, n, ECCM$pBaseArray);
}

# 88 "ECC.nc"
inline static void ECDSAM$ECC$win_mul_base(Point *P0, NN_DIGIT *n){
#line 88
  ECCM$ECC$win_mul_base(P0, n);
#line 88
}
#line 88
# 81 "NN.nc"
inline static void ECDSAM$NN$ModSmall(NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits){
#line 81
  NNM$NN$ModSmall(b, c, digits);
#line 81
}
#line 81
# 37 "SHA1.nc"
inline static int ECDSAM$SHA1$reset(SHA1Context *context){
#line 37
  int result;
#line 37

#line 37
  result = SHA1M$SHA1$reset(context);
#line 37

#line 37
  return result;
#line 37
}
#line 37

inline static int ECDSAM$SHA1$update(SHA1Context *context, const uint8_t *message_array, uint32_t length){
#line 38
  int result;
#line 38

#line 38
  result = SHA1M$SHA1$update(context, message_array, length);
#line 38

#line 38
  return result;
#line 38
}
#line 38

inline static int ECDSAM$SHA1$digest(SHA1Context *context, uint8_t Message_Digest[20]){
#line 39
  int result;
#line 39

#line 39
  result = SHA1M$SHA1$digest(context, Message_Digest);
#line 39

#line 39
  return result;
#line 39
}
#line 39
# 386 "SHA1M.nc"
static inline void SHA1M$SHA1PadMessage(SHA1Context *context)
{






  if (context->Message_Block_Index > 55) {
      context->Message_Block[context->Message_Block_Index++] = 0x80;
      while (context->Message_Block_Index < 64) {
          context->Message_Block[context->Message_Block_Index++] = 0;
        }

      SHA1M$SHA1ProcessMessageBlock(context);

      while (context->Message_Block_Index < 56) {
          context->Message_Block[context->Message_Block_Index++] = 0;
        }
    }
  else 
#line 405
    {
      context->Message_Block[context->Message_Block_Index++] = 0x80;
      while (context->Message_Block_Index < 56) {
          context->Message_Block[context->Message_Block_Index++] = 0;
        }
    }




  context->Message_Block[56] = context->Length_High >> 24;
  context->Message_Block[57] = context->Length_High >> 16;
  context->Message_Block[58] = context->Length_High >> 8;
  context->Message_Block[59] = context->Length_High;
  context->Message_Block[60] = context->Length_Low >> 24;
  context->Message_Block[61] = context->Length_Low >> 16;
  context->Message_Block[62] = context->Length_Low >> 8;
  context->Message_Block[63] = context->Length_Low;

  SHA1M$SHA1ProcessMessageBlock(context);
}

# 3376 "NNM.nc"
static inline void NNM$NN$Decode(NN_DIGIT *a, NN_UINT digits, unsigned char *b, NN_UINT len)
{
  NNM$NN_Decode(a, digits, b, len);
}

# 41 "NN.nc"
inline static void ECDSAM$NN$Decode(NN_DIGIT *a, NN_UINT digits, unsigned char *b, NN_UINT len){
#line 41
  NNM$NN$Decode(a, digits, b, len);
#line 41
}
#line 41
#line 119
inline static unsigned int ECDSAM$NN$Bits(NN_DIGIT *a, NN_UINT digits){
#line 119
  unsigned int result;
#line 119

#line 119
  result = NNM$NN$Bits(a, digits);
#line 119

#line 119
  return result;
#line 119
}
#line 119
# 2846 "NNM.nc"
static inline void NNM$NN_Mod(NN_DIGIT *a, NN_DIGIT *b, NN_UINT bDigits, NN_DIGIT *c, NN_UINT cDigits)
{

  NNM$NN$ModBarrett(a, b, bDigits, c, cDigits);
}

#line 3457
static inline void NNM$NN$Mod(NN_DIGIT *a, NN_DIGIT *b, NN_UINT bDigits, NN_DIGIT *c, NN_UINT cDigits)
{
  NNM$NN_Mod(a, b, bDigits, c, cDigits);
}

# 73 "NN.nc"
inline static void ECDSAM$NN$Mod(NN_DIGIT *a, NN_DIGIT *b, NN_UINT bDigits, NN_DIGIT *c, NN_UINT cDigits){
#line 73
  NNM$NN$Mod(a, b, bDigits, c, cDigits);
#line 73
}
#line 73
# 3581 "NNM.nc"
static inline void NNM$NN$ModMult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits)
{
  NNM$NN_ModMult(a, b, c, d, digits);
}

# 89 "NN.nc"
inline static void ECDSAM$NN$ModMult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits){
#line 89
  NNM$NN$ModMult(a, b, c, d, digits);
#line 89
}
#line 89
#line 83
inline static void ECDSAM$NN$ModAdd(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits){
#line 83
  NNM$NN$ModAdd(a, b, c, d, digits);
#line 83
}
#line 83
# 3879 "NNM.nc"
static inline int NNM$NN$Cmp(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits)
{
  return NNM$NN_Cmp(a, b, digits);
}

# 113 "NN.nc"
inline static int ECDSAM$NN$Cmp(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits){
#line 113
  int result;
#line 113

#line 113
  result = NNM$NN$Cmp(a, b, digits);
#line 113

#line 113
  return result;
#line 113
}
#line 113
#line 93
inline static void ECDSAM$NN$ModMultOpt(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_DIGIT *omega, NN_UINT digits){
#line 93
  NNM$NN$ModMultOpt(a, b, c, d, omega, digits);
#line 93
}
#line 93










inline static void ECDSAM$NN$ModInv(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits){
#line 103
  NNM$NN$ModInv(a, b, c, digits);
#line 103
}
#line 103
# 3891 "NNM.nc"
static inline int NNM$NN$One(NN_DIGIT *a, NN_UINT digits)
{
  uint8_t i;

  for (i = 1; i < digits; i++) 
    if (a[i]) {
      return FALSE;
      }
#line 3898
  if (a[0] == 1) {
    return TRUE;
    }
  return FALSE;
}

# 117 "NN.nc"
inline static int ECDSAM$NN$One(NN_DIGIT *a, NN_UINT digits){
#line 117
  int result;
#line 117

#line 117
  result = NNM$NN$One(a, digits);
#line 117

#line 117
  return result;
#line 117
}
#line 117
# 3403 "NNM.nc"
static inline void NNM$NN$AssignZero(NN_DIGIT *a, NN_UINT digits)
{
  NNM$NN_AssignZero(a, digits);
}

# 51 "NN.nc"
inline static void ECCM$NN$AssignZero(NN_DIGIT *a, NN_UINT digits){
#line 51
  NNM$NN$AssignZero(a, digits);
#line 51
}
#line 51
# 3929 "NNM.nc"
static inline int NNM$NN$Equal(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits)
{
  return !NNM$NN_Cmp(a, b, digits);
}

# 125 "NN.nc"
inline static int ECCM$NN$Equal(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits){
#line 125
  int result;
#line 125

#line 125
  result = NNM$NN$Equal(a, b, digits);
#line 125

#line 125
  return result;
#line 125
}
#line 125
# 94 "ECCM.nc"
static inline bool ECCM$p_equal(Point *P1, Point *P2)
{
  if (ECCM$NN$Equal(P1->x, P2->x, 160 / 16 + 1)) {
    if (ECCM$NN$Equal(P1->y, P2->y, 160 / 16 + 1)) {
      return TRUE;
      }
    }
#line 99
  return FALSE;
}

#line 226
static inline void ECCM$c_add_projective(Point *P0, NN_DIGIT *Z0, Point *P1, NN_DIGIT *Z1, Point *P2, NN_DIGIT *Z2)
{
  NN_DIGIT n0[160 / 16 + 1];
  NN_DIGIT n1[160 / 16 + 1];
  NN_DIGIT n2[160 / 16 + 1];
  NN_DIGIT n3[160 / 16 + 1];
  NN_DIGIT n4[160 / 16 + 1];
  NN_DIGIT n5[160 / 16 + 1];
  NN_DIGIT n6[160 / 16 + 1];

  if (ECCM$NN$Zero(Z1, 160 / 16 + 1)) 
    {
      ECCM$p_copy(P0, P2);
      ECCM$NN$Assign(Z0, Z2, 160 / 16 + 1);
      return;
    }

  if (ECCM$NN$Zero(Z2, 160 / 16 + 1)) 
    {
      ECCM$p_copy(P0, P1);
      ECCM$NN$Assign(Z0, Z1, 160 / 16 + 1);
      return;
    }


  if (ECCM$p_equal(P1, P2)) 
    {
      ECCM$c_dbl_projective(P0, Z0, P1, Z1);
      return;
    }



  if (ECCM$Z_is_one(Z2)) 
    {

      ECCM$NN$Assign(n1, P1->x, 160 / 16 + 1);
      ECCM$NN$Assign(n2, P1->y, 160 / 16 + 1);
    }
  else 
    {

      ECCM$NN$ModSqrOpt(n0, Z2, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
      ECCM$NN$ModMultOpt(n1, P1->x, n0, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);

      ECCM$NN$ModMultOpt(n0, n0, Z2, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
      ECCM$NN$ModMultOpt(n2, P1->y, n0, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
    }


  if (ECCM$Z_is_one(Z1)) 
    {

      ECCM$NN$Assign(n3, P2->x, 160 / 16 + 1);
      ECCM$NN$Assign(n4, P2->y, 160 / 16 + 1);
    }
  else 
    {

      ECCM$NN$ModSqrOpt(n0, Z1, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
      ECCM$NN$ModMultOpt(n3, P2->x, n0, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);

      ECCM$NN$ModMultOpt(n0, n0, Z1, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
      ECCM$NN$ModMultOpt(n4, P2->y, n0, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
    }


  ECCM$NN$ModSub(n5, n1, n3, ECCM$param.p, 160 / 16 + 1);
  ECCM$NN$ModSub(n6, n2, n4, ECCM$param.p, 160 / 16 + 1);

  if (ECCM$NN$Zero(n5, 160 / 16 + 1)) {
    if (ECCM$NN$Zero(n6, 160 / 16 + 1)) 
      {

        ECCM$c_dbl_projective(P0, Z0, P1, Z1);
        return;
      }
    else 
      {

        ECCM$NN$AssignZero(Z0, 160 / 16 + 1);
        return;
      }
    }

  ECCM$NN$ModAdd(n1, n1, n3, ECCM$param.p, 160 / 16 + 1);
  ECCM$NN$ModAdd(n2, n2, n4, ECCM$param.p, 160 / 16 + 1);


  if (ECCM$Z_is_one(Z1) && ECCM$Z_is_one(Z2)) 
    {
      ECCM$NN$Assign(Z0, n5, 160 / 16 + 1);
    }
  else 
    {
      if (ECCM$Z_is_one(Z1)) {
        ECCM$NN$Assign(n0, Z2, 160 / 16 + 1);
        }
      else {
#line 323
        if (ECCM$Z_is_one(Z2)) {
          ECCM$NN$Assign(n0, Z1, 160 / 16 + 1);
          }
        else {
#line 326
          ECCM$NN$ModMultOpt(n0, Z1, Z2, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
          }
        }
#line 328
      ECCM$NN$ModMultOpt(Z0, n0, n5, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
    }


  ECCM$NN$ModSqrOpt(n0, n6, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
  ECCM$NN$ModSqrOpt(n4, n5, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
  ECCM$NN$ModMultOpt(n3, n1, n4, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
  ECCM$NN$ModSub(P0->x, n0, n3, ECCM$param.p, 160 / 16 + 1);


  ECCM$NN$LShift(n0, P0->x, 1, 160 / 16 + 1);
  ECCM$NN$ModSmall(n0, ECCM$param.p, 160 / 16 + 1);
  ECCM$NN$ModSub(n0, n3, n0, ECCM$param.p, 160 / 16 + 1);


  ECCM$NN$ModMultOpt(n0, n0, n6, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
  ECCM$NN$ModMultOpt(n5, n4, n5, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
  ECCM$NN$ModMultOpt(n1, n2, n5, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
  ECCM$NN$ModSub(n0, n0, n1, ECCM$param.p, 160 / 16 + 1);

  if (n0[0] % 2 == 1) {
    ECCM$NN$Add(n0, n0, ECCM$param.p, 160 / 16 + 1);
    }
  ECCM$NN$RShift(P0->y, n0, 1, 160 / 16 + 1);
}

#line 982
static inline void ECCM$ECC$add_proj(Point *P0, NN_DIGIT *Z0, Point *P1, NN_DIGIT *Z1, Point *P2, NN_DIGIT *Z2)
#line 982
{
  return ECCM$c_add_projective(P0, Z0, P1, Z1, P2, Z2);
}

# 69 "ECC.nc"
inline static void ECDSAM$ECC$add_proj(Point *P0, NN_DIGIT *Z0, Point *P1, NN_DIGIT *Z1, Point *P2, NN_DIGIT *Z2){
#line 69
  ECCM$ECC$add_proj(P0, Z0, P1, Z1, P2, Z2);
#line 69
}
#line 69
# 986 "ECCM.nc"
static inline void ECCM$ECC$dbl_proj(Point *P0, NN_DIGIT *Z0, Point *P1, NN_DIGIT *Z1)
#line 986
{
  return ECCM$c_dbl_projective(P0, Z0, P1, Z1);
}

# 72 "ECC.nc"
inline static void ECDSAM$ECC$dbl_proj(Point *P0, NN_DIGIT *Z0, Point *P1, NN_DIGIT *Z1){
#line 72
  ECCM$ECC$dbl_proj(P0, Z0, P1, Z1);
#line 72
}
#line 72
# 123 "NN.nc"
inline static unsigned int ECDSAM$NN$Digits(NN_DIGIT *a, NN_UINT digits){
#line 123
  unsigned int result;
#line 123

#line 123
  result = NNM$NN$Digits(a, digits);
#line 123

#line 123
  return result;
#line 123
}
#line 123
#line 51
inline static void ECDSAM$NN$AssignZero(NN_DIGIT *a, NN_UINT digits){
#line 51
  NNM$NN$AssignZero(a, digits);
#line 51
}
#line 51
# 197 "ECDSAM.nc"
static inline void ECDSAM$shamir(Point *P0, NN_DIGIT *u1, NN_DIGIT *u2)
#line 197
{
#line 229
  int16_t i;
#line 229
  int16_t tmp;
#line 229
  int16_t tmp2;
  int8_t j;
#line 230
  int8_t k;
  NN_DIGIT windex;
  NN_DIGIT Z0[160 / 16 + 1];
  NN_DIGIT Z1[160 / 16 + 1];


  ECDSAM$NN$AssignZero(P0->x, 160 / 16 + 1);
  ECDSAM$NN$AssignZero(P0->y, 160 / 16 + 1);


  ECDSAM$NN$AssignZero(Z0, 160 / 16 + 1);
  ECDSAM$NN$AssignZero(Z1, 160 / 16 + 1);
  Z1[0] = 0x01;

  tmp = ECDSAM$NN$Digits(u1, 160 / 16 + 1);
  tmp2 = ECDSAM$NN$Digits(u2, 160 / 16 + 1);
  if (tmp2 > tmp) {
    tmp = tmp2;
    }
  for (i = tmp - 1; i >= 0; i--) 
    {

      for (j = 16 / 2 - 1; j >= 0; j--) 
        {

          for (k = 0; k < 2; k++) 
            ECDSAM$ECC$dbl_proj(P0, Z0, P0, Z0);

          if (j != 0) {
              windex = ((ECDSAM$s_mask[j] & u1[i]) >> (j - 1) * 2) | ((ECDSAM$s_mask[j] & u2[i]) >> j * 2);
            }
          else 
#line 260
            {
              windex = ((ECDSAM$s_mask[j] & u1[i]) << 2) | (ECDSAM$s_mask[j] & u2[i]);
            }
          if (windex) {
              ECDSAM$ECC$add_proj(P0, Z0, P0, Z0, &ECDSAM$pqBaseArray[windex - 1], Z1);
            }
        }
    }




  if (ECDSAM$NN$One(Z0, 160 / 16 + 1) == FALSE) 
    {
      ECDSAM$NN$ModInv(Z1, Z0, ECDSAM$param->p, 160 / 16 + 1);
      ECDSAM$NN$ModMultOpt(Z0, Z1, Z1, ECDSAM$param->p, ECDSAM$param->omega, 160 / 16 + 1);
      ECDSAM$NN$ModMultOpt(P0->x, P0->x, Z0, ECDSAM$param->p, ECDSAM$param->omega, 160 / 16 + 1);
      ECDSAM$NN$ModMultOpt(Z0, Z0, Z1, ECDSAM$param->p, ECDSAM$param->omega, 160 / 16 + 1);
      ECDSAM$NN$ModMultOpt(P0->y, P0->y, Z0, ECDSAM$param->p, ECDSAM$param->omega, 160 / 16 + 1);
    }
}

# 41 "/opt/tinyos-2.0.2/tos/interfaces/Random.nc"
inline static uint16_t testECDSAM$Random$rand16(void ){
#line 41
  unsigned int result;
#line 41

#line 41
  result = RandomLfsrP$Random$rand16();
#line 41

#line 41
  return result;
#line 41
}
#line 41
# 69 "/opt/tinyos-2.0.2/tos/interfaces/AMSend.nc"
inline static error_t testECDSAM$PriKeyMsg$send(am_addr_t addr, message_t *msg, uint8_t len){
#line 69
  unsigned char result;
#line 69

#line 69
  result = /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMSend$send(AM_PRIVATE_KEY_MSG, addr, msg, len);
#line 69

#line 69
  return result;
#line 69
}
#line 69
# 40 "ECC.nc"
inline static void testECDSAM$ECC$gen_private_key(NN_DIGIT *PrivateKey){
#line 40
  ECCM$ECC$gen_private_key(PrivateKey);
#line 40
}
#line 40
# 85 "testECDSAM.nc"
static inline void testECDSAM$gen_PrivateKey(void )
#line 85
{
  private_key_msg *pPrivateKey;



  testECDSAM$ECC$gen_private_key(testECDSAM$PrivateKey);
#line 144
  pPrivateKey = (private_key_msg *)testECDSAM$report.data;
  pPrivateKey->len = 160 / 16 * (16 / 8);
  testECDSAM$NN$Encode(pPrivateKey->d, 160 / 16 * (16 / 8), testECDSAM$PrivateKey, 160 / 16);
  testECDSAM$PriKeyMsg$send(1, &testECDSAM$report, sizeof(private_key_msg ));
}

# 235 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_uint8(const void * source)
#line 235
{
  const uint8_t *base = source;

#line 237
  return base[0];
}

# 206 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$unlockBuffer(uint8_t which)
#line 206
{
  if (which) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.bufOneLocked = 0;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.bufZeroLocked = 0;
    }
}

# 89 "/opt/tinyos-2.0.2/tos/lib/serial/SerialActiveMessageP.nc"
static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Receive$default$receive(uint8_t id, message_t *msg, void *payload, uint8_t len)
#line 89
{
  return msg;
}

# 67 "/opt/tinyos-2.0.2/tos/interfaces/Receive.nc"
inline static message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Receive$receive(am_id_t arg_0x40cc8298, message_t *msg, void *payload, uint8_t len){
#line 67
  nx_struct message_t *result;
#line 67

#line 67
    result = /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Receive$default$receive(arg_0x40cc8298, msg, payload, len);
#line 67

#line 67
  return result;
#line 67
}
#line 67
# 102 "/opt/tinyos-2.0.2/tos/lib/serial/SerialActiveMessageP.nc"
static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubReceive$receive(message_t *msg, void *payload, uint8_t len)
#line 102
{
  return /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Receive$receive(/*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$type(msg), msg, msg->data, len);
}

# 362 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
static inline message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Receive$default$receive(uart_id_t idxxx, message_t *msg, 
void *payload, 
uint8_t len)
#line 364
{
  return msg;
}

# 67 "/opt/tinyos-2.0.2/tos/interfaces/Receive.nc"
inline static message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Receive$receive(uart_id_t arg_0x40d90918, message_t *msg, void *payload, uint8_t len){
#line 67
  nx_struct message_t *result;
#line 67

#line 67
  switch (arg_0x40d90918) {
#line 67
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 67
      result = /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubReceive$receive(msg, payload, len);
#line 67
      break;
#line 67
    default:
#line 67
      result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Receive$default$receive(arg_0x40d90918, msg, payload, len);
#line 67
      break;
#line 67
    }
#line 67

#line 67
  return result;
#line 67
}
#line 67
# 46 "/opt/tinyos-2.0.2/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP$Info$upperLength(message_t *msg, uint8_t dataLinkLen)
#line 46
{
  return dataLinkLen - sizeof(serial_header_t );
}

# 356 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$upperLength(uart_id_t id, message_t *msg, 
uint8_t dataLinkLen)
#line 357
{
  return 0;
}

# 31 "/opt/tinyos-2.0.2/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$upperLength(uart_id_t arg_0x40d8dc60, message_t *msg, uint8_t dataLinkLen){
#line 31
  unsigned char result;
#line 31

#line 31
  switch (arg_0x40d8dc60) {
#line 31
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 31
      result = SerialPacketInfoActiveMessageP$Info$upperLength(msg, dataLinkLen);
#line 31
      break;
#line 31
    default:
#line 31
      result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$upperLength(arg_0x40d8dc60, msg, dataLinkLen);
#line 31
      break;
#line 31
    }
#line 31

#line 31
  return result;
#line 31
}
#line 31
# 269 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTask$runTask(void )
#line 269
{
  uart_id_t myType;
  message_t *myBuf;
  uint8_t mySize;
  uint8_t myWhich;

#line 274
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 274
    {
      myType = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskType;
      myBuf = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskBuf;
      mySize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskSize;
      myWhich = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskWhich;
    }
#line 279
    __nesc_atomic_end(__nesc_atomic); }
  mySize -= /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$offset(myType);
  mySize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$upperLength(myType, myBuf, mySize);
  myBuf = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Receive$receive(myType, myBuf, myBuf, mySize);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 283
    {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$messagePtrs[myWhich] = myBuf;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$unlockBuffer(myWhich);
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskPending = FALSE;
    }
#line 287
    __nesc_atomic_end(__nesc_atomic); }
}

# 270 "testECDSAM.nc"
static inline void testECDSAM$SerialControl$stopDone(error_t e)
#line 270
{
}

# 117 "/opt/tinyos-2.0.2/tos/interfaces/SplitControl.nc"
inline static void SerialP$SplitControl$stopDone(error_t error){
#line 117
  testECDSAM$SerialControl$stopDone(error);
#line 117
}
#line 117
# 97 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP$0$Usart$resetUsart(bool reset){
#line 97
  HplMsp430Usart1P$Usart$resetUsart(reset);
#line 97
}
#line 97
#line 128
inline static void /*Msp430Uart1P.UartP*/Msp430UartP$0$Usart$disableUart(void ){
#line 128
  HplMsp430Usart1P$Usart$disableUart();
#line 128
}
#line 128
#line 179
inline static void /*Msp430Uart1P.UartP*/Msp430UartP$0$Usart$disableIntr(void ){
#line 179
  HplMsp430Usart1P$Usart$disableIntr();
#line 179
}
#line 179
# 89 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP$0$ResourceConfigure$unconfigure(uint8_t id)
#line 89
{
  /*Msp430Uart1P.UartP*/Msp430UartP$0$Usart$resetUsart(TRUE);
  /*Msp430Uart1P.UartP*/Msp430UartP$0$Usart$disableIntr();
  /*Msp430Uart1P.UartP*/Msp430UartP$0$Usart$disableUart();
  /*Msp430Uart1P.UartP*/Msp430UartP$0$Usart$resetUsart(FALSE);
}

# 201 "/opt/tinyos-2.0.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceConfigure$default$unconfigure(uint8_t id)
#line 201
{
}

# 55 "/opt/tinyos-2.0.2/tos/interfaces/ResourceConfigure.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceConfigure$unconfigure(uint8_t arg_0x40ef8df0){
#line 55
  switch (arg_0x40ef8df0) {
#line 55
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C$0$CLIENT_ID:
#line 55
      /*Msp430Uart1P.UartP*/Msp430UartP$0$ResourceConfigure$unconfigure(/*PlatformSerialC.UartC*/Msp430Uart1C$0$CLIENT_ID);
#line 55
      break;
#line 55
    default:
#line 55
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceConfigure$default$unconfigure(arg_0x40ef8df0);
#line 55
      break;
#line 55
    }
#line 55
}
#line 55
# 109 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline error_t HplMsp430Usart1P$AsyncStdControl$stop(void )
#line 109
{
  HplMsp430Usart1P$Usart$disableSpi();
  HplMsp430Usart1P$Usart$disableUart();
  return SUCCESS;
}

# 84 "/opt/tinyos-2.0.2/tos/interfaces/AsyncStdControl.nc"
inline static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$AsyncStdControl$stop(void ){
#line 84
  unsigned char result;
#line 84

#line 84
  result = HplMsp430Usart1P$AsyncStdControl$stop();
#line 84

#line 84
  return result;
#line 84
}
#line 84
# 74 "/opt/tinyos-2.0.2/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$PowerDownCleanup$default$cleanup(void )
#line 74
{
}

# 52 "/opt/tinyos-2.0.2/tos/lib/power/PowerDownCleanup.nc"
inline static void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$PowerDownCleanup$cleanup(void ){
#line 52
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$PowerDownCleanup$default$cleanup();
#line 52
}
#line 52
# 69 "/opt/tinyos-2.0.2/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$ResourceDefaultOwner$granted(void )
#line 69
{
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$PowerDownCleanup$cleanup();
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$AsyncStdControl$stop();
}

# 46 "/opt/tinyos-2.0.2/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceDefaultOwner$granted(void ){
#line 46
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$ResourceDefaultOwner$granted();
#line 46
}
#line 46
# 56 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$grantedTask$postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$grantedTask);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 58 "/opt/tinyos-2.0.2/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$FcfsQueue$dequeue(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    if (/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$qHead != /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$NO_ENTRY) {
        uint8_t id = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$qHead;

#line 62
        /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$qHead = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$resQ[/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$qHead];
        if (/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$qHead == /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$NO_ENTRY) {
          /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$qTail = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$NO_ENTRY;
          }
#line 65
        /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$resQ[id] = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 66
          id;

#line 66
          return __nesc_temp;
        }
      }
#line 68
    {
      unsigned char __nesc_temp = 
#line 68
      /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$NO_ENTRY;

#line 68
      return __nesc_temp;
    }
  }
}

# 60 "/opt/tinyos-2.0.2/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$Queue$dequeue(void ){
#line 60
  unsigned char result;
#line 60

#line 60
  result = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$FcfsQueue$dequeue();
#line 60

#line 60
  return result;
#line 60
}
#line 60
# 50 "/opt/tinyos-2.0.2/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$FcfsQueue$isEmpty(void )
#line 50
{
  return /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$qHead == /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$NO_ENTRY;
}

# 43 "/opt/tinyos-2.0.2/tos/interfaces/ResourceQueue.nc"
inline static bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$Queue$isEmpty(void ){
#line 43
  unsigned char result;
#line 43

#line 43
  result = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$FcfsQueue$isEmpty();
#line 43

#line 43
  return result;
#line 43
}
#line 43
# 107 "/opt/tinyos-2.0.2/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$Resource$release(uint8_t id)
#line 107
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 108
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$RES_BUSY && /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$resId == id) {
          if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$Queue$isEmpty() == FALSE) {
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$reqResId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$Queue$dequeue();
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$RES_GRANTING;
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$grantedTask$postTask();
            }
          else {
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$default_owner_id;
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$RES_CONTROLLED;
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceDefaultOwner$granted();
            }
          /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceConfigure$unconfigure(id);
        }
    }
#line 122
    __nesc_atomic_end(__nesc_atomic); }
  return FAIL;
}

# 191 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartResource$default$release(uint8_t id)
#line 191
{
#line 191
  return FAIL;
}

# 110 "/opt/tinyos-2.0.2/tos/interfaces/Resource.nc"
inline static error_t /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartResource$release(uint8_t arg_0x40e24540){
#line 110
  unsigned char result;
#line 110

#line 110
  switch (arg_0x40e24540) {
#line 110
    case /*PlatformSerialC.UartC*/Msp430Uart1C$0$CLIENT_ID:
#line 110
      result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$Resource$release(/*PlatformSerialC.UartC.UsartC*/Msp430Usart1C$0$CLIENT_ID);
#line 110
      break;
#line 110
    default:
#line 110
      result = /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartResource$default$release(arg_0x40e24540);
#line 110
      break;
#line 110
    }
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 76 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP$0$Resource$release(uint8_t id)
#line 76
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP$0$m_rx_buf || /*Msp430Uart1P.UartP*/Msp430UartP$0$m_tx_buf) {
    return EBUSY;
    }
#line 79
  return /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartResource$release(id);
}

# 110 "/opt/tinyos-2.0.2/tos/interfaces/Resource.nc"
inline static error_t TelosSerialP$Resource$release(void ){
#line 110
  unsigned char result;
#line 110

#line 110
  result = /*Msp430Uart1P.UartP*/Msp430UartP$0$Resource$release(/*PlatformSerialC.UartC*/Msp430Uart1C$0$CLIENT_ID);
#line 110

#line 110
  return result;
#line 110
}
#line 110
# 13 "/opt/tinyos-2.0.2/tos/platforms/telosa/TelosSerialP.nc"
static inline error_t TelosSerialP$StdControl$stop(void )
#line 13
{
  TelosSerialP$Resource$release();
  return SUCCESS;
}

# 84 "/opt/tinyos-2.0.2/tos/interfaces/StdControl.nc"
inline static error_t SerialP$SerialControl$stop(void ){
#line 84
  unsigned char result;
#line 84

#line 84
  result = TelosSerialP$StdControl$stop();
#line 84

#line 84
  return result;
#line 84
}
#line 84
# 330 "/opt/tinyos-2.0.2/tos/lib/serial/SerialP.nc"
static inline void SerialP$SerialFlush$flushDone(void )
#line 330
{
  SerialP$SerialControl$stop();
  SerialP$SplitControl$stopDone(SUCCESS);
}

static inline void SerialP$defaultSerialFlushTask$runTask(void )
#line 335
{
  SerialP$SerialFlush$flushDone();
}

# 56 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP$defaultSerialFlushTask$postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(SerialP$defaultSerialFlushTask);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 338 "/opt/tinyos-2.0.2/tos/lib/serial/SerialP.nc"
static inline void SerialP$SerialFlush$default$flush(void )
#line 338
{
  SerialP$defaultSerialFlushTask$postTask();
}

# 38 "/opt/tinyos-2.0.2/tos/lib/serial/SerialFlush.nc"
inline static void SerialP$SerialFlush$flush(void ){
#line 38
  SerialP$SerialFlush$default$flush();
#line 38
}
#line 38
# 326 "/opt/tinyos-2.0.2/tos/lib/serial/SerialP.nc"
static inline void SerialP$stopDoneTask$runTask(void )
#line 326
{
  SerialP$SerialFlush$flush();
}

# 53 "/opt/tinyos-2.0.2/tos/lib/timer/Counter.nc"
inline static /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$get(void ){
#line 53
  unsigned long result;
#line 53

#line 53
  result = /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$get();
#line 53

#line 53
  return result;
#line 53
}
#line 53
# 75 "/opt/tinyos-2.0.2/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getNow(void )
{
  return /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$get();
}

# 98 "/opt/tinyos-2.0.2/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getNow(void ){
#line 98
  unsigned long result;
#line 98

#line 98
  result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getNow();
#line 98

#line 98
  return result;
#line 98
}
#line 98
# 85 "/opt/tinyos-2.0.2/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow(void )
{
#line 86
  return /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getNow();
}

# 125 "/opt/tinyos-2.0.2/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(void ){
#line 125
  unsigned long result;
#line 125

#line 125
  result = /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$getNow();
#line 125

#line 125
  return result;
#line 125
}
#line 125
# 56 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 132 "/opt/tinyos-2.0.2/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[num];

#line 135
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$postTask();
}






static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$startTimer(num, /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow(), dt, TRUE);
}

# 62 "/opt/tinyos-2.0.2/tos/lib/timer/Timer.nc"
inline static void testECDSAM$myTimer$startOneShot(uint32_t dt){
#line 62
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$startOneShot(0U, dt);
#line 62
}
#line 62
# 266 "testECDSAM.nc"
static inline void testECDSAM$SerialControl$startDone(error_t e)
#line 266
{
  testECDSAM$myTimer$startOneShot(5000);
}

# 92 "/opt/tinyos-2.0.2/tos/interfaces/SplitControl.nc"
inline static void SerialP$SplitControl$startDone(error_t error){
#line 92
  testECDSAM$SerialControl$startDone(error);
#line 92
}
#line 92
# 126 "/opt/tinyos-2.0.2/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceDefaultOwner$release(void )
#line 126
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 127
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$resId == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$default_owner_id) {
          if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$RES_GRANTING) {
              /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$grantedTask$postTask();
              {
                unsigned char __nesc_temp = 
#line 131
                SUCCESS;

                {
#line 131
                  __nesc_atomic_end(__nesc_atomic); 
#line 131
                  return __nesc_temp;
                }
              }
            }
          else {
#line 133
            if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$RES_IMM_GRANTING) {
                /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$resId = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$reqResId;
                /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$RES_BUSY;
                {
                  unsigned char __nesc_temp = 
#line 136
                  SUCCESS;

                  {
#line 136
                    __nesc_atomic_end(__nesc_atomic); 
#line 136
                    return __nesc_temp;
                  }
                }
              }
            }
        }
    }
#line 142
    __nesc_atomic_end(__nesc_atomic); }
#line 140
  return FAIL;
}

# 56 "/opt/tinyos-2.0.2/tos/interfaces/ResourceDefaultOwner.nc"
inline static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$ResourceDefaultOwner$release(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceDefaultOwner$release();
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 105 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline error_t HplMsp430Usart1P$AsyncStdControl$start(void )
#line 105
{
  return SUCCESS;
}

# 74 "/opt/tinyos-2.0.2/tos/interfaces/AsyncStdControl.nc"
inline static error_t /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$AsyncStdControl$start(void ){
#line 74
  unsigned char result;
#line 74

#line 74
  result = HplMsp430Usart1P$AsyncStdControl$start();
#line 74

#line 74
  return result;
#line 74
}
#line 74
# 64 "/opt/tinyos-2.0.2/tos/lib/power/AsyncPowerManagerP.nc"
static inline void /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$ResourceDefaultOwner$immediateRequested(void )
#line 64
{
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$AsyncStdControl$start();
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$ResourceDefaultOwner$release();
}

# 81 "/opt/tinyos-2.0.2/tos/interfaces/ResourceDefaultOwner.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceDefaultOwner$immediateRequested(void ){
#line 81
  /*Msp430UsartShare1P.PowerManagerC.PowerManager*/AsyncPowerManagerP$0$ResourceDefaultOwner$immediateRequested();
#line 81
}
#line 81
# 189 "/opt/tinyos-2.0.2/tos/system/ArbiterP.nc"
static inline void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceRequested$default$immediateRequested(uint8_t id)
#line 189
{
}

# 51 "/opt/tinyos-2.0.2/tos/interfaces/ResourceRequested.nc"
inline static void /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceRequested$immediateRequested(uint8_t arg_0x40ef9a38){
#line 51
    /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceRequested$default$immediateRequested(arg_0x40ef9a38);
#line 51
}
#line 51
# 89 "/opt/tinyos-2.0.2/tos/system/ArbiterP.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$Resource$immediateRequest(uint8_t id)
#line 89
{
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceRequested$immediateRequested(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 91
    {
      if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$state == /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$RES_CONTROLLED) {
          /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$RES_IMM_GRANTING;
          /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$reqResId = id;
        }
      else {
          unsigned char __nesc_temp = 
#line 96
          FAIL;

          {
#line 96
            __nesc_atomic_end(__nesc_atomic); 
#line 96
            return __nesc_temp;
          }
        }
    }
#line 99
    __nesc_atomic_end(__nesc_atomic); }
#line 98
  /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceDefaultOwner$immediateRequested();
  if (/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$resId == id) {
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ResourceConfigure$configure(/*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$resId);
      return SUCCESS;
    }
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 103
    /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$state = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$RES_CONTROLLED;
#line 103
    __nesc_atomic_end(__nesc_atomic); }
  return FAIL;
}

# 190 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartResource$default$immediateRequest(uint8_t id)
#line 190
{
#line 190
  return FAIL;
}

# 87 "/opt/tinyos-2.0.2/tos/interfaces/Resource.nc"
inline static error_t /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartResource$immediateRequest(uint8_t arg_0x40e24540){
#line 87
  unsigned char result;
#line 87

#line 87
  switch (arg_0x40e24540) {
#line 87
    case /*PlatformSerialC.UartC*/Msp430Uart1C$0$CLIENT_ID:
#line 87
      result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$Resource$immediateRequest(/*PlatformSerialC.UartC.UsartC*/Msp430Usart1C$0$CLIENT_ID);
#line 87
      break;
#line 87
    default:
#line 87
      result = /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartResource$default$immediateRequest(arg_0x40e24540);
#line 87
      break;
#line 87
    }
#line 87

#line 87
  return result;
#line 87
}
#line 87
# 64 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline error_t /*Msp430Uart1P.UartP*/Msp430UartP$0$Resource$immediateRequest(uint8_t id)
#line 64
{
  return /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartResource$immediateRequest(id);
}

# 87 "/opt/tinyos-2.0.2/tos/interfaces/Resource.nc"
inline static error_t TelosSerialP$Resource$immediateRequest(void ){
#line 87
  unsigned char result;
#line 87

#line 87
  result = /*Msp430Uart1P.UartP*/Msp430UartP$0$Resource$immediateRequest(/*PlatformSerialC.UartC*/Msp430Uart1C$0$CLIENT_ID);
#line 87

#line 87
  return result;
#line 87
}
#line 87
# 10 "/opt/tinyos-2.0.2/tos/platforms/telosa/TelosSerialP.nc"
static inline error_t TelosSerialP$StdControl$start(void )
#line 10
{
  return TelosSerialP$Resource$immediateRequest();
}

# 74 "/opt/tinyos-2.0.2/tos/interfaces/StdControl.nc"
inline static error_t SerialP$SerialControl$start(void ){
#line 74
  unsigned char result;
#line 74

#line 74
  result = TelosSerialP$StdControl$start();
#line 74

#line 74
  return result;
#line 74
}
#line 74
# 320 "/opt/tinyos-2.0.2/tos/lib/serial/SerialP.nc"
static inline void SerialP$startDoneTask$runTask(void )
#line 320
{
  SerialP$SerialControl$start();
  SerialP$SplitControl$startDone(SUCCESS);
}

# 45 "/opt/tinyos-2.0.2/tos/lib/serial/SerialFrameComm.nc"
inline static error_t SerialP$SerialFrameComm$putDelimiter(void ){
#line 45
  unsigned char result;
#line 45

#line 45
  result = HdlcTranslateC$SerialFrameComm$putDelimiter();
#line 45

#line 45
  return result;
#line 45
}
#line 45
# 56 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$signalSendDone$postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$signalSendDone);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 188 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$sendCompleted(error_t error)
#line 188
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 189
    /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendError = error;
#line 189
    __nesc_atomic_end(__nesc_atomic); }
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$signalSendDone$postTask();
}

# 80 "/opt/tinyos-2.0.2/tos/lib/serial/SendBytePacket.nc"
inline static void SerialP$SendBytePacket$sendCompleted(error_t error){
#line 80
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$sendCompleted(error);
#line 80
}
#line 80
# 242 "/opt/tinyos-2.0.2/tos/lib/serial/SerialP.nc"
static __inline bool SerialP$ack_queue_is_empty(void )
#line 242
{
  bool ret;

#line 244
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 244
    ret = SerialP$ackQ.writePtr == SerialP$ackQ.readPtr;
#line 244
    __nesc_atomic_end(__nesc_atomic); }
  return ret;
}











static __inline uint8_t SerialP$ack_queue_top(void )
#line 258
{
  uint8_t tmp = 0;

  /* atomic removed: atomic calls only */
#line 260
  {
    if (!SerialP$ack_queue_is_empty()) {
        tmp = SerialP$ackQ.buf[SerialP$ackQ.readPtr];
      }
  }
  return tmp;
}

static inline uint8_t SerialP$ack_queue_pop(void )
#line 268
{
  uint8_t retval = 0;

#line 270
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 270
    {
      if (SerialP$ackQ.writePtr != SerialP$ackQ.readPtr) {
          retval = SerialP$ackQ.buf[SerialP$ackQ.readPtr];
          if (++ SerialP$ackQ.readPtr > SerialP$ACK_QUEUE_SIZE) {
#line 273
            SerialP$ackQ.readPtr = 0;
            }
        }
    }
#line 276
    __nesc_atomic_end(__nesc_atomic); }
#line 276
  return retval;
}

#line 539
static inline void SerialP$RunTx$runTask(void )
#line 539
{
  uint8_t idle;
  uint8_t done;
  uint8_t fail;









  error_t result = SUCCESS;
  bool send_completed = FALSE;
  bool start_it = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 556
    {
      SerialP$txPending = 0;
      idle = SerialP$txState == SerialP$TXSTATE_IDLE;
      done = SerialP$txState == SerialP$TXSTATE_FINISH;
      fail = SerialP$txState == SerialP$TXSTATE_ERROR;
      if (done || fail) {
          SerialP$txState = SerialP$TXSTATE_IDLE;
          SerialP$txBuf[SerialP$txIndex].state = SerialP$BUFFER_AVAILABLE;
        }
    }
#line 565
    __nesc_atomic_end(__nesc_atomic); }


  if (done || fail) {
      SerialP$txSeqno++;
      if (SerialP$txProto == SERIAL_PROTO_ACK) {
          SerialP$ack_queue_pop();
        }
      else {
          result = done ? SUCCESS : FAIL;
          send_completed = TRUE;
        }
      idle = TRUE;
    }


  if (idle) {
      bool goInactive;

#line 583
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 583
        goInactive = SerialP$offPending;
#line 583
        __nesc_atomic_end(__nesc_atomic); }
      if (goInactive) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 585
            SerialP$txState = SerialP$TXSTATE_INACTIVE;
#line 585
            __nesc_atomic_end(__nesc_atomic); }
        }
      else {

          uint8_t myAckState;
          uint8_t myDataState;

#line 591
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 591
            {
              myAckState = SerialP$txBuf[SerialP$TX_ACK_INDEX].state;
              myDataState = SerialP$txBuf[SerialP$TX_DATA_INDEX].state;
            }
#line 594
            __nesc_atomic_end(__nesc_atomic); }
          if (!SerialP$ack_queue_is_empty() && myAckState == SerialP$BUFFER_AVAILABLE) {
              { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 596
                {
                  SerialP$txBuf[SerialP$TX_ACK_INDEX].state = SerialP$BUFFER_COMPLETE;
                  SerialP$txBuf[SerialP$TX_ACK_INDEX].buf = SerialP$ack_queue_top();
                }
#line 599
                __nesc_atomic_end(__nesc_atomic); }
              SerialP$txProto = SERIAL_PROTO_ACK;
              SerialP$txIndex = SerialP$TX_ACK_INDEX;
              start_it = TRUE;
            }
          else {
#line 604
            if (myDataState == SerialP$BUFFER_FILLING || myDataState == SerialP$BUFFER_COMPLETE) {
                SerialP$txProto = SERIAL_PROTO_PACKET_NOACK;
                SerialP$txIndex = SerialP$TX_DATA_INDEX;
                start_it = TRUE;
              }
            else {
              }
            }
        }
    }
  else {
    }


  if (send_completed) {
      SerialP$SendBytePacket$sendCompleted(result);
    }

  if (SerialP$txState == SerialP$TXSTATE_INACTIVE) {
      SerialP$testOff();
      return;
    }

  if (start_it) {

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 629
        {
          SerialP$txCRC = 0;
          SerialP$txByteCnt = 0;
          SerialP$txState = SerialP$TXSTATE_PROTO;
        }
#line 633
        __nesc_atomic_end(__nesc_atomic); }
      if (SerialP$SerialFrameComm$putDelimiter() != SUCCESS) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 635
            SerialP$txState = SerialP$TXSTATE_ERROR;
#line 635
            __nesc_atomic_end(__nesc_atomic); }
          SerialP$MaybeScheduleTx();
        }
    }
}

# 56 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP$stopDoneTask$postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(SerialP$stopDoneTask);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 92 "/opt/tinyos-2.0.2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$startAt(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type t0, /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type dt){
#line 92
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$startAt(t0, dt);
#line 92
}
#line 92
# 47 "/opt/tinyos-2.0.2/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$startAt(t0, dt);
}

#line 82
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 83
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(t0, dt, TRUE);
}

# 118 "/opt/tinyos-2.0.2/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(uint32_t t0, uint32_t dt){
#line 118
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$startOneShotAt(t0, dt);
#line 118
}
#line 118
# 54 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents();
}

# 62 "/opt/tinyos-2.0.2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Alarm$stop();
#line 62
}
#line 62
# 91 "/opt/tinyos-2.0.2/tos/lib/timer/TransformAlarmC.nc"
static inline void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$stop(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$stop();
}

# 62 "/opt/tinyos-2.0.2/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$stop(void ){
#line 62
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$stop();
#line 62
}
#line 62
# 60 "/opt/tinyos-2.0.2/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop(void )
{
#line 61
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$stop();
}

# 67 "/opt/tinyos-2.0.2/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$stop(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$stop();
#line 67
}
#line 67
# 88 "/opt/tinyos-2.0.2/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$runTask(void )
{




  uint32_t now = /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint8_t num;

  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$stop();

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$fireTimers(now);
        }
      else {
#line 123
        /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(now, min_remaining);
        }
    }
}

# 273 "testECDSAM.nc"
static inline void testECDSAM$myTimer$fired(void )
#line 273
{
  testECDSAM$round_index = 1;
  testECDSAM$init_data();
}

# 192 "/opt/tinyos-2.0.2/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$default$fired(uint8_t num)
{
}

# 72 "/opt/tinyos-2.0.2/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$fired(uint8_t arg_0x40c93e08){
#line 72
  switch (arg_0x40c93e08) {
#line 72
    case 0U:
#line 72
      testECDSAM$myTimer$fired();
#line 72
      break;
#line 72
    default:
#line 72
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$default$fired(arg_0x40c93e08);
#line 72
      break;
#line 72
    }
#line 72
}
#line 72
# 127 "/opt/tinyos-2.0.2/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired(void )
{
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$fireTimers(/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$getNow());
}

# 72 "/opt/tinyos-2.0.2/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$fired(void ){
#line 72
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$TimerFrom$fired();
#line 72
}
#line 72
# 80 "/opt/tinyos-2.0.2/tos/lib/timer/TransformAlarmC.nc"
static inline /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getAlarm(void )
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 82
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type __nesc_temp = 
#line 82
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt;

      {
#line 82
        __nesc_atomic_end(__nesc_atomic); 
#line 82
        return __nesc_temp;
      }
    }
#line 84
    __nesc_atomic_end(__nesc_atomic); }
}

# 105 "/opt/tinyos-2.0.2/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getAlarm(void ){
#line 105
  unsigned long result;
#line 105

#line 105
  result = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$getAlarm();
#line 105

#line 105
  return result;
#line 105
}
#line 105
# 63 "/opt/tinyos-2.0.2/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$start(/*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Alarm$getAlarm(), /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$m_dt, FALSE);
    }
#line 67
  /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$Timer$fired();
}

# 49 "/opt/tinyos-2.0.2/tos/types/TinyError.h"
static inline error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 62 "/opt/tinyos-2.0.2/tos/system/RandomLfsrP.nc"
static inline error_t RandomLfsrP$Init$init(void )
#line 62
{
  /* atomic removed: atomic calls only */
#line 63
  {
    RandomLfsrP$shiftReg = 119 * 119 * (TOS_NODE_ID + 1);
    RandomLfsrP$initSeed = RandomLfsrP$shiftReg;
    RandomLfsrP$mask = 137 * 29 * (TOS_NODE_ID + 1);
  }
  return SUCCESS;
}

# 46 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$CC2int(/*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t x)
#line 46
{
#line 46
  union /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$__nesc_unnamed4333 {
#line 46
    /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t f;
#line 46
    uint16_t t;
  } 
#line 46
  c = { .f = x };

#line 46
  return c.t;
}

static inline uint16_t /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$compareControl(void )
{
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$cc_t x = { 
  .cm = 1, 
  .ccis = 0, 
  .clld = 0, 
  .cap = 0, 
  .ccie = 0 };

  return /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$CC2int(x);
}

#line 94
static inline void /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$setControlAsCompare(void )
{
  * (volatile uint16_t *)386U = /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$compareControl();
}

# 36 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerControl.nc"
inline static void /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$setControlAsCompare(void ){
#line 36
  /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Control$setControlAsCompare();
#line 36
}
#line 36
# 42 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430AlarmC.nc"
static inline error_t /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Init$init(void )
{
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$disableEvents();
  /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Msp430TimerControl$setControlAsCompare();
  return SUCCESS;
}

# 214 "/opt/tinyos-2.0.2/tos/lib/serial/SerialP.nc"
static __inline void SerialP$ackInit(void )
#line 214
{
  SerialP$ackQ.writePtr = SerialP$ackQ.readPtr = 0;
}

#line 205
static __inline void SerialP$rxInit(void )
#line 205
{
  SerialP$rxBuf.writePtr = SerialP$rxBuf.readPtr = 0;
  SerialP$rxState = SerialP$RXSTATE_NOSYNC;
  SerialP$rxByteCnt = 0;
  SerialP$rxProto = 0;
  SerialP$rxSeqno = 0;
  SerialP$rxCRC = 0;
}

#line 193
static __inline void SerialP$txInit(void )
#line 193
{
  uint8_t i;

  /* atomic removed: atomic calls only */
#line 195
  for (i = 0; i < SerialP$TX_BUFFER_COUNT; i++) SerialP$txBuf[i].state = SerialP$BUFFER_AVAILABLE;
  SerialP$txState = SerialP$TXSTATE_IDLE;
  SerialP$txByteCnt = 0;
  SerialP$txProto = 0;
  SerialP$txSeqno = 0;
  SerialP$txCRC = 0;
  SerialP$txPending = FALSE;
  SerialP$txIndex = 0;
}

#line 218
static inline error_t SerialP$Init$init(void )
#line 218
{

  SerialP$txInit();
  SerialP$rxInit();
  SerialP$ackInit();

  return SUCCESS;
}

# 45 "/opt/tinyos-2.0.2/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$Init$init(void )
#line 45
{
  memset(/*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$resQ, /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$NO_ENTRY, sizeof /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$resQ);
  return SUCCESS;
}

# 51 "/opt/tinyos-2.0.2/tos/interfaces/Init.nc"
inline static error_t RealMainP$SoftwareInit$init(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = /*Msp430UsartShare1P.ArbiterC.Queue*/FcfsResourceQueueC$0$Init$init();
#line 51
  result = ecombine(result, SerialP$Init$init());
#line 51
  result = ecombine(result, /*HilTimerMilliC.AlarmMilli32C.AlarmFrom.Msp430Alarm*/Msp430AlarmC$0$Init$init());
#line 51
  result = ecombine(result, RandomLfsrP$Init$init());
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 56 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP$startDoneTask$postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(SerialP$startDoneTask);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 342 "/opt/tinyos-2.0.2/tos/lib/serial/SerialP.nc"
static inline error_t SerialP$SplitControl$start(void )
#line 342
{
  SerialP$startDoneTask$postTask();
  return SUCCESS;
}

# 83 "/opt/tinyos-2.0.2/tos/interfaces/SplitControl.nc"
inline static error_t testECDSAM$SerialControl$start(void ){
#line 83
  unsigned char result;
#line 83

#line 83
  result = SerialP$SplitControl$start();
#line 83

#line 83
  return result;
#line 83
}
#line 83
# 262 "testECDSAM.nc"
static inline void testECDSAM$Boot$booted(void )
#line 262
{
  testECDSAM$SerialControl$start();
}

# 49 "/opt/tinyos-2.0.2/tos/interfaces/Boot.nc"
inline static void RealMainP$Boot$booted(void ){
#line 49
  testECDSAM$Boot$booted();
#line 49
}
#line 49
# 190 "/opt/tinyos-2.0.2/tos/chips/msp430/msp430hardware.h"
static inline void __nesc_disable_interrupt(void )
{
   __asm volatile ("dint");
   __asm volatile ("nop");}

# 124 "/opt/tinyos-2.0.2/tos/chips/msp430/McuSleepC.nc"
static inline mcu_power_t McuSleepC$McuPowerOverride$default$lowestState(void )
#line 124
{
  return MSP430_POWER_LPM4;
}

# 54 "/opt/tinyos-2.0.2/tos/interfaces/McuPowerOverride.nc"
inline static mcu_power_t McuSleepC$McuPowerOverride$lowestState(void ){
#line 54
  unsigned char result;
#line 54

#line 54
  result = McuSleepC$McuPowerOverride$default$lowestState();
#line 54

#line 54
  return result;
#line 54
}
#line 54
# 66 "/opt/tinyos-2.0.2/tos/chips/msp430/McuSleepC.nc"
static inline mcu_power_t McuSleepC$getPowerState(void )
#line 66
{
  mcu_power_t pState = MSP430_POWER_LPM3;





  if (((((
#line 69
  TA0CCTL0 & 0x0010 || 
  TA0CCTL1 & 0x0010) || 
  TA0CCTL2 & 0x0010) && (
  TA0CTL & (3 << 8)) == 2 << 8) || (
  ME1 & ((1 << 7) | (1 << 6)) && U0TCTL & 0x20)) || (
  ME2 & ((1 << 5) | (1 << 4)) && U1TCTL & 0x20)) {






    pState = MSP430_POWER_LPM1;
    }


  if (ADC12CTL0 & 0x0010) {
      if (ADC12CTL1 & (2 << 3)) {

          if (ADC12CTL1 & (1 << 3)) {
            pState = MSP430_POWER_LPM1;
            }
          else {
#line 91
            pState = MSP430_POWER_ACTIVE;
            }
        }
      else {
#line 92
        if (ADC12CTL1 & 0x0400 && (TA0CTL & (3 << 8)) == 2 << 8) {



            pState = MSP430_POWER_LPM1;
          }
        }
    }

  return pState;
}

# 178 "/opt/tinyos-2.0.2/tos/chips/msp430/msp430hardware.h"
static inline mcu_power_t mcombine(mcu_power_t m1, mcu_power_t m2)
#line 178
{
  return m1 < m2 ? m1 : m2;
}

# 104 "/opt/tinyos-2.0.2/tos/chips/msp430/McuSleepC.nc"
static inline void McuSleepC$computePowerState(void )
#line 104
{
  McuSleepC$powerState = mcombine(McuSleepC$getPowerState(), 
  McuSleepC$McuPowerOverride$lowestState());
}

static inline void McuSleepC$McuSleep$sleep(void )
#line 109
{
  uint16_t temp;

#line 111
  if (McuSleepC$dirty) {
      McuSleepC$computePowerState();
    }

  temp = McuSleepC$msp430PowerBits[McuSleepC$powerState] | 0x0008;
   __asm volatile ("bis  %0, r2" :  : "m"(temp));
  __nesc_disable_interrupt();
}

# 59 "/opt/tinyos-2.0.2/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP$McuSleep$sleep(void ){
#line 59
  McuSleepC$McuSleep$sleep();
#line 59
}
#line 59
# 67 "/opt/tinyos-2.0.2/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP$popTask(void )
{
  if (SchedulerBasicP$m_head != SchedulerBasicP$NO_TASK) 
    {
      uint8_t id = SchedulerBasicP$m_head;

#line 72
      SchedulerBasicP$m_head = SchedulerBasicP$m_next[SchedulerBasicP$m_head];
      if (SchedulerBasicP$m_head == SchedulerBasicP$NO_TASK) 
        {
          SchedulerBasicP$m_tail = SchedulerBasicP$NO_TASK;
        }
      SchedulerBasicP$m_next[id] = SchedulerBasicP$NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP$NO_TASK;
    }
}

#line 138
static inline void SchedulerBasicP$Scheduler$taskLoop(void )
{
  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP$popTask()) == SchedulerBasicP$NO_TASK) 
            {
              SchedulerBasicP$McuSleep$sleep();
            }
        }
#line 150
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP$TaskBasic$runTask(nextTask);
    }
}

# 61 "/opt/tinyos-2.0.2/tos/interfaces/Scheduler.nc"
inline static void RealMainP$Scheduler$taskLoop(void ){
#line 61
  SchedulerBasicP$Scheduler$taskLoop();
#line 61
}
#line 61
# 155 "/opt/tinyos-2.0.2/tos/system/ArbiterP.nc"
static inline uint8_t /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ArbiterInfo$userId(void )
#line 155
{
  /* atomic removed: atomic calls only */
#line 156
  {
    unsigned char __nesc_temp = 
#line 156
    /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$resId;

#line 156
    return __nesc_temp;
  }
}

# 88 "/opt/tinyos-2.0.2/tos/interfaces/ArbiterInfo.nc"
inline static uint8_t /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$ArbiterInfo$userId(void ){
#line 88
  unsigned char result;
#line 88

#line 88
  result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ArbiterInfo$userId();
#line 88

#line 88
  return result;
#line 88
}
#line 88
# 387 "/opt/tinyos-2.0.2/tos/lib/serial/SerialP.nc"
static inline void SerialP$SerialFrameComm$dataReceived(uint8_t data)
#line 387
{
  SerialP$rx_state_machine(FALSE, data);
}

# 83 "/opt/tinyos-2.0.2/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC$SerialFrameComm$dataReceived(uint8_t data){
#line 83
  SerialP$SerialFrameComm$dataReceived(data);
#line 83
}
#line 83
# 384 "/opt/tinyos-2.0.2/tos/lib/serial/SerialP.nc"
static inline void SerialP$SerialFrameComm$delimiterReceived(void )
#line 384
{
  SerialP$rx_state_machine(TRUE, 0);
}

# 74 "/opt/tinyos-2.0.2/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC$SerialFrameComm$delimiterReceived(void ){
#line 74
  SerialP$SerialFrameComm$delimiterReceived();
#line 74
}
#line 74
# 61 "/opt/tinyos-2.0.2/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC$UartStream$receivedByte(uint8_t data)
#line 61
{






  if (data == HDLC_FLAG_BYTE) {

      HdlcTranslateC$SerialFrameComm$delimiterReceived();
      return;
    }
  else {
#line 73
    if (data == HDLC_CTLESC_BYTE) {

        HdlcTranslateC$state.receiveEscape = 1;
        return;
      }
    else {
#line 78
      if (HdlcTranslateC$state.receiveEscape) {

          HdlcTranslateC$state.receiveEscape = 0;
          data = data ^ 0x20;
        }
      }
    }
#line 83
  HdlcTranslateC$SerialFrameComm$dataReceived(data);
}

# 79 "/opt/tinyos-2.0.2/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP$0$UartStream$receivedByte(uint8_t byte){
#line 79
  HdlcTranslateC$UartStream$receivedByte(byte);
#line 79
}
#line 79
# 116 "/opt/tinyos-2.0.2/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC$UartStream$receiveDone(uint8_t *buf, uint16_t len, error_t error)
#line 116
{
}

# 99 "/opt/tinyos-2.0.2/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP$0$UartStream$receiveDone(uint8_t *buf, uint16_t len, error_t error){
#line 99
  HdlcTranslateC$UartStream$receiveDone(buf, len, error);
#line 99
}
#line 99
# 123 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartInterrupts$rxDone(uint8_t data)
#line 123
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP$0$m_rx_buf) {
      /*Msp430Uart1P.UartP*/Msp430UartP$0$m_rx_buf[/*Msp430Uart1P.UartP*/Msp430UartP$0$m_rx_pos++] = data;
      if (/*Msp430Uart1P.UartP*/Msp430UartP$0$m_rx_pos >= /*Msp430Uart1P.UartP*/Msp430UartP$0$m_rx_len) {
          uint8_t *buf = /*Msp430Uart1P.UartP*/Msp430UartP$0$m_rx_buf;

#line 128
          /*Msp430Uart1P.UartP*/Msp430UartP$0$m_rx_buf = (void *)0;
          /*Msp430Uart1P.UartP*/Msp430UartP$0$UartStream$receiveDone(buf, /*Msp430Uart1P.UartP*/Msp430UartP$0$m_rx_len, SUCCESS);
        }
    }
  else {
      /*Msp430Uart1P.UartP*/Msp430UartP$0$UartStream$receivedByte(data);
    }
}

# 65 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$Interrupts$default$rxDone(uint8_t id, uint8_t data)
#line 65
{
}

# 54 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$Interrupts$rxDone(uint8_t arg_0x40eb5bd0, uint8_t data){
#line 54
  switch (arg_0x40eb5bd0) {
#line 54
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C$0$CLIENT_ID:
#line 54
      /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartInterrupts$rxDone(data);
#line 54
      break;
#line 54
    default:
#line 54
      /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$Interrupts$default$rxDone(arg_0x40eb5bd0, data);
#line 54
      break;
#line 54
    }
#line 54
}
#line 54
# 146 "/opt/tinyos-2.0.2/tos/system/ArbiterP.nc"
static inline bool /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ArbiterInfo$inUse(void )
#line 146
{
  return TRUE;
}

# 80 "/opt/tinyos-2.0.2/tos/interfaces/ArbiterInfo.nc"
inline static bool /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$ArbiterInfo$inUse(void ){
#line 80
  unsigned char result;
#line 80

#line 80
  result = /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$ArbiterInfo$inUse();
#line 80

#line 80
  return result;
#line 80
}
#line 80
# 54 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$RawInterrupts$rxDone(uint8_t data)
#line 54
{
  if (/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$ArbiterInfo$inUse()) {
    /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$Interrupts$rxDone(/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$ArbiterInfo$userId(), data);
    }
}

# 54 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart1P$Interrupts$rxDone(uint8_t data){
#line 54
  /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$RawInterrupts$rxDone(data);
#line 54
}
#line 54
# 391 "/opt/tinyos-2.0.2/tos/lib/serial/SerialP.nc"
static inline bool SerialP$valid_rx_proto(uint8_t proto)
#line 391
{
  switch (proto) {
      case SERIAL_PROTO_PACKET_ACK: 
        return TRUE;
      case SERIAL_PROTO_ACK: 
        case SERIAL_PROTO_PACKET_NOACK: 
          default: 
            return FALSE;
    }
}

# 197 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$lockCurrentBuffer(void )
#line 197
{
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.which) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.bufOneLocked = 1;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.bufZeroLocked = 1;
    }
}

#line 193
static inline bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$isCurrentBufferLocked(void )
#line 193
{
  return /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.which ? /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.bufZeroLocked : /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.bufOneLocked;
}

#line 220
static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$startPacket(void )
#line 220
{
  error_t result = SUCCESS;

  /* atomic removed: atomic calls only */
#line 222
  {
    if (!/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$isCurrentBufferLocked()) {


        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$lockCurrentBuffer();
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$RECV_STATE_BEGIN;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvIndex = 0;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvType = TOS_SERIAL_UNKNOWN_ID;
      }
    else {
        result = EBUSY;
      }
  }
  return result;
}

# 51 "/opt/tinyos-2.0.2/tos/lib/serial/ReceiveBytePacket.nc"
inline static error_t SerialP$ReceiveBytePacket$startPacket(void ){
#line 51
  unsigned char result;
#line 51

#line 51
  result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$startPacket();
#line 51

#line 51
  return result;
#line 51
}
#line 51
# 309 "/opt/tinyos-2.0.2/tos/lib/serial/SerialP.nc"
static __inline uint16_t SerialP$rx_current_crc(void )
#line 309
{
  uint16_t crc;
  uint8_t tmp = SerialP$rxBuf.writePtr;

#line 312
  tmp = tmp == 0 ? SerialP$RX_DATA_BUFFER_SIZE : tmp - 1;
  crc = SerialP$rxBuf.buf[tmp] & 0x00ff;
  crc = (crc << 8) & 0xFF00;
  tmp = tmp == 0 ? SerialP$RX_DATA_BUFFER_SIZE : tmp - 1;
  crc |= SerialP$rxBuf.buf[tmp] & 0x00FF;
  return crc;
}

# 69 "/opt/tinyos-2.0.2/tos/lib/serial/ReceiveBytePacket.nc"
inline static void SerialP$ReceiveBytePacket$endPacket(error_t result){
#line 69
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$endPacket(result);
#line 69
}
#line 69
# 215 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveBufferSwap(void )
#line 215
{
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.which = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.which ? 0 : 1;
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveBuffer = (uint8_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$messagePtrs[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.which];
}

# 56 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTask$postTask(void ){
#line 56
  unsigned char result;
#line 56

#line 56
  result = SchedulerBasicP$TaskBasic$postTask(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTask);
#line 56

#line 56
  return result;
#line 56
}
#line 56
# 232 "/opt/tinyos-2.0.2/tos/lib/serial/SerialP.nc"
static __inline bool SerialP$ack_queue_is_full(void )
#line 232
{
  uint8_t tmp;
#line 233
  uint8_t tmp2;

  /* atomic removed: atomic calls only */
#line 234
  {
    tmp = SerialP$ackQ.writePtr;
    tmp2 = SerialP$ackQ.readPtr;
  }
  if (++tmp > SerialP$ACK_QUEUE_SIZE) {
#line 238
    tmp = 0;
    }
#line 239
  return tmp == tmp2;
}







static __inline void SerialP$ack_queue_push(uint8_t token)
#line 248
{
  if (!SerialP$ack_queue_is_full()) {
      /* atomic removed: atomic calls only */
#line 250
      {
        SerialP$ackQ.buf[SerialP$ackQ.writePtr] = token;
        if (++ SerialP$ackQ.writePtr > SerialP$ACK_QUEUE_SIZE) {
#line 252
          SerialP$ackQ.writePtr = 0;
          }
      }
#line 254
      SerialP$MaybeScheduleTx();
    }
}

# 238 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$byteReceived(uint8_t b)
#line 238
{
  /* atomic removed: atomic calls only */
#line 239
  {
    switch (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.state) {
        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$RECV_STATE_BEGIN: 
          /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$RECV_STATE_DATA;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvIndex = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$offset(b);
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvType = b;
        break;

        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$RECV_STATE_DATA: 
          if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvIndex < sizeof(message_t )) {
              /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveBuffer[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvIndex] = b;
              /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvIndex++;
            }
          else {
            }




        break;

        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$RECV_STATE_IDLE: 
          default: 
#line 260
            ;
      }
  }
}

# 58 "/opt/tinyos-2.0.2/tos/lib/serial/ReceiveBytePacket.nc"
inline static void SerialP$ReceiveBytePacket$byteReceived(uint8_t data){
#line 58
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$byteReceived(data);
#line 58
}
#line 58
# 299 "/opt/tinyos-2.0.2/tos/lib/serial/SerialP.nc"
static __inline uint8_t SerialP$rx_buffer_top(void )
#line 299
{
  uint8_t tmp = SerialP$rxBuf.buf[SerialP$rxBuf.readPtr];

#line 301
  return tmp;
}

#line 303
static __inline uint8_t SerialP$rx_buffer_pop(void )
#line 303
{
  uint8_t tmp = SerialP$rxBuf.buf[SerialP$rxBuf.readPtr];

#line 305
  if (++ SerialP$rxBuf.readPtr > SerialP$RX_DATA_BUFFER_SIZE) {
#line 305
    SerialP$rxBuf.readPtr = 0;
    }
#line 306
  return tmp;
}

#line 295
static __inline void SerialP$rx_buffer_push(uint8_t data)
#line 295
{
  SerialP$rxBuf.buf[SerialP$rxBuf.writePtr] = data;
  if (++ SerialP$rxBuf.writePtr > SerialP$RX_DATA_BUFFER_SIZE) {
#line 297
    SerialP$rxBuf.writePtr = 0;
    }
}

# 55 "/opt/tinyos-2.0.2/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC$SerialFrameComm$resetReceive(void )
#line 55
{
  HdlcTranslateC$state.receiveEscape = 0;
}

# 68 "/opt/tinyos-2.0.2/tos/lib/serial/SerialFrameComm.nc"
inline static void SerialP$SerialFrameComm$resetReceive(void ){
#line 68
  HdlcTranslateC$SerialFrameComm$resetReceive();
#line 68
}
#line 68
#line 54
inline static error_t SerialP$SerialFrameComm$putData(uint8_t data){
#line 54
  unsigned char result;
#line 54

#line 54
  result = HdlcTranslateC$SerialFrameComm$putData(data);
#line 54

#line 54
  return result;
#line 54
}
#line 54
# 513 "/opt/tinyos-2.0.2/tos/lib/serial/SerialP.nc"
static inline error_t SerialP$SendBytePacket$completeSend(void )
#line 513
{
  bool ret = FAIL;

  /* atomic removed: atomic calls only */
#line 515
  {
    SerialP$txBuf[SerialP$TX_DATA_INDEX].state = SerialP$BUFFER_COMPLETE;
    ret = SUCCESS;
  }
  return ret;
}

# 60 "/opt/tinyos-2.0.2/tos/lib/serial/SendBytePacket.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$completeSend(void ){
#line 60
  unsigned char result;
#line 60

#line 60
  result = SerialP$SendBytePacket$completeSend();
#line 60

#line 60
  return result;
#line 60
}
#line 60
# 172 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$nextByte(void )
#line 172
{
  uint8_t b;
  uint8_t indx;

  /* atomic removed: atomic calls only */
#line 175
  {
    b = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendBuffer[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendIndex];
    /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendIndex++;
    indx = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendIndex;
  }
  if (indx > /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendLen) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$completeSend();
      return 0;
    }
  else {
      return b;
    }
}

# 70 "/opt/tinyos-2.0.2/tos/lib/serial/SendBytePacket.nc"
inline static uint8_t SerialP$SendBytePacket$nextByte(void ){
#line 70
  unsigned char result;
#line 70

#line 70
  result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$nextByte();
#line 70

#line 70
  return result;
#line 70
}
#line 70
# 642 "/opt/tinyos-2.0.2/tos/lib/serial/SerialP.nc"
static inline void SerialP$SerialFrameComm$putDone(void )
#line 642
{
  {
    error_t txResult = SUCCESS;

    switch (SerialP$txState) {

        case SerialP$TXSTATE_PROTO: 

          txResult = SerialP$SerialFrameComm$putData(SerialP$txProto);

        SerialP$txState = SerialP$TXSTATE_INFO;



        SerialP$txCRC = crcByte(SerialP$txCRC, SerialP$txProto);
        break;

        case SerialP$TXSTATE_SEQNO: 
          txResult = SerialP$SerialFrameComm$putData(SerialP$txSeqno);
        SerialP$txState = SerialP$TXSTATE_INFO;
        SerialP$txCRC = crcByte(SerialP$txCRC, SerialP$txSeqno);
        break;

        case SerialP$TXSTATE_INFO: /* atomic removed: atomic calls only */
          {
            txResult = SerialP$SerialFrameComm$putData(SerialP$txBuf[SerialP$txIndex].buf);
            SerialP$txCRC = crcByte(SerialP$txCRC, SerialP$txBuf[SerialP$txIndex].buf);
            ++SerialP$txByteCnt;

            if (SerialP$txIndex == SerialP$TX_DATA_INDEX) {
                uint8_t nextByte;

#line 673
                nextByte = SerialP$SendBytePacket$nextByte();
                if (SerialP$txBuf[SerialP$txIndex].state == SerialP$BUFFER_COMPLETE || SerialP$txByteCnt >= SerialP$SERIAL_MTU) {
                    SerialP$txState = SerialP$TXSTATE_FCS1;
                  }
                else {
                    SerialP$txBuf[SerialP$txIndex].buf = nextByte;
                  }
              }
            else {
                SerialP$txState = SerialP$TXSTATE_FCS1;
              }
          }
        break;

        case SerialP$TXSTATE_FCS1: 
          txResult = SerialP$SerialFrameComm$putData(SerialP$txCRC & 0xff);
        SerialP$txState = SerialP$TXSTATE_FCS2;
        break;

        case SerialP$TXSTATE_FCS2: 
          txResult = SerialP$SerialFrameComm$putData((SerialP$txCRC >> 8) & 0xff);
        SerialP$txState = SerialP$TXSTATE_ENDFLAG;
        break;

        case SerialP$TXSTATE_ENDFLAG: 
          txResult = SerialP$SerialFrameComm$putDelimiter();
        SerialP$txState = SerialP$TXSTATE_ENDWAIT;
        break;

        case SerialP$TXSTATE_ENDWAIT: 
          SerialP$txState = SerialP$TXSTATE_FINISH;
        case SerialP$TXSTATE_FINISH: 
          SerialP$MaybeScheduleTx();
        break;
        case SerialP$TXSTATE_ERROR: 
          default: 
            txResult = FAIL;
        break;
      }

    if (txResult != SUCCESS) {
        SerialP$txState = SerialP$TXSTATE_ERROR;
        SerialP$MaybeScheduleTx();
      }
  }
}

# 89 "/opt/tinyos-2.0.2/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC$SerialFrameComm$putDone(void ){
#line 89
  SerialP$SerialFrameComm$putDone();
#line 89
}
#line 89
# 48 "/opt/tinyos-2.0.2/tos/interfaces/UartStream.nc"
inline static error_t HdlcTranslateC$UartStream$send(uint8_t *buf, uint16_t len){
#line 48
  unsigned char result;
#line 48

#line 48
  result = /*Msp430Uart1P.UartP*/Msp430UartP$0$UartStream$send(buf, len);
#line 48

#line 48
  return result;
#line 48
}
#line 48
# 104 "/opt/tinyos-2.0.2/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC$UartStream$sendDone(uint8_t *buf, uint16_t len, 
error_t error)
#line 105
{
  if (HdlcTranslateC$state.sendEscape) {
      HdlcTranslateC$state.sendEscape = 0;
      HdlcTranslateC$m_data = HdlcTranslateC$txTemp;
      HdlcTranslateC$UartStream$send(&HdlcTranslateC$m_data, 1);
    }
  else {
      HdlcTranslateC$SerialFrameComm$putDone();
    }
}

# 57 "/opt/tinyos-2.0.2/tos/interfaces/UartStream.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP$0$UartStream$sendDone(uint8_t *buf, uint16_t len, error_t error){
#line 57
  HdlcTranslateC$UartStream$sendDone(buf, len, error);
#line 57
}
#line 57
# 384 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static inline void HplMsp430Usart1P$Usart$tx(uint8_t data)
#line 384
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 385
    HplMsp430Usart1P$U1TXBUF = data;
#line 385
    __nesc_atomic_end(__nesc_atomic); }
}

# 224 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430Usart.nc"
inline static void /*Msp430Uart1P.UartP*/Msp430UartP$0$Usart$tx(uint8_t data){
#line 224
  HplMsp430Usart1P$Usart$tx(data);
#line 224
}
#line 224
# 149 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
static inline void /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartInterrupts$txDone(void )
#line 149
{
  if (/*Msp430Uart1P.UartP*/Msp430UartP$0$m_tx_pos < /*Msp430Uart1P.UartP*/Msp430UartP$0$m_tx_len) {
      /*Msp430Uart1P.UartP*/Msp430UartP$0$Usart$tx(/*Msp430Uart1P.UartP*/Msp430UartP$0$m_tx_buf[/*Msp430Uart1P.UartP*/Msp430UartP$0$m_tx_pos++]);
    }
  else {
      uint8_t *buf = /*Msp430Uart1P.UartP*/Msp430UartP$0$m_tx_buf;

#line 155
      /*Msp430Uart1P.UartP*/Msp430UartP$0$m_tx_buf = (void *)0;
      /*Msp430Uart1P.UartP*/Msp430UartP$0$UartStream$sendDone(buf, /*Msp430Uart1P.UartP*/Msp430UartP$0$m_tx_len, SUCCESS);
    }
}

# 64 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$Interrupts$default$txDone(uint8_t id)
#line 64
{
}

# 49 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$Interrupts$txDone(uint8_t arg_0x40eb5bd0){
#line 49
  switch (arg_0x40eb5bd0) {
#line 49
    case /*PlatformSerialC.UartC.UsartC*/Msp430Usart1C$0$CLIENT_ID:
#line 49
      /*Msp430Uart1P.UartP*/Msp430UartP$0$UsartInterrupts$txDone();
#line 49
      break;
#line 49
    default:
#line 49
      /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$Interrupts$default$txDone(arg_0x40eb5bd0);
#line 49
      break;
#line 49
    }
#line 49
}
#line 49
# 49 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UsartShareP.nc"
static inline void /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$RawInterrupts$txDone(void )
#line 49
{
  if (/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$ArbiterInfo$inUse()) {
    /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$Interrupts$txDone(/*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$ArbiterInfo$userId());
    }
}

# 49 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430UsartInterrupts.nc"
inline static void HplMsp430Usart1P$Interrupts$txDone(void ){
#line 49
  /*Msp430UsartShare1P.UsartShareP*/Msp430UsartShareP$0$RawInterrupts$txDone();
#line 49
}
#line 49
# 210 "/opt/tinyos-2.0.2/tos/chips/msp430/msp430hardware.h"
 __nesc_atomic_t __nesc_atomic_start(void )
{
  __nesc_atomic_t result = (({
#line 212
    uint16_t __x;

#line 212
     __asm volatile ("mov	r2, %0" : "=r"((uint16_t )__x));__x;
  }
  )
#line 212
   & 0x0008) != 0;

#line 213
  __nesc_disable_interrupt();
   __asm volatile ("" :  :  : "memory");
  return result;
}

 void __nesc_atomic_end(__nesc_atomic_t reenable_interrupts)
{
   __asm volatile ("" :  :  : "memory");
  if (reenable_interrupts) {
    __nesc_enable_interrupt();
    }
}

# 11 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
 __attribute((wakeup)) __attribute((interrupt(12))) void sig_TIMERA0_VECTOR(void )
#line 11
{
#line 11
  Msp430TimerCommonP$VectorTimerA0$fired();
}

# 169 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCapComP.nc"
static void /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$captured(/*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA0*/Msp430TimerCapComP$0$Compare$fired();
    }
}

#line 169
static void /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$captured(/*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA1*/Msp430TimerCapComP$1$Compare$fired();
    }
}

#line 169
static void /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Event$fired(void )
{
  if (/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Control$getControl().cap) {
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$captured(/*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Capture$getEvent());
    }
  else {
#line 174
    /*Msp430TimerC.Msp430TimerA2*/Msp430TimerCapComP$2$Compare$fired();
    }
}

# 12 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
 __attribute((wakeup)) __attribute((interrupt(10))) void sig_TIMERA1_VECTOR(void )
#line 12
{
#line 12
  Msp430TimerCommonP$VectorTimerA1$fired();
}

#line 13
 __attribute((wakeup)) __attribute((interrupt(26))) void sig_TIMERB0_VECTOR(void )
#line 13
{
#line 13
  Msp430TimerCommonP$VectorTimerB0$fired();
}

# 135 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$default$fired(uint8_t n)
{
}

# 28 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerEvent.nc"
static void /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$fired(uint8_t arg_0x405d6c08){
#line 28
  switch (arg_0x405d6c08) {
#line 28
    case 0:
#line 28
      /*Msp430TimerC.Msp430TimerB0*/Msp430TimerCapComP$3$Event$fired();
#line 28
      break;
#line 28
    case 1:
#line 28
      /*Msp430TimerC.Msp430TimerB1*/Msp430TimerCapComP$4$Event$fired();
#line 28
      break;
#line 28
    case 2:
#line 28
      /*Msp430TimerC.Msp430TimerB2*/Msp430TimerCapComP$5$Event$fired();
#line 28
      break;
#line 28
    case 3:
#line 28
      /*Msp430TimerC.Msp430TimerB3*/Msp430TimerCapComP$6$Event$fired();
#line 28
      break;
#line 28
    case 4:
#line 28
      /*Msp430TimerC.Msp430TimerB4*/Msp430TimerCapComP$7$Event$fired();
#line 28
      break;
#line 28
    case 5:
#line 28
      /*Msp430TimerC.Msp430TimerB5*/Msp430TimerCapComP$8$Event$fired();
#line 28
      break;
#line 28
    case 6:
#line 28
      /*Msp430TimerC.Msp430TimerB6*/Msp430TimerCapComP$9$Event$fired();
#line 28
      break;
#line 28
    case 7:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Overflow$fired();
#line 28
      break;
#line 28
    default:
#line 28
      /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Event$default$fired(arg_0x405d6c08);
#line 28
      break;
#line 28
    }
#line 28
}
#line 28
# 159 "/opt/tinyos-2.0.2/tos/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP$TaskBasic$postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 161
    {
#line 161
      {
        unsigned char __nesc_temp = 
#line 161
        SchedulerBasicP$pushTask(id) ? SUCCESS : EBUSY;

        {
#line 161
          __nesc_atomic_end(__nesc_atomic); 
#line 161
          return __nesc_temp;
        }
      }
    }
#line 164
    __nesc_atomic_end(__nesc_atomic); }
}

# 96 "/opt/tinyos-2.0.2/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$set_alarm(void )
{
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type now = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Counter$get();
#line 98
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type expires;
#line 98
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type remaining;




  expires = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt;


  remaining = (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type )(expires - now);


  if (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 <= now) 
    {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 && 
      expires <= now) {
        remaining = 0;
        }
    }
  else {
      if (expires >= /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 || 
      expires <= now) {
        remaining = 0;
        }
    }
#line 121
  if (remaining > /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$MAX_DELAY) 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 = now + /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$MAX_DELAY;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt = remaining - /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$MAX_DELAY;
      remaining = /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$MAX_DELAY;
    }
  else 
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 += /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt = 0;
    }
  /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$AlarmFrom$startAt((/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_size_type )now << 5, 
  (/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$from_size_type )remaining << 5);
}

# 69 "/opt/tinyos-2.0.2/tos/lib/timer/TransformCounterC.nc"
static /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type /*CounterMilli32C.Transform*/TransformCounterC$0$Counter$get(void )
{
  /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type rv = 0;

#line 72
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*CounterMilli32C.Transform*/TransformCounterC$0$upper_count_type high = /*CounterMilli32C.Transform*/TransformCounterC$0$m_upper;
      /*CounterMilli32C.Transform*/TransformCounterC$0$from_size_type low = /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$get();

#line 76
      if (/*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$isOverflowPending()) 
        {






          high++;
          low = /*CounterMilli32C.Transform*/TransformCounterC$0$CounterFrom$get();
        }
      {
        /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type high_to = high;
        /*CounterMilli32C.Transform*/TransformCounterC$0$to_size_type low_to = low >> /*CounterMilli32C.Transform*/TransformCounterC$0$LOW_SHIFT_RIGHT;

#line 90
        rv = (high_to << /*CounterMilli32C.Transform*/TransformCounterC$0$HIGH_SHIFT_LEFT) | low_to;
      }
    }
#line 92
    __nesc_atomic_end(__nesc_atomic); }
  return rv;
}

# 51 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerP.nc"
static uint16_t /*Msp430TimerC.Msp430TimerB*/Msp430TimerP$1$Timer$get(void )
{




  if (1) {
      /* atomic removed: atomic calls only */
#line 58
      {
        uint16_t t0;
        uint16_t t1 = * (volatile uint16_t *)400U;

#line 61
        do {
#line 61
            t0 = t1;
#line 61
            t1 = * (volatile uint16_t *)400U;
          }
        while (
#line 61
        t0 != t1);
        {
          unsigned int __nesc_temp = 
#line 62
          t1;

#line 62
          return __nesc_temp;
        }
      }
    }
  else 
#line 65
    {
      return * (volatile uint16_t *)400U;
    }
}

# 14 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430TimerCommonP.nc"
 __attribute((wakeup)) __attribute((interrupt(24))) void sig_TIMERB1_VECTOR(void )
#line 14
{
#line 14
  Msp430TimerCommonP$VectorTimerB1$fired();
}

# 52 "/opt/tinyos-2.0.2/tos/system/RealMainP.nc"
  int main(void )
#line 52
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
#line 60
      ;

      RealMainP$Scheduler$init();





      RealMainP$PlatformInit$init();
      while (RealMainP$Scheduler$runNextTask()) ;





      RealMainP$SoftwareInit$init();
      while (RealMainP$Scheduler$runNextTask()) ;
    }
#line 77
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP$Boot$booted();


  RealMainP$Scheduler$taskLoop();




  return -1;
}

# 160 "/opt/tinyos-2.0.2/tos/chips/msp430/timer/Msp430ClockP.nc"
static void Msp430ClockP$set_dco_calib(int calib)
{
  BCSCTL1 = (BCSCTL1 & ~0x07) | ((calib >> 8) & 0x07);
  DCOCTL = calib & 0xff;
}

# 16 "/opt/tinyos-2.0.2/tos/platforms/telosb/MotePlatformC.nc"
static void MotePlatformC$TOSH_FLASH_M25P_DP_bit(bool set)
#line 16
{
  if (set) {
    TOSH_SET_SIMO0_PIN();
    }
  else {
#line 20
    TOSH_CLR_SIMO0_PIN();
    }
#line 21
  TOSH_SET_UCLK0_PIN();
  TOSH_CLR_UCLK0_PIN();
}

# 123 "/opt/tinyos-2.0.2/tos/system/SchedulerBasicP.nc"
static bool SchedulerBasicP$Scheduler$runNextTask(void )
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 127
  {
    nextTask = SchedulerBasicP$popTask();
    if (nextTask == SchedulerBasicP$NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
#line 131
          FALSE;

#line 131
          return __nesc_temp;
        }
      }
  }
#line 134
  SchedulerBasicP$TaskBasic$runTask(nextTask);
  return TRUE;
}

#line 164
static void SchedulerBasicP$TaskBasic$default$runTask(uint8_t id)
{
}

# 64 "/opt/tinyos-2.0.2/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP$TaskBasic$runTask(uint8_t arg_0x4053c5c0){
#line 64
  switch (arg_0x4053c5c0) {
#line 64
    case /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired:
#line 64
      /*HilTimerMilliC.AlarmToTimerC*/AlarmToTimerC$0$fired$runTask();
#line 64
      break;
#line 64
    case /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer:
#line 64
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$runTask();
#line 64
      break;
#line 64
    case SerialP$RunTx:
#line 64
      SerialP$RunTx$runTask();
#line 64
      break;
#line 64
    case SerialP$startDoneTask:
#line 64
      SerialP$startDoneTask$runTask();
#line 64
      break;
#line 64
    case SerialP$stopDoneTask:
#line 64
      SerialP$stopDoneTask$runTask();
#line 64
      break;
#line 64
    case SerialP$defaultSerialFlushTask:
#line 64
      SerialP$defaultSerialFlushTask$runTask();
#line 64
      break;
#line 64
    case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$signalSendDone:
#line 64
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$signalSendDone$runTask();
#line 64
      break;
#line 64
    case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTask:
#line 64
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTask$runTask();
#line 64
      break;
#line 64
    case /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$grantedTask:
#line 64
      /*Msp430UsartShare1P.ArbiterC.Arbiter*/ArbiterP$0$grantedTask$runTask();
#line 64
      break;
#line 64
    default:
#line 64
      SchedulerBasicP$TaskBasic$default$runTask(arg_0x4053c5c0);
#line 64
      break;
#line 64
    }
#line 64
}
#line 64
# 82 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
static void /*Msp430Uart1P.UartP*/Msp430UartP$0$ResourceConfigure$configure(uint8_t id)
#line 82
{
  msp430_uart_union_config_t *config = /*Msp430Uart1P.UartP*/Msp430UartP$0$Msp430UartConfigure$getConfig(id);

#line 84
  /*Msp430Uart1P.UartP*/Msp430UartP$0$m_byte_time = config->uartConfig.ubr / 2;
  /*Msp430Uart1P.UartP*/Msp430UartP$0$Usart$setModeUart(config);
  /*Msp430Uart1P.UartP*/Msp430UartP$0$Usart$enableIntr();
}

# 251 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
static void HplMsp430Usart1P$Usart$disableSpi(void )
#line 251
{
  /* atomic removed: atomic calls only */
#line 252
  {
    HplMsp430Usart1P$ME2 &= ~(1 << 4);
    HplMsp430Usart1P$SIMO$selectIOFunc();
    HplMsp430Usart1P$SOMI$selectIOFunc();
    HplMsp430Usart1P$UCLK$selectIOFunc();
  }
}

#line 211
static void HplMsp430Usart1P$Usart$disableUart(void )
#line 211
{
  /* atomic removed: atomic calls only */
#line 212
  {
    HplMsp430Usart1P$ME2 &= ~((1 << 5) | (1 << 4));
    HplMsp430Usart1P$UTXD$selectIOFunc();
    HplMsp430Usart1P$URXD$selectIOFunc();
  }
}

# 53 "/opt/tinyos-2.0.2/tos/lib/serial/SerialActiveMessageP.nc"
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMSend$send(am_id_t id, am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 55
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$getHeader(msg);

#line 57
  __nesc_hton_uint16(header->dest.data, dest);





  __nesc_hton_uint8(header->type.data, id);
  __nesc_hton_uint8(header->length.data, len);

  return /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubSend$send(msg, len);
}

# 502 "/opt/tinyos-2.0.2/tos/lib/serial/SerialP.nc"
static void SerialP$MaybeScheduleTx(void )
#line 502
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 503
    {
      if (SerialP$txPending == 0) {
          if (SerialP$RunTx$postTask() == SUCCESS) {
              SerialP$txPending = 1;
            }
        }
    }
#line 509
    __nesc_atomic_end(__nesc_atomic); }
}

# 586 "ECCM.nc"
static void ECCM$ECC$init(void )

{


  ECCM$CurveParam$get_param(&ECCM$param);


  ECCM$NN$ModBarrettInit(ECCM$param.p, 160 / 16 + 1, &ECCM$Bbuf);




  ECCM$ECC$win_precompute(& ECCM$param.G, ECCM$pBaseArray);
}

# 3465 "NNM.nc"
static void NNM$NN$ModBarrettInit(NN_DIGIT *c, NN_UINT cDigits, Barrett *pbuf)
{
  NN_DIGIT tmp[2 * (160 / 16 + 1) + 1];
#line 3467
  NN_DIGIT residue[160 / 16 + 1];

  NNM$pBarrett = pbuf;
  NNM$pBarrett->km = NNM$NN_Digits(c, cDigits);

  memset(tmp, 0, (2 * (160 / 16 + 1) + 1) * (16 / 8));
  memset(NNM$pBarrett->mu, 0, (2 * (160 / 16 + 1) + 1) * (16 / 8));
  tmp[2 * NNM$pBarrett->km] = 1;

  NNM$NN_Div(NNM$pBarrett->mu, residue, tmp, 2 * NNM$pBarrett->km + 1, c, cDigits);
  NNM$pBarrett->mu_len = NNM$NN_Digits(NNM$pBarrett->mu, 2 * (160 / 16 + 1) + 1);
}

#line 3154
static unsigned int NNM$NN_Digits(NN_DIGIT *a, NN_UINT digits)
{
  int i;

  for (i = digits - 1; i >= 0; i--) 
    if (a[i]) {
      break;
      }
  return i + 1;
}

#line 3364
static unsigned int NNM$NN_DigitBits(NN_DIGIT a)
{
  unsigned int i;

  for (i = 0; i < 16; i++, a >>= 1) 
    if (a == 0) {
      break;
      }
  return i;
}

#line 233
static void NNM$NN_AssignZero(NN_DIGIT *a, NN_UINT digits)
{

  uint8_t i;

  for (i = 0; i < digits; i++) 
    a[i] = 0;
}

#line 2740
static NN_DIGIT NNM$NN_LShift(NN_DIGIT *a, NN_DIGIT *b, NN_UINT c, NN_UINT digits)
{
  NN_DIGIT bi;
#line 2742
  NN_DIGIT carry;
  NN_UINT i;
#line 2743
  NN_UINT t;

  if (c >= 16) {
    return 0;
    }
  t = 16 - c;

  carry = 0;

  for (i = 0; i < digits; i++) {
      bi = b[i];
      a[i] = (bi << c) | carry;
      carry = c ? bi >> t : 0;
    }

  return carry;
}

#line 3108
static int NNM$NN_Cmp(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits)
{
  int i;

  for (i = digits - 1; i >= 0; i--) {
      if (a[i] > b[i]) {
        return 1;
        }
      else {
#line 3116
        if (a[i] < b[i]) {
          return -1;
          }
        }
    }
#line 3120
  return 0;
}

#line 357
static __attribute((noinline)) NN_DIGIT NNM$NN_Sub(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits)
{
#line 392
  NN_DIGIT borrow;
  NN_UINT i;

  borrow = 1;

  for (i = 0; i < digits; i++) {
       __asm volatile ("rrc %1 \n\t"
      "subc %3, %2 \n\t"
      "mov %2, %0 \n\t"
      "rlc %1 \n\t" : 
      "=r"(a[i]), "+r"(borrow) : 
      "r"(b[i]), "r"(c[i]));}


  borrow = borrow ^ 0x0001;
  return borrow;
}

#line 2766
static NN_DIGIT NNM$NN_RShift(NN_DIGIT *a, NN_DIGIT *b, NN_UINT c, NN_UINT digits)
{
  NN_DIGIT bi;
#line 2768
  NN_DIGIT carry;
  int i;
  NN_UINT t;

  if (c >= 16) {
    return 0;
    }
  t = 16 - c;

  carry = 0;

  for (i = digits - 1; i >= 0; i--) {
      bi = b[i];
      a[i] = (bi >> c) | carry;
      carry = c ? bi << t : 0;
    }

  return carry;
}

# 117 "ECCM.nc"
static void ECCM$c_add(Point *P0, Point *P1, Point *P2)
{
  NN_DIGIT Z0[160 / 16 + 1];
  NN_DIGIT Z1[160 / 16 + 1];
  NN_DIGIT Z2[160 / 16 + 1];

  ECCM$p_clear(P0);
  ECCM$NN$AssignZero(Z0, 160 / 16 + 1);
  ECCM$NN$AssignZero(Z1, 160 / 16 + 1);
  ECCM$NN$AssignZero(Z2, 160 / 16 + 1);
  Z1[0] = 0x01;
  Z2[0] = 0x01;


  ECCM$c_add_mix(P0, Z0, P1, Z1, P2);




  if (!ECCM$Z_is_one(Z0)) 
    {
      ECCM$NN$ModInv(Z1, Z0, ECCM$param.p, 160 / 16 + 1);
      ECCM$NN$ModMultOpt(Z0, Z1, Z1, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
      ECCM$NN$ModMultOpt(P0->x, P0->x, Z0, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
      ECCM$NN$ModMultOpt(Z0, Z0, Z1, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
      ECCM$NN$ModMultOpt(P0->y, P0->y, Z0, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
    }
}

#line 69
static void ECCM$p_clear(Point *P0)
{
  ECCM$NN$AssignZero(P0->x, 160 / 16 + 1);
  ECCM$NN$AssignZero(P0->y, 160 / 16 + 1);
}

#line 148
static void ECCM$c_add_mix(Point *P0, NN_DIGIT *Z0, Point *P1, NN_DIGIT *Z1, Point *P2)
{
  NN_DIGIT t1[160 / 16 + 1];
  NN_DIGIT t2[160 / 16 + 1];
  NN_DIGIT t3[160 / 16 + 1];
  NN_DIGIT t4[160 / 16 + 1];
  NN_DIGIT Z2[160 / 16 + 1];


  if (ECCM$NN$Zero(P2->x, 160 / 16 + 1)) {
      if (ECCM$NN$Zero(P2->y, 160 / 16 + 1)) {
          ECCM$p_copy(P0, P1);
          ECCM$NN$Assign(Z0, Z1, 160 / 16 + 1);
          return;
        }
    }


  if (ECCM$NN$Zero(Z1, 160 / 16 + 1)) {
      ECCM$p_copy(P0, P2);
      ECCM$NN$AssignDigit(Z0, 1, 160 / 16 + 1);
      return;
    }

  ECCM$NN$ModSqrOpt(t1, Z1, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);

  ECCM$NN$ModMultOpt(t2, t1, Z1, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);

  ECCM$NN$ModMultOpt(t1, t1, P2->x, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);

  ECCM$NN$ModMultOpt(t2, t2, P2->y, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);

  ECCM$NN$ModSub(t1, t1, P1->x, ECCM$param.p, 160 / 16 + 1);

  ECCM$NN$ModSub(t2, t2, P1->y, ECCM$param.p, 160 / 16 + 1);

  if (ECCM$NN$Zero(t1, 160 / 16 + 1)) {
      if (ECCM$NN$Zero(t2, 160 / 16 + 1)) {
          ECCM$NN$AssignDigit(Z2, 1, 160 / 16 + 1);
          ECCM$c_dbl_projective(P0, Z0, P2, Z2);
          return;
        }
      else 
#line 189
        {
          ECCM$NN$AssignDigit(Z0, 0, 160 / 16 + 1);
          return;
        }
    }

  ECCM$NN$ModMultOpt(Z0, Z1, t1, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);

  ECCM$NN$ModSqrOpt(t3, t1, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);

  ECCM$NN$ModMultOpt(t4, t3, t1, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);

  ECCM$NN$ModMultOpt(t3, t3, P1->x, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);

  ECCM$NN$LShift(t1, t3, 1, 160 / 16 + 1);
  ECCM$NN$ModSmall(t1, ECCM$param.p, 160 / 16 + 1);

  ECCM$NN$ModSqrOpt(P0->x, t2, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);

  ECCM$NN$ModSub(P0->x, P0->x, t1, ECCM$param.p, 160 / 16 + 1);

  ECCM$NN$ModSub(P0->x, P0->x, t4, ECCM$param.p, 160 / 16 + 1);

  ECCM$NN$ModSub(t3, t3, P0->x, ECCM$param.p, 160 / 16 + 1);

  ECCM$NN$ModMultOpt(t3, t3, t2, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);

  ECCM$NN$ModMultOpt(t4, t4, P1->y, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);

  ECCM$NN$ModSub(P0->y, t3, t4, ECCM$param.p, 160 / 16 + 1);

  return;
}

# 3127 "NNM.nc"
static int NNM$NN_Zero(NN_DIGIT *a, NN_UINT digits)
{
  NN_UINT i;

  for (i = 0; i < digits; i++) 
    if (a[i]) {
      return 0;
      }
  return 1;
}

# 76 "ECCM.nc"
static void ECCM$p_copy(Point *P0, Point *P1)
{
  ECCM$NN$Assign(P0->x, P1->x, 160 / 16 + 1);
  ECCM$NN$Assign(P0->y, P1->y, 160 / 16 + 1);
}

# 3683 "NNM.nc"
static void NNM$NN$ModSqrOpt(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *d, NN_DIGIT *omega, NN_UINT digits)
{

  NN_DIGIT t1[2 * (160 / 16 + 1)];
  NN_DIGIT t2[2 * (160 / 16 + 1)];
  NN_DIGIT *pt1;
  NN_UINT len_t1;
#line 3689
  NN_UINT len_t2;

  t1[2 * (160 / 16 + 1) - 1] = 0;
  t1[2 * (160 / 16 + 1) - 2] = 0;
  t2[2 * (160 / 16 + 1) - 1] = 0;
  t2[2 * (160 / 16 + 1) - 2] = 0;

  NNM$NN_Sqr(t1, b, 160 / 16);

  pt1 = &t1[160 / 16];
  len_t2 = 2 * (160 / 16);

  while (!NNM$NN_Zero(pt1, 160 / 16)) {
      memset(t2, 0, len_t2 * (16 / 8));
      len_t2 -= 160 / 16;
      len_t1 = len_t2;
      len_t2 = NNM$CurveParam$omega_mul(t2, pt1, omega, len_t2);
      memset(pt1, 0, len_t1 * (16 / 8));
      NNM$NN_Add(t1, t2, t1, (160 / 16 < len_t2 ? len_t2 : 160 / 16) + 1);
    }

  while (NNM$NN_Cmp(t1, d, digits) > 0) 
    NNM$NN_Sub(t1, t1, d, digits);
  NNM$NN_Assign(a, t1, digits);
}

#line 1758
static __attribute((noinline)) void NNM$NN_Sqr(NN_DIGIT *a, NN_DIGIT *b, NN_UINT digits)
{
#line 2500
   __asm volatile ("clr r9 \n\t"
  "clr r10 \n\t"
  "clr r11 \n\t"
  "clr r12 \n\t"
  "dec %2 \n\t"
  "SQR_LOOP1: clr r13 \n\t"
  "mov r12, r14 \n\t"
  "inc r14 \n\t"
  "add r14, r14 \n\t"
  "add %1, r14 \n\t"
  "SQR_LOOP2: mov r12, r15 \n\t"
  "sub r13, r15 \n\t"
  "cmp r13, r15 \n\t"
  "jeq EQ1 \n\t"
  "jge GR1 \n\t"
  "jmp SQR_LOOP2_EXIT \n\t"
  "GR1: sub #2, r14 \n\t"
  "mov r13, r15 \n\t"
  "add r15, r15 \n\t"
  "add %1, r15 \n\t"
  "mov @r14, &0x130 \n\t"
  "mov @r15, &0x138 \n\t"
  "add &0x13A, r9 \n\t"
  "addc &0x13C, r10 \n\t"
  "addc #0, r11 \n\t"
  "add &0x13A, r9 \n\t"
  "addc &0x13C, r10 \n\t"
  "addc #0, r11 \n\t"
  "inc r13 \n\t"
  "jmp SQR_LOOP2 \n\t"
  "EQ1: sub #2, r14 \n\t"
  "mov r13, r15 \n\t"
  "add r15, r15 \n\t"
  "add %1, r15 \n\t"
  "mov @r14, &0x130 \n\t"
  "mov @r15, &0x138 \n\t"
  "add &0x13A, r9 \n\t"
  "addc &0x13C, r10 \n\t"
  "addc #0, r11 \n\t"
  "SQR_LOOP2_EXIT: mov r9, 0(%0) \n\t"
  "add #2, %0 \n\t"
  "mov r10, r9 \n\t"
  "mov r11, r10 \n\t"
  "clr r11 \n\t"
  "cmp %2, r12 \n\t"
  "jeq SQR_LOOP1_EXIT \n\t"
  "inc r12 \n\t"
  "jmp SQR_LOOP1 \n\t"
  "SQR_LOOP1_EXIT: inc r12 \n\t"
  "SQR_LOOP3: mov r12, r13 \n\t"
  "sub %2, r13 \n\t"
  "mov %2, r14 \n\t"
  "inc r14 \n\t"
  "add r14, r14 \n\t"
  "add %1, r14 \n\t"
  "SQR_LOOP4: mov r12, r15 \n\t"
  "sub r13, r15 \n\t"
  "cmp r13, r15 \n\t"
  "jeq EQ2 \n\t"
  "jge GR2 \n\t"
  "jmp SQR_LOOP4_EXIT \n\t"
  "GR2: sub #2, r14 \n\t"
  "mov r13, r15 \n\t"
  "add r15, r15 \n\t"
  "add %1, r15 \n\t"
  "mov @r14, &0x130 \n\t"
  "mov @r15, &0x138 \n\t"
  "add &0x13A, r9 \n\t"
  "addc &0x13C, r10 \n\t"
  "addc #0, r11 \n\t"
  "add &0x13A, r9 \n\t"
  "addc &0x13C, r10 \n\t"
  "addc #0, r11 \n\t"
  "inc r13 \n\t"
  "jmp SQR_LOOP4 \n\t"
  "EQ2: sub #2, r14 \n\t"
  "mov r13, r15 \n\t"
  "add r15, r15 \n\t"
  "add %1, r15 \n\t"
  "mov @r14, &0x130 \n\t"
  "mov @r15, &0x138 \n\t"
  "add &0x13A, r9 \n\t"
  "addc &0x13C, r10 \n\t"
  "addc #0, r11 \n\t"
  "SQR_LOOP4_EXIT: mov r9, 0(%0) \n\t"
  "add #2, %0 \n\t"
  "mov r10, r9 \n\t"
  "mov r11, r10 \n\t"
  "mov #0, r11 \n\t"
  "mov %2, r15 \n\t"
  "add r15, r15 \n\t"
  "cmp r12, r15 \n\t"
  "jeq SQR_LOOP3_EXIT \n\t"
  "inc r12 \n\t"
  "jmp SQR_LOOP3 \n\t"
  "SQR_LOOP3_EXIT:mov r9, 0(%0) \n\t" :  : 

  "r"(a), "r"(b), "r"(digits) : 
  "r9", "r10", "r11", "r12", "r13", "r14", "r15");}

# 337 "secp160r1.nc"
static __attribute((noinline)) NN_UINT secp160r1$CurveParam$omega_mul(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *omega, NN_UINT digits)
{
#line 490
  secp160r1$NN$Assign(a, b, digits);
  a[digits + 1] += secp160r1$NN$AddDigitMult(&a[1], &a[1], omega[1], b, digits);
  return digits + 2;
}

# 263 "NNM.nc"
static __attribute((noinline)) NN_DIGIT NNM$NN_Add(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits)
{
#line 297
  NN_DIGIT carry;
  NN_UINT i;

  carry = 0;

  for (i = 0; i < digits; i++) {
       __asm volatile ("rrc %1 \n\t"
      "addc %3, %2 \n\t"
      "mov %2, %0 \n\t"
      "rlc %1 \n\t" : 
      "=r"(a[i]), "+r"(carry) : 
      "r"(b[i]), "r"(c[i]));}


  return carry;
}

#line 3607
static void NNM$NN$ModMultOpt(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_DIGIT *omega, NN_UINT digits)
{

  NN_DIGIT t1[2 * (160 / 16 + 1)];
  NN_DIGIT t2[2 * (160 / 16 + 1)];
  NN_DIGIT *pt1;
  NN_UINT len_t2;
#line 3613
  NN_UINT len_t1;



  t1[2 * (160 / 16 + 1) - 1] = 0;
  t1[2 * (160 / 16 + 1) - 2] = 0;
  t2[2 * (160 / 16 + 1) - 1] = 0;
  t2[2 * (160 / 16 + 1) - 2] = 0;

  NNM$NN_Mult(t1, b, c, 160 / 16);

  pt1 = &t1[160 / 16];
  len_t2 = 2 * (160 / 16);


  while (!NNM$NN_Zero(pt1, 160 / 16)) {
      memset(t2, 0, len_t2 * (16 / 8));
      len_t2 -= 160 / 16;
      len_t1 = len_t2;
      len_t2 = NNM$CurveParam$omega_mul(t2, pt1, omega, len_t2);
      memset(pt1, 0, len_t1 * (16 / 8));
      NNM$NN_Add(t1, t2, t1, (160 / 16 < len_t2 ? len_t2 : 160 / 16) + 1);
    }

  while (NNM$NN_Cmp(t1, d, digits) > 0) 
    NNM$NN_Sub(t1, t1, d, digits);
  NNM$NN_Assign(a, t1, digits);
}

#line 454
static __attribute((noinline)) void NNM$NN_Mult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_UINT digits)
{
#line 1501
   __asm volatile ("clr r9 \n\t"
  "clr r10 \n\t"
  "clr r11 \n\t"
  "clr r12 \n\t"
  "dec %3 \n\t"
  "LOOP1: clr r13 \n\t"
  "mov r12, r14 \n\t"
  "inc r14 \n\t"
  "add r14, r14 \n\t"
  "add %1, r14 \n\t"
  "LOOP2: sub #2, r14 \n\t"
  "mov r13, r15 \n\t"
  "add r15, r15 \n\t"
  "add %2, r15 \n\t"
  "mov @r14, &0x130 \n\t"
  "mov @r15, &0x138 \n\t"
  "add &0x13A, r9 \n\t"
  "addc &0x13C, r10 \n\t"
  "addc #0, r11 \n\t"
  "cmp r12, r13 \n\t"
  "jeq LOOP2_EXIT \n\t"
  "inc r13 \n\t"
  "jmp LOOP2 \n\t"
  "LOOP2_EXIT: mov r9, 0(%0) \n\t"
  "add #2, %0 \n\t"
  "mov r10, r9 \n\t"
  "mov r11, r10 \n\t"
  "clr r11 \n\t"
  "cmp %3, r12 \n\t"
  "jeq LOOP1_EXIT \n\t"
  "inc r12 \n\t"
  "jmp LOOP1 \n\t"
  "LOOP1_EXIT: inc r12 \n\t"
  "LOOP3: mov r12, r13 \n\t"
  "sub %3, r13 \n\t"
  "mov %3, r14 \n\t"
  "inc r14 \n\t"
  "add r14, r14 \n\t"
  "add %1, r14 \n\t"
  "LOOP4: sub #2, r14 \n\t"
  "mov r13, r15 \n\t"
  "add r15, r15 \n\t"
  "add %2, r15 \n\t"
  "mov @r14, &0x130 \n\t"
  "mov @r15, &0x138 \n\t"
  "add &0x13A, r9 \n\t"
  "addc &0x13C, r10 \n\t"
  "addc #0, r11 \n\t"
  "cmp %3, r13 \n\t"
  "jeq LOOP4_EXIT \n\t"
  "inc r13 \n\t"
  "jmp LOOP4 \n\t"
  "LOOP4_EXIT: mov r9, 0(%0) \n\t"
  "add #2, %0 \n\t"
  "mov r10, r9 \n\t"
  "mov r11, r10 \n\t"
  "mov #0, r11 \n\t"
  "mov %3, r15 \n\t"
  "add r15, r15 \n\t"
  "cmp r12, r15 \n\t"
  "jeq LOOP3_EXIT \n\t"
  "inc r12 \n\t"
  "jmp LOOP3 \n\t"
  "LOOP3_EXIT:mov r9, 0(%0) \n\t" :  : 

  "r"(a), "r"(b), "r"(c), "r"(digits) : 
  "r9", "r10", "r11", "r12", "r13", "r14", "r15");}

#line 3562
static void NNM$NN$ModSub(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits)
{
  NN_DIGIT tmp[160 / 16 + 1];
  NN_DIGIT borrow;

  borrow = NNM$NN_Sub(tmp, b, c, digits);
  if (borrow) {
    NNM$NN_Add(a, tmp, d, digits);
    }
  else {
#line 3571
    NNM$NN_Assign(a, tmp, digits);
    }
}

# 358 "ECCM.nc"
static void ECCM$c_dbl_projective(Point *P0, NN_DIGIT *Z0, Point *P1, NN_DIGIT *Z1)
{
  NN_DIGIT n0[160 / 16 + 1];
  NN_DIGIT n1[160 / 16 + 1];
  NN_DIGIT n2[160 / 16 + 1];
  NN_DIGIT n3[160 / 16 + 1];

  if (ECCM$NN$Zero(Z1, 160 / 16 + 1)) 
    {
      ECCM$NN$AssignZero(Z0, 160 / 16 + 1);
      return;
    }


  if (ECCM$Z_is_one(Z1)) 
    {

      ECCM$NN$ModSqrOpt(n0, P1->x, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
      ECCM$NN$LShift(n1, n0, 1, 160 / 16 + 1);
      ECCM$NN$ModSmall(n1, ECCM$param.p, 160 / 16 + 1);
      ECCM$NN$ModAdd(n0, n0, n1, ECCM$param.p, 160 / 16 + 1);
      ECCM$NN$ModAdd(n1, n0, ECCM$param.E.a, ECCM$param.p, 160 / 16 + 1);
    }
  else 
    {
      if (ECCM$param.E.a_minus3) 
        {


          ECCM$NN$ModSqrOpt(n1, Z1, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
          ECCM$NN$ModAdd(n0, P1->x, n1, ECCM$param.p, 160 / 16 + 1);
          ECCM$NN$ModSub(n2, P1->x, n1, ECCM$param.p, 160 / 16 + 1);
          ECCM$NN$ModMultOpt(n1, n0, n2, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
          ECCM$NN$LShift(n0, n1, 1, 160 / 16 + 1);
          ECCM$NN$ModSmall(n0, ECCM$param.p, 160 / 16 + 1);
          ECCM$NN$ModAdd(n1, n0, n1, ECCM$param.p, 160 / 16 + 1);
        }
      else {
#line 395
        if (ECCM$param.E.a_zero) 
          {

            ECCM$NN$ModSqrOpt(n0, P1->x, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
            ECCM$NN$LShift(n1, n0, 1, 160 / 16 + 1);
            ECCM$NN$ModSmall(n1, ECCM$param.p, 160 / 16 + 1);
            ECCM$NN$ModAdd(n1, n0, n1, ECCM$param.p, 160 / 16 + 1);
          }
        else 
          {

            ECCM$NN$ModSqrOpt(n0, P1->x, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
            ECCM$NN$LShift(n1, n0, 1, 160 / 16 + 1);
            ECCM$NN$ModSmall(n1, ECCM$param.p, 160 / 16 + 1);
            ECCM$NN$ModAdd(n0, n0, n1, ECCM$param.p, 160 / 16 + 1);
            ECCM$NN$ModSqrOpt(n1, Z1, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
            ECCM$NN$ModSqrOpt(n1, n1, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
            ECCM$NN$ModMultOpt(n1, n1, ECCM$param.E.a, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
            ECCM$NN$ModAdd(n1, n1, n0, ECCM$param.p, 160 / 16 + 1);
          }
        }
    }

  if (ECCM$Z_is_one(Z1)) 
    {
      ECCM$NN$Assign(n0, P1->y, 160 / 16 + 1);
    }
  else 
    {
      ECCM$NN$ModMultOpt(n0, P1->y, Z1, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
    }
  ECCM$NN$LShift(Z0, n0, 1, 160 / 16 + 1);
  ECCM$NN$ModSmall(Z0, ECCM$param.p, 160 / 16 + 1);


  ECCM$NN$ModSqrOpt(n3, P1->y, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
  ECCM$NN$ModMultOpt(n2, P1->x, n3, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
  ECCM$NN$LShift(n2, n2, 2, 160 / 16 + 1);
  ECCM$NN$ModSmall(n2, ECCM$param.p, 160 / 16 + 1);


  ECCM$NN$LShift(n0, n2, 1, 160 / 16 + 1);
  ECCM$NN$ModSmall(n0, ECCM$param.p, 160 / 16 + 1);
  ECCM$NN$ModSqrOpt(P0->x, n1, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
  ECCM$NN$ModSub(P0->x, P0->x, n0, ECCM$param.p, 160 / 16 + 1);


  ECCM$NN$ModSqrOpt(n0, n3, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
  ECCM$NN$LShift(n3, n0, 3, 160 / 16 + 1);
  ECCM$NN$ModSmall(n3, ECCM$param.p, 160 / 16 + 1);


  ECCM$NN$ModSub(n0, n2, P0->x, ECCM$param.p, 160 / 16 + 1);
  ECCM$NN$ModMultOpt(n0, n1, n0, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
  ECCM$NN$ModSub(P0->y, n0, n3, ECCM$param.p, 160 / 16 + 1);
}

#line 103
static bool ECCM$Z_is_one(NN_DIGIT *Z)
{
  uint8_t i;

  for (i = 1; i < 160 / 16 + 1; i++) 
    if (Z[i]) {
      return FALSE;
      }
#line 110
  if (Z[0] == 1) {
    return TRUE;
    }
  return FALSE;
}

# 3544 "NNM.nc"
static void NNM$NN$ModAdd(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits)
{
  NN_DIGIT tmp[160 / 16 + 1];
  NN_DIGIT carry;

  carry = NNM$NN_Add(tmp, b, c, digits);
  if (carry) {
    NNM$NN_Sub(a, tmp, d, digits);
    }
  else {
#line 3552
    if (NNM$NN_Cmp(tmp, d, digits) >= 0) {
      NNM$NN_Sub(a, tmp, d, digits);
      }
    else {
#line 3555
      NNM$NN_Assign(a, tmp, digits);
      }
    }
}

#line 3022
static void NNM$NN_ModDivOpt(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits)
{
  NN_DIGIT A[160 / 16 + 1];
#line 3024
  NN_DIGIT B[160 / 16 + 1];
#line 3024
  NN_DIGIT U[160 / 16 + 1];
#line 3024
  NN_DIGIT V[160 / 16 + 1];
  int tmp_even;

  NNM$NN_Assign(A, c, digits);
  NNM$NN_Assign(B, d, digits);
  NNM$NN_Assign(U, b, digits);
  NNM$NN_AssignZero(V, digits);

  while ((tmp_even = NNM$NN_Cmp(A, B, digits)) != 0) {
      if (digits == 0 || !(A[0] & 1)) {
          NNM$NN_RShift(A, A, 1, digits);
          if (digits == 0 || !(U[0] & 1)) {
              NNM$NN_RShift(U, U, 1, digits);
            }
          else 
#line 3037
            {
              NNM$NN_Add(U, U, d, digits);
              NNM$NN_RShift(U, U, 1, digits);
            }
        }
      else {
#line 3041
        if (digits == 0 || !(B[0] & 1)) {
            NNM$NN_RShift(B, B, 1, digits);
            if (digits == 0 || !(V[0] & 1)) {
                NNM$NN_RShift(V, V, 1, digits);
              }
            else 
#line 3045
              {
                NNM$NN_Add(V, V, d, digits);
                NNM$NN_RShift(V, V, 1, digits);
              }
          }
        else {
#line 3049
          if (tmp_even > 0) {
              NNM$NN_Sub(A, A, B, digits);
              NNM$NN_RShift(A, A, 1, digits);
              if (NNM$NN_Cmp(U, V, digits) < 0) {
                  NNM$NN_Add(U, U, d, digits);
                }
              NNM$NN_Sub(U, U, V, digits);
              if (digits == 0 || !(U[0] & 1)) {
                  NNM$NN_RShift(U, U, 1, digits);
                }
              else 
#line 3058
                {
                  NNM$NN_Add(U, U, d, digits);
                  NNM$NN_RShift(U, U, 1, digits);
                }
            }
          else 
#line 3062
            {
              NNM$NN_Sub(B, B, A, digits);
              NNM$NN_RShift(B, B, 1, digits);
              if (NNM$NN_Cmp(V, U, digits) < 0) {
                  NNM$NN_Add(V, V, d, digits);
                }
              NNM$NN_Sub(V, V, U, digits);
              if (digits == 0 || !(V[0] & 1)) {
                  NNM$NN_RShift(V, V, 1, digits);
                }
              else 
#line 3071
                {
                  NNM$NN_Add(V, V, d, digits);
                  NNM$NN_RShift(V, V, 1, digits);
                }
            }
          }
        }
    }
#line 3078
  NNM$NN_Assign(a, U, digits);
}

# 804 "ECCM.nc"
static void ECCM$win_mul(Point *P0, NN_DIGIT *n, Point *pointArray)
{
#line 829
  int16_t i;
#line 829
  int16_t tmp;
  int8_t j;
  NN_DIGIT windex;
  NN_DIGIT Z0[160 / 16 + 1];
  NN_DIGIT Z1[160 / 16 + 1];




  ECCM$p_clear(P0);


  ECCM$NN$AssignZero(Z0, 160 / 16 + 1);
  ECCM$NN$AssignZero(Z1, 160 / 16 + 1);
  Z1[0] = 0x01;

  tmp = ECCM$NN$Digits(n, 160 / 16 + 1);

  for (i = tmp - 1; i >= 0; i--) {
      for (j = 16 / 4 - 1; j >= 0; j--) {






          ECCM$c_m_dbl_projective(P0, Z0, 4);


          windex = ECCM$mask[j] & n[i];

          if (windex) {
              windex = windex >> j * 4;

              ECCM$c_add_mix(P0, Z0, P0, Z0, &pointArray[windex - 1]);
            }
        }
    }






  if (!ECCM$Z_is_one(Z0)) 
    {

      ECCM$NN$ModInv(Z1, Z0, ECCM$param.p, 160 / 16 + 1);
      ECCM$NN$ModMultOpt(Z0, Z1, Z1, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
      ECCM$NN$ModMultOpt(P0->x, P0->x, Z0, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
      ECCM$NN$ModMultOpt(Z0, Z0, Z1, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
      ECCM$NN$ModMultOpt(P0->y, P0->y, Z0, ECCM$param.p, ECCM$param.omega, 160 / 16 + 1);
    }
}

# 204 "NNM.nc"
static void NNM$NN_Encode(unsigned char *a, NN_UINT len, NN_DIGIT *b, NN_UINT digits)
{
  NN_DIGIT t;
  int j;
  unsigned int i;
#line 208
  unsigned int u;

  for (i = 0, j = len - 1; i < digits && j >= 0; i++) {
      t = b[i];
      for (u = 0; j >= 0 && u < 16; j--, u += 8) 
        a[j] = (unsigned char )(t >> u);
    }

  for (; j >= 0; j--) 
    a[j] = 0;
}

# 289 "ECDSAM.nc"
static __attribute((noinline)) error_t ECDSAM$ECDSA$init(Point *pKey)
{
  ECDSAM$ECC$init();
  ECDSAM$param = ECDSAM$ECC$get_param();


  ECDSAM$shamir_init(pKey, ECDSAM$pqBaseArray);






  ECDSAM$NN$ModBarrettInit(ECDSAM$param->r, 160 / 16 + 1, &ECDSAM$Bbuf);


  return SUCCESS;
}


static __attribute((noinline)) void ECDSAM$ECDSA$sign(uint8_t *msg, uint8_t len, NN_DIGIT *r, NN_DIGIT *s, NN_DIGIT *d)
{

  bool done = FALSE;
  NN_DIGIT k[160 / 16 + 1];
  NN_DIGIT k_inv[160 / 16 + 1];
  NN_DIGIT tmp[160 / 16 + 1];
  NN_DIGIT digest[160 / 16 + 1];
  Point P;
  uint8_t sha1sum[20];
  NN_DIGIT sha1tmp[20 / (16 / 8)];
  SHA1Context ctx;
  NN_UINT result_bit_len;
#line 321
  NN_UINT order_bit_len;



  ECDSAM$NN$BarrettSetBuf(&ECDSAM$Bbuf);

  while (!done) {



      ECDSAM$ECC$gen_private_key(k);


      if (ECDSAM$NN$Zero(k, 160 / 16 + 1) == 1) {
        continue;
        }

      ECDSAM$ECC$BarrettSetBuf();




      ECDSAM$ECC$win_mul_base(&P, k);





      ECDSAM$NN$BarrettSetBuf(&ECDSAM$Bbuf);
      ECDSAM$NN$Assign(r, P.x, 160 / 16 + 1);
      ECDSAM$NN$ModSmall(r, ECDSAM$param->r, 160 / 16 + 1);




      if (ECDSAM$NN$Zero(r, 160 / 16 + 1) == 1) {
        continue;
        }
#line 358
      ECDSAM$NN$ModInv(k_inv, k, ECDSAM$param->r, 160 / 16 + 1);

      memset(sha1sum, 0, 20);
      memset(digest, 0, (160 / 16 + 1) * (16 / 8));
      ECDSAM$SHA1$reset(&ctx);
      ECDSAM$SHA1$update(&ctx, msg, len);
      ECDSAM$SHA1$digest(&ctx, sha1sum);
      ECDSAM$NN$Decode(sha1tmp, 20 / (16 / 8), sha1sum, 20);

      result_bit_len = ECDSAM$NN$Bits(sha1tmp, 20 / (16 / 8));
      order_bit_len = ECDSAM$NN$Bits(ECDSAM$param->r, 160 / 16 + 1);
      if (result_bit_len > order_bit_len) {
          ECDSAM$NN$Mod(digest, sha1tmp, 20 / (16 / 8), ECDSAM$param->r, 160 / 16 + 1);
        }
      else 
#line 371
        {
          memset(digest, 0, (160 / 16 + 1) * (16 / 8));
          ECDSAM$NN$Assign(digest, sha1tmp, 20 / (16 / 8));
          if (result_bit_len == order_bit_len) {
            ECDSAM$NN$ModSmall(digest, ECDSAM$param->r, 160 / 16 + 1);
            }
        }
      ECDSAM$NN$ModMult(k, d, r, ECDSAM$param->r, 160 / 16 + 1);
      ECDSAM$NN$ModAdd(tmp, digest, k, ECDSAM$param->r, 160 / 16 + 1);
      ECDSAM$NN$ModMult(s, k_inv, tmp, ECDSAM$param->r, 160 / 16 + 1);
      if (ECDSAM$NN$Zero(s, 160 / 16 + 1) != 1) {
        done = TRUE;
        }
    }
}

# 994 "ECCM.nc"
static void ECCM$ECC$gen_private_key(NN_DIGIT *PrivateKey)
#line 994
{


  NN_UINT order_digit_len;
#line 997
  NN_UINT order_bit_len;
  bool done = FALSE;
  uint8_t ri;
  NN_DIGIT digit_mask;

  order_bit_len = ECCM$NN$Bits(ECCM$param.r, 160 / 16 + 1);
  order_digit_len = ECCM$NN$Digits(ECCM$param.r, 160 / 16 + 1);

  while (!done) {

      for (ri = 0; ri < order_digit_len; ri++) {



          PrivateKey[ri] = (NN_DIGIT )ECCM$Random$rand16();
        }


      for (ri = order_digit_len; ri < 160 / 16 + 1; ri++) {
          PrivateKey[ri] = 0;
        }

      if (order_bit_len % 16 != 0) {
          digit_mask = 0xffff >> (16 - order_bit_len % 16);
          PrivateKey[order_digit_len - 1] = PrivateKey[order_digit_len - 1] & digit_mask;
        }
      ECCM$NN$ModSmall(PrivateKey, ECCM$param.r, 160 / 16 + 1);

      if (ECCM$NN$Zero(PrivateKey, 160 / 16 + 1) != 1) {
        done = TRUE;
        }
    }
}

# 3142 "NNM.nc"
static unsigned int NNM$NN_Bits(NN_DIGIT *a, NN_UINT digits)
{
  if ((digits = NNM$NN_Digits(a, digits)) == 0) {
    return 0;
    }
  return (digits - 1) * 16 + NNM$NN_DigitBits(a[digits - 1]);
}

# 72 "/opt/tinyos-2.0.2/tos/system/RandomLfsrP.nc"
static uint16_t RandomLfsrP$Random$rand16(void )
#line 72
{
  bool endbit;
  uint16_t tmpShiftReg;

#line 75
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 75
    {
      tmpShiftReg = RandomLfsrP$shiftReg;
      endbit = (tmpShiftReg & 0x8000) != 0;
      tmpShiftReg <<= 1;
      if (endbit) {
        tmpShiftReg ^= 0x100b;
        }
#line 81
      tmpShiftReg++;
      RandomLfsrP$shiftReg = tmpShiftReg;
      tmpShiftReg = tmpShiftReg ^ RandomLfsrP$mask;
    }
#line 84
    __nesc_atomic_end(__nesc_atomic); }
  return tmpShiftReg;
}

# 74 "SHA1M.nc"
static int SHA1M$SHA1$reset(SHA1Context *context)
{
  if (!context) {
      return shaNull;
    }

  context->Length_Low = 0;
  context->Length_High = 0;
  context->Message_Block_Index = 0;

  context->Intermediate_Hash[0] = 0x67452301;
  context->Intermediate_Hash[1] = 0xEFCDAB89;
  context->Intermediate_Hash[2] = 0x98BADCFE;
  context->Intermediate_Hash[3] = 0x10325476;
  context->Intermediate_Hash[4] = 0xC3D2E1F0;

  context->Computed = 0;
  context->Corrupted = 0;

  return shaSuccess;
}

#line 167
static int SHA1M$SHA1$update(SHA1Context *context, const uint8_t *message_array, uint32_t length)
{
  if (!length) {
      return shaSuccess;
    }

  if (!context || !message_array) {
      return shaNull;
    }

  if (context->Computed) {
      context->Corrupted = shaStateError;

      return shaStateError;
    }

  if (context->Corrupted) {
      return context->Corrupted;
    }
  while (length-- && ! context->Corrupted) {
      context->Message_Block[context->Message_Block_Index++] = 
      *message_array & 0xFF;

      context->Length_Low += 8;
      if (context->Length_Low == 0) {
          context->Length_High++;
          if (context->Length_High == 0) {

              context->Corrupted = 1;
            }
        }

      if (context->Message_Block_Index == 64) {
          SHA1M$SHA1ProcessMessageBlock(context);
        }

      message_array++;
    }

  return shaSuccess;
}

#line 230
static void SHA1M$SHA1ProcessMessageBlock(SHA1Context *context)
{
  const uint32_t K[4] = { 
  0x5A827999, 
  0x6ED9EBA1, 
  0x8F1BBCDC, 
  0xCA62C1D6 };

  uint8_t t;
  uint32_t temp;

  uint8_t s;
  uint32_t W[16];



  uint32_t A;
#line 246
  uint32_t B;
#line 246
  uint32_t C;
#line 246
  uint32_t D;
#line 246
  uint32_t E;




  for (t = 0; t < 16; t++) {
      W[t] = (uint32_t )context->Message_Block[t * 4] << 24;
      W[t] |= (uint32_t )context->Message_Block[t * 4 + 1] << 16;
      W[t] |= (uint32_t )context->Message_Block[t * 4 + 2] << 8;
      W[t] |= (uint32_t )context->Message_Block[t * 4 + 3];
    }







  A = context->Intermediate_Hash[0];
  B = context->Intermediate_Hash[1];
  C = context->Intermediate_Hash[2];
  D = context->Intermediate_Hash[3];
  E = context->Intermediate_Hash[4];

  for (t = 0; t < 20; t++) {

      s = t & 0x0f;
      if (t >= 16) {
          W[s] = ((((W[(s + 13) & 0x0f] ^ W[(s + 8) & 0x0f]) ^ W[(s + 2) & 0x0f]) ^ W[s]) << 1) | ((((W[(s + 13) & 0x0f] ^ W[(s + 8) & 0x0f]) ^ W[(s + 2) & 0x0f]) ^ W[s]) >> (32 - 1));
        }




      temp = ((A << 5) | (A >> (32 - 5))) + ((
      B & C) | (~B & D)) + E + W[s] + K[0];




      E = D;
      D = C;
      C = (B << 30) | (B >> (32 - 30));

      B = A;
      A = temp;
    }

  for (t = 20; t < 40; t++) {

      s = t & 0x0f;
      W[s] = ((((W[(s + 13) & 0x0f] ^ W[(s + 8) & 0x0f]) ^ W[(s + 2) & 0x0f]) ^ W[s]) << 1) | ((((W[(s + 13) & 0x0f] ^ W[(s + 8) & 0x0f]) ^ W[(s + 2) & 0x0f]) ^ W[s]) >> (32 - 1));




      temp = ((A << 5) | (A >> (32 - 5))) + ((B ^ C) ^ D) + E + W[s] + K[1];



      E = D;
      D = C;
      C = (B << 30) | (B >> (32 - 30));
      B = A;
      A = temp;
    }

  for (t = 40; t < 60; t++) {

      s = t & 0x0f;
      W[s] = ((((W[(s + 13) & 0x0f] ^ W[(s + 8) & 0x0f]) ^ W[(s + 2) & 0x0f]) ^ W[s]) << 1) | ((((W[(s + 13) & 0x0f] ^ W[(s + 8) & 0x0f]) ^ W[(s + 2) & 0x0f]) ^ W[s]) >> (32 - 1));




      temp = ((A << 5) | (A >> (32 - 5))) + (((
      B & C) | (B & D)) | (C & D)) + E + W[s] + K[2];




      E = D;
      D = C;
      C = (B << 30) | (B >> (32 - 30));
      B = A;
      A = temp;
    }

  for (t = 60; t < 80; t++) {

      s = t & 0x0f;
      W[s] = ((((W[(s + 13) & 0x0f] ^ W[(s + 8) & 0x0f]) ^ W[(s + 2) & 0x0f]) ^ W[s]) << 1) | ((((W[(s + 13) & 0x0f] ^ W[(s + 8) & 0x0f]) ^ W[(s + 2) & 0x0f]) ^ W[s]) >> (32 - 1));




      temp = ((A << 5) | (A >> (32 - 5))) + ((B ^ C) ^ D) + E + W[s] + K[3];



      E = D;
      D = C;
      C = (B << 30) | (B >> (32 - 30));
      B = A;
      A = temp;
    }

  context->Intermediate_Hash[0] += A;
  context->Intermediate_Hash[1] += B;
  context->Intermediate_Hash[2] += C;
  context->Intermediate_Hash[3] += D;
  context->Intermediate_Hash[4] += E;

  context->Message_Block_Index = 0;
}

#line 115
static int SHA1M$SHA1$digest(SHA1Context *context, uint8_t Message_Digest[20])
{
  uint8_t i;

  if (!context || !Message_Digest) {
      return shaNull;
    }

  if (context->Corrupted) {
      return context->Corrupted;
    }

  if (! context->Computed) {
      SHA1M$SHA1PadMessage(context);
      for (i = 0; i < 64; ++i) {

          context->Message_Block[i] = 0;
        }
      context->Length_Low = 0;
      context->Length_High = 0;
      context->Computed = 1;
    }


  for (i = 0; i < 20; ++i) {
      Message_Digest[i] = context->Intermediate_Hash[i >> 2]
       >> 8 * (3 - (i & 0x03));
    }

  return shaSuccess;
}

# 180 "NNM.nc"
static void NNM$NN_Decode(NN_DIGIT *a, NN_UINT digits, unsigned char *b, NN_UINT len)
{
  NN_DIGIT t;
  int j;
  unsigned int i;
#line 184
  unsigned int u;

  for (i = 0, j = len - 1; i < digits && j >= 0; i++) {
      t = 0;
      for (u = 0; j >= 0 && u < 16; j--, u += 8) 
        t |= (NN_DIGIT )b[j] << u;
      a[i] = t;
    }

  for (; i < digits; i++) 
    a[i] = 0;
}

#line 3504
static void NNM$NN$ModBarrett(NN_DIGIT *a, NN_DIGIT *b, NN_UINT bDigits, NN_DIGIT *c, NN_UINT cDigits)
{

  NN_DIGIT q2[2 * (160 / 16 + 1) + 6];
#line 3507
  NN_DIGIT q1[160 / 16 + 1 + 4];
#line 3507
  NN_DIGIT r2[2 * (160 / 16 + 1) + 8];
#line 3507
  NN_DIGIT r1[160 / 16 + 1 + 4];
#line 3507
  NN_DIGIT m[160 / 16 + 1 + 4];


  memset(q2, 0, (2 * (160 / 16 + 1) + 6) * (16 / 8));
  memcpy(q1, b + NNM$pBarrett->km - 1, (bDigits - NNM$pBarrett->km + 1) * (16 / 8));
  memset(q1 + bDigits - NNM$pBarrett->km + 1, 0, 160 / 16 + 1 + 4 - bDigits + NNM$pBarrett->km - 1);
  memcpy(r1, b, (NNM$pBarrett->km + 1) * (16 / 8));
  memset(r1 + NNM$pBarrett->km + 1, 0, (160 / 16 + 1 + 4 - NNM$pBarrett->km - 1) * (16 / 8));
  memcpy(m, c, cDigits * (16 / 8));
  memset(m + cDigits, 0, 160 / 16 + 1 + 4 - cDigits);


  NNM$NN_Mult(q2, q1, NNM$pBarrett->mu, NNM$pBarrett->mu_len);

  NNM$NN_Mult(r2, q2 + NNM$pBarrett->km + 1, m, NNM$pBarrett->mu_len);
  memset(r2 + NNM$pBarrett->km + 1, 0, (2 * (160 / 16 + 1) + 8 - NNM$pBarrett->km - 1) * (16 / 8));
  if (NNM$NN_Cmp(r1, r2, NNM$pBarrett->km + 1) < 0) {
    r1[NNM$pBarrett->km + 1] = 1;
    }
#line 3525
  NNM$NN_Sub(r1, r1, r2, NNM$pBarrett->km + 2);

  while (NNM$NN_Cmp(r1, m, cDigits) >= 0) 
    NNM$NN_Sub(r1, r1, m, cDigits);

  memcpy(a, r1, cDigits * (16 / 8));
}

#line 2860
static void NNM$NN_ModMult(NN_DIGIT *a, NN_DIGIT *b, NN_DIGIT *c, NN_DIGIT *d, NN_UINT digits)
{


  NN_DIGIT q2[2 * (160 / 16 + 1) + 8];
#line 2864
  NN_DIGIT x[2 * (160 / 16 + 1) + 8];
#line 2864
  NN_DIGIT r2[2 * (160 / 16 + 1) + 8];
#line 2864
  NN_DIGIT m[160 / 16 + 1 + 4];

  if (digits > 4) {
    memset(x + 2 * digits - 8, 0, (2 * (160 / 16 + 1) - 2 * digits + 16) * (16 / 8));
    }
  else {
#line 2869
    memset(x, 0, (2 * (160 / 16 + 1) + 8) * (16 / 8));
    }

  NNM$NN_Mult(x, b, c, digits);
  memcpy(m, d, digits * (16 / 8));
  memset(m + digits, 0, (160 / 16 + 1 + 4 - digits) * (16 / 8));


  NNM$NN_Mult(q2, x + NNM$pBarrett->km - 1, NNM$pBarrett->mu, NNM$pBarrett->mu_len);

  NNM$NN_Mult(r2, q2 + NNM$pBarrett->km + 1, m, NNM$pBarrett->mu_len);
  memset(r2 + NNM$pBarrett->km + 1, 0, (2 * (160 / 16 + 1) + 8 - NNM$pBarrett->km - 1) * (16 / 8));
  memset(x + NNM$pBarrett->km + 1, 0, (2 * (160 / 16 + 1) + 8 - NNM$pBarrett->km - 1) * (16 / 8));

  if (NNM$NN_Cmp(x, r2, NNM$pBarrett->km + 1) < 0) {
    x[NNM$pBarrett->km + 1] = 1;
    }
#line 2885
  NNM$NN_Sub(x, x, r2, NNM$pBarrett->km + 2);

  while (NNM$NN_Cmp(x, m, digits) >= 0) 
    NNM$NN_Sub(x, x, m, digits);

  memcpy(a, x, digits * (16 / 8));
}

# 386 "ECDSAM.nc"
static __attribute((noinline)) uint8_t ECDSAM$ECDSA$verify(uint8_t *msg, uint8_t len, NN_DIGIT *r, NN_DIGIT *s, Point *Q)
{

  uint8_t sha1sum[20];
  NN_DIGIT sha1tmp[20 / (16 / 8)];
  SHA1Context ctx;
  NN_DIGIT w[160 / 16 + 1];
  NN_DIGIT u1[160 / 16 + 1];
  NN_DIGIT u2[160 / 16 + 1];
  NN_DIGIT digest[160 / 16 + 1];
  NN_UINT result_bit_len;
#line 396
  NN_UINT order_bit_len;



  Point final;

  if (ECDSAM$NN$Cmp(r, ECDSAM$param->r, 160 / 16 + 1) >= 0) {
    return 3;
    }
#line 404
  if (ECDSAM$NN$Zero(r, 160 / 16 + 1) == 1) {
    return 4;
    }
#line 406
  if (ECDSAM$NN$Cmp(s, ECDSAM$param->r, 160 / 16 + 1) >= 0) {
    return 5;
    }
#line 408
  if (ECDSAM$NN$Zero(s, 160 / 16 + 1) == 1) {
    return 6;
    }

  ECDSAM$NN$BarrettSetBuf(&ECDSAM$Bbuf);


  ECDSAM$NN$ModInv(w, s, ECDSAM$param->r, 160 / 16 + 1);


  memset(sha1sum, 0, 20);
  memset(digest, 0, (160 / 16 + 1) * (16 / 8));
  ECDSAM$SHA1$reset(&ctx);
  ECDSAM$SHA1$update(&ctx, msg, len);
  ECDSAM$SHA1$digest(&ctx, sha1sum);
  ECDSAM$NN$Decode(sha1tmp, 20 / (16 / 8), sha1sum, 20);
  result_bit_len = ECDSAM$NN$Bits(sha1tmp, 20 / (16 / 8));
  order_bit_len = ECDSAM$NN$Bits(ECDSAM$param->r, 160 / 16 + 1);
  if (result_bit_len > order_bit_len) {
      ECDSAM$NN$Mod(digest, sha1tmp, 20 / (16 / 8), ECDSAM$param->r, 160 / 16 + 1);
    }
  else 
#line 428
    {
      ECDSAM$NN$Assign(digest, sha1tmp, 20 / (16 / 8));
      if (result_bit_len == order_bit_len) {
        ECDSAM$NN$ModSmall(digest, ECDSAM$param->r, 160 / 16 + 1);
        }
    }
  ECDSAM$NN$ModMult(u1, digest, w, ECDSAM$param->r, 160 / 16 + 1);

  ECDSAM$NN$ModMult(u2, r, w, ECDSAM$param->r, 160 / 16 + 1);


  ECDSAM$ECC$BarrettSetBuf();



  ECDSAM$shamir(&final, u1, u2);
#line 455
  ECDSAM$NN$BarrettSetBuf(&ECDSAM$Bbuf);

  result_bit_len = ECDSAM$NN$Bits(final.x, 160 / 16 + 1);
  order_bit_len = ECDSAM$NN$Bits(ECDSAM$param->r, 160 / 16 + 1);
  if (result_bit_len > order_bit_len) {
      ECDSAM$NN$Mod(w, final.x, 160 / 16 + 1, ECDSAM$param->r, 160 / 16 + 1);
    }
  else 
#line 461
    {
      ECDSAM$NN$Assign(w, final.x, 160 / 16 + 1);
      if (result_bit_len == order_bit_len) {
        ECDSAM$NN$ModSmall(w, ECDSAM$param->r, 160 / 16 + 1);
        }
    }
  if (ECDSAM$NN$Cmp(w, r, 160 / 16 + 1) == 0) {
      return 1;
    }
  else 
#line 469
    {
      return 2;
    }
}

# 48 "testECDSAM.nc"
static void testECDSAM$init_data(void )
#line 48
{




  uint8_t j;


  testECDSAM$pass = 0;
  testECDSAM$t = 0;


  memset(testECDSAM$message, 0, 52);

  memset(testECDSAM$PrivateKey, 0, (160 / 16 + 1) * (16 / 8));

  memset(testECDSAM$PublicKey.x, 0, (160 / 16 + 1) * (16 / 8));
  memset(testECDSAM$PublicKey.y, 0, (160 / 16 + 1) * (16 / 8));

  memset(testECDSAM$r, 0, (160 / 16 + 1) * (16 / 8));
  memset(testECDSAM$s, 0, (160 / 16 + 1) * (16 / 8));



  for (j = 0; j < 52; j++) {
      testECDSAM$message[j] = (uint8_t )testECDSAM$Random$rand16();
    }





  testECDSAM$ECC$init();

  testECDSAM$gen_PrivateKey();
}

# 158 "/opt/tinyos-2.0.2/tos/lib/serial/SerialActiveMessageP.nc"
static am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$type(message_t *amsg)
#line 158
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$getHeader(amsg);

#line 160
  return __nesc_ntoh_uint8(header->type.data);
}

# 347 "/opt/tinyos-2.0.2/tos/lib/serial/SerialP.nc"
static void SerialP$testOff(void )
#line 347
{
  bool turnOff = FALSE;

#line 349
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 349
    {
      if (SerialP$txState == SerialP$TXSTATE_INACTIVE && 
      SerialP$rxState == SerialP$RXSTATE_INACTIVE) {
          turnOff = TRUE;
        }
    }
#line 354
    __nesc_atomic_end(__nesc_atomic); }
  if (turnOff) {
      SerialP$stopDoneTask$postTask();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 357
        SerialP$offPending = FALSE;
#line 357
        __nesc_atomic_end(__nesc_atomic); }
    }
  else {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 360
        SerialP$offPending = TRUE;
#line 360
        __nesc_atomic_end(__nesc_atomic); }
    }
}

# 86 "/opt/tinyos-2.0.2/tos/lib/serial/HdlcTranslateC.nc"
static error_t HdlcTranslateC$SerialFrameComm$putDelimiter(void )
#line 86
{
  HdlcTranslateC$state.sendEscape = 0;
  HdlcTranslateC$m_data = HDLC_FLAG_BYTE;
  return HdlcTranslateC$UartStream$send(&HdlcTranslateC$m_data, 1);
}

# 137 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/Msp430UartP.nc"
static error_t /*Msp430Uart1P.UartP*/Msp430UartP$0$UartStream$send(uint8_t *buf, uint16_t len)
#line 137
{
  if (len == 0) {
    return FAIL;
    }
  else {
#line 140
    if (/*Msp430Uart1P.UartP*/Msp430UartP$0$m_tx_buf) {
      return EBUSY;
      }
    }
#line 142
  /*Msp430Uart1P.UartP*/Msp430UartP$0$m_tx_buf = buf;
  /*Msp430Uart1P.UartP*/Msp430UartP$0$m_tx_len = len;
  /*Msp430Uart1P.UartP*/Msp430UartP$0$m_tx_pos = 0;
  /*Msp430Uart1P.UartP*/Msp430UartP$0$Usart$tx(buf[/*Msp430Uart1P.UartP*/Msp430UartP$0$m_tx_pos++]);
  return SUCCESS;
}

# 62 "/opt/tinyos-2.0.2/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$fireTimers(uint32_t now)
{
  uint8_t num;

  for (num = 0; num < /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 79
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$Timer$fired(num);
            }
        }
    }
  /*HilTimerMilliC.VirtualizeTimerC*/VirtualizeTimerC$0$updateFromTimer$postTask();
}

# 136 "/opt/tinyos-2.0.2/tos/lib/timer/TransformAlarmC.nc"
static void /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$Alarm$startAt(/*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type t0, /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$to_size_type dt)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_t0 = t0;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$m_dt = dt;
      /*HilTimerMilliC.AlarmMilli32C.Transform*/TransformAlarmC$0$set_alarm();
    }
#line 143
    __nesc_atomic_end(__nesc_atomic); }
}

# 96 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
 __attribute((wakeup)) __attribute((interrupt(6))) void sig_UART1RX_VECTOR(void )
#line 96
{
  uint8_t temp = U1RXBUF;

#line 98
  HplMsp430Usart1P$Interrupts$rxDone(temp);
}

# 402 "/opt/tinyos-2.0.2/tos/lib/serial/SerialP.nc"
static void SerialP$rx_state_machine(bool isDelimeter, uint8_t data)
#line 402
{

  switch (SerialP$rxState) {

      case SerialP$RXSTATE_NOSYNC: 
        if (isDelimeter) {
            SerialP$rxInit();
            SerialP$rxState = SerialP$RXSTATE_PROTO;
          }
      break;

      case SerialP$RXSTATE_PROTO: 
        if (!isDelimeter) {
            SerialP$rxCRC = crcByte(SerialP$rxCRC, data);
            SerialP$rxState = SerialP$RXSTATE_TOKEN;
            SerialP$rxProto = data;
            if (!SerialP$valid_rx_proto(SerialP$rxProto)) {
              goto nosync;
              }
            if (SerialP$rxProto != SERIAL_PROTO_PACKET_ACK) {
                goto nosync;
              }
            if (SerialP$ReceiveBytePacket$startPacket() != SUCCESS) {
                goto nosync;
              }
          }
      break;

      case SerialP$RXSTATE_TOKEN: 
        if (isDelimeter) {
            goto nosync;
          }
        else {
            SerialP$rxSeqno = data;
            SerialP$rxCRC = crcByte(SerialP$rxCRC, SerialP$rxSeqno);
            SerialP$rxState = SerialP$RXSTATE_INFO;
          }
      break;

      case SerialP$RXSTATE_INFO: 
        if (SerialP$rxByteCnt < SerialP$SERIAL_MTU) {
            if (isDelimeter) {
                if (SerialP$rxByteCnt >= 2) {
                    if (SerialP$rx_current_crc() == SerialP$rxCRC) {
                        SerialP$ReceiveBytePacket$endPacket(SUCCESS);
                        SerialP$ack_queue_push(SerialP$rxSeqno);
                        goto nosync;
                      }
                    else {
                        goto nosync;
                      }
                  }
                else {
                    goto nosync;
                  }
              }
            else {
                if (SerialP$rxByteCnt >= 2) {
                    SerialP$ReceiveBytePacket$byteReceived(SerialP$rx_buffer_top());
                    SerialP$rxCRC = crcByte(SerialP$rxCRC, SerialP$rx_buffer_pop());
                  }
                SerialP$rx_buffer_push(data);
                SerialP$rxByteCnt++;
              }
          }
        else 

          {
            goto nosync;
          }
      break;

      default: 
        goto nosync;
    }
  goto done;

  nosync: 

    SerialP$rxInit();
  SerialP$SerialFrameComm$resetReceive();
  SerialP$ReceiveBytePacket$endPacket(FAIL);
  if (SerialP$offPending) {
      SerialP$rxState = SerialP$RXSTATE_INACTIVE;
      SerialP$testOff();
    }
  else {
    if (isDelimeter) {
        SerialP$rxState = SerialP$RXSTATE_PROTO;
      }
    }
  done: ;
}

# 80 "/opt/tinyos-2.0.2/tos/system/crc.h"
static uint16_t crcByte(uint16_t crc, uint8_t b)
#line 80
{
  crc = (uint8_t )(crc >> 8) | (crc << 8);
  crc ^= b;
  crc ^= (uint8_t )(crc & 0xff) >> 4;
  crc ^= crc << 12;
  crc ^= (crc & 0xff) << 5;
  return crc;
}

# 290 "/opt/tinyos-2.0.2/tos/lib/serial/SerialDispatcherP.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$endPacket(error_t result)
#line 290
{
  uint8_t postsignalreceive = FALSE;

  /* atomic removed: atomic calls only */
#line 292
  {
    if (!/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskPending && result == SUCCESS) {
        postsignalreceive = TRUE;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskPending = TRUE;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskType = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvType;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskWhich = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.which;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskBuf = (message_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveBuffer;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskSize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvIndex;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveBufferSwap();
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$RECV_STATE_IDLE;
      }
  }
  if (postsignalreceive) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTask$postTask();
    }
}

# 101 "/opt/tinyos-2.0.2/tos/chips/msp430/usart/HplMsp430Usart1P.nc"
 __attribute((wakeup)) __attribute((interrupt(4))) void sig_UART1TX_VECTOR(void )
#line 101
{
  HplMsp430Usart1P$Interrupts$txDone();
}

# 92 "/opt/tinyos-2.0.2/tos/lib/serial/HdlcTranslateC.nc"
static error_t HdlcTranslateC$SerialFrameComm$putData(uint8_t data)
#line 92
{
  if (data == HDLC_CTLESC_BYTE || data == HDLC_FLAG_BYTE) {
      HdlcTranslateC$state.sendEscape = 1;
      HdlcTranslateC$txTemp = data ^ 0x20;
      HdlcTranslateC$m_data = HDLC_CTLESC_BYTE;
    }
  else {
      HdlcTranslateC$m_data = data;
    }
  return HdlcTranslateC$UartStream$send(&HdlcTranslateC$m_data, 1);
}

