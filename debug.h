#ifndef _DEBUG_H_
#define _DEBUG_H_

#ifdef DEBUGMSG
#include "serial.h"
#define DEBUG(args...)	fprintf(&serial_out,args)
#define DEBUGP(args...)	fprintf_P(&serial_out,args)
#else
#define DEBUG(args...)
#define DEBUGP(args...)
#endif

#endif
