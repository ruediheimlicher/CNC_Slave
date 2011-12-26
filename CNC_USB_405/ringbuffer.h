// AT90USB/ringbuffer.h
// Simple Ring-Buffer (FIFO) for Elements of type Q
// S. Salewski, 20-MAR-2007

#ifndef _RING_BUFFER_H_
#define _RING_BUFFER_H_

#include <stdint.h>

#define BufElements 1024
#define Q uint16_t

extern uint16_t RB_Entries;

#define RB_FreeSpace() (BufElements - RB_Entries)
#define RB_IsFull() (RB_Entries == BufElements)
#define RB_IsEmpty() (RB_Entries == 0)

void RB_Init(void);
void RB_Write(Q el);
Q RB_Read(void);

#endif
