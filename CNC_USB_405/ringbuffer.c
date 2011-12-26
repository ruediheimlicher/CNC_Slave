// AT90USB/ringbuffer.c
// Simple Ring-Buffer (FIFO) for Elements of type Q
// S. Salewski, 19-MAR-2007

/*
t-> o
    o <-w
    x 
    x <-r
b-> x
*/

#include <stdint.h>
#include "ringbuffer.h"

static Q buf[BufElements];
uint16_t RB_Entries;

#define t &buf[BufElements - 1]
#define b &buf[0]

//Q *t = &buf[BufElements - 1];
//Q *b = &buf[0];
Q *r; // position from where we can read (if RB_Entries > 0)
Q *w; // next free position (if RB_Entries < BufElements))

void
RB_Init(void)
{
  r = b;
  w = b;
  RB_Entries = 0;
}

Q
RB_Read(void)
{
//Assert(RB_Entries > 0);
  RB_Entries--;
  if (r > t) r = b;
  return *r++;
}

void
RB_Write(Q el)
{
//Assert(RB_Entries < BufElements);
  RB_Entries++;
  if (w > t) w = b;
  *w++ = el;
}
