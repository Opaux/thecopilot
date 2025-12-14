#ifndef MMAP_XIL_IO_H
#define MMAP_XIL_IO_H

// MUST include xil_types.h because the original bare-metal functions use types like u32 and UINTPTR.
#include "xil_types.h"

// Re-implementation of Xil_Out32 for Linux userspace.
static inline void Xil_Out32(UINTPTR Address, u32 Value)
{
    volatile unsigned int *local_ptr = (volatile unsigned int *)Address;
    *local_ptr = Value;
}

//Re-implementation of Xil_In32 for Linux userspace.
static inline u32 Xil_In32(UINTPTR Address)
{
    volatile unsigned int *local_ptr = (volatile unsigned int *)Address;
    return *local_ptr;
}

#endif // MMAP_XIL_IO_H
