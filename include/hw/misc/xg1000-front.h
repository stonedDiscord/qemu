#ifndef HW_MISC_XG1000_FRONT_H
#define HW_MISC_XG1000_FRONT_H
#include "system/memory.h"
#include "hw/core/irq.h"
typedef struct XG1000Front XG1000Front;
XG1000Front *xg1000_front_new(void);
void xg1000_front_init(XG1000Front *front, MemoryRegion *sysmem, qemu_irq irq);
#endif
