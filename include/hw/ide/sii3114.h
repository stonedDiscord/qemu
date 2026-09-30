#ifndef HW_IDE_SII3114_H
#define HW_IDE_SII3114_H

#include "hw/ide/ide-bus.h"
#include "hw/pci/pci_device.h"

IDEBus *sii3114_get_bus(PCIDevice *dev, unsigned int port);

#endif
