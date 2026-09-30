/*
 * QEMU SiI3114A PCI to Serial ATA Controller Emulation
 *
 * Copyright (C) 2017 BALATON Zoltan <balaton@eik.bme.hu>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 */

/* For documentation on this and similar cards see:
 * http://wiki.osdev.org/User:Quok/Silicon_Image_Datasheets
 */

#include "qemu/osdep.h"
#include "hw/ide/pci.h"
#include "hw/ide/sii3114.h"
#include "qemu/module.h"
#include "trace.h"
#include "qom/object.h"
#include "ide-internal.h"

#define TYPE_SII3114_PCI "sii3114"
#define SII3114_PORTS 4
#define SII3114_MMIO_SIZE 0x400
#define SII3114_INTR_STEERING (1U << 1)
#define SII3114_IRQ_MASK (0x0fU << 22)
OBJECT_DECLARE_SIMPLE_TYPE(SiI3114PCIState, SII3114_PCI)

typedef struct SiI3114Regs {
    uint32_t confstat;
    uint32_t scontrol;
    uint16_t sien;
    uint8_t swdata;
    uint8_t xfer_mode;
} SiI3114Regs;

struct SiI3114PCIState {
    PCIIDEState i;
    MemoryRegion mmio;
    IDEBus bus[2];
    BMDMAState bmdma[2];
    SiI3114Regs regs[SII3114_PORTS];
    uint32_t syscfg;
};

static const uint16_t sii3114_tf[SII3114_PORTS] = {
    0x80, 0xc0, 0x280, 0x2c0,
};
static const uint16_t sii3114_ctl[SII3114_PORTS] = {
    0x8a, 0xca, 0x28a, 0x2ca,
};
static const uint16_t sii3114_bmdma[SII3114_PORTS] = {
    0x00, 0x08, 0x200, 0x208,
};
static const uint16_t sii3114_scr[SII3114_PORTS] = {
    0x100, 0x180, 0x300, 0x380,
};
static const uint16_t sii3114_sien[SII3114_PORTS] = {
    0x148, 0x1c8, 0x348, 0x3c8,
};
static const uint16_t sii3114_xfer_mode[SII3114_PORTS] = {
    0xb4, 0xf4, 0x2b4, 0x2f4,
};

static BMDMAState *sii3114_get_bmdma(SiI3114PCIState *d, unsigned int port)
{
    return port < 2 ? &d->i.bmdma[port] : &d->bmdma[port - 2];
}

IDEBus *sii3114_get_bus(PCIDevice *dev, unsigned int port)
{
    SiI3114PCIState *d = SII3114_PCI(dev);

    g_assert(port < SII3114_PORTS);
    return port < 2 ? &d->i.bus[port] : &d->bus[port - 2];
}

static void sii3114_update_irq(SiI3114PCIState *s);

/* The sii3114_reg_read and sii3114_reg_write functions implement the
 * Internal Register Space - BAR5 (section 6.7 of the data sheet).
 */

static uint64_t sii3114_reg_read(void *opaque, hwaddr addr,
                                unsigned int size)
{
    SiI3114PCIState *d = opaque;
    uint64_t val = 0;
    unsigned int port;

    if (addr == 0x48) {
        return d->syscfg;
    }
    if (addr == 0x10 || addr == 0x18) {
        unsigned int first = addr == 0x10 ? 0 : 2;
        BMDMAState *bm0 = sii3114_get_bmdma(d, first);
        BMDMAState *bm1 = sii3114_get_bmdma(d, first + 1);

        val = bm0->cmd;
        val |= d->regs[first].confstat & (1U << 11) ? 1U << 4 : 0;
        val |= d->regs[first + 1].confstat & (1U << 11) ? 1U << 6 : 0;
        val |= bm1->status & BM_STATUS_INT ? 1U << 14 : 0;
        val |= (uint32_t)bm0->status << 16;
        val |= (uint32_t)bm1->status << 24;
        trace_sii3112_read(size, addr, val);
        return val;
    }
    for (port = 0; port < SII3114_PORTS; port++) {
        IDEBus *bus = sii3114_get_bus(PCI_DEVICE(d), port);
        BMDMAState *bm = sii3114_get_bmdma(d, port);

        if (addr == sii3114_tf[port] + 0x20) {
            return d->regs[port].confstat;
        }
        if (addr == sii3114_scr[port]) {
            return d->regs[port].scontrol;
        }
        if (addr == sii3114_scr[port] + 4) {
            return bus->ifs[0].blk ? 0x113 : 0;
        }
        if (addr == sii3114_sien[port]) {
            return (uint32_t)d->regs[port].sien << 16;
        }
        if (addr == sii3114_xfer_mode[port]) {
            return d->regs[port].xfer_mode;
        }
        if (port == 2 && addr == 0x200) {
            return SII3114_INTR_STEERING;
        }
        if (addr == sii3114_bmdma[port]) {
            val = bm->cmd;
            if (port == 2) {
                val |= SII3114_INTR_STEERING;
            }
            return val;
        }
        if (addr == sii3114_bmdma[port] + 1) {
            return d->regs[port].swdata;
        }
        if (addr == sii3114_bmdma[port] + 2) {
            return bm->status;
        }
        if (addr == sii3114_bmdma[port] + 3) {
            return 0;
        }
        if (addr >= sii3114_bmdma[port] + 4 &&
            addr < sii3114_bmdma[port] + 8) {
            return bmdma_addr_ioport_ops.read(bm,
                                      addr - sii3114_bmdma[port] - 4, size);
        }
        if (addr >= sii3114_tf[port] && addr < sii3114_tf[port] + 8) {
            return pci_ide_data_le_ops.read(bus, addr - sii3114_tf[port], size);
        }
        if (addr == sii3114_ctl[port]) {
            return pci_ide_cmd_le_ops.read(bus, 2, size);
        }
    }

    trace_sii3112_read(size, addr, val);
    return 0;
}

static void sii3114_reg_write(void *opaque, hwaddr addr,
                              uint64_t val, unsigned int size)
{
    SiI3114PCIState *d = opaque;
    unsigned int port;

    trace_sii3112_write(size, addr, val);
    if (addr == 0x48) {
        d->syscfg = val & ~SII3114_IRQ_MASK;
        return;
    }
    for (port = 0; port < SII3114_PORTS; port++) {
        IDEBus *bus = sii3114_get_bus(PCI_DEVICE(d), port);
        BMDMAState *bm = sii3114_get_bmdma(d, port);

        if (addr == sii3114_tf[port] + 0x20) {
            d->regs[port].confstat = val;
            sii3114_update_irq(d);
            return;
        }
        if (addr == sii3114_bmdma[port]) {
            bmdma_cmd_writeb(bm, val);
            return;
        }
        if (addr == sii3114_bmdma[port] + 1) {
            d->regs[port].swdata = val & 0x3f;
            return;
        }
        if (addr == sii3114_bmdma[port] + 2) {
            bmdma_status_writeb(bm, val);
            return;
        }
        if (addr >= sii3114_bmdma[port] + 4 &&
            addr < sii3114_bmdma[port] + 8) {
            bmdma_addr_ioport_ops.write(bm,
                                 addr - sii3114_bmdma[port] - 4, val, size);
            return;
        }
        if (addr >= sii3114_tf[port] && addr < sii3114_tf[port] + 8) {
            pci_ide_data_le_ops.write(bus, addr - sii3114_tf[port], val, size);
            return;
        }
        if (addr == sii3114_ctl[port]) {
            pci_ide_cmd_le_ops.write(bus, 2, val, size);
            return;
        }
        if (addr == sii3114_scr[port]) {
            d->regs[port].scontrol = val & 0xfff;
            if (val & 1) {
                ide_bus_reset(bus, IDE_RESET_HARDWARE);
                d->regs[port].confstat &= ~(1U << 11);
                sii3114_update_irq(d);
            }
            return;
        }
        if (addr == sii3114_sien[port]) {
            d->regs[port].sien = (val >> 16) & 0x3eed;
            return;
        }
        if (addr == sii3114_xfer_mode[port]) {
            d->regs[port].xfer_mode = val & 0x33;
            return;
        }
    }
}

static const MemoryRegionOps sii3114_reg_ops = {
    .read = sii3114_reg_read,
    .write = sii3114_reg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* the PCI irq level is the logical OR of the two channels */
static void sii3114_update_irq(SiI3114PCIState *s)
{
    int i, set = 0;

    for (i = 0; i < SII3114_PORTS; i++) {
        set |= s->regs[i].confstat & (1UL << 11);
    }
    pci_set_irq(PCI_DEVICE(s), (set ? 1 : 0));
}

static void sii3114_set_irq(void *opaque, int channel, int level)
{
    SiI3114PCIState *s = opaque;

    trace_sii3112_set_irq(channel, level);
    if (level) {
        s->regs[channel].confstat |= (1UL << 11);
    } else {
        s->regs[channel].confstat &= ~(1UL << 11);
    }

    if (channel == 0 || channel == 1) {
        BMDMAState *bm = sii3114_get_bmdma(s, channel);
        bm->status = level ? bm->status | BM_STATUS_INT :
                             bm->status & ~BM_STATUS_INT;
    }

    sii3114_update_irq(s);
}

static void sii3114_reset(DeviceState *dev)
{
    SiI3114PCIState *s = SII3114_PCI(dev);
    int i;

    for (i = 0; i < SII3114_PORTS; i++) {
        s->regs[i].confstat = 0x6515 << 16;
        ide_bus_reset(sii3114_get_bus(PCI_DEVICE(s), i),
                      IDE_RESET_HARDWARE);
    }
}

static void sii3114_pci_realize(PCIDevice *dev, Error **errp)
{
    SiI3114PCIState *d = SII3114_PCI(dev);
    PCIIDEState *s = PCI_IDE(dev);
    DeviceState *ds = DEVICE(dev);
    MemoryRegion *mr;
    int i;

    pci_config_set_interrupt_pin(dev->config, 1);
    pci_set_byte(dev->config + PCI_CACHE_LINE_SIZE, 8);

    /* BAR5 is in PCI memory space */
    memory_region_init_io(&d->mmio, OBJECT(d), &sii3114_reg_ops, d,
                         "sii3114.bar5", SII3114_MMIO_SIZE);
    pci_register_bar(dev, 5, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->mmio);

    /* BAR0-BAR4 are PCI I/O space aliases into BAR5 */
    mr = g_new(MemoryRegion, 1);
    memory_region_init_alias(mr, OBJECT(d), "sii3114.bar0", &d->mmio, 0x80, 8);
    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_IO, mr);
    mr = g_new(MemoryRegion, 1);
    memory_region_init_alias(mr, OBJECT(d), "sii3114.bar1", &d->mmio, 0x88, 4);
    pci_register_bar(dev, 1, PCI_BASE_ADDRESS_SPACE_IO, mr);
    mr = g_new(MemoryRegion, 1);
    memory_region_init_alias(mr, OBJECT(d), "sii3114.bar2", &d->mmio, 0xc0, 8);
    pci_register_bar(dev, 2, PCI_BASE_ADDRESS_SPACE_IO, mr);
    mr = g_new(MemoryRegion, 1);
    memory_region_init_alias(mr, OBJECT(d), "sii3114.bar3", &d->mmio, 0xc8, 4);
    pci_register_bar(dev, 3, PCI_BASE_ADDRESS_SPACE_IO, mr);
    mr = g_new(MemoryRegion, 1);
    memory_region_init_alias(mr, OBJECT(d), "sii3114.bar4", &d->mmio, 0, 16);
    pci_register_bar(dev, 4, PCI_BASE_ADDRESS_SPACE_IO, mr);

    qdev_init_gpio_in(ds, sii3114_set_irq, SII3114_PORTS);
    for (i = 0; i < SII3114_PORTS; i++) {
        IDEBus *bus = sii3114_get_bus(dev, i);
        BMDMAState *bm = sii3114_get_bmdma(d, i);

        ide_bus_init(bus, sizeof(*bus), ds, i, 1);
        ide_bus_init_output_irq(bus, qdev_get_gpio_in(ds, i));
        bmdma_init(bus, bm, s);
        ide_bus_register_restart_cb(bus);
    }
}

static void sii3114_pci_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *pd = PCI_DEVICE_CLASS(klass);

    pd->vendor_id = 0x1095;
    pd->device_id = 0x3114;
    pd->class_id = PCI_CLASS_STORAGE_RAID;
    pd->revision = 1;
    pd->realize = sii3114_pci_realize;
    device_class_set_legacy_reset(dc, sii3114_reset);
    dc->desc = "SiI3114A SATA controller";
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
}

static const TypeInfo sii3114_pci_info = {
    .name = TYPE_SII3114_PCI,
    .parent = TYPE_PCI_IDE,
    .instance_size = sizeof(SiI3114PCIState),
    .class_init = sii3114_pci_class_init,
};

static void sii3114_register_types(void)
{
    type_register_static(&sii3114_pci_info);
}

type_init(sii3114_register_types)
