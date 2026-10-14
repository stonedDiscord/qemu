/*
 * XG1000 VW2010/VW2005 PCI register and memory interface.
 * Local experimental model, based on vw2010_module.ko accesses.
 * The codec processors, firmware execution and DMA are not implemented.
 */
#include "qemu/osdep.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_device.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qom/object.h"

#define TYPE_VW2010 "vw2010"
OBJECT_DECLARE_SIMPLE_TYPE(VW2010State, VW2010)

#define VW_BAR_SIZE 0x40000

struct VW2010State {
    PCIDevice parent_obj;
    MemoryRegion bar;
    uint8_t regs[VW_BAR_SIZE];
    GHashTable *ram[2];
};

/* Read and write register banks differ by one 0x1000-byte page. */
static hwaddr vw_reg_addr(hwaddr addr)
{
    switch (addr & ~0xfffULL) {
    case 0x6000:
    case 0x16000:
        return addr - 0x1000;
    default:
        return addr;
    }
}

static uint32_t vw_ram_addr(VW2010State *s, unsigned bank)
{
    unsigned port = bank ? 0x1802e : 0xa1ae;

    return lduw_le_p(s->regs + port) |
           ((uint32_t)(lduw_le_p(s->regs + port + 2) & 0x7fff) << 16);
}

static uint64_t vw_read(void *opaque, hwaddr addr, unsigned size)
{
    VW2010State *s = opaque;
    uint64_t value = 0;

    for (unsigned i = 0; i < size; i++) {
        hwaddr a = addr + i;
        unsigned bank = a >= 0x17040 && a < 0x17044;
        unsigned port = bank ? 0x17040 : 0xb1b2;
        uint8_t byte;

        if (a >= port && a < port + 4) {
            uint32_t word = GPOINTER_TO_UINT(g_hash_table_lookup(
                s->ram[bank], GUINT_TO_POINTER(vw_ram_addr(s, bank))));
            byte = word >> ((a - port) * 8);
        } else {
            byte = s->regs[vw_reg_addr(a)];
        }
        value |= (uint64_t)byte << (i * 8);
    }
    return value;
}

static void vw_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    VW2010State *s = opaque;

    for (unsigned i = 0; i < size; i++) {
        hwaddr a = addr + i;
        unsigned bank = a >= 0x18040 && a < 0x18044;
        unsigned port = bank ? 0x18040 : 0xa1b2;
        uint8_t byte = value >> (i * 8);

        /* Revision strap is read-only, including across decoder resets. */
        if (vw_reg_addr(a) == 0x1502c) {
            byte = (byte & ~2U) | 2;
        }
        s->regs[vw_reg_addr(a)] = byte;
        /* Decoder RAM reads use 0x1602e/30; writes use 0x1802e/30. */
        if (a >= 0x1602e && a < 0x16032) {
            s->regs[a + 0x2000] = byte;
        }
        if (a >= port && a < port + 4) {
            uint32_t key = vw_ram_addr(s, bank);
            uint32_t word = GPOINTER_TO_UINT(g_hash_table_lookup(
                s->ram[bank], GUINT_TO_POINTER(key)));
            unsigned shift = (a - port) * 8;

            word = (word & ~(0xffU << shift)) | ((uint32_t)byte << shift);
            if (word) {
                g_hash_table_insert(s->ram[bank], GUINT_TO_POINTER(key),
                                    GUINT_TO_POINTER(word));
            } else {
                g_hash_table_remove(s->ram[bank], GUINT_TO_POINTER(key));
            }
        }
    }
}

static const MemoryRegionOps vw_ops = {
    .read = vw_read,
    .write = vw_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void vw_reset(DeviceState *dev)
{
    VW2010State *s = VW2010(dev);

    memset(s->regs, 0, sizeof(s->regs));
    /* The physical XG1000 boot log identifies its chip as VW2005. */
    stw_le_p(s->regs + 0x1502c, 2);
    for (unsigned i = 0; i < 2; i++) {
        g_hash_table_remove_all(s->ram[i]);
    }
    pci_set_irq(PCI_DEVICE(s), 0);
}

static void vw_realize(PCIDevice *dev, Error **errp)
{
    VW2010State *s = VW2010(dev);

    for (unsigned i = 0; i < 2; i++) {
        s->ram[i] = g_hash_table_new(g_direct_hash, g_direct_equal);
    }
    dev->config[PCI_INTERRUPT_PIN] = 1;
    memory_region_init_io(&s->bar, OBJECT(s), &vw_ops, s,
                          "vw2010-registers", VW_BAR_SIZE);
    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar);
    vw_reset(DEVICE(dev));
}

static void vw_exit(PCIDevice *dev)
{
    VW2010State *s = VW2010(dev);

    for (unsigned i = 0; i < 2; i++) {
        g_hash_table_destroy(s->ram[i]);
    }
}

static const VMStateDescription vw_vmstate = {
    .name = TYPE_VW2010,
    .unmigratable = 1,
};

static void vw_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *pc = PCI_DEVICE_CLASS(klass);

    pc->realize = vw_realize;
    pc->exit = vw_exit;
    pc->vendor_id = 0x16f4;
    pc->device_id = 0x8000;
    pc->class_id = PCI_CLASS_MULTIMEDIA_OTHER;
    dc->desc = "VW2010/VW2005 codec register interface (experimental)";
    dc->vmsd = &vw_vmstate;
    device_class_set_legacy_reset(dc, vw_reset);
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo vw_info = {
    .name = TYPE_VW2010,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(VW2010State),
    .class_init = vw_class_init,
    .interfaces = (const InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static void vw_register_types(void)
{
    type_register_static(&vw_info);
}

type_init(vw_register_types)
