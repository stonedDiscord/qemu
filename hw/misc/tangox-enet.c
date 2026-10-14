/* Local TangoX Ethernet MAC/DMA experiment for the XG1000 guest. */
#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "system/dma.h"
#include "qemu/module.h"
#include "qemu/bswap.h"
#include "system/reset.h"
#include "net/net.h"

#define TYPE_TANGOX_ENET "tangox-enet"
OBJECT_DECLARE_SIMPLE_TYPE(TangoXEnet, TANGOX_ENET)
struct TangoXEnet {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    qemu_irq irq;
    NICConf conf;
    NICState *nic;
    uint32_t regs[0x1100 / 4];
    uint16_t phy[32];
    uint32_t tx, rx;
};

static void enet_irq(TangoXEnet *s)
{
    qemu_set_irq(s->irq, (s->regs[0x1014 / 4] &
                         s->regs[0x101c / 4] & 0x7fff) != 0);
}

static uint32_t enet_next(TangoXEnet *s, uint32_t addr, uint32_t *d,
                          uint32_t base)
{
    if (d[1] & (1U << 24)) {
        return d[3];
    }
    return d[1] & (1U << 25) ? base :
        addr + 16 + ((s->regs[0x1000 / 4] >> 2) & 0x1f) * 4;
}

static bool enet_desc(uint32_t addr, uint32_t *d)
{
    uint8_t bytes[16];
    if (dma_memory_read(&address_space_memory, addr, bytes, 16,
                        MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return false;
    }
    for (int i = 0; i < 4; i++) {
        d[i] = ldl_le_p(bytes + i * 4);
    }
    return true;
}

static void enet_complete(uint32_t addr, uint32_t status)
{
    uint8_t bytes[4];
    stl_le_p(bytes, status);
    dma_memory_write(&address_space_memory, addr, bytes, 4,
                     MEMTXATTRS_UNSPECIFIED);
}

static void enet_tx(TangoXEnet *s)
{
    uint8_t packet[65536];
    size_t length = 0;
    if (!(s->regs[0x1018 / 4] & (1 << 13))) {
        return;
    }
    for (int n = 0; n < 256; n++) {
        uint32_t d[4];
        if (!s->tx || !enet_desc(s->tx, d) || !(d[0] & (1U << 31))) {
            break;
        }
        if (d[1] & (1 << 29)) {
            length = 0;
        }
        for (int b = 0; b < 2; b++) {
            size_t len = (d[1] >> (b * 11)) & 0x7ff;
            if (b && (d[1] & (1 << 24))) {
                break;
            }
            if (len > sizeof(packet) - length ||
                dma_memory_read(&address_space_memory, d[2 + b],
                                packet + length, len,
                                MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
                return;
            }
            length += len;
        }
        enet_complete(s->tx, d[0] & ~(1U << 31));
        s->tx = enet_next(s, s->tx, d, s->regs[0x1010 / 4]);
        if (d[1] & (1 << 30)) {
            qemu_send_packet(qemu_get_queue(s->nic), packet, length);
            length = 0;
        }
        s->regs[0x1014 / 4] |= 0x10001;
    }
    enet_irq(s);
}

static bool enet_can_receive(NetClientState *nc)
{
    TangoXEnet *s = qemu_get_nic_opaque(nc);
    uint32_t d[4];
    return (s->regs[0x1018 / 4] & 2) && s->rx &&
        enet_desc(s->rx, d) && (d[0] & (1U << 31));
}

static ssize_t enet_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    TangoXEnet *s = qemu_get_nic_opaque(nc);
    uint32_t d[4];
    if (!enet_can_receive(nc) || !enet_desc(s->rx, d)) {
        return 0;
    }
    /* The supplied driver uses one 1536-byte buffer per descriptor. */
    if (size + 4 > (d[1] & 0x7ff)) {
        return size;
    }
    if (dma_memory_write(&address_space_memory, d[2], buf, size,
                         MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        return size;
    }
    enet_complete(s->rx, ((size + 4) << 16) | 0x300);
    s->rx = enet_next(s, s->rx, d, s->regs[0x100c / 4]);
    s->regs[0x1014 / 4] |= 0x10040;
    enet_irq(s);
    return size;
}

static uint64_t enet_read(void *opaque, hwaddr addr, unsigned size)
{
    TangoXEnet *s = opaque;
    return s->regs[addr / 4];
}

static void enet_write(void *opaque, hwaddr addr, uint64_t value, unsigned size)
{
    TangoXEnet *s = opaque;
    if (addr == 0x1014) {
        s->regs[addr / 4] &= ~value;
    } else {
        s->regs[addr / 4] = value;
    }
    switch (addr) {
    case 0x14: {
        unsigned phy = (value >> 11) & 31;
        unsigned reg = (value >> 6) & 31;
        if (value & 2) {
            if (phy == 9) {
                s->phy[reg] = s->regs[0x18 / 4];
                if (reg == 0) {
                    s->phy[0] &= ~0x8000;
                }
            }
        } else {
            s->regs[0x18 / 4] = phy == 9 ? s->phy[reg] : 0xffff;
        }
        s->regs[0x14 / 4] &= ~1U;
        break;
    }
    case 0x1000:
        s->regs[addr / 4] &= ~1U;
        break;
    case 0x100c:
        s->rx = value;
        break;
    case 0x1010:
        s->tx = value;
        break;
    case 0x1004:
    case 0x1018:
        enet_tx(s);
        break;
    }
    enet_irq(s);
    if (addr == 0x1008 || addr == 0x1018) {
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
    }
}

static const MemoryRegionOps enet_ops = {
    .read = enet_read, .write = enet_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};
static NetClientInfo enet_info = {
    .type = NET_CLIENT_DRIVER_NIC, .size = sizeof(NICState),
    .can_receive = enet_can_receive, .receive = enet_receive,
};
static void enet_reset(void *opaque)
{
    TangoXEnet *s = opaque;
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->phy, 0, sizeof(s->phy));
    s->tx = s->rx = 0;
    s->phy[0] = 0x3100;
    s->phy[1] = 0x782d;
    s->phy[3] = 0x8201;
    s->phy[4] = s->phy[5] = 0x01e1;
    enet_irq(s);
}

static void enet_realize(DeviceState *dev, Error **errp)
{
    TangoXEnet *s = TANGOX_ENET(dev);
    memory_region_init_io(&s->mmio, OBJECT(s), &enet_ops, s,
                          "tangox-enet", sizeof(s->regs));
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->phy[0] = 0x3100;
    s->phy[1] = 0x782d; /* autonegotiation complete, link up */
    s->phy[2] = 0x0000;
    s->phy[3] = 0x8201;
    s->phy[4] = s->phy[5] = 0x01e1;
    qemu_macaddr_default_if_unset(&s->conf.macaddr);
    s->nic = qemu_new_nic(&enet_info, &s->conf, TYPE_TANGOX_ENET, dev->id,
                          &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
    qemu_register_reset(enet_reset, s);
}
static const Property enet_props[] = {
    DEFINE_NIC_PROPERTIES(TangoXEnet, conf),
};
static void enet_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = enet_realize;
    device_class_set_props(dc, enet_props);
    dc->user_creatable = false;
}
static const TypeInfo enet_type = {
    .name = TYPE_TANGOX_ENET, .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(TangoXEnet), .class_init = enet_class_init,
};
static void enet_register(void)
{
    type_register_static(&enet_type);
}
type_init(enet_register)
