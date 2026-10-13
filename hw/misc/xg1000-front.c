/* XG1000 front-panel peripheral. */
#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "system/reset.h"
#include "hw/misc/xg1000-front.h"

struct XG1000Front {
    MemoryRegion uart;
    MemoryRegion irq_mask;
    QEMUTimer *timer;
    uint32_t regs[0x100 / 4];
    uint16_t mask;
    uint8_t rx[64];
    unsigned rx_head, rx_count;
    uint8_t command;
    bool command_pending;
    uint8_t volume[3];
    uint8_t settings[64];
    qemu_irq irq;
};
#define CPU_UART1_BASE 0x6c200
#define FRONT_IRQ (1U << 15)

XG1000Front *xg1000_front_new(void)
{
    return g_new0(XG1000Front, 1);
}
/* Local behavioral model of the controller attached to CPU UART1.
 * front_mod.ko services UART RX/TX from FPGA IRQ 23, not UART IRQ 10.
 * A 1 ms service tick approximates the FPGA cadence; its exact period is
 * not known. RX stays latched until the driver clears LSR bit 0 (it reads
 * RBR several times while decoding a single byte).
 */
static void xg1000_front_reply(XG1000Front *s, uint8_t command, uint8_t data)
{
    unsigned tail;

    if (s->rx_count > ARRAY_SIZE(s->rx) - 2) {
        qemu_log_mask(LOG_GUEST_ERROR, "xg1000 front RX overflow\n");
        return;
    }
    tail = (s->rx_head + s->rx_count) % ARRAY_SIZE(s->rx);
    s->rx[tail] = command | ((data & 0x80) >> 1);
    s->rx[(tail + 1) % ARRAY_SIZE(s->rx)] = data & 0x7f;
    s->rx_count += 2;
}

static void xg1000_front_tx(XG1000Front *s, uint8_t byte)
{
    uint8_t command, data;

    if (byte & 0x80) {
        s->command = byte;
        s->command_pending = true;
        return;
    }
    if (!s->command_pending) {
        return;
    }
    command = s->command & 0xbf;
    data = byte | ((s->command & 0x40) << 1);
    s->command_pending = false;
    s->settings[command & 0x3f] = data;
    if (command == 0x90) {
        switch (data) {
        case 3: /* Read the three volume positions. */
            for (unsigned i = 0; i < 3; i++) {
                xg1000_front_reply(s, 0xa3 + i, s->volume[i]);
            }
            break;
        case 5: /* Program version: library expands 0x20 to 0x0200. */
            xg1000_front_reply(s, 0xa6, 0x20);
            break;
        case 7: /* Samsung LTE700WQ-F05, as in the hardware boot log. */
            xg1000_front_reply(s, 0xa7, 0);
            break;
        default:
            break;
        }
    }
}

static void xg1000_front_tick(void *opaque)
{
    XG1000Front *s = opaque;

    if (s->mask & 0xe00) {
        qemu_set_irq(s->irq, 1);
        timer_mod_ns(s->timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000);
    }
}

static uint64_t xg1000_front_read(void *opaque, hwaddr addr, unsigned size)
{
    XG1000Front *s = opaque;
    uint32_t value;

    switch (addr & ~3ULL) {
    case 0:
        value = s->rx_count ? s->rx[s->rx_head] : 0;
        break;
    case 0x1c:
        value = 0x60 | (s->rx_count != 0);
        break;
    default:
        value = s->regs[addr / 4];
        break;
    }
    return value >> ((addr & 3) * 8);
}

static void xg1000_front_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned size)
{
    XG1000Front *s = opaque;
    unsigned shift = (addr & 3) * 8;
    uint32_t mask = (UINT32_MAX >> ((4 - size) * 8)) << shift;

    s->regs[addr / 4] = (s->regs[addr / 4] & ~mask) |
                             ((value << shift) & mask);
    if (addr == 4) {
        xg1000_front_tx(s, value);
    } else if (addr == 0x1c && !(value & 1) && s->rx_count) {
        s->rx_head = (s->rx_head + 1) % ARRAY_SIZE(s->rx);
        s->rx_count--;
    }
}

static const MemoryRegionOps xg1000_front_ops = {
    .read = xg1000_front_read,
    .write = xg1000_front_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static uint64_t xg1000_front_mask_read(void *opaque, hwaddr addr, unsigned size)
{
    XG1000Front *s = opaque;

    return s->mask >> (addr * 8);
}

static void xg1000_front_mask_write(void *opaque, hwaddr addr,
                                   uint64_t value, unsigned size)
{
    XG1000Front *s = opaque;
    unsigned shift = addr * 8;
    uint16_t mask = (size == 2 ? 0xffff : 0xff) << shift;

    s->mask = (s->mask & ~mask) | ((value << shift) & mask);
    if (s->mask & 0xe00) {
        timer_mod_ns(s->timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000);
    } else {
        timer_del(s->timer);
        qemu_set_irq(s->irq, 0);
    }
}

static const MemoryRegionOps xg1000_front_mask_ops = {
    .read = xg1000_front_mask_read,
    .write = xg1000_front_mask_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 2 },
};

static void xg1000_front_reset(void *opaque)
{
    XG1000Front *s = opaque;

    timer_del(s->timer);
    s->mask = 0;
    s->rx_head = s->rx_count = 0;
    s->command_pending = false;
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->settings, 0, sizeof(s->settings));
    s->volume[0] = 4;
    s->volume[1] = 48;
    s->volume[2] = 77;
    qemu_set_irq(s->irq, 0);
}

void xg1000_front_init(XG1000Front *s, MemoryRegion *sysmem, qemu_irq irq)
{
    s->irq = irq;
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xg1000_front_tick, s);
    memory_region_init_io(&s->uart, NULL, &xg1000_front_ops, s,
                          "xg1000-front-uart", 0x100);
    memory_region_init_io(&s->irq_mask, NULL, &xg1000_front_mask_ops, s,
                          "xg1000-front-irq-mask", 2);
    memory_region_add_subregion_overlap(sysmem, CPU_UART1_BASE,
                                        &s->uart, 3);
    MemoryRegion *alias = g_new(MemoryRegion, 1);
    memory_region_init_alias(alias, NULL, "xg1000-front-uart-kseg1",
                             &s->uart, 0, 0x100);
    memory_region_add_subregion_overlap(sysmem, 0xa0000000ULL + CPU_UART1_BASE,
                                        alias, 3);
    memory_region_add_subregion_overlap(sysmem, 0x4c00d800,
                                        &s->irq_mask, 3);
    qemu_register_reset(xg1000_front_reset, s);
    xg1000_front_reset(s);
}

