/* Sigma Designs SMP863x media engines: initial DCC register model. */
#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "system/address-spaces.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "qemu/bswap.h"
#include "qapi/error.h"
#include "system/reset.h"

#define TYPE_EM86XX "em86xx"
OBJECT_DECLARE_SIMPLE_TYPE(EM86XXState, EM86XX)

typedef struct AudioSlot {
    EM86XXState *owner;
    unsigned offset;
    bool status;
} AudioSlot;

struct EM86XXState {
    SysBusDevice parent_obj;
    MemoryRegion regs_region;
    MemoryRegion risc_ram;
    MemoryRegion dmem_mutex[5];
    uint8_t regs[0x70000];
    uint32_t read_count;
    bool audio_stub;
    MemoryRegion audio_commands[8];
    AudioSlot audio_slots[8];
    bool display_assist;
    QEMUTimer *display_timer;
};

/* Local XG1000 experiment for the supplied em8xxx.ko shared-memory ABI.
 * This is deliberately opt-in: these addresses are firmware data, not
 * hardware registers, and are not valid for arbitrary MRUA versions.
 */
static uint32_t em86xx_shared_read(hwaddr addr)
{
    return address_space_ldl_le(&address_space_memory, addr,
                                MEMTXATTRS_UNSPECIFIED, NULL);
}

static void em86xx_shared_write(hwaddr addr, uint32_t value)
{
    address_space_stl_le(&address_space_memory, addr, value,
                         MEMTXATTRS_UNSPECIFIED, NULL);
}

static void em86xx_display_tick(void *opaque)
{
    EM86XXState *s = opaque;
    const hwaddr task = 0x10003140;
    static const hwaddr blocks[] = {
        0x1000193c, /* digital output */
        0x10001cd8, /* component output */
        0x10001e80, /* routing */
    };

    /* Wait until the guest installs the display task. Preserve queued
     * notifications, and never touch the software interrupt mutex.
     */
    if (em86xx_shared_read(0x10005284) == task &&
        em86xx_shared_read(task + 0x28) == 3 &&
        !(em86xx_shared_read(task + 0x2c) & 2)) {
        for (unsigned i = 0; i < ARRAY_SIZE(blocks); i++) {
            if (em86xx_shared_read(blocks[i])) {
                em86xx_shared_write(blocks[i] + 8,
                                    em86xx_shared_read(blocks[i] + 4));
                em86xx_shared_write(blocks[i], 0);
            }
        }
        em86xx_shared_write(task + 0x20,
                            em86xx_shared_read(task + 0x20) | 7);
        em86xx_shared_write(task + 0x2c,
                            em86xx_shared_read(task + 0x2c) | 0x22);
        em86xx_shared_write(0x617f8, em86xx_shared_read(0x617f8) | 0x10);
        em86xx_shared_write(0x6e010, 0x10);
    }
    timer_mod(s->display_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 16666667);
}

/* Local silent stub for audio_set_command/audio_get_command in em8xxx.ko.
 * Each engine has two command slots at PMEM + 0x10674/+0x1069c.
 * The driver writes the payload first and the command last; zero means
 * firmware has accepted the request. Keep payload/configuration in RAM.
 * Status words at command + 12 report stopped (1), allowing format
 * changes through change_State(). This acknowledges control only: it does not decode or render audio.
 */
static uint64_t em86xx_audio_read(void *opaque, hwaddr addr, unsigned size)
{
    AudioSlot *slot = opaque;
    uint8_t *ram = memory_region_get_ram_ptr(&slot->owner->risc_ram);
    uint64_t value = 0;
    for (unsigned i = 0; i < size; i++) {
        value |= (uint64_t)ram[slot->offset + addr + i] << (i * 8);
    }
    return value;
}

static void em86xx_audio_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned size)
{
    AudioSlot *slot = opaque;
    uint8_t *ram = memory_region_get_ram_ptr(&slot->owner->risc_ram);
    if (!slot->status && !addr && size == 4 && value > 0 && value <= 7) {
        /* Driver commands are 1..7. Firmware uploads remain normal RAM. */
        stl_le_p(ram + slot->offset, 0);
        stl_le_p(ram + slot->offset + 12, 1);
        return;
    }
    for (unsigned i = 0; i < size; i++) {
        ram[slot->offset + addr + i] = value >> (i * 8);
    }
}

static const MemoryRegionOps em86xx_audio_ops = {
    .read = em86xx_audio_read,
    .write = em86xx_audio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

/* Engine register indices in emhwlib_registers_tango2.h are word indices.
 * The guest RUA property implementation performs their GBUS translation.
 * Model the free-running counters, while retaining reset/microcode registers
 * for readback. No media instruction execution is implemented yet.
 */
static bool em86xx_timer(hwaddr addr, hwaddr *divider)
{
    static const hwaddr bases[] = { 0x80000, 0x90000, 0xa0000,
                                   0xc0000, 0xd0000 };
    static const unsigned indices[] = { 0xffc, 0xffc, 0x2f0c,
                                       0x3e8c, 0x3e8c };
    for (unsigned i = 0; i < ARRAY_SIZE(bases); i++) {
        hwaddr count = bases[i] - 0x70000 + indices[i] * 4;
        if ((addr & ~3ULL) == count) {
            *divider = count - 4;
            return true;
        }
    }
    return false;
}

static bool em86xx_engine(hwaddr addr)
{
    unsigned block = (addr + 0x70000) >> 16;
    return block == 8 || block == 9 || block == 10 ||
           block == 12 || block == 13;
}

static bool em86xx_mutex(hwaddr addr)
{
    unsigned block = (addr + 0x70000) >> 16;
    unsigned offset = addr & 0xffff;
    unsigned first;

    switch (block) {
    case 8:
    case 9:
        first = 0xfe0 * 4;
        break;
    case 10:
        first = 0x2f10 * 4;
        break;
    case 12:
    case 13:
        first = 0x3e90 * 4;
        break;
    default:
        return false;
    }
    return offset >= first && offset < first + 8 * 4;
}

static uint64_t em86xx_read(void *opaque, hwaddr addr, unsigned size)
{
    EM86XXState *s = opaque;
    hwaddr divider;
    uint32_t value = 0;
    if (em86xx_timer(addr, &divider)) {
        uint32_t div = ldl_le_p(s->regs + divider);
        uint64_t ticks = muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
                                 27000000, 1000000000);
        value = ticks / ((uint64_t)div + 1);
        return (value >> ((addr & 3) * 8)) &
               (UINT64_MAX >> ((8 - size) * 8));
    }
    for (unsigned i = 0; i < size; i++) {
        value |= (uint32_t)s->regs[addr + i] << (i * 8);
    }
    /* gbus_mutex_lock() polls until the old owner is zero. Reading an
     * unlocked mutex atomically acquires it; zero writes release it.
     * GBUS_MUTEX_LOCAL identifies the local CPU port in the SDK header.
     */
    if (size == 4 && !(addr & 3) && em86xx_mutex(addr) && !value) {
        stl_le_p(s->regs + addr, 0x10);
    }
    if (++s->read_count <= 256 || !(s->read_count & 0xfffff)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "em86xx read address=%08" HWADDR_PRIx
                      " value=%08x count=%u\n", addr + 0x70000,
                      value, s->read_count);
    }
    return value;
}

static void em86xx_write(void *opaque, hwaddr addr, uint64_t value,
                         unsigned size)
{
    EM86XXState *s = opaque;
    for (unsigned i = 0; i < size; i++) {
        s->regs[addr + i] = value >> (i * 8);
    }
    /* ucode_risc_bist() starts G2L BIST with bit 31, polls BUSY & MASK,
     * then checks PASS & MASK for failures. The storage-backed PMEM/DMEM
     * completes immediately with no failing banks.
     */
    if (em86xx_engine(addr) && (addr & 0xffff) == 0xffe0 &&
        size == 4 && (value & (1U << 31))) {
        stl_le_p(s->regs + addr, 0);
        stl_le_p(s->regs + addr + 4, 0);
    }
    qemu_log_mask(LOG_GUEST_ERROR,
                  "em86xx write address=%08" HWADDR_PRIx
                  " size=%u value=%08" PRIx64 "\n",
                  addr + 0x70000, size, value);
}

static const MemoryRegionOps em86xx_ops = {
    .read = em86xx_read,
    .write = em86xx_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void em86xx_reset(void *opaque)
{
    EM86XXState *s = opaque;
    memset(s->regs, 0, sizeof(s->regs));
    s->read_count = 0;
    if (s->display_timer) {
        timer_mod(s->display_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 16666667);
    }
}

static void em86xx_realize(DeviceState *dev, Error **errp)
{
    EM86XXState *s = EM86XX(dev);
    static const struct {
        hwaddr regs;
        hwaddr dmem;
        unsigned index;
    } mutex_banks[] = {
        { 0x80000, 0x110000, 0xfe0 },
        { 0x90000, 0x130000, 0xfe0 },
        { 0xa0000, 0x150000, 0x2f10 },
        { 0xc0000, 0x190000, 0x3e90 },
        { 0xd0000, 0x1b0000, 0x3e90 },
    };
    memory_region_init_io(&s->regs_region, OBJECT(s), &em86xx_ops, s,
                          "em86xx-registers", sizeof(s->regs));
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->regs_region);
    if (!memory_region_init_ram(&s->risc_ram, OBJECT(s), "em86xx-risc-ram",
                                0xc0000, errp)) {
        return;
    }
    /* SDK gbus_mutex addresses use DMEM_BASE + 4 * engine_mutexN.
     * Overlay only the action-on-read words; firmware data stays in RAM.
     */
    for (unsigned i = 0; i < ARRAY_SIZE(mutex_banks); i++) {
        hwaddr offset = mutex_banks[i].index * 4;

        memory_region_init_alias(&s->dmem_mutex[i], OBJECT(s),
                                 "em86xx-dmem-mutex", &s->regs_region,
                                 mutex_banks[i].regs - 0x70000 + offset,
                                 8 * 4);
        memory_region_add_subregion(&s->risc_ram,
                                    mutex_banks[i].dmem - 0x100000 + offset,
                                    &s->dmem_mutex[i]);
    }
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->risc_ram);
    if (s->audio_stub) {
        static const unsigned slots[] = { 0x90674, 0x9069c,
                                          0xb0674, 0xb069c,
                                          0x90680, 0x906a8,
                                          0xb0680, 0xb06a8 };
        for (unsigned i = 0; i < ARRAY_SIZE(slots); i++) {
            s->audio_slots[i] = (AudioSlot) { s, slots[i], i >= 4 };
            memory_region_init_io(&s->audio_commands[i], OBJECT(s),
                                  &em86xx_audio_ops, &s->audio_slots[i],
                                  "em86xx-silent-audio-command", 4);
            memory_region_add_subregion(&s->risc_ram, slots[i],
                                        &s->audio_commands[i]);
        }
    }
    if (s->display_assist) {
        s->display_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                        em86xx_display_tick, s);
    }
    qemu_register_reset(em86xx_reset, s);
}

static const Property em86xx_properties[] = {
    DEFINE_PROP_BOOL("xg1000-audio-stub", EM86XXState, audio_stub, false),
    DEFINE_PROP_BOOL("xg1000-display-assist", EM86XXState, display_assist, false),
};

static void em86xx_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = em86xx_realize;
    device_class_set_props(dc, em86xx_properties);
    dc->desc = "Sigma Designs em86xx media engines (partial)";
    dc->user_creatable = false;
}

static const TypeInfo em86xx_type = {
    .name = TYPE_EM86XX,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(EM86XXState),
    .class_init = em86xx_class_init,
};

static void em86xx_register_types(void)
{
    type_register_static(&em86xx_type);
}
type_init(em86xx_register_types)
