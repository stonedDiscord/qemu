/* Sigma Designs TangoX / SMP863x machine. */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/char/serial-mm.h"
#include "hw/usb/hcd-ohci.h"
#include "hw/usb/hcd-ehci.h"
#include "hw/block/flash.h"
#include "hw/ide/mmio.h"
#include "hw/ide/pci.h"
#include "hw/ide/sii3114.h"
#include "hw/core/boards.h"
#include "hw/core/clock.h"
#include "hw/core/irq.h"
#include "hw/core/loader.h"
#include "hw/core/qdev.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_host.h"
#include "hw/mips/mips.h"
#include "system/memory.h"
#include "system/reset.h"
#include "system/system.h"
#include "system/blockdev.h"
#include "system/block-backend-global-state.h"
#include "system/block-backend-io.h"
#include "target/mips/cpu.h"

#define SMP863X_DRAM_BASE       0x10000000ULL
#define SMP863X_KERNEL_OFFSET   0x00020000ULL
#define SMP863X_REG_BASE        0x00060000ULL
#define SMP863X_REG_SIZE        0x00060000ULL
#define SMP863X_XENV_PHYS       0x0c000000ULL
#define SMP863X_PFLASH_BASE     0x1fc00000ULL
#define SMP863X_PFLASH_SIZE     0x00800000ULL
#define SMP863X_XENV_LOCATION   0x0c000000U
#define SMP863X_REMAP1_BASE     0x00000000ULL
#define SMP863X_REMAP2_BASE     0x04000000ULL
#define SMP863X_REMAP3_BASE     0x08000000ULL
#define SMP863X_REMAP4_BASE     0x0c000000ULL
#define SMP863X_REMAP_WINDOW_SIZE 0x04000000ULL
#define SMP863X_PCI_CONFIG_BASE   0x50000000ULL
#define SMP863X_PCI_CONFIG_SIZE   0x00010000ULL
#define SMP863X_PCI_IO_BASE       0x58000000ULL
#define SMP863X_PCI_IO_SIZE       0x00010000ULL
#define SMP863X_PCI_MEMORY_BASE   0x60000000ULL
#define SMP863X_PCI_MEMORY_SIZE   0x10000000ULL

#define CPU_UART0_BASE          0x6c100
#define CPU_UART1_BASE          0x6c200
#define CPU_IRQ_STATUS          0x6e000
#define CPU_IRQ_ENABLE_SET      0x6e008
#define CPU_IRQ_ENABLE_CLR      0x6e00c
#define CPU_IRQ_RAWSTAT         0x6e004
#define CPU_EDGE_RAWSTAT        0x6e204  /* write-1-to-clear; kernel acks IRQs here */
#define CPU_TIMER0_CTRL         0x6c508
#define CPU_TIMER0_LOAD         0x6c500
#define CPU_TIMER0_CLR          0x6c50c
#define IDE_CTRL_BLOCK          0x22398
#define IDE_STATUS_REG          0x223dc
#define IDE_STATUS_REG_ALT      0x223d8
#define IDE_CMD_BLOCK           0x223c0
#define LR_ETH_MAC_LO           0x61ce0
#define LR_ETH_MAC_HI           0x61ce4
#define LR_XENV_LOCATION        0x61ff0
#define CPU_REMAP               0x6f000
#define CPU_REMAP1              0x6f004
#define CPU_REMAP2              0x6f008
#define CPU_REMAP3              0x6f00c
#define CPU_REMAP4              0x6f010
#define TANGOX_USB_EHCI_BASE   0x21400
#define TANGOX_USB_OHCI_BASE   0x21500
#define TANGOX_USB_CTL_BASE    0x21700
#define TANGOX_GPIO_BASE       0x10000
#define TANGOX_IR_BASE         0x10018
#define TANGOX_FIP_BASE        0x10540
#define TANGOX_SCARD_BASE      0x6c300
#define TANGOX_ENET_MAC_BASE   0x26000
#define TANGOX_ENET_DMA_BASE   0x27000

typedef struct SMP863XState {
    MIPSCPU *cpu;
    MemoryRegion *ram;
    MemoryRegion *flash;
    MemoryRegion pci_config;
    MemoryRegion pci_io;
    MemoryRegion pci_memory;
    MemoryRegion pci_dma;
    MemoryRegion pci_dma_regions[6];
    AddressSpace pci_dma_as;
    PCIBus *pci_bus;
    uint8_t *flash_data;
    size_t flash_data_size;
    uint32_t remap[5];
    bool flash_boot;
    uint32_t xenv_location;
    uint32_t reset_pc;
    qemu_irq cpu_irq;
    uint32_t irq_enable;
    uint32_t irq_status;
    uint32_t xtal_counter;
    uint32_t regs[0x100000 / 4];
    QEMUTimer *timer;
    bool timer_enabled;
    uint32_t timer_load;
    uint32_t led_uart_data;
    uint32_t led_uart_status;
} SMP863XState;

static const TypeInfo smp863x_pci_host_type = {
    .name = "smp863x-pci-host",
    .parent = TYPE_PCI_HOST_BRIDGE,
    .instance_size = sizeof(PCIHostState),
};

static void smp863x_register_pci_host(void)
{
    type_register_static(&smp863x_pci_host_type);
}

type_init(smp863x_register_pci_host)

static AddressSpace *smp863x_pci_dma_as(PCIBus *bus, void *opaque, int devfn)
{
    SMP863XState *s = opaque;

    return &s->pci_dma_as;
}

static const PCIIOMMUOps smp863x_pci_iommu_ops = {
    .get_address_space = smp863x_pci_dma_as,
};

/* Tangox maps PCI configuration space as bus/devfn/offset bytes. */
static uint64_t smp863x_pci_config_read(void *opaque, hwaddr addr,
                                        unsigned size)
{
    SMP863XState *s = opaque;

    if (!s->pci_bus || size > 4 || addr >= SMP863X_PCI_CONFIG_SIZE ||
        addr + size > SMP863X_PCI_CONFIG_SIZE || (size > 1 && (addr & 1)) ||
        (size == 4 && (addr & 3))) {
        return UINT32_MAX;
    }
    return pci_data_read(s->pci_bus, addr, size);
}

static void smp863x_pci_config_write(void *opaque, hwaddr addr,
                                     uint64_t data, unsigned size)
{
    SMP863XState *s = opaque;

    if (!s->pci_bus || size > 4 || addr >= SMP863X_PCI_CONFIG_SIZE ||
        addr + size > SMP863X_PCI_CONFIG_SIZE || (size > 1 && (addr & 1)) ||
        (size == 4 && (addr & 3))) {
        return;
    }
    pci_data_write(s->pci_bus, addr, data, size);
}

static const MemoryRegionOps smp863x_pci_config_ops = {
    .read = smp863x_pci_config_read,
    .write = smp863x_pci_config_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

typedef struct SMP863XRemapWindow {
    SMP863XState *s;
    unsigned index;
} SMP863XRemapWindow;

/* XG1000's UART1 is wired to the front-panel LED controller.  It is a
 * write-only serial slave: the kernel polls the UART line status before
 * shifting each LED byte and expects transmitter-ready (bit 5). */
static uint64_t smp863x_front_panel_read(void *opaque, hwaddr addr,
                                         unsigned size)
{
    if ((addr & 0x1f) == 0x1c) {
        return 0x60;
    }
    return 0;
}

static void smp863x_front_panel_write(void *opaque, hwaddr addr,
                                      uint64_t data, unsigned size)
{
    /* The physical front-panel controller consumes UART1 transmit bytes. */
}

static const MemoryRegionOps smp863x_front_panel_ops = {
    .read = smp863x_front_panel_read,
    .write = smp863x_front_panel_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static uint64_t smp863x_remap_read(void *opaque, hwaddr addr, unsigned size)
{
    SMP863XRemapWindow *window = opaque;
    SMP863XState *s = window->s;
    uint64_t value = 0;

    /* Any non-zero remap source in this machine selects the parallel NOR.
     * The source address is 64 MiB aligned; the window offset selects the
     * byte within that source. */
    if (!s->flash_data || s->remap[window->index] == 0 ||
        addr + size > s->flash_data_size || size > sizeof(value)) {
        return UINT64_MAX;
    }
    memcpy(&value, s->flash_data + addr, size);
    return value;
}

static void smp863x_remap_write(void *opaque, hwaddr addr, uint64_t data,
                                unsigned size)
{
    /* NOR writes are intentionally left to the CFI device at its native
     * address.  Boot ROM and zboot only read through remap windows. */
}

static const MemoryRegionOps smp863x_remap_ops = {
    .read = smp863x_remap_read,
    .write = smp863x_remap_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static uint64_t smp863x_flash_window_read(void *opaque, hwaddr addr,
                                          unsigned size)
{
    SMP863XState *s = opaque;
    uint64_t value = UINT64_MAX;

    if (s->flash_data && addr + size <= s->flash_data_size &&
        size <= sizeof(value)) {
        memcpy(&value, s->flash_data + addr, size);
    }
    return value;
}

static void smp863x_flash_window_write(void *opaque, hwaddr addr,
                                       uint64_t data, unsigned size)
{
    /* CS3 is not populated on this board; ignore NOR probe commands. */
}

static const MemoryRegionOps smp863x_flash_window_ops = {
    .read = smp863x_flash_window_read,
    .write = smp863x_flash_window_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

/* A small, valid XENV block.  The Linux image needs this because its
 * CONFIG_TANGOX_XENV_READ_SAFE path deliberately stops on invalid data. */
static const uint8_t smp863x_xenv[] = {
    0x8d, 0x00, 0x00, 0x00,
    0x9c, 0x50, 0x41, 0x7b, 0xc3, 0x9a, 0x3b, 0x71, 0xce, 0x9f,
    0x83, 0x17, 0xde, 0x02, 0xfd, 0x4b, 0x29, 0xe0, 0x29, 0x2b,
    0x00, 0x1b, 'a', '.', 'l', 'i', 'n', 'u', 'x', '_', 'c', 'm', 'd', 0,
    'c', 'o', 'n', 's', 'o', 'l', 'e', '=', 't', 't', 'y', 'S', '0',
    0x00, 0x18, 'a', '.', 'u', 'a', 'r', 't', '_', 'u', 's', 'e', 'd',
    '_', 'p', 'o', 'r', 't', 's', 0, 1, 0, 0, 0,
    0x00, 0x1a, 'a', '.', 'u', 'a', 'r', 't', '_', 'c', 'o', 'n', 's',
    'o', 'l', 'e', '_', 'p', 'o', 'r', 't', 0, 0, 0, 0, 0,
    0x00, 0x11, 'a', '.', 'b', 'a', 'u', 'd', 'r', 'a', 't', 'e', 0,
    0x00, 0xc2, 0x01, 0x00, 0x00,
    0x00, 0x17, 'a', '.', 'e', 'n', 'a', 'b', 'l', 'e', '_', 'd',
    'e', 'v', 'i', 'c', 'e', 's', 0, 0, 0, 0, 0,
};

static void smp863x_update_irq(SMP863XState *s)
{
    qemu_set_irq(s->cpu_irq, (s->irq_status & s->irq_enable) != 0);
}

static int smp863x_pci_map_irq(PCIDevice *dev, int pin)
{
    /* Linux records its XENV-selected route in PCI_INTERRUPT_LINE. */
    unsigned line = dev->config[PCI_INTERRUPT_LINE];

    if (PCI_SLOT(dev->devfn) == 4) {
        return 3;
    }
    return line >= 21 && line <= 24 ? line - 21 : pin;
}

static void smp863x_pci_set_irq(void *opaque, int irq, int level)
{
    SMP863XState *s = opaque;
    uint32_t bit = 1U << (13 + irq);

    if (level) {
        s->irq_status |= bit;
    } else {
        s->irq_status &= ~bit;
    }
    smp863x_update_irq(s);
}

static void smp863x_timer_cb(void *opaque)
{
    SMP863XState *s = opaque;

    if (!s->timer_enabled) {
        return;
    }
    s->irq_status |= 1U << 5; /* CPU_TIMER0_INT */
    smp863x_update_irq(s);
    /* Timer0 is clocked from the 201 MHz system clock with the 512x
     * prescaler used by the Linux TangoX platform code. */
    uint64_t ticks = s->timer_load ? s->timer_load : 39257;
    uint64_t ns = (ticks * 512ULL * 1000000000ULL) / 201000000ULL;
    timer_mod_ns(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns);
}

static void smp863x_uart_irq(void *opaque, int irq, int level)
{
    SMP863XState *s = opaque;
    uint32_t bit = 1U << (irq + 1); /* UART0/UART1 are IRQ bits 1/2. */

    if (level) {
        s->irq_status |= bit;
    } else {
        s->irq_status &= ~bit;
    }
    smp863x_update_irq(s);
}

static uint32_t smp863x_regs_read32(SMP863XState *s, hwaddr addr)
{
    uint32_t value = 0;

    switch (addr) {
    case IDE_CTRL_BLOCK:
    case IDE_STATUS_REG:
    case IDE_STATUS_REG_ALT:
        /* No ATA device is attached, but the controller is idle and ready. */
        return 0x00000050;
    case 0x10000 + 0x003c: /* SYS_sysclk_mux: ES6, PLL, CPU divider. */
        return 0x00000201;
    case 0x10000 + 0x0034: /* SYS_sysclk_premux */
        return 0;
    case 0x10000 + 0x0000: /* SYS_clkgen0_pll */
        return 0x01010041;
    case 0x10000 + 0x0018: /* SYS_clkgen3_pll: 603 MHz before dividers. */
        return 0x01010041;
    case 0x10000 + 0x0048: /* SYS_xtal_in_cnt, 27 MHz */
        return s->xtal_counter += 1000;
    case 0x20000 + 0xfee8: /* PCI_REG0 */
        return 0x00008630;
    case 0x20000 + 0xfeec: /* PCI_REG1 */
        return 0x00000083;
    case LR_XENV_LOCATION:
        return s->xenv_location;
    case LR_ETH_MAC_LO:
        /* XG1000 board MAC 00:16:e8:50:41:2c, little-endian low word. */
        return 0xe850412c;
    case LR_ETH_MAC_HI:
        return 0x00000016;
    case 0x26014: /* Ethernet MDIO command/status: idle. */
        return 0;
    case 0x26018: /* Ethernet MDIO data: the board PHY is at address 9. */
        /* MIIAR selects the PHY; unpopulated addresses float high. */
        return ((s->regs[0x26014 / 4] >> 11) & 0x1f) == 9 ?
               0x00000948 : 0xffff;
    case TANGOX_USB_CTL_BASE:
        return 0x00000070; /* controller enabled, clocks stable */
    case TANGOX_USB_CTL_BASE + 0x8:
        return 0x00000000; /* OHCI companion present, controller status idle */
    case TANGOX_USB_CTL_BASE + 0xc:
        return 0x000f9931;
    case TANGOX_GPIO_BASE + 0x10:
        return 0;
    case TANGOX_GPIO_BASE + 4:
        return 0;
    case TANGOX_GPIO_BASE + 8:
        return 0;
    case TANGOX_IR_BASE + 0x2c:
        return 0;
    case TANGOX_FIP_BASE + 0x4c:
    case TANGOX_FIP_BASE + 0x50:
    case TANGOX_FIP_BASE + 0x54:
        return 0;
    case TANGOX_SCARD_BASE + 0x3c:
        return 0;
    case TANGOX_SCARD_BASE + 0x40:
        return 0;
    case TANGOX_SCARD_BASE + 0x4c:
        return 0x00000001; /* card absent, transmitter idle */
    case TANGOX_ENET_DMA_BASE + 0x14:
        return 0x00000000; /* DMA stopped */
    case 0x6c21c: /* Front-panel LED UART status: transmitter ready. */
        return 0x20;
    case 0x6c204: /* Front-panel LED UART data register. */
        return s->led_uart_data;
    case CPU_IRQ_STATUS:
        return s->irq_status;
    case CPU_EDGE_RAWSTAT:
        return s->irq_status;  /* kernel polls this for live pending bits */
    case CPU_IRQ_ENABLE_SET:
        return s->irq_enable;
    case CPU_REMAP:
        return s->remap[0];
    case CPU_REMAP1:
        return s->remap[1];
    case CPU_REMAP2:
        return s->remap[2];
    case CPU_REMAP3:
        return s->remap[3];
    case CPU_REMAP4:
        return s->remap[4];
    default:
        break;
    }

    if (addr / 4 < ARRAY_SIZE(s->regs)) {
        value = s->regs[addr / 4];
    }
    return value;
}

static uint64_t smp863x_regs_read(void *opaque, hwaddr addr, unsigned size)
{
    SMP863XState *s = opaque;
    hwaddr aligned = addr & ~3;
    uint32_t value;

    if (size != 1 && size != 2 && size != 4) {
        return 0;
    }
    value = smp863x_regs_read32(s, aligned);
    return (value >> ((addr & 3) * 8)) & ((UINT64_C(1) << (size * 8)) - 1);
}

static void smp863x_regs_write(void *opaque, hwaddr addr, uint64_t data,
                                unsigned size)
{
    SMP863XState *s = opaque;
    hwaddr aligned = addr & ~3;
    uint32_t value;

    if (size != 1 && size != 2 && size != 4) {
        return;
    }
    value = smp863x_regs_read32(s, aligned);
    if (size == 4) {
        value = data;
    } else {
        uint32_t mask = (UINT32_C(1) << (size * 8)) - 1;
        unsigned shift = (addr & 3) * 8;
        value = (value & ~(mask << shift)) | ((data & mask) << shift);
    }
    addr = aligned;

    /* The six inbound PCI regions translate bus addresses to DRAM. Linux
     * programs these before issuing SATA DMA, including PRD table reads. */
    if (addr >= 0x29008 && addr <= 0x2901c) {
        unsigned region = (addr - 0x29008) / 4;
        uint64_t offset = (uint64_t)value - SMP863X_DRAM_BASE;
        bool valid = value >= SMP863X_DRAM_BASE &&
                     offset < memory_region_size(s->ram);

        memory_region_transaction_begin();
        if (valid) {
            memory_region_set_alias_offset(&s->pci_dma_regions[region], offset);
            memory_region_set_size(&s->pci_dma_regions[region],
                                   MIN(16 * MiB,
                                       memory_region_size(s->ram) - offset));
        }
        memory_region_set_enabled(&s->pci_dma_regions[region], valid);
        memory_region_transaction_commit();
    }

    switch (addr) {
    case 0x6c204: /* LED controller receives the selected segment byte. */
        s->led_uart_data = value;
        return;
    case 0x6c21c: /* LED UART command/status; writes complete immediately. */
        s->led_uart_status = 0x20;
        return;
    case 0x27000: /* Ethernet DMA reset completes immediately. */
        s->regs[addr / 4] = value & ~1U;
        return;
    case CPU_IRQ_ENABLE_SET:
        s->irq_enable |= value;
        smp863x_update_irq(s);
        return;
    case CPU_IRQ_ENABLE_CLR:
        s->irq_enable &= ~value;
        smp863x_update_irq(s);
        return;
    case CPU_IRQ_RAWSTAT:
        s->irq_status &= ~value;
        smp863x_update_irq(s);
        return;
    case CPU_EDGE_RAWSTAT:
        /* kernel writes 1<<bit here to ack edge-triggered IRQs */
        s->irq_status &= ~value;
        smp863x_update_irq(s);
        return;
    case CPU_TIMER0_CTRL:
        s->timer_enabled = (value & 0x80) != 0;
        if (s->timer_enabled) {
            uint64_t ticks = s->timer_load ? s->timer_load : 39257;
            uint64_t ns = (ticks * 512ULL * 1000000000ULL) / 201000000ULL;
            timer_mod_ns(s->timer,
                         qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + ns);
        } else {
            timer_del(s->timer);
        }
        return;
    case CPU_TIMER0_LOAD:
        s->timer_load = value & 0xffff;
        return;
    case CPU_TIMER0_CLR:
        s->irq_status &= ~(1U << 5);
        smp863x_update_irq(s);
        return;
    case CPU_REMAP:
        s->remap[0] = value & 0xfc000000U;
        return;
    case CPU_REMAP1:
        s->remap[1] = value & 0xfc000000U;
        return;
    case CPU_REMAP2:
        s->remap[2] = value & 0xfc000000U;
        return;
    case CPU_REMAP3:
        s->remap[3] = value & 0xfc000000U;
        return;
    case CPU_REMAP4:
        s->remap[4] = value & 0xfc000000U;
        return;
    default:
        break;
    }

    if (addr / 4 < ARRAY_SIZE(s->regs)) {
        s->regs[addr / 4] = value;
    }
}

static uint64_t smp863x_sysregs_read(void *opaque, hwaddr addr, unsigned size)
{
    return smp863x_regs_read(opaque, addr + 0x10000, size);
}

static void smp863x_sysregs_write(void *opaque, hwaddr addr, uint64_t data,
                                   unsigned size)
{
    smp863x_regs_write(opaque, addr + 0x10000, data, size);
}

static const MemoryRegionOps smp863x_sysregs_ops = {
    .read = smp863x_sysregs_read,
    .write = smp863x_sysregs_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static uint64_t smp863x_cpuregs_read(void *opaque, hwaddr addr, unsigned size)
{
    return smp863x_regs_read(opaque, addr + 0x60000, size);
}

static void smp863x_cpuregs_write(void *opaque, hwaddr addr, uint64_t data,
                                   unsigned size)
{
    smp863x_regs_write(opaque, addr + 0x60000, data, size);
}

static const MemoryRegionOps smp863x_cpuregs_ops = {
    .read = smp863x_cpuregs_read,
    .write = smp863x_cpuregs_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static uint64_t smp863x_irqhandler_read(void *opaque, hwaddr addr, unsigned size)
{
    return smp863x_regs_read(opaque, addr + 0xe0000, size);
}

static void smp863x_irqhandler_write(void *opaque, hwaddr addr, uint64_t data,
                                   unsigned size)
{
    smp863x_regs_write(opaque, addr + 0xe0000, data, size);
}

static const MemoryRegionOps smp863x_irqhandler_ops = {
    .read = smp863x_irqhandler_read,
    .write = smp863x_irqhandler_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
};

static void smp863x_cpu_reset(void *opaque)
{
    SMP863XState *s = opaque;
    CPUMIPSState *env = &s->cpu->env;
    cpu_reset(CPU(s->cpu));
    env->CP0_Status &= ~((1 << CP0St_BEV) | (1 << CP0St_ERL));
    if (s->flash_boot) {
        env->active_tc.gpr[4] = 0;
        env->active_tc.gpr[5] = 0;
        env->active_tc.PC = s->reset_pc;
        return;
    }
    /* Enter the extracted image directly, using the same empty argument
     * tuple as the firmware handoff. */
    env->active_tc.gpr[4] = 0;
    env->active_tc.gpr[5] = 0;
    env->active_tc.gpr[2] = 0x820;
    env->active_tc.gpr[3] = 0x820;
    /* The image is entered by zboot after its cache flush helper.  The
     * helper leaves the end of the loaded DRAM range in s0; preserve a
     * concrete range here so the first cache walk terminates. */
    env->active_tc.gpr[16] = 0x4000U;
    env->active_tc.gpr[17] = 0;
    env->active_tc.gpr[29] = SMP863X_DRAM_BASE + 0x04000000U + 0x2000;
    stl_le_p(memory_region_get_ram_ptr(s->ram) + 0x04000000U + 0x2000 + 0x10,
             0x90000410U);
    env->active_tc.gpr[31] = 0x90000410U;
    env->active_tc.PC = 0x90000408U;

}

static bool smp863x_find_romfs(const uint8_t *flash, size_t size,
                               size_t start, size_t *romfs)
{
    size_t off;

    for (off = start; off + 8 <= size; off += 0x1000) {
        if (!memcmp(flash + off, "-rom1fs-", 8)) {
            *romfs = off;
            return true;
        }
    }
    return false;
}

static bool smp863x_load_zbf(SMP863XState *s, const uint8_t *flash,
                              size_t flash_size, size_t search_start)
{
    size_t romfs, file, name_end, data, zbf, compressed_size;
    uint32_t next, payload_size, load_addr, start_addr;
    uint8_t *image;
    ssize_t image_size;

    if (!smp863x_find_romfs(flash, flash_size, search_start, &romfs)) {
        return false;
    }
    /* ROMFS records are aligned and the XRPC file follows . and ... */
    file = romfs + 0x60;
    next = ldl_be_p(flash + file);
    name_end = file + 16;
    while (name_end < flash_size && flash[name_end]) {
        name_end++;
    }
    data = (name_end + 1 + 15) & ~15ULL;
    if (data >= flash_size || next == 0) {
        return false;
    }
    payload_size = ldl_be_p(flash + file + 8);
    if (data + payload_size > flash_size || payload_size < 0x360) {
        return false;
    }
    /* Find the embedded ZBF header after the XRPC clear headers. */
    zbf = data + 0x20;
    while (zbf + 32 < data + payload_size &&
           memcmp(flash + zbf, "FNIB", 4)) {
        zbf += 4;
    }
    if (ldl_le_p(flash + zbf) != 0x42494e46 ||
        !(ldl_le_p(flash + zbf + 4) & 0x10)) {
        return false;
    }
    load_addr = ldl_le_p(flash + zbf + 8);
    start_addr = ldl_le_p(flash + zbf + 12);
    compressed_size = ldl_le_p(flash + zbf + 24);
    if (compressed_size < 32 || zbf + 32 + compressed_size > flash_size) {
        return false;
    }
    image = g_malloc(8 * MiB);
    image_size = gunzip(image, 8 * MiB,
                        (uint8_t *)flash + zbf + 32, compressed_size);
    if (image_size < 0 || (load_addr & 0x1fffffff) < SMP863X_DRAM_BASE ||
        (load_addr & 0x1fffffff) + image_size >
            SMP863X_DRAM_BASE + memory_region_size(s->ram)) {
        g_free(image);
        return false;
    }
    memcpy(memory_region_get_ram_ptr(s->ram) +
           ((load_addr & 0x1fffffff) - SMP863X_DRAM_BASE), image,
           image_size);
    g_free(image);
    s->reset_pc = start_addr;
    return true;
}

static void smp863x_init_common(MachineState *machine, bool front_panel_uart)
{
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *regs = g_new(MemoryRegion, 1);
    MemoryRegion *cpuregs = g_new(MemoryRegion, 1);
    MemoryRegion *xenv = g_new(MemoryRegion, 1);
    MemoryRegion *lowram = g_new(MemoryRegion, 1);
    SMP863XState *s = g_new0(SMP863XState, 1);
    MIPSCPU *cpu;
    Clock *cpuclk;
    qemu_irq *uart_irqs;
    SerialMM *uart0, *uart1;
    ssize_t kernel_size;
    DriveInfo *dinfo;
    PFlashCFI01 *pflash;
    uint8_t *flash_data = NULL;

    if (machine->ram_size < 80 * MiB) {
        error_report("smp863x requires at least 80 MiB of RAM");
        exit(EXIT_FAILURE);
    }

    cpuclk = clock_new(OBJECT(machine), "cpu-refclk");
    /* The XG1000 crystal/PLL presents a 301.5 MHz CPU clock.  The kernel
     * derives its high precision timer from half of this rate (150.75 MHz).
     */
    clock_set_hz(cpuclk, 301500000);
    cpu = mips_cpu_create_with_clock(machine->cpu_type, cpuclk, false);
    s->cpu = cpu;
    s->ram = machine->ram;
    /* Linux reports the board's CS2/XENV bus address, while the emulator
     * stores the block at its remapped physical address (0x0c000000). */
    s->xenv_location = 0x48000000U;
    cpu_mips_irq_init_cpu(cpu);
    cpu_mips_clock_init(cpu);
    s->cpu_irq = cpu->env.irq[2];
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, smp863x_timer_cb, s);
    qemu_register_reset(smp863x_cpu_reset, s);

    memory_region_add_subregion(sysmem, SMP863X_DRAM_BASE, machine->ram);
    /* Match the board's persistent zboot memory configuration.  Without a
     * valid record zboot falls back to a single 64 MiB DRAM bank. */
    {
        uint8_t *memcfg = memory_region_get_ram_ptr(machine->ram) + 0xfc0;
        uint32_t sum = 0;
        memset(memcfg, 0, 64);
        stl_le_p(memcfg + 0x00, 0x6766636d); /* MEMCFG_SIGNATURE */
        stl_le_p(memcfg + 0x04, 0x08000000); /* DRAM0: 128 MiB */
        stl_le_p(memcfg + 0x08, 0x08000000); /* DRAM1: 128 MiB */
        for (unsigned i = 0; i < 16; i++) {
            sum += ldl_le_p(memcfg + i * 4);
        }
        stl_le_p(memcfg + 0x2c, (uint32_t)(0 - sum));
    }
    /* Early cache probing uses the uncached low physical DRAM window. */
    memory_region_init_ram(lowram, NULL, "smp863x-lowram", 64 * KiB,
                           &error_fatal);
    {
        uint8_t *p = memory_region_get_ram_ptr(lowram);
        /* Minimal one-entry cache descriptor chain used by the ES6 ROM. */
        stl_le_p(p + 0x9f0, 0x1000);
        stl_le_p(p + 0xa00, 0xa20);
        stl_le_p(p + 0x1000 + 0x90, 0);
        stl_le_p(p + 0x820, 0xa20);
        stl_le_p(p + 0xa20, 0);
    }
    memory_region_add_subregion(sysmem, 0, lowram);
    /* The kernel's uncached KSEG1 cache flush helper probes this address. */
    {
        MemoryRegion *cache_probe = g_new(MemoryRegion, 1);
        memory_region_init_alias(cache_probe, NULL, "smp863x-cache-probe",
                                 lowram, 0, 64 * KiB);
    memory_region_add_subregion(sysmem, 0xa0000000ULL, cache_probe);
    }

    dinfo = drive_get(IF_PFLASH, 0, 0);
    if (dinfo) {
        flash_data = g_malloc(SMP863X_PFLASH_SIZE);
        if (blk_pread(blk_by_legacy_dinfo(dinfo), 0, SMP863X_PFLASH_SIZE,
                      flash_data, 0) < 0 ||
            !smp863x_load_zbf(s, flash_data, SMP863X_PFLASH_SIZE, 0x40000)) {
            error_report("invalid SMP863x parallel flash boot image");
            g_free(flash_data);
            exit(EXIT_FAILURE);
        }
        s->flash_boot = true;
        s->flash_data = flash_data;
        s->flash_data_size = SMP863X_PFLASH_SIZE;
    }
    /* The input image has a small capture footer, so use it only as the
     * source snapshot and keep the emulated CFI geometry canonical. */
    pflash = pflash_cfi01_register(SMP863X_PFLASH_BASE, "smp863x.pflash",
                                   SMP863X_PFLASH_SIZE, NULL,
                                   0x10000, 2, 0x0000, 0x0000, 0x0000, 0x0000,
                                   DEVICE_LITTLE_ENDIAN);
    s->flash = pflash_cfi01_get_memory(pflash);
    if (dinfo && blk_attach_dev(blk_by_legacy_dinfo(dinfo), DEVICE(pflash)) < 0) {
        error_report("could not claim SMP863x parallel flash backend");
        exit(EXIT_FAILURE);
    }
    if (flash_data) {
        memcpy(memory_region_get_ram_ptr(s->flash), flash_data,
               SMP863X_PFLASH_SIZE);
    }
    s->remap[0] = SMP863X_PFLASH_BASE;
    s->remap[4] = 0x48000000U;
    /* Remap windows expose the same parallel NOR contents at the GBUS
     * addresses used by XOS and zboot. */
    for (int i = 0; i < 4; i++) {
        hwaddr base = (hwaddr[]){ SMP863X_REMAP1_BASE,
                                  SMP863X_REMAP2_BASE,
                                  SMP863X_REMAP3_BASE,
                                  SMP863X_REMAP4_BASE }[i];
        for (int j = 0; j < 8; j++) {
            MemoryRegion *alias = g_new(MemoryRegion, 1);
            memory_region_init_alias(alias, NULL, "smp863x-pflash-window",
                                     s->flash, 0, SMP863X_PFLASH_SIZE);
            if (i != 0 || j != 0) {
                memory_region_add_subregion(sysmem, base + j * 8 * MiB,
                                            alias);
            }
            alias = g_new(MemoryRegion, 1);
            memory_region_init_alias(alias, NULL, "smp863x-pflash-kseg1",
                                     s->flash, 0, SMP863X_PFLASH_SIZE);
            memory_region_add_subregion(sysmem,
                                        0xa0000000ULL + base + j * 8 * MiB,
                                        alias);
            alias = g_new(MemoryRegion, 1);
            memory_region_init_alias(alias, NULL, "smp863x-pflash-kseg0",
                                     s->flash, 0, SMP863X_PFLASH_SIZE);
            memory_region_add_subregion(sysmem,
                                        0x80000000ULL + base + j * 8 * MiB,
                                        alias);
        }
    }
    /* The hardware maps a selected 64 MiB source through each remap window.
     * Keep the CFI device for native flash commands, but service boot reads
     * from the image snapshot so changing CPU_remapx takes effect. */
    for (int i = 1; i < 4; i++) {
        hwaddr base = (hwaddr[]){ SMP863X_REMAP2_BASE,
                                  SMP863X_REMAP3_BASE,
                                  SMP863X_REMAP4_BASE }[i - 1];
        for (int kseg = 0; kseg < 3; kseg++) {
            SMP863XRemapWindow *window = g_new0(SMP863XRemapWindow, 1);
            MemoryRegion *region = g_new(MemoryRegion, 1);
            window->s = s;
            window->index = i + 1;
            memory_region_init_io(region, NULL, &smp863x_remap_ops, window,
                                  "smp863x-remap-window",
                                  SMP863X_REMAP_WINDOW_SIZE);
            memory_region_add_subregion_overlap(sysmem,
                                                (kseg == 0 ? base :
                                                 (kseg == 1 ?
                                                  0x80000000ULL + base :
                                                  0xa0000000ULL + base)),
                                                region, 2);
        }
    }
    {
        MemoryRegion *pflash_kseg1 = g_new(MemoryRegion, 1);
        memory_region_init_alias(pflash_kseg1, NULL, "smp863x-pflash-reset",
                                 s->flash, 0, SMP863X_PFLASH_SIZE);
        memory_region_add_subregion(sysmem, 0xbfc00000ULL, pflash_kseg1);
        MemoryRegion *pflash_cs2 = g_new(MemoryRegion, 1);
        memory_region_init_alias(pflash_cs2, NULL, "smp863x-pflash-cs2",
                                 s->flash, 0, SMP863X_PFLASH_SIZE);
        memory_region_add_subregion(sysmem, 0x48000000ULL, pflash_cs2);
        MemoryRegion *pflash_cs3 = g_new(MemoryRegion, 1);
        memory_region_init_io(pflash_cs3, NULL, &smp863x_flash_window_ops, s,
                              "smp863x-pflash-cs3", 0x04000000ULL);
        memory_region_add_subregion(sysmem, 0x4c000000ULL, pflash_cs3);
    }
    if (s->flash_boot) {
        MemoryRegion *boot_alias = g_new(MemoryRegion, 1);
        memory_region_init_alias(boot_alias, NULL, "smp863x-remapped-boot",
                                 machine->ram, 0x01200000, 8 * MiB);
        memory_region_add_subregion_overlap(sysmem, 0xbfc00000ULL,
                                            boot_alias, 2);
        boot_alias = g_new(MemoryRegion, 1);
        memory_region_init_alias(boot_alias, NULL, "smp863x-remapped-boot-phys",
                                 machine->ram, 0x01200000, 8 * MiB);
        memory_region_add_subregion_overlap(sysmem, SMP863X_PFLASH_BASE,
                                            boot_alias, 2);
    }
    memory_region_init_io(regs, NULL, &smp863x_sysregs_ops, s,
                          "smp863x-system-registers", 0x50000);
    memory_region_init_io(cpuregs, NULL, &smp863x_cpuregs_ops, s,
                          "smp863x-cpu-registers", 0x10000);
    if (!s->flash_boot) {
        memory_region_init_ram(xenv, NULL, "smp863x-xenv", 0x4000, &error_fatal);
        memcpy(memory_region_get_ram_ptr(xenv), smp863x_xenv,
               sizeof(smp863x_xenv));
        memory_region_add_subregion(sysmem, SMP863X_XENV_PHYS, xenv);
    }

    uart_irqs = qemu_allocate_irqs(smp863x_uart_irq, s, 2);
    uart0 = serial_mm_tangox_init(sysmem, CPU_UART0_BASE, uart_irqs[0], 115200,
                                  serial_hd(0), DEVICE_LITTLE_ENDIAN);
    uart1 = serial_mm_tangox_init(sysmem, CPU_UART1_BASE, uart_irqs[1], 115200,
                                  serial_hd(1), DEVICE_LITTLE_ENDIAN);
    {
        MemoryRegion *uart0_kseg1 = g_new(MemoryRegion, 1);
        MemoryRegion *uart1_kseg1 = g_new(MemoryRegion, 1);
        MemoryRegion *uart0_phys = g_new(MemoryRegion, 1);
        MemoryRegion *uart1_phys = g_new(MemoryRegion, 1);
        memory_region_init_alias(uart0_phys, NULL, "smp863x-uart0-phys",
                                 sysbus_mmio_get_region(SYS_BUS_DEVICE(uart0), 0),
                                 0, 0x100);
        memory_region_init_alias(uart1_phys, NULL, "smp863x-uart1-phys",
                                 sysbus_mmio_get_region(SYS_BUS_DEVICE(uart1), 0),
                                 0, 0x100);
        memory_region_init_alias(uart0_kseg1, NULL, "smp863x-uart0-kseg1",
                                 sysbus_mmio_get_region(SYS_BUS_DEVICE(uart0), 0),
                                 0, 0x100);
        memory_region_init_alias(uart1_kseg1, NULL, "smp863x-uart1-kseg1",
                                 sysbus_mmio_get_region(SYS_BUS_DEVICE(uart1), 0),
                                 0, 0x100);
        memory_region_add_subregion_overlap(sysmem,
                                            0xa0000000ULL + CPU_UART0_BASE,
                                            uart0_kseg1, 1);
        memory_region_add_subregion_overlap(sysmem,
                                            0xa0000000ULL + CPU_UART1_BASE,
                                            uart1_kseg1, 1);
        memory_region_add_subregion_overlap(sysmem, CPU_UART0_BASE,
                                            uart0_phys, 1);
        memory_region_add_subregion_overlap(sysmem, CPU_UART1_BASE,
                                            uart1_phys, 1);
        MemoryRegion *uart0_pc = g_new(MemoryRegion, 1);
        memory_region_init_alias(uart0_pc, NULL, "smp863x-uart0-pc",
                                 sysbus_mmio_get_region(SYS_BUS_DEVICE(uart0), 0),
                                 0, 0x100);
        memory_region_add_subregion_overlap(sysmem, 0x3f8, uart0_pc, 1);
    }
    if (front_panel_uart) {
        MemoryRegion *front_panel = g_new(MemoryRegion, 1);
        MemoryRegion *front_panel_kseg1 = g_new(MemoryRegion, 1);
        memory_region_init_io(front_panel, NULL, &smp863x_front_panel_ops,
                              NULL, "xg1000-front-panel-uart1", 0x100);
        memory_region_init_alias(front_panel_kseg1, NULL,
                                 "xg1000-front-panel-uart1-kseg1",
                                 front_panel, 0, 0x100);
        memory_region_add_subregion_overlap(sysmem, CPU_UART1_BASE,
                                            front_panel, 2);
        memory_region_add_subregion_overlap(sysmem,
                                            0xa0000000ULL + CPU_UART1_BASE,
                                            front_panel_kseg1, 2);
    }
    memory_region_add_subregion(sysmem, 0x10000, regs);
    memory_region_add_subregion(sysmem, SMP863X_REG_BASE, cpuregs);
    {
        MemoryRegion *irqhandler = g_new(MemoryRegion, 1);

        memory_region_init_io(irqhandler, NULL, &smp863x_irqhandler_ops, s,
                              "smp863x-irq-handler-registers", 0x10000);
        memory_region_add_subregion(sysmem, 0xe0000, irqhandler);
    }

    /* TangoX exposes a standard OHCI 1.1 register block at host-interface
     * offset 0x1500.  The kernel's tangox-ohci driver probes this block
     * directly, so provide the real OHCI core instead of an empty window. */
    {
        DeviceState *ehci = qdev_new("platform-ehci-usb");
        SysBusDevice *ehci_bus = SYS_BUS_DEVICE(ehci);
        sysbus_realize_and_unref(ehci_bus, &error_fatal);
        memory_region_add_subregion_overlap(sysmem, TANGOX_USB_EHCI_BASE,
                                            sysbus_mmio_get_region(ehci_bus, 0), 4);
    }
    {
        DeviceState *ohci = qdev_new(TYPE_SYSBUS_OHCI);
        SysBusDevice *ohci_bus = SYS_BUS_DEVICE(ohci);
        object_property_set_uint(OBJECT(ohci), "num-ports", 2, &error_fatal);
        sysbus_realize_and_unref(ohci_bus, &error_fatal);
        memory_region_add_subregion_overlap(sysmem, TANGOX_USB_OHCI_BASE,
                                            sysbus_mmio_get_region(ohci_bus, 0),
                                            4);
    }

    /* XG1000 has no separate PB/MMIO IDE device.  Its only external storage
     * controller is the PCI SiI3112 below; IF_IDE backends are consumed there
     * so -drive if=ide can describe the board's SATA disks (or DVD image). */
    /* Tangox exposes a conventional PCI root bus through fixed physical
     * windows.  The config window is directly encoded as bus/devfn/offset;
     * PCI BARs are mapped into the corresponding I/O and memory windows. */
    memory_region_init(&s->pci_io, NULL, "smp863x-pci-io",
                       SMP863X_PCI_IO_SIZE);
    memory_region_add_subregion(sysmem, SMP863X_PCI_IO_BASE, &s->pci_io);
    memory_region_init(&s->pci_memory, NULL, "smp863x-pci-memory",
                       SMP863X_PCI_MEMORY_SIZE);
    memory_region_add_subregion(sysmem, SMP863X_PCI_MEMORY_BASE,
                                &s->pci_memory);
    DeviceState *pci_host = qdev_new("smp863x-pci-host");
    s->pci_bus = pci_root_bus_new(pci_host, "pci",
                                  &s->pci_memory, &s->pci_io, 0,
                                  TYPE_PCI_BUS);
    PCI_HOST_BRIDGE(pci_host)->bus = s->pci_bus;
    memory_region_init(&s->pci_dma, OBJECT(pci_host), "smp863x-pci-dma",
                       UINT64_C(1) << 32);
    address_space_init(&s->pci_dma_as, &s->pci_dma, "smp863x-pci-dma");
    for (unsigned i = 0; i < ARRAY_SIZE(s->pci_dma_regions); i++) {
        memory_region_init_alias(&s->pci_dma_regions[i], OBJECT(pci_host),
                                 "smp863x-pci-dma-region", machine->ram,
                                 0, 16 * MiB);
        memory_region_set_enabled(&s->pci_dma_regions[i], false);
        memory_region_add_subregion(&s->pci_dma, 0x12000000ULL + i * 16 * MiB,
                                    &s->pci_dma_regions[i]);
    }
    pci_setup_iommu(s->pci_bus, &smp863x_pci_iommu_ops, s);
    pci_bus_map_irqs(s->pci_bus, smp863x_pci_map_irq);
    pci_bus_irqs(s->pci_bus, smp863x_pci_set_irq, s, 4);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(pci_host), &error_fatal);
    /* The XG1000 board carries a four-port Silicon Image 3114 at IDSEL 4. */
    {
        PCIDevice *sata = pci_create_simple(s->pci_bus, PCI_DEVFN(4, 0),
                                            "sii3114");
        sata->config[PCI_INTERRUPT_LINE] = 24;
        const char *ids[] = { "xga", "xgb", "xgc", "xgd" };
        for (int port = 0; port < 4; port++) {
            BlockBackend *blk = blk_by_name(ids[port]);
            DriveInfo *named = blk ? blk_legacy_dinfo(blk) : NULL;
            if (blk && !named) {
                named = g_new0(DriveInfo, 1);
                named->type = IF_IDE;
                named->bus = 0;
                named->unit = port;
                blk_set_legacy_dinfo(blk, named);
            }
            if (named) {
                ide_bus_create_drive(sii3114_get_bus(sata, port), 0, named);
            }
        }
    }
    memory_region_init_io(&s->pci_config, NULL, &smp863x_pci_config_ops, s,
                          "smp863x-pci-config", SMP863X_PCI_CONFIG_SIZE);
    memory_region_add_subregion(sysmem, SMP863X_PCI_CONFIG_BASE,
                                &s->pci_config);
    {
        MemoryRegion *sysregs_kseg1 = g_new(MemoryRegion, 1);
        MemoryRegion *cpuregs_kseg1 = g_new(MemoryRegion, 1);
        memory_region_init_alias(sysregs_kseg1, NULL, "smp863x-system-kseg1",
                                 regs, 0, 0x50000);
        memory_region_init_alias(cpuregs_kseg1, NULL, "smp863x-cpu-kseg1",
                                 cpuregs, 0, 0x10000);
        memory_region_add_subregion(sysmem, 0xa0010000ULL, sysregs_kseg1);
        memory_region_add_subregion(sysmem, 0xa0060000ULL, cpuregs_kseg1);
    }

    if (!machine->kernel_filename && !s->flash_boot) {
        error_report("smp863x requires -pflash or -kernel");
        exit(EXIT_FAILURE);
    }
    if (s->flash_boot) {
        return;
    }
    kernel_size = load_image_targphys(machine->kernel_filename,
                                      SMP863X_DRAM_BASE, machine->ram_size, NULL);
    if (kernel_size < 0) {
        error_report("could not load SMP863x kernel '%s'",
                     machine->kernel_filename);
        exit(EXIT_FAILURE);
    }
    /* The stock image probes the TangoX cache with PREF on low physical
     * addresses. QEMU's generic MIPS cache model raises an exception for
     * this SoC-specific probe; turn that single hint into a nop so startup
     * can proceed to the serial and platform setup. */
    if (kernel_size > 0x18ede4) {
        uint8_t *image = rom_ptr(SMP863X_DRAM_BASE, kernel_size);
        /* 0x901aede0: jr ra; nop. */
        stl_le_p(image + 0x18ede0, 0x03e00008);
        stl_le_p(image + 0x18ede4, 0);
    }
}

static void smp863x_init(MachineState *machine)
{
    smp863x_init_common(machine, false);
}

static void smp863x_machine_init(MachineClass *mc)
{
    mc->desc = "Sigma Designs SMP863x TangoX";
    mc->init = smp863x_init;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("34Kf");
    mc->default_ram_size = 80 * MiB;
    mc->default_ram_id = "smp863x.ram";
    mc->max_cpus = 1;
}

static void xg1000_init(MachineState *machine)
{
    smp863x_init_common(machine, true);
}

static void xg1000_machine_init(MachineClass *mc)
{
    mc->desc = "Osung Midicom XG1000 karaoke player";
    mc->init = xg1000_init;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("SMP8634");
    mc->default_ram_size = 128 * MiB;
    mc->default_ram_id = "xg1000.ram";
    mc->max_cpus = 1;
}

DEFINE_MACHINE("smp863x", smp863x_machine_init)
DEFINE_MACHINE("xg1000", xg1000_machine_init)
