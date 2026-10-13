/* Sigma Designs TangoX / SMP863x machine. */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/timer.h"
#include "qemu/bcd.h"
#include "system/rtc.h"
#include "hw/i2c/bitbang_i2c.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "hw/char/serial-mm.h"
#include "hw/usb/hcd-ohci.h"
#include "hw/usb/hcd-ehci.h"
#include "hw/block/flash.h"
#include "hw/nvram/eeprom_at24c.h"
#include "hw/ide/mmio.h"
#include "hw/ide/pci.h"
#include "hw/ide/sii3114.h"
#include "hw/core/boards.h"
#include "hw/core/clock.h"
#include "hw/core/irq.h"
#include "hw/core/loader.h"
#include "hw/core/qdev.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_host.h"
#include "hw/mips/mips.h"
#include "system/memory.h"
#include "system/address-spaces.h"
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
#define LR_XPU_STAGE            0x61ffc
#define XPU_IRQ_SOFTSET         0xee010
#define SOFT_IRQ_XRPC           (1U << 4)
#define SOFT_IRQ_IH_UNINSTALL   (1U << 9)
#define RM_OK                  6
#define RM_NOTIMPLEMENTED      11
#define RM_INVALID_PARAMETER  15
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
    bool irq_handler_installed;
    bitbang_i2c_interface rtc_i2c;
    int rtc_sda;
} SMP863XState;

/* uart.ko uses a 16-bit FPGA FIFO interface, not a 16550 register bank. */
typedef struct XG1000UART {
    MemoryRegion mmio;
    CharFrontend chr;
    uint16_t baud;
} XG1000UART;

static uint64_t xg1000_uart_read(void *opaque, hwaddr addr, unsigned size)
{
    /* No peripheral is attached: both FIFO counts are zero. */
    return 0;
}

static void xg1000_uart_write(void *opaque, hwaddr addr,
                             uint64_t value, unsigned size)
{
    XG1000UART *s = opaque;

    if (addr == 0) {
        uint8_t byte = value;
        qemu_chr_fe_write_all(&s->chr, &byte, 1);
    } else if (addr == 8) {
        s->baud = value;
    }
}

static const MemoryRegionOps xg1000_uart_ops = {
    .read = xg1000_uart_read,
    .write = xg1000_uart_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 2, .max_access_size = 2 },
};

static void xg1000_uart_init(MemoryRegion *sysmem)
{
    static const hwaddr bases[] = { 0x4c009000, 0x4c010800, 0x4c012800 };

    for (unsigned i = 0; i < ARRAY_SIZE(bases); i++) {
        XG1000UART *s = g_new0(XG1000UART, 1);
        g_autofree char *name = g_strdup_printf("xg1000-fpga-uart%u", i + 1);
        Chardev *chr = serial_hd(i + 2);

        if (!chr) {
            chr = qemu_chr_new(name, "vc:80Cx24C", NULL);
        }
        qemu_chr_fe_init(&s->chr, chr, &error_fatal);
        memory_region_init_io(&s->mmio, NULL, &xg1000_uart_ops, s, name, 0x800);
        memory_region_add_subregion_overlap(sysmem, bases[i], &s->mmio, 3);
    }
}

/* PCF8593 clock/calendar and RAM used by the XG1000 firmware.
 * Alarm/event-counter modes and the unconnected interrupt pin are not modeled.
 */
#define TYPE_XG1000_RTC "xg1000-pcf8593"
OBJECT_DECLARE_SIMPLE_TYPE(XG1000RTCState, XG1000_RTC)

struct XG1000RTCState {
    I2CSlave parent_obj;
    uint8_t ram[16];
    uint8_t ptr;
    bool address_byte;
    int64_t offset;
};

static void xg1000_rtc_capture(XG1000RTCState *s)
{
    struct tm now;

    if (s->ram[0] & 0x80) { /* STOP */
        return;
    }
    qemu_get_timedate(&now, s->offset);
    s->ram[1] = 0;
    s->ram[2] = to_bcd(now.tm_sec);
    s->ram[3] = to_bcd(now.tm_min);
    s->ram[4] = to_bcd(now.tm_hour);
    s->ram[5] = to_bcd(now.tm_mday) | ((now.tm_year & 3) << 6);
    s->ram[6] = to_bcd(now.tm_mon + 1) | (now.tm_wday << 5);
}

static int xg1000_rtc_event(I2CSlave *dev, enum i2c_event event)
{
    XG1000RTCState *s = XG1000_RTC(dev);

    if (event == I2C_START_SEND) {
        s->address_byte = true;
    } else if (event == I2C_START_RECV) {
        xg1000_rtc_capture(s);
    }
    return 0;
}

static uint8_t xg1000_rtc_recv(I2CSlave *dev)
{
    XG1000RTCState *s = XG1000_RTC(dev);

    uint8_t value = s->ram[s->ptr];

    s->ptr = (s->ptr + 1) & 0x0f;
    return value;
}

static int xg1000_rtc_send(I2CSlave *dev, uint8_t data)
{
    XG1000RTCState *s = XG1000_RTC(dev);
    struct tm now;
    unsigned reg;

    if (s->address_byte) {
        s->ptr = data & 0x0f;
        s->address_byte = false;
        return 0;
    }
    reg = s->ptr;
    s->ptr = (s->ptr + 1) & 0x0f;
    if (reg >= 2 && reg <= 6) {
        qemu_get_timedate(&now, s->offset);
        switch (reg) {
        case 2:
            now.tm_sec = from_bcd(data & 0x7f);
            break;
        case 3:
            now.tm_min = from_bcd(data & 0x7f);
            break;
        case 4:
            now.tm_hour = from_bcd(data & 0x3f);
            break;
        case 5:
            now.tm_mday = from_bcd(data & 0x3f);
            now.tm_year = (now.tm_year & ~3) | (data >> 6);
            break;
        case 6:
            now.tm_mon = from_bcd(data & 0x1f) - 1;
            break;
        }
        s->offset = qemu_timedate_diff(&now);
    }
    s->ram[reg] = data;
    return 0;
}

static void xg1000_rtc_realize(DeviceState *dev, Error **errp)
{
    XG1000RTCState *s = XG1000_RTC(dev);
    struct tm now;
    unsigned year;

    qemu_get_timedate(&now, 0);
    year = now.tm_year + 1900;
    /* Firmware-specific BCD year storage in battery-backed RAM. */
    s->ram[8] = to_bcd(year / 100);
    s->ram[9] = to_bcd(year % 100);
    xg1000_rtc_capture(s);
}

static void xg1000_rtc_class_init(ObjectClass *klass, const void *data)
{
    I2CSlaveClass *ic = I2C_SLAVE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    ic->event = xg1000_rtc_event;
    ic->recv = xg1000_rtc_recv;
    ic->send = xg1000_rtc_send;
    dc->realize = xg1000_rtc_realize;
    dc->desc = "XG1000 PCF8593 clock/calendar";
}

static const TypeInfo xg1000_rtc_type = {
    .name = TYPE_XG1000_RTC,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(XG1000RTCState),
    .class_init = xg1000_rtc_class_init,
};

static void xg1000_rtc_register_type(void)
{
    type_register_static(&xg1000_rtc_type);
}

type_init(xg1000_rtc_register_type)

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

static uint64_t smp863x_remap_read(void *opaque, hwaddr addr, unsigned size)
{
    SMP863XRemapWindow *window = opaque;
    SMP863XState *s = window->s;
    uint64_t value = 0;

    if ((s->remap[window->index] == 0x48000000U ||
         s->remap[window->index] == SMP863X_PFLASH_BASE) &&
        addr + size <= SMP863X_PFLASH_SIZE) {
        memory_region_dispatch_read(s->flash, addr, &value, size_memop(size),
                                    MEMTXATTRS_UNSPECIFIED);
        return value;
    }

    if (s->remap[window->index] >= SMP863X_DRAM_BASE &&
        (uint64_t)s->remap[window->index] + addr + size <=
            SMP863X_DRAM_BASE + memory_region_size(s->ram)) {
        uint8_t bytes[8] = { 0 };
        address_space_read(&address_space_memory,
                           s->remap[window->index] + addr,
                           MEMTXATTRS_UNSPECIFIED, bytes, size);
        return ldq_le_p(bytes);
    }

    /* Boot reads use the image snapshot, independent of CFI query mode. */
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
    SMP863XRemapWindow *window = opaque;
    SMP863XState *s = window->s;

    if (s->remap[window->index] && addr + size <= SMP863X_PFLASH_SIZE) {
        memory_region_dispatch_write(s->flash, addr, data, size_memop(size),
                                     MEMTXATTRS_UNSPECIFIED);
    }
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

    if (addr >= 0xee000 && addr <= 0xee314 &&
        (addr & 0x300) != 0x200 &&
        ((addr & 0xff) == 0xc || (addr & 0xff) == 0x14)) {
        return s->regs[(addr & ~4ULL) / 4];
    }

    switch (addr) {
    case 0x10504:
        return (s->regs[addr / 4] & ~2U) | (s->rtc_sda << 1);
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

/* Model the XOS mailbox protocol without claiming to execute XPU firmware. */
static void smp863x_xrpc(SMP863XState *s)
{
    uint32_t request = s->regs[LR_XPU_STAGE / 4];
    uint8_t header[36];
    uint32_t result = RM_INVALID_PARAMETER;
    uint32_t id = UINT32_MAX;
    uint64_t ram_end = SMP863X_DRAM_BASE + memory_region_size(s->ram);

    /* Requests must reside entirely in DRAM; never recurse into MMIO. */
    if (!(request & 3) && request >= SMP863X_DRAM_BASE &&
        (uint64_t)request + sizeof(header) <= ram_end &&
        address_space_read(&address_space_memory, request,
                           MEMTXATTRS_UNSPECIFIED, header,
                           sizeof(header)) == MEMTX_OK) {
        uint32_t length = ldl_le_p(header + 28);

        id = ldl_le_p(header + 4);
        if (length >= 32 && (uint64_t)request + length <= ram_end) {
            result = RM_NOTIMPLEMENTED;
            /* Clear CT1 CPU code: transport the task to param1.  Zboot
             * parses and decompresses its ZBF header in guest code. */
            if (id == 5 && header[34] == 1 && header[35] == 0xff) {
                uint32_t count = ldl_le_p(header + 8);
                uint32_t dest = ldl_le_p(header + 12);
                if (count && length >= 32 + count &&
                    length - count == 0x324 &&
                    dest >= SMP863X_DRAM_BASE &&
                    (uint64_t)dest + count <= ram_end) {
                    uint8_t *ram = memory_region_get_ram_ptr(s->ram);
                    memmove(ram + dest - SMP863X_DRAM_BASE,
                            ram + request + length - count - SMP863X_DRAM_BASE,
                            count);
                    result = RM_OK;
                } else {
                    result = RM_INVALID_PARAMETER;
                }
            }
            /* XLOAD certificate type 6 installs the IRQ handler. Its
             * encrypted payload is represented by host-side behavior,
             * rather than decoded or executed by an auxiliary CPU. */
            if (id == 5 && length >= 32 + 1044 && header[34] == 6 &&
                ldl_le_p(header + 8) == length - 32 - 1044) {
                s->irq_handler_installed = true;
                result = RM_OK;
            }
        }
    }
    qemu_log_mask(LOG_GUEST_ERROR,
                  "smp863x XRPC: request=%08x id=%u result=%u ih=%u\n",
                  request, id, result, s->irq_handler_installed);
    s->regs[LR_XPU_STAGE / 4] = result;
    s->regs[XPU_IRQ_SOFTSET / 4] &= ~SOFT_IRQ_XRPC;
}

static void smp863x_regs_write(void *opaque, hwaddr addr, uint64_t data,
                                unsigned size)
{
    SMP863XState *s = opaque;
    hwaddr aligned = addr & ~3;
    uint32_t value;
    uint32_t write_bits;

    if (size != 1 && size != 2 && size != 4) {
        return;
    }
    write_bits = (uint32_t)data << ((addr & 3) * 8);
    value = smp863x_regs_read32(s, aligned);
    if (size == 4) {
        value = data;
    } else {
        uint32_t mask = (UINT32_C(1) << (size * 8)) - 1;
        unsigned shift = (addr & 3) * 8;
        value = (value & ~(mask << shift)) | ((data & mask) << shift);
    }
    addr = aligned;

    /* GPIO writes use high-half enables and low-half values. GPIO0/1
     * carry the board's bit-banged SCL/SDA; inputs release the bus high.
     */
    if (addr == 0x10500 || addr == 0x10504 ||
        addr == 0x2710c || addr == 0x27110) {
        uint32_t mask = write_bits >> 16;
        s->regs[addr / 4] = (s->regs[addr / 4] & ~mask) |
                              (write_bits & mask);
        if (addr == 0x10500 || addr == 0x10504) {
            uint32_t dir = s->regs[0x10500 / 4];
            uint32_t pins = s->regs[0x10504 / 4] | ~dir;
            s->rtc_sda = bitbang_i2c_set(&s->rtc_i2c, BITBANG_I2C_SDA,
                                        (pins >> 1) & 1);
            s->rtc_sda = bitbang_i2c_set(&s->rtc_i2c, BITBANG_I2C_SCL,
                                        pins & 1);
        }
        return;
    }

    /* XPU IRQ/FIQ/IIQ enable and software-request set/clear pairs. */
    if (addr >= 0xee000 && addr <= 0xee314 &&
        (addr & 0x300) != 0x200) {
        unsigned reg = addr & 0xff;
        if (reg == 8 || reg == 0xc || reg == 0x10 || reg == 0x14) {
            hwaddr set = addr & ~4ULL;
            uint32_t bits = write_bits;

            if (reg & 4) {
                s->regs[set / 4] &= ~bits;
            } else {
                s->regs[set / 4] |= bits;
            }
            if (addr == XPU_IRQ_SOFTSET) {
                if (bits & SOFT_IRQ_XRPC) {
                    smp863x_xrpc(s);
                }
                if (bits & SOFT_IRQ_IH_UNINSTALL) {
                    s->irq_handler_installed = false;
                    s->regs[set / 4] &= ~SOFT_IRQ_IH_UNINSTALL;
                }
            }
            return;
        }
    }

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
        s->remap[0] = value & 0xffff0000U;
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

static uint64_t smp863x_media_read(void *opaque, hwaddr addr, unsigned size)
{
    return smp863x_regs_read(opaque, addr + 0x70000, size);
}

static void smp863x_media_write(void *opaque, hwaddr addr, uint64_t data,
                                  unsigned size)
{
    smp863x_regs_write(opaque, addr + 0x70000, data, size);
}

static const MemoryRegionOps smp863x_media_ops = {
    .read = smp863x_media_read,
    .write = smp863x_media_write,
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

/* XOS hands the main CPU the clear CT0 task selected by XENV x.boot.
 * Authentication and XPU execution are not emulated here. */
static bool smp863x_load_zboot(SMP863XState *s, const uint8_t *flash,
                               size_t flash_size)
{
    uint32_t env_size, boot = UINT32_MAX, length, task_size, load;
    size_t pos, payload;

    if (flash_size < 24) {
        return false;
    }
    env_size = ldl_le_p(flash);
    if (env_size < 24 || env_size > flash_size) {
        return false;
    }
    for (pos = 24; pos + 2 <= env_size;) {
        size_t len = ((flash[pos] & 15) << 8) | flash[pos + 1];
        if (len < 3 || len > env_size - pos) {
            return false;
        }
        if (len == 13 && !memcmp(flash + pos + 2, "x.boot", 7)) {
            boot = ldl_le_p(flash + pos + 9);
            break;
        }
        pos += len;
    }
    if (boot >= 0x48000000U && boot < 0x48000000U + flash_size) {
        boot -= 0x48000000U;
    }
    if (boot > flash_size || flash_size - boot < 36) {
        return false;
    }
    task_size = ldl_le_p(flash + boot + 8);
    load = ldl_le_p(flash + boot + 12);
    length = ldl_le_p(flash + boot + 28);
    if (ldl_le_p(flash + boot + 4) != 5 || flash[boot + 35] != 0xff ||
        flash[boot + 34] != 0 || length > flash_size - boot ||
        length < 32 || task_size > length - 32 || !task_size ||
        load < SMP863X_DRAM_BASE ||
        (uint64_t)load + task_size >
            SMP863X_DRAM_BASE + memory_region_size(s->ram)) {
        return false;
    }
    payload = boot + length - task_size;
    memcpy(memory_region_get_ram_ptr(s->ram) + load - SMP863X_DRAM_BASE,
           flash + payload, task_size);
    s->remap[0] = load;
    /* XOS remaps the physical reset window to the loaded CT0 task. */
    s->reset_pc = 0xbfc00000U;
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
    PFlashCFI02 *pflash;
    uint8_t *flash_data = NULL;

    if (machine->ram_size < 80 * MiB) {
        error_report("smp863x requires at least 80 MiB of RAM");
        exit(EXIT_FAILURE);
    }

    {
        DeviceState *bridge = qdev_new(TYPE_GPIO_I2C);
        I2CBus *bus;

        sysbus_realize_and_unref(SYS_BUS_DEVICE(bridge), &error_fatal);
        bus = I2C_BUS(qdev_get_child_bus(bridge, "i2c"));
        bitbang_i2c_init(&s->rtc_i2c, bus);
        s->rtc_i2c.current_addr = -1;
        s->rtc_sda = 1;
        i2c_slave_create_simple(bus, TYPE_XG1000_RTC, 0x51);
        /* A0/A1/A2 are low: AT24C02 uses 7-bit address 0x50. */
        if (blk_by_name("xgeeprom")) {
            I2CSlave *eeprom = i2c_slave_new("at24c-eeprom", 0x50);
            qdev_prop_set_uint32(DEVICE(eeprom), "rom-size", 256);
            qdev_prop_set_drive(DEVICE(eeprom), "drive",
                                blk_by_name("xgeeprom"));
            i2c_slave_realize_and_unref(eeprom, bus, &error_fatal);
        } else {
            uint8_t *erased = g_malloc(256);
            memset(erased, 0xff, 256);
            at24c_eeprom_init_rom(bus, 0x50, 256, erased, 256);
        }
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
            !smp863x_load_zboot(s, flash_data, SMP863X_PFLASH_SIZE)) {
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
    /* S29GL064M: one x16 AMD-command-set chip, 128 64 KiB sectors. */
    pflash = pflash_cfi02_register(SMP863X_PFLASH_BASE, "smp863x.pflash",
                                 SMP863X_PFLASH_SIZE, NULL,
                                 0x10000, 1, 2, 0x0001, 0x227e, 0x220c, 0x2201,
                                 0x555, 0x2aa, 0);
    s->flash = sysbus_mmio_get_region(SYS_BUS_DEVICE(pflash), 0);
    if (dinfo && blk_attach_dev(blk_by_legacy_dinfo(dinfo), DEVICE(pflash)) < 0) {
        error_report("could not claim SMP863x parallel flash backend");
        exit(EXIT_FAILURE);
    }
    if (flash_data) {
        memcpy(memory_region_get_ram_ptr(s->flash), flash_data,
               SMP863X_PFLASH_SIZE);
    }
    if (!s->flash_boot) {
        s->remap[0] = SMP863X_PFLASH_BASE;
    }
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
                                 machine->ram, s->remap[0] - SMP863X_DRAM_BASE, 64 * KiB);
        memory_region_add_subregion_overlap(sysmem, 0xbfc00000ULL,
                                            boot_alias, 2);
        boot_alias = g_new(MemoryRegion, 1);
        memory_region_init_alias(boot_alias, NULL, "smp863x-remapped-boot-phys",
                                 machine->ram, s->remap[0] - SMP863X_DRAM_BASE, 64 * KiB);
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
                                  serial_hd(1) ?: qemu_chr_new("smp863x-uart1",
                                      "vc:80Cx24C", NULL),
                                  DEVICE_LITTLE_ENDIAN);
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
        xg1000_uart_init(sysmem);
    }
    memory_region_add_subregion(sysmem, 0x10000, regs);
    memory_region_add_subregion(sysmem, SMP863X_REG_BASE, cpuregs);
    {
        MemoryRegion *media = g_new(MemoryRegion, 1);
        MemoryRegion *risc_ram = g_new(MemoryRegion, 1);

        /* Display and six media-engine register blocks. */
        memory_region_init_io(media, NULL, &smp863x_media_ops, s,
                              "smp863x-media-registers", 0x70000);
        memory_region_add_subregion(sysmem, 0x70000, media);
        /* Six RISC program/data RAM pairs, each 128 KiB. */
        memory_region_init_ram(risc_ram, NULL, "smp863x-media-risc-ram",
                               0xc0000, &error_fatal);
        memory_region_add_subregion(sysmem, 0x100000, risc_ram);
    }
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
