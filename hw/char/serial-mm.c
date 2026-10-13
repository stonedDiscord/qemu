/*
 * QEMU 16550A UART emulation
 *
 * Copyright (c) 2003-2004 Fabrice Bellard
 * Copyright (c) 2008 Citrix Systems, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "hw/char/serial-mm.h"
#include "exec/cpu-common.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "hw/core/qdev-properties.h"

static uint64_t serial_mm_read(void *opaque, hwaddr addr, unsigned size)
{
    SerialMM *s = SERIAL_MM(opaque);
    unsigned reg = addr >> s->regshift;

    if (s->tangox_layout) {
        switch (reg) {
        case 0: reg = 0; break;
        case 2: reg = 1; break;
        case 3: reg = 2; break;
        case 5: reg = 3; break;
        case 6: reg = 4; break;
        case 7: reg = 5; break;
        case 8: reg = 6; break;
        case 9: reg = 7; break;
        case 10: return s->serial.divider;
        default: return 0;
        }
        /* TangoX has a separate divisor register; DLAB never aliases RBR/IER. */
        uint8_t lcr = s->serial.lcr;
        s->serial.lcr &= ~0x80;
        uint64_t value = serial_io_ops.read(&s->serial, reg, 1);
        s->serial.lcr = lcr;
        return value;
    }
    return serial_io_ops.read(&s->serial, reg, 1);
}

static void serial_mm_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    SerialMM *s = SERIAL_MM(opaque);
    unsigned reg = addr >> s->regshift;

    if (s->tangox_layout) {
        switch (reg) {
        case 1: reg = 0; break;
        case 2: reg = 1; break;
        case 4: reg = 2; break;
        case 5: reg = 3; break;
        case 6: reg = 4; break;
        case 9: reg = 7; break;
        case 10:
            /* Program both bytes through the 16550 divisor latch. */
            {
                uint8_t lcr = s->serial.lcr;
                s->serial.lcr |= 0x80;
                serial_io_ops.write(&s->serial, 0, value & 255, 1);
                serial_io_ops.write(&s->serial, 1, (value >> 8) & 255, 1);
                s->serial.lcr = lcr;
            }
            return;
        default: return;
        }
        if (reg == 0 || reg == 1) {
            uint8_t lcr = s->serial.lcr;
            s->serial.lcr &= ~0x80;
            serial_io_ops.write(&s->serial, reg, value & 255, 1);
            s->serial.lcr = lcr;
            return;
        }
    }
    value &= 255;
    serial_io_ops.write(&s->serial, reg, value, 1);
}

static const MemoryRegionOps serial_mm_ops[] = {
    [DEVICE_NATIVE_ENDIAN] = {
        .read = serial_mm_read,
        .write = serial_mm_write,
        .endianness = DEVICE_NATIVE_ENDIAN,
        .valid.max_access_size = 8,
        .impl.max_access_size = 8,
    },
    [DEVICE_LITTLE_ENDIAN] = {
        .read = serial_mm_read,
        .write = serial_mm_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
        .valid.max_access_size = 8,
        .impl.max_access_size = 8,
    },
    [DEVICE_BIG_ENDIAN] = {
        .read = serial_mm_read,
        .write = serial_mm_write,
        .endianness = DEVICE_BIG_ENDIAN,
        .valid.max_access_size = 8,
        .impl.max_access_size = 8,
    },
};

static void serial_mm_realize(DeviceState *dev, Error **errp)
{
    SerialMM *smm = SERIAL_MM(dev);
    SerialState *s = &smm->serial;

    if (!qdev_realize(DEVICE(s), NULL, errp)) {
        return;
    }

    memory_region_init_io(&s->io, OBJECT(dev),
                          &serial_mm_ops[smm->endianness], smm, "serial",
                          smm->tangox_layout ? 0x100 : 8 << smm->regshift);
    sysbus_init_mmio(SYS_BUS_DEVICE(smm), &s->io);
    sysbus_init_irq(SYS_BUS_DEVICE(smm), &smm->serial.irq);
}

static const VMStateDescription vmstate_serial_mm = {
    .name = "serial",
    .version_id = 3,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
    VMSTATE_STRUCT(serial, SerialMM, 0, vmstate_serial, SerialState),
        VMSTATE_BOOL(tangox_layout, SerialMM),
        VMSTATE_END_OF_LIST()
    }
};

SerialMM *serial_mm_init(MemoryRegion *address_space,
                         hwaddr base, int regshift,
                         qemu_irq irq, int baudbase,
                         Chardev *chr, enum device_endian end)
{
    SerialMM *smm = SERIAL_MM(qdev_new(TYPE_SERIAL_MM));
    MemoryRegion *mr;

    qdev_prop_set_uint8(DEVICE(smm), "regshift", regshift);
    qdev_prop_set_uint32(DEVICE(smm), "baudbase", baudbase);
    qdev_prop_set_chr(DEVICE(smm), "chardev", chr);
    qdev_set_legacy_instance_id(DEVICE(smm), base, 2);
    qdev_prop_set_uint8(DEVICE(smm), "endianness", end);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(smm), &error_fatal);

    sysbus_connect_irq(SYS_BUS_DEVICE(smm), 0, irq);
    mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(smm), 0);
    memory_region_add_subregion(address_space, base, mr);

    return smm;
}

SerialMM *serial_mm_tangox_init(MemoryRegion *address_space,
                                hwaddr base, qemu_irq irq, int baudbase,
                                Chardev *chr, enum device_endian end)
{
    SerialMM *smm = SERIAL_MM(qdev_new(TYPE_SERIAL_MM));
    MemoryRegion *mr;

    qdev_prop_set_uint8(DEVICE(smm), "regshift", 2);
    qdev_prop_set_uint32(DEVICE(smm), "baudbase", baudbase);
    qdev_prop_set_chr(DEVICE(smm), "chardev", chr);
    qdev_set_legacy_instance_id(DEVICE(smm), base, 2);
    qdev_prop_set_uint8(DEVICE(smm), "endianness", end);
    smm->tangox_layout = true;
    sysbus_realize_and_unref(SYS_BUS_DEVICE(smm), &error_fatal);
    sysbus_connect_irq(SYS_BUS_DEVICE(smm), 0, irq);
    mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(smm), 0);
    memory_region_add_subregion(address_space, base, mr);
    return smm;
}

static void serial_mm_instance_init(Object *o)
{
    SerialMM *smm = SERIAL_MM(o);

    object_initialize_child(o, "serial", &smm->serial, TYPE_SERIAL);

    qdev_alias_all_properties(DEVICE(&smm->serial), o);
}

static const Property serial_mm_properties[] = {
    /*
     * Set the spacing between adjacent memory-mapped UART registers.
     * Each register will be at (1 << regshift) bytes after the previous one.
     */
    DEFINE_PROP_UINT8("regshift", SerialMM, regshift, 0),
    DEFINE_PROP_UINT8("endianness", SerialMM, endianness, DEVICE_NATIVE_ENDIAN),
};

static void serial_mm_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_props(dc, serial_mm_properties);
    dc->realize = serial_mm_realize;
    dc->vmsd = &vmstate_serial_mm;
}

static const TypeInfo types[] = {
    {
        .name = TYPE_SERIAL_MM,
        .parent = TYPE_SYS_BUS_DEVICE,
        .class_init = serial_mm_class_init,
        .instance_init = serial_mm_instance_init,
        .instance_size = sizeof(SerialMM),
    },
};

DEFINE_TYPES(types)
