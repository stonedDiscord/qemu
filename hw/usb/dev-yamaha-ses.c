/* Experimental Yamaha SES USB transport for the XG1000. */
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/usb/usb.h"
#include "migration/vmstate.h"
#include "desc.h"

typedef struct YamahaSES {
    USBDevice parent_obj;
    uint8_t tx[0x10000];
    unsigned tx_count, tx_expected, tx_finished;
    uint8_t rx[8];
    unsigned rx_count, rx_offset;
} YamahaSES;

static void ses_reset(USBDevice *dev)
{
    YamahaSES *s = (YamahaSES *)dev;

    s->tx_count = s->tx_expected = s->tx_finished = 0;
    s->rx_count = s->rx_offset = 0;
}

static const USBDescStrings ses_strings = {
    [1] = "YAMAHA Corporation",
    [2] = "YAMAHA SES Board",
};

static const USBDescIface ses_iface = {
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_VENDOR_SPEC,
    .eps = (USBDescEndpoint[]) {
        { .bEndpointAddress = USB_DIR_IN | 1,
          .bmAttributes = USB_ENDPOINT_XFER_BULK, .wMaxPacketSize = 64 },
        { .bEndpointAddress = USB_DIR_OUT | 2,
          .bmAttributes = USB_ENDPOINT_XFER_BULK, .wMaxPacketSize = 64 },
    },
};

static const USBDescDevice ses_device = {
    .bcdUSB = 0x0110,
    .bMaxPacketSize0 = 8,
    .bNumConfigurations = 1,
    .confs = (USBDescConfig[]) {
        {
            .bNumInterfaces = 1,
            .bConfigurationValue = 1,
            .bmAttributes = USB_CFG_ATT_ONE,
            .bMaxPower = 50,
            .nif = 1,
            .ifs = &ses_iface,
        },
    },
};

static const USBDesc ses_desc = {
    /* ses.ko matches Yamaha 0499:4007 and 0499:13f0. */
    .id = { .idVendor = 0x0499, .idProduct = 0x4007,
            .bcdDevice = 0x0100, .iManufacturer = 1, .iProduct = 2 },
    .full = &ses_device,
    .str = ses_strings,
};

static void ses_realize(USBDevice *dev, Error **errp)
{
    usb_desc_init(dev);
}

static void ses_control(USBDevice *dev, USBPacket *p, int request,
                        int value, int index, int length, uint8_t *data)
{
    YamahaSES *s = (YamahaSES *)dev;

    if (usb_desc_handle_control(dev, p, request, value, index, length,
                                data) >= 0) {
        return;
    }

    /* sesSetSetup counts four-byte words in little endian. SEMP headers
     * and payloads use big endian. Only connect/disconnect are modeled. */
    if (request >= 0xc200 && request <= 0xc203 && index == 0 && length == 2) {
        unsigned words = 0;

        if (request == 0xc200 && !s->rx_count && value > 0 && value <= 0x3fff) {
            s->tx_count = 0;
            s->tx_finished = 0;
            s->tx_expected = value * 4;
            words = value;
        } else if (request == 0xc201) {
            if (s->tx_expected && s->tx_count == s->tx_expected) {
                s->tx_finished = s->tx_count / 4;
            }
            words = s->tx_finished;
            if (s->tx_count == s->tx_expected && s->tx_count == 4 &&
                s->tx[0] == 0 && (s->tx[1] == 1 || s->tx[1] == 2) &&
                s->tx[2] == 0 && s->tx[3] == 0) {
                memset(s->rx, 0, sizeof(s->rx));
                s->rx[0] = 0x80; /* response */
                s->rx[1] = s->tx[1];
                s->rx[3] = 1; /* one word: success status */
                s->rx_count = sizeof(s->rx);
                s->rx_offset = 0;
            }
            s->tx_count = s->tx_expected = 0;
        } else if (request == 0xc202) {
            words = s->rx_count / 4;
            s->rx_offset = 0;
        } else if (request == 0xc203 && s->rx_offset == s->rx_count) {
            s->rx_count = s->rx_offset = 0;
        }
        data[0] = words;
        data[1] = words >> 8;
        p->actual_length = 2;
        return;
    }
    p->status = USB_RET_STALL;
}

static void ses_data(USBDevice *dev, USBPacket *p)
{
    YamahaSES *s = (YamahaSES *)dev;
    unsigned count;

    if (p->pid == USB_TOKEN_OUT && p->ep->nr == 2 && s->tx_expected &&
        p->iov.size <= s->tx_expected - s->tx_count) {
        count = p->iov.size;
        usb_packet_copy(p, s->tx + s->tx_count, count);
        s->tx_count += count;
        return;
    }
    if (p->pid == USB_TOKEN_IN && p->ep->nr == 1 &&
        s->rx_offset < s->rx_count) {
        count = MIN(p->iov.size, s->rx_count - s->rx_offset);
        usb_packet_copy(p, s->rx + s->rx_offset, count);
        s->rx_offset += count;
        return;
    }
    p->status = USB_RET_NAK;
}

static const VMStateDescription ses_vmstate = {
    .name = "yamaha-ses",
    .unmigratable = 1,
};

static void ses_class_init(ObjectClass *klass, const void *data)
{
    USBDeviceClass *uc = USB_DEVICE_CLASS(klass);

    uc->product_desc = "Yamaha SES USB transport";
    uc->usb_desc = &ses_desc;
    uc->realize = ses_realize;
    uc->handle_reset = ses_reset;
    uc->handle_control = ses_control;
    uc->handle_data = ses_data;
    DEVICE_CLASS(klass)->vmsd = &ses_vmstate;
}

static const TypeInfo ses_type = {
    .name = "usb-yamaha-ses",
    .parent = TYPE_USB_DEVICE,
    .instance_size = sizeof(YamahaSES),
    .class_init = ses_class_init,
};

static void ses_register_types(void)
{
    type_register_static(&ses_type);
}

type_init(ses_register_types)
