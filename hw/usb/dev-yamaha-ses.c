/* Experimental Yamaha SES USB transport for the XG1000. */
#include "qemu/osdep.h"
#include "qemu/module.h"
#include "hw/usb/usb.h"
#include "migration/vmstate.h"
#include "desc.h"

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
    if (usb_desc_handle_control(dev, p, request, value, index, length,
                                data) >= 0) {
        return;
    }

    /* sesSetSetup uses vendor/endpoint IN requests with a two-byte
     * little-endian word count. Requests 0/2 reserve send/receive space;
     * 1/3 finish transfers. There is no DSP backend yet: advertise zero
     * send capacity and no received data, rather than accept lost writes.
     * The guest retries empty RECV_START at its configured interval. */
    if (request >= 0xc200 && request <= 0xc203 && index == 0 && length == 2) {
        data[0] = 0;
        data[1] = 0;
        p->actual_length = 2;
        return;
    }
    p->status = USB_RET_STALL;
}

static void ses_data(USBDevice *dev, USBPacket *p)
{
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
    uc->handle_control = ses_control;
    uc->handle_data = ses_data;
    DEVICE_CLASS(klass)->vmsd = &ses_vmstate;
}

static const TypeInfo ses_type = {
    .name = "usb-yamaha-ses",
    .parent = TYPE_USB_DEVICE,
    .instance_size = sizeof(USBDevice),
    .class_init = ses_class_init,
};

static void ses_register_types(void)
{
    type_register_static(&ses_type);
}

type_init(ses_register_types)
