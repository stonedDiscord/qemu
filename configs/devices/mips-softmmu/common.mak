# Common mips*-softmmu CONFIG defines

# Uncomment the following lines to disable these optional devices:
# CONFIG_PCI_DEVICES=n
# CONFIG_TEST_DEVICES=n

# Boards are selected by default, uncomment to keep out of the build.
# CONFIG_MALTA=n

# TangoX uses the generic sysbus OHCI core for its on-chip USB host.
CONFIG_USB_OHCI_SYSBUS=y
# TangoX also exposes an EHCI 2.0 controller at host-interface + 0x1400.
CONFIG_USB_EHCI=y
CONFIG_USB_EHCI_SYSBUS=y
# The XG1000 kernel expects its system disk on a SATA controller.
CONFIG_IDE_SII3112=y
