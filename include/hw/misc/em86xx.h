/* Local XG1000 digital video feed. */
#ifndef HW_MISC_EM86XX_H
#define HW_MISC_EM86XX_H

#include "hw/core/qdev.h"

bool em86xx_tft_scanout(DeviceState *dev, uint32_t *dst, unsigned width,
                        unsigned height, unsigned stride);

#endif
