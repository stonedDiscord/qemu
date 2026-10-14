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
#include "qemu/audio.h"
#include "ui/console.h"
#include "hw/core/qdev-properties-system.h"
#ifdef CONFIG_XG1000_AUDIO_FFMPEG
#include <libavcodec/avcodec.h>
#include <libswresample/swresample.h>
#endif

#define TYPE_EM86XX "em86xx"
OBJECT_DECLARE_SIMPLE_TYPE(EM86XXState, EM86XX)

typedef struct AudioSlot {
    EM86XXState *owner;
    unsigned offset;
    bool playing;
    SWVoiceOut *voice;
#ifdef CONFIG_XG1000_AUDIO_FFMPEG
    AVCodecParserContext *parser;
    AVCodecContext *decoder;
    SwrContext *resampler;
    int sample_rate;
    enum AVSampleFormat sample_format;
    AVChannelLayout layout;
    uint8_t pcm[65536];
    size_t pcm_pos, pcm_size;
#endif
} AudioSlot;

struct EM86XXState {
    SysBusDevice parent_obj;
    MemoryRegion regs_region;
    MemoryRegion risc_ram;
    MemoryRegion dmem_mutex[5];
    uint8_t regs[0x70000];
    uint32_t read_count;
    bool audio_stub;
    MemoryRegion audio_commands[4];
    AudioSlot audio_slots[4];
    QEMUTimer *audio_timer;
    uint32_t audio_pending;
    uint32_t audio_blocked;
    AudioBackend *audio_be;
    bool display_assist;
    QEMUTimer *display_timer;
    QemuConsole *console;
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

static bool em86xx_task_address(hwaddr addr)
{
    return !(addr & 3) && addr >= 0x10000000 && addr < 0x18000000 - 0x60;
}

static void em86xx_audio_discard(AudioSlot *slot)
{
    slot->playing = false;
    if (slot->voice) {
        audio_be_set_active_out(slot->owner->audio_be, slot->voice, false);
    }
#ifdef CONFIG_XG1000_AUDIO_FFMPEG
    av_parser_close(slot->parser);
    slot->parser = NULL;
    avcodec_free_context(&slot->decoder);
    swr_free(&slot->resampler);
    av_channel_layout_uninit(&slot->layout);
    slot->pcm_pos = slot->pcm_size = 0;
#endif
}

static void em86xx_audio_out(void *opaque, int available)
{
#ifdef CONFIG_XG1000_AUDIO_FFMPEG
    AudioSlot *slot = opaque;
    if (slot->playing && slot->pcm_pos < slot->pcm_size) {
        slot->pcm_pos += audio_be_write(slot->owner->audio_be, slot->voice,
            slot->pcm + slot->pcm_pos,
            MIN((size_t)available, slot->pcm_size - slot->pcm_pos));
    }
#endif
}

static void em86xx_audio_decode(AudioSlot *slot)
{
#ifdef CONFIG_XG1000_AUDIO_FFMPEG
    EM86XXState *s = slot->owner;
    uint8_t *ram = memory_region_get_ram_ptr(&s->risc_ram);
    hwaddr fifo = 0x100000 + slot->offset - 0x58 +
                  ((slot - s->audio_slots) % 2 ? 4 : 0);
    uint32_t base, size, rd, wr, count;
    uint8_t input[4096 + AV_INPUT_BUFFER_PADDING_SIZE] = { 0 };
    AVPacket *packet;
    AVFrame *frame;
    int consumed;

    if (!slot->playing || slot->pcm_pos != slot->pcm_size ||
        (s->audio_blocked & (1U << (slot - s->audio_slots))) ||
        (ldl_le_p(ram + slot->offset + 16) & 0xff) != 2) {
        return;
    }
    base = em86xx_shared_read(fifo);
    size = em86xx_shared_read(fifo + 4);
    rd = em86xx_shared_read(fifo + 8);
    wr = em86xx_shared_read(fifo + 12);
    /* FIFO indices are byte offsets, not guest pointers. Never follow a
     * firmware-upload word or a malformed descriptor outside guest DRAM.
     */
    if (base < 0x10000000 || base >= 0x18000000 || !size ||
        size > 0x18000000 - base || rd >= size || wr >= size || rd == wr) {
        return;
    }
    count = MIN(sizeof(input) - AV_INPUT_BUFFER_PADDING_SIZE,
                MIN(wr > rd ? wr - rd : size - rd, size - rd));
    if (address_space_read(&address_space_memory, base + rd,
                           MEMTXATTRS_UNSPECIFIED, input, count) != MEMTX_OK) {
        return;
    }
    if (!slot->parser) {
        slot->parser = av_parser_init(AV_CODEC_ID_MP3);
        slot->decoder = avcodec_alloc_context3(NULL);
        if (!slot->parser || !slot->decoder) {
            em86xx_audio_discard(slot);
            return;
        }
    }
    packet = av_packet_alloc();
    frame = av_frame_alloc();
    if (!packet || !frame) {
        av_packet_free(&packet);
        av_frame_free(&frame);
        return;
    }
    consumed = av_parser_parse2(slot->parser, slot->decoder,
                               &packet->data, &packet->size, input, count,
                               AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
    if (consumed > 0) {
        em86xx_shared_write(fifo + 8, (rd + consumed) % size);
    }
    if (packet->size >= 4) {
        unsigned layer = (packet->data[1] >> 1) & 3;
        enum AVCodecID id = layer == 3 ? AV_CODEC_ID_MP1 :
                            layer == 2 ? AV_CODEC_ID_MP2 : AV_CODEC_ID_MP3;
        if (!slot->decoder->codec || slot->decoder->codec_id != id) {
            avcodec_free_context(&slot->decoder);
            slot->decoder = avcodec_alloc_context3(avcodec_find_decoder(id));
            if (!slot->decoder || avcodec_open2(slot->decoder,
                    avcodec_find_decoder(id), NULL) < 0) {
                em86xx_audio_discard(slot);
                goto done;
            }
        }
        if (avcodec_send_packet(slot->decoder, packet) < 0 ||
            avcodec_receive_frame(slot->decoder, frame) < 0) {
            goto done;
        }
        if (!slot->resampler || slot->sample_rate != frame->sample_rate ||
            slot->sample_format != frame->format ||
            av_channel_layout_compare(&slot->layout, &frame->ch_layout)) {
            AVChannelLayout stereo = AV_CHANNEL_LAYOUT_STEREO;
            swr_free(&slot->resampler);
            av_channel_layout_uninit(&slot->layout);
            av_channel_layout_copy(&slot->layout, &frame->ch_layout);
            slot->sample_rate = frame->sample_rate;
            slot->sample_format = frame->format;
            if (swr_alloc_set_opts2(&slot->resampler, &stereo, AV_SAMPLE_FMT_S16,
                    44100, &frame->ch_layout, frame->format,
                    frame->sample_rate, 0, NULL) < 0 ||
                swr_init(slot->resampler) < 0) {
                swr_free(&slot->resampler);
                goto done;
            }
        }
        uint8_t *out = slot->pcm;
        int samples = swr_convert(slot->resampler, &out, sizeof(slot->pcm) / 4,
                                  (const uint8_t **)frame->extended_data,
                                  frame->nb_samples);
        if (samples > 0) {
            slot->pcm_pos = 0;
            slot->pcm_size = samples * 4;
        }
    }
done:
    av_frame_free(&frame);
    /* Parser output aliases the input or the parser's own buffer. */
    packet->data = NULL;
    packet->size = 0;
    av_packet_free(&packet);
#endif
}

static bool em86xx_dram_range(hwaddr base, uint32_t size)
{
    return base >= 0x10000000 && base < 0x18000000 &&
           size <= 0x18000000 - base;
}

/* WRITE_PTS_DMEM_32/16 tasks reserve the decoder FIFO's write pointer
 * before queuing a 24-byte host buffer descriptor. Copy from the task's
 * independent transfer cursor before letting the decoder see those bytes.
 * Completion advances rd only; the driver owns the descriptor eraser.
 */
static void em86xx_audio_transfer(EM86XXState *s, hwaddr task)
{
    hwaddr fifo = em86xx_shared_read(task + 0x30);
    uint32_t mode = em86xx_shared_read(task + 0x2c);
    uint32_t queue = em86xx_shared_read(task + 0x44);
    uint32_t qsize = em86xx_shared_read(task + 0x48);
    uint32_t qrd = em86xx_shared_read(task + 0x50);
    uint32_t qwr = em86xx_shared_read(task + 0x54);
    unsigned i;

    for (i = 0; i < ARRAY_SIZE(s->audio_slots); i++) {
        hwaddr expected = 0x100000 + s->audio_slots[i].offset - 0x58 +
                          (i % 2 ? 4 : 0);
        if (fifo == expected) {
            break;
        }
    }
    if (i == ARRAY_SIZE(s->audio_slots)) {
        return;
    }
    if (qrd != qwr) {
        s->audio_blocked |= 1U << i;
    }
    if (mode > 1 ||
        em86xx_shared_read(task + 0x1c) != 0 ||
        !em86xx_dram_range(queue, qsize) || qsize < 24 ||
        qrd >= qsize || qwr >= qsize) {
        return;
    }
    /* Bound work per tick, including malformed descriptors. */
    for (unsigned n = 0; n < 8 && qrd != qwr; n++) {
        uint32_t bytes, source, base, size, cursor, offset;
        uint8_t buffer[4096];
        if (qsize - qrd < 24 ||
            (qwr >= qrd ? qwr - qrd : qsize - qrd + qwr) < 24) {
            break;
        }
        bytes = em86xx_shared_read(queue + qrd);
        source = em86xx_shared_read(queue + qrd + 4);
        base = em86xx_shared_read(fifo);
        size = em86xx_shared_read(fifo + 4);
        cursor = em86xx_shared_read(task + 0x40);
        if (!em86xx_dram_range(source, bytes) ||
            !em86xx_dram_range(base, size) || !size || bytes >= size ||
            bytes > 1024 * 1024 ||
            cursor < base || cursor >= (hwaddr)base + size) {
            break;
        }
        offset = cursor - base;
        for (uint32_t copied = 0; copied < bytes;) {
            uint32_t count = MIN(sizeof(buffer), MIN(bytes - copied,
                                                     size - offset));
            if (address_space_read(&address_space_memory, source + copied,
                MEMTXATTRS_UNSPECIFIED, buffer, count) != MEMTX_OK ||
                address_space_write(&address_space_memory, base + offset,
                MEMTXATTRS_UNSPECIFIED, buffer, count) != MEMTX_OK) {
                return;
            }
            copied += count;
            offset = (offset + count) % size;
        }
        em86xx_shared_write(task + 0x40, base + offset);
        qrd = (qrd + 24) % qsize;
        em86xx_shared_write(task + 0x50, qrd);
        /* CPUBlock::ProcessInterrupt bit 1 walks the host transfer list
         * and reclaims completed descriptors through SendBufferComplete.
         */
        em86xx_shared_write(0x617f8, em86xx_shared_read(0x617f8) | 2);
        em86xx_shared_write(0x6e010, 2);
    }
    if (qrd == qwr) {
        s->audio_blocked &= ~(1U << i);
    }
}

static void em86xx_audio_tick(void *opaque)
{
    EM86XXState *s = opaque;
    uint8_t *ram = memory_region_get_ram_ptr(&s->risc_ram);
    s->audio_blocked = 0;

    /* CPUBlock::FlushTransferTask waits for the auxiliary interrupt handler
     * to quiesce a registered host/MBUS transfer before erasing its FIFO.
     * Walk only installed tasks, with a bound for malformed guest lists.
     */
    for (unsigned i = 0; i < 2; i++) {
        hwaddr head = 0x10005244 + i * 12;
        hwaddr task = em86xx_shared_read(head);
        for (unsigned n = 0; n < 256 && task != head &&
             em86xx_task_address(task); n++) {
            if (em86xx_shared_read(task + 0x18) == 1) {
                em86xx_shared_write(task + 0x18, 2);
            } else if (!i && !em86xx_shared_read(task + 0x18)) {
                em86xx_audio_transfer(s, task);
            }
            task = em86xx_shared_read(task);
        }
    }

    for (unsigned i = 0; i < ARRAY_SIZE(s->audio_slots); i++) {
        em86xx_audio_decode(&s->audio_slots[i]);
        if (!(s->audio_pending & (1U << i))) {
            continue;
        }
        hwaddr head = 0x1000526c + (i / 2) * 8;
        hwaddr task = em86xx_shared_read(head);
        for (unsigned n = 0; n < 256 && task != head &&
             em86xx_task_address(task); n++) {
            unsigned offset = s->audio_slots[i].offset;
            if (em86xx_shared_read(task + 0x28) ==
                (0x1c + i * 0x100) &&
                !(em86xx_shared_read(task + 0x2c) & 2)) {
                stl_le_p(ram + offset + 4,
                         ldl_le_p(ram + offset + 4) | 1);
                em86xx_shared_write(task + 0x20,
                                    em86xx_shared_read(task + 0x20) | 1);
                em86xx_shared_write(task + 0x2c,
                                    em86xx_shared_read(task + 0x2c) | 0x22);
                em86xx_shared_write(0x617f8,
                                    em86xx_shared_read(0x617f8) | 0x10);
                em86xx_shared_write(0x6e010, 0x10);
                s->audio_pending &= ~(1U << i);
                break;
            }
            task = em86xx_shared_read(task);
        }
    }
    timer_mod(s->audio_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000);
}

static bool em86xx_display_address(hwaddr addr, uint64_t size)
{
    return (addr >= 0x10000000 && addr + size <= 0x18000000) ||
           (addr >= 0x20000000 && addr + size <= 0x28000000);
}

/* Local scanout of the surfaces attached by libdcc. The guest owns the
 * surface and picture descriptors; read them afresh so freeing or rebinding
 * a layer cannot leave host pointers into old guest allocations. */
static bool em86xx_display_update(void *opaque)
{
    EM86XXState *s = opaque;
    static const struct {
        hwaddr info;
        hwaddr control;
    } layers[] = {
        { 0x10002ce4, 0x10002d9c },
        { 0x10002bf4, 0x10002c9c },
        { 0x10002de4, 0x10002e2c },
        { 0x10002e74, 0x10002eb4 },
    };
    DisplaySurface *out = qemu_console_surface(s->console);
    uint32_t *dst = surface_data(out);
    int stride = surface_stride(out) / 4;
    const unsigned width = surface_width(out), height = surface_height(out);

    memset(dst, 0, surface_stride(out) * height);
    for (unsigned l = 0; l < ARRAY_SIZE(layers); l++) {
        hwaddr surf = em86xx_shared_read(layers[l].info);
        if (!em86xx_shared_read(layers[l].control + 8) ||
            !em86xx_display_address(surf, 0x58)) {
            continue;
        }
        hwaddr pic = em86xx_shared_read(surf + 12);
        unsigned mode = em86xx_shared_read(surf + 0x24);
        unsigned format = em86xx_shared_read(surf + 0x28);
        unsigned colorspace = em86xx_shared_read(surf + 0x20);
        if (!em86xx_display_address(pic, 0xf0)) {
            continue;
        }
        unsigned w = em86xx_shared_read(pic + 0x18);
        unsigned h = em86xx_shared_read(pic + 0x1c);
        hwaddr pixels = em86xx_shared_read(pic);
        unsigned bpp = mode == 4 ? 1 : 4;
        if (!w || !h || w > 1920 || h > 4320 ||
            (mode != 4 && !((mode == 5 || mode == 6) && format == 4)) ||
            !em86xx_display_address(pixels, (uint64_t)w * h * bpp)) {
            continue;
        }
        uint8_t *data = g_malloc((size_t)w * h * bpp);
        if (address_space_read(&address_space_memory, pixels,
                               MEMTXATTRS_UNSPECIFIED, data,
                               (size_t)w * h * bpp) != MEMTX_OK) {
            g_free(data);
            continue;
        }
        uint32_t palette[256] = { 0 };
        if (mode == 4) {
            hwaddr pal = em86xx_shared_read(pic + 0xe0);
            if (!em86xx_display_address(pal, sizeof(palette))) {
                g_free(data);
                continue;
            }
            for (unsigned i = 0; i < 256; i++) {
                palette[i] = em86xx_shared_read(pal + i * 4);
            }
        }
        for (unsigned y = 0; y < height; y++) {
            for (unsigned x = 0; x < width; x++) {
                size_t pos = ((uint64_t)y * h / height * w +
                              (uint64_t)x * w / width) * bpp;
                uint32_t p = mode == 4 ? palette[data[pos]] :
                                        ldl_le_p(data + pos);
                unsigned a = p >> 24;
                int r = (p >> 16) & 255, g = (p >> 8) & 255, b = p & 255;
                if (colorspace == 4 || colorspace == 5) {
                    int yy = r - 16, u = b - 128, v = g - 128;
                    r = (298 * yy + 409 * v + 128) >> 8;
                    g = (298 * yy - 100 * u - 208 * v + 128) >> 8;
                    b = (298 * yy + 516 * u + 128) >> 8;
                }
                uint32_t old = dst[y * stride + x];
                r = (CLAMP(r, 0, 255) * a +
                     ((old >> 16) & 255) * (255 - a)) / 255;
                g = (CLAMP(g, 0, 255) * a +
                     ((old >> 8) & 255) * (255 - a)) / 255;
                b = (CLAMP(b, 0, 255) * a +
                     (old & 255) * (255 - a)) / 255;
                dst[y * stride + x] = (r << 16) | (g << 8) | b;
            }
        }
        g_free(data);
    }
    qemu_console_update_full(s->console);
    return true;
}

static const GraphicHwOps em86xx_graphic_ops = {
    .gfx_update = em86xx_display_update,
};

static void em86xx_display_tick(void *opaque)
{
    EM86XXState *s = opaque;
    const hwaddr task = 0x10003140;
    static const hwaddr blocks[] = {
        0x1000193c, /* digital output */
        0x10001b0c, /* main analog / composite output */
        0x10001cd8, /* component output */
        0x10001e80, /* routing */
        0x10002050, /* main mixer */
        0x10002ab4, /* VCR multi-scaler */
        0x10002c9c, /* graphics multi-scaler */
        0x10002d9c, /* main video scaler */
        0x10002e2c, /* OSD scaler */
        0x10002eb4, /* subpicture scaler */
        0x10002f30, /* video plane */
        0x10002fb4, /* HD/SD converter */
    };

    /* Wait until the guest installs the display task. Preserve queued
     * notifications, and never touch the software interrupt mutex.
     */
    if (em86xx_shared_read(0x10005284) == task &&
        em86xx_shared_read(task + 0x28) == 3 &&
        !(em86xx_shared_read(task + 0x2c) & 2)) {
        for (unsigned i = 0; i < ARRAY_SIZE(blocks); i++) {
            /* Enable/disable writes do not necessarily set Validate's
             * pending mask.  Active state must still follow the request
             * at VSYNC, particularly before a surface can be freed. */
            em86xx_shared_write(blocks[i] + 8,
                                em86xx_shared_read(blocks[i] + 4));
            if (em86xx_shared_read(blocks[i])) {
                em86xx_shared_write(blocks[i], 0);
            }
        }
        /* SDK VSYNC bits for the outputs and blocks consumed above.
         * Do not invent NEW_PICTURE or in-band command events. */
        em86xx_shared_write(task + 0x20,
                            em86xx_shared_read(task + 0x20) | 0x7de7);
        em86xx_shared_write(task + 0x2c,
                            em86xx_shared_read(task + 0x2c) | 0x22);
        em86xx_shared_write(0x617f8, em86xx_shared_read(0x617f8) | 0x10);
        em86xx_shared_write(0x6e010, 0x10);
    }
    timer_mod(s->display_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 16666667);
}

/* Local firmware assist for audio_set_command/audio_get_command in em8xxx.ko.
 * Each engine has two command slots at PMEM + 0x10674/+0x1069c.
 * The driver writes the payload first and the command last; zero means
 * firmware has accepted the request. Keep payload/configuration in RAM.
 * Status words at command + 12 distinguish play (1), pause (2), stop (3)
 * and uninitialized (7). MPEG bitstreams use an optional host decoder.
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
    if (!addr && size == 4 &&
        (value == 1 || value == 2 || value == 3 || value == 6 || value == 7)) {
        /* Driver commands are 1..7. Firmware uploads remain normal RAM. */
        stl_le_p(ram + slot->offset, 0);
        if (value == 3 || value == 6 || value == 7) {
            em86xx_audio_discard(slot);
        }
        if (value == 1) {
            struct audsettings settings = {
                .freq = 44100, .nchannels = 2, .fmt = AUDIO_FORMAT_S16,
                .big_endian = HOST_BIG_ENDIAN,
            };
            slot->voice = audio_be_open_out(slot->owner->audio_be, slot->voice,
                "xg1000-mpeg", slot, em86xx_audio_out, &settings);
        }
        slot->playing = value == 1;
        if (slot->voice) {
            audio_be_set_active_out(slot->owner->audio_be, slot->voice,
                                     slot->playing);
            if (value == 7) {
                audio_be_close_out(slot->owner->audio_be, slot->voice);
                slot->voice = NULL;
            }
        }
        stl_le_p(ram + slot->offset + 12,
                  value == 1 ? 1 : value == 2 ? 2 : value == 7 ? 7 : 3);
        slot->owner->audio_pending |=
            1U << (slot - slot->owner->audio_slots);
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
    s->audio_pending = 0;
    s->audio_blocked = 0;
    for (unsigned i = 0; i < ARRAY_SIZE(s->audio_slots); i++) {
        em86xx_audio_discard(&s->audio_slots[i]);
        if (s->audio_slots[i].voice) {
            audio_be_close_out(s->audio_be, s->audio_slots[i].voice);
            s->audio_slots[i].voice = NULL;
        }
    }
    if (s->audio_timer) {
        timer_mod(s->audio_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000);
    }
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
        if (!audio_be_check(&s->audio_be, errp)) {
            return;
        }
        s->audio_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                      em86xx_audio_tick, s);
        static const unsigned slots[] = { 0x90674, 0x9069c,
                                          0xb0674, 0xb069c };
        for (unsigned i = 0; i < ARRAY_SIZE(slots); i++) {
            s->audio_slots[i].owner = s;
            s->audio_slots[i].offset = slots[i];
            memory_region_init_io(&s->audio_commands[i], OBJECT(s),
                                  &em86xx_audio_ops, &s->audio_slots[i],
                                  "em86xx-audio-command", 4);
            memory_region_add_subregion(&s->risc_ram, slots[i],
                                        &s->audio_commands[i]);
        }
    }
    if (s->display_assist) {
        s->console = qemu_graphic_console_create(dev, 0,
                                                &em86xx_graphic_ops, s);
        qemu_console_resize(s->console, 1280, 720);
        s->display_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                        em86xx_display_tick, s);
    }
    qemu_register_reset(em86xx_reset, s);
}

static const Property em86xx_properties[] = {
    DEFINE_AUDIO_PROPERTIES(EM86XXState, audio_be),
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
