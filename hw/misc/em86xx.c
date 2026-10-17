/* Sigma Designs SMP863x media engines: initial DCC register model. */
#include "qemu/osdep.h"
#include "hw/core/sysbus.h"
#include "hw/misc/em86xx.h"
#include "hw/core/qdev-properties.h"
#include "system/address-spaces.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/log.h"
#include "qemu/bswap.h"
#include "qemu/units.h"
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
    uint32_t events;
    SWVoiceOut *voice;
#ifdef CONFIG_XG1000_AUDIO_FFMPEG
    AVCodecParserContext *parser;
    AVCodecContext *decoder;
    SwrContext *resampler;
    int sample_rate;
    enum AVSampleFormat sample_format;
    AVChannelLayout layout;
    uint8_t *pcm;
    size_t pcm_capacity;
    size_t pcm_pos, pcm_size;
    uint32_t bytes_read;
#endif
} AudioSlot;

#ifdef CONFIG_XG1000_AUDIO_FFMPEG
typedef struct VideoDecoder {
    AVCodecContext *codec;
    AVCodecParserContext *parser;
    AVPacket *packet;
    AVFrame *frame;
    enum AVCodecID codec_id;
    hwaddr context;
    hwaddr fifo;
    uint32_t frames;
    uint32_t sequence;
    uint32_t bytes_read;
    uint32_t events;
    bool draining;
    unsigned width, height;
    bool pending;
} VideoDecoder;
#endif

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
    QEMUTimer *video_timer;
    QemuConsole *console;
    hwaddr display_surface[4];
    hwaddr display_picture[4];
#ifdef CONFIG_XG1000_AUDIO_FFMPEG
    VideoDecoder video_decoder[2];
#endif
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
    slot->events = 0;
    if (slot->voice) {
        audio_be_set_active_out(slot->owner->audio_be, slot->voice, false);
    }
#ifdef CONFIG_XG1000_AUDIO_FFMPEG
    av_parser_close(slot->parser);
    slot->parser = NULL;
    avcodec_free_context(&slot->decoder);
    swr_free(&slot->resampler);
    g_clear_pointer(&slot->pcm, g_free);
    slot->pcm_capacity = 0;
    av_channel_layout_uninit(&slot->layout);
    slot->pcm_pos = slot->pcm_size = 0;
    slot->bytes_read = 0;
#endif
}

static void em86xx_audio_decode(AudioSlot *slot);

static void em86xx_audio_out(void *opaque, int available)
{
#ifdef CONFIG_XG1000_AUDIO_FFMPEG
    AudioSlot *slot = opaque;

    /* A backend request may span several decoded blocks. Refill immediately
     * instead of leaving the rest silent until the next firmware tick. Bound
     * decoding work when the input contains headers but produces no PCM.
     */
    for (unsigned n = 0; slot->playing && available > 0 && n < 32; n++) {
        if (slot->pcm_pos == slot->pcm_size) {
            em86xx_audio_decode(slot);
            if (slot->pcm_pos == slot->pcm_size) {
                break;
            }
        }
        size_t written = audio_be_write(slot->owner->audio_be, slot->voice,
            slot->pcm + slot->pcm_pos,
            MIN((size_t)available, slot->pcm_size - slot->pcm_pos));
        if (!written) {
            break;
        }
        slot->pcm_pos += written;
        available -= written;
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
    uint8_t input[65536 + AV_INPUT_BUFFER_PADDING_SIZE] = { 0 };
    AVPacket *packet;
    AVFrame *frame;
    int consumed;
    unsigned profile = ldl_le_p(ram + slot->offset + 16) & 0x7f;
    bool wma = profile == 7;

    if (!slot->playing || slot->pcm_pos != slot->pcm_size ||
        (s->audio_blocked & (1U << (slot - s->audio_slots))) ||
        (profile != 2 && !wma)) {
        return;
    }
    if (wma) {
        fifo = em86xx_shared_read(fifo + 0x24);
        if (fifo < 0x10000000 || fifo > 0x18000000 - 20) {
            return;
        }
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
    count = (wr + size - rd) % size;
    if (wma) {
        /* audio_set_wma_parameters packs rate/channels in word 0,
         * ASF block alignment in word 1, options in word 2 and bitrate
         * in word 4. audio_set_command copies these after the profile.
         */
        unsigned block_bits = ldl_le_p(ram + slot->offset + 20);
        unsigned block = block_bits / 8;
        if (block_bits & 7) {
            return;
        }
        if (!block || block > sizeof(input) - AV_INPUT_BUFFER_PADDING_SIZE ||
            count < block) {
            return;
        }
        count = block;
    } else {
        count = MIN(4096U, MIN(count, size - rd));
    }
    unsigned first = MIN(count, size - rd);
    if (address_space_read(&address_space_memory, base + rd,
                           MEMTXATTRS_UNSPECIFIED, input, first) != MEMTX_OK ||
        (first < count && address_space_read(&address_space_memory, base,
            MEMTXATTRS_UNSPECIFIED, input + first, count - first) != MEMTX_OK)) {
        return;
    }
    if (wma && !slot->decoder) {
        static const int rates[] = {
            8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100,
            48000, 64000, 88200, 96000, 128000, 176400, 192000
        };
        unsigned config = ldl_le_p(ram + slot->offset + 16) >> 8;
        unsigned rate = (config >> 4) & 0xf;
        unsigned channels = (config >> 12) & 0xf;
        const AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_WMAV2);
        if (!codec || rate >= ARRAY_SIZE(rates) || !channels || channels > 2 ||
            (config & 0xf)) {
            return;
        }
        slot->decoder = avcodec_alloc_context3(codec);
        if (!slot->decoder) {
            return;
        }
        slot->decoder->sample_rate = rates[rate];
        av_channel_layout_default(&slot->decoder->ch_layout, channels);
        slot->decoder->block_align = count;
        slot->decoder->bit_rate = ldl_le_p(ram + slot->offset + 32);
        slot->decoder->extradata = av_mallocz(6 + AV_INPUT_BUFFER_PADDING_SIZE);
        slot->decoder->extradata_size = 6;
        stw_le_p(slot->decoder->extradata + 4,
                 ldl_le_p(ram + slot->offset + 24) & 0xffff);
        if (avcodec_open2(slot->decoder, codec, NULL) < 0) {
            avcodec_free_context(&slot->decoder);
            return;
        }
    } else if (!wma && !slot->parser) {
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
    if (wma) {
        packet->data = input;
        packet->size = count;
        consumed = count;
    } else {
        consumed = av_parser_parse2(slot->parser, slot->decoder,
                                   &packet->data, &packet->size, input, count,
                                   AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
    }
    if (consumed > 0) {
        em86xx_shared_write(fifo + 8, (rd + consumed) % size);
        slot->bytes_read += consumed;
        unsigned index = slot - s->audio_slots;
        hwaddr pf = em86xx_shared_read(0x190850 + (index / 2) * 0x20000 +
                                       (index % 2) * 20);
        if (pf >= 0x10000000 && pf <= 0x18000000 - 16) {
            uint32_t pb = em86xx_shared_read(pf);
            uint32_t ps = em86xx_shared_read(pf + 4);
            uint32_t pr = em86xx_shared_read(pf + 8);
            uint32_t pw = em86xx_shared_read(pf + 12);
            if (ps && ps <= 65536 && pr < ps && pw < ps &&
                pb >= 0x10000000 && pb <= 0x18000000 - ps * 8) {
                for (unsigned n = 0; n < ps && pr != pw; n++) {
                    if ((int32_t)(em86xx_shared_read(pb + pr * 8) -
                                  slot->bytes_read) > 0) {
                        break;
                    }
                    pr = (pr + 1) % ps;
                }
                em86xx_shared_write(pf + 8, pr);
            }
        }
    }
    if (packet->size >= 4) {
        unsigned layer = (packet->data[1] >> 1) & 3;
        enum AVCodecID id = layer == 3 ? AV_CODEC_ID_MP1 :
                            layer == 2 ? AV_CODEC_ID_MP2 : AV_CODEC_ID_MP3;
        if (!wma && (!slot->decoder->codec || slot->decoder->codec_id != id)) {
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
        /* WMA superframes can contain more than 16384 stereo samples.
         * Keep the whole converted block: a fixed 64 KiB destination leaves
         * an increasing resampler delay which is lost when EOS closes it.
         */
        int capacity = swr_get_out_samples(slot->resampler, frame->nb_samples);
        if (capacity <= 0 || capacity > 1024 * 1024) {
            goto done;
        }
        if ((size_t)capacity * 4 > slot->pcm_capacity) {
            slot->pcm_capacity = (size_t)capacity * 4;
            slot->pcm = g_realloc(slot->pcm, slot->pcm_capacity);
        }
        uint8_t *out = slot->pcm;
        int samples = swr_convert(slot->resampler, &out, capacity,
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
        if (fifo == expected || fifo == em86xx_shared_read(expected + 0x24)) {
            break;
        }
    }
    if (i == ARRAY_SIZE(s->audio_slots) &&
        !em86xx_dram_range(fifo, 20)) {
        return;
    }
    if (i < ARRAY_SIZE(s->audio_slots) && qrd != qwr) {
        s->audio_blocked |= 1U << i;
    }
    uint32_t deferred = em86xx_shared_read(task + 0x1c);
    if ((mode > 1 && mode != 3) || deferred > 1 ||
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
        if (deferred && !cursor) {
            cursor = base + em86xx_shared_read(fifo + 12);
        }
        if (!em86xx_dram_range(source, bytes) ||
            !em86xx_dram_range(base, size) || !size || bytes >= size ||
            bytes > 1024 * 1024 ||
            cursor < base || cursor >= (hwaddr)base + size) {
            break;
        }
        offset = cursor - base;
        if (deferred) {
            uint32_t read = em86xx_shared_read(fifo + 8);
            if (read >= size || bytes > (read + size - offset - 1) % size) {
                break;
            }
            /* Before the first firmware transfer, SendBuffer queues its PTS
             * metadata without reserving bitstream space. Firmware performs
             * data_fifo_write_pts and publishes wr for these descriptors.
             */
            hwaddr pf = em86xx_shared_read(task + 0x38);
            uint32_t total = em86xx_shared_read(task + 0x24);
            if (em86xx_shared_read(queue + qrd + 16) & 1) {
                uint32_t pb = em86xx_shared_read(pf);
                uint32_t ps = em86xx_shared_read(pf + 4);
                uint32_t pr = em86xx_shared_read(pf + 8);
                uint32_t pw = em86xx_shared_read(pf + 12);
                if (!ps || ps > 65536 || pr >= ps || pw >= ps ||
                    !em86xx_dram_range(pb, ps * 8) ||
                    (pw + 1) % ps == pr) {
                    break;
                }
                em86xx_shared_write(pb + pw * 8,
                    total + em86xx_shared_read(queue + qrd + 12));
                em86xx_shared_write(pb + pw * 8 + 4,
                    em86xx_shared_read(queue + qrd + 8));
                em86xx_shared_write(pf + 12, (pw + 1) % ps);
            }
            em86xx_shared_write(task + 0x24, total + bytes);
        }
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
        if (deferred) {
            em86xx_shared_write(fifo + 12, offset);
        }
        qrd = (qrd + 24) % qsize;
        em86xx_shared_write(task + 0x50, qrd);
        /* CPUBlock::ProcessInterrupt bit 1 walks the host transfer list
         * and reclaims completed descriptors through SendBufferComplete.
         */
        em86xx_shared_write(0x617f8, em86xx_shared_read(0x617f8) | 2);
        em86xx_shared_write(0x6e010, 0x10);
    }
    if (deferred && qrd == qwr && em86xx_shared_read(task + 0x40)) {
        em86xx_shared_write(task + 0x1c, 0);
    }
    if (i < ARRAY_SIZE(s->audio_slots) && qrd == qwr) {
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
        AudioSlot *slot = &s->audio_slots[i];
        em86xx_audio_decode(slot);
#ifdef CONFIG_XG1000_AUDIO_FFMPEG
        /* audio_get_inband_fifo returns a DRAM entry eraser FIFO. Retire
         * EOS only after its byte coordinate and the queued PCM have drained;
         * the driver owns erase_ptr and exposes the consumed tag to RUA.
         */
        hwaddr ib = em86xx_shared_read(0x190828 + (i / 2) * 0x20000 +
                                      (i % 2) * 20);
        if (slot->playing && slot->pcm_pos == slot->pcm_size &&
            !(s->audio_blocked & (1U << i)) && em86xx_dram_range(ib, 20)) {
            uint32_t base = em86xx_shared_read(ib);
            uint32_t size = em86xx_shared_read(ib + 4);
            uint32_t rd = em86xx_shared_read(ib + 8);
            uint32_t wr = em86xx_shared_read(ib + 12);
            if (size && size <= 256 && rd < size && wr < size && rd != wr &&
                em86xx_dram_range(base, size * 64) &&
                (em86xx_shared_read(base + rd * 64) & 0xff) == 0x16 &&
                (int32_t)(em86xx_shared_read(base + rd * 64 + 4) -
                          slot->bytes_read) <= 0) {
                em86xx_shared_write(ib + 8, (rd + 1) % size);
                slot->events |= 8;
            }
        }
#endif
        uint32_t events = slot->events |
                          ((s->audio_pending & (1U << i)) ? 1 : 0);
        if (!events) {
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
                         ldl_le_p(ram + offset + 4) | events);
                em86xx_shared_write(task + 0x20,
                                    em86xx_shared_read(task + 0x20) | events);
                em86xx_shared_write(task + 0x1c,
                                    em86xx_shared_read(task + 0x1c) | events);
                em86xx_shared_write(task + 0x2c,
                                    em86xx_shared_read(task + 0x2c) | 0x23);
                em86xx_shared_write(0x617f8,
                                    em86xx_shared_read(0x617f8) | 0x10);
                em86xx_shared_write(0x6e010, 0x10);
                s->audio_pending &= ~(1U << i);
                slot->events = 0;
                break;
            }
            task = em86xx_shared_read(task);
        }
    }
    timer_mod(s->audio_timer,
              qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000);
}

#ifdef CONFIG_XG1000_AUDIO_FFMPEG
static void em86xx_video_discard(VideoDecoder *v)
{
    av_parser_close(v->parser);
    avcodec_free_context(&v->codec);
    av_packet_free(&v->packet);
    av_frame_free(&v->frame);
    memset(v, 0, sizeof(*v));
}

static bool em86xx_video_publish(VideoDecoder *v, hwaddr vti)
{
    hwaddr df = v->context + 0x80;
    uint32_t base = em86xx_shared_read(df);
    uint32_t size = em86xx_shared_read(df + 4);
    uint32_t rd = em86xx_shared_read(df + 8);
    uint32_t wr = em86xx_shared_read(df + 12);
    unsigned w = v->frame->width, h = v->frame->height;
    unsigned stride = (w + 31) & ~31U;
    uint64_t bytes = (uint64_t)stride * (h + (h + 1) / 2);
    hwaddr storage = em86xx_shared_read(vti + 0x2c);
    uint32_t storage_size = em86xx_shared_read(vti + 0x30);
    hwaddr picture;

    /* Display FIFOs count entries, whereas the bitstream FIFO counts bytes.
     * The guest reserves the data-context buffer for firmware pictures.
     */
    if (!size || size > 33 || rd >= size || wr >= size ||
        (wr + 1) % size == rd || !em86xx_dram_range(base, size * 4) ||
        !w || !h || w > 1920 || h > 1088 ||
        v->frame->format != AV_PIX_FMT_YUV420P ||
        bytes > storage_size || !em86xx_dram_range(storage, storage_size)) {
        return false;
    }
    /* Keep pictures distinct until the display releases their FIFO entries. */
    unsigned slots = MIN(32, storage_size / bytes);
    if (!slots || (wr + size - rd) % size >= slots) {
        return false;
    }
    picture = v->context + 0x298 + (v->frames % slots) * 0xf0;
    if (em86xx_shared_read(picture + 0xbc)) {
        return false;
    }
    storage += (v->frames % slots) * bytes;
    hwaddr chroma = storage + stride * h;
    for (unsigned y = 0; y < h; y++) {
        address_space_write(&address_space_memory, storage + y * stride,
            MEMTXATTRS_UNSPECIFIED,
            v->frame->data[0] + y * v->frame->linesize[0], w);
    }
    g_autofree uint8_t *uv = g_malloc((w + 1) & ~1U);
    for (unsigned y = 0; y < (h + 1) / 2; y++) {
        for (unsigned x = 0; x < (w + 1) / 2; x++) {
            uv[2 * x] = v->frame->data[1][y * v->frame->linesize[1] + x];
            uv[2 * x + 1] = v->frame->data[2][y * v->frame->linesize[2] + x];
        }
        address_space_write(&address_space_memory, chroma + y * stride,
                            MEMTXATTRS_UNSPECIFIED, uv, (w + 1) & ~1U);
    }
    em86xx_shared_write(picture, storage);
    em86xx_shared_write(picture + 4, stride);
    em86xx_shared_write(picture + 8, chroma);
    em86xx_shared_write(picture + 12, stride);
    em86xx_shared_write(picture + 0x18, w);
    em86xx_shared_write(picture + 0x1c, h);
    em86xx_shared_write(picture + 0x28, (w + 1) / 2);
    em86xx_shared_write(picture + 0x2c, (h + 1) / 2);
    em86xx_shared_write(picture + 0x30, w);
    em86xx_shared_write(picture + 0x34, h);
    em86xx_shared_write(picture + 0x78, w);
    em86xx_shared_write(picture + 0x7c, h);
    em86xx_shared_write(picture + 0xb8, 0);
    em86xx_shared_write(picture + 0xbc, 1);
    em86xx_shared_write(picture + 0xec, ++v->frames);
    em86xx_shared_write(vti + 0x48, v->frames);
    if (v->frame->pts != AV_NOPTS_VALUE) {
        em86xx_shared_write(vti + 0x40, v->frame->pts);
    }
    /* Entries are descriptor addresses (gbus_entry_fifo, entry size 4). */
    em86xx_shared_write(base + wr * 4, picture);
    em86xx_shared_write(df + 12, (wr + 1) % size);
    av_frame_unref(v->frame);
    v->pending = false;
    return true;
}

static void em86xx_video_decode(EM86XXState *s, hwaddr vti, unsigned engine)
{
    VideoDecoder *v = &s->video_decoder[engine];
    hwaddr context = vti - 0x2098;
    if (!em86xx_dram_range(context, 0x2118) ||
        em86xx_shared_read(context + 0x48) != vti) {
        return;
    }
    hwaddr fifo = em86xx_shared_read(context + 0x4c);
    if (!em86xx_dram_range(fifo, 20)) {
        return;
    }
    uint32_t base = em86xx_shared_read(fifo);
    uint32_t size = em86xx_shared_read(fifo + 4);
    uint32_t rd = em86xx_shared_read(fifo + 8);
    uint32_t wr = em86xx_shared_read(fifo + 12);
    hwaddr head = 0x10005244;
    hwaddr transfer = em86xx_shared_read(head);
    for (unsigned n = 0; n < 256 && transfer != head &&
         em86xx_task_address(transfer); n++) {
        if (em86xx_shared_read(transfer + 0x30) == fifo &&
            em86xx_shared_read(transfer + 0x50) !=
            em86xx_shared_read(transfer + 0x54)) {
            uint32_t cursor = em86xx_shared_read(transfer + 0x40);
            wr = cursor >= base && cursor < (hwaddr)base + size ?
                 cursor - base : rd;
            break;
        }
        transfer = em86xx_shared_read(transfer);
    }
    uint32_t profile = em86xx_shared_read(vti + 0x1c) & 0xff;
    uint32_t sequence = em86xx_shared_read(vti + 0x6c);
    unsigned width = em86xx_shared_read(vti + 0x64);
    unsigned height = em86xx_shared_read(vti + 0x68);
    enum AVCodecID codec_id;

    /* set_CodecX maps public MPEG2/WMV/VC1 codec IDs to 0/5/6. */
    switch (profile) {
    case 0: codec_id = AV_CODEC_ID_MPEG2VIDEO; break;
    case 5: codec_id = AV_CODEC_ID_WMV3; break;
    case 6: codec_id = AV_CODEC_ID_VC1; break;
    default: return;
    }
    if (!em86xx_dram_range(base, size) || !size ||
        size > 8 * 1024 * 1024 || rd >= size || wr >= size) {
        return;
    }
    if (v->codec && (v->context != context || v->fifo != fifo ||
        v->codec_id != codec_id || (codec_id == AV_CODEC_ID_WMV3 &&
        (v->sequence != sequence || v->width != width || v->height != height)))) {
        em86xx_video_discard(v);
    }
    if (!v->codec) {
        const AVCodec *codec = avcodec_find_decoder(codec_id);
        if (!codec || (codec_id == AV_CODEC_ID_WMV3 &&
            (!width || !height || width > 1920 || height > 1088))) {
            return;
        }
        v->codec = avcodec_alloc_context3(codec);
        v->packet = av_packet_alloc();
        v->frame = av_frame_alloc();
        v->codec_id = codec_id;
        v->context = context;
        v->fifo = fifo;
        v->width = width;
        v->height = height;
        v->sequence = sequence;
        v->bytes_read = rd;
        if (codec_id == AV_CODEC_ID_WMV3) {
            v->codec->width = width;
            v->codec->height = height;
            v->codec->extradata = av_mallocz(4 + AV_INPUT_BUFFER_PADDING_SIZE);
            v->codec->extradata_size = 4;
            stl_be_p(v->codec->extradata, sequence);
        } else {
            v->parser = av_parser_init(codec_id);
        }
        v->codec->flags |= AV_CODEC_FLAG_LOW_DELAY;
        if (!v->packet || !v->frame ||
            avcodec_open2(v->codec, codec, NULL) < 0) {
            em86xx_video_discard(v);
            return;
        }
    }
    if (v->pending && !em86xx_video_publish(v, vti)) {
        return;
    }
    if (avcodec_receive_frame(v->codec, v->frame) == 0) {
        v->pending = true;
        em86xx_video_publish(v, vti);
        return;
    }
    uint32_t count = (wr + size - rd) % size;
    /* In-band commands carry cumulative byte offsets, independent of FIFO
     * wrap. Retire commands through the driver's eraser/event path, and
     * drain a terminal picture only at an explicit sequence boundary.
     */
    hwaddr ib = context + 0x6c;
    uint32_t ibase = em86xx_shared_read(ib);
    uint32_t isize = em86xx_shared_read(ib + 4);
    uint32_t ir = em86xx_shared_read(ib + 8);
    uint32_t iw = em86xx_shared_read(ib + 12);
    bool sequence_end = false;
    bool terminal_packet = false;
    if (isize && isize <= 256 && ir < isize && iw < isize &&
        em86xx_dram_range(ibase, isize * 64)) {
        for (unsigned n = 0; n < isize && ir != iw; n++) {
            hwaddr entry = ibase + ir * 64;
            uint32_t offset = em86xx_shared_read(entry + 4);
            uint32_t tag = em86xx_shared_read(entry) & 0xff;
            int32_t distance = offset - v->bytes_read;
            if (distance > 0) {
                terminal_packet = (tag == 0x10 || tag == 0x16) &&
                                  distance <= count;
                count = MIN(count, distance);
                break;
            }
            if (tag == 0x10 || tag == 0x16) {
                sequence_end = true;
                break;
            }
            ir = (ir + 1) % isize;
            em86xx_shared_write(ib + 8, ir);
            v->events |= 8;
        }
    }
    if (sequence_end) {
        uint8_t *packet = NULL;
        int length = 0;
        if (v->parser && !v->draining) {
            av_parser_parse2(v->parser, v->codec, &packet, &length,
                             NULL, 0, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
        }
        if (length) {
            av_packet_unref(v->packet);
            if (av_new_packet(v->packet, length) == 0) {
                memcpy(v->packet->data, packet, length);
                avcodec_send_packet(v->codec, v->packet);
            }
            return;
        }
        if (!v->draining) {
            avcodec_send_packet(v->codec, NULL);
            v->draining = true;
            return;
        }
        em86xx_shared_write(ib + 8, (ir + 1) % isize);
        v->events |= 12;
        avcodec_flush_buffers(v->codec);
        if (v->parser) {
            av_parser_close(v->parser);
            v->parser = av_parser_init(codec_id);
        }
        v->draining = false;
        return;
    }
    if (!count) {
        return;
    }
    /* WMV has no elementary-stream start codes. Consecutive PTS entries
     * delimit ASF media objects, including objects split over transfers.
     */
    hwaddr pf = context + 0x5c;
    uint32_t pb = em86xx_shared_read(pf);
    uint32_t ps = em86xx_shared_read(pf + 4);
    uint32_t pr = em86xx_shared_read(pf + 8);
    uint32_t pw = em86xx_shared_read(pf + 12);
    if (codec_id == AV_CODEC_ID_WMV3) {
        if (ps < 2 || ps > 65536 || pr >= ps || pw >= ps ||
            !em86xx_dram_range(pb, (uint64_t)ps * 8) ||
            ((pw + ps - pr) % ps < 2 &&
             !(terminal_packet && pr != pw))) {
            return;
        }
        uint32_t start = em86xx_shared_read(pb + pr * 8);
        uint32_t end = (pw + ps - pr) % ps >= 2 ?
            em86xx_shared_read(pb + ((pr + 1) % ps) * 8) : start + count;
        uint32_t packet_size = end - start;
        if (!packet_size || packet_size > count || start % size != rd) {
            return;
        }
        count = packet_size;
    }
    g_autofree uint8_t *input = g_malloc0(count + AV_INPUT_BUFFER_PADDING_SIZE);
    unsigned first = MIN(count, size - rd);
    address_space_read(&address_space_memory, base + rd,
                       MEMTXATTRS_UNSPECIFIED, input, first);
    if (first < count) {
        address_space_read(&address_space_memory, base,
                           MEMTXATTRS_UNSPECIFIED, input + first, count - first);
    }
    uint8_t *packet = input;
    int packet_size = count;
    int consumed = count;
    if (v->parser) {
        consumed = av_parser_parse2(v->parser, v->codec, &packet, &packet_size,
                                    input, count, AV_NOPTS_VALUE,
                                    AV_NOPTS_VALUE, 0);
        if (consumed < 0) {
            return;
        }
    }
    em86xx_shared_write(fifo + 8, (rd + consumed) % size);
    v->bytes_read += consumed;
    /* This is an eraser FIFO: the driver's DMA completion owns erase_ptr. */
    if (codec_id == AV_CODEC_ID_WMV3) {
        em86xx_shared_write(pf + 8, (pr + 1) % ps);
    } else if (ps && ps <= 65536 && pr < ps && pw < ps &&
               em86xx_dram_range(pb, ps * 8)) {
        /* MPEG/VC-1 use the parser for packet boundaries, but their PTS
         * queue still needs a firmware consumer.  Leaving these entries
         * behind eventually makes data_fifo_pts_writable reject SendData
         * even while the bitstream FIFO has plenty of free space.
         * PTS offsets, like in-band offsets, are cumulative byte counts.
         */
        for (unsigned n = 0; n < ps && pr != pw; n++) {
            if ((int32_t)(em86xx_shared_read(pb + pr * 8) -
                          v->bytes_read) > 0) {
                break;
            }
            pr = (pr + 1) % ps;
        }
        em86xx_shared_write(pf + 8, pr);
    }
    if (packet_size) {
        av_packet_unref(v->packet);
        if (av_new_packet(v->packet, packet_size) < 0) {
            return;
        }
        memcpy(v->packet->data, packet, packet_size);
        v->packet->pts = codec_id == AV_CODEC_ID_WMV3 ?
            em86xx_shared_read(pb + pr * 8 + 4) : AV_NOPTS_VALUE;
        if (avcodec_send_packet(v->codec, v->packet) < 0) {
            return;
        }
        if (avcodec_receive_frame(v->codec, v->frame) == 0) {
            v->pending = true;
            em86xx_video_publish(v, vti);
        }
    }
}
#else
static void em86xx_video_decode(EM86XXState *s, hwaddr vti, unsigned engine)
{
}
#endif

/* Local video firmware stub for the supplied em8xxx ABI. The driver installs
 * an IRQ task with pointers to the VTI mask/status words. The VTI command is
 * two words before that mask. Control commands and the opt-in MPEG-2/WMV
 * picture paths are handled here; newer codecs remain unsupported.
 */
static void em86xx_video_tick(void *opaque)
{
    EM86XXState *s = opaque;

    for (unsigned engine = 0; engine < 2; engine++) {
        hwaddr head = 0x1000525c + engine * 8;
        hwaddr task = em86xx_shared_read(head);
        for (unsigned n = 0; n < 256 && task != head &&
             em86xx_task_address(task); n++) {
            uint32_t module = em86xx_shared_read(task + 0x28);
            uint32_t flags = em86xx_shared_read(task + 0x2c);
            /* +8 is the GBUS mutex, followed by mask/status pointers. */
            hwaddr mask = em86xx_shared_read(task + 12);
            hwaddr irq = em86xx_shared_read(task + 16);
            if ((module & 0xff) == 0x1a &&
                mask >= 8 && em86xx_dram_range(mask - 8, 16) &&
                irq == mask + 4 && em86xx_dram_range(irq, 4)) {
                hwaddr command = mask - 8;
                /* video_get_irq_info returns VTI+8/+12, so the command
                 * is mask-8 and decoder state is mask-4. FF is idle. */
                uint32_t cmd = em86xx_shared_read(command);
                uint32_t events = 0;
                em86xx_video_decode(s, command, engine);
                /* A pending guest callback must not stop the firmware's
                 * bitstream consumer.  Retain new events until the guest
                 * releases the task, then deliver them normally.
                 */
                if (flags & 3) {
                    task = em86xx_shared_read(task);
                    continue;
                }
#ifdef CONFIG_XG1000_AUDIO_FFMPEG
                events = s->video_decoder[engine].events &
                         em86xx_shared_read(mask);
                s->video_decoder[engine].events = 0;
#endif
                if (cmd <= 7 && cmd != 5 &&
                    (em86xx_shared_read(mask) & 1)) {
                    em86xx_shared_write(command + 4, cmd == 1 ? 2 : cmd);
                    em86xx_shared_write(command, 0xff);
                    events |= 1;
                }
                if (events) {
                    em86xx_shared_write(irq, em86xx_shared_read(irq) | events);
                    /* Deliver the auxiliary engine's emhwlib callback first,
                     * then its application event, through the existing IRQ. */
                    em86xx_shared_write(task + 0x1c,
                        em86xx_shared_read(task + 0x1c) | events);
                    em86xx_shared_write(task + 0x20,
                        em86xx_shared_read(task + 0x20) | events);
                    em86xx_shared_write(task + 0x2c, flags | 0x23);
                    em86xx_shared_write(0x617f8,
                        em86xx_shared_read(0x617f8) | 0x10);
                    em86xx_shared_write(0x6e010, 0x10);
                }
            }
            task = em86xx_shared_read(task);
        }
    }
    timer_mod(s->video_timer,
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
static const struct {
        hwaddr info;
        hwaddr control;
        hwaddr window;
} em86xx_layers[] = {
        { 0x10002ce4, 0x10002d9c, 0x10002d30 },
        { 0x10002bf4, 0x10002c9c, 0x10002c3c },
        { 0x10002de4, 0x10002e2c, 0x10002df0 },
        { 0x10002e74, 0x10002eb4, 0x10002e80 },
    };

/* EMhwlibDisplayWindow uses 4096 for the complete source extent. CScreen
 * allocates several vertically stacked pages and selects one here; the
 * picture dimensions describe storage, not the visible viewport.
 */
static void em86xx_window_axis(hwaddr window, unsigned axis, unsigned extent,
                               int64_t *start, unsigned *size)
{
    int32_t pos = em86xx_shared_read(window + axis * 4);
    uint32_t length = em86xx_shared_read(window + 8 + axis * 4);
    unsigned anchor = em86xx_shared_read(window + 16 + axis * 4);
    unsigned pos_mode = em86xx_shared_read(window + 24 + axis * 4);
    unsigned size_mode = em86xx_shared_read(window + 32 + axis * 4);

    *start = 0;
    *size = extent;
    if (anchor < 1 || anchor > 4 || pos_mode < 1 || pos_mode > 2 ||
        size_mode < 1 || size_mode > 2) {
        return;
    }
    int64_t offset = pos_mode == 2 ? (int64_t)pos * extent / 4096 : pos;
    uint64_t span = size_mode == 2 ? (uint64_t)length * extent / 4096 : length;
    *size = MIN(span, UINT32_MAX);
    switch (anchor) {
    case 1: *start = offset; break;
    case 2: *start = (int64_t)extent - offset - *size; break;
    case 3: *start = offset - *size / 2; break;
    case 4: *start = (int64_t)extent - offset - *size / 2; break;
    }
}

static void em86xx_display_render(EM86XXState *s, uint32_t *dst, int stride,
                                  unsigned width, unsigned height,
                                  bool graphics_only)
{
    g_autofree unsigned *xmap = g_new(unsigned, width);

    memset(dst, 0, stride * 4 * height);
    for (unsigned l = 0; l < ARRAY_SIZE(em86xx_layers); l++) {
        if (graphics_only && l != 1) {
            continue;
        }
        hwaddr surf = em86xx_shared_read(em86xx_layers[l].info);
        if (!em86xx_shared_read(em86xx_layers[l].control + 8) ||
            !em86xx_display_address(surf, 0x58)) {
            continue;
        }
        hwaddr pic = em86xx_shared_read(surf + 8) ?
                     (s->display_surface[l] == surf ? s->display_picture[l] : 0) :
                     em86xx_shared_read(surf + 12);
        unsigned mode = em86xx_shared_read(surf + 0x24);
        unsigned format = em86xx_shared_read(surf + 0x28);
        unsigned colorspace = em86xx_shared_read(surf + 0x20);
        if (!em86xx_display_address(pic, 0xf0)) {
            continue;
        }
        unsigned w = em86xx_shared_read(pic + 0x18);
        unsigned h = em86xx_shared_read(pic + 0x1c);
        hwaddr pixels = em86xx_shared_read(pic);
        if ((mode == 7 || mode == 8) &&
            (!w || !h || w > 4096 || h > 4096)) {
            w = em86xx_shared_read(pic + 0x30);
            h = em86xx_shared_read(pic + 0x34);
        }
        if ((mode == 7 || mode == 8) && w && h) {
            hwaddr yaddr = em86xx_shared_read(pic);
            hwaddr ystride = em86xx_shared_read(pic + 4);
            hwaddr uvaddr = em86xx_shared_read(pic + 8);
            hwaddr uvstride = em86xx_shared_read(pic + 12);
            if (w <= 1920 && h <= 1088 && ystride >= w &&
                uvstride >= ((w + 1) & ~1U) &&
                ystride <= 8192 && uvstride <= 8192 &&
                em86xx_display_address(yaddr, ystride * h) &&
                em86xx_display_address(uvaddr, uvstride * ((h + 1) / 2))) {
                g_autofree uint8_t *yp = g_malloc(ystride * h);
                g_autofree uint8_t *uvp = g_malloc(uvstride * ((h + 1) / 2));
                address_space_read(&address_space_memory, yaddr,
                                   MEMTXATTRS_UNSPECIFIED, yp, ystride * h);
                address_space_read(&address_space_memory, uvaddr,
                                   MEMTXATTRS_UNSPECIFIED, uvp,
                                   uvstride * ((h + 1) / 2));
                for (unsigned x = 0; x < width; x++) {
                    xmap[x] = (uint64_t)x * w / width;
                }
                for (unsigned y = 0; y < height; y++) {
                    unsigned sy = (uint64_t)y * h / height;
                    for (unsigned x = 0; x < width; x++) {
                        unsigned sx = xmap[x];
                        int yy = yp[sy * ystride + sx] - 16;
                        unsigned uv = (sy / 2) * uvstride + (sx & ~1U);
                        int uu = uvp[uv] - 128, vv = uvp[uv + 1] - 128;
                        int r = (298 * yy + 409 * vv + 128) >> 8;
                        int g = (298 * yy - 100 * uu - 208 * vv + 128) >> 8;
                        int b = (298 * yy + 516 * uu + 128) >> 8;
                        dst[y * stride + x] = (CLAMP(r, 0, 255) << 16) |
                            (CLAMP(g, 0, 255) << 8) | CLAMP(b, 0, 255);
                    }
                }
            }
            continue;
        }
        unsigned bpp = mode == 4 ? 1 : 4;
        unsigned pitch = em86xx_shared_read(pic + 4);
        unsigned bx = em86xx_shared_read(pic + 0x10);
        unsigned by = em86xx_shared_read(pic + 0x14);
        if (!pitch) {
            pitch = w;
        }
        uint64_t bytes = (uint64_t)pitch * ((uint64_t)by + h) * bpp;
        if (!w || !h || w > 1920 || h > 4320 ||
            pitch > 8192 || (uint64_t)bx + w > pitch || bytes > 128 * MiB ||
            (mode != 4 && !((mode == 5 || mode == 6) && format == 4)) ||
            !em86xx_display_address(pixels, bytes)) {
            continue;
        }
        int64_t left, top;
        unsigned view_w, view_h;
        em86xx_window_axis(em86xx_layers[l].window, 0, w, &left, &view_w);
        em86xx_window_axis(em86xx_layers[l].window, 1, h, &top, &view_h);
        if (!view_w || !view_h) {
            continue;
        }
        /* Copy only the sampled rows, not all of CScreen's drawing pages. */
        int64_t first = MAX(top, 0);
        int64_t last = MIN(top + (int64_t)(height - 1) * view_h / height,
                           (int64_t)h - 1);
        if (first > last) {
            continue;
        }
        pixels += ((uint64_t)by + first) * pitch * bpp;
        bytes = (last - first + 1) * pitch * bpp;
        uint8_t *data = g_malloc(bytes);
        if (address_space_read(&address_space_memory, pixels,
                               MEMTXATTRS_UNSPECIFIED, data, bytes) != MEMTX_OK) {
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
        for (unsigned x = 0; x < width; x++) {
            int64_t sx = left + (int64_t)x * view_w / width;
            xmap[x] = sx < 0 || sx >= w ? UINT_MAX : (bx + sx) * bpp;
        }
        for (unsigned y = 0; y < height; y++) {
            int64_t sy = top + (int64_t)y * view_h / height;
            if (sy < 0 || sy >= h) {
                continue;
            }
            const uint8_t *row = data + (sy - first) * pitch * bpp;
            for (unsigned x = 0; x < width; x++) {
                unsigned offset = xmap[x];
                if (offset == UINT_MAX) {
                    continue;
                }
                uint32_t p = mode == 4 ? palette[row[offset]] :
                                        ldl_le_p(row + offset);
                unsigned a = p >> 24;
                if (!a) {
                    continue;
                }
                int r = (p >> 16) & 255, g = (p >> 8) & 255, b = p & 255;
                if (colorspace == 4 || colorspace == 5) {
                    int yy = r - 16, u = b - 128, v = g - 128;
                    r = (298 * yy + 409 * v + 128) >> 8;
                    g = (298 * yy - 100 * u - 208 * v + 128) >> 8;
                    b = (298 * yy + 516 * u + 128) >> 8;
                }
                if (a == 255) {
                    dst[y * stride + x] = (CLAMP(r, 0, 255) << 16) |
                        (CLAMP(g, 0, 255) << 8) | CLAMP(b, 0, 255);
                    continue;
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
}

static bool em86xx_display_update(void *opaque)
{
    EM86XXState *s = opaque;
    DisplaySurface *out = qemu_console_surface(s->console);
    em86xx_display_render(s, surface_data(out), surface_stride(out) / 4,
                          surface_width(out), surface_height(out), false);
    qemu_console_update_full(s->console);
    return true;
}

bool em86xx_tft_scanout(DeviceState *dev, uint32_t *dst, unsigned width,
                        unsigned height, unsigned stride)
{
    EM86XXState *s = EM86XX(dev);
    /* SelectTFTVideo routes graphics (source 0) or the main mixer (source 2)
     * to DispDigitalOut. Registers are the driver's routing shadow block.
     */
    uint32_t route = em86xx_shared_read(0x10001e60);
    uint32_t source = em86xx_shared_read(0x10001e64);
    if (!s->display_assist || !(route & 4) || (source != 1 && source != 3)) {
        return false;
    }
    /* The local display assist presents the main mixer beneath the TFT UI
     * even when the application selects graphics-only digital output.
     * This approximates the external video path, not the digital mux itself.
     */
    em86xx_display_render(s, dst, stride, width, height, false);
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

    /* Advance decoder-backed surfaces at VSYNC, retaining the current
     * picture when no replacement has arrived. Bitmap surfaces have no FIFO.
     */
    for (unsigned l = 0; l < ARRAY_SIZE(em86xx_layers); l++) {
        hwaddr surf = em86xx_shared_read(em86xx_layers[l].info);
        if (surf != s->display_surface[l]) {
            s->display_surface[l] = surf;
            s->display_picture[l] = 0;
        }
        if (!em86xx_dram_range(surf, 16) ||
            !em86xx_shared_read(em86xx_layers[l].control + 4)) {
            continue;
        }
        hwaddr fifo = em86xx_shared_read(surf + 8);
        if (!em86xx_dram_range(fifo, 16)) {
            continue;
        }
        uint32_t base = em86xx_shared_read(fifo);
        uint32_t size = em86xx_shared_read(fifo + 4);
        uint32_t rd = em86xx_shared_read(fifo + 8);
        uint32_t wr = em86xx_shared_read(fifo + 12);
        if (!size || size > 33 || rd >= size || wr >= size || rd == wr ||
            !em86xx_dram_range(base, size * 4)) {
            continue;
        }
        hwaddr picture = em86xx_shared_read(base + rd * 4);
        if (!em86xx_dram_range(picture, 0xf0)) {
            continue;
        }
        if (s->display_picture[l]) {
            em86xx_shared_write(s->display_picture[l] + 0xbc, 0);
        }
        s->display_picture[l] = picture;
        em86xx_shared_write(fifo + 8, (rd + 1) % size);
    }

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
#ifdef CONFIG_XG1000_AUDIO_FFMPEG
    for (unsigned i = 0; i < ARRAY_SIZE(s->video_decoder); i++) {
        em86xx_video_discard(&s->video_decoder[i]);
    }
#endif
    memset(s->display_surface, 0, sizeof(s->display_surface));
    memset(s->display_picture, 0, sizeof(s->display_picture));
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
    if (s->video_timer) {
        timer_mod(s->video_timer,
                  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 1000000);
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
        s->video_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                      em86xx_video_tick, s);
    }
    qemu_register_reset(em86xx_reset, s);
}

static const Property em86xx_properties[] = {
    DEFINE_AUDIO_PROPERTIES(EM86XXState, audio_be),
    DEFINE_PROP_BOOL("xg1000-audio-stub", EM86XXState, audio_stub, true),
    DEFINE_PROP_BOOL("xg1000-display-assist", EM86XXState, display_assist, true),
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
