/* MIDI 1.0 stream framing. SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "chardev/midi.h"

void midi_parser_write(MidiParser *s, const uint8_t *data, size_t len,
                       MidiSend send, void *opaque)
{
    for (size_t i = 0; i < len; i++) {
        uint8_t b = data[i];

        /* Real-time messages may interrupt any message, including SysEx. */
        if (b >= 0xf8) {
            if (b != 0xf9 && b != 0xfd) {
                send(opaque, &b, 1);
            }
            continue;
        }
        if (s->sysex) {
            if (b < 0x80 || b == 0xf7) {
                if (s->used < sizeof(s->data)) {
                    s->data[s->used++] = b;
                } else {
                    s->overflow = true;
                }
                if (b == 0xf7) {
                    if (!s->overflow) {
                        send(opaque, s->data, s->used);
                    }
                    s->sysex = false;
                    s->used = 0;
                }
                continue;
            }
            /* A new status aborts an unterminated SysEx. */
            s->sysex = false;
        }
        if (b & 0x80) {
            s->used = s->needed = 0;
            s->running = b < 0xf0 ? b : 0;
            if (b == 0xf0) {
                s->sysex = true;
                s->overflow = false;
            } else if (b < 0xf0) {
                s->needed = (b & 0xe0) == 0xc0 ? 2 : 3;
            } else if (b == 0xf1 || b == 0xf3) {
                s->needed = 2;
            } else if (b == 0xf2) {
                s->needed = 3;
            } else {
                if (b == 0xf6) {
                    send(opaque, &b, 1);
                }
                continue;
            }
            s->data[s->used++] = b;
        } else {
            if (!s->used) {
                if (!s->running) {
                    continue;
                }
                s->data[s->used++] = s->running;
            }
            s->data[s->used++] = b;
        }
        if (!s->sysex && s->used == s->needed) {
            send(opaque, s->data, s->used);
            s->used = 0;
        }
    }
}
