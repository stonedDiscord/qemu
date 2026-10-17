/*
 * Host MIDI output and byte-stream framing.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#ifndef QEMU_CHARDEV_MIDI_H
#define QEMU_CHARDEV_MIDI_H

#include "qapi/error.h"

#define MIDI_SYSEX_MAX 65536

typedef void (*MidiSend)(void *opaque, const uint8_t *data, size_t len);

typedef struct MidiParser {
    uint8_t data[MIDI_SYSEX_MAX];
    size_t used;
    unsigned needed;
    uint8_t running;
    bool sysex;
    bool overflow;
} MidiParser;

void midi_parser_write(MidiParser *s, const uint8_t *data, size_t len,
                       MidiSend send, void *opaque);

typedef struct HostMidi HostMidi;
HostMidi *host_midi_open(const char *name, const char *path, Error **errp);
void host_midi_send(HostMidi *s, const uint8_t *data, size_t len);
void host_midi_close(HostMidi *s);

#endif
