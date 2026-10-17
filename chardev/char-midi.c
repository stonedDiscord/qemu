/* Host MIDI output chardev. SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "chardev/char.h"
#include "chardev/midi.h"
#include "qemu/module.h"
#include "qemu/option.h"

#define TYPE_CHARDEV_MIDI "chardev-midi"
typedef struct MidiChardev {
    Chardev parent;
    HostMidi *host;
    MidiParser parser;
} MidiChardev;

DECLARE_INSTANCE_CHECKER(MidiChardev, MIDI_CHARDEV, TYPE_CHARDEV_MIDI)

static void midi_send(void *opaque, const uint8_t *data, size_t len)
{
    host_midi_send(opaque, data, len);
}

static int midi_chr_write(Chardev *chr, const uint8_t *data, int len)
{
    MidiChardev *s = MIDI_CHARDEV(chr);

    midi_parser_write(&s->parser, data, len, midi_send, s->host);
    return len;
}

static bool midi_chr_open(Chardev *chr, ChardevBackend *backend, Error **errp)
{
    MidiChardev *s = MIDI_CHARDEV(chr);

    s->host = host_midi_open(chr->label, backend->u.midi.data->path, errp);
    if (!s->host) {
        return false;
    }
    qemu_chr_be_event(chr, CHR_EVENT_OPENED);
    return true;
}

static void midi_chr_parse(QemuOpts *opts, ChardevBackend *backend,
                           Error **errp)
{
    ChardevMidi *midi = g_new0(ChardevMidi, 1);

    backend->type = CHARDEV_BACKEND_KIND_MIDI;
    backend->u.midi.data = midi;
    qemu_chr_parse_common(opts, qapi_ChardevMidi_base(midi));
    midi->path = g_strdup(qemu_opt_get(opts, "path"));
}

static void midi_chr_finalize(Object *obj)
{
    host_midi_close(MIDI_CHARDEV(obj)->host);
}

static void midi_chr_class_init(ObjectClass *oc, const void *data)
{
    ChardevClass *cc = CHARDEV_CLASS(oc);

    cc->chr_parse = midi_chr_parse;
    cc->chr_open = midi_chr_open;
    cc->chr_write = midi_chr_write;
}

static const TypeInfo midi_chr_type = {
    .name = TYPE_CHARDEV_MIDI,
    .parent = TYPE_CHARDEV,
    .instance_size = sizeof(MidiChardev),
    .instance_finalize = midi_chr_finalize,
    .class_init = midi_chr_class_init,
};

static void midi_chr_register(void)
{
    type_register_static(&midi_chr_type);
}
type_init(midi_chr_register);
