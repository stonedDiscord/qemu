/* ALSA sequencer MIDI output. SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include <alsa/asoundlib.h>
#include "chardev/midi.h"
#include "qemu/error-report.h"

struct HostMidi {
    snd_seq_t *seq;
    snd_midi_event_t *encoder;
    int port;
    bool warned;
};

HostMidi *host_midi_open(const char *name, const char *path, Error **errp)
{
    HostMidi *s = g_new0(HostMidi, 1);
    snd_seq_addr_t dest;
    int ret;

    ret = snd_seq_open(&s->seq, "default", SND_SEQ_OPEN_OUTPUT,
                       SND_SEQ_NONBLOCK);
    if (ret < 0) {
        goto fail;
    }
    snd_seq_set_client_name(s->seq, "QEMU MIDI");
    s->port = snd_seq_create_simple_port(s->seq, name,
                  SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ,
                  SND_SEQ_PORT_TYPE_MIDI_GENERIC |
                  SND_SEQ_PORT_TYPE_APPLICATION);
    if (s->port < 0) {
        ret = s->port;
        goto fail;
    }
    if (path) {
        ret = snd_seq_parse_address(s->seq, &dest, path);
        if (ret < 0) {
            goto fail;
        }
        ret = snd_seq_connect_to(s->seq, s->port, dest.client, dest.port);
        if (ret < 0) {
            goto fail;
        }
    }
    ret = snd_midi_event_new(MIDI_SYSEX_MAX, &s->encoder);
    if (ret < 0) {
        goto fail;
    }
    return s;

fail:
    error_setg(errp, "Cannot open MIDI output%s%s: %s",
               path ? " " : "", path ?: "", snd_strerror(ret));
    host_midi_close(s);
    return NULL;
}

void host_midi_send(HostMidi *s, const uint8_t *data, size_t len)
{
    snd_seq_event_t ev;
    long ret;

    snd_seq_ev_clear(&ev);
    snd_midi_event_reset_encode(s->encoder);
    ret = snd_midi_event_encode(s->encoder, data, len, &ev);
    if (ret == len && ev.type != SND_SEQ_EVENT_NONE) {
        snd_seq_ev_set_source(&ev, s->port);
        snd_seq_ev_set_subs(&ev);
        snd_seq_ev_set_direct(&ev);
        ret = snd_seq_event_output_direct(s->seq, &ev);
    }
    if (ret < 0 && !s->warned) {
        warn_report("MIDI output dropped a message: %s", snd_strerror(ret));
        s->warned = true;
    }
}

void host_midi_close(HostMidi *s)
{
    if (!s) {
        return;
    }
    if (s->encoder) {
        snd_midi_event_free(s->encoder);
    }
    if (s->seq) {
        snd_seq_close(s->seq);
    }
    g_free(s);
}
