/* Windows WinMM MIDI output. SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include <mmsystem.h>
#include "chardev/midi.h"
#include "qemu/cutils.h"
#include "qemu/error-report.h"

struct HostMidi {
    HMIDIOUT out;
    GQueue pending;
    bool warned;
};

static void midi_winmm_reap(HostMidi *s)
{
    GList *entry = s->pending.head;

    while (entry) {
        GList *next = entry->next;
        MIDIHDR *hdr = entry->data;

        if ((hdr->dwFlags & MHDR_DONE) &&
            midiOutUnprepareHeader(s->out, hdr, sizeof(*hdr)) ==
            MMSYSERR_NOERROR) {
            g_queue_delete_link(&s->pending, entry);
            g_free(hdr);
        }
        entry = next;
    }
}

HostMidi *host_midi_open(const char *name, const char *path, Error **errp)
{
    HostMidi *s = g_new0(HostMidi, 1);
    UINT device = MIDI_MAPPER;
    MMRESULT ret;

    if (path) {
        unsigned index;
        bool found = false;
        g_autoptr(GString) list = g_string_new("");

        for (UINT i = 0; i < midiOutGetNumDevs(); i++) {
            MIDIOUTCAPSA caps;

            if (midiOutGetDevCapsA(i, &caps, sizeof(caps)) !=
                MMSYSERR_NOERROR) {
                continue;
            }
            g_string_append_printf(list, "\n  %u: %s", i, caps.szPname);
            if ((!qemu_strtoui(path, NULL, 10, &index) && index == i) ||
                !strcmp(path, caps.szPname)) {
                device = i;
                found = true;
            }
        }
        if (!found) {
            error_setg(errp, "Unknown MIDI output '%s'. Available ports:%s",
                       path, list->str);
            g_free(s);
            return NULL;
        }
    }
    ret = midiOutOpen(&s->out, device, 0, 0, CALLBACK_NULL);
    if (ret != MMSYSERR_NOERROR) {
        char message[MAXERRORLENGTH] = { 0 };

        midiOutGetErrorTextA(ret, message, sizeof(message));
        error_setg(errp, "Cannot open MIDI output: %s (%u)", message, ret);
        g_free(s);
        return NULL;
    }
    return s;
}

void host_midi_send(HostMidi *s, const uint8_t *data, size_t len)
{
    MMRESULT ret;

    midi_winmm_reap(s);
    if (data[0] == 0xf0) {
        MIDIHDR *hdr;

        /* Keep asynchronous SysEx storage bounded if the device stalls. */
        if (s->pending.length >= 64) {
            ret = MIDIERR_NOTREADY;
            goto fail;
        }
        hdr = g_malloc0(sizeof(*hdr) + len);
        hdr->lpData = (char *)(hdr + 1);
        hdr->dwBufferLength = len;
        memcpy(hdr->lpData, data, len);
        ret = midiOutPrepareHeader(s->out, hdr, sizeof(*hdr));
        if (ret != MMSYSERR_NOERROR) {
            g_free(hdr);
            goto fail;
        }
        ret = midiOutLongMsg(s->out, hdr, sizeof(*hdr));
        if (ret != MMSYSERR_NOERROR) {
            midiOutUnprepareHeader(s->out, hdr, sizeof(*hdr));
            g_free(hdr);
            goto fail;
        }
        g_queue_push_tail(&s->pending, hdr);
    } else {
        DWORD message = 0;

        for (size_t i = 0; i < len; i++) {
            message |= (DWORD)data[i] << (i * 8);
        }
        ret = midiOutShortMsg(s->out, message);
    }
fail:
    if (ret != MMSYSERR_NOERROR && !s->warned) {
        warn_report("MIDI output dropped a message (WinMM error %u)", ret);
        s->warned = true;
    }
}

void host_midi_close(HostMidi *s)
{
    if (!s) {
        return;
    }
    midiOutReset(s->out);
    midi_winmm_reap(s);
    midiOutClose(s->out);
    g_free(s);
}
