/*
 * Test ALSA output without a host MIDI device.
 * SPDX-License-Identifier: GPL-2.0-or-later
 */
#include "qemu/osdep.h"
#include <alsa/asoundlib.h>
#include "chardev/midi.h"

/* Substitute only the sequencer transport; use ALSA's real MIDI codec. */
static bool closed;
static int destination;
static unsigned sent;
static snd_midi_event_t *decoder;
static const uint8_t *expected;
static size_t expected_len;

int snd_seq_open(snd_seq_t **seq, const char *name, int streams, int mode)
{
    g_assert_cmpstr(name, ==, "default");
    g_assert_cmpint(streams, ==, SND_SEQ_OPEN_OUTPUT);
    g_assert_true(mode & SND_SEQ_NONBLOCK);
    *seq = (snd_seq_t *)&closed;
    closed = false;
    return 0;
}

int snd_seq_close(snd_seq_t *seq)
{
    closed = true;
    return 0;
}

int snd_seq_set_client_name(snd_seq_t *seq, const char *name)
{
    g_assert_cmpstr(name, ==, "QEMU MIDI");
    return 0;
}

int snd_seq_create_simple_port(snd_seq_t *seq, const char *name,
                               unsigned int caps, unsigned int type)
{
    g_assert_cmpstr(name, ==, "test-output");
    g_assert_true(caps & SND_SEQ_PORT_CAP_SUBS_READ);
    g_assert_true(caps & SND_SEQ_PORT_CAP_READ);
    return 7;
}

int snd_seq_connect_to(snd_seq_t *seq, int src, int client, int port)
{
    g_assert_cmpint(src, ==, 7);
    destination = client * 256 + port;
    return 0;
}

int snd_seq_event_output_direct(snd_seq_t *seq, snd_seq_event_t *ev)
{
    g_autofree uint8_t *data = g_malloc(MIDI_SYSEX_MAX);
    long len;

    g_assert_cmpint(ev->source.port, ==, 7);
    g_assert_cmpint(ev->dest.client, ==, SND_SEQ_ADDRESS_SUBSCRIBERS);
    g_assert_cmpint(ev->queue, ==, SND_SEQ_QUEUE_DIRECT);
    len = snd_midi_event_decode(decoder, data, MIDI_SYSEX_MAX, ev);
    g_assert_cmpint(len, ==, expected_len);
    g_assert_cmpmem(data, len, expected, expected_len);
    sent++;
    return 0;
}

static void test_output(void)
{
    Error *err = NULL;
    HostMidi *host;
    uint8_t messages[][4] = {
        { 0x90, 60, 100 }, { 0x80, 60, 0 }, { 0xc0, 5 },
        { 0xf2, 1, 2 }, { 0xf8 }, { 0xf0, 0x43, 0x12, 0xf7 },
    };
    size_t sizes[] = { 3, 3, 2, 3, 1, 4 };

    g_assert_cmpint(snd_midi_event_new(MIDI_SYSEX_MAX, &decoder), ==, 0);
    snd_midi_event_no_status(decoder, 1);
    host = host_midi_open("test-output", "128:2", &err);
    g_assert_null(err);
    g_assert_nonnull(host);
    g_assert_cmpint(destination, ==, 128 * 256 + 2);
    for (size_t i = 0; i < G_N_ELEMENTS(messages); i++) {
        expected = messages[i];
        expected_len = sizes[i];
        host_midi_send(host, expected, expected_len);
    }
    g_assert_cmpuint(sent, ==, G_N_ELEMENTS(messages));
    host_midi_close(host);
    g_assert_true(closed);
    /* No destination means the source can be connected after launch. */
    destination = -1;
    host = host_midi_open("test-output", NULL, &err);
    g_assert_null(err);
    g_assert_nonnull(host);
    g_assert_cmpint(destination, ==, -1);
    host_midi_close(host);
    host_midi_close(NULL);
    snd_midi_event_free(decoder);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/midi/alsa-output", test_output);
    return g_test_run();
}
