/* MIDI stream regression tests. SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"
#include "chardev/midi.h"

static void capture(void *opaque, const uint8_t *data, size_t len)
{
    g_ptr_array_add(opaque, g_bytes_new(data, len));
}

static void check(GPtrArray *messages, unsigned index,
                  const uint8_t *expected, size_t len)
{
    size_t size;
    const void *actual;

    g_assert_cmpuint(index, <, messages->len);
    actual = g_bytes_get_data(g_ptr_array_index(messages, index), &size);
    g_assert_cmpmem(actual, size, expected, len);
}

static void test_stream(void)
{
    g_autofree MidiParser *s = g_new0(MidiParser, 1);
    g_autoptr(GPtrArray) out = g_ptr_array_new_with_free_func(
                                          (GDestroyNotify)g_bytes_unref);
    const uint8_t stream[] = {
        0x12, 0x90, 60, 0xf8, 100, 61, 101, 0xc0, 5, 6,
        0xf2, 1, 2, 3, 0xf0, 0x43, 0xfe, 1, 0xf7,
        0xf0, 2, 0x80, 60, 0, 61, 1, 0xf6, 5,
        0xf4, 1, 0xf5, 1, 0xf7, 1, 0xf9, 0xfd,
    };
    const uint8_t *expected[] = {
        (uint8_t[]) { 0xf8 }, (uint8_t[]) { 0x90, 60, 100 },
        (uint8_t[]) { 0x90, 61, 101 }, (uint8_t[]) { 0xc0, 5 },
        (uint8_t[]) { 0xc0, 6 }, (uint8_t[]) { 0xf2, 1, 2 },
        (uint8_t[]) { 0xfe }, (uint8_t[]) { 0xf0, 0x43, 1, 0xf7 },
        (uint8_t[]) { 0x80, 60, 0 }, (uint8_t[]) { 0x80, 61, 1 },
        (uint8_t[]) { 0xf6 },
    };
    size_t sizes[] = { 1, 3, 3, 2, 2, 3, 1, 4, 3, 3, 1 };

    /* Exercise every split point; FPGA writes arrive one byte at a time. */
    for (size_t split = 0; split <= sizeof(stream); split++) {
        memset(s, 0, sizeof(*s));
        g_ptr_array_set_size(out, 0);
        midi_parser_write(s, stream, split, capture, out);
        midi_parser_write(s, stream + split, sizeof(stream) - split,
                           capture, out);
        g_assert_cmpuint(out->len, ==, G_N_ELEMENTS(expected));
        for (size_t i = 0; i < G_N_ELEMENTS(expected); i++) {
            check(out, i, expected[i], sizes[i]);
        }
    }
    memset(s, 0, sizeof(*s));
    g_ptr_array_set_size(out, 0);
    for (size_t i = 0; i < sizeof(stream); i++) {
        midi_parser_write(s, &stream[i], 1, capture, out);
    }
    g_assert_cmpuint(out->len, ==, G_N_ELEMENTS(expected));
    for (size_t i = 0; i < G_N_ELEMENTS(expected); i++) {
        check(out, i, expected[i], sizes[i]);
    }
}

static void test_sysex_limit(void)
{
    g_autofree MidiParser *s = g_new0(MidiParser, 1);
    g_autofree uint8_t *data = g_malloc0(MIDI_SYSEX_MAX + 1);
    g_autoptr(GPtrArray) out = g_ptr_array_new_with_free_func(
                                          (GDestroyNotify)g_bytes_unref);
    const uint8_t recovery[] = { 0xf0, 0x43, 0xf7, 0x90, 60, 100 };

    data[0] = 0xf0;
    data[MIDI_SYSEX_MAX - 1] = 0xf7;
    midi_parser_write(s, data, MIDI_SYSEX_MAX, capture, out);
    g_assert_cmpuint(out->len, ==, 1);
    check(out, 0, data, MIDI_SYSEX_MAX);
    data[MIDI_SYSEX_MAX - 1] = 0;
    data[MIDI_SYSEX_MAX] = 0xf7;
    midi_parser_write(s, data, MIDI_SYSEX_MAX + 1, capture, out);
    g_assert_cmpuint(out->len, ==, 1);
    midi_parser_write(s, recovery, sizeof(recovery), capture, out);
    g_assert_cmpuint(out->len, ==, 3);
    check(out, 1, recovery, 3);
    check(out, 2, recovery + 3, 3);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/midi/stream", test_stream);
    g_test_add_func("/midi/sysex-limit", test_sysex_limit);
    return g_test_run();
}
