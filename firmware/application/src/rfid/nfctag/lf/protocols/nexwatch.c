#include "nexwatch.h"

#include <stdlib.h>
#include <string.h>

#include "nordic_common.h"
#include "protocols.h"
#include "t55xx.h"
#include "tag_base_type.h"
#include "utils/pskdemod.h"

#define NEXWATCH_RAW_SIZE (96)  // 96-bit frame (3 x 32-bit blocks)
#define NEXWATCH_DATA_SIZE (12) // 12 bytes stored

// PSK1 fc/2 RF/32: 16 carrier cycles per bit
// Each carrier cycle = 2 PWM entries for explicit phase control.
// PWM base clock is 500 kHz for PSK: counter_top=4 → 4/500kHz = 8µs per entry.
#define NEXWATCH_PSK_CYCLES_PER_BIT (16)
#define NEXWATCH_PSK_ENTRIES_PER_CYCLE (2)
#define NEXWATCH_PSK_COUNTER_TOP (4)  // 500 kHz clock: 4 ticks = 8µs per entry

#define NEXWATCH_T55XX_BLOCK_COUNT (4) // config + 3 data blocks

// Reader decoder: Period-8 DDC for PSK1 fc/8 at 125kHz SAADC.
// Same approach as Keri: T55XX configured for fc/8, 32 SAADC samples per bit.
#define NEXWATCH_BPS (32)              // samples per bit (32 carrier cycles at RF/32)
#define NEXWATCH_PSK_BUF_SIZE (6144)   // ~192 bits worth (~2 frames)
#define NEXWATCH_SKIP (1024)           // skip first 32 bits — settling transient
#define NEXWATCH_MAX_BITS (200)        // max bits in decode buffer (~2 frames)
#define NEXWATCH_SHIFT (2560)          // non-aligned shift (80 bits) to sweep gap

#define NRF_LOG_MODULE_NAME nexwatch_protocol
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

typedef struct {
    uint8_t data[NEXWATCH_DATA_SIZE];
    psk_t *modem;
} nexwatch_codec;

static nexwatch_codec *nexwatch_alloc(void) {
    nexwatch_codec *codec = malloc(sizeof(nexwatch_codec));
    codec->modem = psk_alloc(PSK_BITRATE_32, 0);
    codec->modem->buf_size = NEXWATCH_PSK_BUF_SIZE;
    codec->modem->samples = psk_shared_samples;    // shared static buffer
    return codec;
};

static void nexwatch_free(nexwatch_codec *d) {
    if (d->modem) {
        d->modem->samples = NULL;  // static buffer, don't free
        psk_free(d->modem);
        d->modem = NULL;
    }
    free(d);
};

static uint8_t *nexwatch_get_data(nexwatch_codec *d) {
    return d->data;
};

static void nexwatch_decoder_start(nexwatch_codec *d, uint8_t format) {
    memset(d->data, 0, NEXWATCH_DATA_SIZE);
    d->modem->sample_count = 0;
    d->modem->phase_offset = 0;
};

// NexWatch parity: XOR all nibbles of hex[0..4] (scrambled ID + mode), then swap (1234)->(4231).
// Covers bytes 5-9 of the 12-byte frame (bits 40-79 of the 96-bit frame).
static uint8_t nexwatch_parity_swap(uint8_t parity) {
    uint8_t a = ((parity >> 3) & 1);
    a |= (((parity >> 1) & 1) << 1);
    a |= (((parity >> 2) & 1) << 2);
    a |= ((parity & 1) << 3);
    return a;
}

static bool nexwatch_check_parity(uint8_t *data) {
    // hex[0..4] = data[5..9], with data[9] masked to upper nibble (mode only)
    uint8_t p = 0;
    for (uint8_t i = 5; i <= 9; i++) {
        uint8_t b = (i == 9) ? (data[i] & 0xF0) : data[i];
        p ^= (b >> 4) & 0x0F;
        p ^= b & 0x0F;
    }
    uint8_t calc = nexwatch_parity_swap(p);
    uint8_t stored = data[9] & 0x0F;
    return calc == stored;
}

// Build byte from 8 raw bits (MSB-first) at position pos, optionally inverted
static inline uint8_t nexwatch_byte_at(uint8_t *bits, uint16_t pos, bool inv) {
    uint8_t b = 0;
    for (int j = 0; j < 8; j++) {
        b = (b << 1) | (bits[pos + j] ^ (inv ? 1 : 0));
    }
    return b;
}

// Period-8 rectangular wave DDC mixer for fc/8 PSK.
// cos-like: [+1,+1,+1,+1,-1,-1,-1,-1], sin-like: [-1,-1,+1,+1,+1,+1,-1,-1]
static inline void nexwatch_integrate_bit(const uint16_t *samples, uint16_t base,
                                          int32_t *out_I, int32_t *out_Q) {
    static const int8_t cos8[8] = { 1,  1,  1,  1, -1, -1, -1, -1};
    static const int8_t sin8[8] = {-1, -1,  1,  1,  1,  1, -1, -1};
    int32_t I = 0, Q = 0;
    for (uint8_t k = 0; k < NEXWATCH_BPS; k++) {
        int32_t s = (int32_t)samples[base + k];
        uint8_t p = (base + k) & 7;
        I += cos8[p] * s;
        Q += sin8[p] * s;
    }
    *out_I = I;
    *out_Q = Q;
}

static bool nexwatch_try_decode(nexwatch_codec *d) {
    psk_t *m = d->modem;
    uint16_t n = m->sample_count;

    // Need: skip + 32 alignment offsets + at least 96 bits
    if (n < NEXWATCH_SKIP + NEXWATCH_BPS + NEXWATCH_RAW_SIZE * NEXWATCH_BPS) return false;

    // Phase 1: Alignment search — find best bit boundary offset.
    uint8_t best_off = 0;
    int64_t best_mag = 0;

    for (uint8_t off = 0; off < NEXWATCH_BPS; off++) {
        int64_t total_mag = 0;
        for (uint8_t bit = 0; bit < NEXWATCH_RAW_SIZE; bit++) {
            uint16_t base = NEXWATCH_SKIP + off + (uint16_t)bit * NEXWATCH_BPS;
            if (base + NEXWATCH_BPS > n) break;

            int32_t I, Q;
            nexwatch_integrate_bit(m->samples, base, &I, &Q);
            total_mag += (int64_t)I * I + (int64_t)Q * Q;
        }
        if (total_mag > best_mag) {
            best_mag = total_mag;
            best_off = off;
        }
    }

    // Phase 2: Compute IQ vectors for all available bits at best alignment.
    // Static to avoid ~2KB stack pressure from reader framework call depth.
    static int32_t bit_I[NEXWATCH_MAX_BITS], bit_Q[NEXWATCH_MAX_BITS];
    uint16_t num_bits = 0;

    for (uint16_t bit = 0; bit < NEXWATCH_MAX_BITS; bit++) {
        uint16_t base = NEXWATCH_SKIP + best_off + (uint16_t)bit * NEXWATCH_BPS;
        if (base + NEXWATCH_BPS > n) break;

        nexwatch_integrate_bit(m->samples, base, &bit_I[num_bits], &bit_Q[num_bits]);
        num_bits++;
    }

    if (num_bits < NEXWATCH_RAW_SIZE) return false;

    NRF_LOG_INFO("NEXWATCH: off=%d nb=%d mag=%d",
        best_off, num_bits, (int32_t)(best_mag >> 20));

    // Phase 3: Differential PSK1 decode.
    static uint8_t diff_bits[NEXWATCH_MAX_BITS];
    uint16_t ndiff = 0;
    for (uint16_t j = 1; j < num_bits; j++) {
        int64_t dot = (int64_t)bit_I[j] * bit_I[j - 1] +
                      (int64_t)bit_Q[j] * bit_Q[j - 1];
        diff_bits[ndiff++] = (dot < 0) ? 1 : 0;
    }

    // Phase 4: Integrate diff bits -> raw data bits (cumulative XOR).
    static uint8_t raw_bits[NEXWATCH_MAX_BITS];
    raw_bits[0] = 0;
    for (uint16_t j = 0; j < ndiff; j++) {
        raw_bits[j + 1] = raw_bits[j] ^ diff_bits[j];
    }
    uint16_t nraw = ndiff + 1;

    // Phase 5: Search for NexWatch frame with double-frame confirmation.
    // Collect positions where byte 0 = 0x56 (normal) or 0xA9 (inverted).
    // Confirm by finding a pair separated by exactly 96 bits.
    uint16_t norm_pos[8];
    uint8_t nnorm = 0;
    uint16_t inv_pos[8];
    uint8_t ninv = 0;

    for (uint16_t pos = 0; pos + 8 <= nraw; pos++) {
        uint8_t byte0 = nexwatch_byte_at(raw_bits, pos, false);
        if (byte0 == 0x56 && nnorm < 8) {
            norm_pos[nnorm++] = pos;
        }
        if (byte0 == 0xA9 && ninv < 8) {
            inv_pos[ninv++] = pos;
        }
    }

    // Find confirmed pair (separated by exactly NEXWATCH_RAW_SIZE)
    // Extract data from first frame, validate with NexWatch parity check
    for (uint8_t i = 0; i < nnorm; i++) {
        if (norm_pos[i] + NEXWATCH_RAW_SIZE + 8 > nraw) continue;
        for (uint8_t j = i + 1; j < nnorm; j++) {
            if (norm_pos[j] - norm_pos[i] == NEXWATCH_RAW_SIZE) {
                for (int b = 0; b < NEXWATCH_DATA_SIZE; b++) {
                    d->data[b] = nexwatch_byte_at(raw_bits, norm_pos[i] + b * 8, false);
                }
                if (!nexwatch_check_parity(d->data)) {
                    NRF_LOG_INFO("NEXWATCH: parity fail at pos=%d", norm_pos[i]);
                    continue;
                }
                NRF_LOG_INFO("NEXWATCH: DONE pos=%d confirmed at %d",
                    norm_pos[i], norm_pos[j]);
                return true;
            }
        }
    }

    // Same for inverted
    for (uint8_t i = 0; i < ninv; i++) {
        if (inv_pos[i] + NEXWATCH_RAW_SIZE + 8 > nraw) continue;
        for (uint8_t j = i + 1; j < ninv; j++) {
            if (inv_pos[j] - inv_pos[i] == NEXWATCH_RAW_SIZE) {
                for (int b = 0; b < NEXWATCH_DATA_SIZE; b++) {
                    d->data[b] = nexwatch_byte_at(raw_bits, inv_pos[i] + b * 8, true);
                }
                if (!nexwatch_check_parity(d->data)) {
                    NRF_LOG_INFO("NEXWATCH: inv parity fail at pos=%d", inv_pos[i]);
                    continue;
                }
                NRF_LOG_INFO("NEXWATCH: DONE pos=%d (inv) confirmed at %d",
                    inv_pos[i], inv_pos[j]);
                return true;
            }
        }
    }

    return false;
}

static bool nexwatch_decoder_feed(nexwatch_codec *d, uint16_t val) {
    psk_t *m = d->modem;
    psk_feed_sample(m, val);

    // Wait for full buffer before attempting decode
    if (m->sample_count < m->buf_size) {
        return false;
    }

    if (nexwatch_try_decode(d)) {
        return true;
    }

    // Non-aligned shift (80 bits) to sweep frame boundary across gap zone
    psk_shift(m, NEXWATCH_SHIFT);
    return false;
};

// PSK1 modulator: fc/2 carrier at RF/32 (500 kHz PWM clock, counter_top=4)
static const nrf_pwm_sequence_t *nexwatch_modulator(nexwatch_codec *d, uint8_t *buf) {
    int k = 0;

    for (int i = 0; i < NEXWATCH_RAW_SIZE; i++) {
        uint8_t byte_idx = i / 8;
        uint8_t bit_idx = 7 - (i % 8);
        bool cur_bit = (buf[byte_idx] >> bit_idx) & 1;

        uint16_t first  = cur_bit ? 0 : NEXWATCH_PSK_COUNTER_TOP;
        uint16_t second = cur_bit ? NEXWATCH_PSK_COUNTER_TOP : 0;

        for (int j = 0; j < NEXWATCH_PSK_CYCLES_PER_BIT; j++) {
            psk_shared_pwm_vals[k].channel_0 = first;
            psk_shared_pwm_vals[k].counter_top = NEXWATCH_PSK_COUNTER_TOP;
            k++;
            psk_shared_pwm_vals[k].channel_0 = second;
            psk_shared_pwm_vals[k].counter_top = NEXWATCH_PSK_COUNTER_TOP;
            k++;
        }
    }

    psk_shared_pwm_seq.length = k * 4;
    return &psk_shared_pwm_seq;
};

const protocol nexwatch = {
    .tag_type = TAG_TYPE_NEXWATCH,
    .data_size = NEXWATCH_DATA_SIZE,
    .alloc = (codec_alloc)nexwatch_alloc,
    .free = (codec_free)nexwatch_free,
    .get_data = (codec_get_data)nexwatch_get_data,
    .modulator = (modulator)nexwatch_modulator,
    .decoder =
        {
            .start = (decoder_start)nexwatch_decoder_start,
            .feed = (decoder_feed)nexwatch_decoder_feed,
        },
};

// Encode NexWatch data to T55xx blocks
uint8_t nexwatch_t55xx_writer(uint8_t *uid, uint32_t *blks) {
    blks[0] = T5577_NEXWATCH_CONFIG;
    blks[1] = (uid[0] << 24) | (uid[1] << 16) | (uid[2] << 8) | uid[3];
    blks[2] = (uid[4] << 24) | (uid[5] << 16) | (uid[6] << 8) | uid[7];
    blks[3] = (uid[8] << 24) | (uid[9] << 16) | (uid[10] << 8) | uid[11];
    return NEXWATCH_T55XX_BLOCK_COUNT;
}
