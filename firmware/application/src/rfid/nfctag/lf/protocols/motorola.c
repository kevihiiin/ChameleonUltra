#include "motorola.h"

#include <stdlib.h>
#include <string.h>

#include "nordic_common.h"
#include "protocols.h"
#include "t55xx.h"
#include "tag_base_type.h"
#include "utils/pskdemod.h"

#define MOTOROLA_RAW_SIZE (64)  // 64-bit frame
#define MOTOROLA_DATA_SIZE (8)  // 8 bytes stored

// PSK1 fc/2 RF/32: 16 carrier cycles per bit (32 / 2 = 16)
// Each carrier cycle = 2 PWM entries, explicitly controlling each half-period.
// PWM base clock is 500 kHz for PSK: counter_top=4 → 4/500kHz = 8µs per entry.
// nRF52840 requires COUNTERTOP ≥ 3; the default 125 kHz clock would need counter_top=1.
#define MOTOROLA_PSK_CYCLES_PER_BIT (16)
#define MOTOROLA_PSK_ENTRIES_PER_CYCLE (2)  // 2 entries per carrier cycle (H,L or L,H)
#define MOTOROLA_PSK_COUNTER_TOP (4)        // 500 kHz clock: 4 ticks = 8µs per entry

#define MOTOROLA_T55XX_BLOCK_COUNT (3) // config + 2 data blocks

// Reader decoder: Period-8 DDC for PSK1 fc/8 at 125kHz SAADC.
// Same approach as Keri: T55XX configured for fc/8, 32 SAADC samples per bit.
#define MOTOROLA_BPS (32)              // samples per bit (32 carrier cycles at RF/32)
#define MOTOROLA_PSK_BUF_SIZE (6144)   // ~192 bits (3 frames)
#define MOTOROLA_SKIP (1024)           // skip first 32 bits — settling transient
#define MOTOROLA_MAX_BITS (160)        // max bits in decode buffer

// Motorola preamble: Indala-style 0xA0000000 in upper 32 bits
#define MOTOROLA_PREAMBLE_MASK  0xFFFFFFFF00000000ULL
#define MOTOROLA_PREAMBLE_VALUE 0xA000000000000000ULL

#define NRF_LOG_MODULE_NAME motorola_protocol
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

typedef struct {
    uint8_t data[MOTOROLA_DATA_SIZE];
    psk_t *modem;
} motorola_codec;

static motorola_codec *motorola_alloc(void) {
    motorola_codec *codec = malloc(sizeof(motorola_codec));
    codec->modem = psk_alloc(PSK_BITRATE_32, 0);
    codec->modem->buf_size = MOTOROLA_PSK_BUF_SIZE;
    codec->modem->samples = psk_shared_samples;    // shared static buffer
    return codec;
};

static void motorola_free(motorola_codec *d) {
    if (d->modem) {
        d->modem->samples = NULL;  // static buffer, don't free
        psk_free(d->modem);
        d->modem = NULL;
    }
    free(d);
};

static uint8_t *motorola_get_data(motorola_codec *d) {
    return d->data;
};

static void motorola_decoder_start(motorola_codec *d, uint8_t format) {
    memset(d->data, 0, MOTOROLA_DATA_SIZE);
    d->modem->sample_count = 0;
    d->modem->phase_offset = 0;
};

// Motorola frame check: upper 32 bits must be 0xA0000000 (Indala-style preamble)
static bool motorola_check_frame(uint64_t reg) {
    return (reg & MOTOROLA_PREAMBLE_MASK) == MOTOROLA_PREAMBLE_VALUE;
}

static void motorola_extract_data(motorola_codec *d, uint64_t reg) {
    // Store raw 64-bit frame as 8 bytes, MSB first
    for (int i = 0; i < MOTOROLA_DATA_SIZE; i++) {
        d->data[i] = (uint8_t)(reg >> (56 - i * 8));
    }
}

// Period-8 rectangular wave DDC mixer for fc/8 PSK.
// cos-like: [+1,+1,+1,+1,-1,-1,-1,-1], sin-like: [-1,-1,+1,+1,+1,+1,-1,-1]
static inline void motorola_integrate_bit(const uint16_t *samples, uint16_t base,
                                           int32_t *out_I, int32_t *out_Q) {
    static const int8_t cos8[8] = { 1,  1,  1,  1, -1, -1, -1, -1};
    static const int8_t sin8[8] = {-1, -1,  1,  1,  1,  1, -1, -1};
    int32_t I = 0, Q = 0;
    for (uint8_t k = 0; k < MOTOROLA_BPS; k++) {
        int32_t s = (int32_t)samples[base + k];
        uint8_t p = (base + k) & 7;
        I += cos8[p] * s;
        Q += sin8[p] * s;
    }
    *out_I = I;
    *out_Q = Q;
}

static bool motorola_try_decode(motorola_codec *d) {
    psk_t *m = d->modem;
    uint16_t n = m->sample_count;

    // Need: skip + 32 alignment offsets + at least 64 bits
    if (n < MOTOROLA_SKIP + MOTOROLA_BPS + MOTOROLA_RAW_SIZE * MOTOROLA_BPS) return false;

    // Phase 1: Alignment search — find best bit boundary offset.
    uint8_t best_off = 0;
    int64_t best_mag = 0;

    for (uint8_t off = 0; off < MOTOROLA_BPS; off++) {
        int64_t total_mag = 0;
        for (uint8_t bit = 0; bit < MOTOROLA_RAW_SIZE; bit++) {
            uint16_t base = MOTOROLA_SKIP + off + (uint16_t)bit * MOTOROLA_BPS;
            if (base + MOTOROLA_BPS > n) break;

            int32_t I, Q;
            motorola_integrate_bit(m->samples, base, &I, &Q);
            total_mag += (int64_t)I * I + (int64_t)Q * Q;
        }
        if (total_mag > best_mag) {
            best_mag = total_mag;
            best_off = off;
        }
    }

    // Phase 2: Compute IQ vectors for all available bits at best alignment.
    int32_t bit_I[MOTOROLA_MAX_BITS], bit_Q[MOTOROLA_MAX_BITS];
    uint16_t num_bits = 0;

    for (uint16_t bit = 0; bit < MOTOROLA_MAX_BITS; bit++) {
        uint16_t base = MOTOROLA_SKIP + best_off + (uint16_t)bit * MOTOROLA_BPS;
        if (base + MOTOROLA_BPS > n) break;

        motorola_integrate_bit(m->samples, base, &bit_I[num_bits], &bit_Q[num_bits]);
        num_bits++;
    }

    if (num_bits < MOTOROLA_RAW_SIZE) return false;

    NRF_LOG_INFO("MOTOROLA: off=%d nb=%d mag=%d",
        best_off, num_bits, (int32_t)(best_mag >> 20));

    // Phase 3: Differential PSK1 decode.
    uint8_t diff_bits[MOTOROLA_MAX_BITS];
    uint16_t ndiff = 0;
    for (uint16_t j = 1; j < num_bits; j++) {
        int64_t dot = (int64_t)bit_I[j] * bit_I[j - 1] +
                      (int64_t)bit_Q[j] * bit_Q[j - 1];
        diff_bits[ndiff++] = (dot < 0) ? 1 : 0;
    }

    // Phase 4: Integrate diff bits -> raw data bits (cumulative XOR).
    uint8_t raw_bits[MOTOROLA_MAX_BITS];
    raw_bits[0] = 0;
    for (uint16_t j = 0; j < ndiff; j++) {
        raw_bits[j + 1] = raw_bits[j] ^ diff_bits[j];
    }
    uint16_t nraw = ndiff + 1;

    // Search for Motorola frame in integrated bitstream (both polarities).
    for (uint16_t pos = 0; pos + MOTOROLA_RAW_SIZE <= nraw; pos++) {
        uint64_t reg = 0;
        for (uint8_t j = 0; j < MOTOROLA_RAW_SIZE; j++) {
            reg = (reg << 1) | raw_bits[pos + j];
        }

        if (motorola_check_frame(reg)) {
            NRF_LOG_INFO("MOTOROLA: DONE pos=%d reg=%08x%08x",
                pos, (uint32_t)(reg >> 32), (uint32_t)reg);
            motorola_extract_data(d, reg);
            return true;
        }
        if (motorola_check_frame(~reg)) {
            NRF_LOG_INFO("MOTOROLA: DONE pos=%d reg=%08x%08x (inv)",
                pos, (uint32_t)((~reg) >> 32), (uint32_t)(~reg));
            motorola_extract_data(d, ~reg);
            return true;
        }
    }

    return false;
}

static bool motorola_decoder_feed(motorola_codec *d, uint16_t val) {
    psk_t *m = d->modem;
    psk_feed_sample(m, val);

    // Wait for full buffer before attempting decode
    if (m->sample_count < m->buf_size) {
        return false;
    }

    if (motorola_try_decode(d)) {
        return true;
    }

    // Shift by 1 frame (2048 samples) to bring in fresh data
    psk_shift(m, MOTOROLA_RAW_SIZE * MOTOROLA_BPS);
    return false;
};

// PSK1 modulator: fc/2 carrier at RF/32 (500 kHz PWM clock, counter_top=4)
// Each carrier cycle uses 2 PWM entries with counter_top=4 (8µs each):
//   Phase A (bit=0): {ch0=CT,ct=CT},{ch0=0,ct=CT} → HIGH,LOW (one fc/2 cycle)
//   Phase B (bit=1): {ch0=0,ct=CT},{ch0=CT,ct=CT} → LOW,HIGH (180° shifted)
// 16 carrier cycles per bit = 32 PWM entries per bit.
static const nrf_pwm_sequence_t *motorola_modulator(motorola_codec *d, uint8_t *buf) {
    int k = 0;

    for (int i = 0; i < MOTOROLA_RAW_SIZE; i++) {
        uint8_t byte_idx = i / 8;
        uint8_t bit_idx = 7 - (i % 8); // MSB first
        bool cur_bit = (buf[byte_idx] >> bit_idx) & 1;

        // PSK1: phase = data bit value
        // ch0 = COUNTER_TOP → 100% duty (HIGH), ch0 = 0 → 0% duty (LOW)
        uint16_t first  = cur_bit ? 0 : MOTOROLA_PSK_COUNTER_TOP;
        uint16_t second = cur_bit ? MOTOROLA_PSK_COUNTER_TOP : 0;

        for (int j = 0; j < MOTOROLA_PSK_CYCLES_PER_BIT; j++) {
            psk_shared_pwm_vals[k].channel_0 = first;
            psk_shared_pwm_vals[k].counter_top = MOTOROLA_PSK_COUNTER_TOP;
            k++;
            psk_shared_pwm_vals[k].channel_0 = second;
            psk_shared_pwm_vals[k].counter_top = MOTOROLA_PSK_COUNTER_TOP;
            k++;
        }
    }

    psk_shared_pwm_seq.length = k * 4;
    return &psk_shared_pwm_seq;
};

const protocol motorola = {
    .tag_type = TAG_TYPE_MOTOROLA,
    .data_size = MOTOROLA_DATA_SIZE,
    .alloc = (codec_alloc)motorola_alloc,
    .free = (codec_free)motorola_free,
    .get_data = (codec_get_data)motorola_get_data,
    .modulator = (modulator)motorola_modulator,
    .decoder =
        {
            .start = (decoder_start)motorola_decoder_start,
            .feed = (decoder_feed)motorola_decoder_feed,
        },
};

// Encode Motorola 64-bit data to T55xx blocks
uint8_t motorola_t55xx_writer(uint8_t *uid, uint32_t *blks) {
    blks[0] = T5577_MOTOROLA_CONFIG;
    blks[1] = (uid[0] << 24) | (uid[1] << 16) | (uid[2] << 8) | uid[3];
    blks[2] = (uid[4] << 24) | (uid[5] << 16) | (uid[6] << 8) | uid[7];
    return MOTOROLA_T55XX_BLOCK_COUNT;
}
