#include "keri.h"

#include <stdlib.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf_pwm.h"
#include "protocols.h"
#include "t55xx.h"
#include "tag_base_type.h"
#include "utils/pskdemod.h"

#define NRF_LOG_MODULE_NAME keri_protocol
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

#define KERI_RAW_SIZE (64)  // 64-bit frame
#define KERI_DATA_SIZE (8)  // 8 bytes stored

// PSK1 fc/2 RF/32: 16 carrier cycles per bit (emulation)
// Each carrier cycle = 2 PWM entries for explicit phase control.
// PWM base clock is 500 kHz for PSK: counter_top=4 → 4/500kHz = 8µs per entry.
#define KERI_PSK_CYCLES_PER_BIT (16)
#define KERI_PSK_ENTRIES_PER_CYCLE (2)
#define KERI_PSK_COUNTER_TOP (4)  // 500 kHz clock: 4 ticks = 8µs per entry

#define KERI_T55XX_BLOCK_COUNT (3) // config + 2 data blocks

// Reader decoder: Period-8 DDC for PSK1 fc/8 at 125kHz SAADC.
// Same approach as Indala: T55XX configured for fc/8, 32 SAADC samples per bit.
#define KERI_BPS (32)              // samples per bit (32 carrier cycles at RF/32)
#define KERI_PSK_BUF_SIZE (6144)   // ~192 bits (3 frames)
#define KERI_SKIP (1024)           // skip first 32 bits — settling transient
#define KERI_MAX_BITS (160)        // max bits in decode buffer

static nrf_pwm_values_wave_form_t m_keri_pwm_seq_vals[KERI_RAW_SIZE * KERI_PSK_CYCLES_PER_BIT * KERI_PSK_ENTRIES_PER_CYCLE] = {};

static nrf_pwm_sequence_t m_keri_pwm_seq = {
    .values.p_wave_form = m_keri_pwm_seq_vals,
    .length = 0,  // Set dynamically by modulator
    .repeats = 0,
    .end_delay = 0,
};

typedef struct {
    uint8_t data[KERI_DATA_SIZE];
    psk_t *modem;
} keri_codec;

static keri_codec *keri_alloc(void) {
    keri_codec *codec = malloc(sizeof(keri_codec));
    codec->modem = psk_alloc(PSK_BITRATE_32, 0);
    codec->modem->buf_size = KERI_PSK_BUF_SIZE;
    codec->modem->samples = psk_shared_samples;    // shared static buffer
    return codec;
};

static void keri_free(keri_codec *d) {
    if (d->modem) {
        d->modem->samples = NULL;  // static buffer, don't free
        psk_free(d->modem);
        d->modem = NULL;
    }
    free(d);
};

static uint8_t *keri_get_data(keri_codec *d) {
    return d->data;
};

static void keri_decoder_start(keri_codec *d, uint8_t format) {
    memset(d->data, 0, KERI_DATA_SIZE);
    d->modem->sample_count = 0;
    d->modem->phase_offset = 0;
};

// Keri frame check: 29 zero preamble + bit 34 must be 1 + trailer 111
static bool keri_check_frame(uint64_t reg) {
    // Upper 29 bits must be zero (preamble)
    if (reg >> 35 != 0) return false;
    // Bit 34 must be 1 (MSB of internal ID, forced by | 0x80000000)
    if (!(reg & ((uint64_t)1 << 34))) return false;
    // Lower 3 bits must be 111 (trailer)
    if ((reg & 7) != 7) return false;
    return true;
}

static void keri_extract_data(keri_codec *d, uint64_t reg) {
    // Store raw 64-bit frame as 8 bytes, MSB first
    for (int i = 0; i < KERI_DATA_SIZE; i++) {
        d->data[i] = (uint8_t)(reg >> (56 - i * 8));
    }
}

// Period-8 rectangular wave DDC mixer for fc/8 PSK.
// cos-like: [+1,+1,+1,+1,-1,-1,-1,-1], sin-like: [-1,-1,+1,+1,+1,+1,-1,-1]
static inline void keri_integrate_bit(const uint16_t *samples, uint16_t base,
                                      int32_t *out_I, int32_t *out_Q) {
    static const int8_t cos8[8] = { 1,  1,  1,  1, -1, -1, -1, -1};
    static const int8_t sin8[8] = {-1, -1,  1,  1,  1,  1, -1, -1};
    int32_t I = 0, Q = 0;
    for (uint8_t k = 0; k < KERI_BPS; k++) {
        int32_t s = (int32_t)samples[base + k];
        uint8_t p = (base + k) & 7;
        I += cos8[p] * s;
        Q += sin8[p] * s;
    }
    *out_I = I;
    *out_Q = Q;
}

static bool keri_try_decode(keri_codec *d) {
    psk_t *m = d->modem;
    uint16_t n = m->sample_count;

    // Need: skip + 32 alignment offsets + at least 64 bits
    if (n < KERI_SKIP + KERI_BPS + KERI_RAW_SIZE * KERI_BPS) return false;

    // Phase 1: Alignment search — find best bit boundary offset.
    uint8_t best_off = 0;
    int64_t best_mag = 0;

    for (uint8_t off = 0; off < KERI_BPS; off++) {
        int64_t total_mag = 0;
        for (uint8_t bit = 0; bit < KERI_RAW_SIZE; bit++) {
            uint16_t base = KERI_SKIP + off + (uint16_t)bit * KERI_BPS;
            if (base + KERI_BPS > n) break;

            int32_t I, Q;
            keri_integrate_bit(m->samples, base, &I, &Q);
            total_mag += (int64_t)I * I + (int64_t)Q * Q;
        }
        if (total_mag > best_mag) {
            best_mag = total_mag;
            best_off = off;
        }
    }

    // Phase 2: Compute IQ vectors for all available bits at best alignment.
    int32_t bit_I[KERI_MAX_BITS], bit_Q[KERI_MAX_BITS];
    uint16_t num_bits = 0;

    for (uint16_t bit = 0; bit < KERI_MAX_BITS; bit++) {
        uint16_t base = KERI_SKIP + best_off + (uint16_t)bit * KERI_BPS;
        if (base + KERI_BPS > n) break;

        keri_integrate_bit(m->samples, base, &bit_I[num_bits], &bit_Q[num_bits]);
        num_bits++;
    }

    if (num_bits < KERI_RAW_SIZE) return false;

    NRF_LOG_INFO("KERI: off=%d nb=%d mag=%d",
        best_off, num_bits, (int32_t)(best_mag >> 20));

    // Phase 3: Differential PSK1 decode.
    uint8_t diff_bits[KERI_MAX_BITS];
    uint16_t ndiff = 0;
    for (uint16_t j = 1; j < num_bits; j++) {
        int64_t dot = (int64_t)bit_I[j] * bit_I[j - 1] +
                      (int64_t)bit_Q[j] * bit_Q[j - 1];
        diff_bits[ndiff++] = (dot < 0) ? 1 : 0;
    }

    // Phase 4: Integrate diff bits -> raw data bits (cumulative XOR).
    uint8_t raw_bits[KERI_MAX_BITS];
    raw_bits[0] = 0;
    for (uint16_t j = 0; j < ndiff; j++) {
        raw_bits[j + 1] = raw_bits[j] ^ diff_bits[j];
    }
    uint16_t nraw = ndiff + 1;

    // Search for Keri frame in integrated bitstream (both polarities).
    for (uint16_t pos = 0; pos + KERI_RAW_SIZE <= nraw; pos++) {
        uint64_t reg = 0;
        for (uint8_t j = 0; j < KERI_RAW_SIZE; j++) {
            reg = (reg << 1) | raw_bits[pos + j];
        }

        if (keri_check_frame(reg)) {
            NRF_LOG_INFO("KERI: DONE pos=%d reg=%08x%08x",
                pos, (uint32_t)(reg >> 32), (uint32_t)reg);
            keri_extract_data(d, reg);
            return true;
        }
        if (keri_check_frame(~reg)) {
            NRF_LOG_INFO("KERI: DONE pos=%d reg=%08x%08x (inv)",
                pos, (uint32_t)((~reg) >> 32), (uint32_t)(~reg));
            keri_extract_data(d, ~reg);
            return true;
        }
    }

    return false;
}

static bool keri_decoder_feed(keri_codec *d, uint16_t val) {
    psk_t *m = d->modem;
    psk_feed_sample(m, val);

    // Wait for full buffer before attempting decode
    if (m->sample_count < m->buf_size) {
        return false;
    }

    if (keri_try_decode(d)) {
        return true;
    }

    // Shift by 1 frame (2048 samples) to bring in fresh data
    psk_shift(m, KERI_RAW_SIZE * KERI_BPS);
    return false;
};

// PSK1 modulator: fc/2 carrier at RF/32 (500 kHz PWM clock, counter_top=4)
static const nrf_pwm_sequence_t *keri_modulator(keri_codec *d, uint8_t *buf) {
    int k = 0;

    for (int i = 0; i < KERI_RAW_SIZE; i++) {
        uint8_t byte_idx = i / 8;
        uint8_t bit_idx = 7 - (i % 8);
        bool cur_bit = (buf[byte_idx] >> bit_idx) & 1;

        uint16_t first  = cur_bit ? 0 : KERI_PSK_COUNTER_TOP;
        uint16_t second = cur_bit ? KERI_PSK_COUNTER_TOP : 0;

        for (int j = 0; j < KERI_PSK_CYCLES_PER_BIT; j++) {
            m_keri_pwm_seq_vals[k].channel_0 = first;
            m_keri_pwm_seq_vals[k].counter_top = KERI_PSK_COUNTER_TOP;
            k++;
            m_keri_pwm_seq_vals[k].channel_0 = second;
            m_keri_pwm_seq_vals[k].counter_top = KERI_PSK_COUNTER_TOP;
            k++;
        }
    }

    m_keri_pwm_seq.length = k * 4;
    return &m_keri_pwm_seq;
};

const protocol keri = {
    .tag_type = TAG_TYPE_KERI,
    .data_size = KERI_DATA_SIZE,
    .alloc = (codec_alloc)keri_alloc,
    .free = (codec_free)keri_free,
    .get_data = (codec_get_data)keri_get_data,
    .modulator = (modulator)keri_modulator,
    .decoder =
        {
            .start = (decoder_start)keri_decoder_start,
            .feed = (decoder_feed)keri_decoder_feed,
        },
};

// Encode Keri data to T55xx blocks (fc/8 config for CU reader compatibility)
uint8_t keri_t55xx_writer(uint8_t *uid, uint32_t *blks) {
    blks[0] = T5577_KERI_CONFIG;
    blks[1] = (uid[0] << 24) | (uid[1] << 16) | (uid[2] << 8) | uid[3];
    blks[2] = (uid[4] << 24) | (uid[5] << 16) | (uid[6] << 8) | uid[7];
    return KERI_T55XX_BLOCK_COUNT;
}
