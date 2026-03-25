#include "idteck.h"

#include <stdlib.h>
#include <string.h>

#include "nordic_common.h"
#include "protocols.h"
#include "t55xx.h"
#include "tag_base_type.h"
#include "utils/pskdemod.h"

#define IDTECK_RAW_SIZE (64)  // 64-bit frame
#define IDTECK_DATA_SIZE (8)  // 8 bytes stored

// PSK1 fc/2 RF/32: 16 carrier cycles per bit (32 / 2 = 16)
// Each carrier cycle = 2 PWM entries, explicitly controlling each half-period.
// PWM base clock is 500 kHz for PSK: counter_top=4 → 4/500kHz = 8µs per entry.
// nRF52840 requires COUNTERTOP ≥ 3; the default 125 kHz clock would need counter_top=1.
#define IDTECK_PSK_CYCLES_PER_BIT (16)
#define IDTECK_PSK_ENTRIES_PER_CYCLE (2)  // 2 entries per carrier cycle (H,L or L,H)
#define IDTECK_PSK_COUNTER_TOP (4)        // 500 kHz clock: 4 ticks = 8µs per entry

#define IDTECK_T55XX_BLOCK_COUNT (3) // config + 2 data blocks

// Reader decoder: Period-8 DDC for PSK1 fc/8 at 125kHz SAADC.
// Same approach as Keri/Motorola: T55XX/PM3 configured for fc/8.
#define IDTECK_BPS (32)              // samples per bit (32 carrier cycles at RF/32)
#define IDTECK_PSK_BUF_SIZE (6144)   // ~192 bits (3 frames)
#define IDTECK_SKIP (1024)           // skip first 32 bits — settling transient
#define IDTECK_MAX_BITS (160)        // max bits in decode buffer

// IDTECK preamble: 0x4944544B ("IDTK" in ASCII) in upper 32 bits
#define IDTECK_PREAMBLE_MASK  0xFFFFFFFF00000000ULL
#define IDTECK_PREAMBLE_VALUE 0x4944544B00000000ULL

#define NRF_LOG_MODULE_NAME idteck_protocol
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

typedef struct {
    uint8_t data[IDTECK_DATA_SIZE];
    psk_t *modem;
} idteck_codec;

static idteck_codec *idteck_alloc(void) {
    idteck_codec *codec = malloc(sizeof(idteck_codec));
    codec->modem = psk_alloc(PSK_BITRATE_32, 0);
    codec->modem->buf_size = IDTECK_PSK_BUF_SIZE;
    codec->modem->samples = psk_shared_samples;    // shared static buffer
    return codec;
};

static void idteck_free(idteck_codec *d) {
    if (d->modem) {
        d->modem->samples = NULL;  // static buffer, don't free
        psk_free(d->modem);
        d->modem = NULL;
    }
    free(d);
};

static uint8_t *idteck_get_data(idteck_codec *d) {
    return d->data;
};

static void idteck_decoder_start(idteck_codec *d, uint8_t format) {
    memset(d->data, 0, IDTECK_DATA_SIZE);
    d->modem->sample_count = 0;
    d->modem->phase_offset = 0;
};

// IDTECK frame check: upper 32 bits must be 0x4944544B ("IDTK" preamble)
static bool idteck_check_frame(uint64_t reg) {
    return (reg & IDTECK_PREAMBLE_MASK) == IDTECK_PREAMBLE_VALUE;
}

static void idteck_extract_data(idteck_codec *d, uint64_t reg) {
    for (int i = 0; i < IDTECK_DATA_SIZE; i++) {
        d->data[i] = (uint8_t)(reg >> (56 - i * 8));
    }
}

// Period-8 rectangular wave DDC mixer for fc/8 PSK.
static inline void idteck_integrate_bit(const uint16_t *samples, uint16_t base,
                                         int32_t *out_I, int32_t *out_Q) {
    static const int8_t cos8[8] = { 1,  1,  1,  1, -1, -1, -1, -1};
    static const int8_t sin8[8] = {-1, -1,  1,  1,  1,  1, -1, -1};
    int32_t I = 0, Q = 0;
    for (uint8_t k = 0; k < IDTECK_BPS; k++) {
        int32_t s = (int32_t)samples[base + k];
        uint8_t p = (base + k) & 7;
        I += cos8[p] * s;
        Q += sin8[p] * s;
    }
    *out_I = I;
    *out_Q = Q;
}

static bool idteck_try_decode(idteck_codec *d) {
    psk_t *m = d->modem;
    uint16_t n = m->sample_count;

    if (n < IDTECK_SKIP + IDTECK_BPS + IDTECK_RAW_SIZE * IDTECK_BPS) return false;

    // Phase 1: Alignment search — find best bit boundary offset.
    uint8_t best_off = 0;
    int64_t best_mag = 0;

    for (uint8_t off = 0; off < IDTECK_BPS; off++) {
        int64_t total_mag = 0;
        for (uint8_t bit = 0; bit < IDTECK_RAW_SIZE; bit++) {
            uint16_t base = IDTECK_SKIP + off + (uint16_t)bit * IDTECK_BPS;
            if (base + IDTECK_BPS > n) break;

            int32_t I, Q;
            idteck_integrate_bit(m->samples, base, &I, &Q);
            total_mag += (int64_t)I * I + (int64_t)Q * Q;
        }
        if (total_mag > best_mag) {
            best_mag = total_mag;
            best_off = off;
        }
    }

    // Phase 2: Compute IQ vectors for all available bits at best alignment.
    int32_t bit_I[IDTECK_MAX_BITS], bit_Q[IDTECK_MAX_BITS];
    uint16_t num_bits = 0;

    for (uint16_t bit = 0; bit < IDTECK_MAX_BITS; bit++) {
        uint16_t base = IDTECK_SKIP + best_off + (uint16_t)bit * IDTECK_BPS;
        if (base + IDTECK_BPS > n) break;

        idteck_integrate_bit(m->samples, base, &bit_I[num_bits], &bit_Q[num_bits]);
        num_bits++;
    }

    if (num_bits < IDTECK_RAW_SIZE) return false;

    NRF_LOG_INFO("IDTECK: off=%d nb=%d mag=%d",
        best_off, num_bits, (int32_t)(best_mag >> 20));

    // Phase 3: Differential PSK1 decode.
    uint8_t diff_bits[IDTECK_MAX_BITS];
    uint16_t ndiff = 0;
    for (uint16_t j = 1; j < num_bits; j++) {
        int64_t dot = (int64_t)bit_I[j] * bit_I[j - 1] +
                      (int64_t)bit_Q[j] * bit_Q[j - 1];
        diff_bits[ndiff++] = (dot < 0) ? 1 : 0;
    }

    // Phase 4: Integrate diff bits -> raw data bits (cumulative XOR).
    uint8_t raw_bits[IDTECK_MAX_BITS];
    raw_bits[0] = 0;
    for (uint16_t j = 0; j < ndiff; j++) {
        raw_bits[j + 1] = raw_bits[j] ^ diff_bits[j];
    }
    uint16_t nraw = ndiff + 1;

    // Search for IDTECK frame in integrated bitstream (both polarities).
    for (uint16_t pos = 0; pos + IDTECK_RAW_SIZE <= nraw; pos++) {
        uint64_t reg = 0;
        for (uint8_t j = 0; j < IDTECK_RAW_SIZE; j++) {
            reg = (reg << 1) | raw_bits[pos + j];
        }

        if (idteck_check_frame(reg)) {
            NRF_LOG_INFO("IDTECK: DONE pos=%d reg=%08x%08x",
                pos, (uint32_t)(reg >> 32), (uint32_t)reg);
            idteck_extract_data(d, reg);
            return true;
        }
        if (idteck_check_frame(~reg)) {
            NRF_LOG_INFO("IDTECK: DONE pos=%d reg=%08x%08x (inv)",
                pos, (uint32_t)((~reg) >> 32), (uint32_t)(~reg));
            idteck_extract_data(d, ~reg);
            return true;
        }
    }

    return false;
}

static bool idteck_decoder_feed(idteck_codec *d, uint16_t val) {
    psk_t *m = d->modem;
    psk_feed_sample(m, val);

    if (m->sample_count < m->buf_size) {
        return false;
    }

    if (idteck_try_decode(d)) {
        return true;
    }

    // Shift by 1 frame (2048 samples) to bring in fresh data
    psk_shift(m, IDTECK_RAW_SIZE * IDTECK_BPS);
    return false;
};

// PSK1 modulator: fc/2 carrier at RF/32 (500 kHz PWM clock, counter_top=4)
// Each carrier cycle uses 2 PWM entries with counter_top=4 (8µs each):
//   Phase A (bit=0): {ch0=CT,ct=CT},{ch0=0,ct=CT} → HIGH,LOW (one fc/2 cycle)
//   Phase B (bit=1): {ch0=0,ct=CT},{ch0=CT,ct=CT} → LOW,HIGH (180° shifted)
// 16 carrier cycles per bit = 32 PWM entries per bit.
static const nrf_pwm_sequence_t *idteck_modulator(idteck_codec *d, uint8_t *buf) {
    int k = 0;

    for (int i = 0; i < IDTECK_RAW_SIZE; i++) {
        uint8_t byte_idx = i / 8;
        uint8_t bit_idx = 7 - (i % 8); // MSB first
        bool cur_bit = (buf[byte_idx] >> bit_idx) & 1;

        // PSK1: phase = data bit value
        // ch0 = COUNTER_TOP → 100% duty (HIGH), ch0 = 0 → 0% duty (LOW)
        uint16_t first  = cur_bit ? 0 : IDTECK_PSK_COUNTER_TOP;
        uint16_t second = cur_bit ? IDTECK_PSK_COUNTER_TOP : 0;

        for (int j = 0; j < IDTECK_PSK_CYCLES_PER_BIT; j++) {
            psk_shared_pwm_vals[k].channel_0 = first;
            psk_shared_pwm_vals[k].counter_top = IDTECK_PSK_COUNTER_TOP;
            k++;
            psk_shared_pwm_vals[k].channel_0 = second;
            psk_shared_pwm_vals[k].counter_top = IDTECK_PSK_COUNTER_TOP;
            k++;
        }
    }

    psk_shared_pwm_seq.length = k * 4;
    return &psk_shared_pwm_seq;
};

const protocol idteck = {
    .tag_type = TAG_TYPE_IDTECK,
    .data_size = IDTECK_DATA_SIZE,
    .alloc = (codec_alloc)idteck_alloc,
    .free = (codec_free)idteck_free,
    .get_data = (codec_get_data)idteck_get_data,
    .modulator = (modulator)idteck_modulator,
    .decoder =
        {
            .start = (decoder_start)idteck_decoder_start,
            .feed = (decoder_feed)idteck_decoder_feed,
        },
};

// Encode IDTECK 64-bit data to T55xx blocks
uint8_t idteck_t55xx_writer(uint8_t *uid, uint32_t *blks) {
    blks[0] = T5577_IDTECK_CONFIG;
    blks[1] = (uid[0] << 24) | (uid[1] << 16) | (uid[2] << 8) | uid[3];
    blks[2] = (uid[4] << 24) | (uid[5] << 16) | (uid[6] << 8) | uid[7];
    return IDTECK_T55XX_BLOCK_COUNT;
}
