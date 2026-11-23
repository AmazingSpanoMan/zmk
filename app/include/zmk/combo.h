/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zmk/behavior.h>

#define ZMK_COMBO_RUNTIME_MAX_KEYS 4

struct zmk_combo_runtime_slot {
    bool enabled;
    int32_t timeout_ms;
    int16_t require_prior_idle_ms;
    uint32_t layer_mask;
    bool slow_release;
    int16_t key_position_len;
    int32_t key_positions[ZMK_COMBO_RUNTIME_MAX_KEYS];
    struct zmk_behavior_binding behavior;
};

size_t zmk_combo_runtime_slot_count(void);
int zmk_combo_get_slot(size_t slot, struct zmk_combo_runtime_slot *out);
int zmk_combo_set_slot(size_t slot, const struct zmk_combo_runtime_slot *in);
int zmk_combo_save_settings(void);
int zmk_combo_reset_settings(void);
