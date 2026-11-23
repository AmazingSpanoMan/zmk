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

#define ZMK_GREMLIN_MAX_HOLD_TAPS 24
#define ZMK_GREMLIN_MAX_TAP_DANCES 8
#define ZMK_GREMLIN_MAX_TAP_DANCE_STEPS 4
#define ZMK_GREMLIN_MAX_MACROS 8
#define ZMK_GREMLIN_MAX_MACRO_STEPS 12

enum zmk_gremlin_hold_tap_flavor {
    ZMK_GREMLIN_HT_HOLD_PREFERRED = 0,
    ZMK_GREMLIN_HT_BALANCED = 1,
    ZMK_GREMLIN_HT_TAP_PREFERRED = 2,
};

struct zmk_gremlin_hold_tap_slot {
    bool enabled;
    int32_t tapping_term_ms;
    int32_t quick_tap_ms;
    int32_t require_prior_idle_ms;
    uint8_t flavor;
    bool retro_tap;
    struct zmk_behavior_binding hold_binding;
    struct zmk_behavior_binding tap_binding;
};

struct zmk_gremlin_tap_dance_slot {
    bool enabled;
    uint32_t tapping_term_ms;
    uint8_t binding_count;
    struct zmk_behavior_binding bindings[ZMK_GREMLIN_MAX_TAP_DANCE_STEPS];
};

enum zmk_gremlin_macro_step_type {
    ZMK_GREMLIN_MACRO_STEP_TAP = 0,
    ZMK_GREMLIN_MACRO_STEP_PRESS = 1,
    ZMK_GREMLIN_MACRO_STEP_RELEASE = 2,
    ZMK_GREMLIN_MACRO_STEP_WAIT_MS = 3,
};

struct zmk_gremlin_macro_step {
    uint8_t type;
    struct zmk_behavior_binding binding;
    uint32_t delay_ms;
};

struct zmk_gremlin_macro_slot {
    bool enabled;
    uint32_t wait_ms;
    uint32_t tap_ms;
    uint8_t step_count;
    struct zmk_gremlin_macro_step steps[ZMK_GREMLIN_MAX_MACRO_STEPS];
};

size_t zmk_gremlin_hold_tap_count(void);
int zmk_gremlin_hold_tap_get(size_t slot, struct zmk_gremlin_hold_tap_slot *out);
int zmk_gremlin_hold_tap_set(size_t slot, const struct zmk_gremlin_hold_tap_slot *in);

size_t zmk_gremlin_tap_dance_count(void);
int zmk_gremlin_tap_dance_get(size_t slot, struct zmk_gremlin_tap_dance_slot *out);
int zmk_gremlin_tap_dance_set(size_t slot, const struct zmk_gremlin_tap_dance_slot *in);

size_t zmk_gremlin_macro_count(void);
int zmk_gremlin_macro_get(size_t slot, struct zmk_gremlin_macro_slot *out);
int zmk_gremlin_macro_set(size_t slot, const struct zmk_gremlin_macro_slot *in);

int zmk_gremlin_behaviors_save_settings(void);
int zmk_gremlin_behaviors_reset_settings(void);

int gremlin_hold_tap_save_all(void);
int gremlin_hold_tap_reset_all(void);
int gremlin_tap_dance_save_all(void);
int gremlin_tap_dance_reset_all(void);
int gremlin_macro_save_all(void);
int gremlin_macro_reset_all(void);
