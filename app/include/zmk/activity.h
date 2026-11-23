/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>

enum zmk_activity_state { ZMK_ACTIVITY_ACTIVE, ZMK_ACTIVITY_IDLE, ZMK_ACTIVITY_SLEEP };

enum zmk_activity_state zmk_activity_get_state(void);

uint32_t zmk_activity_get_idle_timeout_ms(void);
uint32_t zmk_activity_get_sleep_timeout_ms(void);
int zmk_activity_set_timeouts(uint32_t idle_ms, uint32_t sleep_ms);
int zmk_activity_save_timeouts(void);
int zmk_activity_reset_timeouts(void);
