/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>

#define ZMK_COMBOS_UTIL_ONE(n) +1

#define ZMK_DT_COMBOS_LEN                                                                          \
    COND_CODE_1(DT_HAS_COMPAT_STATUS_OKAY(zmk_combos),                                             \
                (0 DT_FOREACH_CHILD_STATUS_OKAY(DT_INST(0, zmk_combos), ZMK_COMBOS_UTIL_ONE)),     \
                (0))

#if CONFIG_ZMK_COMBO_RUNTIME_SLOTS > 0
#define ZMK_COMBOS_LEN MAX(ZMK_DT_COMBOS_LEN, CONFIG_ZMK_COMBO_RUNTIME_SLOTS)
#else
#define ZMK_COMBOS_LEN ZMK_DT_COMBOS_LEN
#endif
