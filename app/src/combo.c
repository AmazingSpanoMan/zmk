/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_combos

#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/dlist.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#if IS_ENABLED(CONFIG_SETTINGS)
#include <zephyr/settings/settings.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <drivers/behavior.h>

#include <zmk/behavior.h>
#include <zmk/combo.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/hid.h>
#include <zmk/matrix.h>
#include <zmk/keymap.h>
#include <zmk/virtual_key_position.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) || (CONFIG_ZMK_COMBO_RUNTIME_SLOTS > 0)

#if CONFIG_ZMK_COMBO_MAX_KEYS_PER_COMBO > 0

#warning                                                                                           \
    "CONFIG_ZMK_COMBO_MAX_KEYS_PER_COMBO is deprecated, and is auto-calculated from the devicetree now."

#endif

#if CONFIG_ZMK_COMBO_MAX_COMBOS_PER_KEY > 0

#warning "CONFIG_ZMK_COMBO_MAX_COMBOS_PER_KEY is deprecated, and is auto-calculated."

#endif

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
#define COMBOS_KEYS_BYTE_ARRAY(node_id)                                                            \
    uint8_t _CONCAT(combo_prop_, node_id)[DT_PROP_LEN(node_id, key_positions)];

#define DT_MAX_COMBO_KEYS sizeof(union {DT_INST_FOREACH_CHILD(0, COMBOS_KEYS_BYTE_ARRAY)})
#define COMBO_ONE(n) +1
#define COMBO_CHILDREN_COUNT (0 DT_INST_FOREACH_CHILD(0, COMBO_ONE))
#else
#define DT_MAX_COMBO_KEYS 0
#define COMBO_CHILDREN_COUNT 0
#endif

#if CONFIG_ZMK_COMBO_RUNTIME_SLOTS > 0
#define MAX_COMBO_KEYS MAX(DT_MAX_COMBO_KEYS, ZMK_COMBO_RUNTIME_MAX_KEYS)
#define TOTAL_COMBOS MAX(COMBO_CHILDREN_COUNT, CONFIG_ZMK_COMBO_RUNTIME_SLOTS)
#else
#define MAX_COMBO_KEYS DT_MAX_COMBO_KEYS
#define TOTAL_COMBOS COMBO_CHILDREN_COUNT
#endif

struct combo_cfg {
    int32_t key_positions[MAX_COMBO_KEYS];
    int16_t key_position_len;
    int16_t require_prior_idle_ms;
    int32_t timeout_ms;
    uint32_t layer_mask;
    struct zmk_behavior_binding behavior;
    // if slow release is set, the combo releases when the last key is released.
    // otherwise, the combo releases when the first key is released.
    bool slow_release;
};

struct active_combo {
    uint16_t combo_idx;
    // key_positions_pressed is filled with key_positions when the combo is pressed.
    // The keys are removed from this array when they are released.
    // Once this array is empty, the behavior is released.
    uint16_t key_positions_pressed_count;
    struct zmk_position_state_changed_event key_positions_pressed[MAX_COMBO_KEYS];
};

#define PROP_BIT_AT_IDX(n, prop, idx) BIT(DT_PROP_BY_IDX(n, prop, idx))

#define NODE_PROP_BITMASK(n, prop)                                                                 \
    COND_CODE_1(DT_NODE_HAS_PROP(n, prop),                                                         \
                (DT_FOREACH_PROP_ELEM_SEP(n, prop, PROP_BIT_AT_IDX, (|))), (0))

#define GET_KEY_POSITION_MASK_PORTION(idx, n) ((NODE_PROP_BITMASK(n, key_positions) >> idx) & 0xFF)

#define COMBO_INST(n, positions)                                                                   \
    COND_CODE_1(IS_EQ(DT_PROP_LEN(n, key_positions), positions),                                   \
                (                                                                                  \
                    {                                                                              \
                        .timeout_ms = DT_PROP(n, timeout_ms),                                      \
                        .require_prior_idle_ms = DT_PROP(n, require_prior_idle_ms),                \
                        .key_positions = DT_PROP(n, key_positions),                                \
                        .key_position_len = DT_PROP_LEN(n, key_positions),                         \
                        .behavior = ZMK_KEYMAP_EXTRACT_BINDING(0, n),                              \
                        .slow_release = DT_PROP(n, slow_release),                                  \
                        .layer_mask = NODE_PROP_BITMASK(n, layers),                                \
                    }, ),                                                                          \
                ())

#define COMBO_CONFIGS_WITH_MATCHING_POSITIONS_LEN(positions, _ignore)                              \
    DT_INST_FOREACH_CHILD_VARGS(0, COMBO_INST, positions)

#if COMBO_CHILDREN_COUNT > 0
static const struct combo_cfg dt_combos[COMBO_CHILDREN_COUNT] = {
    LISTIFY(20, COMBO_CONFIGS_WITH_MATCHING_POSITIONS_LEN, (), 0)};
#endif

#if CONFIG_ZMK_COMBO_RUNTIME_SLOTS > 0
struct combo_slot_entry {
    bool enabled;
    bool custom;
    bool dirty;
    uint16_t pending_local_id;
    struct combo_cfg cfg;
};

static struct combo_slot_entry user_combos[TOTAL_COMBOS];
static struct combo_cfg combos[TOTAL_COMBOS];
static size_t active_combos_len = 0;
#elif COMBO_CHILDREN_COUNT > 0
static const struct combo_cfg *combos = dt_combos;
static const size_t active_combos_len = COMBO_CHILDREN_COUNT;
#else
static const size_t active_combos_len = 0;
#endif

// We need at least 4 bytes to avoid alignment issues
#define BYTES_FOR_COMBOS_MASK MAX(1, DIV_ROUND_UP(TOTAL_COMBOS, 32))

static K_MUTEX_DEFINE(combo_mutex);

uint8_t pressed_keys_count = 0;
// set of keys pressed
struct zmk_position_state_changed_event pressed_keys[MAX_COMBO_KEYS] = {};
// the set of candidate combos based on the currently pressed_keys
uint32_t candidates[BYTES_FOR_COMBOS_MASK];
// the last candidate that was completely pressed
int16_t fully_pressed_combo = INT16_MAX;
// a lookup dict that maps a key position to all combos on that position
uint32_t combo_lookup[ZMK_KEYMAP_LEN][BYTES_FOR_COMBOS_MASK] = {};
// combos that have been activated and still have (some) keys pressed
// this array is always contiguous from 0.
struct active_combo active_combos[CONFIG_ZMK_COMBO_MAX_PRESSED_COMBOS] = {};
uint8_t active_combo_count = 0;

struct k_work_delayable timeout_task;
int64_t timeout_task_timeout_at;

// this keeps track of the last non-combo, non-mod key tap
int64_t last_tapped_timestamp = INT32_MIN;
// this keeps track of the last time a combo was pressed
int64_t last_combo_timestamp = INT32_MIN;

static void store_last_tapped(int64_t timestamp) {
    if (timestamp > last_combo_timestamp) {
        last_tapped_timestamp = timestamp;
    }
}

// Store the combo key pointer in the combos array, one pointer for each key position
// The combos are sorted shortest-first, then by virtual-key-position.
static int initialize_combo(size_t index) {
    const struct combo_cfg *new_combo = &combos[index];

    for (size_t kp = 0; kp < (size_t)new_combo->key_position_len; kp++) {
        int32_t pos = new_combo->key_positions[kp];
        if (pos >= 0 && pos < ZMK_KEYMAP_LEN) {
            sys_bitfield_set_bit((mem_addr_t)&combo_lookup[pos], index);
        }
    }

    return 0;
}

static bool combo_active_on_layer(const struct combo_cfg *combo, uint8_t layer) {
    if (!combo->layer_mask) {
        return true;
    }

    return combo->layer_mask & BIT(layer);
}

static bool is_quick_tap(const struct combo_cfg *combo, int64_t timestamp) {
    return (last_tapped_timestamp + combo->require_prior_idle_ms) > timestamp;
}

static int setup_candidates_for_first_keypress(int32_t position, int64_t timestamp) {
    int number_of_combo_candidates = 0;
    uint8_t highest_active_layer = zmk_keymap_highest_layer_active();

    for (size_t i = 0; i < active_combos_len; i++) {
        if (sys_bitfield_test_bit((mem_addr_t)&combo_lookup[position], i)) {
            const struct combo_cfg *combo = &combos[i];
            if (combo_active_on_layer(combo, highest_active_layer) &&
                !is_quick_tap(combo, timestamp)) {
                sys_bitfield_set_bit((mem_addr_t)&candidates, i);
                number_of_combo_candidates++;
            }
            // LOG_DBG("combo timeout %d %d %d", position, i, candidates[i].timeout_at);
        }
    }

    return number_of_combo_candidates;
}

static inline uint8_t zero_one_or_more_bits(uint32_t field) {
    if (field == 0) {
        return 0;
    }
    if ((field & (field - 1)) == 0) {
        return 1;
    }
    return 2;
}

static int filter_candidates(int32_t position) {
    int matches = 0;
    for (int i = 0; i < BYTES_FOR_COMBOS_MASK; i++) {
        candidates[i] &= combo_lookup[position][i];
        if (matches < 2) {
            matches += zero_one_or_more_bits(candidates[i]);
        }
    }

    LOG_DBG("combo matches after filter %d", matches);
    return matches;
}

static int64_t first_candidate_timeout() {
    if (pressed_keys_count == 0) {
        return LONG_MAX;
    }

    int64_t first_timeout = LONG_MAX;
    for (int i = 0; i < (int)active_combos_len; i++) {
        if (sys_bitfield_test_bit((mem_addr_t)&candidates, i)) {
            first_timeout = MIN(first_timeout, combos[i].timeout_ms);
        }
    }

    return pressed_keys[0].data.timestamp + first_timeout;
}

static inline bool candidate_is_completely_pressed(const struct combo_cfg *candidate) {
    // this code assumes set(pressed_keys) <= set(candidate->key_positions)
    // this invariant is enforced by filter_candidates
    // since events may have been reraised after clearing one or more slots at
    // the start of pressed_keys (see: release_pressed_keys), we have to check
    // that each key needed to trigger the combo was pressed, not just the last.
    return candidate->key_position_len == pressed_keys_count;
}

static int cleanup();

static int filter_timed_out_candidates(int64_t timestamp) {
    __ASSERT(pressed_keys_count > 0, "Searching for a candidate timeout with no keys pressed");

    int remaining_candidates = 0;
    for (int i = 0; i < (int)active_combos_len; i++) {
        if (sys_bitfield_test_bit((mem_addr_t)&candidates, i)) {

            if (pressed_keys[0].data.timestamp + combos[i].timeout_ms > timestamp) {
                remaining_candidates++;
            } else {
                sys_bitfield_clear_bit((mem_addr_t)&candidates, i);
            }
        }
    }

    LOG_DBG(
        "after filtering out timed out combo candidates: remaining_candidates=%d timestamp=%lld",
        remaining_candidates, timestamp);

    return remaining_candidates;
}

static int capture_pressed_key(const struct zmk_position_state_changed *ev) {
    if (pressed_keys_count == MAX_COMBO_KEYS) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    pressed_keys[pressed_keys_count++] = copy_raised_zmk_position_state_changed(ev);
    return ZMK_EV_EVENT_CAPTURED;
}

const struct zmk_listener zmk_listener_combo;

static int release_pressed_keys() {
    uint8_t count = pressed_keys_count;
    pressed_keys_count = 0;
    for (int i = 0; i < count; i++) {
        struct zmk_position_state_changed_event *ev = &pressed_keys[i];
        if (i == 0) {
            LOG_DBG("combo: releasing position event %d", ev->data.position);
            ZMK_EVENT_RELEASE(*ev);
        } else {
            // reprocess events (see tests/combo/fully-overlapping-combos-3 for why this is needed)
            LOG_DBG("combo: reraising position event %d", ev->data.position);
            ZMK_EVENT_RAISE(*ev);
        }
    }

    return count;
}

static inline int press_combo_behavior(int combo_idx, const struct combo_cfg *combo,
                                       int32_t timestamp) {
    struct zmk_behavior_binding_event event = {
        .position = ZMK_VIRTUAL_KEY_POSITION_COMBO(combo_idx),
        .timestamp = timestamp,
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
    };

    last_combo_timestamp = timestamp;

    return zmk_behavior_invoke_binding(&combo->behavior, event, true);
}

static inline int release_combo_behavior(int combo_idx, const struct combo_cfg *combo,
                                         int32_t timestamp) {
    struct zmk_behavior_binding_event event = {
        .position = ZMK_VIRTUAL_KEY_POSITION_COMBO(combo_idx),
        .timestamp = timestamp,
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
    };

    return zmk_behavior_invoke_binding(&combo->behavior, event, false);
}

static void move_pressed_keys_to_active_combo(struct active_combo *active_combo) {

    int combo_length = MIN(pressed_keys_count, combos[active_combo->combo_idx].key_position_len);
    for (int i = 0; i < combo_length; i++) {
        active_combo->key_positions_pressed[i] = pressed_keys[i];
    }
    active_combo->key_positions_pressed_count = combo_length;

    // move any other pressed keys up
    for (int i = 0; i + combo_length < pressed_keys_count; i++) {
        pressed_keys[i] = pressed_keys[i + combo_length];
    }

    pressed_keys_count -= combo_length;
}

static struct active_combo *store_active_combo(int32_t combo_idx) {
    for (int i = 0; i < CONFIG_ZMK_COMBO_MAX_PRESSED_COMBOS; i++) {
        if (active_combos[i].combo_idx == UINT16_MAX) {
            active_combos[i].combo_idx = combo_idx;
            active_combo_count++;
            return &active_combos[i];
        }
    }
    LOG_ERR("Unable to store combo; already %d active. Increase "
            "CONFIG_ZMK_COMBO_MAX_PRESSED_COMBOS",
            CONFIG_ZMK_COMBO_MAX_PRESSED_COMBOS);
    return NULL;
}

static void activate_combo(int combo_idx) {
    struct active_combo *active_combo = store_active_combo(combo_idx);
    if (active_combo == NULL) {
        // unable to store combo
        release_pressed_keys();
        return;
    }
    move_pressed_keys_to_active_combo(active_combo);
    press_combo_behavior(combo_idx, &combos[combo_idx],
                         active_combo->key_positions_pressed[0].data.timestamp);
}

static void deactivate_combo(int active_combo_index) {
    active_combo_count--;
    if (active_combo_index != active_combo_count) {
        memcpy(&active_combos[active_combo_index], &active_combos[active_combo_count],
               sizeof(struct active_combo));
    }
    active_combos[active_combo_count] = (struct active_combo){0};
    active_combos[active_combo_count].combo_idx = UINT16_MAX;
}

/* returns true if a key was released. */
static bool release_combo_key(int32_t position, int64_t timestamp) {
    for (int combo_idx = 0; combo_idx < active_combo_count; combo_idx++) {
        struct active_combo *active_combo = &active_combos[combo_idx];

        bool key_released = false;
        bool all_keys_pressed = active_combo->key_positions_pressed_count ==
                                combos[active_combo->combo_idx].key_position_len;
        bool all_keys_released = true;
        for (int i = 0; i < active_combo->key_positions_pressed_count; i++) {
            if (key_released) {
                active_combo->key_positions_pressed[i - 1] = active_combo->key_positions_pressed[i];
                all_keys_released = false;
            } else if (active_combo->key_positions_pressed[i].data.position != position) {
                all_keys_released = false;
            } else { // position matches
                key_released = true;
            }
        }

        if (key_released) {
            active_combo->key_positions_pressed_count--;
            const struct combo_cfg *c = &combos[active_combo->combo_idx];
            if ((c->slow_release && all_keys_released) || (!c->slow_release && all_keys_pressed)) {
                release_combo_behavior(active_combo->combo_idx, c, timestamp);
            }
            if (all_keys_released) {
                deactivate_combo(combo_idx);
            }
            return true;
        }
    }
    return false;
}

static int cleanup() {
    k_work_cancel_delayable(&timeout_task);
    memset(candidates, 0, BYTES_FOR_COMBOS_MASK * sizeof(uint32_t));
    if (fully_pressed_combo != INT16_MAX) {
        activate_combo(fully_pressed_combo);
        fully_pressed_combo = INT16_MAX;
    }
    return release_pressed_keys();
}

static void update_timeout_task() {
    int64_t first_timeout = first_candidate_timeout();
    if (timeout_task_timeout_at == first_timeout) {
        return;
    }
    if (first_timeout == LLONG_MAX) {
        timeout_task_timeout_at = 0;
        k_work_cancel_delayable(&timeout_task);
        return;
    }
    if (k_work_schedule(&timeout_task, K_MSEC(first_timeout - k_uptime_get())) >= 0) {
        timeout_task_timeout_at = first_timeout;
    }
}

static int position_state_down(const zmk_event_t *ev, struct zmk_position_state_changed *data) {
    int num_candidates;
    if (!pressed_keys_count) {
        num_candidates = setup_candidates_for_first_keypress(data->position, data->timestamp);
        if (num_candidates == 0) {
            return ZMK_EV_EVENT_BUBBLE;
        }
    } else {
        filter_timed_out_candidates(data->timestamp);
        num_candidates = filter_candidates(data->position);
    }

    LOG_DBG("combo: capturing position event %d", data->position);
    int ret = capture_pressed_key(data);
    update_timeout_task();

    if (num_candidates) {
        for (int i = 0; i < (int)active_combos_len; i++) {
            if (sys_bitfield_test_bit((mem_addr_t)&candidates, i)) {
                const struct combo_cfg *candidate_combo = &combos[i];
                if (candidate_is_completely_pressed(candidate_combo)) {
                    fully_pressed_combo = i;
                    if (num_candidates == 1) {
                        cleanup();
                    }
                }

                return ret;
            }
        }
    } else {
        cleanup();
        return ret;
    }

    return -EINVAL;
}

static int position_state_up(const zmk_event_t *ev, struct zmk_position_state_changed *data) {
    int released_keys = cleanup();
    if (release_combo_key(data->position, data->timestamp)) {
        return ZMK_EV_EVENT_HANDLED;
    }
    if (released_keys > 1) {
        // The second and further key down events are re-raised. To preserve
        // correct order for e.g. hold-taps, reraise the key up event too.
        struct zmk_position_state_changed_event dupe_ev =
            copy_raised_zmk_position_state_changed(data);
        ZMK_EVENT_RAISE(dupe_ev);
        return ZMK_EV_EVENT_CAPTURED;
    }
    return ZMK_EV_EVENT_BUBBLE;
}

static void combo_timeout_handler(struct k_work *item) {
    k_mutex_lock(&combo_mutex, K_FOREVER);
    if (timeout_task_timeout_at == 0 || k_uptime_get() < timeout_task_timeout_at) {
        // timer was cancelled or rescheduled.
        k_mutex_unlock(&combo_mutex);
        return;
    }
    if (filter_timed_out_candidates(timeout_task_timeout_at) == 0) {
        LOG_DBG("CLEANUP!");
        cleanup();
    }

    LOG_DBG("ABOUT TO UPDATE IN TIMEOUT");
    update_timeout_task();
    k_mutex_unlock(&combo_mutex);
}

static int position_state_changed_listener(const zmk_event_t *ev) {
    struct zmk_position_state_changed *data = as_zmk_position_state_changed(ev);
    if (data == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (data->state) { // keydown
        return position_state_down(ev, data);
    } else { // keyup
        return position_state_up(ev, data);
    }
}

static int keycode_state_changed_listener(const zmk_event_t *eh) {
    struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
    if (ev->state && !is_mod(ev->usage_page, ev->keycode)) {
        store_last_tapped(ev->timestamp);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

int behavior_combo_listener(const zmk_event_t *eh) {
    k_mutex_lock(&combo_mutex, K_FOREVER);
    int ret = ZMK_EV_EVENT_BUBBLE;
    if (as_zmk_position_state_changed(eh) != NULL) {
        ret = position_state_changed_listener(eh);
    } else if (as_zmk_keycode_state_changed(eh) != NULL) {
        ret = keycode_state_changed_listener(eh);
    }
    k_mutex_unlock(&combo_mutex);
    return ret;
}

ZMK_LISTENER(combo, behavior_combo_listener);
ZMK_SUBSCRIPTION(combo, zmk_position_state_changed);
ZMK_SUBSCRIPTION(combo, zmk_keycode_state_changed);

#if CONFIG_ZMK_COMBO_RUNTIME_SLOTS > 0
static void reset_active_combo_state_locked(void) {
    k_work_cancel_delayable(&timeout_task);
    timeout_task_timeout_at = 0;
    memset(candidates, 0, BYTES_FOR_COMBOS_MASK * sizeof(uint32_t));
    pressed_keys_count = 0;
    fully_pressed_combo = INT16_MAX;

    int64_t ts = k_uptime_get();
    for (int i = 0; i < (int)active_combo_count; i++) {
        uint16_t idx = active_combos[i].combo_idx;
        if (idx < active_combos_len) {
            release_combo_behavior(idx, &combos[idx], (int32_t)ts);
        }
    }
    for (size_t i = 0; i < CONFIG_ZMK_COMBO_MAX_PRESSED_COMBOS; i++) {
        active_combos[i] = (struct active_combo){0};
        active_combos[i].combo_idx = UINT16_MAX;
    }
    active_combo_count = 0;
}

static void rebuild_combos_locked(void) {
    memset(combo_lookup, 0, sizeof(combo_lookup));
    memset(combos, 0, sizeof(combos));
    active_combos_len = 0;

    /* Insert shortest-first so overlapping combos resolve identically to DTS build order */
    for (int len = 2; len <= MAX_COMBO_KEYS; len++) {
        for (size_t s = 0; s < TOTAL_COMBOS; s++) {
            const struct combo_slot_entry *entry = &user_combos[s];
            if (!entry->enabled || entry->cfg.key_position_len != len ||
                !entry->cfg.behavior.behavior_dev) {
                continue;
            }
            combos[active_combos_len] = entry->cfg;
            initialize_combo(active_combos_len);
            active_combos_len++;
        }
    }
}

static void populate_dt_defaults_locked(void) {
    for (size_t s = 0; s < TOTAL_COMBOS; s++) {
        user_combos[s] = (struct combo_slot_entry){0};
        user_combos[s].pending_local_id = UINT16_MAX;
        user_combos[s].cfg.timeout_ms = 50;
#if COMBO_CHILDREN_COUNT > 0
        if (s < COMBO_CHILDREN_COUNT) {
            user_combos[s].enabled = true;
            user_combos[s].cfg = dt_combos[s];
        }
#endif
    }
}

size_t zmk_combo_runtime_slot_count(void) { return TOTAL_COMBOS; }

int zmk_combo_get_slot(size_t slot, struct zmk_combo_runtime_slot *out) {
    if (slot >= TOTAL_COMBOS || !out) {
        return -EINVAL;
    }

    k_mutex_lock(&combo_mutex, K_FOREVER);
    const struct combo_slot_entry *e = &user_combos[slot];
    *out = (struct zmk_combo_runtime_slot){
        .enabled = e->enabled,
        .timeout_ms = e->cfg.timeout_ms,
        .require_prior_idle_ms = e->cfg.require_prior_idle_ms,
        .layer_mask = e->cfg.layer_mask,
        .slow_release = e->cfg.slow_release,
        .key_position_len = MIN(e->cfg.key_position_len, (int16_t)ZMK_COMBO_RUNTIME_MAX_KEYS),
        .behavior = e->cfg.behavior,
    };
    for (int i = 0; i < out->key_position_len; i++) {
        out->key_positions[i] = e->cfg.key_positions[i];
    }
    k_mutex_unlock(&combo_mutex);

    return 0;
}

int zmk_combo_set_slot(size_t slot, const struct zmk_combo_runtime_slot *in) {
    if (slot >= TOTAL_COMBOS || !in) {
        return -EINVAL;
    }

    if (in->enabled) {
        if (in->key_position_len < 2 || in->key_position_len > ZMK_COMBO_RUNTIME_MAX_KEYS ||
            in->timeout_ms <= 0 || !in->behavior.behavior_dev) {
            return -EINVAL;
        }
        for (int i = 0; i < in->key_position_len; i++) {
            if (in->key_positions[i] < 0 || in->key_positions[i] >= ZMK_KEYMAP_LEN) {
                return -EINVAL;
            }
            for (int j = i + 1; j < in->key_position_len; j++) {
                if (in->key_positions[i] == in->key_positions[j]) {
                    return -EINVAL;
                }
            }
        }
        if (zmk_behavior_validate_binding(&in->behavior) < 0) {
            return -EINVAL;
        }
    }

    k_mutex_lock(&combo_mutex, K_FOREVER);
    reset_active_combo_state_locked();

    struct combo_slot_entry *e = &user_combos[slot];
    e->enabled = in->enabled;
    e->custom = true;
    e->dirty = true;
    e->pending_local_id = UINT16_MAX;
    memset(&e->cfg, 0, sizeof(e->cfg));
    e->cfg.timeout_ms = in->timeout_ms > 0 ? in->timeout_ms : 50;
    e->cfg.require_prior_idle_ms = MAX(in->require_prior_idle_ms, (int16_t)0);
    e->cfg.layer_mask = in->layer_mask;
    e->cfg.slow_release = in->slow_release;
    e->cfg.key_position_len =
        CLAMP(in->key_position_len, (int16_t)0, (int16_t)ZMK_COMBO_RUNTIME_MAX_KEYS);
    for (int i = 0; i < e->cfg.key_position_len; i++) {
        e->cfg.key_positions[i] = in->key_positions[i];
    }
    e->cfg.behavior = in->behavior;

    rebuild_combos_locked();
    k_mutex_unlock(&combo_mutex);

    return 0;
}

#if IS_ENABLED(CONFIG_SETTINGS)
#define COMBO_NVS_MAGIC 0x47434D31U

struct combo_nvs_rec {
    uint32_t magic;
    uint32_t layer_mask;
    int32_t timeout_ms;
    int16_t require_prior_idle_ms;
    int16_t key_position_len;
    int32_t key_positions[ZMK_COMBO_RUNTIME_MAX_KEYS];
    uint16_t behavior_local_id;
    uint8_t enabled;
    uint8_t slow_release;
    uint32_t param1;
    uint32_t param2;
};

int zmk_combo_save_settings(void) {
    int err = 0;
    for (size_t s = 0; s < TOTAL_COMBOS; s++) {
        k_mutex_lock(&combo_mutex, K_FOREVER);
        bool save = user_combos[s].dirty && user_combos[s].custom;
        struct combo_slot_entry copy = user_combos[s];
        k_mutex_unlock(&combo_mutex);

        if (!save) {
            continue;
        }

        char key[24];
        snprintf(key, sizeof(key), "combo/slot/%u", (unsigned int)s);

        struct combo_nvs_rec rec = {
            .magic = COMBO_NVS_MAGIC,
            .layer_mask = copy.cfg.layer_mask,
            .timeout_ms = copy.cfg.timeout_ms,
            .require_prior_idle_ms = copy.cfg.require_prior_idle_ms,
            .key_position_len = copy.cfg.key_position_len,
            .behavior_local_id = copy.cfg.behavior.behavior_dev
                ? zmk_behavior_get_local_id(copy.cfg.behavior.behavior_dev)
                : UINT16_MAX,
            .enabled = copy.enabled ? 1U : 0U,
            .slow_release = copy.cfg.slow_release ? 1U : 0U,
            .param1 = copy.cfg.behavior.param1,
            .param2 = copy.cfg.behavior.param2,
        };
        for (int i = 0; i < ZMK_COMBO_RUNTIME_MAX_KEYS; i++) {
            rec.key_positions[i] = copy.cfg.key_positions[i];
        }

        int r = settings_save_one(key, &rec, sizeof(rec));
        if (r == 0) {
            k_mutex_lock(&combo_mutex, K_FOREVER);
            user_combos[s].dirty = false;
            k_mutex_unlock(&combo_mutex);
        } else if (err == 0) {
            err = r;
        }
    }
    return err;
}

int zmk_combo_reset_settings(void) {
    int err = 0;
    for (size_t s = 0; s < TOTAL_COMBOS; s++) {
        char key[24];
        snprintf(key, sizeof(key), "combo/slot/%u", (unsigned int)s);
        int r = settings_delete(key);
        if (r < 0 && r != -ENOENT && err == 0) {
            err = r;
        }
    }

    k_mutex_lock(&combo_mutex, K_FOREVER);
    reset_active_combo_state_locked();
    populate_dt_defaults_locked();
    rebuild_combos_locked();
    k_mutex_unlock(&combo_mutex);

    return err;
}

static int combo_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                              void *cb_arg) {
    const char *next;
    if (!name || !settings_name_steq(name, "slot", &next) || !next) {
        return -ENOENT;
    }

    char *endptr = NULL;
    unsigned long slot = strtoul(next, &endptr, 10);
    if (endptr == next || *endptr != '\0' || slot >= TOTAL_COMBOS) {
        return -ENOENT;
    }

    struct combo_nvs_rec rec;
    if (len != sizeof(rec)) {
        return -EINVAL;
    }

    int rc = read_cb(cb_arg, &rec, sizeof(rec));
    if (rc < 0) {
        return rc;
    }
    if (rec.magic != COMBO_NVS_MAGIC) {
        return -EINVAL;
    }

    k_mutex_lock(&combo_mutex, K_FOREVER);
    struct combo_slot_entry *e = &user_combos[slot];
    e->enabled = rec.enabled != 0U;
    e->custom = true;
    e->dirty = false;
    e->pending_local_id = rec.behavior_local_id;
    memset(&e->cfg, 0, sizeof(e->cfg));
    e->cfg.timeout_ms = rec.timeout_ms;
    e->cfg.require_prior_idle_ms = rec.require_prior_idle_ms;
    e->cfg.layer_mask = rec.layer_mask;
    e->cfg.slow_release = rec.slow_release != 0U;
    e->cfg.key_position_len =
        CLAMP(rec.key_position_len, (int16_t)0, (int16_t)ZMK_COMBO_RUNTIME_MAX_KEYS);
    for (int i = 0; i < e->cfg.key_position_len; i++) {
        e->cfg.key_positions[i] = rec.key_positions[i];
    }
    e->cfg.behavior.behavior_dev = NULL;
    e->cfg.behavior.param1 = rec.param1;
    e->cfg.behavior.param2 = rec.param2;
    k_mutex_unlock(&combo_mutex);

    return 0;
}

static int combo_settings_commit(void) {
    k_mutex_lock(&combo_mutex, K_FOREVER);
    for (size_t s = 0; s < TOTAL_COMBOS; s++) {
        struct combo_slot_entry *e = &user_combos[s];
        if (e->custom && e->pending_local_id != UINT16_MAX && !e->cfg.behavior.behavior_dev) {
            e->cfg.behavior.behavior_dev =
                zmk_behavior_find_behavior_name_from_local_id(e->pending_local_id);
        }
    }
    rebuild_combos_locked();
    k_mutex_unlock(&combo_mutex);
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(combo, "combo", NULL, combo_settings_set, combo_settings_commit, NULL);
#else
int zmk_combo_save_settings(void) { return 0; }
int zmk_combo_reset_settings(void) {
    k_mutex_lock(&combo_mutex, K_FOREVER);
    reset_active_combo_state_locked();
    populate_dt_defaults_locked();
    rebuild_combos_locked();
    k_mutex_unlock(&combo_mutex);
    return 0;
}
#endif
#else
size_t zmk_combo_runtime_slot_count(void) { return COMBO_CHILDREN_COUNT; }
int zmk_combo_get_slot(size_t slot, struct zmk_combo_runtime_slot *out) {
    ARG_UNUSED(slot);
    ARG_UNUSED(out);
    return -ENOTSUP;
}
int zmk_combo_set_slot(size_t slot, const struct zmk_combo_runtime_slot *in) {
    ARG_UNUSED(slot);
    ARG_UNUSED(in);
    return -ENOTSUP;
}
int zmk_combo_save_settings(void) { return 0; }
int zmk_combo_reset_settings(void) { return 0; }
#endif

static int combo_init(void) {
    for (size_t i = 0; i < CONFIG_ZMK_COMBO_MAX_PRESSED_COMBOS; i++) {
        active_combos[i].combo_idx = UINT16_MAX;
    }

    k_work_init_delayable(&timeout_task, combo_timeout_handler);
#if CONFIG_ZMK_COMBO_RUNTIME_SLOTS > 0
    k_mutex_lock(&combo_mutex, K_FOREVER);
    populate_dt_defaults_locked();
    rebuild_combos_locked();
    k_mutex_unlock(&combo_mutex);
    LOG_WRN("Combo runtime enabled: %d slots (%d active)", TOTAL_COMBOS, (int)active_combos_len);
#else
    LOG_WRN("Have %d combos!", (int)active_combos_len);
    for (int i = 0; i < (int)active_combos_len; i++) {
        initialize_combo(i);
    }
#endif
    return 0;
}

SYS_INIT(combo_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

#endif
