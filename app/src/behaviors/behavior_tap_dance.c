/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_tap_dance

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>
#if IS_ENABLED(CONFIG_SETTINGS)
#include <zephyr/settings/settings.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zmk/behavior.h>
#include <zmk/gremlin_behaviors.h>
#include <zmk/keymap.h>
#include <zmk/matrix.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/hid.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define ZMK_BHV_TAP_DANCE_MAX_HELD CONFIG_ZMK_BEHAVIOR_TAP_DANCE_MAX_HELD

#define ZMK_BHV_TAP_DANCE_POSITION_FREE UINT32_MAX

struct behavior_tap_dance_config {
    uint32_t tapping_term_ms;
    size_t behavior_count;
    struct zmk_behavior_binding *behaviors;
    int gremlin_slot;
    bool enabled;
};

struct active_tap_dance {
    // Tap Dance Data
    int counter;
    uint32_t position;
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
    uint8_t source;
#endif
    uint32_t param1;
    uint32_t param2;
    bool is_pressed;
    const struct behavior_tap_dance_config *config;

    // Timer Data
    bool timer_started;
    bool timer_cancelled;
    bool tap_dance_decided;
    int64_t release_at;
    struct k_work_delayable release_timer;
};

struct active_tap_dance active_tap_dances[ZMK_BHV_TAP_DANCE_MAX_HELD] = {};

static struct active_tap_dance *find_tap_dance(uint32_t position) {
    for (int i = 0; i < ZMK_BHV_TAP_DANCE_MAX_HELD; i++) {
        if (active_tap_dances[i].position == position && !active_tap_dances[i].timer_cancelled) {
            return &active_tap_dances[i];
        }
    }
    return NULL;
}

static int new_tap_dance(struct zmk_behavior_binding_event *event,
                         const struct behavior_tap_dance_config *config,
                         struct active_tap_dance **tap_dance) {
    for (int i = 0; i < ZMK_BHV_TAP_DANCE_MAX_HELD; i++) {
        struct active_tap_dance *const ref_dance = &active_tap_dances[i];
        if (ref_dance->position == ZMK_BHV_TAP_DANCE_POSITION_FREE) {
            ref_dance->counter = 0;
            ref_dance->position = event->position;
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
            ref_dance->source = event->source;
#endif
            ref_dance->config = config;
            ref_dance->release_at = 0;
            ref_dance->is_pressed = true;
            ref_dance->timer_started = true;
            ref_dance->timer_cancelled = false;
            ref_dance->tap_dance_decided = false;
            *tap_dance = ref_dance;
            return 0;
        }
    }
    return -ENOMEM;
}

static void clear_tap_dance(struct active_tap_dance *tap_dance) {
    tap_dance->position = ZMK_BHV_TAP_DANCE_POSITION_FREE;
}

static int stop_timer(struct active_tap_dance *tap_dance) {
    int timer_cancel_result = k_work_cancel_delayable(&tap_dance->release_timer);
    if (timer_cancel_result == -EINPROGRESS) {
        // too late to cancel, we'll let the timer handler clear up.
        tap_dance->timer_cancelled = true;
    }
    return timer_cancel_result;
}

static void reset_timer(struct active_tap_dance *tap_dance,
                        struct zmk_behavior_binding_event event) {
    tap_dance->release_at = event.timestamp + tap_dance->config->tapping_term_ms;
    int32_t ms_left = tap_dance->release_at - k_uptime_get();
    if (ms_left > 0) {
        k_work_schedule(&tap_dance->release_timer, K_MSEC(ms_left));
        LOG_DBG("Successfully reset timer at position %d", tap_dance->position);
    }
}

static inline int press_tap_dance_behavior(struct active_tap_dance *tap_dance, int64_t timestamp) {
    tap_dance->tap_dance_decided = true;
    struct zmk_behavior_binding binding = tap_dance->config->behaviors[tap_dance->counter - 1];
    struct zmk_behavior_binding_event event = {
        .position = tap_dance->position,
        .timestamp = timestamp,
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = tap_dance->source,
#endif
    };
    return zmk_behavior_invoke_binding(&binding, event, true);
}

static inline int release_tap_dance_behavior(struct active_tap_dance *tap_dance,
                                             int64_t timestamp) {
    struct zmk_behavior_binding binding = tap_dance->config->behaviors[tap_dance->counter - 1];
    struct zmk_behavior_binding_event event = {
        .position = tap_dance->position,
        .timestamp = timestamp,
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = tap_dance->source,
#endif
    };
    clear_tap_dance(tap_dance);
    return zmk_behavior_invoke_binding(&binding, event, false);
}

static int on_tap_dance_binding_pressed(struct zmk_behavior_binding *binding,
                                        struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_tap_dance_config *cfg = dev->config;
    if (!cfg->enabled || cfg->behavior_count == 0) {
        return ZMK_BEHAVIOR_OPAQUE;
    }
    struct active_tap_dance *tap_dance;
    tap_dance = find_tap_dance(event.position);
    if (tap_dance == NULL) {
        if (new_tap_dance(&event, cfg, &tap_dance) == -ENOMEM) {
            LOG_ERR("Unable to create new tap dance. Insufficient space in active_tap_dances[].");
            return ZMK_BEHAVIOR_OPAQUE;
        }
        LOG_DBG("%d created new tap dance", event.position);
    }
    tap_dance->is_pressed = true;
    LOG_DBG("%d tap dance pressed", event.position);
    stop_timer(tap_dance);
    // Increment the counter on keypress. If the counter has reached its maximum
    // value, invoke the last binding available.
    if (tap_dance->counter < cfg->behavior_count) {
        tap_dance->counter++;
    }
    if (tap_dance->counter == cfg->behavior_count) {
        // LOG_DBG("Tap dance has been decided via maximum counter value");
        press_tap_dance_behavior(tap_dance, event.timestamp);
        return ZMK_EV_EVENT_BUBBLE;
    }
    reset_timer(tap_dance, event);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_tap_dance_binding_released(struct zmk_behavior_binding *binding,
                                         struct zmk_behavior_binding_event event) {
    LOG_DBG("%d tap dance keybind released", event.position);
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_tap_dance_config *cfg = dev ? dev->config : NULL;
    struct active_tap_dance *tap_dance = find_tap_dance(event.position);
    if (tap_dance == NULL) {
        if (cfg && (!cfg->enabled || cfg->behavior_count == 0)) {
            return ZMK_BEHAVIOR_OPAQUE;
        }
        LOG_ERR("ACTIVE TAP DANCE CLEARED TOO EARLY");
        return ZMK_BEHAVIOR_OPAQUE;
    }
    tap_dance->is_pressed = false;
    if (tap_dance->tap_dance_decided) {
        release_tap_dance_behavior(tap_dance, event.timestamp);
    }
    return ZMK_BEHAVIOR_OPAQUE;
}

void behavior_tap_dance_timer_handler(struct k_work *item) {
    struct k_work_delayable *d_work = k_work_delayable_from_work(item);
    struct active_tap_dance *tap_dance =
        CONTAINER_OF(d_work, struct active_tap_dance, release_timer);
    if (tap_dance->position == ZMK_BHV_TAP_DANCE_POSITION_FREE) {
        return;
    }
    if (tap_dance->timer_cancelled) {
        return;
    }
    LOG_DBG("Tap dance has been decided via timer. Counter reached: %d", tap_dance->counter);
    press_tap_dance_behavior(tap_dance, tap_dance->release_at);
    if (tap_dance->is_pressed) {
        return;
    }
    release_tap_dance_behavior(tap_dance, tap_dance->release_at);
}

static const struct behavior_driver_api behavior_tap_dance_driver_api = {
    .binding_pressed = on_tap_dance_binding_pressed,
    .binding_released = on_tap_dance_binding_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
};

static int tap_dance_position_state_changed_listener(const zmk_event_t *eh);

ZMK_LISTENER(behavior_tap_dance, tap_dance_position_state_changed_listener);
ZMK_SUBSCRIPTION(behavior_tap_dance, zmk_position_state_changed);

static int tap_dance_position_state_changed_listener(const zmk_event_t *eh) {
    struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    if (!ev->state) {
        LOG_DBG("Ignore upstroke at position %d.", ev->position);
        return ZMK_EV_EVENT_BUBBLE;
    }
    for (int i = 0; i < ZMK_BHV_TAP_DANCE_MAX_HELD; i++) {
        struct active_tap_dance *tap_dance = &active_tap_dances[i];
        if (tap_dance->position == ZMK_BHV_TAP_DANCE_POSITION_FREE) {
            continue;
        }
        if (tap_dance->position == ev->position) {
            continue;
        }
        stop_timer(tap_dance);
        LOG_DBG("Tap dance interrupted, activating tap-dance at %d", tap_dance->position);
        if (!tap_dance->tap_dance_decided) {
            press_tap_dance_behavior(tap_dance, ev->timestamp);
            if (!tap_dance->is_pressed) {
                release_tap_dance_behavior(tap_dance, ev->timestamp);
            }
            return ZMK_EV_EVENT_BUBBLE;
        }
    }
    return ZMK_EV_EVENT_BUBBLE;
}

struct gremlin_td_reg {
    struct behavior_tap_dance_config *cfg;
    bool custom;
    bool dirty;
    uint16_t pending_local_ids[ZMK_GREMLIN_MAX_TAP_DANCE_STEPS];
};

static struct gremlin_td_reg gremlin_td_registry[ZMK_GREMLIN_MAX_TAP_DANCES];
static K_MUTEX_DEFINE(gremlin_td_mutex);

static int behavior_tap_dance_init(const struct device *dev) {
    static bool init_first_run = true;
    if (init_first_run) {
        for (int i = 0; i < ZMK_BHV_TAP_DANCE_MAX_HELD; i++) {
            k_work_init_delayable(&active_tap_dances[i].release_timer,
                                  behavior_tap_dance_timer_handler);
            clear_tap_dance(&active_tap_dances[i]);
        }
    }
    init_first_run = false;

    struct behavior_tap_dance_config *cfg = (struct behavior_tap_dance_config *)(uintptr_t)dev->config;
    if (cfg && cfg->gremlin_slot >= 0 && cfg->gremlin_slot < ZMK_GREMLIN_MAX_TAP_DANCES) {
        gremlin_td_registry[cfg->gremlin_slot].cfg = cfg;
        for (int i = 0; i < ZMK_GREMLIN_MAX_TAP_DANCE_STEPS; i++) {
            gremlin_td_registry[cfg->gremlin_slot].pending_local_ids[i] = UINT16_MAX;
        }
    }
    return 0;
}

size_t zmk_gremlin_tap_dance_count(void) {
    size_t count = 0;
    for (size_t i = 0; i < ZMK_GREMLIN_MAX_TAP_DANCES; i++) {
        if (gremlin_td_registry[i].cfg != NULL) {
            count++;
        } else {
            break;
        }
    }
    return count;
}

int zmk_gremlin_tap_dance_get(size_t slot, struct zmk_gremlin_tap_dance_slot *out) {
    if (slot >= ZMK_GREMLIN_MAX_TAP_DANCES || !out) {
        return -EINVAL;
    }

    k_mutex_lock(&gremlin_td_mutex, K_FOREVER);
    struct behavior_tap_dance_config *cfg = gremlin_td_registry[slot].cfg;
    if (!cfg) {
        k_mutex_unlock(&gremlin_td_mutex);
        return -ENODEV;
    }
    memset(out, 0, sizeof(*out));
    out->enabled = cfg->enabled;
    out->tapping_term_ms = cfg->tapping_term_ms;
    out->binding_count = (uint8_t)MIN(cfg->behavior_count, (size_t)ZMK_GREMLIN_MAX_TAP_DANCE_STEPS);
    for (uint8_t i = 0; i < out->binding_count; i++) {
        out->bindings[i] = cfg->behaviors[i];
    }
    k_mutex_unlock(&gremlin_td_mutex);
    return 0;
}

int zmk_gremlin_tap_dance_set(size_t slot, const struct zmk_gremlin_tap_dance_slot *in) {
    if (slot >= ZMK_GREMLIN_MAX_TAP_DANCES || !in) {
        return -EINVAL;
    }

    if (in->enabled) {
        if (in->tapping_term_ms == 0 || in->binding_count == 0 ||
            in->binding_count > ZMK_GREMLIN_MAX_TAP_DANCE_STEPS) {
            return -EINVAL;
        }
        for (uint8_t i = 0; i < in->binding_count; i++) {
            if (!in->bindings[i].behavior_dev ||
                zmk_behavior_validate_binding(&in->bindings[i]) < 0) {
                return -EINVAL;
            }
        }
    }

    k_mutex_lock(&gremlin_td_mutex, K_FOREVER);
    struct gremlin_td_reg *reg = &gremlin_td_registry[slot];
    struct behavior_tap_dance_config *cfg = reg->cfg;
    if (!cfg) {
        k_mutex_unlock(&gremlin_td_mutex);
        return -ENODEV;
    }

    for (int i = 0; i < ZMK_BHV_TAP_DANCE_MAX_HELD; i++) {
        if (active_tap_dances[i].position != ZMK_BHV_TAP_DANCE_POSITION_FREE &&
            active_tap_dances[i].config == cfg) {
            stop_timer(&active_tap_dances[i]);
            clear_tap_dance(&active_tap_dances[i]);
        }
    }

    cfg->enabled = in->enabled;
    cfg->tapping_term_ms = in->tapping_term_ms > 0 ? in->tapping_term_ms : 200;
    cfg->behavior_count = MIN(in->binding_count, (uint8_t)ZMK_GREMLIN_MAX_TAP_DANCE_STEPS);
    for (size_t i = 0; i < ZMK_GREMLIN_MAX_TAP_DANCE_STEPS; i++) {
        cfg->behaviors[i] = (i < cfg->behavior_count) ? in->bindings[i]
                                                      : (struct zmk_behavior_binding){0};
        reg->pending_local_ids[i] = UINT16_MAX;
    }
    reg->custom = true;
    reg->dirty = true;
    k_mutex_unlock(&gremlin_td_mutex);
    return 0;
}

#if IS_ENABLED(CONFIG_SETTINGS)
#define GREMLIN_TD_NVS_MAGIC 0x47544431U

struct gremlin_td_nvs_rec {
    uint32_t magic;
    uint32_t tapping_term_ms;
    uint8_t enabled;
    uint8_t binding_count;
    uint8_t reserved[2];
    uint16_t local_ids[ZMK_GREMLIN_MAX_TAP_DANCE_STEPS];
    uint32_t param1[ZMK_GREMLIN_MAX_TAP_DANCE_STEPS];
    uint32_t param2[ZMK_GREMLIN_MAX_TAP_DANCE_STEPS];
};

int gremlin_tap_dance_save_all(void) {
    int err = 0;
    for (size_t s = 0; s < ZMK_GREMLIN_MAX_TAP_DANCES; s++) {
        k_mutex_lock(&gremlin_td_mutex, K_FOREVER);
        struct gremlin_td_reg *reg = &gremlin_td_registry[s];
        struct behavior_tap_dance_config *cfg = reg->cfg;
        bool should_save = cfg && reg->dirty && reg->custom;
        struct gremlin_td_nvs_rec rec = {0};
        if (should_save) {
            rec.magic = GREMLIN_TD_NVS_MAGIC;
            rec.tapping_term_ms = cfg->tapping_term_ms;
            rec.enabled = cfg->enabled ? 1U : 0U;
            rec.binding_count =
                (uint8_t)MIN(cfg->behavior_count, (size_t)ZMK_GREMLIN_MAX_TAP_DANCE_STEPS);
            for (uint8_t i = 0; i < ZMK_GREMLIN_MAX_TAP_DANCE_STEPS; i++) {
                if (i < rec.binding_count && cfg->behaviors[i].behavior_dev) {
                    rec.local_ids[i] =
                        zmk_behavior_get_local_id(cfg->behaviors[i].behavior_dev);
                    rec.param1[i] = cfg->behaviors[i].param1;
                    rec.param2[i] = cfg->behaviors[i].param2;
                } else {
                    rec.local_ids[i] = UINT16_MAX;
                }
            }
        }
        k_mutex_unlock(&gremlin_td_mutex);

        if (!should_save) {
            continue;
        }

        char key[24];
        snprintf(key, sizeof(key), "gremlin_td/%u", (unsigned int)s);
        int r = settings_save_one(key, &rec, sizeof(rec));
        if (r == 0) {
            k_mutex_lock(&gremlin_td_mutex, K_FOREVER);
            reg->dirty = false;
            k_mutex_unlock(&gremlin_td_mutex);
        } else if (err == 0) {
            err = r;
        }
    }
    return err;
}

int gremlin_tap_dance_reset_all(void) {
    int err = 0;
    for (size_t s = 0; s < ZMK_GREMLIN_MAX_TAP_DANCES; s++) {
        char key[24];
        snprintf(key, sizeof(key), "gremlin_td/%u", (unsigned int)s);
        int r = settings_delete(key);
        if (r < 0 && r != -ENOENT && err == 0) {
            err = r;
        }
        k_mutex_lock(&gremlin_td_mutex, K_FOREVER);
        struct gremlin_td_reg *reg = &gremlin_td_registry[s];
        if (reg->cfg) {
            reg->cfg->enabled = false;
            reg->cfg->tapping_term_ms = 200;
            reg->cfg->behavior_count = 0;
            for (int i = 0; i < ZMK_GREMLIN_MAX_TAP_DANCE_STEPS; i++) {
                reg->cfg->behaviors[i] = (struct zmk_behavior_binding){0};
                reg->pending_local_ids[i] = UINT16_MAX;
            }
        }
        reg->custom = false;
        reg->dirty = false;
        k_mutex_unlock(&gremlin_td_mutex);
    }
    return err;
}

static int gremlin_td_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                   void *cb_arg) {
    if (!name || *name == '\0') {
        return -ENOENT;
    }

    char *endptr = NULL;
    unsigned long slot = strtoul(name, &endptr, 10);
    if (endptr == name || *endptr != '\0' || slot >= ZMK_GREMLIN_MAX_TAP_DANCES) {
        return -ENOENT;
    }

    struct gremlin_td_nvs_rec rec;
    if (len != sizeof(rec)) {
        return -EINVAL;
    }

    int rc = read_cb(cb_arg, &rec, sizeof(rec));
    if (rc < 0) {
        return rc;
    }
    if (rec.magic != GREMLIN_TD_NVS_MAGIC) {
        return -EINVAL;
    }

    k_mutex_lock(&gremlin_td_mutex, K_FOREVER);
    struct gremlin_td_reg *reg = &gremlin_td_registry[slot];
    if (reg->cfg) {
        reg->cfg->enabled = rec.enabled != 0U;
        reg->cfg->tapping_term_ms = rec.tapping_term_ms;
        reg->cfg->behavior_count =
            MIN(rec.binding_count, (uint8_t)ZMK_GREMLIN_MAX_TAP_DANCE_STEPS);
        for (int i = 0; i < ZMK_GREMLIN_MAX_TAP_DANCE_STEPS; i++) {
            reg->cfg->behaviors[i] = (struct zmk_behavior_binding){
                .behavior_dev = NULL, .param1 = rec.param1[i], .param2 = rec.param2[i]};
            reg->pending_local_ids[i] = rec.local_ids[i];
        }
        reg->custom = true;
        reg->dirty = false;
    }
    k_mutex_unlock(&gremlin_td_mutex);
    return 0;
}

static int gremlin_td_settings_commit(void) {
    k_mutex_lock(&gremlin_td_mutex, K_FOREVER);
    for (size_t s = 0; s < ZMK_GREMLIN_MAX_TAP_DANCES; s++) {
        struct gremlin_td_reg *reg = &gremlin_td_registry[s];
        if (!reg->cfg || !reg->custom) {
            continue;
        }
        for (size_t i = 0; i < reg->cfg->behavior_count; i++) {
            if (reg->pending_local_ids[i] != UINT16_MAX && !reg->cfg->behaviors[i].behavior_dev) {
                reg->cfg->behaviors[i].behavior_dev =
                    zmk_behavior_find_behavior_name_from_local_id(reg->pending_local_ids[i]);
            }
        }
    }
    k_mutex_unlock(&gremlin_td_mutex);
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(gremlin_td, "gremlin_td", NULL, gremlin_td_settings_set,
                               gremlin_td_settings_commit, NULL);
#else
int gremlin_tap_dance_save_all(void) { return 0; }
int gremlin_tap_dance_reset_all(void) { return 0; }
#endif

#define _TRANSFORM_ENTRY(idx, node) ZMK_KEYMAP_EXTRACT_BINDING(idx, node)

#define TRANSFORMED_BINDINGS(node)                                                                 \
    {LISTIFY(DT_INST_PROP_LEN(node, bindings), _TRANSFORM_ENTRY, (, ), DT_DRV_INST(node))}

#define TD_BINDINGS_ALLOC_LEN(n)                                                                   \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(n, gremlin_slot), (ZMK_GREMLIN_MAX_TAP_DANCE_STEPS),         \
                (DT_INST_PROP_LEN(n, bindings)))

#define KP_INST(n)                                                                                 \
    static struct zmk_behavior_binding                                                             \
        behavior_tap_dance_config_##n##_bindings[TD_BINDINGS_ALLOC_LEN(n)] =                       \
            TRANSFORMED_BINDINGS(n);                                                               \
    static struct behavior_tap_dance_config behavior_tap_dance_config_##n = {                      \
        .tapping_term_ms = DT_INST_PROP(n, tapping_term_ms),                                       \
        .behaviors = behavior_tap_dance_config_##n##_bindings,                                     \
        .behavior_count = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, gremlin_slot), (0),                 \
                                      (DT_INST_PROP_LEN(n, bindings))),                            \
        .gremlin_slot = DT_INST_PROP_OR(n, gremlin_slot, -1),                                      \
        .enabled = !DT_INST_NODE_HAS_PROP(n, gremlin_slot)};                                       \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_tap_dance_init, NULL, NULL,                                \
                            &behavior_tap_dance_config_##n, POST_KERNEL,                           \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_tap_dance_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KP_INST)

#endif
