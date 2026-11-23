/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#if IS_ENABLED(CONFIG_SETTINGS)
#include <zephyr/settings/settings.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zmk/behavior.h>
#include <zmk/behavior_queue.h>
#include <zmk/gremlin_behaviors.h>
#include <zmk/keymap.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

enum behavior_macro_mode {
    MACRO_MODE_TAP,
    MACRO_MODE_PRESS,
    MACRO_MODE_RELEASE,
};

enum param_source { PARAM_SOURCE_BINDING, PARAM_SOURCE_MACRO_1ST, PARAM_SOURCE_MACRO_2ND };

struct behavior_macro_trigger_state {
    uint32_t wait_ms;
    uint32_t tap_ms;
    enum behavior_macro_mode mode;
    uint16_t start_index;
    uint16_t count;
    enum param_source param1_source;
    enum param_source param2_source;
};

struct behavior_macro_state {
    struct behavior_macro_trigger_state release_state;

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    struct behavior_parameter_metadata_set set;
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)

    uint32_t press_bindings_count;
};

struct behavior_macro_config {
    uint32_t default_wait_ms;
    uint32_t default_tap_ms;
    uint32_t count;
    int gremlin_slot;
    struct zmk_behavior_binding bindings[];
};

struct gremlin_mac_reg {
    const struct device *dev;
    bool custom;
    bool dirty;
    struct zmk_gremlin_macro_slot slot;
    uint16_t pending_local_ids[ZMK_GREMLIN_MAX_MACRO_STEPS];
};

static struct gremlin_mac_reg gremlin_mac_registry[ZMK_GREMLIN_MAX_MACROS];
static K_MUTEX_DEFINE(gremlin_mac_mutex);

#define TAP_MODE DEVICE_DT_NAME(DT_INST(0, zmk_macro_control_mode_tap))
#define PRESS_MODE DEVICE_DT_NAME(DT_INST(0, zmk_macro_control_mode_press))
#define REL_MODE DEVICE_DT_NAME(DT_INST(0, zmk_macro_control_mode_release))

#define TAP_TIME DEVICE_DT_NAME(DT_INST(0, zmk_macro_control_tap_time))
#define WAIT_TIME DEVICE_DT_NAME(DT_INST(0, zmk_macro_control_wait_time))
#define WAIT_REL DEVICE_DT_NAME(DT_INST(0, zmk_macro_pause_for_release))

#define P1TO1 DEVICE_DT_NAME(DT_INST(0, zmk_macro_param_1to1))
#define P1TO2 DEVICE_DT_NAME(DT_INST(0, zmk_macro_param_1to2))
#define P2TO1 DEVICE_DT_NAME(DT_INST(0, zmk_macro_param_2to1))
#define P2TO2 DEVICE_DT_NAME(DT_INST(0, zmk_macro_param_2to2))

#define ZM_IS_NODE_MATCH(a, b) (strcmp(a, b) == 0)
#define IS_TAP_MODE(dev) ZM_IS_NODE_MATCH(dev, TAP_MODE)
#define IS_PRESS_MODE(dev) ZM_IS_NODE_MATCH(dev, PRESS_MODE)
#define IS_RELEASE_MODE(dev) ZM_IS_NODE_MATCH(dev, REL_MODE)

#define IS_TAP_TIME(dev) ZM_IS_NODE_MATCH(dev, TAP_TIME)
#define IS_WAIT_TIME(dev) ZM_IS_NODE_MATCH(dev, WAIT_TIME)
#define IS_PAUSE(dev) ZM_IS_NODE_MATCH(dev, WAIT_REL)

#define IS_P1TO1(dev) ZM_IS_NODE_MATCH(dev, P1TO1)
#define IS_P1TO2(dev) ZM_IS_NODE_MATCH(dev, P1TO2)
#define IS_P2TO1(dev) ZM_IS_NODE_MATCH(dev, P2TO1)
#define IS_P2TO2(dev) ZM_IS_NODE_MATCH(dev, P2TO2)

static bool handle_control_binding(struct behavior_macro_trigger_state *state,
                                   const struct zmk_behavior_binding *binding) {
    if (IS_TAP_MODE(binding->behavior_dev)) {
        state->mode = MACRO_MODE_TAP;
        LOG_DBG("macro mode set: tap");
    } else if (IS_PRESS_MODE(binding->behavior_dev)) {
        state->mode = MACRO_MODE_PRESS;
        LOG_DBG("macro mode set: press");
    } else if (IS_RELEASE_MODE(binding->behavior_dev)) {
        state->mode = MACRO_MODE_RELEASE;
        LOG_DBG("macro mode set: release");
    } else if (IS_TAP_TIME(binding->behavior_dev)) {
        state->tap_ms = binding->param1;
        LOG_DBG("macro tap time set: %d", state->tap_ms);
    } else if (IS_WAIT_TIME(binding->behavior_dev)) {
        state->wait_ms = binding->param1;
        LOG_DBG("macro wait time set: %d", state->wait_ms);
    } else if (IS_P1TO1(binding->behavior_dev)) {
        state->param1_source = PARAM_SOURCE_MACRO_1ST;
        LOG_DBG("macro param: 1to1");
    } else if (IS_P1TO2(binding->behavior_dev)) {
        state->param2_source = PARAM_SOURCE_MACRO_1ST;
        LOG_DBG("macro param: 1to2");
    } else if (IS_P2TO1(binding->behavior_dev)) {
        state->param1_source = PARAM_SOURCE_MACRO_2ND;
        LOG_DBG("macro param: 2to1");
    } else if (IS_P2TO2(binding->behavior_dev)) {
        state->param2_source = PARAM_SOURCE_MACRO_2ND;
        LOG_DBG("macro param: 2to2");
    } else {
        return false;
    }

    return true;
}

static int behavior_macro_init(const struct device *dev) {
    const struct behavior_macro_config *cfg = dev->config;
    struct behavior_macro_state *state = dev->data;
    state->press_bindings_count = cfg->count;
    state->release_state.start_index = cfg->count;
    state->release_state.count = 0;

    if (cfg->gremlin_slot >= 0 && cfg->gremlin_slot < ZMK_GREMLIN_MAX_MACROS) {
        gremlin_mac_registry[cfg->gremlin_slot].dev = dev;
        gremlin_mac_registry[cfg->gremlin_slot].slot.wait_ms = cfg->default_wait_ms;
        gremlin_mac_registry[cfg->gremlin_slot].slot.tap_ms = cfg->default_tap_ms;
        for (int i = 0; i < ZMK_GREMLIN_MAX_MACRO_STEPS; i++) {
            gremlin_mac_registry[cfg->gremlin_slot].pending_local_ids[i] = UINT16_MAX;
        }
        return 0;
    }

    LOG_DBG("Precalculate initial release state:");
    for (int i = 0; i < cfg->count; i++) {
        if (handle_control_binding(&state->release_state, &cfg->bindings[i])) {
            // Updated state used for initial state on release.
        } else if (IS_PAUSE(cfg->bindings[i].behavior_dev)) {
            state->release_state.start_index = i + 1;
            state->release_state.count = cfg->count - state->release_state.start_index;
            state->press_bindings_count = i;
            LOG_DBG("Release will resume at %d", state->release_state.start_index);
            break;
        } else {
            // Mostly ignore regular invokable bindings, except they will consume macro parameters
            state->release_state.param1_source = PARAM_SOURCE_BINDING;
            state->release_state.param2_source = PARAM_SOURCE_BINDING;
        }
    }

    return 0;
};

static uint32_t select_param(enum param_source param_source, uint32_t source_binding,
                             const struct zmk_behavior_binding *macro_binding) {
    switch (param_source) {
    case PARAM_SOURCE_MACRO_1ST:
        return macro_binding->param1;
    case PARAM_SOURCE_MACRO_2ND:
        return macro_binding->param2;
    default:
        return source_binding;
    }
};

static void replace_params(struct behavior_macro_trigger_state *state,
                           struct zmk_behavior_binding *binding,
                           const struct zmk_behavior_binding *macro_binding) {
    binding->param1 = select_param(state->param1_source, binding->param1, macro_binding);
    binding->param2 = select_param(state->param2_source, binding->param2, macro_binding);

    state->param1_source = PARAM_SOURCE_BINDING;
    state->param2_source = PARAM_SOURCE_BINDING;
}

static void queue_macro(struct zmk_behavior_binding_event *event,
                        const struct zmk_behavior_binding bindings[],
                        struct behavior_macro_trigger_state state,
                        const struct zmk_behavior_binding *macro_binding) {
    LOG_DBG("Iterating macro bindings - starting: %d, count: %d", state.start_index, state.count);
    for (int i = state.start_index; i < state.start_index + state.count; i++) {
        if (!handle_control_binding(&state, &bindings[i])) {
            struct zmk_behavior_binding binding = bindings[i];
            replace_params(&state, &binding, macro_binding);

            switch (state.mode) {
            case MACRO_MODE_TAP:
                zmk_behavior_queue_add(event, binding, true, state.tap_ms);
                zmk_behavior_queue_add(event, binding, false, state.wait_ms);
                break;
            case MACRO_MODE_PRESS:
                zmk_behavior_queue_add(event, binding, true, state.wait_ms);
                break;
            case MACRO_MODE_RELEASE:
                zmk_behavior_queue_add(event, binding, false, state.wait_ms);
                break;
            default:
                LOG_ERR("Unknown macro mode: %d", state.mode);
                break;
            }
        }
    }
}

static void queue_dynamic_macro(int slot_idx, struct zmk_behavior_binding_event *event) {
    if (slot_idx < 0 || slot_idx >= ZMK_GREMLIN_MAX_MACROS) {
        return;
    }

    k_mutex_lock(&gremlin_mac_mutex, K_FOREVER);
    struct zmk_gremlin_macro_slot snap = gremlin_mac_registry[slot_idx].slot;
    k_mutex_unlock(&gremlin_mac_mutex);

    if (!snap.enabled || snap.step_count == 0) {
        return;
    }

    uint32_t tap_ms = snap.tap_ms > 0 ? snap.tap_ms : CONFIG_ZMK_MACRO_DEFAULT_TAP_MS;
    uint32_t wait_ms = snap.wait_ms > 0 ? snap.wait_ms : CONFIG_ZMK_MACRO_DEFAULT_WAIT_MS;

    for (uint8_t i = 0; i < MIN(snap.step_count, (uint8_t)ZMK_GREMLIN_MAX_MACRO_STEPS); i++) {
        const struct zmk_gremlin_macro_step *st = &snap.steps[i];
        switch (st->type) {
        case ZMK_GREMLIN_MACRO_STEP_TAP:
            if (st->binding.behavior_dev) {
                zmk_behavior_queue_add(event, st->binding, true, tap_ms);
                zmk_behavior_queue_add(event, st->binding, false, wait_ms);
            }
            break;
        case ZMK_GREMLIN_MACRO_STEP_PRESS:
            if (st->binding.behavior_dev) {
                zmk_behavior_queue_add(event, st->binding, true, wait_ms);
            }
            break;
        case ZMK_GREMLIN_MACRO_STEP_RELEASE:
            if (st->binding.behavior_dev) {
                zmk_behavior_queue_add(event, st->binding, false, wait_ms);
            }
            break;
        case ZMK_GREMLIN_MACRO_STEP_WAIT_MS:
#if DT_HAS_COMPAT_STATUS_OKAY(zmk_behavior_none)
            {
                struct zmk_behavior_binding wait_b = {
                    .behavior_dev = DEVICE_DT_NAME(DT_INST(0, zmk_behavior_none)),
                };
                zmk_behavior_queue_add(event, wait_b, true, st->delay_ms);
            }
#endif
            break;
        default:
            break;
        }
    }
}

static int on_macro_binding_pressed(struct zmk_behavior_binding *binding,
                                    struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_macro_config *cfg = dev->config;
    if (cfg->gremlin_slot >= 0) {
        queue_dynamic_macro(cfg->gremlin_slot, &event);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    struct behavior_macro_state *state = dev->data;
    struct behavior_macro_trigger_state trigger_state = {.mode = MACRO_MODE_TAP,
                                                         .tap_ms = cfg->default_tap_ms,
                                                         .wait_ms = cfg->default_wait_ms,
                                                         .start_index = 0,
                                                         .count = state->press_bindings_count};

    queue_macro(&event, cfg->bindings, trigger_state, binding);

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_macro_binding_released(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_macro_config *cfg = dev->config;
    if (cfg->gremlin_slot >= 0) {
        return ZMK_BEHAVIOR_OPAQUE;
    }

    struct behavior_macro_state *state = dev->data;

    queue_macro(&event, cfg->bindings, state->release_state, binding);

    return ZMK_BEHAVIOR_OPAQUE;
}

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)

static void assign_values_to_set(enum param_source param_source,
                                 struct behavior_parameter_metadata_set *set,
                                 const struct behavior_parameter_value_metadata *values,
                                 size_t values_len) {
    if (param_source == PARAM_SOURCE_MACRO_1ST) {
        set->param1_values = values;
        set->param1_values_len = values_len;
    } else {
        set->param2_values = values;
        set->param2_values_len = values_len;
    }
}

// This function will dynamically determine the parameter metadata for a particular macro by
// inspecting the macro *bindings* to see what behaviors in that list receive the macro parameters,
// and then using the metadata from those behaviors for the macro itself.
//
// Care need be taken, where a behavior in the list takes two parameters, and the macro passes along
// a value for the *second* parameter, we need to make sure we find the right metadata set for the
// referenced behavior that matches the first parameter.
static int get_macro_parameter_metadata(const struct device *macro,
                                        struct behavior_parameter_metadata *param_metadata) {
    const struct behavior_macro_config *cfg = macro->config;
    if (cfg->gremlin_slot >= 0) {
        return zmk_behavior_get_empty_param_metadata(macro, param_metadata);
    }

    struct behavior_macro_state *data = macro->data;
    struct behavior_macro_trigger_state state = {0};

    for (int i = 0; (i < cfg->count) && (!data->set.param1_values || !data->set.param2_values);
         i++) {
        if (handle_control_binding(&state, &cfg->bindings[i]) ||
            (state.param1_source == PARAM_SOURCE_BINDING &&
             state.param2_source == PARAM_SOURCE_BINDING)) {
            continue;
        }

        LOG_DBG("checking %d for the given state", i);

        struct behavior_parameter_metadata binding_meta;
        int err = behavior_get_parameter_metadata(
            zmk_behavior_get_binding(cfg->bindings[i].behavior_dev), &binding_meta);
        if (err < 0 || binding_meta.sets_len == 0) {
            LOG_WRN("Failed to fetch macro binding parameter details %d", err);
            return -ENOTSUP;
        }

        // If both macro parameters get passed to this one entry, use
        // the metadata for this behavior verbatim.
        if (state.param1_source != PARAM_SOURCE_BINDING &&
            state.param2_source != PARAM_SOURCE_BINDING) {
            param_metadata->sets_len = binding_meta.sets_len;
            param_metadata->sets = binding_meta.sets;
            return 0;
        }

        if (state.param1_source != PARAM_SOURCE_BINDING) {
            assign_values_to_set(state.param1_source, &data->set,
                                 binding_meta.sets[0].param1_values,
                                 binding_meta.sets[0].param1_values_len);
        }

        if (state.param2_source != PARAM_SOURCE_BINDING) {
            // For the param2 metadata, we need to find a set that matches fully bound first
            // parameter of our macro entry, and use the metadata from that set.
            for (int s = 0; s < binding_meta.sets_len; s++) {
                if (zmk_behavior_validate_param_values(binding_meta.sets[s].param1_values,
                                                       binding_meta.sets[s].param1_values_len,
                                                       cfg->bindings[i].param1) >= 0) {
                    assign_values_to_set(state.param2_source, &data->set,
                                         binding_meta.sets[s].param2_values,
                                         binding_meta.sets[s].param2_values_len);
                    break;
                }
            }
        }

        state.param1_source = PARAM_SOURCE_BINDING;
        state.param2_source = PARAM_SOURCE_BINDING;
    }

    param_metadata->sets_len = 1;
    param_metadata->sets = &data->set;

    return 0;
}

#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)

static const struct behavior_driver_api behavior_macro_driver_api = {
    .binding_pressed = on_macro_binding_pressed,
    .binding_released = on_macro_binding_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = get_macro_parameter_metadata,
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
};

size_t zmk_gremlin_macro_count(void) {
    size_t count = 0;
    for (size_t i = 0; i < ZMK_GREMLIN_MAX_MACROS; i++) {
        if (gremlin_mac_registry[i].dev != NULL) {
            count++;
        } else {
            break;
        }
    }
    return count;
}

int zmk_gremlin_macro_get(size_t slot, struct zmk_gremlin_macro_slot *out) {
    if (slot >= ZMK_GREMLIN_MAX_MACROS || !out) {
        return -EINVAL;
    }

    k_mutex_lock(&gremlin_mac_mutex, K_FOREVER);
    if (!gremlin_mac_registry[slot].dev) {
        k_mutex_unlock(&gremlin_mac_mutex);
        return -ENODEV;
    }
    *out = gremlin_mac_registry[slot].slot;
    k_mutex_unlock(&gremlin_mac_mutex);
    return 0;
}

int zmk_gremlin_macro_set(size_t slot, const struct zmk_gremlin_macro_slot *in) {
    if (slot >= ZMK_GREMLIN_MAX_MACROS || !in) {
        return -EINVAL;
    }

    if (in->enabled) {
        if (in->step_count > ZMK_GREMLIN_MAX_MACRO_STEPS) {
            return -EINVAL;
        }
        for (uint8_t i = 0; i < in->step_count; i++) {
            const struct zmk_gremlin_macro_step *st = &in->steps[i];
            if (st->type > ZMK_GREMLIN_MACRO_STEP_WAIT_MS) {
                return -EINVAL;
            }
            if (st->type != ZMK_GREMLIN_MACRO_STEP_WAIT_MS) {
                if (!st->binding.behavior_dev ||
                    zmk_behavior_validate_binding(&st->binding) < 0) {
                    return -EINVAL;
                }
            }
        }
    }

    k_mutex_lock(&gremlin_mac_mutex, K_FOREVER);
    struct gremlin_mac_reg *reg = &gremlin_mac_registry[slot];
    if (!reg->dev) {
        k_mutex_unlock(&gremlin_mac_mutex);
        return -ENODEV;
    }
    reg->slot = *in;
    reg->slot.step_count = MIN(in->step_count, (uint8_t)ZMK_GREMLIN_MAX_MACRO_STEPS);
    reg->custom = true;
    reg->dirty = true;
    for (int i = 0; i < ZMK_GREMLIN_MAX_MACRO_STEPS; i++) {
        reg->pending_local_ids[i] = UINT16_MAX;
    }
    k_mutex_unlock(&gremlin_mac_mutex);
    return 0;
}

#if IS_ENABLED(CONFIG_SETTINGS)
#define GREMLIN_MAC_NVS_MAGIC 0x474D4331U

struct gremlin_mac_nvs_step {
    uint16_t local_id;
    uint8_t type;
    uint8_t reserved;
    uint32_t param1;
    uint32_t param2;
    uint32_t delay_ms;
};

struct gremlin_mac_nvs_rec {
    uint32_t magic;
    uint32_t wait_ms;
    uint32_t tap_ms;
    uint8_t enabled;
    uint8_t step_count;
    uint8_t reserved[2];
    struct gremlin_mac_nvs_step steps[ZMK_GREMLIN_MAX_MACRO_STEPS];
};

int gremlin_macro_save_all(void) {
    int err = 0;
    for (size_t s = 0; s < ZMK_GREMLIN_MAX_MACROS; s++) {
        k_mutex_lock(&gremlin_mac_mutex, K_FOREVER);
        struct gremlin_mac_reg *reg = &gremlin_mac_registry[s];
        bool should_save = reg->dev && reg->dirty && reg->custom;
        struct gremlin_mac_nvs_rec rec = {0};
        if (should_save) {
            rec.magic = GREMLIN_MAC_NVS_MAGIC;
            rec.wait_ms = reg->slot.wait_ms;
            rec.tap_ms = reg->slot.tap_ms;
            rec.enabled = reg->slot.enabled ? 1U : 0U;
            rec.step_count = MIN(reg->slot.step_count, (uint8_t)ZMK_GREMLIN_MAX_MACRO_STEPS);
            for (uint8_t i = 0; i < ZMK_GREMLIN_MAX_MACRO_STEPS; i++) {
                rec.steps[i].type = reg->slot.steps[i].type;
                rec.steps[i].delay_ms = reg->slot.steps[i].delay_ms;
                rec.steps[i].param1 = reg->slot.steps[i].binding.param1;
                rec.steps[i].param2 = reg->slot.steps[i].binding.param2;
                rec.steps[i].local_id =
                    (i < rec.step_count && reg->slot.steps[i].binding.behavior_dev)
                        ? zmk_behavior_get_local_id(reg->slot.steps[i].binding.behavior_dev)
                        : UINT16_MAX;
            }
        }
        k_mutex_unlock(&gremlin_mac_mutex);

        if (!should_save) {
            continue;
        }

        char key[24];
        snprintf(key, sizeof(key), "gremlin_mac/%u", (unsigned int)s);
        int r = settings_save_one(key, &rec, sizeof(rec));
        if (r == 0) {
            k_mutex_lock(&gremlin_mac_mutex, K_FOREVER);
            reg->dirty = false;
            k_mutex_unlock(&gremlin_mac_mutex);
        } else if (err == 0) {
            err = r;
        }
    }
    return err;
}

int gremlin_macro_reset_all(void) {
    int err = 0;
    for (size_t s = 0; s < ZMK_GREMLIN_MAX_MACROS; s++) {
        char key[24];
        snprintf(key, sizeof(key), "gremlin_mac/%u", (unsigned int)s);
        int r = settings_delete(key);
        if (r < 0 && r != -ENOENT && err == 0) {
            err = r;
        }
        k_mutex_lock(&gremlin_mac_mutex, K_FOREVER);
        struct gremlin_mac_reg *reg = &gremlin_mac_registry[s];
        memset(&reg->slot, 0, sizeof(reg->slot));
        if (reg->dev) {
            const struct behavior_macro_config *cfg = reg->dev->config;
            reg->slot.wait_ms = cfg->default_wait_ms;
            reg->slot.tap_ms = cfg->default_tap_ms;
        }
        reg->custom = false;
        reg->dirty = false;
        for (int i = 0; i < ZMK_GREMLIN_MAX_MACRO_STEPS; i++) {
            reg->pending_local_ids[i] = UINT16_MAX;
        }
        k_mutex_unlock(&gremlin_mac_mutex);
    }
    return err;
}

static int gremlin_mac_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                    void *cb_arg) {
    if (!name || *name == '\0') {
        return -ENOENT;
    }

    char *endptr = NULL;
    unsigned long slot = strtoul(name, &endptr, 10);
    if (endptr == name || *endptr != '\0' || slot >= ZMK_GREMLIN_MAX_MACROS) {
        return -ENOENT;
    }

    struct gremlin_mac_nvs_rec rec;
    if (len != sizeof(rec)) {
        return -EINVAL;
    }

    int rc = read_cb(cb_arg, &rec, sizeof(rec));
    if (rc < 0) {
        return rc;
    }
    if (rec.magic != GREMLIN_MAC_NVS_MAGIC) {
        return -EINVAL;
    }

    k_mutex_lock(&gremlin_mac_mutex, K_FOREVER);
    struct gremlin_mac_reg *reg = &gremlin_mac_registry[slot];
    if (reg->dev) {
        memset(&reg->slot, 0, sizeof(reg->slot));
        reg->slot.enabled = rec.enabled != 0U;
        reg->slot.wait_ms = rec.wait_ms;
        reg->slot.tap_ms = rec.tap_ms;
        reg->slot.step_count = MIN(rec.step_count, (uint8_t)ZMK_GREMLIN_MAX_MACRO_STEPS);
        for (int i = 0; i < ZMK_GREMLIN_MAX_MACRO_STEPS; i++) {
            reg->slot.steps[i].type = rec.steps[i].type;
            reg->slot.steps[i].delay_ms = rec.steps[i].delay_ms;
            reg->slot.steps[i].binding.behavior_dev = NULL;
            reg->slot.steps[i].binding.param1 = rec.steps[i].param1;
            reg->slot.steps[i].binding.param2 = rec.steps[i].param2;
            reg->pending_local_ids[i] = rec.steps[i].local_id;
        }
        reg->custom = true;
        reg->dirty = false;
    }
    k_mutex_unlock(&gremlin_mac_mutex);
    return 0;
}

static int gremlin_mac_settings_commit(void) {
    k_mutex_lock(&gremlin_mac_mutex, K_FOREVER);
    for (size_t s = 0; s < ZMK_GREMLIN_MAX_MACROS; s++) {
        struct gremlin_mac_reg *reg = &gremlin_mac_registry[s];
        if (!reg->dev || !reg->custom) {
            continue;
        }
        for (uint8_t i = 0; i < reg->slot.step_count; i++) {
            if (reg->pending_local_ids[i] != UINT16_MAX &&
                !reg->slot.steps[i].binding.behavior_dev) {
                reg->slot.steps[i].binding.behavior_dev =
                    zmk_behavior_find_behavior_name_from_local_id(reg->pending_local_ids[i]);
            }
        }
    }
    k_mutex_unlock(&gremlin_mac_mutex);
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(gremlin_mac, "gremlin_mac", NULL, gremlin_mac_settings_set,
                               gremlin_mac_settings_commit, NULL);
#else
int gremlin_macro_save_all(void) { return 0; }
int gremlin_macro_reset_all(void) { return 0; }
#endif

#define TRANSFORMED_BEHAVIORS(n)                                                                   \
    {LISTIFY(DT_PROP_LEN(n, bindings), ZMK_KEYMAP_EXTRACT_BINDING, (, ), n)},

#define MACRO_INST(inst)                                                                           \
    static struct behavior_macro_state behavior_macro_state_##inst = {};                           \
    static struct behavior_macro_config behavior_macro_config_##inst = {                           \
        .default_wait_ms = DT_PROP_OR(inst, wait_ms, CONFIG_ZMK_MACRO_DEFAULT_WAIT_MS),            \
        .default_tap_ms = DT_PROP_OR(inst, tap_ms, CONFIG_ZMK_MACRO_DEFAULT_TAP_MS),               \
        .count = DT_PROP_LEN(inst, bindings),                                                      \
        .gremlin_slot = DT_PROP_OR(inst, gremlin_slot, -1),                                        \
        .bindings = TRANSFORMED_BEHAVIORS(inst)};                                                  \
    BEHAVIOR_DT_DEFINE(inst, behavior_macro_init, NULL, &behavior_macro_state_##inst,              \
                       &behavior_macro_config_##inst, POST_KERNEL,                                 \
                       CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_macro_driver_api);

DT_FOREACH_STATUS_OKAY(zmk_behavior_macro, MACRO_INST)
DT_FOREACH_STATUS_OKAY(zmk_behavior_macro_one_param, MACRO_INST)
DT_FOREACH_STATUS_OKAY(zmk_behavior_macro_two_param, MACRO_INST)
