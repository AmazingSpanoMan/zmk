/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_hold_tap

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zmk/keys.h>
#include <dt-bindings/zmk/keys.h>
#include <zephyr/logging/log.h>
#if IS_ENABLED(CONFIG_SETTINGS)
#include <zephyr/settings/settings.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zmk/behavior.h>
#include <zmk/gremlin_behaviors.h>
#include <zmk/matrix.h>
#include <zmk/endpoints.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/keycode_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define ZMK_BHV_HOLD_TAP_MAX_HELD CONFIG_ZMK_BEHAVIOR_HOLD_TAP_MAX_HELD
#define ZMK_BHV_HOLD_TAP_MAX_CAPTURED_EVENTS CONFIG_ZMK_BEHAVIOR_HOLD_TAP_MAX_CAPTURED_EVENTS

// increase if you have keyboard with more keys.
#define ZMK_BHV_HOLD_TAP_POSITION_NOT_USED 9999

enum flavor {
    FLAVOR_HOLD_PREFERRED,
    FLAVOR_BALANCED,
    FLAVOR_TAP_PREFERRED,
    FLAVOR_TAP_UNLESS_INTERRUPTED,
};

enum status {
    STATUS_UNDECIDED,
    STATUS_TAP,
    STATUS_HOLD_INTERRUPT,
    STATUS_HOLD_TIMER,
};

enum decision_moment {
    HT_KEY_DOWN,
    HT_KEY_UP,
    HT_OTHER_KEY_DOWN,
    HT_OTHER_KEY_UP,
    HT_TIMER_EVENT,
    HT_QUICK_TAP,
};

struct behavior_hold_tap_config {
    int tapping_term_ms;
    char *hold_behavior_dev;
    char *tap_behavior_dev;
    int quick_tap_ms;
    int require_prior_idle_ms;
    enum flavor flavor;
    bool hold_while_undecided;
    bool hold_while_undecided_linger;
    bool retro_tap;
    bool hold_trigger_on_release;
    int gremlin_slot;
    bool enabled;
    struct zmk_behavior_binding fixed_hold_binding;
    struct zmk_behavior_binding fixed_tap_binding;
    int32_t hold_trigger_key_positions_len;
    int32_t hold_trigger_key_positions[];
};

struct behavior_hold_tap_data {
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    struct behavior_parameter_metadata_set set;
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
};

// this data is specific for each hold-tap
struct active_hold_tap {
    int32_t position;
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
    uint8_t source;
#endif
    uint32_t param_hold;
    uint32_t param_tap;
    int64_t timestamp;
    enum status status;
    const struct behavior_hold_tap_config *config;
    struct k_work_delayable work;
    bool work_is_cancelled;

    // initialized to -1, which is to be interpreted as "no other key has been pressed yet"
    int32_t position_of_first_other_key_pressed;
};

// The undecided hold tap is the hold tap that needs to be decided before
// other keypress events can be released. While the undecided_hold_tap is
// not NULL, most events are captured in captured_events.
// After the hold_tap is decided, it will stay in the active_hold_taps until
// its key-up has been processed and the delayed work is cleaned up.
struct active_hold_tap *undecided_hold_tap = NULL;
struct active_hold_tap active_hold_taps[ZMK_BHV_HOLD_TAP_MAX_HELD] = {};
// We capture most position_state_changed events and some modifiers_state_changed events.

enum captured_event_tag {
    ET_NONE,
    ET_POS_CHANGED,
    ET_CODE_CHANGED,
};

union captured_event_data {
    struct zmk_position_state_changed_event position;
    struct zmk_keycode_state_changed_event keycode;
};

struct captured_event {
    enum captured_event_tag tag;
    union captured_event_data data;
};

struct captured_event captured_events[ZMK_BHV_HOLD_TAP_MAX_CAPTURED_EVENTS] = {};

// Keep track of which key was tapped most recently for the standard, if it is a hold-tap
// a position, will be given, if not it will just be INT32_MIN
struct last_tapped {
    int32_t position;
    int64_t timestamp;
};

// Set time stamp to large negative number initially for test suites, but not
// int64 min since it will overflow if -1 is added
struct last_tapped last_tapped = {INT32_MIN, INT32_MIN};

static void store_last_tapped(int64_t timestamp) {
    if (timestamp > last_tapped.timestamp) {
        last_tapped.position = INT32_MIN;
        last_tapped.timestamp = timestamp;
    }
}

static void store_last_hold_tapped(struct active_hold_tap *hold_tap) {
    last_tapped.position = hold_tap->position;
    last_tapped.timestamp = hold_tap->timestamp;
}

static bool is_quick_tap(struct active_hold_tap *hold_tap) {
    if ((last_tapped.timestamp + hold_tap->config->require_prior_idle_ms) > hold_tap->timestamp) {
        return true;
    } else {
        return (last_tapped.position == hold_tap->position) &&
               (last_tapped.timestamp + hold_tap->config->quick_tap_ms) > hold_tap->timestamp;
    }
}

static int capture_event(struct captured_event *data) {
    for (int i = 0; i < ZMK_BHV_HOLD_TAP_MAX_CAPTURED_EVENTS; i++) {
        if (captured_events[i].tag == ET_NONE) {
            captured_events[i] = *data;
            return 0;
        }
    }
    return -ENOMEM;
}

static bool have_captured_keydown_event(uint32_t position) {
    for (int i = 0; i < ZMK_BHV_HOLD_TAP_MAX_CAPTURED_EVENTS; i++) {
        struct captured_event *ev = &captured_events[i];
        if (ev->tag == ET_NONE) {
            return false;
        }

        if (ev->tag != ET_POS_CHANGED) {
            continue;
        }

        if (ev->data.position.data.position == position && ev->data.position.data.state) {
            return true;
        }
    }
    return false;
}

const struct zmk_listener zmk_listener_behavior_hold_tap;

static void release_captured_events() {
    if (undecided_hold_tap != NULL) {
        return;
    }

    // We use a trick to prevent copying the captured_events array.
    //
    // Events for different mod-tap instances are separated by a NULL pointer.
    //
    // The first event popped will never be caught by the next active hold-tap
    // because to start capturing a mod-tap-key-down event must first completely
    // go through the events queue.
    //
    // Example of this release process;
    // [mt2_down, k1_down, k1_up, mt2_up, null, ...]
    //  ^
    // mt2_down position event isn't captured because no hold-tap is active.
    // mt2_down behavior event is handled, now we have an undecided hold-tap
    // [null, k1_down, k1_up, mt2_up, null, ...]
    //        ^
    // k1_down  is captured by the mt2 mod-tap
    // !note that searches for find_captured_keydown_event by the mt2 behavior will stop at the
    // first null encountered [mt1_down, null, k1_up, mt2_up, null, ...]
    //                  ^
    // k1_up event is captured by the new hold-tap:
    // [k1_down, k1_up, null, mt2_up, null, ...]
    //                        ^
    // mt2_up event is not captured but causes release of mt2 behavior
    // [k1_down, k1_up, null, null, null, ...]
    // now mt2 will start releasing it's own captured positions.
    for (int i = 0; i < ZMK_BHV_HOLD_TAP_MAX_CAPTURED_EVENTS; i++) {
        struct captured_event *captured_event = &captured_events[i];
        enum captured_event_tag tag = captured_event->tag;

        if (tag == ET_NONE) {
            return;
        }

        captured_events[i].tag = ET_NONE;
        if (undecided_hold_tap != NULL) {
            k_msleep(10);
        }

        switch (tag) {
        case ET_CODE_CHANGED:
            LOG_DBG("Releasing mods changed event 0x%02X %s",
                    captured_event->data.keycode.data.keycode,
                    (captured_event->data.keycode.data.state ? "pressed" : "released"));
            ZMK_EVENT_RAISE_AT(captured_event->data.keycode, behavior_hold_tap);
            break;
        case ET_POS_CHANGED:
            LOG_DBG("Releasing key position event for position %d %s",
                    captured_event->data.position.data.position,
                    (captured_event->data.position.data.state ? "pressed" : "released"));
            ZMK_EVENT_RAISE_AT(captured_event->data.position, behavior_hold_tap);
            break;
        default:
            LOG_ERR("Unhandled captured event type");
            break;
        }
    }
}

static struct active_hold_tap *find_hold_tap(uint32_t position) {
    for (int i = 0; i < ZMK_BHV_HOLD_TAP_MAX_HELD; i++) {
        if (active_hold_taps[i].position == position) {
            return &active_hold_taps[i];
        }
    }
    return NULL;
}

static struct active_hold_tap *store_hold_tap(struct zmk_behavior_binding_event *event,
                                              uint32_t param_hold, uint32_t param_tap,
                                              const struct behavior_hold_tap_config *config) {
    for (int i = 0; i < ZMK_BHV_HOLD_TAP_MAX_HELD; i++) {
        if (active_hold_taps[i].position != ZMK_BHV_HOLD_TAP_POSITION_NOT_USED) {
            continue;
        }
        active_hold_taps[i].position = event->position;
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        active_hold_taps[i].source = event->source;
#endif
        active_hold_taps[i].status = STATUS_UNDECIDED;
        active_hold_taps[i].config = config;
        active_hold_taps[i].param_hold = param_hold;
        active_hold_taps[i].param_tap = param_tap;
        active_hold_taps[i].timestamp = event->timestamp;
        active_hold_taps[i].position_of_first_other_key_pressed = -1;
        return &active_hold_taps[i];
    }
    return NULL;
}

static void clear_hold_tap(struct active_hold_tap *hold_tap) {
    hold_tap->position = ZMK_BHV_HOLD_TAP_POSITION_NOT_USED;
    hold_tap->status = STATUS_UNDECIDED;
    hold_tap->work_is_cancelled = false;
}

static void decide_balanced(struct active_hold_tap *hold_tap, enum decision_moment event) {
    switch (event) {
    case HT_KEY_UP:
        hold_tap->status = STATUS_TAP;
        return;
    case HT_OTHER_KEY_UP:
        hold_tap->status = STATUS_HOLD_INTERRUPT;
        return;
    case HT_TIMER_EVENT:
        hold_tap->status = STATUS_HOLD_TIMER;
        return;
    case HT_QUICK_TAP:
        hold_tap->status = STATUS_TAP;
        return;
    default:
        return;
    }
}

static void decide_tap_preferred(struct active_hold_tap *hold_tap, enum decision_moment event) {
    switch (event) {
    case HT_KEY_UP:
        hold_tap->status = STATUS_TAP;
        return;
    case HT_TIMER_EVENT:
        hold_tap->status = STATUS_HOLD_TIMER;
        return;
    case HT_QUICK_TAP:
        hold_tap->status = STATUS_TAP;
        return;
    default:
        return;
    }
}

static void decide_tap_unless_interrupted(struct active_hold_tap *hold_tap,
                                          enum decision_moment event) {
    switch (event) {
    case HT_KEY_UP:
        hold_tap->status = STATUS_TAP;
        return;
    case HT_OTHER_KEY_DOWN:
        hold_tap->status = STATUS_HOLD_INTERRUPT;
        return;
    case HT_TIMER_EVENT:
        hold_tap->status = STATUS_TAP;
        return;
    case HT_QUICK_TAP:
        hold_tap->status = STATUS_TAP;
        return;
    default:
        return;
    }
}

static void decide_hold_preferred(struct active_hold_tap *hold_tap, enum decision_moment event) {
    switch (event) {
    case HT_KEY_UP:
        hold_tap->status = STATUS_TAP;
        return;
    case HT_OTHER_KEY_DOWN:
        hold_tap->status = STATUS_HOLD_INTERRUPT;
        return;
    case HT_TIMER_EVENT:
        hold_tap->status = STATUS_HOLD_TIMER;
        return;
    case HT_QUICK_TAP:
        hold_tap->status = STATUS_TAP;
        return;
    default:
        return;
    }
}

static inline const char *flavor_str(enum flavor flavor) {
    switch (flavor) {
    case FLAVOR_HOLD_PREFERRED:
        return "hold-preferred";
    case FLAVOR_BALANCED:
        return "balanced";
    case FLAVOR_TAP_PREFERRED:
        return "tap-preferred";
    case FLAVOR_TAP_UNLESS_INTERRUPTED:
        return "tap-unless-interrupted";
    default:
        return "UNKNOWN FLAVOR";
    }
}

static inline const char *status_str(enum status status) {
    switch (status) {
    case STATUS_UNDECIDED:
        return "undecided";
    case STATUS_HOLD_TIMER:
        return "hold-timer";
    case STATUS_HOLD_INTERRUPT:
        return "hold-interrupt";
    case STATUS_TAP:
        return "tap";
    default:
        return "UNKNOWN STATUS";
    }
}

static inline const char *decision_moment_str(enum decision_moment decision_moment) {
    switch (decision_moment) {
    case HT_KEY_UP:
        return "key-up";
    case HT_OTHER_KEY_DOWN:
        return "other-key-down";
    case HT_OTHER_KEY_UP:
        return "other-key-up";
    case HT_QUICK_TAP:
        return "quick-tap";
    case HT_TIMER_EVENT:
        return "timer";
    default:
        return "UNKNOWN STATUS";
    }
}

static int press_hold_binding(struct active_hold_tap *hold_tap) {
    struct zmk_behavior_binding_event event = {
        .position = hold_tap->position,
        .timestamp = hold_tap->timestamp,
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = hold_tap->source,
#endif
    };

    struct zmk_behavior_binding binding = (hold_tap->config->gremlin_slot >= 0)
        ? hold_tap->config->fixed_hold_binding
        : (struct zmk_behavior_binding){.behavior_dev = hold_tap->config->hold_behavior_dev,
                                        .param1 = hold_tap->param_hold};
    if (!binding.behavior_dev) {
        return 0;
    }
    return zmk_behavior_invoke_binding(&binding, event, true);
}

static int press_tap_binding(struct active_hold_tap *hold_tap) {
    struct zmk_behavior_binding_event event = {
        .position = hold_tap->position,
        .timestamp = hold_tap->timestamp,
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = hold_tap->source,
#endif
    };

    struct zmk_behavior_binding binding = (hold_tap->config->gremlin_slot >= 0)
        ? hold_tap->config->fixed_tap_binding
        : (struct zmk_behavior_binding){.behavior_dev = hold_tap->config->tap_behavior_dev,
                                        .param1 = hold_tap->param_tap};
    store_last_hold_tapped(hold_tap);
    if (!binding.behavior_dev) {
        return 0;
    }
    return zmk_behavior_invoke_binding(&binding, event, true);
}

static int release_hold_binding(struct active_hold_tap *hold_tap) {
    struct zmk_behavior_binding_event event = {
        .position = hold_tap->position,
        .timestamp = hold_tap->timestamp,
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = hold_tap->source,
#endif
    };

    struct zmk_behavior_binding binding = (hold_tap->config->gremlin_slot >= 0)
        ? hold_tap->config->fixed_hold_binding
        : (struct zmk_behavior_binding){.behavior_dev = hold_tap->config->hold_behavior_dev,
                                        .param1 = hold_tap->param_hold};
    if (!binding.behavior_dev) {
        return 0;
    }
    return zmk_behavior_invoke_binding(&binding, event, false);
}

static int release_tap_binding(struct active_hold_tap *hold_tap) {
    struct zmk_behavior_binding_event event = {
        .position = hold_tap->position,
        .timestamp = hold_tap->timestamp,
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = hold_tap->source,
#endif
    };

    struct zmk_behavior_binding binding = (hold_tap->config->gremlin_slot >= 0)
        ? hold_tap->config->fixed_tap_binding
        : (struct zmk_behavior_binding){.behavior_dev = hold_tap->config->tap_behavior_dev,
                                        .param1 = hold_tap->param_tap};
    if (!binding.behavior_dev) {
        return 0;
    }
    return zmk_behavior_invoke_binding(&binding, event, false);
}

static int press_binding(struct active_hold_tap *hold_tap) {
    if (hold_tap->config->retro_tap && hold_tap->status == STATUS_HOLD_TIMER) {
        return 0;
    }

    if (hold_tap->status == STATUS_HOLD_TIMER || hold_tap->status == STATUS_HOLD_INTERRUPT) {
        if (hold_tap->config->hold_while_undecided) {
            // the hold is already active, so we don't need to press it again
            return 0;
        } else {
            return press_hold_binding(hold_tap);
        }
    } else {
        if (hold_tap->config->hold_while_undecided &&
            !hold_tap->config->hold_while_undecided_linger) {
            // time to release the hold before pressing the tap
            release_hold_binding(hold_tap);
        }
        return press_tap_binding(hold_tap);
    }
}

static int release_binding(struct active_hold_tap *hold_tap) {
    if (hold_tap->config->retro_tap && hold_tap->status == STATUS_HOLD_TIMER) {
        return 0;
    }

    if (hold_tap->status == STATUS_HOLD_TIMER || hold_tap->status == STATUS_HOLD_INTERRUPT) {
        return release_hold_binding(hold_tap);
    } else {
        return release_tap_binding(hold_tap);
    }
}

static bool is_first_other_key_pressed_trigger_key(struct active_hold_tap *hold_tap) {
    for (int i = 0; i < hold_tap->config->hold_trigger_key_positions_len; i++) {
        if (hold_tap->config->hold_trigger_key_positions[i] ==
            hold_tap->position_of_first_other_key_pressed) {
            return true;
        }
    }
    return false;
}

// Force a tap decision if the positional conditions for a hold decision are not met.
static void decide_positional_hold(struct active_hold_tap *hold_tap) {
    // Only force a tap decision if the positional hold/tap feature is enabled.
    if (!(hold_tap->config->hold_trigger_key_positions_len > 0)) {
        return;
    }

    // Only force a tap decision if another key was pressed after
    // the hold/tap key.
    if (hold_tap->position_of_first_other_key_pressed == -1) {
        return;
    }

    // Only force a tap decision if the first other key to be pressed
    // (after the hold/tap key) is not one of the trigger keys.
    if (is_first_other_key_pressed_trigger_key(hold_tap)) {
        return;
    }

    // Since the positional key conditions have failed, force a TAP decision.
    hold_tap->status = STATUS_TAP;
}

static void decide_hold_tap(struct active_hold_tap *hold_tap,
                            enum decision_moment decision_moment) {
    if (hold_tap->status != STATUS_UNDECIDED) {
        return;
    }

    if (hold_tap != undecided_hold_tap) {
        LOG_DBG("ERROR found undecided tap hold that is not the active tap hold");
        return;
    }

    if (hold_tap->config->hold_while_undecided && decision_moment == HT_KEY_DOWN) {
        LOG_DBG("%d hold behavior pressed while undecided", hold_tap->position);
        press_hold_binding(hold_tap);
        return;
    }

    // If the hold-tap behavior is still undecided, attempt to decide it.
    switch (hold_tap->config->flavor) {
    case FLAVOR_HOLD_PREFERRED:
        decide_hold_preferred(hold_tap, decision_moment);
        break;
    case FLAVOR_BALANCED:
        decide_balanced(hold_tap, decision_moment);
        break;
    case FLAVOR_TAP_PREFERRED:
        decide_tap_preferred(hold_tap, decision_moment);
        break;
    case FLAVOR_TAP_UNLESS_INTERRUPTED:
        decide_tap_unless_interrupted(hold_tap, decision_moment);
        break;
    }

    if (hold_tap->status == STATUS_UNDECIDED) {
        return;
    }

    decide_positional_hold(hold_tap);

    // Since the hold-tap has been decided, clean up undecided_hold_tap and
    // execute the decided behavior.
    LOG_DBG("%d decided %s (%s decision moment %s)", hold_tap->position,
            status_str(hold_tap->status), flavor_str(hold_tap->config->flavor),
            decision_moment_str(decision_moment));
    undecided_hold_tap = NULL;
    press_binding(hold_tap);
    release_captured_events();
}

static void decide_retro_tap(struct active_hold_tap *hold_tap) {
    if (!hold_tap->config->retro_tap) {
        return;
    }
    if (hold_tap->status == STATUS_HOLD_TIMER) {
        release_binding(hold_tap);
        LOG_DBG("%d retro tap", hold_tap->position);
        hold_tap->status = STATUS_TAP;
        press_binding(hold_tap);
        return;
    }
}

static void update_hold_status_for_retro_tap(uint32_t ignore_position) {
    for (int i = 0; i < ZMK_BHV_HOLD_TAP_MAX_HELD; i++) {
        struct active_hold_tap *hold_tap = &active_hold_taps[i];
        if (hold_tap->position == ignore_position ||
            hold_tap->position == ZMK_BHV_HOLD_TAP_POSITION_NOT_USED ||
            hold_tap->config->retro_tap == false) {
            continue;
        }
        if (hold_tap->status == STATUS_HOLD_TIMER) {
            LOG_DBG("Update hold tap %d status to hold-interrupt", hold_tap->position);
            hold_tap->status = STATUS_HOLD_INTERRUPT;
            press_binding(hold_tap);
        }
    }
}

static int on_hold_tap_binding_pressed(struct zmk_behavior_binding *binding,
                                       struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_hold_tap_config *cfg = dev->config;

    if (cfg->gremlin_slot >= 0 && !cfg->enabled) {
        return ZMK_BEHAVIOR_OPAQUE;
    }

    if (undecided_hold_tap != NULL) {
        LOG_DBG("ERROR another hold-tap behavior is undecided.");
        // if this happens, make sure the behavior events occur AFTER other position events.
        return ZMK_BEHAVIOR_OPAQUE;
    }

    struct active_hold_tap *hold_tap =
        store_hold_tap(&event, binding->param1, binding->param2, cfg);

    if (hold_tap == NULL) {
        LOG_ERR("unable to store hold-tap info, did you press more than %d hold-taps?",
                ZMK_BHV_HOLD_TAP_MAX_HELD);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    LOG_DBG("%d new undecided hold_tap", event.position);
    undecided_hold_tap = hold_tap;

    if (is_quick_tap(hold_tap)) {
        decide_hold_tap(hold_tap, HT_QUICK_TAP);
    }

    decide_hold_tap(hold_tap, HT_KEY_DOWN);

    // if this behavior was queued we have to adjust the timer to only
    // wait for the remaining time.
    int32_t tapping_term_ms_left = (hold_tap->timestamp + cfg->tapping_term_ms) - k_uptime_get();
    k_work_schedule(&hold_tap->work, K_MSEC(tapping_term_ms_left));

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_hold_tap_binding_released(struct zmk_behavior_binding *binding,
                                        struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_hold_tap_config *cfg = dev ? dev->config : NULL;

    struct active_hold_tap *hold_tap = find_hold_tap(event.position);
    if (hold_tap == NULL) {
        if (cfg && cfg->gremlin_slot >= 0 && !cfg->enabled) {
            return ZMK_BEHAVIOR_OPAQUE;
        }
        LOG_ERR("ACTIVE_HOLD_TAP_CLEANED_UP_TOO_EARLY");
        return ZMK_BEHAVIOR_OPAQUE;
    }

    // If these events were queued, the timer event may be queued too late or not at all.
    // We insert a timer event before the TH_KEY_UP event to verify.
    int work_cancel_result = k_work_cancel_delayable(&hold_tap->work);
    if (event.timestamp > (hold_tap->timestamp + hold_tap->config->tapping_term_ms)) {
        decide_hold_tap(hold_tap, HT_TIMER_EVENT);
    }

    decide_hold_tap(hold_tap, HT_KEY_UP);
    decide_retro_tap(hold_tap);
    release_binding(hold_tap);

    if (hold_tap->config->hold_while_undecided && hold_tap->config->hold_while_undecided_linger) {
        release_hold_binding(hold_tap);
    }

    if (work_cancel_result == -EINPROGRESS) {
        // let the timer handler clean up
        // if we'd clear now, the timer may call back for an uninitialized active_hold_tap.
        LOG_DBG("%d hold-tap timer work in event queue", event.position);
        hold_tap->work_is_cancelled = true;
    } else {
        LOG_DBG("%d cleaning up hold-tap", event.position);
        clear_hold_tap(hold_tap);
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
static int hold_tap_parameter_metadata(const struct device *hold_tap,
                                       struct behavior_parameter_metadata *param_metadata) {
    const struct behavior_hold_tap_config *cfg = hold_tap->config;
    struct behavior_hold_tap_data *data = hold_tap->data;
    int err;
    struct behavior_parameter_metadata child_meta;

    if (cfg->gremlin_slot >= 0) {
        return zmk_behavior_get_empty_param_metadata(hold_tap, param_metadata);
    }

    err = behavior_get_parameter_metadata(zmk_behavior_get_binding(cfg->hold_behavior_dev),
                                          &child_meta);
    if (err < 0) {
        LOG_WRN("Failed to get the hold behavior parameter: %d", err);
        return err;
    }

    if (child_meta.sets_len > 0) {
        data->set.param1_values = child_meta.sets[0].param1_values;
        data->set.param1_values_len = child_meta.sets[0].param1_values_len;
    }

    err = behavior_get_parameter_metadata(zmk_behavior_get_binding(cfg->tap_behavior_dev),
                                          &child_meta);
    if (err < 0) {
        LOG_WRN("Failed to get the tap behavior parameter: %d", err);
        return err;
    }

    if (child_meta.sets_len > 0) {
        data->set.param2_values = child_meta.sets[0].param1_values;
        data->set.param2_values_len = child_meta.sets[0].param1_values_len;
    }

    param_metadata->sets = &data->set;
    param_metadata->sets_len = 1;

    return 0;
}

#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)

static const struct behavior_driver_api behavior_hold_tap_driver_api = {
    .binding_pressed = on_hold_tap_binding_pressed,
    .binding_released = on_hold_tap_binding_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = hold_tap_parameter_metadata,
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
};

static int position_state_changed_listener(const zmk_event_t *eh) {
    struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);

    update_hold_status_for_retro_tap(ev->position);

    if (undecided_hold_tap == NULL) {
        LOG_DBG("%d bubble (no undecided hold_tap active)", ev->position);
        return ZMK_EV_EVENT_BUBBLE;
    }

    // Store the position of pressed key for positional hold-tap purposes.
    if ((undecided_hold_tap->config->hold_trigger_on_release !=
         ev->state) // key has been pressed and hold_trigger_on_release is not set, or key
                    // has been released and hold_trigger_on_release is set
        && (undecided_hold_tap->position_of_first_other_key_pressed ==
            -1) // no other key has been pressed yet
    ) {
        undecided_hold_tap->position_of_first_other_key_pressed = ev->position;
    }

    if (undecided_hold_tap->position == ev->position) {
        if (ev->state) { // keydown
            LOG_ERR("hold-tap listener should be called before before most other listeners!");
            return ZMK_EV_EVENT_BUBBLE;
        } else { // keyup
            LOG_DBG("%d bubble undecided hold-tap keyrelease event", undecided_hold_tap->position);
            return ZMK_EV_EVENT_BUBBLE;
        }
    }

    // If these events were queued, the timer event may be queued too late or not at all.
    // We make a timer decision before the other key events are handled if the timer would
    // have run out.
    if (ev->timestamp >
        (undecided_hold_tap->timestamp + undecided_hold_tap->config->tapping_term_ms)) {
        decide_hold_tap(undecided_hold_tap, HT_TIMER_EVENT);
    }

    if (undecided_hold_tap == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (!ev->state && !have_captured_keydown_event(ev->position)) {
        // no keydown event has been captured, let it bubble.
        // we'll catch modifiers later in modifier_state_changed_listener
        LOG_DBG("%d bubbling %d %s event", undecided_hold_tap->position, ev->position,
                ev->state ? "down" : "up");
        return ZMK_EV_EVENT_BUBBLE;
    }

    LOG_DBG("%d capturing %d %s event", undecided_hold_tap->position, ev->position,
            ev->state ? "down" : "up");
    struct captured_event capture = {
        .tag = ET_POS_CHANGED,
        .data = {.position = copy_raised_zmk_position_state_changed(ev)},
    };
    capture_event(&capture);
    decide_hold_tap(undecided_hold_tap, ev->state ? HT_OTHER_KEY_DOWN : HT_OTHER_KEY_UP);
    return ZMK_EV_EVENT_CAPTURED;
}

static int keycode_state_changed_listener(const zmk_event_t *eh) {
    // we want to catch layer-up events too... how?
    struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);

    if (ev->state && !is_mod(ev->usage_page, ev->keycode)) {
        store_last_tapped(ev->timestamp);
    }

    if (undecided_hold_tap == NULL) {
        // LOG_DBG("0x%02X bubble (no undecided hold_tap active)", ev->keycode);
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (!is_mod(ev->usage_page, ev->keycode)) {
        // LOG_DBG("0x%02X bubble (not a mod)", ev->keycode);
        return ZMK_EV_EVENT_BUBBLE;
    }

    // hold-while-undecided can produce a mod, but we don't want to capture it.
    if (undecided_hold_tap->config->hold_while_undecided &&
        undecided_hold_tap->status == STATUS_UNDECIDED) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    // only key-up events will bubble through position_state_changed_listener
    // if a undecided_hold_tap is active.
    LOG_DBG("%d capturing 0x%02X %s event", undecided_hold_tap->position, ev->keycode,
            ev->state ? "down" : "up");
    struct captured_event capture = {
        .tag = ET_CODE_CHANGED, .data = {.keycode = copy_raised_zmk_keycode_state_changed(ev)}};
    capture_event(&capture);
    return ZMK_EV_EVENT_CAPTURED;
}

int behavior_hold_tap_listener(const zmk_event_t *eh) {
    if (as_zmk_position_state_changed(eh) != NULL) {
        return position_state_changed_listener(eh);
    } else if (as_zmk_keycode_state_changed(eh) != NULL) {
        return keycode_state_changed_listener(eh);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(behavior_hold_tap, behavior_hold_tap_listener);
ZMK_SUBSCRIPTION(behavior_hold_tap, zmk_position_state_changed);
// this should be modifiers_state_changed, but unfrotunately that's not implemented yet.
ZMK_SUBSCRIPTION(behavior_hold_tap, zmk_keycode_state_changed);

void behavior_hold_tap_timer_work_handler(struct k_work *item) {
    struct k_work_delayable *d_work = k_work_delayable_from_work(item);
    struct active_hold_tap *hold_tap = CONTAINER_OF(d_work, struct active_hold_tap, work);

    if (hold_tap->work_is_cancelled) {
        clear_hold_tap(hold_tap);
    } else {
        decide_hold_tap(hold_tap, HT_TIMER_EVENT);
    }
}

struct gremlin_ht_reg {
    struct behavior_hold_tap_config *cfg;
    bool custom;
    bool dirty;
    uint16_t pending_hold_id;
    uint16_t pending_tap_id;
};

static struct gremlin_ht_reg gremlin_ht_registry[ZMK_GREMLIN_MAX_HOLD_TAPS];
static K_MUTEX_DEFINE(gremlin_ht_mutex);

static int behavior_hold_tap_init(const struct device *dev) {
    static bool init_first_run = true;

    if (init_first_run) {
        for (int i = 0; i < ZMK_BHV_HOLD_TAP_MAX_HELD; i++) {
            k_work_init_delayable(&active_hold_taps[i].work, behavior_hold_tap_timer_work_handler);
            active_hold_taps[i].position = ZMK_BHV_HOLD_TAP_POSITION_NOT_USED;
        }
    }
    init_first_run = false;

    struct behavior_hold_tap_config *cfg = (struct behavior_hold_tap_config *)(uintptr_t)dev->config;
    if (cfg && cfg->gremlin_slot >= 0 && cfg->gremlin_slot < ZMK_GREMLIN_MAX_HOLD_TAPS) {
        gremlin_ht_registry[cfg->gremlin_slot].cfg = cfg;
    }

    return 0;
}

static size_t gremlin_ht_slot_count_internal(void) {
    size_t count = 0;
    for (size_t i = 0; i < ZMK_GREMLIN_MAX_HOLD_TAPS; i++) {
        if (gremlin_ht_registry[i].cfg != NULL) {
            count++;
        } else {
            break;
        }
    }
    return count;
}

size_t zmk_gremlin_hold_tap_count(void) { return gremlin_ht_slot_count_internal(); }

int zmk_gremlin_hold_tap_get(size_t slot, struct zmk_gremlin_hold_tap_slot *out) {
    if (slot >= ZMK_GREMLIN_MAX_HOLD_TAPS || !out) {
        return -EINVAL;
    }

    k_mutex_lock(&gremlin_ht_mutex, K_FOREVER);
    struct behavior_hold_tap_config *cfg = gremlin_ht_registry[slot].cfg;
    if (!cfg) {
        k_mutex_unlock(&gremlin_ht_mutex);
        return -ENODEV;
    }
    *out = (struct zmk_gremlin_hold_tap_slot){
        .enabled = cfg->enabled,
        .tapping_term_ms = cfg->tapping_term_ms,
        .quick_tap_ms = cfg->quick_tap_ms,
        .require_prior_idle_ms = cfg->require_prior_idle_ms,
        .flavor = (uint8_t)cfg->flavor,
        .retro_tap = cfg->retro_tap,
        .hold_binding = cfg->fixed_hold_binding,
        .tap_binding = cfg->fixed_tap_binding,
    };
    k_mutex_unlock(&gremlin_ht_mutex);
    return 0;
}

int zmk_gremlin_hold_tap_set(size_t slot, const struct zmk_gremlin_hold_tap_slot *in) {
    if (slot >= ZMK_GREMLIN_MAX_HOLD_TAPS || !in) {
        return -EINVAL;
    }

    if (in->enabled) {
        if (in->tapping_term_ms <= 0 || in->flavor > ZMK_GREMLIN_HT_TAP_PREFERRED) {
            return -EINVAL;
        }
        if (in->hold_binding.behavior_dev &&
            zmk_behavior_validate_binding(&in->hold_binding) < 0) {
            return -EINVAL;
        }
        if (in->tap_binding.behavior_dev &&
            zmk_behavior_validate_binding(&in->tap_binding) < 0) {
            return -EINVAL;
        }
    }

    k_mutex_lock(&gremlin_ht_mutex, K_FOREVER);
    struct gremlin_ht_reg *reg = &gremlin_ht_registry[slot];
    struct behavior_hold_tap_config *cfg = reg->cfg;
    if (!cfg) {
        k_mutex_unlock(&gremlin_ht_mutex);
        return -ENODEV;
    }

    for (int i = 0; i < ZMK_BHV_HOLD_TAP_MAX_HELD; i++) {
        if (active_hold_taps[i].position != ZMK_BHV_HOLD_TAP_POSITION_NOT_USED &&
            active_hold_taps[i].config == cfg) {
            k_work_cancel_delayable(&active_hold_taps[i].work);
            if (undecided_hold_tap == &active_hold_taps[i]) {
                undecided_hold_tap = NULL;
            }
            clear_hold_tap(&active_hold_taps[i]);
        }
    }

    cfg->enabled = in->enabled;
    cfg->tapping_term_ms = in->tapping_term_ms > 0 ? in->tapping_term_ms : 200;
    cfg->quick_tap_ms = in->quick_tap_ms;
    cfg->require_prior_idle_ms = in->require_prior_idle_ms;
    cfg->flavor = (enum flavor)in->flavor;
    cfg->retro_tap = in->retro_tap;
    cfg->fixed_hold_binding = in->hold_binding;
    cfg->fixed_tap_binding = in->tap_binding;
    reg->custom = true;
    reg->dirty = true;
    reg->pending_hold_id = UINT16_MAX;
    reg->pending_tap_id = UINT16_MAX;

    k_mutex_unlock(&gremlin_ht_mutex);
    return 0;
}

#if IS_ENABLED(CONFIG_SETTINGS)
#define GREMLIN_HT_NVS_MAGIC 0x47485431U

struct gremlin_ht_nvs_rec {
    uint32_t magic;
    int32_t tapping_term_ms;
    int32_t quick_tap_ms;
    int32_t require_prior_idle_ms;
    uint16_t hold_local_id;
    uint16_t tap_local_id;
    uint32_t hold_param1;
    uint32_t hold_param2;
    uint32_t tap_param1;
    uint32_t tap_param2;
    uint8_t enabled;
    uint8_t flavor;
    uint8_t retro_tap;
    uint8_t reserved;
};

int gremlin_hold_tap_save_all(void) {
    int err = 0;
    for (size_t s = 0; s < ZMK_GREMLIN_MAX_HOLD_TAPS; s++) {
        k_mutex_lock(&gremlin_ht_mutex, K_FOREVER);
        struct gremlin_ht_reg *reg = &gremlin_ht_registry[s];
        struct behavior_hold_tap_config *cfg = reg->cfg;
        bool should_save = cfg && reg->dirty && reg->custom;
        struct gremlin_ht_nvs_rec rec = {0};
        if (should_save) {
            rec.magic = GREMLIN_HT_NVS_MAGIC;
            rec.tapping_term_ms = cfg->tapping_term_ms;
            rec.quick_tap_ms = cfg->quick_tap_ms;
            rec.require_prior_idle_ms = cfg->require_prior_idle_ms;
            rec.hold_local_id = cfg->fixed_hold_binding.behavior_dev
                ? zmk_behavior_get_local_id(cfg->fixed_hold_binding.behavior_dev)
                : UINT16_MAX;
            rec.tap_local_id = cfg->fixed_tap_binding.behavior_dev
                ? zmk_behavior_get_local_id(cfg->fixed_tap_binding.behavior_dev)
                : UINT16_MAX;
            rec.hold_param1 = cfg->fixed_hold_binding.param1;
            rec.hold_param2 = cfg->fixed_hold_binding.param2;
            rec.tap_param1 = cfg->fixed_tap_binding.param1;
            rec.tap_param2 = cfg->fixed_tap_binding.param2;
            rec.enabled = cfg->enabled ? 1U : 0U;
            rec.flavor = (uint8_t)cfg->flavor;
            rec.retro_tap = cfg->retro_tap ? 1U : 0U;
        }
        k_mutex_unlock(&gremlin_ht_mutex);

        if (!should_save) {
            continue;
        }

        char key[24];
        snprintf(key, sizeof(key), "gremlin_ht/%u", (unsigned int)s);
        int r = settings_save_one(key, &rec, sizeof(rec));
        if (r == 0) {
            k_mutex_lock(&gremlin_ht_mutex, K_FOREVER);
            reg->dirty = false;
            k_mutex_unlock(&gremlin_ht_mutex);
        } else if (err == 0) {
            err = r;
        }
    }
    return err;
}

int gremlin_hold_tap_reset_all(void) {
    int err = 0;
    for (size_t s = 0; s < ZMK_GREMLIN_MAX_HOLD_TAPS; s++) {
        char key[24];
        snprintf(key, sizeof(key), "gremlin_ht/%u", (unsigned int)s);
        int r = settings_delete(key);
        if (r < 0 && r != -ENOENT && err == 0) {
            err = r;
        }
        k_mutex_lock(&gremlin_ht_mutex, K_FOREVER);
        struct gremlin_ht_reg *reg = &gremlin_ht_registry[s];
        if (reg->cfg) {
            reg->cfg->enabled = false;
            reg->cfg->tapping_term_ms = 200;
            reg->cfg->quick_tap_ms = -1;
            reg->cfg->require_prior_idle_ms = -1;
            reg->cfg->flavor = FLAVOR_HOLD_PREFERRED;
            reg->cfg->retro_tap = false;
            reg->cfg->fixed_hold_binding = (struct zmk_behavior_binding){0};
            reg->cfg->fixed_tap_binding = (struct zmk_behavior_binding){0};
        }
        reg->custom = false;
        reg->dirty = false;
        reg->pending_hold_id = UINT16_MAX;
        reg->pending_tap_id = UINT16_MAX;
        k_mutex_unlock(&gremlin_ht_mutex);
    }
    return err;
}

static int gremlin_ht_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                   void *cb_arg) {
    if (!name || *name == '\0') {
        return -ENOENT;
    }

    char *endptr = NULL;
    unsigned long slot = strtoul(name, &endptr, 10);
    if (endptr == name || *endptr != '\0' || slot >= ZMK_GREMLIN_MAX_HOLD_TAPS) {
        return -ENOENT;
    }

    struct gremlin_ht_nvs_rec rec;
    if (len != sizeof(rec)) {
        return -EINVAL;
    }

    int rc = read_cb(cb_arg, &rec, sizeof(rec));
    if (rc < 0) {
        return rc;
    }
    if (rec.magic != GREMLIN_HT_NVS_MAGIC) {
        return -EINVAL;
    }

    k_mutex_lock(&gremlin_ht_mutex, K_FOREVER);
    struct gremlin_ht_reg *reg = &gremlin_ht_registry[slot];
    if (reg->cfg) {
        reg->cfg->enabled = rec.enabled != 0U;
        reg->cfg->tapping_term_ms = rec.tapping_term_ms;
        reg->cfg->quick_tap_ms = rec.quick_tap_ms;
        reg->cfg->require_prior_idle_ms = rec.require_prior_idle_ms;
        reg->cfg->flavor = (enum flavor)rec.flavor;
        reg->cfg->retro_tap = rec.retro_tap != 0U;
        reg->cfg->fixed_hold_binding = (struct zmk_behavior_binding){
            .behavior_dev = NULL, .param1 = rec.hold_param1, .param2 = rec.hold_param2};
        reg->cfg->fixed_tap_binding = (struct zmk_behavior_binding){
            .behavior_dev = NULL, .param1 = rec.tap_param1, .param2 = rec.tap_param2};
        reg->pending_hold_id = rec.hold_local_id;
        reg->pending_tap_id = rec.tap_local_id;
        reg->custom = true;
        reg->dirty = false;
    }
    k_mutex_unlock(&gremlin_ht_mutex);
    return 0;
}

static int gremlin_ht_settings_commit(void) {
    k_mutex_lock(&gremlin_ht_mutex, K_FOREVER);
    for (size_t s = 0; s < ZMK_GREMLIN_MAX_HOLD_TAPS; s++) {
        struct gremlin_ht_reg *reg = &gremlin_ht_registry[s];
        if (!reg->cfg || !reg->custom) {
            continue;
        }
        if (reg->pending_hold_id != UINT16_MAX && !reg->cfg->fixed_hold_binding.behavior_dev) {
            reg->cfg->fixed_hold_binding.behavior_dev =
                zmk_behavior_find_behavior_name_from_local_id(reg->pending_hold_id);
        }
        if (reg->pending_tap_id != UINT16_MAX && !reg->cfg->fixed_tap_binding.behavior_dev) {
            reg->cfg->fixed_tap_binding.behavior_dev =
                zmk_behavior_find_behavior_name_from_local_id(reg->pending_tap_id);
        }
    }
    k_mutex_unlock(&gremlin_ht_mutex);
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(gremlin_ht, "gremlin_ht", NULL, gremlin_ht_settings_set,
                               gremlin_ht_settings_commit, NULL);
#else
int gremlin_hold_tap_save_all(void) { return 0; }
int gremlin_hold_tap_reset_all(void) { return 0; }
#endif

#define KP_INST(n)                                                                                 \
    static struct behavior_hold_tap_config behavior_hold_tap_config_##n = {                        \
        .tapping_term_ms = DT_INST_PROP(n, tapping_term_ms),                                       \
        .hold_behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(n, bindings, 0)),               \
        .tap_behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(n, bindings, 1)),                \
        .quick_tap_ms = DT_INST_PROP(n, quick_tap_ms),                                             \
        .require_prior_idle_ms = DT_INST_PROP(n, global_quick_tap)                                 \
                                     ? DT_INST_PROP(n, quick_tap_ms)                               \
                                     : DT_INST_PROP(n, require_prior_idle_ms),                     \
        .flavor = DT_ENUM_IDX(DT_DRV_INST(n), flavor),                                             \
        .hold_while_undecided = DT_INST_PROP(n, hold_while_undecided),                             \
        .hold_while_undecided_linger = DT_INST_PROP(n, hold_while_undecided_linger),               \
        .retro_tap = DT_INST_PROP(n, retro_tap),                                                   \
        .hold_trigger_on_release = DT_INST_PROP(n, hold_trigger_on_release),                       \
        .gremlin_slot = DT_INST_PROP_OR(n, gremlin_slot, -1),                                      \
        .enabled = !DT_INST_NODE_HAS_PROP(n, gremlin_slot),                                        \
        .hold_trigger_key_positions = DT_INST_PROP(n, hold_trigger_key_positions),                 \
        .hold_trigger_key_positions_len = DT_INST_PROP_LEN(n, hold_trigger_key_positions),         \
    };                                                                                             \
    static struct behavior_hold_tap_data behavior_hold_tap_data_##n = {};                          \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_hold_tap_init, NULL, &behavior_hold_tap_data_##n,          \
                            &behavior_hold_tap_config_##n, POST_KERNEL,                            \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_hold_tap_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KP_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
