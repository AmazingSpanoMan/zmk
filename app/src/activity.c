/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#if IS_ENABLED(CONFIG_SETTINGS)
#include <zephyr/settings/settings.h>
#endif
#include <zephyr/sys/poweroff.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>

#include <zmk/pm.h>

#include <zmk/activity.h>

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
#include <zmk/usb.h>
#endif

#if IS_ENABLED(CONFIG_ZMK_POINTING)
#include <zephyr/input/input.h>
#endif

bool is_usb_power_present(void) {
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
    return zmk_usb_is_powered();
#else
    return false;
#endif /* IS_ENABLED(CONFIG_USB_DEVICE_STACK) */
}

static enum zmk_activity_state activity_state;

static uint32_t activity_last_uptime;
static struct k_spinlock activity_cfg_lock;
static uint32_t runtime_idle_ms = CONFIG_ZMK_IDLE_TIMEOUT;
#if IS_ENABLED(CONFIG_ZMK_SLEEP)
static uint32_t runtime_sleep_ms = CONFIG_ZMK_IDLE_SLEEP_TIMEOUT;
#else
static uint32_t runtime_sleep_ms = 0;
#endif
static bool activity_cfg_custom = false;
static bool activity_cfg_dirty = false;

int raise_event(void) {
    return raise_zmk_activity_state_changed(
        (struct zmk_activity_state_changed){.state = activity_state});
}

int set_state(enum zmk_activity_state state) {
    if (activity_state == state)
        return 0;

    activity_state = state;
    return raise_event();
}

enum zmk_activity_state zmk_activity_get_state(void) { return activity_state; }

static int note_activity(void) {
    k_spinlock_key_t key = k_spin_lock(&activity_cfg_lock);
    activity_last_uptime = (uint32_t)k_uptime_get();
    k_spin_unlock(&activity_cfg_lock, key);

    return set_state(ZMK_ACTIVITY_ACTIVE);
}

static int activity_event_listener(const zmk_event_t *eh) { return note_activity(); }

void activity_work_handler(struct k_work *work) {
    k_spinlock_key_t key = k_spin_lock(&activity_cfg_lock);
    uint32_t current = (uint32_t)k_uptime_get();
    uint32_t inactive_time = current - activity_last_uptime;
    uint32_t idle_limit = runtime_idle_ms;
    uint32_t sleep_limit = runtime_sleep_ms;
    k_spin_unlock(&activity_cfg_lock, key);

#if IS_ENABLED(CONFIG_ZMK_SLEEP)
    if (sleep_limit > 0U && inactive_time > sleep_limit && !is_usb_power_present()) {
        // Put devices in suspend power mode before sleeping
        set_state(ZMK_ACTIVITY_SLEEP);

        if (zmk_pm_suspend_devices() < 0) {
            LOG_ERR("Failed to suspend all the devices");
            zmk_pm_resume_devices();
            return;
        }

        sys_poweroff();
    } else
#endif /* IS_ENABLED(CONFIG_ZMK_SLEEP) */
        if (idle_limit > 0U && inactive_time > idle_limit) {
            set_state(ZMK_ACTIVITY_IDLE);
        }
}

uint32_t zmk_activity_get_idle_timeout_ms(void) {
    k_spinlock_key_t key = k_spin_lock(&activity_cfg_lock);
    uint32_t v = runtime_idle_ms;
    k_spin_unlock(&activity_cfg_lock, key);
    return v;
}

uint32_t zmk_activity_get_sleep_timeout_ms(void) {
    k_spinlock_key_t key = k_spin_lock(&activity_cfg_lock);
    uint32_t v = runtime_sleep_ms;
    k_spin_unlock(&activity_cfg_lock, key);
    return v;
}

int zmk_activity_set_timeouts(uint32_t idle_ms, uint32_t sleep_ms) {
    k_spinlock_key_t key = k_spin_lock(&activity_cfg_lock);
    runtime_idle_ms = idle_ms;
#if IS_ENABLED(CONFIG_ZMK_SLEEP)
    runtime_sleep_ms = sleep_ms;
#else
    ARG_UNUSED(sleep_ms);
    runtime_sleep_ms = 0;
#endif
    activity_last_uptime = (uint32_t)k_uptime_get();
    activity_cfg_custom = true;
    activity_cfg_dirty = true;
    k_spin_unlock(&activity_cfg_lock, key);

    set_state(ZMK_ACTIVITY_ACTIVE);
    return 0;
}

#if IS_ENABLED(CONFIG_SETTINGS)
#define ACTIVITY_NVS_MAGIC 0x47414331U

struct activity_nvs_cfg {
    uint32_t magic;
    uint32_t idle_ms;
    uint32_t sleep_ms;
};

int zmk_activity_save_timeouts(void) {
    k_spinlock_key_t key = k_spin_lock(&activity_cfg_lock);
    bool should_save = activity_cfg_dirty && activity_cfg_custom;
    struct activity_nvs_cfg rec = {
        .magic = ACTIVITY_NVS_MAGIC,
        .idle_ms = runtime_idle_ms,
        .sleep_ms = runtime_sleep_ms,
    };
    k_spin_unlock(&activity_cfg_lock, key);

    if (!should_save) {
        return 0;
    }

    int err = settings_save_one("activity/timeouts", &rec, sizeof(rec));
    if (err == 0) {
        key = k_spin_lock(&activity_cfg_lock);
        activity_cfg_dirty = false;
        k_spin_unlock(&activity_cfg_lock, key);
    }
    return err;
}

int zmk_activity_reset_timeouts(void) {
    int err = settings_delete("activity/timeouts");
    k_spinlock_key_t key = k_spin_lock(&activity_cfg_lock);
    runtime_idle_ms = CONFIG_ZMK_IDLE_TIMEOUT;
#if IS_ENABLED(CONFIG_ZMK_SLEEP)
    runtime_sleep_ms = CONFIG_ZMK_IDLE_SLEEP_TIMEOUT;
#else
    runtime_sleep_ms = 0;
#endif
    activity_cfg_custom = false;
    activity_cfg_dirty = false;
    activity_last_uptime = (uint32_t)k_uptime_get();
    k_spin_unlock(&activity_cfg_lock, key);
    return (err == -ENOENT) ? 0 : err;
}

static int activity_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                 void *cb_arg) {
    if (!name || strcmp(name, "timeouts") != 0) {
        return -ENOENT;
    }

    struct activity_nvs_cfg rec;
    if (len != sizeof(rec)) {
        return -EINVAL;
    }

    int rc = read_cb(cb_arg, &rec, sizeof(rec));
    if (rc < 0) {
        return rc;
    }
    if (rec.magic != ACTIVITY_NVS_MAGIC) {
        return -EINVAL;
    }

    k_spinlock_key_t key = k_spin_lock(&activity_cfg_lock);
    runtime_idle_ms = rec.idle_ms;
#if IS_ENABLED(CONFIG_ZMK_SLEEP)
    runtime_sleep_ms = rec.sleep_ms;
#else
    runtime_sleep_ms = 0;
#endif
    activity_cfg_custom = true;
    activity_cfg_dirty = false;
    k_spin_unlock(&activity_cfg_lock, key);
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(activity_cfg, "activity", NULL, activity_settings_set, NULL, NULL);
#else
int zmk_activity_save_timeouts(void) { return 0; }
int zmk_activity_reset_timeouts(void) {
    return zmk_activity_set_timeouts(CONFIG_ZMK_IDLE_TIMEOUT,
#if IS_ENABLED(CONFIG_ZMK_SLEEP)
                                     CONFIG_ZMK_IDLE_SLEEP_TIMEOUT
#else
                                     0
#endif
    );
}
#endif

K_WORK_DEFINE(activity_work, activity_work_handler);

void activity_expiry_function(struct k_timer *_timer) { k_work_submit(&activity_work); }

K_TIMER_DEFINE(activity_timer, activity_expiry_function, NULL);

static int activity_init(void) {
    activity_last_uptime = k_uptime_get();

    k_timer_start(&activity_timer, K_SECONDS(1), K_SECONDS(1));
    return 0;
}

ZMK_LISTENER(activity, activity_event_listener);
ZMK_SUBSCRIPTION(activity, zmk_position_state_changed);
ZMK_SUBSCRIPTION(activity, zmk_sensor_event);

#if IS_ENABLED(CONFIG_ZMK_POINTING)

static void note_activity_work_cb(struct k_work *_work) { note_activity(); }

K_WORK_DEFINE(note_activity_work, note_activity_work_cb);

static void activity_input_listener(struct input_event *ev) { k_work_submit(&note_activity_work); }

INPUT_CALLBACK_DEFINE(NULL, activity_input_listener);

#endif

SYS_INIT(activity_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
