// SPDX-License-Identifier: Apache-2.0
#include "raylib_lite_example_board.h"
#include "platform_esp_audio.h"

#include <stdatomic.h>
#include <string.h>
#include "esp_board_manager_includes.h"
#include "mosaico_hardware.h"
#include "esp_check.h"
#include "esp_display_present.h"
#include "esp_display_present_config.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mosaico_video.h"
#include "raylib_lite_action.h"
#include "nvs_flash.h"

#define INPUT_CAPACITY 32U
#define ESP_MOSAICO_GAME_WIDTH 480
#define ESP_MOSAICO_GAME_HEIGHT 480
/* Grip orientation: dominate one axis, settle before turning, and keep the last
 * orientation while the device lies nearly flat (same policy as Vibe Mode). */
#define ORIENTATION_FLAT_Z 0.8f
#define ORIENTATION_AXIS_RATIO 1.7f
#define ORIENTATION_SETTLE_US 300000

static const char *TAG = "game_board";

struct raylib_lite_example_board {
    raylib_lite_example_board_config_t config;
    raylib_lite_platform_t services;
    raylib_lite_input_queue_t input;
    raylib_lite_input_event_t events[INPUT_CAPACITY];
    esp_lcd_touch_handle_t touch;
    esp_lcd_panel_handle_t panel;
    esp_display_presenter_t *presenter;
    mosaico_video_t *video;
    SemaphoreHandle_t mutex;
    /* Serializes panel rotation with touch coordinate mapping. */
    SemaphoreHandle_t present_lock;
    SemaphoreHandle_t touch_done;
    SemaphoreHandle_t imu_done;
    TaskHandle_t touch_task;
    TaskHandle_t imu_task;
    atomic_bool sampling;
    bool bmgr_initialized;
    int rotation;
    void *imu;
    /* One-shot warning so a dead accelerometer is visible in boot evidence. */
    uint32_t imu_errors;
};

static struct raylib_lite_example_board s_board;
static bool s_claimed;

/* Optional product input hardware, such as an expansion-slot controller. These
 * are weak references so a standalone Engine build without a product extension
 * still links; a product that defines them owns their lifecycle. */
extern esp_err_t raylib_lite_example_board_extra_input_start(
    raylib_lite_example_board_t *board) __attribute__((weak));
extern void raylib_lite_example_board_extra_input_stop(
    raylib_lite_example_board_t *board) __attribute__((weak));

static esp_err_t to_esp(raylib_lite_result_t result)
{
    switch (result) {
    case RAYLIB_LITE_OK:
        return ESP_OK;
    case RAYLIB_LITE_INVALID_ARGUMENT:
        return ESP_ERR_INVALID_ARG;
    case RAYLIB_LITE_INVALID_STATE:
        return ESP_ERR_INVALID_STATE;
    case RAYLIB_LITE_NO_MEMORY:
        return ESP_ERR_NO_MEM;
    case RAYLIB_LITE_NOT_SUPPORTED:
        return ESP_ERR_NOT_SUPPORTED;
    case RAYLIB_LITE_BUSY:
    case RAYLIB_LITE_TIMEOUT:
        return ESP_ERR_TIMEOUT;
    default:
        return ESP_FAIL;
    }
}

esp_err_t raylib_lite_example_board_haptic_init(void)
{
    return mosaico_hardware_haptic_set(0);
}

esp_err_t raylib_lite_example_board_haptic_set(uint8_t strength)
{
    return mosaico_hardware_haptic_set(strength);
}

static void lock_queue(void *ctx)
{
    (void)xSemaphoreTake(ctx, portMAX_DELAY);
}

static void unlock_queue(void *ctx)
{
    (void)xSemaphoreGive(ctx);
}

static uint64_t now_us(void *ctx)
{
    (void)ctx;
    return (uint64_t)esp_timer_get_time();
}

static void sleep_us(void *ctx, uint64_t us)
{
    (void)ctx;
    if (!us) {
        taskYIELD();
        return;
    }
    TickType_t ticks = pdMS_TO_TICKS((us + 999U) / 1000U);
    vTaskDelay(ticks ? ticks : 1);
}

static bool mosaico_touch_valid(int32_t x, int32_t y)
{
    return x >= 0 && x < ESP_MOSAICO_GAME_WIDTH &&
           y >= 0 && y < ESP_MOSAICO_GAME_HEIGHT;
}

static bool push_event(struct raylib_lite_example_board *board,
                       const raylib_lite_input_event_t *event)
{
    return raylib_lite_input_push(&board->input, event) == RAYLIB_LITE_OK;
}

esp_err_t raylib_lite_example_board_push_action(
    raylib_lite_example_board_t *board, int32_t action, bool pressed)
{
    if (!board || board != &s_board || action < 0 ||
            action >= RAYLIB_LITE_ACTION_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    const raylib_lite_input_event_t event = {
        .type = RAYLIB_LITE_INPUT_BUTTON,
        .value = action,
        .pressed = pressed,
        .timestamp_us = now_us(NULL),
    };
    return push_event(board, &event) ? ESP_OK : ESP_ERR_TIMEOUT;
}

/* Auto-rotation gives the touch controller the same swap/mirror matrix as the
 * panel, so its reports are already in the rotated logical frame. Map through
 * the presenter as well: it owns logical-surface geometry and is identity while
 * it stays at rotation 0. */
static bool touch_to_logical(struct raylib_lite_example_board *board,
                             int32_t *x, int32_t *y)
{
    if (xSemaphoreTake(board->present_lock, portMAX_DELAY) != pdTRUE) {
        return false;
    }
    const bool mapped =
        esp_display_presenter_map_point(board->presenter, x, y) == ESP_OK;
    xSemaphoreGive(board->present_lock);
    return mapped && mosaico_touch_valid(*x, *y);
}

/* Retry unqueued release events on the next poll rather than losing them
 * permanently when the bounded Game input queue fills. */
static void touch_worker(void *arg)
{
    struct raylib_lite_example_board *board = arg;
    uint8_t max = board->config.touch_points ? board->config.touch_points : 1;
    if (max > RAYLIB_LITE_ACTION_CONTACT_CAPACITY)
        max = RAYLIB_LITE_ACTION_CONTACT_CAPACITY;

    bool was_pressed = false;
    int32_t last_x = 0, last_y = 0;
    raylib_lite_input_contact_t old[RAYLIB_LITE_ACTION_CONTACT_CAPACITY] = {0};

    while (atomic_load_explicit(&board->sampling, memory_order_acquire)) {
        esp_lcd_touch_point_data_t points[RAYLIB_LITE_ACTION_CONTACT_CAPACITY] = {0};
        uint8_t count = 0;
        if (esp_lcd_touch_read_data(board->touch) == ESP_OK &&
                esp_lcd_touch_get_data(board->touch, points, &count, max) == ESP_OK) {
            if (count > max) count = max;
            uint64_t stamp = now_us(NULL);
            if (max == 1) {
                bool pressed = count > 0;
                int32_t x = last_x, y = last_y;
                if (pressed) {
                    int32_t px = points[0].x, py = points[0].y;
                    if (touch_to_logical(board, &px, &py)) {
                        x = px;
                        y = py;
                    } else {
                        pressed = false;
                    }
                }
                if ((pressed && (!was_pressed || x != last_x || y != last_y)) ||
                        (!pressed && was_pressed)) {
                    raylib_lite_input_event_t event = {
                        .type = RAYLIB_LITE_INPUT_POINTER,
                        .x = x, .y = y, .pressed = pressed,
                        .timestamp_us = stamp,
                    };
                    if (push_event(board, &event)) {
                        was_pressed = pressed;
                        last_x = x;
                        last_y = y;
                    }
                }
            } else {
                int32_t logical_x[RAYLIB_LITE_ACTION_CONTACT_CAPACITY] = {0};
                int32_t logical_y[RAYLIB_LITE_ACTION_CONTACT_CAPACITY] = {0};
                bool logical_valid[RAYLIB_LITE_ACTION_CONTACT_CAPACITY] = {0};
                for (uint8_t j = 0; j < count; j++) {
                    int32_t px = points[j].x, py = points[j].y;
                    logical_valid[j] = touch_to_logical(board, &px, &py);
                    logical_x[j] = px;
                    logical_y[j] = py;
                }
                /* Release absent contacts first; retained old[] entries retry
                 * when the queue cannot accept an important release edge. */
                for (size_t i = 0; i < RAYLIB_LITE_ACTION_CONTACT_CAPACITY; i++) {
                    if (!old[i].active) continue;
                    bool found = false;
                    for (uint8_t j = 0; j < count; j++) {
                        if (points[j].track_id == old[i].track_id &&
                                logical_valid[j]) {
                            found = true;
                            break;
                        }
                    }
                    if (!found) {
                        raylib_lite_input_event_t event = {
                            .type = RAYLIB_LITE_INPUT_TOUCH,
                            .x = old[i].x, .y = old[i].y,
                            .value = old[i].track_id, .pressed = false,
                            .timestamp_us = stamp,
                        };
                        if (push_event(board, &event)) old[i].active = false;
                    }
                }
                for (uint8_t j = 0; j < count; j++) {
                    if (!logical_valid[j]) continue;
                    const int32_t x = logical_x[j], y = logical_y[j];
                    int slot = -1;
                    for (size_t i = 0; i < RAYLIB_LITE_ACTION_CONTACT_CAPACITY; i++) {
                        if (old[i].active && old[i].track_id == points[j].track_id) {
                            slot = (int)i;
                            break;
                        }
                    }
                    if (slot >= 0 && old[slot].x == x && old[slot].y == y) continue;
                    if (slot < 0) {
                        for (size_t i = 0; i < RAYLIB_LITE_ACTION_CONTACT_CAPACITY; i++) {
                            if (!old[i].active) {
                                slot = (int)i;
                                break;
                            }
                        }
                    }
                    if (slot < 0) continue;
                    raylib_lite_input_event_t event = {
                        .type = RAYLIB_LITE_INPUT_TOUCH,
                        .x = x, .y = y, .value = points[j].track_id,
                        .pressed = true, .timestamp_us = stamp,
                    };
                    if (push_event(board, &event)) {
                        old[slot] = (raylib_lite_input_contact_t) {
                            .track_id = points[j].track_id,
                            .x = x, .y = y, .active = true,
                        };
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(12));
    }
    xSemaphoreGive(board->touch_done);
    vTaskDelete(NULL);
}

/* A rotating GRAM panel cannot be turned by the presenter: it rejects
 * non-zero rotation unless tearing-effect sync composes each tile into a full
 * framebuffer first, and one full 480x480 frame transfer (about 33 ms) is twice
 * the 16.8 ms panel frame, so that composition never finds a safe window. The
 * panel's own MADCTL axis matrix rotates the pixels during the existing DMA
 * writes instead: zero extra bandwidth, no full-frame buffer. These flag sets
 * are the CO5300 matrix for each angle, and the touch controller is given the
 * same matrix so its pixels and coordinates stay in agreement. */
typedef struct {
    bool swap_xy;
    bool mirror_x;
    bool mirror_y;
} panel_orientation_t;

static bool orientation_flags(int degrees, panel_orientation_t *out)
{
    switch (degrees) {
    case 0:
        *out = (panel_orientation_t) {false, false, false};
        return true;
    case 90:
        *out = (panel_orientation_t) {true, true, false};
        return true;
    case 180:
        *out = (panel_orientation_t) {false, true, true};
        return true;
    case 270:
        *out = (panel_orientation_t) {true, false, true};
        return true;
    default:
        return false;
    }
}

/* Caller holds present_lock; frame submission must be paused first. */
static esp_err_t apply_panel_orientation(struct raylib_lite_example_board *p,
                                         int degrees)
{
    panel_orientation_t flags = {0};
    if (!orientation_flags(degrees, &flags)) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(p->panel, flags.swap_xy),
                        TAG, "panel axis swap failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(p->panel, flags.mirror_x, flags.mirror_y),
                        TAG, "panel mirror failed");
    ESP_RETURN_ON_ERROR(esp_lcd_touch_set_swap_xy(p->touch, flags.swap_xy),
                        TAG, "touch axis swap failed");
    ESP_RETURN_ON_ERROR(esp_lcd_touch_set_mirror_x(p->touch, flags.mirror_x),
                        TAG, "touch X mirror failed");
    ESP_RETURN_ON_ERROR(esp_lcd_touch_set_mirror_y(p->touch, flags.mirror_y),
                        TAG, "touch Y mirror failed");
    return ESP_OK;
}

/* Turn the surface to follow the grip. The strip worker owns the panel, so
 * pause frame submission before the MADCTL change; a rejected change leaves the
 * previous orientation and presentation untouched. */
static bool apply_rotation(struct raylib_lite_example_board *p, int degrees)
{
    if (degrees == p->rotation) {
        return true;
    }
    raylib_lite_result_t paused = RAYLIB_LITE_INVALID_STATE;
    for (unsigned attempt = 0; attempt < 3 && paused != RAYLIB_LITE_OK; ++attempt) {
        paused = mosaico_video_pause(p->video, 100);
        if (paused != RAYLIB_LITE_OK) {
            /* The render task holds a frame while it draws; retry briefly. */
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
    if (paused != RAYLIB_LITE_OK) {
        ESP_LOGW(TAG, "display rotation=%d deferred: renderer busy", degrees);
        return false;
    }
    xSemaphoreTake(p->present_lock, portMAX_DELAY);
    const esp_err_t err = apply_panel_orientation(p, degrees);
    const int previous = p->rotation;
    if (err == ESP_OK) {
        p->rotation = degrees;
    } else {
        (void)apply_panel_orientation(p, previous);
    }
    xSemaphoreGive(p->present_lock);
    mosaico_video_resume(p->video);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "display rotation=%d failed: %s", degrees,
                 esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "display rotation=%d", degrees);
    return true;
}

/* Classify one accelerometer sample into a display rotation. Returns -1 while
 * the device lies nearly flat or the axes are ambiguous, so the caller keeps
 * the orientation it already has. */
static int orientation_from_accel(float x, float y, float z)
{
    const float ax = x < 0.0f ? -x : x;
    const float ay = y < 0.0f ? -y : y;
    const float az = z < 0.0f ? -z : z;
    if (az >= ORIENTATION_FLAT_Z) {
        return -1;
    }
    if (ax > ay * ORIENTATION_AXIS_RATIO) {
        return x > 0 ? 90 : 270;
    }
    if (ay > ax * ORIENTATION_AXIS_RATIO) {
        return y > 0 ? 0 : 180;
    }
    return -1;
}

static void imu_worker(void *ctx)
{
    struct raylib_lite_example_board *p = ctx;
    TickType_t period = pdMS_TO_TICKS(
        p->config.imu_sample_ms ? p->config.imu_sample_ms : 20);
    if (!period) {
        period = 1;
    }
    TickType_t wake = xTaskGetTickCount();
    int orientation = p->rotation;
    int candidate = orientation;
    int64_t candidate_since = esp_timer_get_time();
    bool sampled_before = false;
    int last_sample = 0;

    while (atomic_load_explicit(&p->sampling, memory_order_acquire)) {
        float x = 0;
        float y = 0;
        float z = 0;
        if (mosaico_hardware_imu_read(p->imu, &x, &y, &z) == ESP_OK) {
            const int sample = orientation_from_accel(x, y, z);
            /* Log transitions only: the first sample plus every change of the
             * classified grip, so a dead or miswired axis is still visible. */
            if (!sampled_before || sample != last_sample) {
                sampled_before = true;
                last_sample = sample;
                ESP_LOGI(TAG, "imu x=%.2f y=%.2f z=%.2f sample=%d rotation=%d",
                         x, y, z, sample, p->rotation);
            }
            raylib_lite_input_event_t e = {
                .type = RAYLIB_LITE_INPUT_IMU,
                .x = (int32_t)(x * 1000),
                .y = (int32_t)(y * 1000),
                .value = (int32_t)(z * 1000),
                .timestamp_us = now_us(NULL),
            };
            (void)push_event(p, &e);
            if (p->config.enable_auto_rotate) {
                const int64_t now = esp_timer_get_time();
                const int wanted = sample < 0 ? candidate : sample;
                if (wanted != candidate) {
                    candidate = wanted;
                    candidate_since = now;
                } else if (wanted != orientation &&
                           now - candidate_since >= ORIENTATION_SETTLE_US) {
                    if (apply_rotation(p, wanted)) {
                        orientation = wanted;
                    } else {
                        candidate_since = now;
                    }
                }
            }
        } else if (p->imu_errors++ == 0U) {
            ESP_LOGW(TAG, "imu read failed");
        }
        xTaskDelayUntil(&wake, period);
    }
    xSemaphoreGive(p->imu_done);
    vTaskDelete(NULL);
}

esp_err_t raylib_lite_example_board_create(const raylib_lite_example_board_config_t *c,
                                        raylib_lite_example_board_t **out)
{
    if (!c || !out) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = NULL;
    if (c->logical_width != ESP_MOSAICO_GAME_WIDTH ||
            c->logical_height != ESP_MOSAICO_GAME_HEIGHT) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (s_claimed) {
        return ESP_ERR_INVALID_STATE;
    }
    s_claimed = true;

    struct raylib_lite_example_board *p = &s_board;
    memset(p, 0, sizeof(*p));
    p->config = *c;
    atomic_init(&p->sampling, false);
    *out = p;

    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) {
        goto failed;
    }
    if ((err = mosaico_hardware_prepare()) != ESP_OK) goto failed;
    if ((err = esp_board_manager_init()) != ESP_OK) goto failed;
    p->bmgr_initialized = true;
    void *power = NULL;
    if ((err = esp_board_manager_get_device_handle("board_power", &power)) != ESP_OK || !power) {
        if (err == ESP_OK) err = ESP_ERR_INVALID_STATE;
        goto failed;
    }
    dev_display_lcd_handles_t *lcd = NULL;
    dev_lcd_touch_handles_t *touch = NULL;
    if ((err = esp_board_manager_get_device_handle("display_lcd", (void **)&lcd)) != ESP_OK ||
            (err = esp_board_manager_get_device_handle("lcd_touch", (void **)&touch)) != ESP_OK) {
        goto failed;
    }
    if (!lcd || !lcd->panel_handle || !lcd->io_handle || !touch || !touch->touch_handle) {
        err = ESP_ERR_INVALID_STATE;
        goto failed;
    }
    p->touch = touch->touch_handle;
    p->panel = lcd->panel_handle;
    esp_display_presenter_config_t present_config = {
        .width = ESP_MOSAICO_GAME_WIDTH,
        .height = ESP_MOSAICO_GAME_HEIGHT,
        .pixel_format = ESP_DISPLAY_PRESENT_PIXEL_FORMAT_RGB565,
        .max_damage_areas = 1,
        .target = {
            .hw = {
                .panel = lcd->panel_handle,
                .io = lcd->io_handle,
                .panel_type = ESP_DISPLAY_PRESENT_PANEL_IO,
                .input_pixel_format = ESP_DISPLAY_PRESENT_PIXEL_FORMAT_RGB565,
                /* Grip rotation is applied by the panel's MADCTL matrix, not
                 * by the presenter, so the logical surface stays unrotated
                 * and TE composition (which cannot meet this bus's frame
                 * budget) stays off. */
                .rotation = ESP_DISPLAY_PRESENT_ROTATE_0,
                .swap_bytes = true,
                .te_enabled = false,
                .te_sync = ESP_DISPLAY_PRESENT_TE_SYNC_DISABLED(),
            },
            .fb = {.mode = ESP_DISPLAY_PRESENT_MODE_NONE},
            .drawbuf = {
                .lines = c->drawbuf_lines ? c->drawbuf_lines : 34,
                .buffers = c->drawbuf_count ? c->drawbuf_count : 2,
                .in_psram = false,
            },
        },
    };
    if ((err = esp_display_presenter_create(&present_config, &p->presenter)) != ESP_OK) {
        goto failed;
    }

    /* Optional IMU upload starts only after display transport is ready. */
    if (c->enable_imu) {
        if ((err = esp_board_manager_init_device_by_name("imu_sensor")) != ESP_OK ||
                (err = esp_board_manager_get_device_handle("imu_sensor", &p->imu)) != ESP_OK) {
            goto failed;
        }
        float px = 0, py = 0, pz = 0;
        const esp_err_t probe = mosaico_hardware_imu_read(p->imu, &px, &py, &pz);
        ESP_LOGI(TAG, "imu probe: %s x=%.2f y=%.2f z=%.2f",
                 esp_err_to_name(probe), px, py, pz);
    }

    /* Seed the rotation before the first frame so a held device starts upright
     * instead of showing frames at the wrong orientation. No worker is running
     * yet, so the panel change needs no pause. */
    if (c->enable_auto_rotate && p->imu) {
        float x = 0, y = 0, z = 0;
        const int initial = mosaico_hardware_imu_read(p->imu, &x, &y, &z) == ESP_OK
                            ? orientation_from_accel(x, y, z) : -1;
        if (initial > 0 && apply_panel_orientation(p, initial) == ESP_OK) {
            p->rotation = initial;
            ESP_LOGI(TAG, "display rotation=%d", initial);
        }
    }

    raylib_lite_result_t rr = mosaico_video_open(
        p->presenter, ESP_MOSAICO_GAME_WIDTH, ESP_MOSAICO_GAME_HEIGHT, &p->video);
    if (rr != RAYLIB_LITE_OK) {
        err = to_esp(rr);
        goto failed;
    }

    p->mutex = xSemaphoreCreateMutex();
    p->present_lock = xSemaphoreCreateMutex();
    p->touch_done = xSemaphoreCreateBinary();
    p->imu_done = xSemaphoreCreateBinary();
    if (!p->mutex || !p->present_lock || !p->touch_done || !p->imu_done) {
        err = ESP_ERR_NO_MEM;
        goto failed;
    }

    raylib_lite_input_sync_t sync = {
        .context = p->mutex,
        .lock = lock_queue,
        .unlock = unlock_queue,
    };
    rr = raylib_lite_input_queue_init(&p->input, p->events, INPUT_CAPACITY, &sync);
    if (rr != RAYLIB_LITE_OK) {
        err = to_esp(rr);
        goto failed;
    }

    p->services = (raylib_lite_platform_t){
        .video = mosaico_video_backend(p->video),
        .clock = {
            .monotonic_us = now_us,
            .sleep_for_us = sleep_us,
        },
        .audio = NULL,
    };
    return ESP_OK;

failed:
    {
        esp_err_t cleanup = raylib_lite_example_board_retry_cleanup(p, 3000);
        if (cleanup == ESP_OK) *out = NULL;
        return cleanup == ESP_OK ? err : cleanup;
    }
}

const raylib_lite_platform_t *raylib_lite_example_board_services(
    raylib_lite_example_board_t *p)
{
    return p ? &p->services : NULL;
}

raylib_lite_input_queue_t *raylib_lite_example_board_input(raylib_lite_example_board_t *p)
{
    return p ? &p->input : NULL;
}

esp_err_t raylib_lite_example_board_start_input(raylib_lite_example_board_t *p)
{
    if (!p || p != &s_board || !s_claimed || !p->input.initialized ||
            p->touch_task || p->imu_task || atomic_load(&p->sampling)) {
        return ESP_ERR_INVALID_STATE;
    }
    atomic_store(&p->sampling, true);
    if (p->config.enable_auto_rotate && p->imu) {
        float x = 0, y = 0, z = 0;
        const int initial = mosaico_hardware_imu_read(p->imu, &x, &y, &z) == ESP_OK
                            ? orientation_from_accel(x, y, z) : -1;
        if (initial > 0) {
            (void)apply_rotation(p, initial);
        }
    }
    if (xTaskCreate(touch_worker, "game_touch", 4096, p, 5,
                    &p->touch_task) != pdPASS) {
        goto fail;
    }
    if (p->config.enable_imu &&
            xTaskCreate(imu_worker, "game_imu", 4096, p, 5,
                        &p->imu_task) != pdPASS) {
        goto fail;
    }
    if (raylib_lite_example_board_extra_input_start) {
        const esp_err_t err = raylib_lite_example_board_extra_input_start(p);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "product input unavailable: %s", esp_err_to_name(err));
        }
    }
    return ESP_OK;

fail:
    atomic_store(&p->sampling, false);
    return ESP_ERR_NO_MEM;
}

esp_err_t raylib_lite_example_board_stop(raylib_lite_example_board_t *p,
                                      uint32_t timeout_ms)
{
    if (!p) {
        return ESP_ERR_INVALID_ARG;
    }
    atomic_store(&p->sampling, false);
    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
    if (p->touch_task) {
        if (xSemaphoreTake(p->touch_done, ticks) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
        p->touch_task = NULL;
    }
    if (p->imu_task) {
        if (xSemaphoreTake(p->imu_done, ticks) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
        p->imu_task = NULL;
    }
    if (raylib_lite_example_board_extra_input_stop) {
        raylib_lite_example_board_extra_input_stop(p);
    }
    return ESP_OK;
}

esp_err_t raylib_lite_example_board_retry_cleanup(raylib_lite_example_board_t *p,
                                               uint32_t timeout_ms)
{
    if (!p || p != &s_board) return ESP_ERR_INVALID_ARG;
    raylib_lite_result_t audio_result = raylib_lite_game_audio_shutdown(timeout_ms);
    if (audio_result != RAYLIB_LITE_OK) return to_esp(audio_result);
    esp_err_t err = raylib_lite_example_board_stop(p, timeout_ms);
    if (err != ESP_OK) {
        return err;
    }
    if (p->video) {
        raylib_lite_result_t rr = mosaico_video_close(p->video, timeout_ms);
        if (rr != RAYLIB_LITE_OK) {
            return to_esp(rr);
        }
        p->video = NULL;
    }
    if (p->presenter) {
        err = esp_display_presenter_delete(p->presenter);
        if (err != ESP_OK) return err;
        p->presenter = NULL;
    }
    if (p->bmgr_initialized) {
        err = esp_board_manager_deinit();
        if (err != ESP_OK) return err;
        p->bmgr_initialized = false;
        p->touch = NULL;
        p->imu = NULL;
    }
    err = mosaico_hardware_release();
    if (err != ESP_OK) return err;
    if (p->input.initialized) {
        raylib_lite_input_queue_deinit(&p->input);
    }
    if (p->mutex) {
        vSemaphoreDelete(p->mutex);
        p->mutex = NULL;
    }
    if (p->touch_done) {
        vSemaphoreDelete(p->touch_done);
        p->touch_done = NULL;
    }
    if (p->imu_done) {
        vSemaphoreDelete(p->imu_done);
        p->imu_done = NULL;
    }
    if (p->present_lock) {
        vSemaphoreDelete(p->present_lock);
        p->present_lock = NULL;
    }
    s_claimed = false;
    return ESP_OK;
}

esp_err_t raylib_lite_example_board_destroy(raylib_lite_example_board_t *p,
                                         uint32_t timeout_ms)
{
    if (!p || p != &s_board) {
        return ESP_ERR_INVALID_ARG;
    }
    return raylib_lite_example_board_retry_cleanup(p, timeout_ms);
}
