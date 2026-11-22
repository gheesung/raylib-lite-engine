// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stdint.h>
#include "esp_display_present.h"
#include "raylib_lite_video.h"

typedef struct mosaico_video mosaico_video_t;

raylib_lite_result_t mosaico_video_open(
    esp_display_presenter_t *presenter, uint16_t width, uint16_t height,
    mosaico_video_t **out);
raylib_lite_video_backend_t mosaico_video_backend(mosaico_video_t *video);
/* Stop submitting to the presenter while keeping the renderer running: frames
 * completed during the pause are dropped. Pause waits for in-flight strip work
 * and the presenter fence so configuration such as rotation can be rebound. */
raylib_lite_result_t mosaico_video_pause(
    mosaico_video_t *video, uint32_t timeout_ms);
void mosaico_video_resume(mosaico_video_t *video);
raylib_lite_result_t mosaico_video_close(
    mosaico_video_t *video, uint32_t timeout_ms);
