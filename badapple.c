/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/scanvideo.h"
#include "pico/scanvideo/composable_scanline.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "ff.h"
#include "f_util.h"
#include "hw_config.h"

#define VGA_MODE vga_mode_320x240_60
extern const struct scanvideo_pio_program video_24mhz_composable;

static void frame_update_logic();
static void render_scanline(struct scanvideo_scanline_buffer *dest, int core);

static int16_t buf_1[256] = {0};
static int16_t buf_2[256] = {0};
static uint8_t * vbuf = NULL;
static int vbuf_size = 0;
static uint8_t * vbuf_pos;
static int16_t * volatile buf = buf_1;
static int16_t * obuf = buf_1;
static FIL audio, video;

bool shouldFlip = false;
bool w = false;
int n = 0, total = 0;

struct semaphore video_setup_complete;

struct WAVE {
    char magic[4];
    uint32_t size;
    char type[4];
    char chunk1type[4];
    uint32_t chunk1size;
    uint16_t format;
    uint16_t nchannels;
    uint32_t samplerate;
    uint32_t byterate;
    uint16_t blockalign;
    uint16_t bitdepth;
    char chunk2type[4];
    uint32_t datasize;
} metadata;
struct MVID {
    char magic[4];
    uint16_t width;
    uint16_t height;
    uint16_t fps;
    uint16_t framecount;
} video_info;
int bytedepth;
bool running = true;

// "Worker thread" for each core
void render_loop() {
    static uint32_t last_frame_num = 0;
    int core_num = get_core_num();
    printf("Rendering on core %d\n", core_num);
    while (running) {
        struct scanvideo_scanline_buffer *scanline_buffer = scanvideo_begin_scanline_generation(true);

        render_scanline(scanline_buffer, core_num);

        // Release the rendered buffer into the wild
        scanvideo_end_scanline_generation(scanline_buffer);

        uint32_t frame_num = scanvideo_frame_number(scanline_buffer->scanline_id);
        uint16_t line_num = scanvideo_scanline_number(scanline_buffer->scanline_id);
        
        if (line_num >= VGA_MODE.height - 32 && frame_num != last_frame_num) {
            last_frame_num = frame_num;
            frame_update_logic();
            scanvideo_wait_for_vblank();
        }
    }
}

void core1_func() {
    scanvideo_setup(&VGA_MODE);
    scanvideo_timing_enable(true);
    sem_acquire_blocking(&video_setup_complete);
    for (int i = 0; i < metadata.datasize / bytedepth; i += metadata.nchannels) {
        absolute_time_t start = get_absolute_time();
        if (i % (sizeof(buf_1) / bytedepth) == 0) {
            if (buf == buf_1) buf = buf_2;
            else buf = buf_1;
        }
        if (bytedepth == 1) pwm_set_gpio_level(26, ((uint8_t*)buf)[i % sizeof(buf_1)] >> 0);
        else pwm_set_gpio_level(26, ((int)buf[i % (sizeof(buf_1) / bytedepth)] + 32768) >> 8);
        absolute_time_t next = delayed_by_us(start, 990000 / metadata.samplerate);
        if (!time_reached(next)) busy_wait_until(next);
    }
}

int vga_main(void) {
    sem_init(&video_setup_complete, 0, 1);

    multicore_launch_core1(core1_func);

    gpio_init(26);
    gpio_set_function(26, GPIO_FUNC_PWM);
    pwm_config conf = pwm_get_default_config();
    conf.top = 512;
    pwm_config_set_clkdiv(&conf, 1);
    pwm_init(pwm_gpio_to_slice_num(26), &conf, true);
    
    sd_card_t *pSD = sd_get_by_num(0);
    pwm_set_gpio_level(26, 0);
    sleep_ms(5000);
    FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
    printf("done\n");
    if (FR_OK != fr) panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);

    fr = f_open(&audio, "badapple.wav", FA_READ);
    if (FR_OK != fr && FR_EXIST != fr)
        panic("f_open(badapple.wav) error: %s (%d)\n", FRESULT_str(fr), fr);
    UINT read = 0;
    fr = f_read(&audio, &metadata, sizeof(metadata), &read);
    if (read != sizeof(metadata)) panic("could not read audio file\n");
    bytedepth = metadata.bitdepth / 8;
    fr = f_read(&audio, buf, sizeof(buf_1), &read);
    if (fr != FR_OK || read != sizeof(buf_1)) return 0;

    fr = f_open(&video, "badapple.mvd", FA_READ);
    if (FR_OK != fr && FR_EXIST != fr)
        panic("f_open(badapple.mvd) error: %s (%d)\n", FRESULT_str(fr), fr);
    fr = f_read(&video, &video_info, sizeof(video_info), &read);
    if (read != sizeof(video_info)) panic("could not read video file\n");
    uint16_t size = 0;
    f_read(&video, &size, 2, &read);
    if (size > vbuf_size) vbuf = realloc(vbuf, size);
    f_read(&video, vbuf, size, &read);
    if (read < size) panic("no more video!");
    vbuf_pos = vbuf;

    sem_release(&video_setup_complete);
    render_loop();
    
    f_close(&video);
    f_close(&audio);
    f_unmount("/");
    pwm_set_gpio_level(26, 0);

    gpio_put(25, false);

    return 0;
}

void frame_update_logic() {
    total += n;
    UINT read = 0;
    while (total < video_info.width * video_info.height) {
        n = *vbuf_pos++;
        if (n & 0x80) {
            n = (n & 0x7F) | ((int)*vbuf_pos++ << 7);
        }
        int c = n > video_info.width * video_info.height - total ? video_info.width * video_info.height - total : n;
        n -= c;
        total += c;
    }

    shouldFlip = false;
    w = false;
    n = 0;
    total = 0;

    if (buf != obuf) {
        UINT read = 0;
        FRESULT fr = f_read(&audio, obuf, sizeof(buf_1), &read);
        if (fr != FR_OK || read != sizeof(buf_1)) return;
        obuf = buf;
    }

    uint16_t size = 0;
    f_read(&video, &size, 2, &read);
    if (read < 2) {
        running = false;
        return;
    }
    if (size > vbuf_size) vbuf = realloc(vbuf, size);
    f_read(&video, vbuf, size, &read);
    if (read < size) panic("no more video!");
    vbuf_pos = vbuf;
}

int32_t single_color_scanline(uint32_t *buf, size_t buf_length, int width, uint32_t color16) {
    assert(buf_length >= 2);

    assert(width >= 3);
    // | jmp color_run | color | count-3 |  buf[0] =
    buf[0] = COMPOSABLE_COLOR_RUN | (color16 << 16);
    buf[1] = (width - 3) | (COMPOSABLE_RAW_1P << 16);
    // note we must end with a black pixel
    buf[2] = 0 | (COMPOSABLE_EOL_ALIGN << 16);

    return 3;
}

void render_scanline(struct scanvideo_scanline_buffer *dest, int core) {
    uint16_t *buf = (uint16_t*)dest->data;
    size_t buf_length = dest->data_max;

    int x = 0, i = 0;
    char tmp = 0;
    UINT read = 0;
    while (x < VGA_MODE.width) {
        if (i + 1 >= buf_length * 2) panic("Not enough space for line! Limit is %d\n", buf_length * 2);
        if (total >= video_info.width * video_info.height) {
            dest->data_used = single_color_scanline((uint32_t*)buf, buf_length, VGA_MODE.width, 0);
            dest->status = SCANLINE_OK;
            return;
        }
        while (n <= 0) {
            if (shouldFlip) w = !w;
            n = *vbuf_pos++;
            if (n & 0x80) {
                n = (n & 0x7F) | ((int)*vbuf_pos++ << 7);
            }
            shouldFlip = n < 32767;
        }
        int c = n > VGA_MODE.width - x ? VGA_MODE.width - x : n;
        n -= c;
        x += c;
        total += c;
        switch (c) {
        case 1:
            buf[i++] = COMPOSABLE_RAW_1P;
            buf[i++] = w ? 0xFFFF : 0;
            break;
        case 2:
            buf[i++] = COMPOSABLE_RAW_2P;
            buf[i++] = w ? 0xFFFF : 0;
            buf[i++] = w ? 0xFFFF : 0;
            break;
        default:
            buf[i++] = COMPOSABLE_COLOR_RUN;
            buf[i++] = w ? 0xFFFF : 0;
            buf[i++] = c - 3;
            break;
        }
    }
    buf[i++] = COMPOSABLE_RAW_1P;
    buf[i++] = 0;
    if (i % 2 == 0) {
        buf[i++] = COMPOSABLE_EOL_SKIP_ALIGN;
        buf[i++] = 0;
    } else buf[i++] = COMPOSABLE_EOL_ALIGN;
    dest->data_used = i / 2;
    dest->status = SCANLINE_OK;
}


int main(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_function(16, GPIO_FUNC_SPI);
    gpio_set_function(17, GPIO_FUNC_SPI);
    gpio_set_function(18, GPIO_FUNC_SPI);
    gpio_set_function(19, GPIO_FUNC_SPI);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, true);
    stdio_init_all();
    set_spi_dma_irq_channel(true, false);

    return vga_main();
}
