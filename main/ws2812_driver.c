#include "ws2812_driver.h"
#include "driver/rmt_encoder.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "WS2812";

// WS2812时序参数（单位：ns）
#define WS2812_T0H_NS   (350)  // 0码高电平时间
#define WS2812_T0L_NS   (1000) // 0码低电平时间
#define WS2812_T1H_NS   (1000) // 1码高电平时间
#define WS2812_T1L_NS   (350)  // 1码低电平时间
#define WS2812_RES_NS   (280000) // 复位时间（>50us）

// RMT编码器上下文
typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} rmt_led_strip_encoder_t;

// WS2812句柄结构
struct ws2812_handle_s {
    rmt_channel_handle_t rmt_chan;
    rmt_encoder_handle_t led_encoder;
    uint8_t *led_buffer;
    uint32_t led_num;
};

static size_t rmt_encode_led_strip(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                    const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = led_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = led_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    
    switch (led_encoder->state) {
    case 0: // 发送RGB数据
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = 1; // 切换到发送复位码状态
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // 如果内存满了，退出
        }
        // fall-through
    case 1: // 发送复位码
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_encoder->reset_code,
                                                sizeof(led_encoder->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = RMT_ENCODING_RESET; // 回到初始状态
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // 如果内存满了，退出
        }
        break;
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder)
{
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_del_encoder(led_encoder->bytes_encoder);
    rmt_del_encoder(led_encoder->copy_encoder);
    free(led_encoder);
    return ESP_OK;
}

static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_reset(led_encoder->bytes_encoder);
    rmt_encoder_reset(led_encoder->copy_encoder);
    led_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

esp_err_t ws2812_init(gpio_num_t gpio_num, uint32_t led_num, ws2812_handle_t **handle)
{
    ESP_LOGI(TAG, "初始化WS2812，GPIO=%d，LED数量=%lu", gpio_num, led_num);
    
    ws2812_handle_t *ws2812 = calloc(1, sizeof(ws2812_handle_t));
    ESP_RETURN_ON_FALSE(ws2812 != NULL, ESP_ERR_NO_MEM, TAG, "分配内存失败");
    
    ws2812->led_num = led_num;
    ws2812->led_buffer = calloc(led_num * 3, sizeof(uint8_t));
    ESP_RETURN_ON_FALSE(ws2812->led_buffer != NULL, ESP_ERR_NO_MEM, TAG, "分配LED缓冲区失败");
    
    // RMT配置
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = gpio_num,
        .mem_block_symbols = 64,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .trans_queue_depth = 4,
    };
    esp_err_t ret = rmt_new_tx_channel(&tx_chan_config, &ws2812->rmt_chan);
    ESP_RETURN_ON_ERROR(ret, TAG, "创建RMT通道失败");
    
    // 创建字节编码器
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = WS2812_T0H_NS * 10 / 1000, // 转换为tick（10MHz）
            .level1 = 0,
            .duration1 = WS2812_T0L_NS * 10 / 1000,
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = WS2812_T1H_NS * 10 / 1000,
            .level1 = 0,
            .duration1 = WS2812_T1L_NS * 10 / 1000,
        },
        .flags.msb_first = 1,
    };
    rmt_encoder_handle_t bytes_encoder = NULL;
    ret = rmt_new_bytes_encoder(&bytes_encoder_config, &bytes_encoder);
    ESP_RETURN_ON_ERROR(ret, TAG, "创建字节编码器失败");
    
    // 创建复制编码器（用于复位码）
    rmt_copy_encoder_config_t copy_encoder_config = {};
    rmt_encoder_handle_t copy_encoder = NULL;
    ret = rmt_new_copy_encoder(&copy_encoder_config, &copy_encoder);
    ESP_RETURN_ON_ERROR(ret, TAG, "创建复制编码器失败");
    
    // 创建LED条带编码器
    rmt_led_strip_encoder_t *led_encoder = calloc(1, sizeof(rmt_led_strip_encoder_t));
    ESP_RETURN_ON_FALSE(led_encoder != NULL, ESP_ERR_NO_MEM, TAG, "分配编码器内存失败");
    
    led_encoder->base.encode = rmt_encode_led_strip;
    led_encoder->base.del = rmt_del_led_strip_encoder;
    led_encoder->base.reset = rmt_led_strip_encoder_reset;
    led_encoder->bytes_encoder = bytes_encoder;
    led_encoder->copy_encoder = copy_encoder;
    led_encoder->state = RMT_ENCODING_RESET;
    
    // 复位码
    led_encoder->reset_code = (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = WS2812_RES_NS / 10,
        .level1 = 0,
        .duration1 = 0,
    };
    
    ws2812->led_encoder = (rmt_encoder_handle_t)led_encoder;
    
    // 使能RMT通道
    ret = rmt_enable(ws2812->rmt_chan);
    ESP_RETURN_ON_ERROR(ret, TAG, "使能RMT通道失败");
    
    *handle = ws2812;
    ESP_LOGI(TAG, "WS2812初始化成功");
    return ESP_OK;
}

esp_err_t ws2812_set_pixel(ws2812_handle_t *handle, uint32_t index, uint8_t r, uint8_t g, uint8_t b)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "句柄为空");
    ESP_RETURN_ON_FALSE(index < handle->led_num, ESP_ERR_INVALID_ARG, TAG, "LED索引超出范围");
    
    // WS2812使用GRB格式
    uint32_t offset = index * 3;
    handle->led_buffer[offset + 0] = g;
    handle->led_buffer[offset + 1] = r;
    handle->led_buffer[offset + 2] = b;
    
    return ESP_OK;
}

esp_err_t ws2812_clear(ws2812_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "句柄为空");
    memset(handle->led_buffer, 0, handle->led_num * 3);
    return ESP_OK;
}

esp_err_t ws2812_refresh(ws2812_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "句柄为空");
    
    // 重置编码器状态
    rmt_encoder_reset(handle->led_encoder);
    
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };
    
    // 发送LED数据（编码器会自动添加复位码）
    esp_err_t ret = rmt_transmit(handle->rmt_chan, handle->led_encoder, 
                                  handle->led_buffer, handle->led_num * 3, &tx_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "发送LED数据失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 等待传输完成（包括复位码）
    ret = rmt_tx_wait_all_done(handle->rmt_chan, portMAX_DELAY);  // 无限等待直到完成
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "等待传输完成失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

esp_err_t ws2812_deinit(ws2812_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (handle->led_encoder != NULL) {
        rmt_del_encoder(handle->led_encoder);
    }
    
    if (handle->rmt_chan != NULL) {
        rmt_disable(handle->rmt_chan);
        rmt_del_channel(handle->rmt_chan);
    }
    
    if (handle->led_buffer != NULL) {
        free(handle->led_buffer);
    }
    
    free(handle);
    return ESP_OK;
}
