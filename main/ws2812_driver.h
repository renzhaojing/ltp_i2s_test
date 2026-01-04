#ifndef WS2812_DRIVER_H
#define WS2812_DRIVER_H

#include <stdint.h>
#include "driver/rmt_tx.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ws2812_handle_s ws2812_handle_t;

/**
 * @brief 初始化WS2812 LED灯带
 * 
 * @param gpio_num GPIO引脚号
 * @param led_num LED数量
 * @param handle 返回的句柄指针
 * @return esp_err_t 
 */
esp_err_t ws2812_init(gpio_num_t gpio_num, uint32_t led_num, ws2812_handle_t **handle);

/**
 * @brief 设置LED颜色
 * 
 * @param handle WS2812句柄
 * @param index LED索引（0开始）
 * @param r 红色分量（0-255）
 * @param g 绿色分量（0-255）
 * @param b 蓝色分量（0-255）
 * @return esp_err_t 
 */
esp_err_t ws2812_set_pixel(ws2812_handle_t *handle, uint32_t index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 清除所有LED（设置为黑色）
 * 
 * @param handle WS2812句柄
 * @return esp_err_t 
 */
esp_err_t ws2812_clear(ws2812_handle_t *handle);

/**
 * @brief 刷新LED显示（发送数据到LED灯带）
 * 
 * @param handle WS2812句柄
 * @return esp_err_t 
 */
esp_err_t ws2812_refresh(ws2812_handle_t *handle);

/**
 * @brief 释放WS2812资源
 * 
 * @param handle WS2812句柄
 * @return esp_err_t 
 */
esp_err_t ws2812_deinit(ws2812_handle_t *handle);

#ifdef __cplusplus
}
#endif

#endif // WS2812_DRIVER_H

