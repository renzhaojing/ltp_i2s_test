#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_common.h" // I2S通用API
#include "soc/soc_caps.h"      // SOC能力定义

// 检查是否支持PDM RX
#if SOC_I2S_SUPPORTS_PDM_RX
#include "driver/i2s_pdm.h"   // PDM API (参考pdm_mic_test.c)
#else
// ESP32-C3不支持PDM RX API
#endif
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_sleep.h"

static const char *TAG = "PDM_MIC_TEST";

/*
 * ESP32-C3 PDM麦克风配置说明：
 * - 参考: cursor_mic_pdm/pdm_c_code/main/pdm_mic_test.c
 * - 使用PDM专用API (driver/i2s_pdm.h)
 * - PDM引脚配置：GPIO19(CLK), GPIO9(DATA)
 * - 初始化顺序：i2s_new_channel() -> i2s_channel_init_pdm_rx_mode() -> i2s_channel_enable()
 * - PDM模式：只使用CLK和DATA，不使用WS和SCK
 */

// PDM麦克风引脚配置
#define PDM_MIC_CLK_PIN     GPIO_NUM_19   // PDM时钟引脚 (CLK) - 连接到WS pin
#define PDM_MIC_DATA_PIN    GPIO_NUM_9    // PDM数据引脚 (DATA) - 连接到SD pin

// 音频参数
#define SAMPLE_RATE         16000   // 使用16kHz采样率
#define BITS_PER_SAMPLE     16
#define BUFFER_SIZE         2048     // 读取缓冲区大小（样本数）
#define DMA_BUF_LEN         512      // DMA缓冲区长度（必须 <= 1024，且 > 8）
#define RECORD_DURATION_MS  10000    // 10秒录音

// 音频缓冲区
static int16_t mic_buffer[BUFFER_SIZE];    // 16位MIC输入缓冲区

// I2S通道句柄
static i2s_chan_handle_t pdm_rx_handle = NULL;

// 状态标志
static volatile bool is_recording = false;

// 新版PDM API不需要通道格式定义






/**
 * @brief 初始化PDM麦克风 (参考pdm_mic_test.c)
 * 注意: ESP32-C3不支持PDM RX API，此函数会返回错误
 */
static esp_err_t init_pdm_microphone(void)
{
    ESP_LOGI(TAG, "初始化PDM麦克风 (ESP32-C3)...");
    ESP_LOGI(TAG, "引脚配置: CLK=%d, DATA=%d", PDM_MIC_CLK_PIN, PDM_MIC_DATA_PIN);
    ESP_LOGI(TAG, "参考: cursor_mic_pdm/pdm_c_code/main/pdm_mic_test.c (ESP32-S3)");
    
#if !SOC_I2S_SUPPORTS_PDM_RX
    ESP_LOGE(TAG, "================================================");
    ESP_LOGE(TAG, "错误: ESP32-C3不支持PDM RX API");
    ESP_LOGE(TAG, "================================================");
    ESP_LOGE(TAG, "原因: SOC配置中未定义SOC_I2S_SUPPORTS_PDM_RX");
    ESP_LOGE(TAG, "参考代码pdm_mic_test.c是针对ESP32-S3的，ESP32-S3支持PDM RX");
    ESP_LOGE(TAG, "");
    ESP_LOGE(TAG, "解决方案:");
    ESP_LOGE(TAG, "1. 使用支持PDM RX的芯片（如ESP32-S3、ESP32）");
    ESP_LOGE(TAG, "2. 使用I2S数字麦克风（如SPH0645L）代替PDM麦克风");
    ESP_LOGE(TAG, "3. 使用外部PDM到PCM转换芯片");
    ESP_LOGE(TAG, "参考: https://github.com/espressif/esp-idf/issues/8796");
    ESP_LOGE(TAG, "================================================");
    return ESP_ERR_NOT_SUPPORTED;
#else
    ESP_LOGI(TAG, "硬件检查建议:");
    ESP_LOGI(TAG, "  1. 确认PDM麦克风电源为3.3V (VDD)");
    ESP_LOGI(TAG, "  2. 确认GND连接正确");
    ESP_LOGI(TAG, "  3. 确认CLK/DATA引脚连接正确");
    ESP_LOGI(TAG, "  4. 确认引脚未被其他功能占用");

    // I2S通道配置 (参考pdm_mic_test.c)
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &pdm_rx_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "创建I2S通道失败: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "I2S通道创建成功");

    // PDM RX配置 (参考pdm_mic_test.c)
    i2s_pdm_rx_config_t pdm_rx_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = PDM_MIC_CLK_PIN,   // PDM时钟引脚
            .din = PDM_MIC_DATA_PIN,  // PDM数据输入引脚
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };

    err = i2s_channel_init_pdm_rx_mode(pdm_rx_handle, &pdm_rx_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "初始化PDM RX模式失败: %s", esp_err_to_name(err));
        i2s_del_channel(pdm_rx_handle);
        pdm_rx_handle = NULL;
        return err;
    }
    ESP_LOGI(TAG, "PDM RX模式初始化成功");

    err = i2s_channel_enable(pdm_rx_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "启用I2S通道失败: %s", esp_err_to_name(err));
        i2s_channel_disable(pdm_rx_handle);
        i2s_del_channel(pdm_rx_handle);
        pdm_rx_handle = NULL;
        return err;
    }
    ESP_LOGI(TAG, "I2S通道已启用");

    ESP_LOGI(TAG, "PDM麦克风初始化成功 (采样率: %d Hz)", SAMPLE_RATE);
    return ESP_OK;
#endif
}



/**
 * @brief PDM数据读取和打印任务 (参考pdm_mic_test.c)
 */
static void mic_test_task(void *arg)
{
    ESP_LOGI(TAG, "PDM麦克风测试任务启动");

    size_t bytes_read = 0;
    uint32_t total_samples = 0;
    uint32_t start_time = esp_timer_get_time() / 1000;

    ESP_LOGI(TAG, "开始读取PDM麦克风数据...");

    while (is_recording) {
        // 从PDM麦克风读取数据 (参考pdm_mic_test.c)
        esp_err_t ret = i2s_channel_read(pdm_rx_handle, (void *)mic_buffer,
                                         BUFFER_SIZE * sizeof(int16_t),
                                         &bytes_read, portMAX_DELAY);

        if (ret == ESP_OK && bytes_read > 0) {
            size_t samples_read = bytes_read / sizeof(int16_t);
            total_samples += samples_read;

            // 显示进度和音频统计 (参考pdm_mic_test.c)
            uint32_t elapsed = (esp_timer_get_time() / 1000) - start_time;
            
            // 计算音频电平
            int64_t sum_squares = 0;
            int16_t max_sample = 0;
            int16_t min_sample = INT16_MAX;

            for (size_t i = 0; i < samples_read; i++) {
                int16_t sample = mic_buffer[i];
                int16_t abs_sample = (sample < 0) ? -sample : sample;
                sum_squares += (int64_t)sample * sample;
                if (abs_sample > max_sample) {
                    max_sample = abs_sample;
                }
                if (sample < min_sample) {
                    min_sample = sample;
                }
            }

            float rms = sqrt((float)sum_squares / samples_read);

            // 每秒打印一次统计信息
            if (elapsed % 1000 == 0 && elapsed > 0) {
                ESP_LOGI(TAG, "进度: %" PRIu32 " ms | 样本: %" PRIu32 " | RMS: %.0f | 峰值: %d | 最小值: %d", 
                         elapsed, total_samples, rms, max_sample, min_sample);
            }

            // 打印前16个样本（每100ms打印一次，避免过于频繁）
            if (elapsed % 100 == 0) {
                ESP_LOGI(TAG, "PDM数据样本 (前16个，16位格式):");
                for (size_t i = 0; i < 16 && i < samples_read; i++) {
                    int16_t sample = mic_buffer[i];
                    ESP_LOGI(TAG, "  [%zu]: 16bit=%d (0x%04X)", 
                             i, sample, (uint16_t)sample);
                }
            }
            
            // 判断PDM数据是否正常
            if (rms > 100.0f && rms < 10000.0f) {
                // 数据正常，不打印（避免过于频繁）
            } else if (rms < 10.0f) {
                ESP_LOGW(TAG, "⚠ RMS值过小 (%.0f)，可能是静音或硬件连接问题", rms);
            } else if (rms > 10000.0f) {
                ESP_LOGW(TAG, "⚠ RMS值过大 (%.0f)，可能数据溢出", rms);
            }

        } else if (ret != ESP_OK) {
            ESP_LOGE(TAG, "读取PDM麦克风数据失败: %s", esp_err_to_name(ret));
        }

        // 检查录音时长
        uint32_t current_time = esp_timer_get_time() / 1000;
        if (current_time - start_time >= RECORD_DURATION_MS) {
            ESP_LOGI(TAG, "PDM麦克风测试完成，总时长: %" PRIu32 " ms", current_time - start_time);
            break;
        }

        // 小延迟，避免打印过于频繁
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    is_recording = false;

    // 最终统计信息
    uint32_t total_duration = (esp_timer_get_time() / 1000) - start_time;
    ESP_LOGI(TAG, "=== PDM麦克风测试结束统计 ===");
    ESP_LOGI(TAG, "总时长: %" PRIu32 " ms", total_duration);
    ESP_LOGI(TAG, "总样本数: %" PRIu32, total_samples);
    ESP_LOGI(TAG, "实际采样率: %.1f Hz", (float)total_samples / (total_duration / 1000.0f));
    ESP_LOGI(TAG, "==========================");

    ESP_LOGI(TAG, "PDM麦克风测试任务结束");
    vTaskDelete(NULL);
}

/**
 * @brief 开始PDM麦克风测试
 */
static void start_mic_test(void)
{
    ESP_LOGI(TAG, "开始PDM麦克风测试...");

    is_recording = true;

    // 创建PDM麦克风测试任务
    xTaskCreate(mic_test_task, "pdm_mic_test_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "PDM麦克风测试已启动，测试时长: %d ms", RECORD_DURATION_MS);
}

/**
 * @brief 清理I2S/PDM资源 (参考pdm_mic_test.c)
 */
static void cleanup_i2s(void)
{
    if (pdm_rx_handle != NULL) {
        i2s_channel_disable(pdm_rx_handle);
        i2s_del_channel(pdm_rx_handle);
        pdm_rx_handle = NULL;
        ESP_LOGI(TAG, "I2S/PDM资源已清理");
    }
}


/**
 * @brief 主函数
 */
void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-C3 PDM麦克风测试程序启动");
    ESP_LOGI(TAG, "=== ESP32-C3 PDM麦克风测试配置 ===");
    ESP_LOGI(TAG, "采样率: %d Hz, 位深: %d bit", SAMPLE_RATE, BITS_PER_SAMPLE);
    ESP_LOGI(TAG, "缓冲区大小: %d 样本", BUFFER_SIZE);
    ESP_LOGI(TAG, "测试时长: %d ms", RECORD_DURATION_MS);
    ESP_LOGI(TAG, "开发板: ESP32-C3 (RISC-V架构)");
    ESP_LOGI(TAG, "麦克风类型: PDM数字麦克风");
#if SOC_I2S_SUPPORTS_PDM_RX
    ESP_LOGI(TAG, "API: PDM专用API (driver/i2s_pdm.h)");
    ESP_LOGI(TAG, "模式: PDM RX模式");
#else
    ESP_LOGI(TAG, "API: 不支持PDM RX API");
    ESP_LOGI(TAG, "模式: ESP32-C3不支持PDM RX");
#endif
    ESP_LOGI(TAG, "数据格式: 16位 (直接读取)");
    ESP_LOGI(TAG, "通信格式: I2S_COMM_FORMAT_STAND_I2S (标准I2S)");
    ESP_LOGI(TAG, "PDM麦克风引脚配置:");
    ESP_LOGI(TAG, "  CLK: GPIO%d (连接到WS pin)", PDM_MIC_CLK_PIN);
    ESP_LOGI(TAG, "  DATA: GPIO%d (连接到SD pin)", PDM_MIC_DATA_PIN);
    ESP_LOGI(TAG, "注意: PDM模式不使用BCK引脚");
    ESP_LOGI(TAG, "===================================");

    // 初始化PDM麦克风
    esp_err_t init_err = init_pdm_microphone();
    if (init_err != ESP_OK) {
        ESP_LOGE(TAG, "================================================");
        ESP_LOGE(TAG, "PDM麦克风初始化失败！");
        ESP_LOGE(TAG, "错误代码: %s (0x%x)", esp_err_to_name(init_err), init_err);
        ESP_LOGE(TAG, "================================================");
        if (init_err == ESP_ERR_NOT_SUPPORTED) {
            ESP_LOGE(TAG, "原因: ESP32-C3不支持PDM RX API");
            ESP_LOGE(TAG, "参考代码pdm_mic_test.c是针对ESP32-S3的");
        } else {
            ESP_LOGE(TAG, "原因: PDM麦克风初始化失败");
            ESP_LOGE(TAG, "可能的原因:");
            ESP_LOGE(TAG, "1. 硬件连接错误 (CLK/DATA引脚)");
            ESP_LOGE(TAG, "2. 引脚与其他功能冲突");
            ESP_LOGE(TAG, "3. ESP-IDF版本不包含PDM RX支持");
        }
        ESP_LOGE(TAG, "参考: https://github.com/espressif/esp-idf/issues/8796");
        ESP_LOGE(TAG, "================================================");
        ESP_LOGE(TAG, "程序将退出...");
        return;  // 优雅退出，不调用abort
    }

    // 等待系统稳定
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 开始PDM麦克风测试
    ESP_LOGI(TAG, "=== 开始PDM麦克风测试 ===");
    start_mic_test();

    // 等待测试完成
    vTaskDelay(pdMS_TO_TICKS(RECORD_DURATION_MS + 1000));

    // 清理资源
    cleanup_i2s();

    ESP_LOGI(TAG, "ESP32-C3 PDM麦克风测试程序执行完成");
    ESP_LOGI(TAG, "测试结果说明:");
    ESP_LOGI(TAG, "如果看到持续变化的16位音频数据，说明PDM麦克风工作正常");
    ESP_LOGI(TAG, "正常情况下应该看到RMS值在100-5000范围内变化");
#if SOC_I2S_SUPPORTS_PDM_RX
    ESP_LOGI(TAG, "ESP32-C3 PDM功能说明:");
    ESP_LOGI(TAG, "- 使用PDM专用API (driver/i2s_pdm.h)");
    ESP_LOGI(TAG, "- 参考代码: cursor_mic_pdm/pdm_c_code/main/pdm_mic_test.c");
#else
    ESP_LOGI(TAG, "ESP32-C3 PDM功能说明:");
    ESP_LOGI(TAG, "- ESP32-C3不支持PDM RX API (SOC配置限制)");
    ESP_LOGI(TAG, "- 参考代码pdm_mic_test.c是针对ESP32-S3的");
    ESP_LOGI(TAG, "- ESP32-S3支持PDM RX，但ESP32-C3不支持");
#endif
    ESP_LOGI(TAG, "- 参考: https://github.com/espressif/esp-idf/issues/8796");
#if !SOC_I2S_SUPPORTS_PDM_RX
    ESP_LOGI(TAG, "ESP32-C3不支持PDM RX API，无法测试PDM麦克风");
    ESP_LOGI(TAG, "建议:");
    ESP_LOGI(TAG, "1. 使用支持PDM RX的芯片（如ESP32-S3、ESP32）");
    ESP_LOGI(TAG, "2. 使用I2S数字麦克风（如SPH0645L）代替PDM麦克风");
#else
    ESP_LOGI(TAG, "如果数据始终为0，可能的原因:");
    ESP_LOGI(TAG, "1. 硬件连接错误 (CLK/DATA引脚)");
    ESP_LOGI(TAG, "2. 麦克风电源供电不足 (需要3.3V)");
    ESP_LOGI(TAG, "3. 引脚与其他功能冲突");
#endif
    ESP_LOGI(TAG, "建议: 检查硬件连接、电源供应，或考虑使用I2S数字麦克风");

    // 进入深度睡眠
    ESP_LOGI(TAG, "进入深度睡眠...");
    // esp_deep_sleep_start();
} 