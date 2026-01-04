#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"      // 旧版I2S API头文件 (与WLED兼容)
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_sleep.h"

static const char *TAG = "I2S_MIC_TEST";

/*
 * ESP32-C3 SPH0645L麦克风配置说明：
 * - 使用WLED兼容的旧版I2S API (i2s_driver_install)
 * - ESP32-C3不支持PDM模式，只能使用标准I2S模式
 * - 引脚配置：GPIO19(WS), GPIO8(SCK), GPIO9(DATA)
 * - 参考WLED项目的I2SSource和SPH0654类实现
 * - 初始化顺序：i2s_driver_install() -> i2s_set_pin() -> i2s_set_clk()
 */

// ESP32-C3 I2S麦克风引脚配置 (SPH0645L专用)
// ESP32-C3 I2S0引脚 (推荐配置)
#define I2S_MIC_WS_PIN      GPIO_NUM_19   // 字时钟引脚 (WS) - GPIO4
#define I2S_MIC_SCK_PIN     GPIO_NUM_8   // 位时钟引脚 (SCK/BCLK) - GPIO5
#define I2S_MIC_DIN_PIN     GPIO_NUM_9   // 数据输入引脚 (SD/DATA) - GPIO6

// 音频参数
#define SAMPLE_RATE         16000   // 使用16kHz采样率
#define BITS_PER_SAMPLE     16
#define BUFFER_SIZE         2048
#define RECORD_DURATION_MS  10000    // 10秒录音

// 音频缓冲区
static int16_t mic_buffer[BUFFER_SIZE];    // 16位MIC输入缓冲区

// I2S配置结构体（旧版API）
static i2s_config_t i2s_config;
static i2s_pin_config_t i2s_pin_config;

// 状态标志
static volatile bool is_recording = false;






/**
 * @brief 初始化SPH0645L麦克风 (ESP32-C3专用配置，参考WLED实现)
 */
static esp_err_t init_i2s_microphone(void)
{
    ESP_LOGI(TAG, "初始化SPH0645L I2S麦克风 (ESP32-C3)...");
    ESP_LOGI(TAG, "引脚配置: WS=%d, SCK=%d, DIN=%d", I2S_MIC_WS_PIN, I2S_MIC_SCK_PIN, I2S_MIC_DIN_PIN);
    ESP_LOGI(TAG, "使用旧版I2S API (与WLED兼容)");

    // 先卸载可能存在的I2S驱动（参考WLED做法）
    i2s_driver_uninstall(I2S_NUM_0);
    vTaskDelay(pdMS_TO_TICKS(100));  // 给麦克风一些时间设置

    // 配置I2S（参考WLED的I2SSource配置）
    i2s_config = (i2s_config_t){
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // 单声道，左通道（参考WLED）
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 2, 0)
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,  // 标准I2S格式（SPH0645需要）
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2,
        .dma_buf_count = 8,           // DMA缓冲区数量（参考WLED）
        .dma_buf_len = BUFFER_SIZE,   // DMA缓冲区长度
        .use_apll = false,            // ESP32-C3不支持APLL
        .bits_per_chan = I2S_BITS_PER_CHAN_16BIT,
#else
        .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = BUFFER_SIZE,
        .use_apll = false
#endif
    };

    // 配置I2S引脚
    i2s_pin_config = (i2s_pin_config_t){
        .bck_io_num = I2S_MIC_SCK_PIN,      // 位时钟
        .ws_io_num = I2S_MIC_WS_PIN,        // 字时钟
        .data_out_num = I2S_PIN_NO_CHANGE,   // 不使用输出
        .data_in_num = I2S_MIC_DIN_PIN      // 数据输入
    };

    // 步骤1: 安装I2S驱动
    esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "安装I2S驱动失败: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "I2S驱动安装成功");

    // 步骤2: 设置I2S引脚
    err = i2s_set_pin(I2S_NUM_0, &i2s_pin_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "设置I2S引脚失败: %s", esp_err_to_name(err));
        i2s_driver_uninstall(I2S_NUM_0);
        return err;
    }
    ESP_LOGI(TAG, "I2S引脚配置成功");

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 2, 0)
    // 步骤3: 设置I2S时钟（关键步骤，必须在i2s_set_pin之后调用）
    // 参考WLED: i2s_set_clk() after i2s_set_pin() for SPH0645
    err = i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "设置I2S时钟失败: %s", esp_err_to_name(err));
        i2s_driver_uninstall(I2S_NUM_0);
        return err;
    }
    ESP_LOGI(TAG, "I2S时钟配置成功 (采样率: %d Hz)", SAMPLE_RATE);
#endif

    ESP_LOGI(TAG, "SPH0645L I2S麦克风初始化完成");
    ESP_LOGI(TAG, "配置: 标准I2S格式, 16位, 单声道左通道, DMA缓冲区8x%d", BUFFER_SIZE);
    
    return ESP_OK;
}



/**
 * @brief 麦克风数据读取和打印任务
 */
static void mic_test_task(void *arg)
{
    ESP_LOGI(TAG, "麦克风测试任务启动");

    size_t bytes_read = 0;
    uint32_t total_samples = 0;
    uint32_t start_time = esp_timer_get_time() / 1000;

    ESP_LOGI(TAG, "开始读取麦克风数据...");

    while (is_recording) {
        // 从I2S麦克风读取数据 (16位数据) - 旧版API (参考WLED)
        esp_err_t ret = i2s_read(I2S_NUM_0, (void *)mic_buffer,
                                 BUFFER_SIZE * sizeof(int16_t),
                                 &bytes_read, portMAX_DELAY);

        if (ret == ESP_OK && bytes_read > 0) {
            size_t samples_read = bytes_read / sizeof(int16_t);
            total_samples += samples_read;

            // 打印每个缓冲区的麦克风数据
            ESP_LOGI(TAG, "麦克风数据包 #%" PRIu32 ": %zu 样本", total_samples / samples_read, samples_read);

            // 打印前16个样本
            ESP_LOGI(TAG, "数据样本 (前16个):");
            for (size_t i = 0; i < 16 && i < samples_read; i++) {
                ESP_LOGI(TAG, "  [%zu]: %d", i, mic_buffer[i]);
            }

            // 计算音频统计信息
            int32_t sum_squares = 0;
            int16_t max_sample = INT16_MIN;
            int16_t min_sample = INT16_MAX;

            for (size_t i = 0; i < samples_read; i++) {
                int16_t sample = mic_buffer[i];
                sum_squares += (int32_t)sample * sample;
                if (sample > max_sample) max_sample = sample;
                if (sample < min_sample) min_sample = sample;
            }

            float rms = sqrt((float)sum_squares / samples_read);
            float avg_level = (float)sum_squares / samples_read;

            ESP_LOGI(TAG, "统计信息 - RMS: %.2f, 平均功率: %.0f, 峰值: %d/%d",
                     rms, avg_level, min_sample, max_sample);

        } else if (ret != ESP_OK) {
            ESP_LOGE(TAG, "读取麦克风数据失败: %s", esp_err_to_name(ret));
        }

        // 检查录音时长
        uint32_t current_time = esp_timer_get_time() / 1000;
        if (current_time - start_time >= RECORD_DURATION_MS) {
            ESP_LOGI(TAG, "麦克风测试完成，总时长: %" PRIu32 " ms", current_time - start_time);
            break;
        }

        // 小延迟，避免打印过于频繁
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    is_recording = false;

    // 最终统计信息
    uint32_t total_duration = (esp_timer_get_time() / 1000) - start_time;
    ESP_LOGI(TAG, "=== 麦克风测试结束统计 ===");
    ESP_LOGI(TAG, "总时长: %" PRIu32 " ms", total_duration);
    ESP_LOGI(TAG, "总样本数: %" PRIu32, total_samples);
    ESP_LOGI(TAG, "实际采样率: %.1f Hz", (float)total_samples / (total_duration / 1000.0f));
    ESP_LOGI(TAG, "==========================");

    ESP_LOGI(TAG, "麦克风测试任务结束");
    vTaskDelete(NULL);
}

/**
 * @brief 开始麦克风测试
 */
static void start_mic_test(void)
{
    ESP_LOGI(TAG, "开始麦克风测试...");

    is_recording = true;

    // 创建麦克风测试任务
    xTaskCreate(mic_test_task, "mic_test_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "麦克风测试已启动，测试时长: %d ms", RECORD_DURATION_MS);
}

/**
 * @brief 清理I2S资源 (旧版API)
 */
static void cleanup_i2s(void)
{
    esp_err_t err = i2s_driver_uninstall(I2S_NUM_0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "卸载I2S驱动失败: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "I2S资源已清理");
    }
}


/**
 * @brief 主函数
 */
void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-C3 SPH0645L麦克风测试程序启动");
    ESP_LOGI(TAG, "=== ESP32-C3 SPH0645L麦克风测试配置 ===");
    ESP_LOGI(TAG, "采样率: %d Hz, 位深: %d bit", SAMPLE_RATE, BITS_PER_SAMPLE);
    ESP_LOGI(TAG, "缓冲区大小: %d 样本", BUFFER_SIZE);
    ESP_LOGI(TAG, "测试时长: %d ms", RECORD_DURATION_MS);
    ESP_LOGI(TAG, "开发板: ESP32-C3 (RISC-V架构)");
    ESP_LOGI(TAG, "麦克风: SPH0645L (I2S数字麦克风)");
    ESP_LOGI(TAG, "API: 旧版I2S (i2s_driver_install) - 参考WLED实现");
    ESP_LOGI(TAG, "通道格式: I2S_CHANNEL_FMT_ONLY_LEFT (单声道左通道)");
    ESP_LOGI(TAG, "通信格式: I2S_COMM_FORMAT_STAND_I2S (标准I2S)");
    ESP_LOGI(TAG, "麦克风引脚配置:");
    ESP_LOGI(TAG, "  SD(DATA): GPIO%d", I2S_MIC_DIN_PIN);
    ESP_LOGI(TAG, "  WS: GPIO%d", I2S_MIC_WS_PIN);
    ESP_LOGI(TAG, "  SCK(BCLK): GPIO%d", I2S_MIC_SCK_PIN);
    ESP_LOGI(TAG, "===================================");

    // 初始化I2S麦克风
    ESP_ERROR_CHECK(init_i2s_microphone());

    // 等待系统稳定
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 开始麦克风测试
    ESP_LOGI(TAG, "=== 开始麦克风测试 ===");
    start_mic_test();

    // 等待测试完成
    vTaskDelay(pdMS_TO_TICKS(RECORD_DURATION_MS + 1000));

    // 清理资源
    cleanup_i2s();

    ESP_LOGI(TAG, "ESP32-C3 SPH0645L麦克风测试程序执行完成");
    ESP_LOGI(TAG, "测试结果说明:");
    ESP_LOGI(TAG, "如果看到持续变化的16位音频数据，说明SPH0645L工作正常");
    ESP_LOGI(TAG, "正常情况下应该看到RMS值在100-5000范围内变化");
    ESP_LOGI(TAG, "ESP32-C3 I2S功能说明:");
    ESP_LOGI(TAG, "- 不支持PDM模式，只能使用标准I2S");
    ESP_LOGI(TAG, "- I2S功能可能有性能局限性");
    ESP_LOGI(TAG, "如果数据始终为0，可能的原因:");
    ESP_LOGI(TAG, "1. 硬件连接错误 (SD/WS/SCK引脚)");
    ESP_LOGI(TAG, "2. 麦克风电源供电不足 (需要3.3V)");
    ESP_LOGI(TAG, "3. ESP32-C3 I2S配置不兼容");
    ESP_LOGI(TAG, "4. 引脚与其他功能冲突");
    ESP_LOGI(TAG, "建议: 检查硬件连接和电源供应");

    // 进入深度睡眠
    ESP_LOGI(TAG, "进入深度睡眠...");
    // esp_deep_sleep_start();
} 