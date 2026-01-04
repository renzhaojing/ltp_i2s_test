#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_task_wdt.h"

static const char *TAG = "ADC_MIC_TEST";

/*
 * ESP32-C3 模拟麦克风（ADC）配置说明：
 * - 使用ADC直接读取模拟麦克风数据
 * - 当前硬件使用GPIO2连接模拟麦克风 (ADC_CHANNEL_2)
 * - 采样率：通过定时器控制，约16kHz
 * - ESP32-C3的ADC1支持GPIO0-4，共5个通道
 */

// 模拟麦克风ADC引脚配置
// 注意：ESP32-C3的ADC1只支持GPIO0-4，不支持GPIO18
// 当前硬件使用GPIO2连接模拟麦克风
#define ADC_MIC_PIN            GPIO_NUM_2   // ADC输入引脚 (GPIO2 = ADC_CHANNEL_2)
#define ADC_MIC_CHANNEL        ADC_CHANNEL_2  // ADC通道2 (新API使用ADC_CHANNEL_X)

// GPIO到ADC通道映射表（ESP32-C3）
// GPIO0 -> ADC1_CHANNEL_0
// GPIO1 -> ADC1_CHANNEL_1
// GPIO2 -> ADC1_CHANNEL_2
// GPIO3 -> ADC1_CHANNEL_3
// GPIO4 -> ADC1_CHANNEL_4
// GPIO18 -> 不支持ADC（需要使用外部ADC芯片）

// 音频参数
// 注意：使用软件定时器控制采样率，实际采样率可能远低于目标值
// FreeRTOS的vTaskDelay最小延迟约1ms，因此实际采样率最高约1000Hz
// 如需更高采样率，建议使用硬件定时器或DMA方式
#define SAMPLE_RATE            8000    // 目标采样率8kHz（软件定时器实际约500-1000Hz）
#define BITS_PER_SAMPLE        12      // ADC为12位
#define BUFFER_SIZE            1024    // 读取缓冲区大小（样本数）- 减小以更快显示统计
#define RECORD_DURATION_MS     10000   // 10秒录音
#define ADC_SAMPLE_INTERVAL_US (1000000 / SAMPLE_RATE)  // 采样间隔（微秒）

// 音频缓冲区
static int16_t mic_buffer[BUFFER_SIZE];    // 16位MIC输入缓冲区（12位ADC数据转换为16位）

// ADC句柄和校准参数
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool adc_calibration_init = false;

// 状态标志
static volatile bool is_recording = false;

/**
 * @brief 初始化ADC模拟麦克风
 */
static esp_err_t init_adc_microphone(void)
{
    ESP_LOGI(TAG, "初始化ADC模拟麦克风 (ESP32-C3)...");
    ESP_LOGI(TAG, "ADC输入引脚: GPIO%d (ADC_CHANNEL_%d)", ADC_MIC_PIN, ADC_MIC_CHANNEL);
    ESP_LOGI(TAG, "硬件检查建议:");
    ESP_LOGI(TAG, "  1. 确认模拟麦克风电源为3.3V (VDD)");
    ESP_LOGI(TAG, "  2. 确认GND连接正确");
    ESP_LOGI(TAG, "  3. 确认ADC输入引脚连接正确 (GPIO%d)", ADC_MIC_PIN);
    ESP_LOGI(TAG, "  4. 建议在ADC输入引脚添加0.1uF对地滤波电容");

    // 配置ADC1单元（使用新的ADC API）
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t err = adc_oneshot_new_unit(&init_config, &adc1_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "创建ADC1单元失败: %s", esp_err_to_name(err));
        return err;
    }

    // 配置ADC通道
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,  // 12dB衰减，支持0-2500mV范围（替代已弃用的11dB）
    };
    err = adc_oneshot_config_channel(adc1_handle, ADC_MIC_CHANNEL, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "配置ADC通道失败: %s", esp_err_to_name(err));
        adc_oneshot_del_unit(adc1_handle);
        adc1_handle = NULL;
        return err;
    }

    // 初始化ADC校准
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    err = adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle);
    if (err == ESP_OK) {
        adc_calibration_init = true;
        ESP_LOGI(TAG, "ADC校准: 使用曲线拟合方案");
    } else {
        ESP_LOGW(TAG, "ADC校准初始化失败: %s，将使用原始ADC值", esp_err_to_name(err));
        adc_calibration_init = false;
    }

    ESP_LOGI(TAG, "ADC模拟麦克风初始化成功");
    ESP_LOGI(TAG, "配置: 12位ADC, 12dB衰减, 采样率: %d Hz", SAMPLE_RATE);
    ESP_LOGI(TAG, "有效测量范围: 0 ~ 2500 mV");
    
    return ESP_OK;
}

/**
 * @brief ADC数据读取和打印任务
 */
static void mic_test_task(void *arg)
{
    // 将当前任务添加到看门狗监控（如果还没有添加）
    // 注意：如果任务已经在看门狗监控中，这个调用会被忽略
    esp_task_wdt_add(NULL);
    
    ESP_LOGI(TAG, "ADC模拟麦克风测试任务启动");

    uint32_t total_samples = 0;
    uint32_t start_time = esp_timer_get_time() / 1000;
    int64_t last_sample_time = esp_timer_get_time();

    ESP_LOGI(TAG, "开始读取ADC模拟麦克风数据...");
    ESP_LOGI(TAG, "采样间隔: %d 微秒 (目标采样率: %d Hz)", ADC_SAMPLE_INTERVAL_US, SAMPLE_RATE);

    size_t buffer_idx = 0;

    while (is_recording) {
        // 控制采样率：等待到下一个采样时间
        int64_t current_time = esp_timer_get_time();
        int64_t elapsed = current_time - last_sample_time;
        
        if (elapsed < ADC_SAMPLE_INTERVAL_US) {
            // 使用vTaskDelay让出CPU时间，避免看门狗超时
            // 注意：FreeRTOS的vTaskDelay最小延迟约1个tick（通常10ms），无法精确控制微秒级延迟
            // 实际采样率会略低于目标值，但可以避免CPU占用过高和看门狗超时
            int64_t wait_us = ADC_SAMPLE_INTERVAL_US - elapsed;
            if (wait_us >= 1000) {
                // 如果等待时间大于等于1ms，使用vTaskDelay
                vTaskDelay(pdMS_TO_TICKS(wait_us / 1000));
            } else {
                // 如果等待时间小于1ms，至少延迟1个tick让出CPU给IDLE任务
                // 这样可以避免看门狗超时
                vTaskDelay(1);
            }
            continue;
        }
        
        // 定期重置看门狗（每读取一定次数后重置）
        static uint32_t wdt_counter = 0;
        if (++wdt_counter >= 50) {  // 每50次ADC读取重置一次看门狗
            esp_task_wdt_reset();
            wdt_counter = 0;
        }

        // 读取ADC值（使用新的ADC API）
        int adc_raw = 0;
        esp_err_t ret = adc_oneshot_read(adc1_handle, ADC_MIC_CHANNEL, &adc_raw);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "读取ADC失败: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // 转换为电压值（mV）- 如果校准可用
        int voltage_mv = 0;
        if (adc_calibration_init) {
            adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage_mv);
        } else {
            // 如果没有校准，使用估算值
            voltage_mv = (adc_raw * 2500) / 4095;
        }

        // 将12位ADC值（0-4095）转换为16位有符号整数（-2048到2047）
        // ADC值范围：0-4095，中心值：2048
        int16_t sample = (int16_t)(adc_raw - 2048);  // 转换为有符号，以2048为中心

        // 存储到缓冲区
        mic_buffer[buffer_idx] = sample;
        buffer_idx++;
        total_samples++;

        // 当缓冲区满时，处理并打印数据
        if (buffer_idx >= BUFFER_SIZE) {
            // 计算音频统计信息
            int64_t sum_squares = 0;
            int16_t max_sample = INT16_MIN;
            int16_t min_sample = INT16_MAX;
            uint32_t sum_voltage = 0;

            for (size_t i = 0; i < BUFFER_SIZE; i++) {
                int16_t s = mic_buffer[i];
                sum_squares += (int64_t)s * s;
                if (s > max_sample) max_sample = s;
                if (s < min_sample) min_sample = s;
                // 计算原始电压（从样本值反推）
                sum_voltage += (mic_buffer[i] + 2048) * 2500 / 4095;  // 估算电压
            }

            float rms = sqrt((float)sum_squares / BUFFER_SIZE);
            float avg_voltage = (float)sum_voltage / BUFFER_SIZE;

            uint32_t elapsed = (esp_timer_get_time() / 1000) - start_time;

            // 每秒打印一次统计信息（或缓冲区满时打印）
            static uint32_t last_print_time = 0;
            if (elapsed - last_print_time >= 1000 || elapsed < 1000) {
                int16_t peak_to_peak = max_sample - min_sample;  // 峰峰值
                ESP_LOGI(TAG, "进度: %" PRIu32 " ms | 样本: %" PRIu32 " | RMS: %.1f | 峰值: %d/%d (峰峰值: %d) | 平均电压: %.1f mV", 
                         elapsed, total_samples, rms, min_sample, max_sample, peak_to_peak, avg_voltage);
                last_print_time = elapsed;
                
                // 判断数据变化情况
                if (peak_to_peak < 10) {
                    ESP_LOGW(TAG, "⚠ 数据变化很小（峰峰值: %d），可能是静音或硬件连接问题", peak_to_peak);
                } else if (peak_to_peak > 100) {
                    ESP_LOGI(TAG, "✓ 检测到明显的声音信号（峰峰值: %d）", peak_to_peak);
                }
            }

            // 打印前16个样本（每100ms打印一次）
            if (elapsed % 100 == 0) {
                ESP_LOGI(TAG, "ADC数据样本 (前16个，12位ADC转换为16位):");
                for (size_t i = 0; i < 16 && i < BUFFER_SIZE; i++) {
                    int16_t sample = mic_buffer[i];
                    int adc_raw_val = sample + 2048;  // 转换回原始ADC值
                    uint32_t voltage = (adc_raw_val * 2500) / 4095;  // 估算电压
                    ESP_LOGI(TAG, "  [%zu]: ADC=%d (0x%03X) -> 16bit=%d (0x%04X) -> ~%" PRIu32 "mV", 
                             i, adc_raw_val, adc_raw_val, sample, (uint16_t)sample, (uint32_t)voltage);
                }
            }

            // 判断ADC数据是否正常（已在上面打印时判断）

            buffer_idx = 0;  // 重置缓冲区索引
        }

        last_sample_time = current_time;

        // 检查录音时长
        uint32_t current_time_ms = esp_timer_get_time() / 1000;
        if (current_time_ms - start_time >= RECORD_DURATION_MS) {
            ESP_LOGI(TAG, "ADC模拟麦克风测试完成，总时长: %" PRIu32 " ms", current_time_ms - start_time);
            break;
        }
    }

    is_recording = false;

    // 最终统计信息
    uint32_t total_duration = (esp_timer_get_time() / 1000) - start_time;
    float actual_sample_rate = (float)total_samples / (total_duration / 1000.0f);
    ESP_LOGI(TAG, "=== ADC模拟麦克风测试结束统计 ===");
    ESP_LOGI(TAG, "总时长: %" PRIu32 " ms", total_duration);
    ESP_LOGI(TAG, "总样本数: %" PRIu32, total_samples);
    ESP_LOGI(TAG, "实际采样率: %.1f Hz (目标: %d Hz)", actual_sample_rate, SAMPLE_RATE);
    ESP_LOGI(TAG, "==========================");

    ESP_LOGI(TAG, "ADC模拟麦克风测试任务结束");
    
    // 从看门狗监控中移除任务
    esp_task_wdt_delete(NULL);
    
    vTaskDelete(NULL);
}

/**
 * @brief 开始ADC模拟麦克风测试
 */
static void start_mic_test(void)
{
    ESP_LOGI(TAG, "开始ADC模拟麦克风测试...");

    is_recording = true;

    // 创建ADC模拟麦克风测试任务
    xTaskCreate(mic_test_task, "adc_mic_test_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "ADC模拟麦克风测试已启动，测试时长: %d ms", RECORD_DURATION_MS);
}

/**
 * @brief 清理ADC资源
 */
static void cleanup_adc(void)
{
    if (adc_calibration_init && adc1_cali_handle != NULL) {
        adc_cali_delete_scheme_curve_fitting(adc1_cali_handle);
        adc1_cali_handle = NULL;
    }
    if (adc1_handle != NULL) {
        adc_oneshot_del_unit(adc1_handle);
        adc1_handle = NULL;
    }
    ESP_LOGI(TAG, "ADC资源已清理");
}

/**
 * @brief 主函数
 */
void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-C3 ADC模拟麦克风测试程序启动");
    ESP_LOGI(TAG, "=== ESP32-C3 ADC模拟麦克风测试配置 ===");
    ESP_LOGI(TAG, "采样率: %d Hz, ADC位深: %d bit", SAMPLE_RATE, BITS_PER_SAMPLE);
    ESP_LOGI(TAG, "缓冲区大小: %d 样本", BUFFER_SIZE);
    ESP_LOGI(TAG, "测试时长: %d ms", RECORD_DURATION_MS);
    ESP_LOGI(TAG, "开发板: ESP32-C3 (RISC-V架构)");
    ESP_LOGI(TAG, "麦克风类型: 模拟麦克风（ADC输入）");
    ESP_LOGI(TAG, "ADC配置:");
    ESP_LOGI(TAG, "  ADC输入引脚: GPIO%d (ADC_CHANNEL_%d)", ADC_MIC_PIN, ADC_MIC_CHANNEL);
    ESP_LOGI(TAG, "  衰减: 12dB (有效范围: 0-2500mV)");
    ESP_LOGI(TAG, "  分辨率: 12位");
    ESP_LOGI(TAG, "  采样率: %d Hz", SAMPLE_RATE);
    ESP_LOGI(TAG, "===================================");

    // 初始化ADC模拟麦克风
    esp_err_t init_err = init_adc_microphone();
    if (init_err != ESP_OK) {
        ESP_LOGE(TAG, "================================================");
        ESP_LOGE(TAG, "ADC模拟麦克风初始化失败！");
        ESP_LOGE(TAG, "错误代码: %s (0x%x)", esp_err_to_name(init_err), init_err);
        ESP_LOGE(TAG, "================================================");
        ESP_LOGE(TAG, "可能的原因:");
        ESP_LOGE(TAG, "1. GPIO引脚不支持ADC（ESP32-C3只支持GPIO0-4）");
        ESP_LOGE(TAG, "2. 硬件连接错误");
        ESP_LOGE(TAG, "3. ADC配置错误");
        ESP_LOGE(TAG, "================================================");
        ESP_LOGE(TAG, "程序将退出...");
        return;
    }

    // 等待系统稳定
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 开始ADC模拟麦克风测试
    ESP_LOGI(TAG, "=== 开始ADC模拟麦克风测试 ===");
    start_mic_test();

    // 等待测试完成
    vTaskDelay(pdMS_TO_TICKS(RECORD_DURATION_MS + 1000));

    // 清理资源
    cleanup_adc();

    ESP_LOGI(TAG, "ESP32-C3 ADC模拟麦克风测试程序执行完成");
    ESP_LOGI(TAG, "测试结果说明:");
    ESP_LOGI(TAG, "如果看到持续变化的ADC数据，说明模拟麦克风工作正常");
    ESP_LOGI(TAG, "正常情况下应该看到:");
    ESP_LOGI(TAG, "  - RMS值在10-500范围内变化（取决于声音大小）");
    ESP_LOGI(TAG, "  - 电压值在合理范围内变化（通常几百到一千多mV）");
    ESP_LOGI(TAG, "  - 样本值围绕0上下波动");
    ESP_LOGI(TAG, "ESP32-C3 ADC功能说明:");
    ESP_LOGI(TAG, "- ADC1支持GPIO0-4，共5个通道");
    ESP_LOGI(TAG, "- 12位分辨率，0-4095范围");
    ESP_LOGI(TAG, "- 11dB衰减支持0-2500mV测量范围");
    ESP_LOGI(TAG, "- 采样率通过软件定时器控制，实际采样率可能略低于目标值");
    ESP_LOGI(TAG, "如果数据始终为0或不变，可能的原因:");
    ESP_LOGI(TAG, "1. 硬件连接错误（ADC输入引脚）");
    ESP_LOGI(TAG, "2. 麦克风电源供电不足");
    ESP_LOGI(TAG, "3. GPIO引脚不支持ADC（ESP32-C3只支持GPIO0-4）");
    ESP_LOGI(TAG, "4. 信号幅度太小，需要放大电路");
    ESP_LOGI(TAG, "建议: 检查硬件连接、电源供应，确认使用支持ADC的GPIO引脚");

    // 进入深度睡眠
    ESP_LOGI(TAG, "进入深度睡眠...");
    // esp_deep_sleep_start();
}
