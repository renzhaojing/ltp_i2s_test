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
#include "driver/rmt_tx.h"
#include "ws2812_driver.h"

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

// WS2812 LED灯带配置
#define LED_STRIP_PIN          GPIO_NUM_7   // WS2812数据引脚（可根据实际硬件修改）
#define LED_STRIP_NUM          16           // LED数量
#define LED_UPDATE_INTERVAL_MS 50           // LED更新间隔（50ms），更快响应

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
#define BUFFER_SIZE            50      // 读取缓冲区大小（样本数）- 约6.25ms数据，极快响应
#define RECORD_DURATION_MS     600000   // 20秒录音
#define ADC_SAMPLE_INTERVAL_US (1000000 / SAMPLE_RATE)  // 采样间隔（微秒）
#define AUDIO_UPDATE_INTERVAL_MS 50   // 音频统计更新间隔（50ms），更快响应
#define LED_UPDATE_INTERVAL_MS 50     // LED更新间隔（50ms），更快响应

// 音频缓冲区
static int16_t mic_buffer[BUFFER_SIZE];    // 16位MIC输入缓冲区（12位ADC数据转换为16位）

// ADC句柄和校准参数
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool adc_calibration_init = false;

// 状态标志
static volatile bool is_recording = false;

// LED灯带句柄
static ws2812_handle_t *led_strip = NULL;

// 音频数据共享（用于LED控制）
typedef struct {
    int peak_to_peak;      // 峰峰值
    float rms;             // RMS值
    float volume_percent;  // 音量强度百分比
    bool updated;          // 数据是否更新
} audio_data_t;

static audio_data_t audio_data = {0};
static SemaphoreHandle_t audio_data_mutex = NULL;

// 函数声明
static void mic_test_task(void *arg);
static void led_control_task(void *arg);
static esp_err_t init_led_strip(void);

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
        // 控制采样率：为了更快响应，减少延迟等待
        int64_t current_time = esp_timer_get_time();
        int64_t elapsed = current_time - last_sample_time;
        
        if (elapsed < ADC_SAMPLE_INTERVAL_US) {
            // 为了更快响应，只等待较长的延迟（>=5ms）
            // 对于短延迟，直接采样以提高响应速度
            int64_t wait_us = ADC_SAMPLE_INTERVAL_US - elapsed;
            if (wait_us >= 5000) {  // 只等待>=5ms的延迟
                vTaskDelay(pdMS_TO_TICKS(wait_us / 1000));
            } else {
                // 短延迟时，至少延迟1个tick让出CPU给IDLE任务
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

        // 当缓冲区满时，立即处理数据（不等待，实时响应）
        if (buffer_idx >= BUFFER_SIZE) {
            // 计算音频统计信息
            int64_t sum_squares = 0;
            int16_t max_sample = INT16_MIN;
            int16_t min_sample = INT16_MAX;
            uint32_t sum_voltage = 0;
            // 跟踪ADC原始值范围（在整个测试过程中）
            static int adc_raw_min = 4095;
            static int adc_raw_max = 0;
            static bool first_buffer = true;
            
            if (first_buffer) {
                adc_raw_min = 4095;  // 重置为最大值
                adc_raw_max = 0;     // 重置为最小值
                first_buffer = false;
            }

            // 计算绝对值的平均值（用于区分声音大小）
            int64_t sum_abs = 0;
            for (size_t i = 0; i < BUFFER_SIZE; i++) {
                int16_t s = mic_buffer[i];
                sum_squares += (int64_t)s * s;
                sum_abs += (s < 0) ? -s : s;  // 绝对值累加
                if (s > max_sample) max_sample = s;
                if (s < min_sample) min_sample = s;
                // 计算原始电压（从样本值反推）
                sum_voltage += (mic_buffer[i] + 2048) * 2500 / 4095;  // 估算电压
                // 计算ADC原始值范围（从16位样本值反推）
                int adc_raw_val = mic_buffer[i] + 2048;
                if (adc_raw_val < adc_raw_min) adc_raw_min = adc_raw_val;
                if (adc_raw_val > adc_raw_max) adc_raw_max = adc_raw_val;
            }

            float rms = sqrt((float)sum_squares / BUFFER_SIZE);
            float avg_abs = (float)sum_abs / BUFFER_SIZE;  // 绝对值的平均值
            float avg_voltage = (float)sum_voltage / BUFFER_SIZE;

            uint32_t elapsed = (esp_timer_get_time() / 1000) - start_time;

            // 每50ms更新一次音频数据（用于LED控制），减少日志打印频率
            static uint32_t last_update_time = 0;
            static uint32_t last_print_time = 0;
            
            // 总是更新音频数据到共享结构体（用于LED控制，实时响应）
            int16_t peak_to_peak = max_sample - min_sample;  // 峰峰值
            float volume_percent = 0.0f;
            const int PEAK_LOW = 5;    // 进一步降低阈值，提高灵敏度（正常说话也能检测）
            const int PEAK_HIGH = 50;  // 进一步降低高音量阈值，让正常说话就能达到高百分比
            if (peak_to_peak > PEAK_LOW) {
                // 使用更激进的映射，增强低音量响应
                float normalized = ((float)(peak_to_peak - PEAK_LOW) / (PEAK_HIGH - PEAK_LOW));
                if (normalized > 1.0f) normalized = 1.0f;  // 限制在0-1范围
                normalized = sqrtf(normalized);  // 平方根映射
                normalized = normalized * normalized;  // 平方，让低音量响应更明显
                volume_percent = normalized * 100.0f;
                if (volume_percent > 100.0f) volume_percent = 100.0f;
            }
            
            // 实时更新音频数据（不等待打印时间）
            if (audio_data_mutex != NULL) {
                if (xSemaphoreTake(audio_data_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                    audio_data.peak_to_peak = peak_to_peak;
                    audio_data.rms = rms;
                    audio_data.volume_percent = volume_percent;
                    audio_data.updated = true;
                    xSemaphoreGive(audio_data_mutex);
                }
            }
            
            // 每500ms打印一次统计信息（减少日志输出，提高性能）
            if (elapsed - last_print_time >= 500 || elapsed < 500) {
                // peak_to_peak已在上面计算
                // 计算ADC原始值范围
                int adc_raw_range_min = min_sample + 2048;  // 16位最小值转回ADC原始值
                int adc_raw_range_max = max_sample + 2048;  // 16位最大值转回ADC原始值
                // 计算平均ADC原始值（反映声音的平均电平）
                float avg_adc_raw = avg_voltage * 4095.0f / 2500.0f;
                
                ESP_LOGI(TAG, "进度: %" PRIu32 " ms | 样本: %" PRIu32 " | RMS: %.1f | 峰值: %d/%d (峰峰值: %d)", 
                         elapsed, total_samples, rms, min_sample, max_sample, peak_to_peak);
                ESP_LOGI(TAG, "  音量指标: RMS=%.1f | 平均绝对值=%.1f | 平均ADC=%.1f | 平均电压=%.1f mV", 
                         rms, avg_abs, avg_adc_raw, avg_voltage);
                ESP_LOGI(TAG, "  ADC原始值范围: %d-%d (12位, 0-4095) | 电压范围: %.1f-%.1f mV", 
                         adc_raw_range_min, adc_raw_range_max,
                         (float)adc_raw_range_min * 2500.0f / 4095.0f,
                         (float)adc_raw_range_max * 2500.0f / 4095.0f);
                ESP_LOGI(TAG, "  历史ADC范围: %d-%d (总范围: %d)", 
                         adc_raw_min, adc_raw_max, adc_raw_max - adc_raw_min);
                
                // 声音大小判断（使用动态阈值和相对值）
                // 根据实际数据调整：静音时RMS约1590，拍打时RMS约1575-1600
                // 使用峰峰值作为主要指标，RMS作为辅助
                static float baseline_rms = 0.0f;  // 基线RMS值（静音时的RMS）
                static bool baseline_set = false;
                
                // 初始化基线（静音时的RMS值）
                if (!baseline_set && peak_to_peak < 20) {
                    baseline_rms = rms;
                    baseline_set = true;
                    ESP_LOGI(TAG, "  📊 设置静音基线: RMS=%.1f", baseline_rms);
                }
                
                // 使用峰峰值作为主要音量指标（更敏感）
                const int PEAK_LOW = 5;     // 峰峰值阈值：低音量（进一步降低，正常说话也能检测）
                const int PEAK_MID = 25;    // 峰峰值阈值：中音量（进一步降低）
                const int PEAK_HIGH = 50;   // 峰峰值阈值：高音量（进一步降低，让正常说话就能达到高音量）
                
                // RMS相对变化（相对于基线）
                float rms_change = 0.0f;
                if (baseline_set) {
                    rms_change = rms - baseline_rms;
                }
                
                // 音量等级判断（优先使用峰峰值）
                const char* volume_level;
                if (peak_to_peak < PEAK_LOW) {
                    volume_level = "静音/极低";
                } else if (peak_to_peak < PEAK_MID) {
                    volume_level = "低";
                } else if (peak_to_peak < PEAK_HIGH) {
                    volume_level = "中";
                } else {
                    volume_level = "高";
                }
                
                ESP_LOGI(TAG, "  🔊 音量等级: %s | 峰峰值=%d | RMS=%.1f (变化=%.1f) | 平均绝对值=%.1f", 
                         volume_level, peak_to_peak, rms, rms_change, avg_abs);
                
                // 音量强度百分比（基于峰峰值，归一化到0-100%）
                // 注意：这里的volume_percent仅用于日志显示，LED控制使用的是上面计算的volume_percent
                float volume_percent_log = 0.0f;
                const int PEAK_LOW_LOG = 5;
                const int PEAK_HIGH_LOG = 50;  // 与LED控制保持一致
                if (peak_to_peak > PEAK_LOW_LOG) {
                    float normalized = ((float)(peak_to_peak - PEAK_LOW_LOG) / (PEAK_HIGH_LOG - PEAK_LOW_LOG));
                    if (normalized > 1.0f) normalized = 1.0f;
                    normalized = sqrtf(normalized);  // 平方根映射
                    normalized = normalized * normalized;  // 平方，增强低音量响应
                    volume_percent_log = normalized * 100.0f;
                    if (volume_percent_log > 100.0f) volume_percent_log = 100.0f;
                }
                ESP_LOGI(TAG, "  📈 音量强度: %.1f%% (基于峰峰值)", volume_percent_log);
                last_print_time = elapsed;
                
                // 判断数据变化情况
                if (peak_to_peak < 10) {
                    ESP_LOGW(TAG, "⚠ 数据变化很小（峰峰值: %d），可能是静音或硬件连接问题", peak_to_peak);
                } else if (peak_to_peak > 100) {
                    ESP_LOGI(TAG, "✓ 检测到明显的声音信号（峰峰值: %d）", peak_to_peak);
                }
            }

            // 打印前16个样本（每500ms打印一次，减少日志输出）
            if (elapsed % 500 == 0) {
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

    // 创建互斥锁（如果还没有创建）
    if (audio_data_mutex == NULL) {
        audio_data_mutex = xSemaphoreCreateMutex();
        if (audio_data_mutex == NULL) {
            ESP_LOGE(TAG, "创建音频数据互斥锁失败");
            return;
        }
    }

    // 创建ADC模拟麦克风测试任务
    xTaskCreate(mic_test_task, "adc_mic_test_task", 4096, NULL, 5, NULL);
    
    // 创建LED控制任务
    xTaskCreate(led_control_task, "led_control_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "ADC模拟麦克风测试已启动，测试时长: %d ms", RECORD_DURATION_MS);
    ESP_LOGI(TAG, "LED灯带控制已启动，更新间隔: %d ms", LED_UPDATE_INTERVAL_MS);
}

/**
 * @brief 初始化WS2812 LED灯带
 */
static esp_err_t init_led_strip(void)
{
    ESP_LOGI(TAG, "初始化WS2812 LED灯带...");
    ESP_LOGI(TAG, "LED引脚: GPIO%d, LED数量: %d", LED_STRIP_PIN, LED_STRIP_NUM);
    
    esp_err_t ret = ws2812_init(LED_STRIP_PIN, LED_STRIP_NUM, &led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "初始化WS2812失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 清除LED
    ret = ws2812_clear(led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "清除LED失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = ws2812_refresh(led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "刷新LED失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "WS2812 LED灯带初始化成功");
    return ESP_OK;
}

/**
 * @brief LED流水效果控制任务
 */
static void led_control_task(void *arg)
{
    ESP_LOGI(TAG, "LED控制任务启动");
    
    uint32_t last_update_time = 0;
    
    while (is_recording) {
        uint32_t current_time = esp_timer_get_time() / 1000;
        
        // 每50ms更新一次LED（更快响应）
        if (current_time - last_update_time >= LED_UPDATE_INTERVAL_MS) {
            // 获取音频数据
            int peak_to_peak = 0;
            float volume_percent = 0.0f;
            
            if (xSemaphoreTake(audio_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                peak_to_peak = audio_data.peak_to_peak;
                volume_percent = audio_data.volume_percent;
                audio_data.updated = false;
                xSemaphoreGive(audio_data_mutex);
            }
            
            // 根据峰峰值计算要亮的LED数量
            // 进一步降低阈值范围，让更低的声音段也能看到明显效果
            // 例如：5-50的峰峰值范围就能实现1-16个LED全亮（更激进的映射）
            int led_count = 0;
            const int PEAK_MIN = 5;    // 最小峰峰值阈值（触发LED）
            const int PEAK_MAX = 50;   // 最大峰峰值（进一步降低，让正常说话就能触发全亮）
            
            if (peak_to_peak > PEAK_MIN) {
                // 使用更激进的映射：先平方根，再平方，增强低音量响应
                // 这样更小的声音变化也能产生更大的LED数量变化
                float normalized = ((float)(peak_to_peak - PEAK_MIN) / (PEAK_MAX - PEAK_MIN));
                if (normalized > 1.0f) normalized = 1.0f;  // 限制在0-1范围
                // 使用平方根映射，让低音量范围有更大的响应
                normalized = sqrtf(normalized);
                // 进一步放大低音量响应：使用平方函数
                normalized = normalized * normalized;  // 平方，让低音量响应更明显
                led_count = (int)(normalized * LED_STRIP_NUM) + 1;  // 至少亮1个LED
                if (led_count > LED_STRIP_NUM) led_count = LED_STRIP_NUM;
            } else {
                led_count = 0;  // 静音时不亮LED
            }
            
            if (led_strip != NULL) {
                // 清除所有LED
                ws2812_clear(led_strip);
                
                // 根据音量设置LED颜色和亮度
                // 音量越大，颜色越亮（从绿色->黄色->红色）
                uint8_t r = 0, g = 0, b = 0;
                if (volume_percent > 0) {
                    if (volume_percent < 15.0f) {
                        // 极低音量：暗绿色（让正常说话也能看到）
                        g = (uint8_t)(100 + 100 * volume_percent / 15.0f);  // 100-200，更明显
                    } else if (volume_percent < 40.0f) {
                        // 低音量：绿色（正常说话范围）
                        g = (uint8_t)(200 + 55 * (volume_percent - 15.0f) / 25.0f);  // 200-255
                    } else if (volume_percent < 70.0f) {
                        // 中音量：黄色（绿+红）
                        g = 255;
                        r = (uint8_t)(255 * (volume_percent - 40.0f) / 30.0f);
                    } else {
                        // 高音量：红色
                        r = 255;
                        g = (uint8_t)(255 * (100.0f - volume_percent) / 30.0f);
                    }
                }
                
                // 从第1个LED（索引0）开始顺序点亮led_count个LED
                // 根据声音强弱，依次点亮更多LED
                for (int i = 0; i < led_count; i++) {
                    // 渐变效果：前面的LED最亮，后面的逐渐变暗
                    float brightness = 1.0f;
                    if (led_count > 1) {
                        brightness = 1.0f - ((float)i / (float)led_count) * 0.5f;  // 亮度从1.0到0.5
                    }
                    if (brightness < 0.5f) brightness = 0.5f;  // 最小亮度
                    
                    uint8_t led_r = (uint8_t)(r * brightness);
                    uint8_t led_g = (uint8_t)(g * brightness);
                    uint8_t led_b = (uint8_t)(b * brightness);
                    
                    // 从LED 0开始顺序点亮
                    ws2812_set_pixel(led_strip, i, led_r, led_g, led_b);
                }
                
                // 刷新LED显示
                ws2812_refresh(led_strip);
            }
            
            last_update_time = current_time;
        }
        
        vTaskDelay(pdMS_TO_TICKS(5));  // 5ms延迟，更快响应
    }
    
    // 清除LED
    if (led_strip != NULL) {
        ws2812_clear(led_strip);
        ws2812_refresh(led_strip);
        ws2812_deinit(led_strip);
        led_strip = NULL;
    }
    
    ESP_LOGI(TAG, "LED控制任务结束");
    vTaskDelete(NULL);
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
    ESP_LOGI(TAG, "LED配置:");
    ESP_LOGI(TAG, "  LED引脚: GPIO%d", LED_STRIP_PIN);
    ESP_LOGI(TAG, "  LED数量: %d", LED_STRIP_NUM);
    ESP_LOGI(TAG, "  LED更新间隔: %d ms", LED_UPDATE_INTERVAL_MS);
    ESP_LOGI(TAG, "  音频更新间隔: %d ms", AUDIO_UPDATE_INTERVAL_MS);
    ESP_LOGI(TAG, "===================================");

    // 初始化WS2812 LED灯带
    ESP_LOGI(TAG, "=== 初始化WS2812 LED灯带 ===");
    esp_err_t led_init_err = init_led_strip();
    if (led_init_err != ESP_OK) {
        ESP_LOGW(TAG, "LED灯带初始化失败: %s，将继续运行但不显示LED效果", esp_err_to_name(led_init_err));
    }

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
