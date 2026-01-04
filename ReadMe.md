
# 当前程序为了测试esp32-s3 板子， 外接的i2s麦克风和i2s喇叭是否可以正常录制和发声。
麦克风和喇叭的采用率都为16000

麦克风的gpio，左声道，单声道
#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_48
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_40
#define AUDIO_I2S_MIC_GPIO_DIN  GPIO_NUM_39

喇叭的gpio，右声道，单声道
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_13
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_14
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_21
#define AUDIO_I2S_SPK_GPIO_MUTE GPIO_NUM_47 // 需要设置为高电平，喇叭才可用


# 实现两个测试，
一个正弦波声音播放，测试喇叭
第二个为录制音频并通过喇叭播放出来，回环测试。

## 编译和运行

1. 确保已安装 ESP-IDF 环境
2. 在项目根目录运行：
   ```bash
   idf.py build
   idf.py flash
   idf.py monitor
   ```

## 技术实现

- 使用 ESP-IDF I2S 标准 API (i2s_std.h)，支持 channel-based 配置
- 麦克风：单声道，左声道输入 (I2S_STD_SLOT_LEFT, bit_shift=true)
- 喇叭：单声道，右声道输出 (I2S_STD_SLOT_RIGHT, bit_shift=true)
- 采样率：16000Hz，16-bit PCM 音频格式
- 使用独立的 I2S channels：I2S_NUM_1（麦克风）和 I2S_NUM_0（喇叭）
- **麦克风配置**：按照标准SPH0645LM4H配置，16位采样，左声道

## SPH0645LM4H 数据处理

麦克风配置为16位采样，直接读取SPH0645LM4H 24位数据的有效位（通常是高16位）。这种配置简化了数据处理，提高了系统性能。

## 使用说明

程序启动后会直接开始录音回放测试：
- 录制麦克风音频并实时通过喇叭播放，实现回环测试

## 硬件连接

确保按照以下 GPIO 配置连接硬件：
- 麦克风: WS=48, SCK=40, DIN=39, SELECT=38
- 喇叭: DOUT=13, BCLK=14, LRCK=21, MUTE=47

注意：
- MUTE 引脚需要设置为高电平才能启用喇叭输出
- SPH0645LM4H的SELECT引脚必须连接，用于选择左右声道（低电平=左声道）
- 麦克风需要1.8V电源供电
- 麦克风数据输出为24位I2S格式

## 麦克风规格 (SPH0645LM4H-1)

- 灵敏度: -26 dBFS (典型值，94dB SPL @ 1kHz)
- SNR: 65 dB(A)
- 供电电压: 1.62-3.6V (典型1.8V)
- 时钟频率: 1024-4096 kHz (典型3072 kHz)
- 数据格式: 24位I2S，LSB填充0
