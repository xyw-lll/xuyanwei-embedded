# xuyanwei-embedded

智能天气时钟，基于光流传感器实现飞行器定点功能的代码。

两个 STM32 嵌入式项目源码合集（个人学习 / 课程项目）。

## 项目一：Smart Weather Clock（智能天气时钟）

基于 STM32F103 的联网天气时钟：ESP-AT 模块连 WiFi 请求心知天气接口，ST7735 彩屏显示时间、天气与图标。

- **主控**：STM32F103C8（Cortex-M3）
- **显示**：ST7735 SPI TFT 彩屏（自绘字库 / 天气图标）
- **联网**：ESP32/ESP8266 + AT 指令（`driver/esp-at`），HTTP 拉取天气 JSON 并解析
- **其他外设**：RTC 实时时钟、MPU6050、定时器、LED
- **工程文件**：`mdk/stm32f103.uvprojx`（Keil MDK）

### 目录结构

```
Smart Weather Clock-project/
├── app/          # 业务层：main、weather 解析、字库与图标
├── driver/       # 外设驱动：esp-at、mpu6050、st7735、rtc、timer、led
├── firmware/     # CMSIS + 标准外设库（STM32F10x）
├── third_lib/    # 第三方库
├── mdk/          # Keil MDK 工程（stm32f103.uvprojx）
└── .vscode/      # VSCode 配置
```

### 运行前配置

`app/main.c` 中的 WiFi 账号密码与心知天气 API Key 已替换为占位符，编译前请填成自己的：

```c
static const char *wifi_ssid = "YOUR_WIFI_SSID";
static const char *wifi_password = "YOUR_WIFI_PASSWORD";
static const char *weather_uri = "https://api.seniverse.com/v3/weather/now.json?key=YOUR_SENIVERSE_API_KEY&location=hefei&language=en&unit=c";
```

## 项目二：基于光流传感器实现飞行器定点功能

基于 WHEELTEC F570 飞控板的定点飞行功能实现：读取光流传感器数据，结合姿态与 PID 控制实现水平位置保持。

- **主控**：STM32F405RG（Cortex-M4F）
- **RTOS**：FreeRTOS（CMSIS-RTOS v2）
- **传感器**：光流模块（LC307）、IMU（ICM20948）、STP23L 测距、ADC
- **外设**：USB Host（HID 手柄）、UART、I2C、TIM、DMA
- **应用层**：`WHEELTEC_APP` 下含 balance_task、LC307_task、pid、ps2_task、stp23l_task、bluetooth_task、show 等
- **工程文件**：`WHEELTEC_F570/MDK-ARM/WHEELTEC_F570.uvprojx`（Keil MDK），STM32CubeMX 工程为 `WHEELTEC_F570.ioc`

### 目录结构

```
基于光流传感器实现飞行器定点功能/
└── WHEELTEC_F570/
    ├── Core/         # CubeMX 生成的 main / 外设初始化 / FreeRTOS
    ├── Drivers/      # CMSIS + STM32F4xx HAL 驱动
    ├── Middlewares/  # FreeRTOS、STM32 USB Host Library
    ├── USB_HOST/     # USB Host 应用与移植层
    ├── WHEELTEC_APP/ # 应用层任务与 PID 控制
    ├── WHEELTEC_BSP/ # 板级支持包（IMU、ADC、光流等）
    └── MDK-ARM/      # Keil MDK 工程
```

## 编译说明

1. 使用 **Keil MDK 5** 打开对应 `.uvprojx` 工程；
2. Cortex-M3 / Cortex-M4 器件支持包（Keil.STM32F1xx_DFP、Keil.STM32F4xx_DFP）需自行安装；
3. 编译产物（`.o`、`.crf`、`.axf`、`Objects/`、`Listings/` 等）已通过 `.gitignore` 排除，克隆后重新编译即可。
