#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"
#include "main.h"
#include "led.h"
#include "rtc.h"
#include "timer.h"
#include "esp_at.h"
#include "mpu6050.h"
#include "st7735.h"
#include "stfonts.h"
#include "stimage.h"
#include "weather.h"


/* 使用前请填入自己的 WiFi 账号密码与心知天气 API Key */
static const char *wifi_ssid = "YOUR_WIFI_SSID";
static const char *wifi_password = "YOUR_WIFI_PASSWORD";
static const char *weather_uri = "https://api.seniverse.com/v3/weather/now.json?key=YOUR_SENIVERSE_API_KEY&location=hefei&language=en&unit=c";


static uint32_t runms;
static uint32_t disp_height;


static void timer_elapsed_callback(void)
{
    runms++;
    if (runms >= 24 * 60 * 60 * 1000)
    {
        runms = 0;
    }
}

static void wifi_init(void)
{
    st7735_write_string(0, disp_height, "Init ESP32...", &font_ascii_8x16, ST7735_WHITE, ST7735_BLACK);
    disp_height += font_ascii_8x16.height;
    if (!esp_at_init())
    {
        st7735_write_string(0, disp_height, "Failed!!!", &font_ascii_8x16, ST7735_RED, ST7735_BLACK);
        disp_height += font_ascii_8x16.height;
        while (1);
    }

    st7735_write_string(0, disp_height, "Init WIFI...", &font_ascii_8x16, ST7735_WHITE, ST7735_BLACK);
    disp_height += font_ascii_8x16.height;
    if (!esp_at_wifi_init())
    {
        st7735_write_string(0, disp_height, "Failed!!!", &font_ascii_8x16, ST7735_RED, ST7735_BLACK);
        disp_height += font_ascii_8x16.height;
        while (1);
    }

    st7735_write_string(0, disp_height, "Connect WIFI...", &font_ascii_8x16, ST7735_WHITE, ST7735_BLACK);
    disp_height += font_ascii_8x16.height;
    if (!esp_at_wifi_connect(wifi_ssid, wifi_password))
    {
        st7735_write_string(0, disp_height, "Failed!!!", &font_ascii_8x16, ST7735_RED, ST7735_BLACK);
        disp_height += font_ascii_8x16.height;
        while (1);
    }

    st7735_write_string(0, disp_height, "Sync Time...", &font_ascii_8x16, ST7735_WHITE, ST7735_BLACK);
    disp_height += font_ascii_8x16.height;
    if (!esp_at_sntp_init())
    {
        st7735_write_string(0, disp_height, "Failed!!!", &font_ascii_8x16, ST7735_RED, ST7735_BLACK);
        disp_height += font_ascii_8x16.height;
        while (1);
    }
}

int main(void)
{
    board_lowlevel_init();

    led_init();
    rtc_init();
    timer_init(1000);
    timer_elapsed_register(timer_elapsed_callback);
    timer_start();

    mpu6050_init();

    st7735_init();
    st7735_fill_screen(ST7735_BLACK);

    // 显示开机内容
    st7735_write_string(0, 0, "Initializing...", &font_ascii_8x16, ST7735_WHITE, ST7735_BLACK);
    disp_height += font_ascii_8x16.height;
    delay_ms(500);

    st7735_write_string(0, disp_height, "Wait ESP32...", &font_ascii_8x16, ST7735_WHITE, ST7735_BLACK);
    disp_height += font_ascii_8x16.height;
    delay_ms(1500);

    wifi_init();

    st7735_write_string(0, disp_height, "Ready", &font_ascii_8x16, ST7735_GREEN, ST7735_BLACK);
    disp_height += font_ascii_8x16.height;
    delay_ms(500);

    st7735_fill_screen(ST7735_BLACK);

    runms = 0;
    uint32_t last_runms = runms;
    bool weather_ok = false;
    bool sntp_ok = false;
    char str[64];
    while (1)
    {
        if (runms == last_runms)
        {
            continue;
        }
        last_runms = runms;

        // 更新时间信息
        if (last_runms % 100 == 0)
        {
            rtc_date_t date;
            rtc_get_date(&date);
            snprintf(str, sizeof(str), "%02d-%02d-%02d", date.year % 100, date.month, date.day);
            st7735_write_string(0, 0, str, &font_ascii_8x16, ST7735_WHITE, ST7735_BLACK);
            snprintf(str, sizeof(str), "%02d%s%02d", date.hour, date.second % 2 ? " " : ":", date.minute);
            st7735_write_string(0, 78, str, &font_time_24x48, ST7735_CYAN, ST7735_BLACK);
        }

        // 联网同步时间
        if (!sntp_ok || last_runms % (60 * 60 * 1000) == 0)
        {
            uint32_t ts;
            sntp_ok = esp_at_get_time(&ts);
            rtc_set_timestamp(ts + 8 * 60 * 60);
        }

        // 更新天气信息
        if (!weather_ok || last_runms % (10 * 60 * 1000) == 0)
        {
            const char *rsp;
            weather_ok = esp_at_get_http(weather_uri, &rsp, NULL, 10000);
            weather_t weather;
            weather_parse(rsp, &weather);

            const st_image_t *img = NULL;
            if (strcmp(weather.weather, "Cloudy") == 0) {
                img = &icon_weather_duoyun;
            } else if (strcmp(weather.weather, "Wind") == 0) {
                img = &icon_weather_feng;
            } else if (strcmp(weather.weather, "Clear") == 0) {
                img = &icon_weather_qing;
            } else if (strcmp(weather.weather, "Snow") == 0) {
                img = &icon_weather_xue;
            } else if (strcmp(weather.weather, "Overcast") == 0) {
                img = &icon_weather_yin;
            } else if (strcmp(weather.weather, "Rain") == 0) {
                img = &icon_weather_yu;
            }
            if (img != NULL) {
                st7735_draw_image(0, 16, img->width, img->height, img->data);
            } else {
                snprintf(str, sizeof(str), "%s", weather.weather);
                st7735_write_string(0, 16, str, &font_ascii_8x16, ST7735_YELLOW, ST7735_BLACK);
            }

            snprintf(str, sizeof(str), "%sC", weather.temperature);
            st7735_write_string(78, 0, str, &font_temper_16x32, ST7735_BLUE, ST7735_BLACK);
        }

        // 更新环境温度
        if (last_runms % (1 * 1000) == 0)
        {
            float temper = mpu6050_read_temper();
            snprintf(str, sizeof(str), "%5.1fC", temper);
            st7735_write_string(78, 32, str, &font_ascii_8x16, ST7735_GREEN, ST7735_BLACK);
        }

        // 更新网络信息
        // if (last_runms % (30 * 1000) == 0)
        // {
        //     st7735_write_string(0, 127-48, wifi_ssid, &font_ascii_8x16, ST7735_WHITE, ST7735_BLACK);
        //     char ip[16];
        //     esp_at_wifi_get_ip(ip);
        //     st7735_write_string(0, 127-32, ip, &font_ascii_8x16, ST7735_WHITE, ST7735_BLACK);
        // }
    }
}
