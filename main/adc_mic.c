/*
 * 该文件主要是通过ADC 读取MIC的电压信号，用来控制演奏的音量。
 * 同时通过WS2812指示灯指示音量的大小
 * 在MIDI协议中音量的控制有多种方式，比如主音量和呼吸控制。
 * 在iOS版本的“自乐班”中，采用的是呼吸控制器控制音量，在“库乐队”中，使用主音量控制音量
 * 在Android版本的“自乐班”中，采用主音量控制音量。
 */

#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/rmt.h"
#include "led_strip.h"

#include "ble_midi.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"


int cc_control = 1; //音量控制方式(可修改并保存)   0:主音量控制，适用于安卓，1:呼吸控制，适用于iOS

led_strip_t *strip = NULL;

void leds_init()
{
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(GPIO_NUM_25, RMT_CHANNEL_0);//WS2812引脚
    //rmt_config_t config = RMT_DEFAULT_CONFIG_TX(GPIO_NUM_14, RMT_CHANNEL_0);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(7, (led_strip_dev_t)config.channel);
    strip = led_strip_new_rmt_ws2812(&strip_config);

    if (!strip) {
        ESP_LOGE("WS2812", "install WS2812 driver failed");
    }

    ESP_ERROR_CHECK(strip->clear(strip, 100));
    vTaskDelay(20);

    if(cc_control == 1) //呼吸控制音量
    {
        for(int i = 0; i < 7; i+=2) //0246亮绿灯(iOS)
            ESP_ERROR_CHECK(strip->set_pixel(strip, i, 0, 100, 0));
    }
    else
    {
        for(int i = 0; i < 7; i+=2) //0246亮蓝灯(Android)
            ESP_ERROR_CHECK(strip->set_pixel(strip, i, 0, 0, 180));
    }

    ESP_ERROR_CHECK(strip->refresh(strip, 100));
    vTaskDelay(200);

    strip->clear(strip, 100);//全部熄灭
}


/**
 * Ws2812灯珠亮度吹起大小而改变
 */
#include "ble_midi.h"
uint8_t leds_on_count = 0; //亮起来的LED数量
uint8_t ws2812_pause = 0;  //暂时停止刷新LED
static void ws2812_task(void *arg)
{
    leds_init();
    int not_connect_time = 0; //连接指示灯 熄灭时间记录
    while (1) 
    {
        vTaskDelay(50);

        if(ws2812_pause == 1) continue;

        for(int i = 0; i < 7; i++) //全部熄灭
        {
            ESP_ERROR_CHECK(strip->set_pixel(strip, i, 0, 0, 0));
        }

        while(leds_on_count --) //吹气气流指示
            ESP_ERROR_CHECK(strip->set_pixel(strip, 6-leds_on_count, 0, 50, 0));

        if(ble_midi_state() == false) //蓝牙未连接
        {
            not_connect_time ++;
            if(not_connect_time > 30) //蓝牙未连接，第一颗灯1.5S闪烁一次
            {
                if(not_connect_time == 40)not_connect_time = 0;
                ESP_ERROR_CHECK(strip->set_pixel(strip, 0, 0, 0, 50));
            }
        }

        ESP_ERROR_CHECK(strip->refresh(strip, 100));
    }
}

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_0;     //GPIO36 if ADC1, GPIO14 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;

static const adc_atten_t atten = ADC_ATTEN_DB_2_5;
static const adc_unit_t unit = ADC_UNIT_1;
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

#define WINDOWS_W 10
#define MAX_POOL 20 

static uint8_t midi_vol[]  = {0x88, 0x88, 0xB0, 0x07, 0}; //音量控制07主音量，02呼吸控制器
uint32_t adc_value_windows[WINDOWS_W] ={0};
int adc_index = 0;
uint8_t last_val = 127;

void adc_task()
{
    while (1) 
    {
        vTaskDelay(pdMS_TO_TICKS(30));

        uint32_t adc_reading = 0;

        for (int i = 0; i < MAX_POOL; i++) 
        {
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        }
        adc_reading /= MAX_POOL;
        adc_value_windows[adc_index++] = adc_reading;

        if(adc_index == WINDOWS_W) adc_index = 0;
        vTaskDelay(1);

        adc_reading = 0;
        for(int i = 0; i < WINDOWS_W; i ++)
        {
            adc_reading += adc_value_windows[i];
        }
        adc_reading /= WINDOWS_W;

        if(adc_reading > 2800) adc_reading = 2800;
        if(adc_reading <  200) adc_reading = 0;

        leds_on_count = adc_reading / 360 ;//2800/360 = 7.7 最多7个灯亮

        if(cc_control == 1) //呼吸控制器控制音量0-127(iOS用)
        {
            midi_vol[3] = 0x02;
            midi_vol[4] = adc_reading/22;//2800/22 = 127
        }
        else//主音量控制器(协议上是0-127，自乐班Andriod范围0-255)
        {
            midi_vol[3] = 0x07;
            midi_vol[4] = adc_reading/11;
        }

        if(last_val != midi_vol[4])
        {
            MiDi_Send(midi_vol,5);
            last_val = midi_vol[4];
        }

        //printf("$0,%d;\r\n",last_val);
    }
}

void adc_mic_init(void)
{
    printf("Init ADC...\n");

    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);

    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    xTaskCreate(adc_task, "adc_task", 4096, NULL, 10, NULL);

    xTaskCreate(ws2812_task, "ws2812_task", 2048, NULL, 6, NULL);
}
 