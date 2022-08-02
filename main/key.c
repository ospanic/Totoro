/* 该文件主要实现了按键检测，不同的按键组合代表不同的音符。
* 按住Flash按键(IO0) 进入设置模式，可配合指法按键执行不同功能
*
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "led_strip.h"

#include "ble_midi.h"

#include "nvs_flash.h"
#include "nvs.h"

#define FIRST_TUNE 0x43 //定义全按音符，即最低音

uint8_t midi_on[]  = {0x80, 0x80, 0x90, 0x3B, 0x63};
uint8_t midi_off[] = {0x80, 0x80, 0x80, 0x3B, 0x63};

uint8_t tune_start = FIRST_TUNE;
int8_t tune_q[] = { 0, 2,-1, 4,-1,-1,-1, 5, //指法表
                   -1,-1,-1,-1,-1,-1,-1, 7,
                   -1,-1,-1,-1,-1,-1,-1,10,
                   -1,-1,-1,-1,-1,-1,-1, 9,
                   12,14,-1,16,-1,-1,-1,17,
                   -1,-1,-1,-1,-1,-1,-1,19,
                   -1,-1,-1,-1,-1,-1,-1,-1,
                   -1,-1,-1,-1,-1,-1,-1,21}; //定义每一个按键现对于 FIRST_TUNE 的偏移

//uint8_t key_q[] = { 12,26,27,2,4,23,0,32}; //按键的GPIO序列(旧版硬件)
uint8_t key_q[] = { 12,27,33,2,4,22,0,32}; //按键的GPIO序列
uint8_t key_status[8] = { 1,1,1,1,1,1,1,1 }; //用来记录每一个按键的状态

#define ESP_INTR_FLAG_DEFAULT 0
xQueueHandle gpio_evt_queue = NULL;

extern led_strip_t *strip; //WS2812灯带
extern int cc_control; //Android 或 iOS 切换

nvs_handle_t my_handle;

void fn_key_handle(int n) //功能设置处理函数
{
    if(n == 0) //功能键 + 第1个键 ：降调
    {
        tune_start -= 1;
        ESP_ERROR_CHECK(strip->set_pixel(strip, 6, 100, 0, 0));
        ESP_ERROR_CHECK(strip->refresh(strip, 100));
        nvs_set_u8(my_handle, "tune_start", tune_start); //保存设置
    }

    if(n == 1) //功能键 + 第2个键 ：切换 CC控制 或 音量控制
    {
        if(cc_control == 1)  
        {
            cc_control = 0; //Andriod版本 
            ESP_ERROR_CHECK(strip->set_pixel(strip, 3, 0, 0, 180)); //亮蓝色灯
        }
        else 
        {
            cc_control = 1; //iOS版本
            ESP_ERROR_CHECK(strip->set_pixel(strip, 3, 0, 100, 0));//亮绿色灯
        }
        ESP_ERROR_CHECK(strip->refresh(strip, 100));
        nvs_set_i32(my_handle, "cc_control", cc_control);//保存设置
    }

    if(n == 2) //功能键 + 第3个键 ：降调
    {
        tune_start += 1;
        ESP_ERROR_CHECK(strip->set_pixel(strip, 1, 100, 0, 0));
        ESP_ERROR_CHECK(strip->refresh(strip, 100));
        nvs_set_u8(my_handle, "tune_start", tune_start);//保存设置
    }
    nvs_commit(my_handle); //提交保存
}

//GPIO中断函数
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
} 

extern uint8_t ws2812_pause; //WS2812 刷新任务是否暂停
int key_value_list[] = {1,2,4,8,16,32}; //按键的二进制组合
int key_value = 63; //按键全按下为 63
int last_midi = 0; //上次扫描得到的音符
void Key_task()
{
    uint32_t io_num;
    for(;;) 
    {
        //vTaskDelay(10);
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) 
        {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(key_q[io_num]));

            if(gpio_get_level(key_q[6]) == 0) //IO0按键被按下(按住不放则为设置状态)
            {
                if(gpio_get_level(key_q[io_num]) == 0)//指法按键按下
                {
                  ws2812_pause = 1; //停止刷新WS2812
                  fn_key_handle(io_num);
                  while(gpio_get_level(key_q[io_num]) == 0)vTaskDelay(30);  //等待按键抬起
                  ws2812_pause = 0; //恢复刷新WS2812
                }
                //vTaskDelay(100); //消抖延时
                xQueueReset(gpio_evt_queue); //消抖
                continue;
            }

            if(io_num == 5) //八度转换有变化
            {
                vTaskDelay(80); //消抖延时
            }

            key_value = 63; //读取指法按键电平，判断音符
            if(gpio_get_level(key_q[0]) == 0) key_value -= key_value_list[0];
            if(gpio_get_level(key_q[1]) == 0) key_value -= key_value_list[1];
            if(gpio_get_level(key_q[2]) == 0) key_value -= key_value_list[2];
            if(gpio_get_level(key_q[3]) == 0) key_value -= key_value_list[3];
            if(gpio_get_level(key_q[4]) == 0) key_value -= key_value_list[4];
            if(gpio_get_level(key_q[5]) == 0) key_value -= key_value_list[5];

            xQueueReset(gpio_evt_queue); //清理所有未处理按键(基本都是误触)

            if(tune_q[key_value] == -1) continue; //无效按键组合

            if(last_midi != tune_start + tune_q[key_value]) //如果音符发生变化，则通过蓝牙发送数据
            {
                midi_on[3] = tune_start + tune_q[key_value];
                MiDi_Send(midi_on,5);

                midi_off[3] = last_midi;
                MiDi_Send(midi_off,5);

                last_midi = tune_start + tune_q[key_value];

                ESP_LOGE("KEY", "Send Send %02X",last_midi);
            }
        }
    }
}

void key_init()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = ((1ULL << key_q[0]) | (1ULL << key_q[1] ) |
                            (1ULL << key_q[2]) | (1ULL << key_q[3] ) |
                            (1ULL << key_q[4]) | (1ULL << key_q[5] ) |
                            (1ULL << key_q[6]) | (1ULL << key_q[7] ) );
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(32, sizeof(uint32_t));

    
    xTaskCreate(Key_task, "gpio_task_example", 2048, NULL, 10, NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    for(int i =0;i<6;i++)
    {
        gpio_isr_handler_add(key_q[i], gpio_isr_handler, (void*) i);
    }

    //读取设置数据
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if(err != ESP_OK) return;

    err = nvs_get_i32(my_handle, "cc_control", &cc_control); //读取音量控制器(Android OR iOS)
    if(err != ESP_OK) cc_control = 1;

    err = nvs_get_u8(my_handle, "tune_start", &tune_start); //读取升降调信息
    if(err != ESP_OK) tune_start = FIRST_TUNE;
}