# dian
用于提交项目成果

 //////////////////////////level 1///////////////////////////// 
#include <stdio.h>
#include"driver/gpio.h"
#include"freertos/FreeRTOS.h"
#include"freertos/task.h"

#define LED_pin GPIO_NUM_14
void app_main(void)
{
    esp_rom_gpio_pad_select_gpio(LED_pin);//使能gpio口
    //设置输入输出模式
    gpio_set_direction(LED_pin,GPIO_MODE_OUTPUT);
    //设置电平
    gpio_set_level(LED_pin,0);

    while(1)
    {
       
        gpio_set_level(LED_pin,0);
       // 延迟一秒
        vTaskDelay(1000/portTICK_PERIOD_MS);
         gpio_set_level(LED_pin,1);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }



///////////////////////levle 2.0//////////////////////////////

#include <Arduino.h>
#include<driver/uart.h>
#include<stdio.h>

void setup()
{
   Serial.begin(9600);
}

void loop()
{
    //if(Serial.available()>0)
    {
        Serial.println("hello world");
    }
}















