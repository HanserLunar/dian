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


#include <stdio.h>
#include"driver/gpio.h"
#include"freertos/FreeRTOS.h"
#include"freertos/task.h"
#include"esp_system.h"
#include"driver/uart.h"
#include"string.h"
#include"driver/ledc.h"
#include<esp_log.h>

static const int RX_BUF_SIZE=1024 ;
#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)

void init()
{
  uart_config_t uart_struct=
    {
       .baud_rate=115200,
        .data_bits=UART_DATA_8_BITS,
        .flow_ctrl=UART_HW_FLOWCTRL_DISABLE,
        .parity=UART_PARITY_DISABLE,
        .source_clk=UART_SCLK_APB,
        .stop_bits=UART_STOP_BITS_1
    };
      uart_param_config(UART_NUM_0,&uart_struct);
      uart_driver_install(UART_NUM_0,RX_BUF_SIZE*2,0,0,NULL,0);
      uart_set_pin(UART_NUM_0,TXD_PIN,RXD_PIN,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);

}

void app_main()
{
    init();
      while(1)
   {
    printf("hello world\n");
    vTaskDelay(2000/portTICK_PERIOD_MS);
   }
}













