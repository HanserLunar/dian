#include <stdio.h>
 #include"driver/gpio.h" 
 #include"freertos/FreeRTOS.h"
  #include"freertos/task.h"
   #include"esp_system.h" 
   #include"driver/uart.h"
    #include"string.h"
 #include"driver/ledc.h"
  #include<esp_log.h> 
      //#include"mpu6050.h"

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

void app_main() {

 uint8_t data[128];
 int length = 0;
 char* test_str = "hello world\n";
init();
while(1)
{
    //往uart寄存器写字符串
    uart_write_bytes(UART_NUM_0, (const char*)test_str, strlen(test_str));
     //从uart寄存器读取字符串
     ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_0, (size_t*)&length));
     length = uart_read_bytes(UART_NUM_0, data, length, 100);
     //延时函数
     // vTaskDelay(1000/portTICK_PERIOD_MS); 
}
}