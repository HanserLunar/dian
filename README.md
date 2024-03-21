# dian
用于提交项目成果

                                            以 下 是 程 序
 /////////////////////////////////////////////////////////////////level 1///////////////////////////////////////////////////////////////

 
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


}
////////////////////////////////////////////////////////////////////levle 2.0/////////////////////////////////////////////////////////////////


#include <stdio.h>
 #include"driver/gpio.h" 
 #include"freertos/FreeRTOS.h" 
 #include"freertos/task.h"
  #include"esp_system.h" 
  #include"driver/uart.h" 
  #include"string.h" 
  #include"driver/ledc.h"
   #include<esp_log.h>
#include"mpu6050.h"

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
          vTaskDelay(2000/portTICK_PERIOD_MS); 
    }
}

////////////////////////////////////////////////////////////////level 2.1/////////////////////////////////////////////////////////////////
///////////////////我认为操作mpu6050寄存器部分过于困难，所以选择利用“mpu6050.h”，相对简单的读取加速度，角速度速度等数据////////////////////////

#include <stdio.h>
#include"driver/gpio.h"
#include"freertos/FreeRTOS.h"
#include"freertos/task.h"
#include"esp_system.h"
#include"driver/uart.h"
#include"string.h"
#include"driver/ledc.h"
#include<esp_log.h>
#include <stdio.h>
#include"driver/gpio.h"
#include"freertos/FreeRTOS.h"
#include"freertos/task.h"
#include"esp_system.h"
#include"driver/uart.h"
#include"string.h"
#include"driver/ledc.h"
#include<esp_log.h>
#include "driver/i2c.h"
#include"unity.h"
#include"mpu6050.h"


                                               正文  正文  正文  正文  正文  正文  正文  正文  正文  正文  
                                               正文  正文  正文  正文  正文  正文  正文  正文  正文  正文  
                                               正文  正文  正文  正文  正文  正文  正文  正文  正文  正文  
                                               正文  正文  正文  正文  正文  正文  正文  正文  正文  正文  
                                               正文  正文  正文  正文  正文  正文  正文  正文  正文  正文  
                                               正文  正文  正文  正文  正文  正文  正文  正文  正文  正文  
                                               正文  正文  正文  正文  正文  正文  正文  正文  正文  正文  

uint16_t mm=0x68;
static mpu6050_handle_t mpu6050=NULL;
//初始化i2c通信
void init()
{
  i2c_config_t i2c_struct=
  {
    .mode=I2C_MODE_MASTER,
    .scl_io_num=GPIO_NUM_4,
    .sda_io_num=GPIO_NUM_5,
    .scl_pullup_en=GPIO_PULLUP_ENABLE,
    .sda_pullup_en=GPIO_PULLUP_ENABLE,
    .master.clk_speed=1000000,
    .clk_flags=I2C_SCLK_SRC_FLAG_FOR_NOMAL
  };
    i2c_param_config(I2C_NUM_0,&i2c_struct);
   i2c_driver_install(I2C_NUM_0,I2C_MODE_MASTER,0,0,0);
} 

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*此块与读取mpu6050无关，仅属于学习产物（草稿）////////////
i2c_cmd_handle_t cmd_struct;
uint8_t data[128];
static esp_err_t readMessage()
{
  int ret;
  cmd_struct=i2c_cmd_link_create();//创建一个i2c命令链接，为之后的i2c操作执行
  i2c_master_start(cmd_struct);//i2c运行开始函数，只是开始标记，初始化cmd_struct

  //发送一个字节数据  包括从机地址 读写方向 0表示主机输出 1表示从机输出
  i2c_master_write_byte(cmd_struct,mm<<1|I2C_MASTER_READ,0X1);//存疑//向mpu6050读取数据，等待从机返回数据
  
  //再发一个字节数据，包括从机寄存器地址
  i2c_master_write_byte(cmd_struct,0x3B,0x1);

  //重新发起一次通信，改变通信方向
  i2c_master_start(cmd_struct);

  //从该寄存器读取一个字节数据,存入data数组
  i2c_master_read_byte(cmd_struct,data,0x0);

  //无需应答
  //停止通信
  i2c_master_stop(cmd_struct);
  ret=i2c_master_cmd_begin(I2C_NUM_0,cmd_struct,1000/portTICK_PERIOD_MS);

  i2c_cmd_link_delete(cmd_struct);
  //i2c_master_read()
  return ret;
}
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////mpu初始化//////////////////////////////////////////////////////////



static void i2c_sensor_mpu6050_init(void)
{
  mpu6050=mpu6050_create(I2C_NUM_0,0X68);
  mpu6050_config(mpu6050,ACCE_FS_4G,GYRO_FS_500DPS);
  mpu6050_wake_up(mpu6050);
}



/////////////////////////////////////////////////////////////////////////////////////////
void app_main()
{


    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;
  
    init();
    i2c_sensor_mpu6050_init();
    mpu6050_get_acce(mpu6050,&acce);
    printf("x =%.2f  y =%.2f  z =%.2f\n",acce.acce_x,acce.acce_y,acce.acce_z);
    mpu6050_get_gyro(mpu6050,&gyro);
    printf("x =%.2f  y =%.2f  z =%.2f\n",gyro.gyro_x,gyro.gyro_y,gyro.gyro_z);
    mpu6050_get_temp(mpu6050,&temp);
    printf("temp is %.2f\n",temp.temp);
    while(1)
    {
       mpu6050_get_acce(mpu6050,&acce);
       printf(" acce :x =%.2f  y =%.2f  z =%.2f\n",acce.acce_x,acce.acce_y,acce.acce_z);
     
       mpu6050_get_gyro(mpu6050,&gyro);
       printf("gyro :x =%.2f  y =%.2f  z =%.2f\n",gyro.gyro_x,gyro.gyro_y,gyro.gyro_z);
      
       mpu6050_get_temp(mpu6050,&temp);
       printf("tempreture : %.2f\n",temp.temp);
       
       vTaskDelay(1000/portTICK_PERIOD_MS);
    }
  
}











