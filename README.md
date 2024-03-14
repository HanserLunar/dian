# dian
用于提交项目成果

 //////////////////////////level 1///////////////////////////// 
 
 #include <Arduino.h>


void setup() {
 
  int result = myFunction(2, 3);
  int a=12;
  pinMode(a,OUTPUT);
  digitalWrite(a,HIGH);
}

void loop() 
{
  
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















