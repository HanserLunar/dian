# dian
用于提交项目成果

 level 1
 
 #include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  int a=12;
  pinMode(a,OUTPUT);
  digitalWrite(a,HIGH);
}

void loop() 
{
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y)
 {
  return x + y;
}
