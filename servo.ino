#include <Servo.h>

int arr[8] = {0, 1, 1, 1, 0, 1, 0, 0};
int sz = 8;

int angleOpen[8] = {0, 17, 34, 54, 100, 126, 150, 174};

Servo s;

void setup()
{
  s.attach(8);
  s.write(72);
  Serial.begin(9600);
}

void loop()
{

  char c = ' ';
  
  int c1 = 0;
  int c2 = 0; 
  
  for (int i=0; i<sz; i++) {
    if (arr[i] == 0) {
      s.write(angleOpen[3 - c1]);
      Serial.println(angleOpen[3 - c1]);
      c1 += 1;
    } else {
      s.write(angleOpen[4 + c2]);
      Serial.println(angleOpen[4 + c2]);
      c2 += 1;
    }
    delay(3000);
    s.write(72);
    delay(500);
  }
  s.write(72);
  while (1);
}
