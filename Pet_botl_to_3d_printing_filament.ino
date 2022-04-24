#include "GyverPID.h"
#include "GyverTimers.h"
#define CLK 8
#define DIO 9

#include "GyverTM1637.h"

GyverTM1637 disp(CLK, DIO);
GyverPID regulator(1.8, 0.02, 0.01, 10);

int delay_clc = 0;

int prevSetTemp = 0;
unsigned long timeSetTemp = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Timer2.setPeriod(500);     // Устанавливаем период таймера 20000 мкс -> 50 гц
  Timer2.enableISR(CHANNEL_A);   // Или просто .enableISR(), запускаем прерывание на канале А таймера 2

  pinMode( A3, INPUT );
  pinMode( A2, INPUT );
  pinMode( D6, OUTPUT );

  pinMode(D3, OUTPUT);  

  pinMode(D5, OUTPUT);  
  pinMode( D4, OUTPUT );
  digitalWrite(D4,1);

  disp.clear();
  disp.brightness(7);  // яркость, 0 - 7 (минимум - максимум)
  
  regulator.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulator.setLimits(0, 255);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
  regulator.setpoint = 0;        // сообщаем регулятору температуру, которую он должен поддерживать
}

void step()
{
  static unsigned long previousMicros = 0;
  static bool state = 0;
  unsigned long currentMicros = micros();
  if(currentMicros - previousMicros < delay_clc || delay_clc > 4000)
  {
    return;
  }

  //Serial.println(delay);
  digitalWrite(5, state);
  previousMicros = currentMicros;
  state = !state;
}

ISR(TIMER2_A) {   
  step();  
}

 int get_temp(int port)
{
  int res = analogRead(port);
  res += analogRead(port);
  res += analogRead(port);
  res += analogRead(port);
  res /=4;
  //Serial.println(res);
  const short temp_mass[41] = {4039,4002,3947,3869,3762,3621,3444,3232,2989,2724,2447,2169,1901,1650,1423,1221,1045,893,763,653,560,482,416,360,313,273,239,210,185,164,146,130,116,105,94,85,77,70,64,58,54};
  int i = 0;
  float celsius;
  for( i = 0; i < 40; i++)
  {
    if(res > temp_mass[i])
    {
      celsius = ((i-1)*10) + (res - temp_mass[i-1]) * ((float)(10.0) / ((float)(temp_mass[i]) - temp_mass[i-1]));
      return celsius;
    }
  }
  return 999;
  
}

unsigned long timeLed = 0;
unsigned long tempTime = 0;
unsigned long tempGetTemp = 0;

int temp = 0;
void loop() {
  
  //step();
  tempTime = millis();
  if(tempTime-tempGetTemp>10)
  {
    tempGetTemp = tempTime;
    temp = get_temp(A0); 
    regulator.input = temp;

    delay_clc = analogRead(A3);
  }     
  
  analogWrite(3, regulator.getResultTimer());
   

 if(tempTime-timeLed>200)
{  
  timeLed = tempTime;

  int setTemp = analogRead(A2);
  setTemp = map(setTemp,0,4096,0,350);

  
  if((setTemp-prevSetTemp>5 || prevSetTemp-setTemp>5) || tempTime - timeSetTemp<2000)
  {
    if(setTemp-prevSetTemp>5 || prevSetTemp-setTemp>5)
    {
      timeSetTemp = tempTime;        
      prevSetTemp = setTemp;
      regulator.setpoint = setTemp;
    }
    
    disp.displayInt(setTemp);    
  }
  else
  {    
    disp.displayInt(temp);
  }
}

}



