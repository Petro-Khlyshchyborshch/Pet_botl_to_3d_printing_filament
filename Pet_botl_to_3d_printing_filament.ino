
#define DRIVER_STEP_TIME 2  
#define CLK 9
#define DT 8
#define SW 7

#define B 3950 // B-коэффициент
#define SERIAL_R 98000 // сопротивление последовательного резистора, 102 кОм
#define THERMISTOR_R 100000 // номинальное сопротивления термистора, 100 кОм
#define NOMINAL_T 25 // номинальная температура (при которой TR = 100 кОм)

//#define PID_INTEGER
#include "GyverStepper.h"
#include "GyverEncoder.h"
#include "GyverTimers.h"
#include "GyverPID.h"
#include <LiquidCrystal_I2C.h>
GStepper< STEPPER2WIRE> stepper(200, 5, 4, 6);
Encoder enc1(CLK, DT, SW);
LiquidCrystal_I2C lcd(0x3F, 16, 2);
//GyverPID regulator(15.0, 0.2, 10);
GyverPID regulator(18.0, 0.4, 40.0);
// 2 - STEP
// 3 - DIR
// 4 - EN
#define FILTER_STEP 50
#define FILTER_COEF 0.05
int val;
int val_f;
int period=500;
unsigned long filter_timer;
const byte tempPin = A0;
bool menu=0;
float time_;
int temp=0;
int speed_=0;
bool en=0;



void setup() {
  pinMode( tempPin, INPUT );
  pinMode( 3, OUTPUT );
  enc1.setType(TYPE2);
  
  stepper.setRunMode(KEEP_SPEED); // режим поддержания скорости
  //stepper.setRunMode(FOLLOW_POS);
 // stepper.setMaxSpeed(speed_);
  stepper.setAcceleration(500);
  stepper.setSpeed(speed_,SMOOTH);
  
  regulator.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulator.setLimits(0, 255);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
  regulator.setpoint = temp;        // сообщаем регулятору температуру, которую он должен поддерживать
  regulator.setDt(period);
  
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Temp");
  lcd.setCursor(0,1);
  lcd.print("Speed");
  
  Serial.begin(9600);
  
  Timer2.setPeriod(800);
  Timer2.enableISR();
}
ISR(TIMER2_A) {
  stepper.tick(); // тикаем тут
  enc1.tick();
}
void loop() {
 static uint32_t tmr;
 if(millis()-tmr>=period){
  tmr = millis();
  val_f=get_temp();
  regulator.input = val_f;
  regulator.getResult();
  analogWrite(3, regulator.output);
  Serial.print(val_f);
 Serial.print(" ");
 Serial.print(temp);
 Serial.print(" ");
 Serial.println(regulator.output);
 }
 
  if (enc1.isRight())
  {
    
    if(menu){
      speed_+=10; 
      stepper.setSpeed(speed_,SMOOTH);
  }
  else{
    if(temp<280)temp+=5; regulator.setpoint = temp;  
    }
    print_lcd();
  }
  if (enc1.isLeft()){
    
    if(menu){
      speed_-=10;
      stepper.setSpeed(speed_,SMOOTH);
     
    
    } else{
    if(temp>0)temp-=5; regulator.setpoint = temp;  
    }
    print_lcd();
   }
  if (enc1.isClick()){menu=!menu;print_lcd();}
  if((millis()- time_)>1000){print_lcd();}
 if(enc1.isHolded()){
   en=!en;
   print_lcd();
   if(en) stepper.disable();
   else stepper.enable();
  }
}
void print_lcd(){

  if(menu){
     lcd.setCursor(15,1);
      lcd.print("<");
      lcd.setCursor(15,0);
      lcd.print(" ");
    }
    else{
      lcd.setCursor(15,0);
      lcd.print("<");
      lcd.setCursor(15,1);
      lcd.print("    ");
    }
      
     lcd.setCursor(8,1);
     lcd.print("    "); 
     lcd.setCursor(8,1); 
     lcd.print(speed_);

     lcd.setCursor(12,1);
     lcd.print(" "); 
     lcd.setCursor(12,1);
     if(en) lcd.print("S");
     else lcd.print("R");
     
     
     lcd.setCursor(8,0);
     lcd.print("       "); 
     lcd.setCursor(8,0); 
     lcd.print(int(val_f));
     lcd.print("/");
     lcd.print(temp);  
     time_=millis();
  
  }
 
const short temptable_11[][2] PROGMEM = {
    {1, 841},
   {54, 255},
   {107, 209},
   {160, 184},
   {213, 166},
   {266, 153},
   {319, 142},
   {372, 132},
   {425, 124},
   {478, 116},
   {531, 108},
   {584, 101},
   {637, 93},
   {690, 86},
   {743, 78},
   {796, 70},
   {849, 61},
   {902, 50},
   {955, 34},
   {1008, 3}
};

# define BEDTEMPTABLE_LEN (sizeof(temptable_11)/sizeof(*temptable_11))
#define PGM_RD_W(x)   (short)pgm_read_word(&x)
static float analog2tempBed(int raw) {
    float celsius = 0;
    byte i;
 
    for (i = 1; i < BEDTEMPTABLE_LEN; i++)
    {
        if (PGM_RD_W(temptable_11[i][0]) > raw)
        {
            celsius = PGM_RD_W(temptable_11[i - 1][1]) +
                (raw - PGM_RD_W(temptable_11[i - 1][0])) *
                (float)(PGM_RD_W(temptable_11[i][1]) - PGM_RD_W(temptable_11[i - 1][1])) /
                (float)(PGM_RD_W(temptable_11[i][0]) - PGM_RD_W(temptable_11[i - 1][0]));
            break;
        }
    }
 
    // Overflow: Set to last value in the table
    if (i == BEDTEMPTABLE_LEN) celsius = PGM_RD_W(temptable_11[i - 1][1]);
 
    return celsius;
}

float get_temp(){
  int v = analogRead(A0);
  return analog2tempBed(v);
  }
