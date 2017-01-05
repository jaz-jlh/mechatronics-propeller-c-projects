#include "simpletools.h"                      // Include simple tools

#define ALARM -1
#define ARMED 1
#define DISARMED 0

// VOLATILE GLOBAL VARIABLES //
volatile int passcodeflag = 0;                        
volatile int laststate = 1;
volatile int buttonPressed = 0;                   // keep track of butotn
volatile int laserBroken;                 // keep track of laser
volatile int lock;                            // for servo to check


// IR COMMUNICATION //
void irTranslator() {                     // Method for ir communication, will run continuously on a cog
  int lastButtonTime;
  int buttonTime;
  int button;
  int code[] = {3,5,7,2};
  int passcode[4];
  while(1) { 
    for(int i = 0; i < 4; ++i) {
      button = (irInput(6) & 0x7f) + 1;
      if(button == 21) { break; } else {
      buttonTime = CNT;
      if(buttonTime - lastButtonTime > 10000000) {
        passcode[i] = button;
      } else {
        i = i - 1;
      }        
      lastButtonTime = buttonTime;
      }
     }
 if(passcode[0] == code[0] && passcode[1] == code[1] && passcode[2] == code[2] && passcode[3] == code[3]) {
   passcodeflag = 1;
   passcode[0] = 0;
   passcode[1] = 0;
   passcode[2] = 0;
   passcode[3] = 0;
   //print("AYYYYYYYYY!!!!!!!\n");
 } else {
   passcodeflag = 0;
   passcode[0] = 0;
   passcode[1] = 0;
   passcode[2] = 0;
   passcode[3] = 0;
   //print("BOOOOOoooooooOOOOOoooooOOOOoOOoOO........\n");
   //print("Incorrect Passcode\n");
 }   
  }    
}
int irInput(int pin){
  int code = 0;
  input(pin);
CTRA = 0x30000000;
CTRA += pin;
FRQA = 1;
int pulse = 0;

//wait for a start pulse between 2.25 and 2.75 ms
while(pulse < 2250 || pulse > 2750){
waitpeq(0, 1 << pin);
PHSA = 0;
waitpeq(1 << pin, 1 << pin);
pulse = PHSA / 100;
}
for(int i = 0; i < 12; i++){
waitpeq(0, 1 << pin);
PHSA = 0;
waitpeq(1 << pin, 1 << pin);
pulse = PHSA / 100;
if(pulse > 1000 && pulse < 1400) code += 1 << i;   //long pulse
else if(pulse < 300 || pulse > 1400) return -1;     //error
}
return code;
}
// IR COMMUNICATION //

// BUTTON //
void buttonChecker() {
  while(1) {
    waitpeq(0b10000,0b10000);
    buttonPressed = 1;
  }    
}  
// BUTTON //

// LAAAAZERZ //
void laser() {
  //high(20);                   //turn on laser
  int decaytime = 0;
  while(1) {
  CTRA = 0x20000000;                          // POS detector Mode
  CTRA += 2;
  FRQA = 1;
  
  set_direction(0, 1);                        //set output 
  set_output(0, 1);                           // set high to charge
  waitcnt(CNT + 100000);                       //wait to charge
  
  set_direction(0, 0);                        //set to input
  PHSA = 0;
  
  waitpeq(0, 0b0100);
  CTRA = 0;
  decaytime = PHSA;
  if((decaytime / 100) > 200) {               //threshold of 200 may or may not be good
      laserBroken = 1; //dark
    } else {
      laserBroken = 0; //light
    }
}  
}
// LAAAZERZ //




// TONE //
void tone() {
  CTRA = 0x10000000;                        // NCO mode
  CTRA += 7;                    // set pin that will produce tone
  PHSA = 0;                       // start phsa at 0
  set_direction(7,1);           // set pin to be an output
  int duration = 0.5;
  while(1) {
    //low(16);
    //FRQA = 494 * 42.95;
    //ode to joy
    low(16);
    FRQA = 554 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 554 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 587 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 659 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 659 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 587 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 554 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 494 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 440 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 440 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 494 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 554 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 554 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 494 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 494 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 554 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 554 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 587 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 659 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 659 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 587 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 554 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 494 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 440 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 440 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 494 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 554 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 494 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 440 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);
    low(16);
    FRQA = 440 * 42.95;
    pause(100);
    high(16);
    FRQA = 0;
    pause(100);

    //ode to joy
    //high(16);
    //FRQA = 0;
    //pause(100);
    
  }    
}
// TONE //


// SERVO //
void Servo(int dur)
{
    {
      int cycleNum = dur*100;
      PHSA = -cycleNum;
      pause(10);//waitcnt(CNT + 1000000);
    }   
}
void RCcontrol()
{
  CTRA=0x10000000;
  CTRA += 11;
  PHSA = 0;
  FRQA = 1;
  set_direction(11, 1);
  while(1)
  {
    if(lock == 1) {
      Servo(2300);
    } else if(lock == 0) {
      Servo(700);
    }
  }    
}
// SERVO //



int main()                                    // Main function
{
  int state = 0;                                  // alarm, armed, or disarmed
  int* ircog = cog_run(irTranslator, 128);
  int *buttoncog = cog_run(buttonChecker, 128);
  int *lasercog = cog_run(laser, 128);
  int *tonecog = cog_run(tone, 128);
  int *servocog = cog_run(RCcontrol, 128);
  while(1)
  {  
  if(passcodeflag == 1 && state != DISARMED) {
      print("Correct passcode entered\n");
      passcodeflag = 0;
      buttonPressed = 0;
      state = DISARMED;
    }         
    if(buttonPressed == 1 && state == DISARMED) {
      state = ARMED;
      buttonPressed = 0;
    }
    if(laserBroken == 1 && state == ARMED) {
      state = ALARM;
      laserBroken = 0;
      buttonPressed =0;    
    } 
    if(state == DISARMED && laststate != DISARMED) {
      cog_end(tonecog);
      print("Disarming...\n");
      cog_end(ircog);
      cog_end(lasercog);
      low(20);
      buttoncog = cog_run(buttonChecker, 128);
      high(17);
      low(16);
      laststate = DISARMED;
      passcodeflag = 0;
      buttonPressed = 0;
      laserBroken = 0;
      lock = 0;
    }
    if(state == ARMED && laststate != ARMED) {
      print("Arming...\n");
      ircog = cog_run(irTranslator, 128);
      lasercog = cog_run(laser, 128);
      cog_end(buttoncog);
      high(16);
      low(17);
      high(20);
      laststate = ARMED;
      passcodeflag = 0;
      laserBroken = 0;
      buttonPressed = 0;
      lock = 1;
    }
    if(state == ALARM && laststate != ALARM) {
      print("ALARM!\n");
      tonecog = cog_run(tone, 128);
      low(20);
      low(16);
      laststate = ALARM;
      passcodeflag = 0;
      laserBroken = 0;
      buttonPressed = 0;
      lock = 1;
      cog_end(lasercog);
    }
    
                   
  }  
}
