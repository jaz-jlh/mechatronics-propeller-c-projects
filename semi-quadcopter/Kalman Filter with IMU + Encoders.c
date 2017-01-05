/*
  Blank Simple Project.c
  http://learn.parallax.com/propeller-c-tutorials 
*/
#include "simpletools.h"                      // Include simple tools
#include "simplei2c.h"

// H-Bridge
#define IN1 6
#define IN2 5
#define PWM 4
#define STBY 7
// Encoders
#define A 1
#define B 0
// Motor
#define dutyCycle 75
#define motorFreq 1000
// IMU Addresses
#define ACCEL_READ 0x3B
#define ACCEL_WRITE 0x3A
#define GYRO_READ 0xD7
#define GYRO_WRITE 0xD6

#define SCL 12 
#define SDA 11

volatile int counter = 0;

volatile float accel_xy_angle = 0;
volatile float accel_zy_angle = 0;
volatile float accel_xz_angle = 0;

volatile float gyro_xy_angle = 0;
volatile float gyro_zy_angle = 0;
volatile float gyro_xz_angle = 0;


void readIMU() {
  
  int ax =0;
  int ay =0;
  int az =0;
  int wx = 0;
  int wy = 0;
  int wz = 0;
  
  int current = 0;
  
  i2c my_i2c;
  i2c_open(&my_i2c,SCL,SDA,0);
  
  // Start Accelerometer
  i2c_start(&my_i2c);
  i2c_writeByte(&my_i2c,ACCEL_WRITE);
  i2c_writeByte(&my_i2c, 0x20); // CTRL1 register
  i2c_writeByte(&my_i2c, 0x77);
  i2c_stop(&my_i2c);
  unsigned char accel_bytes[6];
  
  // Start Gyroscope
  i2c_start(&my_i2c);
  i2c_writeByte(&my_i2c,GYRO_WRITE);
  i2c_writeByte(&my_i2c, 0x20); // CTRL1 register
  i2c_writeByte(&my_i2c, 0x3f); // page 36-37
  i2c_stop(&my_i2c);
  
  // Specify Gyro Scale
  i2c_start(&my_i2c);
  i2c_writeByte(&my_i2c,GYRO_WRITE);
  i2c_writeByte(&my_i2c, 0x23); // CTRL4 register
  i2c_writeByte(&my_i2c, 0x20); // page 39
  i2c_stop(&my_i2c);
  unsigned char gyro_bytes[6];
  
  unsigned count = CNT;
  
  while(1) {
    count += 4000000; //25Hz
    
    // Accelerometer
    i2c_start(&my_i2c);
    i2c_writeByte(&my_i2c,ACCEL_WRITE);
    i2c_writeByte(&my_i2c, 0x28 + 0x80); //OUT_X_L_A from page 29 of Accelerometer data sheet
    i2c_start(&my_i2c);
    i2c_writeByte(&my_i2c,ACCEL_READ);
    i2c_readData(&my_i2c,accel_bytes,6);
    i2c_stop(&my_i2c);
    
    current = accel_bytes[0];
    ax = current;
    current = accel_bytes[1];
    ax += current << 8;
    if(ax & (1 << 15)) ax -= (1 << 16);
    
    current = accel_bytes[2];
    ay = current;
    current = accel_bytes[3];
    ay += current << 8;
    if(ay & (1 << 15)) ay -= (1 << 16);
    
    current = accel_bytes[4];
    az = current;
    current = accel_bytes[5];
    az += current << 8;
    if(az & (1 << 15)) az -= (1 << 16);
    
    accel_xy_angle = atan2(((float)ax),((float)ay));  //angle about z is x/y
    accel_zy_angle = atan2(((float)az),((float)ay));  //angle about x is z/y
    accel_xz_angle = atan2(((float)ax),((float)az));  //angle about y is x/z
    
    // Gyroscope
    i2c_start(&my_i2c);
    i2c_writeByte(&my_i2c,GYRO_WRITE);
    i2c_writeByte(&my_i2c, 0x28 + 0x80); //OUT_X_L from page 34 of gyro data sheet
    i2c_start(&my_i2c);
    i2c_writeByte(&my_i2c,GYRO_READ);
    i2c_readData(&my_i2c,gyro_bytes,6);
    i2c_stop(&my_i2c);
    
    current = gyro_bytes[0];
    wx = current;
    current = gyro_bytes[1];
    wx += current << 8;
    if(wx & (1 << 15)) wx -= (1 << 16);
    
    current = gyro_bytes[2];
    wy = current;
    current = gyro_bytes[3];
    wy += current << 8;
    if(wy & (1 << 15)) wy -= (1 << 16);
    
    current = gyro_bytes[4];
    wz = current;
    current = gyro_bytes[5];
    wz += current << 8;
    if(wz & (1 << 15)) wz -= (1 << 16);
    
    gyro_xy_angle += (float)(wz*(2000/(1 << 15))) * 0.04;
    gyro_zy_angle += (float)(wx*(2000/(1 << 15))) * 0.04;
    gyro_xz_angle += (float)(wy*(2000/(1 << 15))) * 0.04;
    
    waitcnt(count);
    }  
}

void pwmRoutine() {
  CTRA = 0x10000000;
  CTRA += PWM;
  FRQA = 1;
  low(PWM);
  unsigned endTime = CNT;
  while(1) {
    endTime += 10000;
    PHSA = - ((((int)dutyCycle) * 10000) / 100);
    waitcnt(endTime);
  }    
}

void encoder() {
  unsigned stateA = 0;
  unsigned stateB = 0;
  unsigned lastA = 0;
  unsigned lastB = 0;
  set_directions(A,B,0x00);
  set_directions(IN1,IN2,0xff);
  low(IN1);
  high(IN2);
  while(1) {
    stateA = input(A);
    stateB = input(B);
    if(stateA != lastA) {
      if(stateA == stateB) {
        counter++;
      } else {
        counter--;
      }          
    }
    if(stateB != lastB) {
      if(stateA != stateB) {
        counter++;
      } else {
        counter--;
      }
    }            
    lastA = stateA;
    lastB = stateB;
    
    // Switch Motor Directions
    if(counter > motorFreq) {
        low(IN1);
        high(IN2);
      } else if (counter < -motorFreq){
        high(IN1);
        low(IN2);
      }          
  }    
}  

int main()                                    // Main function
{
  cog_run(readIMU,256);
  set_direction(STBY,0xf);
  high(STBY);
  cog_run(pwmRoutine,128);
  cog_run(encoder,128);
  
  
  
  while(1)
  {
    // Add main loop code here.
    print("X = %2.1f, Y = %2.1f, Z = %2.1f  ", accel_xy_angle, accel_zy_angle, accel_xz_angle);
    print("X = %2.1f, Y = %2.1f, Z = %2.1f\n", gyro_xy_angle, gyro_zy_angle, gyro_xz_angle);
    pause(200);
    //print("Encoder: %2.1f deg, Accel: %2.1f rad, Gyro: %2.1f rad\n", counter/10., accel_xy_angle, gyro_xy_angle);
    //pause(200);
  }  
}
